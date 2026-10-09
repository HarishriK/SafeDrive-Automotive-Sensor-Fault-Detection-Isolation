/*
 * stop_at_distance_ft.c  -  QNX 8 / Raspberry Pi 4   (FAULT-TOLERANT version)
 *
 * Combines  stop_at_3cm.c (ultrasonic + rotary encoder)  and
 *           stop_at_distance_imu.c (ultrasonic + MPU6050)
 * into ONE program that keeps working when a sensor fails.
 *
 *  Phase 1  Robot rests REST_TIME_MS.  HC-SR04 measures the distance, the
 *           MPU6050 bias/noise is measured.  Ultrasonic is then switched off.
 *  Phase 2  Robot drives. Distance / velocity source (best available wins):
 *             encoder OK                -> distance = encoder counts,
 *                                          velocity = counts / VEL_INTERVAL_MS
 *             encoder FAULTY, IMU OK    -> IMU dead-reckoning (velocity was
 *                                          seeded from the encoder, so no drift)
 *             IMU FAULTY, encoder OK    -> encoder velocity (fixed interval)
 *             both FAULTY               -> safe stop (or velocity-hold, see
 *                                          BOTH_LOST_STOP)
 *           The brake point is  remaining <= COAST_MIN_CM + COAST_K * v^2,
 *           i.e. it brakes earlier the faster the robot is going.
 *
 *  Every sensor has a state: OK / FAULTY / RECOVERING.  Faults are printed
 *  the moment they are detected and summarised at the end.
 *
 *  Recovery:
 *    Ultrasonic : re-measure up to US_RETRIES times; if still bad, use the
 *                 distance given on the command line (-d <cm>), else don't move.
 *    MPU6050    : I2C re-open + device reset + re-init, done WITHOUT blocking
 *                 the control loop (state machine). On success the IMU velocity
 *                 is re-seeded from the encoder.
 *    Encoder    : GPIO re-armed; if pulses return (and the IMU confirms the
 *                 robot is moving) the encoder is re-synchronised to the
 *                 dead-reckoned position and used again.
 *
 *  A robot that simply does not move (motors / driver / battery) is reported
 *  as STALLED and is NOT blamed on a sensor.
 *
 * Build:  qcc -V gcc_ntoaarch64le -o stop_at_distance_ft stop_at_distance_ft.c
 * Usage (root, robot STILL during the first 3 s):
 *   ./stop_at_distance_ft              normal run
 *   ./stop_at_distance_ft -d 60        manual distance (cm) used ONLY if the
 *                                      ultrasonic is faulty
 *   ./stop_at_distance_ft -s           sensor self-test, motors stay off
 *                                      (spin a wheel by hand when asked)
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <sched.h>
#include <pthread.h>
#include <devctl.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <hw/i2c.h>

/* ------------------------- USER SETTINGS ------------------------- */
/* geometry / encoder */
#define WHEEL_CIRC_CM      20.4   /* tyre circumference                            */
#define ENC_SLOTS          20     /* slots in YOUR encoder disc  <-- CHECK THIS    */
#define COUNT_BOTH_EDGES   1      /* 1 = rising+falling (2 counts per slot)        */

/* task */
#define STOP_GAP_CM        5.0    /* final gap to the object                       */
#define SENSOR_OFFSET_CM   0.0    /* HC-SR04 face -> robot's front edge            */
#define SAFETY_MARGIN_CM   2.0    /* extra distance to stay SHORT of the target    */
#define REST_TIME_MS       3000   /* rest + measuring time                         */
#define MAX_TRAVEL_CM      300.0  /* safety limit                                  */
#define MAX_DRIVE_S        20.0   /* absolute drive-time limit                     */

/* motor duty (multiple of 10). Keep 100 if lower duty makes the robot stall.      */
#define DUTY_FAR           100
#define DUTY_NEAR          100
#define DUTY_CREEP         100
#define SLOW_ZONE_CM       12.0
#define CREEP_ZONE_CM      4.0

/* braking model: brake when remaining <= COAST_MIN + COAST_K * v^2  (v in cm/s)   */
#define COAST_MIN_CM       1.0
#define COAST_K            0.004  /* cm / (cm/s)^2 ; the run prints a better value */
#define DEG_IMU_EXTRA_CM   4.0    /* extra early brake when only the IMU is left   */
#define DEG_BLIND_EXTRA_CM 10.0   /* extra early brake when no sensor is left      */
#define BOTH_LOST_STOP     1      /* 1 = brake at once if encoder AND IMU are lost */

/* velocity from the encoder over a fixed time interval */
#define VEL_INTERVAL_MS    100
#define VEL_SMOOTH         0.5    /* 0..1, weight of the newest interval           */

/* ultrasonic health */
#define US_MIN_VALID       10     /* valid echoes needed in the rest window        */
#define US_MAX_SPREAD_CM   5.0    /* central-80% spread above this = unstable      */
#define US_RETRIES         3

/* IMU health / recovery */
#define MAX_BIAS_SD        0.20   /* m/s^2, noise at rest                          */
#define MAX_I2C_FAILS      5      /* consecutive read errors => fault              */
#define IMU_FROZEN_N       40     /* identical raw samples in a row => fault       */
#define IMU_MAX_RECOVER    3
#define FWD_SIGN           1.0    /* set to -1.0 if +X of the IMU points backward  */
#define ACC_ALPHA          0.15
#define ACC_DEADBAND       0.05
#define ZUPT_ACC           0.15   /* m/s^2: below this the robot is standing still */

/* encoder health / recovery */
#define ENC_GRACE_MS       600    /* start-up time before "no pulses" is judged    */
#define ENC_FAULT_MS       400    /* no pulses this long while driving             */
#define ENC_RECOVER_COUNTS 4      /* pulses needed to declare the encoder back     */
#define ENC_STILL_MS       200    /* no pulses this long after brake = stopped     */

/* "robot is moving" detector from IMU vibration */
#define MOVE_VIB_FACTOR    3.0    /* x rest noise                                  */
#define MOVE_VIB_MIN       0.10   /* m/s^2 minimum threshold                       */

#define LOOP_US            2000   /* control period                                */

/* Flip these if a motor turns the wrong way */
#define LEFT_FWD_IN1       0
#define LEFT_FWD_IN2       1
#define RIGHT_FWD_IN3      0
#define RIGHT_FWD_IN4      1
/* ----------------------------------------------------------------- */

/* GPIO numbers (BCM) */
#define PIN_TRIG 23
#define PIN_ECHO 24
#define PIN_ENC  17
#define PIN_IN1   5
#define PIN_IN2   6
#define PIN_IN3  13
#define PIN_IN4  19

#define PRIO_RT  56
#define MPU_ADDR 0x68
#define G_MS2    9.80665

static inline double absd(double x) { return x < 0 ? -x : x; }

/* ============================ Timing ============================ */
static inline uint64_t now_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000ull + t.tv_nsec / 1000;
}
static void sleep_ms(int ms)
{
    struct timespec t = { ms / 1000, (long)(ms % 1000) * 1000000L };
    nanosleep(&t, NULL);
}

/* ============================ Sensor status / messages ============================ */
typedef enum { SEN_OK, SEN_FAULTY, SEN_RECOVERING } sen_t;
typedef struct {
    const char *name;
    volatile sen_t st;
    char why[96];
    int faults, recovered;
} sensor_t;

static sensor_t S_US  = { "Ultrasonic HC-SR04", SEN_OK, "", 0, 0 };
static sensor_t S_IMU = { "MPU6050 IMU",        SEN_OK, "", 0, 0 };
static sensor_t S_ENC = { "Rotary encoder",     SEN_OK, "", 0, 0 };
static uint64_t t_origin;

static void say(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    int n = snprintf(buf, sizeof buf, "[%7.2fs] ", (now_us() - t_origin) * 1e-6);
    va_start(ap, fmt);
    vsnprintf(buf + n, sizeof buf - n, fmt, ap);
    va_end(ap);
    printf("%s\n", buf);
    fflush(stdout);
}
static void set_fault(sensor_t *s, const char *why)
{
    s->st = SEN_FAULTY;
    s->faults++;
    snprintf(s->why, sizeof s->why, "%s", why);
    say("*** %s is FAULTY: %s", s->name, why);
}
static void set_ok(sensor_t *s, const char *how)
{
    s->st = SEN_OK;
    s->recovered++;
    say("+++ %s RECOVERED (%s)", s->name, how);
}
static void print_status(void)
{
    sensor_t *all[3] = { &S_US, &S_IMU, &S_ENC };
    printf("\n================== SENSOR STATUS ==================\n");
    for (int i = 0; i < 3; i++) {
        sensor_t *s = all[i];
        const char *txt = (s->st == SEN_FAULTY)     ? "FAULTY"
                        : (s->st == SEN_RECOVERING) ? "RECOVERING"
                        : (s->faults ? "OK (recovered)" : "OK");
        printf(" %-20s : %s", s->name, txt);
        if (s->faults) printf("   [faults=%d recovered=%d] last: %s", s->faults, s->recovered, s->why);
        printf("\n");
    }
    printf("===================================================\n");
    fflush(stdout);
}

/* ============================ HAL: GPIO ============================ */
#define BCM2711_GPIO_BASE 0xFE200000UL
static volatile uint32_t *gpio;

static int gpio_init(void)
{
    gpio = mmap_device_memory(NULL, 0x1000, PROT_READ | PROT_WRITE | PROT_NOCACHE,
                              0, BCM2711_GPIO_BASE);
    return (gpio == MAP_FAILED) ? -1 : 0;
}
static void gpio_mode(int pin, int out)
{
    volatile uint32_t *r = &gpio[pin / 10];
    int sh = (pin % 10) * 3;
    uint32_t v = *r & ~(7u << sh);
    if (out) v |= (1u << sh);
    *r = v;
}
static inline void gpio_write(int pin, int v)
{
    if (v) gpio[0x1C / 4] = 1u << pin;
    else   gpio[0x28 / 4] = 1u << pin;
}
static inline int gpio_read(int pin) { return (gpio[0x34 / 4] >> pin) & 1u; }

/* ============================ Motors ============================ */
static void motors_forward(void)
{
    gpio_write(PIN_IN1, LEFT_FWD_IN1);  gpio_write(PIN_IN2, LEFT_FWD_IN2);
    gpio_write(PIN_IN3, RIGHT_FWD_IN3); gpio_write(PIN_IN4, RIGHT_FWD_IN4);
}
static void motors_coast(void)
{
    gpio_write(PIN_IN1, 0); gpio_write(PIN_IN2, 0);
    gpio_write(PIN_IN3, 0); gpio_write(PIN_IN4, 0);
}
static void motors_brake(void)
{
    gpio_write(PIN_IN1, 1); gpio_write(PIN_IN2, 1);
    gpio_write(PIN_IN3, 1); gpio_write(PIN_IN4, 1);
}
static void on_signal(int s) { (void)s; motors_brake(); _exit(1); }

/* ============================ HAL: I2C / MPU6050 ============================ */
static int i2c_fd = -1;

static int mpu_write(uint8_t reg, uint8_t val)
{
    struct { i2c_send_t hdr; uint8_t d[2]; } m;
    memset(&m, 0, sizeof m);
    m.hdr.slave.addr = MPU_ADDR;
    m.hdr.slave.fmt  = I2C_ADDRFMT_7BIT;
    m.hdr.len  = 2;
    m.hdr.stop = 1;
    m.d[0] = reg; m.d[1] = val;
    return (devctl(i2c_fd, DCMD_I2C_SEND, &m, sizeof m, NULL) == EOK) ? 0 : -1;
}
static int mpu_read(uint8_t reg, uint8_t *out, int n)
{
    struct { i2c_sendrecv_t hdr; uint8_t d[8]; } m;
    memset(&m, 0, sizeof m);
    m.hdr.slave.addr = MPU_ADDR;
    m.hdr.slave.fmt  = I2C_ADDRFMT_7BIT;
    m.hdr.send_len = 1;
    m.hdr.recv_len = n;
    m.hdr.stop = 1;
    m.d[0] = reg;
    if (devctl(i2c_fd, DCMD_I2C_SENDRECV, &m, sizeof m.hdr + n, NULL) != EOK) return -1;
    memcpy(out, m.d, n);
    return 0;
}
static int i2c_reopen(void)
{
    if (i2c_fd >= 0) close(i2c_fd);
    i2c_fd = open("/dev/i2c1", O_RDWR);
    return (i2c_fd >= 0) ? 0 : -1;
}
/* blocking init (rest phase only): returns 0 on success */
static int mpu_init(void)
{
    uint8_t who = 0;
    if (i2c_reopen() != 0) return -1;
    if (mpu_read(0x75, &who, 1) != 0 || who != 0x68) return -1;
    if (mpu_write(0x6B, 0x80)) return -1;          /* device reset            */
    sleep_ms(100);
    if (mpu_write(0x6B, 0x01)) return -1;          /* wake, clock = gyro PLL  */
    sleep_ms(50);
    if (mpu_write(0x1A, 0x03)) return -1;          /* DLPF ~44 Hz             */
    if (mpu_write(0x1C, 0x00)) return -1;          /* +/-2 g, 16384 LSB/g     */
    sleep_ms(50);
    return 0;
}
static int mpu_raw(int16_t *raw)
{
    uint8_t b[2];
    if (mpu_read(0x3B, b, 2) != 0) return -1;      /* ACCEL_XOUT_H/L */
    *raw = (int16_t)((b[0] << 8) | b[1]);
    return 0;
}
static inline double raw_to_ms2(int16_t raw) { return FWD_SIGN * (raw / 16384.0) * G_MS2; }

/* ---- non-blocking IMU recovery (used while driving) ---- */
static struct {
    int phase, attempts, good, same;
    int16_t last_raw;
    uint64_t t_next;
    volatile int done;
} rec;

static void imu_fault_begin(const char *why)
{
    set_fault(&S_IMU, why);
    if (rec.attempts < IMU_MAX_RECOVER) {
        S_IMU.st = SEN_RECOVERING;
        rec.phase = 0;
        rec.t_next = now_us() + 100000;
        say(">>> MPU6050 recovery started (attempt %d/%d). Encoder velocity is used meanwhile.",
            rec.attempts + 1, IMU_MAX_RECOVER);
    }
}
static void imu_recover_step(uint64_t t)
{
    int ok = 1;
    if (t < rec.t_next) return;
    switch (rec.phase) {
    case 0:  ok = (i2c_reopen() == 0) && (mpu_write(0x6B, 0x80) == 0);
             rec.t_next = t + 100000; break;
    case 1:  ok = (mpu_write(0x6B, 0x01) == 0);
             rec.t_next = t + 20000;  break;
    case 2:  ok = (mpu_write(0x1A, 0x03) == 0) && (mpu_write(0x1C, 0x00) == 0);
             rec.t_next = t + 60000; rec.good = 0; rec.same = 0; rec.last_raw = 0; break;
    default: {                                   /* phase 3: verify fresh samples */
             int16_t raw;
             if (mpu_raw(&raw) != 0) { ok = 0; break; }
             if (rec.good > 0 && raw == rec.last_raw) rec.same++;
             rec.last_raw = raw;
             if (++rec.good < 12) { rec.t_next = t + 4000; return; }
             if (rec.same >= rec.good - 2) { ok = 0; break; }
             rec.attempts = 0;
             rec.done = 1;
             set_ok(&S_IMU, "device reset + re-init");
             return;
    }}
    if (!ok) {
        rec.attempts++;
        if (rec.attempts >= IMU_MAX_RECOVER) {
            S_IMU.st = SEN_FAULTY;
            say("*** MPU6050 recovery FAILED %d times - IMU stays FAULTY", rec.attempts);
        } else {
            say(">>> MPU6050 recovery attempt failed, retrying (%d/%d)", rec.attempts + 1, IMU_MAX_RECOVER);
            rec.phase = 0;
            rec.t_next = t + 300000;
        }
        return;
    }
    rec.phase++;
}

/* ============================ Ultrasonic (rest phase only) ============================ */
static double ultrasonic_read_cm(void)
{
    uint64_t t, s;
    gpio_write(PIN_TRIG, 0);
    t = now_us(); while (now_us() - t < 4) ;
    gpio_write(PIN_TRIG, 1);
    t = now_us(); while (now_us() - t < 10) ;
    gpio_write(PIN_TRIG, 0);
    t = now_us();
    while (!gpio_read(PIN_ECHO)) if (now_us() - t > 30000) return -1.0;
    s = now_us();
    while (gpio_read(PIN_ECHO))  if (now_us() - s > 30000) return -1.0;
    return (double)(now_us() - s) / 58.0;
}
static int cmp_d(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

/* ============================ Rest phase ============================ */
typedef struct {
    double us_v[100];
    int    us_n, us_tries;
    double sum, sum2;
    long   cnt, same, i2c_err;
    int16_t last_raw;
    int    have_last;
} rest_t;

static void rest_reset_imu(rest_t *R)
{
    R->sum = R->sum2 = 0; R->cnt = R->same = R->i2c_err = 0; R->have_last = 0;
}
static void rest_collect(rest_t *R, int ms, int use_us, int use_imu)
{
    uint64_t start = now_us();
    while ((now_us() - start) < (uint64_t)ms * 1000) {
        if (use_us && R->us_n < 100) {
            double d = ultrasonic_read_cm();
            R->us_tries++;
            if (d >= 2.0 && d <= 400.0) R->us_v[R->us_n++] = d;
        }
        if (!use_imu) { sleep_ms(60); continue; }      /* HC-SR04 needs >= 60 ms */
        uint64_t t0 = now_us();
        while (now_us() - t0 < 60000) {
            int16_t raw;
            if (mpu_raw(&raw) == 0) {
                double a = raw_to_ms2(raw);
                R->sum += a; R->sum2 += a * a; R->cnt++;
                if (R->have_last && raw == R->last_raw) R->same++;
                R->last_raw = raw; R->have_last = 1;
            } else R->i2c_err++;
            sleep_ms(4);
        }
    }
}
static int us_analyze(rest_t *R, double *dist, char *why, size_t wn)
{
    if (R->us_n < US_MIN_VALID) {
        snprintf(why, wn, "only %d of %d pings gave a valid echo (check TRIG/ECHO wiring, 1k/2k divider, 5 V)",
                 R->us_n, R->us_tries);
        return -1;
    }
    qsort(R->us_v, R->us_n, sizeof R->us_v[0], cmp_d);
    double lo = R->us_v[R->us_n / 10], hi = R->us_v[R->us_n - 1 - R->us_n / 10];
    if (hi - lo > US_MAX_SPREAD_CM) {
        snprintf(why, wn, "unstable readings (spread %.1f cm)", hi - lo);
        return -1;
    }
    *dist = R->us_v[R->us_n / 2];
    return 0;
}
static int imu_analyze(rest_t *R, double *bias, double *sd, char *why, size_t wn)
{
    if (R->cnt < 100) {
        snprintf(why, wn, "too few samples (%ld ok, %ld I2C errors)", R->cnt, R->i2c_err);
        return -1;
    }
    double b = R->sum / R->cnt;
    double var = R->sum2 / R->cnt - b * b;
    double s = var > 0 ? __builtin_sqrt(var) : 0;
    if (s < 1e-6 || R->same > (R->cnt * 9) / 10) {
        snprintf(why, wn, "output frozen / constant");
        return -1;
    }
    if (s > MAX_BIAS_SD) {
        snprintf(why, wn, "noise %.3f m/s^2 too high (robot moving or sensor faulty)", s);
        return -1;
    }
    if (absd(b) > 8.0) {
        snprintf(why, wn, "bias %.2f m/s^2 out of range (X axis vertical or sensor faulty)", b);
        return -1;
    }
    *bias = b; *sd = s;
    return 0;
}

/* ============================ Drive ============================ */
typedef struct {
    double x_brake_cm, x_total_cm, v_brake_cms, t_brake_s;
    long counts;
    const char *reason;
    int aborted;
} run_t;

static void drive(double travel_cm, double bias, double rest_sd, run_t *r)
{
    const double cm_per_count = WHEEL_CIRC_CM / (ENC_SLOTS * (COUNT_BOTH_EDGES ? 2.0 : 1.0));
    const double move_thr = (MOVE_VIB_FACTOR * rest_sd > MOVE_VIB_MIN) ? MOVE_VIB_FACTOR * rest_sd : MOVE_VIB_MIN;

    long counts = 0, counts_iv = 0, edges_since_fault = 0;
    int prev = gpio_read(PIN_ENC), last = prev;
    uint64_t t_start = now_us(), tprev = t_start, t_last_change = t_start, t_iv = t_start;
    uint64_t nomove_since = 0;

    double enc_off = 0;                 /* resync offset after encoder recovery (cm) */
    double v_enc_f = 0;                 /* smoothed encoder velocity (cm/s)          */
    double af = 0, v_imu = 0;           /* IMU: filtered accel (m/s^2), velocity (cm/s) */
    double x = 0, v = 0, v_hold = 0;    /* fused distance (cm), velocity (cm/s)      */
    double win_sum = 0, win_sum2 = 0; int win_n = 0, moving = 0;
    int fails = 0, same = 0, have_raw = 0; int16_t last_raw = 0;
    unsigned tick = 0;
    const char *src = "encoder";

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    memset(&rec, 0, sizeof rec);
    r->reason = "?"; r->aborted = 0;

    for (;;) {
        next.tv_nsec += LOOP_US * 1000L;
        while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

        uint64_t t = now_us();
        double dt = (t - tprev) * 1e-6; tprev = t;
        double ts = (t - t_start) * 1e-6;

        /* ---------- encoder poll (always, so a faulty encoder can recover) ---------- */
        int s = gpio_read(PIN_ENC);
        if (s == prev && s != last) {
            if (COUNT_BOTH_EDGES || s == 1) { counts++; edges_since_fault++; }
            last = s;
            t_last_change = t;
        }
        prev = s;

        /* ---------- IMU ---------- */
        if (S_IMU.st == SEN_OK) {
            int16_t raw;
            if (mpu_raw(&raw) == 0) {
                fails = 0;
                if (have_raw && raw == last_raw) same++; else same = 0;
                last_raw = raw; have_raw = 1;
                double a = raw_to_ms2(raw) - bias;
                af += ACC_ALPHA * (a - af);
                double ae = (af > -ACC_DEADBAND && af < ACC_DEADBAND) ? 0.0 : af;
                v_imu += ae * dt * 100.0;
                if (v_imu < 0) v_imu = 0;
                win_sum += a; win_sum2 += a * a; win_n++;
                if (same >= IMU_FROZEN_N) imu_fault_begin("output frozen (identical samples)");
            } else if (++fails >= MAX_I2C_FAILS) {
                imu_fault_begin("I2C read errors");
            }
        } else if (S_IMU.st == SEN_RECOVERING) {
            imu_recover_step(t);
            if (rec.done) {                       /* re-seed after recovery */
                rec.done = 0;
                af = 0; fails = 0; same = 0; have_raw = 0; win_n = 0; win_sum = win_sum2 = 0;
                v_imu = (S_ENC.st == SEN_OK) ? v_enc_f : v_hold;
            }
        }

        /* ---------- fixed-interval block: velocity, motion, health ---------- */
        if ((t - t_iv) >= (uint64_t)VEL_INTERVAL_MS * 1000) {
            double div = (t - t_iv) * 1e-6;
            double v_raw = (counts - counts_iv) * cm_per_count / div;
            counts_iv = counts; t_iv = t;
            if (S_ENC.st == SEN_OK) v_enc_f = VEL_SMOOTH * v_raw + (1.0 - VEL_SMOOTH) * v_enc_f;

            if (win_n >= 5) {
                double m = win_sum / win_n, var = win_sum2 / win_n - m * m;
                double sdw = var > 0 ? __builtin_sqrt(var) : 0;
                moving = (sdw > move_thr) || (absd(af) > 0.3);
            }
            win_n = 0; win_sum = win_sum2 = 0;

            /* seed IMU velocity from the encoder -> no drift, clean hand-over */
            if (S_ENC.st == SEN_OK && S_IMU.st == SEN_OK) v_imu = v_enc_f;

            int imu_ok = (S_IMU.st == SEN_OK);
            int silent = (ts * 1000.0 > ENC_GRACE_MS) &&
                         ((t - t_last_change) > (uint64_t)ENC_FAULT_MS * 1000);

            if (S_ENC.st == SEN_OK && silent) {
                if (imu_ok && moving) {
                    set_fault(&S_ENC, "no pulses while the IMU detects motion (check DO wire / disc / sensor)");
                    edges_since_fault = 0;
                    gpio_mode(PIN_ENC, 0);
                } else if (imu_ok) {
                    r->reason = "STALLED: robot is not moving (check motors / driver / battery) - not a sensor fault";
                    r->aborted = 1; break;
                } else {
                    r->reason = "encoder silent and IMU unavailable - motion cannot be verified, safe stop";
                    r->aborted = 1; break;
                }
            } else if (S_ENC.st == SEN_FAULTY) {
                gpio_mode(PIN_ENC, 0);                           /* re-arm the pin */
                if (edges_since_fault >= ENC_RECOVER_COUNTS && (!imu_ok || moving)) {
                    enc_off = x - counts * cm_per_count;         /* resync to dead-reckoned x */
                    counts_iv = counts; t_iv = t; v_enc_f = v;
                    t_last_change = t;
                    set_ok(&S_ENC, "pulses are back, re-synchronised");
                }
            }

            /* robot not moving while the encoder is already lost */
            if (imu_ok && ts * 1000.0 > ENC_GRACE_MS) {
                if (moving) nomove_since = 0;
                else if (!nomove_since) nomove_since = t;
                else if (S_ENC.st != SEN_OK && (t - nomove_since) > (uint64_t)ENC_FAULT_MS * 1000) {
                    r->reason = "STALLED: IMU sees no motion and encoder is lost, safe stop";
                    r->aborted = 1; break;
                }
            }
        }

        /* ---------- choose the distance / velocity source ---------- */
        if (S_ENC.st == SEN_OK) {
            x = counts * cm_per_count + enc_off;
            v = v_enc_f; v_hold = v; src = "encoder";
        } else if (S_IMU.st == SEN_OK) {
            v = v_imu; x += v * dt; v_hold = v; src = "IMU dead-reckoning";
        } else {
            if (BOTH_LOST_STOP) {
                r->reason = "encoder AND IMU lost - safe stop";
                r->aborted = 1; break;
            }
            v = v_hold; x += v * dt; src = "velocity hold (no sensor)";
        }

        /* ---------- brake decision ---------- */
        double extra = (S_ENC.st == SEN_OK) ? 0.0 : (S_IMU.st == SEN_OK ? DEG_IMU_EXTRA_CM : DEG_BLIND_EXTRA_CM);
        double comp  = COAST_MIN_CM + COAST_K * v * v + extra;
        double remaining = travel_cm - x;
        if (remaining <= comp) { r->reason = "target distance reached"; break; }
        if (ts >= MAX_DRIVE_S) { r->reason = "safety time limit"; r->aborted = 1; break; }

        int duty = (remaining > SLOW_ZONE_CM) ? DUTY_FAR : (remaining > CREEP_ZONE_CM) ? DUTY_NEAR : DUTY_CREEP;
        if ((int)(tick++ % 10) < duty / 10) motors_forward();
        else                                motors_coast();
    }

    motors_brake();
    r->x_brake_cm = x; r->v_brake_cms = v;
    r->t_brake_s = (now_us() - t_start) * 1e-6;
    say("BRAKE: x=%.1f cm  v=%.1f cm/s  source=%s  (%s)", x, v, src, r->reason);

    /* ---------- settle: keep tracking until the robot has really stopped ---------- */
    uint64_t t0 = now_us(), still_since = 0;
    tprev = t0;
    while (now_us() - t0 < 1500000ull) {
        uint64_t t = now_us();
        double dt = (t - tprev) * 1e-6; tprev = t;

        int s = gpio_read(PIN_ENC);
        if (s == prev && s != last) {
            if (COUNT_BOTH_EDGES || s == 1) counts++;
            last = s; t_last_change = t;
        }
        prev = s;

        int still;
        if (S_ENC.st == SEN_OK) {
            x = counts * cm_per_count + enc_off;
            still = (t - t_last_change) > (uint64_t)ENC_STILL_MS * 1000;
        } else if (S_IMU.st == SEN_OK) {
            int16_t raw;
            if (mpu_raw(&raw) == 0) {
                double a = raw_to_ms2(raw) - bias;
                af += ACC_ALPHA * (a - af);
                double ae = (af > -ACC_DEADBAND && af < ACC_DEADBAND) ? 0.0 : af;
                v_imu += ae * dt * 100.0; if (v_imu < 0) v_imu = 0;
                x += v_imu * dt;
                if (af > -ZUPT_ACC && af < ZUPT_ACC) {
                    if (!still_since) still_since = t;
                } else still_since = 0;
            }
            still = still_since && (t - still_since) > (uint64_t)ENC_STILL_MS * 1000;
        } else {
            still = (t - t0) > 800000ull;
        }
        if (still) break;
        usleep(LOOP_US);
    }
    motors_coast();
    r->x_total_cm = x;
    r->counts = counts;
}

/* ============================ Main ============================ */
int main(int argc, char **argv)
{
    double manual_dist = 0;
    int selftest = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) manual_dist = atof(argv[++i]);
        else if (strcmp(argv[i], "-s") == 0)           selftest = 1;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1) { perror("ThreadCtl (run as root)"); return 1; }
    if (gpio_init() != 0)                 { perror("GPIO mmap");               return 1; }

    struct sched_param sp = { .sched_priority = PRIO_RT };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);

    gpio_mode(PIN_TRIG, 1); gpio_mode(PIN_ECHO, 0); gpio_mode(PIN_ENC, 0);
    gpio_mode(PIN_IN1, 1);  gpio_mode(PIN_IN2, 1);
    gpio_mode(PIN_IN3, 1);  gpio_mode(PIN_IN4, 1);
    motors_brake();
    t_origin = now_us();

    /* ---------- IMU start-up (with retries) ---------- */
    int imu_up = 0;
    for (int i = 0; i <= IMU_MAX_RECOVER && !imu_up; i++) {
        if (mpu_init() == 0) {
            imu_up = 1;
            if (i > 0) set_ok(&S_IMU, "re-init after start-up failure");
        } else {
            if (i == 0) set_fault(&S_IMU, "not found on I2C (WHO_AM_I / wiring / address 0x68)");
            if (i < IMU_MAX_RECOVER) { say(">>> MPU6050 init retry %d/%d", i + 1, IMU_MAX_RECOVER); sleep_ms(300); }
        }
    }

    /* ---------- rest phase ---------- */
    static rest_t R;
    memset(&R, 0, sizeof R);
    printf("Keep the robot STILL for %d ms...\n", REST_TIME_MS);
    fflush(stdout);
    rest_collect(&R, REST_TIME_MS, 1, imu_up);
    gpio_write(PIN_TRIG, 0);

    char why[96];
    double dist_cm = 0, bias = 0, sd = 0;

    /* ultrasonic: check + retry */
    int have_dist = 0;
    for (int a = 0; a <= US_RETRIES; a++) {
        if (us_analyze(&R, &dist_cm, why, sizeof why) == 0) {
            have_dist = 1;
            if (a > 0) set_ok(&S_US, "valid distance after retry");
            break;
        }
        if (a == 0) set_fault(&S_US, why);
        if (a < US_RETRIES) {
            say(">>> Ultrasonic retry %d/%d", a + 1, US_RETRIES);
            R.us_n = 0; R.us_tries = 0;
            rest_collect(&R, 1500, 1, 0);
        }
    }
    gpio_write(PIN_TRIG, 0);
    gpio_mode(PIN_TRIG, 0);                       /* ultrasonic OFF from here */

    /* IMU: check + retry */
    int imu_ok = 0;
    if (imu_up) {
        for (int a = 0; a <= IMU_MAX_RECOVER; a++) {
            if (imu_analyze(&R, &bias, &sd, why, sizeof why) == 0) {
                imu_ok = 1;
                if (a > 0) set_ok(&S_IMU, "clean rest data after retry");
                break;
            }
            if (a == 0) set_fault(&S_IMU, why);
            if (a < IMU_MAX_RECOVER) {
                say(">>> MPU6050 retry %d/%d (re-init + re-sample)", a + 1, IMU_MAX_RECOVER);
                mpu_init();
                rest_reset_imu(&R);
                rest_collect(&R, 1500, 0, 1);
            }
        }
    }
    if (!imu_ok) { S_IMU.st = SEN_FAULTY; bias = 0; sd = 0; }
    else printf("IMU bias = %.4f m/s^2, noise = %.4f m/s^2\n", bias, sd);

    /* ---------- self-test mode ---------- */
    if (selftest) {
        printf("Spin a wheel BY HAND now (4 s) to test the encoder...\n");
        fflush(stdout);
        long edges = 0; int p = gpio_read(PIN_ENC), l = p;
        uint64_t t0 = now_us();
        while (now_us() - t0 < 4000000ull) {
            int v = gpio_read(PIN_ENC);
            if (v == p && v != l) { edges++; l = v; }
            p = v;
            usleep(500);
        }
        if (edges < 4) set_fault(&S_ENC, "no pulses while the wheel was turned (check DO wire / disc / sensor)");
        else           printf("Encoder: %ld edges counted.\n", edges);
        if (have_dist) printf("Ultrasonic distance: %.1f cm\n", dist_cm);
        print_status();
        return 0;
    }

    /* ---------- distance: ultrasonic, else manual fallback ---------- */
    if (!have_dist) {
        if (manual_dist > 0) {
            dist_cm = manual_dist;
            say(">>> RECOVERY: ultrasonic unusable, using manual distance %.1f cm (-d)", dist_cm);
        } else {
            printf("\nCannot determine the distance and no manual distance given.\n"
                   "Fix the ultrasonic sensor or run again with  -d <distance_cm>.  NOT MOVING.\n");
            print_status();
            return 2;
        }
    }

    /* ---------- real run ---------- */
    double travel_cm = dist_cm - SENSOR_OFFSET_CM - STOP_GAP_CM - SAFETY_MARGIN_CM;
    printf("Distance to object : %.1f cm%s\n", dist_cm, have_dist ? "" : "  (manual)");
    printf("Travel required    : %.1f cm (incl. %.1f cm safety margin)\n", travel_cm, SAFETY_MARGIN_CM);
    print_status();
    if (travel_cm <= 0.0)          { printf("Already inside the gap. Not moving.\n"); return 0; }
    if (travel_cm > MAX_TRAVEL_CM) { printf("Travel exceeds safety limit.\n");        return 3; }

    run_t r;
    memset(&r, 0, sizeof r);
    drive(travel_cm, bias, sd, &r);

    printf("\nStopped by          : %s\n", r.reason);
    printf("Drive time          : %.2f s\n", r.t_brake_s);
    printf("Distance @brake     : %.1f cm   speed @brake: %.1f cm/s\n", r.x_brake_cm, r.v_brake_cms);
    printf("Distance total      : %.1f cm   (encoder counts: %ld)\n", r.x_total_cm, r.counts);
    printf("Expected final gap  : %.1f cm\n", dist_cm - SENSOR_OFFSET_CM - r.x_total_cm);
    double coast = r.x_total_cm - r.x_brake_cm;
    if (r.v_brake_cms > 5.0 && S_ENC.st == SEN_OK)
        printf("Coasted after brake : %.1f cm  -> suggested COAST_K = %.5f\n",
               coast, (coast - COAST_MIN_CM) / (r.v_brake_cms * r.v_brake_cms));
    if (S_ENC.faults == 0 && r.counts == 0 && !r.aborted)
        say("NOTE: no encoder pulses were ever counted");
    print_status();
    printf("MEASURE the real gap with a ruler; adjust COAST_K / COAST_MIN_CM if needed.\n");
    return r.aborted ? 4 : 0;
}
