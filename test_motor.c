/* DC motor driver test (L298N / L293D / TB6612 style: IN1, IN2, EN/PWM)
 * IN1=GPIO5 (pin29)  IN2=GPIO6 (pin31)  EN/PWM=GPIO13 (pin33)
 * Motor supply is EXTERNAL; connect driver GND to Pi GND.
 * Software PWM (~500 Hz). Lift the wheels off the ground for the first test! */
#include "qgpio.h"
#include <signal.h>
#define IN1 5
#define IN2 6
#define EN  13
#define PERIOD_US 2000

static volatile int run = 1;
static void stop(int s) { (void)s; run = 0; }

static void drive(int dir, int duty, int ms) {   /* dir: 1 fwd, -1 rev, 0 stop; duty 0-100 */
    gpio_write(IN1, dir > 0); gpio_write(IN2, dir < 0);
    uint64_t end = now_us() + (uint64_t)ms * 1000;
    while (run && now_us() < end) {
        if (duty > 0)   { gpio_write(EN, 1); nanospin_ns((unsigned long)PERIOD_US * duty * 10); }
        if (duty < 100) { gpio_write(EN, 0); nanospin_ns((unsigned long)PERIOD_US * (100 - duty) * 10); }
    }
}

int main(void) {
    if (gpio_init() < 0) return 1;
    signal(SIGINT, stop);
    gpio_mode(IN1, GPIO_OUT); gpio_mode(IN2, GPIO_OUT); gpio_mode(EN, GPIO_OUT);
    gpio_write(EN, 0);

    printf("Forward 40%%\n");  drive(1, 40, 2000);
    printf("Forward 80%%\n");  drive(1, 80, 2000);
    printf("Stop\n");          drive(0, 0, 1000);
    printf("Reverse 40%%\n");  drive(-1, 40, 2000);
    printf("Reverse 80%%\n");  drive(-1, 80, 2000);
    printf("Stop\n");

    gpio_write(EN, 0); gpio_write(IN1, 0); gpio_write(IN2, 0);
    return 0;
}
