# Wiring and Priorities: QNX Fault-Tolerant Robot

Target: Raspberry Pi 4 running QNX. Pin numbers are the **physical header pin** and the **BCM GPIO** number.

## 1. Connections

### Ultrasonic HC-SR04 (5 V part: ECHO needs a divider)
| HC-SR04 | Pi pin | BCM |
|---|---|---|
| VCC | Pin 2 (5V) | - |
| GND | Pin 6 (GND) | - |
| TRIG | Pin 16 | GPIO23 |
| ECHO | **1 kOhm -> node -> Pin 18**, node **-> 2 kOhm -> GND** | GPIO24 |

ECHO swings 0 to 5 V and the Pi GPIO tolerates 3.3 V max. The 1k/2k divider gives about 3.3 V.

### IMU MPU6050 (GY-521)
| MPU6050 | Pi pin | BCM |
|---|---|---|
| VCC | Pin 1 (3V3) | - |
| GND | Pin 9 (GND) | - |
| SDA | Pin 3 | GPIO2 (I2C1 SDA) |
| SCL | Pin 5 | GPIO3 (I2C1 SCL) |
| AD0 | GND (address 0x68) | - |

Mount the board so that **+X points forward** (the code uses the X axis as forward acceleration).

### Wheel encoder (LM393 slotted opto module, one wheel)
| Encoder | Pi pin | BCM |
|---|---|---|
| VCC | Pin 17 (3V3) | - |
| GND | Pin 14 (GND) | - |
| DO | Pin 11 | GPIO17 |

Power it from 3.3 V so DO is safe for the Pi. Set `ENC_PPR` and `WHEEL_DIAM_M` in the code to your disc and wheel.

### Motor driver L298N + 2 DC motors
| L298N | Connect to |
|---|---|
| IN1 | Pin 29 (GPIO5) |
| IN2 | Pin 31 (GPIO6) |
| IN3 | Pin 33 (GPIO13) |
| IN4 | Pin 35 (GPIO19) |
| GND | Pin 34 (Pi GND) **and** battery negative (common ground) |
| +12V terminal | Battery positive (7-12 V pack) |
| ENA / ENB | Leave the jumpers fitted (full-on) |
| 5V out | **Do not connect to the Pi** |
| OUT1/2, OUT3/4 | Left / right motor |

Power the Pi from its own supply (power bank). Speed is held by bang-bang on/off at 50 Hz, so no PWM pins are needed.

```
                    Raspberry Pi 4 (QNX)
 HC-SR04 TRIG  <--- GPIO23 (16)        GPIO5/6  (29/31) ---> L298N IN1/IN2
 HC-SR04 ECHO  ---> divider --> GPIO24 (18)   GPIO13/19 (33/35) ---> L298N IN3/IN4
 Encoder DO    ---> GPIO17 (11)
 MPU6050 SDA/SCL <-> GPIO2/GPIO3 (3/5)
 Common GND: Pi (34) = L298N GND = battery (-) = all sensor GNDs
```

### Fault-injection points for the demo
| Scenario | What you do |
|---|---|
| Ultrasonic failure | Pull the TRIG or ECHO wire |
| IMU failure | Pull SDA (or SCL) |
| Encoder failure | Pull the DO wire, or block the slot sensor with tape |
| Recovery | Plug the wire back in |

## 2. Task priorities (QNX SCHED_FIFO)

| Task | Period | Prio | Why |
|---|---|---|---|
| Encoder sampler | 1 ms | **56** | Shortest period (rate-monotonic). It polls the GPIO level and must not miss slots. |
| Fault Manager + Recovery | 20 ms | **54** | Safety supervisor. It sits above everything that consumes its verdict. |
| Control | 20 ms | **52** | Runs just below the Fault Manager so it always sees fresh health. It also gets an instant pulse on mode change. |
| IMU acquisition | 20 ms | **50** | Producer task. |
| Ultrasonic acquisition | 60 ms | **48** | Longest period. It busy-waits on the echo for up to about 30 ms, so everything important must be able to preempt it. |
| Logger / dashboard | 200 ms | **20** | Must never disturb the real-time tasks. |

Rules applied:
1. Shorter period gets higher priority (rate-monotonic scheduling); ties are broken by criticality.
2. Everything is `SCHED_FIFO` with `PTHREAD_EXPLICIT_SCHED`, so the priorities are really applied.
3. One shared mutex uses `PTHREAD_PRIO_INHERIT`, so there is no priority inversion. Critical sections are copies only, and the logger prints outside the lock.
4. The Fault Manager notifies the Control task with a **QNX pulse** (`MsgSendPulse`) the moment the system mode changes. Control also has a watchdog: if the Fault Manager stops running for more than 100 ms, the motors stop.
5. Every task records its worst-case execution time, shown on the dashboard.

The "Recovery Task" from your concept is implemented inside the Fault Manager thread (the `H_FAILED -> H_RECOVERING -> H_HEALTHY` branch). That avoids a second thread racing on the same health state.

## 3. Build and run
```
source ~/qnx800/qnxsdp-env.sh      # your SDP path
make
scp qnx_ftrobot root@<pi-ip>:/tmp/
# on the Pi (as root)
ls /dev/i2c*                       # /dev/i2c1 must exist
/tmp/qnx_ftrobot
```
Keep the robot **still for the first 3 s** (IMU bias calibration, `INIT` mode).

## 4. Behaviour implemented
- States: HEALTHY -> SUSPECT -> FAILED -> RECOVERING -> HEALTHY.
- Detection: a sensor must be bad for 8 consecutive 20 ms ticks (160 ms) before it is declared FAILED.
- Checks: timeout/comm error, stuck value, out-of-range, impossible jump, and 2-of-3 velocity voting.
- Recovery: 5 **consecutive** fresh readings within +/-10 % of the reference (with a per-sensor floor, because 10 % of 0 m/s is impossible). A single bad reading resets the counter to 0.
- Modes: NORMAL, DEGRADED (speed halved, obstacle stop only while the ultrasonic sensor is healthy), SAFE (all failed: motors stop).

## 5. Things to check or tune on your hardware
- **GPIO and I2C access are in one HAL block.** GPIO uses a direct `mmap` of the BCM2711 registers at `0xFE200000`. I2C uses `devctl(DCMD_I2C_SENDRECV)` on `/dev/i2c1`. I could only syntax-check the file here, not run it on QNX, so confirm that your QNX image provides `/dev/i2c1` and allows the GPIO mmap. If your BSP prefers its own GPIO or I2C library, only the HAL functions need to change.
- The ultrasonic velocity assumes a **static target** in front of the robot, so point it at a wall during the demo. If the beam sees nothing, it reports "no target" (not a fault).
- The encoder has a single channel, so it cannot tell direction (forward only).
- IMU velocity is dead-reckoned with a slow (5 s) pull toward the fused reference, plus a zero-velocity reset whenever the robot is stopped. That makes it a short-term backup, not a long-term odometer.
- Tune `FLOOR_V[]`, `REL_DET`, `STRIKES_TO_FAIL` and `V_TARGET` first if you get false alarms.
