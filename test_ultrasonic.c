/* HC-SR04 test.  TRIG=GPIO23 (pin16)  ECHO=GPIO24 (pin18)
 * !! ECHO is 5V: use a divider (1k + 2k) to bring it to 3.3V !! */
#include "qgpio.h"
#include <unistd.h>
#define TRIG 23
#define ECHO 24
#define TIMEOUT_US 30000

static double measure_cm(void) {
    gpio_write(TRIG, 0); nanospin_ns(2000);
    gpio_write(TRIG, 1); nanospin_ns(10000);
    gpio_write(TRIG, 0);

    uint64_t t0 = now_us();
    while (!gpio_read(ECHO)) if (now_us() - t0 > TIMEOUT_US) return -1;
    uint64_t start = now_us();
    while (gpio_read(ECHO))  if (now_us() - start > TIMEOUT_US) return -2;
    uint64_t end = now_us();
    return (double)(end - start) / 58.0;   /* us -> cm */
}

int main(void) {
    if (gpio_init() < 0) return 1;
    gpio_mode(TRIG, GPIO_OUT);
    gpio_mode(ECHO, GPIO_IN);
    gpio_pull(ECHO, PULL_DOWN);
    gpio_write(TRIG, 0);
    usleep(100000);
    for (int i = 0; i < 100; i++) {
        double d = measure_cm();
        if (d < 0) printf("timeout/no echo (%.0f)\n", d);
        else       printf("Distance: %.1f cm\n", d);
        usleep(200000);
    }
    return 0;
}
