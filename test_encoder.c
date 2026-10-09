/* Rotary (quadrature) encoder test.  A=GPIO17 (pin11)  B=GPIO27 (pin13)  common/GND -> GND
 * Internal pull-ups enabled. Rotate the shaft; prints count + direction. Ctrl-C to quit. */
#include "qgpio.h"
#include <signal.h>
#define PIN_A 17
#define PIN_B 27
static volatile int run = 1;
static void stop(int s) { (void)s; run = 0; }

int main(void) {
    if (gpio_init() < 0) return 1;
    signal(SIGINT, stop);
    gpio_mode(PIN_A, GPIO_IN); gpio_mode(PIN_B, GPIO_IN);
    gpio_pull(PIN_A, PULL_UP); gpio_pull(PIN_B, PULL_UP);

    /* state-transition table: index = (prev<<2)|curr */
    static const int8_t tbl[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
    int prev = (gpio_read(PIN_A) << 1) | gpio_read(PIN_B);
    long count = 0, last_print = 0;
    printf("Rotate encoder... (Ctrl-C to quit)\n");
    while (run) {
        int cur = (gpio_read(PIN_A) << 1) | gpio_read(PIN_B);
        if (cur != prev) {
            count += tbl[(prev << 2) | cur];
            prev = cur;
            if (count != last_print) {
                printf("count=%ld  (%s)\n", count, count > last_print ? "CW" : "CCW");
                last_print = count;
            }
        }
        nanospin_ns(20000);   /* ~50 kHz polling */
    }
    printf("Final count: %ld\n", count);
    return 0;
}
