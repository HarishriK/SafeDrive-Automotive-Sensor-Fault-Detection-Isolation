/* Minimal GPIO access for Raspberry Pi 4 (BCM2711) on QNX via memory-mapped registers.
 * No external library needed. Must run as root. */
#ifndef QGPIO_H
#define QGPIO_H
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/mman.h>

#define GPIO_BASE 0xFE200000UL   /* BCM2711 peripheral GPIO base */
#define GPIO_LEN  0x1000

static volatile uint32_t *gpio_reg;
enum { GPIO_IN = 0, GPIO_OUT = 1 };
enum { PULL_NONE = 0, PULL_UP = 1, PULL_DOWN = 2 };

static int gpio_init(void) {
    void *p = mmap_device_memory(NULL, GPIO_LEN, PROT_READ | PROT_WRITE | PROT_NOCACHE, 0, GPIO_BASE);
    if (p == MAP_FAILED) { perror("mmap_device_memory (run as root)"); return -1; }
    gpio_reg = (volatile uint32_t *)p;
    return 0;
}
static void gpio_mode(int pin, int mode) {
    volatile uint32_t *r = &gpio_reg[pin / 10];
    int sh = (pin % 10) * 3;
    uint32_t v = *r;
    v &= ~(7u << sh);
    v |= ((uint32_t)mode << sh);
    *r = v;
}
static inline void gpio_write(int pin, int v) {
    if (v) gpio_reg[0x1C / 4 + pin / 32] = 1u << (pin % 32);   /* GPSET */
    else   gpio_reg[0x28 / 4 + pin / 32] = 1u << (pin % 32);   /* GPCLR */
}
static inline int gpio_read(int pin) {
    return (gpio_reg[0x34 / 4 + pin / 32] >> (pin % 32)) & 1;  /* GPLEV */
}
static void gpio_pull(int pin, int pull) {   /* BCM2711 pull register */
    volatile uint32_t *r = &gpio_reg[0xE4 / 4 + pin / 16];
    int sh = (pin % 16) * 2;
    uint32_t v = *r;
    v &= ~(3u << sh);
    v |= ((uint32_t)pull << sh);
    *r = v;
}
static inline uint64_t now_us(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000ULL + t.tv_nsec / 1000;
}
#endif
