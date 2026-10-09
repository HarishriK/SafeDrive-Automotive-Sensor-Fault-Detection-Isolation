/* MPU6050 test over I2C1 (SDA=GPIO2 pin3, SCL=GPIO3 pin5, VCC=3.3V, AD0=GND -> addr 0x68)
 * Uses QNX native I2C resource manager (/dev/i2c1) via devctl - no extra library. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <devctl.h>
#include <hw/i2c.h>

#define I2C_DEV  "/dev/i2c1"
#define MPU_ADDR 0x68

static int fd;

static int reg_write(uint8_t reg, uint8_t val) {
    struct { i2c_send_t hdr; uint8_t d[2]; } m;
    memset(&m, 0, sizeof(m));
    m.hdr.slave.addr = MPU_ADDR; m.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    m.hdr.len = 2; m.hdr.stop = 1;
    m.d[0] = reg; m.d[1] = val;
    return devctl(fd, DCMD_I2C_SEND, &m, sizeof(m.hdr) + 2, NULL);
}
static int reg_read(uint8_t reg, uint8_t *out, int n) {
    struct { i2c_sendrecv_t hdr; uint8_t d[16]; } m;
    memset(&m, 0, sizeof(m));
    m.hdr.slave.addr = MPU_ADDR; m.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    m.hdr.send_len = 1; m.hdr.recv_len = n; m.hdr.stop = 1;
    m.d[0] = reg;
    int r = devctl(fd, DCMD_I2C_SENDRECV, &m, sizeof(m.hdr) + (n > 1 ? n : 1), NULL);
    if (r == EOK) memcpy(out, m.d, n);
    return r;
}

int main(void) {
    fd = open(I2C_DEV, O_RDWR);
    if (fd < 0) { perror("open " I2C_DEV " (is the i2c driver running?)"); return 1; }

    uint8_t id = 0;
    if (reg_read(0x75, &id, 1) != EOK) { fprintf(stderr, "No response from MPU6050 (check wiring/address)\n"); return 1; }
    printf("WHO_AM_I = 0x%02X (expect 0x68)\n", id);

    reg_write(0x6B, 0x00);   /* wake up, internal 8MHz osc */
    reg_write(0x1B, 0x00);   /* gyro  +-250 dps  -> 131 LSB/(deg/s) */
    reg_write(0x1C, 0x00);   /* accel +-2 g      -> 16384 LSB/g     */
    usleep(100000);

    for (int i = 0; i < 100; i++) {
        uint8_t b[14];
        if (reg_read(0x3B, b, 14) != EOK) { fprintf(stderr, "read error\n"); break; }
        int16_t ax = (b[0] << 8) | b[1],  ay = (b[2] << 8) | b[3],  az = (b[4] << 8) | b[5];
        int16_t tp = (b[6] << 8) | b[7];
        int16_t gx = (b[8] << 8) | b[9],  gy = (b[10] << 8) | b[11], gz = (b[12] << 8) | b[13];
        printf("Acc[g] %6.2f %6.2f %6.2f | Gyro[dps] %7.1f %7.1f %7.1f | T=%.1fC\n",
               ax / 16384.0, ay / 16384.0, az / 16384.0,
               gx / 131.0, gy / 131.0, gz / 131.0, tp / 340.0 + 36.53);
        usleep(200000);
    }
    close(fd);
    return 0;
}
