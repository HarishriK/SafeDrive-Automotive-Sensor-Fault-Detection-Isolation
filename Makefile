# Cross-compile on host after:  source ~/qnx710/qnxsdp-env.sh   (or qnx800)
# Pi 4 = aarch64 little-endian
CC      = qcc
VARIANT = gcc_ntoaarch64le
CFLAGS  = -V$(VARIANT) -Wall -O2
LDFLAGS = -V$(VARIANT)

TARGETS = test_ultrasonic test_mpu6050 test_encoder test_motor

all: $(TARGETS)

test_%: test_%.c qgpio.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

# make deploy PI=192.168.x.x
PI ?= 192.168.1.50
deploy: all
	scp $(TARGETS) qnxuser@$(PI):/data/home/qnxuser/

clean:
	rm -f $(TARGETS) *.o
