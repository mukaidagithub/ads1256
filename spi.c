#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>


#define SPI_SPEED  50000
#define CMD_RESET   0xFE  // reset
#define CMD_SDATAC  0x0F  // Stop Read Data Continuous
#define CMD_RREG    0x10  // | (reg_addr) ; 次バイトで n-1 を渡す
#define CMD_WREG    0x50  // | (reg_addr)
#define CMD_RDATA   0x01  // read data
#define CMD_SYNC    0xFC  // sync
#define CMD_WAKEUP  0xFF  // wakeup

static int fd = -1;

static int spi_init(void) {
	fd = open("/dev/spidev0.0", O_RDWR);
	if (fd < 0) return -1;

	uint8_t mode = SPI_MODE_1;
	uint8_t bits = 8;
	uint32_t speed = SPI_SPEED;

	ioctl(fd, SPI_IOC_WR_MODE, &mode);
	ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

	return 0;
}


static void spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len) {
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.speed_hz = SPI_SPEED,
		.bits_per_word = 8,
		.delay_usecs = 0,
	};

	if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1)
		printf(" error SPI transfer\n");
}


static void device_reset(void) {
	uint8_t cmd = CMD_RESET;
	spi_xfer(&cmd, NULL, 1);
	usleep(50000);
}


static void device_stop_continuous(void) {
	uint8_t cmd = CMD_SDATAC;
	spi_xfer(&cmd, NULL, 1);
	usleep(10);
}

static void device_sync(void) {
	uint8_t cmd = CMD_SYNC;
	spi_xfer(&cmd, NULL, 1);
	usleep(1000);
}

static void device_wakeup(void) {
	uint8_t cmd = CMD_WAKEUP;
	spi_xfer(&cmd, NULL, 1);
	usleep(1000);
}

static void device_read(void) {
	device_sync();
	device_wakeup();

	uint8_t tx[4] = {CMD_RDATA, 0xFF, 0xFF, 0xFF};
	uint8_t rx[4]  = {0};
	//spi_xfer(&tx, NULL, 1);
	spi_xfer(tx, rx, sizeof(rx));
	usleep(1000);

	printf(" Data %x %x %x\n", rx[1], rx[2], rx[3]);
}

static uint8_t device_read_reg(uint8_t addr) {
	uint8_t tx[3] = {CMD_RREG | addr, 0x00, 0xFF};
	uint8_t rx[3] = {0};

	spi_xfer(tx, rx, sizeof(tx));

	return rx[2];
}

static void device_write_reg(uint8_t addr, uint8_t val, uint8_t mask) {
	device_stop_continuous();

	uint8_t reg = (device_read_reg(addr) & ~mask) | (val & mask);

	uint8_t tx[3] = {CMD_WREG | addr, 0x00, reg };
	spi_xfer(tx, NULL, sizeof(tx));
}

static void dump_reg(void) {
	char* regs[11] = {"STATUS", "MUX", "ADCON", "DRATE", "IO", "OFC0", "OFC1", "OFC2", "FSC0", "FSC1", "FSC2"};

	for (int i=0; i<11; i++) {
		printf(" REG[%s] = 0x%02X\n", regs[i], device_read_reg(i));
	}
	printf("--------------------------------------------------\n");
}


int main(void)
{
	if (spi_init() < 0) {
		printf(" spi_init error\n");
		return -1;
	}

	device_reset();
	device_stop_continuous();

	dump_reg();

	device_write_reg(0x2, 0x7, 0x7); // ADCON GAIN 64

	dump_reg();

	device_read();

	device_write_reg(0x1, 0x23, 0xff); // MUX AIN2/AIN3
	device_read();

	return 0;
}
