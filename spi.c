#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "spi.h"


// Register info
char *reg_name[] = {
	"STATUS",
	"MUX   ",
	"ADCON ",
	"DRATE ",
	"IO    ",
	"OFC0  ",
	"OFC2  ",
	"OFC3  ",
	"FSC0  ",
	"FSC1  ",
	"FSC2  ",
};

// Reg
#define REG_MUX	0x1
#define REG_ADCON 0x2
#define REG_DRATE 0x3


// CMD info
#define CMD_RESET	0xFE
#define CMD_RDATA	0x01
#define CMD_RDATAC	0x03
#define CMD_RREG	0x10
#define CMD_WREG	0x50
#define CMD_STANDBY	0xFD
#define CMD_WAKEUP	0xFF
#define SPI_SPEED	1000000 // 1MHz

static int sensor_fd = 0;
static int sensor_fd_1 = 0;
static int sensor_fd_2 = 0; 

static void spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len, uint8_t cs)
{
	struct spi_ioc_transfer tr = {0};

	tr.tx_buf        = (unsigned long)tx;
	tr.rx_buf        = (unsigned long)rx;
	tr.len           = len; // cmd: 2byte + data: 1byte
	tr.speed_hz      = SPI_SPEED;
	tr.cs_change     = cs;
	tr.bits_per_word = 8;

	if (ioctl(sensor_fd, SPI_IOC_MESSAGE(1), &tr) < 1)
		printf(" error SPI transfer\n");
}


/*
 * READ REGISTER (CMD RREG)
 */
uint8_t read_reg(uint8_t addr)
{
	uint8_t tx1[2] = { CMD_RREG | addr, 0x00 }; // read one data
	uint8_t rx1[2] = { 0, 0};

	// transfer
	spi_xfer(tx1, rx1, sizeof(tx1), 1);

	// wait
	usleep(20);

	uint8_t tx2[1] = {0};
	uint8_t rx2[1] = {0};
	// recieve
	spi_xfer(tx2, rx2, sizeof(tx2), 0);

	// wait
	usleep(20);

	return rx2[0];
}

/*
 * WRITE REGISTER (CMD WREG)
 */
void write_reg(uint8_t addr, uint8_t val, uint8_t mask)
{
	uint8_t rd    = read_reg(addr);
	uint8_t reg   = (rd & ~mask) | (val & mask);
	uint8_t tx[3] = { CMD_WREG | addr, 0x00, reg };

	spi_xfer(tx, NULL, sizeof(tx), 0);

	// wait
	usleep(20);
}

void dump_reg(void)
{
	uint8_t tx1[3]  = { CMD_RREG | 0x00, 0xb };
	uint8_t rx1[3]  = { 0, 0, 0 };

	struct spi_ioc_transfer tr1 = {0};
	tr1.tx_buf = (unsigned long)tx1;
	tr1.rx_buf = (unsigned long)rx1;
	tr1.len = 2;
	tr1.cs_change = 1;
	tr1.speed_hz = SPI_SPEED;
	tr1.bits_per_word = 8;

	if (ioctl(sensor_fd, SPI_IOC_MESSAGE(1), &tr1) < 1)
		printf(" error SPI transfer1\n");

	// wait
	usleep(20);

	uint8_t tx2[11]; memset(tx2, 0x0, 11);
	uint8_t rx2[11]; memset(rx2, 0xa, 11);

        struct spi_ioc_transfer tr2 = {0};
        tr2.tx_buf = (unsigned long)tx2;
        tr2.rx_buf = (unsigned long)rx2;
        tr2.len = 11;
	tr2.cs_change = 0;
        tr2.speed_hz = SPI_SPEED;
        tr2.bits_per_word = 8;

	if (ioctl(sensor_fd, SPI_IOC_MESSAGE(1), &tr2) < 1)
		printf(" error SPI transfer2\n");

	for (int i=0; i<11; i++) {
		printf("%s[%x]\n", reg_name[i], rx2[i]);
	}

	// wait
	usleep(20);
}

/*
 * command
 */
int32_t device_rdata(void)
{
	uint8_t cmd = CMD_RDATA;
	int32_t ret;

	spi_xfer(&cmd, NULL, 1, 1);

	usleep(20);

	uint8_t tx[3] = {0};
	uint8_t rx[3] = {0};

	// recieve
	spi_xfer(tx, rx, sizeof(tx), 0);

	ret = ((int32_t)rx[0] << 16) | ((int32_t)rx[1] << 8) | (int32_t)rx[2];

	//printf(" read data %02x %02x %02x\n", rx[0], rx[1], rx[2]);
	// wait
	usleep(20);


	return ret;
}

static void device_reset(void)
{
	uint8_t cmd = CMD_RESET;

	// transfer
	spi_xfer(&cmd, NULL, 1, 0);

	// wait preparation of device
	usleep(20000); // 20ms
}

void device_standby(void)
{
	uint8_t cmd = CMD_STANDBY;

	// transfer
	spi_xfer(&cmd, NULL, 1, 0);

	// wait preparation of device
	usleep(1000); // 1ms
}

void device_wakeup(void)
{
	uint8_t cmd = CMD_WAKEUP;

	// transfer
	spi_xfer(&cmd, NULL, 1, 0);

	// wait preparation of device
	usleep(1000); // 1ms
}


/*
 * utils
 */
static void spi_setting(void)
{
//	write_reg(REG_ADCON, 0x7, 0x7); // Gain 64
	write_reg(REG_ADCON, 0x5, 0x7); // Gain 32

	// gain caribration
	usleep(30000); // 30ms

	write_reg(REG_DRATE, 0x42, 0xff); // 100 SPS
}

void  device_preparation(void)
{
	device_reset();
	spi_setting();
}

void switch_sensor(uint8_t sensor)
{
	if (sensor == 0)
		sensor_fd = sensor_fd_1;
	else
		sensor_fd = sensor_fd_2;
}


void switch_ch(uint8_t ch)
{
	switch (ch) {
	case 0:
		write_reg(REG_MUX, 0x01, 0xff); // AIN0 AIN1
		break;
	case 1:
		write_reg(REG_MUX, 0x23, 0xff); // AIN2 AIN3
		break;
	case 2:
		write_reg(REG_MUX, 0x45, 0xff); // AIN4 AIN5
		break;
	case 3:
		write_reg(REG_MUX, 0x67, 0xff); // AIN6 AIN7
		break;
	default:
		break;
	}
}

uint32_t get_gain(void)
{
	uint8_t gain;

	gain = read_reg(REG_ADCON) & 0x7;

	return (gain == 0x7 ? 64 : (1 << gain));
}

void spi_init(void)
{
	uint8_t mode = SPI_MODE_1;   // recommend for ads1256
	uint8_t bits = 8;            // recommend for ads1256
	uint32_t speed = SPI_SPEED;  // 1MHz

	ioctl(sensor_fd, SPI_IOC_WR_MODE, &mode);
	ioctl(sensor_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	ioctl(sensor_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
}

void spi_open(void)
{
	sensor_fd_1 = open("/dev/spidev0.0", O_RDWR);
	sensor_fd_2 = open("/dev/spidev0.1", O_RDWR);

	sensor_fd = sensor_fd_1;
}

void spi_close(void)
{
	close(sensor_fd_1);
	close(sensor_fd_2);
}

