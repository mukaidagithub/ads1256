#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>


#define SPI_SPEED  500000 // 500khz
//#define SPI_SPEED  10000
#define CMD_RESET   0xFE  // reset
#define CMD_SDATAC  0x0F  // Stop Read Data Continuous
#define CMD_RREG    0x10  // | (reg_addr) ; 次バイトで n-1 を渡す
#define CMD_WREG    0x50  // | (reg_addr)
#define CMD_RDATA   0x01  // read data
#define CMD_SYNC    0xFC  // sync
#define CMD_WAKEUP  0xFF  // wakeup
#define VREF	5  // 5V
#define GAIN	64

static void device_reset();
static int fd = -1;

static int spi_init(void) {
	fd = open("/dev/spidev0.0", O_RDWR);
	if (fd < 0) {
		printf(" =====> device open error\n");
		return -1;
	}

	uint8_t mode = SPI_MODE_1;
	uint8_t bits = 8;
	uint32_t speed = SPI_SPEED;

	ioctl(fd, SPI_IOC_WR_MODE, &mode);
	ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

	return 0;
}

static void spi_deinit(void) {
	device_reset();
	
	close(fd);
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

static double device_code_to_mV(int32_t code, double vref, int gain) {
	const double full_scale = 8388607.0;  // 2^23 - 1
	return ((double)code / full_scale) * (vref * 1000.0 / gain);
}

static double device_read(void) {
	device_sync();
	device_wakeup();

	uint8_t tx[4] = {CMD_RDATA, 0xFF, 0xFF, 0xFF};
	uint8_t rx[4]  = {0};
	//spi_xfer(&tx, NULL, 1);
	spi_xfer(tx, rx, sizeof(rx));
	usleep(1000);

	int32_t code = (rx[1] << 16) | (rx[2] << 8) | rx[3];
	if (code & 0x800000) code |= 0xFF000000;

	printf(" Voltage = %.6f mV   Data %x%x%x\n",
		device_code_to_mV(code, VREF, GAIN), rx[1], rx[2], rx[3]);

	return device_code_to_mV(code, VREF, GAIN);
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

	for (int i=0; i<5; i++) {
		printf(" REG[%s] = 0x%02X\n", regs[i], device_read_reg(i));
	}
	printf("--------------------------------------------------\n");
}

double K_type_temp_to_mv(double T)
{
    /* NIST多項式 (0℃～1372℃) */
    double c[] = {
        0.000000000000E+00,
        0.394501280250E-01,
        0.236223735980E-04,
        -0.328589067840E-06,
        -0.499048287770E-08,
        -0.675090591730E-10,
        -0.574103274280E-12,
        -0.310888728940E-14,
        -0.104516093650E-16,
        -0.198892668780E-19,
        -0.163226974860E-22
    };
    double mv = 0.0;
    double p = 1.0;
    for (int i = 0; i <= 10; i++) {
        mv += c[i] * p;
        p *= T;
    }
    return mv;
}


static double K_type_mv_to_temp(double mv)
{
    /* NIST逆多項式 (0mV～54.886mV, 0℃～1372℃) */
    double d[] = {
        0.0000000E+00,
        2.508355E+01,
        7.860106E-02,
        -2.503131E-01,
        8.315270E-02,
        -1.228034E-02,
        9.804036E-04,
        -4.413030E-05,
        1.057734E-06,
        -1.052755E-08
    };
    double T = 0.0;
    double p = 1.0;
    for (int i = 0; i <= 9; i++) {
        T += d[i] * p;
        p *= mv;
    }
    return T;
}


static double thermocouple_K(double T_ref, double V_tc)
{
	double E_ref = K_type_temp_to_mv(T_ref);
	double E_total = E_ref + V_tc;
	double T_hot = K_type_mv_to_temp(E_total);

	return T_hot;
}

static double check_temp(void)
{
	double temp = -0xFFFF;	
	int temp_;
	FILE *fp = fopen("/sys/bus/w1/devices/28-0000005a1175/temperature", "r");
	if (fp == NULL) {
		printf(" error w1 open\n");
		return -0xFFFF;
	}
	if (fscanf(fp, "%d", &temp_) == 1) {
		temp = temp_ / 1000.0;
	}

	fclose(fp);

	return temp;
}

int main(void)
{
	// check w1
	double temp = check_temp();
	printf(" current tempperature %.3f\n", temp);


	// check adb1256
	double mv1, mv2, mv3, mv4;
	double temp1, temp2, temp3, temp4;

	if (spi_init() < 0) {
		printf(" spi_init error\n");
		return -1;
	}

	//dump_reg();

	device_reset();
	device_stop_continuous();

	//dump_reg();

	device_write_reg(0x2, 0x7, 0x7); // ADCON GAIN 64

	//dump_reg();
	usleep(1000000);

	device_write_reg(0x1, 0x10, 0xff); // MUX AIN0/AIN1
	mv1 = device_read();
	device_write_reg(0x1, 0x32, 0xff); // MUX AIN2/AIN3
	mv2 = device_read();
	device_write_reg(0x1, 0x54, 0xff); // MUX AIN4/AIN5
	mv3 = device_read();
	device_write_reg(0x1, 0x76, 0xff); // MUX AIN6/AIN7
	mv4 = device_read();

	spi_deinit();

	temp1 = thermocouple_K(temp, mv1);
	printf("Measured Temp = %.2f °C\n", temp1);
	temp2 = thermocouple_K(temp, mv2);
	printf("Measured Temp = %.2f °C\n", temp2);
	temp3 = thermocouple_K(temp, mv3);
	printf("Measured Temp = %.2f °C\n", temp3);
	temp4 = thermocouple_K(temp, mv4);
	printf("Measured Temp = %.2f °C\n", temp4);

	return 0;
}
