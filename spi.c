#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>


#define SPI_SPEED  10000 // 10khz
#define CMD_RESET   0xFE  // reset
#define CMD_SDATAC  0x0F  // Stop Read Data Continuous
#define CMD_SELFCAL 0xF0  // Self Calibration
#define CMD_SYNC        0xFC  // Sync
#define CMD_WAKEUP  0xFF  // Wakeup
#define CMD_RREG        0x10  // | (reg_addr)
#define CMD_WREG        0x50  // | (reg_addr)
#define CMD_RDATA   0x01  // read data
#define CMD_RDATAC  0x03  // read data continue
#define VREF    5  // 5V
#define GAIN    4
#define GAIN_REG        2

static void init_config(int);
static void device_reset();
static int sensor_n = 1;
static int fd1 = -1;
static int fd2 = -1;

static int spi_init(void) {
        fd1 = open("/dev/spidev0.0", O_RDWR);
        if (fd1 < 0) {
                printf(" =====> device open error\n");
                return -1;
        }

        fd2 = open("/dev/spidev0.1", O_RDWR);
        if (fd2 < 0) {
                printf(" =====> device open error\n");
                return -1;
        }

        uint8_t mode = SPI_MODE_1;
        uint8_t bits = 8;
        uint32_t speed = SPI_SPEED;

        ioctl(fd1, SPI_IOC_WR_MODE, &mode);
        ioctl(fd1, SPI_IOC_WR_BITS_PER_WORD, &bits);
        ioctl(fd1, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

        ioctl(fd2, SPI_IOC_WR_MODE, &mode);
        ioctl(fd2, SPI_IOC_WR_BITS_PER_WORD, &bits);
        ioctl(fd2, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

        return 0;
}

static void spi_deinit(void) {
        close(fd1);
        close(fd2);
}

static void spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len) {
        struct spi_ioc_transfer tr = {
                .tx_buf = (unsigned long)tx,
                .rx_buf = (unsigned long)rx,
                .len = len + 2,
                .speed_hz = SPI_SPEED,
                .bits_per_word = 8,
        };

        if (sensor_n == 0) {
                if (ioctl(fd1, SPI_IOC_MESSAGE(1), &tr) < 1)
                        printf(" error SPI transfer\n");
        } else if (sensor_n == 1) {
                if (ioctl(fd2, SPI_IOC_MESSAGE(1), &tr) < 1)
                        printf(" error SPI transfer\n");
        }
}

static void device_reset(void) {
        uint8_t cmd = CMD_RESET;
        spi_xfer(&cmd, NULL, 1);
        usleep(5000);
}

static void device_read_continuous(void) {
        uint8_t cmd = CMD_RDATAC;
        spi_xfer(&cmd, NULL, 1);
        usleep(1000);
}
static void device_stop_continuous(void) {
        uint8_t cmd = CMD_SDATAC;
        spi_xfer(&cmd, NULL, 1);
        usleep(1000);
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

static void device_selfcal(void) {
                uint8_t cmd = CMD_SELFCAL; //0xF0
                spi_xfer(&cmd, NULL, 1);
                usleep(1000);
}

static double device_code_to_mV(int32_t code, double vref, int gain) {
        const double full_scale = 8388607.0;  // 2^23 - 1
        return ((double)code / full_scale) * (vref * 1000.0 / gain);
}

static double device_read(void) {
        uint8_t tx[4] = {CMD_RDATA, 0xFF, 0xFF, 0xFF};
        uint8_t rx[4]  = {0};
        spi_xfer(tx, rx, sizeof(rx));
        usleep(1000);

        int32_t code = (rx[1] << 16) | (rx[2] << 8) | rx[3];
        if (code & 0x800000) code |= 0xFF000000;

#if 0
        printf(" Voltage = %.6f mV   Data %x%x%x\n",
                device_code_to_mV(code, VREF, GAIN), rx[1], rx[2], rx[3]);
#endif
        return device_code_to_mV(code, VREF, GAIN);
}

static uint8_t device_read_reg(uint8_t addr) {
        uint8_t tx[3] = {CMD_RREG | addr, 0x00, 0xFF};
        uint8_t rx[3] = {0};
        spi_xfer(tx, rx, sizeof(tx));

        return rx[2];
}

static void device_write_reg(uint8_t addr, uint8_t val, uint8_t mask) {
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

int comp_d(const void *a, const void* b)
{
        double t1 = *(double*)a;
        double t2 = *(double*)b;

        if (t1 < t2) return -1;
        else if (t1 > t2) return 1;
        else return 0;
}


double get_ave_mv(int pin)
{
#if 1
        double samples[10];
        double sum = 0;

        for (int i=0; i<10; i++) {
                init_config(pin);
                usleep(10000);
                samples[i] = device_read();
        }

        qsort(samples, 10, sizeof(double), comp_d);

        for (int i=2; i<8; i++) {
                sum += samples[i];
        }

        return sum / 6.0;
#else
        device_read(); // delete
        usleep(50);
        return device_read();
#endif
}

void init_config(int pin)
{
        device_reset();
        usleep(1000);

        device_write_reg(0x2, GAIN_REG, 0xff); // ADCON GAIN 64
        device_write_reg(0x3, 0x82, 0xff); // Rate setting 100SPS
        //device_stop_continuous();

        switch(pin) {
        case 0:
                device_write_reg(0x1, 0x01, 0xff); // MUX AIN0/AIN1
                break;
        case 1:
                device_write_reg(0x1, 0x23, 0xff); // MUX AIN2/AIN3
                break;
        case 2:
                device_write_reg(0x1, 0x45, 0xff); // MUX AIN4/AIN5
                break;
        case 3:
                device_write_reg(0x1, 0x67, 0xff); // MUX AIN6/AIN7
                break;
        default:
                break;
        }
        device_selfcal();
        device_sync();
        device_wakeup();
        //device_read_continuous();
}

int main(int argc, char* argv[])
{
        if (access(argv[1], F_OK) == 0) {
                printf(" already file exist %s\n", argv[1]);
                return 1;
        }

        // clock
        struct timespec start, now;
        long elapsed_sec;
        clock_gettime(CLOCK_MONOTONIC, &start);


        // check w1
        double temp = check_temp();
        //printf(" current tempperature %.3f\n", temp);


        // check adb1256
        double mv1, mv2, mv3, mv4;
        double temp1[2], temp2[2], temp3[2], temp4[2];

        if (spi_init() < 0) {
                printf(" spi_init error\n");
                return 1;
        }

        FILE *fp = fopen(argv[1], "w");
        if (!fp) {
                printf("fopen\n");
                return 1;
        }

        unsigned int loop_count = 0;
        unsigned int sleep_time = 0;
        while (1) {
                //printf(" SENSOR1\n");
                sensor_n = 0;

                mv1 = get_ave_mv(0);
                usleep(50000);

                mv2 = get_ave_mv(1);
                usleep(50000);

                mv3 = get_ave_mv(2);
                usleep(50000);

                mv4 = get_ave_mv(3);
                usleep(50000);

                temp1[0] = thermocouple_K(temp, mv1);
                //printf("Measured Temp = %.2f °C\n", temp1[0]);
                temp2[0] = thermocouple_K(temp, mv2);
                //printf("Measured Temp = %.2f °C\n", temp2[0]);
                temp3[0] = thermocouple_K(temp, mv3);
                //printf("Measured Temp = %.2f °C\n", temp3[0]);
                temp4[0] = thermocouple_K(temp, mv4);
                //printf("Measured Temp = %.2f °C\n", temp4[0]);


                //printf(" SENSOR2\n");
                sensor_n = 1;

                mv1 = get_ave_mv(0);
                usleep(50000);

                mv2 = get_ave_mv(1);
                usleep(50000);

                mv3 = get_ave_mv(2);
                usleep(50000);

                mv4 = get_ave_mv(3);
                usleep(50000);

                temp1[1] = thermocouple_K(temp, mv1);
                //printf("Measured Temp = %.2f °C\n", temp1[1]);
                temp2[1] = thermocouple_K(temp, mv2);
                //printf("Measured Temp = %.2f °C\n", temp2[1]);
                temp3[1] = thermocouple_K(temp, mv3);
                //printf("Measured Temp = %.2f °C\n", temp3[1]);
                temp4[1] = thermocouple_K(temp, mv4);
                //printf("Measured Temp = %.2f °C\n", temp4[1]);


                printf("%ld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                        loop_count * 2, temp1[0], temp2[0], temp3[0], temp4[0],
                         temp1[1], temp2[1], temp3[1], temp4[1]);
                fprintf(fp, "%ld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                        loop_count * 2, temp1[0], temp2[0], temp3[0], temp4[0],
                         temp1[1], temp2[1], temp3[1], temp4[1]);

                // sleep time
                loop_count++;
                clock_gettime(CLOCK_MONOTONIC, &now);
                elapsed_sec = (now.tv_sec - start.tv_sec);
                sleep_time = 120 * loop_count - elapsed_sec;
                //printf(" sleep time:%ld\n", sleep_time);
                fflush(fp);
                fsync(fileno(fp));

                //sleep(sleep_time);
        }

        fclose(fp);
        spi_deinit();

        return 0;
}
