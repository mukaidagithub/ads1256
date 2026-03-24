#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <dirent.h>


#include "temp.h"
#include "spi.h"

static double get_base_temp(void);
static double get_ave_of_5_mv(void);
static double thermocouple_K(double, double);
static double code_to_mv(int32_t, int);


double get_temp(void)
{
        double base = get_base_temp();
        double mv   = get_ave_of_5_mv();

        return thermocouple_K(base, mv);
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

static FILE* open_temp_sensor(void)
{
	DIR *dir;
	struct dirent *entry;
	char path[256];
	FILE *fp = NULL;

	dir = opendir("/sys/bus/w1/devices/");
	if (!dir) {
		perror("opendir");
		return NULL;
	}

	while ((entry = readdir(dir)) != NULL) {
		if (strncmp(entry->d_name, "28-", 3) == 0) {
			snprintf(path, sizeof(path),
				"/sys/bus/w1/devices/%s/temperature",
				entry->d_name);

			fp = fopen(path, "r");
			if (!fp) {
				perror("fopen");
				closedir(dir);
				return NULL;
			}

			break;
		}
	}

	closedir(dir);

	if (!fp) {
		printf(" can't find temp sensor\n");
	}

	return fp;
}

static double get_base_temp(void)
{
	double temp = 20.0;
	int temp_;

	char path[256];
	FILE *fp = open_temp_sensor();
	if (!fp) {
		printf("return default temp 20.0\n");
		return 20.0; // default
	}

	if (fscanf(fp, "%d", &temp_) == 1) {
		temp = temp_ / 1000.0;
	} else {
		printf("return default temp 20.0\n");
		fclose(fp);
		return 20.0;
	}
	
	fclose(fp);

	//printf(" base temp:%.2f\n", temp);
	
        return temp;
}


static double get_ave_of_5_mv(void)
{
        uint32_t gain = get_gain();
        double sum = 0;
	const int count = 5;

        for (int i=0; i<count; i++) {
                int32_t val = device_rdata();
		if (val & 0x800000)
			val |= 0xff000000;
		double mv = code_to_mv(val, gain);
                sum += mv;
		//printf(" mv:%.6f val:%x sum:%.6f gain:%d\n", mv, val, sum, gain);
        }

        return sum / count;
}

static double code_to_mv(int32_t code, int gain)
{
        const double full_scale = 8388607.0;
        const double vref       = 2.5; // 2.5V

        return ((double)code / full_scale) * (vref * 1000.0 / gain);
}

