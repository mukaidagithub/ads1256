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
#include "temp.h"
#include "file.h"

int main(int argc, char* argv[])
{
	int ret;
	FILE *csv_fd;
	int sensor_n = 0;
	double temp[8] = {0};

	// csv file
	ret = create_csv();
	if (ret) {
		printf(" csv file open error\n");
		return 1;
	}

	// clock
	int sleep_time = 0;
	struct timespec start, now;
	long elapsed_sec;

	// spi setting
	spi_open();
        spi_init();

	clock_gettime(CLOCK_MONOTONIC, &start);

	device_preparation();

        int loop = 0;
	int wait_sec = 120; // 120 sec
	int wait_min = 2;   // 2 min
	int get_temp_error_count = 0;
	int abnormal_temp_error_count = 0;
	double prev_temp[8] = { 0 };
        while (1) {

		for (int ch = 0; ch < 8; ch++) {
			if (ch == 0) {
				sensor_n = 0;
				switch_sensor(sensor_n);
			} else if (ch == 4) {
				sensor_n = 1;
				switch_sensor(sensor_n);
			}

			switch_ch(ch - sensor_n * 4);

retry_get_temp:
			temp[ch] = get_temp();
			// when over 50 C or under 50 C, update temp to previous temp.
			if ((temp[ch] > prev_temp[ch] + 50) || (temp[ch] < prev_temp[ch] - 50)) {
				get_temp_error_count++;

				if (get_temp_error_count > 6)
					break;

				goto retry_get_temp;
			}
			prev_temp[ch] = temp[ch];


			if ((temp[ch] > 100) || (temp[ch] < -10)) {
				abnormal_temp_error_count++;
			}

			// dump_reg();

			usleep(50000); // 50ms wait
		}

		csv_fd = open_csv();
		if (abnormal_temp_error_count) {
			printf("%ld, data reset\n", loop * wait_min);
			fprintf(csv_fd, "%ld, data reset\n", loop * wait_min);
		} else if (get_temp_error_count) {
			printf("%ld, data error\n", loop * wait_min);
			fprintf(csv_fd, "%ld, data error\n", loop * wait_min);
		} else {
			printf("%ld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
				loop * wait_min, temp[0], temp[1], temp[2], temp[3], temp[4],
				temp[5], temp[6], temp[7]);
			fprintf(csv_fd, "%ld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                                loop * wait_min, temp[0], temp[1], temp[2], temp[3], temp[4],
                                temp[5], temp[6], temp[7]);
		}
		close_csv();
		get_temp_error_count = 0;

		loop++;
		if (loop > 240) // 8h
			break;

		clock_gettime(CLOCK_MONOTONIC, &now);
		elapsed_sec = (now.tv_sec - start.tv_sec);
		if (elapsed_sec > wait_sec * loop)
			sleep_time = wait_sec;
		else
			sleep_time = wait_sec * loop - elapsed_sec;
			

		sleep(sleep_time);

		if (abnormal_temp_error_count) {
			clock_gettime(CLOCK_MONOTONIC, &start);
			device_preparation();
			abnormal_temp_error_count = 0;
		}
        }

	spi_close();

	printf(" FIN 480 min.\n");

	sleep(36000);

        return 0;
}

