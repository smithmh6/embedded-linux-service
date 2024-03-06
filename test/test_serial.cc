/* copyright 2023 thorlabs */

#include <cstddef>
#include <stdio.h>
#include <cerrno>
#include <modbus.h>
#include <cstring>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <fcntl.h>
#include <math.h>
#include <sys/ioctl.h>

// linux headers
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

int main() {
	// open port
	int serial_port = open("/dev/ttyACM0", O_RDWR);

	// new termios struct
	struct termios tty;

	// read existing setting, handle errors
	if (tcgetattr(serial_port, &tty) != 0) {
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
		return 1;
	}

	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS;
	tty.c_cflag |= CREAD | CLOCAL;

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO;
	tty.c_lflag &= ~ECHOE;
	tty.c_lflag &= ~ECHONL;
	tty.c_lflag &= ~ISIG;

	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

	tty.c_oflag &= ~OPOST;
	tty.c_oflag &= ~ONLCR;

	tty.c_cc[VTIME] = 10;
	tty.c_cc[VMIN] = 0;

	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
		printf("error %i from tcsetattr: %s\n", errno, strerror(errno));
		return 1;
	}

	uint8_t data_frame[32768];
	uint8_t read_buf[128];
	int gain = 1;
	double adc_scale = ((2.4 / gain) / pow(2.0, 24.0));
	double norm = 1 / 2.4;
	double data_arr[1024];
	uint8_t pos_arr[1024];



	while (true) {
		int r = 0;
		int total_bytes = 0;

		while (total_bytes < 32768) {

			r = read(serial_port, &read_buf, sizeof(read_buf));

			//printf("Read %i bytes\n", r);
			//for (int i = 0; i < r; i++) {
			//	printf("0x%02X ", read_buf[i]);
			//}
			//printf("\n");

			if (r > 0) {
				memcpy(&data_frame[total_bytes], read_buf, r);
				total_bytes += r;
			} else if (r == -1) {
				printf("Error %i reading serial: %s\n", errno, strerror(errno));
				return 1;
			}

		}

		int n = 0;
		//printf("-----\n");
		//for (int i = 0; i < 4096 - 1; i++) {
		//	for (int j = 0; j < 8; j++) {
		//		printf("0x%02X ", data_frame[n]);
		//		n++;
		//	}
		//	printf("\n");
		//}

		n = 0;
		int idx = 0;
		int n_peaks = 0;
		double area = 0;
		bool calculate = false;

		double test_vals[24];

		int start, stop;

		while (n < 4096) {
			int32_t ch0;
			int32_t upperByte = (int32_t) data_frame[n] << 24;
			n++;
			int32_t middleByte = (int32_t) data_frame[n] << 16;
			n++;
			int32_t lowerByte = (int32_t) data_frame[n] << 8;
			n++;
			ch0 = (upperByte | middleByte | lowerByte) >> 8;
			double ch0_f = adc_scale * ch0;
			//printf("%f\n", ch0_f);

			// read position
			uint8_t pos = data_frame[n];
			n++;



			// add to data array
			data_arr[idx] = (-ch0_f + 1.2) * norm;
			if (idx > 2) {
				double prev_slope = data_arr[idx - 1] - data_arr[idx - 2];
				double slope = data_arr[idx] - data_arr[idx - 1];

				if (slope < 0 && slope > -0.01 && !calculate && prev_slope > 0) {
					//printf("starting sum at [%i] %f  %f  %f\n", idx, data_arr[idx], data_arr[idx - 1], slope);
					calculate = true;
					area = 0;
					start = idx;
					pos_arr[n_peaks] = pos;
				}

				if (slope < -0.01 && calculate) {
					//printf("ending sum at [%i] %f  %f  %f\n", idx, data_arr[idx], data_arr[idx - 1], slope);

					stop = idx;
					test_vals[n_peaks] = area / (stop - start);
					calculate = false;
					printf("%f,%i\n", test_vals[n_peaks], pos_arr[n_peaks]);
					n_peaks++;

				}

				if (calculate) {
					area += data_arr[idx];
				}
			}


			idx++;

			//printf("%f 0x%02X\n", ch0_f, pos);
			//printf("%f\n", ch0_f);
		}

		//for (int i = 0; i < 1024; i++) {
		//	printf("%f,%i\n", data_arr[i], pos_arr[i]);
		//}

	}

	close(serial_port);

	return 0;
}
