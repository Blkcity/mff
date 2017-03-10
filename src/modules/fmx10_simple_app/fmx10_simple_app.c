/****************************************************************************
 *
 *   Copyright (c) 2012-2016 FMX10 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name FMX10 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file fmx10_simple_app.c
 * Minimal application example for FMX10 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <fmx10_config.h>
#include <fmx10_tasks.h>
#include <fmx10_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_mag.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int fmx10_simple_app_main(int argc, char *argv[]);

int fmx10_simple_app_main(int argc, char *argv[])
{
	FMX10_INFO("Hello Sky!");

	/* subscribe to sensor_combined topic */
//	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	int sensor_accel_fd0 = orb_subscribe_multi(ORB_ID(sensor_accel),0);
	int sensor_accel_fd1 = orb_subscribe_multi(ORB_ID(sensor_accel),1);
	int sensor_gyro_fd0 = orb_subscribe_multi(ORB_ID(sensor_gyro),0);
	int sensor_gyro_fd1 = orb_subscribe_multi(ORB_ID(sensor_gyro),1);
	int sensor_baro_fd0 = orb_subscribe_multi(ORB_ID(sensor_baro),0);
	int sensor_baro_fd1 = orb_subscribe_multi(ORB_ID(sensor_baro),1);
	int sensor_mag_fd0 = orb_subscribe_multi(ORB_ID(sensor_mag),0);
	int sensor_mag_fd1 = orb_subscribe_multi(ORB_ID(sensor_mag),1);
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_accel_fd0, 50);
	orb_set_interval(sensor_accel_fd1, 50);
	orb_set_interval(sensor_gyro_fd0, 50);
	orb_set_interval(sensor_gyro_fd1, 50);
	orb_set_interval(sensor_baro_fd0, 50);
	orb_set_interval(sensor_baro_fd1, 50);
	orb_set_interval(sensor_mag_fd0, 50);
	orb_set_interval(sensor_mag_fd1, 50);

	/* advertise attitude topic */
//	struct vehicle_attitude_s att;
//	memset(&att, 0, sizeof(att));
//	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	/* one could wait for multiple topics with this technique, just using one here */
	fmx10_pollfd_struct_t fds[] = {
		{ .fd = sensor_accel_fd0,   .events = POLLIN },
		{ .fd = sensor_accel_fd1,   .events = POLLIN },
		{ .fd = sensor_gyro_fd0,   .events = POLLIN },
		{ .fd = sensor_gyro_fd1,   .events = POLLIN },
		{ .fd = sensor_baro_fd0,   .events = POLLIN },
		{ .fd = sensor_baro_fd1,   .events = POLLIN },
		{ .fd = sensor_mag_fd0,   .events = POLLIN },
		{ .fd = sensor_mag_fd1,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 50000; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = fmx10_poll(fds, sizeof(fds)/sizeof(fds[0]), 2000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			FMX10_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				FMX10_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {
			if (fds[0].revents & POLLIN) {
				struct accel_report raw_a;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_accel), sensor_accel_fd0, &raw_a);
				FMX10_INFO("Accel0:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw_a.x,
					 (double)raw_a.y,
					 (double)raw_a.z);
			}
			if (fds[1].revents & POLLIN) {
				struct accel_report raw_a1;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_accel), sensor_accel_fd1, &raw_a1);
				FMX10_INFO("Accel1:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw_a1.x,
					 (double)raw_a1.y,
					 (double)raw_a1.z);
			}
			if (fds[2].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				//struct sensor_combined_s raw;
				struct gyro_report raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_gyro), sensor_gyro_fd0, &raw);
				FMX10_INFO("Gyro0:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw.x,
					 (double)raw.y,
					 (double)raw.z);

				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/
			}
			if (fds[3].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				//struct sensor_combined_s raw;
				struct gyro_report raw1;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_gyro), sensor_gyro_fd1, &raw1);
				FMX10_INFO("Gyro1:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw1.x,
					 (double)raw1.y,
					 (double)raw1.z);

				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/
			}
			if (fds[4].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				//struct sensor_combined_s raw;
				struct baro_report ba_raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_baro), sensor_baro_fd0, &ba_raw);
				FMX10_INFO("Baro0:\t%8.4f\t%8.4f\t%8.4f",
					 (double)ba_raw.temperature,
					 (double)ba_raw.pressure,
					 (double)ba_raw.altitude);

				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/
			}
			if (fds[5].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				//struct sensor_combined_s raw;
				struct baro_report ba_raw1;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_baro), sensor_baro_fd1, &ba_raw1);
				FMX10_INFO("Baro1:\t%8.4f\t%8.4f\t%8.4f",
					 (double)ba_raw1.temperature,
					 (double)ba_raw1.pressure,
					 (double)ba_raw1.altitude);

				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/
			}
			if (fds[6].revents & POLLIN) {
				struct mag_report raw_m;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_mag), sensor_mag_fd0, &raw_m);
				FMX10_INFO("Mag0:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw_m.x,
					 (double)raw_m.y,
					 (double)raw_m.z);
			}
			if (fds[7].revents & POLLIN) {
				struct mag_report raw_m1;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_mag), sensor_mag_fd1, &raw_m1);
				FMX10_INFO("Mag1:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw_m1.x,
					 (double)raw_m1.y,
					 (double)raw_m1.z);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	FMX10_INFO("exiting");

	return 0;
}
