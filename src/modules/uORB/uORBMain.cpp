/****************************************************************************
 *
 *   Copyright (c) 2012-2015 FMX10 Development Team. All rights reserved.
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

#include <string.h>
#include "uORBDevices.hpp"
#include "uORBManager.hpp"
#include "uORB.h"
#include "uORBCommon.hpp"
#include <fmx10_log.h>

extern "C" { __EXPORT int uorb_main(int argc, char *argv[]); }

static uORB::DeviceMaster *g_dev = nullptr;
static void usage()
{
	FMX10_INFO("Usage: uorb 'start', 'status', 'top [-a] [<filter1> [<filter2> ...]]'");
	FMX10_INFO("       -a: print all instead of only currently publishing topics");
	FMX10_INFO("       <filter>: topic(s) to match (implies -a)");
}

int
uorb_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return -EINVAL;
	}

	/*
	 * Start/load the driver.
	 *
	 * XXX it would be nice to have a wrapper for this...
	 */
	if (!strcmp(argv[1], "start")) {

		if (g_dev != nullptr) {
			FMX10_WARN("already loaded");
			/* user wanted to start uorb, its already running, no error */
			return 0;
		}

		if (!uORB::Manager::initialize()) {
			FMX10_ERR("uorb manager alloc failed");
			return -ENOMEM;
		}

		/* create the driver */
		g_dev = uORB::Manager::get_instance()->get_device_master(uORB::PUBSUB);

		if (g_dev == nullptr) {
			return -errno;
		}

#if !defined(__FMX10_QURT) && !defined(__FMX10_POSIX_EAGLE)
		/* FIXME: this fails on Snapdragon (see https://github.com/FMX10/Firmware/issues/5406),
		 * so we disable logging messages to the ulog for now. This needs further investigations.
		 */
		fmx10_log_initialize();
#endif

		return OK;
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "status")) {
		if (g_dev != nullptr) {
			g_dev->printStatistics(true);

		} else {
			FMX10_INFO("uorb is not running");
		}

		return OK;
	}

	if (!strcmp(argv[1], "top")) {
		if (g_dev != nullptr) {
			g_dev->showTop(argv + 2, argc - 2);

		} else {
			FMX10_INFO("uorb is not running");
		}

		return OK;
	}

	usage();
	return -EINVAL;
}