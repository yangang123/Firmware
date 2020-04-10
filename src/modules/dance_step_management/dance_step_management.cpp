/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 * @file dance_step_management.cpp
 *
 * Bottle drop module for Outback Challenge 2014, Team Swiss Fang
 *
 * @author Dominik Juchli <juchlid@ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <parameters/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <lib/ecl/geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include "dance_step_management.h"

/**
 * dance_step_management app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int dance_step_management_main(int argc, char *argv[]);



namespace dance_step_management
{

static DanceStepManagement	*g_dance_step_management;

DanceStepManagement::DanceStepManagement() :
	_task_should_exit(false),
	_main_task(-1)
{

}

DanceStepManagement::~DanceStepManagement()
{
	if (_main_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			px4_usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_main_task);
				break;
			}
		} while (_main_task != -1);
	}

	dance_step_management::g_dance_step_management = nullptr;
}

int
DanceStepManagement::start()
{
	/* start the task */
	_main_task = px4_task_spawn_cmd("dance_step_management",
					SCHED_DEFAULT,
					SCHED_PRIORITY_DEFAULT + 15,
					1500,
					(px4_main_t)&DanceStepManagement::task_main_trampoline,
					nullptr);

	if (_main_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}


void
DanceStepManagement::status()
{
}

void
DanceStepManagement::task_main()
{
	while (!_task_should_exit) {

		// 舞步管理实现
		warnx("DanceStepManagement run");
		sleep(1);


	}

	warnx("exiting.");

	_main_task = -1;
	_exit(0);
}

int
DanceStepManagement::task_main_trampoline(int argc, char *argv[])
{
	dance_step_management::g_dance_step_management->task_main();
	return 0;
}

}


static void usage()
{
	errx(1, "usage: dance_step_management {start|stop|status}");
}

int dance_step_management_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (dance_step_management::g_dance_step_management != nullptr) {
			errx(1, "already running");
		}

		dance_step_management::g_dance_step_management = new dance_step_management::DanceStepManagement;

		if (dance_step_management::g_dance_step_management == nullptr) {
			errx(1, "alloc failed");
		}

		if (OK != dance_step_management::g_dance_step_management->start()) {
			delete dance_step_management::g_dance_step_management;
			dance_step_management::g_dance_step_management = nullptr;
			err(1, "start failed");
		}

		return 0;
	}

	if (dance_step_management::g_dance_step_management == nullptr) {
		errx(1, "not running");
	}

	if (!strcmp(argv[1], "stop")) {
		delete dance_step_management::g_dance_step_management;
		dance_step_management::g_dance_step_management = nullptr;

	} else if (!strcmp(argv[1], "status")) {
		dance_step_management::g_dance_step_management->status();

	} else {
		usage();
	}

	return 0;
}
