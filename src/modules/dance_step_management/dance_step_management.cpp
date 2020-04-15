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
 * @author nhy <nhy_navigator@163.com>

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


#include "dance_step_management.h"

extern "C" __EXPORT int dance_step_management_main(int argc, char *argv[]);

using  namespace dance_step_management;

namespace dance_step_management
{


#define  WORK_ITEM_SIZE_MAX  8
work_queue_item_t work_queue_item_pool[WORK_ITEM_SIZE_MAX];
uint16_t g_free_count;
static sq_queue_t g_free_q;
static sq_queue_t g_work_q;

DanceStepManagement::DanceStepManagement()
{
	sq_init(&g_free_q);
	sq_init(&g_work_q);

	work_queue_item_t *item = work_queue_item_pool;
	for (size_t i = 0; i < WORK_ITEM_SIZE_MAX; i++) {
		sq_addlast((FAR sq_entry_t *)item++, &g_free_q);
	}
	g_free_count = WORK_ITEM_SIZE_MAX;
}

DanceStepManagement::~DanceStepManagement()
{

}

work_queue_item_t * DanceStepManagement::create_item(void)
{
	work_queue_item_t *item_p;

       item_p = ( work_queue_item_t *)sq_remfirst(&g_free_q);

       if (item_p != NULL)
        {
          g_free_count--;
          item_p->next  = NULL;
        }

	return item_p;
}

void DanceStepManagement::remove_item(work_queue_item_t *item)
{
      if (item) {
	sq_addlast((sq_entry_t *)item, &g_free_q);
	g_free_count++;
      }
}

void DanceStepManagement::print_queue(void)
{
     warnx("g_free_count:%d", g_free_count);
}

int DanceStepManagement::task_spawn(int argc, char *argv[])
{
	/* start the task */
	_task_id = px4_task_spawn_cmd("dance_step_management",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT + 15,
				      2000,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int DanceStepManagement::print_status()
{
	return 0;
}

int DanceStepManagement::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


void DanceStepManagement::run()
{
	work_queue_item_t *p;
	p = create_item();
	p = create_item();
	p = create_item();
	p = create_item();
	print_queue();

	remove_item(p);
	print_queue();

	while (!should_exit()) {

		// 舞步管理实现
		warnx("DanceStepManagement run");
		sleep(1);
	}

	warnx("exiting.");
}

int DanceStepManagement::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

The provided functionality includes:

### Implementation

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dance_step_management", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

DanceStepManagement *DanceStepManagement::instantiate(int argc, char *argv[])
{
	return new DanceStepManagement();
}

}

extern "C" __EXPORT int dance_step_management_main(int argc, char *argv[])
{
	return DanceStepManagement::main(argc, argv);
}
