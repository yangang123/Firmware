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
#include <ecl/geo/geo.h>

#include "dance_step_management.h"

extern "C" __EXPORT int dance_step_management_main(int argc, char *argv[]);

using  namespace dance_step_management;

namespace dance_step_management
{


#define  WORK_ITEM_SIZE_MAX  8

static 	work_queue_item_t work_queue_item_pool[WORK_ITEM_SIZE_MAX];
static	uint16_t 	g_free_count;
static	uint16_t 	g_work_count;
static 	sq_queue_t 	g_free_q;
static 	sq_queue_t 	g_work_q;

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

work_queue_item_t * DanceStepManagement::alloc_item(void)
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


void DanceStepManagement::free_item(work_queue_item_t *item)
{
	if (item) {
		sq_addlast((sq_entry_t *)item, &g_free_q);
		g_free_count++;
	}
}

void DanceStepManagement::add_item_to_work_queue(work_queue_item_t *item)
{
	sq_addlast((sq_entry_t *)item, &g_work_q);
	g_work_count++;
}


work_queue_item_t * DanceStepManagement::get_item_from_work_queue(void)
{
	work_queue_item_t *item_p;

        item_p = ( work_queue_item_t *)sq_remfirst(&g_work_q);

	if (item_p != NULL)
	{
		g_work_count--;
	}

	return item_p;
}

void DanceStepManagement::print_queue(void)
{
     warnx("g_free_count:%d", g_free_count);
     warnx("g_work_count:%d", g_work_count);
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
	print_queue();
	return 0;
}

int DanceStepManagement::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

void
DanceStepManagement::handle_message_set_position_target_global_int(dance_step_position_s &set_position_target_global_int)
{
	const bool values_finite =
		PX4_ISFINITE(set_position_target_global_int.alt) &&
		PX4_ISFINITE(set_position_target_global_int.vx) &&
		PX4_ISFINITE(set_position_target_global_int.vy) &&
		PX4_ISFINITE(set_position_target_global_int.vz) &&
		PX4_ISFINITE(set_position_target_global_int.afx) &&
		PX4_ISFINITE(set_position_target_global_int.afy) &&
		PX4_ISFINITE(set_position_target_global_int.afz) &&
		PX4_ISFINITE(set_position_target_global_int.yaw);

	/* Only accept messages which are intended for this system */
	if ((set_position_target_global_int.target_system == 0) &&
	    ( set_position_target_global_int.target_component == 0) &&
	    values_finite) {

		offboard_control_mode_s offboard_control_mode{};

		/* convert mavlink type (local, NED) to uORB offboard control struct */
		offboard_control_mode.ignore_position = (bool)(set_position_target_global_int.type_mask &
							(POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_X_IGNORE
									| POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_Y_IGNORE
									| POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_Z_IGNORE));
		offboard_control_mode.ignore_alt_hold = (bool)(set_position_target_global_int.type_mask & 0x4);
		offboard_control_mode.ignore_velocity = (bool)(set_position_target_global_int.type_mask &
							(POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_VX_IGNORE
									| POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_VY_IGNORE
									| POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_VZ_IGNORE));
		offboard_control_mode.ignore_acceleration_force = (bool)(set_position_target_global_int.type_mask &
				(POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_AX_IGNORE
				 | POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_AY_IGNORE
				 | POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_AZ_IGNORE));
		/* yaw ignore flag mapps to ignore_attitude */
		offboard_control_mode.ignore_attitude = (bool)(set_position_target_global_int.type_mask &
							POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_YAW_IGNORE);
		offboard_control_mode.ignore_bodyrate_x = (bool)(set_position_target_global_int.type_mask &
				POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE);
		offboard_control_mode.ignore_bodyrate_y = (bool)(set_position_target_global_int.type_mask &
				POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE);
		offboard_control_mode.ignore_bodyrate_z = (bool)(set_position_target_global_int.type_mask &
				POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE);

		bool is_force_sp = (bool)(set_position_target_global_int.type_mask & (1 << 9));

		offboard_control_mode.timestamp = hrt_absolute_time();
		_offboard_control_mode_pub.publish(offboard_control_mode);
		warnx("send offboard_control_mode_pub");

		/* If we are in offboard control mode and offboard control loop through is enabled
		 * also publish the setpoint topic which is read by the controller */

		vehicle_control_mode_s control_mode{};
		_control_mode_sub.copy(&control_mode);
		warnx("offboard mode value:%d", control_mode.flag_control_offboard_enabled); 
		if (control_mode.flag_control_offboard_enabled) {
			
			if (is_force_sp && offboard_control_mode.ignore_position &&
				offboard_control_mode.ignore_velocity) {

				PX4_WARN("force setpoint not supported");

			} else {
				/* It's not a pure force setpoint: publish to setpoint triplet  topic */
				position_setpoint_triplet_s pos_sp_triplet{};

				pos_sp_triplet.timestamp = hrt_absolute_time();
				pos_sp_triplet.previous.valid = false;
				pos_sp_triplet.next.valid = false;
				pos_sp_triplet.current.valid = true;

				/* Order of statements matters. Takeoff can override loiter.
					* See https://github.com/mavlink/mavlink/pull/670 for a broader conversation. */
				if (set_position_target_global_int.type_mask & 0x3000) { //Loiter setpoint
					pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;

				} else if (set_position_target_global_int.type_mask & 0x1000) { //Takeoff setpoint
					pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

				} else if (set_position_target_global_int.type_mask & 0x2000) { //Land setpoint
					pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LAND;

				} else if (set_position_target_global_int.type_mask & 0x4000) { //Idle setpoint
					pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;

				} else {
					pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
				}

				/* set the local pos values */
				vehicle_local_position_s local_pos{};

				if (!offboard_control_mode.ignore_position && _vehicle_local_position_sub.copy(&local_pos)) {
					if (!globallocalconverter_initialized()) {
						globallocalconverter_init(local_pos.ref_lat, local_pos.ref_lon,
										local_pos.ref_alt, local_pos.ref_timestamp);
						pos_sp_triplet.current.position_valid = false;

					} else {
						globallocalconverter_tolocal(set_position_target_global_int.lat_int / 1e7,
										set_position_target_global_int.lon_int / 1e7, set_position_target_global_int.alt,
										&pos_sp_triplet.current.x, &pos_sp_triplet.current.y, &pos_sp_triplet.current.z);
						pos_sp_triplet.current.position_valid = true;
					}

				} else {
					pos_sp_triplet.current.position_valid = false;
				}

				/* set the local velocity values */
				if (!offboard_control_mode.ignore_velocity) {

					pos_sp_triplet.current.velocity_valid = true;
					pos_sp_triplet.current.vx = set_position_target_global_int.vx;
					pos_sp_triplet.current.vy = set_position_target_global_int.vy;
					pos_sp_triplet.current.vz = set_position_target_global_int.vz;

					pos_sp_triplet.current.velocity_frame = set_position_target_global_int.coordinate_frame;

				} else {
					pos_sp_triplet.current.velocity_valid = false;
				}

				if (!offboard_control_mode.ignore_alt_hold) {
					pos_sp_triplet.current.alt_valid = true;

				} else {
					pos_sp_triplet.current.alt_valid = false;
				}

				/* set the local acceleration values if the setpoint type is 'local pos' and none
					* of the accelerations fields is set to 'ignore' */
				if (!offboard_control_mode.ignore_acceleration_force) {

					pos_sp_triplet.current.acceleration_valid = true;
					pos_sp_triplet.current.a_x = set_position_target_global_int.afx;
					pos_sp_triplet.current.a_y = set_position_target_global_int.afy;
					pos_sp_triplet.current.a_z = set_position_target_global_int.afz;
					pos_sp_triplet.current.acceleration_is_force = is_force_sp;

				} else {
					pos_sp_triplet.current.acceleration_valid = false;
				}

				/* set the yaw setpoint */
				if (!offboard_control_mode.ignore_attitude) {
					pos_sp_triplet.current.yaw_valid = true;
					pos_sp_triplet.current.yaw = set_position_target_global_int.yaw;

				} else {
					pos_sp_triplet.current.yaw_valid = false;
				}

				/* set the yawrate sp value */
				if (!(offboard_control_mode.ignore_bodyrate_x ||
					offboard_control_mode.ignore_bodyrate_y ||
					offboard_control_mode.ignore_bodyrate_z)) {

					pos_sp_triplet.current.yawspeed_valid = true;
					pos_sp_triplet.current.yawspeed = set_position_target_global_int.yaw_rate;

				} else {
					pos_sp_triplet.current.yawspeed_valid = false;
				}

				_pos_sp_triplet_pub.publish(pos_sp_triplet);

				warnx("send _pos_sp_triplet_pub");
			}
		}

	}
}


void DanceStepManagement::run()
{

	hrt_abstime current = hrt_absolute_time();

	while (!should_exit()) {

		//接收mavlink发送过来的orb
		/*
		if (_dance_step_position_sub.updated())
		{
			work_queue_item_t * item_recv =  alloc_item();
			if (item_recv) {


				
				_dance_step_position_sub.copy(&item_recv->data);
				add_item_to_work_queue(item_recv);
			}
		}
		*/

		//定时器发送舞步到offboard
		if (hrt_absolute_time() - current > 10000) {
			current = hrt_absolute_time();
			work_queue_item_t item;
			work_queue_item_t *item_send = &item;
			//  虚拟数据航点信息赋值
			item_send->data.lat_int =  42.391138   * 1e7;
			item_send->data.lon_int =  123.3718778 * 1e7;
			item_send->data.vx = 1;
			item_send->data.vy = 1;
			item_send->data.vz = 1;
			item_send->data.afx = 1;
			item_send->data.afy = 1;
			item_send->data.afz = 1;
			item_send->data.yaw = 0;
			item_send->data.yaw_rate = 0;
			item_send->data.type_mask = POSITION_TARGET_TYPEMASK_X_IGNORE;
			item_send->data.target_system =0;
			item_send->data.target_component =0;
			handle_message_set_position_target_global_int(item_send->data);

			warnx("send one point to offboard");
			
		}

		//线程延时5ms
		px4_usleep(5000);

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
