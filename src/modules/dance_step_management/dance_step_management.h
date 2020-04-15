/****************************************************************************
 *
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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

#pragma once

/**
 * @file rc_update.h
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>

#include <px4_platform_common/posix.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/dance_step_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>


namespace dance_step_management
{

typedef enum POSITION_TARGET_TYPEMASK
{
   POSITION_TARGET_TYPEMASK_X_IGNORE=1, /* Ignore position x | */
   POSITION_TARGET_TYPEMASK_Y_IGNORE=2, /* Ignore position y | */
   POSITION_TARGET_TYPEMASK_Z_IGNORE=4, /* Ignore position z | */
   POSITION_TARGET_TYPEMASK_VX_IGNORE=8, /* Ignore velocity x | */
   POSITION_TARGET_TYPEMASK_VY_IGNORE=16, /* Ignore velocity y | */
   POSITION_TARGET_TYPEMASK_VZ_IGNORE=32, /* Ignore velocity z | */
   POSITION_TARGET_TYPEMASK_AX_IGNORE=64, /* Ignore acceleration x | */
   POSITION_TARGET_TYPEMASK_AY_IGNORE=128, /* Ignore acceleration y | */
   POSITION_TARGET_TYPEMASK_AZ_IGNORE=256, /* Ignore acceleration z | */
   POSITION_TARGET_TYPEMASK_FORCE_SET=512, /* Use force instead of acceleration | */
   POSITION_TARGET_TYPEMASK_YAW_IGNORE=1024, /* Ignore yaw | */
   POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE=2048, /* Ignore yaw rate | */
   POSITION_TARGET_TYPEMASK_ENUM_END=2049, /*  | */
} POSITION_TARGET_TYPEMASK;

typedef struct {
	sq_entry_t *next;
	dance_step_position_s data;
} work_queue_item_t;

class DanceStepManagement:public ModuleBase<DanceStepManagement>
{
public:
	/**
	 * Constructor
	 */
	DanceStepManagement();

	/**
	 * Destructor, also kills task.
	 */
	~DanceStepManagement() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static DanceStepManagement *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	//从空闲队列中取1个item
	work_queue_item_t *alloc_item(void);

        //将item放到空闲队尾
	void free_item(work_queue_item_t *item);

	//将item放到工作队列
	void add_item_to_work_queue(work_queue_item_t *item);

        //将item从工作队列释放
	work_queue_item_t * get_item_from_work_queue(void);

	//打印队列状态
	void print_queue(void);

	void handle_message_set_position_target_global_int(dance_step_position_s &set_position_target_global_int);

	uORB::Publication<offboard_control_mode_s>		_offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<position_setpoint_triplet_s>		_pos_sp_triplet_pub{ORB_ID(position_setpoint_triplet)};
	uORB::Publication<vehicle_local_position_s>		_local_pos_pub{ORB_ID(vehicle_local_position)};

	uORB::Subscription					_vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription				  	_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription					_dance_step_position_sub{ORB_ID(dance_step_position)};
};

} /* namespace DanceStepManagement */
