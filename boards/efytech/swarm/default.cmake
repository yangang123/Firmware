
px4_add_board(
	PLATFORM nuttx
	VENDOR efytech
	MODEL swarm
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT px4fmu_common
	BUILD_BOOTLOADER
	TESTING
#	UAVCAN_INTERFACES 2  - No H7 or FD can support in UAVCAN
	SERIAL_PORTS
		GPS1:/dev/ttyS2
	DRIVERS
		gps
		imu/bmi088
		pwm_out_sim
		px4fmu
		test_ppm
		rc_input
	MODULES
		airspeed_selector
		attitude_estimator_q
		commander
		dataman
		ekf2
		events
		land_detector
		landing_target_estimator
		load_mon
		local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_pos_control
		mc_rate_control
		navigator
		rc_update
		rover_pos_control
		sensors
		sih
		temperature_compensation
		vmount
	SYSTEMCMDS
		bl_update
		config
		dmesg
		dumpfile
		esc_calib
		hardfault_log
		i2cdetect
		led_control
		mixer
		motor_ramp
		motor_test
		nshterm
		param
		perf
		pwm
		reboot
		reflect
		sd_bench
		shutdown
		top
		topic_listener
		tune_control
		usb_connected
		ver
		work_queue
	EXAMPLES
	)
