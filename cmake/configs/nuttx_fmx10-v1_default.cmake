include(nuttx/fmx10_impl_nuttx)

fmx10_nuttx_configure(HWCLASS m7 CONFIG nsh ROMFS y ROMFSROOT fmx10_common)

set(CMAKE_TOOLCHAIN_FILE ${FMX10_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-none-eabi.cmake)

#set(config_uavcan_num_ifaces 2)

set(config_module_list
	#
	# Board support modules
	#
		drivers/device
		drivers/stm32
		drivers/stm32/adc
	#	drivers/stm32/tone_alarm
		drivers/led
		drivers/fmx10fmu
		drivers/boards/fmx10-v1
	#	drivers/rgbled
		drivers/mpu6000
		drivers/mpu6500
	#	drivers/mpu9250
		drivers/hmc5883
		drivers/ms5611
		drivers/ist8310
		drivers/spl06001
	#	drivers/mb12xx
	#	drivers/srf02
	#	drivers/sf0x
	#	drivers/ll40ls
	#	drivers/trone
	#	drivers/gps
	#	drivers/pwm_out_sim
	#	drivers/hott
	#	drivers/hott/hott_telemetry
	#	drivers/hott/hott_sensors
	#	drivers/blinkm
	#	drivers/airspeed
	#	drivers/ets_airspeed
	#	drivers/meas_airspeed
	#	drivers/frsky_telemetry
	#	modules/sensors
	#	drivers/mkblctrl
	#	drivers/fmx10flow
	#	drivers/oreoled
	#	drivers/pwm_input
	#	drivers/camera_trigger
	#	drivers/bst
	#	drivers/snapdragon_rc_pwm
	#	drivers/lis3mdl
	#	drivers/bmp280
	#	drivers/bma180
	#	drivers/bmi160
	#	drivers/tap_esc

	#
	# System commands
	#
	#	systemcmds/bl_update
	#	systemcmds/config
	#	systemcmds/dumpfile
	#	systemcmds/esc_calib
	#	systemcmds/hardfault_log
	#	systemcmds/mixer
	#	systemcmds/motor_ramp
	#	systemcmds/mtd
	#	systemcmds/nshterm
		systemcmds/param
	#	systemcmds/perf
	#	systemcmds/pwm
	#	systemcmds/reboot
	#	systemcmds/sd_bench
	#	systemcmds/top
	#	systemcmds/topic_listener
	#	systemcmds/ver

	#
	# General system control
	#
	#    	modules/commander
	#	modules/load_mon
	#	modules/navigator
	#	modules/mavlink
	#	modules/gpio_led
	#	modules/uavcan
	#	modules/land_detector
		modules/hello
		modules/msgtopic
	#	modules/ipcproxy
	#	modules/ipcproxy/send
	#	modules/ipcproxy/receive

	#
	# Estimation modules
	#
	#	modules/attitude_estimator_q
	#	modules/position_estimator_inav
	#	modules/local_position_estimator
	#	modules/ekf2

	#
	# Vehicle Control
	#
	# modules/segway # XXX Needs GCC 4.7 fix
	#	modules/fw_pos_control_l1
	#	modules/fw_att_control
	#	modules/mc_att_control
	#	modules/mc_pos_control
	#	modules/vtol_att_control

	#
	# Logging
	#
	#	modules/logger
	#	modules/sdlog2

	#
	# Library modules
	#
		modules/param
		modules/systemlib
		modules/systemlib/mixer
		modules/uORB
		modules/fmx10_simple_app
	#	modules/uORB/uORB_tests
	#	modules/dataman

	#
	# Libraries
	#
	#	lib/controllib
		lib/mathlib
		lib/mathlib/math/filter
		lib/rc
	#	lib/ecl
	#	lib/external_lgpl
	#	lib/geo
	#	lib/geo_lookup
		lib/conversion
	#	lib/launchdetection
	#	lib/terrain_estimation
	#	lib/runway_takeoff
	#	lib/tailsitter_recovery
		lib/DriverFramework/framework
		platforms/nuttx

	# had to add for cmake, not sure why wasn't in original config
		platforms/common
		platforms/nuttx/fmx10_layer

	#
	# OBC challenge
	#
	#	modules/bottle_drop

	#
	# Rover apps
	#
	#	examples/rover_steering_control

	#
	# Demo apps
	#
	#examples/math_demo
	# Tutorial code from
	# https://px4.io/dev/px4_simple_app
	#	examples/px4_simple_app

	# Tutorial code from
	# https://px4.io/dev/daemon
	#examples/px4_daemon_app
	#examples/fm_simple_app

	# Tutorial code from
	# https://px4.io/dev/debug_values
	#examples/px4_mavlink_debug

	# Tutorial code from
	# https://px4.io/dev/example_fixedwing_control
	#examples/fixedwing_control

	# Hardware test
	#examples/hwtest
)

#set(config_extra_builtin_cmds
#	serdis
#	sercon
#	)

#set(config_extra_libs
#	uavcan
#	uavcan_stm32_driver
#	)

#set(config_io_extra_libs
#	)

#add_custom_target(sercon)
#set_target_properties(sercon PROPERTIES
#	PRIORITY "SCHED_PRIORITY_DEFAULT"
#	MAIN "sercon"
#	STACK_MAIN "2048")

#add_custom_target(serdis)
#set_target_properties(serdis PROPERTIES
#	PRIORITY "SCHED_PRIORITY_DEFAULT"
#	MAIN "serdis"
#	STACK_MAIN "2048")
