/****************************************************************************
 *
 *   Copyright (c) 2014 FMX10 Development Team. All rights reserved.
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
 * @file fmx10_includes.h
 *
 * Includes headers depending on the build target
 */

#pragma once

#include <stdbool.h>

#if defined(__FMX10_ROS)
/*
 * Building for running within the ROS environment
 */

#ifdef __cplusplus
#include "ros/ros.h"
//#include <fmx10_rc_channels.h>
//#include <fmx10_vehicle_attitude.h>
//#include <fmx10_vehicle_attitude_setpoint.h>
//#include <fmx10_manual_control_setpoint.h>
//#include <fmx10_actuator_controls.h>
//#include <fmx10_vehicle_rates_setpoint.h>
//#include <fmx10_mc_virtual_rates_setpoint.h>
//#include <fmx10_vehicle_attitude.h>
//#include <fmx10_control_state.h>
//#include <fmx10_vehicle_control_mode.h>
//#include <fmx10_actuator_armed.h>
//#include <fmx10_parameter_update.h>
//#include <fmx10_vehicle_status.h>
//#include <fmx10_vehicle_local_position_setpoint.h>
//#include <fmx10_vehicle_global_velocity_setpoint.h>
//#include <fmx10_vehicle_local_position.h>
//#include <fmx10_position_setpoint_triplet.h>
//#include <fmx10_offboard_control_mode.h>
//#include <fmx10_vehicle_force_setpoint.h>
#endif

#elif defined(__FMX10_NUTTX)
/*
 * Building for NuttX
 */
#include <nuttx/config.h>
#include <uORB/uORB.h>
#ifdef __cplusplus
//#include <platforms/nuttx/fmx10_messages/fmx10_rc_channels.h>
//#include <platforms/nuttx/fmx10_messages/fmx10_vehicle_attitude_setpoint.h>
//#include <platforms/nuttx/fmx10_messages/fmx10_manual_control_setpoint.h>
//#include <platforms/nuttx/fmx10_messages/fmx10_actuator_controls.h>
//#include <platforms/nuttx/fmx10_messages/fmx10_vehicle_rates_setpoint.h>
//#include <platforms/nuttx/fmx10_messages/fmx10_vehicle_attitude.h>
//#include <platforms/nuttx/fmx10_messages/fmx10_control_state.h>
//#include <platforms/nuttx/fmx10_messages/fmx10_vehicle_control_mode.h>
//#include <platforms/nuttx/fmx10_messages/fmx10_actuator_armed.h>
//#include <platforms/nuttx/fmx10_messages/fmx10_parameter_update.h>
//#include <platforms/nuttx/fmx10_messages/fmx10_vehicle_status.h>
//#include <platforms/nuttx/fmx10_messages/fmx10_vehicle_local_position_setpoint.h>
//#include <platforms/nuttx/fmx10_messages/fmx10_vehicle_global_velocity_setpoint.h>
//#include <platforms/nuttx/fmx10_messages/fmx10_vehicle_local_position.h>
//#include <platforms/nuttx/fmx10_messages/fmx10_position_setpoint_triplet.h>
//#include <platforms/nuttx/fmx10_messages/fmx10_offboard_control_mode.h>
//#include <platforms/nuttx/fmx10_messages/fmx10_vehicle_force_setpoint.h>
//#include <platforms/nuttx/fmx10_messages/fmx10_camera_trigger.h>
#endif
#include <err.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>

#elif defined(__FMX10_POSIX) && !defined(__FMX10_QURT)
#include <string.h>
#include <assert.h>
#include <uORB/uORB.h>

#define ASSERT(x) assert(x)

#ifdef __cplusplus
//#include <platforms/posix/fmx10_messages/fmx10_rc_channels.h>
//#include <platforms/posix/fmx10_messages/fmx10_vehicle_attitude_setpoint.h>
//#include <platforms/posix/fmx10_messages/fmx10_manual_control_setpoint.h>
//#include <platforms/posix/fmx10_messages/fmx10_actuator_controls.h>
//#include <platforms/posix/fmx10_messages/fmx10_vehicle_rates_setpoint.h>
//#include <platforms/posix/fmx10_messages/fmx10_vehicle_attitude.h>
//#include <platforms/posix/fmx10_messages/fmx10_control_state.h>
//#include <platforms/posix/fmx10_messages/fmx10_vehicle_control_mode.h>
//#include <platforms/posix/fmx10_messages/fmx10_actuator_armed.h>
//#include <platforms/posix/fmx10_messages/fmx10_parameter_update.h>
//#include <platforms/posix/fmx10_messages/fmx10_vehicle_status.h>
//#include <platforms/posix/fmx10_messages/fmx10_vehicle_local_position_setpoint.h>
//#include <platforms/posix/fmx10_messages/fmx10_vehicle_global_velocity_setpoint.h>
//#include <platforms/posix/fmx10_messages/fmx10_vehicle_local_position.h>
//#include <platforms/posix/fmx10_messages/fmx10_position_setpoint_triplet.h>
#endif
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#elif defined(__FMX10_QURT)
#include <string.h>
#include <assert.h>
#include <uORB/uORB.h>

#define ASSERT(x) assert(x)

#ifdef __cplusplus
//#include <platforms/qurt/fmx10_messages/fmx10_rc_channels.h>
//#include <platforms/qurt/fmx10_messages/fmx10_vehicle_attitude_setpoint.h>
//#include <platforms/qurt/fmx10_messages/fmx10_manual_control_setpoint.h>
//#include <platforms/qurt/fmx10_messages/fmx10_actuator_controls.h>
//#include <platforms/qurt/fmx10_messages/fmx10_vehicle_rates_setpoint.h>
//#include <platforms/qurt/fmx10_messages/fmx10_vehicle_attitude.h>
//#include <platforms/qurt/fmx10_messages/fmx10_control_state.h>
//#include <platforms/qurt/fmx10_messages/fmx10_vehicle_control_mode.h>
//#include <platforms/qurt/fmx10_messages/fmx10_actuator_armed.h>
//#include <platforms/qurt/fmx10_messages/fmx10_parameter_update.h>
//#include <platforms/qurt/fmx10_messages/fmx10_vehicle_status.h>
//#include <platforms/qurt/fmx10_messages/fmx10_vehicle_local_position_setpoint.h>
//#include <platforms/qurt/fmx10_messages/fmx10_vehicle_global_velocity_setpoint.h>
//#include <platforms/qurt/fmx10_messages/fmx10_vehicle_local_position.h>
//#include <platforms/qurt/fmx10_messages/fmx10_position_setpoint_triplet.h>
#endif
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#else
#error "No target platform defined"
#endif
