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
 * @file fmx10_defines.h
 *
 * Generally used magic defines
 */

#pragma once

#include <fmx10_log.h>
#include <math.h>

/* Get the name of the default value fiven the param name */
#define FMX10_PARAM_DEFAULT_VALUE_NAME(_name) PARAM_##_name##_DEFAULT

/* Shortcuts to define parameters when the default value is defined according to FMX10_PARAM_DEFAULT_VALUE_NAME */
#define FMX10_PARAM_DEFINE_INT32(_name) PARAM_DEFINE_INT32(_name, FMX10_PARAM_DEFAULT_VALUE_NAME(_name))
#define FMX10_PARAM_DEFINE_FLOAT(_name) PARAM_DEFINE_FLOAT(_name, FMX10_PARAM_DEFAULT_VALUE_NAME(_name))

#define FMX10_ERROR (-1)
#define FMX10_OK 0

#if defined(__FMX10_ROS)
/*
 * Building for running within the ROS environment
 */
#define noreturn_function
#ifdef __cplusplus
#include "ros/ros.h"
#endif

/* Main entry point */
#define FMX10_MAIN_FUNCTION(_prefix) int main(int argc, char **argv)

/* Get value of parameter by name, which is equal to the handle for ros */
#define FMX10_PARAM_GET_BYNAME(_name, _destpt) ros::param::get(_name, *_destpt)

#define FMX10_ISFINITE(x) std::isfinite(x)

#elif defined(__FMX10_NUTTX) || defined(__FMX10_POSIX)
/*
 * Building for NuttX or POSIX
 */
#include <platforms/fmx10_includes.h>
/* Main entry point */
#define FMX10_MAIN_FUNCTION(_prefix) int _prefix##_task_main(int argc, char *argv[])

/* Parameter handle datatype */
#include <systemlib/param/param.h>
typedef param_t fmx10_param_t;

/* Get value of parameter by name */
#define FMX10_PARAM_GET_BYNAME(_name, _destpt) param_get(param_find(_name), _destpt)

#else
#error "No target OS defined"
#endif

/*
 * NuttX Specific defines
 */
#if defined(__FMX10_NUTTX)

#define FMX10_ROOTFSDIR

#define _FMX10_IOC(x,y) _IOC(x,y)

#define fmx10_statfs_buf_f_bavail_t int

#define FMX10_ISFINITE(x) isfinite(x)

// mode for open with O_CREAT
#define FMX10_O_MODE_777 0777
#define FMX10_O_MODE_666 0666
#define FMX10_O_MODE_600 0600

#ifndef PRIu64
#define PRIu64 "llu"
#endif
#ifndef PRId64
#define PRId64 "lld"
#endif

#if !defined(offsetof)
#  define offsetof(TYPE, MEMBER) __builtin_offsetof (TYPE, MEMBER)
#endif

/*
 * POSIX Specific defines
 */
#elif defined(__FMX10_POSIX)

// Flag is meaningless on Linux
#define O_BINARY 0

// mode for open with O_CREAT
#define FMX10_O_MODE_777 (S_IRWXU | S_IRWXG | S_IRWXO)
#define FMX10_O_MODE_666 (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH )
#define FMX10_O_MODE_600 (S_IRUSR | S_IWUSR)


// NuttX _IOC is equivalent to Linux _IO
#define _FMX10_IOC(x,y) _IO(x,y)

/* FIXME - Used to satisfy build */
#define getreg32(a)    (*(volatile uint32_t *)(a))

#ifdef __FMX10_QURT
#define FMX10_TICKS_PER_SEC 1000L
#else
__BEGIN_DECLS
extern long FMX10_TICKS_PER_SEC;
__END_DECLS
#endif

#define USEC_PER_TICK (1000000UL/FMX10_TICKS_PER_SEC)
#define USEC2TICK(x) (((x)+(USEC_PER_TICK/2))/USEC_PER_TICK)

#define fmx10_statfs_buf_f_bavail_t unsigned long

#if defined(__FMX10_QURT)
#define FMX10_ROOTFSDIR
#elif defined(__FMX10_POSIX_EAGLE)
#define FMX10_ROOTFSDIR "/home/linaro"
#elif defined(__FMX10_POSIX_BEBOP)
#define FMX10_ROOTFSDIR "/home/root"
#else
#define FMX10_ROOTFSDIR "rootfs"
#endif

#endif


/*
 * Defines for ROS and Linux
 */
#if defined(__FMX10_ROS) || defined(__FMX10_POSIX)
#define OK 0
#define ERROR -1

#if defined(__FMX10_QURT)
#include "dspal_math.h"
__BEGIN_DECLS
#include <math.h>
__END_DECLS
#else
#include <math.h>
#endif

/* Float defines of the standard double length constants  */
#define M_E_F			(float)M_E
#define M_LOG2E_F		(float)M_LOG2E
#define M_LOG10E_F		(float)M_LOG10E
#define M_LN2_F			(float)M_LN2
#define M_LN10_F		(float)M_LN10
#define M_PI_F			(float)M_PI
#define M_TWOPI_F       	(M_PI_F * 2.0f)
#define M_PI_2_F		(float)M_PI_2
#define M_PI_4_F		(float)M_PI_4
#define M_3PI_4_F		(float)2.3561944901923448370E0f
#define M_SQRTPI_F      	(float)1.77245385090551602792981f
#define M_1_PI_F		(float)M_1_PI
#define M_2_PI_F		(float)M_2_PI
#define M_2_SQRTPI_F		1.12837916709551257390f
#define M_DEG_TO_RAD_F 		0.01745329251994f
#define M_RAD_TO_DEG_F 		57.2957795130823f
#define M_SQRT2_F		(float)M_SQRT2
#define M_SQRT1_2_F		(float)M_SQRT1_2
#define M_LN2LO_F       	1.9082149292705877000E-10f
#define M_LN2HI_F       	6.9314718036912381649E-1f
#define M_SQRT3_F		1.73205080756887719000f
#define M_IVLN10_F      	0.43429448190325182765f /* 1 / log(10) */
#define M_LOG2_E_F      	_M_LN2_F
#define M_INVLN2_F      	1.4426950408889633870E0f/* 1 / log(2)  */
#define M_DEG_TO_RAD 		0.01745329251994
#define M_RAD_TO_DEG 		57.2957795130823

#ifndef __FMX10_QURT

#if defined(__cplusplus)
#include <cmath>
#define FMX10_ISFINITE(x) std::isfinite(x)
#else
#define FMX10_ISFINITE(x) isfinite(x)
#endif
#endif

#endif

#if defined(__FMX10_QURT)

#define DEFAULT_PARAM_FILE "/fs/eeprom/parameters"

#define SIOCDEVPRIVATE 999999

// Missing math.h defines
#define FMX10_ISFINITE(x) __builtin_isfinite(x)

#endif

/*
 *Defines for all platforms
 */

/* wrapper for 2d matrices */
#define FMX10_ARRAY2D(_array, _ncols, _x, _y) (_array[_x * _ncols + _y])

/* wrapper for rotation matrices stored in arrays */
#define FMX10_R(_array, _x, _y) FMX10_ARRAY2D(_array, 3, _x, _y)
