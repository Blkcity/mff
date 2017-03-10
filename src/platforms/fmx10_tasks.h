/****************************************************************************
 *
 *   Copyright (C) 2012 FMX10 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
 *   Author: Mark Charlebois <charlebm@gmail.com> 2015
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
 * @file fmx10_tasks.h
 * Preserve existing task API call signature with OS abstraction
 */

#pragma once

#include <stdbool.h>

#ifdef __FMX10_ROS

#error "FMX10 tasks not supported in ROS"

#elif defined(__FMX10_NUTTX)
typedef int fmx10_task_t;

#include <sys/prctl.h>
#define fmx10_prctl prctl

/** Default scheduler type */
#if CONFIG_RR_INTERVAL > 0
# define SCHED_DEFAULT  SCHED_RR
#else
# define SCHED_DEFAULT  SCHED_FIFO
#endif

#define fmx10_task_exit(x) _exit(x)

#elif defined(__FMX10_POSIX) || defined(__FMX10_QURT)
#include <pthread.h>
#include <sched.h>

/** Default scheduler type */
#define SCHED_DEFAULT	SCHED_FIFO
#ifdef __FMX10_LINUX
#define SCHED_PRIORITY_MAX sched_get_priority_max(SCHED_FIFO)
#define SCHED_PRIORITY_MIN sched_get_priority_min(SCHED_FIFO)
#define SCHED_PRIORITY_DEFAULT (((sched_get_priority_max(SCHED_FIFO) - sched_get_priority_min(SCHED_FIFO)) / 2) + sched_get_priority_min(SCHED_FIFO))
#elif defined(__FMX10_DARWIN)
#define SCHED_PRIORITY_MAX sched_get_priority_max(SCHED_FIFO)
#define SCHED_PRIORITY_MIN sched_get_priority_min(SCHED_FIFO)
#define SCHED_PRIORITY_DEFAULT (((sched_get_priority_max(SCHED_FIFO) - sched_get_priority_min(SCHED_FIFO)) / 2) + sched_get_priority_min(SCHED_FIFO))
#elif defined(__FMX10_QURT)
#define SCHED_PRIORITY_MAX 255
#define SCHED_PRIORITY_MIN 0
#define SCHED_PRIORITY_DEFAULT 20
#else
#error "No target OS defined"
#endif

#if defined (__FMX10_LINUX)
#include <sys/prctl.h>
#else
#define PR_SET_NAME	1
#endif

typedef int fmx10_task_t;

typedef struct {
	int argc;
	char **argv;
} fmx10_task_args_t;
#else
#error "No target OS defined"
#endif

typedef int (*fmx10_main_t)(int argc, char *argv[]);

__BEGIN_DECLS

/** Reboots the board */
__EXPORT void fmx10_systemreset(bool to_bootloader) noreturn_function;

/** Starts a task and performs any specific accounting, scheduler setup, etc. */
__EXPORT fmx10_task_t fmx10_task_spawn_cmd(const char *name,
				       int scheduler,
				       int priority,
				       int stack_size,
				       fmx10_main_t entry,
				       char *const argv[]);

/** Deletes a task - does not do resource cleanup **/
__EXPORT int fmx10_task_delete(fmx10_task_t pid);

/** Send a signal to a task **/
__EXPORT int fmx10_task_kill(fmx10_task_t pid, int sig);

/** Exit current task with return value **/
__EXPORT void fmx10_task_exit(int ret);

/** Show a list of running tasks **/
__EXPORT void fmx10_show_tasks(void);

/** See if a task is running **/
__EXPORT bool fmx10_task_is_running(const char *taskname);

#ifdef __FMX10_POSIX
/** set process (and thread) options */
__EXPORT int fmx10_prctl(int option, const char *arg2, fmx10_task_t pid);
#endif

/** return the name of the current task */
__EXPORT const char *fmx10_get_taskname(void);

__END_DECLS

