/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *   Author: @author Mark Charlebois <charlebm#gmail.com>
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
 * @file fmx10_posix_tasks.c
 * Implementation of existing task API for Linux
 */

#include <fmx10_log.h>
#include <fmx10_defines.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdbool.h>
#include <signal.h>
#include <fcntl.h>
#include <sched.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <limits.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <string>

#include <fmx10_tasks.h>
#include <fmx10_posix.h>
#include <systemlib/err.h>

#define MAX_CMD_LEN 100

#define FMX10_MAX_TASKS 50
#define SHELL_TASK_ID (FMX10_MAX_TASKS+1)

pthread_t _shell_task_id = 0;
pthread_mutex_t task_mutex = PTHREAD_MUTEX_INITIALIZER;

struct task_entry {
	pthread_t pid;
	std::string name;
	bool isused;
	task_entry() : isused(false) {}
};

static task_entry taskmap[FMX10_MAX_TASKS] = {};

typedef struct {
	fmx10_main_t entry;
	char name[16]; //pthread_setname_np is restricted to 16 chars
	int argc;
	char *argv[];
	// strings are allocated after the struct data
} pthdata_t;

static void *entry_adapter(void *ptr)
{
	pthdata_t *data = (pthdata_t *) ptr;

	int rv;

	// set the threads name
#ifdef __FMX10_DARWIN
	rv = pthread_setname_np(data->name);
#else
	rv = pthread_setname_np(pthread_self(), data->name);
#endif

	if (rv) {
		FMX10_ERR("fmx10_task_spawn_cmd: failed to set name of thread %d %d\n", rv, errno);
	}

	data->entry(data->argc, data->argv);
	free(ptr);
	FMX10_DEBUG("Before fmx10_task_exit");
	fmx10_task_exit(0);
	FMX10_DEBUG("After fmx10_task_exit");

	return NULL;
}

void
fmx10_systemreset(bool to_bootloader)
{
	FMX10_WARN("Called fmx10_system_reset");
	exit(0);
}

fmx10_task_t fmx10_task_spawn_cmd(const char *name, int scheduler, int priority, int stack_size, fmx10_main_t entry,
			      char *const argv[])
{

	int rv;
	int argc = 0;
	int i;
	unsigned int len = 0;
	unsigned long offset;
	unsigned long structsize;
	char *p = (char *)argv;

	pthread_attr_t attr;
	struct sched_param param = {};

	// Calculate argc
	while (p != (char *)0) {
		p = argv[argc];

		if (p == (char *)0) {
			break;
		}

		++argc;
		len += strlen(p) + 1;
	}

	structsize = sizeof(pthdata_t) + (argc + 1) * sizeof(char *);

	// not safe to pass stack data to the thread creation
	pthdata_t *taskdata = (pthdata_t *)malloc(structsize + len);
	memset(taskdata, 0, structsize + len);
	offset = ((unsigned long)taskdata) + structsize;

	strncpy(taskdata->name, name, 16);
	taskdata->name[15] = 0;
	taskdata->entry = entry;
	taskdata->argc = argc;

	for (i = 0; i < argc; i++) {
		FMX10_DEBUG("arg %d %s\n", i, argv[i]);
		taskdata->argv[i] = (char *)offset;
		strcpy((char *)offset, argv[i]);
		offset += strlen(argv[i]) + 1;
	}

	// Must add NULL at end of argv
	taskdata->argv[argc] = (char *)0;

	FMX10_DEBUG("starting task %s", name);

	rv = pthread_attr_init(&attr);

	if (rv != 0) {
		FMX10_ERR("fmx10_task_spawn_cmd: failed to init thread attrs");
		free(taskdata);
		return (rv < 0) ? rv : -rv;
	}

#ifndef __FMX10_DARWIN

	if (stack_size < PTHREAD_STACK_MIN) {
		stack_size = PTHREAD_STACK_MIN;
	}

	rv = pthread_attr_setstacksize(&attr, FMX10_STACK_ADJUSTED(stack_size));

	if (rv != 0) {
		FMX10_ERR("pthread_attr_setstacksize to %d returned error (%d)", stack_size, rv);
		pthread_attr_destroy(&attr);
		free(taskdata);
		return (rv < 0) ? rv : -rv;
	}

#endif

	rv = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

	if (rv != 0) {
		FMX10_ERR("fmx10_task_spawn_cmd: failed to set inherit sched");
		pthread_attr_destroy(&attr);
		free(taskdata);
		return (rv < 0) ? rv : -rv;
	}

	rv = pthread_attr_setschedpolicy(&attr, scheduler);

	if (rv != 0) {
		FMX10_ERR("fmx10_task_spawn_cmd: failed to set sched policy");
		pthread_attr_destroy(&attr);
		free(taskdata);
		return (rv < 0) ? rv : -rv;
	}

	param.sched_priority = priority;

	rv = pthread_attr_setschedparam(&attr, &param);

	if (rv != 0) {
		FMX10_ERR("fmx10_task_spawn_cmd: failed to set sched param");
		pthread_attr_destroy(&attr);
		free(taskdata);
		return (rv < 0) ? rv : -rv;
	}

	pthread_mutex_lock(&task_mutex);

	int taskid = 0;

	for (i = 0; i < FMX10_MAX_TASKS; ++i) {
		if (taskmap[i].isused == false) {
			taskmap[i].name = name;
			taskmap[i].isused = true;
			taskid = i;
			break;
		}
	}

	if (i >= FMX10_MAX_TASKS) {
		pthread_attr_destroy(&attr);
		pthread_mutex_unlock(&task_mutex);
		free(taskdata);
		return -ENOSPC;
	}

	rv = pthread_create(&taskmap[taskid].pid, &attr, &entry_adapter, (void *) taskdata);

	if (rv != 0) {

		if (rv == EPERM) {
			//printf("WARNING: NOT RUNING AS ROOT, UNABLE TO RUN REALTIME THREADS\n");
			rv = pthread_create(&taskmap[taskid].pid, NULL, &entry_adapter, (void *) taskdata);

			if (rv != 0) {
				FMX10_ERR("fmx10_task_spawn_cmd: failed to create thread %d %d\n", rv, errno);
				taskmap[taskid].isused = false;
				pthread_attr_destroy(&attr);
				pthread_mutex_unlock(&task_mutex);
				free(taskdata);
				return (rv < 0) ? rv : -rv;
			}

		} else {
			pthread_attr_destroy(&attr);
			pthread_mutex_unlock(&task_mutex);
			free(taskdata);
			return (rv < 0) ? rv : -rv;
		}
	}

	pthread_attr_destroy(&attr);
	pthread_mutex_unlock(&task_mutex);

	return i;
}

int fmx10_task_delete(fmx10_task_t id)
{
	int rv = 0;
	pthread_t pid;
	FMX10_DEBUG("Called fmx10_task_delete");

	if (id < FMX10_MAX_TASKS && taskmap[id].isused) {
		pid = taskmap[id].pid;

	} else {
		return -EINVAL;
	}

	pthread_mutex_lock(&task_mutex);

	// If current thread then exit, otherwise cancel
	if (pthread_self() == pid) {
		taskmap[id].isused = false;
		pthread_mutex_unlock(&task_mutex);
		pthread_exit(0);

	} else {
		rv = pthread_cancel(pid);
	}

	taskmap[id].isused = false;
	pthread_mutex_unlock(&task_mutex);

	return rv;
}

void fmx10_task_exit(int ret)
{
	int i;
	pthread_t pid = pthread_self();

	// Get pthread ID from the opaque ID
	for (i = 0; i < FMX10_MAX_TASKS; ++i) {
		if (taskmap[i].pid == pid) {
			pthread_mutex_lock(&task_mutex);
			taskmap[i].isused = false;
			break;
		}
	}

	if (i >= FMX10_MAX_TASKS)  {
		FMX10_ERR("fmx10_task_exit: self task not found!");

	} else {
		FMX10_DEBUG("fmx10_task_exit: %s", taskmap[i].name.c_str());
	}

	pthread_mutex_unlock(&task_mutex);

	pthread_exit((void *)(unsigned long)ret);
}

int fmx10_task_kill(fmx10_task_t id, int sig)
{
	int rv = 0;
	pthread_t pid;
	FMX10_DEBUG("Called fmx10_task_kill %d", sig);

	if (id < FMX10_MAX_TASKS && taskmap[id].isused && taskmap[id].pid != 0) {
		pthread_mutex_lock(&task_mutex);
		pid = taskmap[id].pid;
		pthread_mutex_unlock(&task_mutex);

	} else {
		return -EINVAL;
	}

	// If current thread then exit, otherwise cancel
	rv = pthread_kill(pid, sig);

	return rv;
}

void fmx10_show_tasks()
{
	int idx;
	int count = 0;

	FMX10_INFO("Active Tasks:");

	for (idx = 0; idx < FMX10_MAX_TASKS; idx++) {
		if (taskmap[idx].isused) {
			FMX10_INFO("   %-10s %lu", taskmap[idx].name.c_str(), (unsigned long)taskmap[idx].pid);
			count++;
		}
	}

	if (count == 0) {
		FMX10_INFO("   No running tasks");
	}

}

bool fmx10_task_is_running(const char *taskname)
{
	int idx;

	for (idx = 0; idx < FMX10_MAX_TASKS; idx++) {
		if (taskmap[idx].isused && (strcmp(taskmap[idx].name.c_str(), taskname) == 0)) {
			return true;
		}
	}

	return false;
}

fmx10_task_t fmx10_getpid()
{
	pthread_t pid = pthread_self();
	fmx10_task_t ret = -1;

	pthread_mutex_lock(&task_mutex);

	for (int i = 0; i < FMX10_MAX_TASKS; i++) {
		if (taskmap[i].isused && taskmap[i].pid == pid) {
			ret = i;
		}
	}

	pthread_mutex_unlock(&task_mutex);
	return ret;
}

const char *fmx10_get_taskname()
{
	pthread_t pid = pthread_self();
	const char *prog_name = "UnknownApp";

	pthread_mutex_lock(&task_mutex);

	for (int i = 0; i < FMX10_MAX_TASKS; i++) {
		if (taskmap[i].isused && taskmap[i].pid == pid) {
			prog_name = taskmap[i].name.c_str();
		}
	}

	pthread_mutex_unlock(&task_mutex);

	return prog_name;
}

int fmx10_prctl(int option, const char *arg2, fmx10_task_t pid)
{
	int rv;

	switch (option) {
	case PR_SET_NAME:
		// set the threads name
#ifdef __FMX10_DARWIN
		rv = pthread_setname_np(arg2);
#else
		rv = pthread_setname_np(pthread_self(), arg2);
#endif
		break;

	default:
		rv = -1;
		FMX10_WARN("FAILED SETTING TASK NAME");
		break;
	}

	return rv;
}

