/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * @file fmx10_posix.h
 *
 * Includes POSIX-like functions for virtual character devices
 */

#pragma once

#include <fmx10_defines.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>

#if defined(__FMX10_QURT)
#include <dspal_types.h>
#else
#include <sys/types.h>
#endif

#include "fmx10_sem.h"


#ifdef __FMX10_NUTTX

#define  FMX10_F_RDONLY 1
#define  FMX10_F_WRONLY 2

typedef struct pollfd fmx10_pollfd_struct_t;

#if defined(__cplusplus)
#define _GLOBAL ::
#else
#define _GLOBAL
#endif
#define fmx10_open 	_GLOBAL open
#define fmx10_close 	_GLOBAL close
#define fmx10_ioctl 	_GLOBAL ioctl
#define fmx10_write 	_GLOBAL write
#define fmx10_read 	_GLOBAL read
#define fmx10_poll 	_GLOBAL poll
#define fmx10_fsync 	_GLOBAL fsync
#define fmx10_access 	_GLOBAL access
#define fmx10_getpid 	_GLOBAL getpid

#define  FMX10_STACK_OVERHEAD	0

#elif defined(__FMX10_POSIX)

#define  FMX10_F_RDONLY O_RDONLY
#define  FMX10_F_WRONLY O_WRONLY
#define  FMX10_F_CREAT  O_CREAT
#define	 FMX10_STACK_OVERHEAD	8192

typedef short pollevent_t;

typedef struct {
	/* This part of the struct is POSIX-like */
	int		fd;       /* The descriptor being polled */
	pollevent_t 	events;   /* The input event flags */
	pollevent_t 	revents;  /* The output event flags */

	/* Required for FMX10 compatibility */
	fmx10_sem_t   *sem;  	/* Pointer to semaphore used to post output event */
	void   *priv;     	/* For use by drivers */
} fmx10_pollfd_struct_t;

__BEGIN_DECLS

__EXPORT int 		fmx10_open(const char *path, int flags, ...);
__EXPORT int 		fmx10_close(int fd);
__EXPORT ssize_t	fmx10_read(int fd, void *buffer, size_t buflen);
__EXPORT ssize_t	fmx10_write(int fd, const void *buffer, size_t buflen);
__EXPORT int		fmx10_ioctl(int fd, int cmd, unsigned long arg);
__EXPORT int		fmx10_poll(fmx10_pollfd_struct_t *fds, nfds_t nfds, int timeout);
__EXPORT int		fmx10_fsync(int fd);
__EXPORT int		fmx10_access(const char *pathname, int mode);
__EXPORT fmx10_task_t	fmx10_getpid(void);

__EXPORT void		fmx10_enable_sim_lockstep(void);
__EXPORT void		fmx10_sim_start_delay(void);
__EXPORT void		fmx10_sim_stop_delay(void);
__EXPORT bool		fmx10_sim_delay_enabled(void);

__END_DECLS
#else
#error "No TARGET OS Provided"
#endif


// The stack size is intended for 32-bit architectures; therefore
// we often run out of stack space when pointers are larger than 4 bytes.
// Double the stack size on posix when we're on a 64-bit architecture.
// Most full-scale OS use 1-4K of memory from the stack themselves
#define FMX10_STACK_ADJUSTED(_s) (_s * (__SIZEOF_POINTER__ >> 2) + FMX10_STACK_OVERHEAD)

__BEGIN_DECLS
extern int fmx10_errno;

__EXPORT void		fmx10_show_devices(void);
__EXPORT void		fmx10_show_files(void);
__EXPORT const char 	*fmx10_get_device_names(unsigned int *handle);

__EXPORT void		fmx10_show_topics(void);
__EXPORT const char 	*fmx10_get_topic_names(unsigned int *handle);

#ifndef __FMX10_QURT
/*
 * The UNIX epoch system time following the system clock
 */
__EXPORT uint64_t	hrt_system_time(void);

__EXPORT bool		fmx10_exit_requested(void);
#endif

__END_DECLS
