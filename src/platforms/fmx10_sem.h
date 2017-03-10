/****************************************************************************
 *
 *   Copyright (c) 2016 FMX10 Development Team. All rights reserved.
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
 * @file fmx10_sem.h
 *
 * Synchronization primitive: Semaphore
 */

#pragma once

#include <semaphore.h>


#ifdef __FMX10_DARWIN

__BEGIN_DECLS

typedef struct {
	pthread_mutex_t lock;
	pthread_cond_t wait;
	int value;
} fmx10_sem_t;

__EXPORT int		fmx10_sem_init(fmx10_sem_t *s, int pshared, unsigned value);
__EXPORT int		fmx10_sem_wait(fmx10_sem_t *s);
__EXPORT int		fmx10_sem_timedwait(fmx10_sem_t *sem, const struct timespec *abstime);
__EXPORT int		fmx10_sem_post(fmx10_sem_t *s);
__EXPORT int		fmx10_sem_getvalue(fmx10_sem_t *s, int *sval);
__EXPORT int		fmx10_sem_destroy(fmx10_sem_t *s);

__END_DECLS

#else

__BEGIN_DECLS

typedef sem_t fmx10_sem_t;

#define fmx10_sem_init	 sem_init
#define fmx10_sem_wait	 sem_wait
#define fmx10_sem_post	 sem_post
#define fmx10_sem_getvalue sem_getvalue
#define fmx10_sem_destroy	 sem_destroy

#ifdef __FMX10_QURT
__EXPORT int		fmx10_sem_timedwait(fmx10_sem_t *sem, const struct timespec *abstime);
#else
#define fmx10_sem_timedwait	 sem_timedwait
#endif

__END_DECLS

#endif
