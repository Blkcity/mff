/****************************************************************************
 *
 *   Copyright (c) 2015 FMX10 Development Team. All rights reserved.
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
 * @file fmx10_sem.cpp
 *
 * FMX10 Middleware Wrapper Linux Implementation
 */

#include <fmx10_defines.h>
#include <fmx10_middleware.h>
#include <fmx10_workqueue.h>
#include <stdint.h>
#include <stdio.h>
#include <pthread.h>
#include <errno.h>

#ifdef __FMX10_DARWIN

#include <fmx10_posix.h>

int fmx10_sem_init(fmx10_sem_t *s, int pshared, unsigned value)
{
	// We do not used the process shared arg
	(void)pshared;
	s->value = value;
	pthread_cond_init(&(s->wait), NULL);
	pthread_mutex_init(&(s->lock), NULL);

	return 0;
}

int fmx10_sem_wait(fmx10_sem_t *s)
{
	int ret = pthread_mutex_lock(&(s->lock));

	if (ret) {
		return ret;
	}

	s->value--;

	if (s->value < 0) {
		ret = pthread_cond_wait(&(s->wait), &(s->lock));

	} else {
		ret = 0;
	}

	if (ret) {
		FMX10_WARN("fmx10_sem_wait failure");
	}

	int mret = pthread_mutex_unlock(&(s->lock));

	return (ret) ? ret : mret;
}

int fmx10_sem_timedwait(fmx10_sem_t *s, const struct timespec *abstime)
{
	int ret = pthread_mutex_lock(&(s->lock));

	if (ret) {
		return ret;
	}

	s->value--;
	errno = 0;

	if (s->value < 0) {
		ret = pthread_cond_timedwait(&(s->wait), &(s->lock), abstime);

	} else {
		ret = 0;
	}

	int err = ret;

	if (err != 0 && err != ETIMEDOUT) {
		setbuf(stdout, NULL);
		setbuf(stderr, NULL);
		const unsigned NAMELEN = 32;
		char thread_name[NAMELEN] = {};
		(void)pthread_getname_np(pthread_self(), thread_name, NAMELEN);
		FMX10_WARN("%s: fmx10_sem_timedwait failure: ret: %d, %s", thread_name, ret, strerror(err));
	}

	int mret = pthread_mutex_unlock(&(s->lock));

	return (err) ? err : mret;
}

int fmx10_sem_post(fmx10_sem_t *s)
{
	int ret = pthread_mutex_lock(&(s->lock));

	if (ret) {
		return ret;
	}

	s->value++;

	if (s->value <= 0) {
		ret = pthread_cond_signal(&(s->wait));

	} else {
		ret = 0;
	}

	if (ret) {
		FMX10_WARN("fmx10_sem_post failure");
	}

	int mret = pthread_mutex_unlock(&(s->lock));

	// return the cond signal failure if present,
	// else return the mutex status
	return (ret) ? ret : mret;
}

int fmx10_sem_getvalue(fmx10_sem_t *s, int *sval)
{
	int ret = pthread_mutex_lock(&(s->lock));

	if (ret) {
		FMX10_WARN("fmx10_sem_getvalue failure");
	}

	if (ret) {
		return ret;
	}

	*sval = s->value;
	ret = pthread_mutex_unlock(&(s->lock));

	return ret;
}

int fmx10_sem_destroy(fmx10_sem_t *s)
{
	pthread_mutex_lock(&(s->lock));
	pthread_cond_destroy(&(s->wait));
	pthread_mutex_unlock(&(s->lock));
	pthread_mutex_destroy(&(s->lock));

	return 0;
}

#endif