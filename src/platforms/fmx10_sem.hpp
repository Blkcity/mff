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
 * @file fmx10_sem.hpp
 *
 * C++ synchronization helpers
 */

#pragma once

#include "fmx10_sem.h"


/**
 * @class Smart locking object that uses a semaphore. It automatically
 * takes the lock when created and releases the lock when the object goes out of
 * scope. Use like this:
 *
 *   fmx10_sem_t my_lock;
 *   int ret = fmx10_sem_init(&my_lock, 0, 1);
 *   ...
 *
 *   {
 *       SmartLock smart_lock(my_lock);
 *       //critical section start
 *       ...
 *       //critical section end
 *   }
 */
class SmartLock
{
public:
	SmartLock(fmx10_sem_t &sem) : _sem(sem) { do {} while (fmx10_sem_wait(&_sem) != 0); }
	~SmartLock() { fmx10_sem_post(&_sem); }
private:
	fmx10_sem_t &_sem;
};
