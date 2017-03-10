/****************************************************************************
 *
 *   Copyright (c) 2012-2015 FMX10 Development Team. All rights reserved.
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

#include "uORBUtils.hpp"
#include <stdio.h>
#include <errno.h>

int uORB::Utils::node_mkpath
(
	char *buf,
	Flavor f,
	const struct orb_metadata *meta,
	int *instance
)
{
	unsigned len;

	unsigned index = 0;

	if (instance != nullptr) {
		index = *instance;
	}

	len = snprintf(buf, orb_maxpath, "/%s/%s%d",
		       (f == PUBSUB) ? "obj" : "param",
		       meta->o_name, index);

	if (len >= orb_maxpath) {
		return -ENAMETOOLONG;
	}

	return OK;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int uORB::Utils::node_mkpath(char *buf, Flavor f,
			     const char *orbMsgName)
{
	unsigned len;

	unsigned index = 0;

	len = snprintf(buf, orb_maxpath, "/%s/%s%d", (f == PUBSUB) ? "obj" : "param",
		       orbMsgName, index);

	if (len >= orb_maxpath) {
		return -ENAMETOOLONG;
	}

	return OK;
}