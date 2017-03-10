//***************************************************************************
// examples/helloxx/helloxx_main.cxx
//
//   Copyright (C) 2009, 2011-2013 Gregory Nutt. All rights reserved.
//   Author: Gregory Nutt <gnutt@nuttx.org>
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
// 3. Neither the name NuttX nor the names of its contributors may be
//    used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//***************************************************************************

//***************************************************************************
// Included Files
//***************************************************************************

#include <nuttx/config.h>

#include <stdio.h>
#include <errno.h>
#include <uORB/uORBCommon.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/output_pwm.h>
#include <fmx10_config.h>
#include <fmx10_time.h>
#include <drivers/drv_hrt.h>


extern "C" { __EXPORT int msg_main(int argc, char *argv[]); }


int msg_main(int argc, char *argv[])
 {
     struct output_pwm_s t;
     orb_advert_t ptopic = NULL;
     int i = 0;

     memset(&t,0,sizeof(t));
     while(1)
     {
	t.timestamp = hrt_absolute_time();
	t.channel_count = 2;
	t.values[0] = i;
	t.values[1] = i + 1;
	t._padding0[0] = i;
	t._padding0[1] = i + 1;
	if(ptopic == NULL)
	    ptopic = orb_advertise(ORB_ID(output_pwm), &t);
	else
	    orb_publish(ORB_ID(output_pwm), ptopic, &t);
	i++;

	usleep(1);
     }
     printf("hello world!\n");
    return 0;
}
