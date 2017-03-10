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
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <poll.h>
#include <arch/math.h>
#include <uORB/uORBCommon.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/output_pwm.h>
#include <fmx10_config.h>
#include <fmx10_time.h>


extern "C" { __EXPORT int helloxx_main(int argc, char *argv[]); }

/*
int helloxx_main(int argc, char *argv[])
 {
     int error_count = 0;
     struct output_pwm_s t;
     bool fdupdate;
     int msg_sub_fd = orb_subscribe(ORB_ID(output_pwm));

     fmx10_pollfd_struct_t fds[1];
     fds[0].fd = msg_sub_fd;
     fds[0].events = POLLIN;
    while(1)
    {
	int poll_ret = fmx10_poll(fds,1,1000);
	if (poll_ret < 0)
	{
	    if (error_count < 10 || error_count % 50 == 0)
		printf("error return value from poll():%d\n",poll_ret);
	    error_count++;
	}
	orb_check(msg_sub_fd, &fdupdate);
	if(fdupdate)
	{
//	    if (fds[0].revents & POLLIN)
	    {
		orb_copy(ORB_ID(output_pwm),msg_sub_fd,&t);
		printf("get values:%d\n",t.values[0]);
	    }
	}
    }
    return 0;
}
*/

#define PI  	3.1415926f
#define N	10


float Creat_M(float *p,float *p_creat, int m, int n, int k);
float MatDet(float *p, int n);
//-----------------------------------------------
//功能: 求矩阵(n*n)的行列式
//入口参数: 矩阵的首地址，矩阵的行数
//返回值: 矩阵的行列式值
//----------------------------------------------
float MatDet(float *p, int n)
{
    int r, c, m;
    int lop = 0;
    float result = 0;
    float mid = 1;

    if (n != 1)
    {
        lop = (n == 2) ? 1 : n;            //控制求和循环次数,若为2阶，则循环1次，否则为n次
        for (m = 0; m < lop; m++)
        {
            mid = 1;            //顺序求和, 主对角线元素相乘之和
            for (r = 0, c = m; r < n; r++, c++)
            {
                mid = mid * (*(p+r*n+c%n));
            }
            result += mid;
        }
        for (m = 0; m < lop; m++)
        {
            mid = 1;            //逆序相减, 减去次对角线元素乘积
            for (r = 0, c = n-1-m+n; r < n; r++, c--)
            {
                mid = mid * (*(p+r*n+c%n));
            }
            result -= mid;
        }
    }
    else 
        result = *p;
    return result;
}

//----------------------------------------------------------------------------
//功能: 求k*k矩阵中元素A(m, n)的代数余之式
//入口参数: k*k矩阵的首地址，矩阵元素A的下标m,n,矩阵行数k
//返回值: k*k矩阵中元素A(m, n)的代数余之式
//----------------------------------------------------------------------------
float Creat_M(float *p,float *p_creat, int m, int n, int k)
{
    int i, j;
    float mid_result = 0;
    int sign = 1;
    float *p_mid;

    p_mid = p_creat;
    for (i = 0; i < k; i++)
    {
        for (j = 0; j < k; j++)
        {
            if (i != m && j != n) //将除第i行和第j列外的所有元素存储到以p_mid为首地址的内存单元
            {
                *p_mid++ = *(p+i*k+j);
            }
        }
    }
    sign = (m+n)%2 == 0 ? 1 : -1;    //代数余之式前面的正、负号
    mid_result = (float)sign*MatDet(p_creat, k-1);
    return mid_result;
}

int helloxx_main(int argc, char *argv[])
{
    hrt_abstime now = 0;
    hrt_abstime pre = 0;
    int i,j,n;
#if 0
    double val_sin,val_cos,arsin,arcos;
#endif
    float arcs[N][N];
    float ascs[N][N];
    float determ;
    int sec = N;
    float *p = &arcs[0][0];
    float *pc = &ascs[0][0];

    while(1)
    {
	for(i=0;i<N;i++)
	{
	    for(j=0;j<N;j++)
	    {
		arcs[i][j] = (pre%10)*PI;
	    }
	}
	now = hrt_absolute_time();
	for(i=0;i<100;i++)	
	{
	    determ = MatDet(p,sec);	
	    for(n = 0; n < N; n++)
	    {
		for(j = 0;j < N;j++)
		{
		    determ = Creat_M(p,pc,n,j,sec)/determ;
		}
	    }
	}
	pre = hrt_absolute_time();
#if 0
	{
	    now = hrt_absolute_time();
	    for(i=0;i<100000;i++)	
	    {
		val_sin = sin(i*PI/180);
		val_cos = cos(i*PI/180);
		arsin = asin(i/PI);
		arcos = acos(i/PI);
	    }
	    pre = hrt_absolute_time();
	    
    //	usleep(1000000);
	    arcos += val_sin+val_cos+arsin;
	    val_sin = arcos;
	}
#endif
	printf("use time:%llu,i:%d\n",pre - now,i);
    }	

    return 0;
}
