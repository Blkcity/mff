/****************************************************************************
 *
 *   Copyright (C) 2015-2016 FMX10 Development Team. All rights reserved.
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
 * @file fmx10_log.h
 * Platform dependant logging/debug implementation
 */

#pragma once

#include <visibility.h>

#define _FMX10_LOG_LEVEL_ALWAYS		0
#define _FMX10_LOG_LEVEL_DEBUG		1
#define _FMX10_LOG_LEVEL_WARN		2
#define _FMX10_LOG_LEVEL_ERROR		3
#define _FMX10_LOG_LEVEL_PANIC		4

// Used to silence unused variable warning
static inline void do_nothing(int level, ...)
{
	(void)level;
}

__BEGIN_DECLS

/**
 * initialize the orb logging. Logging to console still works without or before calling this.
 */
__EXPORT extern void fmx10_log_initialize(void);

__END_DECLS

/****************************************************************************
 * __fmx10_log_omit:
 * Compile out the message
 ****************************************************************************/
#define __fmx10_log_omit(level, FMT, ...)   do_nothing(level, ##__VA_ARGS__)

#if defined(__FMX10_ROS)

#include <ros/console.h>
#define FMX10_PANIC(...)	ROS_FATAL(__VA_ARGS__)
#define FMX10_ERR(...)	ROS_ERROR(__VA_ARGS__)
#define FMX10_WARN(...) 	ROS_WARN(__VA_ARGS__)
#define FMX10_INFO(...) 	ROS_INFO(__VA_ARGS__)
#define FMX10_DEBUG(...)	ROS_DEBUG(__VA_ARGS__)
#define FMX10_BACKTRACE()

#elif defined(__FMX10_QURT)
#include "qurt_log.h"
/****************************************************************************
 * Messages that should never be filtered or compiled out
 ****************************************************************************/
#define FMX10_LOG(FMT, ...) 	qurt_log(_FMX10_LOG_LEVEL_ALWAYS, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define FMX10_INFO(FMT, ...) 	qurt_log(_FMX10_LOG_LEVEL_ALWAYS, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define FMX10_BACKTRACE()

#if defined(TRACE_BUILD)
/****************************************************************************
 * Extremely Verbose settings for a Trace build
 ****************************************************************************/
#define FMX10_PANIC(FMT, ...)	qurt_log(_FMX10_LOG_LEVEL_PANIC, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define FMX10_ERR(FMT, ...)	qurt_log(_FMX10_LOG_LEVEL_ERROR, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define FMX10_WARN(FMT, ...) 	qurt_log(_FMX10_LOG_LEVEL_WARN,  __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define FMX10_DEBUG(FMT, ...) 	qurt_log(_FMX10_LOG_LEVEL_DEBUG, __FILE__, __LINE__, FMT, ##__VA_ARGS__)

#elif defined(DEBUG_BUILD)
/****************************************************************************
 * Verbose settings for a Debug build
 ****************************************************************************/
#define FMX10_PANIC(FMT, ...)	qurt_log(_FMX10_LOG_LEVEL_PANIC, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define FMX10_ERR(FMT, ...)	qurt_log(_FMX10_LOG_LEVEL_ERROR, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define FMX10_WARN(FMT, ...) 	qurt_log(_FMX10_LOG_LEVEL_WARN, __FILE__, __LINE__,  FMT, ##__VA_ARGS__)
#define FMX10_DEBUG(FMT, ...) 	qurt_log(_FMX10_LOG_LEVEL_DEBUG, __FILE__, __LINE__, FMT, ##__VA_ARGS__)

#elif defined(RELEASE_BUILD)
/****************************************************************************
 * Non-verbose settings for a Release build to minimize strings in build
 ****************************************************************************/
#define FMX10_PANIC(FMT, ...)	qurt_log(_FMX10_LOG_LEVEL_PANIC, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define FMX10_ERR(FMT, ...)	qurt_log(_FMX10_LOG_LEVEL_ERROR, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define FMX10_WARN(FMT, ...) 	__fmx10_log_omit(_FMX10_LOG_LEVEL_WARN, FMT, ##__VA_ARGS__)
#define FMX10_DEBUG(FMT, ...) 	__fmx10_log_omit(_FMX10_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)

#else
/****************************************************************************
 * Medium verbose settings for a default build
 ****************************************************************************/
#define FMX10_PANIC(FMT, ...)	qurt_log(_FMX10_LOG_LEVEL_PANIC, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define FMX10_ERR(FMT, ...)	qurt_log(_FMX10_LOG_LEVEL_ERROR, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define FMX10_WARN(FMT, ...) 	qurt_log(_FMX10_LOG_LEVEL_WARN,  __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define FMX10_DEBUG(FMT, ...) 	__fmx10_log_omit(_FMX10_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)

#endif
#define FMX10_LOG_NAMED(name, FMT, ...) 	qurt_log( _FMX10_LOG_LEVEL_ALWAYS, __FILE__, __LINE__, "%s " FMT, name, ##__VA_ARGS__)
#define FMX10_LOG_NAMED_COND(name, cond, FMT, ...) if( cond ) qurt_log( _FMX10_LOG_LEVEL_ALWAYS, __FILE__, __LINE__, "%s " FMT, name,  ##__VA_ARGS__)

#else

#include <inttypes.h>
#include <stdint.h>
#include <sys/cdefs.h>
#include <stdio.h>
#include <stdarg.h>

#include <fmx10_defines.h>

__BEGIN_DECLS

__EXPORT extern const char *__fmx10_log_level_str[_FMX10_LOG_LEVEL_PANIC + 1];
__EXPORT extern const char *__fmx10_log_level_color[_FMX10_LOG_LEVEL_PANIC + 1];
__EXPORT extern void fmx10_backtrace(void);
__EXPORT void fmx10_log_modulename(int level, const char *moduleName, const char *fmt, ...);

__END_DECLS

#define FMX10_BACKTRACE() fmx10_backtrace()

/****************************************************************************
 * Implementation of log section formatting based on printf
 *
 * To write to a specific stream for each message type, open the streams and
 * set __fmx10__log_startline to something like:
 * 	printf(_fmx10_fd[level],
 *
 * Additional behavior can be added using "{\" for __fmx10__log_startline and
 * "}" for __fmx10__log_endline and any other required setup or teardown steps
 ****************************************************************************/
#define __fmx10__log_printcond(cond, ...)	    if (cond) printf(__VA_ARGS__)
#define __fmx10__log_printline(level, ...)    printf(__VA_ARGS__)


#ifndef MODULE_NAME
#define MODULE_NAME "Unknown"
#endif

#define __fmx10__log_timestamp_fmt	"%-10" PRIu64 " "
#define __fmx10__log_timestamp_arg 	,hrt_absolute_time()
#define __fmx10__log_level_fmt		"%-5s "
#define __fmx10__log_level_arg(level)	,__fmx10_log_level_str[level]
#define __fmx10__log_thread_fmt		"%#X "
#define __fmx10__log_thread_arg		,(unsigned int)pthread_self()
#define __fmx10__log_modulename_fmt	"%-10s "
#define __fmx10__log_modulename_pfmt	"[%s] "
#define __fmx10__log_modulename_arg	,"[" MODULE_NAME "]"

#define __fmx10__log_file_and_line_fmt 	" (file %s line %u)"
#define __fmx10__log_file_and_line_arg 	, __FILE__, __LINE__
#define __fmx10__log_end_fmt 		"\n"

#define FMX10_ANSI_COLOR_RED     "\x1b[31m"
#define FMX10_ANSI_COLOR_GREEN   "\x1b[32m"
#define FMX10_ANSI_COLOR_YELLOW  "\x1b[33m"
#define FMX10_ANSI_COLOR_BLUE    "\x1b[34m"
#define FMX10_ANSI_COLOR_MAGENTA "\x1b[35m"
#define FMX10_ANSI_COLOR_CYAN    "\x1b[36m"
#define FMX10_ANSI_COLOR_GRAY    "\x1B[37m"
#define FMX10_ANSI_COLOR_RESET   "\x1b[0m"

#ifdef __FMX10_POSIX
#define FMX10_LOG_COLORIZED_OUTPUT //if defined and output is a tty, colorize the output according to the log level
#endif /* __FMX10_POSIX */


#ifdef FMX10_LOG_COLORIZED_OUTPUT
#include <unistd.h>
#define FMX10_LOG_COLOR_START \
	int use_color = isatty(STDOUT_FILENO); \
	if (use_color) printf("%s", __fmx10_log_level_color[level]);
#define FMX10_LOG_COLOR_MODULE \
	if (use_color) printf(FMX10_ANSI_COLOR_GRAY);
#define FMX10_LOG_COLOR_MESSAGE \
	if (use_color) printf("%s", __fmx10_log_level_color[level]);
#define FMX10_LOG_COLOR_END \
	if (use_color) printf(FMX10_ANSI_COLOR_RESET);
#else
#define FMX10_LOG_COLOR_START
#define FMX10_LOG_COLOR_MODULE
#define FMX10_LOG_COLOR_MESSAGE
#define FMX10_LOG_COLOR_END
#endif /* FMX10_LOG_COLORIZED_OUTPUT */

/****************************************************************************
 * Output format macros
 * Use these to implement the code level macros below
 ****************************************************************************/


/****************************************************************************
 * __fmx10_log_named_cond:
 * Convert a message in the form:
 * 	FMX10_LOG_COND(__dbg_enabled, "val is %d", val);
 * to
 * 	printf("%-5s val is %d\n", "LOG", val);
 * if the first arg/condition is true.
 ****************************************************************************/
#define __fmx10_log_named_cond(name, cond, FMT, ...) \
	__fmx10__log_printcond(cond,\
			     "%s " \
			     FMT\
			     __fmx10__log_end_fmt \
			     ,name, ##__VA_ARGS__\
			    )

/****************************************************************************
 * __fmx10_log:
 * Convert a message in the form:
 * 	FMX10_WARN("val is %d", val);
 * to
 * 	printf("%-5s val is %d\n", __fmx10_log_level_str[3], val);
 ****************************************************************************/
#define __fmx10_log(level, FMT, ...) \
	__fmx10__log_printline(level,\
			     __fmx10__log_level_fmt \
			     FMT\
			     __fmx10__log_end_fmt \
			     __fmx10__log_level_arg(level), ##__VA_ARGS__\
			    )

/****************************************************************************
 * __fmx10_log_modulename:
 * Convert a message in the form:
 * 	FMX10_WARN("val is %d", val);
 * to
 * 	printf("%-5s [%s] val is %d\n", __fmx10_log_level_str[3],
 *		MODULENAME, val);
 ****************************************************************************/

#define __fmx10_log_modulename(level, fmt, ...) \
	do { \
		fmx10_log_modulename(level, MODULE_NAME, fmt, ##__VA_ARGS__); \
	} while(0)
/****************************************************************************
 * __fmx10_log_timestamp:
 * Convert a message in the form:
 * 	FMX10_WARN("val is %d", val);
 * to
 * 	printf("%-5s %10lu val is %d\n", __fmx10_log_level_str[3],
 *		hrt_absolute_time(), val);
 ****************************************************************************/
#define __fmx10_log_timestamp(level, FMT, ...) \
	__fmx10__log_printline(level,\
			     __fmx10__log_level_fmt\
			     __fmx10__log_timestamp_fmt\
			     FMT\
			     __fmx10__log_end_fmt\
			     __fmx10__log_level_arg(level)\
			     __fmx10__log_timestamp_arg\
			     , ##__VA_ARGS__\
			    )

/****************************************************************************
 * __fmx10_log_timestamp_thread:
 * Convert a message in the form:
 * 	FMX10_WARN("val is %d", val);
 * to
 * 	printf("%-5s %10lu %#X val is %d\n", __fmx10_log_level_str[3],
 *		hrt_absolute_time(), pthread_self(), val);
 ****************************************************************************/
#define __fmx10_log_timestamp_thread(level, FMT, ...) \
	__fmx10__log_printline(level,\
			     __fmx10__log_level_fmt\
			     __fmx10__log_timestamp_fmt\
			     __fmx10__log_thread_fmt\
			     FMT\
			     __fmx10__log_end_fmt\
			     __fmx10__log_level_arg(level)\
			     __fmx10__log_timestamp_arg\
			     __fmx10__log_thread_arg\
			     , ##__VA_ARGS__\
			    )

/****************************************************************************
 * __fmx10_log_file_and_line:
 * Convert a message in the form:
 * 	FMX10_WARN("val is %d", val);
 * to
 * 	printf("%-5s val is %d (file %s line %u)\n",
 *		__fmx10_log_level_str[3], val, __FILE__, __LINE__);
 ****************************************************************************/
#define __fmx10_log_file_and_line(level, FMT, ...) \
	__fmx10__log_printline(level,\
			     __fmx10__log_level_fmt\
			     __fmx10__log_timestamp_fmt\
			     FMT\
			     __fmx10__log_file_and_line_fmt\
			     __fmx10__log_end_fmt\
			     __fmx10__log_level_arg(level)\
			     __fmx10__log_timestamp_arg\
			     , ##__VA_ARGS__\
			     __fmx10__log_file_and_line_arg\
			    )

/****************************************************************************
 * __fmx10_log_timestamp_file_and_line:
 * Convert a message in the form:
 * 	FMX10_WARN("val is %d", val);
 * to
 * 	printf("%-5s %-10lu val is %d (file %s line %u)\n",
 *		__fmx10_log_level_str[3], hrt_absolute_time(),
 *		val, __FILE__, __LINE__);
 ****************************************************************************/
#define __fmx10_log_timestamp_file_and_line(level, FMT, ...) \
	__fmx10__log_printline(level,\
			     __fmx10__log_level_fmt\
			     __fmx10__log_timestamp_fmt\
			     FMT\
			     __fmx10__log_file_and_line_fmt\
			     __fmx10__log_end_fmt\
			     __fmx10__log_level_arg(level)\
			     __fmx10__log_timestamp_arg\
			     , ##__VA_ARGS__\
			     __fmx10__log_file_and_line_arg\
			    )

/****************************************************************************
 * __fmx10_log_thread_file_and_line:
 * Convert a message in the form:
 * 	FMX10_WARN("val is %d", val);
 * to
 * 	printf("%-5s %#X val is %d (file %s line %u)\n",
 *		__fmx10_log_level_str[3], pthread_self(),
 *		val, __FILE__, __LINE__);
 ****************************************************************************/
#define __fmx10_log_thread_file_and_line(level, FMT, ...) \
	__fmx10__log_printline(level,\
			     __fmx10__log_level_fmt\
			     __fmx10__log_thread_fmt\
			     FMT\
			     __fmx10__log_file_and_line_fmt\
			     __fmx10__log_end_fmt\
			     __fmx10__log_level_arg(level)\
			     __fmx10__log_thread_arg\
			     , ##__VA_ARGS__\
			     __fmx10__log_file_and_line_arg\
			    )

/****************************************************************************
 * __fmx10_log_timestamp_thread_file_and_line:
 * Convert a message in the form:
 * 	FMX10_WARN("val is %d", val);
 * to
 * 	printf("%-5s %-10lu %#X val is %d (file %s line %u)\n",
 *		__fmx10_log_level_str[3], hrt_absolute_time(),
 *		pthread_self(), val, __FILE__, __LINE__);
 ****************************************************************************/
#define __fmx10_log_timestamp_thread_file_and_line(level, FMT, ...) \
	__fmx10__log_printline(level,\
			     __fmx10__log_level_fmt\
			     __fmx10__log_timestamp_fmt\
			     __fmx10__log_thread_fmt\
			     FMT\
			     __fmx10__log_file_and_line_fmt\
			     __fmx10__log_end_fmt\
			     __fmx10__log_level_arg(level)\
			     __fmx10__log_timestamp_arg\
			     __fmx10__log_thread_arg\
			     , ##__VA_ARGS__\
			     __fmx10__log_file_and_line_arg\
			    )


/****************************************************************************
 * Code level macros
 * These are the log APIs that should be used by the code
 ****************************************************************************/

/****************************************************************************
 * Messages that should never be filtered or compiled out
 ****************************************************************************/
#define FMX10_LOG(FMT, ...) 	__fmx10_log(_FMX10_LOG_LEVEL_ALWAYS, FMT, ##__VA_ARGS__)
#define FMX10_INFO(FMT, ...) 	__fmx10_log_modulename(_FMX10_LOG_LEVEL_ALWAYS, FMT, ##__VA_ARGS__)

#if defined(TRACE_BUILD)
/****************************************************************************
 * Extremely Verbose settings for a Trace build
 ****************************************************************************/
#define FMX10_PANIC(FMT, ...)	__fmx10_log_timestamp_thread_file_and_line(_FMX10_LOG_LEVEL_PANIC, FMT, ##__VA_ARGS__)
#define FMX10_ERR(FMT, ...)	__fmx10_log_timestamp_thread_file_and_line(_FMX10_LOG_LEVEL_ERROR, FMT, ##__VA_ARGS__)
#define FMX10_WARN(FMT, ...) 	__fmx10_log_timestamp_thread_file_and_line(_FMX10_LOG_LEVEL_WARN,  FMT, ##__VA_ARGS__)
#define FMX10_DEBUG(FMT, ...) 	__fmx10_log_timestamp_thread(_FMX10_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)

#elif defined(DEBUG_BUILD)
/****************************************************************************
 * Verbose settings for a Debug build
 ****************************************************************************/
#define FMX10_PANIC(FMT, ...)	__fmx10_log_timestamp_file_and_line(_FMX10_LOG_LEVEL_PANIC, FMT, ##__VA_ARGS__)
#define FMX10_ERR(FMT, ...)	__fmx10_log_timestamp_file_and_line(_FMX10_LOG_LEVEL_ERROR, FMT, ##__VA_ARGS__)
#define FMX10_WARN(FMT, ...) 	__fmx10_log_timestamp_file_and_line(_FMX10_LOG_LEVEL_WARN,  FMT, ##__VA_ARGS__)
#define FMX10_DEBUG(FMT, ...) 	__fmx10_log_timestamp(_FMX10_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)

#elif defined(RELEASE_BUILD)
/****************************************************************************
 * Non-verbose settings for a Release build to minimize strings in build
 ****************************************************************************/
#define FMX10_PANIC(FMT, ...)	__fmx10_log_modulename(_FMX10_LOG_LEVEL_PANIC, FMT, ##__VA_ARGS__)
#define FMX10_ERR(FMT, ...)	__fmx10_log_modulename(_FMX10_LOG_LEVEL_ERROR, FMT, ##__VA_ARGS__)
#define FMX10_WARN(FMT, ...) 	__fmx10_log_omit(_FMX10_LOG_LEVEL_WARN,  FMT, ##__VA_ARGS__)
#define FMX10_DEBUG(FMT, ...) 	__fmx10_log_omit(_FMX10_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)

#else
/****************************************************************************
 * Medium verbose settings for a default build
 ****************************************************************************/
#define FMX10_PANIC(FMT, ...)	__fmx10_log_modulename(_FMX10_LOG_LEVEL_PANIC, FMT, ##__VA_ARGS__)
#define FMX10_ERR(FMT, ...)	__fmx10_log_modulename(_FMX10_LOG_LEVEL_ERROR, FMT, ##__VA_ARGS__)
#define FMX10_WARN(FMT, ...) 	__fmx10_log_modulename(_FMX10_LOG_LEVEL_WARN,  FMT, ##__VA_ARGS__)
#define FMX10_DEBUG(FMT, ...) 	__fmx10_log_omit(_FMX10_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)

#endif
#define FMX10_LOG_NAMED(name, FMT, ...) 	__fmx10_log_named_cond(name, true, FMT, ##__VA_ARGS__)
#define FMX10_LOG_NAMED_COND(name, cond, FMT, ...) __fmx10_log_named_cond(name, cond, FMT, ##__VA_ARGS__)
#endif
