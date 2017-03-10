#pragma once

#include <sys/types.h>
#include <time.h>

#if defined(__FMX10_APPLE_LEGACY)

__BEGIN_DECLS

#define clockid_t unsigned

int fmx10_clock_gettime(clockid_t clk_id, struct timespec *tp);
int fmx10_clock_settime(clockid_t clk_id, struct timespec *tp);

__EXPORT unsigned int sleep(unsigned int sec);

__END_DECLS

#elif defined(__FMX10_LINUX) || defined(__FMX10_NUTTX) || defined(__FMX10_DARWIN)

#define fmx10_clock_gettime clock_gettime
#define fmx10_clock_settime clock_settime

#elif defined(__FMX10_QURT)

#include <sys/timespec.h>

__BEGIN_DECLS

int fmx10_clock_gettime(clockid_t clk_id, struct timespec *tp);
int fmx10_clock_settime(clockid_t clk_id, struct timespec *tp);

__EXPORT unsigned int sleep(unsigned int sec);

__END_DECLS
#endif
