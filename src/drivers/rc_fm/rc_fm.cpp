#include <fmx10_config.h>

#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>
//#include <dev_fs_lib_serial.h>
#include <fmx10_tasks.h>
#include <fmx10_posix.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>


#define RC_BAUD	921600

extern "C" {__EXPORT int rc_fm_main(int argc, char *argv[]); }

namespace fm_rc
{
volatile bool _task_should_exit = false;
static bool _is_running = false;
static fmx10_task_t _task_handle = -1;
static int uart_fd; 

int start(void);
int stop(void);
int info(void);
void usage(void);
void task_main(int argc, char *argv[]);
int rc_dev_init(void);

int rc_dev_init(void)
{
   int baud_speed = B921600;

   uart_fd = open(RC_SERIAL_PORT, O_RDWR | O_NOCTTY);

   if(uart_fd < 0)
   {
	FMX10_ERR("RC: failed to open serial port: %s err: %d",RC_SERIAL_PORT,errno);
	return -1;
   }

   switch(RC_BAUD)
   {
	case 9600:
	    baud_speed = B9600;
	    break;
	case 19200:
	    baud_speed = B19200;
	    break;
	case 38400:
	    baud_speed = B38400;
	    break;
	case 57600:
	    baud_speed = B57600;
	    break;
	case 115200:
	    baud_speed = B115200;
	    break;
	case 230400:
	    baud_speed = B230400;
	    break;
	case 256000:
	    baud_speed = B256000;
	    break;
	case 460800:
	    baud_speed = B460800;
	    break;
	case 500000:
	    baud_speed = B500000;
	    break;
	case 576000:
	    baud_speed = B576000;
	    break;
	case 921600:
	    baud_speed = B921600;
	    break;
   }
   struct termios uart_config;
   int termios_state;

   tcgetattr(uart_fd, &uart_config);

   uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

   uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OLCUC | OPOST);

   uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

   uart_config.c_cflag &= ~(CSTOPB | PARENB);

   if ((termios_state = cfsetispeed(&uart_config, baud_speed)) < 0)
   {
	FMX10_ERR("ERR: %d (cfsetispeed)",termios_state);
	return -1;
   }
   if ((termios_state = cfsetospeed(&uart_config, baud_speed)) < 0)
   {
	FMX10_ERR("ERR: %d (cfsetospeed)",termios_state);
	return -1;
   }
   if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0)
   {
	FMX10_ERR("ERR: %d (tcsetattr)",termios_state);
	return -1;
   }

   return 0;
}

void task_main(int argc, char *argv[])
{
   if(rc_dev_init() < 0)
   {
	FMX10_ERR("rc uart init failed!");
	return;
   }
   
    _is_running = true;
   uint8_t rx_buf[64];

   while(!_task_should_exit)
   {
	int newbytes = read(uart_fd, &rx_buf[0], sizeof(rx_buf));
	if(newbytes == 0)
	    FMX10_INFO("no date");
/*
	printf("heart: ");
	for(i = 0;i<newbytes;i++)
	{
	    printf("0x%02x ",rx_buf[i]);
	}
	printf("\n");
*/
   }
    _is_running = false;
}

int start(void)
{
    if(_is_running)
    {
	FMX10_WARN("already running!");
	return -1;
    }

    ASSERT(_task_handle == -1);
    _task_should_exit = false;

    _task_handle = fmx10_task_spawn_cmd("fm_rc_main",
					SCHED_DEFAULT,
					SCHED_PRIORITY_DEFAULT,
					2000,
					(fmx10_main_t)&task_main,
					nullptr);
    if(_task_handle < 0)
    {
	FMX10_ERR("task start failed!");
	return -1;
    }

    return 0;
}

int stop(void)
{
    if(!_is_running)
    {
	FMX10_WARN("already running!");
	return -1;
    }
    _task_should_exit = true;
    
    while(_is_running)
    {
	usleep(200000);
	FMX10_INFO(".");
    }

    _task_handle = -1;

    return 0;
}

int info (void)
{
    FMX10_INFO("running: %s",_is_running ? "yes" : "no");

    return 0;
}

void usage(void)
{
    FMX10_INFO("Usage: rc_fm {start|info|stop}");
}
}

int rc_fm_main(int argc, char *argv[])
{
    int myoptind = 1;
    if(argc <= 1)
    {
	fm_rc::usage();
	return 1;	
    }

    const char *verb = argv[myoptind];

    if(!strcmp(verb,"start"))
	return fm_rc::start();
    else if (!strcmp(verb,"stop"))
	return fm_rc::stop();
    else if (!strcmp(verb,"info"))
	return fm_rc::info();
    else
	fm_rc::usage();

    return 1;
}
