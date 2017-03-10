#include <stdlib.h>
#include <fmx10_log.h>
#ifdef __FMX10_POSIX
#include <execinfo.h>
#endif
#include <uORB/uORB.h>
#include <uORB/topics/log_message.h>
#include <drivers/drv_hrt.h>

static orb_advert_t orb_log_message_pub = NULL;

__EXPORT const char *__fmx10_log_level_str[_FMX10_LOG_LEVEL_PANIC + 1] = { "INFO", "DEBUG", "WARN", "ERROR", "PANIC" };
__EXPORT const char *__fmx10_log_level_color[_FMX10_LOG_LEVEL_PANIC + 1] =
{ FMX10_ANSI_COLOR_RESET, FMX10_ANSI_COLOR_GREEN, FMX10_ANSI_COLOR_YELLOW, FMX10_ANSI_COLOR_RED, FMX10_ANSI_COLOR_RED };


void fmx10_log_initialize(void)
{
	ASSERT(orb_log_message_pub == NULL);

	/* we need to advertise with a valid message */
	struct log_message_s log_message;
	log_message.timestamp = hrt_absolute_time();
	log_message.severity = 6; //info
	strcpy((char *)log_message.text, "initialized uORB logging");

	orb_log_message_pub = orb_advertise_queue(ORB_ID(log_message), &log_message, 2);

	if (!orb_log_message_pub) {
		FMX10_ERR("failed to advertise log_message");
	}
}

void fmx10_backtrace()
{
#ifdef __FMX10_POSIX
	void *buffer[10];
	char **callstack;
	int bt_size;
	int idx;

	bt_size = backtrace(buffer, 10);
	callstack = backtrace_symbols(buffer, bt_size);

	FMX10_INFO("Backtrace: %d", bt_size);

	for (idx = 0; idx < bt_size; idx++) {
		FMX10_INFO("%s", callstack[idx]);
	}

	free(callstack);
#endif
}

__EXPORT void fmx10_log_modulename(int level, const char *moduleName, const char *fmt, ...)
{
	FMX10_LOG_COLOR_START
	printf(__fmx10__log_level_fmt __fmx10__log_level_arg(level));
	FMX10_LOG_COLOR_MODULE
	printf(__fmx10__log_modulename_pfmt, moduleName);
	FMX10_LOG_COLOR_MESSAGE
	va_list argptr;
	va_start(argptr, fmt);
	vprintf(fmt, argptr);
	va_end(argptr);
	FMX10_LOG_COLOR_END
	printf("\n");

	/* publish an orb log message */
	if (level >= _FMX10_LOG_LEVEL_WARN && orb_log_message_pub) { //only publish important messages

		struct log_message_s log_message;
		const unsigned max_length = sizeof(log_message.text);
		log_message.timestamp = hrt_absolute_time();

		const uint8_t log_level_table[] = {
			6, /* _FMX10_LOG_LEVEL_ALWAYS */
			7, /* _FMX10_LOG_LEVEL_DEBUG */
			4, /* _FMX10_LOG_LEVEL_WARN */
			3, /* _FMX10_LOG_LEVEL_ERROR */
			0  /* _FMX10_LOG_LEVEL_PANIC */
		};
		log_message.severity = log_level_table[level];

		unsigned pos = 0;

		pos += snprintf((char *)log_message.text + pos, max_length - pos, __fmx10__log_modulename_pfmt, moduleName);
		va_start(argptr, fmt);
		pos += vsnprintf((char *)log_message.text + pos, max_length - pos, fmt, argptr);
		va_end(argptr);
		log_message.text[max_length - 1] = 0; //ensure 0-termination

		orb_publish(ORB_ID(log_message), orb_log_message_pub, &log_message);

	}
}

