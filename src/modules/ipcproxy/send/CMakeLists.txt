fmx10_add_module(
	MODULE modules__ipcproxy__send
	MAIN send
	STACK_MAIN 1024
	COMPILE_FLAGS
	SRCS
		ipcsend.c
	DEPENDS
		modules__ipcproxy
	)
