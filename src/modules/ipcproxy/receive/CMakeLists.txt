fmx10_add_module(
	MODULE modules__ipcproxy__receive
	MAIN receive
	STACK_MAIN 1024
	COMPILE_FLAGS
	SRCS
		ipcreceive.c
	DEPENDS
		modules__ipcproxy
	)
