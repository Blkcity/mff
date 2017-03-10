fmx10_add_module(

    MODULE modules__ipcproxy
    STACK_MAX	1200
    COMPILE_FLAGS
	-Wno-sign-compare
    SRCS
	ipcinterface.c
	ipcproxy.cxx
    DEPENDS
	platforms__common
    )
