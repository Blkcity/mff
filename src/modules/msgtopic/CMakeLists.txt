fmx10_add_module(
        MODULE modules__msgtopic
        MAIN msg
	STACK_MAIN 1024
	COMPILE_FLAGS
        SRCS
                msg_main.cpp
        DEPENDS
		platforms__common
        )
