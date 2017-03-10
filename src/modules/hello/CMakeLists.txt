fmx10_add_module(
        MODULE modules__hello
        MAIN helloxx
	STACK_MAIN 2048
	COMPILE_FLAGS
        SRCS
                helloxx_main.cpp
        DEPENDS
		platforms__common
        )
