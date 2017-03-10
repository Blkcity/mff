cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(FMX10_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(FMX10_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")


set(CMAKE_BUILD_TYPE "" CACHE STRING "build type")
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
        STRINGS ";Debug;Release;RelWithDebInfo;MinSizeRel")
set(CONFIG "posix_sitl_default" CACHE STRING "desired configuration")
file(GLOB_RECURSE configs RELATIVE cmake/configs "cmake/configs/*.cmake")
set_property(CACHE CONFIG PROPERTY STRINGS ${configs})
set(THREADS "4" CACHE STRING
        "number of threads to use for external build processes")
set(DEBUG_PORT "/dev/ttyACM0" CACHE STRING "debugging port")
set(EXTERNAL_MODULES_LOCATION "" CACHE STRING "External modules source location")

set(nuttx_configure "$ENV{NUTTX_CONFIGURE}")
message(STATUS "nuttx make config flag:${nuttx_configure}")


string(REPLACE "_" ";" config_args ${CONFIG})
list(GET config_args 0 OS)
list(GET config_args 1 BOARD)
list(GET config_args 2 LABEL)
set(target_name "${OS}-${BOARD}-${LABEL}")

message(STATUS "${target_name}")

#  The Url for the elf file for crash logging

if (DEFINED ENV{BUILD_URI})
        set (BUILD_URI $ENV{BUILD_URI})
else()
        set (BUILD_URI "localhost")
endif()

add_definitions(-DBUILD_URI=${BUILD_URI})

include(GNUInstallDirs)

include(ExternalProject)


list(APPEND CMAKE_MODULE_PATH "${FMX10_SOURCE_DIR}/cmake")
message(STATUS "cmake module path: ${CMAKE_MODULE_PATH},CONFIG:${CONFIG}")
set(config_module "configs/${CONFIG}")
include(${config_module})


# require fmx10 module interface
set(fmx10_required_interface
	fmx10_os_prebuild_targets
	fmx10_os_add_flags
	)
foreach(cmd ${fmx10_required_interface})
	if(NOT COMMAND ${cmd})
		message(FATAL_ERROR "${config_module} must implement ${cmd}")
	endif()
endforeach()
set(fmx10_required_config
        config_module_list
        )
foreach(conf ${fmx10_required_config})
        if(NOT DEFINED ${conf})
                message(FATAL_ERROR "cmake/${config_module} must define ${conf}")
        endif()
endforeach()


project(fmx10 CXX C ASM)
if (NOT ${CMAKE_VERSION} VERSION_LESS 3.0.0)
        cmake_policy(SET CMP0045 NEW) # error on non-existent target in get prop
        cmake_policy(SET CMP0046 NEW) # no non-existent targets as dependencies
        cmake_policy(SET CMP0025 OLD) # still report AppleClang as Clang
endif()
if (NOT ${CMAKE_VERSION} VERSION_LESS 3.1.0)
        cmake_policy(SET CMP0054 NEW) # don't dereference quoted variables
endif()

set(version_major 1)
set(version_minor 5)
set(version_patch 0)
set(version "${version_major}.${version_minor}.${version_patch}")
set(package-contact "jlb@fimi.cn")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


#=============================================================================
# check required toolchain variables
#
set(required_variables
	CMAKE_C_COMPILER_ID
	)
foreach(var ${required_variables})
	if (NOT ${var})
		message(FATAL_ERROR "Toolchain/config must define ${var}")
	endif()
endforeach()
#=============================================================================
# external libraries
#
fmx10_os_prebuild_targets(OUT prebuild_targets
        BOARD ${BOARD}
        THREADS ${THREADS})

#=============================================================================
# build flags
#
fmx10_os_add_flags(
        BOARD ${BOARD}
        C_FLAGS c_flags
        CXX_FLAGS cxx_flags
        OPTIMIZATION_FLAGS optimization_flags
        EXE_LINKER_FLAGS exe_linker_flags
        INCLUDE_DIRS include_dirs
        LINK_DIRS link_dirs
        DEFINITIONS definitions)

fmx10_join(OUT CMAKE_EXE_LINKER_FLAGS LIST "${exe_linker_flags}" GLUE " ")
fmx10_join(OUT CMAKE_C_FLAGS LIST "${c_flags};${optimization_flags}" GLUE " ")
fmx10_join(OUT CMAKE_CXX_FLAGS LIST "${cxx_flags};${optimization_flags}" GLUE " ")

include_directories(${include_dirs})
#message("INCLUDE_DIRS=${include_dirs}")
link_directories(${link_dirs})
add_definitions(${definitions})

#=============================================================================
# source code generation
#

add_subdirectory(msg)
fmx10_generate_messages(TARGET msg_gen
    MSG_FILES ${msg_files}
    OS ${OS}
    #    DEPENDS git_genmsg git_gencpp prebuild_targets
    DEPENDS prebuild_targets
    )
fmx10_generate_parameters_xml(OUT parameters.xml BOARD ${BOARD})
fmx10_generate_airframes_xml(OUT airframes.xml BOARD ${BOARD})
add_custom_target(xml_gen
    DEPENDS parameters.xml airframes.xml)

#=============================================================================
# external projects
#
#=============================================================================
# subdirectories
#
set(module_libraries)
foreach(module ${config_module_list})
        string(REGEX MATCH "^[./]" external_module ${module})
        if(external_module)
                STRING(REGEX REPLACE "//" "/" EXT_MODULE ${module})
                STRING(REGEX REPLACE "/" "__" EXT_MODULE_PREFIX ${EXT_MODULE})
                add_subdirectory(${module} ${FMX10_BINARY_DIR}/${EXT_MODULE_PREFIX})
        else()
                add_subdirectory(src/${module})
        endif()
        fmx10_mangle_name(${module} mangled_name)
        list(APPEND module_libraries ${mangled_name})
        #message(STATUS "adding module: ${module}")
endforeach()
#=============================================================================
# DriverFramework Drivers
#

#execute_process(COMMAND /setenv.sh WORKING_DIRECTORY ${NUTTX_SOURCE_DIR})
#execute_process(COMMAND make WORKING_DIRECTORY ${NUTTX_SOURCE_DIR})
#message(STATUS "Nuttx build for ${BOARD} on ${HWCLASS} hardware, using ${CONFIG} ${HASROMFS}")

add_subdirectory(src/firmware/${OS})
if (NOT "${OS}" STREQUAL "nuttx")
endif()
