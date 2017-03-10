#!/usr/bin/bash

CMAKE_VER := $(shell Tools/check_cmake.sh; echo $$?)
ifneq ($(CMAKE_VER),0)
    $(warning Not a valid CMake version or CMake not installed.)
    $(warning On Ubuntu 16.04, install or upgrade via:)
    $(warning )
    $(warning 3rd party PPA:)
    $(warning sudo add-apt-repository ppa:george-edison55/cmake-3.x -y)
    $(warning sudo apt-get update)
    $(warning sudo apt-get install cmake)
    $(warning )
    $(warning Official website:)
    $(warning wget https://cmake.org/files/v3.4/cmake-3.4.3-Linux-x86_64.sh)
    $(warning chmod +x cmake-3.4.3-Linux-x86_64.sh)
    $(warning sudo mkdir /opt/cmake-3.4.3)
    $(warning sudo ./cmake-3.4.3-Linux-x86_64.sh --prefix=/opt/cmake-3.4.3 --exclude-subdir)
    $(warning export PATH=/opt/cmake-3.4.3/bin:$$PATH)
    $(warning )
    $(error Fatal)
endif

ifdef SYSTEMROOT
    # Windows
    FM_CMAKE_GENERATOR ?= "MSYS Makefiles"
else
    FM_CMAKE_GENERATOR ?= "Unix Makefiles"
endif
FM_MAKE = $(MAKE)
FM_MAKE_ARGS = -j$(j) --no-print-directory

ifdef replay
    BUILD_DIR_SUFFIX := _replay
else
    BUILD_DIR_SUFFIX :=
endif

CMAKE_ARGS :=

SRC_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

define cmake-build
+@$(eval BUILD_DIR = $(SRC_DIR)/build_$@$(BUILD_DIR_SUFFIX))
+@if [ ! -e $(BUILD_DIR)/CMakeCache.txt ]; then mkdir -p $(BUILD_DIR) && cd $(BUILD_DIR) && cmake .. -G$(FM_CMAKE_GENERATOR) -DCONFIG=$(1) $(CMAKE_ARGS) || (cd .. && rm -rf $(BUILD_DIR)); fi
+@echo "FMX10 CONFIG: $(BUILD_DIR) $(FM_MAKE) $(BUILD_DIR) $(FM_MAKE_ARGS) $(ARGS)"
+@$(FM_MAKE) -C "$(BUILD_DIR)" $(FM_MAKE_ARGS) $(ARGS)
endef

define colorecho
	@tput setaf 6
	@echo $1
	@tput sgr0
endef

ALL_CONFIG_TARGETS := $(basename $(shell find "$(SRC_DIR)/cmake/configs" -name '*.cmake' -print | sed  -e 's:^.*/::' | sort))

NUTTX_CONFIG_TARGETS := $(patsubst nuttx_%,%,$(filter nuttx_%,$(ALL_CONFIG_TARGETS)))


$(ALL_CONFIG_TARGETS):
	$(call cmake-build,$@)

$(NUTTX_CONFIG_TARGETS):
	$(call cmake-build,nuttx_$@)

all_nuttx_targets: $(NUTTX_CONFIG_TARGETS)

.PHONY: all_nuttx_targets


clean:
	@rm -rf build_*/
	-@$(MAKE) -C NuttX/nuttx clean

%:
	$(if $(filter $(FIRST_ARG),$@), \
		$(error "$@ cannot be the first argument. Use '$(MAKE) help|list_config_targets' to get a list of all possible [configuration] targets."),@#)


.PHONY: clean

CONFIGS:=$(shell ls cmake/configs | sed -e "s~.*/~~" | sed -e "s~\..*~~")

help:
	@echo "Usage: $(MAKE) <target>"
	@echo "Where <target> is one of:"
	@$(MAKE) -pRrq -f $(lastword $(MAKEFILE_LIST)) : 2>/dev/null | \
		awk -v RS= -F: '/^# File/,/^# Finished Make data base/ {if ($$1 !~ "^[#.]") {print $$1}}' | sort | \
		egrep -v -e '^[^[:alnum:]]' -e '^($(subst $(space),|,$(ALL_CONFIG_TARGETS) $(NUTTX_CONFIG_TARGETS)))$$' -e '_default$$' -e '^(posix|eagle|Makefile)'
	@echo
	@echo "Or, $(MAKE) <config_target> [<make_target(s)>]"
	@echo "Use '$(MAKE) list_config_targets' for a list of configuration targets."

empty :=
space := $(empty) $(empty)

list_config_targets:
	@for targ in $(patsubst nuttx_%,[nuttx_]%,$(ALL_CONFIG_TARGETS)); do echo $$targ; done



