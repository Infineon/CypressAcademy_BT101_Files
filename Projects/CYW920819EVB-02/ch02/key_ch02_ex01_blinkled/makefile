#
# Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
# Cypress Semiconductor Corporation. All Rights Reserved.
#
# This software, including source code, documentation and related
# materials ("Software"), is owned by Cypress Semiconductor Corporation
# or one of its subsidiaries ("Cypress") and is protected by and subject to
# worldwide patent protection (United States and foreign),
# United States copyright laws and international treaty provisions.
# Therefore, you may use this Software only as provided in the license
# agreement accompanying the software package from which you
# obtained this Software ("EULA").
# If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
# non-transferable license to copy, modify, and compile the Software
# source code solely for use in connection with Cypress's
# integrated circuit products. Any reproduction, modification, translation,
# compilation, or representation of this Software except as specified
# above is prohibited without the express written permission of Cypress.
#
# Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
# reserves the right to make changes to the Software without notice. Cypress
# does not assume any liability arising out of the application or use of the
# Software or any product or circuit described in the Software. Cypress does
# not authorize its products for use in any products where a malfunction or
# failure of the Cypress product may reasonably be expected to result in
# significant property damage, injury or death ("High Risk Product"). By
# including Cypress's product in a High Risk Product, the manufacturer
# of such system or application assumes all risk of such use and in doing
# so agrees to indemnify Cypress against all liability.
#

ifeq ($(WHICHFILE),true)
$(info Processing $(lastword $(MAKEFILE_LIST)))
endif

#
# Basic Configuration
#
APPNAME=app
TOOLCHAIN=GCC_ARM
CONFIG=Debug
VERBOSE=

# default target
TARGET=CYW920819EVB-02
SUPPORTED_TARGETS = \
  CYW920819EVB-02 \
  CYW920820EVB-02 \
  CYBT-213043-EVAL \
  CYBT-213043-MESH \
  CYW989820EVB-01 \
  CYW920721B2EVK-01 \
  CYW920721B2EVK-03 \
  CYW920719B2Q40EVB-01

TARGET_DEVICE_MAP = \
  CYW920819EVB-02/20819A1 \
  CYW920820EVB-02/20820A1 \
  CYBT-213043-EVAL/20819A1 \
  CYBT-213043-MESH/20819A1 \
  CYW989820EVB-01/20820A1 \
  CYW920721B2EVK-01/20721B2 \
  CYW920721B2EVK-03/20721B2 \
  CYW920719B2Q40EVB-01/20719B2

CY_TARGET_DEVICE = $(patsubst $(TARGET)/%,%,$(filter $(TARGET)%,$(TARGET_DEVICE_MAP)))

ifeq ($(filter $(TARGET),$(SUPPORTED_TARGETS)),)
$(error TARGET $(TARGET) not supported for this code example)
endif

#
# Advanced Configuration
#
SOURCES=
INCLUDES=\
    $(CY_BASELIB_PATH)/include \
    $(CY_BASELIB_PATH)/include/hal \
    $(CY_BASELIB_PATH)/include/internal \
    $(CY_BASELIB_PATH)/include/stack \
    $(CY_BASELIB_PATH)/internal/$(CY_TARGET_DEVICE) \
    $(CY_SHARED_PATH)/dev-kit/btsdk-include \
    $(CY_BSP_PATH)
DEFINES=
VFP_SELECT=
CFLAGS=
CXXFLAGS=
ASFLAGS=
LDFLAGS=
LDLIBS=
LINKER_SCRIPT=
PREBUILD=
POSTBUILD=
FEATURES=

#
# App features/defaults
#
OTA_FW_UPGRADE?=0
BT_DEVICE_ADDRESS?=default
UART?=AUTO
XIP?=xip
TRANSPORT?=UART
ENABLE_DEBUG?=0

# wait for SWD attach
ifeq ($(ENABLE_DEBUG),1)
CY_APP_DEFINES+=-DENABLE_DEBUG=1
endif

CY_APP_DEFINES+=\
    -DWICED_BT_TRACE_ENABLE

#
# Components (middleware libraries)
#
COMPONENTS += bsp_design_modus

################################################################################
# Paths
################################################################################

# Path (absolute or relative) to the project
CY_APP_PATH=.

# Path (absolute or relative) to the bt-sdk folder (at repo root)
CY_SHARED_PATH=$(CY_APP_PATH)/../wiced_btsdk

# Path (absolute or relative) to the base library
CY_BASELIB_PATH=$(CY_SHARED_PATH)/dev-kit/baselib/$(CY_TARGET_DEVICE)

# Path to the bsp library
CY_BSP_PATH=$(CY_SHARED_PATH)/dev-kit/bsp/TARGET_$(TARGET)

INCLUDES+=\
    $(CY_BASELIB_PATH)/WICED/common

CY_DEVICESUPPORT_PATH=$(CY_BASELIB_PATH)

# Absolute path to the compiler (Default: GCC in the tools)
CY_COMPILER_PATH=

# Locate ModusToolbox IDE helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_* \
    $(CY_IDE_TOOLS_DIR))

# If you install ModusToolbox IDE in a custom location, add the path to its
# "tools_X.Y" folder (where X and Y are the version number of the tools
# folder).
CY_TOOLS_PATHS+=

# Default to the newest installed tools folder, or the users override (if it's
# found).
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS))
endif

# path to WICED tools root folder
CY_WICED_TOOLS_ROOT?=$(CY_SHARED_PATH)/dev-kit/btsdk-tools

# tools that can be launched with "make open CY_OPEN_TYPE=<tool>
CY_BT_APP_TOOLS=

# getlibs path
CY_GETLIBS_PATH=.

# paths to shared_libs targets
SEARCH_LIBS_AND_INCLUDES=$(CY_BSP_PATH) $(CY_BASELIB_PATH)

# common make arguments for lib builds
MAKE_OVERRIDES=OTA_FW_UPGRADE=$(OTA_FW_UPGRADE)
export CONFIG
export COMPONENTS
export DISABLE_COMPONENTS
CY_SHARED_LIB_ARGS:=$(MAKECMDGOALS) $(MAKE_OVERRIDES) CY_APP_DEFINES+="$(CY_APP_DEFINES)" CY_TARGET_DEVICE=$(CY_TARGET_DEVICE) TARGET=$(TARGET)

.PHONY: $(SEARCH_LIBS_AND_INCLUDES) shared_libs

# recursive make on each shared_lib project
shared_libs: $(SEARCH_LIBS_AND_INCLUDES)
$(SEARCH_LIBS_AND_INCLUDES):
	$(MAKE) -C $@ $(CY_SHARED_LIB_ARGS)

CY_APP_LOCATION=$(lastword $(MAKEFILE_LIST))

CY_APP_BUILD_GOALS:=build qbuild clean program qprogram debug qdebug all
-include internal.mk
ifeq ($(filter setup teardown,$(MAKECMDGOALS)),)

# ensure pre-requisite wiced_btsdk project is present
ifeq ($(wildcard $(CY_SHARED_PATH)),)
# if not present, IDE will be trying to run make eclipse to get launch configuration
# need to store in that case and re-run later after the wiced_btsdk project exists
ifeq ($(MAKECMDGOALS),eclipse)
$(shell echo $(CY_IDE_PRJNAME) > ./.cy_ide_prjname)
endif
ifneq ($(filter eclipse $(CY_APP_BUILD_GOALS),$(MAKECMDGOALS)),)
$(warning BTSDK application projects require the wiced_btsdk project as a prerequisite.)
$(warning Please create the wiced_btsdk project via New Application wizard in the IDE or via git clone for CLI.)
$(error Missing prerequisite wiced_btsdk project)
endif
endif

# ensure that the wiced_btsdk project has performed getlibs to populate the assets (CLI only)
ifneq ($(filter $(CY_APP_BUILD_GOALS),$(MAKECMDGOALS)),)
ifeq ($(wildcard $(CY_BASELIB_PATH)),)
$(warning Prerequisite baselib path $(CY_BASELIB_PATH) not found under wiced_btsdk project.)
$(warning Please perform 'make getlibs' in the $(CY_SHARED_PATH) folder.)
$(error Missing prerequisite $(CY_BASELIB_PATH))
endif
endif

# all prerequisites satisfied, check for .cy_ide_prjname if stored earlier
ifeq ($(CY_MAKE_IDE),eclipse)
ifneq ($(filter $(CY_APP_BUILD_GOALS),$(MAKECMDGOALS)),)
ifneq ($(wildcard ./.cy_ide_prjname),)
CY_IDE_PRJNAME=`cat ./.cy_ide_prjname`
$(MAKECMDGOALS): regenerate_launches
regenerate_launches:
	@make -C . eclipse CY_IDE_PRJNAME=$(CY_IDE_PRJNAME)
	@rm ./.cy_ide_prjname
endif
endif
endif
include $(CY_TOOLS_DIR)/make/start.mk
endif
