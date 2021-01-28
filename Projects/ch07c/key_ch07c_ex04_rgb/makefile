#
# Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
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
APPNAME=BLE_Mesh_LightDimmable
TOOLCHAIN=GCC_ARM
CONFIG=Debug
VERBOSE=

# default target
TARGET=CYBT-213043-MESH

SUPPORTED_TARGETS = \
  CYW920819EVB-02 \
  CYBT-213043-MESH \
  CYW920820EVB-02 \
  CYBT-213043-EVAL \
  CYW920721B2EVK-03 \
  CYW920721B2EVK-02 \
  CYW920719B2Q40EVB-01 \
  CYW920706WCDEVAL \
  CYW920735Q60EVB-01

#
# Advanced Configuration
#
SOURCES=
INCLUDES=
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
OTA_FW_UPGRADE?=1
UART?=AUTO
XIP?=xip
TRANSPORT?=UART
ENABLE_DEBUG?=0

# wait for SWD attach
ifeq ($(ENABLE_DEBUG),1)
CY_APP_DEFINES+=-DENABLE_DEBUG=1
endif
REMOTE_PROVISION_SRV?=0

CY_APP_DEFINES+=\
    -DWICED_BT_TRACE_ENABLE \
    -DHCI_CONTROL

ifeq ($(REMOTE_PROVISION_SRV),1)
CY_APP_DEFINES += -DREMOTE_PROVISION_SERVER_SUPPORTED
endif

# value of the LOW_POWER_NODE defines mode. It can be normal node (0), or low power node (1)
LOW_POWER_NODE ?= 0
CY_APP_DEFINES += -DLOW_POWER_NODE=$(LOW_POWER_NODE)

# If PTS is defined then device gets hardcoded BD address from make target
# Otherwise it is random for all mesh apps.
# Do not try to use BT_DEVICE_ADDRESS unless testing with PTS=1
BT_DEVICE_ADDRESS?=default
PTS ?= 0
ifeq ($(PTS),1)
CY_APP_DEFINES += -DPTS
endif # PTS

# These flags control whether the prebuilt mesh libs (core, models)
# will be the trace enabled versions or not
MESH_MODELS_DEBUG_TRACES ?= 0
MESH_CORE_DEBUG_TRACES ?= 0

#
# Components (middleware libraries)
#
COMPONENTS +=bsp_design_modus
COMPONENTS += mesh_app_lib
ifeq ($(OTA_FW_UPGRADE),1)
COMPONENTS += fw_upgrade_lib
endif

# prebuilt libs - link release libs by default, or debug trace enabled if flag set
ifeq ($(MESH_CORE_DEBUG_TRACES),1)
COMPONENTS += mesh_core_lib_debug
else
COMPONENTS += mesh_core_lib
endif
ifeq ($(MESH_MODELS_DEBUG_TRACES),1)
COMPONENTS += mesh_models_lib_debug
else
COMPONENTS += mesh_models_lib
endif

# Chip-specific patch libs
CY_20706A2_APP_PATCH_LIBS += rtc_lib.a
CY_20706A2_APP_PATCH_LIBS += wiced_bt_mesh.a

################################################################################
# Paths
################################################################################

# Path (absolute or relative) to the project
CY_APP_PATH=.

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI><COMMIT><LOCATION>. If the <LOCATION> field
# begins with $$ASSET_REPO$$, then the repo is deposited in the path specified by
# the CY_GETLIBS_SHARED_PATH variable. The default location is one directory level
# above the current app directory.
# This is used with CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

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

# tools that can be launched with "make open CY_OPEN_TYPE=<tool>
CY_BT_APP_TOOLS=BTSpy ClientControlMesh

-include internal.mk
ifeq ($(filter $(TARGET),$(SUPPORTED_TARGETS)),)
$(error TARGET $(TARGET) not supported for this code example)
endif
include $(CY_TOOLS_DIR)/make/start.mk
