################################################################################
# \file Makefile
# \version 1.0
#
# \brief
# Bluetooth LE HID Keyboard Solution Demo Application Makefile.
#
################################################################################
# \copyright
# Copyright 2018-2020 Cypress Semiconductor Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

################################################################################
# Basic Configuration
################################################################################

# Type of ModusToolbox Makefile Options include:
#
# COMBINED    -- Top Level Makefile usually for single standalone application
# APPLICATION -- Top Level Makefile usually for multi project application
# PROJECT     -- Project Makefile under Application
#
MTB_TYPE=COMBINED
# Target board/hardware (BSP).
# To change the target, it is recommended to use the Library manager
# ('make modlibs' from command line), which will also update Eclipse IDE launch
# configurations. If TARGET is manually edited, ensure TARGET_<BSP>.mtb with a
# valid URL exists in the application, run 'make getlibs' to fetch BSP contents
# and update or regenerate launch configurations for your IDE.
TARGET=CYW920829-KEYBOARD

# Core processor
CORE?=CM33

# Name of application (used to derive name of final linked file).
#
# If APPNAME is edited, ensure to update or regenerate launch
# configurations for your IDE.
APPNAME=mtb-example-btstack-freertos-cyw20829-keyboard

# Name of toolchain to use. Options include:
#
# GCC_ARM -- GCC 7.2.1, provided with ModusToolbox IDE
# ARM     -- ARM Compiler (must be installed separately)
# IAR     -- IAR Compiler (must be installed separately)
#
# See also: CY_COMPILER_PATH below
TOOLCHAIN=GCC_ARM

# Default build configuration. Options include:
#
# Debug   -- build with minimal optimizations, focus on debugging.
# Release -- build with full optimizations
# Custom -- build with custom configuration, set the optimization flag in CFLAGS
# 
# If CONFIG is manually edited, ensure to update or regenerate launch configurations 
# for your IDE.
CONFIG=Custom

# If set to "true" or "1", display full command-lines when building.
VERBOSE=

################################################################################
# Advanced Configuration
################################################################################
OTA_ENABLE = 0

##############################
# COMPONENTS
##############################
# Enable optional code that is ordinarily disabled by default.
#
# Available components depend on the specific targeted hardware and firmware
# in use. In general, if you have
#
#    COMPONENTS=foo bar
#
# ... then code in directories named COMPONENT_foo and COMPONENT_bar will be
# added to the build
#

COMPONENTS=FREERTOS WICED_BLE

# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS=

##############################
# Defines/ Includes
##############################
# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
SOURCES=

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
INCLUDES=./app_configs

OTA_APP_VERSION_MAJOR=1
OTA_APP_VERSION_MINOR=0
OTA_APP_VERSION_BUILD=0

# Add additional defines to the build process (without a leading -D).
DEFINES=CY_RETARGET_IO_CONVERT_LF_TO_CRLF CY_RTOS_AWARE STACK_INSIDE_FREE_RTOS CY_CFG_PWR_DEEPSLEEP_RAM_LATENCY=15

DEFINES+= CYBT_PLATFORM_TRACE_ENABLE=0
 DEFINES+=\
        APP_VERSION_MAJOR=$(OTA_APP_VERSION_MAJOR)\
        APP_VERSION_MINOR=$(OTA_APP_VERSION_MINOR)\
        APP_VERSION_BUILD=$(OTA_APP_VERSION_BUILD)


##############################
# Floating point usage
##############################
# Select softfp or hardfp floating point. Default is softfp.
VFP_SELECT=

CY_BUILD_LOCATION:=./build


##############################
# Compiler and Linker Flags
##############################

# Additional / custom C compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CFLAGS+=-O2

# Additional / custom C++ compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CXXFLAGS=

# Additional / custom assembler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ASFLAGS=

# Additional / custom linker flags.
# These are needed for enabling debugging with OpenOCD and GDB in MTB
ifeq ($(TOOLCHAIN),GCC_ARM)
LDFLAGS=-Wl,--undefined=uxTopUsedPriority
else
ifeq ($(TOOLCHAIN),IAR)
LDFLAGS=--keep uxTopUsedPriority
else
ifeq ($(TOOLCHAIN),ARM)
LDFLAGS=--undefined=uxTopUsedPriority
else
LDFLAGS=
endif # ARM
endif # IAR
endif # GCC_ARM

# Additional / custom libraries to link in to the application.
LDLIBS=
# Custom pre-build commands to run.
PREBUILD=

# Custom post-build commands to run.
POSTBUILD=

ifeq ($(OTA_ENABLE),1)
OTA_PLATFORM = CYW20829
OTA_SUPPORT = 1
OTA_BT_ONLY = 1
OTA_BT_SUPPORT = 1
OTA_BT_SECURE = 0
FLASH_BASE_ADDRESS = 0x60000000
CY_IGNORE+= $(SEARCH_mcuboot)
OTA_FLASH_MAP?=$(RELATIVE_FILE1_FILE2)/../configs/flashmap/cyw20829_xip_swap_single.json
DEFINES+=ENABLE_OTA_LOGS ENABLE_OTA
OTA_LINKER_FILE = ./templates/TARGET_CYW920829-KEYBOARD/COMPONENT_CM33/TOOLCHAIN_GCC_ARM/cyw20829_ns_flash_cbus_ota_xip.ld
ifneq ($(MAKECMDGOALS),getlibs)
ifneq ($(MAKECMDGOALS),get_app_info)
ifneq ($(MAKECMDGOALS),printlibs)
    include ../mtb_shared/ota-update/release-v*/makefiles/target_ota.mk
    include ../mtb_shared/ota-update/release-v*/makefiles/mcuboot_flashmap.mk
endif
endif
endif
include ./local.mk
else
CY_IGNORE+=./app_bt_ota
CY_IGNORE+=./local.mk
CY_IGNORE+= $(SEARCH_aws-iot-device-sdk-embedded-C)
CY_IGNORE+= $(SEARCH_ota-update)
CY_IGNORE+= $(SEARCH_aws-iot-device-sdk-port)
CY_IGNORE+= $(SEARCH_connectivity-utilities)
CY_IGNORE+= $(SEARCH_cy-mbedtls-acceleration)
CY_IGNORE+= $(SEARCH_http-client)
CY_IGNORE+= $(SEARCH_lwip-freertos-integration)
CY_IGNORE+= $(SEARCH_lwip-network-interface-integration)
CY_IGNORE+= $(SEARCH_lwip)
CY_IGNORE+= $(SEARCH_mbedtls)
CY_IGNORE+= $(SEARCH_mqtt)
CY_IGNORE+= $(SEARCH_secure-sockets)
CY_IGNORE+= $(SEARCH_whd-bsp-integration)
CY_IGNORE+= $(SEARCH_wifi-connection-manager)
CY_IGNORE+= $(SEARCH_wifi-core-freertos-lwip-mbedtls)
CY_IGNORE+= $(SEARCH_wifi-host-driver)
CY_IGNORE+= $(SEARCH_wpa3-external-supplicant)
CY_IGNORE+= $(SEARCH_mcuboot)
# Path to the linker script to use (if empty, use the default linker script).
LINKER_SCRIPT=./templates/TARGET_CYW920829-KEYBOARD/COMPONENT_CM33/TOOLCHAIN_GCC_ARM/linker.ld
endif

################################################################################
# Paths
################################################################################

# Relative path to the project directory (default is the Makefile's directory).
#
# This controls where automatic source code discovery looks for code.
CY_APP_PATH=

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI>#<COMMIT>#<LOCATION>. If the <LOCATION> field 
# begins with $$ASSET_REPO$$, then the repo is deposited in the path specified by 
# the CY_GETLIBS_SHARED_PATH variable. The default location is one directory level 
# above the current app directory.
# This is used with CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

# Absolute path to the compiler's "bin" directory.
#
# The default depends on the selected TOOLCHAIN (GCC_ARM uses the ModusToolbox
# IDE provided compiler by default).
CY_COMPILER_PATH=


# Locate ModusToolbox IDE helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_*)

# If you install ModusToolbox IDE in a custom location, add the path to its
# "tools_X.Y" folder (where X and Y are the version number of the tools
# folder). Make sure you use forward slashes.
CY_TOOLS_PATHS+=

# Default to the newest installed tools folder, or the users override (if it's
# found).
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS). On Windows, use forward slashes.)
endif

$(info Tools Directory: $(CY_TOOLS_DIR))

include $(CY_TOOLS_DIR)/make/start.mk
