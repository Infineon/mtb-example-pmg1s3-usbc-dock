################################################################################
# \file Makefile
# \version 1.0
#
# \brief
# Top-level application make file.
#
################################################################################
# \copyright
# Copyright 2024, Cypress Semiconductor Corporation (an Infineon company)
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
# COMBINED     -- Top Level Makefile usually for single standalone application
# APPLICATION -- Top Level Makefile usually for multi project application
# PROJECT      -- Project Makefile under Application
#
MTB_TYPE=COMBINED

# Target board/hardware (BSP).
# To change the target, it is recommended to use the Library manager
# ('make library-manager' from command line), which will also update Eclipse IDE launch
# configurations.
TARGET=PMG1S3DUAL

# Name of application (used to derive name of final linked file).
#
# If APPNAME is edited, ensure to update or regenerate launch
# configurations for your IDE.
APPNAME=mtb-example-pmg1s3-usbc-dock
BOOTNAME=mtb-example-pmg1-dock-bootstrap_APP_PMG1S3DUAL_GCC_ARM_Release_1_0_0_5
APPNAME_EXT=fw1

# Name of toolchain to use. Options include:
#
# GCC_ARM -- GCC provided with ModusToolbox software
# ARM      -- ARM Compiler (must be installed separately)
# IAR      -- IAR Compiler (must be installed separately)
#
# See also: CY_COMPILER_PATH below
TOOLCHAIN=GCC_ARM

# Default build configuration. Options include:
#
# Debug -- build with minimal optimizations, focus on debugging.
# Release -- build with full optimizations
# Custom -- build with custom configuration, set the optimization flag in CFLAGS
#
# If CONFIG is manually edited, ensure to update or regenerate launch configurations
# for your IDE.
CONFIG=Release

# If set to "true" or "1", display full command-lines when building.
VERBOSE=


################################################################################
# Advanced Configuration
################################################################################

# Enable optional code that is ordinarily disabled by default.
#
# Available components depend on the specific targeted hardware and firmware
# in use. In general, if you have
#
#     COMPONENTS=foo bar
#
# ... then code in directories named COMPONENT_foo and COMPONENT_bar will be
# added to the build
#
COMPONENTS=PMG1_PD3_DRP_EPR_CFG HPI_MASTER ANX7443 \
          SMART_POWER FLASH_LOG DMC_FWUPDATE


# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS=

# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
SOURCES=

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
INCLUDES=

# Add additional defines to the build process (without a leading -D).
DEFINES=CY_APP_LED_CONTROL_ENABLE=1 CY_USE_CONFIG_TABLE=1 CY_CONFIG_TABLE_TYPE=4

PMG_APP_COMMON_DEFINES=CY_APP_ROLE_PREFERENCE_ENABLE=1 CY_APP_POWER_ROLE_PREFERENCE_ENABLE=1 CY_APP_PD_PDO_SEL_ALGO=0 \
                       CY_APP_RESET_ON_ERROR_ENABLE=1 CY_APP_WATCHDOG_HARDWARE_RESET_ENABLE=1  \
                       CY_APP_FLASH_LOG_ROW_NUM=0x3F5 CY_APP_BOOT_LOADER_LAST_ROW=0x0F CY_APP_IMG1_LAST_FLASH_ROW_NUM=0x010F \
                       CY_APP_IMG2_LAST_FLASH_ROW_NUM=0x3EF CY_APP_FLASH_LOG_BACKUP_ROW_NUM=0x3F7 \
                       CY_APP_PD_USB4_SUPPORT_ENABLE=0

#DMC Defines
DMC_DEFINES=CY_APP_DMC_ENABLE=1

#PD Stack
PD_DEFINES=CY_PD_SINK_ONLY=0 CY_PD_SOURCE_ONLY=0 CY_PD_REV3_ENABLE=1 CY_PD_CBL_DISC_DISABLE=0 \
           NO_OF_BC_PORTS=0 BATTERY_CHARGING_ENABLE=0 CY_PD_VCONN_DISABLE=0 \
           VBUS_SLOW_DISCHARGE_EN=0 VBUS_C_DISCHG_DS=8 CY_PD_USB4_SUPPORT_ENABLE=1

# PDAltMode
PDALTMODE_DEFINES=TBT_DFP_SUPP=0 TBT_UFP_SUPP=0

#Debug Logging                
DEBUG_DEFINES=CY_APP_DEBUG_LEVEL=0 CY_APP_UART_DEBUG_ENABLE=1 CY_APP_LOG_DISCONNECT_EVT_ENABLE=0

ifeq ($(APPNAME_EXT), fw1)

DEFINES+=SYS_DEEPSLEEP_ENABLE=0

DMC_DEFINES+=CY_APP_USB_ENABLE=0 CY_APP_SMART_POWER_ENABLE=0 \
             CY_APP_DMC_PHASE1_UPDATE_ENABLE=0 CCGX_UPDATE=0 HX3_BOOT_WAIT_ENABLE=0 \
             CY_HPI_MASTER_ENABLE=0 CY_APP_USB_HID_INTF_ENABLE=0 ETAG_SUPPORT_ENABLE=1

PD_DEFINES+=CY_PD_EPR_ENABLE=0 CY_PD_EPR_AVS_ENABLE=0 CY_PD_PPS_SRC_ENABLE=0 \
            EXTENDED_ALERT_EVENTS_SUPPORT=0 CY_APP_PD_ENABLE=0 PMG1_V5V_CHANGE_DETECT=0 \
            CY_APP_BUCKBOOST_MP4247_ENABLE=0 CY_APP_BUCKBOOST_RT6190_ENABLE=0

FAULT_DEFINES+=VBUS_RCP_ENABLE=0 VBUS_SCP_ENABLE=0 VCONN_OCP_ENABLE=0 VBUS_OCP_ENABLE=0 \
               VBUS_OVP_ENABLE=0 VBUS_UVP_ENABLE=0 CY_APP_DEFER_SNK_VBUS_UVP_HANDLING=0

DEBUG_DEFINES+=CY_APP_FLASH_LOG_ENABLE=0

PDALTMODE_DEFINES+=DFP_ALT_MODE_SUPP=0 UFP_ALT_MODE_SUPP=0 PMG1_HPD_RX_ENABLE=0 \
                   CY_HPD_ENABLE=0 CCG_BB_ENABLE=0 \
                   DP_DFP_SUPP=0 DP_UFP_SUPP=0 MINOR_SVDM_VER_SUPPORT=0 \
                   BILLBOARD_1_2_2_SUPPORT=0 \
                   CUSTOM_ALT_MODE_UFP_SUPP=0 CUSTOM_ALT_MODE_DFP_SUPP=0 CUSTOM_OBJ_POSITION=0

else

DEFINES+=SYS_DEEPSLEEP_ENABLE=1 CY_APP_GET_REVISION_ENABLE=1

DMC_DEFINES+=CY_APP_USB_ENABLE=1 CY_APP_SMART_POWER_ENABLE=1 \
             CY_APP_DMC_PHASE1_UPDATE_ENABLE=1 CCGX_UPDATE=1 HX3_ENABLE=1 \
             CY_HPI_MASTER_ENABLE=1 CY_APP_USB_HID_INTF_ENABLE=1 ETAG_SUPPORT_ENABLE=1

PD_DEFINES+=CY_PD_EPR_ENABLE=1 CY_PD_EPR_AVS_ENABLE=1 CY_PD_PPS_SRC_ENABLE=1 \
            EXTENDED_ALERT_EVENTS_SUPPORT=1 CY_APP_PD_ENABLE=1 PMG1_V5V_CHANGE_DETECT=1 \
            CY_APP_BUCKBOOST_MP4247_ENABLE=1 CY_APP_BUCKBOOST_RT6190_ENABLE=0

FAULT_DEFINES+=VBUS_RCP_ENABLE=0 VBUS_SCP_ENABLE=1 VCONN_OCP_ENABLE=1 VBUS_OCP_ENABLE=1     \
               VBUS_OVP_ENABLE=1 VBUS_UVP_ENABLE=1 CY_APP_DEFER_SNK_VBUS_UVP_HANDLING=1

DEBUG_DEFINES+=CY_APP_FLASH_LOG_ENABLE=1

PDALTMODE_DEFINES+=DFP_ALT_MODE_SUPP=1 UFP_ALT_MODE_SUPP=1 \
                   PMG1_HPD_RX_ENABLE=1 CY_HPD_ENABLE=1 CCG_BB_ENABLE=1 \
                   DP_DFP_SUPP=1 DP_UFP_SUPP=1 MINOR_SVDM_VER_SUPPORT=1 \
                   BILLBOARD_1_2_2_SUPPORT=1 CY_APP_USBC_DOCK_EXCHANGE_CAP_DP_USB3=1 \
                   CUSTOM_ALT_MODE_UFP_SUPP=0 CUSTOM_ALT_MODE_DFP_SUPP=0 CUSTOM_OBJ_POSITION=1

endif

DEFINES +=$(DMC_DEFINES)
DEFINES +=$(PD_DEFINES)
DEFINES +=$(FAULT_DEFINES)
DEFINES +=$(DEBUG_DEFINES)
DEFINES +=$(PDALTMODE_DEFINES)
DEFINES +=$(PMG_APP_COMMON_DEFINES)

# Select softfp or hardfp floating point. Default is softfp.
VFP_SELECT=

# Additional / custom C compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CFLAGS=-mcpu=cortex-m0plus -mthumb -g -Wall -ffunction-sections -Os -fno-strict-aliasing -ffat-lto-objects -flto

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
LDFLAGS=

# Additional / custom libraries to link in to the application.
LDLIBS=

# Path to the linker script to use (if empty, use the default linker script).
ifeq ($(CONFIG), Release)
ifeq ($(TOOLCHAIN), GCC_ARM)
ifeq ($(APPNAME_EXT), fw1)
DEFINES += CY_APP_TYPE=0
LINKER_SCRIPT=$(MTB_TOOLS__TARGET_DIR)/COMPONENT_$(MTB_RECIPE__CORE)/TOOLCHAIN_$(TOOLCHAIN)/pmg1s3_app1.ld
else
DEFINES += CY_APP_TYPE=1
LINKER_SCRIPT=$(MTB_TOOLS__TARGET_DIR)/COMPONENT_$(MTB_RECIPE__CORE)/TOOLCHAIN_$(TOOLCHAIN)/pmg1s3_app2.ld
endif
endif
else
#default linker script
LINKER_SCRIPT=
endif


# Custom pre-build commands to run.
PREBUILD=

# Custom post-build commands to run.
ifeq ($(CONFIG), Release)
POSTBUILD=./utils/post_build.sh $(CY_TOOL_cymcuelftool_EXE_ABS) \
                                $(MTB_TOOLS__OUTPUT_CONFIG_DIR) \
                                $(APPNAME_EXT) \
                                $(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME) \
                                $(BOOTNAME) \
                                $(CY_TOOLS_DIR) \
                                $(OS)
else
POSTBUILD=
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
# software provided compiler by default).
CY_COMPILER_PATH=


# Locate ModusToolbox helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
     $(CY_WIN_HOME)/ModusToolbox/tools_* \
     $(HOME)/ModusToolbox/tools_* \
     /Applications/ModusToolbox/tools_*)

# If you install ModusToolbox software in a custom location, add the path to its
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
