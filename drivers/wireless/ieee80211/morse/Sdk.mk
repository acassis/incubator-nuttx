############################################################################
# drivers/wireless/ieee80211/morse/Sdk.mk
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

# Location of the Morse Micro MM-IoT-SDK and of the pre-built libmorse.
# Shared by the driver build (Make.defs) and by the architecture link step
# (arch/arm/src/common/Make.defs).

MORSE_DIR = $(TOPDIR)$(DELIM)drivers$(DELIM)wireless$(DELIM)ieee80211$(DELIM)morse

MORSE_SDK_VERSION := $(patsubst "%",%,$(strip $(CONFIG_IEEE80211_MORSE_SDK_VERSION)))
MORSE_SDK_PATH    := $(patsubst "%",%,$(strip $(CONFIG_IEEE80211_MORSE_SDK_PATH)))

ifeq ($(MORSE_SDK_PATH),)
  MORSE_SDK = $(MORSE_DIR)$(DELIM)mm-iot-sdk-$(MORSE_SDK_VERSION)
else
  MORSE_SDK = $(MORSE_SDK_PATH)
endif

MORSE_FRAMEWORK = $(MORSE_SDK)$(DELIM)framework

ifeq ($(CONFIG_ARCH_CORTEXM7),y)
  MORSE_CORE = arm-cortex-m7f
else ifeq ($(CONFIG_ARCH_CORTEXM33),y)
  MORSE_CORE = arm-cortex-m33f
else
  MORSE_CORE = arm-cortex-m4f
endif

MORSE_LIB = $(MORSE_FRAMEWORK)$(DELIM)morselib$(DELIM)lib$(DELIM)$(MORSE_CORE)$(DELIM)libmorse.a
