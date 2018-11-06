# Copyright 2018 The Fuchsia Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_TYPE := driver

MODULE_SRCS := \
    $(LOCAL_DIR)/imx-sdhci.c

MODULE_STATIC_LIBS := system/ulib/ddk system/ulib/sync system/ulib/pretty

MODULE_LIBS := system/ulib/driver system/ulib/zircon system/ulib/c

MODULE_HEADER_DEPS := system/dev/lib/imx8m

MODULE_BANJO_LIBS := \
    system/banjo/ddk-protocol-block \
    system/banjo/ddk-protocol-gpio \
    system/banjo/ddk-protocol-platform-device \

include make/module.mk
