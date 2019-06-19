// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/device.h>
#include <ddktl/device.h>
#include <lib/device-protocol/pdev.h>
#include <ddktl/protocol/ethernet/board.h>

#include <optional>

namespace eth {

class Asix88179Ethernet;
using DeviceType = ddk::Device<Asix88179Ethernet, ddk::Unbindable>;

class Asix88179Ethernet : public DeviceType,
                    public ddk::EthBoardProtocol<Asix88179Ethernet, ddk::base_protocol> {
public:
    DISALLOW_COPY_AND_ASSIGN_ALLOW_MOVE(Asix88179Ethernet);

    explicit Asix88179Ethernet(zx_device_t* parent)
        : DeviceType(parent) {}

    static zx_status_t Create(void* ctx, zx_device_t* parent);

    // DDK Hooks.
    void DdkRelease();
    void DdkUnbind();

    // ETH_BOARD protocol.
    zx_status_t EthBoardResetPhy();

private:
};

} // namespace eth
