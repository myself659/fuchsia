// Copyright 2019 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#pragma once

#include "../fakes/fake_bus.h"
#include "../fakes/fake_config.h"
#include "../fakes/fake_pciroot.h"
#include "../fakes/fake_upstream_node.h"
#include "driver_tests.h"

#include <ddk/device.h>
#include <ddktl/device.h>

namespace pci {

class FakeBusDriver;
using FakeBusDriverType = ddk::Device<FakeBusDriver>;
class FakeBusDriver : public FakeBusDriverType {
public:
    ~FakeBusDriver() = default;
    static zx_status_t Create(zx_device_t* parent, const char* name);
    zx_status_t CreateDevice(pci_bdf_t bdf, uint8_t* base_cfg, size_t base_cfg_size);

    FakePciType0Config& GetDevice(pci_bdf_t bdf) { return pciroot().ecam().get(bdf).device; }
    FakePciType1Config& GetBridge(pci_bdf_t bdf) { return pciroot().ecam().get(bdf).bridge; }
    uint8_t* GetRawConfig(pci_bdf_t bdf) { return pciroot().ecam().get(bdf).config; }
    uint8_t* GetRawExtConfig(pci_bdf_t bdf) { return pciroot().ecam().get(bdf).config; }

    FakePciroot& pciroot() { return *pciroot_; }
    FakeUpstreamNode& upstream() { return upstream_; }
    FakeBus& bus() { return bus_; }
    pci_bdf_t const test_bdf() { return test_bdf_; }
    void DdkRelease() { delete this; }

private:
    FakeBusDriver(zx_device_t* parent, std::unique_ptr<FakePciroot> root)
        : FakeBusDriverType(parent), pciroot_(std::move(root)),
          upstream_(UpstreamNode::Type::ROOT, 0) {}

    std::unique_ptr<FakePciroot> pciroot_;
    FakeUpstreamNode upstream_;
    FakeBus bus_;
    const pci_bdf_t test_bdf_ = {PCI_TEST_BUS_ID, PCI_TEST_DEV_ID, PCI_TEST_FUNC_ID};
};
} // namespace pci
