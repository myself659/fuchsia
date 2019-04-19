// Copyright 2019 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#pragma once

#include "../../bus.h"
#include <ddk/mmio-buffer.h>
#include <ddktl/protocol/pciroot.h>
#include <hwreg/bitfields.h>
#include <zircon/hw/pci.h>

namespace pci {

class FakeBus : public BusDeviceInterface {
public:
    void LinkDevice(fbl::RefPtr<pci::Device> device) final {
        fbl::AutoLock dev_list_lock(&dev_list_lock_);
        device_list_.insert(device);
    }

    void UnlinkDevice(pci::Device* device) final {
        fbl::AutoLock dev_list_lock(&dev_list_lock_);
        device_list_.erase(*device);
    }

    zx_status_t GetBti(pci_bdf_t, uint32_t index, zx::bti* bti) {
        return ZX_ERR_NOT_SUPPORTED;
    }

    pci::Device& get_device(pci_bdf_t bdf) {
        return *device_list_.find(bdf);
    }

    // For use with Devices that need to link to a Bus.
    BusDeviceInterface* bdi() { return static_cast<BusDeviceInterface*>(this); }

    const pci::DeviceList& device_list() { return device_list_; }

private:
    mutable fbl::Mutex dev_list_lock_;
    pci::DeviceList device_list_;
};

} // namespace pci
