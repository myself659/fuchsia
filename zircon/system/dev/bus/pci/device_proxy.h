// Copyright 2019 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#pragma once

#include <ddktl/device.h>
#include <ddktl/protocol/pci.h>
#include <lib/zx/channel.h>
#include <stdio.h>
#include <sys/types.h>

namespace pci {

enum PciRpcOp : uint32_t {
    PCI_OP_INVALID = 0,
    PCI_OP_CONFIG_READ,
    PCI_OP_CONFIG_WRITE,
    PCI_OP_CONNECT_SYSMEM,
    PCI_OP_ENABLE_BUS_MASTER,
    PCI_OP_GET_AUXDATA,
    PCI_OP_GET_BAR,
    PCI_OP_GET_BTI,
    PCI_OP_GET_DEVICE_INFO,
    PCI_OP_GET_NEXT_CAPABILITY,
    PCI_OP_MAP_INTERRUPT,
    PCI_OP_QUERY_IRQ_MODE,
    PCI_OP_RESET_DEVICE,
    PCI_OP_SET_IRQ_MODE,
    PCI_OP_MAX,
};

// TODO(ZX-3146): When the kernel driver is removed we should consolidate the pci banjo
// definitions and these rpc messages to avoid duplication.
struct PciMsgCfg {
    uint16_t offset;
    uint16_t width;
    uint32_t value;
};

// In the event of an MMIO bar all the information here will be available
// via VMO operations on the handle passed back.
struct PciMsgBar {
    uint32_t id;
    bool is_mmio;
    uint16_t io_addr;
    uint16_t io_size;
};

// For use with QUERY_IRQ_MODE, SET_IRQ_MODE, and MAP_INTERRUPT
struct PciMsgIrq {
    zx_pci_irq_mode_t mode;
    union {
        int32_t which_irq;
        uint32_t max_irqs;
        uint32_t requested_irqs;
    };
};

constexpr uint16_t kPciCapabilityOffsetFirst = 4907u;
struct PciMsgCapaility {
    uint16_t id;
    uint16_t offset;
    bool is_extended;
};

// Outside the range of both standard and extended capailities, this
// exists to give a value to allow GetNextCapaility and GetFirstCapability
// to be served by the same impl on the other end of RPC.
const uint16_t PciCapOffsetFirst = 4097u;

// TODO(ZX-3927): port this to non-zx_pcie structures
using PciMsgDeviceInfo = zx_pcie_device_info_t;

struct PciRpcMsg {
    zx_txid_t txid; // handled by zx_channel_call
    uint32_t op;
    zx_status_t ret;
    // Subtract the size of the preceeding 6 uint32_ts to keep
    // the structure inside a single page.
    union {
        bool enable;
        PciMsgCfg cfg;
        PciMsgIrq irq;
        PciMsgBar bar;
        PciMsgDeviceInfo info;
        PciMsgCapaility cap;
        uint32_t bti_index;
    };
};
static_assert(sizeof(PciRpcMsg) <= ZX_PAGE_SIZE);

class DeviceProxy;
using PciDeviceProxyType = ddk::Device<pci::DeviceProxy, ddk::GetProtocolable>;
class DeviceProxy : public PciDeviceProxyType,
                    public ddk::PciProtocol<pci::DeviceProxy> {
public:
    DeviceProxy(zx_device_t* parent, zx_handle_t rpcch)
        : PciDeviceProxyType(parent), rpcch_(rpcch) {}
    static zx_status_t Create(zx_device_t* parent, zx_handle_t rpcch, const char* name);
    // A helper method to reduce the complexity of each individual PciProtocol method.
    zx_status_t RpcRequest(PciRpcOp op, zx_handle_t* handle, PciRpcMsg* req, PciRpcMsg* resp);
    zx_status_t DdkGetProtocol(uint32_t proto_id, void* out);
    void DdkRelease() { delete this; }

    // ddk::PciProtocol implementations.
    zx_status_t PciGetBar(uint32_t bar_id, zx_pci_bar_t* out_res);
    zx_status_t PciEnableBusMaster(bool enable);
    zx_status_t PciResetDevice();
    zx_status_t PciMapInterrupt(zx_status_t which_irq, zx::interrupt* out_handle);
    zx_status_t PciQueryIrqMode(zx_pci_irq_mode_t mode, uint32_t* out_max_irqs);
    zx_status_t PciSetIrqMode(zx_pci_irq_mode_t mode, uint32_t requested_irq_count);
    zx_status_t PciGetDeviceInfo(zx_pcie_device_info_t* out_into);
    zx_status_t PciConfigRead8(uint16_t offset, uint8_t* out_value);
    zx_status_t PciConfigRead16(uint16_t offset, uint16_t* out_value);
    zx_status_t PciConfigRead32(uint16_t offset, uint32_t* out_value);
    zx_status_t PciConfigWrite8(uint16_t offset, uint8_t value);
    zx_status_t PciConfigWrite16(uint16_t offset, uint16_t value);
    zx_status_t PciConfigWrite32(uint16_t offset, uint32_t value);
    zx_status_t PciGetFirstCapability(uint8_t cap_id, uint8_t* out_offset);
    zx_status_t PciGetNextCapability(uint8_t cap_id, uint8_t offset, uint8_t* out_offset);
    zx_status_t PciGetAuxdata(const char* args,
                              void* out_data_buffer,
                              size_t data_size,
                              size_t* out_data_actual);
    zx_status_t PciGetBti(uint32_t index, zx::bti* out_bti);

private:
    // Helpers to marshal config-based RPC.
    template <typename T>
    zx_status_t PciConfigRead(uint16_t offset, T* out_value);
    template <typename T>
    zx_status_t PciConfigWrite(uint16_t offset, T value);
    zx::channel rpcch_;
};

} // namespace pci
