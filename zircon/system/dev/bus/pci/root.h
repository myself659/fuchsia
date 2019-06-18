// Copyright 2019 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#pragma once

#include "allocation.h"
#include "ref_counted.h"
#include "upstream_node.h"
#include <fbl/intrusive_wavl_tree.h>
#include <fbl/macros.h>
#include <fbl/ref_ptr.h>
#include <zircon/compiler.h>

namespace pci {
// Forward declaration of Bus to avoid recursive header inclusion between
// bus.h and root.h
class Bus;
class PciRoot : public UpstreamNode {
public:
    // Implement refcounting for UpstreamNode
    PCI_IMPLEMENT_REFCOUNTED;

    // Disallow copying, assigning and moving.
    PciRoot(const PciRoot&) = delete;
    PciRoot(PciRoot&&) = delete;
    PciRoot& operator=(const PciRoot&) = delete;
    PciRoot& operator=(PciRoot&&) = delete;
    virtual ~PciRoot() {}

    // Prefetch ranges can be allocated from the general mmio allocator
    // without any specific restrictions. There's no distinction between
    // prefetchable and non-prefetchable address space at this point
    // in the upstream allocation chain. This matters for bridges which
    // specifically will need their prefetch devices to be within the
    // prefetch window of upstream bridges.
    PciAllocator& mmio_regions() final { return mmio_regions_; }
    PciAllocator& pf_mmio_regions() final { return pf_mmio_regions_; }
    PciAllocator& pio_regions() final { return pio_regions_; }

protected:
    // PciRootAllocators can be trivially constructed because they only need
    // a way to call protocol operations. We set three of them up for the root
    // to use for accessing address space over the pciroot protocol.
    //
    // 1) mmio which tries to get mmio addresses < 4GB
    // 2) pf_mmio which will allocate anywhere in the mmio space
    // 3) pio which will attempt to allocate from the pio allocator
    PciRoot(uint32_t mbus_id, ddk::PcirootProtocolClient proto)
        : UpstreamNode(UpstreamNode::Type::ROOT, mbus_id),
          mmio_regions_(proto, PCI_ADDRESS_SPACE_MMIO, true),
          pf_mmio_regions_(proto, PCI_ADDRESS_SPACE_MMIO, false),
          pio_regions_(proto, PCI_ADDRESS_SPACE_IO, false) {}

    PciRootAllocator mmio_regions_;
    PciRootAllocator pf_mmio_regions_;
    PciRootAllocator pio_regions_;

    // Only allow the bus driver to instantiate a PciRoot
    friend class Bus;
};

// The PcieRoot derived version exists for support of RCRB (root complex
// register block), but is not implemented yet.
class PcieRoot : public PciRoot {
    PcieRoot(uint32_t managed_bus_id, ddk::PcirootProtocolClient proto)
        : PciRoot(managed_bus_id, proto) {}
    virtual ~PcieRoot() {}
};

} // namespace pci
