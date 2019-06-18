// Copyright 2019 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#pragma once

#include "../../allocation.h"
#include <lib/zx/vmo.h>

namespace pci {

// Normally we would track the allocations and assert on issues during
// cleanup, but presently with an IsolatedDevmgr we don't have a way
// to cleanly tear down the FakeBusDriver, so no dtors on anything
// will be called anyway.
class FakeAllocation : public PciAllocation {
public:
    FakeAllocation(zx_paddr_t base, size_t size)
        : PciAllocation(zx::resource(ZX_HANDLE_INVALID)), base_(base), size_(size) {}
    zx_paddr_t base() const final { return base_; }
    size_t size() const final { return size_; }
    zx_status_t CreateVmObject(zx::vmo* out_vmo) const final {
        return zx::vmo::create(size_, 0, out_vmo);
    }

private:
    zx_paddr_t base_;
    size_t size_;
};

class FakeAllocator : public PciAllocator {
public:
    zx_status_t AllocateWindow(zx_paddr_t base, size_t size,
                               fbl::unique_ptr<PciAllocation>* out_alloc) final {

        *out_alloc = std::unique_ptr<PciAllocation>(new FakeAllocation(base, size));
        return ZX_OK;
    }

    zx_status_t GrantAddressSpace(fbl::unique_ptr<PciAllocation> alloc) final {
        alloc.release();
        return ZX_OK;
    }
};

} // namespace pci
