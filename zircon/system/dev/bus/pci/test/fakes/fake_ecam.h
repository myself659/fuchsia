// Copyright 2019 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#pragma once

#include "../../common.h"

#include <ddktl/protocol/pciroot.h>
#include <hwreg/bitfields.h>
#include <lib/mmio/mmio.h>
#include <zircon/hw/pci.h>

struct IoBaseAddress {
    uint32_t value;
    DEF_SUBBIT(value, 0, is_io_space);
    // bit 1 is reserved.
    DEF_SUBFIELD(value, 31, 2, address);
};
static_assert(sizeof(IoBaseAddress) == 4, "Bad size for IoBaseAddress");

struct Mmio32BaseAddress {
    uint32_t value;
    DEF_SUBBIT(value, 0, is_io_space);
    // bit 1 is reserved.
    DEF_SUBBIT(value, 2, is_64bit);
    DEF_SUBBIT(value, 3, is_prefetchable);
    DEF_SUBFIELD(value, 31, 4, address);
};
static_assert(sizeof(Mmio32BaseAddress) == 4, "Bad size for Mmio32BaseAddress");

struct FakeBaseAddress {
    union {
        IoBaseAddress io;
        Mmio32BaseAddress mmio32;
        uint32_t mmio64;
    };
};
static_assert(sizeof(FakeBaseAddress) == 4, "Bad size for FakeBaseAddress");

// Defines set_NAME and NAME() in addition to a private member to match the same
// chaining possible using the set_ methods DEF_SUBBIT generates.
//
// This macro has a pitfall if used following a private: declaration in the
// class/struct, so only use it in the public section.
#define DEF_WRAPPED_FIELD(TYPE, NAME) \
private:                              \
    TYPE NAME##_;                     \
                                      \
public:                               \
    TYPE NAME() {                     \
        return NAME##_;               \
    }                                 \
    auto& set_##NAME(TYPE val) {      \
        NAME##_ = val;                \
        return *this;                 \
    }                                 \
    static_assert(true) // eat a ;

// A fake implementation of a PCI device configuration (Type 00h)
struct FakePciType0Config {
    DEF_WRAPPED_FIELD(uint16_t, vendor_id);
    DEF_WRAPPED_FIELD(uint16_t, device_id);
    DEF_WRAPPED_FIELD(uint16_t, command);
    DEF_SUBBIT(command_, 0, io_space_en);
    DEF_SUBBIT(command_, 1, mem_space_en);
    DEF_SUBBIT(command_, 2, bus_master_en);
    DEF_SUBBIT(command_, 3, special_cycles_en);
    DEF_SUBBIT(command_, 4, men_write_and_inval_en);
    DEF_SUBBIT(command_, 5, vga_palette_snoop_en);
    DEF_SUBBIT(command_, 6, parity_error_resp);
    // bit 7 is hardwired to 0.
    DEF_SUBBIT(command_, 8, serr_en);
    DEF_SUBBIT(command_, 9, fast_back_to_back_en);
    DEF_SUBBIT(command_, 10, interrupt_disable);
    DEF_WRAPPED_FIELD(uint16_t, status);
    // bits 2:0 are reserved.
    DEF_SUBBIT(status_, 3, int_status);
    DEF_SUBBIT(status_, 4, capabilities_list);
    DEF_SUBBIT(status_, 5, is_66mhz_capable);
    // bit 6 is reserved.
    DEF_SUBBIT(status_, 7, fast_back_to_back_capable);
    DEF_SUBBIT(status_, 8, master_data_parity_error);
    DEF_SUBFIELD(status_, 10, 9, devsel_timing);
    DEF_SUBBIT(status_, 11, signaled_target_abort);
    DEF_SUBBIT(status_, 12, received_target_abort);
    DEF_SUBBIT(status_, 13, received_master_abort);
    DEF_SUBBIT(status_, 14, signaled_system_error);
    DEF_SUBBIT(status_, 15, detected_parity_error);
    DEF_WRAPPED_FIELD(uint8_t, revision_id);
    DEF_WRAPPED_FIELD(uint8_t, program_interface);
    DEF_WRAPPED_FIELD(uint8_t, sub_class);
    DEF_WRAPPED_FIELD(uint8_t, base_class);
    DEF_WRAPPED_FIELD(uint8_t, cache_line_size);
    DEF_WRAPPED_FIELD(uint8_t, latency_timer);
    DEF_WRAPPED_FIELD(uint8_t, header_type);
    DEF_WRAPPED_FIELD(uint8_t, bist);
    DEF_SUBFIELD(bist_, 3, 0, completion_code);
    // bits 4-5 are reserved.
    DEF_SUBBIT(bist_, 6, start_bist);
    DEF_SUBBIT(bist_, 7, bist_capable);
    FakeBaseAddress base_address[6];
    DEF_WRAPPED_FIELD(uint32_t, cardbus_cis_ptr);
    DEF_WRAPPED_FIELD(uint16_t, subsystem_vendor_id);
    DEF_WRAPPED_FIELD(uint16_t, subsystem_id);
    DEF_WRAPPED_FIELD(uint32_t, expansion_rom_address);
    DEF_WRAPPED_FIELD(uint8_t, capabilities_ptr);
    uint8_t reserved_0[3];
    uint32_t reserved_1;
    DEF_WRAPPED_FIELD(uint8_t, interrupt_line);
    DEF_WRAPPED_FIELD(uint8_t, interrupt_pin);
    DEF_WRAPPED_FIELD(uint8_t, min_grant);
    DEF_WRAPPED_FIELD(uint8_t, max_latency);
};
static_assert(sizeof(FakePciType0Config) == 64, "Bad size for PciType0Config");

// A fake implementation of a PCI bridge configuration (Type 01h)
struct FakePciType1Config {
    DEF_WRAPPED_FIELD(uint16_t, vendor_id);
    DEF_WRAPPED_FIELD(uint16_t, device_id);
    DEF_WRAPPED_FIELD(uint16_t, command);
    DEF_SUBBIT(command_, 0, io_space_en);
    DEF_SUBBIT(command_, 1, mem_space_en);
    DEF_SUBBIT(command_, 2, bus_master_en);
    DEF_SUBBIT(command_, 3, special_cycles_en);
    DEF_SUBBIT(command_, 4, men_write_and_inval_en);
    DEF_SUBBIT(command_, 5, vga_palette_snoop_en);
    DEF_SUBBIT(command_, 6, parity_error_resp);
    // bit 7 is hardwired to 0.
    DEF_SUBBIT(command_, 8, serr_en);
    DEF_SUBBIT(command_, 9, fast_back_to_back_en);
    DEF_SUBBIT(command_, 10, interrupt_disable);
    DEF_WRAPPED_FIELD(uint16_t, status);
    // bits 2:0 are reserved.
    DEF_SUBBIT(status_, 3, int_status);
    DEF_SUBBIT(status_, 4, capabilities_list);
    DEF_SUBBIT(status_, 5, is_66mhz_capable);
    // bit 6 is reserved.
    DEF_SUBBIT(status_, 7, fast_back_to_back_capable);
    DEF_SUBBIT(status_, 8, master_data_parity_error);
    DEF_SUBFIELD(status_, 10, 9, devsel_timing);
    DEF_SUBBIT(status_, 11, signaled_target_abort);
    DEF_SUBBIT(status_, 12, received_target_abort);
    DEF_SUBBIT(status_, 13, received_master_abort);
    DEF_SUBBIT(status_, 14, signaled_system_error);
    DEF_SUBBIT(status_, 15, detected_parity_error);
    DEF_WRAPPED_FIELD(uint8_t, revision_id);
    DEF_WRAPPED_FIELD(uint8_t, program_interface);
    DEF_WRAPPED_FIELD(uint8_t, sub_class);
    DEF_WRAPPED_FIELD(uint8_t, base_class);
    DEF_WRAPPED_FIELD(uint8_t, cache_line_size);
    DEF_WRAPPED_FIELD(uint8_t, latency_timer);
    DEF_WRAPPED_FIELD(uint8_t, header_type);
    DEF_WRAPPED_FIELD(uint8_t, bist);
    DEF_SUBFIELD(bist_, 3, 0, completion_code);
    // bits 5:4 are reserved.
    DEF_SUBBIT(bist_, 6, start_bist);
    DEF_SUBBIT(bist_, 7, bist_capable);
    FakeBaseAddress base_address[2];
    DEF_WRAPPED_FIELD(uint8_t, primary_bus_number);
    DEF_WRAPPED_FIELD(uint8_t, secondary_bus_number);
    DEF_WRAPPED_FIELD(uint8_t, subordinate_bus_number);
    DEF_WRAPPED_FIELD(uint8_t, secondary_latency_timer);
    DEF_WRAPPED_FIELD(uint8_t, io_base);
    DEF_WRAPPED_FIELD(uint8_t, io_limit);
    DEF_WRAPPED_FIELD(uint16_t, secondary_status);
    // bits 4:0 are reserved.
    DEF_SUBBIT(secondary_status_, 5, secondary_is_66mhz_capable);
    // bit 6 is reserved.
    DEF_SUBBIT(secondary_status_, 7, secondary_fast_back_to_back_capable);
    DEF_SUBBIT(secondary_status_, 8, secondary_master_data_parity_error);
    DEF_SUBFIELD(secondary_status_, 10, 9, secondary_devsel_timing);
    DEF_SUBBIT(secondary_status_, 11, secondary_signaled_target_abort);
    DEF_SUBBIT(secondary_status_, 12, secondary_received_target_abort);
    DEF_SUBBIT(secondary_status_, 13, secondary_received_master_abort);
    DEF_SUBBIT(secondary_status_, 14, secondary_signaled_system_error);
    DEF_SUBBIT(secondary_status_, 15, secondary_detected_parity_error);
    DEF_WRAPPED_FIELD(uint16_t, memory_base);
    DEF_WRAPPED_FIELD(uint16_t, memory_limit);
    DEF_WRAPPED_FIELD(uint16_t, prefetchable_memory_base);
    DEF_WRAPPED_FIELD(uint16_t, prefetchable_memory_limit);
    DEF_WRAPPED_FIELD(uint32_t, prfetchable_memory_base_upper);
    DEF_WRAPPED_FIELD(uint32_t, prfetchable_memory_limit_upper);
    DEF_WRAPPED_FIELD(uint16_t, io_base_upper);
    DEF_WRAPPED_FIELD(uint16_t, io_limit_upper);
    DEF_WRAPPED_FIELD(uint8_t, capabilities_ptr);
    uint8_t reserved_0[3];
    DEF_WRAPPED_FIELD(uint32_t, expansion_rom_address);
    DEF_WRAPPED_FIELD(uint8_t, interrupt_line);
    DEF_WRAPPED_FIELD(uint8_t, interrupt_pin);
    DEF_WRAPPED_FIELD(uint16_t, bridge_control);
    DEF_SUBBIT(bridge_control_, 0, secondary_parity_error_resp);
    DEF_SUBBIT(bridge_control_, 1, secondary_serr_en);
    DEF_SUBBIT(bridge_control_, 2, isa_enable);
    DEF_SUBBIT(bridge_control_, 3, vga_enable);
    DEF_SUBBIT(bridge_control_, 4, vga_16bit_decode);
    DEF_SUBBIT(bridge_control_, 5, master_abort_mode);
    DEF_SUBBIT(bridge_control_, 6, seconday_bus_reset);
    DEF_SUBBIT(bridge_control_, 7, secondary_fast_back_to_back_en);
    DEF_SUBBIT(bridge_control_, 8, primary_discard_timer);
    DEF_SUBBIT(bridge_control_, 9, secondary_discard_timer);
    DEF_SUBBIT(bridge_control_, 10, discard_timer_status);
    DEF_SUBBIT(bridge_control_, 11, discard_timer_serr_en);
    // bits 15:12 are reserved.
};
static_assert(sizeof(FakePciType1Config) == 64, "Bad size for PciType1Config");
#undef DEF_WRAPPED_FIELD

union FakeDeviceConfig {
    FakePciType0Config device;
    FakePciType1Config bridge;
    uint8_t config[PCI_BASE_CONFIG_SIZE];
    uint8_t ext_config[PCI_EXT_CONFIG_SIZE];
};
static_assert(sizeof(FakeDeviceConfig) == 4096, "Bad size for FakeDeviceConfig");

// FakeEcam represents a contiguous block of PCI devices covering the bus range
// from |bus_start|:|bus_end|. This allows tests to create a virtual collection
// of buses that look like a real contiguous ecam with valid devices to scan
// and poke at by the PCI bus driver.
class FakeEcam {
public:
    // Allow assign / move.
    FakeEcam(FakeEcam&&) = default;
    FakeEcam& operator=(FakeEcam&&) = default;
    // Disallow copy.
    FakeEcam(const FakeEcam&) = delete;
    FakeEcam& operator=(const FakeEcam&) = delete;

    static zx_status_t Create(uint8_t bus_start,
                              uint8_t bus_end,
                              std::optional<FakeEcam>* out_ecam) {
        const size_t cnt = (bus_end - bus_start + 1) * PCI_MAX_FUNCTIONS_PER_BUS;
        const size_t bytes = sizeof(FakeDeviceConfig) * cnt;

        zx::vmo vmo;
        zx_status_t st = zx::vmo::create(bytes, 0, &vmo);
        if (st != ZX_OK) {
            return st;
        }
        std::optional<ddk::MmioBuffer> mmio;
        st = ddk::MmioBuffer::Create(0, bytes, std::move(vmo), ZX_CACHE_POLICY_UNCACHED, &mmio);
        if (st != ZX_OK) {
            return st;
        }

        *out_ecam = FakeEcam(std::move(*mmio), bus_start, bus_end, cnt);
        return ZX_OK;
    }

    // Provide ways to access individual devices in the ecam by BDF address.
    FakeDeviceConfig& get(uint8_t bus_id, uint8_t dev_id, uint8_t func_id) {
        ZX_ASSERT(bus_id >= bus_start_);
        ZX_ASSERT(bus_id <= bus_end_);

        size_t offset = bus_id * PCI_MAX_FUNCTIONS_PER_BUS;
        offset += dev_id * PCI_MAX_FUNCTIONS_PER_DEVICE;
        offset += func_id;
        ZX_ASSERT(offset < config_cnt_);
        return configs_[offset];
    }

    FakeDeviceConfig& get(pci_bdf_t bdf) {
        return get(bdf.bus_id, bdf.device_id, bdf.function_id);
    }

    uint8_t bus_start() const { return bus_start_; }
    uint8_t bus_end() const { return bus_end_; }
    ddk::MmioBuffer& mmio() { return mmio_; }

    void reset() {
        // Memset optimizations cause faults on uncached memory, so zero out
        // the memory by hand.
        assert(mmio_.get_size() % ZX_PAGE_SIZE == 0);
        assert(mmio_.get_size() % sizeof(uint64_t) == 0);
        assert(reinterpret_cast<uintptr_t>(mmio_.get()) % sizeof(uint64_t) == 0);
        for (size_t i = 0; i < mmio_.get_size(); i += sizeof(uint64_t)) {
            mmio_.Write<uint64_t>(0, i);
        }

        // Mark all vendor & device ids as invalid so that only the devices
        // explicitly configured will be considered in a proper bus scan.
        for (size_t i = 0; i < config_cnt_; i++) {
            configs_[i].device.set_vendor_id(0xffff).set_device_id(0xffff);
        }
    }

private:
    FakeEcam(ddk::MmioBuffer&& mmio, uint8_t bus_start, uint8_t bus_end, size_t cnt)
        : bus_start_(bus_start), bus_end_(bus_end), mmio_(std::move(mmio)),
          configs_(static_cast<FakeDeviceConfig*>(mmio_.get())), config_cnt_(cnt) {
        reset();
    }

    uint8_t bus_start_;
    uint8_t bus_end_;
    ddk::MmioBuffer mmio_;
    FakeDeviceConfig* configs_;
    size_t config_cnt_;
};
