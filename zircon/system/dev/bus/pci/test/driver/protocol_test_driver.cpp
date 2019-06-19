// Copyright 2019 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#include "protocol_test_driver.h"
#include "../../common.h"
#include "../../config.h"
#include "../fakes/test_device.h"

#include <ddk/binding.h>
#include <ddk/device.h>
#include <ddk/platform-defs.h>
#include <ddktl/protocol/pci.h>
#include <fuchsia/device/test/c/fidl.h>
#include <lib/zx/object.h>
#include <stdio.h>
#include <zircon/hw/pci.h>

ProtocolTestDriver* ProtocolTestDriver::instance_;

TEST_F(PciProtocolTests, TestResetDeviceUnsupported) {
    EXPECT_EQ(pci().ResetDevice(), ZX_ERR_NOT_SUPPORTED);
}

// Do basic reads work in the config header?
TEST_F(PciProtocolTests, ConfigReadHeader) {
    uint16_t rd_val16 = 0;
    ASSERT_OK(pci().ConfigRead16(PCI_CFG_VENDOR_ID, &rd_val16));
    ASSERT_EQ(rd_val16, PCI_TEST_DRIVER_VID);
    ASSERT_OK(pci().ConfigRead16(PCI_CFG_DEVICE_ID, &rd_val16));
    ASSERT_EQ(rd_val16, PCI_TEST_DRIVER_DID);
}

TEST_F(PciProtocolTests, ConfigBounds) {
    uint8_t rd_val8 = 0;
    uint16_t rd_val16 = 0;
    uint32_t rd_val32 = 0;

    // Reads/Writes outside of config space should be invalid.
    ASSERT_EQ(pci().ConfigRead8(PCI_EXT_CONFIG_SIZE, &rd_val8), ZX_ERR_OUT_OF_RANGE);
    ASSERT_EQ(pci().ConfigRead16(PCI_EXT_CONFIG_SIZE, &rd_val16), ZX_ERR_OUT_OF_RANGE);
    ASSERT_EQ(pci().ConfigRead32(PCI_EXT_CONFIG_SIZE, &rd_val32), ZX_ERR_OUT_OF_RANGE);
    ASSERT_EQ(pci().ConfigWrite8(PCI_EXT_CONFIG_SIZE, UINT8_MAX), ZX_ERR_OUT_OF_RANGE);
    ASSERT_EQ(pci().ConfigWrite16(PCI_EXT_CONFIG_SIZE, UINT16_MAX), ZX_ERR_OUT_OF_RANGE);
    ASSERT_EQ(pci().ConfigWrite32(PCI_EXT_CONFIG_SIZE, UINT32_MAX), ZX_ERR_OUT_OF_RANGE);

    // Writes within the config header are not allowed.
    for (uint16_t addr = 0; addr < PCI_CONFIG_HDR_SIZE; addr++) {
        ASSERT_EQ(pci().ConfigWrite8(addr, UINT8_MAX), ZX_ERR_ACCESS_DENIED);
        ASSERT_EQ(pci().ConfigWrite16(addr, UINT16_MAX), ZX_ERR_ACCESS_DENIED);
        ASSERT_EQ(pci().ConfigWrite32(addr, UINT32_MAX), ZX_ERR_ACCESS_DENIED);
    }
}

// A simple offset / pattern for confirming reads and writes.
// Ensuring it never returns 0.
constexpr uint16_t kTestPatternStart = 0x800;
constexpr uint16_t kTestPatternEnd = 0x1000;
constexpr uint8_t TestPatternValue(int address) {
    return static_cast<uint8_t>((address % UINT8_MAX) + 1u);
}

// These pattern tests use ConfigRead/ConfigWrite of all sizes to read and write
// patterns to the back half of the fake device's config space, using the standard
// Pci Protocol methods and the actual device Config object.
TEST_F(PciProtocolTests, ConfigPattern8) {
    uint8_t rd_val = 0;

    // Clear it out. Important if this test runs out of order.
    for (uint16_t addr = kTestPatternStart; addr < kTestPatternEnd; addr++) {
        ASSERT_OK(pci().ConfigWrite8(addr, 0));
    }

    // Verify the clear.
    for (uint16_t addr = kTestPatternStart; addr < kTestPatternEnd; addr++) {
        ASSERT_OK(pci().ConfigRead8(addr, &rd_val));
        ASSERT_EQ(rd_val, 0);
    }

    // Write the pattern out.
    for (uint16_t addr = kTestPatternStart; addr < kTestPatternEnd; addr++) {
        ASSERT_OK(pci().ConfigWrite8(addr, TestPatternValue(addr)));
    }

    // Verify the pattern.
    for (uint16_t addr = kTestPatternStart; addr < kTestPatternEnd; addr++) {
        ASSERT_OK(pci().ConfigRead8(addr, &rd_val));
        ASSERT_EQ(rd_val, TestPatternValue(addr));
    }
}

TEST_F(PciProtocolTests, ConfigPattern16) {
    uint16_t rd_val = 0;
    auto PatternValue = [](uint16_t addr) -> uint16_t {
        return static_cast<uint16_t>((TestPatternValue(addr + 1) << 8) | TestPatternValue(addr));
    };

    // Clear it out. Important if this test runs out of order.
    for (uint16_t addr = kTestPatternStart;
         addr < kTestPatternEnd - 1;
         addr = static_cast<uint16_t>(addr + 2)) {
        ASSERT_OK(pci().ConfigWrite16(addr, 0));
    }

    // Verify the clear.
    for (uint16_t addr = kTestPatternStart;
         addr < kTestPatternEnd - 1;
         addr = static_cast<uint16_t>(addr + 2)) {
        ASSERT_OK(pci().ConfigRead16(addr, &rd_val));
        ASSERT_EQ(rd_val, 0);
    }

    // Write the pattern out.
    for (uint16_t addr = kTestPatternStart;
         addr < kTestPatternEnd - 1;
         addr = static_cast<uint16_t>(addr + 2)) {
        ASSERT_OK(pci().ConfigWrite16(addr, PatternValue(addr)));
    }

    // Verify the pattern.
    for (uint16_t addr = kTestPatternStart;
         addr < kTestPatternEnd - 1;
         addr = static_cast<uint16_t>(addr + 2)) {
        ASSERT_OK(pci().ConfigRead16(addr, &rd_val));
        ASSERT_EQ(rd_val, PatternValue(addr));
    }
}

TEST_F(PciProtocolTests, ConfigPattern32) {
    uint32_t rd_val = 0;
    auto PatternValue = [](uint16_t addr) -> uint32_t {
        return static_cast<uint32_t>((TestPatternValue(addr + 3) << 24)
                                    | (TestPatternValue(addr + 2) << 16)
                                    | (TestPatternValue(addr + 1) << 8)
                                    | TestPatternValue(addr));
    };

    // Clear it out. Important if this test runs out of order.
    for (uint16_t addr = kTestPatternStart;
         addr < kTestPatternEnd - 3;
         addr = static_cast<uint16_t>(addr + 4)) {
        ASSERT_OK(pci().ConfigWrite32(static_cast<uint16_t>(addr), 0));
    }

    // Verify the clear.
    for (uint16_t addr = kTestPatternStart;
         addr < kTestPatternEnd - 3;
         addr = static_cast<uint16_t>(addr + 4)) {
        ASSERT_OK(pci().ConfigRead32(addr, &rd_val));
        ASSERT_EQ(rd_val, 0);
    }

    // Write the pattern out.
    for (uint16_t addr = kTestPatternStart;
         addr < kTestPatternEnd - 3;
         addr = static_cast<uint16_t>(addr + 4)) {
        ASSERT_OK(pci().ConfigWrite32(addr, PatternValue(addr)));
    }

    // Verify the pattern.
    for (uint16_t addr = kTestPatternStart;
         addr < kTestPatternEnd - 3;
         addr = static_cast<uint16_t>(addr + 4)) {
        ASSERT_OK(pci().ConfigRead32(addr, &rd_val));
        ASSERT_EQ(rd_val, PatternValue(addr));
    }
}

TEST_F(PciProtocolTests, EnableBusMaster) {
    struct pci::config::Command cmd_reg = {};
    uint16_t cached_value = 0;

    // Ensure Bus mastering is already enabled in our test quadro.
    ASSERT_OK(pci().ConfigRead16(PCI_CONFIG_COMMAND, &cmd_reg.value));
    ASSERT_EQ(true, cmd_reg.bus_master());
    cached_value = cmd_reg.value; // cache so we can test other bits are preserved

    // Ensure we can disable it.
    ASSERT_OK(pci().EnableBusMaster(false));
    ASSERT_OK(pci().ConfigRead16(PCI_CONFIG_COMMAND, &cmd_reg.value));
    ASSERT_EQ(false, cmd_reg.bus_master());
    ASSERT_EQ(cached_value & ~PCI_COMMAND_BUS_MASTER_EN, cmd_reg.value);

    // Enable and confirm it.
    ASSERT_OK(pci().EnableBusMaster(true));
    ASSERT_OK(pci().ConfigRead16(PCI_CONFIG_COMMAND, &cmd_reg.value));
    ASSERT_EQ(true, cmd_reg.bus_master());
    ASSERT_EQ(cached_value, cmd_reg.value);
}

TEST_F(PciProtocolTests, GetBarArgumentCheck) {
    zx_pci_bar_t info = {};
    // Test that only valid BAR ids are accepted.
    ASSERT_EQ(ZX_ERR_INVALID_ARGS, pci().GetBar(PCI_MAX_BAR_REGS, &info));
}

// These individual BAR tests are coupled closely to the device configuration
// stored in test_device.h. If that configuration is changed in a way that
// affects the expected BAR information then these tests also need to be
// updated.
TEST_F(PciProtocolTests, GetBar0) {
    zx_pci_bar_t info = {};
    zx::vmo vmo;
    size_t size;

    // BAR 0 (32-bit mmio, non-pf, size 16M)
    ASSERT_OK(pci().GetBar(0, &info));
    ASSERT_EQ(info.id, 0);
    ASSERT_EQ(info.type, ZX_PCI_BAR_TYPE_MMIO);
    vmo.reset(info.handle);
    vmo.get_size(&size);
    ASSERT_EQ(size, kTestDeviceBars[0].size);
}

TEST_F(PciProtocolTests, GetBar1) {
    zx_pci_bar_t info = {};
    zx::vmo vmo;
    size_t size;

    // BAR 1 (32-bit mmio, pf, size 256M)
    ASSERT_OK(pci().GetBar(1, &info));
    ASSERT_EQ(info.id, 1);
    ASSERT_EQ(info.type, ZX_PCI_BAR_TYPE_MMIO);
    vmo.reset(info.handle);
    vmo.get_size(&size);
    ASSERT_EQ(size, kTestDeviceBars[1].size);
}

TEST_F(PciProtocolTests, GetBar2) {
    zx_pci_bar_t info = {};
    // BAR 2 (unused)
    ASSERT_EQ(ZX_ERR_NOT_FOUND, pci().GetBar(2, &info));
}

TEST_F(PciProtocolTests, GetBar3) {
    zx_pci_bar_t info = {};
    zx::vmo vmo;
    size_t size;

    // BAR 3 (64-bit mmio, pf, size 32M)
    ASSERT_OK(pci().GetBar(3, &info));
    ASSERT_EQ(info.id, 3);
    ASSERT_EQ(info.type, ZX_PCI_BAR_TYPE_MMIO);
    vmo.reset(info.handle);
    vmo.get_size(&size);
    ASSERT_EQ(size, kTestDeviceBars[3].size);
}

TEST_F(PciProtocolTests, GetBar4) {
    zx_pci_bar_t info = {};
    // BAR 4 (Bar 3 second half, should be NOT_FOUND)
    ASSERT_EQ(ZX_ERR_NOT_FOUND, pci().GetBar(4, &info));
}

TEST_F(PciProtocolTests, GetBar5) {
    zx_pci_bar_t info = {};
    // BAR 5 (I/O ports @ 0x2000, size 128)
    ASSERT_OK(pci().GetBar(5, &info));
    ASSERT_EQ(info.type, ZX_PCI_BAR_TYPE_PIO);
    ASSERT_EQ(info.id, 5);
    ASSERT_EQ(info.addr, kTestDeviceBars[5].address);
    ASSERT_EQ(info.size, kTestDeviceBars[5].size);
}

TEST_F(PciProtocolTests, GetCapabilities) {
    uint8_t offsetA = 0;
    uint8_t offsetB = 0;
    uint8_t val8 = 0;
    // First Power Management Capability is at 0x60.
    ASSERT_OK(pci().GetFirstCapability(0x1, &offsetA));
    ASSERT_EQ(0x60, offsetA);
    ASSERT_OK(pci().ConfigRead8(offsetA, &val8));
    ASSERT_EQ(0x1, val8);

    // Second Power Management Capability is at 0xA0.
    ASSERT_OK(pci().GetNextCapability(0x1, offsetA, &offsetB));
    ASSERT_EQ(0xA0, offsetB);
    ASSERT_OK(pci().ConfigRead8(offsetB, &val8));
    ASSERT_EQ(0x1, val8);
    
    // First Pci Express Capability is at 0x78.
    ASSERT_OK(pci().GetFirstCapability(0x10, &offsetA));
    ASSERT_EQ(0x78, offsetA);
    ASSERT_OK(pci().ConfigRead8(offsetA, &val8));
    ASSERT_EQ(0x10, val8);
    
    // First Vendor Capability is at 0xC4
    ASSERT_OK(pci().GetFirstCapability(0x9, &offsetA));
    ASSERT_EQ(0xC4, offsetA);
    ASSERT_OK(pci().ConfigRead8(offsetA, &val8));
    ASSERT_EQ(0x9, val8);
    
    // Second Vendor Capability is at 0xC8
    ASSERT_OK(pci().GetNextCapability(0x9, offsetA, &offsetB));
    ASSERT_EQ(0xC8, offsetB);
    ASSERT_OK(pci().ConfigRead8(offsetB, &val8));
    ASSERT_EQ(0x9, val8);
    
    // Third Vendor Capability is at 0xD0
    ASSERT_OK(pci().GetNextCapability(0x9, offsetB, &offsetA));
    ASSERT_EQ(0xD0, offsetA);
    ASSERT_OK(pci().ConfigRead8(offsetA, &val8));
    ASSERT_EQ(0x9, val8);
    
    // Fourth Vendor Capability is at 0xE8
    ASSERT_OK(pci().GetNextCapability(0x9, offsetA, &offsetB));
    ASSERT_EQ(0xE8, offsetB);
    ASSERT_OK(pci().ConfigRead8(offsetB, &val8));
    ASSERT_EQ(0x9, val8);
}

TEST_F(PciProtocolTests, GetDeviceInfo) {
    uint16_t vendor_id;
    uint16_t device_id;
    uint8_t base_class;
    uint8_t sub_class;

    uint8_t program_interface;
    uint8_t revision_id;
    uint8_t bus_id = PCI_TEST_BUS_ID;
    uint8_t dev_id = PCI_TEST_DEV_ID;
    uint8_t func_id = PCI_TEST_FUNC_ID;

    ASSERT_OK(pci().ConfigRead16(PCI_CONFIG_VENDOR_ID, &vendor_id));
    ASSERT_OK(pci().ConfigRead16(PCI_CONFIG_DEVICE_ID, &device_id));
    ASSERT_EQ(vendor_id, PCI_TEST_DRIVER_VID);
    ASSERT_EQ(device_id, PCI_TEST_DRIVER_DID);
    ASSERT_OK(pci().ConfigRead8(PCI_CONFIG_CLASS_CODE_BASE, &base_class));
    ASSERT_OK(pci().ConfigRead8(PCI_CONFIG_CLASS_CODE_SUB, &sub_class));
    ASSERT_OK(pci().ConfigRead8(PCI_CONFIG_CLASS_CODE_INTR, &program_interface));
    ASSERT_OK(pci().ConfigRead8(PCI_CONFIG_REVISION_ID, &revision_id));

    zx_pcie_device_info_t info;
    ASSERT_OK(pci().GetDeviceInfo(&info));
    ASSERT_EQ(vendor_id, info.vendor_id);
    ASSERT_EQ(device_id, info.device_id);
    ASSERT_EQ(base_class, info.base_class);
    ASSERT_EQ(sub_class, info.sub_class);
    ASSERT_EQ(program_interface, info.program_interface);
    ASSERT_EQ(revision_id, info.revision_id);
    ASSERT_EQ(bus_id, info.bus_id);
    ASSERT_EQ(dev_id, info.dev_id);
    ASSERT_EQ(func_id, info.func_id);
}

zx_status_t fidl_RunTests(void*, fidl_txn_t* txn) {
    auto driver = ProtocolTestDriver::GetInstance();
    auto zxt = zxtest::Runner::GetInstance();
    zxt->AddObserver(driver);
    RUN_ALL_TESTS(0, nullptr);
    return fuchsia_device_test_DeviceRunTests_reply(txn, ZX_OK, &driver->report());
}

zx_status_t ProtocolTestDriver::DdkMessage(fidl_msg_t* msg, fidl_txn_t* txn) {
    static const fuchsia_device_test_Test_ops_t kOps = {
        .RunTests = fidl_RunTests,
    };

    return fuchsia_device_test_Test_dispatch(this, txn, msg, &kOps);
}

static zx_status_t pci_test_driver_bind(void* ctx, zx_device_t* parent) {
    return ProtocolTestDriver::Create(parent);
}

static zx_driver_ops_t protocol_test_driver_ops = {
    .version = DRIVER_OPS_VERSION,
    .init = nullptr,
    .bind = pci_test_driver_bind,
    .create = nullptr,
    .release = nullptr,
};

// clang-format off
ZIRCON_DRIVER_BEGIN(pci_protocol_test_driver, protocol_test_driver_ops, "zircon", "0.1", 3)
    BI_ABORT_IF(NE, BIND_PROTOCOL, ZX_PROTOCOL_PCI),
    BI_ABORT_IF(NE, BIND_PCI_VID, PCI_TEST_DRIVER_VID),
    BI_MATCH_IF(EQ, BIND_PCI_DID, PCI_TEST_DRIVER_DID),
ZIRCON_DRIVER_END(pci_protocol_test_driver)
