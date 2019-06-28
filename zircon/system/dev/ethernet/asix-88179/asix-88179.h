// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/device.h>
#include <ddktl/device.h>
#include <lib/device-protocol/pdev.h>
#include <ddktl/protocol/ethernet.h>
#include <ddktl/protocol/ethernet/board.h>
#include <ddktl/protocol/ethernet/mac.h>

#include <optional>

namespace eth {

#define READ_REQ_COUNT 8
#define WRITE_REQ_COUNT 8
#define USB_BUF_SIZE 24576
#define MAX_TX_IN_FLIGHT 4
#define INTR_REQ_SIZE 8
#define RX_HEADER_SIZE 4
#define AX88179_MTU 1500
#define MAX_ETH_HDRS 26
#define MAX_MULTICAST_FILTER_ADDRS 32
#define MULTICAST_FILTER_NBYTES 8

/*
 * The constants are determined based on Pluggable gigabit Ethernet adapter(Model: USBC-E1000),
 * connected on pixelbook. At times, the device returns NRDY token when it is unable to match the
 * pace of the client driver but does not recover by sending a ERDY token within the controller's
 * time limit. ETHMAC_INITIAL_TRANSMIT_DELAY helps us avoids getting into this situation by adding
 * a delay at the beginning.
 */
#define ETHMAC_MAX_TRANSMIT_DELAY 100
#define ETHMAC_MAX_RECV_DELAY 100
#define ETHMAC_TRANSMIT_DELAY 10
#define ETHMAC_RECV_DELAY 10
#define ETHMAC_INITIAL_TRANSMIT_DELAY 15
#define ETHMAC_INITIAL_RECV_DELAY 0

typedef struct {
  uint16_t num_pkts;
  uint16_t pkt_hdr_off;
} ax88179_rx_hdr_t;

typedef struct {
  uint16_t tx_len;
  uint16_t unused[3];
  // TODO: support additional tx header fields
} ax88179_tx_hdr_t;

typedef struct txn_info {
  ethmac_netbuf_t netbuf;
  list_node_t node;
} txn_info_t;

class Asix88179Ethernet;

using DeviceType = ddk::Device<Asix88179Ethernet, ddk::Unbindable>;

class Asix88179Ethernet : public DeviceType,
                    public ddk::EthmacProtocol<Asix88179Ethernet, ddk::base_protocol> {
public:
    DISALLOW_COPY_ASSIGN_AND_MOVE(Asix88179Ethernet);

    explicit Asix88179Ethernet(zx_device_t* parent)
        : DeviceType(parent) {}

    static zx_status_t Create(void* ctx, zx_device_t* parent);

    // DDK Hooks.
    void DdkRelease();
    void DdkUnbind();

    zx_status_t EthmacQuery(uint32_t options, ethmac_info_t* info);
    void EthmacStop() __TA_EXCLUDES(lock_);
    zx_status_t EthmacStart(const ethmac_ifc_protocol_t* ifc) __TA_EXCLUDES(lock_);
    zx_status_t EthmacQueueTx(uint32_t options, ethmac_netbuf_t* netbuf) __TA_EXCLUDES(lock_);
    zx_status_t EthmacSetParam(uint32_t param, int32_t value, const void* data, size_t data_size);
    void EthmacGetBti(zx::bti* bti) { bti->reset(); }

    static zx_status_t Bind(void* ctx, zx_device_t* device);

 private:
    zx_status_t Init();

    zx_status_t ReadMac(uint8_t reg_addr,
                                           uint8_t reg_len,
                                           void* data);

    zx_status_t WriteMac(uint8_t reg_addr,
                                            uint8_t reg_len,
                                            void* data);

    zx_status_t ReadPhy(uint8_t reg_addr, uint16_t* data);

    zx_status_t WritePhy(uint8_t reg_addr, uint16_t data);

    zx_status_t ConfigureBulkIn(uint8_t plsr);

    zx_status_t ConfigureMediumMode();

    zx_status_t Recv(usb_request_t* request);

    void ReadComplete(void* ctx, usb_request_t* request);

    zx_status_t AppendToTxReq(usb_request_t* req, ethmac_netbuf_t* netbuf);

    void WriteComplete(void* ctx, usb_request_t* request);

    void RequestComplete(void* ctx, usb_request_t* request);

    void HandleInterrupt( usb_request_t* request);

    void Unbind(void* ctx);

    void Free();

    zx_status_t TwiddleRcrBit(uint16_t bit, bool on);

    zx_status_t SetPromisc(bool on);

    zx_status_t SetMulticastPromisc(bool on);

    void SetFilterBit(const uint8_t* mac, uint8_t* filter);

    zx_status_t SetMulticastFilter(int32_t n_addresses,
                                                      const uint8_t* address_bytes,
                                                      size_t address_size);

    void DumpRegs();

    int Thread();

    zx_device_t* device_;

    usb::UsbDevice usb_;

/*TODO: Jamie - delete
    zx_device_t* usb_device_;
    usb_protocol_t usb_;
*/

    uint8_t mac_addr_[ETH_MAC_SIZE];
    uint8_t status_[INTR_REQ_SIZE];
    bool online_;
    bool multicast_filter_overflow_;
    uint8_t bulk_in_addr_;
    uint8_t bulk_out_addr_;
    uint8_t interface_number_;

    // interrupt in request
    usb_request_t* interrupt_req_;

    // pool of free USB bulk requests
    list_node_t free_read_reqs_;
    list_node_t free_write_reqs_;

    // Locks the usb_tx_in_flight, pending_usb_tx, and pending_netbuf lists.
    fbl::Mutex tx_mutex_;

    // Whether a request has been queued to the USB device.
    uint8_t usb_tx_in_flight_;
    // List of requests that have pending data. Used to buffer data if a USB transaction is in
    // flight. Additional data must be appended to the tail of the list, or if that's full, a
    // request from free_write_reqs must be added to the list.
    list_node_t pending_usb_tx_;
    // List of netbufs that haven't been copied into a USB transaction yet. Should only contain
    // entries if all allocated USB transactions are full.
    list_node_t pending_netbuf_;

    uint64_t rx_endpoint_delay_;    // wait time between 2 recv requests
    uint64_t tx_endpoint_delay_;    // wait time between 2 transmit requests
    // callback interface to attached ethernet layer
    ethmac_ifc_protocol_t ifc_;

    size_t parent_req_size_;
    thrd_t thread_;
    fbl::Mutex mutex_;
};

} // namespace eth
