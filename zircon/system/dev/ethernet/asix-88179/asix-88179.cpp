// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/binding.h>
#include <ddk/debug.h>
#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/protocol/ethernet.h>
#include <ddk/protocol/usb.h>
#include <fbl/auto_call.h>
#include <fbl/auto_lock.h>
#include <fbl/unique_ptr.h>
#include <usb/usb.h>
#include <usb/usb-request.h>
#include <lib/cksum.h>
#include <pretty/hexdump.h>
#include <lib/sync/completion.h>
#include <zircon/assert.h>
#include <zircon/listnode.h>

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <threads.h>
#include <unistd.h>

#include "asix-88179.h"
#include "asix-88179-regs.h"

namespace eth {

zx_status_t Asix88179Ethernet::ReadMac(uint8_t reg_addr,
                           uint8_t reg_len,
                           void* data) {
    size_t out_length;
    zx_status_t status = usb_control_in(&usb_, USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                                        AX88179_REQ_MAC, reg_addr, reg_len, ZX_TIME_INFINITE,
                                        data, reg_len, &out_length);
    if (driver_get_log_flags() & DDK_LOG_SPEW) {
        zxlogf(SPEW, "read mac %#x:\n", reg_addr);
        if (status == ZX_OK) {
            hexdump8(data, out_length);
        }
    }
    return status;
}

zx_status_t Asix88179Ethernet::WriteMac(uint8_t reg_addr,
                            uint8_t reg_len,
                            void* data) {
    if (driver_get_log_flags() & DDK_LOG_SPEW) {
        zxlogf(SPEW, "write mac %#x:\n", reg_addr);
        hexdump8(data, reg_len);
    }
    return usb_control_out(&usb_, USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                           AX88179_REQ_MAC, reg_addr, reg_len, ZX_TIME_INFINITE, data, reg_len);
}

zx_status_t Asix88179Ethernet::ReadPhy(uint8_t reg_addr, uint16_t* data) {
    size_t out_length;
    zx_status_t status = usb_control_in(&usb_, USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                                        AX88179_REQ_PHY, AX88179_PHY_ID, reg_addr, ZX_TIME_INFINITE,
                                        data, sizeof(*data), &out_length);
    if (out_length == sizeof(*data)) {
        zxlogf(SPEW, "read phy %#x: %#x\n", reg_addr, *data);
    }
    return status;
}

zx_status_t Asix88179Ethernet::WritePhy(uint8_t reg_addr, uint16_t data) {
    zxlogf(SPEW, "write phy %#x: %#x\n", reg_addr, data);
    return usb_control_out(&usb_, USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                           AX88179_REQ_PHY, AX88179_PHY_ID, reg_addr, ZX_TIME_INFINITE,
                           &data, sizeof(data));
}

static uint8_t ax88179_media_mode[6][2] = {
    { 0x30, 0x01 }, // 10 Mbps, half-duplex
    { 0x32, 0x01 }, // 10 Mbps, full-duplex
    { 0x30, 0x03 }, // 100 Mbps, half-duplex
    { 0x32, 0x03 }, // 100 Mbps, full-duplex
    { 0, 0 },       // unused
    { 0x33, 0x01 }, // 1000Mbps, full-duplex
};

// The array indices here correspond to the bit positions in the AX88179 MAC
// PLSR register.
static uint8_t ax88179_bulk_in_config[5][5][5] = {
    { { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, },
    { // Full Speed
        { 0 },
        { 0x07, 0xcc, 0x4c, 0x18, 0x08 },  // 10 Mbps
        { 0x07, 0xcc, 0x4c, 0x18, 0x08 },  // 100 Mbps
        { 0 },
        { 0x07, 0xcc, 0x4c, 0x18, 0x08 },  // 1000 Mbps
    },
    { // High Speed
        { 0 },
        { 0x07, 0xcc, 0x4c, 0x18, 0x08 },  // 10 Mbps
        { 0x07, 0xae, 0x07, 0x18, 0xff },  // 100 Mbps
        { 0 },
        { 0x07, 0x20, 0x03, 0x16, 0xff },  // 1000 Mbps
    },
    { { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, },
    { // Super Speed
        { 0 },
        { 0x07, 0xcc, 0x4c, 0x18, 0x08 },  // 10 Mbps
        { 0x07, 0xae, 0x07, 0x18, 0xff },  // 100 Mbps
        { 0 },
        { 0x07, 0x4f, 0x00, 0x12, 0xff },  // 1000 Mbps
    },
};

zx_status_t Asix88179Ethernet::ConfigureBulkIn(uint8_t plsr) {
    uint8_t usb_mode = plsr & AX88179_PLSR_USB_MASK;
    if (usb_mode & (usb_mode-1)) {
        zxlogf(ERROR, "ax88179: invalid usb mode: %#x\n", usb_mode);
        return ZX_ERR_INVALID_ARGS;
    }

    uint8_t speed = plsr & AX88179_PLSR_EPHY_MASK;
    if (speed & (speed-1)) {
        zxlogf(ERROR, "ax88179: invalid eth speed: %#x\n", speed);
    }

    zx_status_t status = WriteMac(AX88179_MAC_RQCR, 5,
                                  ax88179_bulk_in_config[usb_mode][speed >> 4]);
    if (status < 0) {
        zxlogf(ERROR, "WriteMac to %#x failed: %d\n", AX88179_MAC_RQCR, status);
    }
    return status;
}

zx_status_t Asix88179Ethernet::ConfigureMediumMode() {
    uint16_t data = 0;
    zx_status_t status = ReadPhy(AX88179_PHY_PHYSR, &data);
    if (status < 0) {
        zxlogf(ERROR, "ReadPhy to %#x failed: %d\n", AX88179_PHY_PHYSR, status);
        return status;
    }

    unsigned int mode = (data & (AX88179_PHYSR_SPEED|AX88179_PHYSR_DUPLEX)) >> 13;
    zxlogf(TRACE, "ax88179 medium mode: %#x\n", mode);
    if (mode == 4 || mode > 5) {
        zxlogf(ERROR, "ax88179 mode invalid (mode=%u)\n", mode);
        return ZX_ERR_NOT_SUPPORTED;
    }
    status = WriteMac(AX88179_MAC_MSR, 2, ax88179_media_mode[mode]);
    if (status < 0) {
        zxlogf(ERROR, "WriteMac to %#x failed: %d\n", AX88179_MAC_MSR, status);
        return status;
    }

    data = 0;
    status = ReadMac(AX88179_MAC_PLSR, 1, &data);
    if (status < 0) {
        zxlogf(ERROR, "ReadMac to %#x failed: %d\n", AX88179_MAC_PLSR, status);
        return status;
    }
    status = ConfigureBulkIn(static_cast<uint8_t>(data & 0xff));

    return status;
}

zx_status_t Asix88179Ethernet::Recv(usb_request_t* request) {
    zxlogf(SPEW, "request len %" PRIu64"\n", request->response.actual);

    if (request->response.actual < 4) {
        zxlogf(ERROR, "Recv short packet\n");
        return ZX_ERR_INTERNAL;
    }

    uint8_t* read_data;
    zx_status_t status = usb_request_mmap(request, reinterpret_cast<void**>(&read_data));
    if (status != ZX_OK) {
        zxlogf(ERROR, "usb_request_mmap failed: %d\n", status);
        return status;
    }

    ptrdiff_t rxhdr_off = request->response.actual - sizeof(ax88179_rx_hdr_t);
    ax88179_rx_hdr_t* rxhdr = (ax88179_rx_hdr_t*)(read_data + rxhdr_off);
    zxlogf(SPEW, "rxhdr offset %u, num %u\n", rxhdr->pkt_hdr_off, rxhdr->num_pkts);
    if (rxhdr->num_pkts < 1 || rxhdr->pkt_hdr_off >= rxhdr_off) {
        zxlogf(ERROR, "%s bad packet\n", __func__);
        return ZX_ERR_IO_DATA_INTEGRITY;
    }

    size_t offset = 0;
    size_t packet = 0;

    while (packet < rxhdr->num_pkts) {
        zxlogf(SPEW, "next packet: %zd\n", packet);
        ptrdiff_t pkt_idx = packet++ * sizeof(uint32_t);
        uint32_t* pkt_hdr = (uint32_t*)(read_data + rxhdr->pkt_hdr_off + pkt_idx);
        if ((uintptr_t)pkt_hdr >= (uintptr_t)rxhdr) {
            zxlogf(ERROR, "%s packet header out of bounds, packet header=%p rx header=%p\n",
                    __func__, pkt_hdr, rxhdr);
            return ZX_ERR_IO_DATA_INTEGRITY;
        }
        uint16_t pkt_len = le16toh((*pkt_hdr & AX88179_RX_PKTLEN) >> 16);
        zxlogf(SPEW, "pkt_hdr: %0#x pkt_len: %u\n", *pkt_hdr, pkt_len);
        if (pkt_len < 2) {
            zxlogf(ERROR, "%s short packet (len=%u)\n", __func__,  pkt_len);
            return ZX_ERR_IO_DATA_INTEGRITY;
        }
        if (offset + pkt_len > rxhdr->pkt_hdr_off) {
            zxlogf(ERROR, "%s invalid packet length %u > %lu bytes remaining\n",
                    __func__, pkt_len, rxhdr->pkt_hdr_off - offset);
            return ZX_ERR_IO_DATA_INTEGRITY;
        }

        bool drop = false;
        if (*pkt_hdr & AX88179_RX_DROPPKT) {
            zxlogf(SPEW, "%s DropPkt\n", __func__);
            drop = true;
        }
        if (*pkt_hdr & AX88179_RX_MIIER) {
            zxlogf(SPEW, "%s MII-Er\n", __func__);
            drop = true;
        }
        if (*pkt_hdr & AX88179_RX_CRCER) {
            zxlogf(SPEW, "%s CRC-Er\n", __func__);
            drop = true;
        }
        if (!(*pkt_hdr & AX88179_RX_OK)) {
            zxlogf(SPEW, "%s !GoodPkt\n", __func__);
            drop = true;
        }
        if (!drop) {
            zxlogf(SPEW, "offset = %zd\n", offset);
            ethmac_ifc_recv(&ifc_, read_data + offset + 2, pkt_len - 2, 0);
        }

        // Advance past this packet in the completed read
        offset += pkt_len;
        offset = ALIGN(offset, 8);
    }

    return ZX_OK;
}

void Asix88179Ethernet::ReadComplete(void* ctx, usb_request_t* request) {
    if (request->response.status == ZX_ERR_IO_NOT_PRESENT) {
        usb_request_release(request);
        return;
    }

    mtx_lock(&mutex_);
    if (request->response.status == ZX_ERR_IO_REFUSED) {
        zxlogf(TRACE, "ReadComplete usb_reset_endpoint\n");
        usb_reset_endpoint(&usb_, bulk_in_addr_);
    } else if (request->response.status == ZX_ERR_IO_INVALID) {
        zxlogf(TRACE, "ReadComplete Slowing down the requests by %d usec"
               " and resetting the recv endpoint\n", ETHMAC_RECV_DELAY);
        if (rx_endpoint_delay_ < ETHMAC_MAX_RECV_DELAY) {
            rx_endpoint_delay_ += ETHMAC_RECV_DELAY;
        }
        usb_reset_endpoint(&usb_, bulk_in_addr_);
    } else if ((request->response.status == ZX_OK) && ifc_.ops) {
      Recv(request);
    }

    if (online_) {
        zx_nanosleep(zx_deadline_after(ZX_USEC(rx_endpoint_delay_)));
        usb_request_complete_t complete = {
            .callback = nullptr,
//TODO: Jamie            .callback = ReadComplete,
            .ctx = this,
        };
        usb_request_queue(&usb_, request, &complete);
    } else {
        zx_status_t status = usb_req_list_add_head(&free_read_reqs_, request,
                                                   parent_req_size_);
        ZX_DEBUG_ASSERT(status == ZX_OK);
    }
    mtx_unlock(&mutex_);
}

zx_status_t Asix88179Ethernet::AppendToTxReq(usb_protocol_t* usb, usb_request_t* req,
                                 ethmac_netbuf_t* netbuf) {
    zx_off_t offset = ALIGN(req->header.length, 4);
    if (offset + sizeof(ax88179_tx_hdr_t) + netbuf->data_size > USB_BUF_SIZE) {
        return ZX_ERR_BUFFER_TOO_SMALL;
    }
    ax88179_tx_hdr_t hdr = {
        .tx_len = htole16(netbuf->data_size),
        .unused = {},
    };
    usb_request_copy_to(req, &hdr, sizeof(hdr), offset);
    usb_request_copy_to(req, netbuf->data_buffer, netbuf->data_size, offset + sizeof(hdr));
    req->header.length = offset + sizeof(hdr) + netbuf->data_size;
    return ZX_OK;
}

void Asix88179Ethernet::WriteComplete(void* ctx, usb_request_t* request) {

    if (request->response.status == ZX_ERR_IO_NOT_PRESENT) {
        usb_request_release(request);
        return;
    }

    zx_status_t status;
    mtx_lock(&tx_lock_);
    ZX_DEBUG_ASSERT(usb_tx_in_flight_ <= MAX_TX_IN_FLIGHT);

    if (!list_is_empty(&pending_netbuf_)) {
        // If we have any pending netbufs, add them to the recently-freed usb request
        request->header.length = 0;
        txn_info_t* next_txn = list_peek_head_type(&pending_netbuf_, txn_info_t, node);
        while (next_txn != NULL && AppendToTxReq(&usb_, request,
                                                 &next_txn->netbuf) == ZX_OK) {
            list_remove_head_type(&pending_netbuf_, txn_info_t, node);
            mtx_lock(&mutex_);
            if (ifc_.ops) {
                ethmac_ifc_complete_tx(&ifc_, &next_txn->netbuf, ZX_OK);
            }
            mtx_unlock(&mutex_);
            next_txn = list_peek_head_type(&pending_netbuf_, txn_info_t, node);
        }
        status = usb_req_list_add_tail(&pending_usb_tx_, request, parent_req_size_);
        ZX_DEBUG_ASSERT(status == ZX_OK);
    } else {
        status = usb_req_list_add_tail(&free_write_reqs_, request, parent_req_size_);
        ZX_DEBUG_ASSERT(status == ZX_OK);
    }

    if (request->response.status == ZX_ERR_IO_REFUSED) {
        zxlogf(TRACE, "WriteComplete usb_reset_endpoint\n");
        usb_reset_endpoint(&usb_, bulk_out_addr_);
    } else if (request->response.status == ZX_ERR_IO_INVALID) {
        zxlogf(TRACE, "WriteComplete Slowing down the requests by %d usec"
               " and resetting the transmit endpoint\n", ETHMAC_TRANSMIT_DELAY);
        if (tx_endpoint_delay_ < ETHMAC_MAX_TRANSMIT_DELAY) {
            tx_endpoint_delay_ += ETHMAC_TRANSMIT_DELAY;
        }
        usb_reset_endpoint(&usb_, bulk_out_addr_);
    }
    usb_request_t* next = usb_req_list_remove_head(&pending_usb_tx_, parent_req_size_);
    if (next == NULL) {
        usb_tx_in_flight_--;
        zxlogf(DEBUG1, "ax88179: no pending write reqs, %u outstanding\n", usb_tx_in_flight_);
    } else {
        zxlogf(DEBUG1, "ax88179: queuing request (%p) of length %lu, %u outstanding\n",
                 next, next->header.length, usb_tx_in_flight_);
        zx_nanosleep(zx_deadline_after(ZX_USEC(tx_endpoint_delay_)));
        usb_request_complete_t complete = {
            .callback = nullptr,
//TODO: Jamie            .callback = WriteComplete,
            .ctx = this,
        };
        usb_request_queue(&usb_, next, &complete);
    }
    ZX_DEBUG_ASSERT(usb_tx_in_flight_ <= MAX_TX_IN_FLIGHT);
    mtx_unlock(&tx_lock_);
}

void Asix88179Ethernet::InterruptComplete(void* ctx, usb_request_t* request) {

    sync_completion_signal(&completion_);
}

void Asix88179Ethernet::HandleInterrupt(usb_request_t* request) {
    mtx_lock(&mutex_);
    if (request->response.status == ZX_OK && request->response.actual == sizeof(status_)) {
        uint8_t status[INTR_REQ_SIZE];

        usb_request_copy_from(request, status, sizeof(status), 0);
        if (memcmp(status_, status, sizeof(status_))) {
            const uint8_t* b = status;
            zxlogf(TRACE, "ax88179 status changed: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                    b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
            memcpy(status_, status, sizeof(status_));
            uint8_t bb = status_[2];
            bool online = (bb & 1) != 0;
            bool was_online = online_;
            online_ = online;
            if (online && !was_online) {
              ConfigureMediumMode();
                // Now that we are online, queue all our read requests
                usb_req_internal_t* req_int;
                usb_req_internal_t* prev;
                usb_request_t* req;
                list_for_every_entry_safe (&free_read_reqs_, req_int, prev, usb_req_internal_t,
                                           node) {
                    list_delete(&req_int->node);
                    req = REQ_INTERNAL_TO_USB_REQ(req_int, parent_req_size_);
                    usb_request_complete_t complete = {
                        .callback = nullptr,
//TODO: Jamie                        .callback = ReadComplete,
                        .ctx = this,
                    };
                    usb_request_queue(&usb_, req, &complete);
                }
                zxlogf(TRACE, "ax88179 now online\n");
                if (ifc_.ops) {
                    ethmac_ifc_status(&ifc_, ETHMAC_STATUS_ONLINE);
                }
            } else if (!online && was_online) {
                zxlogf(TRACE, "ax88179 now offline\n");
                if (ifc_.ops) {
                    ethmac_ifc_status(&ifc_, 0);
                }
            }
        }
    }

    mtx_unlock(&mutex_);
}

zx_status_t Asix88179Ethernet::QueueTx(void* ctx, uint32_t options, ethmac_netbuf_t* netbuf) {
    size_t length = netbuf->data_size;
    txn_info_t* txn = containerof(netbuf, txn_info_t, netbuf);

    if (length > (AX88179_MTU + MAX_ETH_HDRS)) {
        zxlogf(ERROR, "ax88179: unsupported packet length %zu\n", length);
        return ZX_ERR_INVALID_ARGS;
    }

    mtx_lock(&tx_lock_);
    ZX_DEBUG_ASSERT(usb_tx_in_flight_ <= MAX_TX_IN_FLIGHT);

    usb_request_complete_t complete = {
        .callback = nullptr,
//TODO: Jamie        .callback = WriteComplete,
        .ctx = this,
    };
    zx_status_t status;
    usb_request_t* req = NULL;
    usb_req_internal_t* req_int = NULL;

    zx_nanosleep(zx_deadline_after(ZX_USEC(tx_endpoint_delay_)));
    // If we already have entries in our pending_netbuf list we should put this one there, too.
    // Otherwise, we may end up reordering packets.
    if (!list_is_empty(&pending_netbuf_)) {
        goto bufs_full;
    }

    // Find the last entry in the pending_usb_tx list
    if (list_is_empty(&pending_usb_tx_)) {
        zxlogf(DEBUG1, "ax88179: no pending reqs, getting free write req\n");
        req = usb_req_list_remove_head(&free_write_reqs_, parent_req_size_);
        if (req == NULL) {
            goto bufs_full;
        }
        req->header.length = 0;
        status = usb_req_list_add_tail(&pending_usb_tx_, req, parent_req_size_);
        ZX_DEBUG_ASSERT(status == ZX_OK);
    } else {
        req_int = list_peek_tail_type(&pending_usb_tx_, usb_req_internal_t, node);
        req = REQ_INTERNAL_TO_USB_REQ(req_int, parent_req_size_);
        zxlogf(DEBUG1, "ax88179: got tail req (%p)\n", req);
    }

    zxlogf(DEBUG1, "ax88179: current req len=%lu, next packet len=%zu\n",
            req->header.length, length);

    if (AppendToTxReq(&usb_, req, netbuf) == ZX_ERR_BUFFER_TOO_SMALL) {
        // Our data won't fit - grab a new request
        zxlogf(DEBUG1, "ax88179: getting new write req\n");
        req = usb_req_list_remove_head(&free_write_reqs_, parent_req_size_);
        if (req == NULL) {
            goto bufs_full;
        }
        req->header.length = 0;
        status = usb_req_list_add_tail(&pending_usb_tx_, req, parent_req_size_);
        ZX_DEBUG_ASSERT(status == ZX_OK);

      AppendToTxReq(&usb_, req, netbuf);
    } else if (options & ETHMAC_TX_OPT_MORE) {
        // Don't send data if we have more coming that might fit into the current request. If we
        // already filled up a request, though, we should write it out if we can.
        zxlogf(DEBUG1, "ax88179: waiting for more data, %u outstanding\n", usb_tx_in_flight_);
        mtx_unlock(&tx_lock_);
        return ZX_OK;
    }

    if (usb_tx_in_flight_ == MAX_TX_IN_FLIGHT) {
        zxlogf(DEBUG1, "ax88179: max outstanding tx, waiting\n");
        mtx_unlock(&tx_lock_);
        return ZX_OK;
    }
    req = usb_req_list_remove_head(&pending_usb_tx_, parent_req_size_);
    zxlogf(DEBUG1, "ax88179: queuing request (%p) of length %lu, %u outstanding\n",
             req, req->header.length, usb_tx_in_flight_);

    usb_request_queue(&usb_, req, &complete);
    usb_tx_in_flight_++;
    ZX_DEBUG_ASSERT(usb_tx_in_flight_ <= MAX_TX_IN_FLIGHT);
    mtx_unlock(&tx_lock_);
    return ZX_OK;

bufs_full:
    list_add_tail(&pending_netbuf_, &txn->node);
    zxlogf(DEBUG1, "ax88179: buffers full, there are %zu pending netbufs\n",
            list_length(&pending_netbuf_));
    mtx_unlock(&tx_lock_);
    return ZX_ERR_SHOULD_WAIT;
}

void Asix88179Ethernet::Unbind(void* ctx) {
    device_remove(device_);
}

void Asix88179Ethernet::Free() {
    usb_request_t* req;
    while ((req = usb_req_list_remove_head(&free_read_reqs_, parent_req_size_)) != NULL) {
        usb_request_release(req);
    }
    while ((req = usb_req_list_remove_head(&free_write_reqs_, parent_req_size_)) != NULL) {
        usb_request_release(req);
    }
    while ((req = usb_req_list_remove_head(&pending_usb_tx_, parent_req_size_)) != NULL) {
        usb_request_release(req);
    }
    usb_request_release(interrupt_req_);
}

void Asix88179Ethernet::Release(void* ctx) {

    // wait for thread to finish before cleaning up
    thrd_join(thread_, NULL);

  Free();
}

zx_status_t Asix88179Ethernet::Query(void* ctx, uint32_t options, ethmac_info_t* info) {
    if (options) {
        return ZX_ERR_INVALID_ARGS;
    }

    memset(info, 0, sizeof(*info));
    info->mtu = 1500;
    memcpy(info->mac, mac_addr_, sizeof(mac_addr_));
    info->netbuf_size = sizeof(txn_info_t);

    return ZX_OK;
}

void Asix88179Ethernet::Stop(void* ctx) {
    mtx_lock(&mutex_);
    ifc_.ops = NULL;
    mtx_unlock(&mutex_);
}

zx_status_t Asix88179Ethernet::Start(void* ctx, const ethmac_ifc_protocol_t* ifc) {
    zx_status_t status = ZX_OK;

    mtx_lock(&mutex_);
    if (ifc_.ops) {
        status = ZX_ERR_BAD_STATE;
    } else {
        ifc_ = *ifc;
        ethmac_ifc_status(&ifc_, online_ ? ETHMAC_STATUS_ONLINE : 0);
    }
    mtx_unlock(&mutex_);

    return status;
}

zx_status_t Asix88179Ethernet::TwiddleRcrBit(uint16_t bit, bool on) {
    uint16_t rcr_bits;
    zx_status_t status = ReadMac(AX88179_MAC_RCR, 2, &rcr_bits);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ReadMac from %#x failed: %d\n", AX88179_MAC_RCR, status);
        return status;
    }
    if (on) {
        rcr_bits |= bit;
    } else {
        rcr_bits &= static_cast<uint16_t>(~bit);
    }
    status = WriteMac(AX88179_MAC_RCR, 2, &rcr_bits);
    if (status != ZX_OK) {
        zxlogf(ERROR, "WriteMac to %#x failed: %d\n", AX88179_MAC_RCR, status);
    }
    return status;
}

zx_status_t Asix88179Ethernet::SetPromisc(bool on) {
    return TwiddleRcrBit(AX88179_RCR_PROMISC, on);
}

zx_status_t Asix88179Ethernet::SetMulticastPromisc(bool on) {
    if (multicast_filter_overflow_ && !on) {
        return ZX_OK;
    }
    return TwiddleRcrBit(AX88179_RCR_AMALL, on);
}

void Asix88179Ethernet::SetFilterBit(const uint8_t* mac, uint8_t* filter) {
    // Invert the seed (standard is ~0) and output to get usable bits.
    uint32_t crc = ~crc32(0, mac, ETH_MAC_SIZE);
    uint8_t reverse[8] = {0, 4, 2, 6, 1, 5, 3, 7};
    filter[reverse[crc & 7]] |= static_cast<uint8_t>(1 << reverse[(crc >> 3) & 7]);
}

zx_status_t Asix88179Ethernet::SetMulticastFilter(int32_t n_addresses,
                                      const uint8_t* address_bytes,
                                      size_t address_size) {
    zx_status_t status = ZX_OK;
    multicast_filter_overflow_ = (n_addresses == ETHMAC_MULTICAST_FILTER_OVERFLOW) ||
        (n_addresses > MAX_MULTICAST_FILTER_ADDRS);
    if (multicast_filter_overflow_) {
        status = SetMulticastPromisc(true);
        return status;
    }
    if (address_size < n_addresses * ETH_MAC_SIZE)
        return ZX_ERR_OUT_OF_RANGE;

    uint8_t filter[MULTICAST_FILTER_NBYTES];
    memset(filter, 0, MULTICAST_FILTER_NBYTES);
    for (int32_t i = 0; i < n_addresses; i++) {
      SetFilterBit(address_bytes + i * ETH_MAC_SIZE, filter);
    }
    status = WriteMac(AX88179_MAC_MFA, MULTICAST_FILTER_NBYTES, &filter);
    if (status != ZX_OK) {
        zxlogf(ERROR, "WriteMac to %#x failed: %d\n", AX88179_MAC_MFA, status);
        return status;
    }
    return status;
}

zx_status_t Asix88179Ethernet::SetParam(void* ctx,
                                        uint32_t param,
                                        int32_t value,
                                        const void* data,
                                        size_t data_size) {
    zx_status_t status = ZX_OK;

    mtx_lock(&mutex_);

    switch (param) {
    case ETHMAC_SETPARAM_PROMISC:
        status = SetPromisc((bool) value);
        break;
    case ETHMAC_SETPARAM_MULTICAST_PROMISC:
        status = SetMulticastPromisc((bool) value);
        break;
    case ETHMAC_SETPARAM_MULTICAST_FILTER:
        status =
            SetMulticastFilter(value, (const uint8_t*) data, data_size);
        break;
    case ETHMAC_SETPARAM_DUMP_REGS:
      DumpRegs();
        break;
    default:
        status = ZX_ERR_NOT_SUPPORTED;
    }
    mtx_unlock(&mutex_);

    return status;
}

#define READ_REG(r, len) \
    do { \
        reg = 0; \
        zx_status_t status = ReadMac(r, len, &reg); \
        if (status < 0) { \
            zxlogf(ERROR, "ax88179: could not read reg " #r ": %d\n", status); \
        } else { \
            zxlogf(SPEW, "ax88179: reg " #r " = %" PRIx64 "\n", reg); \
        } \
    } while(0)

void Asix88179Ethernet::DumpRegs() {
    uint64_t reg = 0;
  READ_REG(AX88179_MAC_PLSR, 1);
  READ_REG(AX88179_MAC_GSR, 1);
  READ_REG(AX88179_MAC_SMSR, 1);
  READ_REG(AX88179_MAC_CSR, 1);
  READ_REG(AX88179_MAC_RCR, 2);
  READ_REG(AX88179_MAC_MFA, MULTICAST_FILTER_NBYTES);
  READ_REG(AX88179_MAC_IPGCR, 3);
  READ_REG(AX88179_MAC_TR, 1);
  READ_REG(AX88179_MAC_MSR, 2);
  READ_REG(AX88179_MAC_MMSR, 1);
}

int Asix88179Ethernet::Thread(void* arg) {
    uint32_t data = 0;
    uint64_t count = 0;
    usb_request_t* req = interrupt_req_;

    usb_request_complete_t complete = {
        .callback = nullptr,
//TODO: Jamie        .callback = InterruptComplete,
        .ctx = this,
    };

    uint16_t phy_data = 0;

    // Enable embedded PHY
    zx_status_t status = WriteMac(AX88179_MAC_EPPRCR, 2, &data);
    if (status < 0) {
        zxlogf(ERROR, "WriteMac to %#x failed: %d\n", AX88179_MAC_EPPRCR, status);
        goto fail;
    }
    zx_nanosleep(zx_deadline_after(ZX_MSEC(1)));
    data = 0x0020;
    status = WriteMac(AX88179_MAC_EPPRCR, 2, &data);
    if (status < 0) {
        zxlogf(ERROR, "WriteMac to %#x failed: %d\n", AX88179_MAC_EPPRCR, status);
        goto fail;
    }
    zx_nanosleep(zx_deadline_after(ZX_MSEC(200)));

    // Switch clock to normal speed
    data = 0x03;
    status = WriteMac(AX88179_MAC_CLKSR, 1, &data);
    if (status < 0) {
        zxlogf(ERROR, "WriteMac to %#x failed: %d\n", AX88179_MAC_CLKSR, status);
        goto fail;
    }
    zx_nanosleep(zx_deadline_after(ZX_MSEC(1)));

    // Read the MAC addr
    status = ReadMac(AX88179_MAC_NIDR, 6, mac_addr_);
    if (status < 0) {
        zxlogf(ERROR, "ReadMac to %#x failed: %d\n", AX88179_MAC_NIDR, status);
        goto fail;
    }

    zxlogf(INFO, "ax88179 MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n",
            mac_addr_[0], mac_addr_[1], mac_addr_[2],
            mac_addr_[3], mac_addr_[4], mac_addr_[5]);

    ///* Disabled for now
    // Ensure that the MAC RX is disabled
    data = 0;
    status = WriteMac(AX88179_MAC_RCR, 2, &data);
    if (status < 0) {
        zxlogf(ERROR, "WriteMac to %#x failed: %d\n", AX88179_MAC_RCR, status);
        goto fail;
    }
    //*/

    // Set RX Bulk-in sizes -- use USB 3.0/1000Mbps at this point
    status = ConfigureBulkIn(AX88179_PLSR_USB_SS | AX88179_PLSR_EPHY_1000);
    if (status < 0) {
        zxlogf(ERROR, "WriteMac to %#x failed: %d\n", AX88179_MAC_RQCR, status);
        goto fail;
    }

    // Configure flow control watermark
    data = 0x3c;
    status = WriteMac(AX88179_MAC_PWLLR, 1, &data);
    if (status < 0) {
        zxlogf(ERROR, "WriteMac to %#x failed: %d\n", AX88179_MAC_PWLLR, status);
        goto fail;
    }
    data = 0x5c;
    status = WriteMac(AX88179_MAC_PWLHR, 1, &data);
    if (status < 0) {
        zxlogf(ERROR, "WriteMac to %#x failed: %d\n", AX88179_MAC_PWLHR, status);
        goto fail;
    }

    // RX/TX checksum offload: ipv4, tcp, udp, tcpv6, udpv6
    data = (1<<6) | (1<<5) | (1<<2) | (1<<1) | (1<<0);
    status = WriteMac(AX88179_MAC_CRCR, 1, &data);
    if (status < 0) {
        zxlogf(ERROR, "WriteMac to %#x failed: %d\n", AX88179_MAC_CRCR, status);
        goto fail;
    }
    status = WriteMac(AX88179_MAC_CTCR, 1, &data);
    if (status < 0) {
        zxlogf(ERROR, "WriteMac to %#x failed: %d\n", AX88179_MAC_CTCR, status);
        goto fail;
    }

    // TODO: PHY LED

    // PHY auto-negotiation
    status = ReadPhy(AX88179_PHY_BMCR, &phy_data);
    if (status < 0) {
        zxlogf(ERROR, "ReadPhy to %#x failed: %d\n", AX88179_PHY_BMCR, status);
        goto fail;
    }
    phy_data |= 0x1200;
    status = WritePhy(AX88179_PHY_BMCR, phy_data);
    if (status < 0) {
        zxlogf(ERROR, "WritePhy to %#x failed: %d\n", AX88179_PHY_BMCR, status);
        goto fail;
    }

    // Default Ethernet medium mode
    data = 0x013b;
    status = WriteMac(AX88179_MAC_MSR, 2, &data);
    if (status < 0) {
        zxlogf(ERROR, "WriteMac to %#x failed: %d\n", AX88179_MAC_MSR, status);
        goto fail;
    }

    // Enable MAC RX
    // TODO(eventually): Once we get IGMP, turn off AMALL unless someone wants it.
    data = AX88179_RCR_AMALL | AX88179_RCR_AB | AX88179_RCR_AM | AX88179_RCR_SO |
        AX88179_RCR_DROP_CRCE_N | AX88179_RCR_IPE_N;
    status = WriteMac(AX88179_MAC_RCR, 2, &data);
    if (status < 0) {
        zxlogf(ERROR, "WriteMac to %#x failed: %d\n", AX88179_MAC_RCR, status);
        goto fail;
    }

    uint8_t filter[MULTICAST_FILTER_NBYTES];
    memset(filter, 0, MULTICAST_FILTER_NBYTES);
    status = WriteMac(AX88179_MAC_MFA, MULTICAST_FILTER_NBYTES, &filter);
    if (status < 0) {
        zxlogf(ERROR, "WriteMac to %#x failed: %d\n", AX88179_MAC_MFA, status);
        goto fail;
    }

    // Make the device visible
    device_make_visible(device_);

    while (true) {
        sync_completion_reset(&completion_);
        usb_request_queue(&usb_, req, &complete);
        sync_completion_wait(&completion_, ZX_TIME_INFINITE);
        if (req->response.status != ZX_OK) {
            return req->response.status;
        }
        count++;
      HandleInterrupt(req);
#if AX88179_DEBUG_VERBOSE
        if (count % 32 == 0) {
            ax88179_dump_regs();
        }
#endif
    }

fail:
    device_remove(device_);
    return status;
}

zx_status_t Asix88179Ethernet::AddDevice() {
    usb_protocol_t usb;
    zx_status_t result = device_get_protocol(parent(), ZX_PROTOCOL_USB, &usb);
    if (result != ZX_OK) {
        return result;
    }

    // find our endpoints
    usb_desc_iter_t iter;
    result = usb_desc_iter_init(&usb, &iter);
    if (result < 0) return result;

    usb_interface_descriptor_t* intf = usb_desc_iter_next_interface(&iter, true);
    if (!intf || intf->bNumEndpoints != 3) {
        usb_desc_iter_release(&iter);
        return ZX_ERR_NOT_SUPPORTED;
    }

    uint8_t bulk_in_addr = 0;
    uint8_t bulk_out_addr = 0;
    uint8_t intr_addr = 0;
    device_add_args_t args = {};
    int ret = 0;

    usb_endpoint_descriptor_t* endp = usb_desc_iter_next_endpoint(&iter);
    while (endp) {
        if (usb_ep_direction(endp) == USB_ENDPOINT_OUT) {
            if (usb_ep_type(endp) == USB_ENDPOINT_BULK) {
                bulk_out_addr = endp->bEndpointAddress;
            }
        } else {
            if (usb_ep_type(endp) == USB_ENDPOINT_BULK) {
                bulk_in_addr = endp->bEndpointAddress;
            } else if (usb_ep_type(endp) == USB_ENDPOINT_INTERRUPT) {
                intr_addr = endp->bEndpointAddress;
            }
        }
        endp = usb_desc_iter_next_endpoint(&iter);
    }
    usb_desc_iter_release(&iter);

    if (!bulk_in_addr || !bulk_out_addr || !intr_addr) {
        zxlogf(ERROR, "Bind could not find endpoints\n");
        return ZX_ERR_NOT_SUPPORTED;
    }

    list_initialize(&free_read_reqs_);
    list_initialize(&free_write_reqs_);
    list_initialize(&pending_usb_tx_);
    list_initialize(&pending_netbuf_);
    mtx_init(&tx_lock_, mtx_plain);
    mtx_init(&mutex_, mtx_plain);

    usb_device_ = parent();
    memcpy(&usb_, &usb, sizeof(usb_));
    bulk_in_addr_ = bulk_in_addr;
    bulk_out_addr_ = bulk_out_addr;

    parent_req_size_ = usb_get_request_size(&usb_);
    uint64_t req_size = parent_req_size_ + sizeof(usb_req_internal_t);

    rx_endpoint_delay_ = ETHMAC_INITIAL_RECV_DELAY;
    tx_endpoint_delay_ = ETHMAC_INITIAL_TRANSMIT_DELAY;
    zx_status_t status = ZX_OK;
    for (int i = 0; i < READ_REQ_COUNT; i++) {
        usb_request_t* req;
        status = usb_request_alloc(&req, USB_BUF_SIZE, bulk_in_addr, req_size);
        if (status != ZX_OK) {
            goto fail;
        }
        status = usb_req_list_add_head(&free_read_reqs_, req, parent_req_size_);
        ZX_DEBUG_ASSERT(status == ZX_OK);
    }
    for (int i = 0; i < WRITE_REQ_COUNT; i++) {
        usb_request_t* req;
        status = usb_request_alloc(&req, USB_BUF_SIZE, bulk_out_addr, req_size);
        if (status != ZX_OK) {
            goto fail;
        }
        status = usb_req_list_add_head(&free_write_reqs_, req, parent_req_size_);
        ZX_DEBUG_ASSERT(status == ZX_OK);
    }
    usb_request_t* int_req;
    status = usb_request_alloc(&int_req, INTR_REQ_SIZE, intr_addr, req_size);
    if (status != ZX_OK) {
        goto fail;
    }
    interrupt_req_ = int_req;

    /* This is not needed, as long as the xhci stack does it for us.
    status = usb_set_configuration(device, 1);
    if (status < 0) {
        zxlogf(ERROR, "aax88179_bind could not set configuration: %d\n", status);
        return ZX_ERR_NOT_SUPPORTED;
    }
    */

    // Create the device
    /* TODO: Jamie
    args.version = DEVICE_ADD_ARGS_VERSION;
    args.name = "ax88179";
    args.ctx = eth;
    args.ops = &ax88179_device_proto;
    args.flags = DEVICE_ADD_INVISIBLE;
    args.proto_id = ZX_PROTOCOL_ETHMAC;
    args.proto_ops = &ethmac_ops;
    */

    status = device_add(usb_device_, &args, &device_);
    if (status < 0) {
        zxlogf(ERROR, "ax88179: failed to create device: %d\n", status);
        goto fail;
    }


    ret = thrd_create_with_name(&thread_,
        [](void* arg) -> int {
            return static_cast<Asix88179Ethernet*>(arg)->Thread(arg);
        }, this, "asix-88179-thread");
    if (ret != thrd_success) {
        device_remove(device_);
        return ZX_ERR_BAD_STATE;
    }
    return ZX_OK;

fail:
    zxlogf(ERROR, "Bind failed: %d\n", status);
  Free();
    return status;
}

zx_status_t Asix88179Ethernet::Bind(void* ctx, zx_device_t* dev) {
  fbl::AllocChecker ac;
  auto eth_device = fbl::make_unique_checked<Asix88179Ethernet>(&ac, dev);

  if (!ac.check()) {
    return ZX_ERR_NO_MEMORY;
  }

  zx_status_t status;
  if ((status = eth_device->AddDevice()) != ZX_OK) {
      zxlogf(ERROR, "Asix 88179 ethernet driver failed to get added: %d\n", status);
      return status;
  } else {
      zxlogf(INFO, "Asix 88179 ethernet driver added\n");
  }

  // On successful Add, Devmgr takes ownership (relinquished on DdkRelease),
  // so transfer our ownership to a local var, and let it go out of scope.
  auto __UNUSED temp_ref = eth_device.release();

  return ZX_OK;
}

/* TODO: Jamie

static zx_protocol_device_t ax88179_device_proto = []() {
  zx_protocol_device_t dev = {};
  dev.version = DEVICE_OPS_VERSION;
  dev.unbind = Unbind;
  dev.release = Release;

  return dev;
}();


static ethmac_protocol_ops_t ethmac_ops = []() {
  ethmac_protocol_ops_t ops = {};
  ops.query = Query;
  ops.stop = Stop;
  ops.start = Start;
  ops.queue_tx = QueueTx;
  ops.set_param = SetParam;

  return ops;
}();

*/

} // namespace eth

static constexpr zx_driver_ops_t ax88179_driver_ops = []() {
  zx_driver_ops_t ops = {};
  ops.version = DRIVER_OPS_VERSION;
  ops.bind = &eth::Asix88179Ethernet::Bind;

  return ops;
}();

ZIRCON_DRIVER_BEGIN(ethernet_ax88179, ax88179_driver_ops, "zircon", "0.1", 3)
    BI_ABORT_IF(NE, BIND_PROTOCOL, ZX_PROTOCOL_USB),
    BI_ABORT_IF(NE, BIND_USB_VID, ASIX_VID),
    BI_MATCH_IF(EQ, BIND_USB_PID, AX88179_PID),
ZIRCON_DRIVER_END(ethernet_ax88179)
