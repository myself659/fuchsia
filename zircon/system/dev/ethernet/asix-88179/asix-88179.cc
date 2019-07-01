// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/binding.h>
#include <ddk/debug.h>
#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/protocol/ethernet.h>
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

zx_status_t Asix88179Ethernet::ReadMac(uint8_t reg_addr, uint8_t reg_len, const void* data) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    size_t out_length;
    zx_status_t status = usb_.ControlIn(USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                                        AX88179_REQ_MAC, reg_addr, reg_len, ZX_TIME_INFINITE,
                                        const_cast<void*>(data), reg_len, &out_length);
    if (driver_get_log_flags() & DDK_LOG_SPEW) {
        zxlogf(SPEW, "ax88179: read mac %#x:\n", reg_addr);
        if (status == ZX_OK) {
            hexdump8(data, out_length);
        }
    }
    return status;
}

zx_status_t Asix88179Ethernet::WriteMac(uint8_t reg_addr, uint8_t reg_len, const void* data) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    if (driver_get_log_flags() & DDK_LOG_SPEW) {
        zxlogf(SPEW, "ax88179: write mac %#x:\n", reg_addr);
        hexdump8(data, reg_len);
    }
    return usb_.ControlOut(USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE, AX88179_REQ_MAC,
        reg_addr, reg_len, ZX_TIME_INFINITE, const_cast<void*>(data), reg_len);
}

zx_status_t Asix88179Ethernet::ReadPhy(uint8_t reg_addr, uint16_t* data) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    size_t out_length;
    zx_status_t status = usb_.ControlIn(USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                                        AX88179_REQ_PHY, AX88179_PHY_ID, reg_addr, ZX_TIME_INFINITE,
                                        data, sizeof(*data), &out_length);
    if (out_length == sizeof(*data)) {
        zxlogf(SPEW, "ax88179: read phy %#x: %#x\n", reg_addr, *data);
    }
    return status;
}

zx_status_t Asix88179Ethernet::WritePhy(uint8_t reg_addr, uint16_t data) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    zxlogf(SPEW, "ax88179: write phy %#x: %#x\n", reg_addr, data);
    return usb_.ControlOut(USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                           AX88179_REQ_PHY, AX88179_PHY_ID, reg_addr, ZX_TIME_INFINITE,
                           &data, sizeof(data));
}

zx_status_t Asix88179Ethernet::ConfigureBulkIn(uint8_t plsr) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
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
                                  kBulkInConfig[usb_mode][speed >> 4]);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: WriteMac to %#x failed: %d\n", AX88179_MAC_RQCR, status);
    }
    return status;
}

zx_status_t Asix88179Ethernet::ConfigureMediumMode() {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    uint16_t data = 0;
    zx_status_t status = ReadPhy(AX88179_PHY_PHYSR, &data);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: ReadPhy to %#x failed: %d\n", AX88179_PHY_PHYSR, status);
        return status;
    }

    unsigned int mode = (data & (AX88179_PHYSR_SPEED|AX88179_PHYSR_DUPLEX)) >> 13;
    zxlogf(TRACE, "ax88179: medium mode: %#x\n", mode);
    if (mode == 4 || mode > 5) {
        zxlogf(ERROR, "ax88179: mode invalid (mode=%u)\n", mode);
        return ZX_ERR_NOT_SUPPORTED;
    }
    status = WriteMac(AX88179_MAC_MSR, 2, kMediaMode[mode]);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: WriteMac to %#x failed: %d\n", AX88179_MAC_MSR, status);
        return status;
    }

    data = 0;
    status = ReadMac(AX88179_MAC_PLSR, 1, &data);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: ReadMac to %#x failed: %d\n", AX88179_MAC_PLSR, status);
        return status;
    }
    status = ConfigureBulkIn(static_cast<uint8_t>(data & 0xff));

    return status;
}

zx_status_t Asix88179Ethernet::Recv(usb_request_t* request) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    zxlogf(SPEW, "ax88179: request len %" PRIu64"\n", request->response.actual);

    if (request->response.actual < 4) {
        zxlogf(ERROR, "ax88179: Recv short packet\n");
        return ZX_ERR_INTERNAL;
    }
    uint8_t* read_data;
    zx_status_t status = usb_request_mmap(request, reinterpret_cast<void**>(&read_data));
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: usb_request_mmap failed: %d\n", status);
        return status;
    }

    ptrdiff_t rxhdr_off = request->response.actual - sizeof(RxHdr);
    RxHdr* rxhdr = reinterpret_cast<RxHdr*>(read_data + rxhdr_off);
    zxlogf(SPEW, "ax88179: rxhdr offset %u, num %u\n", rxhdr->pkt_hdr_off, rxhdr->num_pkts);
    if (rxhdr->num_pkts < 1 || rxhdr->pkt_hdr_off >= rxhdr_off) {
        zxlogf(ERROR, "ax88179: %s bad packet\n", __func__);
        return ZX_ERR_IO_DATA_INTEGRITY;
    }

    size_t offset = 0;
    size_t packet = 0;

    while (packet < rxhdr->num_pkts) {
        zxlogf(SPEW, "ax88179: next packet: %zd\n", packet);
        ptrdiff_t pkt_idx = packet++ * sizeof(uint32_t);
        uint32_t* pkt_hdr = (uint32_t*)(read_data + rxhdr->pkt_hdr_off + pkt_idx);
        if ((uintptr_t)pkt_hdr >= (uintptr_t)rxhdr) {
            zxlogf(ERROR, "ax88179: %s packet header out of bounds, packet header=%p rx header=%p\n",
                    __func__, pkt_hdr, rxhdr);
            return ZX_ERR_IO_DATA_INTEGRITY;
        }
        uint16_t pkt_len = le16toh((*pkt_hdr & AX88179_RX_PKTLEN) >> 16);
        zxlogf(SPEW, "ax88179: pkt_hdr: %0#x pkt_len: %u\n", *pkt_hdr, pkt_len);
        if (pkt_len < 2) {
            zxlogf(ERROR, "ax88179: %s short packet (len=%u)\n", __func__,  pkt_len);
            return ZX_ERR_IO_DATA_INTEGRITY;
        }
        if (offset + pkt_len > rxhdr->pkt_hdr_off) {
            zxlogf(ERROR, "ax88179: %s invalid packet length %u > %lu bytes remaining\n",
                    __func__, pkt_len, rxhdr->pkt_hdr_off - offset);
            return ZX_ERR_IO_DATA_INTEGRITY;
        }

        bool drop = false;
        if (*pkt_hdr & AX88179_RX_DROPPKT) {
            zxlogf(SPEW, "ax88179: %s DropPkt\n", __func__);
            drop = true;
        }
        if (*pkt_hdr & AX88179_RX_MIIER) {
            zxlogf(SPEW, "ax88179: %s MII-Er\n", __func__);
            drop = true;
        }
        if (*pkt_hdr & AX88179_RX_CRCER) {
            zxlogf(SPEW, "ax88179: %s CRC-Er\n", __func__);
            drop = true;
        }
        if (!(*pkt_hdr & AX88179_RX_OK)) {
            zxlogf(SPEW, "ax88179: %s !GoodPkt\n", __func__);
            drop = true;
        }
        if (!drop) {
            zxlogf(SPEW, "ax88179: offset = %zd\n", offset);
            ethmac_ifc_recv(&ifc_, read_data + offset + 2, pkt_len - 2, 0);
        }

        // Advance past this packet in the completed read
        offset += pkt_len;
        offset = ALIGN(offset, 8);
    }

    return ZX_OK;
}

void Asix88179Ethernet::ReadComplete(void* ctx, usb_request_t* request) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    if (request->response.status == ZX_ERR_IO_NOT_PRESENT) {
        usb_request_release(request);
        return;
    }

    fbl::AutoLock lock(&lock_);
    if (request->response.status == ZX_ERR_IO_REFUSED) {
        zxlogf(TRACE, "ax88179: ReadComplete usb_reset_endpoint\n");
        usb_.ResetEndpoint(bulk_in_addr_);
    } else if (request->response.status == ZX_ERR_IO_INVALID) {
        zxlogf(TRACE, "ax88179: ReadComplete Slowing down the requests by %d usec"
               " and resetting the recv endpoint\n", kEthMacRecvDelay);
        if (rx_endpoint_delay_ < kEthMacMaxRecvDelay) {
            rx_endpoint_delay_ += kEthMacRecvDelay;
        }
        usb_.ResetEndpoint(bulk_in_addr_);
    } else if ((request->response.status == ZX_OK) && ifc_.ops) {
      Recv(request);
    }

    if (online_) {
        zx_nanosleep(zx_deadline_after(ZX_USEC(rx_endpoint_delay_)));
        usb_request_complete_t complete = {
            .callback = [](void* ctx, usb_request_t* request) {
                reinterpret_cast<Asix88179Ethernet*>(ctx)->ReadComplete(ctx, request);
            },
            .ctx = this,
        };
        usb_.RequestQueue(request, &complete);
    } else {
        zx_status_t status = usb_req_list_add_head(&free_read_reqs_, request,
                                                   parent_req_size_);
        ZX_DEBUG_ASSERT(status == ZX_OK);
    }
}

zx_status_t Asix88179Ethernet::AppendToTxReq(usb_request_t* req,
                                 ethmac_netbuf_t* netbuf) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    zx_off_t offset = ALIGN(req->header.length, 4);
    if (offset + sizeof(TxHdr) + netbuf->data_size > kUsbBufSize) {
        return ZX_ERR_BUFFER_TOO_SMALL;
    }
    TxHdr hdr = {
        .tx_len = htole16(netbuf->data_size),
        .unused = {},
    };
    usb_request_copy_to(req, &hdr, sizeof(hdr), offset);
    usb_request_copy_to(req, netbuf->data_buffer, netbuf->data_size, offset + sizeof(hdr));
    req->header.length = offset + sizeof(hdr) + netbuf->data_size;
    return ZX_OK;
}

void Asix88179Ethernet::WriteComplete(void* ctx, usb_request_t* request) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    if (request->response.status == ZX_ERR_IO_NOT_PRESENT) {
        usb_request_release(request);
        return;
    }

    zx_status_t status;
    fbl::AutoLock tx_lock(&tx_lock_);
    ZX_DEBUG_ASSERT(usb_tx_in_flight_ <= kMaxTxInFlight);

    if (!list_is_empty(&pending_netbuf_)) {
        // If we have any pending netbufs, add them to the recently-freed usb request
        request->header.length = 0;
        TxnInfo* next_txn = list_peek_head_type(&pending_netbuf_, TxnInfo, node);
        while (next_txn != NULL && AppendToTxReq(request,
                                                 &next_txn->netbuf) == ZX_OK) {
            list_remove_head_type(&pending_netbuf_, TxnInfo, node);
            { // Lock scope
                fbl::AutoLock lock(&lock_);
                if (ifc_.ops) {
                    ethmac_ifc_complete_tx(&ifc_, &next_txn->netbuf, ZX_OK);
                }
            }
            next_txn = list_peek_head_type(&pending_netbuf_, TxnInfo, node);
        }
        status = usb_req_list_add_tail(&pending_usb_tx_, request, parent_req_size_);
        ZX_DEBUG_ASSERT(status == ZX_OK);
    } else {
        status = usb_req_list_add_tail(&free_write_reqs_, request, parent_req_size_);
        ZX_DEBUG_ASSERT(status == ZX_OK);
    }

    if (request->response.status == ZX_ERR_IO_REFUSED) {
        zxlogf(TRACE, "ax88179: WriteComplete usb_reset_endpoint\n");
        usb_.ResetEndpoint(bulk_out_addr_);
    } else if (request->response.status == ZX_ERR_IO_INVALID) {
        zxlogf(TRACE, "ax88179: WriteComplete Slowing down the requests by %d usec"
               " and resetting the transmit endpoint\n", kEthMacTransmitDelay);
        if (tx_endpoint_delay_ < kEthMacMaxTransmitDelay) {
            tx_endpoint_delay_ += kEthMacTransmitDelay;
        }
        usb_.ResetEndpoint(bulk_out_addr_);
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
            .callback = [](void* ctx, usb_request_t* request) {
                reinterpret_cast<Asix88179Ethernet*>(ctx)->WriteComplete(ctx, request);
            },
            .ctx = this,
        };
        usb_.RequestQueue(next, &complete);
    }
    ZX_DEBUG_ASSERT(usb_tx_in_flight_ <= kMaxTxInFlight);
}

void Asix88179Ethernet::RequestComplete(void* ctx, usb_request_t* request) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);

    if (ctx) {
        sync_completion_signal(static_cast<sync_completion_t*>(ctx));
    }
}

void Asix88179Ethernet::HandleInterrupt(usb_request_t* request) {
    fbl::AutoLock lock(&lock_);
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    if (request->response.status == ZX_OK && request->response.actual == sizeof(status_)) {
        uint8_t status[kIntrReqSize];

        usb_request_copy_from(request, status, sizeof(status), 0);
        if (memcmp(status_, status, sizeof(status_))) {
            const uint8_t* b = status;
            zxlogf(TRACE, "ax88179: status changed: %02X %02X %02X %02X %02X %02X %02X %02X\n",
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
                        .callback = [](void* ctx, usb_request_t* request) {
                            reinterpret_cast<Asix88179Ethernet*>(ctx)->ReadComplete(ctx, request);
                        },
                        .ctx = this,
                    };
                    usb_.RequestQueue(req, &complete);
                }
                zxlogf(TRACE, "ax88179: now online\n");
                if (ifc_.ops) {
                    ethmac_ifc_status(&ifc_, ETHMAC_STATUS_ONLINE);
                }
            } else if (!online && was_online) {
                zxlogf(TRACE, "ax88179: now offline\n");
                if (ifc_.ops) {
                    ethmac_ifc_status(&ifc_, 0);
                }
            }
        }
    }
}

zx_status_t Asix88179Ethernet::AddToPendingList(TxnInfo* txn) {
    list_add_tail(&pending_netbuf_, &txn->node);
    zxlogf(DEBUG1, "ax88179: buffers full, there are %zu pending netbufs\n",
        list_length(&pending_netbuf_));

    return ZX_ERR_SHOULD_WAIT;
}

zx_status_t Asix88179Ethernet::EthmacQueueTx(uint32_t options, ethmac_netbuf_t* netbuf) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    size_t length = netbuf->data_size;
    TxnInfo* txn = containerof(netbuf, TxnInfo, netbuf);

    if (length > (kMtu + kMaxEthHdrs)) {
        zxlogf(ERROR, "ax88179: unsupported packet length %zu\n", length);
        return ZX_ERR_INVALID_ARGS;
    }

    fbl::AutoLock tx_lock(&tx_lock_);
    ZX_DEBUG_ASSERT(usb_tx_in_flight_ <= kMaxTxInFlight);

    usb_request_complete_t complete = {
        .callback = [](void* ctx, usb_request_t* request) {
            reinterpret_cast<Asix88179Ethernet*>(ctx)->WriteComplete(ctx, request);
        },
        .ctx = this,
    };
    zx_status_t status;
    usb_request_t* req = NULL;
    usb_req_internal_t* req_int = NULL;

    zx_nanosleep(zx_deadline_after(ZX_USEC(tx_endpoint_delay_)));
    // If we already have entries in our pending_netbuf list we should put this one there, too.
    // Otherwise, we may end up reordering packets.
    if (!list_is_empty(&pending_netbuf_)) {
        return AddToPendingList(txn);
    }

    // Find the last entry in the pending_usb_tx list
    if (list_is_empty(&pending_usb_tx_)) {
        zxlogf(DEBUG1, "ax88179: no pending reqs, getting free write req\n");
        req = usb_req_list_remove_head(&free_write_reqs_, parent_req_size_);
        if (req == NULL) {
            return AddToPendingList(txn);
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

    if (AppendToTxReq(req, netbuf) == ZX_ERR_BUFFER_TOO_SMALL) {
        // Our data won't fit - grab a new request
        zxlogf(DEBUG1, "ax88179: getting new write req\n");
        req = usb_req_list_remove_head(&free_write_reqs_, parent_req_size_);
        if (req == NULL) {
            return AddToPendingList(txn);
        }
        req->header.length = 0;
        status = usb_req_list_add_tail(&pending_usb_tx_, req, parent_req_size_);
        ZX_DEBUG_ASSERT(status == ZX_OK);

      AppendToTxReq(req, netbuf);
    } else if (options & ETHMAC_TX_OPT_MORE) {
        // Don't send data if we have more coming that might fit into the current request. If we
        // already filled up a request, though, we should write it out if we can.
        zxlogf(DEBUG1, "ax88179: waiting for more data, %u outstanding\n", usb_tx_in_flight_);
        return ZX_OK;
    }

    if (usb_tx_in_flight_ == kMaxTxInFlight) {
        zxlogf(DEBUG1, "ax88179: max outstanding tx, waiting\n");
        return ZX_OK;
    }
    req = usb_req_list_remove_head(&pending_usb_tx_, parent_req_size_);
    zxlogf(DEBUG1, "ax88179: queuing request (%p) of length %lu, %u outstanding\n",
             req, req->header.length, usb_tx_in_flight_);

    usb_.RequestQueue(req, &complete);
    usb_tx_in_flight_++;
    ZX_DEBUG_ASSERT(usb_tx_in_flight_ <= kMaxTxInFlight);
    return ZX_OK;
}

void Asix88179Ethernet::Unbind(void* ctx) {
    zxlogf(INFO, "ax88179: unbind\n");
    device_remove(device_);
}

void Asix88179Ethernet::Free() {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
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

void Asix88179Ethernet::DdkRelease() {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);

    // wait for thread to finish before cleaning up
    thrd_join(thread_, NULL);

    Free();

    delete this;
}

zx_status_t Asix88179Ethernet::EthmacQuery(uint32_t options, ethmac_info_t* info) {
    zxlogf(INFO, "ax88179: in %s:\n", __func__);
    if (options) {
        return ZX_ERR_INVALID_ARGS;
    }

    memset(info, 0, sizeof(*info));
    info->mtu = 1500;
    memcpy(info->mac, mac_addr_, sizeof(mac_addr_));
    info->netbuf_size = sizeof(TxnInfo);

    return ZX_OK;
}

void Asix88179Ethernet::EthmacStop() {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    fbl::AutoLock lock(&lock_);
    ifc_.ops = NULL;
}

zx_status_t Asix88179Ethernet::EthmacStart(const ethmac_ifc_protocol_t* ifc) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    zx_status_t status = ZX_OK;

    fbl::AutoLock lock(&lock_);
    if (ifc_.ops) {
        status = ZX_ERR_BAD_STATE;
    } else {
        ifc_ = *ifc;
        ethmac_ifc_status(&ifc_, online_ ? ETHMAC_STATUS_ONLINE : 0);
    }

    return status;
}

zx_status_t Asix88179Ethernet::TwiddleRcrBit(uint16_t bit, bool on) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    uint16_t rcr_bits = 0;
    zx_status_t status = ReadMac(AX88179_MAC_RCR, 2, &rcr_bits);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: ReadMac from %#x failed: %d\n", AX88179_MAC_RCR, status);
        return status;
    }
    if (on) {
        rcr_bits |= bit;
    } else {
        rcr_bits &= static_cast<uint16_t>(~bit);
    }
    status = WriteMac(AX88179_MAC_RCR, 2, &rcr_bits);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: WriteMac to %#x failed: %d\n", AX88179_MAC_RCR, status);
    }
    return status;
}

zx_status_t Asix88179Ethernet::SetPromisc(bool on) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    return TwiddleRcrBit(AX88179_RCR_PROMISC, on);
}

zx_status_t Asix88179Ethernet::SetMulticastPromisc(bool on) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    if (multicast_filter_overflow_ && !on) {
        return ZX_OK;
    }
    return TwiddleRcrBit(AX88179_RCR_AMALL, on);
}

void Asix88179Ethernet::SetFilterBit(const uint8_t* mac, uint8_t* filter) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    // Invert the seed (standard is ~0) and output to get usable bits.
    uint32_t crc = ~crc32(0, mac, ETH_MAC_SIZE);
    uint8_t reverse[8] = {0, 4, 2, 6, 1, 5, 3, 7};
    filter[reverse[crc & 7]] |= static_cast<uint8_t>(1 << reverse[(crc >> 3) & 7]);
}

zx_status_t Asix88179Ethernet::SetMulticastFilter(int32_t n_addresses,
                                      const uint8_t* address_bytes,
                                      size_t address_size) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    zx_status_t status = ZX_OK;
    multicast_filter_overflow_ = (n_addresses == ETHMAC_MULTICAST_FILTER_OVERFLOW) ||
        (n_addresses > kMaxMulticastFilterAddrs);
    if (multicast_filter_overflow_) {
        status = SetMulticastPromisc(true);
        return status;
    }
    if (address_size < n_addresses * ETH_MAC_SIZE)
        return ZX_ERR_OUT_OF_RANGE;

    uint8_t filter[kMulticastFilterNBytes];
    memset(filter, 0, kMulticastFilterNBytes);
    for (int32_t i = 0; i < n_addresses; i++) {
      SetFilterBit(address_bytes + i * ETH_MAC_SIZE, filter);
    }
    status = WriteMac(AX88179_MAC_MFA, kMulticastFilterNBytes, &filter);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: WriteMac to %#x failed: %d\n", AX88179_MAC_MFA, status);
        return status;
    }
    return status;
}

zx_status_t Asix88179Ethernet::EthmacSetParam(uint32_t param,
                                        int32_t value,
                                        const void* data,
                                        size_t data_size) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    zx_status_t status = ZX_OK;

    fbl::AutoLock lock(&lock_);

    switch (param) {
    case ETHMAC_SETPARAM_PROMISC:
        status = SetPromisc(static_cast<bool>(value));
        break;
    case ETHMAC_SETPARAM_MULTICAST_PROMISC:
        status = SetMulticastPromisc(static_cast<bool>(value));
        break;
    case ETHMAC_SETPARAM_MULTICAST_FILTER:
        status =
            SetMulticastFilter(value, static_cast<const uint8_t*>(data), data_size);
        break;
    case ETHMAC_SETPARAM_DUMP_REGS:
      DumpRegs();
        break;
    default:
        status = ZX_ERR_NOT_SUPPORTED;
    }

    return status;
}

#define READ_REG(r, len) \
    do { \
        reg = 0; \
        zx_status_t status = ReadMac(r, len, &reg); \
        if (status != ZX_OK) { \
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
  READ_REG(AX88179_MAC_MFA, kMulticastFilterNBytes);
  READ_REG(AX88179_MAC_IPGCR, 3);
  READ_REG(AX88179_MAC_TR, 1);
  READ_REG(AX88179_MAC_MSR, 2);
  READ_REG(AX88179_MAC_MMSR, 1);
}

int Asix88179Ethernet::Thread() {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    uint32_t data = 0;
    usb_request_t* req = interrupt_req_;
    uint16_t phy_data = 0;

    // Enable embedded PHY
    zx_status_t status = WriteMac(AX88179_MAC_EPPRCR, 2, &data);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: WriteMac to %#x failed: %d\n", AX88179_MAC_EPPRCR, status);
        return status;
    }
    zx_nanosleep(zx_deadline_after(ZX_MSEC(1)));
    data = 0x0020;
    status = WriteMac(AX88179_MAC_EPPRCR, 2, &data);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: WriteMac to %#x failed: %d\n", AX88179_MAC_EPPRCR, status);
        return status;
    }
    zx_nanosleep(zx_deadline_after(ZX_MSEC(200)));

    // Switch clock to normal speed
    data = 0x03;
    status = WriteMac(AX88179_MAC_CLKSR, 1, &data);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: WriteMac to %#x failed: %d\n", AX88179_MAC_CLKSR, status);
        return status;
    }
    zx_nanosleep(zx_deadline_after(ZX_MSEC(1)));

    // Read the MAC addr
    status = ReadMac(AX88179_MAC_NIDR, 6, mac_addr_);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: ReadMac to %#x failed: %d\n", AX88179_MAC_NIDR, status);
        return status;
    }

    zxlogf(INFO, "ax88179: MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n",
            mac_addr_[0], mac_addr_[1], mac_addr_[2],
            mac_addr_[3], mac_addr_[4], mac_addr_[5]);

    ///* Disabled for now
    // Ensure that the MAC RX is disabled
    data = 0;
    status = WriteMac(AX88179_MAC_RCR, 2, &data);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: WriteMac to %#x failed: %d\n", AX88179_MAC_RCR, status);
        return status;
    }
    //*/

    // Set RX Bulk-in sizes -- use USB 3.0/1000Mbps at this point
    status = ConfigureBulkIn(AX88179_PLSR_USB_SS | AX88179_PLSR_EPHY_1000);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: WriteMac to %#x failed: %d\n", AX88179_MAC_RQCR, status);
        return status;
    }

    // Configure flow control watermark
    data = 0x3c;
    status = WriteMac(AX88179_MAC_PWLLR, 1, &data);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: WriteMac to %#x failed: %d\n", AX88179_MAC_PWLLR, status);
        return status;
    }
    data = 0x5c;
    status = WriteMac(AX88179_MAC_PWLHR, 1, &data);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: WriteMac to %#x failed: %d\n", AX88179_MAC_PWLHR, status);
        return status;
    }

    // RX/TX checksum offload: ipv4, tcp, udp, tcpv6, udpv6
    data = (1<<6) | (1<<5) | (1<<2) | (1<<1) | (1<<0);
    status = WriteMac(AX88179_MAC_CRCR, 1, &data);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: WriteMac to %#x failed: %d\n", AX88179_MAC_CRCR, status);
        return status;
    }
    status = WriteMac(AX88179_MAC_CTCR, 1, &data);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: WriteMac to %#x failed: %d\n", AX88179_MAC_CTCR, status);
        return status;
    }

    // TODO: PHY LED

    // PHY auto-negotiation
    status = ReadPhy(AX88179_PHY_BMCR, &phy_data);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: ReadPhy to %#x failed: %d\n", AX88179_PHY_BMCR, status);
        return status;
    }
    phy_data |= 0x1200;
    status = WritePhy(AX88179_PHY_BMCR, phy_data);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: WritePhy to %#x failed: %d\n", AX88179_PHY_BMCR, status);
        return status;
    }

    // Default Ethernet medium mode
    data = 0x013b;
    status = WriteMac(AX88179_MAC_MSR, 2, &data);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: WriteMac to %#x failed: %d\n", AX88179_MAC_MSR, status);
        return status;
    }

    // Enable MAC RX
    // TODO(eventually): Once we get IGMP, turn off AMALL unless someone wants it.
    data = AX88179_RCR_AMALL | AX88179_RCR_AB | AX88179_RCR_AM | AX88179_RCR_SO |
        AX88179_RCR_DROP_CRCE_N | AX88179_RCR_IPE_N;
    status = WriteMac(AX88179_MAC_RCR, 2, &data);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: WriteMac to %#x failed: %d\n", AX88179_MAC_RCR, status);
        return status;
    }

    uint8_t filter[kMulticastFilterNBytes];
    memset(filter, 0, kMulticastFilterNBytes);
    status = WriteMac(AX88179_MAC_MFA, kMulticastFilterNBytes, &filter);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: WriteMac to %#x failed: %d\n", AX88179_MAC_MFA, status);
        return status;
    }

    // Make the device visible
    DdkMakeVisible();

    while (true) {
        sync_completion_t completion;
        usb_request_complete_t complete = {
            .callback = [](void* ctx, usb_request_t* request) {
                sync_completion_signal(reinterpret_cast<sync_completion_t*>(ctx));
            },
            .ctx = &completion,
        };
        usb_.RequestQueue(req, &complete);
        sync_completion_wait(&completion, ZX_TIME_INFINITE);
        if (req->response.status != ZX_OK) {
            zxlogf(ERROR, "ax88179: USB request queue failed: %d\n", status);
            return req->response.status;
        }
        HandleInterrupt(req);
    }

    return 0;
}

void Asix88179Ethernet::DdkUnbind() {
    zxlogf(INFO, "ax88179: Unbind\n");
    //TODO: Jamie stop the thread ShutDown();
    DdkRemove();
}

zx_status_t Asix88179Ethernet::Init() {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
    zx_status_t status;
    int ret = 0;

    usb::UsbDevice usb(parent());
    if (!usb.is_valid()) {
        return ZX_ERR_PROTOCOL_NOT_SUPPORTED;
    }

    // find our endpoints
    std::optional<usb::InterfaceList> usb_interface_list;
    status = usb::InterfaceList::Create(usb, true, &usb_interface_list);
    if (status != ZX_OK) {
        return status;
    }

    auto interface = usb_interface_list->begin();
    const usb_interface_descriptor_t* interface_descriptor = interface->descriptor();
    if (interface == usb_interface_list->end()) {
        return ZX_ERR_NOT_SUPPORTED;
    }
    if (interface_descriptor->bNumEndpoints < 3) {
        zxlogf(ERROR, "ax88179: Wrong number of endpoints: %d\n", interface_descriptor->bNumEndpoints);
        return ZX_ERR_NOT_SUPPORTED;
    }

    interface_number_ = interface_descriptor->bInterfaceNumber;
    uint8_t bulk_in_addr = 0;
    uint8_t bulk_out_addr = 0;
    uint8_t intr_addr = 0;

    for (auto endpoint : *interface) {
        usb_endpoint_descriptor_t* endp = &endpoint.descriptor;
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
    }

    if (!bulk_in_addr || !bulk_out_addr || !intr_addr) {
        zxlogf(ERROR, "ax88179: Bind could not find endpoints\n");
        return ZX_ERR_NOT_SUPPORTED;
    }

    list_initialize(&free_read_reqs_);
    list_initialize(&free_write_reqs_);
    list_initialize(&pending_usb_tx_);
    list_initialize(&pending_netbuf_);

    usb_ = usb;
    bulk_in_addr_ = bulk_in_addr;
    bulk_out_addr_ = bulk_out_addr;

    parent_req_size_ = usb_.GetRequestSize();
    uint64_t req_size = parent_req_size_ + sizeof(usb_request_complete_t);

    rx_endpoint_delay_ = kEthMacInitialRecvDelay;
    tx_endpoint_delay_ = kEthMacInitialTransmitDelay;

    for (int i = 0; i < kReadReqCount; i++) {
        usb_request_t* req;
        status = usb_request_alloc(&req, kUsbBufSize, bulk_in_addr, req_size);
        if (status != ZX_OK) {
            Free();
            return status;
        }
        status = usb_req_list_add_head(&free_read_reqs_, req, parent_req_size_);
        ZX_DEBUG_ASSERT(status == ZX_OK);
    }
    for (int i = 0; i < kWriteReqCount; i++) {
        usb_request_t* req;
        status = usb_request_alloc(&req, kUsbBufSize, bulk_out_addr, req_size);
        if (status != ZX_OK) {
            Free();
            return status;
        }
        status = usb_req_list_add_head(&free_write_reqs_, req, parent_req_size_);
        ZX_DEBUG_ASSERT(status == ZX_OK);
    }
    usb_request_t* int_req;
    status = usb_request_alloc(&int_req, kIntrReqSize, intr_addr, req_size);
    if (status != ZX_OK) {
        Free();
        return status;
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
    /*
    device_add_args_t args = {
        .version = DEVICE_ADD_ARGS_VERSION,
        .name = "ax88179",
        .ctx = this,
        .ops = &ddk_device_proto_,
        .flags = DEVICE_ADD_INVISIBLE,
        .proto_id = ZX_PROTOCOL_ETHMAC,
        .proto_ops = &eth_mac_protocol_ops_,
    };

    status = device_add(eth->usb_device, &args, &eth->device);
    if (status < 0) {
        zxlogf(ERROR, "ax88179: failed to create device: %d\n", status);
        goto fail;
    }
*/
    status = DdkAdd("ax88179", DEVICE_ADD_INVISIBLE);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ax88179: failed to create device: %d\n", status);
        Free();
        return status;
    }

    ret = thrd_create_with_name(&thread_,
        [](void* arg) -> int {
            return static_cast<Asix88179Ethernet*>(arg)->Thread();
        }, this, "asix-88179-thread");
    ZX_DEBUG_ASSERT(ret == thrd_success);

    return ZX_OK;
}

zx_status_t Asix88179Ethernet::Bind(void* ctx, zx_device_t* dev) {
    zxlogf(SPEW, "ax88179: in %s:\n", __func__);
  fbl::AllocChecker ac;
  auto eth_device = fbl::make_unique_checked<Asix88179Ethernet>(&ac, dev);

  if (!ac.check()) {
    return ZX_ERR_NO_MEMORY;
  }

  zx_status_t status;
  if ((status = eth_device->Init()) != ZX_OK) {
      zxlogf(ERROR, "ax88179: ethernet driver failed to get added: %d\n", status);
      return status;
  } else {
      zxlogf(INFO, "ax88179: ethernet driver added\n");
  }

  // On successful Add, Devmgr takes ownership (relinquished on DdkRelease),
  // so transfer our ownership to a local var, and let it go out of scope.
  auto __UNUSED temp_ref = eth_device.release();

  return ZX_OK;
}

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
