// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddktl/fidl.h>
#include <fbl/auto_lock.h>
#include <usb/usb.h>
#include <usb/usb-request.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <threads.h>

#include "midi.h"
#include "usb-audio.h"
#include "usb-midi-source.h"

namespace audio {
namespace usb {

constexpr size_t READ_REQ_COUNT = 20;

void UsbMidiSource::UpdateSignals() {
    zx_signals_t new_signals = 0;
    if (dead_) {
        new_signals |= (DEV_STATE_READABLE | DEV_STATE_ERROR);
    } else if (!list_is_empty(&completed_reads_)) {
        new_signals |= DEV_STATE_READABLE;
    }
    if (new_signals != signals_) {
        ClearAndSetState(signals_ & ~new_signals, new_signals & ~signals_);
        signals_ = new_signals;
    }
}

void UsbMidiSource::ReadComplete(usb_request_t* req) {
    if (req->response.status == ZX_ERR_IO_NOT_PRESENT) {
        usb_request_release(req);
        return;
    }

    fbl::AutoLock lock(&mutex_);

    if (req->response.status == ZX_OK && req->response.actual > 0) {
        zx_status_t status = usb_req_list_add_tail(&completed_reads_, req,
                                                   parent_req_size_);
        ZX_DEBUG_ASSERT(status == ZX_OK);
    } else {
        usb_request_complete_t complete = {
            .callback = [](void* ctx, usb_request_t* req) { 
                static_cast<UsbMidiSource*>(ctx)->ReadComplete(req);
            },
            .ctx = this,
        };
        usb_.RequestQueue(req, &complete);
    }
    UpdateSignals();
}

void UsbMidiSource::DdkUnbind() {
    dead_ = true;
    UpdateSignals();
    DdkRemove();
}

UsbMidiSource::~UsbMidiSource() {
    usb_request_t* req;
    while ((req = usb_req_list_remove_head(&free_read_reqs_,
                                           parent_req_size_)) != NULL) {
        usb_request_release(req);
    }
    while ((req = usb_req_list_remove_head(&completed_reads_,
                                           parent_req_size_)) != NULL) {
        usb_request_release(req);
    }
}

void UsbMidiSource::DdkRelease() {
    delete this;
}

zx_status_t UsbMidiSource::DdkOpen(zx_device_t** dev_out, uint32_t flags) {
    zx_status_t result;

    fbl::AutoLock lock(&mutex_);
    if (open_) {
        result = ZX_ERR_ALREADY_BOUND;
    } else {
        open_ = true;
        result = ZX_OK;
    }

    // queue up reads, including stale completed reads
    usb_request_complete_t complete = {
        .callback = [](void* ctx, usb_request_t* req) { 
            static_cast<UsbMidiSource*>(ctx)->ReadComplete(req);
        },
        .ctx = this,
    };
    usb_request_t* req;
    while ((req = usb_req_list_remove_head(&completed_reads_,
                                           parent_req_size_)) != NULL) {
        usb_.RequestQueue(req, &complete);
    }
    while ((req = usb_req_list_remove_head(&free_read_reqs_,
                                           parent_req_size_)) != NULL) {
        usb_.RequestQueue(req, &complete);
    }

    return result;
}

zx_status_t UsbMidiSource::DdkClose(uint32_t flags) {
    fbl::AutoLock lock(&mutex_);
    open_ = false;

    return ZX_OK;
}

zx_status_t UsbMidiSource::DdkRead(void* data, size_t len, zx_off_t off, size_t* actual) {
    if (dead_) {
        return ZX_ERR_IO_NOT_PRESENT;
    }

    zx_status_t status = ZX_OK;
    if (len < 3) {
        return ZX_ERR_BUFFER_TOO_SMALL;
    }

    fbl::AutoLock lock(&mutex_);

    usb_request_complete_t complete = {
        .callback = [](void* ctx, usb_request_t* req) { 
            static_cast<UsbMidiSource*>(ctx)->ReadComplete(req);
        },
        .ctx = this,
    };

    list_node_t* node = list_peek_head(&completed_reads_);
    if (!node) {
        status = ZX_ERR_SHOULD_WAIT;
        goto out;
    }
    usb_req_internal_t* req_int;
    req_int = containerof(node, usb_req_internal_t, node);
    usb_request_t* req;
    req = REQ_INTERNAL_TO_USB_REQ(req_int, parent_req_size_);

    // MIDI events are 4 bytes. We can ignore the zeroth byte
    usb_request_copy_from(req, data, 3, 1);
    *actual = get_midi_message_length(*((uint8_t *)data));
    list_remove_head(&completed_reads_);
    status = usb_req_list_add_head(&free_read_reqs_, req, parent_req_size_);
    ZX_DEBUG_ASSERT(status == ZX_OK);
    while ((req = usb_req_list_remove_head(&free_read_reqs_,
                                           parent_req_size_)) != NULL) {
        usb_.RequestQueue(req, &complete);
    }

out:
    UpdateSignals();
    return status;
}

void UsbMidiSource::GetDirection(GetDirectionCompleter::Sync completer) {
    completer.Reply(llcpp::fuchsia::hardware::midi::Direction::SOURCE);
}


zx_status_t UsbMidiSource::DdkMessage(fidl_msg_t* msg, fidl_txn_t* txn) {
    DdkTransaction transaction(txn);
    llcpp::fuchsia::hardware::midi::Device::Dispatch(this, msg, &transaction);
    return transaction.Status();
}

zx_status_t UsbMidiSource::Create(zx_device_t* parent, const UsbDevice& usb, int index,
                                  const usb_interface_descriptor_t* intf,
                                  const usb_endpoint_descriptor_t* ep,
                                  const size_t req_size) {
    auto dev = std::make_unique<UsbMidiSource>(parent, usb, req_size);
    auto status = dev->Init(index, intf, ep);
    if (status != ZX_OK) {
        return status;
    }    

    // devmgr is now in charge of the device.
    __UNUSED auto* _ = dev.release();
    return ZX_OK;
}

zx_status_t UsbMidiSource::Init(int index, const usb_interface_descriptor_t* intf,
                                const usb_endpoint_descriptor_t* ep) {
    list_initialize(&free_read_reqs_);
    list_initialize(&completed_reads_);

    int packet_size = usb_ep_max_packet(ep);
    if (intf->bAlternateSetting != 0) {
        usb_.SetInterface(intf->bInterfaceNumber, intf->bAlternateSetting);
    }
    for (size_t i = 0; i < READ_REQ_COUNT; i++) {
        usb_request_t* req;
        zx_status_t status = usb_request_alloc(&req, packet_size, ep->bEndpointAddress,
                                               parent_req_size_ + sizeof(usb_req_internal_t));
        if (status != ZX_OK) {
            return status;
        }
        req->header.length = packet_size;
        status = usb_req_list_add_head(&free_read_reqs_, req, parent_req_size_);
        ZX_DEBUG_ASSERT(status == ZX_OK);
    }

    char name[ZX_DEVICE_NAME_MAX];
    snprintf(name, sizeof(name), "usb-midi-source-%d", index);

    return DdkAdd(name);
}

}  // namespace usb
}  // namespace audio
