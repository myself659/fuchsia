// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddktl/fidl.h>
#include <fbl/auto_lock.h>
#include <usb/usb.h>
#include <usb/usb-request.h>
#include <lib/sync/completion.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <threads.h>

#include "midi.h"
#include "usb-audio.h"
#include "usb-midi-sink.h"

constexpr size_t WRITE_REQ_COUNT = 20;

namespace audio {
namespace usb {

void UsbMidiSink::UpdateSignals() {
    zx_signals_t new_signals = 0;
    if (dead_) {
        new_signals |= (DEV_STATE_WRITABLE | DEV_STATE_ERROR);
    } else if (!list_is_empty(&free_write_reqs_)) {
        new_signals |= DEV_STATE_WRITABLE;
    }
    if (new_signals != signals_) {
        ClearAndSetState(signals_ & ~new_signals, new_signals & ~signals_);
        signals_ = new_signals;
    }
}

void UsbMidiSink::WriteComplete(usb_request_t* req) {
    if (req->response.status == ZX_ERR_IO_NOT_PRESENT) {
        usb_request_release(req);
        return;
    }

    // FIXME what to do with error here?
    fbl::AutoLock lock(&mutex_);
    zx_status_t status = usb_req_list_add_tail(&free_write_reqs_, req, parent_req_size_);
    ZX_DEBUG_ASSERT(status == ZX_OK);
    sync_completion_signal(&free_write_completion_);
    UpdateSignals();
}

void UsbMidiSink::DdkUnbind() {
    dead_ = true;
    UpdateSignals();
    sync_completion_signal(&free_write_completion_);
    DdkRemove();
}

UsbMidiSink::~UsbMidiSink() {
    usb_request_t* req;
    while ((req = usb_req_list_remove_head(&free_write_reqs_, parent_req_size_))
           != NULL) {
        usb_request_release(req);
    }
}

void UsbMidiSink::DdkRelease() {
    delete this;
}

zx_status_t UsbMidiSink::DdkOpen(zx_device_t** dev_out, uint32_t flags) {
    zx_status_t result;

    fbl::AutoLock lock(&mutex_);
    if (open_) {
        result = ZX_ERR_ALREADY_BOUND;
    } else {
        open_ = true;
        result = ZX_OK;
    }

    return result;
}

zx_status_t UsbMidiSink::DdkClose(uint32_t flags) {
    fbl::AutoLock lock(&mutex_);
    open_ = false;

    return ZX_OK;
}

zx_status_t UsbMidiSink::DdkWrite(const void* data, size_t length, zx_off_t offset,
                                  size_t* actual) {
    if (dead_) {
        return ZX_ERR_IO_NOT_PRESENT;
    }

    zx_status_t status = ZX_OK;
    size_t out_actual = length;

    const uint8_t* src = (uint8_t *)data;

    while (length > 0) {
        sync_completion_wait(&free_write_completion_, ZX_TIME_INFINITE);
        if (dead_) {
            return ZX_ERR_IO_NOT_PRESENT;
        }

        list_node_t* node;
        {
            fbl::AutoLock lock(&mutex_);
            node = list_remove_head(&free_write_reqs_);
            if (list_is_empty(&free_write_reqs_)) {
                sync_completion_reset(&free_write_completion_);
            }
        }

        if (!node) {
            // shouldn't happen!
            status = ZX_ERR_INTERNAL;
            goto out;
        }
        usb_req_internal_t* req_int = containerof(node, usb_req_internal_t, node);
        usb_request_t* req = REQ_INTERNAL_TO_USB_REQ(req_int, parent_req_size_);

        size_t message_length = get_midi_message_length(*src);
        if (message_length < 1 || message_length > length) return ZX_ERR_INVALID_ARGS;

        uint8_t buffer[4];
        buffer[0] = (src[0] & 0xF0) >> 4;
        buffer[1] = src[0];
        buffer[2] = (message_length > 1 ? src[1] : 0);
        buffer[3] = (message_length > 2 ? src[2] : 0);

        usb_request_copy_to(req, buffer, 4, 0);
        req->header.length = 4;
        usb_request_complete_t complete = {
            .callback = [](void* ctx, usb_request_t* req) { 
                static_cast<UsbMidiSink*>(ctx)->WriteComplete(req);
            },
            .ctx = this,
        };
        usb_.RequestQueue(req, &complete);

        src += message_length;
        length -= message_length;
    }

out:
    UpdateSignals();
    if (status == ZX_OK) {
        *actual = out_actual;
    }
    return status;
}

void UsbMidiSink::GetDirection(GetDirectionCompleter::Sync completer) {
    completer.Reply(llcpp::fuchsia::hardware::midi::Direction::SINK);
}

zx_status_t UsbMidiSink::DdkMessage(fidl_msg_t* msg, fidl_txn_t* txn) {
    DdkTransaction transaction(txn);
    llcpp::fuchsia::hardware::midi::Device::Dispatch(this, msg, &transaction);
    return transaction.Status();
}

zx_status_t UsbMidiSink::Create(zx_device_t* parent, const UsbDevice& usb, int index,
                                const usb_interface_descriptor_t* intf,
                                const usb_endpoint_descriptor_t* ep,
                                const size_t req_size) {
    auto dev = std::make_unique<UsbMidiSink>(parent, usb, req_size);
    auto status = dev->Init(index, intf, ep);
    if (status != ZX_OK) {
        return status;
    }    

    // devmgr is now in charge of the device.
    __UNUSED auto* _ = dev.release();
    return ZX_OK;
}

zx_status_t UsbMidiSink::Init(int index, const usb_interface_descriptor_t* intf,
                              const usb_endpoint_descriptor_t* ep) {
    list_initialize(&free_write_reqs_);

    int packet_size = usb_ep_max_packet(ep);
    if (intf->bAlternateSetting != 0) {
        usb_.SetInterface(intf->bInterfaceNumber, intf->bAlternateSetting);
    }

    for (size_t i = 0; i < WRITE_REQ_COUNT; i++) {
        usb_request_t* req;
        zx_status_t status = usb_request_alloc(&req, usb_ep_max_packet(ep), ep->bEndpointAddress,
                                               parent_req_size_ + sizeof(usb_req_internal_t));
        if (status != ZX_OK) {
            return status;
        }
        req->header.length = packet_size;
        status = usb_req_list_add_head(&free_write_reqs_, req, parent_req_size_);
        ZX_DEBUG_ASSERT(status == ZX_OK);
    }
    sync_completion_signal(&free_write_completion_);

    char name[ZX_DEVICE_NAME_MAX];
    snprintf(name, sizeof(name), "usb-midi-sink-%d", index);

    return DdkAdd(name);
}

}  // namespace usb
}  // namespace audio

