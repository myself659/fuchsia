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
    } else if (!completed_reads_.is_empty()) {
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
        completed_reads_.push(UsbRequest(req, parent_req_size_));
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
    std::optional<UsbRequest> req;
    while ((req = completed_reads_.pop()).has_value()) {
        usb_.RequestQueue(req->take(), &complete);
    }
    while ((req = free_read_reqs_.pop()).has_value()) {
        usb_.RequestQueue(req->take(), &complete);
    }

    return result;
}

zx_status_t UsbMidiSource::DdkClose(uint32_t flags) {
    fbl::AutoLock lock(&mutex_);
    open_ = false;

    return ZX_OK;
}

void UsbMidiSource::GetDirection(GetDirectionCompleter::Sync completer) {
    completer.Reply(llcpp::fuchsia::hardware::midi::Direction::SOURCE);
}

void UsbMidiSource::Read(uint64_t count, ReadCompleter::Sync completer) {
    auto result = llcpp::fuchsia::hardware::midi::Device_Read_Result();
    zx_status_t status = ZX_OK;

    if (dead_) {
        status = ZX_ERR_IO_NOT_PRESENT;
        goto error;
    }

    if (count < 3) {
        status = ZX_ERR_BUFFER_TOO_SMALL;
        goto error;
    }

    {
        fbl::AutoLock lock(&mutex_);

        usb_request_complete_t complete = {
            .callback = [](void* ctx, usb_request_t* req) { 
                static_cast<UsbMidiSource*>(ctx)->ReadComplete(req);
            },
            .ctx = this,
        };

        auto req = completed_reads_.pop();
        if (!req.has_value()) {
            status = ZX_ERR_SHOULD_WAIT;
            goto error;
        }

        // MIDI events are 4 bytes. We can ignore the zeroth byte
        uint8_t data[3];
        req->CopyFrom(data, 3, 1);

        auto response = llcpp::fuchsia::hardware::midi::Device_Read_Response();    
        response.data = fidl::VectorView(get_midi_message_length(data[0]), data);
        result.set_response(std::move(response));
    
        free_read_reqs_.push(std::move(*req));
        while ((req = free_read_reqs_.pop()).has_value()) {
            usb_.RequestQueue(req->take(), &complete);
        }
    }

error:
    UpdateSignals();
    result.set_err(status);
    completer.Reply(std::move(result));
}

void UsbMidiSource::Write(fidl::VectorView<uint8_t> data, WriteCompleter::Sync completer) {
    auto result = llcpp::fuchsia::hardware::midi::Device_Write_Result();
    result.set_err(ZX_ERR_NOT_SUPPORTED);
    completer.Reply(std::move(result));
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
    int packet_size = usb_ep_max_packet(ep);
    if (intf->bAlternateSetting != 0) {
        usb_.SetInterface(intf->bInterfaceNumber, intf->bAlternateSetting);
    }
    for (size_t i = 0; i < READ_REQ_COUNT; i++) {
        std::optional<UsbRequest> req;
        auto status = UsbRequest::Alloc(&req, packet_size, ep->bEndpointAddress, parent_req_size_);
        if (status != ZX_OK) {
            return status;
        }
        req->request()->header.length = packet_size;
        free_read_reqs_.push(std::move(*req));
    }

    char name[ZX_DEVICE_NAME_MAX];
    snprintf(name, sizeof(name), "usb-midi-source-%d", index);

    return DdkAdd(name);
}

}  // namespace usb
}  // namespace audio
