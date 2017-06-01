// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package bindings

import (
	"sync/atomic"

	"syscall/mx"
)

func align(size, alignment int) int {
	return ((size - 1) | (alignment - 1)) + 1
}

// bytesForBits returns minimum number of bytes required to store provided
// number of bits.
func bytesForBits(bits uint64) int {
	return int((bits + 7) / 8)
}

// WriteMessage writes a message to a channel.
func WriteMessage(handle mx.Handle, message *Message) error {
	err := (&mx.Channel{handle}).Write(message.Bytes, message.Handles, 0)
	if err != nil {
		return err
	}
	return nil
}

// StringPointer converts provided string to *string.
func StringPointer(s string) *string {
	return &s
}

// Counter is a simple thread-safe lock-free counter that can issue unique
// numbers starting from 1 to callers.
type Counter interface {
	// Count returns next unused value, each value is returned only once.
	Count() uint64
}

// NewCounter return a new counter that returns numbers starting from 1.
func NewCounter() Counter {
	return &counterImpl{}
}

// counterImpl implements Counter interface.
// This implementation uses atomic operations on an uint64, it should be always
// allocated separatelly to be 8-aligned in order to work correctly on ARM.
// See http://golang.org/pkg/sync/atomic/#pkg-note-BUG.
type counterImpl struct {
	last uint64
}

func (c *counterImpl) Count() uint64 {
	return atomic.AddUint64(&c.last, 1)
}
