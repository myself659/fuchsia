// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package bindings

import (
	"sync"

	"syscall/mx"
	"syscall/mx/mxerror"
)

var errConnectionClosed = mx.Error{Status: mx.ErrPeerClosed}

// Connector owns a channel handle. It can read and write messages
// from the channel waiting on it if necessary. The operation are
// thread-safe.
type Connector struct {
	mu   sync.RWMutex // protects pipe handle
	pipe mx.Handle

	done      chan struct{}
	waitMutex sync.Mutex
	waiter    AsyncWaiter
	waitChan  chan WaitResponse
}

// NewStubConnector returns a new |Connector| instance that sends and
// receives messages from a provided channel handle.
func NewConnector(pipe mx.Handle, waiter AsyncWaiter) *Connector {
	return &Connector{
		pipe:     pipe,
		waiter:   waiter,
		done:     make(chan struct{}),
		waitChan: make(chan WaitResponse, 1),
	}
}

// ReadMessage reads a message from channel waiting on it if necessary.
func (c *Connector) ReadMessage() (*Message, error) {
	// Make sure that only one goroutine at a time waits a the handle.
	// We use separate lock so that we can send messages to the channel
	// while waiting on the pipe.
	//
	// It is better to acquire this lock first so that a potential queue of
	// readers will wait while closing the channel in case of Close()
	// call.
	c.waitMutex.Lock()
	defer c.waitMutex.Unlock()
	// Use read lock to use pipe handle without modifying it.
	c.mu.RLock()
	defer c.mu.RUnlock()

	if !c.pipe.IsValid() {
		return nil, errConnectionClosed
	}

	// Check if we already have a message.
	// TODO: how big this should this be?
	bytes := make([]byte, 256)
	handles := make([]mx.Handle, 16)
	_, _, err := (&mx.Channel{c.pipe}).Read(bytes, handles, 0)
	if mxerror.Status(err) == mx.ErrShouldWait {
		waitId := c.waiter.AsyncWait(c.pipe, mx.SignalChannelReadable, c.waitChan)
		select {
		case <-c.waitChan:
			_, _, err = (&mx.Channel{c.pipe}).Read(bytes, handles, 0)
			if err != nil {
				return nil, err
			}
		case <-c.done:
			c.waiter.CancelWait(waitId)
			return nil, errConnectionClosed
		}
	} else if err != nil {
		return nil, err
	}
	return ParseMessage(bytes, handles)
}

// WriteMessage writes a message to the channel.
func (c *Connector) WriteMessage(message *Message) error {
	// Use read lock to use pipe handle without modifying it.
	c.mu.RLock()
	defer c.mu.RUnlock()
	if !c.pipe.IsValid() {
		return errConnectionClosed
	}
	return WriteMessage(c.pipe, message)
}

// Close closes the channel aborting wait on the channel.
// Panics if you try to close the |Connector| more than once.
func (c *Connector) Close() {
	// Stop waiting to acquire the lock.
	close(c.done)
	// Use write lock to modify the pipe handle.
	c.mu.Lock()
	c.pipe.Close()
	c.mu.Unlock()
}
