// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "src/developer/debug/debug_agent/test_data/test_so_symbols.h"

int InsertBreakpointFunction(int c) { return 10 * c; }

void AnotherFunctionForKicks() {}