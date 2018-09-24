// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "garnet/bin/zxdb/client/step_thread_controller.h"
#include "garnet/bin/zxdb/client/process.h"
#include "garnet/bin/zxdb/client/thread.h"
#include "garnet/bin/zxdb/client/thread_controller_test.h"
#include "garnet/bin/zxdb/common/err.h"
#include "garnet/bin/zxdb/symbols/line_details.h"
#include "garnet/bin/zxdb/symbols/mock_module_symbols.h"
#include "garnet/lib/debug_ipc/protocol.h"

namespace zxdb {

class StepThreadControllerTest : public ThreadControllerTest {
 public:
  // Shared code for the shared lib thunk tests. There are two variants of this
  // test, one where we want to skip the thunks, and one where we don't. The
  // parameter controls which variant of the test to run.
  void DoSharedLibThunkTest(bool stop_on_no_symbols);

  // Backend that runs a test for stepping into an unsymbolized function,
  // both for when we want it to stop (param = true) and continue (param =
  // false).
  void DoUnsymbolizedFunctionTest(bool stop_on_no_symbols);
};

// Software exceptions should always stop execution. These might be from
// something like a hardcoded breakpoint instruction in the code. Doing "step"
// shouldn't skip over these.
TEST_F(StepThreadControllerTest, SofwareException) {
  // Step as long as we're in this range. Using the "code range" for stepping
  // allows us to avoid dependencies on the symbol subsystem.
  constexpr uint64_t kBeginAddr = 0x1000;
  constexpr uint64_t kEndAddr = 0x1010;

  // Set up the thread to be stopped at the beginning of our range.
  debug_ipc::NotifyException exception;
  exception.process_koid = process()->GetKoid();
  exception.type = debug_ipc::NotifyException::Type::kHardware;
  exception.thread.koid = thread()->GetKoid();
  exception.thread.state = debug_ipc::ThreadRecord::State::kBlocked;
  exception.frames.resize(2);
  exception.frames[0].ip = kBeginAddr;
  exception.frames[0].sp = 0x5000;
  exception.frames[0].bp = 0x5000;
  InjectException(exception);

  // Continue the thread with the controller stepping in range.
  auto step_into = std::make_unique<StepThreadController>(
      AddressRange(kBeginAddr, kEndAddr));
  bool continued = false;
  thread()->ContinueWith(std::move(step_into), [&continued](const Err& err) {
    if (!err.has_error())
      continued = true;
  });

  // It should have been able to step without doing any further async work.
  EXPECT_TRUE(continued);
  EXPECT_EQ(1, resume_count());

  // Issue a software exception in the range.
  exception.type = debug_ipc::NotifyException::Type::kSoftware;
  exception.frames[0].ip += 4;
  InjectException(exception);

  // It should have stayed stopped despite being in range.
  EXPECT_EQ(1, resume_count());  // Same count as above.
  EXPECT_EQ(debug_ipc::ThreadRecord::State::kBlocked, thread()->GetState());
}

// Some entries in the line table may have their line number set to zero. These
// indicate code generated by the compiler not associated with any line number.
// These should be transparently stepped over when stepping by line.
//
// This test tests the case where the line table has 10, 0, 10 11. Stepping
// from the first "10" line should end up on "11".
TEST_F(StepThreadControllerTest, Line0) {
  FileLine line0("/path/file.cc", 0);
  FileLine line10("/path/file.cc", 10);
  FileLine line11("/path/file.cc", 11);

  const uint64_t kAddr1 = kSymbolizedModuleAddress + 0x100;  // Line 10
  const uint64_t kAddr2 = kAddr1 + 4;                        // Line 0
  const uint64_t kAddr3 = kAddr2 + 4;                        // Line 10
  const uint64_t kAddr4 = kAddr3 + 4;                        // Line 11

  LineDetails line_details1(line10);
  line_details1.entries().push_back({20, AddressRange(kAddr1, kAddr2)});

  LineDetails line_details2(line0);
  line_details2.entries().push_back({0, AddressRange(kAddr2, kAddr3)});

  LineDetails line_details3(line10);
  line_details3.entries().push_back({10, AddressRange(kAddr3, kAddr4)});

  LineDetails line_details4(line11);
  line_details4.entries().push_back({0, AddressRange(kAddr4, kAddr4 + 4)});

  module_symbols()->AddLineDetails(kAddr1, line_details1);
  module_symbols()->AddLineDetails(kAddr2, line_details2);
  module_symbols()->AddLineDetails(kAddr3, line_details3);
  module_symbols()->AddLineDetails(kAddr4, line_details4);

  // Set up the thread to be stopped at the beginning of our range.
  debug_ipc::NotifyException exception;
  exception.process_koid = process()->GetKoid();
  exception.type = debug_ipc::NotifyException::Type::kHardware;
  exception.thread.koid = thread()->GetKoid();
  exception.thread.state = debug_ipc::ThreadRecord::State::kBlocked;
  exception.frames.resize(2);
  exception.frames[0].ip = kAddr1;
  exception.frames[0].sp = 0x5000;
  exception.frames[0].bp = 0x5000;
  InjectException(exception);

  // Continue the thread with the controller stepping in range.
  auto step_into =
      std::make_unique<StepThreadController>(StepMode::kSourceLine);
  bool continued = false;
  thread()->ContinueWith(std::move(step_into), [&continued](const Err& err) {
    if (!err.has_error())
      continued = true;
  });

  // It should have been able to step without doing any further async work.
  EXPECT_TRUE(continued);
  EXPECT_EQ(1, resume_count());

  // Stop on 2nd instruction (line 0). This should be automatically resumed.
  exception.frames[0].ip = kAddr2;
  InjectException(exception);
  EXPECT_EQ(2, resume_count());

  // Stop on 3rd instruction (line 10). Since this matches the original line,
  // it should be automatically resumed.
  exception.frames[0].ip = kAddr3;
  InjectException(exception);
  EXPECT_EQ(3, resume_count());

  // Stop on 4th instruction. Since this is line 11, we should stay stopped.
  exception.frames[0].ip = kAddr4;
  InjectException(exception);
  EXPECT_EQ(3, resume_count());  // Same count as above.
  EXPECT_EQ(debug_ipc::ThreadRecord::State::kBlocked, thread()->GetState());
}

// Tests shared library thunks which have no symbol information in a module
// which otherwise has symbols.
//
// A cross module function call looks like
//  1. A call to an address in the same module.
//  2. That is an indirect jump to an address (the dynamic loader fills in the
//     destination address when imports are resolved). This jump has no symbol
//     information since it's generated by the linker.
//  3. Normal code in another module.
void StepThreadControllerTest::DoSharedLibThunkTest(bool stop_on_no_symbols) {
  FileLine src_line("/path/src.cc", 1);
  FileLine dest_line("/path/dest.cc", 2);

  const uint64_t kAddrSrc = kSymbolizedModuleAddress + 0x100;  // Line 1
  const uint64_t kAddrThunk =
      kSymbolizedModuleAddress + 0x10000;  // No symbols.
  // This is technically in the same module (normally it would be in a
  // different one) but it doesn't matter for this test and it simplifies
  // things.
  const uint64_t kAddrDest = kSymbolizedModuleAddress + 0x200;

  LineDetails src_details(src_line);
  src_details.entries().push_back({0, AddressRange(kAddrSrc, kAddrSrc + 1)});
  module_symbols()->AddLineDetails(kAddrSrc, src_details);

  LineDetails dest_details(dest_line);
  dest_details.entries().push_back({0, AddressRange(kAddrDest, kAddrDest + 1)});
  module_symbols()->AddLineDetails(kAddrDest, dest_details);

  // Set up the thread to be stopped at the beginning of our range.
  debug_ipc::NotifyException exception;
  exception.process_koid = process()->GetKoid();
  exception.type = debug_ipc::NotifyException::Type::kHardware;
  exception.thread.koid = thread()->GetKoid();
  exception.thread.state = debug_ipc::ThreadRecord::State::kBlocked;
  exception.frames.resize(2);
  exception.frames[0].ip = kAddrSrc;
  exception.frames[0].sp = 0x5000;
  exception.frames[0].bp = 0x5000;
  InjectException(exception);

  // Continue the thread with the controller stepping in range.
  auto step_into =
      std::make_unique<StepThreadController>(StepMode::kSourceLine);
  step_into->set_stop_on_no_symbols(stop_on_no_symbols);
  bool continued = false;
  thread()->ContinueWith(std::move(step_into), [&continued](const Err& err) {
    if (!err.has_error())
      continued = true;
  });

  // It should have been able to step without doing any further async work.
  EXPECT_TRUE(continued);
  EXPECT_EQ(1, resume_count());

  // Stop on the thunk instruction with no line info. This is a separate
  // function so we push an entry on the stack.
  exception.frames.emplace(exception.frames.begin());
  exception.frames[0].ip = kAddrThunk;
  exception.frames[0].sp = 0x4ff0;
  exception.frames[0].bp = 0x5000;
  InjectException(exception);
  if (stop_on_no_symbols) {
    // For this variant of the test, the unsymbolized thunk should have stopped
    // stepping.
    EXPECT_EQ(1, resume_count());
    EXPECT_EQ(debug_ipc::ThreadRecord::State::kBlocked, thread()->GetState());
    return;
  }

  // The rest of this test is the "step over unsymbolized thunks" case. It
  // should have automatically resumed from the previous exception.
  EXPECT_EQ(2, resume_count());

  // Stop on dest instruction. Since it's a different line, we should now stop.
  exception.frames[0].ip = kAddrDest;
  InjectException(exception);
  EXPECT_EQ(2, resume_count());  // Unchanged from previous.
  EXPECT_EQ(debug_ipc::ThreadRecord::State::kBlocked, thread()->GetState());
}

TEST_F(StepThreadControllerTest, SharedLibThunksStepOver) {
  DoSharedLibThunkTest(false);
}

TEST_F(StepThreadControllerTest, SharedLibThunksStepInto) {
  DoSharedLibThunkTest(true);
}

void StepThreadControllerTest::DoUnsymbolizedFunctionTest(
    bool stop_on_no_symbols) {
  FileLine src_line("/path/src.cc", 1);

  // Jump from src to dest and return, then to kOutOfRange.
  const uint64_t kAddrSrc = kSymbolizedModuleAddress + 0x100;
  const uint64_t kAddrDest = kUnsymbolizedModuleAddress + 0x200;
  const uint64_t kAddrReturn = kAddrSrc + 4;
  const uint64_t kAddrOutOfRange = kAddrReturn + 4;

  LineDetails src_details(src_line);
  src_details.entries().push_back({0, AddressRange(kAddrSrc, kAddrOutOfRange)});
  module_symbols()->AddLineDetails(kAddrSrc, src_details);

  // Set up the thread to be stopped at the beginning of our range.
  debug_ipc::NotifyException src_exception;
  src_exception.process_koid = process()->GetKoid();
  src_exception.type = debug_ipc::NotifyException::Type::kHardware;
  src_exception.thread.koid = thread()->GetKoid();
  src_exception.thread.state = debug_ipc::ThreadRecord::State::kBlocked;
  src_exception.frames.resize(2);
  src_exception.frames[0].ip = kAddrSrc;
  src_exception.frames[0].sp = 0x5000;
  src_exception.frames[0].bp = 0x5000;
  src_exception.frames[1].ip = 0x10;
  src_exception.frames[1].sp = 0x5008;
  src_exception.frames[1].bp = 0x5008;
  InjectException(src_exception);

  // Continue the thread with the controller stepping in range.
  auto step_into =
      std::make_unique<StepThreadController>(StepMode::kSourceLine);
  step_into->set_stop_on_no_symbols(stop_on_no_symbols);
  bool continued = false;
  thread()->ContinueWith(std::move(step_into), [&continued](const Err& err) {
    if (!err.has_error())
      continued = true;
  });

  // It should have been able to step without doing any further async work.
  EXPECT_TRUE(continued);
  EXPECT_EQ(1, resume_count());

  // Stop on the destination unsymbolized address.
  debug_ipc::NotifyException dest_exception(src_exception);
  dest_exception.frames.resize(2);
  dest_exception.frames[0].ip = kAddrDest;
  dest_exception.frames[0].sp = 0x4ff0;
  dest_exception.frames[0].bp = 0x4ff0;
  dest_exception.frames[1].ip = kAddrReturn;
  dest_exception.frames[1].sp = 0x5000;
  dest_exception.frames[1].bp = 0x5000;
  InjectException(dest_exception);
  if (stop_on_no_symbols) {
    // For this variant of the test, the unsymbolized thunk should have stopped
    // stepping.
    EXPECT_EQ(1, resume_count());
    EXPECT_EQ(debug_ipc::ThreadRecord::State::kBlocked, thread()->GetState());
    return;
  }

  // The rest of this test is the "step over unsymbolized thunks" case. It
  // should have automatically resumed from the previous exception.
  EXPECT_EQ(2, resume_count());

  // Send a breakpoint completion notification at the previous stack frame.
  // Breakpoint exceptions are "software".
  src_exception.type = debug_ipc::NotifyException::Type::kSoftware;
  src_exception.hit_breakpoints.resize(1);
  src_exception.hit_breakpoints[0].breakpoint_id = last_breakpoint_id();
  src_exception.hit_breakpoints[0].hit_count = 1;
  src_exception.frames[0].ip = kAddrReturn;
  InjectException(src_exception);

  // This should have continued since the return address is still in the
  // original address range.
  EXPECT_EQ(3, resume_count());

  // Stop on dest instruction, this is still in range so we should continue.
  src_exception.frames[0].ip = kAddrOutOfRange;
  InjectException(src_exception);
  EXPECT_EQ(3, resume_count());  // Unchanged from previous.
  EXPECT_EQ(debug_ipc::ThreadRecord::State::kBlocked, thread()->GetState());
}

TEST_F(StepThreadControllerTest, UnsymbolizedCallStepOver) {
  DoUnsymbolizedFunctionTest(false);
}

TEST_F(StepThreadControllerTest, UnsymbolizedCallStepInto) {
  DoUnsymbolizedFunctionTest(true);
}

}  // namespace zxdb
