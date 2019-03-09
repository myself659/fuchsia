// Copyright 2019 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <memory>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <fbl/auto_lock.h>
#include <io-scheduler/io-scheduler.h>
#include <lib/fzl/fifo.h>
#include <unittest/unittest.h>

namespace tests {

using IoScheduler = ioscheduler::Scheduler;
using IoSchedulerUniquePtr = ioscheduler::SchedulerUniquePtr;
using SchedOp = ioscheduler::SchedOp;

constexpr uint32_t kMaxFifoDepth = (4096 / sizeof(void*));

struct TestOp {

};

class TestContext {
public:
    TestContext(IoScheduler* sched) : sched_(sched) {}
    ~TestContext() {}

    IoScheduler* Scheduler() { return sched_; }
    zx_status_t CreateFifo();

    zx_status_t AcquireOps(SchedOp** sop_list, size_t list_count,
                           size_t* actual_count, bool wait) {
        return ZX_OK;
    }

    zx_status_t IssueOp(SchedOp* sop) {
        return ZX_OK;
    }

    zx_status_t ReleaseOp(SchedOp* sop) {
        return ZX_OK;
    }

    void CancelAcquire() {}

    void Fatal() {}

    static bool CanReorder(void* context, SchedOp* first, SchedOp* second) {
        return false;
    }

    static zx_status_t AcquireCallback(void* context, SchedOp** sop_list, size_t list_count,
                                       size_t* actual_count, bool wait) {
        TestContext* test = static_cast<TestContext*>(context);
        return test->AcquireOps(sop_list, list_count, actual_count, wait);
    }

    static zx_status_t IssueCallback(void* context, SchedOp* sop) {
        TestContext* test = static_cast<TestContext*>(context);
        return test->IssueOp(sop);
    }

    static void ReleaseCallback(void* context, SchedOp* sop) {
        TestContext* test = static_cast<TestContext*>(context);
        test->ReleaseOp(sop);
    }

    static void CancelCallback(void* context) {
        TestContext* test = static_cast<TestContext*>(context);
        test->CancelAcquire();
    }

    static void FatalCallback(void* context) {
        TestContext* test = static_cast<TestContext*>(context);
        test->Fatal();
    }

private:
    IoScheduler* sched_;
    fzl::fifo<void*, void*> fifo_to_server_;
    fzl::fifo<void*, void*> fifo_from_server_;
};

zx_status_t TestContext::CreateFifo() {
    return fzl::create_fifo(kMaxFifoDepth, 0, &fifo_to_server_, &fifo_from_server_);
}

ioscheduler::SchedulerCallbacks callbacks = {
    .context = nullptr,
    .CanReorder = TestContext::CanReorder,
    .Acquire = TestContext::AcquireCallback,
    .Issue = TestContext::IssueCallback,
    .Release = TestContext::ReleaseCallback,
    .CancelAcquire = TestContext::CancelCallback,
    .Fatal = TestContext::FatalCallback,
};

enum TestLevel {
    kTestLevelCreate,
    kTestLevelInit,
    kTestLevelOpen,
    kTestLevelServe,
};

static bool iosched_run(TestLevel test_level) {
    BEGIN_TEST;

    IoScheduler* scheduler;
    zx_status_t status = ioscheduler::SchedulerCreate(&callbacks, &scheduler);
    ASSERT_EQ(status, ZX_OK, "Failed to create scheduler");
    TestContext test(scheduler);
    IoSchedulerUniquePtr sched(scheduler);
    callbacks.context = &test;

    do {
        // Create test.
        // --------------------------------
        if (test_level == kTestLevelCreate) break;

        // Init test.
        // --------------------------------
        status = ioscheduler::SchedulerInit(sched.get());
        ASSERT_EQ(status, ZX_OK, "Failed to init scheduler");
        if (test_level == kTestLevelInit) break;

        // Stream open test.
        // --------------------------------
        status = ioscheduler::SchedulerStreamOpen(sched.get(), 5, ioscheduler::kDefaultPri);
        ASSERT_EQ(status, ZX_OK, "Failed to open stream");
        status = ioscheduler::SchedulerStreamOpen(sched.get(), 0, ioscheduler::kDefaultPri);
        ASSERT_EQ(status, ZX_OK, "Failed to open stream");
        status = ioscheduler::SchedulerStreamOpen(sched.get(), 5, ioscheduler::kDefaultPri);
        ASSERT_NE(status, ZX_OK, "Expected failure to open duplicate stream");
        status = ioscheduler::SchedulerStreamOpen(sched.get(), 3, 100000);
        ASSERT_NE(status, ZX_OK, "Expected failure to open with invalid priority");
        status = ioscheduler::SchedulerStreamOpen(sched.get(), 3, 1);
        ASSERT_EQ(status, ZX_OK, "Failed to open stream");
        if (test_level == kTestLevelOpen) break;

        // Serve test.
        // --------------------------------
        status = test.CreateFifo();
        ASSERT_EQ(status, ZX_OK, "Internal test failure");
        status = ioscheduler::SchedulerServe(sched.get());
        ASSERT_EQ(status, ZX_OK, "Failed to begin service");
        if (test_level == kTestLevelServe) break;

        ASSERT_TRUE(false, "Unexpected test level");
    } while (false);

    switch (test_level) {
    case kTestLevelServe:
        // Serve test.
        // --------------------------------
    case kTestLevelOpen:
        // Stream open test.
        // --------------------------------
        status = ioscheduler::SchedulerStreamClose(sched.get(), 5);
        ASSERT_EQ(status, ZX_OK, "Failed to close stream");
        status = ioscheduler::SchedulerStreamClose(sched.get(), 3);
        ASSERT_EQ(status, ZX_OK, "Failed to close stream");
        // Stream 0 intentionally left open here.
        __FALLTHROUGH;
    case kTestLevelInit:
        // Init test.
        // --------------------------------
        ioscheduler::SchedulerShutdown(sched.get());
        __FALLTHROUGH;
    case kTestLevelCreate:
        // Create test.
        // --------------------------------
        break;
    default:
        ASSERT_TRUE(false, "Unexpected test level");
    }

    sched.release();
    END_TEST;
}

static bool iosched_test_create() {
    return iosched_run(kTestLevelCreate);
}

static bool iosched_test_open() {
    return iosched_run(kTestLevelOpen);
}

static bool iosched_test_serve() {
    return iosched_run(kTestLevelServe);
}

BEGIN_TEST_CASE(test_case_iosched)
RUN_TEST(iosched_test_create)
RUN_TEST(iosched_test_open)
RUN_TEST(iosched_test_serve)
END_TEST_CASE(test_case_iosched)

} // namespace tests

int main(int argc, char** argv) {
    bool success = unittest_run_all_tests(argc, argv);
    return success ? 0 : -1;
}
