#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <fbl/auto_lock.h>
#include <fbl/condition_variable.h>
#include <ioqueue/queue.h>
#include <zircon/listnode.h>

struct TestOp {
    list_node_t node;
    uint32_t id;
    bool issued;
    bool released;
    io_op_t op;
};

class IoQueueTest {
public:
    IoQueueTest();
    // ~IoQueueTest();

    void Enqueue(TestOp* top);
    void SetQueue(IoQueue* q) { q_ = q; }
    IoQueue* GetQueue() { return q_; }
    void WaitReleasedAll();

    // Callbacks
    static zx_status_t cb_acquire(void* context, io_op_t** op_list, size_t* op_count, bool wait) {
        IoQueueTest* test = static_cast<IoQueueTest*>(context);
        return test->AcquireOps(op_list, op_count, wait);
    }

    static zx_status_t cb_issue(void* context, io_op_t* op) {
        IoQueueTest* test = static_cast<IoQueueTest*>(context);
        return test->IssueOp(op);
    }

    static void cb_release(void* context, io_op_t* op) {
        IoQueueTest* test = static_cast<IoQueueTest*>(context);
        test->ReleaseOp(op);
    }

    static void cb_cancel_acquire(void* context) {
        IoQueueTest* test = static_cast<IoQueueTest*>(context);
        test->CancelAcquire();
    }

    static void cb_fatal(void* context) {
        IoQueueTest* test = static_cast<IoQueueTest*>(context);
        test->Fatal();
    }

    // zx_status_t Init(IoQueue* q);
private:
    zx_status_t AcquireOps(io_op_t** op_list, size_t* op_count, bool wait);
    void CancelAcquire();
    zx_status_t IssueOp(io_op_t* op);
    void ReleaseOp(io_op_t* op);
    void Fatal();

    IoQueue* q_ = nullptr;

    fbl::Mutex lock_;
    bool closed_ = false;
    uint32_t enqueued_count_ = 0;
    uint32_t issued_count_ = 0;
    uint32_t released_count_ = 0;
    list_node_t in_list_;
    fbl::ConditionVariable in_avail_;
    fbl::ConditionVariable released_all_;
};

IoQueueTest::IoQueueTest() {
    list_initialize(&in_list_);
}

void IoQueueTest::Enqueue(TestOp* top) {
    fbl::AutoLock lock(&lock_);
    list_add_tail(&in_list_, &top->node);
    enqueued_count_++;
}

void IoQueueTest::WaitReleasedAll() {
    fbl::AutoLock lock(&lock_);
    released_all_.Wait(&lock_);
}

zx_status_t IoQueueTest::AcquireOps(io_op_t** op_list, size_t* op_count, bool wait) {
    fbl::AutoLock lock(&lock_);
    printf("cb: acquire\n");

    if (closed_) {
        printf("cb:   closed\n");
        return ZX_ERR_CANCELED;   // Input source closed.
    }
    if (list_is_empty(&in_list_)) {
        if (!wait) {
            return ZX_ERR_SHOULD_WAIT;
        }
        in_avail_.Wait(&lock_);
        if (closed_) {
            return ZX_ERR_CANCELED;
        }
    }
    size_t i, max_ops = *op_count;
    for (i = 0; i < max_ops; i++) {
        list_node_t* node = list_remove_head(&in_list_);
        if (node == nullptr) {
            break;
        }
        // io_list_remove(node);
        TestOp* top = containerof(node, TestOp, node);
        op_list[i] = &top->op;
    }
    *op_count = i;
    return ZX_OK;
}

zx_status_t IoQueueTest::IssueOp(io_op_t* op) {
    printf("cb: issue %u:%u\n", op->sid, op->opcode);
    TestOp* top = containerof(op, TestOp, op);
    top->issued = true;
    op->result = ZX_OK;
    fbl::AutoLock lock(&lock_);
    issued_count_++;
    return ZX_OK;
}

void IoQueueTest::ReleaseOp(io_op_t* op) {
    printf("cb: release %u:%u\n", op->sid, op->opcode);
    TestOp* top = containerof(op, TestOp, op);
    top->released = true;
    fbl::AutoLock lock(&lock_);
    released_count_++;
    if (released_count_ == enqueued_count_) {
        released_all_.Broadcast();
    }
}

void IoQueueTest::CancelAcquire() {
    printf("cb: cancel_acquire\n");
    fbl::AutoLock lock(&lock_);
    closed_ = true;
    in_avail_.Broadcast();
}

void IoQueueTest::Fatal() {
    printf("cb: FATAL\n");
    assert(false);
}

IoQueueCallbacks cb = {
    .context = NULL,
    .acquire = IoQueueTest::cb_acquire,
    .issue = IoQueueTest::cb_issue,
    .release = IoQueueTest::cb_release,
    .cancel_acquire = IoQueueTest::cb_cancel_acquire,
    .fatal = IoQueueTest::cb_fatal,
};

uint32_t num_workers = 1;

void op_test(IoQueueTest* test, int depth) {
    printf("%s\n", __FUNCTION__);
    const size_t num_ops = 5;
    TestOp tops[num_ops];
    memset(tops, 0, sizeof(TestOp) * num_ops);

    tops[0].id = 100;
    tops[0].op.sid = 0;

    tops[1].id = 101;
    tops[1].op.sid = 0;

    test->Enqueue(&tops[0]);
    test->Enqueue(&tops[1]);

    tops[2].id = 102;
    tops[2].op.sid = 2;

    test->Enqueue(&tops[2]);

    tops[3].id = 103;
    tops[3].op.sid = 4;

    tops[4].id = 104;
    tops[4].op.sid = 0;

    test->Enqueue(&tops[3]);
    test->Enqueue(&tops[4]);

    test->WaitReleasedAll();
    printf("%s done\n", __FUNCTION__);
}

void serve_test(IoQueueTest* test, int depth) {
    printf("%s\n", __FUNCTION__);
    IoQueue* q = test->GetQueue();

    // printf("%s:%u\n", __FUNCTION__, __LINE__);
    zx_status_t status = IoQueueServe(q, num_workers);
    assert(status == ZX_OK);

    if (depth) {
        op_test(test, depth);
    }

    IoQueueShutdown(q);
    printf("%s done\n", __FUNCTION__);
}

void open_test(IoQueueTest* test, int depth) {
    printf("%s\n", __FUNCTION__);
    IoQueue* q = test->GetQueue();
    zx_status_t status = IoQueueOpenStream(q, 10, 0); // Open stream 0 at priority 10
    assert(status == ZX_OK);

    status = IoQueueOpenStream(q, 12, 2); // Open stream 2 at priority 12
    assert(status == ZX_OK);

    status = IoQueueOpenStream(q, 12, 1); // Open stream 1 at priority 12
    assert(status == ZX_OK);

    status = IoQueueOpenStream(q, 9, 4); // Open stream 4 at priority 9
    assert(status == ZX_OK);

    status = IoQueueOpenStream(q, 18, 2); // Open stream 2 at priority 18
    // Failure expected, stream already open.
    assert(status == ZX_ERR_ALREADY_EXISTS);

    status = IoQueueOpenStream(q, kIoQueueMaxPri + 5, 7); // Open stream 7 at invalid priority.
    // Failure expected
    assert(status == ZX_ERR_INVALID_ARGS);

    // valid streams are 0, 1, 2, 4
    if (depth) {
        serve_test(test, depth - 1);
    }

    printf("%s done\n", __FUNCTION__);
}

void create_test(int depth) {
    printf("%s\n", __FUNCTION__);
    IoQueue* q;

    IoQueueTest test;
    cb.context = static_cast<void*>(&test);

    zx_status_t status = IoQueueCreate(&cb, &q);
    assert(status == ZX_OK);
    test.SetQueue(q);

    if (depth) {
        open_test(&test, depth - 1);
    }

    IoQueueDestroy(q);
    printf("%s done\n\n", __FUNCTION__);
}

void do_tests() {
    create_test(0);
    create_test(1);
    create_test(3);
}

int main(int argc, char* argv[]) {
    printf("IO Queue test\n");
    do_tests();
    return 0;
}
