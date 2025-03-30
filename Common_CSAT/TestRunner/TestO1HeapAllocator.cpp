#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "Allocator.hpp"

#include <iostream>

TEST_CASE("O1HeapAllocator with int and shared_ptr") {
    char buffer[4192] __attribute__ ((aligned (256)));
    O1HeapInstance* heap = o1heapInit(buffer, 4192);
    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(heap);
    size_t allocated = diagnostics.allocated;
    O1HeapAllocator<int> intAllocator(heap);
    {
        auto intPtr = allocate_shared_custom<int>(intAllocator, 100);
        REQUIRE(intPtr != nullptr);
        CHECK(*intPtr == 100);
        diagnostics = o1heapGetDiagnostics(heap);
        CHECK(allocated != diagnostics.allocated);
    }
    diagnostics = o1heapGetDiagnostics(heap);
    CHECK(allocated == diagnostics.allocated);
}

TEST_CASE("O1HeapAllocator with int and unique_ptr") {
    char buffer[4192] __attribute__ ((aligned (256)));
    O1HeapInstance* heap = o1heapInit(buffer, 4192);
    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(heap);
    size_t allocated = diagnostics.allocated;
    O1HeapAllocator<int> intAllocator(heap);
    {    
        auto intPtr = allocate_unique_custom<int>(intAllocator, 100);
        REQUIRE(intPtr != nullptr);
        CHECK(*intPtr == 100);
        diagnostics = o1heapGetDiagnostics(heap);
        CHECK(allocated != diagnostics.allocated);
    }
    diagnostics = o1heapGetDiagnostics(heap);
    CHECK(allocated == diagnostics.allocated);
}

TEST_CASE("O1HeapAllocator with CanardRxTransfer and shared_ptr") {
    char buffer[4192] __attribute__ ((aligned (256)));
    O1HeapInstance* heap = o1heapInit(buffer, 4192);
    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(heap);
    size_t allocated = diagnostics.allocated;
    O1HeapAllocator<CanardRxTransfer> CanardRxTransferAllocator(heap);
    {    
        auto CanardRxTransferPtr = allocate_shared_custom<CanardRxTransfer>(CanardRxTransferAllocator);
        REQUIRE(CanardRxTransferPtr != nullptr);
        diagnostics = o1heapGetDiagnostics(heap);
        CHECK(allocated != diagnostics.allocated);
        if (CanardRxTransferPtr) CanardRxTransferPtr->payload = o1heapAllocate(heap, 100);
    }
    diagnostics = o1heapGetDiagnostics(heap);
    CHECK(allocated == diagnostics.allocated);
}

TEST_CASE("O1HeapAllocator with CanardRxTransfer and unique_ptr") {
    char buffer[4192] __attribute__ ((aligned (256)));
    O1HeapInstance* heap = o1heapInit(buffer, 4192);
    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(heap);
    size_t allocated = diagnostics.allocated;
    O1HeapAllocator<CanardRxTransfer> CanardRxTransferAllocator(heap);
    {    
        auto CanardRxTransferPtr = allocate_unique_custom<CanardRxTransfer>(CanardRxTransferAllocator);
        REQUIRE(CanardRxTransferPtr != nullptr);
        diagnostics = o1heapGetDiagnostics(heap);
        CHECK(allocated != diagnostics.allocated);
        if (CanardRxTransferPtr) CanardRxTransferPtr->payload = o1heapAllocate(heap, 100);
    }
    diagnostics = o1heapGetDiagnostics(heap);
    CHECK(allocated == diagnostics.allocated);
}

TEST_CASE("O1HeapAllocator with CyphalTransfer and shared_ptr") {
    char buffer[4192] __attribute__ ((aligned (256)));
    O1HeapInstance* heap = o1heapInit(buffer, 4192);
    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(heap);
    size_t allocated = diagnostics.allocated;
    O1HeapAllocator<CyphalTransfer> CyphalTransferAllocator(heap);
    {    
        auto CyphalTransferPtr = allocate_shared_custom<CyphalTransfer>(CyphalTransferAllocator);
        REQUIRE(CyphalTransferPtr != nullptr);
        diagnostics = o1heapGetDiagnostics(heap);
        CHECK(allocated != diagnostics.allocated);
        if (CyphalTransferPtr) CyphalTransferPtr->payload = o1heapAllocate(heap, 100);
    }
    diagnostics = o1heapGetDiagnostics(heap);
    CHECK(allocated == diagnostics.allocated);
}

TEST_CASE("O1HeapAllocator with CyphalTransfer and unique_ptr") {
    char buffer[4192] __attribute__ ((aligned (256)));
    O1HeapInstance* heap = o1heapInit(buffer, 4192);
    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(heap);
    size_t allocated = diagnostics.allocated;
    O1HeapAllocator<CyphalTransfer> CyphalTransferAllocator(heap);
    {    
        auto CyphalTransferPtr = allocate_unique_custom<CyphalTransfer>(CyphalTransferAllocator);
        REQUIRE(CyphalTransferPtr != nullptr);
        diagnostics = o1heapGetDiagnostics(heap);
        CHECK(allocated != diagnostics.allocated);
        if (CyphalTransferPtr) CyphalTransferPtr->payload = o1heapAllocate(heap, 100);
    }
    diagnostics = o1heapGetDiagnostics(heap);
    CHECK(allocated == diagnostics.allocated);
}

TEST_CASE("O1HeapAllocator allocation and deallocation") {
    char buffer[4192] __attribute__ ((aligned (256)));
    O1HeapInstance* heap = o1heapInit(buffer, 4192);
    REQUIRE(heap != nullptr);
    O1HeapAllocator<int> intAllocator(heap);

    int* ptr = intAllocator.allocate(5);
    for (int i = 0; i < 5; ++i) {
        ptr[i] = i;
    }

    for (int i = 0; i < 5; ++i) {
        CHECK(ptr[i] == i);
    }

    intAllocator.deallocate(ptr, 5);
}

#include "Task.hpp"
#include "ServiceManager.hpp"
#include "ProcessRxQueue.hpp"
#include "cyphal.hpp"

class MockTask : public Task
{
public:
    MockTask(uint32_t interval, uint32_t tick, const CyphalTransfer transfer) : Task(interval, tick), transfer_(transfer) {}
    ~MockTask() override {}

    void handleMessage(std::shared_ptr<CyphalTransfer> transfer) override {}
    void handleTaskImpl() override {}
    void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) {}
    void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) {}

    CyphalTransfer transfer_;
};

TEST_CASE("shared MockTask size") {
    char buffer[4192] __attribute__ ((aligned (256)));
    O1HeapInstance* heap = o1heapInit(buffer, 4192);
    CyphalTransfer transfer;
    O1HeapAllocator<MockTask> alloc_MockTask(heap);
    std::shared_ptr<MockTask> task = std::allocate_shared<MockTask>(alloc_MockTask, 2000, 100, transfer);
    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(heap);
    CHECK(diagnostics.allocated == 128);
}

TEST_CASE("shared CyphalTransfer size") {
    char buffer[4192] __attribute__ ((aligned (256)));
    O1HeapInstance* heap = o1heapInit(buffer, 4192);
    O1HeapAllocator<CyphalTransfer> alloc_CyphalTransfer(heap);
    std::shared_ptr<CyphalTransfer> transfer = std::allocate_shared<CyphalTransfer>(alloc_CyphalTransfer);;
    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(heap);
    CHECK(diagnostics.allocated == 128);
}
