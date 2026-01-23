#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "HeapAllocation.hpp"
#include "cyphal.hpp"
#include "canard.h"
#include "serard.h"
#include "udpard.h"

// ------------------------------------------------------------
// Local heap adapter for SafeAllocator
// ------------------------------------------------------------
static O1HeapInstance* heap = nullptr;

struct LocalHeap {
    static void* heapAllocate(void*, size_t amount) {
        return o1heapAllocate(heap, amount);
    }
    static void heapFree(void*, void* ptr) {
        o1heapFree(heap, ptr);
    }
};

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------
static void init_heap(uint8_t* buffer, size_t size) {
    heap = o1heapInit(buffer, size);
    REQUIRE(heap != nullptr);
}

// ------------------------------------------------------------
// Tests for alloc_shared_custom
// ------------------------------------------------------------

TEST_CASE("alloc_shared_custom<int> allocates and frees correctly")
{
    alignas(O1HEAP_ALIGNMENT) static uint8_t buffer[4096];
    init_heap(buffer, sizeof(buffer));

    SafeAllocator<int, LocalHeap> alloc;

    O1HeapDiagnostics before = o1heapGetDiagnostics(heap);
    size_t allocated0 = before.allocated;

    {
        auto p = alloc_shared_custom<int>(alloc, 123);
        REQUIRE(p != nullptr);
        CHECK(*p == 123);

        O1HeapDiagnostics mid = o1heapGetDiagnostics(heap);
        CHECK(mid.allocated > allocated0);
    }

    O1HeapDiagnostics after = o1heapGetDiagnostics(heap);
    CHECK(after.allocated == allocated0);
}

TEST_CASE("alloc_shared_custom<CyphalTransfer> cleans payload")
{
    alignas(O1HEAP_ALIGNMENT) static uint8_t buffer[4096];
    init_heap(buffer, sizeof(buffer));

    SafeAllocator<CyphalTransfer, LocalHeap> alloc;

    size_t allocated0 = o1heapGetDiagnostics(heap).allocated;

    {
        auto p = alloc_shared_custom<CyphalTransfer>(alloc);
        REQUIRE(p != nullptr);

        // allocate payload manually
        p->payload = o1heapAllocate(heap, 100);
        REQUIRE(p->payload != nullptr);

        CHECK(o1heapGetDiagnostics(heap).allocated > allocated0);
    }

    CHECK(o1heapGetDiagnostics(heap).allocated == allocated0);
}

TEST_CASE("alloc_shared_custom<CanardRxTransfer> cleans payload")
{
    alignas(O1HEAP_ALIGNMENT) static uint8_t buffer[4096];
    init_heap(buffer, sizeof(buffer));

    SafeAllocator<CanardRxTransfer, LocalHeap> alloc;

    size_t allocated0 = o1heapGetDiagnostics(heap).allocated;

    {
        auto p = alloc_shared_custom<CanardRxTransfer>(alloc);
        REQUIRE(p != nullptr);

        p->payload = o1heapAllocate(heap, 100);
        REQUIRE(p->payload != nullptr);

        CHECK(o1heapGetDiagnostics(heap).allocated > allocated0);
    }

    CHECK(o1heapGetDiagnostics(heap).allocated == allocated0);
}

// ------------------------------------------------------------
// Tests for alloc_unique_custom
// ------------------------------------------------------------

TEST_CASE("alloc_unique_custom<int> allocates and frees correctly")
{
    alignas(O1HEAP_ALIGNMENT) static uint8_t buffer[4096];
    init_heap(buffer, sizeof(buffer));

    SafeAllocator<int, LocalHeap> alloc;

    size_t allocated0 = o1heapGetDiagnostics(heap).allocated;

    {
        auto p = alloc_unique_custom<int>(alloc, 777);
        REQUIRE(p != nullptr);
        CHECK(*p == 777);

        CHECK(o1heapGetDiagnostics(heap).allocated > allocated0);
    }

    CHECK(o1heapGetDiagnostics(heap).allocated == allocated0);
}

TEST_CASE("alloc_unique_custom<CyphalTransfer> cleans payload")
{
    alignas(O1HEAP_ALIGNMENT) static uint8_t buffer[4096];
    init_heap(buffer, sizeof(buffer));

    SafeAllocator<CyphalTransfer, LocalHeap> alloc;

    size_t allocated0 = o1heapGetDiagnostics(heap).allocated;

    {
        auto p = alloc_unique_custom<CyphalTransfer>(alloc);
        REQUIRE(p != nullptr);

        p->payload = o1heapAllocate(heap, 100);
        REQUIRE(p->payload != nullptr);

        CHECK(o1heapGetDiagnostics(heap).allocated > allocated0);
    }

    CHECK(o1heapGetDiagnostics(heap).allocated == allocated0);
}

TEST_CASE("alloc_unique_custom<CanardRxTransfer> cleans payload")
{
    alignas(O1HEAP_ALIGNMENT) static uint8_t buffer[4096];
    init_heap(buffer, sizeof(buffer));

    SafeAllocator<CanardRxTransfer, LocalHeap> alloc;

    size_t allocated0 = o1heapGetDiagnostics(heap).allocated;

    {
        auto p = alloc_unique_custom<CanardRxTransfer>(alloc);
        REQUIRE(p != nullptr);

        p->payload = o1heapAllocate(heap, 100);
        REQUIRE(p->payload != nullptr);

        CHECK(o1heapGetDiagnostics(heap).allocated > allocated0);
    }

    CHECK(o1heapGetDiagnostics(heap).allocated == allocated0);
}

// ------------------------------------------------------------
// Raw allocate/deallocate
// ------------------------------------------------------------

TEST_CASE("SafeAllocator<int> raw allocate/deallocate")
{
    alignas(O1HEAP_ALIGNMENT) static uint8_t buffer[4096];
    init_heap(buffer, sizeof(buffer));

    SafeAllocator<int, LocalHeap> alloc;

    int* p = alloc.allocate(5);
    REQUIRE(p != nullptr);

    for (int i = 0; i < 5; ++i) p[i] = i;
    for (int i = 0; i < 5; ++i) CHECK(p[i] == i);

    alloc.deallocate(p, 5);
}
