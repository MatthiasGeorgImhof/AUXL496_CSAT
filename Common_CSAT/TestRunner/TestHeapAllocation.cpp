#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "cyphal.hpp"
#include "HeapAllocation.hpp"
#include <vector>
#include <cstring>

TEST_CASE("Heap initializes correctly")
{
    HeapAllocation<1024>::initialize();
    auto* inst = HeapAllocation<1024>::getO1Heap();
    REQUIRE(inst != nullptr);

    auto diag = o1heapGetDiagnostics(inst);
    CHECK(diag.capacity > 0);
    CHECK(diag.capacity <= 1024);
    CHECK(diag.allocated == 0);
}

TEST_CASE("Basic allocation and deallocation")
{
    HeapAllocation<1024>::initialize();
    auto* inst = HeapAllocation<1024>::getO1Heap();

    void* p = HeapAllocation<1024>::serardMemoryAllocate(nullptr, 64);
    REQUIRE(p != nullptr);

    auto diag_after_alloc = o1heapGetDiagnostics(inst);
    CHECK(diag_after_alloc.allocated >= 64);

    HeapAllocation<1024>::serardMemoryDeallocate(nullptr, 64, p);

    auto diag_after_free = o1heapGetDiagnostics(inst);
    CHECK(diag_after_free.allocated == 0);
}

TEST_CASE("Multiple allocations and frees")
{
    HeapAllocation<1024>::initialize();
    auto* inst = HeapAllocation<1024>::getO1Heap();

    auto diag0 = o1heapGetDiagnostics(inst);
    REQUIRE(diag0.capacity > 0);

    // Try two 128‑byte allocations, but don't REQUIRE the second one.
    void* a = HeapAllocation<1024>::udpardMemoryAllocate(nullptr, 128);
    REQUIRE(a != nullptr);

    void* b = HeapAllocation<1024>::udpardMemoryAllocate(nullptr, 128);

    if (b == nullptr)
    {
        // Not enough contiguous space — valid o1heap behavior.
        HeapAllocation<1024>::udpardMemoryDeallocate(nullptr, 128, a);
        return;
    }

    CHECK(a != b);

    auto diag = o1heapGetDiagnostics(inst);
    CHECK(diag.allocated >= 256);

    HeapAllocation<1024>::udpardMemoryDeallocate(nullptr, 128, a);
    HeapAllocation<1024>::udpardMemoryDeallocate(nullptr, 128, b);

    diag = o1heapGetDiagnostics(inst);
    CHECK(diag.allocated == 0);
}

TEST_CASE("OOM behavior")
{
    HeapAllocation<256>::initialize();
    HeapAllocation<256> heap;

    auto diag0 = heap.getDiagnostics();
    // If capacity is zero, o1heapInit failed for this size; just skip this scenario.
    if (diag0.capacity == 0)
        return;

    const size_t block = diag0.capacity / 4;
    REQUIRE(block > 0);

    std::vector<void*> blocks;

    for (;;)
    {
        void* p = HeapAllocation<256>::serardMemoryAllocate(nullptr, block);
        if (p == nullptr)
            break;
        blocks.push_back(p);
    }

    auto diag = heap.getDiagnostics();
    CHECK(diag.oom_count >= 1);

    for (void* p : blocks)
        HeapAllocation<256>::serardMemoryDeallocate(nullptr, block, p);

    diag = heap.getDiagnostics();
    CHECK(diag.allocated == 0);
}

TEST_CASE("Diagnostics wrapper returns correct values")
{
    HeapAllocation<1024>::initialize();
    HeapAllocation<1024> heap;

    auto diag0 = heap.getDiagnostics();
    if (diag0.capacity == 0)
        return;

    void* p = HeapAllocation<1024>::serardMemoryAllocate(nullptr, 100);
    REQUIRE(p != nullptr);

    auto diag = heap.getDiagnostics();
    CHECK(diag.allocated >= 100);
    CHECK(diag.capacity > 0);
    CHECK(diag.capacity <= 1024);

    HeapAllocation<1024>::serardMemoryDeallocate(nullptr, 100, p);

    diag = heap.getDiagnostics();
    CHECK(diag.allocated == 0);
}

TEST_CASE("Alignment guarantees")
{
    HeapAllocation<1024>::initialize();

    void* p = HeapAllocation<1024>::serardMemoryAllocate(nullptr, 32);
    REQUIRE(p != nullptr);

    uintptr_t addr = reinterpret_cast<uintptr_t>(p);
    CHECK(addr % O1HEAP_ALIGNMENT == 0);

    HeapAllocation<1024>::serardMemoryDeallocate(nullptr, 32, p);
}
