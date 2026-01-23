#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "imagebuffer/buffer_state.hpp"

// Helper to construct a BufferState with given geometry
static BufferState make(size_t head,
                        size_t tail,
                        size_t size,
                        size_t cap)
{
    return BufferState(head, tail, size, /*flash_start*/0, cap);
}

// -----------------------------------------------------------------------------
// 1. Basic invariants
// -----------------------------------------------------------------------------
TEST_CASE("BufferState: empty buffer invariants")
{
    BufferState s = make(0, 0, 0, 128);

    CHECK(s.is_empty());
    CHECK(s.size() == 0);
    CHECK(s.count() == 0);
    CHECK(s.available() == 128);
    CHECK(s.capacity() == 128);

    // available_from(start) must return full capacity
    for (size_t i = 0; i < 128; i++)
        CHECK(s.available_from(i) == 128);
}

TEST_CASE("BufferState: non-empty simple case (no wrap)")
{
    // used region = [10, 10+20) = [10,30)
    BufferState s = make(/*head*/10,
                         /*tail*/30,
                         /*size*/20,
                         /*cap*/128);
    s.count_ = 1;

    CHECK(!s.is_empty());
    CHECK(s.size() == 20);
    CHECK(s.available() == 108);

    // free region = [30,128) ∪ [0,10)
    CHECK(s.available_from(30) == 128 - 30 + 10); // wrap free
    CHECK(s.available_from(0)  == 10);
    CHECK(s.available_from(9)  == 1);
    CHECK(s.available_from(10) == 0); // inside used
    CHECK(s.available_from(29) == 0); // inside used
}

// -----------------------------------------------------------------------------
// 2. Wrap-around used region
// -----------------------------------------------------------------------------
TEST_CASE("BufferState: used region wraps around end of buffer")
{
    // cap = 100
    // used region = [90, 90+20) = [90,110) → wraps to [90,100) ∪ [0,10)
    BufferState s = make(/*head*/90,
                         /*tail*/10,
                         /*size*/20,
                         /*cap*/100);
    s.count_ = 1;

    // free region = [10,90)
    CHECK(s.available_from(10) == 80);
    CHECK(s.available_from(50) == 40);
    CHECK(s.available_from(89) == 1);

    // inside used region
    CHECK(s.available_from(90) == 0);
    CHECK(s.available_from(95) == 0);
    CHECK(s.available_from(0)  == 0);
    CHECK(s.available_from(9)  == 0);
}

// -----------------------------------------------------------------------------
// 3. available_from() must be correct for all offsets
// -----------------------------------------------------------------------------
TEST_CASE("BufferState: exhaustive available_from for non-wrapping used region")
{
    // used = [20,50)
    BufferState s = make(20, 50, 30, 100);
    s.count_ = 1;

    for (size_t start = 0; start < 100; start++)
    {
        if (start < 20)
            CHECK(s.available_from(start) == 20 - start);
        else if (start >= 50)
            CHECK(s.available_from(start) == 100 - start + 20);
        else
            CHECK(s.available_from(start) == 0);
    }
}

TEST_CASE("BufferState: exhaustive available_from for wrapping used region")
{
    // used = [80,120) → [80,100) ∪ [0,20)
    BufferState s = make(80, 20, 40, 100);
    s.count_ = 1;

    for (size_t start = 0; start < 100; start++)
    {
        bool in_used =
            (start >= 80) ||
            (start < 20);

        if (in_used)
            CHECK(s.available_from(start) == 0);
        else
            CHECK(s.available_from(start) == 80 - start);
    }
}

// -----------------------------------------------------------------------------
// 4. size/count consistency
// -----------------------------------------------------------------------------
TEST_CASE("BufferState: available() matches size_")
{
    BufferState s = make(0, 0, 0, 64);
    CHECK(s.available() == 64);

    s.size_ = 10;
    CHECK(s.available() == 54);

    s.size_ = 63;
    CHECK(s.available() == 1);

    s.size_ = 64;
    CHECK(s.available() == 0);
}

// -----------------------------------------------------------------------------
// 5. Head/tail invariants
// -----------------------------------------------------------------------------
TEST_CASE("BufferState: head/tail getters")
{
    BufferState s = make(12, 34, 22, 100);
    CHECK(s.get_head() == 12);
    CHECK(s.get_tail() == 34);
}
