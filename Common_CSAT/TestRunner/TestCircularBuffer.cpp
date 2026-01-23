#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include <memory>
#include "CircularBuffer.hpp"

TEST_CASE("CircularBuffer Functionality")
{

    SUBCASE("Testing Capacity and Size")
    {
        CircularBuffer<int, 5> cbf;
        CHECK(cbf.capacity() == 5);
        CHECK(cbf.size() == 0);
    }

    SUBCASE("Testing Empty and Full Flags")
    {
        CircularBuffer<int, 3> cbf;
        CHECK(cbf.is_empty());
        CHECK(!cbf.is_full());

        cbf.push(1);
        CHECK(!cbf.is_empty());
        CHECK(!cbf.is_full());

        cbf.push(2);
        CHECK(!cbf.is_empty());
        CHECK(!cbf.is_full());

        cbf.push(3);
        CHECK(!cbf.is_empty());
        CHECK(cbf.is_full());
    }

    SUBCASE("Testing Push and Pop with Copy")
    {
        CircularBuffer<int, 5> cbf;

        cbf.push(10);
        cbf.push(20);

        CHECK(cbf.size() == 2);

        int val1 = cbf.pop();
        CHECK(val1 == 10);
        CHECK(cbf.size() == 1);

        int val2 = cbf.pop();
        CHECK(val2 == 20);
        CHECK(cbf.size() == 0);
        CHECK(cbf.is_empty());
    }

    SUBCASE("Testing Push and Pop with Move")
    {
        CircularBuffer<std::string, 5> cbf;

        std::string str1 = "Hello";
        std::string str2 = "World";

        cbf.push(std::move(str1));
        cbf.push(std::move(str2));

        CHECK(cbf.size() == 2);
        CHECK(str1.empty()); // ensure the move has taken place
        CHECK(str2.empty()); // ensure the move has taken place
        std::string popped1 = cbf.pop();
        CHECK(popped1 == "Hello");
        CHECK(cbf.size() == 1);

        std::string popped2 = cbf.pop();
        CHECK(popped2 == "World");
        CHECK(cbf.size() == 0);
        CHECK(cbf.is_empty());
    }

    SUBCASE("Testing Multiple Push, Peek, and Pop")
    {
        CircularBuffer<int, 4> cbf;

        cbf.push(1);
        cbf.push(2);
        cbf.push(3);
        CHECK(cbf.size() == 3);

        CHECK(cbf.peek() == 1);
        CHECK(cbf.pop() == 1);
        CHECK(cbf.size() == 2);

        CHECK(cbf.peek() == 2);
        CHECK(cbf.pop() == 2);
        CHECK(cbf.size() == 1);

        cbf.push(4);
        CHECK(cbf.size() == 2);
        CHECK(cbf.peek() == 3);
        CHECK(cbf.pop() == 3);

        CHECK(cbf.size() == 1);
        CHECK(cbf.peek() == 4);
        CHECK(cbf.pop() == 4);

        CHECK(cbf.size() == 0);
        CHECK(cbf.is_empty());
    }

    SUBCASE("Testing const peek() on a const object")
    {
        CircularBuffer<int, 3> cbf;
        cbf.push(10);
        cbf.push(20);

        const CircularBuffer<int, 3> &const_cbf = cbf;
        CHECK(const_cbf.peek() == 10);
    }

    SUBCASE("Testing Overflow behavior")
    {
        CircularBuffer<int, 3> cbf;

        cbf.push(1);
        cbf.push(2);
        cbf.push(3);
        CHECK(cbf.is_full());
        cbf.push(4); // Overwrites the first element (1)
        CHECK(cbf.size() == 3);

        CHECK(cbf.pop() == 2);
        CHECK(cbf.pop() == 3);
        CHECK(cbf.pop() == 4);
    }

    SUBCASE("Testing Underflow behavior Returns Garbage")
    {
        CircularBuffer<int, 3> cbf;
        CHECK(cbf.is_empty());
        cbf.push(1);
        cbf.pop();
        CHECK(cbf.is_empty());
        // Underflow does not throw exception with current implementation
        cbf.pop(); // Expecting garbage value back
    }

    SUBCASE("peek() after multiple wrap-arounds")
    {
        CircularBuffer<int, 3> cbf;

        cbf.push(1);
        cbf.push(2);
        cbf.push(3);
        cbf.push(4); // drop 1
        cbf.push(5); // drop 2

        CHECK(cbf.peek() == 3);
        CHECK(cbf.pop() == 3);
        CHECK(cbf.peek() == 4);
    }

    SUBCASE("pop() on empty buffer preserves invariants")
    {
        CircularBuffer<int, 3> cbf;

        CHECK(cbf.is_empty());

        int v = cbf.pop(); // UB but should not corrupt state
        (void)v;

        CHECK(cbf.size() == 0);
        CHECK(cbf.is_empty());
    }
}

#include <limits> // Required for std::numeric_limits

// Helper function to fill the buffer
template <typename T, size_t Capacity>
void fillBuffer(CircularBuffer<T, Capacity> &cbf, T startValue)
{
    for (size_t i = 0; i < cbf.capacity(); ++i)
    {
        cbf.push(startValue + static_cast<T>(i));
    }
}

TEST_CASE("CircularBuffer - Wrap-Around Overflow")
{
    CircularBuffer<int, 5> cbf;

    // Set head_ close to capacity:
    for (int i = 0; i < 4; i++)
    {
        cbf.push(i);
    }

    cbf.push(4); // head is 0 again

    REQUIRE(cbf.is_full() == true);
    cbf.push(5); // Overflow:  0 is dropped, cbf is now 1,2,3,4,5
    REQUIRE(cbf.size() == 5);

    CHECK(cbf.pop() == 1); // Verify correct wrap-around order
    CHECK(cbf.pop() == 2);
    CHECK(cbf.pop() == 3);
    CHECK(cbf.pop() == 4);
    CHECK(cbf.pop() == 5);
    CHECK(cbf.is_empty());
}

TEST_CASE("CircularBuffer - Peek and Overflow Interaction")
{
    CircularBuffer<int, 3> cbf;

    cbf.push(10);
    cbf.push(20);
    CHECK(cbf.peek() == 10); // Peek at the oldest value
    cbf.push(30);
    CHECK(cbf.is_full());
    cbf.push(40); // Overflow: 10 is dropped, the size is at 3.

    CHECK(cbf.peek() == 20); // Should still return the correct oldest value
    CHECK(cbf.pop() == 20);
    CHECK(cbf.pop() == 30);
    CHECK(cbf.pop() == 40);
}

//
// not needed, static_assert in CircularBuffer.hpp prohibits this case
//
// TEST_CASE("CircularBuffer - Zero-Sized Buffer") {
//     CircularBuffer<int, 0> cbf;

//     CHECK(cbf.capacity() == 0);
//     CHECK(cbf.size() == 0);
//     CHECK(cbf.is_empty());
//     CHECK(cbf.is_full()); // A zero-sized buffer is *always* full!
// }

TEST_CASE("CircularBuffer - Shared Pointers")
{
    CircularBuffer<std::shared_ptr<int>, 3> cbf;

    cbf.push(std::make_shared<int>(1));
    cbf.push(std::make_shared<int>(2));
    cbf.push(std::make_shared<int>(3));

    REQUIRE(cbf.is_full());
    cbf.push(std::make_shared<int>(4)); // Overflow. The first value is still referenced

    REQUIRE(cbf.size() == 3);

    CHECK(*cbf.pop() == 2);
    CHECK(*cbf.pop() == 3);
    CHECK(*cbf.pop() == 4);
    CHECK(cbf.is_empty());
}

TEST_CASE("CircularBuffer - next() test")
{
    CircularBuffer<int, 3> cbf;

    cbf.next() = 10;
    cbf.next() = 20;
    cbf.next() = 30;

    REQUIRE(cbf.is_full());

    cbf.next() = 40; // Overflow.  The first value is deallocated
    REQUIRE(cbf.size() == 3);

    CHECK(cbf.pop() == 20);
    CHECK(cbf.pop() == 30);
    CHECK(cbf.pop() == 40);
    CHECK(cbf.is_empty());
}

TEST_CASE("CircularBuffer - Clear Functionality")
{
    SUBCASE("Clear on an empty buffer")
    {
        CircularBuffer<int, 5> cbf;
        cbf.clear();
        CHECK(cbf.size() == 0);
        CHECK(cbf.is_empty());
    }

    SUBCASE("Clear on a partially filled buffer")
    {
        CircularBuffer<int, 5> cbf;
        cbf.push(10);
        cbf.push(20);
        cbf.clear();
        CHECK(cbf.size() == 0);
        CHECK(cbf.is_empty());
    }

    SUBCASE("Clear on a full buffer")
    {
        CircularBuffer<int, 3> cbf;
        cbf.push(10);
        cbf.push(20);
        cbf.push(30);
        CHECK(cbf.is_full());
        cbf.clear();
        CHECK(cbf.size() == 0);
        CHECK(cbf.is_empty());
    }

    SUBCASE("Clear and then reuse")
    {
        CircularBuffer<int, 3> cbf;
        cbf.push(10);
        cbf.push(20);
        cbf.push(30);
        cbf.clear();
        cbf.push(40);
        CHECK(cbf.size() == 1);
        CHECK(cbf.pop() == 40);
        CHECK(cbf.is_empty());
    }
}