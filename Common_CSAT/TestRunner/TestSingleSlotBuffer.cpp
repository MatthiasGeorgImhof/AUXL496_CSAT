#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "SingleSlotBuffer.hpp"
#include <memory>

struct Dummy
{
    int value;
    Dummy(int v) : value(v) {}
};

TEST_CASE("SingleSlotBuffer with shared_ptr")
{
    SingleSlotBuffer<std::shared_ptr<Dummy>> buffer;

    CHECK(buffer.is_empty());
    CHECK(buffer.size() == 0);
    CHECK(buffer.capacity() == 1);

    SUBCASE("Push and peek")
    {
        auto ptr1 = std::make_shared<Dummy>(42);
        buffer.push(ptr1);

        CHECK(buffer.is_full());
        CHECK(buffer.peek()->value == 42);
        CHECK(ptr1.use_count() == 2); // one in test, one in buffer
    }

    SUBCASE("Overwrite decrements old shared_ptr")
    {
        auto ptr1 = std::make_shared<Dummy>(42);
        buffer.push(ptr1);
        CHECK(ptr1.use_count() == 2);

        auto ptr2 = std::make_shared<Dummy>(99);
        buffer.push(ptr2); // overwrites ptr1

        CHECK(ptr1.use_count() == 1); // buffer released it
        CHECK(ptr2.use_count() == 2);
        CHECK(buffer.peek()->value == 99);
    }

    SUBCASE("Pop releases shared_ptr")
    {
        auto ptr1 = std::make_shared<Dummy>(123);
        buffer.push(ptr1);
        CHECK(ptr1.use_count() == 2);
        {
            auto popped = buffer.pop();
            CHECK(popped->value == 123);
            CHECK(ptr1.use_count() == 2);
        }
        CHECK(ptr1.use_count() == 1);
        CHECK(buffer.is_empty());
    }

    SUBCASE("Clear releases shared_ptr")
    {
        auto ptr1 = std::make_shared<Dummy>(321);
        buffer.push(ptr1);
        CHECK(ptr1.use_count() == 2);

        buffer.clear();
        CHECK(ptr1.use_count() == 1);
        CHECK(buffer.is_empty());
    }
}
