#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "ResourceCounter.hpp"

// Example Resource class (for testing)
struct MyResource {
    int value = 0; // Add some state for verification
    MyResource(int v = 0) : value(v) {}

    bool operator==(const MyResource& other) const {
        return value == other.value;
    }
};

TEST_CASE("ResourceCounter Basic Functionality") {
    ResourceCounter<MyResource> counter;

    SUBCASE("Initial State") {
        CHECK(counter.getCount() == 0);
        CHECK(counter.hasClaims() == false);
        CHECK(counter.resource_.value == 0); // Check default resource value
    }

    SUBCASE("Increment and Decrement") {
        counter.increment();
        CHECK(counter.getCount() == 1);
        CHECK(counter.hasClaims() == true);

        counter.decrement();
        CHECK(counter.getCount() == 0);
        CHECK(counter.hasClaims() == false);
    }

    SUBCASE("Multiple Increments and Decrements") {
        counter.increment();
        counter.increment();
        CHECK(counter.getCount() == 2);

        counter.decrement();
        CHECK(counter.getCount() == 1);

        counter.decrement();
        CHECK(counter.getCount() == 0);
        CHECK(counter.hasClaims() == false);
    }

    SUBCASE("Reset") {
        counter.increment();
        counter.increment();
        counter.reset();
        CHECK(counter.getCount() == 0);
        CHECK(counter.hasClaims() == false);
    }
}

TEST_CASE("ResourceCounter with Initial Value") {
    ResourceCounter<MyResource> counter({}, 5); // Start with 5 claims

    CHECK(counter.getCount() == 5);
    CHECK(counter.hasClaims() == true);

    counter.decrement();
    CHECK(counter.getCount() == 4);

    for (int i = 0; i < 4; ++i) {
        counter.decrement();
    }

    CHECK(counter.getCount() == 0);
    CHECK(counter.hasClaims() == false);
}

TEST_CASE("ResourceCounter with a non-default constructible Resource") {
    // Resource that *requires* a constructor argument

    struct NonDefaultResource {
        int value;
        NonDefaultResource(int v) : value(v) {}

        bool operator==(const NonDefaultResource& other) const {
            return value == other.value;
        }
    };

    ResourceCounter<NonDefaultResource> counter(NonDefaultResource(10));

    CHECK(counter.resource_.value == 10);
}


TEST_CASE("ResourceCounter with a more complex Resource") {
    struct ComplexResource {
        int id;
        std::string name;

        ComplexResource(int i, const std::string& n) : id(i), name(n) {}

        bool operator==(const ComplexResource& other) const {
            return id == other.id && name == other.name;
        }
    };

    ComplexResource initialResource(42, "The Answer");
    ResourceCounter<ComplexResource> counter(initialResource);

    CHECK(counter.resource_.id == 42);
    CHECK(counter.resource_.name == "The Answer");
}