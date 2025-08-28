#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN // Only define in one cpp file

#include "doctest.h"
#include "BoxSet.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>

TEST_CASE("BoxSet Iterator Range-Based For Loop") {
    BoxSet<std::string, 8> box;
    box.add("apple");
    box.add("banana");
    box.add("cherry");
    box.remove(1); // remove banana
    box.add("date");

    std::vector<std::string> expected = {"apple", "date", "cherry"};
    std::vector<std::string> actual;
    
    for (const auto& item : box) {
        actual.push_back(item);
    }
    
    CHECK(actual == expected);
}

TEST_CASE("BoxSet Iterator std::find") {
    BoxSet<std::string, 8> box;
    box.add("apple");
    box.add("banana");
    box.add("cherry");
    box.remove(1); // remove banana
    box.add("date");

    auto it = std::find(box.begin(), box.end(), "date");
    CHECK(it != box.end());
    CHECK(*it == "date");

    it = std::find(box.begin(), box.end(), "banana");
     CHECK(it == box.end());
    
    it = std::find(box.begin(), box.end(), "grape");
    CHECK(it == box.end());
}

TEST_CASE("BoxSet Iterator Empty Iteration") {
    BoxSet<int, 8> box;

    auto count = std::distance(box.begin(), box.end());
    CHECK(count == 0);
   
    auto it = std::find(box.begin(), box.end(), 10);
     CHECK(it == box.end());
    
}