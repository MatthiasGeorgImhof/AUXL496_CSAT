// TestBoxSet.cpp - Corrected `removeMultiple` and comments
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include <array>
#include <string>
#include <vector>
#include <algorithm>
#include "BoxSet.hpp"

// Helper function to create a filled BoxSet
template <typename CONTENT, uint8_t N>
BoxSet<CONTENT, N> createFilledBoxSet(const std::array<CONTENT, N> &init_data)
{
    return BoxSet<CONTENT, N>(init_data);
}

// Helper function to add multiple elements to a BoxSet
template <typename CONTENT, uint8_t N>
void addMultiple(BoxSet<CONTENT, N> &box, size_t num_elements)
{
    for (size_t i = 0; i < num_elements; ++i)
    {
        box.add(static_cast<CONTENT>(std::to_string(i * 10)));
    }
}

// Helper function to remove multiple elements from a BoxSet. Corrected version.
template <typename CONTENT, uint8_t N>
void removeMultiple(BoxSet<CONTENT, N> &box, size_t num_elements)
{
    size_t count = 0;
    for (size_t i = 0; i < N; ++i)
    {
        if (box.is_used(static_cast<uint8_t>(i)))
        {
            box.remove(static_cast<uint8_t>(i));
            count++;
            if (count >= num_elements) return;
        }
    }
}

template <typename CONTENT, uint8_t N>
void testInitialization(BoxSet<CONTENT, N> &box, std::array<CONTENT, N> initial_data)
{
    CHECK(box.is_empty());
    CHECK(box.size() == 0);
    CHECK(box.capacity() == N);
    BoxSet<CONTENT, N> box2 = createFilledBoxSet<CONTENT, N>(initial_data);
    CHECK(!box2.is_empty());
    CHECK(box2.size() == N);
    CHECK(box2.capacity() == N);
    CHECK(box2.is_full());
}

template <typename CONTENT, uint8_t N>
void testAddAndSize(BoxSet<CONTENT, N> &box, size_t num_elements)
{
    addMultiple(box, num_elements);
    CHECK(box.size() == num_elements);
}

template <typename CONTENT, uint8_t N>
void testIsFull(BoxSet<CONTENT, N> &box, size_t num_elements)
{
    CHECK(!box.is_full());
    addMultiple(box, num_elements);
    CHECK(box.is_full());
}
template <typename CONTENT, uint8_t N>
void testAddAndRemove(BoxSet<CONTENT, N> &box, size_t num_elements)
{
    addMultiple(box, num_elements);
    CHECK(box.size() == num_elements);

    removeMultiple(box, num_elements / 2);
    CHECK(box.size() == num_elements - num_elements / 2);

    removeMultiple(box, num_elements - num_elements / 2);
    CHECK(box.size() == 0);
}

template <typename CONTENT, uint8_t N>
void testRemoveByPointer(BoxSet<CONTENT, N> &box, std::vector<CONTENT> &items)
{
    std::vector<CONTENT *> ptrs;
    for (const auto &item : items)
    {
        ptrs.push_back(box.add(item));
    }
    CHECK(box.size() == items.size());
    for (size_t i = 0; i < ptrs.size(); i++)
    {
        box.remove(ptrs[i]);
    }
    CHECK(box.size() == 0);
}

template <typename CONTENT, uint8_t N>
void testAddAndIsUsed(BoxSet<CONTENT, N> &box)
{
    for (int i = 0; i < N; ++i)
    {
        box.add(static_cast<CONTENT>(std::to_string(i)));
        for (int j = 0; j < N; ++j)
            CHECK(box.is_used(static_cast<uint8_t>(j)) == (j <= i));
    }
}

template <typename CONTENT, uint8_t N>
void testMixedOperations(BoxSet<CONTENT, N> &box, size_t num_elements)
{
    addMultiple(box, num_elements);
    CHECK(box.size() == num_elements);

    removeMultiple(box, num_elements / 2);
    CHECK(box.size() == num_elements - num_elements / 2);

    removeMultiple(box, num_elements - num_elements / 2);
    CHECK(box.size() == 0);
}

template <typename CONTENT, uint8_t N>
void testFindOrCreate(BoxSet<CONTENT, N> &box)
{
    for (int i = 0; i < N; ++i)
    {
        auto ptr = box.find_or_create(std::to_string(i), std::equal_to<std::string>());
        CHECK(ptr != nullptr);
        CHECK(box.size() == i + 1);
    }
    auto ptr = box.find_or_create("elderberry", std::equal_to<std::string>());
    CHECK(ptr == nullptr);
    CHECK(box.size() == N);
}

template <typename CONTENT, uint8_t N>
void testContains(BoxSet<CONTENT, N> &box, std::vector<CONTENT> &items)
{
    CHECK(!box.contains("apple"));
    for (const auto &item : items)
        box.add(item);

    CHECK(box.contains("apple"));
    CHECK(box.contains("banana"));
    CHECK(box.contains("cherry"));
    CHECK(!box.contains("date"));

    auto caseInsensitiveCompare = [](const std::string &a, const std::string &b)
    {
        std::string aLower = a;
        std::string bLower = b;
        std::transform(aLower.begin(), aLower.end(), aLower.begin(), ::tolower);
        std::transform(bLower.begin(), bLower.end(), bLower.begin(), ::tolower);
        return aLower == bLower;
    };

    CHECK(box.contains("APPLE", caseInsensitiveCompare));
    CHECK(box.contains("BaNaNa", caseInsensitiveCompare));
    CHECK(box.contains("CHERRY", caseInsensitiveCompare));
    CHECK(!box.contains("Date", caseInsensitiveCompare));
}
template <typename CONTENT, uint8_t N>
void testFind(BoxSet<CONTENT, N> &box, std::vector<CONTENT> &items)
{
    
    for(const auto &item : items)
       box.add(item);
    
    // Test finding existing elements
    for (const auto& item : items) {
        auto foundPtr = box.find(item, std::equal_to<CONTENT>());
        CHECK(foundPtr != nullptr);
        CHECK(*foundPtr == item);
    }

    // Test not finding a non-existent element
    CONTENT notFoundItem = "nonexistent";
    auto notFoundPtr = box.find(notFoundItem, std::equal_to<CONTENT>());
    CHECK(notFoundPtr == nullptr);
    
    //Test with custom comparator
    auto caseInsensitiveCompare = [](const std::string &a, const std::string &b)
    {
        std::string aLower = a;
        std::string bLower = b;
        std::transform(aLower.begin(), aLower.end(), aLower.begin(), ::tolower);
        std::transform(bLower.begin(), bLower.end(), bLower.begin(), ::tolower);
        return aLower == bLower;
    };
    
    
    for (const auto& item : items) {
        auto foundPtr = box.find(item, caseInsensitiveCompare);
         CHECK(foundPtr != nullptr);
         CHECK(caseInsensitiveCompare(*foundPtr,item));
    }
     auto notFoundPtrCustom = box.find("NonExistent",caseInsensitiveCompare);
    CHECK(notFoundPtrCustom == nullptr);


}

TEST_CASE("BoxSet Initialization and Empty Check")
{
    SUBCASE("N=8")
    {
        BoxSet<int, static_cast<uint8_t>(8)> box;
        std::array<int, 8> initial_data = {1, 2, 3, 4, 5, 6, 7, 8};
        testInitialization<int, static_cast<uint8_t>(8)>(box, initial_data);
    }
    SUBCASE("N=16")
    {
        BoxSet<int, static_cast<uint8_t>(16)> box;
        std::array<int, 16> initial_data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
        testInitialization<int, static_cast<uint8_t>(16)>(box, initial_data);
    }
    SUBCASE("N=32")
    {
        BoxSet<int, static_cast<uint8_t>(32)> box;
        std::array<int, 32> initial_data;
        for (uint8_t i = 0; i < 32; ++i)
        {
            initial_data[i] = i + 1U;
        }
        testInitialization<int, static_cast<uint8_t>(32)>(box, initial_data);
    }
    SUBCASE("N=64")
    {
        BoxSet<int, static_cast<uint8_t>(64)> box;
        std::array<int, 64> initial_data;
        for (uint8_t i = 0; i < 64; ++i)
        {
            initial_data[i] = i + 1U;
        }
        testInitialization<int, static_cast<uint8_t>(64)>(box, initial_data);
    }
}

TEST_CASE("BoxSet Add and Size Check")
{
    SUBCASE("N=8")
    {
        BoxSet<std::string, static_cast<uint8_t>(8)> box;
        testAddAndSize<std::string, static_cast<uint8_t>(8)>(box, 4);
    }
    SUBCASE("N=16")
    {
        BoxSet<std::string, static_cast<uint8_t>(16)> box;
        testAddAndSize<std::string, static_cast<uint8_t>(16)>(box, 8);
    }
    SUBCASE("N=32")
    {
        BoxSet<std::string, static_cast<uint8_t>(32)> box;
        testAddAndSize<std::string, static_cast<uint8_t>(32)>(box, 16);
    }
    SUBCASE("N=64")
    {
        BoxSet<std::string, static_cast<uint8_t>(64)> box;
        testAddAndSize<std::string, static_cast<uint8_t>(64)>(box, 32);
    }
}

TEST_CASE("BoxSet IsFull Check")
{
    SUBCASE("N=8")
    {
        BoxSet<std::string, static_cast<uint8_t>(8)> box;
        testIsFull<std::string, static_cast<uint8_t>(8)>(box, 8);
    }
    SUBCASE("N=16")
    {
        BoxSet<std::string, static_cast<uint8_t>(16)> box;
        testIsFull<std::string, static_cast<uint8_t>(16)>(box, 16);
    }
    SUBCASE("N=32")
    {
        BoxSet<std::string, static_cast<uint8_t>(32)> box;
        testIsFull<std::string, static_cast<uint8_t>(32)>(box, 32);
    }
    SUBCASE("N=64")
    {
        BoxSet<std::string, static_cast<uint8_t>(64)> box;
        testIsFull<std::string, static_cast<uint8_t>(64)>(box, 64);
    }
}

TEST_CASE("BoxSet Add and Remove Check")
{
    SUBCASE("N=8")
    {
        BoxSet<std::string, static_cast<uint8_t>(8)> box;
        testAddAndRemove<std::string, static_cast<uint8_t>(8)>(box, 4);
    }
    SUBCASE("N=16")
    {
        BoxSet<std::string, static_cast<uint8_t>(16)> box;
        testAddAndRemove<std::string, static_cast<uint8_t>(16)>(box, 8);
    }
    SUBCASE("N=32")
    {
        BoxSet<std::string, static_cast<uint8_t>(32)> box;
        testAddAndRemove<std::string, static_cast<uint8_t>(32)>(box, 16);
    }
    SUBCASE("N=64")
    {
        BoxSet<std::string, static_cast<uint8_t>(64)> box;
        testAddAndRemove<std::string, static_cast<uint8_t>(64)>(box, 32);
    }
}

TEST_CASE("BoxSet Add and Remove By Pointer Check")
{
    SUBCASE("N=8")
    {
        BoxSet<std::string, static_cast<uint8_t>(8)> box;
        std::vector<std::string> items = {"item1", "item2", "item3", "item4"};
        testRemoveByPointer<std::string, static_cast<uint8_t>(8)>(box, items);
    }
    SUBCASE("N=16")
    {
        BoxSet<std::string, static_cast<uint8_t>(16)> box;
        std::vector<std::string> items;
        for (int i = 0; i < 8; ++i)
            items.push_back("item" + std::to_string(i));
        testRemoveByPointer<std::string, static_cast<uint8_t>(16)>(box, items);
    }
    SUBCASE("N=32")
    {
        BoxSet<std::string, static_cast<uint8_t>(32)> box;
        std::vector<std::string> items;
        for (int i = 0; i < 16; ++i)
            items.push_back("item" + std::to_string(i));
        testRemoveByPointer<std::string, static_cast<uint8_t>(32)>(box, items);
    }
    SUBCASE("N=64")
    {
        BoxSet<std::string, static_cast<uint8_t>(64)> box;
        std::vector<std::string> items;
        for (int i = 0; i < 32; ++i)
            items.push_back("item" + std::to_string(i));
        testRemoveByPointer<std::string, static_cast<uint8_t>(64)>(box, items);
    }
}

TEST_CASE("BoxSet Add and IsUsed Check")
{
    SUBCASE("N=8")
    {
        BoxSet<std::string, static_cast<uint8_t>(8)> box;
        testAddAndIsUsed<std::string, static_cast<uint8_t>(8)>(box);
    }
    SUBCASE("N=16")
    {
        BoxSet<std::string, static_cast<uint8_t>(16)> box;
        testAddAndIsUsed<std::string, static_cast<uint8_t>(16)>(box);
    }
    SUBCASE("N=32")
    {
        BoxSet<std::string, static_cast<uint8_t>(32)> box;
        testAddAndIsUsed<std::string, static_cast<uint8_t>(32)>(box);
    }
    SUBCASE("N=64")
    {
        BoxSet<std::string, static_cast<uint8_t>(64)> box;
        testAddAndIsUsed<std::string, static_cast<uint8_t>(64)>(box);
    }
}

TEST_CASE("BoxSet Mixed Operations")
{
    SUBCASE("N=8")
    {
        BoxSet<std::string, static_cast<uint8_t>(8)> box;
        testMixedOperations<std::string, static_cast<uint8_t>(8)>(box, 4);
    }
    SUBCASE("N=16")
    {
        BoxSet<std::string, static_cast<uint8_t>(16)> box;
        testMixedOperations<std::string, static_cast<uint8_t>(16)>(box, 8);
    }
    SUBCASE("N=32")
    {
        BoxSet<std::string, static_cast<uint8_t>(32)> box;
        testMixedOperations<std::string, static_cast<uint8_t>(32)>(box, 16);
    }
    SUBCASE("N=64")
    {
        BoxSet<std::string, static_cast<uint8_t>(64)> box;
        testMixedOperations<std::string, static_cast<uint8_t>(64)>(box, 32);
    }
}

TEST_CASE("BoxSet find_or_create")
{
    SUBCASE("N=8")
    {
        BoxSet<std::string, static_cast<uint8_t>(8)> box;
        testFindOrCreate<std::string, static_cast<uint8_t>(8)>(box);
    }
    SUBCASE("N=16")
    {
        BoxSet<std::string, static_cast<uint8_t>(16)> box;
        testFindOrCreate<std::string, static_cast<uint8_t>(16)>(box);
    }
    SUBCASE("N=32")
    {
        BoxSet<std::string, static_cast<uint8_t>(32)> box;
        testFindOrCreate<std::string, static_cast<uint8_t>(32)>(box);
    }
    SUBCASE("N=64")
    {
        BoxSet<std::string, static_cast<uint8_t>(64)> box;
        testFindOrCreate<std::string, static_cast<uint8_t>(64)>(box);
    }
}

TEST_CASE("BoxSet contains check")
{
    SUBCASE("N=8")
    {
        BoxSet<std::string, static_cast<uint8_t>(8)> box;
        std::vector<std::string> items = {"apple", "banana", "cherry"};
        testContains<std::string, static_cast<uint8_t>(8)>(box, items);
    }
    SUBCASE("N=16")
    {
        BoxSet<std::string, static_cast<uint8_t>(16)> box;
        std::vector<std::string> items = {"apple", "banana", "cherry"};
        testContains<std::string, static_cast<uint8_t>(16)>(box, items);
    }
    SUBCASE("N=32")
    {
        BoxSet<std::string, static_cast<uint8_t>(32)> box;
        std::vector<std::string> items = {"apple", "banana", "cherry"};
        testContains<std::string, static_cast<uint8_t>(32)>(box, items);
    }
    SUBCASE("N=64")
    {
        BoxSet<std::string, static_cast<uint8_t>(64)> box;
        std::vector<std::string> items = {"apple", "banana", "cherry"};
        testContains<std::string, static_cast<uint8_t>(64)>(box, items);
    }
}
TEST_CASE("BoxSet find check")
{
    SUBCASE("N=8")
    {
        BoxSet<std::string, static_cast<uint8_t>(8)> box;
        std::vector<std::string> items = {"apple", "banana", "cherry"};
       testFind<std::string, static_cast<uint8_t>(8)>(box, items);
    }
    SUBCASE("N=16")
    {
         BoxSet<std::string, static_cast<uint8_t>(16)> box;
        std::vector<std::string> items = {"apple", "banana", "cherry"};
        testFind<std::string, static_cast<uint8_t>(16)>(box, items);
    }
    SUBCASE("N=32")
    {
        BoxSet<std::string, static_cast<uint8_t>(32)> box;
         std::vector<std::string> items = {"apple", "banana", "cherry"};
        testFind<std::string, static_cast<uint8_t>(32)>(box, items);
    }
    SUBCASE("N=64")
    {
         BoxSet<std::string, static_cast<uint8_t>(64)> box;
        std::vector<std::string> items = {"apple", "banana", "cherry"};
        testFind<std::string, static_cast<uint8_t>(64)>(box, items);
    }
}