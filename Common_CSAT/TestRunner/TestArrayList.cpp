#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "ArrayList.hpp"
#include <string>
#include <algorithm> // For std::equal

TEST_CASE("ArrayList Construction and Basic Operations") {
	ArrayList<int, 5> list;
	CHECK(list.empty());
	CHECK(list.size() == 0);
	CHECK(list.capacity() == 5);
	CHECK(!list.full());

	list.push(10);
	CHECK(!list.empty());
	CHECK(list.size() == 1);
	CHECK(list[0] == 10);

	list.push(20);
	CHECK(list.size() == 2);
	CHECK(list[1] == 20);

	list.push(30);
	list.push(40);
	list.push(50);
	CHECK(list.full());

	const ArrayList<int, 5> constList = list;
	CHECK(constList[0] == 10);
	CHECK(constList.size() == 5);
	CHECK(constList.full());
}

TEST_CASE("ArrayList Remove Operation"){
	ArrayList<int, 5> list;
	list.push(10);
	list.push(20);
	list.push(30);
	list.push(40);
	list.push(50);

	// remove from middle
	list.remove(1);
	CHECK(list.size() == 4);
	CHECK(list[0] == 10);
	CHECK(list[1] == 30);
	CHECK(list[2] == 40);
	CHECK(list[3] == 50);

	// remove last
	list.remove(3);
	CHECK(list.size() == 3);
	CHECK(list[0] == 10);
	CHECK(list[1] == 30);
	CHECK(list[2] == 40);

	// remove first
	list.remove(0);
	CHECK(list.size() == 2);
	CHECK(list[0] == 30);
	CHECK(list[1] == 40);

	// fail quiet remove
	list.remove(10);
	CHECK(list.size() == 2);

	// remove all elements
	list.remove(0);
	list.remove(0);
	CHECK(list.size() == 0);
	CHECK(list.empty());
}

TEST_CASE("ArrayList Iterator") {
	ArrayList<int, 5> list;
	list.push(10);
	list.push(20);
	list.push(30);

	auto it = list.begin();
	CHECK(*it == 10);
	++it;
	CHECK(*it == 20);
	it++;
	CHECK(*it == 30);
	++it;
	CHECK(it == list.end());

	int sum = 0;
	for (auto i : list){
		sum+= i;
	}
	CHECK(sum == 60);

	const ArrayList<int,5> constList = list;
	int constSum = 0;
	for (auto i : constList){
		constSum+= i;
	}
	CHECK(constSum == 60);

	// check pre and post decrement
	auto it2 = list.end();
	--it2;
	CHECK(*it2 == 30);
	it2--;
	CHECK(*it2 == 20);

	// check + and - operation
	auto it3 = list.begin();
	CHECK(*(it3+2) == 30);
	CHECK(*(it3- (-1)) == 20);

	auto it4 = it3 + 2;
	CHECK(it4 - it3 == 2);

	//check []
			 CHECK(it3[1] == 20);

			 //check comparison
			 auto beginIt = list.begin();
			 auto endIt = list.end();
			 CHECK(beginIt < endIt);
			 CHECK(beginIt <= endIt);
			 CHECK(endIt > beginIt);
			 CHECK(endIt >= beginIt);
			 CHECK(beginIt == list.begin());
			 CHECK(beginIt != list.end());

}

TEST_CASE("ArrayList Out of Bounds access"){
	ArrayList<int,5> list;
	CHECK(list[0] == 0); // should return a default constructed int
	list.push(10);
	CHECK(list[10] == 0); // should return a default constructed int
	CHECK(list[0] == 10);

	const ArrayList<int,5> constList = list;
	CHECK(constList[0] == 10);
	CHECK(constList[10] == 0);
}


TEST_CASE("ArrayList pushOrReplace operation"){
	ArrayList<int, 5> list;
	list.push(1);
	list.push(2);
	list.push(3);

	// Add 4 or replace if it exists
	list.pushOrReplace(4, [](int a, int b) {return a == b;});

	list.pushOrReplace(4, [](int a, int b) {return a == b;});

	CHECK(list[0] == 1);
	CHECK(list[1] == 2);
	CHECK(list[2] == 3);
	CHECK(list[3] == 4);

	//Replace 4 if it exists
	list.pushOrReplace(5, [](int a, int /*b*/) {return a == 4;});

	CHECK(list[0] == 1);
	CHECK(list[1] == 2);
	CHECK(list[2] == 3);
	CHECK(list[3] == 5);

	ArrayList<std::string, 3> names;
	names.push("Alice");
	names.push("Bob");

	// Replace "Alice" if exists with "Alice"
	names.pushOrReplace("Alice", [](std::string a, std::string b) { return a == b;});
	CHECK(names[0] == "Alice");
	CHECK(names[1] == "Bob");

	// Replace "Bob" if exists with "Charlie"
	names.pushOrReplace("Charlie", [](std::string a, std::string /*b*/) { return a == "Bob";});

	CHECK(names[0] == "Alice");
	CHECK(names[1] == "Charlie");

	//Add "David"
	names.pushOrReplace("David", [](std::string a, std::string b) { return a == b;});
	CHECK(names[0] == "Alice");
	CHECK(names[1] == "Charlie");
	CHECK(names[2] == "David");
}

TEST_CASE("ArrayList Find Operation") {
    constexpr size_t list_capacity = 5;
	ArrayList<int, list_capacity> list;
    list.push(10);
    list.push(20);
    list.push(30);

    // Find an existing element
    size_t index1 = list.find(20, [](int a, int b) { return a == b; });
    CHECK(index1 == 1);
    CHECK(list[index1] == 20);

    // Find a non-existing element (should add it)
    size_t index2 = list.find(40, [](int a, int b) { return a == b; });
	CHECK(index2 == list_capacity);
	list.push(40);
	index2 = list.find(40, [](int a, int b) { return a == b; });
    CHECK(list[index2] == 40);
	CHECK(list.size() == 4);

	//Find an existing element after adding a new one
    size_t index3 = list.find(10, [](int a, int b) { return a == b; });
	CHECK(index3 == 0);
	CHECK(list[index3] == 10);
	CHECK(list.size() == 4);

    constexpr size_t names_capacity = 4;
	ArrayList<std::string, names_capacity> names;
	names.push("Alice");
	names.push("Bob");
	names.push("Charlie");
	// Find an existing string
	size_t index4 = names.find("Bob", [](const std::string& a, const std::string& b) { return a == b; });
	CHECK(index4 == 1);
	CHECK(names[index4] == "Bob");
	// Find a non existing string (should add it)
	size_t index5 = names.find("David", [](const std::string& a, const std::string& b) { return a == b; });
	CHECK(index5 == names_capacity);
	names.push("David");
	index5 = names.find("David", [](const std::string& a, const std::string& b) { return a == b; });
	CHECK(names[index5] == "David");
	CHECK(names.size() == 4);
}

TEST_CASE("ArrayList RemoveIf Operation") {
    ArrayList<int, 5> list;
    list.push(1);
    list.push(2);
    list.push(3);
    list.push(4);
    list.push(5);

    // Remove all even numbers
    list.removeIf([](int x) { return x % 2 == 0; });
    CHECK(list.size() == 3);
    CHECK(list[0] == 1);
    CHECK(list[1] == 3);
    CHECK(list[2] == 5);

    // Remove all numbers greater than 3
    list.removeIf([](int x) { return x > 3; });
    CHECK(list.size() == 2);
    CHECK(list[0] == 1);
    CHECK(list[1] == 3);

    // Remove all numbers (should be empty)
    list.removeIf([](int /*x*/) { return true; });
    CHECK(list.empty());

    ArrayList<std::string, 4> names;
    names.push("Alice");
    names.push("Bob");
    names.push("Charlie");
    names.push("David");

    // Remove names containing "a"
    names.removeIf([](const std::string& name) { return name.find('a') != std::string::npos; });
    CHECK(names.size() == 2);
    CHECK(names[0] == "Alice");
    CHECK(names[1] == "Bob");

    //Remove names equal to Alice
    names.removeIf([](const std::string& name) { return name == "Alice";});
    CHECK(names.size() == 1);


    //Remove names equal to Bob
    names.removeIf([](const std::string& name) { return name == "Bob";});
    CHECK(names.empty());
}

TEST_CASE("ArrayList Equality Operator"){
		ArrayList<int, 5> list1;
		list1.push(1);
		list1.push(2);
		list1.push(3);

		ArrayList<int, 5> list2;
		list2.push(1);
		list2.push(2);
		list2.push(3);

		ArrayList<int, 5> list3;
		list3.push(3);
		list3.push(2);
		list3.push(1);

		ArrayList<int, 4> list4;
		list4.push(1);
		list4.push(2);
		list4.push(3);


		// Check if the lists contain the same elements in the same order
		CHECK(std::equal(list1.begin(), list1.end(), list2.begin(), list2.end()));

		// The sizes are different and should compare equal
		CHECK(list1.size() == list4.size());

		// The lists do not contain the same element and should not compare equal
		CHECK(!std::equal(list1.begin(), list1.end(), list3.begin(), list3.end()));
}

TEST_CASE("ArrayList containsIf") {
    ArrayList<int, 5> list;
    list.push(1);
    list.push(2);
    list.push(3);

    CHECK(list.containsIf([](int x) { return x > 1; }));
    CHECK(!list.containsIf([](int x) { return x > 3; }));
}

TEST_CASE("ArrayList Iterator -> operator") {
    struct MyStruct {
        int value;
    };

    ArrayList<MyStruct, 3> list;
    list.push({10});
    list.push({20});

    auto it = list.begin();
    CHECK(it->value == 10);
    ++it;
    CHECK(it->value == 20);
}

TEST_CASE("ArrayList Iterator empty list") {
	ArrayList<int,5> list;
	CHECK(list.begin() == list.end());

	// Don't dereference an empty iterator, as it's undefined behavior
}

TEST_CASE("ArrayList Iterator Arithmetic out of bounds"){
	ArrayList<int, 5> list;
	list.push(10);
	list.push(20);
	list.push(30);

	auto begin = list.begin();
	auto end = list.end();

	//Moving begin to the end or beyond is valid
	auto it = begin += 3;
	CHECK(it == end);

	//Moving end beyond is still valid. Note that you should avoid dereferencing it
	auto it2 = end += 5;
	CHECK(it2 == end);

	// Moving it3 backwards is valid.
	auto it3 = begin -=3;
	CHECK(it3 == begin);
}

TEST_CASE("ArrayList Copy Constructor and Assignment") {
    ArrayList<int, 3> list1;
    list1.push(1);
    list1.push(2);
    list1.push(3);

    ArrayList<int, 3> list2 = list1; // Copy constructor
    CHECK(list2.size() == 3);
    CHECK(list2[0] == 1);
    CHECK(list2[1] == 2);
    CHECK(list2[2] == 3);

    ArrayList<int, 3> list3;
    list3 = list1; // Assignment operator
    CHECK(list3.size() == 3);
    CHECK(list3[0] == 1);
    CHECK(list3[1] == 2);
    CHECK(list3[2] == 3);

    list1[0] = 5; // Modify list1
    CHECK(list2[0] == 1); // list2 should not be affected
    CHECK(list3[0] == 1); // list3 should not be affected

	// Make sure the dummy members are different.
	CHECK(&(list1[10]) != &(list2[10]));
	CHECK(&(list1[10]) != &(list3[10]));
}

TEST_CASE("ArrayList Move Constructor and Assignment") {
	ArrayList<int, 3> list1;
	list1.push(1);
	list1.push(2);
	list1.push(3);

	ArrayList<int, 3> list2 = std::move(list1); // Move constructor
	CHECK(list2.size() == 3);
	CHECK(list2[0] == 1);
	CHECK(list2[1] == 2);
	CHECK(list2[2] == 3);

	// list1 should be in a valid but unspecified state.  Typically empty in this case.
	CHECK(list1.size() == 0);

	ArrayList<int, 3> list3;
	list3.push(4);
	list3.push(5);
	list3.push(6);

	list1 = std::move(list3); // Move assignment
	CHECK(list1.size() == 3);
	CHECK(list1[0] == 4);
	CHECK(list1[1] == 5);
	CHECK(list1[2] == 6);

	// list3 should be in a valid but unspecified state.
	CHECK(list3.size() == 0);
}

TEST_CASE("ArrayList Const Correctness") {
	const ArrayList<int, 3> list;
	CHECK(list.size() == 0);
	CHECK(list.empty());
	CHECK(!list.full());
	CHECK(list.capacity() == 3);

	ArrayList<int, 3> list2;
	CHECK(list2.size() == 0);
	CHECK(list2.empty());
	CHECK(!list2.full());
	CHECK(list2.capacity() == 3);
}

TEST_CASE("ArrayList pushOrReplace - Stressed") {
    ArrayList<int, 5> list;

    // Empty List
    list.pushOrReplace(1, [](int a, int b) { return a == b; });
    CHECK(list.size() == 1);
    CHECK(list[0] == 1);

    // Consecutive replacements
    list.pushOrReplace(2, [](int a, int /*b*/) { return a == 1; });
    CHECK(list.size() == 1);
    CHECK(list[0] == 2);
    list.pushOrReplace(3, [](int a, int /*b*/) { return a == 2; });
    CHECK(list.size() == 1);
    CHECK(list[0] == 3);

	// Adding more elements for the next test case
	list.push(4);
	list.push(5);
	CHECK(list.size() == 3);

	// pushOrReplace with multiple existing elements that would match the same predicate
	list.pushOrReplace(6, [](int a, int /*b*/) { return a > 0;});
	CHECK(list[0] == 6);
	CHECK(list[1] == 4);
	CHECK(list[2] == 5);

    // Full List
    list.push(7);
    list.push(8);
    CHECK(list.full());
    list.pushOrReplace(9, [](int a, int /*b*/) { return a == 6; }); // Replace existing element
    CHECK(list.size() == 5);
    CHECK(list[0] == 9);
    CHECK(list[1] == 4);
    CHECK(list[2] == 5);
    CHECK(list[3] == 7);
    CHECK(list[4] == 8);
}

TEST_CASE("ArrayList pushOrReplace - Complex Comparison") {
	struct MyStruct {
		int id;
		std::string name;
	};

	ArrayList<MyStruct, 3> list;
	list.push({1, "Alice"});
	list.push({2, "Bob"});

	// Replace based on ID only
	list.pushOrReplace({1, "Charlie"}, [](const MyStruct& a, const MyStruct& b) { return a.id == b.id; });
	CHECK(list[0].id == 1);
	CHECK(list[0].name == "Charlie");
	CHECK(list[1].id == 2);
	CHECK(list[1].name == "Bob");

	// Replacement where the compared value is same, but other members change.
	list.pushOrReplace({1, "David"}, [](const MyStruct& a, const MyStruct& /*b*/) { return a.id == 1; });
	CHECK(list[0].id == 1);
	CHECK(list[0].name == "David");
	CHECK(list[1].id == 2);
	CHECK(list[1].name == "Bob");
}

TEST_CASE("ArrayList removeIf - Stressed") {
    ArrayList<int, 5> list;
    list.push(1);
    list.push(2);
    list.push(3);
    list.push(4);
    list.push(5);

    // Remove all elements
    list.removeIf([](int /*x*/) { return true; });
    CHECK(list.empty());

    // Repopulate
    list.push(1);
    list.push(2);
    list.push(3);

    // Remove no elements
    list.removeIf([](int /*x*/) { return false; });
    CHECK(list.size() == 3);

    // Consecutive removeIf calls
    list.removeIf([](int x) { return x % 2 != 0; }); // Remove odd
    CHECK(list.size() == 1);
    CHECK(list[0] == 2);
    list.removeIf([](int x) { return x == 2; }); // Remove 2
    CHECK(list.empty());

    // removeIf where elements that are at the beginning, middle and the end of the list are all matched:
    list.push(1);
    list.push(2);
    list.push(3);
    list.push(4);
    list.push(5);
    list.removeIf([](int x){return x % 2 != 0;});
    CHECK(list.size() == 2);
    CHECK(list[0] == 2);
    CHECK(list[1] == 4);
}

TEST_CASE("ArrayList removeIf - Complex Predicate") {
	struct MyStruct {
		int id;
		std::string name;
	};

	ArrayList<MyStruct, 4> list;
	list.push({1, "Alice"});
	list.push({2, "Bob"});
	list.push({3, "Alice"});
	list.push({4, "David"});

	// Remove based on name "Alice"
	list.removeIf([](const MyStruct& s) { return s.name == "Alice"; });
	CHECK(list.size() == 2);
	CHECK(list[0].id == 2);
	CHECK(list[1].id == 4);

	// Remove all structs with ID > 2
	list.removeIf([](const MyStruct& s) { return s.id > 2; });
	CHECK(list.size() == 1);
	CHECK(list[0].id == 2);
}