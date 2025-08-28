#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "FileSource.hpp"

#include <cstring>
#include <algorithm>

TEST_CASE("SimpleFileSource Functionality") {
    SimpleFileSource source;

    SUBCASE("Default Initialization") {
        std::array<char, NAME_LENGTH> default_path;
        const char* src = "default.txt";
        std::copy(src, src + std::min(std::strlen(src), (size_t)NAME_LENGTH - 1), default_path.begin());
        default_path[NAME_LENGTH - 1] = '\0'; // Ensure null termination

        REQUIRE(std::strcmp(source.getPath().data(), default_path.data()) == 0);
        REQUIRE(source.offset() == 0);
        REQUIRE(source.chunkSize() == 256);
    }

    SUBCASE("Custom Initialization") {
        SimpleFileSource custom_source("my_file.bin");
        std::array<char, NAME_LENGTH> custom_path;
        const char* src = "my_file.bin";
        std::copy(src, src + std::min(std::strlen(src), (size_t)NAME_LENGTH - 1), custom_path.begin());
        custom_path[NAME_LENGTH - 1] = '\0'; // Ensure null termination
        REQUIRE(std::strcmp(custom_source.getPath().data(), custom_path.data()) == 0);
        REQUIRE(custom_source.offset() == 0);
        REQUIRE(custom_source.chunkSize() == 256);
    }

    SUBCASE("Set and Get Path") {
        std::array<char, NAME_LENGTH> new_path;
        const char* src = "another_file.txt";
        std::copy(src, src + std::min(std::strlen(src), (size_t)NAME_LENGTH - 1), new_path.begin());
        new_path[NAME_LENGTH - 1] = '\0'; // Ensure null termination

        source.setPath(new_path);
        REQUIRE(std::strcmp(source.getPath().data(), new_path.data()) == 0);
    }

    SUBCASE("Set and Get Offset") {
        source.setOffset(12345);
        REQUIRE(source.offset() == 12345);
    }

    SUBCASE("Set and Get Chunk Size") {
        source.setChunkSize(512);
        REQUIRE(source.chunkSize() == 512);
    }
}

TEST_CASE("FileSourceConcept Static Assert") {
    // This test case doesn't directly test anything at runtime.
    // The `static_assert` in FileSourceConcept.hpp ensures at compile time
    // that SimpleFileSource satisfies the FileSourceConcept.

    // If the static_assert fails, the code won't compile.  If it compiles,
    // the test case implicitly passes.

    // We can add a REQUIRE(true) here to make the test case explicit, but it's not necessary.
    REQUIRE(true); // Just to have an explicit REQUIRE
}