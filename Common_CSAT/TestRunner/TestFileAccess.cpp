#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "FileAccess.hpp"

#include <array>
#include <cstring>
#include <fstream>
#include <algorithm>
#include <memory>

TEST_CASE("POSIXFileAccess Test") {
    // Create a test file with known content and size
    constexpr char NAME[] = "posix.txt";
    
    const char* test_content = "This is a test file.";
    size_t content_length = std::strlen(test_content);
    std::ofstream test_file(NAME, std::ios::binary);
     test_file << "This is a test file.";
    if (test_file.fail())
    {
        std::cout << "TestFile write failure " << std::endl;
    }
    test_file.close();

    POSIXFileAccess file_access;
    std::array<char, NAME_LENGTH> path{};

    const char* src = NAME;

    std::copy(src, src + std::min((size_t)NAME_LENGTH - 1, std::strlen(src)), path.begin());
    path[NAME_LENGTH - 1] = '\0'; // Ensure null termination

    uint8_t buffer[100] = {};
    size_t size = sizeof(buffer);
    size_t initial_size = size;

    SUBCASE("File Exists and Can Be Read") {
        REQUIRE(file_access.read(path, 0, buffer, size));
        REQUIRE(size > 0);
        REQUIRE(size <= initial_size);
        REQUIRE(std::strncmp(reinterpret_cast<char*>(buffer), test_content, size) == 0);
    }

    SUBCASE("Offset Works Correctly") {
        size = sizeof(buffer);
        REQUIRE(file_access.read(path, 5, buffer, size));
        REQUIRE(size > 0);
        REQUIRE(size <= initial_size);
        REQUIRE(std::strncmp(reinterpret_cast<char*>(buffer), "is a test file.", sizeof("is a test file.")-1) == 0); // Verify content after offset
    }

    SUBCASE("Read Less Than File Size") {
        size = 5;  // Request a small chunk
        REQUIRE(file_access.read(path, 0, buffer, size));
        REQUIRE(size == 5);
        REQUIRE(std::strncmp(reinterpret_cast<char*>(buffer), "This ", size) == 0);
    }

    SUBCASE("Read More Than File Size") {
        size = 200; // Request much more than available
        REQUIRE(file_access.read(path, 0, buffer, size));
        REQUIRE(size == content_length);  // Should read up to EOF
        REQUIRE(std::strncmp(reinterpret_cast<char*>(buffer), test_content, size) == 0);
    }

    SUBCASE("Read At EOF") {
        size = sizeof(buffer);
        REQUIRE(file_access.read(path, content_length, buffer, size));
        REQUIRE(size == 0); // Should read nothing
    }

    SUBCASE("Read Near EOF") {
        size = sizeof(buffer);
        REQUIRE(file_access.read(path, content_length - 3, buffer, size));
        REQUIRE(size == 3);
        REQUIRE(std::strncmp(reinterpret_cast<char*>(buffer), "le.", size) == 0);
    }

    SUBCASE("File Does Not Exist") {
        std::array<char, NAME_LENGTH> non_existent_path;

        const char* src = "nonexistent.txt";
        std::copy(src, src + std::min((size_t)NAME_LENGTH - 1, std::strlen(src)), non_existent_path.begin());
        non_existent_path[NAME_LENGTH - 1] = '\0';  // Ensure null termination
        size = sizeof(buffer);

        REQUIRE_FALSE(file_access.read(non_existent_path, 0, buffer, size));
        REQUIRE(size == 0);
    }

    // Clean up the test file
    std::remove(NAME);
}

TEST_CASE("ValidatedPOSIXFileAccess Test") {
    // Create a test file
    constexpr char NAME[] = "valid.txt";
    std::ofstream test_file(NAME, std::ios::binary);
       test_file << "This is a validated test file.";
    if (test_file.fail())
    {
        std::cout << "TestFile write failure " << std::endl;
    }
    test_file.close();

    ValidatedPOSIXFileAccess file_access("./"); // Base path is the current directory
    std::array<char, NAME_LENGTH> path;

    const char* src = NAME;
    std::copy(src, src + std::min((size_t)NAME_LENGTH - 1, std::strlen(src)), path.begin());
    path[NAME_LENGTH - 1] = '\0';

    uint8_t buffer[100] = {};
    size_t size = sizeof(buffer);
    size_t initial_size = size;

    SUBCASE("File Exists and Can Be Read") {
        REQUIRE(file_access.read(path, 0, buffer, size));
        REQUIRE(size > 0);
        REQUIRE(size <= initial_size);
    }

    SUBCASE("Invalid Path - Traversal Attempt") {
        std::array<char, NAME_LENGTH> traversal_path;
        const char* src = "../test_validated.txt";

        std::copy(src, src + std::min((size_t)NAME_LENGTH - 1, std::strlen(src)), traversal_path.begin());
        traversal_path[NAME_LENGTH - 1] = '\0';
        size = sizeof(buffer);

        REQUIRE_FALSE(file_access.read(traversal_path, 0, buffer, size));
        REQUIRE(size == 0); //Size should be 0 as file shouldn't be read
    }

    SUBCASE("File Does Not Exist") {
        std::array<char, NAME_LENGTH> non_existent_path;

        const char* src = "nonexistent.txt";
        std::copy(src, src + std::min((size_t)NAME_LENGTH - 1, std::strlen(src)), non_existent_path.begin());
        non_existent_path[NAME_LENGTH - 1] = '\0';
        size = sizeof(buffer);

        REQUIRE_FALSE(file_access.read(non_existent_path, 0, buffer, size));
        REQUIRE(size == 0);
    }

    // Clean up the test file
    std::remove(NAME);
}

TEST_CASE("Virtual File System Test") {
   class MockVirtualFile : public VirtualFile {
    public:
        MockVirtualFile(const std::string& content) : content_(content) {}
        bool open(const std::string& /*mode*/) override { return true; }
        void close() override {}

        size_t read(size_t offset, uint8_t* buffer, size_t size) override {
            if (offset >= content_.size()) return 0;
            size_t bytes_to_read = std::min(size, content_.size() - offset);
            std::copy(std::next(content_.begin(), static_cast<std::string::difference_type>(offset)),
                      std::next(content_.begin(), static_cast<std::string::difference_type>(offset + bytes_to_read)),
                      buffer);
            return bytes_to_read;
        }

        bool isOpen() const override { return true; }

    private:
        std::string content_;
    };

    class MockVirtualFileSystem : public VirtualFileSystem {
    public:
        MockVirtualFileSystem(const std::string& content) : content_(content) {}
        std::shared_ptr<VirtualFile> openFile(const std::string& /*path*/) override {
            return std::make_shared<MockVirtualFile>(content_);
        }
    private:
        std::string content_;
    };
    const char* test_content = "This is a VFS test file.";
    MockVirtualFileSystem vfs(test_content);

    VFSFileAccess file_access(vfs);
    std::array<char, NAME_LENGTH> path;
    const char* src = "test_vfs.txt";
    std::copy(src, src + std::min((size_t)NAME_LENGTH - 1, std::strlen(src)), path.begin());
    path[NAME_LENGTH - 1] = '\0';

    uint8_t buffer[100];
    size_t size = sizeof(buffer);
    size_t initial_size = size;
    size_t content_length = std::strlen(test_content);

    SUBCASE("File Exists and Can Be Read") {
        REQUIRE(file_access.read(path, 0, buffer, size));
        REQUIRE(size > 0);
        REQUIRE(size <= initial_size);
          REQUIRE(std::strncmp(reinterpret_cast<char*>(buffer), test_content, size) == 0);
    }
        SUBCASE("Read Less Than File Size") {
        size = 5;  // Request a small chunk
        REQUIRE(file_access.read(path, 0, buffer, size));
        REQUIRE(size == 5);
        REQUIRE(std::strncmp(reinterpret_cast<char*>(buffer), "This ", size) == 0);
    }

    SUBCASE("Read More Than File Size") {
        size = 200; // Request much more than available
        REQUIRE(file_access.read(path, 0, buffer, size));
        REQUIRE(size == content_length);  // Should read up to EOF
          REQUIRE(std::strncmp(reinterpret_cast<char*>(buffer), test_content, size) == 0);
    }

    SUBCASE("Read At EOF") {
        size = sizeof(buffer);
        REQUIRE(file_access.read(path, content_length, buffer, size));
        REQUIRE(size == 0); // Should read nothing
    }

    SUBCASE("Read Near EOF") {
        size = sizeof(buffer);
        REQUIRE(file_access.read(path, content_length - 3, buffer, size));
        REQUIRE(size >= 0);
        REQUIRE(size <=3);
    }
}

TEST_CASE("Virtual File System Test inexistant file") {
   class MockVirtualFile : public VirtualFile {
    public:
        MockVirtualFile(const std::string& content) : content_(content) {}
        bool open(const std::string& /*mode*/) override { return false; }
        void close() override {}

        size_t read(size_t offset, uint8_t* buffer, size_t size) override {
            if (offset >= content_.size()) return 0;
            size_t bytes_to_read = std::min(size, content_.size() - offset);
            auto begin = std::next(content_.begin(), static_cast<std::string::difference_type>(offset));
            auto end = std::next(begin, static_cast<std::string::difference_type>(bytes_to_read));
            std::copy(begin, end, buffer);
            return bytes_to_read;
        }

        bool isOpen() const override { return true; }

    private:
        std::string content_;
    };

    class MockVirtualFileSystem : public VirtualFileSystem {
    public:
        MockVirtualFileSystem(const std::string& content) : content_(content) {}
        std::shared_ptr<VirtualFile> openFile(const std::string& /*path*/) override {
            return std::make_shared<MockVirtualFile>(content_);
        }
    private:
        std::string content_;
    };
    const char* test_content = "This is a VFS test file.";
    MockVirtualFileSystem vfs(test_content);

    VFSFileAccess file_access(vfs);
    std::array<char, NAME_LENGTH> path;
    const char* src = "test_vfs.txt";
    std::copy(src, src + std::min((size_t)NAME_LENGTH - 1, std::strlen(src)), path.begin());
    path[NAME_LENGTH - 1] = '\0';

    uint8_t buffer[100];
    size_t size = sizeof(buffer);

    SUBCASE("File Does Not Exist") {
        std::array<char, NAME_LENGTH> non_existent_path;
        const char* src = "nonexistent.txt";
        std::copy(src, src + std::min((size_t)NAME_LENGTH - 1, std::strlen(src)), non_existent_path.begin());
        non_existent_path[NAME_LENGTH - 1] = '\0';
        size = sizeof(buffer);
        REQUIRE_FALSE(file_access.read(non_existent_path, 0, buffer, size));
        REQUIRE(size == 0);
    }
}

TEST_CASE("InMemoryFileAccess Test") {
    constexpr char NAME[] = "memory.txt";

    InMemoryFileSystem vfs;
    std::vector<uint8_t> data = {'H', 'e', 'l', 'l', 'o', ' ', 'I', 'n', ' ', 'M', 'e', 'm', 'o', 'r', 'y'};
    vfs.addFile(NAME, data);
    size_t content_length = data.size();

    InMemoryFileAccess file_access(vfs);
    std::array<char, NAME_LENGTH> path{};

    const char* src = NAME;
    std::copy(src, src + std::min((size_t)NAME_LENGTH - 1, std::strlen(src)), path.begin());
    path[NAME_LENGTH - 1] = '\0';

    uint8_t buffer[100];
    size_t size = sizeof(buffer);
    size_t initial_size = size;

    SUBCASE("File Exists and Can Be Read") {
        REQUIRE(file_access.read(path, 0, buffer, size));
        REQUIRE(size > 0);
        REQUIRE(size <= initial_size); // Because InMemoryAccess returns actual bytes read
        REQUIRE(std::strncmp(reinterpret_cast<char*>(buffer), "Hello ", 6) == 0);
    }

    SUBCASE("Read with Offset") {
        size = sizeof(buffer);
        REQUIRE(file_access.read(path, 6, buffer, size));
        REQUIRE(size > 0);
        REQUIRE(size <= initial_size);
        REQUIRE(std::strncmp(reinterpret_cast<char*>(buffer), "I", 1) == 0);
    }

    SUBCASE("Read less than file size") {
        size = 5;
        REQUIRE(file_access.read(path, 0, buffer, size));
        REQUIRE(size == 5);
        REQUIRE(std::strncmp(reinterpret_cast<char*>(buffer), "Hello", size) == 0);
    }

        SUBCASE("Read More Than File Size") {
        size = 200; // Request much more than available
        REQUIRE(file_access.read(path, 0, buffer, size));
        REQUIRE(size == content_length);  // Should read up to EOF
        REQUIRE(std::strncmp(reinterpret_cast<char*>(buffer), (const char*)data.data(), size) == 0);
    }
    SUBCASE("Read At EOF") {
        size = sizeof(buffer);
        REQUIRE(file_access.read(path, content_length, buffer, size));
        REQUIRE(size == 0); // Should read nothing
    }

    SUBCASE("Read Near EOF") {
        size = sizeof(buffer);
        REQUIRE(file_access.read(path, content_length - 3, buffer, size));
        REQUIRE(size >= 0);
        REQUIRE(size <=3);
        REQUIRE(std::strncmp(reinterpret_cast<char*>(buffer), (const char*)(data.data()+content_length-3), size) == 0);
    }


    SUBCASE("File Does Not Exist") {
        std::array<char, NAME_LENGTH> non_existent_path;

        const char* src = "nonexistent.txt";
        std::copy(src, src + std::min((size_t)NAME_LENGTH - 1, std::strlen(src)), non_existent_path.begin());
        non_existent_path[NAME_LENGTH - 1] = '\0';
        size = sizeof(buffer);

        REQUIRE_FALSE(file_access.read(non_existent_path, 0, buffer, size));
        REQUIRE(size == 0);
    }
}