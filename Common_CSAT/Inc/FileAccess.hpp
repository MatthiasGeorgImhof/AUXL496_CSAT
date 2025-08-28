#ifndef __FILEACCESS_HPP__
#define __FILEACCESS_HPP__

#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "InputOutputStream.hpp" // For NAME_LENGTH, Error, etc.

template <typename Accessor>
concept FileAccessConcept = requires(Accessor& a, const std::array<char, NAME_LENGTH>& path, size_t offset, uint8_t* buffer, size_t& size) {
    { a.read(path, offset, buffer, size) } -> std::convertible_to<bool>;
};

class POSIXFileAccess {
public:
    bool read(const std::array<char, NAME_LENGTH>& path, size_t offset, uint8_t* buffer, size_t& size) {
        std::ifstream file(path.data(), std::ios::binary);
        if (!file.is_open()) {
            size = 0;
            return false;  // Indicate error
        }
        file.seekg(static_cast<std::streamoff>(offset));
        file.read(reinterpret_cast<char*>(buffer), static_cast<std::streamsize>(size));
        size = static_cast<size_t>(file.gcount());
        file.close();
        return true;
    }
};

static_assert(FileAccessConcept<POSIXFileAccess>, "POSIXFileAccess does not satisfy FileAccessConcept");

class ValidatedPOSIXFileAccess {
public:
    ValidatedPOSIXFileAccess(const std::string& base_path = "/") : base_path_(base_path) {}

    bool read(const std::array<char, NAME_LENGTH>& path_array, size_t offset, uint8_t* buffer, size_t& size) {
        // Convert path array to string
        std::string path(path_array.data());

        // Validate the path
        if (!isValidPath(path)) {
            size = 0;
            return false;
        }

        // Construct the full path
        std::string full_path = base_path_ + path;

        std::ifstream file(full_path, std::ios::binary);
        if (!file.is_open()) {
            size = 0;
            return false;
        }
        file.seekg(static_cast<std::streamoff>(offset));
        file.read(reinterpret_cast<char*>(buffer), static_cast<std::streamsize>(size));
        size = static_cast<size_t>(file.gcount());
        file.close();
        return true;
    }

private:
    std::string base_path_;

    bool isValidPath(const std::string& path) {
        // Check for absolute paths
        if (path[0] == '/') return false;

        // Check for ".."
        if (path.find("..") != std::string::npos) return false;

        // Check for backslashes
        if (path.find('\\') != std::string::npos) return false;

        // Add more checks as needed (e.g., check for specific characters, length limits)
        return true;
    }
};

static_assert(FileAccessConcept<ValidatedPOSIXFileAccess>, "ValidatedPOSIXFileAccess does not satisfy FileAccessConcept");

class VirtualFile {
public:
    virtual ~VirtualFile() = default;
    virtual bool open(const std::string& mode) = 0;
    virtual void close() = 0;
    virtual size_t read(size_t offset, uint8_t* buffer, size_t size) = 0;
    virtual bool isOpen() const = 0;
};

class VirtualFileSystem {
public:
    virtual ~VirtualFileSystem() = default;
    virtual std::shared_ptr<VirtualFile> openFile(const std::string& path) = 0;
};

class POSIXVirtualFile : public VirtualFile {
public:
    POSIXVirtualFile(const std::string& filename) : filename_(filename), file_() {}
    ~POSIXVirtualFile() override { close(); }

    bool open(const std::string& /*mode*/) override {
        file_.open(filename_, std::ios::binary);
        if (!file_.is_open()) return false;
        return true;
    }

    void close() override {
        if (file_.is_open()) file_.close();
    }

    size_t read(size_t offset, uint8_t* buffer, size_t size) override {
        if (!file_.is_open()) return 0;
        file_.seekg(static_cast<std::streamoff>(offset));
        file_.read(reinterpret_cast<char*>(buffer), static_cast<std::streamsize>(size));
        return static_cast<size_t>(file_.gcount());
    }

    bool isOpen() const override {
        return file_.is_open();
    }

private:
    std::string filename_;
    std::ifstream file_;
};

class POSIXVirtualFileSystem : public VirtualFileSystem {
public:
    std::shared_ptr<VirtualFile> openFile(const std::string& path) override {
        // Security checks can be placed here on the name and path
        // The checks might open the file and read the starting meta data before failing

        return std::make_shared<POSIXVirtualFile>(path);
    }
};

class VFSFileAccess {
public:
    VFSFileAccess(VirtualFileSystem& vfs) : vfs_(vfs) {}

    bool read(const std::array<char, NAME_LENGTH>& path_array, size_t offset, uint8_t* buffer, size_t& size) {
        // Convert path array to string
        std::string path(path_array.data());

        auto file = vfs_.openFile(path);
        if (!file) {
            size = 0;
            return false;
        }

        if (!file->open("rb")) {
            size = 0;
            return false;
        }

        size = file->read(offset, buffer, size);
        file->close();
        return true;
    }

private:
    VirtualFileSystem& vfs_;
};

static_assert(FileAccessConcept<VFSFileAccess>, "VFSFileAccess does not satisfy FileAccessConcept");

class InMemoryFile : public VirtualFile{
public:
    InMemoryFile(const std::vector<uint8_t>& data) : data_(data) {}

    size_t read(size_t offset, uint8_t* buffer, size_t size) override {
        if (offset >= data_.size()) return 0;
        size_t bytes_to_read = std::min(size, data_.size() - offset);
        std::copy(std::next(data_.begin(), static_cast<std::vector<uint8_t>::difference_type>(offset)),
                  std::next(data_.begin(), static_cast<std::vector<uint8_t>::difference_type>(offset + bytes_to_read)),
                  buffer);
        return bytes_to_read;
    }

    bool open(const std::string& /*mode*/) override {return true;}
    void close() override {}
    bool isOpen() const override {return true;}

    size_t size() const { return data_.size(); }

private:
    std::vector<uint8_t> data_;
};

class InMemoryFileSystem : public VirtualFileSystem { // InMemoryFileSystem now inherits from VirtualFileSystem
public:
    std::shared_ptr<VirtualFile> openFile(const std::string& path) override {
        auto it = files_.find(path);
        if (it != files_.end()) {
            return it->second;
        }
        return nullptr;
    }


    void addFile(const std::string& path, const std::vector<uint8_t>& data) {
        files_[path] = std::make_shared<InMemoryFile>(data);
    }
private:
     std::map<std::string, std::shared_ptr<VirtualFile>> files_;
};

class InMemoryFileAccess
{
public:
    InMemoryFileAccess(InMemoryFileSystem& vfs) : vfs_(vfs) {}

    bool read(const std::array<char, NAME_LENGTH>& path_array, size_t offset, uint8_t* buffer, size_t& size) {
        // Convert path array to string
        std::string path(path_array.data());

      std::shared_ptr<VirtualFile> file = vfs_.openFile(path);

        if (!file) {
            size = 0;
            return false;
        }

        size = file->read(offset, buffer, size);
        return true;
    }

private:
    InMemoryFileSystem& vfs_;
};

static_assert(FileAccessConcept<InMemoryFileAccess>, "InMemoryFileAccess does not satisfy FileAccessConcept");

#endif  // __FILEACCESS_HPP__