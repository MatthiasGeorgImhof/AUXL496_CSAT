#ifndef __FILESOURCE_HPP__
#define __FILESOURCE_HPP__

#include <array>
#include <cstddef>
#include <string>

#include "InputOutputStream.hpp" // For NAME_LENGTH, Error, etc.

template <typename Source>
concept FileSourceConcept = requires(Source &s) {
    { s.setPath(std::declval<const std::array<char, NAME_LENGTH>&>()) } -> std::convertible_to<void>;
    { s.getPath() } -> std::convertible_to<std::array<char, NAME_LENGTH>>;
    { s.offset() } -> std::convertible_to<size_t>;
    { s.setOffset(std::declval<size_t>()) } -> std::convertible_to<void>;
    { s.chunkSize() } -> std::convertible_to<size_t>;
    { s.setChunkSize(std::declval<size_t>()) } -> std::convertible_to<void>;
};

class SimpleFileSource
{
public:
    SimpleFileSource(const std::string& default_path = "default.txt") {
        std::strncpy(path_.data(), default_path.c_str(), NAME_LENGTH - 1);
        path_[NAME_LENGTH - 1] = '\0';
    }

    void setPath(const std::array<char, NAME_LENGTH>& path)
    {
        path_ = path;
    }

    std::array<char, NAME_LENGTH> getPath() const {
        return path_;
    }

    size_t offset() const { return offset_; }
    void setOffset(size_t offset) { offset_ = offset; }

    size_t chunkSize() const { return chunk_size_; }
    void setChunkSize(size_t chunk_size) { chunk_size_ = chunk_size; }

private:
    std::array<char, NAME_LENGTH> path_;
    size_t offset_ = 0;
    size_t chunk_size_ = 256;  // example default
};

static_assert(FileSourceConcept<SimpleFileSource>, "SimpleFileSource does not satisfy FileSourceConcept");

#endif // __FILESOURCE_HPP__