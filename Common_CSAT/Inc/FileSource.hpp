// FileSource concept and a simple implementation of it.
#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <algorithm>
#include <cstring>

template <typename Source>
concept FileSourceConcept = requires(Source &s)
{
    { s.getPath() } -> std::same_as<const std::array<char, NAME_LENGTH>&>;
    { s.getPathLength() } -> std::convertible_to<size_t>;
};

class SimpleFileSource
{
public:
    SimpleFileSource(std::string_view path = "default.txt")
    {  
        setPath(path);
    }

    void setPath(const std::string_view& path)
    {
        path_length_ = std::min<std::size_t>(NAME_LENGTH, path.length());
        std::memset(path_.data(), 0, NAME_LENGTH);
        std::memcpy(path_.data(), path.data(), path_length_);
        path_.data()[std::min(NAME_LENGTH, path_length+1)] = '\0';
    }

    const std::array<char, NAME_LENGTH>& getPath() const {
        return path_;
    }

    size_t getPathLength() const {
        return path_length_;
    }

private:
    std::array<char, NAME_LENGTH> path_{};
    size_t path_length_{0};
};

static_assert(FileSourceConcept<SimpleFileSource>, "SimpleFileSource does not satisfy FileSourceConcept");
