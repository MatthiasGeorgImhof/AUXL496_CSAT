#pragma once

#include <vector>
#include <cstring>
#include "imagebuffer/image.hpp"
#include "imagebuffer/buffer_state.hpp"
#include "ImageBuffer.hpp"   // for ImageBufferError enum

class TrivialImageBuffer
{
public:
    TrivialImageBuffer() : has_image_(false), read_offset_(0) {}

    // ------------------------------------------------------------
    // State queries
    // ------------------------------------------------------------
    bool is_empty() const { return !has_image_; }

    // Not meaningful for trivial buffer, but required by concept
    size_t size() const { return has_image_ ? payload_.size() : 0; }
    size_t count() const { return has_image_ ? 1 : 0; }
    size_t available() const { return 1 - count(); }
    size_t capacity() const { return 1; }

    // ------------------------------------------------------------
    // Producer API (TaskMLX90640 calls these)
    // ------------------------------------------------------------
    ImageBufferError add_image(const ImageMetadata& meta)
    {
        if (has_image_)
            return ImageBufferError::FULL_BUFFER;

        meta_ = meta;
        payload_.clear();
        read_offset_ = 0;
        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError add_data_chunk(const uint8_t* data, size_t size)
    {
        if (has_image_)
            return ImageBufferError::FULL_BUFFER;

        payload_.insert(payload_.end(), data, data + size);
        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError push_image()
    {
        has_image_ = true;
        return ImageBufferError::NO_ERROR;
    }

    // ------------------------------------------------------------
    // Consumer API (ImageInputStream calls these)
    // ------------------------------------------------------------
    ImageBufferError get_image(ImageMetadata& out)
    {
        if (!has_image_)
            return ImageBufferError::EMPTY_BUFFER;

        out = meta_;
        read_offset_ = 0;
        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError get_data_chunk(uint8_t* dst, size_t& size)
    {
        if (!has_image_)
            return ImageBufferError::EMPTY_BUFFER;

        size_t remaining = payload_.size() - read_offset_;
        size = std::min(size, remaining);

        if (size > 0)
        {
            std::memcpy(dst, payload_.data() + read_offset_, size);
            read_offset_ += size;
        }

        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError pop_image()
    {
        if (!has_image_)
            return ImageBufferError::EMPTY_BUFFER;

        has_image_ = false;
        payload_.clear();
        read_offset_ = 0;
        return ImageBufferError::NO_ERROR;
    }

private:
    bool has_image_;
    ImageMetadata meta_;
    std::vector<uint8_t> payload_;
    size_t read_offset_;
};
