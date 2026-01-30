#ifndef TRIVIAL_IMAGE_BUFFER_HPP
#define TRIVIAL_IMAGE_BUFFER_HPP

#include <array>
#include <cstring>
#include "imagebuffer/image.hpp"
#include "imagebuffer/buffer_state.hpp"
#include "ImageBuffer.hpp"   // for ImageBufferError enum
#include "ImageBufferConcept.hpp"   // for ImageBufferError enum
#include "Logger.hpp"

template <size_t N>
class TrivialImageBuffer
{
public:
    TrivialImageBuffer()
        : has_image_(false),
          payload_size_(0),
          read_offset_(0)
    {}

    // ------------------------------------------------------------
    // State queries
    // ------------------------------------------------------------
    bool is_empty() const { return !has_image_; }

    bool has_room_for(size_t size) const
    {
        return !has_image_ && (size <= N);
    }

    size_t count() const { return has_image_ ? 1 : 0; }

    size_t size() const { return has_image_ ? payload_size_ : 0; }

    size_t available() const { return has_image_ ? 0 : N; }

    size_t capacity() const { return N; }

    // ------------------------------------------------------------
    // Producer API
    // ------------------------------------------------------------
    ImageBufferError add_image(const ImageMetadata& meta)
    {
        if (has_image_)
            return ImageBufferError::FULL_BUFFER;

        meta_ = meta;
        payload_size_ = 0;
        read_offset_ = 0;
        log(LOG_LEVEL_DEBUG, "TrivialImageBuffer::add_image\r\n");
        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError add_data_chunk(const uint8_t* data, size_t size)
    {
        if (has_image_)
            return ImageBufferError::FULL_BUFFER;

        if (payload_size_ + size > N)
            return ImageBufferError::OUT_OF_BOUNDS;

        std::memcpy(payload_.data() + payload_size_, data, size);
        payload_size_ += size;

        log(LOG_LEVEL_DEBUG, "TrivialImageBuffer::add_data_chunk\r\n");
        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError push_image()
    {
        has_image_ = true;
        log(LOG_LEVEL_DEBUG, "TrivialImageBuffer::push_image\r\n");
        return ImageBufferError::NO_ERROR;
    }

    // ------------------------------------------------------------
    // Consumer API
    // ------------------------------------------------------------
    ImageBufferError get_image(ImageMetadata& out)
    {
        if (!has_image_)
            return ImageBufferError::EMPTY_BUFFER;

        out = meta_;
        read_offset_ = 0;
        log(LOG_LEVEL_DEBUG, "TrivialImageBuffer::get_image\r\n");
        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError get_data_chunk(uint8_t* dst, size_t& size)
    {
        if (!has_image_)
            return ImageBufferError::EMPTY_BUFFER;

        size_t remaining = payload_size_ - read_offset_;
        size = std::min(size, remaining);

        if (size > 0)
        {
            std::memcpy(dst, payload_.data() + read_offset_, size);
            read_offset_ += size;
        }
        log(LOG_LEVEL_DEBUG, "TrivialImageBuffer::get_data_chunk\r\n");

        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError pop_image()
    {
        if (!has_image_)
            return ImageBufferError::EMPTY_BUFFER;

        has_image_ = false;
        payload_size_ = 0;
        read_offset_ = 0;
        log(LOG_LEVEL_DEBUG, "TrivialImageBuffer::pop_image\r\n");

        return ImageBufferError::NO_ERROR;
    }

private:
    bool has_image_;
    ImageMetadata meta_;

    std::array<uint8_t, N> payload_;
    size_t payload_size_;   // how many bytes are valid
    size_t read_offset_;    // how many bytes have been consumed
};

#endif // TRIVIAL_IMAGE_BUFFER_HPP
