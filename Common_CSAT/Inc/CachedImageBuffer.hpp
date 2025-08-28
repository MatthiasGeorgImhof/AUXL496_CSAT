#ifndef CACHED_IMAGE_BUFFER_H
#define CACHED_IMAGE_BUFFER_H

#include <cstdint>
#include <cstddef>
#include <vector>
#include <cstring>
#include <algorithm>
#include <functional>
#include <iostream>
#include <numeric>
#include "imagebuffer/image.hpp"
#include "imagebuffer/imagebuffer.hpp"
#include "Checksum.hpp"

typedef ImageBufferError CachedImageBufferError;
typedef BufferState CachedBufferState; 

template <typename Accessor>
class CachedImageBuffer
{
public:
    CachedImageBuffer(Accessor &access)
        : buffer_state_(0, 0, 0, access.getFlashStartAddress(), access.getFlashMemorySize()), access_(access) {}

    CachedImageBufferError add_image(const ImageMetadata &metadata);
    CachedImageBufferError add_data_chunk(const uint8_t *data, size_t size);
    CachedImageBufferError push_image();

    CachedImageBufferError get_image(ImageMetadata &metadata);
    CachedImageBufferError get_data_chunk(uint8_t *data, size_t &size);
    CachedImageBufferError pop_image();


    inline bool is_empty() const { return buffer_state_.is_empty(); };
    inline size_t size() const { return buffer_state_.size(); };
    inline size_t count() const { return buffer_state_.count(); };
    inline size_t available() const { return buffer_state_.available(); };
    inline size_t capacity() const { return buffer_state_.capacity(); };

    inline size_t get_head() const { return buffer_state_.get_head(); }
    inline size_t get_tail() const { return buffer_state_.get_tail(); }

private:
    bool has_enough_space(size_t data_size) const { return (buffer_state_.available() >= data_size); };
    void wrap_around(size_t &address);
    CachedImageBufferError write(size_t address, const uint8_t *data, size_t size);
    CachedImageBufferError read(size_t address, uint8_t *data, size_t &size);

private:
    CachedBufferState buffer_state_;
    Accessor &access_;
    size_t current_offset_;
    size_t current_size_;
    ChecksumCalculator checksum_calculator_;
};

template <typename Accessor>
CachedImageBufferError CachedImageBuffer<Accessor>::write(size_t address, const uint8_t *data, size_t size)
{
    if (address + size <= buffer_state_.TOTAL_BUFFER_CAPACITY_)
    {
        CachedImageBufferError status = static_cast<CachedImageBufferError>(access_.write(address + buffer_state_.FLASH_START_ADDRESS_, data, size));
        return status;
    }

    size_t first_part = buffer_state_.TOTAL_BUFFER_CAPACITY_ - address;
    size_t second_part = size - first_part;
    CachedImageBufferError status = static_cast<CachedImageBufferError>(access_.write(address + buffer_state_.FLASH_START_ADDRESS_, data, first_part));
    if (status != CachedImageBufferError::NO_ERROR)
    {
        return status;
    }
    status = static_cast<CachedImageBufferError>(access_.write(buffer_state_.FLASH_START_ADDRESS_, data + first_part, second_part));
    if (status != CachedImageBufferError::NO_ERROR)
    {
        return status;
    }
    return CachedImageBufferError::NO_ERROR;
}

template <typename Accessor>
CachedImageBufferError CachedImageBuffer<Accessor>::read(size_t address, uint8_t *data, size_t &size)
{
    size = std::min(size, current_size_ - current_offset_);
    if (address + size <= buffer_state_.TOTAL_BUFFER_CAPACITY_)
    {
        CachedImageBufferError status = static_cast<CachedImageBufferError>(access_.read(address + buffer_state_.FLASH_START_ADDRESS_, data, size));
        return status;
    }

    size_t first_part = buffer_state_.TOTAL_BUFFER_CAPACITY_ - address;
    size_t second_part = size - first_part;
    CachedImageBufferError status = static_cast<CachedImageBufferError>(access_.read(address + buffer_state_.FLASH_START_ADDRESS_, data, first_part));
    if (status != CachedImageBufferError::NO_ERROR)
    {
        return status;
    }
    status = static_cast<CachedImageBufferError>(access_.read(buffer_state_.FLASH_START_ADDRESS_, data + first_part, second_part));
    if (status != CachedImageBufferError::NO_ERROR)
    {
        return status;
    }
    return CachedImageBufferError::NO_ERROR;
}


template <typename Accessor>
void CachedImageBuffer<Accessor>::wrap_around(size_t &address)
{
    if (address >= buffer_state_.TOTAL_BUFFER_CAPACITY_)
    {
        address -= buffer_state_.TOTAL_BUFFER_CAPACITY_;
    }
}

template <typename Accessor>
CachedImageBufferError CachedImageBuffer<Accessor>::add_image(const ImageMetadata &metadata)
{
     
    size_t total_size = METADATA_SIZE + sizeof(crc_t) + metadata.image_size; 
    if (!has_enough_space(total_size))
    {
        // std::cerr << "Write Error: Not enough space in buffer." << std::endl;
        return CachedImageBufferError::FULL_BUFFER;
    }

    current_offset_ = buffer_state_.tail_;

    checksum_calculator_.reset();
    checksum_calculator_.update(reinterpret_cast<const uint8_t *>(&metadata), METADATA_SIZE_WO_CHECKSUM);
    crc_t checksum = checksum_calculator_.get_checksum();

    metadata.checksum = checksum;

    if (write(buffer_state_.tail_, reinterpret_cast<const uint8_t *>(&metadata), METADATA_SIZE) != CachedImageBufferError::NO_ERROR)
    {
        return CachedImageBufferError::WRITE_ERROR;
    }

    checksum_calculator_.reset();
    current_offset_ += METADATA_SIZE;
    wrap_around(current_offset_);
    return CachedImageBufferError::NO_ERROR;
}

template <typename Accessor>
CachedImageBufferError CachedImageBuffer<Accessor>::add_data_chunk(const uint8_t *data, size_t size)
{
    checksum_calculator_.update(data, size);

    if (write(current_offset_, data, size) != CachedImageBufferError::NO_ERROR)
    {
        return CachedImageBufferError::WRITE_ERROR;
    }

    current_offset_ += size;
    wrap_around(current_offset_);
    return CachedImageBufferError::NO_ERROR;
}

template <typename Accessor>
CachedImageBufferError CachedImageBuffer<Accessor>::push_image()
{
    crc_t checksum = checksum_calculator_.get_checksum(); 
    if (write(current_offset_, reinterpret_cast<const uint8_t *>(&checksum), sizeof(crc_t)) != CachedImageBufferError::NO_ERROR)
    {
        return CachedImageBufferError::WRITE_ERROR;
    }

    current_offset_ += sizeof(crc_t);
    wrap_around(current_offset_);

    size_t image_size = current_offset_ - buffer_state_.tail_;
    if (current_offset_ < buffer_state_.tail_) {
        image_size = buffer_state_.TOTAL_BUFFER_CAPACITY_ - buffer_state_.tail_ + current_offset_;
    }

    buffer_state_.size_ += image_size;
    buffer_state_.tail_ = current_offset_;
    wrap_around(buffer_state_.tail_);
    buffer_state_.count_++;

    return CachedImageBufferError::NO_ERROR;
}

template <typename Accessor>
CachedImageBufferError CachedImageBuffer<Accessor>::get_image(ImageMetadata &metadata)
{
    if (is_empty())
    {
        return CachedImageBufferError::EMPTY_BUFFER;
    }

    current_offset_ = buffer_state_.head_;

    current_size_ = METADATA_SIZE;
    size_t size = METADATA_SIZE;
    if (read(current_offset_, reinterpret_cast<uint8_t *>(&metadata), size) != CachedImageBufferError::NO_ERROR)
    {
        return CachedImageBufferError::READ_ERROR;
    }
    if (size != METADATA_SIZE)
    {
        return CachedImageBufferError::READ_ERROR;
    }

    checksum_calculator_.reset();
    checksum_calculator_.update(reinterpret_cast<const uint8_t *>(&metadata), METADATA_SIZE_WO_CHECKSUM);
    crc_t checksum = checksum_calculator_.get_checksum();
    if (metadata.checksum != checksum)
    {
        return CachedImageBufferError::CHECKSUM_ERROR;
    }

    checksum_calculator_.reset();
    current_offset_ += METADATA_SIZE;
    current_size_ += metadata.image_size;
    wrap_around(current_offset_);
    return CachedImageBufferError::NO_ERROR;
}

template <typename Accessor>
CachedImageBufferError CachedImageBuffer<Accessor>::get_data_chunk(uint8_t *data, size_t &size)
{
    if (read(current_offset_, data, size) != CachedImageBufferError::NO_ERROR)
    {
        return CachedImageBufferError::READ_ERROR;
    }
    checksum_calculator_.update(data, size);
    current_offset_ += size;
    wrap_around(current_offset_);
    return CachedImageBufferError::NO_ERROR;
}

template <typename Accessor>
CachedImageBufferError CachedImageBuffer<Accessor>::pop_image()
{
    crc_t checksum = 0;
    current_size_ += sizeof(crc_t);
    size_t size = sizeof(crc_t);
    if (read(current_offset_, reinterpret_cast<uint8_t *>(&checksum), size) != CachedImageBufferError::NO_ERROR)
    {
        return CachedImageBufferError::READ_ERROR;
    }
    if (size != sizeof(crc_t))
    {
        return CachedImageBufferError::READ_ERROR;
    }

    crc_t data_checksum = checksum_calculator_.get_checksum();
    if (checksum != data_checksum)
    {
        return CachedImageBufferError::CHECKSUM_ERROR;
    }

    current_offset_ += sizeof(crc_t);
    wrap_around(current_offset_);

    size_t image_size = current_offset_ - buffer_state_.head_;
    if (current_offset_ < buffer_state_.head_) {
        image_size = buffer_state_.TOTAL_BUFFER_CAPACITY_ - buffer_state_.head_ + current_offset_;
    }

    buffer_state_.size_ -= image_size;
    buffer_state_.head_ = current_offset_;
    wrap_around(buffer_state_.head_);
    buffer_state_.count_--;

    return CachedImageBufferError::NO_ERROR;
}

#endif // CACHED_IMAGE_BUFFER_H