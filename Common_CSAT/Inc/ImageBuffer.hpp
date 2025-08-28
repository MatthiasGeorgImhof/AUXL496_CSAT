#ifndef IMAGE_BUFFER_HPP
#define IMAGE_BUFFER_HPP

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

template <typename Accessor>
class ImageBuffer
{
public:
    ImageBuffer(Accessor &accessor)
        : buffer_state_(0, 0, 0, accessor.getFlashStartAddress(), accessor.getFlashMemorySize()), accessor_(accessor) {}

    ImageBufferError add_image(const ImageMetadata &metadata);
    ImageBufferError add_data_chunk(const uint8_t *data, size_t size);
    ImageBufferError push_image();

    ImageBufferError get_image(ImageMetadata &metadata);
    ImageBufferError get_data_chunk(uint8_t *data, size_t &size);
    ImageBufferError pop_image();


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
    size_t offset_to_alignment(size_t size) const { return (size % accessor_.getAlignment() == 0) ? 0 : accessor_.getAlignment() - (size % accessor_.getAlignment()); }
    ImageBufferError write(size_t address, const uint8_t *data, size_t size);
    ImageBufferError read(size_t address, uint8_t *data, size_t &size);

private:
    BufferState buffer_state_;
    Accessor &accessor_;
    size_t current_offset_;
    size_t current_size_;
    ChecksumCalculator checksum_calculator_;
};

template <typename Accessor>
ImageBufferError ImageBuffer<Accessor>::write(size_t address, const uint8_t *data, size_t size)
{
    if (address + size > buffer_state_.TOTAL_BUFFER_CAPACITY_)
    {
        // std::cerr << "Write Error: Out of bounds." << std::endl;
        return ImageBufferError::OUT_OF_BOUNDS;
    }
    
    if (address + size <= buffer_state_.TOTAL_BUFFER_CAPACITY_)
    {
        ImageBufferError status = static_cast<ImageBufferError>(accessor_.write(static_cast<size_t>(address + buffer_state_.FLASH_START_ADDRESS_), data, size));
        // std::cerr << "Write failure" << std::endl;
        return status;
    }

    size_t first_part = buffer_state_.TOTAL_BUFFER_CAPACITY_ - address;
    size_t second_part = size - first_part;
    ImageBufferError status = static_cast<ImageBufferError>(accessor_.write(static_cast<size_t>(address + buffer_state_.FLASH_START_ADDRESS_), data, first_part));
    if (status != ImageBufferError::NO_ERROR)
    {
        // std::cerr << "Write failure" << std::endl;
        return status;
    }
    status = static_cast<ImageBufferError>(accessor_.write(static_cast<size_t>(buffer_state_.FLASH_START_ADDRESS_), data + first_part, second_part));
    if (status != ImageBufferError::NO_ERROR)
    {
        // std::cerr << "Write failure" << std::endl;
        return status;
    }
    return ImageBufferError::NO_ERROR;
}

template <typename Accessor>
ImageBufferError ImageBuffer<Accessor>::read(size_t address, uint8_t *data, size_t &size)
{
    size = std::min(size, current_size_ - current_offset_);
    if (address + size <= buffer_state_.TOTAL_BUFFER_CAPACITY_)
    {
        ImageBufferError status = static_cast<ImageBufferError>(accessor_.read(static_cast<size_t>(address + buffer_state_.FLASH_START_ADDRESS_), data, size));
        // std::cerr << "Read failure" << std::endl;
        return status;
    }

    size_t first_part = buffer_state_.TOTAL_BUFFER_CAPACITY_ - address;
    size_t second_part = size - first_part;
    ImageBufferError status = static_cast<ImageBufferError>(accessor_.read(static_cast<size_t>(address + buffer_state_.FLASH_START_ADDRESS_), data, first_part));
    if (status != ImageBufferError::NO_ERROR)
    {
        // std::cerr << "Read failure" << std::endl;
        return status;
    }
    status = static_cast<ImageBufferError>(accessor_.read(static_cast<size_t>(buffer_state_.FLASH_START_ADDRESS_), data + first_part, second_part));
    if (status != ImageBufferError::NO_ERROR)
    {
        // std::cerr << "Read failure" << std::endl;
        return status;
    }
    return ImageBufferError::NO_ERROR;
}

template <typename Accessor>
void ImageBuffer<Accessor>::wrap_around(size_t &address)
{
    if (address >= buffer_state_.TOTAL_BUFFER_CAPACITY_)
    {
        address -= buffer_state_.TOTAL_BUFFER_CAPACITY_;
    }
}

template <typename Accessor>
ImageBufferError ImageBuffer<Accessor>::add_image(const ImageMetadata &metadata)
{
     
    size_t total_size = METADATA_SIZE + sizeof(crc_t) + metadata.image_size; 
    if (!has_enough_space(total_size))
    {
        // std::cerr << "Write Error: Not enough space in buffer." << std::endl;
        return ImageBufferError::FULL_BUFFER;
    }

    current_offset_ = buffer_state_.tail_;
    checksum_calculator_.reset();
    checksum_calculator_.update(reinterpret_cast<const uint8_t *>(&metadata), METADATA_SIZE_WO_CHECKSUM);
    crc_t checksum = checksum_calculator_.get_checksum();

    // print(reinterpret_cast<const uint8_t *>(&metadata), METADATA_SIZE_WO_CHECKSUM);
    metadata.checksum = checksum;
    // print(reinterpret_cast<const uint8_t *>(&metadata), METADATA_SIZE);

    if (write(buffer_state_.tail_, reinterpret_cast<const uint8_t *>(&metadata), METADATA_SIZE) != ImageBufferError::NO_ERROR)
    {
        // std::cerr << "Write add_image failure" << std::endl;
        return ImageBufferError::WRITE_ERROR;
    }

    checksum_calculator_.reset();
    current_offset_ += METADATA_SIZE;
    wrap_around(current_offset_);
    return ImageBufferError::NO_ERROR;
}

template <typename Accessor>
ImageBufferError ImageBuffer<Accessor>::add_data_chunk(const uint8_t *data, size_t size)
{
    checksum_calculator_.update(data, size);

    if (write(current_offset_, data, size) != ImageBufferError::NO_ERROR)
    {
        // std::cerr << "Write add_data_chunk failure" << std::endl;
        return ImageBufferError::WRITE_ERROR;
    }

    current_offset_ += size;
    wrap_around(current_offset_);
    return ImageBufferError::NO_ERROR;
}

template <typename Accessor>
ImageBufferError ImageBuffer<Accessor>::push_image()
{
    crc_t checksum = checksum_calculator_.get_checksum(); 
    if (write(current_offset_, reinterpret_cast<const uint8_t *>(&checksum), sizeof(crc_t)) != ImageBufferError::NO_ERROR)
    {
        // std::cerr << "Write push_image failure" << std::endl;
        return ImageBufferError::WRITE_ERROR;
    }

    current_offset_ += sizeof(crc_t);
    current_offset_ += offset_to_alignment(current_offset_);
    buffer_state_.size_ += current_offset_ - buffer_state_.tail_;
    buffer_state_.tail_ = current_offset_;
    wrap_around(buffer_state_.tail_);
    buffer_state_.count_++;

    return ImageBufferError::NO_ERROR;
}

template <typename Accessor>
ImageBufferError ImageBuffer<Accessor>::get_image(ImageMetadata &metadata)
{
    if (is_empty())
    {
        return ImageBufferError::EMPTY_BUFFER;
    }

    current_offset_ = buffer_state_.head_;
    current_size_ = METADATA_SIZE;
    size_t size = METADATA_SIZE;
    if (read(current_offset_, reinterpret_cast<uint8_t *>(&metadata), size) != ImageBufferError::NO_ERROR)
    {
        // std::cerr << "Read get_image failure" << std::endl;
        return ImageBufferError::READ_ERROR;
    }
    if (size != METADATA_SIZE)
    {
        // std::cerr << "Read get_image size error" << std::endl;
        return ImageBufferError::READ_ERROR;
    }

    checksum_calculator_.reset();
    checksum_calculator_.update(reinterpret_cast<const uint8_t *>(&metadata), METADATA_SIZE_WO_CHECKSUM);
    crc_t checksum = checksum_calculator_.get_checksum();
    if (metadata.checksum != checksum)
    {
        // std::cerr << "Read get_image checksum error" << std::endl;
        return ImageBufferError::CHECKSUM_ERROR;
    }

    checksum_calculator_.reset();
    current_offset_ += METADATA_SIZE;
    current_size_ += metadata.image_size;
    wrap_around(current_offset_);
    return ImageBufferError::NO_ERROR;
}

template <typename Accessor>
ImageBufferError ImageBuffer<Accessor>::get_data_chunk(uint8_t *data, size_t &size)
{
    if (read(current_offset_, data, size) != ImageBufferError::NO_ERROR)
    {
        // std::cerr << "Read get_data_chunk failure" << std::endl;
        return ImageBufferError::READ_ERROR;
    }
    checksum_calculator_.update(data, size);
    current_offset_ += size;
    wrap_around(current_offset_);
    return ImageBufferError::NO_ERROR;
}

template <typename Accessor>
ImageBufferError ImageBuffer<Accessor>::pop_image()
{
    crc_t checksum = 0;
    current_size_ += sizeof(crc_t);
    size_t size = sizeof(crc_t);
    if (read(current_offset_, reinterpret_cast<uint8_t *>(&checksum), size) != ImageBufferError::NO_ERROR)
    {
        // std::cerr << "Read pop_image failure" << std::endl;
        return ImageBufferError::READ_ERROR;
    }
    if (size != sizeof(crc_t))
    {
        // std::cerr << "Read pop_image size error" << std::endl;
        return ImageBufferError::READ_ERROR;
    }

    crc_t data_checksum = checksum_calculator_.get_checksum();
    if (checksum != data_checksum)
    {
        // std::cerr << "pop_imageChecksum data mismatch" << std::endl;
        return ImageBufferError::CHECKSUM_ERROR;
    }

    current_offset_ += sizeof(crc_t);
    current_offset_ += offset_to_alignment(current_offset_);
    buffer_state_.size_ -= current_offset_ - buffer_state_.head_;
    buffer_state_.head_ = current_offset_;
    wrap_around(buffer_state_.head_);
    buffer_state_.count_--;

    return ImageBufferError::NO_ERROR;
}

#endif // IMAGE_BUFFER_HPP