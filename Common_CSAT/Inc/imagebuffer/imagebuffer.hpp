#ifndef __image_buffer__hpp__
#define __image_buffer__hpp__

#include <cstdint>
#include <cstddef>

enum class ImageBufferError : uint16_t
{
    NO_ERROR = 0,
    WRITE_ERROR = 1,
    READ_ERROR = 2,
    OUT_OF_BOUNDS = 3,
    CHECKSUM_ERROR = 4,
    EMPTY_BUFFER = 5,
    FULL_BUFFER = 6,
};

struct BufferState
{
    size_t head_;
    size_t tail_;
    size_t size_;
    size_t count_;
    const size_t FLASH_START_ADDRESS_;
    const size_t TOTAL_BUFFER_CAPACITY_;
    uint32_t checksum;

    bool is_empty() const;
    size_t size() const;
    size_t count() const;
    size_t available() const;
    size_t capacity() const;

    size_t get_head() const;
    size_t get_tail() const;

    BufferState(size_t head, size_t tail, size_t size, size_t flash_start, size_t total_capacity) : head_(head), tail_(tail), size_(size), count_(0), FLASH_START_ADDRESS_(flash_start), TOTAL_BUFFER_CAPACITY_(total_capacity) {}
};
bool BufferState::is_empty() const { return size_ == 0; }
size_t BufferState::size() const { return size_; };
size_t BufferState::count() const { return count_; };
size_t BufferState::available() const { return TOTAL_BUFFER_CAPACITY_ - size_; };
size_t BufferState::capacity() const { return TOTAL_BUFFER_CAPACITY_; };
size_t BufferState::get_head() const { return head_; }
size_t BufferState::get_tail() const { return tail_; }

#endif // __image_buffer__hpp__
