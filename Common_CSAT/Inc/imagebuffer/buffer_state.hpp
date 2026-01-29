#ifndef BUFFER_STATE_HPP
#define BUFFER_STATE_HPP

#include <cstddef>
#include <cstdint>

// -----------------------------------------------------------------------------
// BufferState
//   - Holds only ring geometry and flash geometry
//   - No logic beyond simple invariants
//   - Wrap‑aware available_from()
// -----------------------------------------------------------------------------
struct BufferState
{
    size_t head_;   // logical offset of oldest entry
    size_t tail_;   // logical offset of next write
    size_t size_;   // total bytes used
    size_t count_;  // number of entries
    const size_t FLASH_START_ADDRESS_;
    const size_t TOTAL_BUFFER_CAPACITY_;

    BufferState(size_t head,
                size_t tail,
                size_t size,
                size_t flash_start,
                size_t total_capacity)
        : head_(head),
          tail_(tail),
          size_(size),
          count_(0),
          FLASH_START_ADDRESS_(flash_start),
          TOTAL_BUFFER_CAPACITY_(total_capacity)
    {}

    // -------------------------------------------------------------------------
    // Basic queries
    // -------------------------------------------------------------------------
    bool   is_empty() const { return size_ == 0; }
    bool   has_room_for(size_t size) const { return available() >= size; }
    size_t size()     const { return size_; }
    size_t count()    const { return count_; }
    size_t capacity() const { return TOTAL_BUFFER_CAPACITY_; }
    size_t available()const { return TOTAL_BUFFER_CAPACITY_ - size_; }

    size_t get_head() const { return head_; }
    size_t get_tail() const { return tail_; }

    // -------------------------------------------------------------------------
    // available_from(start):
    //   Returns how many free bytes exist starting at logical offset `start`
    //   before hitting used data. Fully wrap‑aware.
    //
    //   Cases:
    //     - empty buffer: everything is free
    //     - non‑empty:
    //         used region is [head_, head_ + size_) modulo capacity
    //         free region is the complement
    //
    //   This is the correct geometric test for a ring buffer.
    // -------------------------------------------------------------------------
    size_t available_from(size_t start) const
    {
        const size_t cap = TOTAL_BUFFER_CAPACITY_;

        // Empty → everything free
        if (size_ == 0)
            return cap;

        // Normalize
        start %= cap;

        // Compute end of used region
        const size_t used_start = head_;
        const size_t used_end   = (head_ + size_) % cap;

        // If used region does NOT wrap:
        //   used = [used_start, used_end)
        //   free = [used_end, used_start)
        if (used_start < used_end)
        {
            if (start < used_start)
                return used_start - start;          // free before used_start
            if (start >= used_end)
                return cap - start + used_start;    // wrap free region
            return 0;                                // inside used region
        }

        // If used region wraps:
        //   used = [used_start, cap) ∪ [0, used_end)
        //   free = [used_end, used_start)
        if (start >= used_end && start < used_start)
            return used_start - start;              // free region
        return 0;                                   // inside used region
    }
};

#endif // BUFFER_STATE_HPP
