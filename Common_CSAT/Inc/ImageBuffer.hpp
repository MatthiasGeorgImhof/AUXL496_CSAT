#ifndef IMAGE_BUFFER_HPP
#define IMAGE_BUFFER_HPP

#include <cstdint>
#include <cstddef>
#include <algorithm>
#include <cstring>
#include <vector>

#include "ImageBufferConcept.hpp"
#include "imagebuffer/accessor.hpp"
#include "imagebuffer/image.hpp"
#include "imagebuffer/storageheader.hpp"
#include "imagebuffer/buffer_state.hpp"
#include "Checksum.hpp"

template <typename Accessor, typename ChecksumPolicy = DefaultChecksumPolicy>
class ImageBuffer
{
public:
    ImageBuffer(Accessor &accessor)
        : buffer_state_(0,
                        0,
                        0,
                        accessor.getFlashStartAddress(),
                        accessor.getFlashMemorySize()),
          accessor_(accessor),
          next_sequence_id_(0),
          write_state_{},
          read_state_{} {}

    // ---------------------------------------------------------------------
    // Public state queries (unchanged API)
    // ---------------------------------------------------------------------
    inline bool   is_empty() const { return buffer_state_.is_empty(); }
    inline bool   has_room_for(size_t size) const { return buffer_state_.has_room_for(size); }
    inline size_t size()     const { return buffer_state_.size(); }
    inline size_t count()    const { return buffer_state_.count(); }
    inline size_t available()const { return buffer_state_.available(); }
    inline size_t capacity() const { return buffer_state_.capacity(); }
    inline size_t get_head() const { return buffer_state_.get_head(); }
    inline size_t get_tail() const { return buffer_state_.get_tail(); }

    // ---------------------------------------------------------------------
    // Public API (unchanged signatures)
    // ---------------------------------------------------------------------
    ImageBufferError add_image(const ImageMetadata &meta);
    ImageBufferError add_data_chunk(const uint8_t *data, size_t size)
    {
        return ring_io(write_state_,
                       const_cast<uint8_t *>(data),
                       size,
                       true,
                       true);
    }
    ImageBufferError push_image();
    ImageBufferError get_image(ImageMetadata &meta);
    ImageBufferError get_data_chunk(uint8_t *data, size_t &size);
    ImageBufferError pop_image();
    ImageBufferError initialize_from_flash();

protected:
    void test_set_tail(size_t t) { buffer_state_.tail_ = t; }
    ImageBufferError validate_entry(size_t offset,
                                    size_t &entry_size,
                                    uint32_t &seq_id,
                                    ImageMetadata &meta_out);

private:
    // ---------------------------------------------------------------------
    // Per-entry streaming state
    // ---------------------------------------------------------------------
    struct EntryState
    {
        size_t offset;       // current ring offset
        size_t entry_size;   // logical entry size (bytes)
        size_t consumed;     // bytes consumed from this entry
        size_t payload_size; // payload bytes (for read path)
    };

    // ---------------------------------------------------------------------
    // Layout helpers (constexpr to avoid repeated size math)
    // ---------------------------------------------------------------------
    static constexpr size_t header_size()   { return sizeof(StorageHeader); }
    static constexpr size_t metadata_size() { return sizeof(ImageMetadata); }
    static constexpr size_t crc_size()      { return sizeof(crc_t); }

    static constexpr size_t overhead_size()
    {
        return header_size() + metadata_size();
    }

    static constexpr size_t entry_total_size(size_t payload)
    {
        return overhead_size() + payload + crc_size();
    }

    // ---------------------------------------------------------------------
    // Alignment helpers (accessor is the single source of truth)
    // ---------------------------------------------------------------------
    size_t entry_alignment() const
    {
        size_t a = accessor_.getAlignment();
        return a == 0 ? 1 : a;
    }

    size_t align_up(size_t v) const
    {
        size_t a = entry_alignment();
        return (v + a - 1) / a * a;
    }

    size_t align_up_wrapped(size_t v) const
    {
        size_t aligned = align_up(v);
        const size_t cap = buffer_state_.TOTAL_BUFFER_CAPACITY_;
        return (aligned >= cap) ? (aligned - cap) : aligned;
    }

    // ---------------------------------------------------------------------
    // Core ring I/O
    //   - s.offset is always a logical ring offset (0..cap-1)
    //   - s.consumed is incremented by `size`
    //   - if update_crc == true, checksum_ must have been reset beforehand
    // ---------------------------------------------------------------------
ImageBufferError ring_io(EntryState &s,
                         uint8_t *data,
                         size_t size,
                         bool write,
                         bool update_crc)
{
    if (size == 0)
        return ImageBufferError::NO_ERROR;

    const size_t cap = buffer_state_.TOTAL_BUFFER_CAPACITY_;
    if (size > cap)
        return ImageBufferError::OUT_OF_BOUNDS;

    using Address = decltype(accessor_.getFlashStartAddress());

    size_t remaining = size;
    while (remaining > 0)
    {
        const size_t chunk = std::min(remaining, cap - s.offset);
        const Address phys_addr =
            static_cast<Address>(accessor_.getFlashStartAddress() + s.offset);

        const auto status = write
            ? accessor_.write(phys_addr, data, chunk)
            : accessor_.read(phys_addr, data, chunk);

            if (status != AccessorError::NO_ERROR)
                return write
                    ? ImageBufferError::WRITE_ERROR
                    : ImageBufferError::READ_ERROR;

            if (update_crc)
                checksum_.update(data, chunk);

            s.offset = (s.offset + chunk) % cap;
            data     += chunk;
            remaining -= chunk;
        }

        s.consumed += size;
        return ImageBufferError::NO_ERROR;
    }

    // ---------------------------------------------------------------------
    // process_struct:
    //   - handles CRC for fixed-size structs (header, metadata)
    //   - uses a local checksum instance to avoid coupling with payload CRC
    // ---------------------------------------------------------------------
    template <typename T>
    ImageBufferError process_struct(EntryState &s,
                                    T &obj,
                                    size_t crc_offset,
                                    bool write)
    {
        ChecksumPolicy cs;

        if (write)
        {
            cs.reset();
            cs.update(reinterpret_cast<uint8_t *>(&obj), crc_offset);
            *reinterpret_cast<crc_t *>(
                reinterpret_cast<uint8_t *>(&obj) + crc_offset) = cs.get();
        }

        auto err = ring_io(s,
                           reinterpret_cast<uint8_t *>(&obj),
                           sizeof(T),
                           write,
                           false);
        if (err != ImageBufferError::NO_ERROR)
            return err;

        if (!write)
        {
            cs.reset();
            cs.update(reinterpret_cast<uint8_t *>(&obj), crc_offset);

            const crc_t stored =
                *reinterpret_cast<crc_t *>(
                    reinterpret_cast<uint8_t *>(&obj) + crc_offset);

            if (cs.get() != stored)
                return ImageBufferError::CHECKSUM_ERROR;
        }

        return ImageBufferError::NO_ERROR;
    }

    // ---------------------------------------------------------------------
    // Erase all erase-blocks touched by an entry
    // ---------------------------------------------------------------------
    ImageBufferError erase_entry_blocks(size_t offset, size_t size)
    {
        const size_t block_sz = accessor_.getEraseBlockSize();
        const size_t cap      = buffer_state_.TOTAL_BUFFER_CAPACITY_;

        for (size_t i = 0; i < size; i += block_sz)
        {
            const size_t block_addr = ((offset + i) % cap / block_sz) * block_sz;
            const size_t phys = buffer_state_.FLASH_START_ADDRESS_ + block_addr;

            if (accessor_.erase(phys) != AccessorError::NO_ERROR)
                return ImageBufferError::WRITE_ERROR;
        }

        return ImageBufferError::NO_ERROR;
    }

    // ---------------------------------------------------------------------
    // Adjust head after popping an entry
    //   - decreases size_
    //   - advances head_
    //   - aligns head_ to next valid boundary
    //   - accounts for padding as "consumed" size
    // ---------------------------------------------------------------------
    void adjust_head(size_t size)
    {
        const size_t cap = buffer_state_.TOTAL_BUFFER_CAPACITY_;

        buffer_state_.size_ -= size;
        buffer_state_.head_  = (buffer_state_.head_ + size) % cap;

        const size_t aligned = align_up_wrapped(buffer_state_.head_);

        // padding between old head and aligned head
        size_t pad;
        if (aligned >= buffer_state_.head_)
            pad = aligned - buffer_state_.head_;
        else
            pad = cap - (buffer_state_.head_ - aligned);

        buffer_state_.size_ =
            (pad <= buffer_state_.size_) ? (buffer_state_.size_ - pad) : 0;

        buffer_state_.head_ = aligned;
        buffer_state_.count_--;
    }

    // ---------------------------------------------------------------------
    // Members
    // ---------------------------------------------------------------------
    BufferState buffer_state_;
    Accessor &accessor_;
    ChecksumPolicy checksum_; // used for payload CRC and validate_entry payload
    uint32_t next_sequence_id_;

    EntryState write_state_;
    EntryState read_state_;
};

// ==========================================================================
// add_image
// ==========================================================================
template <typename A, typename C>
ImageBufferError ImageBuffer<A, C>::add_image(const ImageMetadata &meta)
{
    const size_t total = entry_total_size(meta.payload_size);

    if (buffer_state_.available() < total)
    {
        return ImageBufferError::FULL_BUFFER;
    }

    const size_t tail         = buffer_state_.tail_;
    const size_t aligned_tail = align_up_wrapped(tail);

    buffer_state_.tail_ = aligned_tail;
    write_state_ = { aligned_tail, 0, 0, meta.payload_size };

    // StorageHeader
    StorageHeader hdr{};
    hdr.magic       = STORAGE_MAGIC;
    hdr.version     = STORAGE_HEADER_VERSION;
    hdr.header_size = static_cast<uint16_t>(sizeof(StorageHeader));
    hdr.sequence_id = next_sequence_id_++;
    hdr.total_size  = static_cast<uint32_t>(total - header_size());

    auto err = process_struct(write_state_,
                              hdr,
                              offsetof(StorageHeader, header_crc),
                              true);
    if (err != ImageBufferError::NO_ERROR)
        return err;

    // ImageMetadata
    ImageMetadata m_out = meta;
    m_out.version       = 1;
    m_out.metadata_size = static_cast<uint16_t>(metadata_size());

    err = process_struct(write_state_,
                         m_out,
                         METADATA_SIZE_WO_CRC,
                         true);

    checksum_.reset(); // prepare for payload CRC
    return err;
}

// ==========================================================================
// push_image
// ==========================================================================
template <typename A, typename C>
ImageBufferError ImageBuffer<A, C>::push_image()
{
    crc_t tag = checksum_.get();

    auto err = ring_io(write_state_,
                       reinterpret_cast<uint8_t *>(&tag),
                       crc_size(),
                       true,
                       false);
    if (err != ImageBufferError::NO_ERROR)
        return err;

    buffer_state_.size_ += write_state_.consumed;
    buffer_state_.tail_  = write_state_.offset;
    buffer_state_.count_++;

    return ImageBufferError::NO_ERROR;
}

// ==========================================================================
// get_image
// ==========================================================================
template <typename A, typename C>
ImageBufferError ImageBuffer<A, C>::get_image(ImageMetadata &meta)
{
    if (is_empty())
        return ImageBufferError::EMPTY_BUFFER;

    read_state_ = { buffer_state_.head_, 0, 0, 0 };

    StorageHeader hdr{};
    auto err = process_struct(read_state_,
                              hdr,
                              offsetof(StorageHeader, header_crc),
                              false);
    if (err != ImageBufferError::NO_ERROR ||
        hdr.magic != STORAGE_MAGIC)
        return ImageBufferError::CHECKSUM_ERROR;

    read_state_.entry_size = header_size() + hdr.total_size;

    err = process_struct(read_state_,
                         meta,
                         METADATA_SIZE_WO_CRC,
                         false);
    if (err != ImageBufferError::NO_ERROR)
        return err;

    read_state_.payload_size = meta.payload_size;
    checksum_.reset(); // prepare for payload CRC during get_data_chunk
    return ImageBufferError::NO_ERROR;
}

// ==========================================================================
// get_data_chunk
// ==========================================================================
template <typename A, typename C>
ImageBufferError ImageBuffer<A, C>::get_data_chunk(uint8_t *data, size_t &size)
{
    const size_t overhead = overhead_size();

    const size_t payload_done =
        (read_state_.consumed > overhead)
            ? (read_state_.consumed - overhead)
            : 0;

    const size_t remaining =
        (read_state_.payload_size > payload_done)
            ? (read_state_.payload_size - payload_done)
            : 0;

    size = std::min(size, remaining);

    return ring_io(read_state_,
                   data,
                   size,
                   false,
                   true); // update payload CRC
}

// ==========================================================================
// pop_image
// ==========================================================================
template <typename A, typename C>
ImageBufferError ImageBuffer<A, C>::pop_image()
{
    if (is_empty())
        return ImageBufferError::EMPTY_BUFFER;

    crc_t stored = 0;
    const crc_t actual = checksum_.get();
    const size_t old_head = buffer_state_.head_;
    const size_t total_sz = read_state_.entry_size;

    auto err = ring_io(read_state_,
                       reinterpret_cast<uint8_t *>(&stored),
                       crc_size(),
                       false,
                       false);
    if (err != ImageBufferError::NO_ERROR)
        return ImageBufferError::READ_ERROR;

    if (stored != actual)
        return ImageBufferError::CHECKSUM_ERROR;

    adjust_head(total_sz);
    return erase_entry_blocks(old_head, total_sz);
}

// ==========================================================================
// initialize_from_flash
// ==========================================================================
template <typename A, typename C>
ImageBufferError ImageBuffer<A, C>::initialize_from_flash()
{
    const size_t cap = buffer_state_.TOTAL_BUFFER_CAPACITY_;

    buffer_state_.head_ = buffer_state_.tail_ =
        buffer_state_.size_ = buffer_state_.count_ =
            next_sequence_id_ = 0;

    if (cap == 0)
        return ImageBufferError::NO_ERROR;

    struct Found
    {
        size_t   off;
        size_t   sz;
        uint32_t id;
    };

    std::vector<Found> entries;
    size_t scan = 0;
    const size_t step = entry_alignment();

    // 1) Scan for candidate headers
    while (scan < cap)
    {
        EntryState temp{ scan, 0, 0, 0 };
        StorageHeader hdr{};

        auto hdr_err = process_struct(temp,
                                      hdr,
                                      offsetof(StorageHeader, header_crc),
                                      false);

        if (hdr_err == ImageBufferError::NO_ERROR &&
            hdr.magic == STORAGE_MAGIC)
        {
            const size_t e_size = header_size() + hdr.total_size;
            entries.push_back({ scan, e_size, hdr.sequence_id });

            scan = align_up_wrapped(scan + e_size);
        }
        else
        {
            scan = (scan + step) % cap;
        }

        if (scan == 0)
            break;
    }

    if (entries.empty())
        return ImageBufferError::NO_ERROR;

    // 2) Validation pass
    std::sort(entries.begin(),
              entries.end(),
              [](const Found &a, const Found &b)
              {
                  return a.id < b.id;
              });

    std::vector<Found> good;
    good.reserve(entries.size());

    ImageBufferError first_err = ImageBufferError::NO_ERROR;

    for (const auto &e : entries)
    {
        size_t      validated_size = 0;
        uint32_t    sid            = 0;
        ImageMetadata dummy{};

        auto v_err = validate_entry(e.off,
                                    validated_size,
                                    sid,
                                    dummy);
        if (v_err != ImageBufferError::NO_ERROR)
        {
            first_err = v_err;
            break;
        }

        if (validated_size != e.sz)
        {
            first_err = ImageBufferError::DATA_ERROR;
            break;
        }

        if (!good.empty() &&
            e.id != good.back().id + 1)
        {
            first_err = ImageBufferError::CHECKSUM_ERROR;
            break;
        }

        good.push_back(e);
    }

    if (good.empty())
        return first_err;

    // 3) Commit reconstructed state
    buffer_state_.head_ = good.front().off;
    buffer_state_.size_ = 0;

    for (const auto &e : good)
    {
        buffer_state_.size_ += e.sz;
        buffer_state_.tail_  = (e.off + e.sz) % cap;
    }

    buffer_state_.count_ = good.size();
    next_sequence_id_    = good.back().id + 1;

    return first_err;
}

// ==========================================================================
// validate_entry
// ==========================================================================
template <typename A, typename C>
ImageBufferError ImageBuffer<A, C>::validate_entry(size_t offset,
                                                   size_t &entry_size,
                                                   uint32_t &seq_id,
                                                   ImageMetadata &meta_out)
{
    const size_t cap = buffer_state_.TOTAL_BUFFER_CAPACITY_;
    if (cap == 0)
        return ImageBufferError::CHECKSUM_ERROR;

    EntryState s{ offset, 0, 0, 0 };

    // 1) Header
    StorageHeader hdr{};
    auto err = process_struct(s,
                              hdr,
                              offsetof(StorageHeader, header_crc),
                              false);
    if (err != ImageBufferError::NO_ERROR ||
        hdr.magic != STORAGE_MAGIC)
        return ImageBufferError::CHECKSUM_ERROR;

    entry_size = header_size() + hdr.total_size;
    seq_id     = hdr.sequence_id;

    if (entry_size > cap)
        return ImageBufferError::CHECKSUM_ERROR;

    // 2) Metadata
    err = process_struct(s,
                         meta_out,
                         METADATA_SIZE_WO_CRC,
                         false);
    if (err != ImageBufferError::NO_ERROR)
        return err;

    const size_t payload_size = meta_out.payload_size;
    s.payload_size            = payload_size;

    const size_t expected_total =
        metadata_size() + payload_size + crc_size();

    if (hdr.total_size != expected_total)
        return ImageBufferError::DATA_ERROR;

    // 3) Payload CRC
    checksum_.reset();

    size_t remaining = payload_size;
    uint8_t buf[64];

    while (remaining > 0)
    {
        const size_t chunk = std::min(remaining, sizeof(buf));
        auto io_err = ring_io(s,
                              buf,
                              chunk,
                              false,
                              true);
        if (io_err != ImageBufferError::NO_ERROR)
            return io_err;

        remaining -= chunk;
    }

    // 4) Trailing CRC
    crc_t stored = 0;
    err = ring_io(s,
                  reinterpret_cast<uint8_t *>(&stored),
                  crc_size(),
                  false,
                  false);
    if (err != ImageBufferError::NO_ERROR)
        return err;

    const crc_t actual = checksum_.get();
    if (actual != stored)
        return ImageBufferError::CHECKSUM_ERROR;

    return ImageBufferError::NO_ERROR;
}

#endif // IMAGE_BUFFER_HPP
