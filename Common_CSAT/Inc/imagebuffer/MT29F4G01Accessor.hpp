#ifndef MT29F4G01_ACCESSOR_H
#define MT29F4G01_ACCESSOR_H

#include <cstdint>
#include <cstddef>
#include <array>
#include <algorithm>
#include <cstring>

#include "ImageBuffer.hpp" 
#include "imagebuffer/accessor.hpp" // Accessor concept, AccessorError
#include "Transport.hpp"            // StreamAccessTransport

// Micron MT29F4G01ABAFD SPI NAND command set (subset)
enum class MT29_CMD : uint8_t
{
    // Core commands
    RESET            = 0xFF,
    GET_FEATURE      = 0x0F,
    SET_FEATURE      = 0x1F,
    WRITE_ENABLE     = 0x06,
    WRITE_DISABLE    = 0x04,
    PAGE_READ        = 0x13, // array -> data reg/cache
    READ_FROM_CACHE  = 0x03, // x1 (or 0x0B fast read)

    PROGRAM_LOAD     = 0x02, // cache load (x1)
    PROGRAM_EXECUTE  = 0x10, // cache -> array

    BLOCK_ERASE      = 0xD8,

    // Feature addresses
    FEATURE_ADDR_BLOCK_LOCK = 0xA0,
    FEATURE_ADDR_CONFIG     = 0xB0,
    FEATURE_ADDR_STATUS     = 0xC0,

    // Status bits (FEATURE_ADDR_STATUS)
    STATUS_CRBSY    = 0x80, // cache read busy
    STATUS_ECCS2    = 0x40,
    STATUS_ECCS1    = 0x20,
    STATUS_ECCS0    = 0x10,
    STATUS_P_FAIL   = 0x08,
    STATUS_E_FAIL   = 0x04,
    STATUS_WEL      = 0x02,
    STATUS_OIP      = 0x01 // operation in progress
};

// NOTE: TransportT must satisfy StreamAccessTransport
template <StreamAccessTransport TransportT>
class MT29F4G01Accessor
{
public:
    // ─────────────────────────────────────────────
    // Geometry (per Micron MT29F4G01ABAFD)
    // ─────────────────────────────────────────────
    static constexpr size_t PAGE_SIZE       = 4096;                         // data
    static constexpr size_t SPARE_SIZE      = 256;                          // OOB
    static constexpr size_t PAGE_TOTAL_SIZE = PAGE_SIZE + SPARE_SIZE;       // 4352
    static constexpr size_t PAGES_PER_BLOCK = 64;
    static constexpr size_t BLOCK_SIZE      = PAGE_TOTAL_SIZE * PAGES_PER_BLOCK; // 278,528
    static constexpr size_t TOTAL_BLOCKS    = 2048;
    static constexpr size_t TOTAL_SIZE      = BLOCK_SIZE * TOTAL_BLOCKS;

    // ─────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────
    MT29F4G01Accessor(TransportT& spi_transport,
                      size_t      flash_start = 0)
        : spi_(spi_transport)
        , flash_start_(flash_start)
    {
    }

    // ─────────────────────────────────────────────
    // Accessor concept API
    // ─────────────────────────────────────────────
    AccessorError write(size_t address, const uint8_t* data, size_t size);
    AccessorError read(size_t address, uint8_t* data, size_t size);
    AccessorError erase(size_t address);
    void format(); 

    size_t getAlignment() const         { return PAGE_SIZE; }
    size_t getFlashMemorySize() const   { return TOTAL_SIZE; }
    size_t getFlashStartAddress() const { return flash_start_; }
    size_t getEraseBlockSize() const { return BLOCK_SIZE; }

    // ─────────────────────────────────────────────
    // Public mapping for testing / introspection
    // ─────────────────────────────────────────────
    struct PhysAddr
    {
        uint32_t block;
        uint32_t page_in_block;
        uint32_t column;
    };

    PhysAddr logicalToPhysical(size_t logical_addr) const;

private:
    // ─────────────────────────────────────────────
    // Low-level helpers
    // ─────────────────────────────────────────────
    bool writeEnable();
    bool readStatus(uint8_t& status);
    bool waitReady();

    bool isBadBlock(uint32_t block);

    bool readPage(uint32_t block,
                  uint32_t page_in_block,
                  uint8_t* page_buf);

    bool programPage(uint32_t block,
                     uint32_t page_in_block,
                     const uint8_t* page_buf);

    bool eraseBlock(uint32_t block);

    // Build 3-byte row address (block+page)
    void buildRowAddress(uint32_t block,
                         uint32_t page_in_block,
                         uint8_t row_addr[3]) const;

private:
    TransportT& spi_;
    size_t      flash_start_;

    std::array<uint8_t, PAGE_TOTAL_SIZE> page_cache_{};
};

// ─────────────────────────────────────────────
// Implementation
// ─────────────────────────────────────────────

template <StreamAccessTransport TransportT>
inline typename MT29F4G01Accessor<TransportT>::PhysAddr
MT29F4G01Accessor<TransportT>::logicalToPhysical(size_t logical_addr) const
{
    const size_t   page_index   = logical_addr / PAGE_SIZE;
    const uint32_t column       = static_cast<uint32_t>(logical_addr % PAGE_SIZE);

    const uint32_t block        =
        static_cast<uint32_t>(page_index / PAGES_PER_BLOCK);
    const uint32_t page_in_blk  =
        static_cast<uint32_t>(page_index % PAGES_PER_BLOCK);

    return PhysAddr{ block, page_in_blk, column };
}

template <StreamAccessTransport TransportT>
inline void
MT29F4G01Accessor<TransportT>::buildRowAddress(uint32_t block,
                                               uint32_t page_in_block,
                                               uint8_t row_addr[3]) const
{
    const uint32_t row =
        static_cast<uint32_t>(block) *
        static_cast<uint32_t>(PAGES_PER_BLOCK) +
        static_cast<uint32_t>(page_in_block);

    row_addr[0] = static_cast<uint8_t>((row >> 16) & 0xFF); // RA[23:16] (upper 7 bits dummy)
    row_addr[1] = static_cast<uint8_t>((row >> 8)  & 0xFF); // RA[15:8]
    row_addr[2] = static_cast<uint8_t>( row        & 0xFF); // RA[7:0]
}

template <StreamAccessTransport TransportT>
inline bool
MT29F4G01Accessor<TransportT>::writeEnable()
{
    const uint8_t cmd = static_cast<uint8_t>(MT29_CMD::WRITE_ENABLE);
    return spi_.write(&cmd, 1U);
}

template <StreamAccessTransport TransportT>
inline bool
MT29F4G01Accessor<TransportT>::readStatus(uint8_t& status)
{
    // GET FEATURE (0Fh), feature address C0h (status), then 1 data byte
    uint8_t cmd[2] = { static_cast<uint8_t>(MT29_CMD::GET_FEATURE), static_cast<uint8_t>(MT29_CMD::FEATURE_ADDR_STATUS) };
    if (!spi_.write(cmd, sizeof(cmd)))
        return false;
    if (!spi_.read(&status, 1U))
        return false;
    return true;
}

template <StreamAccessTransport TransportT>
inline bool
MT29F4G01Accessor<TransportT>::waitReady()
{
    // Poll STATUS_OIP until clear. In tests, MockSPITransport.read()
    // returns a pattern where bit 0 is 0, so this returns immediately.
    uint8_t status = 0;
    for (unsigned i = 0; i < 100000U; ++i)
    {
        if (!readStatus(status))
            return false;
        if ((status & static_cast<uint8_t>(MT29_CMD::STATUS_OIP)) == 0U)
            return true;
    }
    // Timeout
    return false;
}

template <StreamAccessTransport TransportT>
inline bool
MT29F4G01Accessor<TransportT>::isBadBlock(uint32_t block)
{
    // TODO: read spare byte 0 of page 0 (or 1) and check != 0xFF → bad.
    // For now, trust all blocks.
    (void)block;
    return false;
}

// ─────────────────────────────────────────────
// Page read: array → cache → host buffer
// ─────────────────────────────────────────────
template <StreamAccessTransport TransportT>
inline bool
MT29F4G01Accessor<TransportT>::readPage(uint32_t block,
                                        uint32_t page_in_block,
                                        uint8_t* page_buf)
{
    if (isBadBlock(block))
        return false;

    uint8_t row_addr[3];
    buildRowAddress(block, page_in_block, row_addr);

    // 1) PAGE READ (13h) with 3-byte row address
    uint8_t cmd_pr[4];
    cmd_pr[0] = static_cast<uint8_t>(MT29_CMD::PAGE_READ);
    cmd_pr[1] = row_addr[0];
    cmd_pr[2] = row_addr[1];
    cmd_pr[3] = row_addr[2];
    if (!spi_.write(cmd_pr, sizeof(cmd_pr)))
        return false;

    // 2) Wait until OIP = 0
    if (!waitReady())
        return false;

    // 3) READ FROM CACHE x1 (03h) from column 0 with 1 dummy byte
    uint8_t cmd_rc[4];
    cmd_rc[0] = static_cast<uint8_t>(MT29_CMD::READ_FROM_CACHE);
    cmd_rc[1] = 0x00; // column low
    cmd_rc[2] = 0x00; // column high (13-bit col, upper bits 0)
    cmd_rc[3] = 0x00; // dummy byte

    if (!spi_.write(cmd_rc, sizeof(cmd_rc)))
        return false;

    if (!spi_.read(page_buf, PAGE_TOTAL_SIZE))
        return false;

    return true;
}

// ─────────────────────────────────────────────
// Page program: host buffer → cache → array
// ─────────────────────────────────────────────
template <StreamAccessTransport TransportT>
inline bool
MT29F4G01Accessor<TransportT>::programPage(uint32_t block,
                                           uint32_t page_in_block,
                                           const uint8_t* page_buf)
{
    if (isBadBlock(block))
        return false;

    uint8_t row_addr[3];
    buildRowAddress(block, page_in_block, row_addr);

    // 1) WRITE ENABLE
    if (!writeEnable())
        return false;

    // 2) PROGRAM LOAD x1 (02h), column 0, then page data
    uint8_t cmd_pl[3];
    cmd_pl[0] = static_cast<uint8_t>(MT29_CMD::PROGRAM_LOAD);
    cmd_pl[1] = 0x00; // column low
    cmd_pl[2] = 0x00; // column high
    if (!spi_.write(cmd_pl, sizeof(cmd_pl)))
        return false;

    if (!spi_.write(page_buf, PAGE_TOTAL_SIZE))
        return false;

    // 3) PROGRAM EXECUTE (10h) with row address
    uint8_t cmd_pe[4];
    cmd_pe[0] = static_cast<uint8_t>(MT29_CMD::PROGRAM_EXECUTE);
    cmd_pe[1] = row_addr[0];
    cmd_pe[2] = row_addr[1];
    cmd_pe[3] = row_addr[2];

    if (!spi_.write(cmd_pe, sizeof(cmd_pe)))
        return false;

    // 4) Wait ready and check P_FAIL
    if (!waitReady())
        return false;

    uint8_t status = 0;
    if (!readStatus(status))
        return false;
    if (status & static_cast<uint8_t>(MT29_CMD::STATUS_P_FAIL))
        return false;

    return true;
}

// ─────────────────────────────────────────────
// Block erase
// ─────────────────────────────────────────────
template <StreamAccessTransport TransportT>
inline bool
MT29F4G01Accessor<TransportT>::eraseBlock(uint32_t block)
{
    if (isBadBlock(block))
        return false;

    uint8_t row_addr[3];
    buildRowAddress(block, 0U, row_addr); // page 0 in block

    // 1) WRITE ENABLE
    if (!writeEnable())
        return false;

    // 2) BLOCK ERASE (D8h) with row address
    uint8_t cmd_ers[4];
    cmd_ers[0] = static_cast<uint8_t>(MT29_CMD::BLOCK_ERASE);
    cmd_ers[1] = row_addr[0];
    cmd_ers[2] = row_addr[1];
    cmd_ers[3] = row_addr[2];

    if (!spi_.write(cmd_ers, sizeof(cmd_ers)))
        return false;

    // 3) Wait ready and check E_FAIL
    if (!waitReady())
        return false;

    uint8_t status = 0;
    if (!readStatus(status))
        return false;
    if (status & static_cast<uint8_t>(MT29_CMD::STATUS_E_FAIL))
        return false;

    return true;
}

// ─────────────────────────────────────────────
// Accessor API: read / write / erase
// ─────────────────────────────────────────────

template <StreamAccessTransport TransportT>
inline AccessorError
MT29F4G01Accessor<TransportT>::read(size_t address,
                                    uint8_t* data,
                                    size_t size)
{
    if (address + size > TOTAL_SIZE)
        return AccessorError::OUT_OF_BOUNDS;

    size_t remaining = size;
    size_t dst_off   = 0;
    size_t logical   = address;

    while (remaining > 0)
    {
        auto   phys          = logicalToPhysical(logical);
        size_t in_page_off   = phys.column;
        size_t bytes_in_page = PAGE_SIZE - in_page_off;
        size_t chunk         = std::min(remaining, bytes_in_page);

        if (!readPage(phys.block, phys.page_in_block, page_cache_.data()))
            return AccessorError::READ_ERROR;

        std::memcpy(data + dst_off,
                    page_cache_.data() + in_page_off,
                    chunk);

        logical   += chunk;
        dst_off   += chunk;
        remaining -= chunk;
    }

    return AccessorError::NO_ERROR;
}

template <StreamAccessTransport TransportT>
inline AccessorError
MT29F4G01Accessor<TransportT>::write(size_t address,
                                     const uint8_t* data,
                                     size_t size)
{
    if (address + size > TOTAL_SIZE)
        return AccessorError::OUT_OF_BOUNDS;

    size_t remaining = size;
    size_t src_off   = 0;
    size_t logical   = address;

    while (remaining > 0)
    {
        auto   phys          = logicalToPhysical(logical);
        size_t in_page_off   = phys.column;
        size_t bytes_in_page = PAGE_SIZE - in_page_off;
        size_t chunk         = std::min(remaining, bytes_in_page);

        // For append-only usage, assume pages are erased and program full page.
        std::fill(page_cache_.begin(), page_cache_.end(), 0xFF);

        std::memcpy(page_cache_.data() + in_page_off,
                    data + src_off,
                    chunk);

        if (!programPage(phys.block, phys.page_in_block, page_cache_.data()))
            return AccessorError::WRITE_ERROR;

        logical   += chunk;
        src_off   += chunk;
        remaining -= chunk;
    }

    return AccessorError::NO_ERROR;
}

template <StreamAccessTransport TransportT>
void MT29F4G01Accessor<TransportT>::format() {
    const size_t block = getEraseBlockSize();
    const size_t start = getFlashStartAddress();
    const size_t size  = getFlashMemorySize();

    for (size_t addr = start; addr < start + size; addr += block)
        erase(addr);
}


template <StreamAccessTransport TransportT>
inline AccessorError
MT29F4G01Accessor<TransportT>::erase(size_t address)
{
    if (address >= TOTAL_SIZE)
        return AccessorError::OUT_OF_BOUNDS;

    const auto phys = logicalToPhysical(address);
    if (!eraseBlock(phys.block))
        return AccessorError::WRITE_ERROR; // or ERASE_ERROR if you add it

    return AccessorError::NO_ERROR;
}

#endif // MT29F4G01_ACCESSOR_H
