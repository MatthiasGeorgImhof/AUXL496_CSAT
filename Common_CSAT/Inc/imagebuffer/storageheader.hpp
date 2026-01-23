#ifndef __STORAGE_HEADER_HPP__
#define __STORAGE_HEADER_HPP__

#include <cstdint>
#include <cstddef>

// -----------------------------------------------------------------------------
// StorageHeader: internal, fixed-size, versioned, CRC-protected metadata block
// This precedes every image entry in NAND.
// -----------------------------------------------------------------------------

// Magic constant for identifying valid entries ("RCRD")
constexpr uint32_t STORAGE_MAGIC =
    (static_cast<uint32_t>('R') << 24) |
    (static_cast<uint32_t>('C') << 16) |
    (static_cast<uint32_t>('R') << 8)  |
    static_cast<uint32_t>('D');

// Version of the StorageHeader format
constexpr uint16_t STORAGE_HEADER_VERSION = 1;

#pragma pack(push, 1)
struct StorageHeader
{
    uint32_t magic;          // 4 bytes: STORAGE_MAGIC
    uint16_t version;        // 2 bytes: STORAGE_HEADER_VERSION
    uint16_t header_size;    // 2 bytes: sizeof(StorageHeader) at creation time

    uint32_t sequence_id;    // 4 bytes: monotonic increasing ID
    uint32_t total_size;     // 4 bytes: size of [ImageMetadata + payload + data CRC]

    uint32_t flags;          // 4 bytes: reserved for VALID/PARTIAL/DELETED/etc.

    uint8_t  reserved[16];   // 16 bytes: future expansion

    uint32_t header_crc;     // 4 bytes: CRC over all previous bytes
};
#pragma pack(pop)

// -----------------------------------------------------------------------------
// Explicit size check using sum of field sizes (future-proof, readable)
// -----------------------------------------------------------------------------
static_assert(
    sizeof(StorageHeader) ==
        sizeof(uint32_t) +        // magic
        sizeof(uint16_t) +        // version
        sizeof(uint16_t) +        // header_size
        sizeof(uint32_t) +        // sequence_id
        sizeof(uint32_t) +        // total_size
        sizeof(uint32_t) +        // flags
        sizeof(uint8_t) * 16 +    // reserved
        sizeof(uint32_t),         // header_crc
    "Unexpected StorageHeader size"
);

// Convenience constants
constexpr size_t STORAGE_SIZE = sizeof(StorageHeader);
constexpr size_t STORAGE_SIZE_WO_CRC = offsetof(StorageHeader, header_crc);

#endif // __STORAGE_HEADER_HPP__
