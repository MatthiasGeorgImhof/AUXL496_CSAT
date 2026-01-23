#ifndef __METADATA_HPP__
#define __METADATA_HPP__

#include <cstdint>
#include <cstddef>

typedef uint32_t crc_t;

// -----------------------------------------------------------------------------
// Semantic source of the image (producer identity)
// -----------------------------------------------------------------------------
enum class METADATA_PRODUCER : uint8_t
{
    CAMERA_1 = 0,
    CAMERA_2 = 1,
    CAMERA_3 = 2,
    THERMAL  = 3,

    // Future sources can be added here
};

enum class METADATA_FORMAT : uint16_t
{
    MX2F = 1,
    UNKN = 0xFFFF,
};

struct Dimensions
{
    uint16_t n1, n2, n3;
};

// -----------------------------------------------------------------------------
// Semantic Image Metadata (versioned, packed, CRC-protected)
// This is NOT the storage header. This is what producers/consumers care about.
// -----------------------------------------------------------------------------
#pragma pack(push, 1)
struct ImageMetadata
{
    uint16_t version;         // metadata version
    uint16_t metadata_size;   // sizeof(ImageMetadata) at creation time

    uint64_t timestamp;       // seconds or ms since epoch
    float    latitude;        // degrees
    float    longitude;       // degrees
    
    uint32_t payload_size;      // payload size in bytes
    Dimensions dimensions;    // payload dimensions

    METADATA_FORMAT format;   // payload record format
    METADATA_PRODUCER producer;// payload producer identity

    uint8_t  reserved[8];     // reserved for future expansion

    crc_t    meta_crc;        // CRC over all previous fields
};
#pragma pack(pop)

// -----------------------------------------------------------------------------
// Explicit size check (future-proof, readable)
// -----------------------------------------------------------------------------
static_assert(
    sizeof(ImageMetadata) ==
        sizeof(uint16_t) +   // version
        sizeof(uint16_t) +   // metadata_size
        sizeof(uint64_t) +   // timestamp
        sizeof(float)   +    // latitude
        sizeof(float)   +    // longitude
        sizeof(uint32_t) +   // payload_size
        sizeof(Dimensions) + // dimensions
        sizeof(METADATA_FORMAT) + // format
        sizeof(METADATA_PRODUCER)  + // producer
        sizeof(uint8_t) * 8 +// reserved
        sizeof(crc_t),       // meta_crc
    "Unexpected ImageMetadata size"
);

// Convenience constants
constexpr size_t METADATA_SIZE = sizeof(ImageMetadata);
constexpr size_t METADATA_SIZE_WO_CRC = offsetof(ImageMetadata, meta_crc);

#endif // __METADATA_HPP__
