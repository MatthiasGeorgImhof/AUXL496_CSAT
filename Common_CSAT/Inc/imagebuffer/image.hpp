#ifndef __IMAGE_HPP__
#define __IMAGE_HPP__

#include <cstdint>
#include <cstddef>

typedef uint32_t crc_t;
typedef uint32_t image_magic_t;
constexpr static image_magic_t IMAGE_MAGIC = ('I' << 24) | ('M' << 16) | ('T' << 8) | 'A';

void print(const uint8_t *data, size_t size)
{
    printf("           ");
    for (size_t i = 0; i < size; ++i)
    {
        printf("%2x ", data[i]);
    }
    printf("\r\n");
}

// Data Structures (same as before)
struct ImageMetadata
{
    const image_magic_t magic;
    uint32_t timestamp;
    size_t image_size;
    float latitude;
    float longitude;
    uint8_t camera_index;
    mutable crc_t checksum;
    
    ImageMetadata() : magic(IMAGE_MAGIC) {}
    ImageMetadata& operator=(const ImageMetadata& other) {
    if (this != &other)
    {
        timestamp = other.timestamp;
        image_size = other.image_size;
        latitude = other.latitude;
        longitude = other.longitude;
        camera_index = other.camera_index;
        checksum = other.checksum;
    }
    return *this;
}
};
constexpr size_t METADATA_SIZE_WO_CHECKSUM = offsetof(ImageMetadata, checksum);
constexpr size_t METADATA_SIZE = sizeof(ImageMetadata);


#endif // __IMAGE_HPP__
