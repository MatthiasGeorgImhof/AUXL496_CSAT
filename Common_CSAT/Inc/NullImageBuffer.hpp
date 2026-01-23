#ifndef NULL_IMAGE_BUFFER_HPP
#define NULL_IMAGE_BUFFER_HPP

#include <cstddef>
#include <cstdint>
#include "imagebuffer/buffer_state.hpp" // ImageBufferError
#include "imagebuffer/image.hpp"        // ImageMetadata
#include "ImageBuffer.hpp"              // ImageMetadata
#include "Logger.hpp"

// -----------------------------------------------------------------------------
// NullImageBuffer: behaves like /dev/null
// - All writes succeed
// - Nothing is stored
// - Always empty
// - Logs metadata headers
// -----------------------------------------------------------------------------
class NullImageBuffer
{
public:
    NullImageBuffer() = default;

    // -------------------------------------------------------------------------
    // Write path
    // -------------------------------------------------------------------------
    ImageBufferError add_image(const ImageMetadata &meta)
    {
        log(LOG_LEVEL_INFO,
            "NullImageBuffer: add_image() v=%u size=%u ts=%llu lat=%.6f lon=%.6f dims=(%u,%u,%u) fmt=%u prod=%u\r\n",
            meta.version,
            meta.payload_size,
            static_cast<unsigned long long>(meta.timestamp),
            static_cast<double>(meta.latitude),
            static_cast<double>(meta.longitude),
            meta.dimensions.n1,
            meta.dimensions.n2,
            meta.dimensions.n3,
            static_cast<unsigned>(meta.format),
            static_cast<unsigned>(meta.producer));

        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError add_data_chunk(const uint8_t * /*data*/, size_t & /*size*/)
    {
        // Accept everything, store nothing.
        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError push_image()
    {
        // Finalize image (no-op).
        log(LOG_LEVEL_DEBUG, "NullImageBuffer: push_image()\r\n");
        return ImageBufferError::NO_ERROR;
    }

    // -------------------------------------------------------------------------
    // Read path — always empty
    // -------------------------------------------------------------------------
    ImageBufferError get_image(ImageMetadata & /*meta*/)
    {
        return ImageBufferError::EMPTY_BUFFER;
    }

    ImageBufferError get_data_chunk(uint8_t * /*data*/, size_t &size)
    {
        size = 0;
        return ImageBufferError::EMPTY_BUFFER;
    }

    ImageBufferError pop_image()
    {
        return ImageBufferError::EMPTY_BUFFER;
    }

    // -------------------------------------------------------------------------
    // State queries — always empty
    // -------------------------------------------------------------------------
    bool is_empty() const { return true; }
    size_t size() const { return 0; }
    size_t count() const { return 0; }
    size_t available() const { return 0; }
    size_t capacity() const { return 0; }
    size_t get_head() const { return 0; }
    size_t get_tail() const { return 0; }

    // -------------------------------------------------------------------------
    // Boot-time reconstruction — trivially empty
    // -------------------------------------------------------------------------
    ImageBufferError initialize_from_flash()
    {
        return ImageBufferError::NO_ERROR;
    }
};

#endif // NULL_IMAGE_BUFFER_HPP
