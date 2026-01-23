#ifndef CHECKSUM_HPP
#define CHECKSUM_HPP

#include <cstdint>
#include <cstddef>
#include <cstdio>

#include "ChecksumPolicy.hpp"

typedef uint32_t crc_t;

class ChecksumCalculator
{
public:
    ChecksumCalculator() : crc_(0xFFFFFFFFu) {}

    void reset() { crc_ = 0xFFFFFFFFu; }

    void update(const uint8_t* data, size_t size)
    {
        for (size_t i = 0; i < size; i++)
        {
            uint32_t c = (crc_ ^ data[i]) & 0xFFu;
            for (int k = 0; k < 8; k++)
                c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            crc_ = (crc_ >> 1) ^ c;
        }
    }

    crc_t get_checksum() const { return crc_ ^ 0xFFFFFFFFu; }

private:
    uint32_t crc_;
};

// -----------------------------------------------------------------------------
// Default checksum policy wrapper
// -----------------------------------------------------------------------------
struct DefaultChecksumPolicy
{
    ChecksumCalculator calc;

    void reset() { calc.reset(); }
    void update(const uint8_t* data, size_t size) { calc.update(data, size); }
    crc_t get() const { return calc.get_checksum(); }
};

#endif // CHECKSUM_HPP