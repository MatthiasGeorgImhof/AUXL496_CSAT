#ifndef CHECKSUM_HPP
#define CHECKSUM_HPP

#include <cstdint>
#include <cstddef>
#include <cstdio>

typedef uint32_t crc_t;

class ChecksumCalculator
{
public:
  ChecksumCalculator(crc_t initial_checksum = 0) : checksum_(initial_checksum) {}

  void update(const uint8_t *data, size_t size)
  {
    // printf("%4zu %4u: ", size, checksum_);
    for (size_t i = 0; i < size; ++i)
    {
      checksum_ ^= data[i];
      // printf("%2x ", data[i]);
    }
    // printf("%2x\n", checksum_);
  }

  crc_t get_checksum() const { return checksum_; }

  void reset(crc_t initial_checksum = 0) { checksum_ = initial_checksum; }

private:
  crc_t checksum_;
};

#endif // CHECKSUM_HPP