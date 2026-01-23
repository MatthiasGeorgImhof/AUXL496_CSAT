#ifndef __OVXXXX_Common_HPP__
#define __OVXXXX_Common_HPP__

#include <cstdint>
#include <cstdio>

using Word_Byte_t = struct
{
    uint16_t addr;
    uint8_t data;
};

static inline void word_byte_to_string(char *buffer,
                         size_t buf_size,
                         const Word_Byte_t *registers,
                         size_t reg_size)
{
    size_t pos = 0;

    for (size_t i = 0; i < reg_size; ++i) {
        int written = snprintf(buffer + pos,
                               buf_size - pos,
                               "0x%04X = 0x%02X\r\n",
                               registers[i].addr,
                               registers[i].data);

        if (written < 0 || static_cast<size_t>(written) >= buf_size - pos) {
            // No more space
            break;
        }

        pos += static_cast<size_t>(written);
    }

    // Ensure null termination
    if (buf_size > 0) {
        buffer[buf_size - 1] = '\0';
    }
}

#endif // __OVXXXX_Common_HPP__
