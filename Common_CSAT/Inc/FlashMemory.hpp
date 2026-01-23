#ifndef FLASH_MEMORY_HPP
#define FLASH_MEMORY_HPP

#include "Transport.hpp"

template <typename Transport>
requires StreamAccessTransport<Transport>
class FlashMemory
{
public:
    FlashMemory(const Transport& transport,
                          GPIO_TypeDef* cs_port,
                          uint16_t cs_pin)
        : transport_(transport),
          cs_port_(cs_port),
          cs_pin_(cs_pin)
    {}

    // Required by StreamAccessTransport
    bool write(const uint8_t* data, uint16_t len) const
    {
        select();
        bool ok = transport_.write(data, len);
        deselect();
        return ok;
    }

    bool read(uint8_t* data, uint16_t len) const
    {
        select();
        bool ok = transport_.read(data, len);
        deselect();
        return ok;
    }

    // Optional: combined write+read
    bool transfer(const uint8_t* tx, uint8_t* rx, uint16_t len) const
    {
        select();
        bool ok = transport_.transfer(tx, rx, len);
        deselect();
        return ok;
    }

private:
    inline void select() const
    {
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    }

    inline void deselect() const
    {
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
    }

    const Transport& transport_;
    GPIO_TypeDef* cs_port_;
    uint16_t cs_pin_;
};

#endif // FLASH_MEMORY_HPP