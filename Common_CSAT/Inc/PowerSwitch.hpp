#ifndef _POWER_SWITCH_H_
#define _POWER_SWITCH_H_

#include "mock_hal.h"
#include <array>
#include <cstdint>

class PowerSwitch
{
public:
    PowerSwitch() = delete;
    PowerSwitch(I2C_HandleTypeDef *hi2c, uint8_t i2c_address) : hi2c_(hi2c), i2c_address_(i2c_address), register_value_(0)
    {
    	uint8_t reset[] = {0,0,0,0,0,0,0,0,0,0,0};
    	writeRegister(IODIR, reset, sizeof(reset));
    }

    bool on(const uint8_t slot)
    {
        if (invalidslot(slot))
        {
            return false;
        }

        uint8_t mask = 1 << (2 * slot);
        register_value_ |= mask;
        return writeRegister(GPIO, &register_value_, 1);
    }

    bool off(const uint8_t slot)
    {
        if (invalidslot(slot))
        {
            return false;
        }

        uint8_t mask = ~(1 << (2 * slot));
        register_value_ &= mask;
        return writeRegister(GPIO, &register_value_, 1);
    }

    bool status(const uint8_t slot)
    {
        if (invalidslot(slot))
        {
            return false;
        }

        uint8_t mask = 1 << (2 * slot);
        return (register_value_ & mask) != 0;
    }

    bool  setState(const uint8_t mask)
    {
    	register_value_ = mask;
        return writeRegister(GPIO, &register_value_, 1);
    }

    uint8_t getState()
    {
    	return register_value_;
    }

private:
    static constexpr uint8_t IODIR = 0x00;
    static constexpr uint8_t GPIO = 0x09;
    static constexpr uint8_t OLAT = 0x0A;

private:
    bool writeRegister(uint8_t reg, uint8_t *data, uint16_t size)
    {
        return (HAL_I2C_Mem_Write(hi2c_, i2c_address_<<1, reg, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY) == HAL_OK);
    }

    inline bool invalidslot(const uint8_t slot)
    {
        return (slot >= 4);
    }

private:
    I2C_HandleTypeDef *hi2c_;
    uint8_t i2c_address_;
    uint8_t register_value_;
};

#endif /*_POWER_SWITCH_H */
