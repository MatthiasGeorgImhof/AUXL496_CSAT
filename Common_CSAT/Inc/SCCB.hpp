#pragma once

#include <cstdint>
#include "GpioPin.hpp"

#ifdef __arm__
#include "stm32l4xx_hal.h"
#elif __x86_64__
#include "mock_hal.h"
#endif

//
// 1. Stateless SCCB core (protocol only)
//

struct SCCB_Core
{
    template <typename Bus>
    static void start(Bus &bus)
    {
        bus.sda_as_output_od();
        bus.sda_high();
        bus.scl_high();
        bus.delay();
        bus.sda_low();
        bus.delay();
        bus.scl_low();
    }

    template <typename Bus>
    static void stop(Bus &bus)
    {
        bus.sda_as_output_od();
        bus.sda_low();
        bus.delay();
        bus.scl_high();
        bus.delay();
        bus.sda_high();
        bus.delay();
    }

    template <typename Bus>
    static void write_byte(Bus &bus, uint8_t b)
    {
        bus.sda_as_output_od();

        for (int i = 0; i < 8; ++i)
        {
            (b & 0x80) ? bus.sda_high() : bus.sda_low();
            bus.delay();
            bus.scl_high();
            bus.delay();
            bus.scl_low();
            bus.delay();
            b <<= 1;
        }

        // ACK ignored
        bus.sda_high();
        bus.delay();
        bus.scl_high();
        bus.delay();
        bus.scl_low();
    }

    template <typename Bus>
    static uint8_t read_byte(Bus &bus)
    {
        uint8_t v = 0;

        bus.sda_as_input(); // release SDA

        for (int i = 0; i < 8; ++i)
        {
            v <<= 1;
            bus.scl_high();
            bus.delay();
            if (bus.sda_read())
                v |= 1;
            bus.scl_low();
            bus.delay();
        }

        // NACK
        bus.sda_as_output_od();
        bus.sda_high();
        bus.delay();
        bus.scl_high();
        bus.delay();
        bus.scl_low();

        return v;
    }
};

//
// 2. Concrete STM32 SCCB bus using GpioPin<PortAddr, Pin>
//

template <typename SCLPin, typename SDAPin, uint32_t DelayCycles = 200>
class STM32_SCCB_Bus
{
public:
    STM32_SCCB_Bus()
        : scl_{}, sda_{} {}

    // GPIO mode control for SDA
    void sda_as_input()
    {
        GPIO_InitTypeDef GPIO_InitStruct{};
        GPIO_InitStruct.Pin = SDAPin::pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(SDAPin::port(), &GPIO_InitStruct);
    }

    void sda_as_output_od()
    {
        GPIO_InitTypeDef GPIO_InitStruct{};
        GPIO_InitStruct.Pin = SDAPin::pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(SDAPin::port(), &GPIO_InitStruct);
    }

    // Configure both pins for SCCB bit‑bang mode
    void reconfigure_pins_to_sccb()
    {
        GPIO_InitTypeDef GPIO_InitStruct{};

        __HAL_RCC_GPIOB_CLK_ENABLE();

        // SCL
        GPIO_InitStruct.Pin = SCLPin::pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(SCLPin::port(), &GPIO_InitStruct);

        // SDA
        GPIO_InitStruct.Pin = SDAPin::pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(SDAPin::port(), &GPIO_InitStruct);

        scl_high();
        sda_high();
    }

    // Restore pins to I2C peripheral mode
    void reconfigure_pins_to_i2c()
    {
        GPIO_InitTypeDef GPIO_InitStruct{};

        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin = SCLPin::pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(SCLPin::port(), &GPIO_InitStruct);

        GPIO_InitStruct.Pin = SDAPin::pin;
        HAL_GPIO_Init(SDAPin::port(), &GPIO_InitStruct);
    }

    // Bit‑level operations used by SCCB_Core
    void scl_high() { scl_.high(); }
    void scl_low()  { scl_.low();  }
    void sda_high() { sda_.high(); }
    void sda_low()  { sda_.low();  }

    bool sda_read() { return sda_.read(); }

    void delay()
    {
#ifndef __arm__
        for (uint32_t i = 0; i < DelayCycles; ++i) {}
#else
        for (uint32_t i = 0; i < DelayCycles; ++i) {
            __asm__ volatile("nop");
        }
#endif
    }

private:
    SCLPin scl_;
    SDAPin sda_;
};

//
// 3. 8‑bit and 16‑bit register helpers on top of SCCB_Core
//

struct SCCB_Regs
{
    template <typename Bus>
    static void write_reg8(Bus &bus, uint8_t dev, uint8_t reg, uint8_t val)
    {
        SCCB_Core::start(bus);
        SCCB_Core::write_byte(bus, static_cast<uint8_t>((dev << 1) | 0)); // write
        SCCB_Core::write_byte(bus, reg);
        SCCB_Core::write_byte(bus, val);
        SCCB_Core::stop(bus);
    }

    template <typename Bus>
    static uint8_t read_reg8(Bus &bus, uint8_t dev, uint8_t reg)
    {
        uint8_t val = 0;

        // Write register address
        SCCB_Core::start(bus);
        SCCB_Core::write_byte(bus, static_cast<uint8_t>((dev << 1) | 0)); // write
        SCCB_Core::write_byte(bus, reg);
        SCCB_Core::stop(bus);

        // Read data
        SCCB_Core::start(bus);
        SCCB_Core::write_byte(bus, static_cast<uint8_t>((dev << 1) | 1)); // read
        val = SCCB_Core::read_byte(bus);
        SCCB_Core::stop(bus);

        return val;
    }

    template <typename Bus>
    static void write_reg16(Bus &bus, uint8_t dev, uint16_t reg, uint8_t val)
    {
        SCCB_Core::start(bus);
        SCCB_Core::write_byte(bus, static_cast<uint8_t>((dev << 1) | 0)); // write
        SCCB_Core::write_byte(bus, static_cast<uint8_t>(reg >> 8));
        SCCB_Core::write_byte(bus, static_cast<uint8_t>(reg & 0xFF));
        SCCB_Core::write_byte(bus, val);
        SCCB_Core::stop(bus);
    }

    template <typename Bus>
    static uint8_t read_reg16(Bus &bus, uint8_t dev, uint16_t reg)
    {
        uint8_t val = 0;

        // Write register address (16‑bit)
        SCCB_Core::start(bus);
        SCCB_Core::write_byte(bus, static_cast<uint8_t>((dev << 1) | 0)); // write
        SCCB_Core::write_byte(bus, static_cast<uint8_t>(reg >> 8));
        SCCB_Core::write_byte(bus, static_cast<uint8_t>(reg & 0xFF));
        SCCB_Core::stop(bus);

        // Read data
        SCCB_Core::start(bus);
        SCCB_Core::write_byte(bus, static_cast<uint8_t>((dev << 1) | 1)); // read
        val = SCCB_Core::read_byte(bus);
        SCCB_Core::stop(bus);

        return val;
    }
};

//
// 4. Example typedefs matching your current usage
//

// Example: B8/B9 as SCCB pins
// using SCCB_Bus0 = STM32_SCCB_Bus<GPIOB_BASE, GPIO_PIN_8,
//                                  GPIOB_BASE, GPIO_PIN_9>;

// Usage in your code:
//
//   SCCB_Bus0 bus;
//   bus.reconfigure_pins_to_sccb();
//   uint8_t id_h = SCCB_Regs::read_reg16(bus, OV5640_ID, 0x300A);
//   uint8_t id_l = SCCB_Regs::read_reg16(bus, OV5640_ID, 0x300B);
//   bus.reconfigure_pins_to_i2c();
//
