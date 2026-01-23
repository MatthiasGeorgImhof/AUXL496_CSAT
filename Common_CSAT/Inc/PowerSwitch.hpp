#ifndef _POWER_SWITCH_H_
#define _POWER_SWITCH_H_

#include "Transport.hpp"
#include <cstdint>
#include <cstring>
#include <array>

// MCP23008 Register Definitions
enum class MCP23008_REGISTERS : uint8_t
{
    MCP23008_IODIR = 0x00,
    MCP23008_IPOL = 0x01,
    MCP23008_GPINTEN = 0x02,
    MCP23008_DEFVAL = 0x03,
    MCP23008_INTCON = 0x04,
    MCP23008_IOCON = 0x05,
    MCP23008_GPPU = 0x06,
    MCP23008_INTF = 0x07,
    MCP23008_INTCAP = 0x08,
    MCP23008_GPIO = 0x09,
    MCP23008_OLAT = 0x0A,
};

enum class CIRCUITS : uint8_t
{
    CIRCUIT_0 = 0b00000001,
    CIRCUIT_1 = 0b00000010,
    CIRCUIT_2 = 0b00000100,
    CIRCUIT_3 = 0b00001000,
    CIRCUIT_4 = 0b00010000,
    CIRCUIT_5 = 0b00100000,
    CIRCUIT_6 = 0b01000000,
    CIRCUIT_7 = 0b10000000,
};

template <typename Transport>
    requires RegisterModeTransport<Transport>
class PowerSwitch
{
public:
    PowerSwitch() = delete;

    explicit PowerSwitch(const Transport &transport, GPIO_TypeDef *resetPort, uint16_t resetPin)
        : transport_(transport), register_value_(0), reset_port_(resetPort), reset_pin_(resetPin)
    {
        releaseReset();
    }

    bool on(CIRCUITS circuit)
    {
        register_value_ |= static_cast<uint8_t>(circuit);
        return writeRegister(MCP23008_REGISTERS::MCP23008_OLAT, &register_value_, 1);
    }

    bool off(CIRCUITS circuit)
    {
        register_value_ &= ~static_cast<uint8_t>(circuit);
        return writeRegister(MCP23008_REGISTERS::MCP23008_OLAT, &register_value_, 1);
    }

    bool status(CIRCUITS circuit) const
    {
        return (register_value_ & static_cast<uint8_t>(circuit));
    }

    bool setState(uint8_t mask)
    {
        register_value_ = mask;
        return writeRegister(MCP23008_REGISTERS::MCP23008_OLAT, &register_value_, 1);
    }

    uint8_t getState()
    {
        register_value_ = readRegister(MCP23008_REGISTERS::MCP23008_OLAT);
        return register_value_;
    }

    void holdReset()
    {
        // Set *Reset pin low
        HAL_GPIO_WritePin(reset_port_, reset_pin_, GPIO_PIN_RESET);
    }

    void releaseReset()
    {
        // Set *Reset pin high
        HAL_GPIO_WritePin(reset_port_, reset_pin_, GPIO_PIN_SET);

        uint8_t reset[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        writeRegister(MCP23008_REGISTERS::MCP23008_IODIR, reset, sizeof(reset));
    }

private:
    bool writeRegister(MCP23008_REGISTERS reg, const uint8_t *data, uint16_t size)
    {
        return transport_.write_reg(static_cast<uint16_t>(reg), data, size);
    }

    uint8_t readRegister(MCP23008_REGISTERS reg) const
    {
        uint8_t rx{};
        transport_.read_reg(static_cast<uint16_t>(reg), &rx, 1);
        return rx;
    }

private:
    const Transport &transport_;
    uint8_t register_value_;
    GPIO_TypeDef *reset_port_;
    uint16_t reset_pin_;
};

#endif /* _POWER_SWITCH_H_ */
