#ifndef _I2C_SWITCH_HPP_
#define _I2C_SWITCH_HPP_

#include "Transport.hpp"
#include "GpioPin.hpp"
#include <cstdint>

enum class I2CSwitchChannel : uint8_t
{
    None     = 0x00,
    Channel0 = 0x01,
    Channel1 = 0x02,
    Channel2 = 0x04,
    Channel3 = 0x08,
    Channel4 = 0x10,
    Channel5 = 0x20,
    Channel6 = 0x40,
    Channel7 = 0x80
};

constexpr uint8_t TCA9546A_ADDRESS = 0x70;
constexpr uint8_t TCA9548A_ADDRESS = 0x70;

template <typename Transport, typename ResetPin>
    requires StreamAccessTransport<Transport>
class I2CSwitch
{
public:
    I2CSwitch() = delete;

    explicit I2CSwitch(const Transport& transport)
        : transport_(transport), reset_{} {}

    I2CSwitch(const Transport& transport, const ResetPin& reset)
    : transport_(transport), reset_(reset) {}

    bool select(I2CSwitchChannel channel)
    {
        uint8_t command = static_cast<uint8_t>(channel);
        return transport_.write(&command, 1);
    }

    bool disableAll()
    {
        uint8_t command = 0x00;
        return transport_.write(&command, 1);
    }

    void holdReset()
    {
        reset_.low();
    }

    void releaseReset()
    {
        reset_.high();
    }

    bool status(uint8_t& out)
    {
        return transport_.read(&out, 1);
    }

private:
    const Transport& transport_;
    ResetPin reset_;
};

#endif // _I2C_SWITCH_HPP_
