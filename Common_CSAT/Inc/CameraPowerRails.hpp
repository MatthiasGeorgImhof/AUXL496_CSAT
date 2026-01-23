#ifndef CAMERA_POWER_RAILS_HPP
#define CAMERA_POWER_RAILS_HPP

#include <cstdint>
#include "GpioPin.hpp"

enum class Rail : uint8_t {
    A,
    B,
    C
};

template <typename RailAPin, typename RailBPin, typename RailCPin>
class CameraPowerRails
{
public:
    CameraPowerRails()
        : railA_{}, railB_{}, railC_{} {}

    void enable(Rail r)
    {
        switch (r)
        {
        case Rail::A: railA_.high(); break;
        case Rail::B: railB_.high(); break;
        case Rail::C: railC_.high(); break;
        }
    }

    void disable(Rail r)
    {
        switch (r)
        {
        case Rail::A: railA_.low(); break;
        case Rail::B: railB_.low(); break;
        case Rail::C: railC_.low(); break;
        }
    }

    void disableAll()
    {
        railA_.low();
        railB_.low();
        railC_.low();
    }

private:
    RailAPin railA_;
    RailBPin railB_;
    RailCPin railC_;
};

#endif // CAMERA_POWER_RAILS_HPP
