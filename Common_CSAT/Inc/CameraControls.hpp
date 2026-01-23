#ifndef __CAMERA_CONTROLS_HPP__
#define __CAMERA_CONTROLS_HPP__

#include <cstdint>
#include "GpioPin.hpp"

enum class CamCtrl : uint8_t {
    Clock,
    Reset,
    PowerDown,
};

template <
    typename ClockPin, typename ResetPin, typename PowerDownPin>
class CameraControls
{
public:
    CameraControls()
        : clk_{}, rst_{}, pwdn_{} {}

    void clock_on()  { clk_.high(); }
    void clock_off() { clk_.low(); }

    void reset_assert()   { rst_.low(); }
    void reset_release()  { rst_.high(); }

    void powerdown_on()   { pwdn_.high(); }
    void powerdown_off()  { pwdn_.low(); }

    void bringup()
    {
        clock_on();
        powerdown_off();
        reset_release();
    }

private:
    ClockPin clk_;
    ResetPin rst_;
    PowerDownPin pwdn_;
};




#endif // __CAMERA_CONTROLS_HPP__