#ifndef __CAMERA_POWER_CONVERTERS_HPP__
#define __CAMERA_POWER_CONVERTERS_HPP__

#include "GpioPin.hpp"

template <typename Rail1V8Pin, typename Rail2V8Pin>
class CameraPowerConverters
{
public:
    CameraPowerConverters() : rail1v8_{}, rail2v8_{} {}

    // Turn both rails on together
    void enable()
    {
        rail1v8_.high();
        rail2v8_.high();
    }

    // Turn both rails off together
    void disable()
    {
        rail1v8_.low();
        rail2v8_.low();
    }

private:
    Rail1V8Pin rail1v8_;
    Rail2V8Pin rail2v8_;
};

#endif // __CAMERA_POWER_CONVERTERS_HPP__