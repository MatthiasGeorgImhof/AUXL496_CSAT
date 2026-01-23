// MagneticBDotController.hpp

#pragma once
#include <Eigen/Core>
#include "NamedVector3f.hpp"
#include "MagnetorquerDriver.hpp"
#include "MagnetorquerHardwareInterface.hpp"

class BDotController {
public:
    explicit BDotController(float gain) : k(gain), last_timestamp(au::make_quantity<au::Milli<au::Seconds>>(0)), initialized(false) {}

    // Call this once per control cycle with current time and field
    DipoleMoment computeDipoleMoment(const MagneticField& B_now, au::QuantityU64<au::Milli<au::Seconds>> timestamp) {
        if (!initialized || timestamp <= last_timestamp) {
            B_prev = B_now;
            last_timestamp = timestamp;
            initialized = true;
            return DipoleMoment::Zero();
        }

        float dt = 0.001f * static_cast<float>((timestamp - last_timestamp).in(au::milli(au::seconds)));
        MagneticField B_dot = (B_now - B_prev) / dt;

        B_prev = B_now;
        last_timestamp = timestamp;

        return DipoleMoment((-k * B_dot).value.eval());
    }

    void reset() {
        initialized = false;
        last_timestamp = au::make_quantity<au::Milli<au::Seconds>>(0);
        B_prev = DipoleMoment::Zero();
    }

private:
    float k;
    au::QuantityU64<au::Milli<au::Seconds>> last_timestamp;
    bool initialized;
    MagneticField B_prev;
};

class DetumblerSystem
{
public:
    struct Config
    {
        float bdot_gain;
        MagnetorquerDriver::Config driver_config;
        MagnetorquerHardwareInterface::ChannelMap pwm_channels;
        MagnetorquerPolarityController::PinMap gpio_pins;
    };

    explicit DetumblerSystem(const Config &cfg)
        : bdot(cfg.bdot_gain),
          driver(cfg.driver_config),
          actuator(cfg.pwm_channels, cfg.gpio_pins) {}

    void apply(const MagneticField &B_body, au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        DipoleMoment m_cmd = bdot.computeDipoleMoment(B_body, timestamp);
        PWMCommand pwm = driver.computePWM(m_cmd);
        actuator.apply(pwm);
    }

    void reset()
    {
        bdot.reset();
    }

    void stopAll() const
    {
        actuator.stopAll();
    }

    void disableAll() const
    {
        actuator.disableAll();
    }

private:
    BDotController bdot;
    MagnetorquerDriver driver;
    MagnetorquerActuator actuator;
};
