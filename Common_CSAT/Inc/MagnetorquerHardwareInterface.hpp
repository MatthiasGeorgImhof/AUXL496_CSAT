// MagnetorquerHardwareInterface.hpp

#pragma once
#include "LVLHAttitudeTarget.hpp"
#include "MagnetorquerDriver.hpp"
#include "Logger.hpp"

#ifdef __arm__
#include "stm32l4xx_hal.h"
#else
#include "mock_hal.h"
#endif

class MagnetorquerHardwareInterface {
public:
    struct Channel {
        TIM_HandleTypeDef* htim;
        uint32_t channel;
        uint32_t arr;
    };

    struct ChannelMap {
        Channel x;
        Channel y;
        Channel z;
    };

    explicit MagnetorquerHardwareInterface(const ChannelMap& map) : channels(map) {}

    void applyPWM(const PWMCommand& pwm) const {
        setDutyCycle(channels.x, pwm.duty_x);
        setDutyCycle(channels.y, pwm.duty_y);
        setDutyCycle(channels.z, pwm.duty_z);
    }

    void stopAll() const {
        setDutyCycle(channels.x, 0.0f);
        setDutyCycle(channels.y, 0.0f);
        setDutyCycle(channels.z, 0.0f);
    }

    void disableAll() const {
        disable(channels.x);
        disable(channels.y);
        disable(channels.z);
    }

private:
    ChannelMap channels;

    void setDutyCycle(const Channel& ch, float duty) const {
        float clamped = std::clamp(duty, -1.0f, 1.0f);
        uint32_t pwm_value = static_cast<uint32_t>(std::round(std::abs(clamped) * static_cast<float>(ch.arr)));

        HAL_TIM_PWM_Start(ch.htim, ch.channel);
        __HAL_TIM_SET_COMPARE(ch.htim, ch.channel, pwm_value);
        // printf("MagnetorquerHardwareInterface::setDutyCycle: arr=%u, duty=%.3f → pwm_value=%u\n", ch.arr, duty, pwm_value);
    }

    void disable(const Channel& ch) const {
        HAL_TIM_PWM_Stop(ch.htim, ch.channel);
    }
};


class MagnetorquerPolarityController {
public:
    struct AxisPins {
        GPIO_TypeDef* enable_port;
        uint16_t enable_pin;
        GPIO_TypeDef* polarity_port;
        uint16_t polarity_pin;
    };

    struct PinMap {
        AxisPins x;
        AxisPins y;
        AxisPins z;
    };

    explicit MagnetorquerPolarityController(const PinMap& map) : pins(map) {}

    void applyPolarityAndEnable(float duty_x, float duty_y, float duty_z) const {
        setAxis(pins.x, duty_x);
        setAxis(pins.y, duty_y);
        setAxis(pins.z, duty_z);
    }

    void disableAll() const {
        disableAxis(pins.x);
        disableAxis(pins.y);
        disableAxis(pins.z);
    }

private:
    PinMap pins;

    void setAxis(const AxisPins& axis, float duty) const {
        // Enable = LOW (active)
        HAL_GPIO_WritePin(axis.enable_port, axis.enable_pin, GPIO_PIN_RESET);

        // Polarity: HIGH for positive, LOW for negative
        GPIO_PinState polarity = (duty > 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(axis.polarity_port, axis.polarity_pin, polarity);
    }

    void disableAxis(const AxisPins& axis) const {
        HAL_GPIO_WritePin(axis.enable_port, axis.enable_pin, GPIO_PIN_SET); // Disable = HIGH
    }
};

class MagnetorquerActuator {
public:
    MagnetorquerActuator(const MagnetorquerHardwareInterface::ChannelMap& pwm_map,
                         const MagnetorquerPolarityController::PinMap& gpio_map)
        : pwm(pwm_map), polarity(gpio_map) {}

    void apply(const PWMCommand& cmd) const {
        polarity.applyPolarityAndEnable(cmd.duty_x, cmd.duty_y, cmd.duty_z);
        pwm.applyPWM(cmd);
    }

    void stopAll() const {
        pwm.stopAll();
    }

    void disableAll() const {
        pwm.disableAll();
        polarity.disableAll();
    }

private:
    MagnetorquerHardwareInterface pwm;
    MagnetorquerPolarityController polarity;
};

class MagnetorquerSystem {
public:
    struct Config {
        AttitudeController controller;
        MagnetorquerDriver driver;
        MagnetorquerHardwareInterface::ChannelMap pwm_channels;
        MagnetorquerPolarityController::PinMap gpio_pins;
    };

    explicit MagnetorquerSystem(const Config& cfg)
        : pipeline({cfg.controller, cfg.driver}),
          actuator(cfg.pwm_channels, cfg.gpio_pins) {}

    void apply(const Eigen::Quaternionf& q_current,
               const AngularVelocity& omega_measured,
               const Eigen::Quaternionf& q_desired,
               const MagneticField& B_body) const
    {
        PWMCommand pwm = pipeline.computePWMCommand(q_current, omega_measured, q_desired, B_body);
        // printf("MagnetorquerSystem::apply PWMCommand: duty_x=%.3f, duty_y=%.3f, duty_z=%.3f\n", pwm.duty_x, pwm.duty_y, pwm.duty_z);
        actuator.apply(pwm);
    }

    void stopAll() const {
        actuator.stopAll();
    }

    void disableAll() const {
        actuator.disableAll();
    }

private:
    MagnetorquerControlPipeline pipeline;
    MagnetorquerActuator actuator;
};

