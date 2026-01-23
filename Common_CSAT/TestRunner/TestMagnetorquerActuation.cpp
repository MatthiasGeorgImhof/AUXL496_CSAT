#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "MagnetorquerHardwareInterface.hpp"
#include "mock_hal.h"

// Shared mock GPIO ports
GPIO_TypeDef GPIOA{}, GPIOB{}, GPIOC{};

// Shared mock timers
TIM_HandleTypeDef htim_x{}, htim_y{}, htim_z{};

TEST_CASE("MagnetorquerHardwareInterface: applyPWM sets correct compare values")
{
    reset_timer_state(&htim_x);
    reset_timer_state(&htim_y);
    reset_timer_state(&htim_z);

    MagnetorquerHardwareInterface::ChannelMap map{
        {&htim_x, TIM_CHANNEL_1, 255},
        {&htim_y, TIM_CHANNEL_1, 255},
        {&htim_z, TIM_CHANNEL_1, 255}};

    MagnetorquerHardwareInterface hw(map);

    PWMCommand pwm{0.5f, -0.25f, 1.0f};
    hw.applyPWM(pwm);

    CHECK(is_pwm_started(&htim_x, TIM_CHANNEL_1));
    CHECK(is_pwm_started(&htim_y, TIM_CHANNEL_1));
    CHECK(is_pwm_started(&htim_z, TIM_CHANNEL_1));

    CHECK(get_compare_value(&htim_x, TIM_CHANNEL_1) == doctest::Approx(128));
    CHECK(get_compare_value(&htim_y, TIM_CHANNEL_1) == doctest::Approx(64));
    CHECK(get_compare_value(&htim_z, TIM_CHANNEL_1) == doctest::Approx(255));
}

TEST_CASE("MagnetorquerHardwareInterface: stopAll sets compare to zero")
{
    reset_timer_state(&htim_x);
    reset_timer_state(&htim_y);
    reset_timer_state(&htim_z);

    MagnetorquerHardwareInterface::ChannelMap map{
        {&htim_x, TIM_CHANNEL_1, 255},
        {&htim_y, TIM_CHANNEL_1, 255},
        {&htim_z, TIM_CHANNEL_1, 255}};

    MagnetorquerHardwareInterface hw(map);
    hw.stopAll();

    CHECK(get_compare_value(&htim_x, TIM_CHANNEL_1) == 0);
    CHECK(get_compare_value(&htim_y, TIM_CHANNEL_1) == 0);
    CHECK(get_compare_value(&htim_z, TIM_CHANNEL_1) == 0);
}

TEST_CASE("MagnetorquerHardwareInterface: disableAll stops PWM")
{
    reset_timer_state(&htim_x);
    reset_timer_state(&htim_y);
    reset_timer_state(&htim_z);

    MagnetorquerHardwareInterface::ChannelMap map{
        {&htim_x, TIM_CHANNEL_1, 255},
        {&htim_y, TIM_CHANNEL_1, 255},
        {&htim_z, TIM_CHANNEL_1, 255}};

    MagnetorquerHardwareInterface hw(map);
    hw.disableAll();

    CHECK(is_pwm_started(&htim_x, TIM_CHANNEL_1) == false);
    CHECK(is_pwm_started(&htim_y, TIM_CHANNEL_1) == false);
    CHECK(is_pwm_started(&htim_z, TIM_CHANNEL_1) == false);
}

TEST_CASE("MagnetorquerPolarityController: polarity and enable logic")
{
    reset_gpio_port_state(&GPIOA);
    reset_gpio_port_state(&GPIOB);
    reset_gpio_port_state(&GPIOC);

    MagnetorquerPolarityController::PinMap pins = {
        .x = MagnetorquerPolarityController::AxisPins{&GPIOA, GPIO_PIN_0, &GPIOA, GPIO_PIN_1},
        .y = MagnetorquerPolarityController::AxisPins{&GPIOB, GPIO_PIN_2, &GPIOB, GPIO_PIN_3},
        .z = MagnetorquerPolarityController::AxisPins{&GPIOC, GPIO_PIN_4, &GPIOC, GPIO_PIN_5}};

    MagnetorquerPolarityController ctrl(pins);

    SUBCASE("Positive duty sets polarity HIGH and enable LOW")
    {
        ctrl.applyPolarityAndEnable(0.5f, 0.25f, 0.1f);

        CHECK(get_gpio_pin_state(&GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET); // enable
        CHECK(get_gpio_pin_state(&GPIOA, GPIO_PIN_1) == GPIO_PIN_SET);   // polarity

        CHECK(get_gpio_pin_state(&GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET);
        CHECK(get_gpio_pin_state(&GPIOB, GPIO_PIN_3) == GPIO_PIN_SET);

        CHECK(get_gpio_pin_state(&GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET);
        CHECK(get_gpio_pin_state(&GPIOC, GPIO_PIN_5) == GPIO_PIN_SET);
    }

    SUBCASE("Negative duty sets polarity LOW")
    {
        ctrl.applyPolarityAndEnable(-0.5f, -0.25f, -0.1f);

        CHECK(get_gpio_pin_state(&GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET);
        CHECK(get_gpio_pin_state(&GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET);
        CHECK(get_gpio_pin_state(&GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET);
    }

    SUBCASE("disableAll sets enable HIGH")
    {
        ctrl.disableAll();

        CHECK(get_gpio_pin_state(&GPIOA, GPIO_PIN_0) == GPIO_PIN_SET);
        CHECK(get_gpio_pin_state(&GPIOB, GPIO_PIN_2) == GPIO_PIN_SET);
        CHECK(get_gpio_pin_state(&GPIOC, GPIO_PIN_4) == GPIO_PIN_SET);
    }
}

TEST_CASE("MagnetorquerActuator: apply sets PWM and polarity")
{
    reset_timer_state(&htim_x);
    reset_timer_state(&htim_y);
    reset_timer_state(&htim_z);
    reset_gpio_port_state(&GPIOA);
    reset_gpio_port_state(&GPIOB);
    reset_gpio_port_state(&GPIOC);

    MagnetorquerHardwareInterface::ChannelMap pwm_map{
        {&htim_x, TIM_CHANNEL_1, 255},
        {&htim_y, TIM_CHANNEL_1, 255},
        {&htim_z, TIM_CHANNEL_1, 255}};

    MagnetorquerPolarityController::PinMap gpio_map = {
        .x = MagnetorquerPolarityController::AxisPins{&GPIOA, GPIO_PIN_0, &GPIOA, GPIO_PIN_1},
        .y = MagnetorquerPolarityController::AxisPins{&GPIOB, GPIO_PIN_2, &GPIOB, GPIO_PIN_3},
        .z = MagnetorquerPolarityController::AxisPins{&GPIOC, GPIO_PIN_4, &GPIOC, GPIO_PIN_5}};

    MagnetorquerActuator actuator(pwm_map, gpio_map);
    PWMCommand cmd{0.5f, -0.25f, 1.0f};
    actuator.apply(cmd);

    // PWM checks
    CHECK(get_compare_value(&htim_x, TIM_CHANNEL_1) == doctest::Approx(128));
    CHECK(get_compare_value(&htim_y, TIM_CHANNEL_1) == doctest::Approx(64));
    CHECK(get_compare_value(&htim_z, TIM_CHANNEL_1) == doctest::Approx(255));

    // Polarity checks
    CHECK(get_gpio_pin_state(&GPIOA, GPIO_PIN_1) == GPIO_PIN_SET);   // X polarity
    CHECK(get_gpio_pin_state(&GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET); // Y polarity
    CHECK(get_gpio_pin_state(&GPIOC, GPIO_PIN_5) == GPIO_PIN_SET);   // Z polarity

    // Enable checks
    CHECK(get_gpio_pin_state(&GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET);
    CHECK(get_gpio_pin_state(&GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET);
    CHECK(get_gpio_pin_state(&GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET);
}

TEST_CASE("MagnetorquerActuator: disableAll disables PWM and GPIO")
{
    reset_timer_state(&htim_x);
    reset_timer_state(&htim_y);
    reset_timer_state(&htim_z);
    reset_gpio_port_state(&GPIOA);
    reset_gpio_port_state(&GPIOB);
    reset_gpio_port_state(&GPIOC);

    MagnetorquerHardwareInterface::ChannelMap pwm_map{
        {&htim_x, TIM_CHANNEL_1, 255},
        {&htim_y, TIM_CHANNEL_1, 255},
        {&htim_z, TIM_CHANNEL_1, 255}};

    MagnetorquerPolarityController::PinMap gpio_map = {
        .x = MagnetorquerPolarityController::AxisPins{&GPIOA, GPIO_PIN_0, &GPIOA, GPIO_PIN_1},
        .y = MagnetorquerPolarityController::AxisPins{&GPIOB, GPIO_PIN_2, &GPIOB, GPIO_PIN_3},
        .z = MagnetorquerPolarityController::AxisPins{&GPIOC, GPIO_PIN_4, &GPIOC, GPIO_PIN_5}};

    MagnetorquerActuator actuator(pwm_map, gpio_map);
    actuator.disableAll();

    CHECK(is_pwm_started(&htim_x, TIM_CHANNEL_1) == false);
    CHECK(is_pwm_started(&htim_y, TIM_CHANNEL_1) == false);
    CHECK(is_pwm_started(&htim_z, TIM_CHANNEL_1) == false);

    CHECK(get_gpio_pin_state(&GPIOA, GPIO_PIN_0) == GPIO_PIN_SET);
    CHECK(get_gpio_pin_state(&GPIOB, GPIO_PIN_2) == GPIO_PIN_SET);
    CHECK(get_gpio_pin_state(&GPIOC, GPIO_PIN_4) == GPIO_PIN_SET);
}

TEST_CASE("MagnetorquerActuator: apply with zero PWM sets compare to zero and polarity LOW") {
    reset_timer_state(&htim_x);
    reset_timer_state(&htim_y);
    reset_timer_state(&htim_z);
    reset_gpio_port_state(&GPIOA);
    reset_gpio_port_state(&GPIOB);
    reset_gpio_port_state(&GPIOC);

    MagnetorquerHardwareInterface::ChannelMap pwm_map{
        {&htim_x, TIM_CHANNEL_1, 255},
        {&htim_y, TIM_CHANNEL_1, 255},
        {&htim_z, TIM_CHANNEL_1, 255}};

    MagnetorquerPolarityController::PinMap gpio_map = {
        .x = MagnetorquerPolarityController::AxisPins{&GPIOA, GPIO_PIN_0, &GPIOA, GPIO_PIN_1},
        .y = MagnetorquerPolarityController::AxisPins{&GPIOB, GPIO_PIN_2, &GPIOB, GPIO_PIN_3},
        .z = MagnetorquerPolarityController::AxisPins{&GPIOC, GPIO_PIN_4, &GPIOC, GPIO_PIN_5}};

    MagnetorquerActuator actuator(pwm_map, gpio_map);
    PWMCommand cmd{0.0f, 0.0f, 0.0f};
    actuator.apply(cmd);

    // PWM checks
    CHECK(get_compare_value(&htim_x, TIM_CHANNEL_1) == 0);
    CHECK(get_compare_value(&htim_y, TIM_CHANNEL_1) == 0);
    CHECK(get_compare_value(&htim_z, TIM_CHANNEL_1) == 0);

    // Polarity checks — conventionally LOW for zero
    CHECK(get_gpio_pin_state(&GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET);
    CHECK(get_gpio_pin_state(&GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET);
    CHECK(get_gpio_pin_state(&GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET);

    // Enable checks — still active
    CHECK(get_gpio_pin_state(&GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET);
    CHECK(get_gpio_pin_state(&GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET);
    CHECK(get_gpio_pin_state(&GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET);
}
