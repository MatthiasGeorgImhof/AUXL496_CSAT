#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "MagneticBDotController.hpp"
#include <Eigen/Core>

// Mock actuator that captures the last PWMCommand
class MockActuator {
public:
    void apply(const PWMCommand& cmd) {
        last_pwm = cmd;
        applied = true;
    }

    void stopAll() { stopped = true; }
    void disableAll() { disabled = true; }

    PWMCommand last_pwm{};
    bool applied = false;
    bool stopped = false;
    bool disabled = false;
};

// DetumblerSystem with injectable actuator for testing
class TestableDetumblerSystem {
public:
    struct Config {
        float bdot_gain;
        MagnetorquerDriver::Config driver_config;
    };

    explicit TestableDetumblerSystem(const Config& cfg, MockActuator& mock)
        : bdot(cfg.bdot_gain),
          driver(cfg.driver_config),
          actuator(mock) {}

    void apply(const MagneticField& B_body, au::QuantityU64<au::Milli<au::Seconds>> timestamp) {
        DipoleMoment m_cmd = bdot.computeDipoleMoment(B_body, timestamp);
        PWMCommand pwm = driver.computePWM(m_cmd);
        actuator.apply(pwm);
    }

    void reset() {
        bdot.reset();
    }

    void stopAll() const {
        actuator.stopAll();
    }

    void disableAll() const {
        actuator.disableAll();
    }

private:
    BDotController bdot;
    MagnetorquerDriver driver;
    MockActuator& actuator;
};

TEST_CASE("DetumblerSystem: integration with mock actuator") {
    MockActuator mock;
    TestableDetumblerSystem::Config config{
        .bdot_gain = 1e4f,
        .driver_config = {0.2f, 0.2f, 0.2f}
    };

    TestableDetumblerSystem system(config, mock);

    MagneticField B1(10e-6f, -5e-6f, 20e-6f);
    MagneticField B2(12e-6f, -4e-6f, 18e-6f);
    auto t0 = au::make_quantity<au::Milli<au::Seconds>>(100ULL);
    auto t1 = au::make_quantity<au::Milli<au::Seconds>>(200ULL);

    SUBCASE("First apply initializes and returns zero PWM") {
        system.apply(B1, t0);
        CHECK(mock.applied);
        CHECK(doctest::Approx(mock.last_pwm.duty_x) == 0.0f);
        CHECK(doctest::Approx(mock.last_pwm.duty_y) == 0.0f);
        CHECK(doctest::Approx(mock.last_pwm.duty_z) == 0.0f);
    }

    SUBCASE("Second apply produces nonzero PWM") {
        system.apply(B1, t0); // initialize
        system.apply(B2, t1); // actual control
        CHECK(mock.applied);
        CHECK(std::abs(mock.last_pwm.duty_x) > 0.0f);
    }

    SUBCASE("Reset clears state and returns zero again") {
        system.apply(B1, t0);
        system.reset();
        system.apply(B2, t1);
        CHECK(mock.applied);
        CHECK(doctest::Approx(mock.last_pwm.duty_x) == 0.0f);
    }

    SUBCASE("Stop and disable flags are set") {
        system.stopAll();
        system.disableAll();
        CHECK(mock.stopped);
        CHECK(mock.disabled);
    }
}
