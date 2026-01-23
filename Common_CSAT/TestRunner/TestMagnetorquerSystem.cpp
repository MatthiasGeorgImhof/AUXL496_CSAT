#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "MagnetorquerHardwareInterface.hpp"

constexpr float TOL = 1e-6f;

TEST_CASE("MagnetorquerSystem: nominal control pipeline") {
    MagnetorquerSystem::Config config{
        AttitudeController(0.5f, 0.1f),
        MagnetorquerDriver({0.2f, 0.2f, 0.2f}),
        {/* mock PWM channels */},
        {/* mock GPIO pins */}
    };

    MagnetorquerSystem system(config);

    Eigen::Quaternionf q_current = Eigen::Quaternionf::Identity();
    Eigen::Quaternionf q_desired(Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitZ()));
    AngularVelocity omega_measured{0.01f, 0.02f, 0.03f};
    MagneticField B_body{0.2f, -0.1f, 0.05f};

    SUBCASE("Apply generates finite PWM") {
        PWMCommand pwm = config.driver.computePWM(
            MagnetorquerController::computeDipoleMoment(
                config.controller.computeOmegaCommand(
                    AttitudeError::rotationVector(
                        AttitudeError::computeQuaternionError(q_desired, q_current)),
                    omega_measured),
                B_body));

        CHECK(std::isfinite(pwm.duty_x));
        CHECK(std::isfinite(pwm.duty_y));
        CHECK(std::isfinite(pwm.duty_z));
    }

    SUBCASE("Apply handles zero field gracefully") {
        MagneticField B_zero = MagneticField::Zero();
        DipoleMoment m_zero = MagnetorquerController::computeDipoleMoment(
            AngularRotation{0.01f, 0.02f, 0.03f}, B_zero);
        CHECK(m_zero.isZero(TOL));
    }
}
