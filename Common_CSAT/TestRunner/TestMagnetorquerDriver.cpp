#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "MagnetorquerDriver.hpp"
#include "LVLHAttitudeTarget.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

constexpr float TOL = 1e-4f;

TEST_CASE("MagnetorquerDriver: computePWM")
{
    MagnetorquerDriver::Config config{0.05f, 0.10f, 0.20f}; // A·m²
    MagnetorquerDriver driver(config);

    SUBCASE("Zero dipole")
    {
        Eigen::Vector3f m_cmd(0.0f, 0.0f, 0.0f);
        auto pwm = driver.computePWM(m_cmd);

        CHECK(std::abs(pwm.duty_x) < TOL);
        CHECK(std::abs(pwm.duty_y) < TOL);
        CHECK(std::abs(pwm.duty_z) < TOL);
    }

    SUBCASE("Nominal dipole within bounds")
    {
        Eigen::Vector3f m_cmd(0.025f, -0.05f, 0.10f);
        auto pwm = driver.computePWM(m_cmd);

        CHECK(std::abs(pwm.duty_x - 0.5f) < TOL);
        CHECK(std::abs(pwm.duty_y + 0.5f) < TOL);
        CHECK(std::abs(pwm.duty_z - 0.5f) < TOL);
    }

    SUBCASE("Saturation clamps to ±1.0")
    {
        Eigen::Vector3f m_cmd(0.10f, -0.20f, 0.50f); // All exceed limits
        auto pwm = driver.computePWM(m_cmd);

        CHECK(std::abs(pwm.duty_x - 1.0f) < TOL);
        CHECK(std::abs(pwm.duty_y + 1.0f) < TOL);
        CHECK(std::abs(pwm.duty_z - 1.0f) < TOL);
    }

    SUBCASE("Zero dipole yields zero duty")
    {
        Eigen::Vector3f m_cmd = Eigen::Vector3f::Zero();
        auto pwm = driver.computePWM(m_cmd);

        CHECK(std::abs(pwm.duty_x) < TOL);
        CHECK(std::abs(pwm.duty_y) < TOL);
        CHECK(std::abs(pwm.duty_z) < TOL);
    }

    SUBCASE("Polarity is preserved")
    {
        Eigen::Vector3f m_cmd_pos(0.01f, 0.01f, 0.01f);
        Eigen::Vector3f m_cmd_neg(-0.01f, -0.01f, -0.01f);

        auto pwm_pos = driver.computePWM(m_cmd_pos);
        auto pwm_neg = driver.computePWM(m_cmd_neg);

        CHECK(pwm_pos.duty_x > 0.0f);
        CHECK(pwm_neg.duty_x < 0.0f);
        CHECK(pwm_pos.duty_y > 0.0f);
        CHECK(pwm_neg.duty_y < 0.0f);
        CHECK(pwm_pos.duty_z > 0.0f);
        CHECK(pwm_neg.duty_z < 0.0f);
    }

    SUBCASE("Asymmetric config scales independently")
    {
        Eigen::Vector3f m_cmd(0.05f, 0.10f, 0.20f); // All at max
        auto pwm = driver.computePWM(m_cmd);

        CHECK(std::abs(pwm.duty_x - 1.0f) < TOL);
        CHECK(std::abs(pwm.duty_y - 1.0f) < TOL);
        CHECK(std::abs(pwm.duty_z - 1.0f) < TOL);
    }
}

TEST_CASE("MagnetorquerControlPipeline: computes correct PWMCommand")
{
    // Setup controller and driver
    AttitudeController controller(0.1f, 0.1f);
    MagnetorquerDriver::Config driver_cfg{0.2f, 0.2f, 0.2f}; // max dipole moments
    MagnetorquerDriver driver(driver_cfg);

    MagnetorquerControlPipeline pipeline({controller, driver});

    // Define test inputs
    Eigen::Quaternionf q_desired = Eigen::Quaternionf::Identity();
    Eigen::Quaternionf q_current(Eigen::AngleAxisf(1.0f, Eigen::Vector3f::UnitY())); // ~57°
    AngularVelocity omega_measured(0.05f, -0.02f, 0.01f);                            // rad/s

    SUBCASE("B-body along X")
    {
        MagneticField B_body{40e-6f, 0.0f, 0.0f}; // field along X in Tesla

        PWMCommand pwm = pipeline.computePWMCommand(q_current, omega_measured, q_desired, B_body);

        // Check that PWMCommand is within valid range
        CHECK(pwm.duty_x >= -1.0f);
        CHECK(pwm.duty_x <= 1.0f);
        CHECK(pwm.duty_y >= -1.0f);
        CHECK(pwm.duty_y <= 1.0f);
        CHECK(pwm.duty_z >= -1.0f);
        CHECK(pwm.duty_z <= 1.0f);

        // Optional: check sign consistency
        CHECK(pwm.duty_x == doctest::Approx(0.0));
        CHECK(pwm.duty_y == doctest::Approx(-0.005f));
        CHECK(pwm.duty_z == doctest::Approx(-0.249713f));
    }

    SUBCASE("B-body along Y")
    {
        MagneticField B_body{0.0f, 40e-6f, 0.0f}; // field along Y in Tesla

        PWMCommand pwm = pipeline.computePWMCommand(q_current, omega_measured, q_desired, B_body);

        // Check that PWMCommand is within valid range
        CHECK(pwm.duty_x >= -1.0f);
        CHECK(pwm.duty_x <= 1.0f);
        CHECK(pwm.duty_y >= -1.0f);
        CHECK(pwm.duty_y <= 1.0f);
        CHECK(pwm.duty_z >= -1.0f);
        CHECK(pwm.duty_z <= 1.0f);

        // Optional: check sign consistency
        CHECK(pwm.duty_x == doctest::Approx(0.005));
        CHECK(pwm.duty_y == doctest::Approx(0.0f));
        CHECK(pwm.duty_z == doctest::Approx(-0.025f));
    }

    SUBCASE("B-body along Z")
    {
        MagneticField B_body{0.0f, 0.0, 40e-6f}; // field along Z in Tesla

        PWMCommand pwm = pipeline.computePWMCommand(q_current, omega_measured, q_desired, B_body);

        // Check that PWMCommand is within valid range
        CHECK(pwm.duty_x >= -1.0f);
        CHECK(pwm.duty_x <= 1.0f);
        CHECK(pwm.duty_y >= -1.0f);
        CHECK(pwm.duty_y <= 1.0f);
        CHECK(pwm.duty_z >= -1.0f);
        CHECK(pwm.duty_z <= 1.0f);

        // Optional: check sign consistency
        CHECK(pwm.duty_x == doctest::Approx(0.249713f));
        CHECK(pwm.duty_y == doctest::Approx(0.025f));
        CHECK(pwm.duty_z == doctest::Approx(0.0f));
    }

    SUBCASE("B-body along XYZ")
    {
        MagneticField B_body{35e-6f, 35e-6f, 35e-6f}; // field along XYZ in Tesla

        PWMCommand pwm = pipeline.computePWMCommand(q_current, omega_measured, q_desired, B_body);

        // Check that PWMCommand is within valid range
        CHECK(pwm.duty_x >= -1.0f);
        CHECK(pwm.duty_x <= 1.0f);
        CHECK(pwm.duty_y >= -1.0f);
        CHECK(pwm.duty_y <= 1.0f);
        CHECK(pwm.duty_z >= -1.0f);
        CHECK(pwm.duty_z <= 1.0f);

        // Optional: check sign consistency
        CHECK(pwm.duty_x == doctest::Approx(0.147059f));
        CHECK(pwm.duty_y == doctest::Approx(0.011547f));
        CHECK(pwm.duty_z == doctest::Approx(-0.158606f));
    }
}

TEST_CASE("MagnetorquerControlPipeline: proportionality check")
{
    // Setup controller and driver
    AttitudeController controller(0.1f, 0.1f);
    MagnetorquerDriver::Config driver_cfg{0.2f, 0.2f, 0.2f}; // max dipole moments
    MagnetorquerDriver driver(driver_cfg);

    MagnetorquerControlPipeline pipeline({controller, driver});

    // Define test inputs
    Eigen::Quaternionf q_desired = Eigen::Quaternionf::Identity();
    Eigen::Quaternionf q_current(Eigen::AngleAxisf(0.05f, Eigen::Vector3f::UnitY()));
    AngularVelocity omega_measured(0.05f, -0.02f, 0.01f); // rad/s

    SUBCASE("B-body along X")
    {
        MagneticField B_body{40e-6f, 0.0f, 0.0f}; // field along X in Tesla
        PWMCommand pwm = pipeline.computePWMCommand(q_current, omega_measured, q_desired, B_body);

        CHECK(pwm.duty_x == doctest::Approx(0.0));
        CHECK(pwm.duty_y < 0.0f);
        CHECK(pwm.duty_z < 0.0f);
    }

    SUBCASE("B-body along XY")
    {
        MagneticField B_body{40e-6f, 40e-6f, 0.0f}; // field along XY in Tesla
        PWMCommand pwm = pipeline.computePWMCommand(q_current, omega_measured, q_desired, B_body);

        CHECK(pwm.duty_x > 0.0f);
        CHECK(pwm.duty_y < 0.0f);
        CHECK(pwm.duty_z < 0.0f);
    }
}
