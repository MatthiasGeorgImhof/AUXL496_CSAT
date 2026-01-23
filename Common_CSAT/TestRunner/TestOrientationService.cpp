#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "OrientationTracker.hpp"
#include "OrientationService.hpp"
#include "mock_hal.h"

constexpr float m_pif = static_cast<float>(std::numbers::pi);

// Mock IMU class
class MockIMUinBodyFrame
{
public:
    MockIMUinBodyFrame() : has_acc_data(false), has_gyro_data(false), has_mag_data(false) {}

    void setAcceleration(float x, float y, float z)
    {
        acceleration= { au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(x), 
                        au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(y),
                        au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(z)};
        has_acc_data = true;
    }

    std::optional<AccelerationInBodyFrame> readAccelerometer()
    {
        if (has_acc_data)
        {
            return acceleration;
        }
        else
        {
            return std::nullopt;
        }
    }

    void setGyroscope(float x, float y, float z)
    {
        gyroscope = { au::make_quantity<au::DegreesPerSecondInBodyFrame>(x),
                      au::make_quantity<au::DegreesPerSecondInBodyFrame>(y),
                      au::make_quantity<au::DegreesPerSecondInBodyFrame>(z)};
        has_gyro_data = true;
    }

    std::optional<AngularVelocityInBodyFrame> readGyroscope()
    {
        if (has_gyro_data)
        {
            return gyroscope;
        }
        else
        {
            return std::nullopt;
        }
    }

    void setMagnetometer(float x, float y, float z)
    {
        magnetometer = { au::make_quantity<au::TeslaInBodyFrame>(x),
                         au::make_quantity<au::TeslaInBodyFrame>(y),
                         au::make_quantity<au::TeslaInBodyFrame>(z)};
        has_mag_data = true;
    }
    
    std::optional<MagneticFieldInBodyFrame> readMagnetometer()
    {
        if (has_mag_data)
        {
            return magnetometer;
        }
        else
        {
            return std::nullopt;
        }
    }

private:
    AccelerationInBodyFrame acceleration;
    AngularVelocityInBodyFrame gyroscope;
    MagneticFieldInBodyFrame magnetometer;
    bool has_acc_data, has_gyro_data, has_mag_data;
};
static_assert(HasBodyAccelerometer<MockIMUinBodyFrame>);
static_assert(HasBodyGyroscope<MockIMUinBodyFrame>);
static_assert(HasBodyMagnetometer<MockIMUinBodyFrame>);

TEST_CASE("GyrMagOrientation predicts identity quaternion with static inputs")
{
    GyrMagOrientationTracker tracker;
    tracker.setReferenceVectors(Eigen::Vector3f(1.f, 0.f, 0.f));

    RTC_HandleTypeDef rtc{};
    rtc.Init.SynchPrediv = 255;

    set_mocked_rtc_time({12, 0, 0, RTC_HOURFORMAT12_AM, 0, 255, RTC_DAYLIGHTSAVING_NONE, RTC_STOREOPERATION_RESET});
    set_mocked_rtc_date({RTC_WEEKDAY_MONDAY, 1, 1, 24});

    MockIMUinBodyFrame imu;

    GyrMagOrientation<GyrMagOrientationTracker<7,3>, MockIMUinBodyFrame, MockIMUinBodyFrame> service(&rtc, tracker, imu, imu);

    std::array<float, 4> q{};
    au::QuantityU64<au::Milli<au::Seconds>> timestamp;
    bool success = service.predict(q, timestamp);

    REQUIRE(success);
    REQUIRE(std::abs(q[0] - 1.f) < 1e-3f); // w component of identity quaternion
    REQUIRE(std::abs(q[1]) < 1e-3f);
    REQUIRE(std::abs(q[2]) < 1e-3f);
    REQUIRE(std::abs(q[3]) < 1e-3f);
}

TEST_CASE("AccGyrMagOrientation predicts identity quaternion with static inputs")
{
    AccGyrMagOrientationTracker tracker;
    tracker.setReferenceVectors(Eigen::Vector3f(0.f, 0.f, 9.81f), Eigen::Vector3f(1.f, 0.f, 0.f));

    RTC_HandleTypeDef rtc{};
    rtc.Init.SynchPrediv = 255;

    set_mocked_rtc_time({12, 0, 0, RTC_HOURFORMAT12_AM, 0, 255, RTC_DAYLIGHTSAVING_NONE, RTC_STOREOPERATION_RESET});
    set_mocked_rtc_date({RTC_WEEKDAY_MONDAY, 1, 1, 24});

    MockIMUinBodyFrame imu;

    AccGyrMagOrientation<AccGyrMagOrientationTracker<7,6>, MockIMUinBodyFrame, MockIMUinBodyFrame> service(&rtc, tracker, imu, imu);

    std::array<float, 4> q{};
    au::QuantityU64<au::Milli<au::Seconds>> timestamp;
    bool success = service.predict(q, timestamp);

    REQUIRE(success);
    REQUIRE(std::abs(q[0] - 1.f) < 1e-3f);
}

TEST_CASE("AccGyrOrientation predicts identity quaternion with static inputs")
{
    AccGyrOrientationTracker tracker;
    tracker.setReferenceVectors(Eigen::Vector3f(0.f, 0.f, 9.81f));

    RTC_HandleTypeDef rtc{};
    rtc.Init.SynchPrediv = 255;

    set_mocked_rtc_time({12, 0, 0, RTC_HOURFORMAT12_AM, 0, 255, RTC_DAYLIGHTSAVING_NONE, RTC_STOREOPERATION_RESET});
    set_mocked_rtc_date({RTC_WEEKDAY_MONDAY, 1, 1, 24});

    MockIMUinBodyFrame imu;

    AccGyrOrientation<AccGyrOrientationTracker<7,3>, MockIMUinBodyFrame> service(&rtc, tracker, imu);

    std::array<float, 4> q{};
    au::QuantityU64<au::Milli<au::Seconds>> timestamp;
    bool success = service.predict(q, timestamp);

    REQUIRE(success);
    REQUIRE(std::abs(q[0] - 1.f) < 1e-3f);
}

TEST_CASE("GyrMagOrientation returns valid OrientationSolution with static inputs")
{
    GyrMagOrientationTracker tracker;
    tracker.setReferenceVectors(Eigen::Vector3f(1.f, 0.f, 0.f));

    RTC_HandleTypeDef rtc{};
    rtc.Init.SynchPrediv = 255;

    set_mocked_rtc_time({12, 0, 0, RTC_HOURFORMAT12_AM, 0, 255, RTC_DAYLIGHTSAVING_NONE, RTC_STOREOPERATION_RESET});
    set_mocked_rtc_date({RTC_WEEKDAY_MONDAY, 1, 1, 24});

    MockIMUinBodyFrame imu;
    imu.setGyroscope(0.f, 0.f, 0.f);
    imu.setMagnetometer(1.f, 0.f, 0.f);

    GyrMagOrientation<GyrMagOrientationTracker<7,3>, MockIMUinBodyFrame, MockIMUinBodyFrame> service(&rtc, tracker, imu, imu);
    OrientationSolution sol = service.predict();

    REQUIRE(sol.has_valid(OrientationSolution::Validity::QUATERNION));
    REQUIRE(sol.has_valid(OrientationSolution::Validity::ANGULAR_VELOCITY));
    REQUIRE(sol.has_valid(OrientationSolution::Validity::MAGNETIC_FIELD));
    REQUIRE(sol.has_valid(OrientationSolution::Validity::ORIENTATIONS));
    REQUIRE(std::abs(sol.q[0] - 1.f) < 1e-3f);
}

TEST_CASE("AccGyrMagOrientation returns valid OrientationSolution with static inputs")
{
    AccGyrMagOrientationTracker tracker;
    tracker.setReferenceVectors(Eigen::Vector3f(0.f, 0.f, 9.81f), Eigen::Vector3f(1.f, 0.f, 0.f));

    RTC_HandleTypeDef rtc{};
    rtc.Init.SynchPrediv = 255;

    set_mocked_rtc_time({12, 0, 0, RTC_HOURFORMAT12_AM, 0, 255, RTC_DAYLIGHTSAVING_NONE, RTC_STOREOPERATION_RESET});
    set_mocked_rtc_date({RTC_WEEKDAY_MONDAY, 1, 1, 24});

    MockIMUinBodyFrame imu;
    imu.setGyroscope(0.f, 0.f, 0.f);
    imu.setAcceleration(0.f, 0.f, 9.81f);
    imu.setMagnetometer(1.f, 0.f, 0.f);

    AccGyrMagOrientation<AccGyrMagOrientationTracker<7,6>, MockIMUinBodyFrame, MockIMUinBodyFrame> service(&rtc, tracker, imu, imu);
    OrientationSolution sol = service.predict();

    REQUIRE(sol.has_valid(OrientationSolution::Validity::QUATERNION));
    REQUIRE(sol.has_valid(OrientationSolution::Validity::ANGULAR_VELOCITY));
    REQUIRE(sol.has_valid(OrientationSolution::Validity::MAGNETIC_FIELD));
    REQUIRE(sol.has_valid(OrientationSolution::Validity::ORIENTATIONS));
    REQUIRE(std::abs(sol.q[0] - 1.f) < 1e-3f);
}

TEST_CASE("AccGyrOrientation returns valid OrientationSolution with static inputs")
{
    AccGyrOrientationTracker tracker;
    tracker.setReferenceVectors(Eigen::Vector3f(0.f, 0.f, 9.81f));

    RTC_HandleTypeDef rtc{};
    rtc.Init.SynchPrediv = 255;

    set_mocked_rtc_time({12, 0, 0, RTC_HOURFORMAT12_AM, 0, 255, RTC_DAYLIGHTSAVING_NONE, RTC_STOREOPERATION_RESET});
    set_mocked_rtc_date({RTC_WEEKDAY_MONDAY, 1, 1, 24});

    MockIMUinBodyFrame imu;
    imu.setGyroscope(0.f, 0.f, 0.f);
    imu.setAcceleration(0.f, 0.f, 9.81f);

    AccGyrOrientation<AccGyrOrientationTracker<7,3>, MockIMUinBodyFrame> service(&rtc, tracker, imu);
    OrientationSolution sol = service.predict();

    REQUIRE(sol.has_valid(OrientationSolution::Validity::QUATERNION));
    REQUIRE(sol.has_valid(OrientationSolution::Validity::ANGULAR_VELOCITY));
    REQUIRE(sol.has_valid(OrientationSolution::Validity::ORIENTATIONS));
    REQUIRE_FALSE(sol.has_valid(OrientationSolution::Validity::MAGNETIC_FIELD));
    REQUIRE(std::abs(sol.q[0] - 1.f) < 1e-3f);
}
