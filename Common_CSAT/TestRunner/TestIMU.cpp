#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "IMU.hpp"
#include "au.hpp"

#include <optional>

struct MockIMU
{
    std::optional<std::array<au::QuantityF<au::MetersPerSecondSquaredInBodyFrame>, 3>> readAccelerometer()
    {
        return std::array{
            au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(1.0f),
            au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(0.0f),
            au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(0.0f)};
    }
};
static_assert(HasBodyAccelerometer<MockIMU>);

struct MockOrientationProvider
{
    void predict(std::array<float, 4> &q_body_to_ned, au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
    {
        q_body_to_ned = {0.0f, 0.0f, 0.0f, 1.0f}; // Identity quaternion
        timestamp = au::make_quantity<au::Milli<au::Seconds>>(1000);
    }
};

struct MockPositionProvider
{
    void predict(std::array<au::QuantityF<au::MetersInEcefFrame>, 3> &pos_ecef,
                 std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> &velocity,
                 const au::QuantityU64<au::Milli<au::Seconds>> & /* timestamp */)
    {
        pos_ecef = {
            au::make_quantity<au::MetersInEcefFrame>(6378137.0f), // x
            au::make_quantity<au::MetersInEcefFrame>(0.0f),       // y
            au::make_quantity<au::MetersInEcefFrame>(0.0f)        // z
        };
        velocity = {
            au::make_quantity<au::MetersPerSecondInEcefFrame>(0.0f),
            au::make_quantity<au::MetersPerSecondInEcefFrame>(0.0f),
            au::make_quantity<au::MetersPerSecondInEcefFrame>(0.0f)};
    }
};

static_assert(HasEcefAccelerometer<IMUWithReorientation<MockIMU, MockOrientationProvider, MockPositionProvider>>);
TEST_CASE("IMUWithReorientation: Identity rotation preserves acceleration direction")
{
    MockIMU imu;
    MockOrientationProvider orientation;
    MockPositionProvider position;

    IMUWithReorientation<MockIMU, MockOrientationProvider, MockPositionProvider> imu_reoriented(imu, orientation, position);

    auto result = imu_reoriented.readAccelerometer();
    REQUIRE(result.has_value());

    auto accel_ecef = result.value();
    CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f);  // X
    CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f);  // Y
    CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == -1.0f); // Z
}

struct EmptyIMU
{
    std::optional<std::array<au::QuantityF<au::MetersPerSecondSquaredInBodyFrame>, 3>> readAccelerometer()
    {
        return std::nullopt;
    }
};
static_assert(HasBodyAccelerometer<EmptyIMU>);

static_assert(HasEcefAccelerometer<IMUWithReorientation<EmptyIMU, MockOrientationProvider, MockPositionProvider>>);
TEST_CASE("IMUWithReorientation: Returns nullopt when IMU data is missing")
{
    EmptyIMU imu;
    MockOrientationProvider orientation;
    MockPositionProvider position;

    IMUWithReorientation<EmptyIMU, MockOrientationProvider, MockPositionProvider> imu_reoriented(imu, orientation, position);

    auto result = imu_reoriented.readAccelerometer();
    CHECK_FALSE(result.has_value());
}

struct MockMagnetometer
{
    std::optional<MagneticFieldInBodyFrame> readMagnetometer()
    {
        return MagneticFieldInBodyFrame{
            au::make_quantity<au::TeslaInBodyFrame>(1.0f),
            au::make_quantity<au::TeslaInBodyFrame>(0.0f),
            au::make_quantity<au::TeslaInBodyFrame>(0.0f)};
    }
};
static_assert(HasBodyMagnetometer<MockMagnetometer>);

static_assert(HasBodyMagnetometer<IMUWithMagneticCorrection<MockMagnetometer>>);
TEST_CASE("IMUWithMagneticCorrection applies hard and soft iron correction")
{
    MockMagnetometer mock;

    Eigen::Vector3f hardIron(0.5f, 0.0f, 0.0f); // subtract 0.5 from x
    Eigen::Matrix3f softIron;
    softIron << 2.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f;

    IMUWithMagneticCorrection<MockMagnetometer> corrected(mock, hardIron, softIron);

    auto result = mock.readMagnetometer();
    REQUIRE(result.has_value());

    const auto &mag = result.value();
    CHECK(mag[0].in(au::teslaInBodyFrame) == doctest::Approx(1.0f)); // (1.0 - 0.5) * 2.0 = 1.0
    CHECK(mag[1].in(au::teslaInBodyFrame) == doctest::Approx(0.0f));
    CHECK(mag[2].in(au::teslaInBodyFrame) == doctest::Approx(0.0f));
}

struct EmptyMagnetometer
{
    std::optional<MagneticFieldInBodyFrame> readMagnetometer()
    {
        return std::nullopt;
    }
};
static_assert(HasBodyMagnetometer<EmptyMagnetometer>);

static_assert(HasBodyMagnetometer<IMUWithMagneticCorrection<EmptyMagnetometer>>);
TEST_CASE("IMUWithMagneticCorrection handles missing magnetometer data")
{

    Eigen::Vector3f hardIron(0.0f, 0.0f, 0.0f);
    Eigen::Matrix3f softIron = Eigen::Matrix3f::Identity();

    EmptyMagnetometer empty;
    IMUWithMagneticCorrection<EmptyMagnetometer> corrected(empty, hardIron, softIron);
    auto result = corrected.readMagnetometer();
    CHECK_FALSE(result.has_value());
}
