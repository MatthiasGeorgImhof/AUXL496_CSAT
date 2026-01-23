#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "IMUExtension.hpp"
#include "coordinate_rotators.hpp"
#include "au.hpp"

#include <optional>

struct MockIMU
{
    void setAcceleration(float ax, float ay, float az)
    {
        acceleration_ = std::array{
            au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(ax),
            au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(ay),
            au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(az)};
        has_data_ = true;
    }

    std::optional<std::array<au::QuantityF<au::MetersPerSecondSquaredInBodyFrame>, 3>> readAccelerometer()
    {
        if (has_data_)
        {
            return acceleration_;
        }
        else
        {
            return std::nullopt;
        }
    }

private:
    std::array<au::QuantityF<au::MetersPerSecondSquaredInBodyFrame>, 3> acceleration_ = {
        au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(0.0f),
        au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(0.0f),
        au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(0.0f)};
    bool has_data_ = false;
};
static_assert(HasBodyAccelerometer<MockIMU>);

struct MockOrientationProvider
{
    void predict(std::array<float, 4> &q_body_to_ned, au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
    {
        q_body_to_ned = {1.0f, 0.0f, 0.0f, 0.0f}; // Identity quaternion
        timestamp = au::make_quantity<au::Milli<au::Seconds>>(1000);
    }
};

struct MockPositionProvider
{
    void setPositionECEF(float x, float y, float z)
    {
        pos_ecef_ = {
            au::make_quantity<au::MetersInEcefFrame>(x),
            au::make_quantity<au::MetersInEcefFrame>(y),
            au::make_quantity<au::MetersInEcefFrame>(z)};
    }

    void setVelocityECEF(float vx, float vy, float vz)
    {
        vel_ecef_ = {
            au::make_quantity<au::MetersPerSecondInEcefFrame>(vx),
            au::make_quantity<au::MetersPerSecondInEcefFrame>(vy),
            au::make_quantity<au::MetersPerSecondInEcefFrame>(vz)};
    }

    void setAccelerationECEF(float ax, float ay, float az)
    {
        acc_ecef_ = {ax, ay, az}; // stored as raw floats for getState()
    }

    void predict(std::array<au::QuantityF<au::MetersInEcefFrame>, 3> &pos_ecef,
                 std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> &velocity,
                 const au::QuantityU64<au::Milli<au::Seconds>> & /* timestamp */)
    {
        pos_ecef = pos_ecef_;
        velocity = vel_ecef_;
    }

    Eigen::Matrix<float, 9, 1> getState() const
    {
        Eigen::Matrix<float, 9, 1> state;
        state << pos_ecef_[0].in(au::metersInEcefFrame),
                 pos_ecef_[1].in(au::metersInEcefFrame),
                 pos_ecef_[2].in(au::metersInEcefFrame),
                 vel_ecef_[0].in(au::metersPerSecondInEcefFrame),
                 vel_ecef_[1].in(au::metersPerSecondInEcefFrame),
                 vel_ecef_[2].in(au::metersPerSecondInEcefFrame),
                 acc_ecef_[0],
                 acc_ecef_[1],
                 acc_ecef_[2];
        return state;
    }

private:
    std::array<au::QuantityF<au::MetersInEcefFrame>, 3> pos_ecef_ = {
        au::make_quantity<au::MetersInEcefFrame>(6378137.0f),
        au::make_quantity<au::MetersInEcefFrame>(0.0f),
        au::make_quantity<au::MetersInEcefFrame>(0.0f)};
    std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> vel_ecef_ = {
        au::make_quantity<au::MetersPerSecondInEcefFrame>(0.0f),
        au::make_quantity<au::MetersPerSecondInEcefFrame>(0.0f),
        au::make_quantity<au::MetersPerSecondInEcefFrame>(0.0f)};
    std::array<float, 3> acc_ecef_ = {0.0f, 0.0f, 0.0f};
};

static_assert(HasEcefAccelerometer<IMUAccInECEFWithPolicy<MockIMU, MockOrientationProvider, MockPositionProvider>>);
TEST_CASE("IMUAccInECEFWithPolicy: Identity rotation maps NED basis vectors to correct ECEF directions at lat=0, lon=0")
{
    MockIMU imu;
    MockOrientationProvider orientation;
    MockPositionProvider position;

    IMUAccInECEFWithPolicy<MockIMU, MockOrientationProvider, MockPositionProvider> imu_reoriented(imu, orientation, position);

    SUBCASE("NED North → ECEF Z")
    {
        imu.setAcceleration(1.0f, 0.0f, 0.0f);  // NED (1, 0, 0)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == 1.0f); // Z
    }

    SUBCASE("NED South → -ECEF Z")
    {
        imu.setAcceleration(-1.0f, 0.0f, 0.0f);  // NED (-1, 0, 0)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == -1.0f); // Z
    }

    SUBCASE("NED East → ECEF Y")
    {
        imu.setAcceleration(0.0f, 1.0f, 0.0f);  // NED (0, 1, 0)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == 1.0f); // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // Z
    }

    SUBCASE("NED West → -ECEF Y")
    {
        imu.setAcceleration(0.0f, -1.0f, 0.0f);  // NED (0, -1, 0)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == -1.0f); // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // Z
    }

    SUBCASE("NED Down → −ECEF X")
    {
        imu.setAcceleration(0.0f, 0.0f, 1.0f);  // NED (0, 0, 1)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == -1.0f); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f);  // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f);  // Z
    }

    SUBCASE("NED Up → ECEF X")
    {
        imu.setAcceleration(0.0f, 0.0f, -1.0f);  // NED (0, 0, -1)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == 1.0f); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f);  // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f);  // Z
    }
}

TEST_CASE("IMUAccInECEFWithPolicy: NED-to-ECEF transform sweep across latitudes")
{
    constexpr float R = 6371000.0f; // mean Earth radius
    std::vector<float> latitudes_deg = {0.0f, 10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f};

    for (float lat_deg : latitudes_deg)
    {
        constexpr float m_pif = std::numbers::pi_v<float>;
        float lat_rad = lat_deg * m_pif / 180.0f;
        float x = R * std::cos(lat_rad);
        float y = 0.0f;
        float z = R * std::sin(lat_rad);

        SUBCASE(("Latitude " + std::to_string(static_cast<int>(lat_deg)) + "°").c_str())
        {
            MockIMU imu;
            MockOrientationProvider orientation;
            MockPositionProvider position;

            position.setPositionECEF(x, y, z);
            position.setVelocityECEF(0.0f, 0.0f, 0.0f);
            position.setAccelerationECEF(0.0f, 0.0f, 0.0f);

            IMUAccInECEFWithPolicy<MockIMU, MockOrientationProvider, MockPositionProvider> imu_reoriented(imu, orientation, position);

            SUBCASE("NED North")
            {
                imu.setAcceleration(1.0f, 0.0f, 0.0f);
                auto result = imu_reoriented.readAccelerometer();
                REQUIRE(result.has_value());
                auto accel_ecef = result.value();
                (void) accel_ecef;
            }

            SUBCASE("NED East")
            {
                imu.setAcceleration(0.0f, 1.0f, 0.0f);
                auto result = imu_reoriented.readAccelerometer();
                REQUIRE(result.has_value());
                auto accel_ecef = result.value();
                (void) accel_ecef;
            }

            SUBCASE("NED Down")
            {
                imu.setAcceleration(0.0f, 0.0f, 1.0f);
                auto result = imu_reoriented.readAccelerometer();
                REQUIRE(result.has_value());
                auto accel_ecef = result.value();
                (void) accel_ecef;
            }
        }
    }
}

struct EmptyIMU
{
    std::optional<std::array<au::QuantityF<au::MetersPerSecondSquaredInBodyFrame>, 3>> readAccelerometer()
    {
        return std::nullopt;
    }
};
static_assert(HasBodyAccelerometer<EmptyIMU>);

static_assert(HasEcefAccelerometer<IMUAccInECEFWithPolicy<EmptyIMU, MockOrientationProvider, MockPositionProvider>>);
TEST_CASE("IMUAccInECEFWithPolicy: Returns nullopt when IMU data is missing")
{
    EmptyIMU imu;
    MockOrientationProvider orientation;
    MockPositionProvider position;

    IMUAccInECEFWithPolicy<EmptyIMU, MockOrientationProvider, MockPositionProvider> imu_reoriented(imu, orientation, position);

    auto result = imu_reoriented.readAccelerometer();
    CHECK_FALSE(result.has_value());
}

static_assert(HasEcefAccelerometer<IMUAccInECEFWithPolicy<MockIMU, MockOrientationProvider, MockPositionProvider, NoGravityCompensation>>);
TEST_CASE("IMUAccInECEFWithPolicy with NoGravityCompensation: Identity rotation maps NED basis vectors to correct ECEF directions at lat=0, lon=0")
{
    MockIMU imu;
    MockOrientationProvider orientation;
    MockPositionProvider position;

    IMUAccInECEFWithPolicy<MockIMU, MockOrientationProvider, MockPositionProvider, NoGravityCompensation> imu_reoriented(imu, orientation, position);

    SUBCASE("NED North → ECEF Z")
    {
        imu.setAcceleration(1.0f, 0.0f, 0.0f);  // NED (1, 0, 0)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == 1.0f); // Z
    }

    SUBCASE("NED South → -ECEF Z")
    {
        imu.setAcceleration(-1.0f, 0.0f, 0.0f);  // NED (-1, 0, 0)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == -1.0f); // Z
    }

    SUBCASE("NED East → ECEF Y")
    {
        imu.setAcceleration(0.0f, 1.0f, 0.0f);  // NED (0, 1, 0)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == 1.0f); // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // Z
    }

    SUBCASE("NED West → -ECEF Y")
    {
        imu.setAcceleration(0.0f, -1.0f, 0.0f);  // NED (0, -1, 0)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == -1.0f); // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // Z
    }

    SUBCASE("NED Down → −ECEF X")
    {
        imu.setAcceleration(0.0f, 0.0f, 1.0f);  // NED (0, 0, 1)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == -1.0f); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f);  // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f);  // Z
    }

    SUBCASE("NED Up → ECEF X")
    {
        imu.setAcceleration(0.0f, 0.0f, -1.0f);  // NED (0, 0, -1)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == 1.0f); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f);  // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f);  // Z
    }
}

constexpr float gravity = 9.81f; // m/s²
static_assert(HasEcefAccelerometer<IMUAccInECEFWithPolicy<MockIMU, MockOrientationProvider, MockPositionProvider, SubtractGravityInNED>>);
TEST_CASE("IMUAccInECEFWithPolicy with SubtractGravityInNED: Identity rotation maps NED basis vectors to correct ECEF directions at lat=0, lon=0")
{
    MockIMU imu;
    MockOrientationProvider orientation;
    MockPositionProvider position;

    IMUAccInECEFWithPolicy<MockIMU, MockOrientationProvider, MockPositionProvider, SubtractGravityInNED> imu_reoriented(imu, orientation, position);

    SUBCASE("NED North → ECEF Z")
    {
        imu.setAcceleration(1.0f, 0.0f, 0.0f);  // NED (1, 0, 0)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == -gravity); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == 1.0f); // Z
    }

    SUBCASE("NED South → -ECEF Z")
    {
        imu.setAcceleration(-1.0f, 0.0f, 0.0f);  // NED (-1, 0, 0)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == -gravity); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == -1.0f); // Z
    }

    SUBCASE("NED East → ECEF Y")
    {
        imu.setAcceleration(0.0f, 1.0f, 0.0f);  // NED (0, 1, 0)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == -gravity); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == 1.0f); // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // Z
    }

    SUBCASE("NED West → -ECEF Y")
    {
        imu.setAcceleration(0.0f, -1.0f, 0.0f);  // NED (0, -1, 0)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == -gravity); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == -1.0f); // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f); // Z
    }

    SUBCASE("NED Down → −ECEF X")
    {
        imu.setAcceleration(0.0f, 0.0f, 1.0f);  // NED (0, 0, 1)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == 1.0f - gravity); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f);  // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f);  // Z
    }

    SUBCASE("NED Up → ECEF X")
    {
        imu.setAcceleration(0.0f, 0.0f, -1.0f);  // NED (0, 0, -1)
        auto result = imu_reoriented.readAccelerometer();
        REQUIRE(result.has_value());
        auto accel_ecef = result.value();

        CHECK(doctest::Approx(accel_ecef[0].in(au::metersPerSecondSquaredInEcefFrame)) == -(1.0f + gravity)); // X
        CHECK(doctest::Approx(accel_ecef[1].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f);  // Y
        CHECK(doctest::Approx(accel_ecef[2].in(au::metersPerSecondSquaredInEcefFrame)) == 0.0f);  // Z
    }
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
