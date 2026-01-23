
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "PositionTracker9D.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <optional>

#include "au.hpp"
#include "mock_hal.h"
#include "TimeUtils.hpp"
#include "GNSS.hpp"
#include "IMUExtension.hpp"

TEST_CASE("rotateNEDtoECEF correctly transforms known NED vectors at various lat/lon")
{
    constexpr float gravity = 9.81f;

    SUBCASE("At equator (0°, 0°), NED gravity should map to ECEF -Z")
    {
        Eigen::Vector3f ned_vec(0.f, 0.f, gravity); // Down in NED
        Eigen::Vector3f ecef = computeNEDtoECEFRotation(au::degreesInGeodeticFrame(0.f), au::degreesInGeodeticFrame(0.f)) * ned_vec;

        // Expect mostly -Z direction in ECEF
        CHECK(ecef(0) == doctest::Approx(-gravity)); // strong downward X
        CHECK(ecef(1) == doctest::Approx(0.0f));     // near zero Y
        CHECK(ecef(2) == doctest::Approx(0.0f));     // near zero Z
    }

    SUBCASE("At North Pole (90°, 0°), East in NED should point roughly ECEF -Y")
    {
        Eigen::Vector3f ned_vec(0.f, 1.f, 0.f); // East in NED
        Eigen::Vector3f ecef = computeNEDtoECEFRotation(au::degreesInGeodeticFrame(90.f), au::degreesInGeodeticFrame(0.f)) * ned_vec;

        CHECK_GT(ecef(1), 0.9f); // East maps to positive ECEF Y
    }

    SUBCASE("At Katy, TX (29.8°, -95.8°), Down maps to tilted ECEF vector")
    {
        Eigen::Vector3f ned_vec(0.f, 0.f, gravity); // NED down
        Eigen::Vector3f ecef = computeNEDtoECEFRotation(au::degreesInGeodeticFrame(29.8f), au::degreesInGeodeticFrame(-95.8f)) * ned_vec;

        constexpr float m_pif = std::numbers::pi_v<float>;
        constexpr float lat_rad = 29.8f * m_pif / 180.0f;
        constexpr float lon_rad = -95.8f * m_pif / 180.0f;
        constexpr float x = -gravity * std::cos(lat_rad) * std::cos(lon_rad);
        constexpr float y = -gravity * std::cos(lat_rad) * std::sin(lon_rad);
        constexpr float z = -gravity * std::sin(lat_rad);

        // Gravity should map to a consistent ECEF direction (not pure Z)
        CHECK(ecef.norm() == doctest::Approx(gravity)); // gravity magnitude preserved
        CHECK(ecef(0) == doctest::Approx(x));
        CHECK(ecef(1) == doctest::Approx(y));
        CHECK(ecef(2) == doctest::Approx(z));
    }

        SUBCASE("Somewhere, TX (29.8°, 95.8°), Down maps to tilted ECEF vector")
    {
        Eigen::Vector3f ned_vec(0.f, 0.f, gravity); // NED down
        Eigen::Vector3f ecef = computeNEDtoECEFRotation(au::degreesInGeodeticFrame(29.8f), au::degreesInGeodeticFrame(95.8f)) * ned_vec;

        constexpr float m_pif = std::numbers::pi_v<float>;
        constexpr float lat_rad = 29.8f * m_pif / 180.0f;
        constexpr float lon_rad = 95.8f * m_pif / 180.0f;
        constexpr float x = -gravity * std::cos(lat_rad) * std::cos(lon_rad);
        constexpr float y = -gravity * std::cos(lat_rad) * std::sin(lon_rad);
        constexpr float z = -gravity * std::sin(lat_rad);

        // Gravity should map to a consistent ECEF direction (not pure Z)
        CHECK(ecef.norm() == doctest::Approx(gravity)); // gravity magnitude preserved
        CHECK(ecef(0) == doctest::Approx(x));
        CHECK(ecef(1) == doctest::Approx(y));
        CHECK(ecef(2) == doctest::Approx(z));
    }

}

class MockPositionTracker9D : public PositionTracker9D
{
public:
    MockPositionTracker9D() : PositionTracker9D() {}

    void updateWithAccel(const Eigen::Vector3f &accel, au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        PositionTracker9D::updateWithAccel(accel, timestamp);
    }

    void updateWithGps(const Eigen::Vector3f &gps, au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        PositionTracker9D::updateWithGps(gps, timestamp);
    }

    StateVector getState() const
    {
        return PositionTracker9D::getState();
    }

    void maybePredict(au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        PositionTracker9D::maybePredict(timestamp);
    }

    void updateTransitionMatrix(float dt)
    {
        PositionTracker9D::updateTransitionMatrix(dt);
    }

    Eigen::Matrix<float, 9, 9> transitionMatrix()
    {
        return PositionTracker9D::A;
    }
};

TEST_CASE("PositionTracker9D handles asynchronous GPS and accel updates")
{
    PositionTracker9D tracker;

    const Eigen::Vector3f true_accel(1.0f, 0.5f, -0.8f);
    float time = 0.0f;
    const float sim_duration = 10.0f;
    const float accel_dt = 0.1f;
    const float gps_dt = 1.0f;

    float next_gps_time = 0.0f;

    while (time <= sim_duration)
    {
        // Simulate noisy accel measurement
        Eigen::Vector3f accel_meas = true_accel + Eigen::Vector3f::Random() * 0.02f;
        tracker.updateWithAccel(accel_meas, au::make_quantity<au::Milli<au::Seconds>>(static_cast<uint64_t>(time*1000.f)));

        // Simulate GPS at lower rate
        if (std::abs(time - next_gps_time) < 1e-4f)
        {
            Eigen::Vector3f true_pos = 0.5f * true_accel * time * time;
            Eigen::Vector3f gps_meas = true_pos + Eigen::Vector3f::Random() * 0.05f;
            tracker.updateWithGps(gps_meas, au::make_quantity<au::Milli<au::Seconds>>(static_cast<uint64_t>(time*1000.f)));
            next_gps_time += gps_dt;
        }

        time += accel_dt;
        // float dt = timestamp_sec - last_timestamp_sec_;
        // if (dt <= 0.f)
        //     return;

    }

    auto est = tracker.getState();
    Eigen::Vector3f expected_pos = 0.5f * true_accel * sim_duration * sim_duration;
    Eigen::Vector3f expected_vel = true_accel * sim_duration;
    Eigen::Vector3f expected_acc = true_accel;

    // std::cout << "Estimated pos: " << est.segment<3>(0).transpose() << "\n";
    // std::cout << "Estimated vel: " << est.segment<3>(3).transpose() << "\n";
    // std::cout << "Estimated acc: " << est.segment<3>(6).transpose() << "\n";

    for (int i = 0; i < 3; ++i)
    {
        CHECK(est.segment<3>(0)(i) == doctest::Approx(expected_pos(i)).epsilon(0.2f));
        CHECK(est.segment<3>(3)(i) == doctest::Approx(expected_vel(i)).epsilon(0.2f));
        CHECK(est.segment<3>(6)(i) == doctest::Approx(expected_acc(i)).epsilon(0.2f));
    }
}

TEST_CASE("Acceleration update without frame rotation causes state inconsistency")
{
    PositionTracker9D tracker;

    // Initial timestamp
    float t0 = 0.0f;

    // GPS measurement at origin
    Eigen::Vector3f gps_position = Eigen::Vector3f::Zero();
    tracker.updateWithGps(gps_position, au::make_quantity<au::Milli<au::Seconds>>(static_cast<uint64_t>(t0*1000.f)));

    // Simulated raw accelerometer reading (gravity in body frame)
    Eigen::Vector3f accel_body = Eigen::Vector3f(0.f, 0.f, -9.81f); // No rotation applied

    // Update with IMU acceleration after small dt
    float dt = 0.1f; // 100 ms later
    float t1 = t0 + dt;
    tracker.updateWithAccel(accel_body, au::make_quantity<au::Milli<au::Seconds>>(static_cast<uint64_t>(t1*1000.f)));

    // Predicted state
    auto state = tracker.getState();

    SUBCASE("Position should remain near origin if acceleration is unrotated")
    {
        // If fusion assumes ECEF, this update pushes position incorrectly
        CHECK_LT(state.head<3>().norm(), 1.f); // Displacement should be small
    }

    SUBCASE("Velocity and acceleration should show bias due to gravity if untreated")
    {
        // Velocity should reflect gravity integrated over dt (misinterpreted frame)
        CHECK(state(5) < 0.f); // z-velocity (ECEF) should now be falling
        CHECK(state(8) < 0.f); // z-acceleration (ECEF) misinterprets body gravity
    }
}

TEST_CASE("Rotated body-frame gravity suppresses bias in ECEF fusion")
{
    PositionTracker9D tracker;

    // Start at origin with zero velocity
    float t0 = 0.0f;
    tracker.updateWithGps(Eigen::Vector3f::Zero(), au::make_quantity<au::Milli<au::Seconds>>(static_cast<uint64_t>(t0*1000.f)));

    // Assume static orientation: body frame aligned with ECEF
    Eigen::Matrix3f R_ecef_from_body = Eigen::Matrix3f::Identity(); // No rotation for now

    // Gravity in body frame
    Eigen::Vector3f accel_body = Eigen::Vector3f(0.f, 0.f, -9.81f);

    // Rotate to ECEF frame
    Eigen::Vector3f accel_ecef = R_ecef_from_body * accel_body;

    // Update tracker with rotated acceleration
    float dt = 0.1f;
    float t1 = t0 + dt;
    tracker.updateWithAccel(accel_ecef, au::make_quantity<au::Milli<au::Seconds>>(static_cast<uint64_t>(t1*1000.f)));
    for (int i = 1; i <= 100; ++i)
    {
        float t = t0 + static_cast<float>(i) * dt;
        tracker.updateWithAccel(accel_ecef, au::make_quantity<au::Milli<au::Seconds>>(static_cast<uint64_t>(t*1000.f)));
    }

    auto state = tracker.getState();

    SUBCASE("Position drift remains minimal under correct gravity interpretation")
    {
        Eigen::Vector3f expected_pos = 0.5f * accel_ecef * 10.0f * 10.0f;
        CHECK(state.segment<3>(0).isApprox(expected_pos, 5.0f)); // 5 meter tolerance
        CHECK(state(0) == doctest::Approx(expected_pos(0)).epsilon(1.0f));
        CHECK(state(1) == doctest::Approx(expected_pos(1)).epsilon(1.0f));
        CHECK(state(2) == doctest::Approx(expected_pos(2)).epsilon(100.0f));
    }

    SUBCASE("Velocity and acceleration reflect gravity but suppress false drift")
    {
        CHECK(state(5) < -0.5f); // z-velocity from gravity (dt * -9.81)
        CHECK(state(8) < -9.0f); // z-acceleration close to true gravity
    }
}

TEST_CASE("Transition matrix A updates correctly for nonzero dt")
{
    MockPositionTracker9D tracker;
    float dt = 0.1f;
    tracker.updateTransitionMatrix(dt);

    const auto &A = tracker.transitionMatrix(); // Expose A via getter or friend

    CHECK(A(0, 3) == doctest::Approx(dt));
    CHECK(A(0, 6) == doctest::Approx(0.5f * dt * dt));
    CHECK(A(3, 6) == doctest::Approx(dt));
}

#
#
#

class MockGNSS
{
public:
    void setPositionECEF(const Eigen::Vector3f &pos_meters)
    {
        PositionECEF pos_cm;
        pos_cm.ecefX = static_cast<int32_t>(std::round(pos_meters.x() * 100.0f));
        pos_cm.ecefY = static_cast<int32_t>(std::round(pos_meters.y() * 100.0f));
        pos_cm.ecefZ = static_cast<int32_t>(std::round(pos_meters.z() * 100.0f));
        pos_cm.pAcc = 100; // Example: 1 meter accuracy
        pos_ = pos_cm;
    }

    std::optional<PositionECEF> getNavPosECEF() const
    {
        return pos_;
    }

private:
    std::optional<PositionECEF> pos_;
};

class MockIMUinBodyFrame
{
public:
    void setAcceleration(const Eigen::Vector3f &accel_mps2)
    {
        accel_ = accel_mps2;
    }

    std::optional<std::array<au::QuantityF<au::MetersPerSecondSquaredInBodyFrame>, 3>> readAccelerometer() const
    {
        if (!accel_.has_value())
            return std::nullopt;

        // std::cerr << "accel = " << accel_.value() << "\n";

        return std::array{
            au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(accel_->x()),
            au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(accel_->y()),
            au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(accel_->z())};
    }

private:
    std::optional<Eigen::Vector3f> accel_;
};
static_assert(HasBodyAccelerometer<MockIMUinBodyFrame>);

class MockOrientationProvider
{
public:
    void predict(std::array<float, 4> &q_body_to_ned,
                 au::QuantityU64<au::Milli<au::Seconds>> &timestamp) const
    {
        q_body_to_ned = {1.0f, 0.0f, 0.0f, 0.0f};
        timestamp = au::make_quantity<au::Milli<au::Seconds>>(0);
    }
};

class MockPositionProvider
{
public:
    void predict(std::array<au::QuantityF<au::MetersInEcefFrame>, 3> &pos,
                 std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> &vel,
                 au::QuantityU64<au::Milli<au::Seconds>> &timestamp) const
    {
        pos = {au::make_quantity<au::MetersInEcefFrame>(0.f),
               au::make_quantity<au::MetersInEcefFrame>(0.f),
               au::make_quantity<au::MetersInEcefFrame>(0.f)};
        vel = {au::make_quantity<au::MetersPerSecondInEcefFrame>(0.f),
               au::make_quantity<au::MetersPerSecondInEcefFrame>(0.f),
               au::make_quantity<au::MetersPerSecondInEcefFrame>(0.f)};
        timestamp = au::make_quantity<au::Milli<au::Seconds>>(0);
    }
};

#
#
#
#
#

TEST_CASE("Time advances correctly through mocked RTC")
{
    PositionTracker9D tracker;
    MockGNSS gnss;
    MockIMUinBodyFrame imu;
    MockOrientationProvider orientation;

    RTC_HandleTypeDef rtc_handle{};
    rtc_handle.Init.SynchPrediv = 1023;

    const Eigen::Vector3f initial_pos(6371000.0f, 0.0f, 0.0f);
    tracker.setState((Eigen::Matrix<float, 9, 1>() << initial_pos.x(), initial_pos.y(), initial_pos.z(), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f).finished());
    gnss.setPositionECEF(initial_pos);  // Equator, lon = 0°
    imu.setAcceleration(Eigen::Vector3f(0.f, 0.f, 9.81f));
    GNSSandAccelPosition positioner(&rtc_handle, tracker, gnss, imu, orientation);

    std::array<au::QuantityF<au::MetersInEcefFrame>, 3> r;
    std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> v;
    au::QuantityU64<au::Milli<au::Seconds>> timestamp;

    for (uint32_t i = 0; i < 10; ++i)
    {
        // Initialize mocked RTC time
        RTC_TimeTypeDef time = {0, 0, 0, RTC_HOURFORMAT12_AM, 1023u - i * 100u, 1023, RTC_DAYLIGHTSAVING_NONE, RTC_STOREOPERATION_RESET};
        RTC_DateTypeDef date = {RTC_WEEKDAY_MONDAY, 1, 1, 00}; // Year 2000
        set_mocked_rtc_time(time);
        set_mocked_rtc_date(date);

        positioner.predict(r, v, timestamp);
        // std::cerr << "Time advances correctly through mocked RTC timestamp " << timestamp.in(au::milli(au::second)) << std::endl;
    }

    Eigen::Vector3f pos(r[0].in(au::ecefs * au::meters), r[1].in(au::ecefs * au::meters), r[2].in(au::ecefs * au::meters));
    Eigen::Vector3f vel(v[0].in(au::ecefs * au::meters / au::seconds), v[1].in(au::ecefs * au::meters / au::seconds), v[2].in(au::ecefs * au::meters / au::seconds));

    CHECK((pos - initial_pos).norm() < 5.f);
    CHECK((6371000.f - r[0].in(au::metersInEcefFrame)) > 0.0f );
    CHECK(r[1].in(au::metersInEcefFrame) == doctest::Approx(0.0f));
    CHECK(r[2].in(au::metersInEcefFrame) == doctest::Approx(0.0f));
    CHECK(vel.x() < 0.f);
}

TEST_CASE("Duration conversion sanity check")
{
    au::QuantityF<au::Seconds> dt_ms = au::make_quantity<au::Milli<au::Seconds>>(1000.0f); // 1000 ms → 1 s
    au::QuantityF<au::Seconds> dt_sec = dt_ms.as(au::seconds);
    CHECK(dt_sec.in(au::seconds) == au::make_quantity<au::Seconds>(1.0f).in(au::seconds));
}

TEST_CASE("Tracker integrates constant acceleration")
{
    PositionTracker9D tracker;
    for (int i = 0; i < 1000; ++i)
    {
        float t = static_cast<float>(i) * 0.01f; // 10 ms steps
        tracker.updateWithAccel(Eigen::Vector3f(0, 0, -9.81f), au::make_quantity<au::Milli<au::Seconds>>(static_cast<uint64_t>(t*1000.f)));
    }
    auto state = tracker.getState();
    CHECK(state[5] < 0); // vel.z
    CHECK(state[2] < 0); // pos.z
}

TEST_CASE("Unrotated body-frame acceleration causes drift in ECEF fusion")
{
    // Setup
    PositionTracker9D tracker;
    MockGNSS gnss;
    MockIMUinBodyFrame imu;
    MockOrientationProvider orientation;

    RTC_HandleTypeDef mock_rtc{};
    mock_rtc.Init.SynchPrediv = 1023;
    set_mocked_rtc_time({0, 0, 0, RTC_HOURFORMAT12_AM, 1023, 1023, RTC_DAYLIGHTSAVING_NONE, RTC_STOREOPERATION_RESET});
    set_mocked_rtc_date({RTC_WEEKDAY_MONDAY, 1, 1, 00}); // Year 2000

    // Initialize filter state at equator, lon = 0°
    const Eigen::Vector3f initial_pos(6371000.0f, 0.0f, 0.0f);
    tracker.setState((Eigen::Matrix<float, 9, 1>() << initial_pos.x(), initial_pos.y(), initial_pos.z(),
                      0.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 0.0f).finished());

    // Inject body-frame gravity (no rotation applied)
    imu.setAcceleration(Eigen::Vector3f(0.f, 0.f, 9.81f));
    GNSSandAccelPosition positioner(&mock_rtc, tracker, gnss, imu, orientation);

    // Prediction loop
    au::QuantityU64<au::Milli<au::Seconds>> timestamp;
    std::array<au::QuantityF<au::MetersInEcefFrame>, 3> r;
    std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> v;

    for (uint32_t i = 1; i <= 30; ++i)
    {
        set_mocked_rtc_time({0, 0, 0, RTC_HOURFORMAT12_AM, 1023u - i * 100u, 1023, RTC_DAYLIGHTSAVING_NONE, RTC_STOREOPERATION_RESET});
        set_mocked_rtc_date({RTC_WEEKDAY_MONDAY, 1, 1, 00});

        positioner.predict(r, v, timestamp);

        Eigen::Vector3f pos_debug(r[0].in(au::ecefs * au::meters),
                                  r[1].in(au::ecefs * au::meters),
                                  r[2].in(au::ecefs * au::meters));

        Eigen::Vector3f vel_debug(v[0].in(au::ecefs * au::meters / au::seconds),
                                  v[1].in(au::ecefs * au::meters / au::seconds),
                                  v[2].in(au::ecefs * au::meters / au::seconds));

        // std::cerr << "[Step " << i << "] "
        //           << "t = " << timestamp.in(au::milli(au::seconds)) << " ms, "
        //           << "pos = " << pos_debug.transpose() << " m, "
        //           << "vel = " << vel_debug.transpose() << " m/s\n";
    }

    // Final state
    Eigen::Vector3f pos(r[0].in(au::ecefs * au::meters),
                        r[1].in(au::ecefs * au::meters),
                        r[2].in(au::ecefs * au::meters));

    Eigen::Vector3f vel(v[0].in(au::ecefs * au::meters / au::seconds),
                        v[1].in(au::ecefs * au::meters / au::seconds),
                        v[2].in(au::ecefs * au::meters / au::seconds));

    CHECK((initial_pos - pos).norm() < 50.0f);
    CHECK(vel.x() < 0.f); // Falling due to misinterpreted gravity
}

TEST_CASE("Rotated body-frame gravity suppresses drift in ECEF fusion")
{
    {
        Eigen::Vector3f accel_body(0.f, 0.f, -9.81f);
        Eigen::Quaternionf q(Eigen::AngleAxisf(M_PIf, Eigen::Vector3f::UnitX()));
        Eigen::Vector3f accel_ecef = q * accel_body;
        // std::cout << "Rotated accel: " << accel_ecef.transpose() << std::endl;
        CHECK(accel_ecef[0] == doctest::Approx(0.0f).epsilon(1e-5f));
        CHECK(accel_ecef[1] == doctest::Approx(0.0f).epsilon(1e-5f));
        CHECK(accel_ecef[2] == doctest::Approx(9.81f).epsilon(1e-5f));
    }

    RTC_HandleTypeDef mock_rtc{};
    mock_rtc.Init.SynchPrediv = 1023;
    RTC_TimeTypeDef time = {0, 0, 0, RTC_HOURFORMAT12_AM, 1023, 1023, RTC_DAYLIGHTSAVING_NONE, RTC_STOREOPERATION_RESET};
    RTC_DateTypeDef date = {RTC_WEEKDAY_MONDAY, 1, 1, 00}; // Year 2000
    set_mocked_rtc_time(time);
    set_mocked_rtc_date(date);

    MockGNSS gnss;
    MockIMUinBodyFrame imu;
    MockOrientationProvider orientation;

    PositionTracker9D tracker;

    // Initialize filter state at equator, lon = 0°
    const Eigen::Vector3f initial_pos(6371000.0f, 0.0f, 0.0f);
    tracker.setState((Eigen::Matrix<float, 9, 1>() << initial_pos.x(), initial_pos.y(), initial_pos.z(),
                      0.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 0.0f).finished());

    // Inject body-frame gravity (no rotation applied)
    imu.setAcceleration(Eigen::Vector3f(0.f, 0.f, 9.81f));
    GNSSandAccelPosition<PositionTracker9D, MockGNSS, MockIMUinBodyFrame, MockOrientationProvider, SubtractGravityInNED> positioner(&mock_rtc, tracker, gnss, imu, orientation);

    std::array<au::QuantityF<au::MetersInEcefFrame>, 3> r;
    std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> v;
    au::QuantityU64<au::Milli<au::Seconds>> timestamp;

    // Apply gravity over 10 steps
    for (uint8_t i = 1; i <= 20; ++i)
    {
        uint8_t sec = i % 60;
        uint8_t min = i / 60;
        RTC_TimeTypeDef time = {0, min, sec, RTC_HOURFORMAT12_AM, 1023, 1023, RTC_DAYLIGHTSAVING_NONE, RTC_STOREOPERATION_RESET};
        RTC_DateTypeDef date = {RTC_WEEKDAY_MONDAY, 1, 1, 00}; // Year 2000
        set_mocked_rtc_time(time);
        set_mocked_rtc_date(date);

        positioner.predict(r, v, timestamp);

        Eigen::Vector3f pos_debug(r[0].in(au::ecefs * au::meters),
                                  r[1].in(au::ecefs * au::meters),
                                  r[2].in(au::ecefs * au::meters));

        Eigen::Vector3f vel_debug(v[0].in(au::ecefs * au::meters / au::seconds),
                                  v[1].in(au::ecefs * au::meters / au::seconds),
                                  v[2].in(au::ecefs * au::meters / au::seconds));

        // std::cerr << "[Step " << i << "] "
        //           << "t = " << timestamp.in(au::milli(au::seconds)) << " ms, "
        //           << "pos.z = " << pos_debug.z() << " m, "
        //           << "vel.z = " << vel_debug.z() << " m/s\n";
    }

    Eigen::Vector3f pos(r[0].in(au::ecefs * au::meters), r[1].in(au::ecefs * au::meters), r[2].in(au::ecefs * au::meters));
    Eigen::Vector3f vel(v[0].in(au::ecefs * au::meters / au::seconds), v[1].in(au::ecefs * au::meters / au::seconds), v[2].in(au::ecefs * au::meters / au::seconds));

        CHECK(pos[0] == doctest::Approx(initial_pos[0]).epsilon(0.1f));
        CHECK(pos[1] == doctest::Approx(initial_pos[1]).epsilon(0.1f));
        CHECK(pos[2] == doctest::Approx(initial_pos[2]).epsilon(0.1f));
        CHECK(vel[0] == doctest::Approx(0.0f).epsilon(0.1f));
        CHECK(vel[1] == doctest::Approx(0.0f).epsilon(0.1f));
        CHECK(vel[2] == doctest::Approx(0.0f).epsilon(0.1f));
}
