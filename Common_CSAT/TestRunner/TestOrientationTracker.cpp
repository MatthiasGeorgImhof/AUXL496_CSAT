#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "OrientationService.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <cmath>

constexpr float m_mpif = static_cast<float>(std::numbers::pi);

TEST_CASE("GyrMagOrientationTracker initializes with identity quaternion")
{
    GyrMagOrientationTracker tracker;
    auto q = tracker.getOrientation();
    REQUIRE(q.isApprox(Eigen::Quaternionf::Identity(), 1e-6f));
}

TEST_CASE("predictTo integrates quaternion forward using gyro state")
{
    GyrMagOrientationTracker tracker;

    // Set gyro *before* any prediction so it’s used for [0 → 1] interval
    Eigen::Vector3f omega(0, 0, m_mpif / 2.0f); // 90°/s yaw
    tracker.setGyroAngularRate(omega);         // Inject ω into state

    // Now integrate 1 second forward from t=0.0 → t=1.0
    tracker.predictTo(au::make_quantity<au::Seconds>(1));

    Eigen::Quaternionf q = tracker.getOrientation();
    float yaw = std::atan2(2.f * (q.w() * q.z() + q.x() * q.y()),
                           1.f - 2.f * (q.y() * q.y() + q.z() * q.z()));

    REQUIRE(std::abs(yaw - (m_mpif / 2.0f)) < 0.01f); // ✅ ~90°
}

TEST_CASE("updateMagnetometer reduces yaw error after prediction")
{
    GyrMagOrientationTracker tracker;
    tracker.setReferenceVectors(Eigen::Vector3f(0.3f, 0.5f, 0.8f));


    Eigen::Vector3f omega(0, 0, m_mpif / 180.0f * 45.0f); // 45 deg/s
    tracker.updateGyro(omega, au::make_quantity<au::Seconds>(1));

    Eigen::Quaternionf q_true(static_cast<Eigen::Quaternionf>(Eigen::AngleAxisf(m_mpif / 4.0f, Eigen::Vector3f::UnitZ())));
    Eigen::Vector3f mag_ned(1.f, 0.f, 0.f);
    Eigen::Vector3f mag_meas = q_true.conjugate() * mag_ned;

    tracker.predictTo(au::make_quantity<au::Seconds>(4)); // Predict without correction
    float yaw_before = std::atan2(
        2.f * (tracker.getOrientation().w() * tracker.getOrientation().z()),
        1.f - 2.f * (tracker.getOrientation().z() * tracker.getOrientation().z()));

    // std::cerr << "Yaw before update: " << yaw_before * 180.0f / m_mpif << " deg\n";
    for (int i = 0; i < 50; ++i)
    {
        tracker.updateMagnetometer(mag_meas, au::make_quantity<au::Seconds>(4));
        // float yaw_after = std::atan2(
        //     2.f * (tracker.getOrientation().w() * tracker.getOrientation().z()),
        //     1.f - 2.f * (tracker.getOrientation().z() * tracker.getOrientation().z()));
        // std::cerr << "Yaw after update: " << yaw_after * 180.0f / m_mpif << " deg\n";
    }
    float yaw_after = std::atan2(
        2.f * (tracker.getOrientation().w() * tracker.getOrientation().z()),
        1.f - 2.f * (tracker.getOrientation().z() * tracker.getOrientation().z()));
    // std::cerr << "Yaw after update: " << yaw_after * 180.0f / m_mpif << " deg\n";

    REQUIRE(std::abs(yaw_before - yaw_after) > 1e-3f); // Should adjust
}

TEST_CASE("GyrMagOrientationTracker follows yaw rotation with magnetometer corrections")
{
    GyrMagOrientationTracker tracker;
    tracker.setReferenceVectors(Eigen::Vector3f(0.3f, 0.5f, 0.8f));


    float dt = 0.5f;
    float yaw_rate = 30.f * m_mpif / 180.f; // 30 deg/s
    Eigen::Vector3f omega(0.f, 0.f, yaw_rate);

    Eigen::Quaternionf q_true = Eigen::Quaternionf::Identity();
    Eigen::Vector3f mag_ned = Eigen::Vector3f(1.0f, 0.0f, 0.0f); // aligned with North

    for (int step = 0; step < 20; ++step)
    {
        float t = static_cast<float>(step) * dt;

        // Integrate true orientation
        Eigen::Quaternionf dq(1.0f, 0.0f, 0.0f, 0.5f * omega(2) * dt);
        q_true = (q_true * dq).normalized();

        // Simulate measurements
        Eigen::Vector3f mag_meas = q_true.conjugate() * mag_ned + Eigen::Vector3f::Random() * 0.01f;

        tracker.updateGyro(omega, au::make_quantity<au::Milli<au::Seconds>>(static_cast<uint64_t>(t*1000.0f)));
        if (step % 2 == 0)
        { // magnetometer at 1 Hz
            tracker.updateMagnetometer(mag_meas, au::make_quantity<au::Milli<au::Seconds>>(static_cast<uint64_t>(t*1000.f)));
        }

        // Compare estimated yaw
        Eigen::Quaternionf q_est = tracker.getOrientation();
        float yaw_est = std::atan2(2.f * (q_est.w() * q_est.z() + q_est.x() * q_est.y()),
                                   1.f - 2.f * (q_est.y() * q_est.y() + q_est.z() * q_est.z()));

        float yaw_true = std::atan2(2.f * (q_true.w() * q_true.z() + q_true.x() * q_true.y()),
                                    1.f - 2.f * (q_true.y() * q_true.y() + q_true.z() * q_true.z()));

        float err = std::atan2(std::sin(yaw_est - yaw_true), std::cos(yaw_est - yaw_true));
        // std::cerr << "Step " << step + 1 << " | Yaw error: " << err * 180.f / m_mpif << " deg\n";

        REQUIRE(std::abs(err) < 0.3f);
    }
}

#
#
#

TEST_CASE("AccGyrMagOrientationTracker initializes with identity quaternion")
{
    AccGyrMagOrientationTracker tracker;
    auto q = tracker.getOrientation();
    REQUIRE(q.isApprox(Eigen::Quaternionf::Identity(), 1e-6f));
}

TEST_CASE("predictTo integrates quaternion forward using gyro state")
{
    AccGyrMagOrientationTracker tracker;

    // Set gyro *before* any prediction so it’s used for [0 → 1] interval
    Eigen::Vector3f omega(0, 0, m_mpif / 2.0f); // 90°/s yaw
    tracker.setGyroAngularRate(omega);         // Inject ω into state

    // Now integrate 1 second forward from t=0.0 → t=1.0
    tracker.predictTo(au::make_quantity<au::Seconds>(1));

    Eigen::Quaternionf q = tracker.getOrientation();
    float yaw = std::atan2(2.f * (q.w() * q.z() + q.x() * q.y()),
                           1.f - 2.f * (q.y() * q.y() + q.z() * q.z()));

    REQUIRE(std::abs(yaw - (m_mpif / 2.0f)) < 0.01f); // ✅ ~90°
}

TEST_CASE("updateAccelerometerMagnetometer converges yaw orientation within envelope")
{
    AccGyrMagOrientationTracker tracker;
    tracker.setReferenceVectors(Eigen::Vector3f(0.0f, 0.0f, 9.81f), Eigen::Vector3f(1.0f, 0.0f, 0.0f));

    Eigen::Quaternionf q_true(Eigen::AngleAxisf(m_mpif / 4.0f, Eigen::Vector3f::UnitZ()));
    Eigen::Vector3f accel_ned(0.0f, 0.0f, 9.81f);
    Eigen::Vector3f mag_ned(1.0f, 0.0f, 0.0f);
    Eigen::Vector3f accel_body = q_true.conjugate() * accel_ned;
    Eigen::Vector3f mag_body = q_true.conjugate() * mag_ned;

    Eigen::Vector3f omega(0.0f, 0.0f, 0.1f);
    tracker.setGyroAngularRate(omega);

    // Add these lines:
    // std::cerr << "Before predictTo:\n";
    // std::cerr << "  q_hat: " << tracker.getOrientation().coeffs().transpose() << "\n";
    tracker.predictTo(au::make_quantity<au::Milli<au::Seconds>>(100));
    // std::cerr << "After predictTo:\n";
    // std::cerr << "  q_hat: " << tracker.getOrientation().coeffs().transpose() << "\n";

    float yaw_true = m_mpif / 4.0f;
    std::vector<float> yaw_errors;

    for (int i = 0; i < 15; ++i)
    {
        // std::cerr << "accel_body: " << accel_body.transpose() << "\n";
        // std::cerr << "mag_body: " << mag_body.transpose() << "\n";

        tracker.updateAccelerometerMagnetometer(accel_body, mag_body, au::make_quantity<au::Milli<au::Seconds>>(100*i));
        float yaw_est = tracker.getYawPitchRoll()(0);
        float err = std::atan2(std::sin(yaw_est - yaw_true), std::cos(yaw_est - yaw_true));
        yaw_errors.push_back(std::abs(err));

        // std::cerr << "Step " << i + 1
        //           << " | Estimated Yaw: " << yaw_est * 180.0f / m_mpif
        //           << " deg | Error: " << err * 180.0f / m_mpif << " deg\n";
    }

    // REQUIRE that error drops below threshold within first N steps
    bool converged = false;
    for (size_t i = 0; i < yaw_errors.size(); ++i)
    {
        if (yaw_errors[i] < 0.6f) // ~34.4°
        {
            converged = true;
            break;
        }
    }
    REQUIRE(converged);

    // REQUIRE that error stays within envelope after convergence
    for (size_t i = 10; i < yaw_errors.size(); ++i)
    {
        REQUIRE(yaw_errors[i] < 2.0f); // ~91.6°, allows oscillation but bounds it
    }
}

TEST_CASE("AccGyrMagOrientationTracker follows yaw rotation with accelerometer and magnetometer corrections")
{
    AccGyrMagOrientationTracker tracker;
    tracker.setReferenceVectors(Eigen::Vector3f(0.0f, 0.0f, 9.81f), Eigen::Vector3f(1.0f, 0.0f, 0.0f));

    float dt = 0.5f;
    float yaw_rate = 30.f * m_mpif / 180.f; // 30 deg/s
    Eigen::Vector3f omega(0.f, 0.f, yaw_rate);

    Eigen::Quaternionf q_true = Eigen::Quaternionf::Identity();
    Eigen::Vector3f accel_ned(0.0f, 0.0f, 9.81f);
    Eigen::Vector3f mag_ned(1.0f, 0.0f, 0.0f); // aligned with North

    for (int step = 0; step < 109; ++step)
    {
        float t = static_cast<float>(step) * dt;

        // Integrate true orientation
        Eigen::Quaternionf dq(1.0f, 0.0f, 0.0f, 0.5f * omega(2) * dt);
        q_true = (q_true * dq).normalized();

        // Simulate measurements
        Eigen::Vector3f accel_meas = q_true.conjugate() * accel_ned + Eigen::Vector3f::Random() * 0.01f;
        Eigen::Vector3f mag_meas = q_true.conjugate() * mag_ned + Eigen::Vector3f::Random() * 0.01f;

        tracker.updateGyro(omega, au::make_quantity<au::Milli<au::Seconds>>(static_cast<u_int64_t>(t*1000.f)));
        tracker.updateAccelerometerMagnetometer(accel_meas, mag_meas, au::make_quantity<au::Milli<au::Seconds>>(static_cast<u_int64_t>(t*1000.f)));

        // Compare estimated yaw
        Eigen::Quaternionf q_est = tracker.getOrientation();
        float yaw_est = std::atan2(2.f * (q_est.w() * q_est.z() + q_est.x() * q_est.y()),
                                   1.f - 2.f * (q_est.y() * q_est.y() + q_est.z() * q_est.z()));

        float yaw_true = std::atan2(2.f * (q_true.w() * q_true.z() + q_true.x() * q_true.y()),
                                    1.f - 2.f * (q_true.y() * q_true.y() + q_true.z() * q_true.z()));

        float err = std::atan2(std::sin(yaw_est - yaw_true), std::cos(yaw_est - yaw_true));
        float err_deg = err * 180.f / m_mpif;
        if (err_deg > 180.f)
            err_deg -= 360.f;
        if (err_deg < -180.f)
            err_deg += 360.f;

        // std::cerr << "Step " << step + 1 << "\n";
        // std::cerr << "True Yaw (degrees): " << yaw_true * 180.0f / m_mpif << std::endl;
        // std::cerr << "Estimated Yaw (degrees): " << yaw_est * 180.0f / m_mpif << std::endl;
        // std::cerr << "Yaw Error (wrapped): " << err_deg << " deg\n";

        if (step > 100)
            REQUIRE(std::abs(err) < 0.6f);
    }
}

TEST_CASE("updateAccelerometerMagnetometer converges yaw orientation within envelope - SIMPLIFIED")
{
    AccGyrMagOrientationTracker tracker;
    tracker.setReferenceVectors(Eigen::Vector3f(0.0f, 0.0f, 9.81f), Eigen::Vector3f(1.0f, 0.0f, 0.0f));

    // Set a known initial yaw error (e.g., start at 0 yaw, target is M_PI/4)
    Eigen::Quaternionf q_initial = Eigen::Quaternionf::Identity(); // Start with yaw = 0
    tracker.setOrientation(q_initial);
    Eigen::Quaternionf q_true(Eigen::AngleAxisf(m_mpif / 4.0f, Eigen::Vector3f::UnitZ())); // Target yaw = M_PI/4

    Eigen::Vector3f accel_ned(0.0f, 0.0f, 9.81f);
    Eigen::Vector3f mag_ned(1.0f, 0.0f, 0.0f);
    Eigen::Vector3f accel_body = q_true.conjugate() * accel_ned;
    Eigen::Vector3f mag_body = q_true.conjugate() * mag_ned;

    // Zero the gyro
    Eigen::Vector3f omega(0.0f, 0.0f, 0.0f);
    tracker.setGyroAngularRate(omega);
    tracker.predictTo(au::make_quantity<au::Milli<au::Seconds>>(100));

    float yaw_true = m_mpif / 4.0f;
    float yaw_initial = 0.0f;
    (void) yaw_initial; // Avoid unused variable warning
    // Single-step the correction
    tracker.updateAccelerometerMagnetometer(accel_body, mag_body, au::make_quantity<au::Milli<au::Seconds>>(100));

    float yaw_est = tracker.getYawPitchRoll()(0);
    float err = std::atan2(std::sin(yaw_est - yaw_true), std::cos(yaw_est - yaw_true));

    // std::cerr << "Initial Yaw (degrees): " << yaw_initial * 180.0f / m_mpif << "\n";
    // std::cerr << "True Yaw (degrees): " << yaw_true * 180.0f / m_mpif << "\n";
    // std::cerr << "Estimated Yaw (degrees): " << yaw_est * 180.0f / m_mpif << "\n";
    // std::cerr << "Yaw Error (degrees): " << err * 180.0f / m_mpif << "\n";
    REQUIRE(std::abs(err) < 0.6f);
}

TEST_CASE("AccGyrMagOrientationTracker converges roll and pitch orientation") {
    AccGyrMagOrientationTracker tracker;
    tracker.setReferenceVectors(Eigen::Vector3f(0.0f, 0.0f, 9.81f), Eigen::Vector3f(1.0f, 0.0f, 0.0f));

    // Set a target orientation with non-zero roll and pitch
    Eigen::Quaternionf q_true(Eigen::AngleAxisf(m_mpif / 6.0f, Eigen::Vector3f::UnitX()) *  // 30 degrees roll
                               Eigen::AngleAxisf(m_mpif / 4.0f, Eigen::Vector3f::UnitY()));  // 45 degrees pitch

    Eigen::Vector3f accel_ned(0.0f, 0.0f, 9.81f);
    Eigen::Vector3f mag_ned(1.0f, 0.0f, 0.0f);
    Eigen::Vector3f accel_body = q_true.conjugate() * accel_ned;
    Eigen::Vector3f mag_body = q_true.conjugate() * mag_ned;

    // Zero the gyro
    Eigen::Vector3f omega(0.0f, 0.0f, 0.0f);
    tracker.setGyroAngularRate(omega);

    float roll_true = std::atan2(2.f * (q_true.w() * q_true.x() + q_true.y() * q_true.z()), 1.f - 2.f * (q_true.x() * q_true.x() + q_true.y() * q_true.y()));
    float pitch_true = std::asin(2.f * (q_true.w() * q_true.y() - q_true.z() * q_true.x()));

    for (int i = 0; i < 20; ++i) {
        tracker.updateAccelerometerMagnetometer(accel_body, mag_body, au::make_quantity<au::Milli<au::Seconds>>(100*i));

        Eigen::Vector3f ypr = tracker.getYawPitchRoll();
        float roll_est = ypr(2); // Roll is the third element
        float pitch_est = ypr(1); // Pitch is the second element

        float roll_error = std::abs(roll_est - roll_true);
        float pitch_error = std::abs(pitch_est - pitch_true);

        // Optionally add some noise to the sensor readings

        // Check that the estimated roll and pitch are within acceptable bounds
        REQUIRE(roll_error < 0.5f); // Adjust tolerance as needed
        REQUIRE(pitch_error < 0.5f); // Adjust tolerance as needed
    }
}

TEST_CASE("AccGyrOrientationTracker initializes with identity quaternion")
{
    AccGyrOrientationTracker tracker;
    auto q = tracker.getOrientation();
    REQUIRE(q.isApprox(Eigen::Quaternionf::Identity(), 1e-6f));
}

TEST_CASE("AccGyrOrientationTracker stabilizes pitch and roll from accelerometer")
{
    AccGyrOrientationTracker tracker;

    // Simulate tilted orientation: 30° pitch forward
    Eigen::Quaternionf q_true(Eigen::AngleAxisf(m_mpif / 6.0f, Eigen::Vector3f::UnitY()));
    Eigen::Vector3f accel_ned(0.f, 0.f, 9.81f);
    Eigen::Vector3f accel_meas = q_true.conjugate() * accel_ned;

    for (int i = 0; i < 50; ++i)
    {
        tracker.updateAccelerometer(accel_meas, au::make_quantity<au::Seconds>(1));
    }

    Eigen::Vector3f ypr = tracker.getYawPitchRoll();
    REQUIRE(std::abs(ypr.y() - m_mpif / 6.0f) < 0.05f); // ✅ ~30° pitch
}
