#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <array>

#include "Kalman.hpp" // Include the KalmanFilter class definition

TEST_CASE("Kalman Filter - 2D State, 1D Measurement")
{
    // Define the state and measurement sizes
    const int StateSize = 2;
    const int MeasurementSize = 1;

    // Define the matrices
    Eigen::Matrix<float, StateSize, StateSize> processNoiseCovarianceMatrix;
    processNoiseCovarianceMatrix << 0.01f, 0.0f,
        0.0f, 0.01f;

    Eigen::Matrix<float, MeasurementSize, MeasurementSize> measurementNoiseCovarianceMatrix;
    measurementNoiseCovarianceMatrix << 0.1f;

    Eigen::Matrix<float, StateSize, StateSize> stateCovarianceMatrix;
    stateCovarianceMatrix << 1.0f, 0.0f,
        0.0f, 1.0f;

    Eigen::Matrix<float, StateSize, 1> stateVector;
    stateVector << 0.0f, 0.0f;

    // Create the Kalman Filter
    KalmanFilter<StateSize, MeasurementSize> kf(processNoiseCovarianceMatrix,
                                                measurementNoiseCovarianceMatrix,
                                                stateCovarianceMatrix,
                                                stateVector);

    // Define the state transition matrix
    Eigen::Matrix<float, StateSize, StateSize> stateTransitionMatrix;
    stateTransitionMatrix << 1.0f, 1.0f,
        0.0f, 1.0f;

    // Define the measurement matrix
    Eigen::Matrix<float, MeasurementSize, StateSize> measurementMatrix;
    measurementMatrix << 1.0f, 0.0f;

    // Define the measurement vector
    Eigen::Matrix<float, MeasurementSize, 1> measurementVector;
    measurementVector << 1.0f;

    // Perform the prediction step
    kf.predict(stateTransitionMatrix);

    // Perform the update step
    kf.update(measurementMatrix, measurementVector);
    // std::cout << "Updated state estimate:\n" << kf.getState().transpose() << std::endl;

    // Get the state estimate
    Eigen::Matrix<float, StateSize, 1> stateEstimate = kf.getState();

    // Check the state estimate
    REQUIRE(stateEstimate(0) == doctest::Approx(0.952607f).epsilon(0.0001f));
    REQUIRE(stateEstimate(1) == doctest::Approx(0.473934f).epsilon(0.0001f));
}

TEST_CASE("1D Kalman Filter deterministic test with known ground truth (numerically stable)")
{
    constexpr int StateSize = 2;
    constexpr int MeasurementSize = 1;
    float dt = 1.0f;
    float accel = 1.0f;

    // State transition matrix
    Eigen::Matrix<float, StateSize, StateSize> A;
    A << 1.0f, dt,
        0.0f, 1.0f;

    // Control input vector (acceleration affects both position and velocity)
    Eigen::Matrix<float, StateSize, 1> B;
    B << 0.5f * dt * dt,
        dt;

    // Measurement matrix: only position is observed
    Eigen::Matrix<float, MeasurementSize, StateSize> H;
    H << 1.0f, 0.0f;

    // Small nonzero covariances to avoid division-by-zero
    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-6f;
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R;
    R << 1e-32f;

    // Initial state estimate and covariance
    Eigen::Matrix<float, StateSize, 1> x0;
    x0 << 0.0f, 0.0f;

    Eigen::Matrix<float, StateSize, StateSize> P0 = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-6f;

    // Instantiate Kalman Filter
    KalmanFilter<StateSize, MeasurementSize> kf(Q, R, P0, x0);

    // Perform 3 steps with known acceleration and clean measurements
    for (int i = 0; i < 3; ++i)
    {
        kf.predict(A);

        float t = static_cast<float>(i + 1);
        float true_position = 0.5f * accel * t * t;

        Eigen::Matrix<float, MeasurementSize, 1> z;
        z << true_position; // no noise for deterministic test

        kf.update(H, z);
    }

    auto est = kf.getState();
    // std::cout << "Final deterministic estimate:\n" << est.transpose() << std::endl;

    CHECK(est(0) == doctest::Approx(4.5f).epsilon(0.001f));     // position
    CHECK(est(1) == doctest::Approx(1.92857f).epsilon(0.001f)); // velocity would be 3 but we ignore acceleration!
}

TEST_CASE("1D Position-Velocity Kalman Filter with Accel & GPS")
{
    constexpr int StateSize = 2;
    constexpr int MeasurementSize = 1;
    float dt = 1.0f;    // 1 second timestep
    float accel = 1.0f; // constant acceleration

    // Dynamics
    Eigen::Matrix<float, StateSize, StateSize> A;
    A << 1.0f, dt,
        0.0f, 1.0f;

    Eigen::Matrix<float, StateSize, 1> B;
    B << 0.5f * dt * dt,
        dt;

    // Measurement (GPS: position only)
    Eigen::Matrix<float, MeasurementSize, StateSize> H;
    H << 1.0f, 0.0f;

    // Noise Covariances
    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 0.01f;
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R;
    R << 0.1f;

    // Initial estimates
    Eigen::Matrix<float, StateSize, StateSize> P0 = Eigen::Matrix<float, StateSize, StateSize>::Identity();
    Eigen::Matrix<float, StateSize, 1> x0;
    x0 << 0.0f, 0.0f;

    KalmanFilter<StateSize, MeasurementSize> kf(Q, R, P0, x0);

    // Simulate a few steps
    for (int i = 0; i < 5; ++i)
    {
        // Control input: constant acceleration
        Eigen::Matrix<float, StateSize, StateSize> A_i = A;
        kf.predict(A_i);

        // Simulated GPS with mild noise around x = 0.5·a·t²
        float true_pos = 0.5f * accel * dt * dt * static_cast<float>((i + 1) * (i + 1));
        Eigen::Matrix<float, MeasurementSize, 1> z;
        z << true_pos + (static_cast<float>(rand() % 100) / 1000.0f - 0.05f); // add noise

        kf.update(H, z);
    }

    auto est = kf.getState();
    // std::cout << "Final estimated state:\n" << est.transpose() << std::endl;

    CHECK(est(0) > 0.0f);
    CHECK(est(1) > 0.0f);
}

TEST_CASE("Kalman Filter estimates acceleration from position-only measurements")
{
    constexpr int StateSize = 3;
    constexpr int MeasurementSize = 1;
    float dt = 1.0f;
    float true_accel = 1.0f;

    // State transition matrix (includes acceleration)
    Eigen::Matrix<float, StateSize, StateSize> A;
    A << 1.0f, dt, 0.5f * dt * dt,
        0.0f, 1.0f, dt,
        0.0f, 0.0f, 1.0f;

    // Measurement matrix (position only)
    Eigen::Matrix<float, MeasurementSize, StateSize> H;
    H << 1.0f, 0.0f, 0.0f;

    // Very low noise for ideal test
    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-6f;
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R;
    R << 1e-6f;

    // Initial state: unknown everything
    Eigen::Matrix<float, StateSize, 1> x0;
    x0 << 0.0f, 0.0f, 0.0f;

    // Slight uncertainty to avoid NaNs
    Eigen::Matrix<float, StateSize, StateSize> P0 = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-6f;

    KalmanFilter<StateSize, MeasurementSize> kf(Q, R, P0, x0);

    // Perform multiple steps with perfect position data
    for (int i = 0; i < 5; ++i)
    {
        float t = static_cast<float>(i + 1);
        float true_pos = 0.5f * true_accel * t * t;

        Eigen::Matrix<float, MeasurementSize, 1> z;
        z << true_pos; // no noise

        kf.predict(A);
        kf.update(H, z);
    }

    auto est = kf.getState();
    // std::cout << "Final estimated state:\n" << est.transpose() << std::endl;

    CHECK(est(0) == doctest::Approx(12.5f).epsilon(0.05f)); // position at t=5
    CHECK(est(1) == doctest::Approx(5.0f).epsilon(0.05f));  // velocity at t=5
    CHECK(est(2) == doctest::Approx(1.0f).epsilon(0.05f));  // estimated acceleration
}

TEST_CASE("3D Position-Velocity Kalman Filter with Accel & GPS")
{
    constexpr int StateSize = 6;       // [px, py, pz, vx, vy, vz]
    constexpr int MeasurementSize = 3; // GPS measures [px, py, pz]
    float dt = 1.0f;                   // timestep

    // State transition matrix A (6x6)
    Eigen::Matrix<float, StateSize, StateSize> A = Eigen::Matrix<float, StateSize, StateSize>::Identity();
    for (int i = 0; i < 3; ++i)
    {
        A(i, i + 3) = dt; // Position updated by velocity
    }

    // Control matrix B (6x3) – acceleration input
    Eigen::Matrix<float, StateSize, 3> B;
    B.setZero();
    for (int i = 0; i < 3; ++i)
    {
        B(i, i) = 0.5f * dt * dt; // affect on position
        B(i + 3, i) = dt;         // affect on velocity
    }

    // Measurement matrix H (3x6) – GPS observes position
    Eigen::Matrix<float, MeasurementSize, StateSize> H = Eigen::Matrix<float, MeasurementSize, StateSize>::Zero();
    H(0, 0) = 1.0f;
    H(1, 1) = 1.0f;
    H(2, 2) = 1.0f;

    // Process noise (Q) and measurement noise (R)
    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 0.01f;
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R = Eigen::Matrix<float, MeasurementSize, MeasurementSize>::Identity() * 0.1f;

    // Initial state
    Eigen::Matrix<float, StateSize, 1> x0;
    x0.setZero();

    Eigen::Matrix<float, StateSize, StateSize> P0 = Eigen::Matrix<float, StateSize, StateSize>::Identity();

    KalmanFilter<StateSize, MeasurementSize> kf(Q, R, P0, x0);

    // Simulate 5 steps with constant acceleration [1, 0.5, -1]
    Eigen::Vector3f accel(1.0f, 0.5f, -1.0f);
    for (int i = 0; i < 5; ++i)
    {
        // Predict with A
        kf.predict(A);

        // Simulated GPS position (noisy)
        float t = static_cast<float>(i + 1);
        Eigen::Vector3f true_pos = 0.5f * accel * t * t;
        Eigen::Vector3f noise;
        noise << static_cast<float>(rand() % 100 - 50) / 1000.0f,
            static_cast<float>(rand() % 100 - 50) / 1000.0f,
            static_cast<float>(rand() % 100 - 50) / 1000.0f;

        Eigen::Matrix<float, MeasurementSize, 1> z = true_pos + noise;

        kf.update(H, z);
    }

    Eigen::Matrix<float, StateSize, 1> est = kf.getState();
    // std::cout << "Final estimated 3D state:\n" << est.transpose() << std::endl;

    CHECK(est(0) > 0.0f); // x
    CHECK(est(1) > 0.0f); // y
    CHECK(est(2) < 0.0f); // z
}

TEST_CASE("3D Position-Velocity Kalman Filter with Repeated Accel & GPS Updates")
{
    constexpr int StateSize = 6;       // [px, py, pz, vx, vy, vz]
    constexpr int MeasurementSize = 3; // GPS measures [px, py, pz]
    float dt = 1.0f;                   // 1 second timestep

    // Acceleration vector (constant in this example)
    Eigen::Vector3f accel(1.0f, 0.5f, -1.0f);

    // State transition matrix A (6x6)
    Eigen::Matrix<float, StateSize, StateSize> A = Eigen::Matrix<float, StateSize, StateSize>::Identity();
    for (int i = 0; i < 3; ++i)
    {
        A(i, i + 3) = dt; // Position updated by velocity
    }

    // Control matrix B (6x3) – acceleration input
    Eigen::Matrix<float, StateSize, 3> B = Eigen::Matrix<float, StateSize, 3>::Zero();
    for (int i = 0; i < 3; ++i)
    {
        B(i, i) = 0.5f * dt * dt;
        B(i + 3, i) = dt;
    }

    // Measurement matrix H (3x6) – GPS observes position only
    Eigen::Matrix<float, MeasurementSize, StateSize> H = Eigen::Matrix<float, MeasurementSize, StateSize>::Zero();
    H(0, 0) = 1.0f;
    H(1, 1) = 1.0f;
    H(2, 2) = 1.0f;

    // Process noise (Q) and measurement noise (R)
    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 0.01f;
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R = Eigen::Matrix<float, MeasurementSize, MeasurementSize>::Identity() * 0.1f;

    // Initial state and covariance
    Eigen::Matrix<float, StateSize, 1> x0 = Eigen::Matrix<float, StateSize, 1>::Zero();
    Eigen::Matrix<float, StateSize, StateSize> P0 = Eigen::Matrix<float, StateSize, StateSize>::Identity();

    KalmanFilter<StateSize, MeasurementSize> kf(Q, R, P0, x0);

    // std::cout << "=== Simulating 10 time steps ===" << std::endl;
    for (int i = 0; i < 10; ++i)
    {
        float t = static_cast<float>(i + 1);

        // Simulate GPS position with added noise
        Eigen::Vector3f true_pos = 0.5f * accel * t * t;
        Eigen::Vector3f noise;
        noise << static_cast<float>(rand() % 100 - 50) / 1000.0f,
            static_cast<float>(rand() % 100 - 50) / 1000.0f,
            static_cast<float>(rand() % 100 - 50) / 1000.0f;

        Eigen::Matrix<float, MeasurementSize, 1> z = true_pos + noise;

        // Predict next state
        kf.predict(A);

        // Update with noisy GPS measurement
        kf.update(H, z);

        // auto state = kf.getState();
        // std::cout << "Step " << i + 1 << " estimate:\n"
        //           << state.transpose() << "\n"
        //           << std::endl;
    }

    // Final state should show motion in +x, +y, -z directions
    Eigen::Matrix<float, StateSize, 1> final = kf.getState();
    CHECK(final(0) > 0.0f); // px
    CHECK(final(1) > 0.0f); // py
    CHECK(final(2) < 0.0f); // pz
    CHECK(final(3) > 0.0f); // vx
    CHECK(final(4) > 0.0f); // vy
    CHECK(final(5) < 0.0f); // vz
}

TEST_CASE("3D Kalman Filter estimates acceleration from GPS-only measurements")
{
    constexpr int StateSize = 9;       // [p, v, a] in x, y, z
    constexpr int MeasurementSize = 3; // GPS: x, y, z
    float dt = 1.0f;

    // Simulated true constant acceleration
    Eigen::Vector3f true_accel(1.0f, 0.5f, -0.8f);

    // Build state transition matrix A (9x9)
    Eigen::Matrix<float, StateSize, StateSize> A = Eigen::Matrix<float, StateSize, StateSize>::Identity();
    for (int i = 0; i < 3; ++i)
    {
        A(i, i + 3) = dt;
        A(i, i + 6) = 0.5f * dt * dt;
        A(i + 3, i + 6) = dt;
    }

    // Measurement matrix H: observe only position
    Eigen::Matrix<float, MeasurementSize, StateSize> H = Eigen::Matrix<float, MeasurementSize, StateSize>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();

    // Process and measurement noise (very small for ideal case)
    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-6f;
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R = Eigen::Matrix3f::Identity() * 1e-6f;

    // Initial state and covariance
    Eigen::Matrix<float, StateSize, 1> x0 = Eigen::Matrix<float, StateSize, 1>::Zero();
    Eigen::Matrix<float, StateSize, StateSize> P0 = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-6f;

    KalmanFilter<StateSize, MeasurementSize> kf(Q, R, P0, x0);

    // Run 5 steps with perfect position observations
    for (int i = 0; i < 5; ++i)
    {
        float t = static_cast<float>(i + 1);
        Eigen::Vector3f pos = 0.5f * true_accel * t * t;

        Eigen::Matrix<float, MeasurementSize, 1> z;
        z << pos;

        kf.predict(A);
        kf.update(H, z);
    }

    Eigen::Matrix<float, StateSize, 1> est = kf.getState();
    // std::cout << "Estimated state:\n" << est.transpose() << std::endl;

    // Expected values at t = 5
    Eigen::Vector3f expected_p = 0.5f * true_accel * 25.0f; // t² = 25
    Eigen::Vector3f expected_v = true_accel * 5.0f;
    Eigen::Vector3f expected_a = true_accel;

    CHECK(est.segment<3>(0).isApprox(expected_p, 0.01f)); // position
    CHECK(est.segment<3>(3).isApprox(expected_v, 0.01f)); // velocity
    CHECK(est.segment<3>(6).isApprox(expected_a, 0.03f)); // acceleration
}

TEST_CASE("9D Kalman fuses sparse GPS updates")
{
    constexpr int StateSize = 9;
    constexpr int MeasSizeGps = 3; // GPS: position only
    constexpr float dt = 1.0f;

    Eigen::Vector3f true_accel(1.0f, 0.5f, -0.8f);

    // State transition matrix A for 9D: p, v, a
    Eigen::Matrix<float, StateSize, StateSize> A = Eigen::Matrix<float, StateSize, StateSize>::Identity();
    for (int i = 0; i < 3; ++i)
    {
        A(i, i + 3) = dt;
        A(i, i + 6) = 0.5f * dt * dt;
        A(i + 3, i + 6) = dt;
    }

    // Control matrix B: only affects acceleration entries
    Eigen::Matrix<float, StateSize, 3> B = Eigen::Matrix<float, StateSize, 3>::Zero();
    B.block<3, 3>(6, 0) = Eigen::Matrix3f::Identity(); // accel input u updates a

    // Measurement matrix for GPS
    Eigen::Matrix<float, MeasSizeGps, StateSize> H_gps = Eigen::Matrix<float, MeasSizeGps, StateSize>::Zero();
    H_gps.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();

    // Noise covariances
    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-4f;
    Eigen::Matrix<float, MeasSizeGps, MeasSizeGps> R_gps = Eigen::Matrix3f::Identity() * 5e-3f;

    Eigen::Matrix<float, StateSize, 1> x0 = Eigen::Matrix<float, StateSize, 1>::Zero();
    Eigen::Matrix<float, StateSize, StateSize> P0 = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-3f;

    KalmanFilter<StateSize, MeasSizeGps> kf(Q, R_gps, P0, x0);

    for (int i = 0; i < 10; ++i)
    {
        float t = static_cast<float>(i + 1);

        // Ground truth position
        Eigen::Vector3f true_pos = 0.5f * true_accel * t * t;

        kf.predict(A);

        // Intermittent GPS
        if (i % 3 == 0)
        {
            Eigen::Vector3f gps_meas = true_pos + Eigen::Vector3f::Random() * 0.05f;
            kf.update(H_gps, gps_meas);
            // std::cout << "[t=" << t << "] GPS fused: " << gps_meas.transpose() << "\n";
        }
        else
        {
            // std::cout << "[t=" << t << "] GPS dropout\n";
        }

        auto est = kf.getState();
        // std::cout << "  est pos: " << est.segment<3>(0).transpose()
        //           << " | vel: " << est.segment<3>(3).transpose()
        //           << " | acc: " << est.segment<3>(6).transpose() << "\n";

        // Optional check: Position improves after GPS step
        if (i == 9)
        {
            Eigen::Vector3f expected_pos = 0.5f * true_accel * t * t;
            Eigen::Vector3f expected_vel = true_accel * t;
            Eigen::Vector3f expected_acc = true_accel;

            for (int j = 0; j < 3; ++j)
            {
                CHECK(est.segment<3>(0)(j) == doctest::Approx(expected_pos(j)).epsilon(0.15f));
                CHECK(est.segment<3>(3)(j) == doctest::Approx(expected_vel(j)).epsilon(0.1f));
                CHECK(est.segment<3>(6)(j) == doctest::Approx(expected_acc(j)).epsilon(0.05f));
            }
        }
    }
}

TEST_CASE("9D Kalman fuses acceleration inputs as measurement and sparse GPS updates")
{
    constexpr int StateSize = 9;
    constexpr int MeasSizeGps = 3; // GPS: position only
    constexpr float dt = 1.0f;

    Eigen::Vector3f true_accel(1.0f, 0.5f, -0.8f);

    // State transition matrix A for 9D: p, v, a
    Eigen::Matrix<float, StateSize, StateSize> A = Eigen::Matrix<float, StateSize, StateSize>::Identity();
    for (int i = 0; i < 3; ++i)
    {
        A(i, i + 3) = dt;
        A(i, i + 6) = 0.5f * dt * dt;
        A(i + 3, i + 6) = dt;
    }

    // GPS measurement matrix: position only
    Eigen::Matrix<float, MeasSizeGps, StateSize> H_gps = Eigen::Matrix<float, MeasSizeGps, StateSize>::Zero();
    H_gps.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();

    // Acceleration measurement matrix: acceleration only
    Eigen::Matrix<float, 3, StateSize> H_acc = Eigen::Matrix<float, 3, StateSize>::Zero();
    H_acc.block<3, 3>(0, 6) = Eigen::Matrix3f::Identity();

    // Noise covariances
    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-4f;
    Eigen::Matrix3f R_gps = Eigen::Matrix3f::Identity() * 5e-3f;
    Eigen::Matrix3f R_accel = Eigen::Matrix3f::Identity() * 1e-2f;

    Eigen::Matrix<float, StateSize, 1> x0 = Eigen::Matrix<float, StateSize, 1>::Zero();
    Eigen::Matrix<float, StateSize, StateSize> P0 = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-3f;

    KalmanFilter<StateSize, MeasSizeGps> kf(Q, R_gps, P0, x0);

    for (int i = 0; i < 10; ++i)
    {
        float t = static_cast<float>(i + 1);

        // Simulate noisy accel measurement and low-pass filter
        Eigen::Vector3f accel_meas = true_accel + Eigen::Vector3f::Random() * 0.02f;

        // Simulate true position
        Eigen::Vector3f true_pos = 0.5f * true_accel * t * t;

        // Predict using process model
        kf.predict(A);

        // Update using acceleration as a measurement
        KalmanFilter<StateSize, 3> accelKF(Q, R_accel, kf.stateCovarianceMatrix, kf.stateVector);
        accelKF.update(H_acc, accel_meas);
        kf.stateVector = accelKF.stateVector;
        kf.stateCovarianceMatrix = accelKF.stateCovarianceMatrix;

        // Intermittent GPS updates
        if (i % 3 == 0)
        {
            Eigen::Vector3f gps_meas = true_pos + Eigen::Vector3f::Random() * 0.05f;
            kf.update(H_gps, gps_meas);
            // std::cout << "[t=" << t << "] GPS fused:  " << gps_meas.transpose() << "\n";
        }
        else
        {
            // std::cout << "[t=" << t << "] GPS dropout\n";
        }

        auto est = kf.getState();
        // std::cout << "  est pos: " << est.segment<3>(0).transpose()
        //           << " | vel: " << est.segment<3>(3).transpose()
        //           << " | acc: " << est.segment<3>(6).transpose() << "\n";

        if (i == 9)
        {
            Eigen::Vector3f expected_pos = 0.5f * true_accel * t * t;
            Eigen::Vector3f expected_vel = true_accel * t;
            Eigen::Vector3f expected_acc = true_accel;

            for (int j = 0; j < 3; ++j)
            {
                CHECK(est.segment<3>(0)(j) == doctest::Approx(expected_pos(j)).epsilon(0.15f));
                CHECK(est.segment<3>(3)(j) == doctest::Approx(expected_vel(j)).epsilon(0.1f));
                CHECK(est.segment<3>(6)(j) == doctest::Approx(expected_acc(j)).epsilon(0.05f));
            }
        }
    }
}
