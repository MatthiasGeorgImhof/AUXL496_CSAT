#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <array>

#include "Kalman.hpp" // Include the KalmanFilter class definition


TEST_CASE("1D Kalman Filter fuses mock SGP4 prediction with noisy GPS: 5 steps") {
    constexpr int StateSize = 2;         // [position, velocity]
    constexpr int MeasurementSize = 1;   // GPS measures position

    // Measurement model: only position observed
    Eigen::Matrix<float, MeasurementSize, StateSize> H;
    H << 1.0f, 0.0f;

    // GPS measurement noise covariance
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R;
    R << 0.01f;  // reasonable GPS std ~ 0.1m

    // Assumed prediction (SGP4) uncertainty
    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 0.005f;

    // Initialize Kalman filter (we'll reassign state at each step)
    Eigen::Matrix<float, StateSize, 1> dummyState = Eigen::Matrix<float, StateSize, 1>::Zero();
    KalmanFilter<StateSize, MeasurementSize> kf(Q, R, Q, dummyState);

    std::vector<float> times = {1, 2, 3, 4, 5};
    std::vector<float> gps_noise = {0.05f, -0.02f, 0.03f, -0.01f, 0.00f};  // fixed for reproducibility

    for (size_t i = 0; i < times.size(); ++i) {
        float t = times[i];

        // Mock SGP4 prediction: sin(t) for position, cos(t) for velocity
        float predicted_p = std::sin(t);
        float predicted_v = std::cos(t);
        Eigen::Matrix<float, StateSize, 1> sgp4_pred;
        sgp4_pred << predicted_p, predicted_v;

        // Assign predicted state and known covariance
        kf.stateVector = sgp4_pred;
        kf.stateCovarianceMatrix = Q;

        // Noisy GPS measurement of position
        Eigen::Matrix<float, MeasurementSize, 1> z;
        z << std::sin(t) + gps_noise[i];

        kf.update(H, z);

        auto est = kf.getState();
        // std::cout << "Time t=" << t << " → "
        //           << "SGP4 p=" << predicted_p << ", "
        //           << "GPS z=" << z(0) << ", "
        //           << "Fused p=" << est(0) << ", v=" << est(1) << std::endl;

        // Validate fused position stays close to SGP4 and measurement
        CHECK(std::abs(est(0) - std::sin(t)) < 0.05f);
        CHECK(std::abs(est(1) - std::cos(t)) < 0.2f); // allow more wiggle for v
    }
}

TEST_CASE("1D Kalman Filter fuses mock SGP4 prediction with noisy GPS: 10 steps") {
    constexpr int StateSize = 2;         // [position, velocity]
    constexpr int MeasurementSize = 1;   // GPS measures position

    // Measurement model: only position observed
    Eigen::Matrix<float, MeasurementSize, StateSize> H;
    H << 1.0f, 0.0f;

    // GPS measurement noise covariance
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R;
    R << 0.01f;  // reasonable GPS std ~ 0.1m

    // Assumed prediction (SGP4) uncertainty
    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 0.005f;

    // Initialize Kalman filter (we'll reassign state at each step)
    Eigen::Matrix<float, StateSize, 1> dummyState = Eigen::Matrix<float, StateSize, 1>::Zero();
    KalmanFilter<StateSize, MeasurementSize> kf(Q, R, Q, dummyState);

    std::vector<float> times = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<float> gps_noise = {0.05f, -0.02f, 0.03f, -0.01f, 0.05f, -0.1, 0.04, 0.15f, -0.2, -0.03};  // fixed for reproducibility

    for (size_t i = 0; i < times.size(); ++i) {
        float t = times[i];

        // Mock SGP4 prediction: sin(t) for position, cos(t) for velocity
        float predicted_p = std::sin(t);
        float predicted_v = std::cos(t);
        Eigen::Matrix<float, StateSize, 1> sgp4_pred;
        sgp4_pred << predicted_p, predicted_v;

        // Assign predicted state and known covariance
        kf.stateVector = sgp4_pred;
        kf.stateCovarianceMatrix = Q;

        // Noisy GPS measurement of position
        Eigen::Matrix<float, MeasurementSize, 1> z;
        z << std::sin(t) + gps_noise[i];

        kf.update(H, z);

        auto est = kf.getState();
        // std::cout << "Time t=" << t << " → "
        //           << "SGP4 p=" << predicted_p << ", "
        //           << "GPS z=" << z(0) << ", "
        //           << "Fused p=" << est(0) << ", v=" << est(1) << std::endl;

        // Validate fused position stays close to SGP4 and measurement
        CHECK(std::abs(est(0) - std::sin(t)) < 0.1f);
        CHECK(std::abs(est(1) - std::cos(t)) < 0.2f); // allow more wiggle for v
    }
}

TEST_CASE("1D Kalman Filter corrects phase-lagged SGP4 using noisy GPS") {
    constexpr int StateSize = 2;        // [position, velocity]
    constexpr int MeasurementSize = 1;  // GPS: position only

    float omega = 0.5f;
    float phase_lag = 0.5f;  // bias in the predictor (SGP4 is behind)

    // Measurement matrix: observe position only
    Eigen::Matrix<float, MeasurementSize, StateSize> H;
    H << 1.0f, 0.0f;

    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R;
    R << 0.01f;

    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 0.005f;

    Eigen::Matrix<float, StateSize, 1> dummy = Eigen::Matrix<float, StateSize, 1>::Zero();
    KalmanFilter<StateSize, MeasurementSize> kf(Q, R, Q, dummy);

    for (int step = 0; step < 10; ++step) {
        float t = static_cast<float>(step + 1);
        float true_pos = std::sin(omega * t);
        float true_vel = omega * std::cos(omega * t);

        // SGP4 prediction is phase-lagged
        float biased_t = t - phase_lag;
        float pred_pos = std::sin(omega * biased_t);
        float pred_vel = omega * std::cos(omega * biased_t);

        Eigen::Matrix<float, StateSize, 1> sgp4_pred;
        sgp4_pred << pred_pos, pred_vel;

        // Noisy GPS measurement
        float noise = static_cast<float>(rand() % 100 - 50) / 1000.0f;
        float z = true_pos + noise;

        // Apply SGP4 as prior
        kf.stateVector = sgp4_pred;
        kf.stateCovarianceMatrix = Q;

        Eigen::Matrix<float, MeasurementSize, 1> z_meas;
        z_meas << z;

        kf.update(H, z_meas);
        auto est = kf.getState();

        // std::cout << "t=" << t
        //           << " | true=" << true_pos
        //           << " | SGP4=" << pred_pos
        //           << " | GPS=" << z
        //           << " | fused=" << est(0)
        //           << " | v=" << est(1) << "\n";

        CHECK(std::abs(est(0) - true_pos) < 0.2f);  // Position tracks truth
        CHECK(std::abs(est(1) - true_vel) < 0.2f);  // Position tracks truth
    }
}

TEST_CASE("1D Kalman Filter corrects phase-lagged SGP4 tuned to using noisy GPS") {
    constexpr int StateSize = 2;        // [position, velocity]
    constexpr int MeasurementSize = 1;  // GPS: position only

    float omega = 0.5f;
    float phase_lag = 0.5f;  // bias in the predictor (SGP4 is behind)

    // Measurement matrix: observe position only
    Eigen::Matrix<float, MeasurementSize, StateSize> H;
    H << 1.0f, 0.0f;

    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R;
    R << 0.001f;

    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 0.02f;

    Eigen::Matrix<float, StateSize, 1> dummy = Eigen::Matrix<float, StateSize, 1>::Zero();
    KalmanFilter<StateSize, MeasurementSize> kf(Q, R, Q, dummy);

    for (int step = 0; step < 10; ++step) {
        float t = static_cast<float>(step + 1);
        float true_pos = std::sin(omega * t);
        float true_vel = omega * std::cos(omega * t);

        // SGP4 prediction is phase-lagged
        float biased_t = t - phase_lag;
        float pred_pos = std::sin(omega * biased_t);
        float pred_vel = omega * std::cos(omega * biased_t);

        Eigen::Matrix<float, StateSize, 1> sgp4_pred;
        sgp4_pred << pred_pos, pred_vel;

        // Noisy GPS measurement
        float noise = static_cast<float>(rand() % 100 - 50) / 1000.0f;
        float z = true_pos + noise;

        // Apply SGP4 as prior
        kf.stateVector = sgp4_pred;
        kf.stateCovarianceMatrix = Q;

        Eigen::Matrix<float, MeasurementSize, 1> z_meas;
        z_meas << z;

        kf.update(H, z_meas);
        auto est = kf.getState();

        // std::cout << "t=" << t
        //           << " | true=" << true_pos
        //           << " | SGP4=" << pred_pos
        //           << " | GPS=" << z
        //           << " | fused=" << est(0)
        //           << " | v=" << est(1) << "\n";

        CHECK(std::abs(est(0) - true_pos) < 0.15f);  // Position tracks truth
        CHECK(std::abs(est(1) - true_vel) < 0.15f);  // Position tracks truth
    }
}

TEST_CASE("3D Kalman Filter fuses mock SGP4 with noisy GPS position") {
    constexpr int StateSize = 6;
    constexpr int MeasurementSize = 3;

    // SGP4 prediction uncertainty
    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 0.01f;

    // GPS measurement noise
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R = Eigen::Matrix3f::Identity() * 0.1f;

    // Measurement matrix: GPS measures position only
    Eigen::Matrix<float, MeasurementSize, StateSize> H = Eigen::Matrix<float, MeasurementSize, StateSize>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();

    // Initial placeholder values
    Eigen::Matrix<float, StateSize, 1> x0 = Eigen::Matrix<float, StateSize, 1>::Zero();

    KalmanFilter<StateSize, MeasurementSize> kf(Q, R, Q, x0);

    for (int step = 0; step < 5; ++step) {
        float t = static_cast<float>(step + 1);
        float omega = 0.5f;

        // Mock SGP4 predicted position and velocity
        Eigen::Vector3f pos_sgp4(std::sin(omega * t),
                                 std::cos(omega * t),
                                 std::sin(omega * t) * std::cos(omega * t));
        Eigen::Vector3f vel_sgp4(omega * std::cos(omega * t),
                                -omega * std::sin(omega * t),
                                 omega * (std::cos(2 * omega * t)));

        Eigen::Matrix<float, StateSize, 1> sgp4_pred;
        sgp4_pred << pos_sgp4, vel_sgp4;

        // Simulated noisy GPS measurement
        Eigen::Vector3f gps_noise;
        gps_noise << static_cast<float>(rand() % 100 - 50) / 1000.0f,
                     static_cast<float>(rand() % 100 - 50) / 1000.0f,
                     static_cast<float>(rand() % 100 - 50) / 1000.0f;

        Eigen::Matrix<float, MeasurementSize, 1> z = pos_sgp4 + gps_noise;

        // Set state and covariance as SGP4 prior
        kf.stateVector = sgp4_pred;
        kf.stateCovarianceMatrix = Q;

        kf.update(H, z);

        auto fused = kf.getState();

        // std::cout << "Step " << step + 1
        //           << " — SGP4 pos: " << pos_sgp4.transpose()
        //           << ", GPS z: " << z.transpose()
        //           << ", Fused pos: " << fused.head<3>().transpose()
        //           << ", vel: " << fused.tail<3>().transpose()
        //           << std::endl;

        CHECK((fused.head<3>() - pos_sgp4).norm() < 0.1f); // Position close to true
        CHECK((fused.tail<3>() - vel_sgp4).norm() < 0.2f); // Velocity also tracked
    }
}


TEST_CASE("3D Kalman Filter with intermittent GPS updates and SGP4 prior") {
    constexpr int StateSize = 6;       // [x, y, z, vx, vy, vz]
    constexpr int MeasurementSize = 3; // GPS: position only
    float omega = 0.5f;

    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 0.01f;
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R = Eigen::Matrix3f::Identity() * 0.1f;

    Eigen::Matrix<float, MeasurementSize, StateSize> H = Eigen::Matrix<float, MeasurementSize, StateSize>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();

    Eigen::Matrix<float, StateSize, 1> x0 = Eigen::Matrix<float, StateSize, 1>::Zero();
    KalmanFilter<StateSize, MeasurementSize> kf(Q, R, Q, x0);

    for (int step = 0; step < 10; ++step) {
        float t = static_cast<float>(step + 1);

        // True SGP4-based prediction
        Eigen::Vector3f p = {
            std::sin(omega * t),
            std::cos(omega * t),
            std::sin(omega * t) * std::cos(omega * t)
        };
        Eigen::Vector3f v = {
            omega * std::cos(omega * t),
           -omega * std::sin(omega * t),
            omega * std::cos(2.0f * omega * t)
        };

        Eigen::Matrix<float, StateSize, 1> sgp4_pred;
        sgp4_pred << p, v;

        // Assign SGP4 prediction as the prior
        kf.stateVector = sgp4_pred;
        kf.stateCovarianceMatrix = Q;

        // std::cout << "Step " << step + 1 << " — SGP4 pos: " << p.transpose();

        if (step % 2 == 0) {
            // Simulate a GPS update
            Eigen::Vector3f noise;
            noise << static_cast<float>(rand() % 100 - 50) / 1000.0f,
                     static_cast<float>(rand() % 100 - 50) / 1000.0f,
                     static_cast<float>(rand() % 100 - 50) / 1000.0f;

            Eigen::Matrix<float, MeasurementSize, 1> z = p + noise;
            kf.update(H, z);
            // std::cout << " | GPS: " << z.transpose();
        } else {
            // std::cout << " | GPS: --- (no update)";
        }

        auto fused = kf.getState();
        // std::cout << " | Fused pos: " << fused.head<3>().transpose()
        //           << " | vel: " << fused.tail<3>().transpose() << "\n";

        // Optionally loosen checks during dropout
        if (step % 2 == 0) {
            CHECK((fused.head<3>() - p).norm() < 0.1f);
            CHECK((fused.tail<3>() - v).norm() < 0.2f);
        }
    }
}

TEST_CASE("3D Kalman corrects systematic bias in SGP4 prediction using noisy GPS") {
    constexpr int StateSize = 6;       // [px, py, pz, vx, vy, vz]
    constexpr int MeasurementSize = 3; // GPS: position only
    float omega = 0.5f;
    float phase_lag = 0.5f;  // predictor lags behind reality

    // Covariances
    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 0.01f;  // Less trust in SGP4
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R = Eigen::Matrix3f::Identity() * 0.005f;                // More trust in GPS

    // Measurement matrix
    Eigen::Matrix<float, MeasurementSize, StateSize> H = Eigen::Matrix<float, MeasurementSize, StateSize>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();

    // Init filter
    Eigen::Matrix<float, StateSize, 1> x0 = Eigen::Matrix<float, StateSize, 1>::Zero();
    KalmanFilter<StateSize, MeasurementSize> kf(Q, R, Q, x0);

    for (int step = 0; step < 10; ++step) {
        float t = static_cast<float>(step + 1);
        float t_biased = t - phase_lag;

        // Ground truth position and velocity
        Eigen::Vector3f p_true(std::sin(omega * t),
                               std::cos(omega * t),
                               std::sin(omega * t) * std::cos(omega * t));
        Eigen::Vector3f v_true(omega * std::cos(omega * t),
                              -omega * std::sin(omega * t),
                               omega * std::cos(2 * omega * t));

        // Biased SGP4 prediction (phase-lagged)
        Eigen::Vector3f p_sgp4(std::sin(omega * t_biased),
                               std::cos(omega * t_biased),
                               std::sin(omega * t_biased) * std::cos(omega * t_biased));
        Eigen::Vector3f v_sgp4(omega * std::cos(omega * t_biased),
                              -omega * std::sin(omega * t_biased),
                               omega * std::cos(2 * omega * t_biased));

        Eigen::Matrix<float, StateSize, 1> sgp4_pred;
        sgp4_pred << p_sgp4, v_sgp4;

        // Noisy GPS measurement
        Eigen::Vector3f gps_noise;
        gps_noise << static_cast<float>(rand() % 100 - 50) / 1000.0f,
                     static_cast<float>(rand() % 100 - 50) / 1000.0f,
                     static_cast<float>(rand() % 100 - 50) / 1000.0f;

        Eigen::Matrix<float, MeasurementSize, 1> z = p_true + gps_noise;

        // Apply biased predictor
        kf.stateVector = sgp4_pred;
        kf.stateCovarianceMatrix = Q;

        kf.update(H, z);

        auto est = kf.getState();

        // std::cout << "t=" << t
        //           << " | true pos: " << p_true.transpose()
        //           << " | SGP4: " << p_sgp4.transpose()
        //           << " | GPS: " << z.transpose()
        //           << " | fused pos: " << est.head<3>().transpose()
        //           << " | vel: " << est.tail<3>().transpose() << "\n";

        // Assert position is closer to true than to biased prediction
        float err_to_true  = (est.head<3>() - p_true).norm();
        float err_to_sgp4  = (est.head<3>() - p_sgp4).norm();
        CHECK(err_to_true < err_to_sgp4);
        CHECK(err_to_true < 0.15);   // should track truth tightly
    }
}

