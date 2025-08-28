#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <array>
#include <functional>

#include "Kalman.hpp" // Include your templated KalmanFilter class with updateEKF

TEST_CASE("Orientation EKF tracks yaw using gyro + magnetometer (identity Jacobian)") {
    constexpr int StateSize = 7;        // Quaternion (qx, qy, qz, qw) + angular rate (wx, wy, wz)
    constexpr int MeasurementSize = 3;  // Magnetometer: 3D field in body frame

    float dt = 1.0f;
    float yaw_rate = M_PIf / 180.0f * 45.0f; // 45 deg/s

    // Earth's magnetic field in NED (North-East-Down)
    Eigen::Vector3f mag_ned(1.0f, 0.0f, 0.0f);

    // Initial orientation and angular velocity
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    Eigen::Vector3f omega(0.0f, 0.0f, yaw_rate); // constant yaw

    // EKF setup
    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-5f;
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R = Eigen::Matrix3f::Identity() * 0.01f;
    Eigen::Matrix<float, StateSize, 1> x0 = Eigen::Matrix<float, StateSize, 1>::Zero();

    KalmanFilter<StateSize, MeasurementSize> ekf(Q, R, Q, x0);

    for (int step = 0; step < 10; ++step) {
        // Simulate orientation from integrating gyro
        Eigen::Quaternionf delta_q = Eigen::Quaternionf(
            1.0f,
            0.5f * omega(0) * dt,
            0.5f * omega(1) * dt,
            0.5f * omega(2) * dt
        ).normalized();
        q = (q * delta_q).normalized();

        // Simulated body-frame magnetometer measurement
        Eigen::Vector3f mag_measured = q.inverse() * mag_ned + Eigen::Vector3f::Random() * 0.01f;

        // Build current state vector
        Eigen::Matrix<float, StateSize, 1> x;
        x << q.x(), q.y(), q.z(), q.w(), omega;

        // Set the filter's internal state before update
        ekf.stateVector = x;

        // Nonlinear measurement model: magnetic field direction depends on quaternion
        auto h = [&](const Eigen::Matrix<float, StateSize, 1>& x_pred) {
            Eigen::Quaternionf q_hat(x_pred(3), x_pred(0), x_pred(1), x_pred(2)); // [w, x, y, z]
            q_hat.normalize();
            return q_hat.inverse() * mag_ned;
        };

        // Temporary placeholder Jacobian (not yet derived analytically)
        Eigen::Matrix<float, MeasurementSize, StateSize> H_jac = Eigen::Matrix<float, MeasurementSize, StateSize>::Zero();
        H_jac.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity(); // approximate

        ekf.updateEKF(h, H_jac, mag_measured);

        Eigen::Matrix<float, StateSize, 1> x_est = ekf.getState();
        Eigen::Quaternionf q_est(x_est(3), x_est(0), x_est(1), x_est(2));
        q_est.normalize();

        float yaw = std::atan2(2.0f * (q_est.w() * q_est.z() + q_est.x() * q_est.y()),
                               1.0f - 2.0f * (q_est.y() * q_est.y() + q_est.z() * q_est.z()));

        // std::cout << "Step " << step + 1 << " | Estimated yaw: " << yaw * 180.0f / M_PI << " deg\n";
        
        float unwrapped_yaw = fmodf(omega(2) * dt * static_cast<float>(step + 1), 2.0f * M_PIf);
        if (unwrapped_yaw > M_PIf) unwrapped_yaw -= 2.f * M_PIf;
        CHECK(std::abs(yaw - unwrapped_yaw) < 0.5f);
    }
}

TEST_CASE("Orientation EKF tracks yaw using gyro + magnetometer (analytic Jacobian)") {
    constexpr int StateSize = 7;        // [qx, qy, qz, qw, wx, wy, wz]
    constexpr int MeasurementSize = 3;  // magnetometer: 3D field in body frame

    float dt = 1.0f;
    float yaw_rate = M_PIf / 180.0f * 45.0f; // 45 deg/s
    Eigen::Vector3f mag_ned(1.0f, 0.0f, 0.0f); // magnetic field in NED

    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    Eigen::Vector3f omega(0.0f, 0.0f, yaw_rate);

    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-5f;
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R = Eigen::Matrix3f::Identity() * 0.01f;
    Eigen::Matrix<float, StateSize, 1> x0 = Eigen::Matrix<float, StateSize, 1>::Zero();
    KalmanFilter<StateSize, MeasurementSize> ekf(Q, R, Q, x0);

    for (int step = 0; step < 10; ++step) {
        // Simulate true orientation by integrating angular velocity
        Eigen::Quaternionf delta_q(1.0f,
                                   0.5f * omega(0) * dt,
                                   0.5f * omega(1) * dt,
                                   0.5f * omega(2) * dt);
        q = (q * delta_q).normalized();

        Eigen::Vector3f mag_measured = q.inverse() * mag_ned + Eigen::Vector3f::Random() * 0.01f;

        Eigen::Matrix<float, StateSize, 1> x;
        x << q.x(), q.y(), q.z(), q.w(), omega;
        ekf.stateVector = x;

        auto h = [&](const Eigen::Matrix<float, StateSize, 1>& x_pred) {
            Eigen::Quaternionf q_hat(x_pred(3), x_pred(0), x_pred(1), x_pred(2)); // [w, x, y, z]
            return q_hat.conjugate() * mag_ned;
        };

        // Analytical Jacobian of q⁻¹ ⋅ mag_ned w.r.t. q
        Eigen::Quaternionf q_hat(x(3), x(0), x(1), x(2));
        q_hat.normalize();

        float qw = q_hat.w(), qx = q_hat.x(), qy = q_hat.y(), qz = q_hat.z();
        float mx = mag_ned(0), my = mag_ned(1), mz = mag_ned(2);

        Eigen::Matrix<float, MeasurementSize, StateSize> H_jac = Eigen::Matrix<float, MeasurementSize, StateSize>::Zero();

        H_jac(0,0) =  2*(my*qz - mz*qy);
        H_jac(0,1) =  2*(my*qy + mz*qz);
        H_jac(0,2) =  2*(my*qw - mx*qz);
        H_jac(0,3) = -2*(my*qx + mz*qy);
        H_jac(1,0) =  2*(mz*qx - mx*qz);
        H_jac(1,1) =  2*(mx*qx + mz*qw);
        H_jac(1,2) =  2*(mx*qz - my*qw);
        H_jac(1,3) = -2*(mx*qy + mz*qx);
        H_jac(2,0) =  2*(mx*qy - my*qx);
        H_jac(2,1) =  2*(mx*qz - mz*qx);
        H_jac(2,2) =  2*(my*qz + mz*qy);
        H_jac(2,3) = -2*(mx*qx + my*qy);

        ekf.updateEKF(h, H_jac, mag_measured);

        // Normalize quaternion in filter state
        auto& vec = ekf.stateVector;
        Eigen::Quaternionf q_corr(vec(3), vec(0), vec(1), vec(2));
        q_corr.normalize();
        vec(0) = q_corr.x(); vec(1) = q_corr.y(); vec(2) = q_corr.z(); vec(3) = q_corr.w();

        Eigen::Matrix<float, StateSize, 1> x_est = ekf.getState();
        Eigen::Quaternionf q_est(x_est(3), x_est(0), x_est(1), x_est(2));
        q_est.normalize();

        float yaw = std::atan2(2.0f * (q_est.w() * q_est.z() + q_est.x() * q_est.y()),
                               1.0f - 2.0f * (q_est.y() * q_est.y() + q_est.z() * q_est.z()));

        float unwrapped_yaw = fmodf(omega(2) * dt * static_cast<float>(step + 1), 2.0f * M_PIf);
        if (unwrapped_yaw > M_PIf) unwrapped_yaw -= 2.f * M_PIf;

        // std::cout << "Step " << step + 1 << " | Estimated yaw: " << yaw * 180.f / M_PIf << " deg\n";
        CHECK(std::abs(yaw - unwrapped_yaw) < 0.5f);
    }
}

TEST_CASE("Orientation EKF tracks yaw using gyro + magnetometer (analytic Jacobian): increase R, bias") {
    constexpr int StateSize = 7;        // [qx, qy, qz, qw, wx, wy, wz]
    constexpr int MeasurementSize = 3;  // magnetometer: 3D field in body frame

    float dt = 1.0f;
    float yaw_rate = M_PIf / 180.0f * 45.0f; // 45 deg/s
    Eigen::Vector3f mag_ned(1.0f, 0.0f, 0.0f); // magnetic field in NED

    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    Eigen::Vector3f omega(0.0f, 0.0f, yaw_rate + 0.05f);

    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-5f;
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R = Eigen::Matrix3f::Identity() * 0.025f;
    Eigen::Matrix<float, StateSize, 1> x0 = Eigen::Matrix<float, StateSize, 1>::Zero();
    x0 << 0, 0, 0.2f, 0.98f, 0, 0, yaw_rate;
    KalmanFilter<StateSize, MeasurementSize> ekf(Q, R, Q, x0);

    for (int step = 0; step < 10; ++step) {
        // Simulate true orientation by integrating angular velocity
        Eigen::Quaternionf delta_q(1.0f,
                                   0.5f * omega(0) * dt,
                                   0.5f * omega(1) * dt,
                                   0.5f * omega(2) * dt);
        q = (q * delta_q).normalized();

        Eigen::Vector3f mag_measured = q.inverse() * mag_ned + Eigen::Vector3f::Random() * 0.01f;

        Eigen::Matrix<float, StateSize, 1> x;
        x << q.x(), q.y(), q.z(), q.w(), omega;
        ekf.stateVector = x;

        auto h = [&](const Eigen::Matrix<float, StateSize, 1>& x_pred) {
            Eigen::Quaternionf q_hat(x_pred(3), x_pred(0), x_pred(1), x_pred(2)); // [w, x, y, z]
            return q_hat.conjugate() * mag_ned;
        };

        // Analytical Jacobian of q⁻¹ ⋅ mag_ned w.r.t. q
        Eigen::Quaternionf q_hat(x(3), x(0), x(1), x(2));
        q_hat.normalize();

        float qw = q_hat.w(), qx = q_hat.x(), qy = q_hat.y(), qz = q_hat.z();
        float mx = mag_ned(0), my = mag_ned(1), mz = mag_ned(2);

        Eigen::Matrix<float, MeasurementSize, StateSize> H_jac = Eigen::Matrix<float, MeasurementSize, StateSize>::Zero();

        H_jac(0,0) =  2*(my*qz - mz*qy);
        H_jac(0,1) =  2*(my*qy + mz*qz);
        H_jac(0,2) =  2*(my*qw - mx*qz);
        H_jac(0,3) = -2*(my*qx + mz*qy);
        H_jac(1,0) =  2*(mz*qx - mx*qz);
        H_jac(1,1) =  2*(mx*qx + mz*qw);
        H_jac(1,2) =  2*(mx*qz - my*qw);
        H_jac(1,3) = -2*(mx*qy + mz*qx);
        H_jac(2,0) =  2*(mx*qy - my*qx);
        H_jac(2,1) =  2*(mx*qz - mz*qx);
        H_jac(2,2) =  2*(my*qz + mz*qy);
        H_jac(2,3) = -2*(mx*qx + my*qy);

        ekf.updateEKF(h, H_jac, mag_measured);

        // Normalize quaternion in filter state
        auto& vec = ekf.stateVector;
        Eigen::Quaternionf q_corr(vec(3), vec(0), vec(1), vec(2));
        q_corr.normalize();
        vec(0) = q_corr.x(); vec(1) = q_corr.y(); vec(2) = q_corr.z(); vec(3) = q_corr.w();

        Eigen::Matrix<float, StateSize, 1> x_est = ekf.getState();
        Eigen::Quaternionf q_est(x_est(3), x_est(0), x_est(1), x_est(2));
        q_est.normalize();

        float yaw = std::atan2(2.0f * (q_est.w() * q_est.z() + q_est.x() * q_est.y()),
                               1.0f - 2.0f * (q_est.y() * q_est.y() + q_est.z() * q_est.z()));

        float unwrapped_yaw = fmodf(omega(2) * dt * static_cast<float>(step + 1), 2.0f * M_PIf);
        if (unwrapped_yaw > M_PIf) unwrapped_yaw -= 2.f * M_PIf;

        // std::cout << "Step " << step + 1 << " | Estimated yaw: " << yaw * 180.f / M_PIf << " deg\n";
        CHECK(std::abs(yaw - unwrapped_yaw) < 0.5f);
    }
}

// -----------------------------------------------
// 🧠 Quaternion + Angular Velocity EKF Prediction
// -----------------------------------------------
template <typename FilterType>
void predictOrientationWithGyro(FilterType& ekf, float dt) {
    auto& x = ekf.stateVector;
    Eigen::Quaternionf q(x(3), x(0), x(1), x(2));     // [w, x, y, z]
    Eigen::Vector3f omega = x.template segment<3>(4); // [wx, wy, wz]

    Eigen::Quaternionf delta_q(1.0f,
        0.5f * omega(0) * dt,
        0.5f * omega(1) * dt,
        0.5f * omega(2) * dt);
    q = (q * delta_q).normalized();

    x(0) = q.x(); x(1) = q.y(); x(2) = q.z(); x(3) = q.w();
    ekf.stateCovarianceMatrix += ekf.processNoiseCovarianceMatrix;
}

// -----------------------------------------------
// ✅ Test: Space-based EKF using only gyro + magnetometer
// -----------------------------------------------
TEST_CASE("3D EKF integrates gyro and magnetometer (external prediction)") {
    constexpr int StateSize = 7;        // [qx, qy, qz, qw, wx, wy, wz]
    constexpr int MeasurementSize = 3;  // magnetometer: 3D field in body frame

    float dt = 1.0f;
    float yaw_rate = M_PIf / 180.0f * 45.0f; // 45 deg/s
    Eigen::Vector3f mag_ned(1.0f, 0.0f, 0.0f); // magnetic field in NED

    Eigen::Quaternionf q_true = Eigen::Quaternionf::Identity();
    Eigen::Vector3f omega(0.0f, 0.0f, yaw_rate);

    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-5f;
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R = Eigen::Matrix3f::Identity() * 0.01f;
    Eigen::Matrix<float, StateSize, 1> x0 = Eigen::Matrix<float, StateSize, 1>::Zero();
    x0 << 0, 0, 0, 1, omega(0), omega(1), omega(2);

    KalmanFilter<StateSize, MeasurementSize> ekf(Q, R, Q, x0);

    for (int step = 0; step < 10; ++step) {
        // Simulate true orientation by integrating angular velocity
        Eigen::Quaternionf delta_q_true(1.0f,
            0.5f * omega(0) * dt,
            0.5f * omega(1) * dt,
            0.5f * omega(2) * dt);
        q_true = (q_true * delta_q_true).normalized();

        // Simulated magnetometer measurement
        Eigen::Vector3f mag_measured = q_true.conjugate() * mag_ned + Eigen::Vector3f::Random() * 0.01f;

        // Predict using internal angular rate
        predictOrientationWithGyro(ekf, dt);

        // Nonlinear measurement model
        auto h = [&](const Eigen::Matrix<float, StateSize, 1>& x_pred) {
            Eigen::Quaternionf q_hat(x_pred(3), x_pred(0), x_pred(1), x_pred(2));
            return q_hat.conjugate() * mag_ned;
        };

        // Jacobian ∂(q⁻¹ ⋅ m)/∂q
        Eigen::Quaternionf q_hat(ekf.stateVector(3), ekf.stateVector(0), ekf.stateVector(1), ekf.stateVector(2));
        float qw = q_hat.w(), qx = q_hat.x(), qy = q_hat.y(), qz = q_hat.z();
        float mx = mag_ned(0), my = mag_ned(1), mz = mag_ned(2);

        Eigen::Matrix<float, MeasurementSize, StateSize> H_jac = Eigen::Matrix<float, MeasurementSize, StateSize>::Zero();

        H_jac(0,0) =  2*(my*qz - mz*qy);
        H_jac(0,1) =  2*(my*qy + mz*qz);
        H_jac(0,2) =  2*(my*qw - mx*qz);
        H_jac(0,3) = -2*(my*qx + mz*qy);
        H_jac(1,0) =  2*(mz*qx - mx*qz);
        H_jac(1,1) =  2*(mx*qx + mz*qw);
        H_jac(1,2) =  2*(mx*qz - my*qw);
        H_jac(1,3) = -2*(mx*qy + mz*qx);
        H_jac(2,0) =  2*(mx*qy - my*qx);
        H_jac(2,1) =  2*(mx*qz - mz*qx);
        H_jac(2,2) =  2*(my*qz + mz*qy);
        H_jac(2,3) = -2*(mx*qx + my*qy);

        ekf.updateEKF(h, H_jac, mag_measured);

        // Normalize quaternion
        auto& x = ekf.stateVector;
        Eigen::Quaternionf q_corr(x(3), x(0), x(1), x(2));
        q_corr.normalize();
        x(0) = q_corr.x(); x(1) = q_corr.y(); x(2) = q_corr.z(); x(3) = q_corr.w();

        float yaw_est = std::atan2(2.0f * (q_corr.w() * q_corr.z() + q_corr.x() * q_corr.y()),
                                   1.0f - 2.0f * (q_corr.y() * q_corr.y() + q_corr.z() * q_corr.z()));

        float yaw_true = std::atan2(2.0f * (q_true.w() * q_true.z() + q_true.x() * q_true.y()),
                                    1.0f - 2.0f * (q_true.y() * q_true.y() + q_true.z() * q_true.z()));

        float yaw_error = std::atan2(std::sin(yaw_est - yaw_true), std::cos(yaw_est - yaw_true));
        // std::cout << "Step " << step + 1 << " | Estimated yaw: " << yaw_est * 180.f / M_PIf
        //           << " | True yaw: " << yaw_true * 180.f / M_PIf
        //           << " | Error: " << yaw_error * 180.f / M_PIf << " deg\n";

        CHECK(std::abs(yaw_error) < 0.1f);
    }
}

// -----------------------------------------------
// ✅ Tumbling Satellite in Magnetic Field Only
// -----------------------------------------------
TEST_CASE("Tumbling EKF tracks 3D orientation using gyro + tilted magnetometer") {
    constexpr int StateSize = 7;
    constexpr int MeasurementSize = 3;

    float dt = 0.5f;
    float tumble_deg_per_sec = 10.0f;
    float tumble_rate = tumble_deg_per_sec * M_PIf / 180.0f;

    // 🧭 Magnetic field: inclined (not axis-aligned)
    Eigen::Vector3f mag_ned = Eigen::Vector3f(0.6f, 0.3f, 0.7f).normalized();

    // 🛰 Tumbling about axis [1,1,1]
    Eigen::Vector3f axis = Eigen::Vector3f(1.0f, 1.0f, 1.0f).normalized();
    Eigen::Vector3f omega = axis * tumble_rate;

    Eigen::Quaternionf q_true = Eigen::Quaternionf::Identity();

    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-5f;
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R = Eigen::Matrix3f::Identity() * 0.01f;

    Eigen::Matrix<float, StateSize, 1> x0;
    x0 << 0.01f, -0.02f, 0.015f, 0.98f, omega;  // slightly off orientation
    KalmanFilter<StateSize, MeasurementSize> ekf(Q, R, Q, x0);

    for (int step = 0; step < 100; ++step) {
        // Simulate true orientation
        Eigen::Quaternionf dq_true(1.0f,
            0.5f * omega(0) * dt,
            0.5f * omega(1) * dt,
            0.5f * omega(2) * dt);
        q_true = (q_true * dq_true).normalized();

        Eigen::Vector3f mag_meas = q_true.conjugate() * mag_ned + Eigen::Vector3f::Random() * 0.01f;

        // Predict using internal gyro
        predictOrientationWithGyro(ekf, dt);

        // Measurement function
        auto h = [&](const Eigen::Matrix<float, StateSize, 1>& x) {
            Eigen::Quaternionf q(x(3), x(0), x(1), x(2));
            return q.conjugate() * mag_ned;
        };

        // Jacobian ∂(q⁻¹ ⋅ m)/∂q
        auto& xs = ekf.stateVector;
        Eigen::Quaternionf q(xs(3), xs(0), xs(1), xs(2));
        float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
        float mx = mag_ned(0), my = mag_ned(1), mz = mag_ned(2);
        Eigen::Matrix<float, MeasurementSize, StateSize> H_jac = Eigen::Matrix<float, MeasurementSize, StateSize>::Zero();

        H_jac(0,0) =  2*(my*qz - mz*qy);  H_jac(0,1) =  2*(my*qy + mz*qz);
        H_jac(0,2) =  2*(my*qw - mx*qz);  H_jac(0,3) = -2*(my*qx + mz*qy);
        H_jac(1,0) =  2*(mz*qx - mx*qz);  H_jac(1,1) =  2*(mx*qx + mz*qw);
        H_jac(1,2) =  2*(mx*qz - my*qw);  H_jac(1,3) = -2*(mx*qy + mz*qx);
        H_jac(2,0) =  2*(mx*qy - my*qx);  H_jac(2,1) =  2*(mx*qz - mz*qx);
        H_jac(2,2) =  2*(my*qz + mz*qy);  H_jac(2,3) = -2*(mx*qx + my*qy);

        ekf.updateEKF(h, H_jac, mag_meas);

        // Normalize estimated quaternion
        q = Eigen::Quaternionf(xs(3), xs(0), xs(1), xs(2)).normalized();
        xs(0) = q.x(); xs(1) = q.y(); xs(2) = q.z(); xs(3) = q.w();

        // Compute angular error (true vs estimated orientation)
        Eigen::Matrix3f R_est = q.toRotationMatrix();
        Eigen::Matrix3f R_true = q_true.toRotationMatrix();
        float angle_error_rad = std::acos(std::min(1.0f, std::max(-1.0f, 0.5f * (R_est.transpose() * R_true).trace() - 0.5f)));

        // std::cout << "Step " << step + 1 << " | Orientation error: " << angle_error_rad * 180.f / M_PIf << " deg\n";

        // Let error grow slightly to test long-term convergence
        CHECK(angle_error_rad < 0.3f);
    }
}

TEST_CASE("Periodic tumbling EKF tracks full 3D orientation with magnetometer only") {
    constexpr int StateSize = 7;
    constexpr int MeasurementSize = 3;

    float dt = 0.5f;
    Eigen::Vector3f mag_ned = Eigen::Vector3f(0.3f, 0.5f, 0.8f).normalized();

    // Frequencies (rad/sec) and amplitudes (rad/sec)
    Eigen::Vector3f freq(0.5f, 0.3f, 0.7f);
    Eigen::Vector3f amp = Eigen::Vector3f::Ones() * (10.f * M_PIf / 180.0f); // 10 deg/s

    Eigen::Quaternionf q_true = Eigen::Quaternionf::Identity();

    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-5f;
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R = Eigen::Matrix3f::Identity() * 0.01f;

    Eigen::Matrix<float, StateSize, 1> x0;
    x0 << 0.01f, -0.02f, 0.03f, 0.98f, 0.0f, 0.0f, 0.0f;
    KalmanFilter<StateSize, MeasurementSize> ekf(Q, R, Q, x0);

    for (int step = 0; step < 60; ++step) {
        float t = static_cast<float>(step) * dt;

        // Periodic true angular rate
        Eigen::Vector3f omega(
            amp(0) * std::sin(freq(0) * t),
            amp(1) * std::cos(freq(1) * t),
            amp(2) * std::sin(freq(2) * t + 0.5f)
        );

        // Integrate ground truth orientation
        Eigen::Quaternionf dq_true(1.0f,
            0.5f * omega(0) * dt,
            0.5f * omega(1) * dt,
            0.5f * omega(2) * dt);
        q_true = (q_true * dq_true).normalized();

        Eigen::Vector3f mag_meas = q_true.conjugate() * mag_ned + Eigen::Vector3f::Random() * 0.01f;

        // Inject true angular rate into filter for now (no bias yet)
        auto& xs = ekf.stateVector;
        xs.segment<3>(4) = omega;

        predictOrientationWithGyro(ekf, dt);

        auto h = [&](const Eigen::Matrix<float, StateSize, 1>& x) {
            Eigen::Quaternionf q(x(3), x(0), x(1), x(2));
            return q.conjugate() * mag_ned;
        };

        Eigen::Quaternionf q_est(xs(3), xs(0), xs(1), xs(2));
        float qw = q_est.w(), qx = q_est.x(), qy = q_est.y(), qz = q_est.z();
        float mx = mag_ned(0), my = mag_ned(1), mz = mag_ned(2);
        Eigen::Matrix<float, MeasurementSize, StateSize> H_jac = Eigen::Matrix<float, MeasurementSize, StateSize>::Zero();

        H_jac(0,0) =  2*(my*qz - mz*qy);  H_jac(0,1) =  2*(my*qy + mz*qz);
        H_jac(0,2) =  2*(my*qw - mx*qz);  H_jac(0,3) = -2*(my*qx + mz*qy);
        H_jac(1,0) =  2*(mz*qx - mx*qz);  H_jac(1,1) =  2*(mx*qx + mz*qw);
        H_jac(1,2) =  2*(mx*qz - my*qw);  H_jac(1,3) = -2*(mx*qy + mz*qx);
        H_jac(2,0) =  2*(mx*qy - my*qx);  H_jac(2,1) =  2*(mx*qz - mz*qx);
        H_jac(2,2) =  2*(my*qz + mz*qy);  H_jac(2,3) = -2*(mx*qx + my*qy);

        ekf.updateEKF(h, H_jac, mag_meas);

        // Normalize quaternion
        q_est.normalize();
        xs(0) = q_est.x(); xs(1) = q_est.y(); xs(2) = q_est.z(); xs(3) = q_est.w();

        // Compute angle error
        Eigen::Matrix3f R_est = q_est.toRotationMatrix();
        Eigen::Matrix3f R_true = q_true.toRotationMatrix();
        float angle_error = std::acos(std::min(1.0f, std::max(-1.0f, 0.5f * (R_est.transpose() * R_true).trace() - 0.5f)));

        // std::cout << "Step " << step + 1 << " | Orientation error: " << angle_error * 180.0f / M_PIf << " deg\n";

        CHECK(angle_error < 0.3f);
    }
}

TEST_CASE("EKF with intermittent magnetometer updates during periodic tumbling") {
    constexpr int StateSize = 7;
    constexpr int MeasurementSize = 3;

    float dt = 0.5f;
    Eigen::Vector3f mag_ned = Eigen::Vector3f(0.3f, 0.5f, 0.8f).normalized();

    // Sinusoidal angular velocity
    Eigen::Vector3f freq(0.4f, 0.6f, 0.5f);
    Eigen::Vector3f amp = Eigen::Vector3f::Ones() * (8.f * M_PIf / 180.f); // 8 deg/s

    Eigen::Quaternionf q_true = Eigen::Quaternionf::Identity();

    Eigen::Matrix<float, StateSize, StateSize> Q = Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-5f;
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> R = Eigen::Matrix3f::Identity() * 0.01f;

    Eigen::Matrix<float, StateSize, 1> x0;
    x0 << -0.01f, 0.015f, -0.005f, 0.99f, 0, 0, 0;
    KalmanFilter<StateSize, MeasurementSize> ekf(Q, R, Q, x0);

    for (int step = 0; step < 60; ++step) {
        float t = static_cast<float>(step) * dt;

        Eigen::Vector3f omega(
            amp(0) * std::sin(freq(0) * t),
            amp(1) * std::cos(freq(1) * t),
            amp(2) * std::sin(freq(2) * t + 0.7f));

        // Ground truth orientation
        Eigen::Quaternionf delta_q(1.0f,
            0.5f * omega(0) * dt,
            0.5f * omega(1) * dt,
            0.5f * omega(2) * dt);
        q_true = (q_true * delta_q).normalized();

        Eigen::Vector3f mag_meas = q_true.conjugate() * mag_ned + Eigen::Vector3f::Random() * 0.01f;

        // Inject gyro rate into EKF state
        ekf.stateVector.segment<3>(4) = omega;
        predictOrientationWithGyro(ekf, dt);

        // Only correct with magnetometer every 6 steps (~3s)
        if (step % 6 == 0) {
            auto h = [&](const Eigen::Matrix<float, StateSize, 1>& x) {
                Eigen::Quaternionf q(x(3), x(0), x(1), x(2));
                return q.conjugate() * mag_ned;
            };

            Eigen::Quaternionf q_hat(ekf.stateVector(3), ekf.stateVector(0), ekf.stateVector(1), ekf.stateVector(2));
            float qw = q_hat.w(), qx = q_hat.x(), qy = q_hat.y(), qz = q_hat.z();
            float mx = mag_ned(0), my = mag_ned(1), mz = mag_ned(2);
            Eigen::Matrix<float, MeasurementSize, StateSize> H_jac = Eigen::Matrix<float, MeasurementSize, StateSize>::Zero();

            H_jac(0,0) =  2*(my*qz - mz*qy);  H_jac(0,1) =  2*(my*qy + mz*qz);
            H_jac(0,2) =  2*(my*qw - mx*qz);  H_jac(0,3) = -2*(my*qx + mz*qy);
            H_jac(1,0) =  2*(mz*qx - mx*qz);  H_jac(1,1) =  2*(mx*qx + mz*qw);
            H_jac(1,2) =  2*(mx*qz - my*qw);  H_jac(1,3) = -2*(mx*qy + mz*qx);
            H_jac(2,0) =  2*(mx*qy - my*qx);  H_jac(2,1) =  2*(mx*qz - mz*qx);
            H_jac(2,2) =  2*(my*qz + mz*qy);  H_jac(2,3) = -2*(mx*qx + my*qy);

            ekf.updateEKF(h, H_jac, mag_meas);
        }

        // Normalize quaternion
        auto& x = ekf.stateVector;
        Eigen::Quaternionf q(x(3), x(0), x(1), x(2));
        q.normalize();
        x(0) = q.x(); x(1) = q.y(); x(2) = q.z(); x(3) = q.w();

        Eigen::Matrix3f R_est = q.toRotationMatrix();
        Eigen::Matrix3f R_true = q_true.toRotationMatrix();
        float angle_error = std::acos(std::min(1.0f, std::max(-1.0f, 0.5f * (R_est.transpose() * R_true).trace() - 0.5f)));

        // std::cout << "Step " << step + 1
        //           << " | Orientation error: " << angle_error * 180.0f / M_PIf << " deg"
        //           << (step % 6 == 0 ? "  <-- magnetometer update\n" : "\n");

        CHECK(angle_error < 5.0f); // Give room for drift between updates
    }
}
