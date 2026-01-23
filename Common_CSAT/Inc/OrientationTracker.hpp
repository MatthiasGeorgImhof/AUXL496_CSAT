#ifndef __ORIENTATION_TRACKER_HPP__
#define __ORIENTATION_TRACKER_HPP__

#include <Eigen/Dense>
#include <functional>
#include "Kalman.hpp"
#include "Quaternion.hpp"
#include "au.hpp"

#include "TimeUtils.hpp"
#ifdef __arm__
#include "usbd_cdc_if.h"
#endif
#ifdef __x86_64__
#include "mock_hal.h"
#endif

// Gyroscope NED (front, right, down)
// Orientation	Axis Rotation Positive Direction
// front		X	Roll	Right wing down
// right/east	Y	Pitch	Nose up
// down			Z	Yaw		Nose right

// Magnetometer NED (front, right, down)
// Katy TX
// Device Heading	Sensor X (forward)	Sensor Y (left)	Sensor Z (down)
// North			+22.9 µT		–2.2 µT				+41 µT
// East				-2.2 µT			-22.9 µT			+41 µT
// South			–22.9 µT		+2.2 µT				+41 µT
// West				+2.2 µT			+22.9 µT			+41 µT

// Accelerometer is front (+0.91), left/west (+9.81). up (+9.81)
// wanted NED (front east down) to have positive +9.81 when oriented
// Orientation	Axis Rotation Positive Direction
// front		X	-9.81 when x back points
// right/east	Y	+9.81 when y points up
// down			Z	+9.81


template <int StateSize, int MeasurementSize>
class BaseOrientationTracker
{
public:
    using StateVector = Eigen::Matrix<float, StateSize, 1>;
    using Measurement = Eigen::Matrix<float, MeasurementSize, 1>;

    BaseOrientationTracker(
        const Eigen::Matrix<float, StateSize, StateSize> &initialProcessNoiseMatrix,
        const Eigen::Matrix<float, MeasurementSize, MeasurementSize> &initialMeasurementNoiseMatrix,
        const Eigen::Matrix<float, StateSize, StateSize> &initialStateCovarianceMatrix,
        const Eigen::Matrix<float, StateSize, 1> &initialStateVector)
        : ekf(
              initialProcessNoiseMatrix,
              initialMeasurementNoiseMatrix,
              initialStateCovarianceMatrix,
              initialStateVector),
          last_timestamp(au::make_quantity<au::Milli<au::Seconds>>(0)),
          prev_orientation(Eigen::Quaternionf::Identity())
    {
    }

    void predictTo(au::QuantityU64<au::Milli<au::Seconds>> new_timestamp)
    {
        float dt = 0.001f * static_cast<float>((new_timestamp - last_timestamp).in(au::milli(au::seconds)));
        if (dt <= 0.f)
            return;

        auto &x = ekf.stateVector;
        Eigen::Quaternionf q(x(3), x(0), x(1), x(2));
        Eigen::Vector3f omega = x.template segment<3>(4);

        float angle = omega.norm() * dt;
        Eigen::Quaternionf delta_q = Eigen::Quaternionf::Identity();
        if (angle > 1e-6f)
        {
            Eigen::Vector3f axis = omega.normalized();
            delta_q = Eigen::AngleAxisf(angle, axis);
        }

        q = (q * delta_q).normalized();
        if (q.dot(prev_orientation) < 0.f)
            q.coeffs() *= -1.f;

        // std::cout << "[predictTo] dt: " << dt << ", omega: " << omega.transpose()
        //           << ", angle: " << angle << "\n";
        // std::cout << "[predictTo] q updated: " << q.coeffs().transpose() << "\n";

        prev_orientation = q;
        x(0) = q.x();
        x(1) = q.y();
        x(2) = q.z();
        x(3) = q.w();
        ekf.stateCovarianceMatrix += ekf.processNoiseCovarianceMatrix;
        last_timestamp = new_timestamp;
    }

    void updateGyro(const Eigen::Vector3f &gyro, au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        predictTo(timestamp);
        ekf.stateVector.template segment<3>(4) = gyro;
    }

    void updateSensorFusion(const Eigen::Vector3f &gyro, au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        updateGyro(gyro, timestamp);
    }

    void setGyroAngularRate(const Eigen::Vector3f &omega)
    {
        ekf.stateVector.template segment<3>(4) = omega;
    }

    void setOrientation(const Eigen::Quaternionf &q)
    {
        ekf.stateVector(0) = q.x();
        ekf.stateVector(1) = q.y();
        ekf.stateVector(2) = q.z();
        ekf.stateVector(3) = q.w();
        prev_orientation = q;
    }

    Eigen::Quaternionf getOrientation() const
    {
        Eigen::Quaternionf q(ekf.stateVector(3), ekf.stateVector(0), ekf.stateVector(1), ekf.stateVector(2));
        if (q.dot(prev_orientation) < 0.f)
            q.coeffs() *= -1.f;
        return q.normalized();
    }

    Eigen::Vector3f getYawPitchRoll() const
    {
        const auto &q = getOrientation();
        float sinp = 2.f * (q.w() * q.y() - q.z() * q.x());
        sinp = std::clamp(sinp, -1.f, 1.f);

        float yaw = std::atan2(2.f * (q.w() * q.z() + q.x() * q.y()),
                               1.f - 2.f * (q.y() * q.y() + q.z() * q.z()));
        float pitch = std::asin(sinp);
        float roll = std::atan2(2.f * (q.w() * q.x() + q.y() * q.z()),
                                1.f - 2.f * (q.x() * q.x() + q.y() * q.y()));
        return Eigen::Vector3f(yaw, pitch, roll);
    }

    void printDebugState(const std::string &label = "") const
    {
        constexpr float m_pif = static_cast<float>(std::numbers::pi);
        const auto &x = ekf.stateVector;
        std::cout << "\n[Tracker State] " << label << "\n";
        std::cout << "Quaternion: [" << x(0) << ", " << x(1) << ", " << x(2) << ", " << x(3) << "]\n";
        std::cout << "Angular rate: [" << x(4) << ", " << x(5) << ", " << x(6) << "]\n";
        std::cout << "Orientation: " << getYawPitchRoll().transpose() * 180.0f / m_pif << " deg\n";
    }

protected:
    KalmanFilter<StateSize, MeasurementSize> ekf;
    au::QuantityU64<au::Milli<au::Seconds>> last_timestamp;
    Eigen::Quaternionf prev_orientation;
};

template <int StateSize = 7, int MeasurementSize = 3>
class GyrMagOrientationTracker : public BaseOrientationTracker<StateSize, MeasurementSize>
{
public:
    using StateVector = Eigen::Matrix<float, StateSize, 1>;
    using Measurement = Eigen::Vector3f;

protected:
    Eigen::Vector3f magnetic_ned;

public:
    GyrMagOrientationTracker() : BaseOrientationTracker<StateSize, MeasurementSize>(
                                     Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-5f,
                                     Eigen::Matrix3f::Identity() * 0.01f,
                                     Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-5f,
                                     []
                                     { StateVector x; x << 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f; return x; }()), magnetic_ned(0.3f, 0.5f, 0.8f)
    {
    }

    void updateMagnetometer(const Eigen::Vector3f &mag_body, au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        Eigen::Vector3f norm_mag_body{mag_body};
        norm_mag_body.normalize();
        BaseOrientationTracker<StateSize, MeasurementSize>::predictTo(timestamp);

        auto h = [&](const StateVector &x)
        {
            Eigen::Quaternionf q(x(3), x(0), x(1), x(2));
            return q.conjugate() * magnetic_ned;
        };

        auto &x = BaseOrientationTracker<StateSize, MeasurementSize>::ekf.stateVector;
        Eigen::Quaternionf q_hat(x(3), x(0), x(1), x(2));
        float qw = q_hat.w(), qx = q_hat.x(), qy = q_hat.y(), qz = q_hat.z();
        float mx = magnetic_ned(0), my = magnetic_ned(1), mz = magnetic_ned(2);

        Eigen::Matrix<float, MeasurementSize, StateSize> H_jac = Eigen::Matrix<float, MeasurementSize, StateSize>::Zero();

        H_jac(0, 0) = 2.0f * (my * qz - mz * qy);
        H_jac(0, 1) = 2.0f * (my * qy + mz * qz);
        H_jac(0, 2) = 2.0f * (my * qw - mx * qz);
        H_jac(0, 3) = -2.0f * (my * qx + mz * qy);
        H_jac(1, 0) = 2.0f * (mz * qx - mx * qz);
        H_jac(1, 1) = 2.0f * (mx * qx + mz * qw);
        H_jac(1, 2) = 2.0f * (mx * qz - my * qw);
        H_jac(1, 3) = -2.0f * (mx * qy + mz * qx);
        H_jac(2, 0) = 2.0f * (mx * qy - my * qx);
        H_jac(2, 1) = 2.0f * (mx * qz - mz * qx);
        H_jac(2, 2) = 2.0f * (my * qz + mz * qy); // Corre
        H_jac(2, 3) = -2.0f * (mx * qx + my * qy);

        BaseOrientationTracker<StateSize, MeasurementSize>::ekf.updateEKF(h, H_jac, norm_mag_body);

        Eigen::Quaternionf q_corr(x(3), x(0), x(1), x(2));
        q_corr.normalize();
        x(0) = q_corr.x();
        x(1) = q_corr.y();
        x(2) = q_corr.z();
        x(3) = q_corr.w();
    }

    void updateSensorFusion(const Eigen::Vector3f &gyro,
                            const Eigen::Vector3f &mag_body,
                            au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        BaseOrientationTracker<StateSize, MeasurementSize>::updateGyro(gyro, timestamp);
        updateMagnetometer(mag_body, timestamp);
    }

    void setReferenceVectors(const Eigen::Vector3f &magnetic_ned_in)
    {
        magnetic_ned = magnetic_ned_in.normalized();
    }
};

#
#
#

template <int StateSize = 7, int MeasurementSize = 6>
class AccGyrMagOrientationTracker : public BaseOrientationTracker<StateSize, MeasurementSize>
{
public:
    using StateVector = Eigen::Matrix<float, StateSize, 1>;
    using Measurement = Eigen::Matrix<float, MeasurementSize, 1>;

private:
    Eigen::Vector3f accel_ned;
    Eigen::Vector3f magnetic_ned;

public:
    AccGyrMagOrientationTracker()
        : BaseOrientationTracker<StateSize, MeasurementSize>(
              Eigen::Matrix<float, StateSize, StateSize>::Identity() * 0.1f,
              Eigen::Matrix<float, MeasurementSize, MeasurementSize>::Identity() * 0.001f,
              Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-5f,
              []
              { StateVector x; x << 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f; return x; }()), accel_ned(0.f, 0.f, 9.81f), magnetic_ned(0.51f, 0.04f, 0.89f)
    {
    }

    void updateAccelerometerMagnetometer(const Eigen::Vector3f &accel_body,
                                         const Eigen::Vector3f &mag_body,
                                         au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        Eigen::Vector3f norm_mag_body{mag_body};
        norm_mag_body.normalize();
        BaseOrientationTracker<StateSize, MeasurementSize>::predictTo(timestamp);

        // Combined measurement vector
        Measurement combined_meas;
        combined_meas.template segment<3>(0) = accel_body;
        combined_meas.template segment<3>(3) = norm_mag_body;

        auto h = [&](const StateVector &x)
        {
            Eigen::Quaternionf q(x(3), x(0), x(1), x(2));
            Eigen::Vector3f predicted_accel_body = q.conjugate() * accel_ned;
            Eigen::Vector3f predicted_mag_body = q.conjugate() * magnetic_ned;

            Measurement z;
            z.template segment<3>(0) = predicted_accel_body;
            z.template segment<3>(3) = predicted_mag_body;
            return z;
        };

        Eigen::Quaternionf q_raw(BaseOrientationTracker<StateSize, MeasurementSize>::ekf.stateVector(3),
                                 BaseOrientationTracker<StateSize, MeasurementSize>::ekf.stateVector(0),
                                 BaseOrientationTracker<StateSize, MeasurementSize>::ekf.stateVector(1),
                                 BaseOrientationTracker<StateSize, MeasurementSize>::ekf.stateVector(2));
        Eigen::Matrix<float, 3, 4> J_accel = computeAnalyticalJacobian(q_raw, accel_ned);
        J_accel = normalizeAnalyticalJacobian(J_accel, q_raw, accel_ned);
        Eigen::Matrix<float, 3, 4> J_mag = computeAnalyticalJacobian(q_raw, magnetic_ned);
        J_mag = normalizeAnalyticalJacobian(J_mag, q_raw, magnetic_ned);

        Eigen::Matrix<float, MeasurementSize, StateSize> H_analytical = Eigen::Matrix<float, MeasurementSize, StateSize>::Zero();
        H_analytical.template block<3, 4>(0, 0) = J_accel;
        H_analytical.template block<3, 4>(3, 0) = J_mag;

        BaseOrientationTracker<StateSize, MeasurementSize>::ekf.updateEKF(h, H_analytical, combined_meas);

        auto &x = BaseOrientationTracker<StateSize, MeasurementSize>::ekf.stateVector;
        Eigen::Quaternionf q_corr(x(3), x(0), x(1), x(2));
        q_corr.normalize();
        x(0) = q_corr.x();
        x(1) = q_corr.y();
        x(2) = q_corr.z();
        x(3) = q_corr.w();
    }

    void updateSensorFusion(const Eigen::Vector3f &gyro,
                            const Eigen::Vector3f &accel_body,
                            const Eigen::Vector3f &mag_body,
                            au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        BaseOrientationTracker<StateSize, MeasurementSize>::updateGyro(gyro, timestamp);
        updateAccelerometerMagnetometer(accel_body, mag_body, timestamp);
    }

    void setReferenceVectors(const Eigen::Vector3f &accel_ned_in,
                             const Eigen::Vector3f &magnetic_ned_in)
    {
        magnetic_ned = magnetic_ned_in.normalized();
        accel_ned = accel_ned_in.normalized() * 9.81f;
    }
};

template <int StateSize = 7, int MeasurementSize = 3>
class AccGyrOrientationTracker : public BaseOrientationTracker<StateSize, MeasurementSize>
{
public:
    using StateVector = Eigen::Matrix<float, StateSize, 1>;
    using Measurement = Eigen::Vector3f;

private:
    Eigen::Vector3f accel_ned;

public:
    AccGyrOrientationTracker()
        : BaseOrientationTracker<StateSize, MeasurementSize>(
              Eigen::Matrix<float, StateSize, StateSize>::Identity() * 0.1f,              // Process noise
              Eigen::Matrix<float, MeasurementSize, MeasurementSize>::Identity() * 0.01f, // Measurement noise
              Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-5f,             // Initial covariance
              []
              { StateVector x; x << 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f; return x; }()), accel_ned(0.f, 0.f, 9.81f)
    {
        accel_ned = Eigen::Vector3f(0.f, 0.f, 9.81f); // Gravity in NED
    }

    void updateAccelerometer(const Eigen::Vector3f &accel_body,
                             au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        BaseOrientationTracker<StateSize, MeasurementSize>::predictTo(timestamp);

        auto h = [&](const StateVector &x)
        {
            Eigen::Quaternionf q(x(3), x(0), x(1), x(2));
            return q.conjugate() * accel_ned;
        };

        Eigen::Quaternionf q_raw(BaseOrientationTracker<StateSize, MeasurementSize>::ekf.stateVector(3),
                                 BaseOrientationTracker<StateSize, MeasurementSize>::ekf.stateVector(0),
                                 BaseOrientationTracker<StateSize, MeasurementSize>::ekf.stateVector(1),
                                 BaseOrientationTracker<StateSize, MeasurementSize>::ekf.stateVector(2));

        Eigen::Matrix<float, 3, 4> J_accel = computeAnalyticalJacobian(q_raw, accel_ned);
        J_accel = normalizeAnalyticalJacobian(J_accel, q_raw, accel_ned);

        Eigen::Matrix<float, MeasurementSize, StateSize> H_analytical = Eigen::Matrix<float, MeasurementSize, StateSize>::Zero();
        H_analytical.template block<3, 4>(0, 0) = J_accel;

        BaseOrientationTracker<StateSize, MeasurementSize>::ekf.updateEKF(h, H_analytical, accel_body);

        auto &x = BaseOrientationTracker<StateSize, MeasurementSize>::ekf.stateVector;
        Eigen::Quaternionf q_corr(x(3), x(0), x(1), x(2));
        q_corr.normalize();
        x(0) = q_corr.x();
        x(1) = q_corr.y();
        x(2) = q_corr.z();
        x(3) = q_corr.w();
    }

    void updateSensorFusion(const Eigen::Vector3f &gyro,
                            const Eigen::Vector3f &accel_body,
                            au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        BaseOrientationTracker<StateSize, MeasurementSize>::updateGyro(gyro, timestamp);
        updateAccelerometer(accel_body, timestamp);
    }

    void setReferenceVectors(const Eigen::Vector3f &accel_ned_in)
    {
        accel_ned = accel_ned_in.normalized() * 9.81f;
    }
};

#endif // __ORIENTATION_TRACKER_HPP__
