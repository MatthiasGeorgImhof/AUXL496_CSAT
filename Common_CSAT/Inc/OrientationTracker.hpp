#pragma once
#include <Eigen/Dense>
#include <functional>
#include "Kalman.hpp"
#include "Quaternion.hpp"
#include "IMU.hpp"

#include "au.hpp"

#include "TimeUtils.hpp"
#ifdef __arm__
#include "usbd_cdc_if.h"
#endif
#ifdef __x86_64__
#include "mock_hal.h"
#endif

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

    void updateGyro(const Eigen::Vector3f &omega, au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        predictTo(timestamp);
        ekf.stateVector.template segment<3>(4) = omega;
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
        const auto &x = ekf.stateVector;
        std::cout << "\n[Tracker State] " << label << "\n";
        std::cout << "Quaternion: [" << x(0) << ", " << x(1) << ", " << x(2) << ", " << x(3) << "]\n";
        std::cout << "Angular rate: [" << x(4) << ", " << x(5) << ", " << x(6) << "]\n";
        std::cout << "Orientation: " << getYawPitchRoll().transpose() * 180.0f / M_PIf << " deg\n";
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
    Eigen::Vector3f mag_ned;

public:
    GyrMagOrientationTracker() : BaseOrientationTracker<StateSize, MeasurementSize>(
                                     Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-5f,
                                     Eigen::Matrix3f::Identity() * 0.01f,
                                     Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-5f,
                                     []{ StateVector x; x << 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f; return x; }())
    {
    }

    void updateMagnetometer(const Eigen::Vector3f &imu_meas_body, au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        BaseOrientationTracker<StateSize, MeasurementSize>::predictTo(timestamp);

        auto h = [&](const StateVector &x)
        {
            Eigen::Quaternionf q(x(3), x(0), x(1), x(2));
            return q.conjugate() * mag_ned;
        };

        auto &x = BaseOrientationTracker<StateSize, MeasurementSize>::ekf.stateVector;
        Eigen::Quaternionf q_hat(x(3), x(0), x(1), x(2));
        float qw = q_hat.w(), qx = q_hat.x(), qy = q_hat.y(), qz = q_hat.z();
        float mx = mag_ned(0), my = mag_ned(1), mz = mag_ned(2);

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

        BaseOrientationTracker<StateSize, MeasurementSize>::ekf.updateEKF(h, H_jac, imu_meas_body);

        Eigen::Quaternionf q_corr(x(3), x(0), x(1), x(2));
        q_corr.normalize();
        x(0) = q_corr.x();
        x(1) = q_corr.y();
        x(2) = q_corr.z();
        x(3) = q_corr.w();
    }

    void setMagneticReference(float mx, float my, float mz)
    {
        mag_ned = Eigen::Vector3f(mx, my, mz);
        mag_ned.normalize();
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
    Eigen::Vector3f mag_ned;

public:
    AccGyrMagOrientationTracker()
        : BaseOrientationTracker<StateSize, MeasurementSize>(
              Eigen::Matrix<float, StateSize, StateSize>::Identity() * 0.1f,
              Eigen::Matrix<float, MeasurementSize, MeasurementSize>::Identity() * 0.001f,
              Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-5f,
              []
              { StateVector x; x << 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f; return x; }())
    {
    }

    void updateAccelerometerMagnetometer(const Eigen::Vector3f &accel_body,
                                         const Eigen::Vector3f &mag_body,
                                         au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        BaseOrientationTracker<StateSize, MeasurementSize>::predictTo(timestamp);

        Eigen::Vector3f accel_ned(0.f, 0.f, 9.81f); // Gravity points down


        // Combined measurement vector
        Measurement combined_meas;
        combined_meas.template segment<3>(0) = accel_body;
        combined_meas.template segment<3>(3) = mag_body;

        auto h = [&](const StateVector &x)
        {
            Eigen::Quaternionf q(x(3), x(0), x(1), x(2));
            Eigen::Vector3f predicted_accel_body = q.conjugate() * accel_ned;
            Eigen::Vector3f predicted_mag_body = q.conjugate() * mag_ned;

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
        Eigen::Matrix<float, 3, 4> J_mag = computeAnalyticalJacobian(q_raw, mag_ned);
        J_mag = normalizeAnalyticalJacobian(J_mag, q_raw, mag_ned);

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

    void setMagneticReference(float mx, float my, float mz)
    {
        mag_ned = Eigen::Vector3f(mx, my, mz);
        mag_ned.normalize();
    }

};

#
#
#
#
#
#
#

template <typename Tracker, typename IMU, typename MAG>
requires HasBodyGyroscope<IMU> && HasBodyMagnetometer<MAG>
class GyrMagOrientation
{
public:
    GyrMagOrientation() = delete;

    GyrMagOrientation(RTC_HandleTypeDef *hrtc, Tracker &tracker, IMU &imu, MAG &mag, uint16_t imu_rate = 1, uint16_t mag_rate = 1) : hrtc_(hrtc), tracker_(tracker), imu_(imu), mag_(mag), imu_rate_(imu_rate), mag_rate_(mag_rate), imu_counter_(0U), mag_counter_(0U) {}

    bool predict(std::array<float, 4> &q, au::QuantityU64<au::Milli<au::Seconds>> &timestamp);

    void update(au::QuantityU64<au::Milli<au::Seconds>> &timestamp);

private:
    RTC_HandleTypeDef *hrtc_;
    Tracker &tracker_;
    IMU &imu_;
    MAG &mag_;

    uint16_t imu_rate_;
    uint16_t mag_rate_;

    uint16_t imu_counter_;
    uint16_t mag_counter_;
};

template <typename Tracker, typename IMU, typename MAG>
bool GyrMagOrientation<Tracker, IMU, MAG>::predict(std::array<float, 4> &q, au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
{
    TimeUtils::RTCDateTimeSubseconds rtc;
    HAL_RTC_GetTime(hrtc_, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc_, &rtc.date, RTC_FORMAT_BIN);
    timestamp = TimeUtils::from_rtc(rtc, hrtc_->Init.SynchPrediv);
    update(timestamp);

    auto q_ = tracker_.getOrientation();
    q[0] = q_.w();
    q[1] = q_.x();
    q[2] = q_.y();
    q[3] = q_.z();

    return true;
}

template <typename Tracker, typename IMU, typename MAG>
void GyrMagOrientation<Tracker, IMU, MAG>::update(au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
{
    if (imu_counter_ % imu_rate_ == 0)
    {
        auto optional_angular = imu_.readGyroscope();
        if (optional_angular.has_value())
        {
            auto angular = optional_angular.value();
            tracker_.updateGyro(Eigen::Vector3f(angular[0].in(au::radiansPerSecondInBodyFrame), angular[1].in(au::radiansPerSecondInBodyFrame), angular[2].in(au::radiansPerSecondInBodyFrame)), timestamp);
        }
    }

    if (mag_counter_ % mag_rate_ == 0)
    {
        auto optional_magnetic = imu_.readMagnetometer();
        if (optional_magnetic.has_value())
        {
            auto magnetic = optional_magnetic.value();
            tracker_.updateMagnetometer(Eigen::Vector3f(magnetic[0].in(au::bodys * au::tesla), magnetic[1].in(au::bodys * au::tesla), magnetic[2].in(au::bodys * au::tesla)), timestamp);
        }
    }

    ++imu_counter_;
    ++mag_counter_;
}

template <typename Tracker, typename IMU, typename MAG>
requires HasBodyGyroscope<IMU> && HasBodyMagnetometer<MAG> && HasBodyAccelerometer<IMU>
class AccGyrMagOrientation
{
public:
    AccGyrMagOrientation() = delete;

    AccGyrMagOrientation(RTC_HandleTypeDef *hrtc, Tracker &tracker, IMU &imu, MAG &mag, uint16_t imu_rate = 1, uint16_t mag_rate = 1)
        : hrtc_(hrtc), tracker_(tracker), imu_(imu), mag_(mag), imu_rate_(imu_rate), mag_rate_(mag_rate), imu_counter_(0U), mag_counter_(0U)
    {
    }

    bool predict(std::array<float, 4> &q, au::QuantityU64<au::Milli<au::Seconds>> &timestamp);

    void update(au::QuantityU64<au::Milli<au::Seconds>> &timestamp);

private:
    RTC_HandleTypeDef *hrtc_;
    Tracker &tracker_;
    IMU &imu_;
    MAG &mag_;

    uint16_t imu_rate_;
    uint16_t mag_rate_;

    uint16_t imu_counter_;
    uint16_t mag_counter_;
};

template <typename Tracker, typename IMU, typename MAG>
bool AccGyrMagOrientation<Tracker, IMU, MAG>::predict(std::array<float, 4> &q, au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
{
    TimeUtils::RTCDateTimeSubseconds rtc;
    HAL_RTC_GetTime(hrtc_, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc_, &rtc.date, RTC_FORMAT_BIN);
    timestamp = TimeUtils::from_rtc(rtc, hrtc_->Init.SynchPrediv);
    update(timestamp);

    auto q_ = tracker_.getOrientation();
    q[0] = q_.w();
    q[1] = q_.x();
    q[2] = q_.y();
    q[3] = q_.z();

    return true;
}

template <typename Tracker, typename IMU, typename MAG>
void AccGyrMagOrientation<Tracker, IMU, MAG>::update(au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
{
    if (imu_counter_ % imu_rate_ == 0)
    {
        auto optional_angular = imu_.getGyroscope();
        if (optional_angular.has_value())
        {
            auto angular = optional_angular.value();
            tracker_.updateGyro(Eigen::Vector3f(angular.x.in(au::radiansPerSecondInBodyFrame), angular.y.in(au::radiansPerSecondInBodyFrame), angular.z.in(au::radiansPerSecondInBodyFrame)), timestamp);

            auto optional_accel = imu_.getAccelerometer();
            auto optional_magnetic = imu_.getMagnetometer();

            if (optional_accel.has_value() && optional_magnetic.has_value())
            {
                auto accel = optional_accel.value();
                auto magnetic = optional_magnetic.value();

                Eigen::Vector3f accel_body(accel.x.in(au::bodys * au::meters / (au::seconds * au::seconds)),
                                           accel.y.in(au::bodys * au::meters / (au::seconds * au::seconds)),
                                           accel.z.in(au::bodys * au::meters / (au::seconds * au::seconds)));
                Eigen::Vector3f mag_body(magnetic.x.in(au::bodys * au::tesla), magnetic.y.in(au::bodys * au::tesla), magnetic.z.in(au::bodys * au::tesla));

                tracker_.updateAccelerometerMagnetometer(accel_body, mag_body, timestamp);
            }
        }
    }

    ++imu_counter_;
    ++mag_counter_;
}
