#ifndef __ORIENTATION_SERVICE_HPP__
#define __ORIENTATION_SERVICE_HPP__

#include <functional>
#include "IMU.hpp"
#include "au.hpp"
#include "TimeUtils.hpp"
#include "OrientationTracker.hpp"
#include "Quaternion.hpp"

#ifdef __arm__
#include "usbd_cdc_if.h"
#endif
#ifdef __x86_64__
#include "mock_hal.h"
#endif

Eigen::Vector3f magVector(const MagneticFieldInBodyFrame &magnetic)
{
    return Eigen::Vector3f(
        magnetic[0].in(au::bodys * au::tesla),
        magnetic[1].in(au::bodys * au::tesla),
        magnetic[2].in(au::bodys * au::tesla));
}

Eigen::Vector3f accVector(const AccelerationInBodyFrame &acceleration)
{
    return Eigen::Vector3f(
        acceleration[0].in(au::bodys * au::metersPerSecondSquared),
        acceleration[1].in(au::bodys * au::metersPerSecondSquared),
        acceleration[2].in(au::bodys * au::metersPerSecondSquared));
}

Eigen::Vector3f gyrVector(const AngularVelocityInBodyFrame &angular_velocity)
{
    return Eigen::Vector3f(
        angular_velocity[0].in(au::radiansPerSecondInBodyFrame),
        angular_velocity[1].in(au::radiansPerSecondInBodyFrame),
        angular_velocity[2].in(au::radiansPerSecondInBodyFrame));
}

struct OrientationSolution
{
    enum class Orientation : uint8_t { YAW = 0, ROLL = 1, PITCH = 2 };

    enum class Validity : uint8_t {
        ORIENTATIONS     = 0b0001,
        MAGNETIC_FIELD   = 0b0010,
        ANGULAR_VELOCITY = 0b0100,
        QUATERNION       = 0b1000
    };

    au::QuantityU64<au::Milli<au::Seconds>> timestamp;

    std::array<float, 4> q; // w, x, y, z
    std::array<au::QuantityF<au::DegreesPerSecondInBodyFrame>, 3> angular_velocity;
    std::array<au::QuantityF<au::TeslaInBodyFrame>, 3> magnetic_field;
    std::array<au::QuantityF<au::RadiansInNedFrame>, 3> euler_angles; // yaw, roll, pitch

    uint8_t validity_flags;

    bool has_valid(Validity v) const {
        return validity_flags & static_cast<uint8_t>(v);
    }
};

std::array<au::QuantityF<au::RadiansInNedFrame>, 3> getEulerAngles(const std::array<float, 4> &q)
{
    float sinp = 2.f * (q[0] * q[2] - q[3] * q[1]);
    sinp = std::clamp(sinp, -1.f, 1.f);

    float yaw = atan2f(2.f * (q[0] * q[3] + q[1] * q[2]),
                       1.f - 2.f * (q[2] * q[2] + q[3] * q[3]));
    float pitch = asinf(sinp);
    float roll = atan2f(2.f * (q[0] * q[1] + q[2] * q[3]),
                        1.f - 2.f * (q[1] * q[1] + q[2] * q[2]));

    return {
        au::make_quantity<au::RadiansInNedFrame>(yaw),
        au::make_quantity<au::RadiansInNedFrame>(pitch),
        au::make_quantity<au::RadiansInNedFrame>(roll)
    };
}

template <typename Tracker, typename IMU, typename MAG>
    requires HasBodyGyroscope<IMU> && HasBodyMagnetometer<MAG>
class GyrMagOrientation
{
public:
    GyrMagOrientation() = delete;
    GyrMagOrientation(RTC_HandleTypeDef *hrtc, Tracker &tracker, IMU &imu, MAG &mag) : hrtc_(hrtc), tracker_(tracker), imu_(imu), mag_(mag) {}

    bool predict(std::array<float, 4> &q, au::QuantityU64<au::Milli<au::Seconds>> &timestamp);
    OrientationSolution predict();
    void update(au::QuantityU64<au::Milli<au::Seconds>> &timestamp);

private:
    RTC_HandleTypeDef *hrtc_;
    Tracker &tracker_;
    IMU &imu_;
    MAG &mag_;
};

template <typename Tracker, typename IMU, typename MAG>
    requires HasBodyGyroscope<IMU> && HasBodyMagnetometer<MAG>
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
    requires HasBodyGyroscope<IMU> && HasBodyMagnetometer<MAG>
OrientationSolution GyrMagOrientation<Tracker, IMU, MAG>::predict()
{
    OrientationSolution result{};
    TimeUtils::RTCDateTimeSubseconds rtc;
    HAL_RTC_GetTime(hrtc_, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc_, &rtc.date, RTC_FORMAT_BIN);
    result.timestamp = TimeUtils::from_rtc(rtc, hrtc_->Init.SynchPrediv);

    auto optional_angular = imu_.readGyroscope();
    auto optional_magnetic = imu_.readMagnetometer();

    if (optional_angular.has_value()) {
        result.angular_velocity = optional_angular.value();
        result.validity_flags |= static_cast<uint8_t>(OrientationSolution::Validity::ANGULAR_VELOCITY);
    }

    if (optional_magnetic.has_value()) {
        result.magnetic_field = optional_magnetic.value();
        result.validity_flags |= static_cast<uint8_t>(OrientationSolution::Validity::MAGNETIC_FIELD);
    }

    update(result.timestamp);  // updates tracker
    auto q_ = tracker_.getOrientation();
    result.q = { q_.w(), q_.x(), q_.y(), q_.z() };
    result.validity_flags |= static_cast<uint8_t>(OrientationSolution::Validity::QUATERNION);

    result.euler_angles = getEulerAngles({ q_.w(), q_.x(), q_.y(), q_.z() });
    result.validity_flags |= static_cast<uint8_t>(OrientationSolution::Validity::ORIENTATIONS);

    return result;
}

template <typename Tracker, typename IMU, typename MAG>
    requires HasBodyGyroscope<IMU> && HasBodyMagnetometer<MAG>
void GyrMagOrientation<Tracker, IMU, MAG>::update(au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
{
    auto optional_angular = imu_.readGyroscope();
    auto optional_magnetic = imu_.readMagnetometer();
    if (optional_angular.has_value() && optional_magnetic.has_value())
    {
        tracker_.updateSensorFusion(gyrVector(optional_angular.value()), magVector(optional_magnetic.value()), timestamp);
    }
}

template <typename Tracker, typename IMU, typename MAG>
    requires HasBodyGyroscope<IMU> && HasBodyMagnetometer<MAG> && HasBodyAccelerometer<IMU>
class AccGyrMagOrientation
{
public:
    AccGyrMagOrientation() = delete;
    AccGyrMagOrientation(RTC_HandleTypeDef *hrtc, Tracker &tracker, IMU &imu, MAG &mag) : hrtc_(hrtc), tracker_(tracker), imu_(imu), mag_(mag) {}

    bool predict(std::array<float, 4> &q, au::QuantityU64<au::Milli<au::Seconds>> &timestamp);
    OrientationSolution predict();
    void update(au::QuantityU64<au::Milli<au::Seconds>> &timestamp);

private:
    RTC_HandleTypeDef *hrtc_;
    Tracker &tracker_;
    IMU &imu_;
    MAG &mag_;
};

template <typename Tracker, typename IMU, typename MAG>
    requires HasBodyGyroscope<IMU> && HasBodyMagnetometer<MAG> && HasBodyAccelerometer<IMU>
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
    requires HasBodyGyroscope<IMU> && HasBodyMagnetometer<MAG> && HasBodyAccelerometer<IMU>
OrientationSolution AccGyrMagOrientation<Tracker, IMU, MAG>::predict()
{
    OrientationSolution result{};

    // Timestamp from RTC
    TimeUtils::RTCDateTimeSubseconds rtc;
    HAL_RTC_GetTime(hrtc_, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc_, &rtc.date, RTC_FORMAT_BIN);
    result.timestamp = TimeUtils::from_rtc(rtc, hrtc_->Init.SynchPrediv);

    // Sensor reads
    auto optional_angular   = imu_.readGyroscope();
    auto optional_accel     = imu_.readAccelerometer();
    auto optional_magnetic  = imu_.readMagnetometer();

    if (optional_angular.has_value()) {
        result.angular_velocity = optional_angular.value();
        result.validity_flags |= static_cast<uint8_t>(OrientationSolution::Validity::ANGULAR_VELOCITY);
    }

    if (optional_magnetic.has_value()) {
        result.magnetic_field = optional_magnetic.value();
        result.validity_flags |= static_cast<uint8_t>(OrientationSolution::Validity::MAGNETIC_FIELD);
    }

    // Update tracker if all required data is present
    if (optional_angular && optional_accel && optional_magnetic) {
        tracker_.updateSensorFusion(
            gyrVector(optional_angular.value()),
            accVector(optional_accel.value()),
            magVector(optional_magnetic.value()),
            result.timestamp);
    }

    // Orientation from tracker
    auto q_ = tracker_.getOrientation();
    result.q = { q_.w(), q_.x(), q_.y(), q_.z() };
    result.validity_flags |= static_cast<uint8_t>(OrientationSolution::Validity::QUATERNION);

    result.euler_angles = getEulerAngles({ q_.w(), q_.x(), q_.y(), q_.z() });
    result.validity_flags |= static_cast<uint8_t>(OrientationSolution::Validity::ORIENTATIONS);

    return result;
}

template <typename Tracker, typename IMU, typename MAG>
    requires HasBodyGyroscope<IMU> && HasBodyMagnetometer<MAG> && HasBodyAccelerometer<IMU>
void AccGyrMagOrientation<Tracker, IMU, MAG>::update(au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
{
    auto optional_angular = imu_.readGyroscope();
    auto optional_accel = imu_.readAccelerometer();
    auto optional_magnetic = imu_.readMagnetometer();

    if (optional_angular.has_value() && optional_accel.has_value() && optional_magnetic.has_value())
    {
        tracker_.updateSensorFusion(gyrVector(optional_angular.value()), accVector(optional_accel.value()), magVector(optional_magnetic.value()), timestamp);
    }
}

template <typename Tracker, typename IMU>
    requires HasBodyGyroscope<IMU> && HasBodyAccelerometer<IMU>
class AccGyrOrientation
{
public:
    AccGyrOrientation() = delete;
    AccGyrOrientation(RTC_HandleTypeDef *hrtc, Tracker &tracker, IMU &imu)
        : hrtc_(hrtc), tracker_(tracker), imu_(imu) {}

    bool predict(std::array<float, 4> &q, au::QuantityU64<au::Milli<au::Seconds>> &timestamp);
    OrientationSolution predict();
    void update(au::QuantityU64<au::Milli<au::Seconds>> &timestamp);

private:
    RTC_HandleTypeDef *hrtc_;
    Tracker &tracker_;
    IMU &imu_;
};

template <typename Tracker, typename IMU>
    requires HasBodyGyroscope<IMU> && HasBodyAccelerometer<IMU>
bool AccGyrOrientation<Tracker, IMU>::predict(std::array<float, 4> &q, au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
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

template <typename Tracker, typename IMU>
    requires HasBodyGyroscope<IMU> && HasBodyAccelerometer<IMU>
OrientationSolution AccGyrOrientation<Tracker, IMU>::predict()
{
    OrientationSolution result{};

    // Timestamp from RTC
    TimeUtils::RTCDateTimeSubseconds rtc;
    HAL_RTC_GetTime(hrtc_, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc_, &rtc.date, RTC_FORMAT_BIN);
    result.timestamp = TimeUtils::from_rtc(rtc, hrtc_->Init.SynchPrediv);

    // Sensor reads
    auto optional_angular = imu_.readGyroscope();
    auto optional_accel   = imu_.readAccelerometer();

    if (optional_angular.has_value()) {
        result.angular_velocity = optional_angular.value();
        result.validity_flags |= static_cast<uint8_t>(OrientationSolution::Validity::ANGULAR_VELOCITY);
    }

    // Update tracker if both sensors are available
    if (optional_angular && optional_accel) {
        tracker_.updateSensorFusion(
            gyrVector(optional_angular.value()),
            accVector(optional_accel.value()),
            result.timestamp);
    }

    // Orientation from tracker
    auto q_ = tracker_.getOrientation();
    result.q = { q_.w(), q_.x(), q_.y(), q_.z() };
    result.validity_flags |= static_cast<uint8_t>(OrientationSolution::Validity::QUATERNION);

    result.euler_angles = getEulerAngles({ q_.w(), q_.x(), q_.y(), q_.z() });
    result.validity_flags |= static_cast<uint8_t>(OrientationSolution::Validity::ORIENTATIONS);

    return result;
}

template <typename Tracker, typename IMU>
    requires HasBodyGyroscope<IMU> && HasBodyAccelerometer<IMU>
void AccGyrOrientation<Tracker, IMU>::update(au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
{
    auto optional_angular = imu_.readGyroscope();
    auto optional_accel = imu_.readAccelerometer();

    if (optional_angular.has_value() && optional_accel.has_value())
    {
        tracker_.updateSensorFusion(gyrVector(optional_angular.value()), accVector(optional_accel.value()), timestamp);
    }
}

#endif // __ORIENTATION_SERVICE_HPP__
