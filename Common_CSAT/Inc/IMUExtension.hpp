#ifndef __IMUEXTENSION__HPP_
#define __IMUEXTENSION__HPP_

#include <concepts>
#include <optional>
#include <cmath>
#include <array>
#include "au.hpp"

#include "IMU.hpp"

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "coordinate_rotators.hpp"

struct NoGravityCompensation {
    static Eigen::Vector3f apply(const Eigen::Vector3f& a_ned) {
        return a_ned;
    }
};

struct SubtractGravityInNED {
    constexpr static float gravity = 9.81f; // m/s²
    static Eigen::Vector3f apply(const Eigen::Vector3f& a_ned) {
        return Eigen::Vector3f(a_ned[0], a_ned[1], gravity - a_ned[2]);
    }
};

template <typename IMU, typename OrientationProvider, typename PositionProvider, typename GravityPolicy = NoGravityCompensation>
class IMUAccInECEFWithPolicy {
public:
    IMUAccInECEFWithPolicy(IMU& imu, OrientationProvider& orientation, PositionProvider& position)
        : imu_(imu), orientation_(orientation), position_(position) {}

    std::optional<std::array<au::QuantityF<au::MetersPerSecondSquaredInEcefFrame>, 3>> readAccelerometer() {
        auto optional_accel_body = imu_.readAccelerometer();
        if (!optional_accel_body.has_value()) return std::nullopt;
        const auto& accel_body = optional_accel_body.value();

        au::QuantityU64<au::Milli<au::Seconds>> timestamp;
        std::array<float, 4> q_body_to_ned;
        orientation_.predict(q_body_to_ned, timestamp);

        auto state = position_.getState();
        std::array<au::QuantityF<au::MetersInEcefFrame>, 3> pos_ecef = {
            au::make_quantity<au::MetersInEcefFrame>(state[0]),
            au::make_quantity<au::MetersInEcefFrame>(state[1]),
            au::make_quantity<au::MetersInEcefFrame>(state[2])
        };

        Eigen::Vector3f a_body(
            accel_body[0].in(au::metersPerSecondSquaredInBodyFrame),
            accel_body[1].in(au::metersPerSecondSquaredInBodyFrame),
            accel_body[2].in(au::metersPerSecondSquaredInBodyFrame));

        Eigen::Quaternionf q_b2n(q_body_to_ned[0], q_body_to_ned[1], q_body_to_ned[2], q_body_to_ned[3]);
        q_b2n.normalize();
        Eigen::Vector3f a_ned = q_b2n * a_body;

        a_ned = GravityPolicy::apply(a_ned);

        Eigen::Matrix3f R_ned_to_ecef = coordinate_rotators::computeNEDtoECEFRotation(pos_ecef);
        Eigen::Vector3f a_ecef = R_ned_to_ecef * a_ned;

        return std::array<au::QuantityF<au::MetersPerSecondSquaredInEcefFrame>, 3>{
            au::make_quantity<au::MetersPerSecondSquaredInEcefFrame>(a_ecef.x()),
            au::make_quantity<au::MetersPerSecondSquaredInEcefFrame>(a_ecef.y()),
            au::make_quantity<au::MetersPerSecondSquaredInEcefFrame>(a_ecef.z())
        };
    }

private:
    IMU& imu_;
    OrientationProvider& orientation_;
    PositionProvider& position_;
};

template <typename IMU>
class IMUWithMagneticCorrection
{
public:
    IMUWithMagneticCorrection(IMU &imu,
                              const Eigen::Vector3f &hardIronOffset,
                              const Eigen::Matrix3f &softIronMatrix)
        : imu_(imu), hardIronOffset_(hardIronOffset), softIronMatrix_(softIronMatrix) {}

    std::optional<MagneticFieldInBodyFrame> readMagnetometer()
    {
        auto raw = imu_.readMagnetometer();
        if (!raw.has_value())
            return std::nullopt;

        const auto &m_raw = raw.value();

        // Convert to Eigen vector
        Eigen::Vector3f m_vec(
            m_raw[0].in(au::teslaInBodyFrame),
            m_raw[1].in(au::teslaInBodyFrame),
            m_raw[2].in(au::teslaInBodyFrame));

        // Apply hard and soft iron corrections
        Eigen::Vector3f m_corrected = softIronMatrix_ * (m_vec - hardIronOffset_);

        return MagneticFieldInBodyFrame{
            au::make_quantity<au::TeslaInBodyFrame>(m_corrected.x()),
            au::make_quantity<au::TeslaInBodyFrame>(m_corrected.y()),
            au::make_quantity<au::TeslaInBodyFrame>(m_corrected.z())};
    }

private:
    IMU &imu_;
    Eigen::Vector3f hardIronOffset_;
    Eigen::Matrix3f softIronMatrix_;
};

#endif // __IMUEXTENSION__HPP_