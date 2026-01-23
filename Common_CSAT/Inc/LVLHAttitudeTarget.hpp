// LVLHAttitudeTarget.hpp

#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "NamedVector3f.hpp"
#include <array>
#include "au.hpp"

class LVLHAttitudeTarget
{
public:
    LVLHAttitudeTarget() = default;

    // Computes desired body-to-NED quaternion for geocentric nadir pointing
    static Eigen::Quaternionf computeDesiredAttitudeECEF(
        const std::array<au::QuantityF<au::MetersInEcefFrame>, 3> &ecef_position,
        const std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> &ecef_velocity);
};

class AttitudeError
{
public:
    static Eigen::Quaternionf computeQuaternionError(
        const Eigen::Quaternionf &q_desired,
        const Eigen::Quaternionf &q_current);

    static AngularRotation rotationVector(const Eigen::Quaternionf &q_error);
};

class AttitudeController
{
public:
    AttitudeController(float kp, float kd) : Kp(kp), Kd(kd) {}

    AngularRotation computeOmegaCommand(
        const AngularRotation &rotation_error,
        const AngularVelocity &omega_measured) const;

private:
    float Kp, Kd;
};

class MagnetorquerController
{
public:
    static DipoleMoment computeDipoleMoment(
        const AngularRotation &omega_cmd,
        const MagneticField &B_body);
};
