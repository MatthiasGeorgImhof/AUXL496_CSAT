// LVLHAttitudeTarget.cpp

#include "LVLHAttitudeTarget.hpp"
#include "coordinate_rotators.hpp"
#include <cstdio>
#include <iostream>

Eigen::Quaternionf LVLHAttitudeTarget::computeDesiredAttitudeECEF(
    const std::array<au::QuantityF<au::MetersInEcefFrame>, 3> &ecef_position,
    const std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> &ecef_velocity)
{
    // Step 1: Convert to Eigen vectors
    Eigen::Vector3f pos_ecef{
        ecef_position[0].in(au::metersInEcefFrame),
        ecef_position[1].in(au::metersInEcefFrame),
        ecef_position[2].in(au::metersInEcefFrame)};

    Eigen::Vector3f vel_ecef{
        ecef_velocity[0].in(au::metersPerSecondInEcefFrame),
        ecef_velocity[1].in(au::metersPerSecondInEcefFrame),
        ecef_velocity[2].in(au::metersPerSecondInEcefFrame)};

    // Step 2: Build LVLH frame in ECEF
    Eigen::Vector3f z_LVLH = -pos_ecef.normalized();                 // Nadir
    Eigen::Vector3f y_LVLH = -pos_ecef.cross(vel_ecef).normalized(); // Opposite angular momentum
    Eigen::Vector3f x_LVLH = y_LVLH.cross(z_LVLH).normalized();      // Completes right-handed frame

    Eigen::Matrix3f R_LVLH_to_ECEF;
    R_LVLH_to_ECEF.col(0) = x_LVLH;
    R_LVLH_to_ECEF.col(1) = y_LVLH;
    R_LVLH_to_ECEF.col(2) = z_LVLH;

    // Step 3: Bridge ECEF to NED
    Eigen::Matrix3f R_NED_to_ECEF = coordinate_rotators::computeNEDtoECEFRotation(ecef_position);
    Eigen::Matrix3f R_LVLH_to_NED = R_NED_to_ECEF.inverse() * R_LVLH_to_ECEF;

    // Step 4: Return body-to-NED quaternion
    return Eigen::Quaternionf(R_LVLH_to_NED);
}

Eigen::Quaternionf AttitudeError::computeQuaternionError(
    const Eigen::Quaternionf &q_desired,
    const Eigen::Quaternionf &q_current)
{
    return q_desired * q_current.conjugate(); // Rotation from current to desired
}

AngularRotation AttitudeError::rotationVector(const Eigen::Quaternionf &q_error)
{
    return AngularRotation(q_error.vec().eval());
}

AngularRotation AttitudeController::computeOmegaCommand(
    const AngularRotation &rotation_error,
    const AngularVelocity &omega_measured) const
{
    return AngularRotation((-Kp * rotation_error.value - Kd * omega_measured.value).eval());
}

DipoleMoment MagnetorquerController::computeDipoleMoment(
    const AngularRotation &omega_cmd,
    const MagneticField &B_body)
{
    constexpr float TOL = 1e-12f; // picotesla
    constexpr float gain = 1.0f;
    float B_norm_sq = B_body.squaredNorm();

    // std::cout << "Eigen::Vector3f MagnetorquerController::computeDipoleMoment" << "\n";
    // std::cout << "omega_cmd: " << omega_cmd.transpose() << "\n";
    // std::cout << "B_body: " << B_body.transpose() << "\n";
    // std::cout << "m_cmd: " << omega_cmd.cross(B_body).transpose() << "\n";
    // std::cout << "B_norm_sq: " << B_norm_sq << "\n";

    if (B_norm_sq < TOL)
        return DipoleMoment(Eigen::Vector3f::Zero().eval()); // Avoid division by zero
    return DipoleMoment((gain * omega_cmd.cross(B_body.normalized())).eval());
}
