#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "LVLHAttitudeTarget.hpp"
#include "coordinate_rotators.hpp"
#include "au.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <numbers>

constexpr float m_pif = static_cast<float>(std::numbers::pi);
constexpr float TOL = 1e-4f;

TEST_CASE("LVLHAttitudeTarget: computeDesiredAttitudeECEF")
{
    LVLHAttitudeTarget target;

    std::array<au::QuantityF<au::MetersInEcefFrame>, 3> ecef_position = {
        au::metersInEcefFrame(6378137.0f),
        au::metersInEcefFrame(0.0f),
        au::metersInEcefFrame(0.0f)};

    std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> ecef_velocity = {
        au::metersPerSecondInEcefFrame(0.0f),
        au::metersPerSecondInEcefFrame(7500.0f),
        au::metersPerSecondInEcefFrame(0.0f)};

    Eigen::Quaternionf q_body_to_NED = target.computeDesiredAttitudeECEF(ecef_position, ecef_velocity);

    SUBCASE("Quaternion is normalized")
    {
        CHECK(std::abs(q_body_to_NED.norm() - 1.0f) < TOL);
    }

    SUBCASE("Rotation matrix is orthonormal and right-handed")
    {
        Eigen::Matrix3f R = q_body_to_NED.toRotationMatrix();
        CHECK(std::abs(R.col(0).norm() - 1.0f) < TOL);
        CHECK(std::abs(R.col(1).norm() - 1.0f) < TOL);
        CHECK(std::abs(R.col(2).norm() - 1.0f) < TOL);
        CHECK(std::abs(R.col(0).dot(R.col(1))) < TOL);
        CHECK(std::abs(R.col(0).dot(R.col(2))) < TOL);
        CHECK(std::abs(R.col(1).dot(R.col(2))) < TOL);
        CHECK((R.col(0).cross(R.col(1)) - R.col(2)).norm() < TOL);
    }

    SUBCASE("Z axis points toward nadir")
    {
        Eigen::Vector3f pos{
            ecef_position[0].in(au::metersInEcefFrame),
            ecef_position[1].in(au::metersInEcefFrame),
            ecef_position[2].in(au::metersInEcefFrame)};

        Eigen::Vector3f vel{
            ecef_velocity[0].in(au::metersPerSecondInEcefFrame),
            ecef_velocity[1].in(au::metersPerSecondInEcefFrame),
            ecef_velocity[2].in(au::metersPerSecondInEcefFrame)};

        Eigen::Vector3f z_expected = -pos.normalized();
        Eigen::Vector3f y_LVLH = -pos.cross(vel).normalized();
        Eigen::Vector3f x_LVLH = y_LVLH.cross(z_expected).normalized();

        Eigen::Matrix3f R_LVLH_to_ECEF;
        R_LVLH_to_ECEF.col(0) = x_LVLH;
        R_LVLH_to_ECEF.col(1) = y_LVLH;
        R_LVLH_to_ECEF.col(2) = z_expected;

        Eigen::Vector3f z_actual = R_LVLH_to_ECEF.col(2);
        CHECK(z_expected.dot(z_actual) > 1.0f - TOL);
    }
}

TEST_CASE("AttitudeError: computeQuaternionError and rotationVector")
{
    Eigen::Quaternionf q_desired(Eigen::AngleAxisf(m_pif / 4.0f, Eigen::Vector3f::UnitZ()));
    Eigen::Quaternionf q_current = Eigen::Quaternionf::Identity();

    Eigen::Quaternionf q_error = AttitudeError::computeQuaternionError(q_desired, q_current);
    Eigen::Vector3f rot_vec = AttitudeError::rotationVector(q_error);

    SUBCASE("Quaternion error is correct")
    {
        CHECK(std::abs(q_error.norm() - 1.0f) < TOL);
        CHECK(std::abs(q_error.angularDistance(q_desired)) < TOL);
    }

    SUBCASE("Rotation vector matches expected axis")
    {
        CHECK(std::abs(rot_vec.x()) < TOL);
        CHECK(std::abs(rot_vec.y()) < TOL);
        CHECK(rot_vec.z() > 0.0f);
    }
}

TEST_CASE("AttitudeController: computeOmegaCommand")
{
    AttitudeController controller(0.5f, 0.1f);

    AngularRotation rotation_error{0.1f, -0.2f, 0.3f};
    AngularVelocity omega_measured{0.05f, 0.05f, 0.05f};

    AngularRotation omega_cmd = controller.computeOmegaCommand(rotation_error, omega_measured);

    SUBCASE("Omega command is computed correctly")
    {
        CHECK(std::abs(omega_cmd.x() + 0.5f * 0.1f + 0.1f * 0.05f) < TOL);
        CHECK(std::abs(omega_cmd.y() - 0.5f * 0.2f + 0.1f * 0.05f) < TOL);
        CHECK(std::abs(omega_cmd.z() + 0.5f * 0.3f + 0.1f * 0.05f) < TOL);
    }
}

TEST_CASE("MagnetorquerController: computeDipoleMoment")
{
    AngularRotation omega_cmd{0.01f, 0.02f, 0.03f};
    MagneticField B_body{0.2f, -0.1f, 0.05f};

    DipoleMoment m_cmd = MagnetorquerController::computeDipoleMoment(omega_cmd, B_body);

    SUBCASE("Dipole moment is orthogonal to B")
    {
        CHECK(std::abs(m_cmd.dot(B_body)) < TOL);
    }

    SUBCASE("Dipole moment magnitude is finite")
    {
        CHECK(std::isfinite(m_cmd.norm()));
        CHECK(m_cmd.norm() > 0.0f);
    }

    SUBCASE("Zero field returns zero dipole")
    {
        MagneticField B_zero{Eigen::Vector3f::Zero()};
        DipoleMoment m_zero = MagnetorquerController::computeDipoleMoment(omega_cmd, B_zero);
        CHECK(m_zero.isZero(TOL));
    }
}
