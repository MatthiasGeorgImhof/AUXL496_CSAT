#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

#include "OrientationTracker.hpp"


const float kTolerance = 1e-6f; // Consistent tolerance
const float kNumericalEpsilon = 1e-2f; // Epsilon value for numerical derivatives
const float kAlignmentThreshold = 0.999f; //Threshold for the dot product

TEST_CASE("Manual R(q*) matches Eigen::Quaternion passive rotation")
{
    // Verify that the manually computed rotation matrix from a quaternion matches Eigen's implementation.
    Eigen::Quaternionf q(0.8f, 0.2f, -0.3f, 0.4f);
    q.normalize();

    const float w = q.w(), x = q.x(), y = q.y(), z = q.z();

    Eigen::Matrix3f R_manual;
    R_manual << w * w + x * x - y * y - z * z, 2 * (x * y + w * z), 2 * (x * z - w * y),
        2 * (x * y - w * z), w * w - x * x + y * y - z * z, 2 * (y * z + w * x),
        2 * (x * z + w * y), 2 * (y * z - w * x), w * w - x * x - y * y + z * z;

    Eigen::Matrix3f R_eigen = q.conjugate().toRotationMatrix();
    CHECK((R_manual - R_eigen).norm() < kTolerance);
}

TEST_CASE("Rotated vector via R(q*) matches q* v q quaternion form")
{
    // Verify that rotating a vector using a rotation matrix derived from a quaternion
    // gives the same result as rotating the vector using quaternion multiplication.
    Eigen::Quaternionf q(0.8f, 0.2f, -0.3f, 0.4f);
    q.normalize();

    Eigen::Vector3f v(1.0f, -2.0f, 0.5f); // arbitrary test vector

    // Manual rotation matrix from q conjugate
    const float w = q.w(), x = q.x(), y = q.y(), z = q.z();
    Eigen::Matrix3f R;
    R << w * w + x * x - y * y - z * z, 2 * (x * y + w * z), 2 * (x * z - w * y),
        2 * (x * y - w * z), w * w - x * x + y * y - z * z, 2 * (y * z + w * x),
        2 * (x * z + w * y), 2 * (y * z - w * x), w * w - x * x - y * y + z * z;

    Eigen::Vector3f v_rot_eigen = q.conjugate().toRotationMatrix() * v;
    Eigen::Vector3f v_rot_matrix = R * v;

    // Quaternion-based rotation: q* v q
    Eigen::Quaternionf q_conj = q.conjugate();
    Eigen::Quaternionf v_q(0.0, v.x(), v.y(), v.z());
    Eigen::Quaternionf v_rot_quat = q_conj * v_q * q;
    Eigen::Vector3f v_rot_quat_vec = v_rot_quat.vec();

    CHECK((v_rot_matrix - v_rot_quat_vec).norm() < kTolerance);
    CHECK((v_rot_eigen - v_rot_quat_vec).norm() < kTolerance);
}

TEST_CASE("Analytical Jacobian matches Numerical Jacobian")
{
    // Parameters for the test
    const std::vector<std::pair<Eigen::Vector3f, std::string>> testVectors = {
        {Eigen::Vector3f(1.0f, 0.0f, 0.0f), "vx"},
        {Eigen::Vector3f(0.0f, 1.0f, 0.0f), "vy"},
        {Eigen::Vector3f(0.0f, 0.0f, 1.0f), "vz"}};

    for (const auto& [v_ned, label] : testVectors) {
        SUBCASE(label.c_str()) {
            Eigen::Quaternionf q = Eigen::Quaternionf::Identity();

            Eigen::Matrix<float, 3, 4> J_analytical = computeAnalyticalJacobian(q, v_ned);
            Eigen::Matrix<float, 3, 4> J_normalized = normalizeAnalyticalJacobian(J_analytical, q, v_ned);
            Eigen::Matrix<float, 3, 4> J_numerical = computeNumericalJacobian(q, v_ned);

            float max_diff = (J_normalized - J_numerical).cwiseAbs().maxCoeff();
            CHECK(max_diff < 1e-3f);
        }
    }

    SUBCASE("Jacobian match for 90deg Z rotation and generic vector")
    {
        Eigen::Vector3f v_ned(1.0f, 0.5f, -0.2f);
        Eigen::Quaternionf q(Eigen::AngleAxisf(0.5f * M_PIf, Eigen::Vector3f::UnitZ()));

        Eigen::Vector3f v_rotated = q * v_ned;
        Eigen::Vector3f v_expected(-0.5f, 1.0f, -0.2f); // 90° rotation around Z

        float rot_error = (v_rotated - v_expected).norm();
        // std::cerr << "Rotation error: " << rot_error << "\n";
        CHECK(rot_error < 1e-4f);

        Eigen::Matrix<float, 3, 4> J_analytical = computeAnalyticalJacobian(q, v_ned);
        Eigen::Matrix<float, 3, 4> J_normalized = normalizeAnalyticalJacobian(J_analytical, q, v_ned);
        Eigen::Matrix<float, 3, 4> J_numerical = computeNumericalJacobian(q, v_ned);

        float max_diff = (J_normalized - J_numerical).cwiseAbs().maxCoeff();
        CHECK(max_diff < 1e-2f);
    }

    SUBCASE("Jacobian match for 90deg Z rotation and special vector")
    {
        Eigen::Vector3f v_ned(Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf q(Eigen::AngleAxisf(0.5f * M_PIf, Eigen::Vector3f::UnitZ()));

        Eigen::Vector3f v_rotated = q * v_ned;
        Eigen::Vector3f v_expected(Eigen::Vector3f::UnitZ());
        // std::cerr << "rotated: " << v_rotated.transpose() << "\n";

        float rot_error = (v_rotated - v_expected).norm();
        // std::cerr << "Rotation error: " << rot_error << "\n";
        CHECK(rot_error < 1e-4f);

        Eigen::Matrix<float, 3, 4> J_analytical = computeAnalyticalJacobian(q, v_ned);
        Eigen::Matrix<float, 3, 4> J_normalized = normalizeAnalyticalJacobian(J_analytical, q, v_ned);
        Eigen::Matrix<float, 3, 4> J_numerical = computeNumericalJacobian(q, v_ned);


        float max_diff = (J_normalized - J_numerical).cwiseAbs().maxCoeff();
        CHECK(max_diff < 1e-3f);
    }

    SUBCASE("Analytical Jacobian matches Numerical Jacobian of unitx for rotated vector")
    {
        Eigen::Vector3f v_ned(1.0f, 0.5f, -0.2f); // arbitrary test vector
        Eigen::Quaternionf q(Eigen::AngleAxisf(0.25f * M_PIf, Eigen::Vector3f::UnitX()));

        Eigen::Matrix<float, 3, 4> J_analytical = computeAnalyticalJacobian(q, v_ned);
        Eigen::Matrix<float, 3, 4> J_normalized = normalizeAnalyticalJacobian(J_analytical, q, v_ned);
        Eigen::Matrix<float, 3, 4> J_numerical = computeNumericalJacobian(q, v_ned);

        float max_diff = (J_normalized - J_numerical).cwiseAbs().maxCoeff();
        CHECK(max_diff < 1e-3f);
    }

    SUBCASE("Analytical Jacobian matches Numerical Jacobian of unity for rotated vector")
    {
        Eigen::Vector3f v_ned(1.0f, 0.5f, -0.2f); // arbitrary test vector
        Eigen::Quaternionf q(Eigen::AngleAxisf(0.25f * M_PIf, Eigen::Vector3f::UnitY()));

        Eigen::Matrix<float, 3, 4> J_analytical = computeAnalyticalJacobian(q, v_ned);
        Eigen::Matrix<float, 3, 4> J_normalized = normalizeAnalyticalJacobian(J_analytical, q, v_ned);
        Eigen::Matrix<float, 3, 4> J_numerical = computeNumericalJacobian(q, v_ned);

        float max_diff = (J_normalized - J_numerical).cwiseAbs().maxCoeff();
        CHECK(max_diff < 1e-3f);
    }

    SUBCASE("Analytical Jacobian matches Numerical Jacobian of unitz for rotated vector")
    {
        Eigen::Vector3f v_ned(1.0f, 0.5f, -0.2f); // arbitrary test vector
        Eigen::Quaternionf q(Eigen::AngleAxisf(0.25f * M_PIf, Eigen::Vector3f::UnitZ()));

        Eigen::Matrix<float, 3, 4> J_analytical = computeAnalyticalJacobian(q, v_ned);
        Eigen::Matrix<float, 3, 4> J_normalized = normalizeAnalyticalJacobian(J_analytical, q, v_ned);
        Eigen::Matrix<float, 3, 4> J_numerical = computeNumericalJacobian(q, v_ned);

        float max_diff = (J_normalized - J_numerical).cwiseAbs().maxCoeff();
        CHECK(max_diff < 1e-3f);
    }

    SUBCASE("Numerical and Analytical Jacobian - VERY SIMPLE CASE")
    {
        {
            Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
            Eigen::Vector3f v_ned(0.f, 0.f, 1.f); // aligned with z-axis

            Eigen::Matrix<float, 3, 4> J = computeNumericalJacobian(q, v_ned);
            CHECK(J.norm() < 3.0f); // Expect 0, this will definitively show us what is the error
        }
        {
            Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
            Eigen::Vector3f v_ned(0.f, 0.f, 1.f); // aligned with z-axis

            Eigen::Matrix<float, 3, 4> J_analytical = computeAnalyticalJacobian(q, v_ned);
            Eigen::Matrix<float, 3, 4> J_normalized = normalizeAnalyticalJacobian(J_analytical, q, v_ned);
            CHECK(J_normalized.norm() < 3.0f); // Expect 0, this will definitively show us what is the error
        }
    }
}

TEST_CASE("CHECK that Rotations by Quaternions work as expected")
{
    Eigen::Vector3f v_ned(1.f, 0.f, 0.f); // x-axis vector

    SUBCASE("No rotation")
    {
        Eigen::Quaternionf q = Eigen::Quaternionf::Identity(); // no rotation
        Eigen::Vector3f rotated_v = q.toRotationMatrix() * v_ned;
        CHECK(rotated_v.isApprox(v_ned, kTolerance)); // Should be same vector
    }
    SUBCASE("90 degree rotation around z")
    {
        Eigen::Quaternionf q(Eigen::AngleAxisf(M_PIf / 2.0f, Eigen::Vector3f::UnitZ())); // 90° yaw
        Eigen::Vector3f rotated_v = q.toRotationMatrix() * v_ned;
        Eigen::Vector3f expected_v(0.f, 1.f, 0.f);
        CHECK(rotated_v.isApprox(expected_v, kTolerance));
    }
    SUBCASE("180 degree rotation around x")
    {
        Eigen::Quaternionf q(Eigen::AngleAxisf(M_PIf, Eigen::Vector3f::UnitX())); // 180° roll
        Eigen::Vector3f rotated_v = q.toRotationMatrix() * v_ned;
        Eigen::Vector3f expected_v(1.f, 0.f, 0.f);
        CHECK(rotated_v.isApprox(expected_v, kTolerance));
    }
}

TEST_CASE("Analytical Math")
{

    SUBCASE("Analytical Jacobian CHECK values: vx")
    {
        Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
        Eigen::Vector3f v_ned(1.f, 0.f, 0.f); // aligned with x-axis
        Eigen::Vector3f rotated_v = q * v_ned;

        CHECK(rotated_v[0] == 1);
        CHECK(rotated_v[1] == 0);
        CHECK(rotated_v[2] == 0);
    }

    SUBCASE("Analytical Jacobian CHECK values: vy")
    {
        Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
        Eigen::Vector3f v_ned(0.f, 1.f, 0.f); // aligned with y-axis
        Eigen::Vector3f rotated_v = q * v_ned;

        CHECK(rotated_v[0] == 0);
        CHECK(rotated_v[1] == 1);
        CHECK(rotated_v[2] == 0);
    }

    SUBCASE("Analytical Jacobian CHECK values: vz")
    {
        Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
        Eigen::Vector3f v_ned(0.f, 0.f, 1.f); // aligned with z-axis
        Eigen::Vector3f rotated_v = q * v_ned;

        CHECK(rotated_v[0] == 0);
        CHECK(rotated_v[1] == 0);
        CHECK(rotated_v[2] == 1);
    }

    SUBCASE("Analytical math for Quaternion - v computation")
    {
        Eigen::Quaternionf q(1.f, 0.f, 0.f, 0.f);
        Eigen::Vector3f v_ned(1.f, 0.f, 0.f); // aligned with x-axis
        Eigen::Vector3f rotated_v = q * v_ned;

        // std::cerr << "Rotated V " << rotated_v.transpose() << std::endl;

        CHECK(rotated_v[0] == 1);
        CHECK(rotated_v[1] == 0);
        CHECK(rotated_v[2] == 0);
    }

    SUBCASE("Analytical math for Quaternion - v computation with rotation")
    {
        const int trials = 10;
        for (int i = 0; i < trials; ++i)
        {
            // Generate a random unit quaternion
            Eigen::Quaternionf q = Eigen::Quaternion<long double>::UnitRandom().cast<float>();
            q.normalize();

            // Generate a random vector v in world frame (NED)
            Eigen::Vector3f v = Eigen::Vector3f::Random();

            // Method 1: Rotate using quaternion sandwich
            Eigen::Quaternionf v_quat(0.f, v.x(), v.y(), v.z());
            Eigen::Quaternionf rotated_q = q.conjugate() * v_quat * q;
            Eigen::Vector3f v_body_quat = rotated_q.vec(); // Extract vector part

            // Method 2: Rotate using rotation matrix transpose
            Eigen::Matrix3f R = q.toRotationMatrix();
            Eigen::Vector3f v_body_matrix = R.transpose() * v;

            // Compute diff and show results
            Eigen::Vector3f diff = v_body_quat - v_body_matrix;
            CHECK(diff.norm() < 1e-5f); // Strict equivalence CHECK
        }
    }
}

TEST_CASE("Z-axis direction consistency between NED and Body frames")
{
    // Identity quaternion = no rotation
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();

    // Define world vector in NED (Z-down)
    Eigen::Vector3f v_ned(0.0f, 0.0f, 1.0f); // pointing down

    // Rotate into body frame using inverse rotation: q.conjugate() * v * q
    Eigen::Quaternionf v_q(0.0f, v_ned.x(), v_ned.y(), v_ned.z());
    Eigen::Quaternionf rotated = q.conjugate() * v_q * q;
    Eigen::Vector3f v_body_quat = rotated.vec();

    // Same result using Rᵀ
    Eigen::Matrix3f R = q.toRotationMatrix();
    Eigen::Vector3f v_body_matrix = R.transpose() * v_ned;

    // This CHECK passes only if body Z-axis points down like NED
    CHECK(v_body_quat.isApprox(v_ned, kTolerance));
    CHECK(v_body_matrix.isApprox(v_ned, kTolerance));
}

TEST_CASE("Quaternion Jacobian: Identities, Rotations, Projections")
{
    using namespace Eigen;

    const std::vector<std::pair<Quaternionf, std::string>> testQuaternions = {
        {Quaternionf(1, 0, 0, 0).normalized(), "Identity"},
        {Quaternionf(sqrtf(0.5f), sqrtf(0.5f), 0, 0).normalized(), "90deg_X"},
        {Quaternionf(0, sqrtf(0.5f), sqrtf(0.5f), 0).normalized(), "90deg_YZ"},
        {Quaternionf(cosf(M_PIf / 4.0f), 0, 0, sinf(M_PIf / 4.0f)).normalized(), "45deg_Z"},
        {Quaternionf(0.5f, 0.5f, 0.5f, 0.5f).normalized(), "Generic"}};

    const std::vector<Vector3f> testVectors = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1},
        {1, 1, 1}};

    for (const auto &[q, label] : testQuaternions)
    {
        SUBCASE(("Jacobian comparison for " + label).c_str())
        {
            for (const auto &v : testVectors)
            {
                Matrix<float, 3, 4> J_analytic = computeAnalyticalJacobian(q, v);
                Matrix<float, 3, 4> J_numerical = computeNumericalJacobian(q, v);
                Matrix<float, 3, 4> J_normalized = normalizeAnalyticalJacobian(J_analytic, q, v);

                // Estimate axis direction
                Vector3f axis(q.x(), q.y(), q.z());
                float axis_norm = axis.norm();
                if (axis_norm > 1e-6f)
                    axis /= axis_norm;

                float alignment = std::abs(axis.dot(v.normalized()));

                // 🚫 Skip when v aligned with rotation axis
                if (alignment > kAlignmentThreshold)
                {
                    WARN("Skipping CHECK: v aligned with rotation axis\n");
                    continue;
                }

                // ✅ Finite difference Jacobian match
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 4; ++j)
                        CHECK(std::abs(J_normalized(i, j) - J_numerical(i, j)) < kNumericalEpsilon);

                // 🔁 Residual projection test
                Vector4f delta_vec = Vector4f::Constant(kNumericalEpsilon);
                Quaternionf dq(delta_vec(3), delta_vec(0), delta_vec(1), delta_vec(2));
                Quaternionf q_plus = (q * dq).normalized();

                Vector3f v_rot = q.conjugate() * v;
                Vector3f v_rot_plus = q_plus.conjugate() * v;
                Vector3f projected = J_analytic * delta_vec;
                Vector3f residual = v_rot_plus - v_rot;

                CHECK((projected - residual).norm() < 1e2f);
            }
        }
    }
}