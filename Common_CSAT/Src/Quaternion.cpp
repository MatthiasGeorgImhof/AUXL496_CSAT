#include "Quaternion.hpp"

Eigen::Matrix<float, 3, 4> computeNumericalJacobian(const Eigen::Quaternionf &q, const Eigen::Vector3f &v)
{
    const float eps = 1e-4f;
    Eigen::Matrix<float, 3, 4> J = Eigen::Matrix<float, 3, 4>::Zero();

    for (int i = 0; i < 4; ++i)
    {
        Eigen::Quaternionf q_plus = q;
        Eigen::Quaternionf q_minus = q;
        q_plus.normalize();
        q_minus.normalize();


        if (i == 0)
        {
            q_plus.x() += eps;
            q_minus.x() -= eps;
        }
        else if (i == 1)
        {
            q_plus.y() += eps;
            q_minus.y() -= eps;
        }
        else if (i == 2)
        {
            q_plus.z() += eps;
            q_minus.z() -= eps;
        }
        else
        {
            q_plus.w() += eps;
            q_minus.w() -= eps;
        }

        // float eps_plus = eps / q_plus.norm();
        // float eps_minus = eps / q_minus.norm();

        // // std::cerr << "Norms " << eps << ": "<< eps_plus << " " << eps_minus << std::endl;

        q_plus.normalize();
        q_minus.normalize();

        Eigen::Matrix3f R_plus = q_plus.conjugate().toRotationMatrix();
        Eigen::Matrix3f R_minus = q_minus.conjugate().toRotationMatrix();

        J.col(i) = (R_plus - R_minus) * v / (2.0f * eps);
    }

    // std::cerr << "Jacobian computed via numerical perturbation:\n" << J << std::endl;
    return J;
}

Eigen::Matrix<float, 3, 4> computeAnalyticalJacobian(const Eigen::Quaternionf &q, const Eigen::Vector3f &v)
{
    const float x = q.x(), y = q.y(), z = q.z(), w = q.w();
    const float vx = v.x(), vy = v.y(), vz = v.z();

    Eigen::Matrix<float, 3, 4> J;

    // d(R^T * v) / dx
    J(0, 0) =  2.0f * x * vx + 2.0f * y * vy + 2.0f * z * vz;
    J(1, 0) =  2.0f * y * vx - 2.0f * x * vy + 2.0f * w * vz;
    J(2, 0) =  2.0f * z * vx - 2.0f * w * vy - 2.0f * x * vz;

    // d(...) / dy
    J(0, 1) = -2.0f * y * vx + 2.0f * x * vy - 2.0f * w * vz;
    J(1, 1) =  2.0f * x * vx + 2.0f * y * vy + 2.0f * z * vz;
    J(2, 1) =  2.0f * w * vx + 2.0f * z * vy - 2.0f * y * vz;

    // d(...) / dz
    J(0, 2) = -2.0f * z * vx + 2.0f * w * vy + 2.0f * x * vz;
    J(1, 2) = -2.0f * w * vx - 2.0f * z * vy + 2.0f * y * vz;
    J(2, 2) =  2.0f * x * vx + 2.0f * y * vy + 2.0f * z * vz;

    // // d(...) / dw
    J(0, 3) =  2.0f * w * vx + 2.0f * z * vy - 2.0f * y * vz;
    J(1, 3) = -2.0f * z * vx + 2.0f * w * vy + 2.0f * x * vz;
    J(2, 3) =  2.0f * y * vx - 2.0f * x * vy + 2.0f * w * vz;

    // std::cerr << "Jacobian computed via analytical formula:\n" << J << std::endl;
    return J;
}

Eigen::Matrix<float, 3, 4> normalizeAnalyticalJacobian(
    const Eigen::Matrix<float, 3, 4>& J_analytical,
    const Eigen::Quaternionf& q,
    const Eigen::Vector3f& v)
{
    Eigen::Matrix<float, 3, 4> J_normalized;
    const float w = q.w(), x = q.x(), y = q.y(), z = q.z();
    const float N = x * x + y * y + z * z + w * w;

    // Rotated vector: R(q*) * v
    Eigen::Vector3f v_rot = q.conjugate().toRotationMatrix() * v;

    for (int i = 0; i < 4; ++i)
    {
        float qi = (i == 0) ? x :
                   (i == 1) ? y :
                   (i == 2) ? z :
                              w;

        J_normalized.col(i) = J_analytical.col(i) / N - (2.0f * qi / (N * N)) * v_rot;
    }

    // std::cerr << "Jacobian normalized via analytical formula:\n" << J_normalized << std::endl;
    return J_normalized;
}
