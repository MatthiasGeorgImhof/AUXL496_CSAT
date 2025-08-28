#ifndef __QUATERION_HPP__
#define __QUATERION_HPP__ 

#include <Eigen/Dense>
#include <cmath>

Eigen::Matrix<float, 3, 4> computeNumericalJacobian(const Eigen::Quaternionf &q, const Eigen::Vector3f &v);

Eigen::Matrix<float, 3, 4> computeAnalyticalJacobian(const Eigen::Quaternionf &q, const Eigen::Vector3f &v);

Eigen::Matrix<float, 3, 4> normalizeAnalyticalJacobian(const Eigen::Matrix<float, 3, 4>& J_analytical, const Eigen::Quaternionf& q, const Eigen::Vector3f& v);

#endif // __QUATERION_HPP__