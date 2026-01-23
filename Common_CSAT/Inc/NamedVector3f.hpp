// NamedVector3f.hpp

#pragma once
#include <Eigen/Dense>

template <typename Tag>
struct NamedVector3f
{
    Eigen::Vector3f value;

    NamedVector3f() = default;
    explicit NamedVector3f(const Eigen::Vector3f &v) : value(v) {}
    explicit NamedVector3f(float x, float y, float z) : value(x, y, z) {}

    NamedVector3f &operator=(const Eigen::Vector3f &v)
    {
        value = v;
        return *this;
    }

    // Implicit conversion for convenience
    operator const Eigen::Vector3f &() const { return value; }
    operator Eigen::Vector3f &() { return value; }

    float x() const { return value.x(); }
    float y() const { return value.y(); }
    float z() const { return value.z(); }

    // Optional: add semantic methods
    static NamedVector3f Zero()
    {
        return NamedVector3f(Eigen::Vector3f::Zero().eval());
    }

    float norm() const { return value.norm(); }

    template <typename OtherTag>
    Eigen::Vector3f cross(const NamedVector3f<OtherTag> &other) const
    {
        return value.cross(other.value);
    }

    template <typename OtherTag>
    float dot(const NamedVector3f<OtherTag> &other) const { return value.dot(other.value); }

    float squaredNorm() const { return value.squaredNorm(); }
    NamedVector3f normalized() const { return NamedVector3f(value.normalized()); }

    bool isZero(float tol = 1e-6f) const { return value.isZero(tol); }

    bool isApprox(const Eigen::Vector3f &other, float tol = 1e-6f) const
    {
        return value.isApprox(other, tol);
    }

    template <typename OtherTag>
    bool isApprox(const NamedVector3f<OtherTag> &other, float tol = 1e-6f) const
    {
        return value.isApprox(other.value, tol);
    }

    friend NamedVector3f operator*(float scalar, const NamedVector3f &v)
    {
        return NamedVector3f(scalar * v.value);
    }

    friend NamedVector3f operator*(const NamedVector3f &v, float scalar)
    {
        return NamedVector3f(v.value * scalar);
    }

    friend NamedVector3f operator-(const NamedVector3f &v)
    {
        return NamedVector3f(-v.value);
    }

    friend NamedVector3f operator-(const NamedVector3f &a, const NamedVector3f &b)
    {
        return NamedVector3f(a.value - b.value);
    }

    friend NamedVector3f operator/(const NamedVector3f &v, float scalar)
    {
        return NamedVector3f(v.value / scalar);
    }
};

struct AngularRotationTag
{
};
using AngularRotation = NamedVector3f<AngularRotationTag>;

struct AngularVelocityTag
{
};
using AngularVelocity = NamedVector3f<AngularVelocityTag>;

struct DipoleMomentTag
{
};
using DipoleMoment = NamedVector3f<DipoleMomentTag>;

struct MagneticFieldTag
{
};
using MagneticField = NamedVector3f<MagneticFieldTag>;

// struct PositionTag
// {
// };
// using Position = NamedVector3f<PositionTag>;

// struct VelocityTag
// {
// };
// using Velocity = NamedVector3f<VelocityTag>;

// struct AccelerationTag
// {
// };
// using Acceleration = NamedVector3f<AccelerationTag>;

