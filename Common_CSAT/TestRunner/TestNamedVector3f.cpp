#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "NamedVector3f.hpp"
#include <Eigen/Core>

constexpr float TOL = 1e-6f;

TEST_CASE("NamedVector3f: construction and accessors") {
    AngularRotation r1{1.0f, 2.0f, 3.0f};
    CHECK(r1.x() == doctest::Approx(1.0f));
    CHECK(r1.y() == doctest::Approx(2.0f));
    CHECK(r1.z() == doctest::Approx(3.0f));

    Eigen::Vector3f raw{4.0f, 5.0f, 6.0f};
    AngularRotation r2;
    r2 = raw;
    CHECK(r2.isApprox(raw));
}

TEST_CASE("NamedVector3f: Zero and normalized") {
    AngularVelocity v1 = AngularVelocity::Zero();
    CHECK(v1.isZero(TOL));

    AngularVelocity v2{3.0f, 0.0f, 0.0f};
    AngularVelocity v3 = v2.normalized();
    CHECK(v3.isApprox(Eigen::Vector3f::UnitX()));
}

TEST_CASE("NamedVector3f: arithmetic operations") {
    MagneticField B1{0.1f, 0.2f, 0.3f};
    MagneticField B2{0.05f, 0.05f, 0.05f};

    MagneticField B3 = B1 - B2;
    CHECK(B3.isApprox(Eigen::Vector3f(0.05f, 0.15f, 0.25f)));

    MagneticField B4 = B3 * 2.0f;
    CHECK(B4.isApprox(Eigen::Vector3f(0.1f, 0.3f, 0.5f)));

    MagneticField B5 = B4 / 2.0f;
    CHECK(B5.isApprox(B3));

    MagneticField B6 = -B5;
    CHECK(B6.isApprox(Eigen::Vector3f(-0.05f, -0.15f, -0.25f)));
}

TEST_CASE("NamedVector3f: dot and cross products") {
    AngularRotation r1{1.0f, 0.0f, 0.0f};
    AngularVelocity v1{0.0f, 1.0f, 0.0f};

    float dot = r1.dot(v1);
    CHECK(dot == doctest::Approx(0.0f));

    Eigen::Vector3f cross = r1.cross(v1);
    CHECK(cross.isApprox(Eigen::Vector3f(0.0f, 0.0f, 1.0f)));
}

TEST_CASE("NamedVector3f: diagnostics") {
    DipoleMoment m1{0.0f, 0.0f, 0.0f};
    DipoleMoment m2{1e-5f, 1e-5f, 1e-5f};

    CHECK(m1.isZero(TOL));
    CHECK(!m2.isZero(TOL));

    CHECK(m2.norm() > 0.0f);
    CHECK(m2.squaredNorm() > 0.0f);
}
