// MagnetorquerDriver.hpp

#pragma once
#include "LVLHAttitudeTarget.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <algorithm>
#include <cmath>

struct PWMCommand {
    float duty_x; // Range: [-1.0, 1.0]
    float duty_y;
    float duty_z;
};

class MagnetorquerDriver {
public:
    struct Config {
        float max_dipole_x; // A·m²
        float max_dipole_y;
        float max_dipole_z;
    };

    explicit MagnetorquerDriver(const Config& config) : cfg(config) {}

    PWMCommand computePWM(const Eigen::Vector3f& m_cmd_body) const {
        return {
            clampAndLog(m_cmd_body.x(), cfg.max_dipole_x, "X"),
            clampAndLog(m_cmd_body.y(), cfg.max_dipole_y, "Y"),
            clampAndLog(m_cmd_body.z(), cfg.max_dipole_z, "Z")
        };
    }

private:
    Config cfg;

    float clampAndLog(float m_cmd, float m_max, const char* /*axis*/) const {
        if (std::abs(m_cmd) > m_max) {
            // Optional: log saturation event
            // std::cout << "[MagnetorquerDriver] Saturation on axis " << axis << ": " << m_cmd << " exceeds " << m_max << std::endl;
        }
        return std::clamp(m_cmd / m_max, -1.0f, 1.0f);
    }
};


class MagnetorquerControlPipeline {
public:
    struct Config {
        AttitudeController controller;
        MagnetorquerDriver driver;
    };

    explicit MagnetorquerControlPipeline(const Config& cfg) : config(cfg) {}

    PWMCommand computePWMCommand(
        const Eigen::Quaternionf& q_current,
        const AngularVelocity& omega_measured,
        const Eigen::Quaternionf& q_desired,
        const MagneticField& B_body) const
    {
        Eigen::Quaternionf q_error = AttitudeError::computeQuaternionError(q_desired, q_current);
        AngularRotation rot_vec = AttitudeError::rotationVector(q_error);
        AngularRotation omega_cmd = config.controller.computeOmegaCommand(rot_vec, omega_measured);
        DipoleMoment m_cmd_body = MagnetorquerController::computeDipoleMoment(omega_cmd, B_body);
        
        // std::cout << "explicit MagnetorquerControlPipeline::computePWMCommand " << "\n";
        // std::cout << "rot_vec: " << rot_vec.transpose() << "\n";
        // std::cout << "omega_cmd: " << omega_cmd.transpose() << "\n";
        // std::cout << "m_cmd: " << m_cmd_body.transpose() << "\n";

        return config.driver.computePWM(m_cmd_body);
    }

private:
    Config config;
};
