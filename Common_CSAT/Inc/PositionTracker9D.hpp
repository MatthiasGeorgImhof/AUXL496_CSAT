#pragma once
#include <Eigen/Dense>
#include "Kalman.hpp"
#include "coordinate_transformations.hpp"
#include "au.hpp"
#include "PositionService.hpp"

#include "TimeUtils.hpp"
#include "GNSS.hpp"
#include "IMU.hpp"
#include "IMUExtension.hpp"
#ifdef __arm__
#include "usbd_cdc_if.h"
#endif
#ifdef __x86_64__
#include "mock_hal.h"
#endif

Eigen::Matrix3f computeNEDtoECEFRotation(au::QuantityF<au::RadiansInGeodeticFrame> latitude, au::QuantityF<au::RadiansInGeodeticFrame> longitude)
{
    float lat = latitude.in(au::radiansInGeodeticFrame);
    float lon = longitude.in(au::radiansInGeodeticFrame);

    float sin_lat = std::sin(lat);
    float cos_lat = std::cos(lat);
    float sin_lon = std::sin(lon);
    float cos_lon = std::cos(lon);

    // NED to ECEF rotation matrix
    Eigen::Matrix3f R;
    R << -sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon,
        -sin_lat * sin_lon, cos_lon, -cos_lat * sin_lon,
        cos_lat, 0.f, -sin_lat;

    return R;
}

Eigen::Matrix3f computeNEDtoECEFRotation(std::array<au::QuantityF<au::MetersInEcefFrame>, 3> &ecef)
{
    // Convert ECEF to geodetic (lat, lon)
    coordinate_transformations::ECEF ecef_ = {ecef[0], ecef[1], ecef[2]};
    coordinate_transformations::Geodetic geo = coordinate_transformations::ecefToGeodetic(ecef_);

    return computeNEDtoECEFRotation(geo.latitude, geo.longitude);
}

class PositionTracker9D
{
public:
    static constexpr int StateSize = 9;
    static constexpr int PosMeasSize = 3;
    static constexpr int VelMeasSize = 3;
    static constexpr int AccMeasSize = 3;

    static_assert(StateSize == PosMeasSize + VelMeasSize + AccMeasSize, "State size must match the sum of position, velocity, and acceleration measurement sizes.");

    using StateVector = Eigen::Matrix<float, StateSize, 1>;

    PositionTracker9D()
        : last_timestamp(au::make_quantity<au::Milli<au::Seconds>>(0)),
          A(Eigen::Matrix<float, StateSize, StateSize>::Identity()),
          H_gps(Eigen::Matrix<float, PosMeasSize, StateSize>::Zero()),
          H_acc(Eigen::Matrix<float, AccMeasSize, StateSize>::Zero()),
          Q(Eigen::Matrix<float, StateSize, StateSize>::Identity() * 1e-4f),
          R_gps(Eigen::Matrix3f::Identity() * 5e-3f),
          R_accel(Eigen::Matrix3f::Identity() * 1e-2f),
          kf(Q, R_gps, Q, StateVector::Zero())

    {
        H_gps.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
        H_acc.block<3, 3>(0, 6) = Eigen::Matrix3f::Identity();
    }

    void updateWithAccel(const Eigen::Vector3f &accel, au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        maybePredict(timestamp);

        // KalmanFilter<StateSize, AccMeasSize> accelKF(Q, R_accel, kf.stateCovarianceMatrix, kf.stateVector);
        // accelKF.update(H_acc, accel);
        // kf.stateVector = accelKF.stateVector;
        // kf.stateCovarianceMatrix = accelKF.stateCovarianceMatrix;

        kf.update(H_acc, accel);

        // std::cerr << "timestamp_sec: " << timestamp_sec << ", A:\n" << A << "\n";
        // std::cerr << "State end of updateWithAccel:\n" << kf.getState().transpose() << "\n";
    }

    void updateWithGps(const Eigen::Vector3f &gps, au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        maybePredict(timestamp);
        kf.update(H_gps, gps);
        // std::cerr << "timestamp_sec: " << timestamp_sec << ", A:\n" << A << "\n";
        // std::cerr << "State end of updateWithGps:\n" << kf.getState().transpose() << "\n";
    }

    StateVector getState() const
    {
        return kf.getState();
    }

    void setState(const StateVector &newState)
    {
        kf.setState(newState);
    }

    void injectGpsWithoutPrediction(const Eigen::Vector3f &gps)
    {
        kf.update(H_gps, gps);
    }

protected:
    void maybePredict(au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        // std::cerr << "maybePredict: last_timestamp_sec_: " << last_timestamp_sec_ << " timestamp_sec: " << timestamp_sec << "\n";
        // std::cerr << "A\n: " << A << "\n";
        // std::cerr << "State end of maybePredict:\n" << kf.getState().transpose() << "\n";

        float dt = 0.001f * static_cast<float>((timestamp - last_timestamp).in(au::milli(au::seconds)));
        if (dt <= 0.f)
            return;

        updateTransitionMatrix(dt);
        kf.predict(A);
        last_timestamp = timestamp;
    }

    void updateTransitionMatrix(float dt)
    {
        // std::cerr << "Updating updateTransitionMatrix A with dt = " << dt << "\n";
        A.setIdentity();
        for (int i = 0; i < 3; ++i)
        {
            A(i, i + 3) = dt;
            A(i, i + 6) = 0.5f * dt * dt;
            A(i + 3, i + 6) = dt;
        }
        // std::cerr << "dt: " << dt << ", A:\n" << A << "\n";
        // std::cerr << "State end of updateTransitionMatrix:\n" << kf.getState().transpose() << "\n";
    }

protected:
    au::QuantityU64<au::Milli<au::Seconds>> last_timestamp;
    Eigen::Matrix<float, StateSize, StateSize> A;
    Eigen::Matrix<float, PosMeasSize, StateSize> H_gps;
    Eigen::Matrix<float, AccMeasSize, StateSize> H_acc;
    Eigen::Matrix<float, StateSize, StateSize> Q;
    Eigen::Matrix3f R_gps, R_accel;
    KalmanFilter<StateSize, PosMeasSize> kf;
};

template <typename PositionTracker, typename GNSS, typename IMU, typename OrientationProvider, typename GravityPolicy = NoGravityCompensation>
    requires(HasBodyAccelerometer<IMU>)
class GNSSandAccelPosition
{
public:
    GNSSandAccelPosition() = delete;

    GNSSandAccelPosition(RTC_HandleTypeDef *hrtc, PositionTracker &tracker, GNSS &gnss, IMU &imu, OrientationProvider &orientation, uint16_t gnss_rate = 1, uint16_t imu_rate = 1) : hrtc_(hrtc), tracker_(tracker), gnss_(gnss), orientation_(orientation), ecef_imu_(imu, orientation, tracker), gnss_rate_(gnss_rate), imu_rate_(imu_rate), gnss_counter_(0U), imu_counter_(0)
    {
    }

    bool predict(std::array<au::QuantityF<au::MetersInEcefFrame>, 3> &r, std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> &v, au::QuantityU64<au::Milli<au::Seconds>> &timestamp);
    PositionSolution predict();

private:
    RTC_HandleTypeDef *hrtc_;
    PositionTracker &tracker_;
    GNSS &gnss_;
    OrientationProvider &orientation_;
    IMUAccInECEFWithPolicy<IMU, OrientationProvider, PositionTracker, GravityPolicy> ecef_imu_;

    uint16_t gnss_rate_;
    uint16_t imu_rate_;

    uint16_t gnss_counter_;
    uint16_t imu_counter_;
};

template <typename PositionTracker, typename GNSS, typename IMU, typename OrientationProvider, typename GravityPolicy>
    requires(HasBodyAccelerometer<IMU>)
PositionSolution GNSSandAccelPosition<PositionTracker, GNSS, IMU, OrientationProvider, GravityPolicy>::predict()
{
    TimeUtils::RTCDateTimeSubseconds rtc;
    HAL_RTC_GetTime(hrtc_, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc_, &rtc.date, RTC_FORMAT_BIN);
    au::QuantityU64<au::Milli<au::Seconds>> timestamp{TimeUtils::from_rtc(rtc, hrtc_->Init.SynchPrediv)};

    if (gnss_counter_ % gnss_rate_ == 0)
    {
        auto optional_pos_ecef = gnss_.getNavPosECEF();
        if (optional_pos_ecef.has_value())
        {
            auto pos_ecef = ConvertPositionECEF(optional_pos_ecef.value());
            // std::cerr << "[GNSS injection] "
            //           << pos_ecef.x.in(au::metersInEcefFrame) << ", "
            //           << pos_ecef.y.in(au::metersInEcefFrame) << ", "
            //           << pos_ecef.z.in(au::metersInEcefFrame) << "\n";

            tracker_.updateWithGps(Eigen::Vector3f(pos_ecef.x.in(au::metersInEcefFrame), pos_ecef.y.in(au::metersInEcefFrame), pos_ecef.z.in(au::metersInEcefFrame)), timestamp);
        }
    }
    // std::cerr << "[GNSS] gnss_counter = " << gnss_counter_ << ", gnss_rate = " << gnss_rate_ << std::endl;
    // std::cerr << "[IMU] timestamp_sec = " << timestamp_sec.in(au::seconds) << std::endl;

    if (imu_counter_ % imu_rate_ == 0)
    {
        auto optional_accel = ecef_imu_.readAccelerometer();
        if (optional_accel.has_value())
        {
            auto accel = optional_accel.value();

            tracker_.updateWithAccel(Eigen::Vector3f(accel[0].in(au::metersPerSecondSquaredInEcefFrame), accel[1].in(au::metersPerSecondSquaredInEcefFrame), accel[2].in(au::metersPerSecondSquaredInEcefFrame)), timestamp);
            // std::cerr << "[IMU] accel.z = " << accel[2].in(au::metersPerSecondSquaredInEcefFrame) << std::endl;
        }
    }
    // std::cerr << "[IMU] imu_counter = " << imu_counter_ << ", imu_rate = " << imu_rate_ << std::endl;
    // std::cerr << "[IMU] timestamp_sec = " << timestamp_sec.in(au::seconds) << std::endl;

    auto state = tracker_.getState();
    // std::cerr << "[PositionTracker] pos.z = " << state[2] << ", vel.z = " << state[5] << std::endl;

    ++gnss_counter_;
    ++imu_counter_;

    return PositionSolution{
        timestamp,
        {
            au::make_quantity<au::MetersInEcefFrame>(state.data()[0]),
            au::make_quantity<au::MetersInEcefFrame>(state.data()[1]),
            au::make_quantity<au::MetersInEcefFrame>(state.data()[2])
        },
        {
            au::make_quantity<au::MetersPerSecondInEcefFrame>(state.data()[3]),
            au::make_quantity<au::MetersPerSecondInEcefFrame>(state.data()[4]),
            au::make_quantity<au::MetersPerSecondInEcefFrame>(state.data()[5])
        },
        {
            au::make_quantity<au::MetersPerSecondSquaredInEcefFrame>(state.data()[6]),
            au::make_quantity<au::MetersPerSecondSquaredInEcefFrame>(state.data()[7]),
            au::make_quantity<au::MetersPerSecondSquaredInEcefFrame>(state.data()[8])
        },
        static_cast<uint8_t>(PositionSolution::Validity::ACCELERATION) |
        static_cast<uint8_t>(PositionSolution::Validity::POSITION) |
        static_cast<uint8_t>(PositionSolution::Validity::VELOCITY)
    };
}

template <typename PositionTracker, typename GNSS, typename IMU, typename OrientationProvider, typename GravityPolicy>
    requires(HasBodyAccelerometer<IMU>)
bool GNSSandAccelPosition<PositionTracker, GNSS, IMU, OrientationProvider, GravityPolicy>::predict(std::array<au::QuantityF<au::MetersInEcefFrame>, 3> &r, std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> &v, au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
{
    PositionSolution solution = predict();
    timestamp = solution.timestamp;
    r = solution.position;
    v = solution.velocity;

    return true;
}