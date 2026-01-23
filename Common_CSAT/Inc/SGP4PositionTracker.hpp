#pragma once
#include <Eigen/Dense>
#include "Kalman.hpp"
#include "au.hpp"

#include "PositionService.hpp"
#include "TimeUtils.hpp"
#ifdef __arm__
#include "usbd_cdc_if.h"
#endif
#ifdef __x86_64__
#include "mock_hal.h"
#endif

class Sgp4PositionTracker
{
public:
    static constexpr int StateSize = 6;       // [px, py, pz, vx, vy, vz]
    static constexpr int MeasurementSize = 3; // GPS: [px, py, pz]

    using StateVector = Eigen::Matrix<float, StateSize, 1>;
    using Measurement = Eigen::Vector3f;

    Sgp4PositionTracker()
        : Q(Eigen::Matrix<float, StateSize, StateSize>::Identity() * 0.01f),
          R(Eigen::Matrix3f::Identity() * 0.1f),
          H(Eigen::Matrix<float, MeasurementSize, StateSize>::Zero()),
          kf(Q, R, Q, StateVector::Zero())
    {
        H.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
    }

    void setPrediction(const Eigen::Vector3f &pos, const Eigen::Vector3f &vel)
    {
        StateVector pred;
        pred << pos, vel;
        kf.stateVector = pred;
        kf.stateCovarianceMatrix = Q;
    }

    void updateWithGps(const Eigen::Vector3f &gps_measurement)
    {
        kf.update(H, gps_measurement);
    }

    StateVector getState() const
    {
        return kf.getState();
    }

private:
    Eigen::Matrix<float, StateSize, StateSize> Q;
    Eigen::Matrix3f R;
    Eigen::Matrix<float, MeasurementSize, StateSize> H;
    KalmanFilter<StateSize, MeasurementSize> kf;
};

template <typename Tracker, typename SGP4, typename GNSS>
class SGP4andGNSSandPosition
{
public:
    SGP4andGNSSandPosition() = delete;

    SGP4andGNSSandPosition(RTC_HandleTypeDef *hrtc, Tracker &tracker, SGP4 &sgp4, GNSS &gnss, uint16_t sgp4_rate = 1, uint16_t gnss_rate = 1) : hrtc_(hrtc), tracker_(tracker), sgp4_(sgp4), gnss_(gnss), sgp4_rate_(sgp4_rate), gnss_rate_(gnss_rate), sgp4_counter_(0), gnss_counter_(0U) {}

    bool predict(std::array<au::QuantityF<au::MetersInEcefFrame>, 3> &r, std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> &v, au::QuantityU64<au::Milli<au::Seconds>> &timestamp);
    PositionSolution predict();

private:
    RTC_HandleTypeDef *hrtc_;
    Tracker tracker_;
    SGP4 &sgp4_;
    GNSS &gnss_;

    uint16_t sgp4_rate_;
    uint16_t gnss_rate_;

    uint16_t sgp4_counter_;
    uint16_t gnss_counter_;
};

template <typename Tracker, typename SGP4, typename GNSS>
bool SGP4andGNSSandPosition<Tracker, SGP4, GNSS>::predict(std::array<au::QuantityF<au::MetersInEcefFrame>, 3> &r, std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> &v, au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
{
    TimeUtils::RTCDateTimeSubseconds rtc;
    HAL_RTC_GetTime(hrtc_, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc_, &rtc.date, RTC_FORMAT_BIN);
    timestamp = TimeUtils::from_rtc(rtc, hrtc_->Init.SynchPrediv);

    if (sgp4_counter_ % sgp4_rate_ == 0)
    {
        sgp4_.predict(r, v, timestamp);
        tracker_.setPrediction(Eigen::Vector3f(r[0].in(au::meters * au::ecefs), r[1].in(au::meters * au::ecefs), r[2].in(au::meters * au::ecefs)),
                               Eigen::Vector3f(v[0].in(au::meters * au::ecefs / au::seconds), v[1].in(au::meters * au::ecefs / au::seconds), v[2].in(au::meters * au::ecefs / au::seconds)));
    }

    if (gnss_counter_ % gnss_rate_ == 0)
    {
        auto optional_pos_ecef = gnss_.getNavPosECEF();
        if (optional_pos_ecef.has_value())
        {
            auto pos_ecef = ConvertPositionECEF(optional_pos_ecef.value());
            tracker_.updateWithGps(Eigen::Vector3f(pos_ecef.x.in(au::meters * au::ecefs), pos_ecef.y.in(au::meters * au::ecefs), pos_ecef.z.in(au::meters * au::ecefs)));
        }
    }

    auto state = tracker_.getState();
    std::transform(state.data(), state.data() + 3, r.begin(), [](const auto &item)
                   { return au::make_quantity<au::MetersInEcefFrame>(item); });
    std::transform(state.data() + 3, state.data() + 6, v.begin(), [](const auto &item)
                   { return au::make_quantity<au::MetersPerSecondInEcefFrame>(item); });

    ++sgp4_counter_;
    ++gnss_counter_;
    return true;
}

template <typename Tracker, typename SGP4, typename GNSS>
PositionSolution SGP4andGNSSandPosition<Tracker, SGP4, GNSS>::predict()
{
    au::QuantityU64<au::Milli<au::Seconds>> timestamp;
    std::array<au::QuantityF<au::MetersInEcefFrame>, 3> r;
    std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> v;

    predict(r, v, timestamp);

    return PositionSolution{
        .timestamp = timestamp,
        .position = r,
        .velocity = v,
        .acceleration = {},
        .validity_flags = static_cast<uint8_t>(PositionSolution::Validity::POSITION) | static_cast<uint8_t>(PositionSolution::Validity::VELOCITY)};
}

template <typename SGP4>
class SGP4Position
{
public:
    SGP4Position() = delete;

    SGP4Position(RTC_HandleTypeDef *hrtc, SGP4 &sgp4) : hrtc_(hrtc), sgp4_(sgp4) {}

    bool predict(std::array<au::QuantityF<au::MetersInEcefFrame>, 3> &r, std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> &v, au::QuantityU64<au::Milli<au::Seconds>> &timestamp);
    PositionSolution predict();

private:
    RTC_HandleTypeDef *hrtc_;
    SGP4 &sgp4_;
};

template <typename SGP4>
bool SGP4Position<SGP4>::predict(std::array<au::QuantityF<au::MetersInEcefFrame>, 3> &r, std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> &v, au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
{
    TimeUtils::RTCDateTimeSubseconds rtc;
    HAL_RTC_GetTime(hrtc_, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc_, &rtc.date, RTC_FORMAT_BIN);
    timestamp = TimeUtils::from_rtc(rtc, hrtc_->Init.SynchPrediv);

    sgp4_.predict(r, v, timestamp);
    return true;
}

template <typename SGP4>
PositionSolution SGP4Position<SGP4>::predict()
{
    au::QuantityU64<au::Milli<au::Seconds>> timestamp;
    std::array<au::QuantityF<au::MetersInEcefFrame>, 3> r;
    std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> v;

    predict(r, v, timestamp);

    return PositionSolution{
        .timestamp = timestamp,
        .position = r,
        .velocity = v,
        .acceleration = {},
        .validity_flags = static_cast<uint8_t>(PositionSolution::Validity::POSITION) | static_cast<uint8_t>(PositionSolution::Validity::VELOCITY)};
}
