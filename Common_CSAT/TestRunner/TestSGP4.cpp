#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "TaskSGP4.hpp"
#include "coordinate_transformations.hpp"

#include "TimeUtils.hpp"
#include <chrono>
#include <cstdint>
#include <cmath>
#include <limits>

#include "au.hpp"
#include "mock_hal.h"

TEST_CASE("Check predict_teme 2025 6 25 18 0 0")
{
    RTC_HandleTypeDef hrtc;
    hrtc.Init.SynchPrediv = 1023;
    set_current_tick(1001);

    TimeUtils::DateTimeComponents epoch{
        .year = 2000,
        .month = 1,
        .day = 1,
        .hour = 12,
        .minute = 0,
        .second = 0,
        .millisecond = 0};
        
    TimeUtils::DateTimeComponents now{
        .year = 2025,
        .month = 6,
        .day = 25,
        .hour = 18,
        .minute = 0,
        .second = 0,
        .millisecond = 0};
    
    SGP4 sgp4(&hrtc);

    // ISS (ZARYA)
    char longstr1[] = "1 25544U 98067A   25176.73245655  .00008102  00000-0  14854-3 0  9994";
    char longstr2[] = "2 25544  51.6390 264.7180 0001990 278.3788 217.2311 15.50240116516482";  

    auto parsed = sgp4_utils::parseTLE(longstr1, longstr2);
    REQUIRE(parsed.has_value());
    SGP4TwoLineElement data = parsed.value();
    sgp4.setSGP4TLE(data);
    
    TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(now, hrtc.Init.SynchPrediv);
    HAL_RTC_SetTime(&hrtc, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &rtc.date, RTC_FORMAT_BIN);
    HAL_RTCEx_SetSynchroShift(&hrtc, RTC_SHIFTADD1S_RESET, rtc.time.SubSeconds);
            
    std::array<au::QuantityF<au::Kilo<au::MetersInTemeFrame>>, 3> r_now;
    std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInTemeFrame>>, 3> v_now;
    au::QuantityU64<au::Milli<au::Seconds>> timestamp_now;

    sgp4.predict_teme(r_now, v_now, timestamp_now);

    std::array<float,3> expected_r {-3006.1573609732827f, 4331.221049310724f, -4290.439626312989f};
    std::array<float,3> expected_v {-3.3808196282756926f, -5.872899089174856f, -3.5610122777771087f};
    CHECK(r_now[0].in(au::kilo(au::metersInTemeFrame)) == doctest::Approx(expected_r[0]).epsilon(0.01));
    CHECK(r_now[1].in(au::kilo(au::metersInTemeFrame)) == doctest::Approx(expected_r[1]).epsilon(0.01));
    CHECK(r_now[2].in(au::kilo(au::metersInTemeFrame)) == doctest::Approx(expected_r[2]).epsilon(0.01));
    CHECK(v_now[0].in(au::kilo(au::metersPerSecondInTemeFrame)) == doctest::Approx(expected_v[0]).epsilon(0.01));
    CHECK(v_now[1].in(au::kilo(au::metersPerSecondInTemeFrame)) == doctest::Approx(expected_v[1]).epsilon(0.01));
    CHECK(v_now[2].in(au::kilo(au::metersPerSecondInTemeFrame)) == doctest::Approx(expected_v[2]).epsilon(0.01));

    float jd_now = TimeUtils::to_fractional_days(TimeUtils::to_timepoint(epoch), TimeUtils::to_timepoint(now));
    auto ecef_pos_now = coordinate_transformations::temeToecef(r_now, jd_now);
    auto ecef_vel_now = coordinate_transformations::temeToecef(v_now, jd_now);

    CHECK(ecef_pos_now[0].in(au::kilo(au::metersInEcefFrame)) == doctest::Approx(2715.4f).epsilon(0.01));
    CHECK(ecef_pos_now[1].in(au::kilo(au::metersInEcefFrame)) == doctest::Approx(-4518.34).epsilon(0.01));
    CHECK(ecef_pos_now[2].in(au::kilo(au::metersInEcefFrame)) == doctest::Approx(-4291.31).epsilon(0.01));
    CHECK(ecef_vel_now[0].in(au::kilo(au::metersPerSecondInEcefFrame)) == doctest::Approx(3.75928f).epsilon(0.01));
    CHECK(ecef_vel_now[1].in(au::kilo(au::metersPerSecondInEcefFrame)) == doctest::Approx(5.63901f).epsilon(0.01));
    CHECK(ecef_vel_now[2].in(au::kilo(au::metersPerSecondInEcefFrame)) == doctest::Approx(-3.55967f).epsilon(0.01));
}

TEST_CASE("Check predict in ecef 2025 6 25 18 0 0")
{
    RTC_HandleTypeDef hrtc;
    hrtc.Init.SynchPrediv = 1023;
    set_current_tick(1001);

    TimeUtils::DateTimeComponents now{
        .year = 2025,
        .month = 6,
        .day = 25,
        .hour = 18,
        .minute = 0,
        .second = 0,
        .millisecond = 0};
    
    SGP4 sgp4(&hrtc);

    // ISS (ZARYA)
    char longstr1[] = "1 25544U 98067A   25176.73245655  .00008102  00000-0  14854-3 0  9994";
    char longstr2[] = "2 25544  51.6390 264.7180 0001990 278.3788 217.2311 15.50240116516482";  

    auto parsed = sgp4_utils::parseTLE(longstr1, longstr2);
    REQUIRE(parsed.has_value());
    SGP4TwoLineElement data = parsed.value();
    sgp4.setSGP4TLE(data);
    
    TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(now, hrtc.Init.SynchPrediv);
    HAL_RTC_SetTime(&hrtc, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &rtc.date, RTC_FORMAT_BIN);
    HAL_RTCEx_SetSynchroShift(&hrtc, RTC_SHIFTADD1S_RESET, rtc.time.SubSeconds);
            
    std::array<au::QuantityF<au::MetersInEcefFrame>, 3> r_now;
    std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> v_now;
    au::QuantityU64<au::Milli<au::Seconds>> timestamp_now;

    sgp4.predict(r_now, v_now, timestamp_now);

    CHECK(r_now[0].in(au::kilo(au::meters * au::ecefs)) == doctest::Approx(2715.4f).epsilon(0.01));
    CHECK(r_now[1].in(au::kilo(au::meters * au::ecefs)) == doctest::Approx(-4518.34f).epsilon(0.01));
    CHECK(r_now[2].in(au::kilo(au::meters * au::ecefs)) == doctest::Approx(-4291.31f).epsilon(0.01));
    CHECK(v_now[0].in(au::kilo(au::meters * au::ecefs / au::seconds)) == doctest::Approx(3.75928f).epsilon(0.01));
    CHECK(v_now[1].in(au::kilo(au::meters * au::ecefs / au::seconds)) == doctest::Approx(5.63901f).epsilon(0.01));
    CHECK(v_now[2].in(au::kilo(au::meters * au::ecefs / au::seconds)) == doctest::Approx(-3.55967f).epsilon(0.01));
}

TEST_CASE("Check predict_teme 2025 7 6 20 43 13")
{
    RTC_HandleTypeDef hrtc;
    hrtc.Init.SynchPrediv = 1023;
    set_current_tick(1001);

    TimeUtils::DateTimeComponents epoch{
        .year = 2000,
        .month = 1,
        .day = 1,
        .hour = 12,
        .minute = 0,
        .second = 0,
        .millisecond = 0};
        
    TimeUtils::DateTimeComponents now{
        .year = 2025,
        .month = 7,
        .day = 6,
        .hour = 20,
        .minute = 43,
        .second = 13,
        .millisecond = 0};
    
    SGP4 sgp4(&hrtc);

    // ISS (ZARYA)
    char longstr1[] = "1 25544U 98067A   25176.73245655  .00008102  00000-0  14854-3 0  9994";
    char longstr2[] = "2 25544  51.6390 264.7180 0001990 278.3788 217.2311 15.50240116516482";  

    auto parsed = sgp4_utils::parseTLE(longstr1, longstr2);
    REQUIRE(parsed.has_value());
    SGP4TwoLineElement data = parsed.value();
    sgp4.setSGP4TLE(data);
    
    TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(now, hrtc.Init.SynchPrediv);
    HAL_RTC_SetTime(&hrtc, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &rtc.date, RTC_FORMAT_BIN);
    HAL_RTCEx_SetSynchroShift(&hrtc, RTC_SHIFTADD1S_RESET, rtc.time.SubSeconds);
            
    std::array<au::QuantityF<au::Kilo<au::MetersInTemeFrame>>, 3> r_now;
    std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInTemeFrame>>, 3> v_now;
    au::QuantityU64<au::Milli<au::Seconds>> timestamp_now;

    sgp4.predict_teme(r_now, v_now, timestamp_now);

    float jd_now = TimeUtils::to_fractional_days(TimeUtils::to_timepoint(epoch), TimeUtils::to_timepoint(now));
    auto ecef_pos_now = coordinate_transformations::temeToecef(r_now, jd_now);
    auto ecef_vel_now = coordinate_transformations::temeToecef(v_now, jd_now);

    CHECK(ecef_pos_now[0].in(au::kilo(au::metersInEcefFrame)) == doctest::Approx(6356.42f).epsilon(0.01));
    CHECK(ecef_pos_now[1].in(au::kilo(au::metersInEcefFrame)) == doctest::Approx(-1504.07f).epsilon(0.01));
    CHECK(ecef_pos_now[2].in(au::kilo(au::metersInEcefFrame)) == doctest::Approx(1859.27f).epsilon(0.01));
    CHECK(ecef_vel_now[0].in(au::kilo(au::metersPerSecondInEcefFrame)) == doctest::Approx(-0.42784f).epsilon(0.01));
    CHECK(ecef_vel_now[1].in(au::kilo(au::metersPerSecondInEcefFrame)) == doctest::Approx(5.18216f).epsilon(0.01));
    CHECK(ecef_vel_now[2].in(au::kilo(au::metersPerSecondInEcefFrame)) == doctest::Approx(5.63173f).epsilon(0.01));
}

TEST_CASE("Check predict in ecef 2025 7 6 20 43 13")
{
    RTC_HandleTypeDef hrtc;
    hrtc.Init.SynchPrediv = 1023;
    set_current_tick(1001);

    TimeUtils::DateTimeComponents now{
        .year = 2025,
        .month = 7,
        .day = 6,
        .hour = 20,
        .minute = 43,
        .second = 13,
        .millisecond = 0};
    
    SGP4 sgp4(&hrtc);

    // ISS (ZARYA)
    char longstr1[] = "1 25544U 98067A   25176.73245655  .00008102  00000-0  14854-3 0  9994";
    char longstr2[] = "2 25544  51.6390 264.7180 0001990 278.3788 217.2311 15.50240116516482";  

    auto parsed = sgp4_utils::parseTLE(longstr1, longstr2);
    REQUIRE(parsed.has_value());
    SGP4TwoLineElement data = parsed.value();
    sgp4.setSGP4TLE(data);
    
    TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(now, hrtc.Init.SynchPrediv);
    HAL_RTC_SetTime(&hrtc, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &rtc.date, RTC_FORMAT_BIN);
    HAL_RTCEx_SetSynchroShift(&hrtc, RTC_SHIFTADD1S_RESET, rtc.time.SubSeconds);
            
    std::array<au::QuantityF<au::MetersInEcefFrame>, 3> r_now;
    std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> v_now;
    au::QuantityU64<au::Milli<au::Seconds>> timestamp_now;

    sgp4.predict(r_now, v_now, timestamp_now);

    CHECK(r_now[0].in(au::kilo(au::meters * au::ecefs)) == doctest::Approx(6356.42f).epsilon(0.01));
    CHECK(r_now[1].in(au::kilo(au::meters * au::ecefs)) == doctest::Approx(-1504.07f).epsilon(0.01));
    CHECK(r_now[2].in(au::kilo(au::meters * au::ecefs)) == doctest::Approx(1859.27f).epsilon(0.01));
    CHECK(v_now[0].in(au::kilo(au::meters * au::ecefs / au::seconds)) == doctest::Approx(-0.42784f).epsilon(0.01));
    CHECK(v_now[1].in(au::kilo(au::meters * au::ecefs / au::seconds)) == doctest::Approx(5.18216f).epsilon(0.01));
    CHECK(v_now[2].in(au::kilo(au::meters * au::ecefs / au::seconds)) == doctest::Approx(5.63173f).epsilon(0.01));
}

TEST_CASE("Check position and velocity 2025 6 25 18 0 0")
{
    RTC_HandleTypeDef hrtc;
    hrtc.Init.SynchPrediv = 1023;
    set_current_tick(1001);

    TimeUtils::DateTimeComponents epoch{
        .year = 2000,
        .month = 1,
        .day = 1,
        .hour = 12,
        .minute = 0,
        .second = 0,
        .millisecond = 0};
        
    TimeUtils::DateTimeComponents now{
        .year = 2025,
        .month = 6,
        .day = 25,
        .hour = 18,
        .minute = 0,
        .second = 0,
        .millisecond = 0};
    
    TimeUtils::DateTimeComponents future{
        .year = 2025,
        .month = 6,
        .day = 25,
        .hour = 18,
        .minute = 0,
        .second = 1,
        .millisecond = 0};

    SGP4 sgp4(&hrtc);

    // ISS (ZARYA)
    char longstr1[] = "1 25544U 98067A   25176.73245655  .00008102  00000-0  14854-3 0  9994";
    char longstr2[] = "2 25544  51.6390 264.7180 0001990 278.3788 217.2311 15.50240116516482";  

    auto parsed = sgp4_utils::parseTLE(longstr1, longstr2);
    REQUIRE(parsed.has_value());
    SGP4TwoLineElement data = parsed.value();
    sgp4.setSGP4TLE(data);
    
    TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(now, hrtc.Init.SynchPrediv);
    HAL_RTC_SetTime(&hrtc, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &rtc.date, RTC_FORMAT_BIN);
    HAL_RTCEx_SetSynchroShift(&hrtc, RTC_SHIFTADD1S_RESET, rtc.time.SubSeconds);
            
    std::array<au::QuantityF<au::Kilo<au::MetersInTemeFrame>>, 3> r_now;
    std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInTemeFrame>>, 3> v_now;
    au::QuantityU64<au::Milli<au::Seconds>> timestamp_now;

    sgp4.predict_teme(r_now, v_now, timestamp_now);

    std::array<float,3> expected_r {-3006.1573609732827f, 4331.221049310724f, -4290.439626312989f};
    std::array<float,3> expected_v {-3.3808196282756926f, -5.872899089174856f, -3.5610122777771087f};
    CHECK(r_now[0].in(au::kilo(au::metersInTemeFrame)) == doctest::Approx(expected_r[0]).epsilon(0.01));
    CHECK(r_now[1].in(au::kilo(au::metersInTemeFrame)) == doctest::Approx(expected_r[1]).epsilon(0.01));
    CHECK(r_now[2].in(au::kilo(au::metersInTemeFrame)) == doctest::Approx(expected_r[2]).epsilon(0.01));
    CHECK(v_now[0].in(au::kilo(au::metersPerSecondInTemeFrame)) == doctest::Approx(expected_v[0]).epsilon(0.01));
    CHECK(v_now[1].in(au::kilo(au::metersPerSecondInTemeFrame)) == doctest::Approx(expected_v[1]).epsilon(0.01));
    CHECK(v_now[2].in(au::kilo(au::metersPerSecondInTemeFrame)) == doctest::Approx(expected_v[2]).epsilon(0.01));

    rtc = TimeUtils::to_rtc(future, hrtc.Init.SynchPrediv);
    HAL_RTC_SetTime(&hrtc, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &rtc.date, RTC_FORMAT_BIN);
    HAL_RTCEx_SetSynchroShift(&hrtc, RTC_SHIFTADD1S_RESET, rtc.time.SubSeconds);
            
    std::array<au::QuantityF<au::Kilo<au::MetersInTemeFrame>>, 3> r_future;
    std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInTemeFrame>>, 3> v_future;
    au::QuantityU64<au::Milli<au::Seconds>> timestamp_future;

    sgp4.predict_teme(r_future, v_future, timestamp_future);

    float jd_now = TimeUtils::to_fractional_days(TimeUtils::to_timepoint(epoch), TimeUtils::to_timepoint(now));
    auto ecef_pos_now = coordinate_transformations::temeToecef(r_now, jd_now);
    auto ecef_vel_now = coordinate_transformations::temeToecef(v_now, jd_now);

    float jd_future = TimeUtils::to_fractional_days(TimeUtils::to_timepoint(epoch), TimeUtils::to_timepoint(future));
    auto ecef_pos_future = coordinate_transformations::temeToecef(r_future, jd_future);
    auto ecef_vel_future = coordinate_transformations::temeToecef(v_future, jd_future);

    CHECK(ecef_pos_future[0].in(au::metersInEcefFrame) - ecef_pos_now[0].in(au::metersInEcefFrame) == doctest::Approx(ecef_vel_now[0].in(au::metersPerSecondInEcefFrame)).epsilon(0.1));
    CHECK(ecef_pos_future[0].in(au::metersInEcefFrame) - ecef_pos_now[0].in(au::metersInEcefFrame) == doctest::Approx(ecef_vel_future[0].in(au::metersPerSecondInEcefFrame)).epsilon(0.1));

    CHECK(ecef_pos_future[1].in(au::metersInEcefFrame) - ecef_pos_now[1].in(au::metersInEcefFrame) == doctest::Approx(ecef_vel_now[1].in(au::metersPerSecondInEcefFrame)).epsilon(0.1));
    CHECK(ecef_pos_future[1].in(au::metersInEcefFrame) - ecef_pos_now[1].in(au::metersInEcefFrame) == doctest::Approx(ecef_vel_future[1].in(au::metersPerSecondInEcefFrame)).epsilon(0.1));

    CHECK(ecef_pos_future[2].in(au::metersInEcefFrame) - ecef_pos_now[2].in(au::metersInEcefFrame) == doctest::Approx(ecef_vel_now[2].in(au::metersPerSecondInEcefFrame)).epsilon(0.1));
    CHECK(ecef_pos_future[2].in(au::metersInEcefFrame) - ecef_pos_now[2].in(au::metersInEcefFrame) == doctest::Approx(ecef_vel_future[2].in(au::metersPerSecondInEcefFrame)).epsilon(0.1));


}
