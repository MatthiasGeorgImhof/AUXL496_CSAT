#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "TimeUtils.hpp"

#include <chrono>
#include <cstdint>
#include <iostream> // debugging

#include "mock_hal.h"

TEST_CASE("RTC <-> epoch_duration Conversions")
{
    // Sample RTC values
    constexpr uint32_t secondFraction = 1023;
    constexpr uint32_t subSeconds = 500;

    RTC_DateTypeDef rtc_date{};
    rtc_date.Year = 24; // Year 2024 (relative to 2000)
    rtc_date.Month = 10;
    rtc_date.Date = 27;

    RTC_TimeTypeDef rtc_time;
    rtc_time.Hours = 10;
    rtc_time.Minutes = 30;
    rtc_time.Seconds = 15;
    rtc_time.TimeFormat = RTC_HOURFORMAT_24;
    rtc_time.SubSeconds = subSeconds;
    rtc_time.SecondFraction = secondFraction;

    TimeUtils::RTCDateTimeSubseconds rtcdatetimeseconds = {
        .date = rtc_date,
        .time = rtc_time,
    };

    // Convert from RTC to epoch_duration
    TimeUtils::epoch_duration time1 =
        TimeUtils::from_rtc(rtcdatetimeseconds, secondFraction);

    // Convert back to RTC
    TimeUtils::RTCDateTimeSubseconds rtc_date_back_time =
        TimeUtils::to_rtc(time1, secondFraction);

    // Check if the values are approximately equal
    CHECK(rtc_date_back_time.date.Year == rtc_date.Year);
    CHECK(rtc_date_back_time.date.Month == rtc_date.Month);
    CHECK(rtc_date_back_time.date.Date == rtc_date.Date);
    CHECK(rtc_date_back_time.time.Hours == rtc_time.Hours);
    CHECK(rtc_date_back_time.time.Minutes == rtc_time.Minutes);
    CHECK(rtc_date_back_time.time.Seconds == rtc_time.Seconds);
    // Subsecond approximation, allow a small difference
    CHECK(abs(static_cast<int>(rtc_date_back_time.time.SubSeconds) -
              static_cast<int>(subSeconds)) <
          10); // Adjust tolerance as needed
}

TEST_CASE("Time Conversions and Extraction")
{
    // Test case 1: Conversion from date/time components to duration
    TimeUtils::DateTimeComponents components1 = {2024, 10, 27, 10, 30, 15, 500};
    TimeUtils::epoch_duration time1 = TimeUtils::to_epoch_duration(components1);
    CHECK(time1.count() > 0); // Basic sanity check

    // Test case 2: Extraction of components
    TimeUtils::DateTimeComponents extractedComponents =
        TimeUtils::extract_date_time(time1);

    CHECK(extractedComponents.year == 2024);
    CHECK(extractedComponents.month == 10);
    CHECK(extractedComponents.day == 27);
    CHECK(extractedComponents.hour == 10);
    CHECK(extractedComponents.minute == 30);
    CHECK(extractedComponents.second == 15);
    CHECK(extractedComponents.millisecond == 500);

    // Test case 3: Round trip conversion (date/time -> duration -> date/time)
    TimeUtils::DateTimeComponents components2 = {2023, 1, 1, 0, 0, 0, 0};
    TimeUtils::epoch_duration time2 = TimeUtils::to_epoch_duration(components2);
    TimeUtils::DateTimeComponents extractedComponents2 =
        TimeUtils::extract_date_time(time2);
    CHECK(extractedComponents2.year == 2023);
    CHECK(extractedComponents2.month == 1);
    CHECK(extractedComponents2.day == 1);
    CHECK(extractedComponents2.hour == 0);
    CHECK(extractedComponents2.minute == 0);
    CHECK(extractedComponents2.second == 0);
    CHECK(extractedComponents2.millisecond == 0);

    // Test case 4: Check the epoch
    TimeUtils::DateTimeComponents epochComponents = {
        TimeUtils::EPOCH_YEAR, TimeUtils::EPOCH_MONTH, TimeUtils::EPOCH_DAY, 0, 0,
        0, 0};
    TimeUtils::epoch_duration epoch_duration_value =
        TimeUtils::to_epoch_duration(epochComponents);
    CHECK(epoch_duration_value.count() == 0);
}

TEST_CASE("Duration Arithmetic")
{
    TimeUtils::DateTimeComponents components1 = {2024, 1, 1, 0, 0, 0, 0};
    TimeUtils::epoch_duration time1 = TimeUtils::to_epoch_duration(components1);
    TimeUtils::epoch_duration time2 =
        time1 + std::chrono::seconds(60); // Add 60 seconds

    TimeUtils::DateTimeComponents extractedComponents =
        TimeUtils::extract_date_time(time2);

    CHECK(extractedComponents.year == 2024);
    CHECK(extractedComponents.month == 1);
    CHECK(extractedComponents.day == 1);
    CHECK(extractedComponents.hour == 0);
    CHECK(extractedComponents.minute == 1);
    CHECK(extractedComponents.second == 0);
    CHECK(extractedComponents.millisecond == 0);
}

TEST_CASE("Edge Cases and Error Handling")
{
    // Test case 1: Invalid month
    TimeUtils::DateTimeComponents invalidComponents1 = {2024, 13, 1, 0, 0, 0, 0};
    TimeUtils::to_epoch_duration(invalidComponents1);

    // Test case 2: Invalid day
    TimeUtils::DateTimeComponents invalidComponents2 = {2024, 2, 30, 0, 0, 0, 0};
    TimeUtils::to_epoch_duration(invalidComponents2);

    // Test case 3: Year before EPOCH
    TimeUtils::DateTimeComponents invalidComponents3 = {1999, 1, 1, 0, 0, 0, 0};
    TimeUtils::to_epoch_duration(invalidComponents3);

    // Test case 4: Valid epoch components
    TimeUtils::DateTimeComponents epochComponents = {
        TimeUtils::EPOCH_YEAR, TimeUtils::EPOCH_MONTH, TimeUtils::EPOCH_DAY, 0, 0,
        0, 0};
    TimeUtils::epoch_duration epoch_duration_value =
        TimeUtils::to_epoch_duration(epochComponents);
    CHECK(epoch_duration_value.count() == 0);
}

TEST_CASE("to_uint64 and from_uint64 conversions")
{
    TimeUtils::DateTimeComponents components = {2024, 11, 15, 12, 30, 45, 750};
    TimeUtils::epoch_duration original_duration =
        TimeUtils::to_epoch_duration(components);

    uint64_t uint64_value = TimeUtils::to_uint64(original_duration);
    TimeUtils::epoch_duration converted_duration =
        TimeUtils::from_uint64(uint64_value);

    CHECK(original_duration.count() == converted_duration.count());
}

TEST_CASE("RTC <-> DateTimeComponents Conversions")
{
    constexpr uint32_t secondFraction = 1023;

    TimeUtils::DateTimeComponents components = {2024, 11, 15, 12, 30, 45, 750};
    TimeUtils::RTCDateTimeSubseconds rtc_datetime = TimeUtils::to_rtc(components, secondFraction);

    CHECK(rtc_datetime.date.Year == components.year - TimeUtils::EPOCH_YEAR);
    CHECK(rtc_datetime.date.Month == components.month);
    CHECK(rtc_datetime.date.Date == components.day);
    CHECK(rtc_datetime.time.Hours == components.hour);
    CHECK(rtc_datetime.time.Minutes == components.minute);
    CHECK(rtc_datetime.time.Seconds == components.second);

    // Ensure SubSeconds is within reasonable bounds
    CHECK(rtc_datetime.time.SubSeconds >= 0);
    CHECK(rtc_datetime.time.SubSeconds <= secondFraction);
}

TEST_CASE("Comprehensive Round Trip Test")
{
    constexpr uint32_t secondFraction = 1023;
    TimeUtils::DateTimeComponents initial_components = {2025, 5, 20, 8, 15, 30, 250};

    // Convert to epoch duration
    TimeUtils::epoch_duration epoch_duration_value =
        TimeUtils::to_epoch_duration(initial_components);

    // Convert to RTC
    TimeUtils::RTCDateTimeSubseconds rtc_datetime =
        TimeUtils::to_rtc(epoch_duration_value, secondFraction);

    // Convert back to epoch duration
    TimeUtils::epoch_duration final_epoch_duration =
        TimeUtils::from_rtc(rtc_datetime, secondFraction);

    // Extract date/time components
    TimeUtils::DateTimeComponents final_components =
        TimeUtils::extract_date_time(final_epoch_duration);

    // Perform checks (allowing for millisecond discrepancies)
    CHECK(final_components.year == initial_components.year);
    CHECK(final_components.month == initial_components.month);
    CHECK(final_components.day == initial_components.day);
    CHECK(final_components.hour == initial_components.hour);
    CHECK(final_components.minute == initial_components.minute);
    CHECK(final_components.second == initial_components.second);
    CHECK(abs(static_cast<int>(final_components.millisecond) - static_cast<int>(initial_components.millisecond)) < 20); // Tolerance for millisecond differences
}

TEST_CASE("Leap Year Test")
{
    TimeUtils::DateTimeComponents components = {2024, 2, 29, 12, 0, 0, 0}; //
    // Leap year
    TimeUtils::epoch_duration duration =
        TimeUtils::to_epoch_duration(components);
    TimeUtils::DateTimeComponents extracted_components =
        TimeUtils::extract_date_time(duration);
    CHECK(extracted_components.year == 2024);
    CHECK(extracted_components.month == 2);
    CHECK(extracted_components.day == 29);
}

TEST_CASE("Non-Leap Year Test")
{
    TimeUtils::DateTimeComponents components = {2023, 2, 28, 12, 0, 0, 0}; //
    // Non-Leap year
    TimeUtils::epoch_duration duration =
        TimeUtils::to_epoch_duration(components);
    TimeUtils::DateTimeComponents extracted_components =
        TimeUtils::extract_date_time(duration);
    CHECK(extracted_components.year == 2023);
    CHECK(extracted_components.month == 2);
    CHECK(extracted_components.day == 28);
}

TEST_CASE("to_rtc Stress Test")
{
    const uint32_t secondFraction = 1023;

    int i = 0;
    for(uint16_t millisecond = 0; millisecond < 1000; millisecond += 125) {
        TimeUtils::DateTimeComponents dtc{
            .year = 2000,
            .month = 1,
            .day = 1,
            .hour = 0,
            .minute = 0,
            .second = 1,
            .millisecond = static_cast<uint16_t>(millisecond)};
        auto duration = TimeUtils::to_epoch_duration(dtc);
        auto rtc = TimeUtils::to_rtc(duration, secondFraction);

        set_mocked_rtc_time(rtc.time);
        set_mocked_rtc_date(rtc.date);

        CHECK(dtc.hour == rtc.time.Hours);
        CHECK(dtc.minute == rtc.time.Minutes);
        CHECK(dtc.second == rtc.time.Seconds);
        CHECK(1023 - i*128 == rtc.time.SubSeconds);
        CHECK(dtc.year == rtc.date.Year + TimeUtils::EPOCH_YEAR);
        CHECK(dtc.month == rtc.date.Month);
        CHECK(dtc.day == rtc.date.Date);
        ++i;
    }
}

TEST_CASE("from_rtc Stress Test")
{
    const uint32_t secondFraction = 1023;
    TimeUtils::DateTimeComponents dtc{
        .year = 2000,
        .month = 1,
        .day = 1,
        .hour = 0,
        .minute = 0,
        .second = 1,
        .millisecond = 0};
    auto duration = TimeUtils::to_epoch_duration(dtc);
    auto rtc = TimeUtils::to_rtc(duration, secondFraction);

    int i = 0;
    for(int16_t fraction = 1023; fraction >= 0; fraction -= 128) {
        rtc.time.SubSeconds = static_cast<uint16_t>(fraction);

        set_mocked_rtc_time(rtc.time);
        set_mocked_rtc_date(rtc.date);

        auto duration = TimeUtils::from_rtc(rtc, secondFraction);
        auto dtc_ = TimeUtils::extract_date_time(duration);

        CHECK(dtc.hour == dtc_.hour);
        CHECK(dtc.minute == dtc_.minute);
        CHECK(dtc.second == dtc_.second);
        CHECK(i == dtc_.millisecond);
        CHECK(dtc.year == dtc_.year);
        CHECK(dtc.month == dtc_.month);
        CHECK(dtc.day == dtc_.day);
        i+=125;
    }
}

TEST_CASE("to_rtc Stress Tests - Edge Cases and Boundaries") {
    const uint32_t secondFraction = 1023;

    SUBCASE("Epoch Boundary") {
        TimeUtils::epoch_duration epoch_time(0); // Exactly at the epoch
        TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(epoch_time, secondFraction);
        CHECK(rtc.date.Year == 0);
        CHECK(rtc.date.Month == 1);
        CHECK(rtc.date.Date == 1);
        CHECK(rtc.time.Hours == 0);
        CHECK(rtc.time.Minutes == 0);
        CHECK(rtc.time.Seconds == 0);
        CHECK(rtc.time.SubSeconds == secondFraction);
    }

    SUBCASE("Near Epoch - Just Before") {
        TimeUtils::epoch_duration near_epoch(-1); // Just before the epoch
        TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(near_epoch, secondFraction);
        //  Year will wrap around, Month and Date are undefined.
        CHECK(rtc.time.Hours == 23);
        CHECK(rtc.time.Minutes == 59);
        CHECK(rtc.time.Seconds == 59);
        CHECK(rtc.time.SubSeconds == 1);
    }

    SUBCASE("Maximum epoch_duration Value") {
        TimeUtils::epoch_duration max_duration(std::chrono::milliseconds::max());
        TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(max_duration, secondFraction);

        // Check for reasonable values.  Exact values are difficult to predict, but should be "large"
        CHECK(rtc.date.Year > 100);  // Well past 2100
        CHECK(rtc.time.Hours >= 0);
        CHECK(rtc.time.Hours < 24);
        CHECK(rtc.time.Minutes >= 0);
        CHECK(rtc.time.Minutes < 60);
        CHECK(rtc.time.Seconds >= 0);
        CHECK(rtc.time.Seconds < 60);
    }

    SUBCASE("Minimum epoch_duration Value") {
        TimeUtils::epoch_duration min_duration(std::chrono::milliseconds::min());
        TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(min_duration, secondFraction);
        // Values will likely be close to zero, year will wrap around to nearly zero
        CHECK(rtc.date.Year < 5); // Likely wrap around to ~0

    }

    SUBCASE("Year 2100 Boundary") {
        TimeUtils::DateTimeComponents components = {2100, 1, 1, 0, 0, 0, 0};
        TimeUtils::epoch_duration duration = TimeUtils::to_epoch_duration(components);
        TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(duration, secondFraction);
        CHECK(rtc.date.Year == 100);
        CHECK(rtc.date.Month == 1);
        CHECK(rtc.date.Date == 1);
        CHECK(rtc.time.Hours == 0);
        CHECK(rtc.time.Minutes == 0);
        CHECK(rtc.time.Seconds == 0);
        CHECK(rtc.time.SubSeconds == secondFraction);
    }

    SUBCASE("Leap Year - Feb 29th") {
        TimeUtils::DateTimeComponents components = {2024, 2, 29, 12, 30, 0, 0};  // Leap year
        TimeUtils::epoch_duration duration = TimeUtils::to_epoch_duration(components);
        TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(duration, secondFraction);
        CHECK(rtc.date.Year == 24);
        CHECK(rtc.date.Month == 2);
        CHECK(rtc.date.Date == 29);
        CHECK(rtc.time.Hours == 12);
        CHECK(rtc.time.Minutes == 30);
        CHECK(rtc.time.Seconds == 0);
        CHECK(rtc.time.SubSeconds == secondFraction);
    }

    SUBCASE("Non-Leap Year - Attempt Feb 29th (should wrap to March 1st)") {
        TimeUtils::DateTimeComponents components = {2023, 2, 29, 12, 30, 0, 0};  // Non-leap year. Should NOT be valid, but the code doesn't explicitly handle this so the result may be weird.
        TimeUtils::epoch_duration duration = TimeUtils::to_epoch_duration(components);
        TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(duration, secondFraction);
        // The exact values are unpredictable due to no proper leap-year check.
        // Just ensure some reasonable values.
        CHECK((rtc.date.Month >= 1 && rtc.date.Month <= 12));
        CHECK((rtc.date.Date >= 1 && rtc.date.Date <= 31));
    }

    SUBCASE("End of Month - March 31st") {
        TimeUtils::DateTimeComponents components = {2024, 3, 31, 23, 59, 59, 999};
        TimeUtils::epoch_duration duration = TimeUtils::to_epoch_duration(components);
        TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(duration, secondFraction);
        CHECK(rtc.date.Year == 24);
        CHECK(rtc.date.Month == 3);
        CHECK(rtc.date.Date == 31);
        CHECK(rtc.time.Hours == 23);
        CHECK(rtc.time.Minutes == 59);
        CHECK(rtc.time.Seconds == 59);
        CHECK(rtc.time.SubSeconds == 1);
    }

    SUBCASE("Subseconds at Millisecond Boundaries") {
        TimeUtils::DateTimeComponents components_0ms = {2024, 1, 1, 12, 0, 0, 0};
        TimeUtils::epoch_duration duration_0ms = TimeUtils::to_epoch_duration(components_0ms);
        TimeUtils::RTCDateTimeSubseconds rtc_0ms = TimeUtils::to_rtc(duration_0ms, secondFraction);
        CHECK(rtc_0ms.time.SubSeconds == secondFraction);

        TimeUtils::DateTimeComponents components_500ms = {2024, 1, 1, 12, 0, 0, 500};
        TimeUtils::epoch_duration duration_500ms = TimeUtils::to_epoch_duration(components_500ms);
        TimeUtils::RTCDateTimeSubseconds rtc_500ms = TimeUtils::to_rtc(duration_500ms, secondFraction);
        CHECK(rtc_500ms.time.SubSeconds == 511);

        TimeUtils::DateTimeComponents components_999ms = {2024, 1, 1, 12, 0, 0, 999};
        TimeUtils::epoch_duration duration_999ms = TimeUtils::to_epoch_duration(components_999ms);
        TimeUtils::RTCDateTimeSubseconds rtc_999ms = TimeUtils::to_rtc(duration_999ms, secondFraction);
        CHECK(rtc_999ms.time.SubSeconds == 1);
    }
}

TEST_CASE("to_rtc round trip tests"){
    // RTC_HandleTypeDef hrtc;
    const uint32_t secondFraction = 1023;
    // hrtc.Init.SynchPrediv = secondFraction;
    SUBCASE("round trip"){
        TimeUtils::DateTimeComponents components = {2024, 3, 31, 23, 59, 59, 999};
        TimeUtils::epoch_duration duration = TimeUtils::to_epoch_duration(components);
        TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(duration, secondFraction);
        TimeUtils::epoch_duration back = TimeUtils::from_rtc(rtc, secondFraction);
        TimeUtils::DateTimeComponents back_components = TimeUtils::extract_date_time(back);
        CHECK(back_components.year == components.year);
        CHECK(back_components.month == components.month);
        CHECK(back_components.day == components.day);
        CHECK(back_components.hour == components.hour);
        CHECK(back_components.minute == components.minute);
        CHECK(back_components.second == components.second);
    }
}

#include "au.hpp"

TEST_CASE("TaskPositionService Test with mock_hal: time round trip")
{
    RTC_HandleTypeDef hrtc;
    RTC_HandleTypeDef *hrtc_ = &hrtc;

    const uint32_t secondFraction = 1023;
    hrtc.Init.SynchPrediv = secondFraction;

    const std::chrono::milliseconds dduration(1);

    TimeUtils::DateTimeComponents dtc{
        .year = 2000,
        .month = 1,
        .day = 1,
        .hour = 0,
        .minute = 0,
        .second = 1,
        .millisecond = 0};
    auto duration = TimeUtils::to_epoch_duration(dtc);
    auto rtc = TimeUtils::to_rtc(duration, secondFraction);
    set_mocked_rtc_time(rtc.time);
    set_mocked_rtc_date(rtc.date);


    for (int i = 0; i < 1000; ++i)
    {
        TimeUtils::RTCDateTimeSubseconds rtc;
        HAL_RTC_GetTime(hrtc_, &rtc.time, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(hrtc_, &rtc.date, RTC_FORMAT_BIN);
        auto from_rtc =TimeUtils::from_rtc(rtc, hrtc_->Init.SynchPrediv).count();
        auto timestamp = au::make_quantity<au::Milli<au::Seconds>>(from_rtc);

        int64_t microsecond = timestamp.in(au::micro(au::seconds));
        CHECK(microsecond == duration.count() * 1000);

        duration += dduration;
        rtc = TimeUtils::to_rtc(duration, secondFraction);
        set_mocked_rtc_time(rtc.time);
        set_mocked_rtc_date(rtc.date);
    }
}

TEST_CASE("TimeUtils::to_fractional_days")
{
    // https://aa.usno.navy.mil/data/JulianDate
    auto j2000 = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents {
            .year = 2000, 
            .month = 1,
            .day = 1,
            .hour = 12,
            .minute = 0, 
            .second = 0,
            .millisecond = 0
    });

    SUBCASE("2001 07 23 02:55::00 UTC")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents {
            .year = 2001, 
            .month = 7,
            .day = 23,
            .hour = 2,
            .minute = 55, 
            .second = 0,
            .millisecond = 0
        });

        float jdut2 = TimeUtils::to_fractional_days(j2000, now);
        CHECK(jdut2 == doctest::Approx(2452113.621528 - 2451545.0).epsilon(1e-6f));
    }

    SUBCASE("2005 07 23 02:55::00 UTC")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents {
            .year = 2005, 
            .month = 7,
            .day = 23,
            .hour = 2,
            .minute = 55, 
            .second = 0,
            .millisecond = 0
        });

        float jdut2 = TimeUtils::to_fractional_days(j2000, now);
        CHECK(jdut2 == doctest::Approx(2453574.621528 - 2451545.0).epsilon(1e-6f));
    }

    SUBCASE("2015 07 23 02:55::00 UTC")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents {
            .year = 2015, 
            .month = 7,
            .day = 23,
            .hour = 2,
            .minute = 55, 
            .second = 0,
            .millisecond = 0
        });

        float jdut2 = TimeUtils::to_fractional_days(j2000, now);
        CHECK(jdut2 == doctest::Approx(2457226.621528 - 2451545.0).epsilon(1e-6f));
    }

    
    SUBCASE("2025 07 23 02:55::00 UTC")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents {
            .year = 2025, 
            .month = 7,
            .day = 23,
            .hour = 2,
            .minute = 55, 
            .second = 0,
            .millisecond = 0
        });

        float jdut2 = TimeUtils::to_fractional_days(j2000, now);
        CHECK(jdut2 == doctest::Approx(2460879.621528 - 2451545.0).epsilon(1e-6f));
    }
    
    SUBCASE("2035 07 23 02:55::00 UTC")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents {
            .year = 2035, 
            .month = 7,
            .day = 23,
            .hour = 2,
            .minute = 55, 
            .second = 0,
            .millisecond = 0
        });

        float jdut2 = TimeUtils::to_fractional_days(j2000, now);
        CHECK(jdut2 == doctest::Approx(2464531.621528 - 2451545.0 ).epsilon(1e-6f));
    }


    SUBCASE("2045 07 23 02:55::00 UTC")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents {
            .year = 2045, 
            .month = 7,
            .day = 23,
            .hour = 2,
            .minute = 55, 
            .second = 0,
            .millisecond = 0
        });

        float jdut2 = TimeUtils::to_fractional_days(j2000, now);
        CHECK(jdut2 == doctest::Approx(2468184.621528 - 2451545.0 ).epsilon(1e-6f));
    }
}

TEST_CASE("TimeUtils::gsTimeJ2000")
{
    // https://aa.usno.navy.mil/data/siderealtime
    auto j2000 = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents {
            .year = 2000, 
            .month = 1,
            .day = 1,
            .hour = 12,
            .minute = 0, 
            .second = 0,
            .millisecond = 0
    });

    
    SUBCASE("6939.833333")
    {
        float jdut2 = 6939.833333f;
        double gstime = TimeUtils::gsTimeJ2000(jdut2);
        CHECK(gstime == doctest::Approx(14.712605328).epsilon(1e-3f));

    }
    
    SUBCASE("2001 07 23 02:55::00 UTC -> 22:58:41.0238")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents {
            .year = 2001, 
            .month = 7,
            .day = 23,
            .hour = 2,
            .minute = 55, 
            .second = 0,
            .millisecond = 0
        });

        float jdut2 = TimeUtils::to_fractional_days(j2000, now);
        double gstime = TimeUtils::gsTimeJ2000(jdut2);
        CHECK(gstime == doctest::Approx(22.0 + 58.0 / 60.0 + 41.0238 / 3600.0).epsilon(1e-3f));
    }

    SUBCASE("2005 07 23 02:55::00 UTC -> 22:58:48.4159")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents {
            .year = 2005, 
            .month = 7,
            .day = 23,
            .hour = 2,
            .minute = 55, 
            .second = 0,
            .millisecond = 0
        });

        float jdut2 = TimeUtils::to_fractional_days(j2000, now);
        double gstime = TimeUtils::gsTimeJ2000(jdut2);
        CHECK(gstime == doctest::Approx(22.0 + 58.0 / 60.0 + 48.4159 / 3600.0).epsilon(1e-3f));
    }

    SUBCASE("2015 07 23 02:55::00 UTC -> 22:57:08.6196")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents {
            .year = 2015, 
            .month = 7,
            .day = 23,
            .hour = 2,
            .minute = 55, 
            .second = 0,
            .millisecond = 0
        });

        float jdut2 = TimeUtils::to_fractional_days(j2000, now);
        double gstime = TimeUtils::gsTimeJ2000(jdut2);
        CHECK(gstime == doctest::Approx(22.0 + 57.0 / 60.0 + 08.6196 / 3600.0).epsilon(1e-3f));
    }

    
    SUBCASE("2025 07 23 02:55::00 UTC -> 22:59:25.3806")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents {
            .year = 2025, 
            .month = 7,
            .day = 23,
            .hour = 2,
            .minute = 55, 
            .second = 0,
            .millisecond = 0
        });

        float jdut2 = TimeUtils::to_fractional_days(j2000, now);
        double gstime = TimeUtils::gsTimeJ2000(jdut2);
        CHECK(gstime == doctest::Approx(22.0 + 59.0 / 60.0 + 25.3806/ 3600.0).epsilon(1e-3f));
    }
    
    SUBCASE("2035 07 23 02:55::00 UTC -> 22:57:45.5880")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents {
            .year = 2035, 
            .month = 7,
            .day = 23,
            .hour = 2,
            .minute = 55, 
            .second = 0,
            .millisecond = 0
        });

        float jdut2 = TimeUtils::to_fractional_days(j2000, now);
        double gstime = TimeUtils::gsTimeJ2000(jdut2);
        CHECK(gstime == doctest::Approx(22.0 + 57.0 / 60.0 + 45.5880 / 3600.0).epsilon(1e-3f));
    }


    SUBCASE("2045 07 23 02:55::00 UTC -> 23:00:02.3526")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents {
            .year = 2045, 
            .month = 7,
            .day = 23,
            .hour = 2,
            .minute = 55, 
            .second = 0,
            .millisecond = 0
        });

        float jdut2 = TimeUtils::to_fractional_days(j2000, now);
        double gstime = TimeUtils::gsTimeJ2000(jdut2);
        CHECK(gstime == doctest::Approx(23.0 + 0 / 60.0 + 2.3526 / 3600.0).epsilon(1e-3f));
    }
}

