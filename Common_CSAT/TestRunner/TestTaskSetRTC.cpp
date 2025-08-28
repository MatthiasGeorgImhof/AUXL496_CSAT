#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "TimeUtils.hpp"
#include <chrono>
#include <cstdint>
#include <iostream> //debugging

#include "mock_hal.h"

TEST_CASE("RTC <-> epoch_duration Conversions") {
    // Sample RTC values
    constexpr uint32_t secondFraction = 1023;
    constexpr uint32_t subSeconds = 500;


    RTC_DateTypeDef rtc_date {};
    rtc_date.Year = 24;    // Year 2024 (relative to 2000)
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
    TimeUtils::epoch_duration time1 = TimeUtils::from_rtc(rtcdatetimeseconds, secondFraction);

    // Convert back to RTC
    TimeUtils::RTCDateTimeSubseconds rtc_date_back_time = TimeUtils::to_rtc(time1, secondFraction);

    // Check if the values are approximately equal
    CHECK(rtc_date_back_time.date.Year == rtc_date.Year);
    CHECK(rtc_date_back_time.date.Month == rtc_date.Month);
    CHECK(rtc_date_back_time.date.Date == rtc_date.Date);
    CHECK(rtc_date_back_time.time.Hours == rtc_time.Hours);
    CHECK(rtc_date_back_time.time.Minutes == rtc_time.Minutes);
    CHECK(rtc_date_back_time.time.Seconds == rtc_time.Seconds);
    // Subsecond approximation, allow a small difference
    CHECK(abs(static_cast<int>(rtc_date_back_time.time.SubSeconds) - static_cast<int>(subSeconds)) < 10); //Adjust tolerance as needed
}

TEST_CASE("Time Conversions and Extraction") {
    // Test case 1: Conversion from date/time components to duration
    TimeUtils::DateTimeComponents components1 = {2024, 10, 27, 10, 30, 15, 500};
    TimeUtils::epoch_duration time1 = TimeUtils::to_epoch_duration(components1);
    CHECK(time1.count() > 0); // Basic sanity check

    // Test case 2: Extraction of components
    TimeUtils::DateTimeComponents extractedComponents = TimeUtils::extract_date_time(time1);

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
    TimeUtils::DateTimeComponents extractedComponents2 = TimeUtils::extract_date_time(time2);
    CHECK(extractedComponents2.year == 2023);
    CHECK(extractedComponents2.month == 1);
    CHECK(extractedComponents2.day == 1);
    CHECK(extractedComponents2.hour == 0);
    CHECK(extractedComponents2.minute == 0);
    CHECK(extractedComponents2.second == 0);
    CHECK(extractedComponents2.millisecond == 0);

    // Test case 4: Check the epoch
    TimeUtils::DateTimeComponents epochComponents = {TimeUtils::EPOCH_YEAR, TimeUtils::EPOCH_MONTH, TimeUtils::EPOCH_DAY, 0, 0, 0, 0};
    TimeUtils::epoch_duration epoch_duration_value = TimeUtils::to_epoch_duration(epochComponents);
    CHECK(epoch_duration_value.count() == 0);
}

TEST_CASE("Duration Arithmetic") {
    TimeUtils::DateTimeComponents components1 = {2024, 1, 1, 0, 0, 0, 0};
    TimeUtils::epoch_duration time1 = TimeUtils::to_epoch_duration(components1);
    TimeUtils::epoch_duration time2 = time1 + std::chrono::seconds(60); // Add 60 seconds

    TimeUtils::DateTimeComponents extractedComponents = TimeUtils::extract_date_time(time2);

    CHECK(extractedComponents.year == 2024);
    CHECK(extractedComponents.month == 1);
    CHECK(extractedComponents.day == 1);
    CHECK(extractedComponents.hour == 0);
    CHECK(extractedComponents.minute == 1);
    CHECK(extractedComponents.second == 0);
    CHECK(extractedComponents.millisecond == 0);
}

TEST_CASE("Edge Cases and Error Handling (Illustrative)") {
    // Note:  This is where you'd add checks for boundary conditions and potential
    // errors, like very large durations that might cause overflows, or invalid date values.
    // For demonstration, let's just check a very early date.  Proper error handling
    // would involve throwing exceptions or returning error codes from the functions.

    TimeUtils::DateTimeComponents epochComponents = {TimeUtils::EPOCH_YEAR, TimeUtils::EPOCH_MONTH, TimeUtils::EPOCH_DAY, 0, 0, 0, 0};
    TimeUtils::epoch_duration early_time = TimeUtils::to_epoch_duration(epochComponents); // The epoch
    CHECK(early_time.count() == 0);
}

TEST_CASE("to_uint64 and from_uint64 conversions") {
    TimeUtils::DateTimeComponents components = {2024, 11, 15, 12, 30, 45, 750};
    TimeUtils::epoch_duration original_duration = TimeUtils::to_epoch_duration(components);

    uint64_t uint64_value = TimeUtils::to_uint64(original_duration);
    TimeUtils::epoch_duration converted_duration = TimeUtils::from_uint64(uint64_value);

    CHECK(original_duration.count() == converted_duration.count());
}

TEST_CASE("RTC <-> DateTimeComponents Conversions") {
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

TEST_CASE("Comprehensive Round Trip Test") {
    constexpr uint32_t secondFraction = 1023;
    TimeUtils::DateTimeComponents initial_components = {2025, 5, 20, 8, 15, 30, 250};

    // Convert to epoch duration
    TimeUtils::epoch_duration epoch_duration_value = TimeUtils::to_epoch_duration(initial_components);

    // Convert to RTC
    TimeUtils::RTCDateTimeSubseconds rtc_datetime = TimeUtils::to_rtc(epoch_duration_value, secondFraction);

    // Convert back to epoch duration
    TimeUtils::epoch_duration final_epoch_duration = TimeUtils::from_rtc(rtc_datetime, secondFraction);

    // Extract date/time components
    TimeUtils::DateTimeComponents final_components = TimeUtils::extract_date_time(final_epoch_duration);

    // Perform checks (allowing for millisecond discrepancies)
    CHECK(final_components.year == initial_components.year);
    CHECK(final_components.month == initial_components.month);
    CHECK(final_components.day == initial_components.day);
    CHECK(final_components.hour == initial_components.hour);
    CHECK(final_components.minute == initial_components.minute);
    CHECK(final_components.second == initial_components.second);
    CHECK(abs(static_cast<int>(final_components.millisecond) - static_cast<int>(initial_components.millisecond)) < 20); // Tolerance for millisecond differences

}

TEST_CASE("Leap Year Test") {
    TimeUtils::DateTimeComponents components = {2024, 2, 29, 12, 0, 0, 0}; // Leap year
    TimeUtils::epoch_duration duration = TimeUtils::to_epoch_duration(components);
    TimeUtils::DateTimeComponents extracted_components = TimeUtils::extract_date_time(duration);
    CHECK(extracted_components.year == 2024);
    CHECK(extracted_components.month == 2);
    CHECK(extracted_components.day == 29);
}

TEST_CASE("Non-Leap Year Test") {
    TimeUtils::DateTimeComponents components = {2023, 2, 28, 12, 0, 0, 0}; // Non-Leap year
    TimeUtils::epoch_duration duration = TimeUtils::to_epoch_duration(components);
    TimeUtils::DateTimeComponents extracted_components = TimeUtils::extract_date_time(duration);
    CHECK(extracted_components.year == 2023);
    CHECK(extracted_components.month == 2);
    CHECK(extracted_components.day == 28);
}

TEST_CASE("New to_epoch_duration with nanoseconds Test") {
    TimeUtils::epoch_duration duration = TimeUtils::to_epoch_duration(2024, 12, 25, 10, 30, 45, 500000000); //500ms
    TimeUtils::DateTimeComponents components = TimeUtils::extract_date_time(duration);

    CHECK(components.year == 2024);
    CHECK(components.month == 12);
    CHECK(components.day == 25);
    CHECK(components.hour == 10);
    CHECK(components.minute == 30);
    CHECK(components.second == 45);
    CHECK(components.millisecond == 500);

    TimeUtils::epoch_duration duration_neg = TimeUtils::to_epoch_duration(2024, 12, 25, 10, 30, 45, -250000000); // -250ms
    TimeUtils::DateTimeComponents components_neg = TimeUtils::extract_date_time(duration_neg);

    CHECK(components_neg.year == 2024);
    CHECK(components_neg.month == 12);
    CHECK(components_neg.day == 25);
    CHECK(components_neg.hour == 10);
    CHECK(components_neg.minute == 30);
    CHECK(components_neg.second == 44);
    CHECK(components_neg.millisecond == 750);
}

TEST_CASE("RTC Conversion with Negative Subseconds (Milliseconds)") {
    constexpr uint32_t secondFraction = 1023;

    // Create a DateTimeComponent representing a time just before a whole second.
    TimeUtils::DateTimeComponents components = {2024, 11, 15, 12, 30, 45, 100}; // 100ms

    //Convert to RTC
    TimeUtils::RTCDateTimeSubseconds rtc_datetime = TimeUtils::to_rtc(components, secondFraction);
    std::cout << "Subseconds value = " << (int)rtc_datetime.time.SubSeconds << std::endl;
    CHECK(rtc_datetime.date.Year == components.year - TimeUtils::EPOCH_YEAR);
    CHECK(rtc_datetime.date.Month == components.month);
    CHECK(rtc_datetime.date.Date == components.day);
    CHECK(rtc_datetime.time.Hours == components.hour);
    CHECK(rtc_datetime.time.Minutes == components.minute);
    CHECK(rtc_datetime.time.Seconds == components.second);

    // Ensure SubSeconds is within reasonable bounds
    CHECK(rtc_datetime.time.SubSeconds >= 0);
    CHECK(rtc_datetime.time.SubSeconds <= secondFraction);

    //Negative test case

    TimeUtils::DateTimeComponents components_neg = {2024, 11, 15, 12, 30, 45, 900}; // 900ms

    //Convert to RTC
    TimeUtils::RTCDateTimeSubseconds rtc_datetime_neg = TimeUtils::to_rtc(components_neg, secondFraction);
    std::cout << "Subseconds value = " << (int)rtc_datetime_neg.time.SubSeconds << std::endl;
    CHECK(rtc_datetime_neg.date.Year == components.year - TimeUtils::EPOCH_YEAR);
    CHECK(rtc_datetime_neg.date.Month == components.month);
    CHECK(rtc_datetime_neg.date.Date == components.day);
    CHECK(rtc_datetime_neg.time.Hours == components.hour);
    CHECK(rtc_datetime_neg.time.Minutes == components.minute);
    CHECK(rtc_datetime_neg.time.Seconds == components.second);

    // Ensure SubSeconds is within reasonable bounds
    CHECK(rtc_datetime_neg.time.SubSeconds >= 0);
    CHECK(rtc_datetime_neg.time.SubSeconds <= secondFraction);
}

TEST_CASE("Boundary Tests") {
    //Max date:
    TimeUtils::DateTimeComponents components_max = {2079, 12, 31, 23, 59, 59, 999};
    TimeUtils::epoch_duration duration_max = TimeUtils::to_epoch_duration(components_max);
    TimeUtils::DateTimeComponents extracted_components_max = TimeUtils::extract_date_time(duration_max);

    CHECK(extracted_components_max.year == 2079);
    CHECK(extracted_components_max.month == 12);
    CHECK(extracted_components_max.day == 31);
    CHECK(extracted_components_max.hour == 23);
    CHECK(extracted_components_max.minute == 59);
    CHECK(extracted_components_max.second == 59);
    CHECK(extracted_components_max.millisecond == 999);
}