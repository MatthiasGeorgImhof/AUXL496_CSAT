#ifndef TIMEUTILS_HPP
#define TIMEUTILS_HPP

#include <chrono>
#include <ctime>
#include <cstdint>
#ifdef __arm__
#include "usbd_cdc_if.h"
#endif
#ifdef __x86_64__
#include "mock_hal.h"
#endif

namespace TimeUtils {

constexpr uint16_t EPOCH_YEAR = 2000;
constexpr uint8_t EPOCH_MONTH = 1;
constexpr uint8_t EPOCH_DAY = 1;

// Define a struct for date and time components
struct DateTimeComponents {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t millisecond;
};

struct RTCDateTimeSubseconds {
  RTC_DateTypeDef date; // Date components
  RTC_TimeTypeDef time; // Time components
};

// Choose your duration type
using epoch_duration = std::chrono::milliseconds;  
using epoch_time_point = std::chrono::time_point<std::chrono::system_clock, epoch_duration>;

// Function declarations
epoch_duration to_epoch_duration(const std::chrono::system_clock::time_point &tp);
epoch_duration to_epoch_duration(const DateTimeComponents &components);
epoch_duration to_epoch_duration(uint16_t year, uint8_t month, uint8_t day,
                                 uint8_t hour, uint8_t minute, uint8_t second,
                                 int32_t nanosecond);

std::chrono::system_clock::time_point to_timepoint(const epoch_duration &d);
std::chrono::system_clock::time_point to_timepoint(uint16_t past_year, float past_fractional_day);
std::chrono::system_clock::time_point to_timepoint(const DateTimeComponents &components);

float to_fractional_days(
    const std::chrono::system_clock::time_point &start,
    const std::chrono::system_clock::time_point &end);
float fractional_days_between(
    uint16_t past_year, float past_fractional_day, uint16_t current_year,
    uint8_t current_month, uint8_t current_day, uint8_t current_hour,
    uint8_t current_minute, uint8_t current_second,
    uint16_t current_millisecond);

DateTimeComponents extract_date_time(const epoch_duration &d);

// Function to get epoch_duration as uint64_t
uint64_t to_uint64(const epoch_duration &d);

// Function to convert uint64_t back to epoch_duration
epoch_duration from_uint64(uint64_t value);

// Convert from STM32 RTC to epoch_duration
epoch_duration from_rtc(const RTCDateTimeSubseconds &rtcdatetimesubseconds, uint32_t secondFraction);

RTCDateTimeSubseconds to_rtc(const epoch_duration &d, uint32_t secondFraction);
RTCDateTimeSubseconds to_rtc(const DateTimeComponents &components, uint32_t secondFraction);

float gsTimeJ2000(float jd2000);
float hoursToRadians(float gsm);

// Error codes (adjust as needed)
enum class TimeUtilsError {
  OK = 0,
  INVALID_YEAR,
  INVALID_MONTH,
  INVALID_DAY,
  // Add more error codes as necessary
};

}  // namespace TimeUtils

#endif  // TimeUtils_HPP