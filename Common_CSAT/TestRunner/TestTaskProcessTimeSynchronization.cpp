#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "TaskProcessTimeSynchronization.hpp"
#include "mock_hal.h"
#include <memory>
#include "uavcan/time/Synchronization_1_0.h"

TEST_CASE("TaskProcessTimeSynchronization - handleMessage") {
    // Mock HAL RTC
    const uint64_t base_millisecond = 1698429000l;

    RTC_HandleTypeDef hrtc;
    RTC_InitTypeDef init;
    init.SynchPrediv = 1023;
    hrtc.Init = init;

    // Initialize RTC with known time
    RTC_TimeTypeDef initial_time;
    initial_time.Hours = 12;
    initial_time.Minutes = 30;
    initial_time.Seconds = 0;
    initial_time.SubSeconds = 0;
    set_mocked_rtc_time(initial_time);

    RTC_DateTypeDef initial_date;
    initial_date.Year = 23; // Year 2023
    initial_date.Month = 10;
    initial_date.Date = 27;
    set_mocked_rtc_date(initial_date);

    // Create the TaskProcessTimeSynchronization instance
    auto task = std::make_shared<TaskProcessTimeSynchronization>(&hrtc, 1000, 0);

    // Create a uavcan_time_Synchronization_1_0 message
    auto transfer1 = std::make_shared<CyphalTransfer>();
    uavcan_time_Synchronization_1_0 time_sync_msg1;
    time_sync_msg1.previous_transmission_timestamp_microsecond = base_millisecond * 1000l;

    // Serialize the UAVCAN message into a dynamically allocated payload buffer
    uint8_t payload1[uavcan_time_Synchronization_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t payload_size1 = sizeof(payload1);
    uavcan_time_Synchronization_1_0_serialize_(&time_sync_msg1, payload1, &payload_size1);

    transfer1->payload = payload1;
    transfer1->payload_size = payload_size1;

    // Get the mocked RTC time and date after the first synchronization
    set_current_tick(1000);
    task->handleMessage(transfer1);
    RTC_TimeTypeDef synced_time1 = get_mocked_rtc_time();
    RTC_DateTypeDef synced_date1 = get_mocked_rtc_date();

    CHECK(synced_time1.Hours == initial_time.Hours);
    CHECK(synced_time1.Minutes == initial_time.Minutes);
    CHECK(synced_time1.Seconds == initial_time.Seconds);
    CHECK(synced_time1.SubSeconds == initial_time.SubSeconds);
    CHECK(synced_date1.Year == initial_date.Year);
    CHECK(synced_date1.Month == initial_date.Month);
    CHECK(synced_date1.Date == initial_date.Date); 

    // Create a uavcan_time_Synchronization_1_0 message
    auto transfer2 = std::make_shared<CyphalTransfer>();
    uavcan_time_Synchronization_1_0 time_sync_msg2;
    time_sync_msg2.previous_transmission_timestamp_microsecond = (base_millisecond + 6000l) * 1000l;

    // Serialize the UAVCAN message into a dynamically allocated payload buffer
    uint8_t payload2[uavcan_time_Synchronization_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t payload_size2 = sizeof(payload2);
    uavcan_time_Synchronization_1_0_serialize_(&time_sync_msg2, payload2, &payload_size2);

    transfer2->payload = payload2;
    transfer2->payload_size = payload_size2;

    // Get the mocked RTC time and date after the first synchronization
    set_current_tick(2000);
    task->handleMessage(transfer2);
    RTC_TimeTypeDef synced_time2 = get_mocked_rtc_time();
    RTC_DateTypeDef synced_date2 = get_mocked_rtc_date();

    // Initialize RTC with known time
    RTC_TimeTypeDef expected_time2;
    expected_time2.Hours = 15;
    expected_time2.Minutes = 47;
    expected_time2.Seconds = 16;
    expected_time2.SubSeconds = 1023;

    RTC_DateTypeDef expected_date2;
    expected_date2.Year = 0;
    expected_date2.Month = 1;
    expected_date2.Date = 20;

    CHECK(synced_time2.Hours == expected_time2.Hours);
    CHECK(synced_time2.Minutes == expected_time2.Minutes);
    CHECK(synced_time2.Seconds == expected_time2.Seconds);
    CHECK(synced_time2.SubSeconds == expected_time2.SubSeconds);
    CHECK(synced_date2.Year == expected_date2.Year);
    CHECK(synced_date2.Month == expected_date2.Month);
    CHECK(synced_date2.Date == expected_date2.Date); 

    // Create a uavcan_time_Synchronization_1_0 message
    auto transfer3 = std::make_shared<CyphalTransfer>();
    uavcan_time_Synchronization_1_0 time_sync_msg3;
    time_sync_msg3.previous_transmission_timestamp_microsecond = (base_millisecond + 11000l) * 1000l;

    // Serialize the UAVCAN message into a dynamically allocated payload buffer
    uint8_t payload3[uavcan_time_Synchronization_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t payload_size3 = sizeof(payload3);
    uavcan_time_Synchronization_1_0_serialize_(&time_sync_msg3, payload3, &payload_size3);

    transfer3->payload = payload3;
    transfer3->payload_size = payload_size3;

    // Get the mocked RTC time and date after the first synchronization
    set_current_tick(3000);
    task->handleMessage(transfer3);
    RTC_TimeTypeDef synced_time3 = get_mocked_rtc_time();
    RTC_DateTypeDef synced_date3 = get_mocked_rtc_date();

    // Initialize RTC with known time
    RTC_TimeTypeDef expected_time3;
    expected_time3.Hours = 15;
    expected_time3.Minutes = 47;
    expected_time3.Seconds = 21;
    expected_time3.SubSeconds = 1023;

    RTC_DateTypeDef expected_date3;
    expected_date3.Year = 0;
    expected_date3.Month = 1;
    expected_date3.Date = 20;

    CHECK(synced_time3.Hours == expected_time3.Hours);
    CHECK(synced_time3.Minutes == expected_time3.Minutes);
    CHECK(synced_time3.Seconds == expected_time3.Seconds);
    CHECK(synced_time3.SubSeconds == expected_time3.SubSeconds);
    CHECK(synced_date3.Year == expected_date3.Year);
    CHECK(synced_date3.Month == expected_date3.Month);
    CHECK(synced_date3.Date == expected_date3.Date); 

    clear_mocked_rtc();
}