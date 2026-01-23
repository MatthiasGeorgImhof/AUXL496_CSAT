#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "TaskSendTimeSynchronization.hpp"
#include "Task.hpp"
#include "cyphal.hpp"
#include <tuple>
#include <memory>
#include "uavcan/time/Synchronization_1_0.h"
#include "loopard_adapter.hpp"
#include "RegistrationManager.hpp"
#include "ServiceManager.hpp"
#include "TimeUtils.hpp"
#include "HeapAllocation.hpp"

#include "mock_hal.h"  // Include the mock HAL header

using Heap = HeapAllocation<>;

TEST_CASE("TaskSendTimeSynchronization: handleTask publishes TimeSynchronization") {
    // Set up mock environment for x86_64
    RTC_HandleTypeDef hrtc_ {};
    hrtc_.Init.SynchPrediv = 1023;
    RTC_HandleTypeDef &hrtc = hrtc_;

    // Initialize the mock RTC time and date
    TimeUtils::RTCDateTimeSubseconds rtcdatetimesubseconds = {
        .date = {
            .WeekDay = RTC_WEEKDAY_THURSDAY,
            .Month = 10,
            .Date = 26,
            .Year = 23
        },
        .time = {
            .Hours = 10,
            .Minutes = 30,
            .Seconds = 0,
            .TimeFormat = RTC_FORMAT_BIN,
            .SubSeconds = 500,
            .SecondFraction = hrtc_.Init.SynchPrediv,
            .DayLightSaving = RTC_DAYLIGHTSAVING_NONE,
            .StoreOperation = RTC_STOREOPERATION_RESET
        }
    };
    set_mocked_rtc_time(rtcdatetimesubseconds.time);
    set_mocked_rtc_date(rtcdatetimesubseconds.date);
    set_mocked_synchro_shift_subfs(rtcdatetimesubseconds.time.SubSeconds);
    TimeUtils::epoch_duration expected_duration = TimeUtils::from_rtc(rtcdatetimesubseconds, hrtc_.Init.SynchPrediv);
    
    constexpr CyphalNodeID id1 = 11;

    Heap::initialize();

    // Create adapters
    LoopardAdapter loopard;
    loopard.memory_allocate = Heap::loopardMemoryAllocate;
    loopard.memory_free = Heap::loopardMemoryDeallocate;
    Cyphal<LoopardAdapter> loopard_cyphal1(&loopard);
    loopard_cyphal1.setNodeID(id1);
    
    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal1);

    // Instantiate the TaskSendTimeSynchronization
    TaskSendTimeSynchronization task(&hrtc, 1000, 0, 0, adapters);

    // Execute the task
    task.handleTaskImpl();

    // Check that a TimeSynchronization message was published on both adapters
    REQUIRE(loopard.buffer.size() == 1);

    // Verify the content of the published message on the first adapter
    CyphalTransfer transfer1 = loopard.buffer.pop();
    REQUIRE(transfer1.metadata.port_id == uavcan_time_Synchronization_1_0_FIXED_PORT_ID_);
    REQUIRE(transfer1.metadata.transfer_kind == CyphalTransferKindMessage);
    REQUIRE(transfer1.metadata.remote_node_id == id1);
    REQUIRE(transfer1.payload_size == uavcan_time_Synchronization_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_);

    // Deserialize the payload and verify the values
    uavcan_time_Synchronization_1_0 received_time_sync1;
    size_t deserialized_size1 = transfer1.payload_size;
    int8_t deserialization_result1 = uavcan_time_Synchronization_1_0_deserialize_(&received_time_sync1, static_cast<const uint8_t*>(transfer1.payload), &deserialized_size1);
    REQUIRE(deserialization_result1 >= 0); // Check if deserialization was successful

    REQUIRE(received_time_sync1.previous_transmission_timestamp_microsecond == 0);
    Heap::loopardMemoryDeallocate(static_cast<uint8_t*>(transfer1.payload));

    REQUIRE(loopard.buffer.size() == 0);
    task.handleTaskImpl();
    REQUIRE(loopard.buffer.size() == 1);

    // Verify the content of the published message on the first adapter
    CyphalTransfer transfer2 = loopard.buffer.pop();
    REQUIRE(transfer2.metadata.port_id == uavcan_time_Synchronization_1_0_FIXED_PORT_ID_);
    REQUIRE(transfer2.metadata.transfer_kind == CyphalTransferKindMessage);
    REQUIRE(transfer2.metadata.remote_node_id == id1);
    REQUIRE(transfer2.payload_size == uavcan_time_Synchronization_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_);

    // Deserialize the payload and verify the values
    uavcan_time_Synchronization_1_0 received_time_sync2;
    size_t deserialized_size2 = transfer2.payload_size;
    int8_t deserialization_result2 = uavcan_time_Synchronization_1_0_deserialize_(&received_time_sync2, static_cast<const uint8_t*>(transfer2.payload), &deserialized_size2);
    REQUIRE(deserialization_result2 >= 0); // Check if deserialization was successful

    REQUIRE(received_time_sync2.previous_transmission_timestamp_microsecond == expected_duration.count() * 1000);
    Heap::loopardMemoryDeallocate(static_cast<uint8_t*>(transfer2.payload));

    clear_mocked_rtc();
}


TEST_CASE("TaskSendTimeSynchronization: snippet to registration with std::alloc") {
    // Set up mock environment for x86_64
    RTC_HandleTypeDef hrtc_ {};
    hrtc_.Init.SynchPrediv = 1023;
    RTC_HandleTypeDef &hrtc = hrtc_;

    // Initialize the mock RTC time and date
    RTC_TimeTypeDef time = {
        .Hours = 10,
        .Minutes = 30,
        .Seconds = 0,
        .TimeFormat = RTC_FORMAT_BIN,
        .SubSeconds = 500,
        .SecondFraction = 1023,
        .DayLightSaving = RTC_DAYLIGHTSAVING_NONE,
        .StoreOperation = RTC_STOREOPERATION_RESET
    };
    RTC_DateTypeDef date = {
        .WeekDay = RTC_WEEKDAY_THURSDAY,
        .Month = 10,
        .Date = 26,
        .Year = 23
    };

    set_mocked_rtc_time(time);
    set_mocked_rtc_date(date);
    set_mocked_synchro_shift_subfs(0x7F);

    constexpr CyphalNodeID id1 = 11;
    constexpr CyphalNodeID id2 = 12;

    Heap::initialize();

    LoopardAdapter loopard1, loopard2;
    Cyphal<LoopardAdapter> loopard_cyphal1(&loopard1), loopard_cyphal2(&loopard2);
    loopard1.memory_allocate = Heap::loopardMemoryAllocate;
    loopard1.memory_free = Heap::loopardMemoryDeallocate;
    loopard2.memory_allocate = Heap::loopardMemoryAllocate;
    loopard2.memory_free = Heap::loopardMemoryDeallocate;
    loopard_cyphal1.setNodeID(id1);
    loopard_cyphal2.setNodeID(id2);
    
    std::tuple<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>> adapters(loopard_cyphal1, loopard_cyphal2);
    auto task_sendtimesync = std::make_shared<TaskSendTimeSynchronization<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>>>(&hrtc, 1000, 0, 0, adapters);
    CHECK(task_sendtimesync.use_count() == 1);

    RegistrationManager registration_manager;
    registration_manager.add(task_sendtimesync);
    CHECK(task_sendtimesync.use_count() == 2);

    CHECK(registration_manager.containsTask(task_sendtimesync));

    registration_manager.remove(task_sendtimesync);
    CHECK(! registration_manager.containsTask(task_sendtimesync));
    CHECK(task_sendtimesync.use_count() == 1);

    clear_mocked_rtc();
}


TEST_CASE("TaskSendTimeSynchronization: snippet to registration with O1HeapAllocator") {
    // Set up mock environment for x86_64
    RTC_HandleTypeDef hrtc_ {};
    hrtc_.Init.SynchPrediv = 1023;
    RTC_HandleTypeDef &hrtc = hrtc_;

    // Initialize the mock RTC time and date
    RTC_TimeTypeDef time = {
        .Hours = 10,
        .Minutes = 30,
        .Seconds = 0,
        .TimeFormat = RTC_FORMAT_BIN,
        .SubSeconds = 500,
        .SecondFraction = 1023,
        .DayLightSaving = RTC_DAYLIGHTSAVING_NONE,
        .StoreOperation = RTC_STOREOPERATION_RESET
    };
    RTC_DateTypeDef date = {
        .WeekDay = RTC_WEEKDAY_THURSDAY,
        .Month = 10,
        .Date = 26,
        .Year = 23
    };

    set_mocked_rtc_time(time);
    set_mocked_rtc_date(date);
    set_mocked_synchro_shift_subfs(0x7F);

    constexpr CyphalNodeID id1 = 11;
    constexpr CyphalNodeID id2 = 12;

    Heap::initialize();
    auto diagnostics = Heap::getDiagnostics();
    size_t allocated = diagnostics.allocated;

    SafeAllocator<TaskSendTimeSynchronization<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>>, Heap> taskAllocator;

    LoopardAdapter loopard1, loopard2;
    Cyphal<LoopardAdapter> loopard_cyphal1(&loopard1), loopard_cyphal2(&loopard2);
    loopard_cyphal1.setNodeID(id1);
    loopard_cyphal2.setNodeID(id2);

    std::tuple<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>> adapters(loopard_cyphal1, loopard_cyphal2);

    auto task_sendtimesync = alloc_shared_custom<TaskSendTimeSynchronization<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>>>(taskAllocator, &hrtc, 1000, 0, 0, adapters);
    diagnostics = Heap::getDiagnostics();
    CHECK(diagnostics.allocated > allocated);
    CHECK(task_sendtimesync.use_count() == 1);

    RegistrationManager registration_manager;
    registration_manager.add(task_sendtimesync);
    CHECK(registration_manager.containsTask(task_sendtimesync));
    CHECK(task_sendtimesync.use_count() == 2);

    registration_manager.remove(task_sendtimesync);
    CHECK(! registration_manager.containsTask(task_sendtimesync));
    
    CHECK(task_sendtimesync.use_count() == 1);
    task_sendtimesync.reset();
    diagnostics = Heap::getDiagnostics();
    CHECK(diagnostics.allocated == allocated);

    clear_mocked_rtc();
}