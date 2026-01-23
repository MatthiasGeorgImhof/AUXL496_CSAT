#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "TaskSGP4.hpp"
#include "SGP4PositionTracker.hpp"
#include "TaskPositionService.hpp"
#include "cyphal.hpp"

#include "TimeUtils.hpp"
#include <chrono>
#include <cstdint>
#include <cmath>
#include <limits>

#include "au.hpp"
#include "mock_hal.h"

#include "loopard_adapter.hpp"
#include "RegistrationManager.hpp"
#include "ServiceManager.hpp"
#include "HeapAllocation.hpp"

using Heap = HeapAllocation<>;

TEST_CASE("duration_in_fractional_days - Basic")
{
    TimeUtils::DateTimeComponents start = {2024, 1, 1, 0, 0, 0, 0};
    TimeUtils::DateTimeComponents end = {2024, 1, 2, 0, 0, 0, 0};
    std::chrono::system_clock::time_point tp_start = TimeUtils::to_timepoint(start);
    std::chrono::system_clock::time_point tp_end = TimeUtils::to_timepoint(end);
    CHECK(TimeUtils::to_fractional_days(tp_start, tp_end) == doctest::Approx(1.0f));

    start = {2024, 1, 1, 0, 0, 0, 0};
    end = {2024, 1, 1, 12, 0, 0, 0};
    tp_start = TimeUtils::to_timepoint(start);
    tp_end = TimeUtils::to_timepoint(end);
    CHECK(TimeUtils::to_fractional_days(tp_start, tp_end) == doctest::Approx(0.5f));

    start = {2024, 1, 1, 0, 0, 0, 0};
    end = {2024, 1, 1, 0, 30, 0, 0};
    tp_start = TimeUtils::to_timepoint(start);
    tp_end = TimeUtils::to_timepoint(end);
    CHECK(TimeUtils::to_fractional_days(tp_start, tp_end) == doctest::Approx(30.0f / (24.0f * 60.0f)));

    start = {2024, 1, 1, 0, 0, 0, 0};
    end = {2024, 1, 1, 0, 0, 30, 0};
    tp_start = TimeUtils::to_timepoint(start);
    tp_end = TimeUtils::to_timepoint(end);
    CHECK(TimeUtils::to_fractional_days(tp_start, tp_end) == doctest::Approx(30.0f / (24.0f * 3600.0f)));
}

TEST_CASE("duration_in_fractional_days - Same day")
{
    TimeUtils::DateTimeComponents start = {2024, 5, 15, 10, 30, 0, 0};
    TimeUtils::DateTimeComponents end = {2024, 5, 15, 12, 0, 0, 0};
    std::chrono::system_clock::time_point tp_start = TimeUtils::to_timepoint(start);
    std::chrono::system_clock::time_point tp_end = TimeUtils::to_timepoint(end);
    CHECK(TimeUtils::to_fractional_days(tp_start, tp_end) == doctest::Approx(1.5f / 24.0f));
}

TEST_CASE("duration_in_fractional_days - Different months")
{
    TimeUtils::DateTimeComponents start = {2024, 1, 31, 0, 0, 0, 0};
    TimeUtils::DateTimeComponents end = {2024, 2, 1, 0, 0, 0, 0};
    std::chrono::system_clock::time_point tp_start = TimeUtils::to_timepoint(start);
    std::chrono::system_clock::time_point tp_end = TimeUtils::to_timepoint(end);
    CHECK(TimeUtils::to_fractional_days(tp_start, tp_end) == doctest::Approx(1.0f));
}

TEST_CASE("duration_in_fractional_days - Different years")
{
    TimeUtils::DateTimeComponents start = {2023, 12, 31, 0, 0, 0, 0};
    TimeUtils::DateTimeComponents end = {2024, 1, 1, 0, 0, 0, 0};
    std::chrono::system_clock::time_point tp_start = TimeUtils::to_timepoint(start);
    std::chrono::system_clock::time_point tp_end = TimeUtils::to_timepoint(end);
    CHECK(TimeUtils::to_fractional_days(tp_start, tp_end) == doctest::Approx(1.0f));
}

TEST_CASE("duration_in_fractional_days - Leap year test")
{
    TimeUtils::DateTimeComponents start = {2024, 2, 28, 0, 0, 0, 0};
    TimeUtils::DateTimeComponents end = {2024, 3, 1, 0, 0, 0, 0};
    std::chrono::system_clock::time_point tp_start = TimeUtils::to_timepoint(start);
    std::chrono::system_clock::time_point tp_end = TimeUtils::to_timepoint(end);
    CHECK(TimeUtils::to_fractional_days(tp_start, tp_end) == doctest::Approx(2.0f)); // 2 days because of Feb 29
}

TEST_CASE("duration_in_fractional_days - End before start (should be negative)")
{
    TimeUtils::DateTimeComponents start = {2024, 1, 2, 0, 0, 0, 0};
    TimeUtils::DateTimeComponents end = {2024, 1, 1, 0, 0, 0, 0};
    std::chrono::system_clock::time_point tp_start = TimeUtils::to_timepoint(start);
    std::chrono::system_clock::time_point tp_end = TimeUtils::to_timepoint(end);
    CHECK(TimeUtils::to_fractional_days(tp_start, tp_end) == doctest::Approx(-1.0f));
}

TEST_CASE("duration_in_fractional_days - Large duration")
{
    TimeUtils::DateTimeComponents start = {2000, 1, 1, 0, 0, 0, 0};
    TimeUtils::DateTimeComponents end = {2050, 1, 1, 0, 0, 0, 0};
    std::chrono::system_clock::time_point tp_start = TimeUtils::to_timepoint(start);
    std::chrono::system_clock::time_point tp_end = TimeUtils::to_timepoint(end);
    CHECK(TimeUtils::to_fractional_days(tp_start, tp_end) == doctest::Approx(365.0f * 50.f + 13.f));
}

TEST_CASE("duration_in_fractional_days - Millisecond Precision")
{
    TimeUtils::DateTimeComponents start = {2024, 1, 1, 0, 0, 0, 0};
    TimeUtils::DateTimeComponents end = {2024, 1, 1, 0, 0, 0, 500};
    std::chrono::system_clock::time_point tp_start = TimeUtils::to_timepoint(start);
    std::chrono::system_clock::time_point tp_end = TimeUtils::to_timepoint(end);
    CHECK(TimeUtils::to_fractional_days(tp_start, tp_end) == doctest::Approx(500.0f / (24.0f * 3600.0f * 1000.0f)));
}

TEST_CASE("receive one TLE")
{
    RTC_HandleTypeDef hrtc;
    set_current_tick(1001);

    constexpr CyphalNodeID id = 11;

    Heap::initialize();

    // Create adapters
    LoopardAdapter loopard;
    loopard.memory_allocate = Heap::loopardMemoryAllocate;
    loopard.memory_free = Heap::loopardMemoryDeallocate;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(id);
    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);

    SGP4 sgp4(&hrtc);
    auto task = std::make_shared<TaskSGP4>(sgp4, 1000, 0);

    // Create a uavcan_time_Synchronization_1_0 message
    auto transfer = std::make_shared<CyphalTransfer>();
    _4111spyglass_sat_data_SPG4TLE_0_1 data{
        .satelliteNumber = 25544,
        .elementNumber = 999,
        .ephemerisType = 0,
        .epochYear = 25,
        .epochDay = 173.704f,
        .meanMotionDerivative1 = 0.00010306f,
        .meanMotionDerivative2 = 0.0f,
        .bStarDrag = 0.00018707f,
        .inclination = 51.6391f,
        .rightAscensionAscendingNode = 279.729f,
        .eccentricity = 0.0002026f,
        .argumentOfPerigee = 272.772f,
        .meanAnomaly = 232.5f,
        .meanMotion = 15.5019f,
        .revolutionNumberAtEpoch = 51601};

    uint8_t payload[_4111spyglass_sat_data_SPG4TLE_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t payload_size = sizeof(payload);
    _4111spyglass_sat_data_SPG4TLE_0_1_serialize_(&data, payload, &payload_size);

    transfer->payload = payload;
    transfer->payload_size = payload_size;

    SGP4TwoLineElement tle = sgp4.getSGP4TLE();
    CHECK(tle.satelliteNumber == 0);

    task->handleMessage(transfer);
    task->handleTask();

    tle = sgp4.getSGP4TLE();
    CHECK(tle.satelliteNumber == data.satelliteNumber);
    CHECK(tle.elementNumber == data.elementNumber);
    CHECK(tle.ephemerisType == data.ephemerisType);
    CHECK(tle.epochYear == data.epochYear);
    CHECK(tle.meanMotionDerivative1 == data.meanMotionDerivative1);
    CHECK(tle.meanMotionDerivative2 == data.meanMotionDerivative2);
    CHECK(tle.bStarDrag == data.bStarDrag);
    CHECK(tle.inclination == data.inclination);
    CHECK(tle.rightAscensionAscendingNode == data.rightAscensionAscendingNode);
    CHECK(tle.eccentricity == data.eccentricity);
    CHECK(tle.argumentOfPerigee == data.argumentOfPerigee);
    CHECK(tle.meanAnomaly == data.meanAnomaly);
    CHECK(tle.meanMotion == data.meanMotion);
    CHECK(tle.revolutionNumberAtEpoch == data.revolutionNumberAtEpoch);
}

TEST_CASE("receive two TLE")
{
    RTC_HandleTypeDef hrtc;
    set_current_tick(1001);

    constexpr CyphalNodeID id = 11;

    Heap::initialize();

    // Create adapters
    LoopardAdapter loopard;
    loopard.memory_allocate = Heap::loopardMemoryAllocate;
    loopard.memory_free = Heap::loopardMemoryDeallocate;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(id);
    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);

    SGP4 sgp4(&hrtc);
    auto task = std::make_shared<TaskSGP4>(sgp4, 1000, 0);

    // Create a uavcan_time_Synchronization_1_0 message
    auto transfer = std::make_shared<CyphalTransfer>();
    _4111spyglass_sat_data_SPG4TLE_0_1 data{
        .satelliteNumber = 25544,
        .elementNumber = 999,
        .ephemerisType = 0,
        .epochYear = 25,
        .epochDay = 173.704f,
        .meanMotionDerivative1 = 0.00010306f,
        .meanMotionDerivative2 = 0.0f,
        .bStarDrag = 0.00018707f,
        .inclination = 51.6391f,
        .rightAscensionAscendingNode = 279.729f,
        .eccentricity = 0.0002026f,
        .argumentOfPerigee = 272.772f,
        .meanAnomaly = 232.5f,
        .meanMotion = 15.5019f,
        .revolutionNumberAtEpoch = 51601};

    uint8_t payload[_4111spyglass_sat_data_SPG4TLE_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t payload_size = sizeof(payload);
    _4111spyglass_sat_data_SPG4TLE_0_1_serialize_(&data, payload, &payload_size);

    transfer->payload = payload;
    transfer->payload_size = payload_size;

    SGP4TwoLineElement tle = sgp4.getSGP4TLE();
    CHECK(tle.satelliteNumber == 0);

    task->handleMessage(transfer);

    data = _4111spyglass_sat_data_SPG4TLE_0_1{
        .satelliteNumber = 99999,
        .elementNumber = 999,
        .ephemerisType = 0,
        .epochYear = 25,
        .epochDay = 173.704f,
        .meanMotionDerivative1 = 0.00010306f,
        .meanMotionDerivative2 = 0.0f,
        .bStarDrag = 0.00018707f,
        .inclination = 51.6391f,
        .rightAscensionAscendingNode = 279.729f,
        .eccentricity = 0.0002026f,
        .argumentOfPerigee = 272.772f,
        .meanAnomaly = 232.5f,
        .meanMotion = 15.5019f,
        .revolutionNumberAtEpoch = 77777};

    _4111spyglass_sat_data_SPG4TLE_0_1_serialize_(&data, payload, &payload_size);

    transfer->payload = payload;
    transfer->payload_size = payload_size;

    task->handleMessage(transfer);
    task->handleTask();

    tle = sgp4.getSGP4TLE();
    CHECK(tle.satelliteNumber == 99999);
    CHECK(tle.elementNumber == data.elementNumber);
    CHECK(tle.ephemerisType == data.ephemerisType);
    CHECK(tle.epochYear == data.epochYear);
    CHECK(tle.meanMotionDerivative1 == data.meanMotionDerivative1);
    CHECK(tle.meanMotionDerivative2 == data.meanMotionDerivative2);
    CHECK(tle.bStarDrag == data.bStarDrag);
    CHECK(tle.inclination == data.inclination);
    CHECK(tle.rightAscensionAscendingNode == data.rightAscensionAscendingNode);
    CHECK(tle.eccentricity == data.eccentricity);
    CHECK(tle.argumentOfPerigee == data.argumentOfPerigee);
    CHECK(tle.meanAnomaly == data.meanAnomaly);
    CHECK(tle.meanMotion == data.meanMotion);
    CHECK(tle.revolutionNumberAtEpoch == data.revolutionNumberAtEpoch);
}

TEST_CASE("send position 2025 6 25 18 0 0")
{
    RTC_HandleTypeDef hrtc;
    hrtc.Init.SynchPrediv = 1023;
    set_current_tick(1001);

    TimeUtils::DateTimeComponents components{
        .year = 2025,
        .month = 6,
        .day = 25,
        .hour = 18,
        .minute = 0,
        .second = 0,
        .millisecond = 0};
    TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(components, hrtc.Init.SynchPrediv);
    HAL_RTC_SetTime(&hrtc, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &rtc.date, RTC_FORMAT_BIN);
    HAL_RTCEx_SetSynchroShift(&hrtc, RTC_SHIFTADD1S_RESET, rtc.time.SubSeconds);

    constexpr CyphalNodeID id = 11;

    Heap::initialize();

    // Create adapters
    LoopardAdapter loopard;
    loopard.memory_allocate = Heap::loopardMemoryAllocate;
    loopard.memory_free = Heap::loopardMemoryDeallocate;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(id);
    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);
        
    SGP4 sgp4(&hrtc);
    SGP4Position<SGP4> sgp4_position(&hrtc, sgp4);
    auto task = std::make_shared<TaskPositionService<SGP4Position<SGP4>, Cyphal<LoopardAdapter>>>(sgp4_position, 1000, 0, 0, adapters);

    // ISS (ZARYA)
    char longstr1[] = "1 25544U 98067A   25176.73245655  .00008102  00000-0  14854-3 0  9994";
    char longstr2[] = "2 25544  51.6390 264.7180 0001990 278.3788 217.2311 15.50240116516482";  

    auto parsed = sgp4_utils::parseTLE(longstr1, longstr2);
    REQUIRE(parsed.has_value());
    SGP4TwoLineElement data = parsed.value();
    sgp4.setSGP4TLE(data);
    
    SGP4TwoLineElement tle = sgp4.getSGP4TLE();
    CHECK(tle.satelliteNumber == data.satelliteNumber);
    CHECK(tle.elementNumber == data.elementNumber);
    CHECK(tle.ephemerisType == data.ephemerisType);
    CHECK(tle.epochYear == data.epochYear);
    CHECK(tle.meanMotionDerivative1 == data.meanMotionDerivative1);
    CHECK(tle.meanMotionDerivative2 == data.meanMotionDerivative2);
    CHECK(tle.bStarDrag == data.bStarDrag);
    CHECK(tle.inclination == data.inclination);
    CHECK(tle.rightAscensionAscendingNode == data.rightAscensionAscendingNode);
    CHECK(tle.eccentricity == data.eccentricity);
    CHECK(tle.argumentOfPerigee == data.argumentOfPerigee);
    CHECK(tle.meanAnomaly == data.meanAnomaly);
    CHECK(tle.meanMotion == data.meanMotion);
    CHECK(tle.revolutionNumberAtEpoch == data.revolutionNumberAtEpoch);

    CHECK(loopard.buffer.is_empty());
    task->handleTaskImpl();
    REQUIRE(loopard.buffer.size() == 1);

    auto transfer = loopard.buffer.pop();
    CHECK(transfer.metadata.port_id == _4111spyglass_sat_solution_PositionSolution_0_1_PORT_ID_);
    CHECK(transfer.metadata.transfer_kind == CyphalTransferKindMessage);
    CHECK(transfer.metadata.remote_node_id == id);
    CHECK(transfer.payload_size == _4111spyglass_sat_solution_PositionSolution_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);

    // Deserialize the payload and verify the values
    _4111spyglass_sat_solution_PositionSolution_0_1 received_data;
    size_t deserialized_size = transfer.payload_size;
    int8_t deserialization_result = _4111spyglass_sat_solution_PositionSolution_0_1_deserialize_(&received_data, static_cast<const uint8_t*>(transfer.payload), &deserialized_size);
    REQUIRE(deserialization_result >= 0);
    CHECK(received_data.timestamp.microsecond == 804189600000000);

    std::array<float,3> expected_r {2715.4f, -4518.34f, -4291.31f};
    std::array<float,3> expected_v {3.75928f, 5.63901f, -3.55967f};
    
    CHECK(received_data.position_ecef.meter[0] == doctest::Approx(expected_r[0]*1000.0f).epsilon(0.01));
    CHECK(received_data.position_ecef.meter[1] == doctest::Approx(expected_r[1]*1000.0f).epsilon(0.01));
    CHECK(received_data.position_ecef.meter[2] == doctest::Approx(expected_r[2]*1000.0f).epsilon(0.01));
    CHECK(received_data.velocity_ecef.meter_per_second[0] == doctest::Approx(expected_v[0]*1000.0f).epsilon(0.01));
    CHECK(received_data.velocity_ecef.meter_per_second[1] == doctest::Approx(expected_v[1]*1000.0f).epsilon(0.01));
    CHECK(received_data.velocity_ecef.meter_per_second[2] == doctest::Approx(expected_v[2]*1000.0f).epsilon(0.01));
    Heap::loopardMemoryDeallocate(transfer.payload);
}

TEST_CASE("send position 2025 7 6 20 43 13")
{
    RTC_HandleTypeDef hrtc;
    hrtc.Init.SynchPrediv = 1023;
    set_current_tick(1001);

    TimeUtils::DateTimeComponents components{
        .year = 2025,
        .month = 7,
        .day = 6,
        .hour = 20,
        .minute = 43,
        .second = 13,
        .millisecond = 0};
    TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(components, hrtc.Init.SynchPrediv);
    HAL_RTC_SetTime(&hrtc, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &rtc.date, RTC_FORMAT_BIN);
    HAL_RTCEx_SetSynchroShift(&hrtc, RTC_SHIFTADD1S_RESET, rtc.time.SubSeconds);

    constexpr CyphalNodeID id = 11;

    Heap::initialize();

    // Create adapters
    LoopardAdapter loopard;
    loopard.memory_allocate = Heap::loopardMemoryAllocate;
    loopard.memory_free = Heap::loopardMemoryDeallocate;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(id);
    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);

    SGP4 sgp4(&hrtc);
    SGP4Position<SGP4> sgp4_position(&hrtc, sgp4);
    auto task = std::make_shared<TaskPositionService<SGP4Position<SGP4>, Cyphal<LoopardAdapter>>>(sgp4_position, 1000, 0, 0, adapters);

    // ISS (ZARYA)
    char longstr1[] = "1 25544U 98067A   25176.73245655  .00008102  00000-0  14854-3 0  9994";
    char longstr2[] = "2 25544  51.6390 264.7180 0001990 278.3788 217.2311 15.50240116516482";  

    auto parsed = sgp4_utils::parseTLE(longstr1, longstr2);
    REQUIRE(parsed.has_value());
    SGP4TwoLineElement data = parsed.value();
    sgp4.setSGP4TLE(data);
    
    SGP4TwoLineElement tle = sgp4.getSGP4TLE();
    CHECK(tle.satelliteNumber == data.satelliteNumber);
    CHECK(tle.elementNumber == data.elementNumber);
    CHECK(tle.ephemerisType == data.ephemerisType);
    CHECK(tle.epochYear == data.epochYear);
    CHECK(tle.meanMotionDerivative1 == data.meanMotionDerivative1);
    CHECK(tle.meanMotionDerivative2 == data.meanMotionDerivative2);
    CHECK(tle.bStarDrag == data.bStarDrag);
    CHECK(tle.inclination == data.inclination);
    CHECK(tle.rightAscensionAscendingNode == data.rightAscensionAscendingNode);
    CHECK(tle.eccentricity == data.eccentricity);
    CHECK(tle.argumentOfPerigee == data.argumentOfPerigee);
    CHECK(tle.meanAnomaly == data.meanAnomaly);
    CHECK(tle.meanMotion == data.meanMotion);
    CHECK(tle.revolutionNumberAtEpoch == data.revolutionNumberAtEpoch);

    CHECK(loopard.buffer.is_empty());
    task->handleTaskImpl();
    REQUIRE(loopard.buffer.size() == 1);

    auto transfer = loopard.buffer.pop();
    CHECK(transfer.metadata.port_id == _4111spyglass_sat_solution_PositionSolution_0_1_PORT_ID_);
    CHECK(transfer.metadata.transfer_kind == CyphalTransferKindMessage);
    CHECK(transfer.metadata.remote_node_id == id);
    CHECK(transfer.payload_size == _4111spyglass_sat_solution_PositionSolution_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);

    // Deserialize the payload and verify the values
    _4111spyglass_sat_solution_PositionSolution_0_1 received_data;
    size_t deserialized_size = transfer.payload_size;
    int8_t deserialization_result = _4111spyglass_sat_solution_PositionSolution_0_1_deserialize_(&received_data, static_cast<const uint8_t*>(transfer.payload), &deserialized_size);
    REQUIRE(deserialization_result >= 0);
    CHECK(received_data.timestamp.microsecond == 805149793000000);

    std::array<float,3> expected_r {6356.42f, -1504.07f, 1859.27f};
    std::array<float,3> expected_v {-0.42784f, 5.18216f, 5.63173f};
    
    CHECK(received_data.position_ecef.meter[0] == doctest::Approx(expected_r[0] * 1000.f).epsilon(0.01));
    CHECK(received_data.position_ecef.meter[1] == doctest::Approx(expected_r[1] * 1000.f).epsilon(0.01));
    CHECK(received_data.position_ecef.meter[2] == doctest::Approx(expected_r[2] * 1000.f).epsilon(0.01));
    CHECK(received_data.velocity_ecef.meter_per_second[0] == doctest::Approx(expected_v[0] * 1000.f).epsilon(0.01));
    CHECK(received_data.velocity_ecef.meter_per_second[1] == doctest::Approx(expected_v[1] * 1000.f).epsilon(0.01));
    CHECK(received_data.velocity_ecef.meter_per_second[2] == doctest::Approx(expected_v[2] * 1000.f).epsilon(0.01));
    Heap::loopardMemoryDeallocate(transfer.payload);
}
