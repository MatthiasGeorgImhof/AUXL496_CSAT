#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "TaskPositionService.hpp"
#include "PositionTracker9D.hpp"
#include "SGP4PositionTracker.hpp"
#include "TaskSGP4.hpp"

#include "GNSS.hpp"
#include "IMU.hpp"
#include "mock_hal.h"
#include "TimeUtils.hpp"
#include "RegistrationManager.hpp"
#include "loopard_adapter.hpp"
#include <array>
#include <tuple>

#include "au.hpp"

// Mock GNSS class
class MockGNSS
{
public:
    MockGNSS() : has_data(false) {}

    void setPosition(int32_t x, int32_t y, int32_t z)
    {
        position.ecefX = x;
        position.ecefY = y;
        position.ecefZ = z;
        has_data = true;
    }

    std::optional<PositionECEF> GetNavPosECEF()
    {
        if (has_data)
        {
            return position;
        }
        else
        {
            return std::nullopt;
        }
    }

private:
    PositionECEF position;
    bool has_data;
};

// Mock IMU class
class MockIMUinBodyFrame
{
public:
    MockIMUinBodyFrame() : has_data(false) {}

    void setAcceleration(float x, float y, float z)
    {
        acceleration = { au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(x),
                         au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(y),
                         au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(z)};
        has_data = true;
    }

    std::optional<AccelerationInBodyFrame> readAccelerometer()
    {
        if (has_data)
        {
            return acceleration;
        }
        else
        {
            return std::nullopt;
        }
    }

private:
    AccelerationInBodyFrame acceleration;
    bool has_data;
};
static_assert(HasBodyAccelerometer<MockIMUinBodyFrame>);

template <typename IMU>
class MockIMUwithoutReorientation
{
public:
    MockIMUwithoutReorientation(IMU &imu) : imu_(imu) {}

    void setAcceleration(float x, float y, float z)
    {
        imu_.setAcceleration(x, y, z);
    }

    std::optional<std::array<au::QuantityF<au::MetersPerSecondSquaredInEcefFrame>, 3>> readAccelerometer()
    {
        auto optional_accel = imu_.readAccelerometer();
        if (optional_accel.has_value())
        {
            auto accel = optional_accel.value();
            return std::array<au::QuantityF<au::MetersPerSecondSquaredInEcefFrame>, 3>{
                au::make_quantity<au::MetersPerSecondSquaredInEcefFrame>(accel[0].in(au::metersPerSecondSquaredInBodyFrame)),
                au::make_quantity<au::MetersPerSecondSquaredInEcefFrame>(accel[1].in(au::metersPerSecondSquaredInBodyFrame)),
                au::make_quantity<au::MetersPerSecondSquaredInEcefFrame>(accel[2].in(au::metersPerSecondSquaredInBodyFrame))};
        }
        else
        {
            return std::nullopt;
        }
    }

private:
    IMU &imu_;
};

class MockSGP4
{
public:
    MockSGP4() : has_data(false) {}

    void setPosition(float x, float y, float z)
    {
        position[0] = au::make_quantity<au::MetersInEcefFrame>(x);
        position[1] = au::make_quantity<au::MetersInEcefFrame>(y);
        position[2] = au::make_quantity<au::MetersInEcefFrame>(z);
        has_data = true;
    }

    void setVelocity(float x, float y, float z)
    {
        velocity[0] = au::make_quantity<au::MetersPerSecondInEcefFrame>(x);
        velocity[1] = au::make_quantity<au::MetersPerSecondInEcefFrame>(y);
        velocity[2] = au::make_quantity<au::MetersPerSecondInEcefFrame>(z);
    }

    bool predict(std::array<au::QuantityF<au::MetersInEcefFrame>, 3> &r, std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> &v, au::QuantityU64<au::Milli<au::Seconds>> timestamp)
    {
        (void)timestamp;
        if (has_data)
        {
            r[0] = position[0];
            r[1] = position[1];
            r[2] = position[2];
            v[0] = velocity[0];
            v[1] = velocity[1];
            v[2] = velocity[2];
            return true;
        }
        else
        {
            return false;
        }
    }

private:
    std::array<au::QuantityF<au::MetersInEcefFrame>, 3> position;
    std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> velocity;
    bool has_data;
};

class MockOrientation
{
public:
    MockOrientation() = default;

    bool predict(std::array<float, 4> &q, au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
    {
        (void)timestamp;
        q = {1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z for identity rotation
        return true;
    }
};

void *loopardMemoryAllocate(size_t amount) { return static_cast<void *>(malloc(amount)); };
void loopardMemoryFree(void *pointer) { free(pointer); };

TEST_CASE("TaskPositionService Test with GNSSandAccelPosition")
{
    // Arrange
    RTC_HandleTypeDef hrtc;
    const uint32_t secondFraction = 1023;
    hrtc.Init.SynchPrediv = secondFraction;

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

    constexpr CyphalNodeID id = 11;
    LoopardAdapter loopard;
    loopard.memory_allocate = loopardMemoryAllocate;
    loopard.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(id);
    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);

    MockGNSS gnss;
    MockIMUinBodyFrame imu_;
    MockIMUwithoutReorientation imu(imu_);
    PositionTracker9D position_tracker;
    GNSSandAccelPosition<PositionTracker9D, MockGNSS, MockIMUwithoutReorientation<MockIMUinBodyFrame>> positionTracker(&hrtc, position_tracker, gnss, imu);
    auto task = TaskPositionService<GNSSandAccelPosition<PositionTracker9D, MockGNSS, MockIMUwithoutReorientation<MockIMUinBodyFrame>>, Cyphal<LoopardAdapter>>(positionTracker, 100, 1, 123, adapters);

    const float x0 = 100.0f;
    const float y0 = 200.0f;
    const float z0 = 300.0f;

    const float vx0 = 10.0f;
    const float vy0 = 20.0f;
    const float vz0 = 30.0f;

    const float ax0 = 0.1f;
    const float ay0 = 0.2f;
    const float az0 = 0.3f;

    const float dt = 0.1f; // 100 ms
    std::chrono::milliseconds dduration(100);

    for (int i = 0; i < 100; ++i)
    {
        float t = dt * static_cast<float>(i);

        float x = x0 + vx0 * t + 0.5f * ax0 * t * t;
        float y = y0 + vy0 * t + 0.5f * ay0 * t * t;
        float z = z0 + vz0 * t + 0.5f * az0 * t * t;

        // GNSS returns cm!
        gnss.setPosition(static_cast<int32_t>(x * 1e2f), static_cast<int32_t>(y * 1e2f), static_cast<int32_t>(z * 1e2f));
        imu.setAcceleration(ax0, ay0, az0);
        task.handleTaskImpl();
        REQUIRE(loopard.buffer.size() == 1);

        auto transfer = loopard.buffer.pop();
        CHECK(transfer.metadata.port_id == _4111spyglass_sat_model_PositionVelocity_0_1_PORT_ID_);
        CHECK(transfer.metadata.transfer_kind == CyphalTransferKindMessage);
        CHECK(transfer.metadata.remote_node_id == id);
        CHECK(transfer.payload_size == _4111spyglass_sat_model_PositionVelocity_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);

        _4111spyglass_sat_model_PositionVelocity_0_1 received_data;
        size_t deserialized_size = transfer.payload_size;
        int8_t deserialization_result = _4111spyglass_sat_model_PositionVelocity_0_1_deserialize_(&received_data, static_cast<const uint8_t *>(transfer.payload), &deserialized_size);
        REQUIRE(deserialization_result >= 0);

        float vx = vx0 + ax0 * t;
        float vy = vy0 + ay0 * t;
        float vz = vz0 + az0 * t;

        if (i > 50)
        {
            CHECK(received_data.timestamp.microsecond == duration.count() * 1000);
            CHECK(received_data.position_m[0] == doctest::Approx(x).epsilon(0.1f));
            CHECK(received_data.position_m[1] == doctest::Approx(y).epsilon(0.1f));
            CHECK(received_data.position_m[2] == doctest::Approx(z).epsilon(0.1f));
            CHECK(received_data.velocity_ms[0] == doctest::Approx(vx).epsilon(0.1f));
            CHECK(received_data.velocity_ms[1] == doctest::Approx(vy).epsilon(0.1f));
            CHECK(received_data.velocity_ms[2] == doctest::Approx(vz).epsilon(0.1f));
        }
        duration += dduration;
        auto rtc = TimeUtils::to_rtc(duration, secondFraction);
        set_mocked_rtc_time(rtc.time);
        set_mocked_rtc_date(rtc.date);
    }
}

TEST_CASE("TaskPositionService Test with GNSSandAccelPosition: noisy measurements")
{
    // Arrange
    RTC_HandleTypeDef hrtc;
    const uint32_t secondFraction = 1023;
    hrtc.Init.SynchPrediv = secondFraction;

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

    constexpr CyphalNodeID id = 11;
    LoopardAdapter loopard;
    loopard.memory_allocate = loopardMemoryAllocate;
    loopard.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(id);
    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);

    MockGNSS gnss;
    MockIMUinBodyFrame imu_;
    MockIMUwithoutReorientation<MockIMUinBodyFrame> imu(imu_);
    PositionTracker9D position_tracker;
    GNSSandAccelPosition<PositionTracker9D, MockGNSS, MockIMUwithoutReorientation<MockIMUinBodyFrame>> positionTracker(&hrtc, position_tracker, gnss, imu);
    auto task = TaskPositionService<GNSSandAccelPosition<PositionTracker9D, MockGNSS, MockIMUwithoutReorientation<MockIMUinBodyFrame>>, Cyphal<LoopardAdapter>>(positionTracker, 100, 1, 123, adapters);

    const float x0 = 100.0f;
    const float y0 = 200.0f;
    const float z0 = 300.0f;

    const float vx0 = 10.0f;
    const float vy0 = 20.0f;
    const float vz0 = 30.0f;

    const float ax0 = 0.1f;
    const float ay0 = 0.2f;
    const float az0 = 0.3f;

    const float dt = 0.1f; // 100 ms
    std::chrono::milliseconds dduration(100);

    for (int i = 0; i < 100; ++i)
    {
        float t = dt * static_cast<float>(i);

        float x = x0 + vx0 * t + 0.5f * ax0 * t * t;
        float y = y0 + vy0 * t + 0.5f * ay0 * t * t;
        float z = z0 + vz0 * t + 0.5f * az0 * t * t;

        float x_ = x + 0.1f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX); // Adding noise
        float y_ = y + 0.1f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX); // Adding noise
        float z_ = z + 0.1f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX); // Adding noise

        float ax_ = ax0 + 0.1f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX); // Adding noise
        float ay_ = ay0 + 0.1f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX); // Adding noise
        float az_ = az0 + 0.1f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX); // Adding noise

        // GNSS returns cm!
        gnss.setPosition(static_cast<int32_t>(x_ * 1e2f), static_cast<int32_t>(y_ * 1e2f), static_cast<int32_t>(z_ * 1e2f));
        imu.setAcceleration(ax_, ay_, az_);
        task.handleTaskImpl();
        REQUIRE(loopard.buffer.size() == 1);

        auto transfer = loopard.buffer.pop();
        CHECK(transfer.metadata.port_id == _4111spyglass_sat_model_PositionVelocity_0_1_PORT_ID_);
        CHECK(transfer.metadata.transfer_kind == CyphalTransferKindMessage);
        CHECK(transfer.metadata.remote_node_id == id);
        CHECK(transfer.payload_size == _4111spyglass_sat_model_PositionVelocity_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);

        _4111spyglass_sat_model_PositionVelocity_0_1 received_data;
        size_t deserialized_size = transfer.payload_size;
        int8_t deserialization_result = _4111spyglass_sat_model_PositionVelocity_0_1_deserialize_(&received_data, static_cast<const uint8_t *>(transfer.payload), &deserialized_size);
        REQUIRE(deserialization_result >= 0);

        float vx = vx0 + ax0 * t;
        float vy = vy0 + ay0 * t;
        float vz = vz0 + az0 * t;

        if (i > 50)
        {
            CHECK(received_data.timestamp.microsecond == duration.count() * 1000);
            CHECK(received_data.position_m[0] == doctest::Approx(x).epsilon(0.1f));
            CHECK(received_data.position_m[1] == doctest::Approx(y).epsilon(0.1f));
            CHECK(received_data.position_m[2] == doctest::Approx(z).epsilon(0.1f));
            CHECK(received_data.velocity_ms[0] == doctest::Approx(vx).epsilon(0.1f));
            CHECK(received_data.velocity_ms[1] == doctest::Approx(vy).epsilon(0.1f));
            CHECK(received_data.velocity_ms[2] == doctest::Approx(vz).epsilon(0.1f));
        }
        duration += dduration;
        auto rtc = TimeUtils::to_rtc(duration, secondFraction);
        set_mocked_rtc_time(rtc.time);
        set_mocked_rtc_date(rtc.date);
    }
}

TEST_CASE("TaskPositionService Test with SGP4andGNSSandPosition")
{
    // Arrange
    RTC_HandleTypeDef hrtc;
    const uint32_t secondFraction = 1023;
    hrtc.Init.SynchPrediv = secondFraction;

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

    constexpr CyphalNodeID id = 11;
    LoopardAdapter loopard;
    loopard.memory_allocate = loopardMemoryAllocate;
    loopard.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(id);
    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);

    MockSGP4 sgp4;
    MockGNSS gnss;
    Sgp4PositionTracker position_tracker;
    SGP4andGNSSandPosition<Sgp4PositionTracker, MockSGP4, MockGNSS> positionTracker(&hrtc, position_tracker, sgp4, gnss);
    auto task = TaskPositionService<SGP4andGNSSandPosition<Sgp4PositionTracker, MockSGP4, MockGNSS>, Cyphal<LoopardAdapter>>(positionTracker, 100, 1, 123, adapters);

    const float x0 = 100.0f;
    const float y0 = 200.0f;
    const float z0 = 300.0f;

    const float vx0 = 10.0f;
    const float vy0 = 20.0f;
    const float vz0 = 30.0f;

    const float ax0 = 0.1f;
    const float ay0 = 0.2f;
    const float az0 = 0.3f;

    const float dt = 0.1f; // 100 ms
    std::chrono::milliseconds dduration(100);

    for (int i = 0; i < 1000; ++i)
    {
        float t = dt * static_cast<float>(i);

        float x = x0 + vx0 * t + 0.5f * ax0 * t * t;
        float y = y0 + vy0 * t + 0.5f * ay0 * t * t;
        float z = z0 + vz0 * t + 0.5f * az0 * t * t;

        float vx = vx0 + ax0 * t;
        float vy = vy0 + ay0 * t;
        float vz = vz0 + az0 * t;

        float xs = x + 10.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX); // Adding noise
        float ys = y + 10.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX); // Adding noise
        float zs = z + 10.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX); // Adding noise

        float xg = x + 10.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX); // Adding noise
        float yg = y + 10.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX); // Adding noise
        float zg = z + 10.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX); // Adding noise

        float vxs = vx + 10.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX); // Adding noise
        float vys = vy + 10.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX); // Adding noise
        float vzs = vz + 10.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX); // Adding noise

        // GNSS returns cm!
        gnss.setPosition(static_cast<int32_t>(xg * 1e2f), static_cast<int32_t>(yg * 1e2f), static_cast<int32_t>(zg * 1e2f));
        sgp4.setPosition(xs, ys, zs);
        sgp4.setVelocity(vxs, vys, vzs);
        task.handleTaskImpl();
        REQUIRE(loopard.buffer.size() == 1);

        auto transfer = loopard.buffer.pop();
        CHECK(transfer.metadata.port_id == _4111spyglass_sat_model_PositionVelocity_0_1_PORT_ID_);
        CHECK(transfer.metadata.transfer_kind == CyphalTransferKindMessage);
        CHECK(transfer.metadata.remote_node_id == id);
        CHECK(transfer.payload_size == _4111spyglass_sat_model_PositionVelocity_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);

        _4111spyglass_sat_model_PositionVelocity_0_1 received_data;
        size_t deserialized_size = transfer.payload_size;
        int8_t deserialization_result = _4111spyglass_sat_model_PositionVelocity_0_1_deserialize_(&received_data, static_cast<const uint8_t *>(transfer.payload), &deserialized_size);
        REQUIRE(deserialization_result >= 0);

        if (i > 250)
        {
            CHECK(received_data.timestamp.microsecond == duration.count() * 1000);
            CHECK(received_data.position_m[0] == doctest::Approx(x).epsilon(0.1f));
            CHECK(received_data.position_m[1] == doctest::Approx(y).epsilon(0.1f));
            CHECK(received_data.position_m[2] == doctest::Approx(z).epsilon(0.1f));
            CHECK(received_data.velocity_ms[0] == doctest::Approx(vx).epsilon(10.f));
            CHECK(received_data.velocity_ms[1] == doctest::Approx(vy).epsilon(10.f));
            CHECK(received_data.velocity_ms[2] == doctest::Approx(vz).epsilon(10.f));
        }
        duration += dduration;
        auto rtc = TimeUtils::to_rtc(duration, secondFraction);
        set_mocked_rtc_time(rtc.time);
        set_mocked_rtc_date(rtc.date);
    }
}

TEST_CASE("plain SGP4: send position 2025 6 25 18 0 0")
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

    // Create adapters
    LoopardAdapter loopard;
    loopard.memory_allocate = loopardMemoryAllocate;
    loopard.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(id);
    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);

    SGP4 sgp4(&hrtc);
    auto task = std::make_shared<TaskPositionService<SGP4, Cyphal<LoopardAdapter>>>(sgp4, 1000, 0, 0, adapters);

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
    CHECK(transfer.metadata.port_id == _4111spyglass_sat_model_PositionVelocity_0_1_PORT_ID_);
    CHECK(transfer.metadata.transfer_kind == CyphalTransferKindMessage);
    CHECK(transfer.metadata.remote_node_id == id);
    CHECK(transfer.payload_size == _4111spyglass_sat_model_PositionVelocity_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);

    // Deserialize the payload and verify the values
    _4111spyglass_sat_model_PositionVelocity_0_1 received_data;
    size_t deserialized_size = transfer.payload_size;
    int8_t deserialization_result = _4111spyglass_sat_model_PositionVelocity_0_1_deserialize_(&received_data, static_cast<const uint8_t *>(transfer.payload), &deserialized_size);
    REQUIRE(deserialization_result >= 0);
    CHECK(received_data.timestamp.microsecond == 804189600000000);

    std::array<float, 3> expected_r{2715.4f, -4518.34f, -4291.31f};
    std::array<float, 3> expected_v{3.75928f, 5.63901f, -3.55967f};

    CHECK(received_data.position_m[0] == doctest::Approx(expected_r[0] * 1000.f).epsilon(0.01));
    CHECK(received_data.position_m[1] == doctest::Approx(expected_r[1] * 1000.f).epsilon(0.01));
    CHECK(received_data.position_m[2] == doctest::Approx(expected_r[2] * 1000.f).epsilon(0.01));
    CHECK(received_data.velocity_ms[0] == doctest::Approx(expected_v[0] * 1000.f).epsilon(0.01));
    CHECK(received_data.velocity_ms[1] == doctest::Approx(expected_v[1] * 1000.f).epsilon(0.01));
    CHECK(received_data.velocity_ms[2] == doctest::Approx(expected_v[2] * 1000.f).epsilon(0.01));
    loopardMemoryFree(transfer.payload);
}

TEST_CASE("plain SGP4: send position 2025 7 6 20 43 13")
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

    // Create adapters
    LoopardAdapter loopard;
    loopard.memory_allocate = loopardMemoryAllocate;
    loopard.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(id);
    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);

    SGP4 sgp4(&hrtc);
    auto task = std::make_shared<TaskPositionService<SGP4, Cyphal<LoopardAdapter>>>(sgp4, 1000, 0, 0, adapters);

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
    CHECK(transfer.metadata.port_id == _4111spyglass_sat_model_PositionVelocity_0_1_PORT_ID_);
    CHECK(transfer.metadata.transfer_kind == CyphalTransferKindMessage);
    CHECK(transfer.metadata.remote_node_id == id);
    CHECK(transfer.payload_size == _4111spyglass_sat_model_PositionVelocity_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);

    // Deserialize the payload and verify the values
    _4111spyglass_sat_model_PositionVelocity_0_1 received_data;
    size_t deserialized_size = transfer.payload_size;
    int8_t deserialization_result = _4111spyglass_sat_model_PositionVelocity_0_1_deserialize_(&received_data, static_cast<const uint8_t *>(transfer.payload), &deserialized_size);
    REQUIRE(deserialization_result >= 0);
    CHECK(received_data.timestamp.microsecond == 805149793000000);

    std::array<float, 3> expected_r{6356.42f, -1504.07f, 1859.27f};
    std::array<float, 3> expected_v{-0.42784f, 5.18216f, 5.63173f};

    CHECK(received_data.position_m[0] == doctest::Approx(expected_r[0] * 1000.f).epsilon(0.01));
    CHECK(received_data.position_m[1] == doctest::Approx(expected_r[1] * 1000.f).epsilon(0.01));
    CHECK(received_data.position_m[2] == doctest::Approx(expected_r[2] * 1000.f).epsilon(0.01));
    CHECK(received_data.velocity_ms[0] == doctest::Approx(expected_v[0] * 1000.f).epsilon(0.01));
    CHECK(received_data.velocity_ms[1] == doctest::Approx(expected_v[1] * 1000.f).epsilon(0.01));
    CHECK(received_data.velocity_ms[2] == doctest::Approx(expected_v[2] * 1000.f).epsilon(0.01));
    loopardMemoryFree(transfer.payload);
}