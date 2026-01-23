#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "TaskOrientationService.hpp"
#include "OrientationService.hpp"

#include "IMU.hpp"
#include "mock_hal.h"
#include "TimeUtils.hpp"
#include "RegistrationManager.hpp"
#include "loopard_adapter.hpp"
#include <array>
#include <tuple>

// Mock IMU class
class MockIMUinBodyFrame
{
public:
    MockIMUinBodyFrame() : has_acc_data(false), has_gyro_data(false), has_mag_data(false) {}

    void setAccelerometer(float x, float y, float z)
    {
        acceleration= { au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(x), 
                        au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(y),
                        au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(z)};
        has_acc_data = true;
    }

    std::optional<AccelerationInBodyFrame> readAccelerometer()
    {
        if (has_acc_data)
        {
            return acceleration;
        }
        else
        {
            return std::nullopt;
        }
    }

    void setGyroscope(float x, float y, float z)
    {
        gyroscope = { au::make_quantity<au::DegreesPerSecondInBodyFrame>(x),
                      au::make_quantity<au::DegreesPerSecondInBodyFrame>(y),
                      au::make_quantity<au::DegreesPerSecondInBodyFrame>(z)};
        has_gyro_data = true;
    }

    std::optional<AngularVelocityInBodyFrame> readGyroscope()
    {
        if (has_gyro_data)
        {
            return gyroscope;
        }
        else
        {
            return std::nullopt;
        }
    }

    void setMagnetometer(float x, float y, float z)
    {
        magnetometer = { au::make_quantity<au::TeslaInBodyFrame>(x),
                         au::make_quantity<au::TeslaInBodyFrame>(y),
                         au::make_quantity<au::TeslaInBodyFrame>(z)};
        has_mag_data = true;
    }
    
    std::optional<MagneticFieldInBodyFrame> readMagnetometer()
    {
        if (has_mag_data)
        {
            return magnetometer;
        }
        else
        {
            return std::nullopt;
        }
    }

private:
    AccelerationInBodyFrame acceleration;
    AngularVelocityInBodyFrame gyroscope;
    MagneticFieldInBodyFrame magnetometer;
    bool has_acc_data, has_gyro_data, has_mag_data;
};
static_assert(HasBodyAccelerometer<MockIMUinBodyFrame>);
static_assert(HasBodyGyroscope<MockIMUinBodyFrame>);
static_assert(HasBodyMagnetometer<MockIMUinBodyFrame>);

void *loopardMemoryAllocate(size_t amount) { return static_cast<void *>(malloc(amount)); };
void loopardMemoryFree(void *pointer) { free(pointer); };

TEST_CASE("TaskOrientationService Test with GyrMagOrientation")
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

    MockIMUinBodyFrame imu;
    GyrMagOrientationTracker tracker;
    GyrMagOrientation orientationTracker(&hrtc, tracker, imu, imu);
    auto task = TaskOrientationService(orientationTracker, 100, 1, 123, adapters);


    const float dt = 0.1f; // 100 ms
    std::chrono::milliseconds dduration(100);

    for (int i = 0; i < 100; ++i)
    {
        float t = dt * static_cast<float>(i);

        float gx = 0.1f * t; // Gyroscope x-axis
        float gy = 0.2f * t; // Gyroscope y-axis
        float gz = 0.3f * t; // Gyroscope z-axis
        
        float mx = 0.4f * t; // Magnetometer x-axis
        float my = 0.5f * t; // Magnetometer y-axis
        float mz = 0.6f * t; // Magnetometer z-axis
        
        imu.setGyroscope(gx, gy, gz);
        imu.setMagnetometer(mx, my, mz);
        task.handleTaskImpl();
        REQUIRE(loopard.buffer.size() == 1);

        auto transfer = loopard.buffer.pop();
        CHECK(transfer.metadata.port_id == _4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_);
        CHECK(transfer.metadata.transfer_kind == CyphalTransferKindMessage);
        CHECK(transfer.metadata.remote_node_id == id);
        CHECK(transfer.payload_size == _4111spyglass_sat_solution_OrientationSolution_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);

        _4111spyglass_sat_solution_OrientationSolution_0_1 received_data;
        size_t deserialized_size = transfer.payload_size;
        int8_t deserialization_result = _4111spyglass_sat_solution_OrientationSolution_0_1_deserialize_(&received_data, static_cast<const uint8_t *>(transfer.payload), &deserialized_size);
        REQUIRE(deserialization_result >= 0);

        Eigen::Quaternionf current_orientation = tracker.getOrientation();
        float qw = current_orientation.w();
        float qx = current_orientation.x();
        float qy = current_orientation.y();
        float qz = current_orientation.z();

        if (i > 50)
        {
            CHECK(received_data.timestamp.microsecond == duration.count() * 1000);
            CHECK(received_data.quaternion_ned.wxyz[0] == doctest::Approx(qw).epsilon(0.1f));
            CHECK(received_data.quaternion_ned.wxyz[1] == doctest::Approx(qx).epsilon(0.1f));
            CHECK(received_data.quaternion_ned.wxyz[2] == doctest::Approx(qy).epsilon(0.1f));
            CHECK(received_data.quaternion_ned.wxyz[3] == doctest::Approx(qz).epsilon(0.1f));
        }
        duration += dduration;
        auto rtc = TimeUtils::to_rtc(duration, secondFraction);
        set_mocked_rtc_time(rtc.time);
        set_mocked_rtc_date(rtc.date);
    }
}

TEST_CASE("TaskOrientationService Test with AccGyrOrientation")
{
    RTC_HandleTypeDef hrtc;
    const uint32_t secondFraction = 1023;
    hrtc.Init.SynchPrediv = secondFraction;

    TimeUtils::DateTimeComponents dtc{2000, 1, 1, 0, 0, 1, 0};
    auto duration = TimeUtils::to_epoch_duration(dtc);
    auto rtc = TimeUtils::to_rtc(duration, secondFraction);
    set_mocked_rtc_time(rtc.time);
    set_mocked_rtc_date(rtc.date);

    constexpr CyphalNodeID id = 13;
    LoopardAdapter loopard;
    loopard.memory_allocate = loopardMemoryAllocate;
    loopard.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(id);
    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);

    MockIMUinBodyFrame imu;
    AccGyrOrientationTracker tracker;
    tracker.setReferenceVectors(Eigen::Vector3f(0.f, 0.f, 9.81f));
    AccGyrOrientation orientationTracker(&hrtc, tracker, imu);
    auto task = TaskOrientationService(orientationTracker, 100, 1, 0, adapters);

    const float dt = 0.1f;
    std::chrono::milliseconds dduration(100);

    for (int i = 0; i < 100; ++i)
    {
        float t = dt * static_cast<float>(i);
        imu.setGyroscope(0.1f * t, 0.2f * t, 0.3f * t);
        imu.setAccelerometer(0.f, 0.f, 9.81f);

        task.handleTaskImpl();
        REQUIRE(loopard.buffer.size() == 1);

        auto transfer = loopard.buffer.pop();
        CHECK(transfer.metadata.port_id == _4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_);
        CHECK(transfer.metadata.remote_node_id == id);

        _4111spyglass_sat_solution_OrientationSolution_0_1 received_data;
        size_t deserialized_size = transfer.payload_size;
        int8_t result = _4111spyglass_sat_solution_OrientationSolution_0_1_deserialize_(&received_data, static_cast<const uint8_t *>(transfer.payload), &deserialized_size);
        REQUIRE(result >= 0);

        Eigen::Quaternionf q = tracker.getOrientation();
        if (i > 50)
        {
            CHECK(received_data.quaternion_ned.wxyz[0] == doctest::Approx(q.w()).epsilon(0.1f));
            CHECK(received_data.quaternion_ned.wxyz[1] == doctest::Approx(q.x()).epsilon(0.1f));
            CHECK(received_data.quaternion_ned.wxyz[2] == doctest::Approx(q.y()).epsilon(0.1f));
            CHECK(received_data.quaternion_ned.wxyz[3] == doctest::Approx(q.z()).epsilon(0.1f));
        }

        duration += dduration;
        auto rtc = TimeUtils::to_rtc(duration, secondFraction);
        set_mocked_rtc_time(rtc.time);
        set_mocked_rtc_date(rtc.date);
    }
}


TEST_CASE("TaskOrientationService Test with AccGyrMagOrientation")
{
    RTC_HandleTypeDef hrtc;
    const uint32_t secondFraction = 1023;
    hrtc.Init.SynchPrediv = secondFraction;

    TimeUtils::DateTimeComponents dtc{2000, 1, 1, 0, 0, 1, 0};
    auto duration = TimeUtils::to_epoch_duration(dtc);
    auto rtc = TimeUtils::to_rtc(duration, secondFraction);
    set_mocked_rtc_time(rtc.time);
    set_mocked_rtc_date(rtc.date);

    constexpr CyphalNodeID id = 12;
    LoopardAdapter loopard;
    loopard.memory_allocate = loopardMemoryAllocate;
    loopard.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(id);
    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);

    MockIMUinBodyFrame imu;
    AccGyrMagOrientationTracker tracker;
    tracker.setReferenceVectors(Eigen::Vector3f(0.f, 0.f, 9.81f), Eigen::Vector3f(1.f, 0.f, 0.f));
    AccGyrMagOrientation orientationTracker(&hrtc, tracker, imu, imu);
    auto task = TaskOrientationService(orientationTracker, 100, 1, 0, adapters);

    const float dt = 0.1f;
    std::chrono::milliseconds dduration(100);

    for (int i = 0; i < 100; ++i)
    {
        float t = dt * static_cast<float>(i);
        imu.setGyroscope(0.1f * t, 0.2f * t, 0.3f * t);
        imu.setAccelerometer(0.f, 0.f, 9.81f);
        imu.setMagnetometer(0.4f * t, 0.5f * t, 0.6f * t);

        task.handleTaskImpl();
        REQUIRE(loopard.buffer.size() == 1);

        auto transfer = loopard.buffer.pop();
        CHECK(transfer.metadata.port_id == _4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_);
        CHECK(transfer.metadata.remote_node_id == id);

        _4111spyglass_sat_solution_OrientationSolution_0_1 received_data;
        size_t deserialized_size = transfer.payload_size;
        int8_t result = _4111spyglass_sat_solution_OrientationSolution_0_1_deserialize_(&received_data, static_cast<const uint8_t *>(transfer.payload), &deserialized_size);
        REQUIRE(result >= 0);

        Eigen::Quaternionf q = tracker.getOrientation();
        if (i > 50)
        {
            CHECK(received_data.quaternion_ned.wxyz[0] == doctest::Approx(q.w()).epsilon(0.1f));
            CHECK(received_data.quaternion_ned.wxyz[1] == doctest::Approx(q.x()).epsilon(0.1f));
            CHECK(received_data.quaternion_ned.wxyz[2] == doctest::Approx(q.y()).epsilon(0.1f));
            CHECK(received_data.quaternion_ned.wxyz[3] == doctest::Approx(q.z()).epsilon(0.1f));
        }

        duration += dduration;
        auto rtc = TimeUtils::to_rtc(duration, secondFraction);
        set_mocked_rtc_time(rtc.time);
        set_mocked_rtc_date(rtc.date);
    }
}


