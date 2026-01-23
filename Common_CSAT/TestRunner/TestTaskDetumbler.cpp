#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "TaskDetumbler.hpp"
#include "RegistrationManager.hpp"
#include "cyphal.hpp"
#include "loopard_adapter.hpp"
#include "nunavut_assert.h"
#include "_4111Spyglass.h"
#include "_4111spyglass/sat/solution/OrientationSolution_0_1.h"

// Dummy Adapter (replace with your real adapter)
class DummyAdapter
{
public:
    DummyAdapter(int id) : id_(id), subscribe_count(0), unsubscribe_count(0) {}

    int8_t cyphalRxSubscribe(CyphalTransferKind /*transfer_kind*/, CyphalPortID port_id, size_t /*extent*/, CyphalMicrosecond /*timeout*/)
    {
        subscribe_count++;
        last_port_id = port_id;
        return 1;
    }

    int8_t cyphalRxUnsubscribe(CyphalTransferKind /*transfer_kind*/, CyphalPortID port_id)
    {
        unsubscribe_count++;
        last_port_id = port_id;
        return 1;
    }

    int id() const { return id_; }

private:
    int id_;

public:
    int subscribe_count;
    int unsubscribe_count;
    CyphalPortID last_port_id;
};

std::shared_ptr<CyphalTransfer> createOrientationSolutionTransfer(uint64_t timestamp_us,
                                                                  const std::array<float, 4> &q,
                                                                  const std::array<float, 3> &angular_velocity,
                                                                  const std::array<float, 3> &magnetic_field,
                                                                  const std::array<float, 3> &euler_radians,
                                                                  uint8_t node_id = 42)
{
    _4111spyglass_sat_solution_OrientationSolution_0_1 data{};
    data.timestamp.microsecond = timestamp_us;

    // Quaternion
    data.quaternion_ned.wxyz[0] = q[0];
    data.quaternion_ned.wxyz[1] = q[1];
    data.quaternion_ned.wxyz[2] = q[2];
    data.quaternion_ned.wxyz[3] = q[3];
    data.valid_quaternion = true;

    // Angular velocity
    data.angular_velocity_ned.radian_per_second[0] = angular_velocity[0];
    data.angular_velocity_ned.radian_per_second[1] = angular_velocity[1];
    data.angular_velocity_ned.radian_per_second[2] = angular_velocity[2];
    data.valid_angular_velocity = true;

    // Magnetic field
    data.magnetic_field_body.tesla[0] = magnetic_field[0];
    data.magnetic_field_body.tesla[1] = magnetic_field[1];
    data.magnetic_field_body.tesla[2] = magnetic_field[2];
    data.valid_magnetic_field = true;

    // Euler angles
    data.yaw_ned.radian = euler_radians[0];
    data.pitch_ned.radian = euler_radians[1];
    data.roll_ned.radian = euler_radians[2];
    data.valid_yaw_pitch_roll = true;

    // Serialize
    constexpr size_t PAYLOAD_SIZE = _4111spyglass_sat_solution_OrientationSolution_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t payload[PAYLOAD_SIZE];
    size_t payload_size = PAYLOAD_SIZE;
    REQUIRE(_4111spyglass_sat_solution_OrientationSolution_0_1_serialize_(&data, payload, &payload_size) == 0);

    // Package transfer
    auto transfer = std::make_shared<CyphalTransfer>();
    transfer->payload_size = payload_size;
    transfer->payload = new uint8_t[payload_size];
    std::memcpy(transfer->payload, payload, payload_size);
    transfer->metadata.port_id = _4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_;
    transfer->metadata.transfer_kind = CyphalTransferKindMessage;
    transfer->metadata.remote_node_id = node_id;

    return transfer;
}

std::shared_ptr<CyphalTransfer> createOrientationSolutionTransfer(uint64_t timestamp_us)
{
    return createOrientationSolutionTransfer(
        timestamp_us,
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.01f, 0.02f, 0.03f},
        {0.0001f, 0.0002f, 0.0003f},
        {0.1f, 0.2f, 0.3f});
}

TEST_CASE("TaskDetumbler registers and unregisters correctly")
{
    RegistrationManager manager;
    DummyAdapter adapter(1);
    std::tuple<DummyAdapter &> adapters(adapter);
    DetumblerSystem::Config detumbler_config{};
    auto task = std::make_shared<TaskDetumbler<DummyAdapter &>>(detumbler_config, 100, 0, adapters);

    task->registerTask(&manager, task);
    REQUIRE(manager.containsTask(task));
    task->unregisterTask(&manager, task);
    REQUIRE(!manager.containsTask(task));
}

TEST_CASE("TaskDetumbler processes valid heartbeat transfer")
{
    DummyAdapter adapter(1);
    std::tuple<DummyAdapter &> adapters(adapter);

    GPIO_TypeDef GPIOE;
    TIM_HandleTypeDef htim15;
    TIM_HandleTypeDef htim16;
    TIM_HandleTypeDef htim17;

DetumblerSystem::Config detumbler_config{
    .bdot_gain = 1e4f,

    .driver_config = {
        .max_dipole_x = 0.5f,
        .max_dipole_y = 0.5f,
        .max_dipole_z = 0.5f
    },

    .pwm_channels = {
        .x = MagnetorquerHardwareInterface::Channel{&htim16, TIM_CHANNEL_1, 999},
        .y = MagnetorquerHardwareInterface::Channel{&htim17, TIM_CHANNEL_1, 999},
        .z = MagnetorquerHardwareInterface::Channel{&htim15, TIM_CHANNEL_1, 999}
    },

    .gpio_pins = {
        .x = MagnetorquerPolarityController::AxisPins{&GPIOE, GPIO_PIN_1, &GPIOE, GPIO_PIN_2},
        .y = MagnetorquerPolarityController::AxisPins{&GPIOE, GPIO_PIN_3, &GPIOE, GPIO_PIN_4},
        .z = MagnetorquerPolarityController::AxisPins{&GPIOE, GPIO_PIN_5, &GPIOE, GPIO_PIN_6}
    }
};

    TaskDetumbler<DummyAdapter &> task(detumbler_config, 100, 0, adapters);

    auto transfer = createOrientationSolutionTransfer(123);
    task.handleMessage(transfer);

    task.handleTaskImpl();
}

TEST_CASE("TaskDetumbler skips empty buffer")
{
    DummyAdapter adapter(1);
    std::tuple<DummyAdapter &> adapters(adapter);
    DetumblerSystem::Config detumbler_config{};

    TaskDetumbler<DummyAdapter &> task(detumbler_config, 100, 0, adapters);
}

