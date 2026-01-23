#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "TaskMagnetorquer.hpp"
#include "RegistrationManager.hpp"
#include "cyphal.hpp"
#include "loopard_adapter.hpp"
#include "nunavut_assert.h"
#include "_4111Spyglass.h"
#include "_4111spyglass/sat/solution/OrientationSolution_0_1.h"
#include "_4111spyglass/sat/solution/PositionSolution_0_1.h"

    GPIO_TypeDef GPIOE;
    TIM_HandleTypeDef htim15;
    TIM_HandleTypeDef htim16;
    TIM_HandleTypeDef htim17;


// Dummy Adapter
class DummyAdapter
{
public:
    DummyAdapter(int id) : subscribe_count(0), unsubscribe_count(0), id_(id) {}

    int8_t cyphalRxSubscribe(CyphalTransferKind, CyphalPortID port_id, size_t, CyphalMicrosecond)
    {
        subscribe_count++;
        last_port_id = port_id;
        return 1;
    }

    int8_t cyphalRxUnsubscribe(CyphalTransferKind, CyphalPortID port_id)
    {
        unsubscribe_count++;
        last_port_id = port_id;
        return 1;
    }

    int id() const { return id_; }

    int subscribe_count;
    int unsubscribe_count;
    CyphalPortID last_port_id;

private:
    int id_;
};

// Transfer builder
std::shared_ptr<CyphalTransfer> createOrientationSolutionTransfer(uint64_t timestamp_us)
{
    _4111spyglass_sat_solution_OrientationSolution_0_1 data{};
    data.timestamp.microsecond = timestamp_us;
    data.valid_quaternion = true;
    data.valid_angular_velocity = true;
    data.valid_magnetic_field = true;

    data.quaternion_ned.wxyz[0] = 0.707f;  // cos(θ/2)
    data.quaternion_ned.wxyz[1] = 0.707f;  // sin(θ/2) * x
    data.quaternion_ned.wxyz[2] = 0.0f;
    data.quaternion_ned.wxyz[3] = 0.0f;

    data.angular_velocity_ned.radian_per_second[0] = 0.01f;
    data.angular_velocity_ned.radian_per_second[1] = 0.02f;
    data.angular_velocity_ned.radian_per_second[2] = 0.03f;

    data.magnetic_field_body.tesla[0] = 0.0001f;
    data.magnetic_field_body.tesla[1] = 0.0002f;
    data.magnetic_field_body.tesla[2] = 0.0003f;

    uint8_t payload[_4111spyglass_sat_solution_OrientationSolution_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t payload_size = sizeof(payload);
    REQUIRE(_4111spyglass_sat_solution_OrientationSolution_0_1_serialize_(&data, payload, &payload_size) == 0);

    auto transfer = std::make_shared<CyphalTransfer>();
    transfer->payload_size = payload_size;
    transfer->payload = new uint8_t[payload_size];
    std::memcpy(transfer->payload, payload, payload_size);
    transfer->metadata.port_id = _4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_;
    transfer->metadata.transfer_kind = CyphalTransferKindMessage;
    transfer->metadata.remote_node_id = 42;

    return transfer;
}

std::shared_ptr<CyphalTransfer> createPositionSolutionTransfer(uint64_t timestamp_us)
{
    _4111spyglass_sat_solution_PositionSolution_0_1 data{};
    data.timestamp.microsecond = timestamp_us;
    data.valid_position = true;
    data.valid_velocity = true;
    data.valid_acceleration = true;

    // Position in ECEF (meters)
    data.position_ecef.meter[0] = 6371000.0f;  // X
    data.position_ecef.meter[1] = 0.0f;        // Y
    data.position_ecef.meter[2] = 0.0f;        // Z

    // Velocity in ECEF (m/s)
    data.velocity_ecef.meter_per_second[0] = 0.0f;
    data.velocity_ecef.meter_per_second[1] = 7660.0f;
    data.velocity_ecef.meter_per_second[2] = 0.0f;

    // Acceleration in body frame (m/s²)
    data.acceleration_ecef.meter_per_second_per_second[0] = 0.0f;
    data.acceleration_ecef.meter_per_second_per_second[1] = 0.0f;
    data.acceleration_ecef.meter_per_second_per_second[2] = 9.81f;

    uint8_t payload[_4111spyglass_sat_solution_PositionSolution_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t payload_size = sizeof(payload);
    REQUIRE(_4111spyglass_sat_solution_PositionSolution_0_1_serialize_(&data, payload, &payload_size) == 0);

    auto transfer = std::make_shared<CyphalTransfer>();
    transfer->payload_size = payload_size;
    transfer->payload = new uint8_t[payload_size];
    std::memcpy(transfer->payload, payload, payload_size);
    transfer->metadata.port_id = _4111spyglass_sat_solution_PositionSolution_0_1_PORT_ID_;
    transfer->metadata.transfer_kind = CyphalTransferKindMessage;
    transfer->metadata.remote_node_id = 42;

    return transfer;
}

TEST_CASE("TaskMagnetorquer registers and unregisters correctly")
{
    RegistrationManager manager;
    DummyAdapter adapter(1);
    std::tuple<DummyAdapter &> adapters(adapter);
    MagnetorquerSystem::Config config{
        AttitudeController(0.5f, 0.1f),
        MagnetorquerDriver({0.5f, 0.5f, 0.5f}),
        {{&htim16, TIM_CHANNEL_1, 999},
         {&htim17, TIM_CHANNEL_1, 999},
         {&htim15, TIM_CHANNEL_1, 999}},
        {{&GPIOE, GPIO_PIN_1, &GPIOE, GPIO_PIN_2},
         {&GPIOE, GPIO_PIN_3, &GPIOE, GPIO_PIN_4},
         {&GPIOE, GPIO_PIN_5, &GPIOE, GPIO_PIN_6}}};

    auto task = std::make_shared<TaskMagnetorquer<DummyAdapter &>>(config, 100, 0, adapters);

    task->registerTask(&manager, task);
    REQUIRE(manager.containsTask(task));
    task->unregisterTask(&manager, task);
    REQUIRE(!manager.containsTask(task));
}

TEST_CASE("TaskMagnetorquer processes OrientationSolution when q_desired is valid")
{
    DummyAdapter adapter(1);
    std::tuple<DummyAdapter &> adapters(adapter);

    GPIO_TypeDef GPIOE;
    TIM_HandleTypeDef htim15, htim16, htim17;

    MagnetorquerSystem::Config config{
        .controller = AttitudeController(0.5f, 0.1f),
        .driver = MagnetorquerDriver({0.5f, 0.5f, 0.5f}),
        .pwm_channels = {
            .x = {&htim16, TIM_CHANNEL_1, 999},
            .y = {&htim17, TIM_CHANNEL_1, 999},
            .z = {&htim15, TIM_CHANNEL_1, 999}},
        .gpio_pins = {.x = {&GPIOE, GPIO_PIN_1, &GPIOE, GPIO_PIN_2}, .y = {&GPIOE, GPIO_PIN_3, &GPIOE, GPIO_PIN_4}, .z = {&GPIOE, GPIO_PIN_5, &GPIOE, GPIO_PIN_6}}};

    TaskMagnetorquer<DummyAdapter &> task(config, 100, 0, adapters);

    auto transfer1 = createPositionSolutionTransfer(123);
    task.handleMessage(transfer1);

    auto transfer2 = createOrientationSolutionTransfer(123);
    task.handleMessage(transfer2);
    task.handleTaskImpl();
}

TEST_CASE("TaskMagnetorquer skips OrientationSolution if q_desired is invalid")
{
    DummyAdapter adapter(1);
    std::tuple<DummyAdapter &> adapters(adapter);
    MagnetorquerSystem::Config config{
        AttitudeController(0.5f, 0.1f),
        MagnetorquerDriver({0.5f, 0.5f, 0.5f}),
        {{&htim16, TIM_CHANNEL_1, 999},
         {&htim17, TIM_CHANNEL_1, 999},
         {&htim15, TIM_CHANNEL_1, 999}},
        {{&GPIOE, GPIO_PIN_1, &GPIOE, GPIO_PIN_2},
         {&GPIOE, GPIO_PIN_3, &GPIOE, GPIO_PIN_4},
         {&GPIOE, GPIO_PIN_5, &GPIOE, GPIO_PIN_6}}};

    TaskMagnetorquer<DummyAdapter &> task(config, 100, 0, adapters);

    auto transfer = createOrientationSolutionTransfer(123);
    task.handleMessage(transfer);
    task.handleTaskImpl(); // Should skip due to invalid q_desired
}

TEST_CASE("TaskMagnetorquer skips empty buffer")
{
    DummyAdapter adapter(1);
    std::tuple<DummyAdapter &> adapters(adapter);
    MagnetorquerSystem::Config config{
        AttitudeController(0.5f, 0.1f),
        MagnetorquerDriver({0.5f, 0.5f, 0.5f}),
        {{&htim16, TIM_CHANNEL_1, 999},
         {&htim17, TIM_CHANNEL_1, 999},
         {&htim15, TIM_CHANNEL_1, 999}},
        {{&GPIOE, GPIO_PIN_1, &GPIOE, GPIO_PIN_2},
         {&GPIOE, GPIO_PIN_3, &GPIOE, GPIO_PIN_4},
         {&GPIOE, GPIO_PIN_5, &GPIOE, GPIO_PIN_6}}};

    TaskMagnetorquer<DummyAdapter &> task(config, 100, 0, adapters);
    task.handleTaskImpl(); // Should do nothing
}

TEST_CASE("TaskMagnetorquer sets correct PWM and GPIO states") {
    reset_timer_state(&htim15);
    reset_timer_state(&htim16);
    reset_timer_state(&htim17);
    reset_gpio_port_state(&GPIOE);

    DummyAdapter adapter(1);
    std::tuple<DummyAdapter&> adapters(adapter);

    MagnetorquerSystem::Config config{
        AttitudeController(0.5f, 0.1f),
        MagnetorquerDriver({0.5f, 0.5f, 0.5f}),
        {{&htim16, TIM_CHANNEL_1, 999},
         {&htim17, TIM_CHANNEL_1, 999},
         {&htim15, TIM_CHANNEL_1, 999}},
        {{&GPIOE, GPIO_PIN_1, &GPIOE, GPIO_PIN_2},
         {&GPIOE, GPIO_PIN_3, &GPIOE, GPIO_PIN_4},
         {&GPIOE, GPIO_PIN_5, &GPIOE, GPIO_PIN_6}}};

    TaskMagnetorquer<DummyAdapter&> task(config, 100, 0, adapters);

    // Provide valid desired attitude
    auto transfer1 = createPositionSolutionTransfer(123);
    task.handleMessage(transfer1);

    // Provide orientation solution
    auto transfer2 = createOrientationSolutionTransfer(123);
    task.handleMessage(transfer2);
    task.handleTaskImpl();

    // PWM checks
    CHECK(is_pwm_started(&htim16, TIM_CHANNEL_1));
    CHECK(is_pwm_started(&htim17, TIM_CHANNEL_1));
    CHECK(is_pwm_started(&htim15, TIM_CHANNEL_1));

    CHECK(get_compare_value(&htim16, TIM_CHANNEL_1) == 0);
    CHECK(get_compare_value(&htim17, TIM_CHANNEL_1) > 0);
    CHECK(get_compare_value(&htim15, TIM_CHANNEL_1) > 0);

    // Polarity checks (based on expected dipole signs)
    CHECK(get_gpio_pin_state(&GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET); // X polarity
    CHECK(get_gpio_pin_state(&GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET); // Y polarity
    CHECK(get_gpio_pin_state(&GPIOE, GPIO_PIN_6) == GPIO_PIN_SET); // Z polarity

    // Enable checks (LOW = active)
    CHECK(get_gpio_pin_state(&GPIOE, GPIO_PIN_1) == GPIO_PIN_RESET); // X enable
    CHECK(get_gpio_pin_state(&GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET); // Y enable
    CHECK(get_gpio_pin_state(&GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET); // Z enable
}
