#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "TaskProcessHeartBeat.hpp"
#include "RegistrationManager.hpp"
#include "cyphal.hpp"
#include "loopard_adapter.hpp"
#include "_4111Spyglass.h"
#include "uavcan/node/Heartbeat_1_0.h"

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

// Helper to create a valid heartbeat transfer
std::shared_ptr<CyphalTransfer> createHeartbeatTransfer(uint32_t uptime, uint8_t node_id = 42)
{
    uavcan_node_Heartbeat_1_0 heartbeat{};
    heartbeat.uptime = uptime;
    heartbeat.health.value = uavcan_node_Health_1_0_NOMINAL;
    heartbeat.mode.value = uavcan_node_Mode_1_0_OPERATIONAL;
    heartbeat.vendor_specific_status_code = 0;

    uint8_t payload[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t payload_size = sizeof(payload);
    REQUIRE(uavcan_node_Heartbeat_1_0_serialize_(&heartbeat, payload, &payload_size) == 0);

    auto transfer = std::make_shared<CyphalTransfer>();
    transfer->payload_size = payload_size;
    transfer->payload = new uint8_t[payload_size];
    std::memcpy(transfer->payload, payload, payload_size);
    transfer->metadata.port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_;
    transfer->metadata.transfer_kind = CyphalTransferKindMessage;
    transfer->metadata.remote_node_id = node_id;

    return transfer;
}

TEST_CASE("TaskProcessHeartBeat registers and unregisters correctly")
{
    RegistrationManager manager;
    DummyAdapter adapter(1);
    std::tuple<DummyAdapter &> adapters(adapter);
    auto task = std::make_shared<TaskProcessHeartBeat<DummyAdapter &>>(100, 0, adapters);

    task->registerTask(&manager, task);
    REQUIRE(manager.containsTask(task));
    task->unregisterTask(&manager, task);
    REQUIRE(! manager.containsTask(task));
}

TEST_CASE("TaskProcessHeartBeat processes valid heartbeat transfer")
{
    DummyAdapter adapter(1);
    std::tuple<DummyAdapter &> adapters(adapter);
    TaskProcessHeartBeat<DummyAdapter &> task(100, 0, adapters);

    auto transfer = createHeartbeatTransfer(123);
    task.handleMessage(transfer);

    task.handleTaskImpl();
}

TEST_CASE("TaskProcessHeartBeat skips empty buffer")
{
    DummyAdapter adapter(1);
    std::tuple<DummyAdapter &> adapters(adapter);
    TaskProcessHeartBeat<DummyAdapter &> task(100, 0, adapters);
}

TEST_CASE("TaskProcessHeartBeat handles malformed payload gracefully")
{
    DummyAdapter adapter(1);
    std::tuple<DummyAdapter &> adapters(adapter);
    TaskProcessHeartBeat<DummyAdapter &> task(100, 0, adapters);

    auto transfer = std::make_shared<CyphalTransfer>();
    transfer->payload_size = 4;
    transfer->payload = new uint8_t[4]{0xFF, 0xFF, 0xFF, 0xFF};
    transfer->metadata.port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_;
    transfer->metadata.transfer_kind = CyphalTransferKindMessage;
    transfer->metadata.remote_node_id = 99;

    task.handleMessage(transfer);
    task.handleTaskImpl();
}
