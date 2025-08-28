#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <memory>
#include <tuple>
#include <vector>

#include "TaskSubscribeNodePortList.hpp"
#include "SubscriptionManager.hpp"
#include "RegistrationManager.hpp"
#include "Task.hpp"
#include "cyphal.hpp"
#include "loopard_adapter.hpp"
#include "_4111Spyglass.h"
#include "uavcan/node/port/List_1_0.h"
#include "uavcan/file/List_0_2.h"

// Mock HAL for testing (if needed)
#ifdef __x86_64__
#include "mock_hal.h" // You might need this or a similar mock HAL
#endif

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

// Helper function to create a CyphalTransfer message
std::shared_ptr<CyphalTransfer> createNodePortListTransfer(const std::vector<CyphalPortID> &publishers, const std::vector<CyphalPortID> &subscribers, const std::vector<CyphalPortID> &clients, const std::vector<CyphalPortID> &servers)
{
    uavcan_node_port_List_1_0 data = {};

    // Populate publishers
    data.publishers.sparse_list.count = publishers.size();
    for (size_t i = 0; i < publishers.size(); ++i)
    {
        data.publishers.sparse_list.elements[i].value = publishers[i];
    }

    // Populate subscribers
    data.subscribers.sparse_list.count = subscribers.size();
    for (size_t i = 0; i < subscribers.size(); ++i)
    {
        data.subscribers.sparse_list.elements[i].value = subscribers[i];
    }

    // Populate clients mask
    for (const auto& port_id : clients)
    {
        nunavutSetBit(data.clients.mask_bitpacked_, sizeof(data.clients.mask_bitpacked_), port_id, true);
    }

    // Populate servers mask
    for (const auto& port_id : servers)
    {
        nunavutSetBit(data.servers.mask_bitpacked_, sizeof(data.servers.mask_bitpacked_), port_id, true);
    }

    size_t payload_size = uavcan_node_port_List_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t payload[uavcan_node_port_List_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
    int8_t result = uavcan_node_port_List_1_0_serialize_(&data, payload, &payload_size);

    CHECK(result >= 0);

    CyphalTransfer transfer = {};
    transfer.payload_size = payload_size;
    transfer.payload = new uint8_t[payload_size];
    memcpy(transfer.payload, payload, payload_size); // Copy the serialized data

    transfer.metadata.port_id = uavcan_node_port_List_1_0_FIXED_PORT_ID_;
    transfer.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer.metadata.remote_node_id = 123; // Some dummy node ID

    return std::make_shared<CyphalTransfer>(transfer);
}

TEST_CASE("TaskSubscribeNodePortList: handleTaskImpl subscribes to ports in NodePortList")
{
    // Create instances
    SubscriptionManager subscription_manager;
    RegistrationManager registration_manager;
    DummyAdapter adapter1(1), adapter2(2); // Replace with your actual adapter
    std::tuple<DummyAdapter &, DummyAdapter &> adapters(adapter1, adapter2);

    // Create the task
    auto task = std::make_shared<TaskSubscribeNodePortList<DummyAdapter &, DummyAdapter &>>(&subscription_manager, 100, 0, adapters); // Replace with your real adapter types

    // Create the Registration manager;
    registration_manager.add(std::static_pointer_cast<Task>(task));

    // Create a NodePortList message with some publisher and subscriber ports
    std::vector<CyphalPortID> publishers = {_4111spyglass_sat_sensor_Magnetometer_0_1_PORT_ID_};
    std::vector<CyphalPortID> subscribers = {_4111spyglass_sat_sensor_GNSS_0_1_PORT_ID_};
    std::vector<CyphalPortID> clients = {uavcan_node_GetInfo_1_0_FIXED_PORT_ID_}; // Example client port
    std::vector<CyphalPortID> servers = {}; // Example server port
    std::shared_ptr<CyphalTransfer> transfer = createNodePortListTransfer(publishers, subscribers, clients, servers);

    // Push the transfer into the task's buffer
    task->handleMessage(transfer);

    // Execute the task's handleTaskImpl
    task->handleTaskImpl();

    // Check registration manager state
    CHECK(registration_manager.getHandlers().size() == 1);

    // Check adapter calls. The `clients` vector contains uavcan_node_GetInfo_1_0_FIXED_PORT_ID_ which will trigger subscription.
    CHECK(adapter1.subscribe_count == 3); // Expecting one call for the publisher, subscriber, and client.
    CHECK(adapter2.subscribe_count == 3); // Expecting one call for the publisher, subscriber, and client.
    CHECK(adapter1.last_port_id == uavcan_node_GetInfo_1_0_FIXED_PORT_ID_);
    CHECK(adapter2.last_port_id == uavcan_node_GetInfo_1_0_FIXED_PORT_ID_);

    // Clean up by unsubscribing (optional, but good practice)
    registration_manager.remove(std::static_pointer_cast<Task>(task));
    delete [] reinterpret_cast<uint8_t*>(transfer->payload);
}

TEST_CASE("TaskSubscribeNodePortList: registerTask and unregisterTask work correctly")
{
    SubscriptionManager subscription_manager;
    RegistrationManager registration_manager;
    DummyAdapter adapter1(1), adapter2(2);
    std::tuple<DummyAdapter &, DummyAdapter &> adapters(adapter1, adapter2);

    auto task = std::make_shared<TaskSubscribeNodePortList<DummyAdapter &, DummyAdapter &>>(&subscription_manager, 100, 0, adapters);

    // Initial state
    CHECK(registration_manager.getHandlers().size() == 0);

    // Register the task
    task->registerTask(&registration_manager, task);
    CHECK(registration_manager.getHandlers().size() == 1);

    // Unregister the task
    task->unregisterTask(&registration_manager, task);
    CHECK(registration_manager.getHandlers().size() == 0);
}

TEST_CASE("TaskSubscribeNodePortList: handleTaskImpl subscribes to client and server ports in NodePortList") {
    // Create instances
    SubscriptionManager subscription_manager;
    RegistrationManager registration_manager;
    DummyAdapter adapter1(1), adapter2(2);
    std::tuple<DummyAdapter &, DummyAdapter &> adapters(adapter1, adapter2);

    // Create the task
    auto task = std::make_shared<TaskSubscribeNodePortList<DummyAdapter &, DummyAdapter &>>(&subscription_manager, 100, 0, adapters);

    // Add the task to the registration manager
    registration_manager.add(std::static_pointer_cast<Task>(task));

    // Define client and server ports
    std::vector<CyphalPortID> publishers = {};
    std::vector<CyphalPortID> subscribers = {};
    std::vector<CyphalPortID> clients = {uavcan_node_GetInfo_1_0_FIXED_PORT_ID_}; // GetInfo request
    std::vector<CyphalPortID> servers = {uavcan_file_List_0_2_FIXED_PORT_ID_};   // File list response, not in list
    // std::vector<CyphalPortID> servers = {uavcan_file_Read_1_1_FIXED_PORT_ID_};   // File read response, not in list

    // Create a NodePortList message with the defined client and server ports
    std::shared_ptr<CyphalTransfer> transfer = createNodePortListTransfer(publishers, subscribers, clients, servers);

    // Push the transfer into the task's buffer
    task->handleMessage(transfer);

    // Execute the task's handleTaskImpl
    task->handleTaskImpl();

    // Validate that the adapters' subscribe counts are correct
    CHECK(adapter1.subscribe_count == 1);
    CHECK(adapter2.subscribe_count == 1);

    // Validate that the last subscribed port id is correct
    CHECK(adapter1.last_port_id == uavcan_node_GetInfo_1_0_FIXED_PORT_ID_);
    CHECK(adapter2.last_port_id == uavcan_node_GetInfo_1_0_FIXED_PORT_ID_);

    // Remove task
    registration_manager.remove(std::static_pointer_cast<Task>(task));
}