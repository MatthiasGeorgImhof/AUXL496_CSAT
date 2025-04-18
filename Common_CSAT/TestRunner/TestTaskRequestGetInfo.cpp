#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <memory>
#include <tuple>
#include <vector>
#include <map>
#include <cstring> // For std::memcpy, std::strcmp
#include <string>  // For std::stoull
#include <iostream>

#include "TaskRequestGetInfo.hpp" // Include the header for the class being tested
#include "Task.hpp"                // Include necessary headers
#include "cyphal.hpp"
#include "loopard_adapter.hpp"    // Or your specific adapter header
#include "RegistrationManager.hpp"
#include "ServiceManager.hpp" // Include ServiceManager header if needed
#include "Allocator.hpp"       // Include Allocator header if needed

#include "uavcan/node/GetInfo_1_0.h"

#ifdef __x86_64__
#include "mock_hal.h"
#endif

void *loopardMemoryAllocate(size_t amount) { return static_cast<void *>(malloc(amount)); };
void loopardMemoryFree(void *pointer) { free(pointer); };

// Helper function to create a CyphalTransfer response (replace with your actual response creation)
std::shared_ptr<CyphalTransfer> createGetInfoResponse(const uint8_t unique_id[16], const char name[50])
{
    uavcan_node_GetInfo_Response_1_0 data = {
        .protocol_version = {uavcan_node_Version_1_0{.major = 1, .minor = 0}},
        .hardware_version = {uavcan_node_Version_1_0{.major = 1, .minor = 0}},
        .software_version = {uavcan_node_Version_1_0{.major = 1, .minor = 0}},
        .software_vcs_revision_id = 0xc5ad8c7dll,
        .unique_id = {},
        .name = {},
        .software_image_crc = {},
        .certificate_of_authenticity = {}};
    std::memcpy(data.unique_id, unique_id, sizeof(data.unique_id));
    std::memcpy(data.name.elements, name, sizeof(data.name.elements));
    data.name.count = std::strlen(name);

    constexpr size_t PAYLOAD_SIZE = uavcan_node_GetInfo_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t payload[PAYLOAD_SIZE];
    size_t payload_size = PAYLOAD_SIZE;

    int8_t serialization_result = uavcan_node_GetInfo_Response_1_0_serialize_(&data, payload, &payload_size);
    REQUIRE(serialization_result >= 0);

    auto transfer = std::make_shared<CyphalTransfer>();
    transfer->metadata.transfer_kind = CyphalTransferKindResponse;
    transfer->metadata.port_id = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_;
    transfer->metadata.remote_node_id = 123; // Some dummy node ID
    transfer->metadata.transfer_id = 0;       // Dummy Transfer ID

    transfer->payload_size = payload_size;
    transfer->payload = new uint8_t[payload_size];
    std::memcpy(transfer->payload, payload, payload_size);

    return transfer;
}

// Class to expose buffer size for testing
template <typename... Adapters>
class MockTaskRequestGetInfo : public TaskRequestGetInfo<Adapters...>
{
public:
    MockTaskRequestGetInfo(uint32_t interval, uint32_t tick, CyphalNodeID node_id, CyphalTransferID transfer_id, std::tuple<Adapters...> &adapters) : TaskRequestGetInfo<Adapters...>(interval, tick, node_id, transfer_id, adapters) {}

    using TaskRequestGetInfo<Adapters...>::handleTaskImpl; // Expose
    using TaskRequestGetInfo<Adapters...>::handleMessage;
    size_t getBufferSize() const { return TaskRequestGetInfo<Adapters...>::buffer_.size(); }
    using TaskRequestGetInfo<Adapters...>::buffer_;

    void handleMessage(std::shared_ptr<CyphalTransfer> transfer) override {
        TaskRequestGetInfo<Adapters...>::buffer_.push(transfer);
    }
};

TEST_CASE("TaskRequestGetInfo: Sends GetInfo request and handles response")
{
    // Create adapters
    LoopardAdapter loopard1, loopard2;
    loopard1.memory_allocate = loopardMemoryAllocate;
    loopard1.memory_free = loopardMemoryFree;
    loopard2.memory_allocate = loopardMemoryAllocate;
    loopard2.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal1(&loopard1), loopard_cyphal2(&loopard2);
    CyphalNodeID node_id1 = 11;
    CyphalNodeID node_id2 = 12;
    loopard_cyphal1.setNodeID(node_id1);
    loopard_cyphal2.setNodeID(node_id2);
    std::tuple<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>> adapters(loopard_cyphal1, loopard_cyphal2);

    // Setup unique_id and name
    uint8_t unique_id[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    char name[50] = "Test Node";
    (void)name; // get rid of warnings
    (void)unique_id;

    CyphalNodeID remote_node_id = 42;
    CyphalTransferID transfer_id = 3;
    uint32_t tick = 0;
    uint32_t interval = 1000;

    // Instantiate the TaskRequestGetInfo (using MockTaskRequestGetInfo for testing)
    MockTaskRequestGetInfo task(interval, tick, remote_node_id, transfer_id, adapters);

    // Initial state: Buffer is empty, so a request should be sent
    CHECK(task.getBufferSize() == 0);
    task.handleTaskImpl();
    CHECK(loopard1.buffer.size() == 1);
    CHECK(loopard2.buffer.size() == 1);

    // Verify that a request was published on both adapters with the defined node_id and transfer_id
    CyphalTransfer request1 = loopard1.buffer.pop();
    CHECK(request1.metadata.port_id == uavcan_node_GetInfo_1_0_FIXED_PORT_ID_);
    CHECK(request1.metadata.transfer_kind == CyphalTransferKindRequest);
    CHECK(request1.metadata.remote_node_id == node_id1); //Request with client node_id
    CHECK(request1.metadata.transfer_id == transfer_id); //Request with defined transfer_id
    delete[] static_cast<uint8_t *>(request1.payload);

    CyphalTransfer request2 = loopard2.buffer.pop();
    CHECK(request2.metadata.port_id == uavcan_node_GetInfo_1_0_FIXED_PORT_ID_);
    CHECK(request2.metadata.transfer_kind == CyphalTransferKindRequest);
    CHECK(request2.metadata.remote_node_id == node_id2); //Request with client node_id
    CHECK(request2.metadata.transfer_id == transfer_id); //Request with defined transfer_id
    delete[] static_cast<uint8_t *>(request2.payload);

    // Simulate receiving a response: Add a response to the buffer
    std::shared_ptr<CyphalTransfer> response = createGetInfoResponse(unique_id, name);
    task.handleMessage(response);

    // Now the buffer should have one response
    CHECK(task.getBufferSize() == 1);

    task.handleTaskImpl();
    CHECK(loopard1.buffer.size() == 0); // No new publish
    CHECK(loopard2.buffer.size() == 0); // No new publish

    //The response should be removed from the buffer
    CHECK(task.getBufferSize() == 0);
}

TEST_CASE("TaskRequestGetInfo: registers and unregisters correctly")
{
    // Create instances
    RegistrationManager registration_manager;

    // Create adapters
    LoopardAdapter loopard1, loopard2;
    loopard1.memory_allocate = loopardMemoryAllocate;
    loopard1.memory_free = loopardMemoryFree;
    loopard2.memory_allocate = loopardMemoryAllocate;
    loopard2.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal1(&loopard1), loopard_cyphal2(&loopard2);
    loopard_cyphal1.setNodeID(11);
    loopard_cyphal2.setNodeID(12);
    std::tuple<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>> adapters(loopard_cyphal1, loopard_cyphal2);

    // Setup unique_id and name
    uint8_t unique_id[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    char name[50] = "Test Node";
    (void) name;
    (void) unique_id;
    CyphalNodeID remote_node_id = 42;
    CyphalTransferID transfer_id = 7;
        uint32_t tick = 0;
    uint32_t interval = 1000;

    // Create a TaskSendHeartBeat object with shared_ptr
    auto task = std::make_shared<MockTaskRequestGetInfo<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>>>(interval, tick, remote_node_id, transfer_id, adapters);

    // Initial state
    CHECK(registration_manager.getClients().size() == 0);

    // Register the task
    task->registerTask(&registration_manager, task);
    CHECK(registration_manager.getClients().size() == 1);
    CHECK(registration_manager.getClients().containsIf([&](const CyphalPortID &port_id)
                                                      { return port_id == uavcan_node_GetInfo_1_0_FIXED_PORT_ID_; }));
    // Unregister the task
    task->unregisterTask(&registration_manager, task);
    CHECK(registration_manager.getClients().size() == 0); // Subscription removed
}