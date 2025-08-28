#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <memory>
#include <tuple>
#include <vector>
#include <map>
#include <cstring> // For std::memcpy, std::strcmp
#include <string> // For std::stoull
#include <iostream>

#include "TaskRespondGetInfo.hpp" // Include the header for the class being tested
#include "Task.hpp"               // Include necessary headers
#include "cyphal.hpp"
#include "loopard_adapter.hpp"   // Or your specific adapter header
#include "RegistrationManager.hpp"
#include "ServiceManager.hpp" // Include ServiceManager header if needed
#include "Allocator.hpp"       // Include Allocator header if needed

#include "uavcan/node/GetInfo_1_0.h"

#ifdef __x86_64__
#include "mock_hal.h"
#endif

void *loopardMemoryAllocate(size_t amount) { return static_cast<void *>(malloc(amount)); };
void loopardMemoryFree(void *pointer) { free(pointer); };

// Helper function to create a CyphalTransfer request
std::shared_ptr<CyphalTransfer> createGetInfoRequest()
{
    auto transfer = std::make_shared<CyphalTransfer>();
    transfer->metadata.transfer_kind = CyphalTransferKindRequest;
    transfer->metadata.port_id = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_;
    transfer->metadata.remote_node_id = 123; // Some dummy node ID
    transfer->metadata.transfer_id = 0;       // Dummy Transfer ID

    // GetInfo requests have no payload.
    transfer->payload_size = 0;
    transfer->payload = nullptr; // Important: no payload!

    return transfer;
}

template <typename... Adapters>
class MockTaskRespondGetInfo : public TaskRespondGetInfo<Adapters...>
{
public:
    MockTaskRespondGetInfo(uint8_t unique_id_[16], char name_[50], uint32_t interval, uint32_t tick, std::tuple<Adapters...> &adapters) : TaskRespondGetInfo<Adapters...>(unique_id_, name_, interval, tick, adapters) {}

    using TaskRespondGetInfo<Adapters...>::handleMessage;
    size_t getBufferSize() const { return TaskRespondGetInfo<Adapters...>::buffer_.size(); }
    using TaskRespondGetInfo<Adapters...>::buffer_;
};

TEST_CASE("TaskRespondGetInfo: Handles GetInfo request and publishes response")
{
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

    // Instantiate the TaskRespondGetInfo
    MockTaskRespondGetInfo task(unique_id, name, 1000, 0, adapters);

    // Create a mock GetInfo request
    std::shared_ptr<CyphalTransfer> request = createGetInfoRequest();

    // Push the request into the task's buffer
    task.handleMessage(request);

    // Verify the buffer size
    CHECK(task.getBufferSize() == 1);

    // Execute the task
    task.handleTaskImpl();

    // Verify that a response was published on both adapters
    CHECK(loopard1.buffer.size() == 1);
    CHECK(loopard2.buffer.size() == 1);

    // Check the contents of the published response (adapter 1)
    CyphalTransfer response1 = loopard1.buffer.pop();
    CHECK(response1.metadata.port_id == uavcan_node_GetInfo_1_0_FIXED_PORT_ID_);
    CHECK(response1.metadata.transfer_kind == CyphalTransferKindResponse);
    CHECK(response1.payload_size <= uavcan_node_GetInfo_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_);
    CHECK(response1.payload_size <= 42);

    // Deserialize the response and verify the values
    uavcan_node_GetInfo_Response_1_0 received_response1;
    size_t deserialized_size1 = response1.payload_size;
    int8_t deserialization_result1 = uavcan_node_GetInfo_Response_1_0_deserialize_(&received_response1, static_cast<const uint8_t *>(response1.payload), &deserialized_size1);

    CHECK(deserialization_result1 >= 0);
    CHECK(std::memcmp(received_response1.unique_id, unique_id, 16) == 0);
    CHECK(std::strncmp(reinterpret_cast<const char*>(received_response1.name.elements), name, received_response1.name.count) == 0);

    loopardMemoryFree(response1.payload); // Clean up the payload

    // Check the contents of the published response (adapter 2)
    CyphalTransfer response2 = loopard2.buffer.pop();
    CHECK(response2.metadata.port_id == uavcan_node_GetInfo_1_0_FIXED_PORT_ID_);
    CHECK(response2.metadata.transfer_kind == CyphalTransferKindResponse);
    CHECK(response2.payload_size <= uavcan_node_GetInfo_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_);
    CHECK(response2.payload_size == 42);

    // Deserialize the response and verify the values
    uavcan_node_GetInfo_Response_1_0 received_response2;
    size_t deserialized_size2 = response2.payload_size;
    int8_t deserialization_result2 = uavcan_node_GetInfo_Response_1_0_deserialize_(&received_response2, static_cast<const uint8_t *>(response2.payload), &deserialized_size2);

    CHECK(deserialization_result2 >= 0);
    CHECK(std::memcmp(received_response2.unique_id, unique_id, 16) == 0);
    CHECK(std::strncmp(reinterpret_cast<const char*>(received_response2.name.elements), name, received_response2.name.count) == 0);  //Use strncmp

    loopardMemoryFree(response2.payload); // Clean up the payload
}

TEST_CASE("TaskRespondGetInfo: registers and unregisters correctly")
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

    // Create a TaskSendHeartBeat object with shared_ptr
    auto task_sendheartbeat = std::make_shared<TaskRespondGetInfo<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>>>(unique_id, name, 1000, 0, adapters);

    // Initial state
    CHECK(registration_manager.getSubscriptions().size() == 0);

    // Register the task
    task_sendheartbeat->registerTask(&registration_manager, task_sendheartbeat);
    CHECK(registration_manager.getServers().size() == 1);
    CHECK(registration_manager.getServers().containsIf([&](const CyphalPortID& port_id) {
        return port_id == uavcan_node_GetInfo_1_0_FIXED_PORT_ID_;
    }));
    // Unregister the task
    task_sendheartbeat->unregisterTask(&registration_manager, task_sendheartbeat);
    CHECK(registration_manager.getSubscriptions().size() == 0); // Subscription removed
}

