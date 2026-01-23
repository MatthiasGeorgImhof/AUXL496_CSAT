#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "TaskSendHeartBeat.hpp"
#include "Task.hpp"
#include "cyphal.hpp"
#include <tuple>
#include <memory>
#include "uavcan/node/Heartbeat_1_0.h" 
#include "loopard_adapter.hpp"
#include "RegistrationManager.hpp"
#include "ServiceManager.hpp"
#include "HeapAllocation.hpp"

#ifdef __x86_64__
#include "mock_hal.h"  // Include the mock HAL header
#endif

using Heap = HeapAllocation<>;
using TaskAlloc = SafeAllocator<TaskSendHeartBeat<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>>,Heap>;

TEST_CASE("TaskSendHeartBeat: handleTask publishes Heartbeat") {
    // Set up mock environment
    set_current_tick(10240); // Use the mock HAL tick function

    constexpr CyphalNodeID id1 = 11;
    constexpr CyphalNodeID id2 = 12;

    Heap::initialize();

    // Create adapters
    LoopardAdapter loopard1, loopard2;
    loopard1.memory_allocate = Heap::loopardMemoryAllocate;
    loopard1.memory_free = Heap::loopardMemoryDeallocate;
    loopard2.memory_allocate = Heap::loopardMemoryAllocate;
    loopard2.memory_free = Heap::loopardMemoryDeallocate;
    Cyphal<LoopardAdapter> loopard_cyphal1(&loopard1), loopard_cyphal2(&loopard2);
    loopard_cyphal1.setNodeID(id1);
    loopard_cyphal2.setNodeID(id2);
    
    std::tuple<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>> adapters(loopard_cyphal1, loopard_cyphal2);

    // Instantiate the TaskSendHeartBeat
    TaskSendHeartBeat task(1000, 0, 0, adapters); // Interval: 1 second, shift: 0, transfer ID: 0

    // Execute the task
    task.handleTask();

    // Check that a heartbeat message was published on both adapters
    REQUIRE(loopard1.buffer.size() == 1);
    REQUIRE(loopard2.buffer.size() == 1);

    // Verify the content of the published message on the first adapter
    CyphalTransfer transfer1 = loopard1.buffer.pop();
    REQUIRE(transfer1.metadata.port_id == uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
    REQUIRE(transfer1.metadata.transfer_kind == CyphalTransferKindMessage);
    REQUIRE(transfer1.metadata.remote_node_id == id1);
    REQUIRE(transfer1.payload_size == uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_);

    // Deserialize the payload and verify the values
    uavcan_node_Heartbeat_1_0 received_heartbeat1;
    size_t deserialized_size1 = transfer1.payload_size;
    int8_t deserialization_result1 = uavcan_node_Heartbeat_1_0_deserialize_(&received_heartbeat1, static_cast<const uint8_t*>(transfer1.payload), &deserialized_size1);
    REQUIRE(deserialization_result1 >= 0); // Check if deserialization was successful
    REQUIRE(received_heartbeat1.uptime == 10); // HAL_GetTick() / 1024 = 10240 / 1024 = 10
    REQUIRE(received_heartbeat1.health.value == uavcan_node_Health_1_0_NOMINAL);
    REQUIRE(received_heartbeat1.mode.value == uavcan_node_Mode_1_0_OPERATIONAL);
    Heap::loopardMemoryDeallocate(transfer1.payload);

    // Verify the content of the published message on the second adapter
    CyphalTransfer transfer2 = loopard2.buffer.pop();
    REQUIRE(transfer2.metadata.port_id == uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
    REQUIRE(transfer2.metadata.transfer_kind == CyphalTransferKindMessage);
    REQUIRE(transfer2.metadata.remote_node_id == id2);
    REQUIRE(transfer2.payload_size == uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_);

    // Deserialize the payload and verify the values
    uavcan_node_Heartbeat_1_0 received_heartbeat2;
    size_t deserialized_size2 = transfer2.payload_size;
    int8_t deserialization_result2 = uavcan_node_Heartbeat_1_0_deserialize_(&received_heartbeat2, static_cast<const uint8_t*>(transfer2.payload), &deserialized_size2);
    REQUIRE(deserialization_result2 >= 0); // Check if deserialization was successful
    REQUIRE(received_heartbeat2.uptime == 10); // HAL_GetTick() / 1024 = 10240 / 1024 = 10
    REQUIRE(received_heartbeat2.health.value == uavcan_node_Health_1_0_NOMINAL);
    REQUIRE(received_heartbeat2.mode.value == uavcan_node_Mode_1_0_OPERATIONAL);
    Heap::loopardMemoryDeallocate(transfer2.payload);
}


TEST_CASE("TaskSendHeartBeat: snippet to registration with std::alloc") {
    constexpr CyphalNodeID id1 = 11;
    constexpr CyphalNodeID id2 = 12;

    LoopardAdapter loopard1, loopard2;
    Cyphal<LoopardAdapter> loopard_cyphal1(&loopard1), loopard_cyphal2(&loopard2);
    loopard_cyphal1.setNodeID(id1);
    loopard_cyphal2.setNodeID(id2);
    
    std::tuple<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>> adapters(loopard_cyphal1, loopard_cyphal2);
    auto task_sendheartbeat = std::make_shared<TaskSendHeartBeat<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>>>(1000, 0, 0, adapters);
    CHECK(task_sendheartbeat.use_count() == 1);

    RegistrationManager registration_manager;
    registration_manager.add(task_sendheartbeat);
    CHECK(task_sendheartbeat.use_count() == 2);

    CHECK(registration_manager.containsTask(task_sendheartbeat));

    registration_manager.remove(task_sendheartbeat);
    CHECK(! registration_manager.containsTask(task_sendheartbeat));
    CHECK(task_sendheartbeat.use_count() == 1);
}


TEST_CASE("TaskSendHeartBeat: snippet to registration with SafeAllocator") {
    constexpr CyphalNodeID id1 = 11;
    constexpr CyphalNodeID id2 = 12;

    Heap::initialize();
    auto diagnostics = Heap::getDiagnostics();
    size_t allocated = diagnostics.allocated;

    TaskAlloc taskAllocator;  // no 'heap' variable; allocator uses Heap statically

    LoopardAdapter loopard1, loopard2;
    loopard1.memory_allocate = Heap::loopardMemoryAllocate;
    loopard1.memory_free     = Heap::loopardMemoryDeallocate;
    loopard2.memory_allocate = Heap::loopardMemoryAllocate;
    loopard2.memory_free     = Heap::loopardMemoryDeallocate;

    Cyphal<LoopardAdapter> loopard_cyphal1(&loopard1), loopard_cyphal2(&loopard2);
    loopard_cyphal1.setNodeID(id1);
    loopard_cyphal2.setNodeID(id2);

    std::tuple<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>> adapters(loopard_cyphal1, loopard_cyphal2);

    auto task_sendheartbeat =
        alloc_shared_custom<TaskSendHeartBeat<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>>>(
            taskAllocator, 1000, 0, 0, adapters);

    diagnostics = Heap::getDiagnostics();
    CHECK(diagnostics.allocated > allocated);
    CHECK(task_sendheartbeat.use_count() == 1);

    RegistrationManager registration_manager;
    registration_manager.add(task_sendheartbeat);
    CHECK(registration_manager.containsTask(task_sendheartbeat));
    CHECK(task_sendheartbeat.use_count() == 2);

    registration_manager.remove(task_sendheartbeat);
    CHECK(!registration_manager.containsTask(task_sendheartbeat));
    CHECK(task_sendheartbeat.use_count() == 1);

    task_sendheartbeat.reset();
    diagnostics = Heap::getDiagnostics();
    CHECK(diagnostics.allocated == allocated);
}
