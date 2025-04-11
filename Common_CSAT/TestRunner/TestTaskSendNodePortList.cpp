#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "TaskSendNodePortList.hpp"
#include "Task.hpp"
#include "cyphal.hpp"
#include <tuple>
#include <memory>
#include "uavcan/node/port/List_1_0.h"
#include "loopard_adapter.hpp"
#include "RegistrationManager.hpp"
#include "ServiceManager.hpp"
#include "Allocator.hpp"
#include "TaskSendHeartBeat.hpp" // Assuming TaskSendHeartBeat exists for creating publications

#ifdef __x86_64__
#include "mock_hal.h"  // Include the mock HAL header
#endif

void *loopardMemoryAllocate(size_t amount) { return static_cast<void *>(malloc(amount)); };
void loopardMemoryFree(void *pointer) { free(pointer); };

TEST_CASE("TaskSendNodePortList: handleTask publishes NodePortList") {
    // Set up mock environment
    set_current_tick(10240); // Use the mock HAL tick function

    constexpr CyphalNodeID id1 = 11;
    constexpr CyphalNodeID id2 = 12;

    // Create adapters
    LoopardAdapter loopard1, loopard2;
    loopard1.memory_allocate = loopardMemoryAllocate;
    loopard1.memory_free = loopardMemoryFree;
    loopard2.memory_allocate = loopardMemoryAllocate;
    loopard2.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal1(&loopard1), loopard_cyphal2(&loopard2);
    loopard_cyphal1.setNodeID(id1);
    loopard_cyphal2.setNodeID(id2);
    
    std::tuple<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>> adapters(loopard_cyphal1, loopard_cyphal2);

    // Create a RegistrationManager
    RegistrationManager registration_manager;

    // Create a TaskSendHeartBeat (as a publisher example) and register it
    auto heartbeat_task = std::make_shared<TaskSendHeartBeat<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>>>(1000, 0, 0, adapters);
    registration_manager.add(heartbeat_task);

    // Instantiate the TaskSendNodePortList
    auto task_sendnodeportlist = std::make_shared<TaskSendNodePortList<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>>>(&registration_manager, 1000, 0, 0, adapters); // Interval: 1 second, shift: 0, transfer ID: 0
    registration_manager.add(task_sendnodeportlist);

    // Execute the task
    task_sendnodeportlist->handleTaskImpl();

    // Check that a NodePortList message was published on both adapters
    REQUIRE(loopard1.buffer.size() == 1);
    REQUIRE(loopard2.buffer.size() == 1);

    // Verify the content of the published message on the first adapter
    CyphalTransfer transfer1 = loopard1.buffer.pop();
    REQUIRE(transfer1.metadata.port_id == uavcan_node_port_List_1_0_FIXED_PORT_ID_);
    REQUIRE(transfer1.metadata.transfer_kind == CyphalTransferKindMessage);
    REQUIRE(transfer1.metadata.remote_node_id == id1);
    REQUIRE(transfer1.payload_size <= uavcan_node_port_List_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_); // Payload size may vary depending on content

    // Deserialize the payload and verify the values.  Important! This section needs adjustment 
    // based on the specifics of the NodePortList message and the number of publications/subscriptions

    uavcan_node_port_List_1_0 received_port_list1;
    size_t deserialized_size1 = transfer1.payload_size;
    int8_t deserialization_result1 = uavcan_node_port_List_1_0_deserialize_(&received_port_list1, static_cast<const uint8_t*>(transfer1.payload), &deserialized_size1);

    REQUIRE(deserialization_result1 >= 0);
    REQUIRE(received_port_list1.publishers.sparse_list.count == 2);  // Expecting 1 publisher now!
    REQUIRE(received_port_list1.subscribers.sparse_list.count == 0); // Expecting one subscriber

    // The first publisher should be Heartbeat
    REQUIRE(received_port_list1.publishers.sparse_list.elements[0].value == uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
    REQUIRE(received_port_list1.publishers.sparse_list.elements[1].value == uavcan_node_port_List_1_0_FIXED_PORT_ID_);

    delete[] static_cast<uint8_t*>(transfer1.payload);

    // Verify the content of the published message on the second adapter (similar to first adapter)
    CyphalTransfer transfer2 = loopard2.buffer.pop();
    REQUIRE(transfer2.metadata.port_id == uavcan_node_port_List_1_0_FIXED_PORT_ID_);
    REQUIRE(transfer2.metadata.transfer_kind == CyphalTransferKindMessage);
    REQUIRE(transfer2.metadata.remote_node_id == id2);
    REQUIRE(transfer2.payload_size <= uavcan_node_port_List_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_);

    uavcan_node_port_List_1_0 received_port_list2;
    size_t deserialized_size2 = transfer2.payload_size;
    int8_t deserialization_result2 = uavcan_node_port_List_1_0_deserialize_(&received_port_list2, static_cast<const uint8_t*>(transfer2.payload), &deserialized_size2);

    REQUIRE(deserialization_result2 >= 0);
    REQUIRE(received_port_list2.publishers.sparse_list.count == 2);  // Expecting 1 publisher now!
    REQUIRE(received_port_list2.subscribers.sparse_list.count == 0); // Expecting one subscriber

    // The first publisher should be Heartbeat
    REQUIRE(received_port_list2.publishers.sparse_list.elements[0].value == uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
    REQUIRE(received_port_list2.publishers.sparse_list.elements[1].value == uavcan_node_port_List_1_0_FIXED_PORT_ID_);
    delete[] static_cast<uint8_t*>(transfer2.payload);
}

TEST_CASE("TaskSendNodePortList: snippet to registration with std::alloc") {
    constexpr CyphalNodeID id1 = 11;
    constexpr CyphalNodeID id2 = 12;

    LoopardAdapter loopard1, loopard2;
    loopard1.memory_allocate = loopardMemoryAllocate;
    loopard1.memory_free = loopardMemoryFree;
    loopard2.memory_allocate = loopardMemoryAllocate;
    loopard2.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal1(&loopard1), loopard_cyphal2(&loopard2);
    loopard_cyphal1.setNodeID(id1);
    loopard_cyphal2.setNodeID(id2);
    
    std::tuple<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>> adapters(loopard_cyphal1, loopard_cyphal2);

    RegistrationManager registration_manager;
    auto task_sendheartbeat = std::make_shared<TaskSendHeartBeat<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>>>(1000, 0, 0, adapters);
    registration_manager.add(task_sendheartbeat);

    auto task_sendnodeportlist = std::make_shared<TaskSendNodePortList<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>>>(&registration_manager, 1000, 0, 0, adapters);
    REQUIRE(task_sendnodeportlist.use_count() == 1);

    registration_manager.add(task_sendnodeportlist);
    REQUIRE(task_sendnodeportlist.use_count() == 2);
    
    REQUIRE(registration_manager.containsTask(task_sendnodeportlist));

    registration_manager.remove(task_sendnodeportlist);
    REQUIRE(! registration_manager.containsTask(task_sendnodeportlist));
    REQUIRE(task_sendnodeportlist.use_count() == 1);

    registration_manager.remove(task_sendheartbeat);
}

TEST_CASE("TaskSendNodePortList: snippet to registration with O1HeapAllocator") {
   constexpr CyphalNodeID id1 = 11;
    constexpr CyphalNodeID id2 = 12;

    char buffer[4192] __attribute__ ((aligned (256)));
    O1HeapInstance* heap = o1heapInit(buffer, 4192);
    REQUIRE(heap != nullptr);
    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(heap);
    size_t allocated = diagnostics.allocated;

    O1HeapAllocator<TaskSendHeartBeat<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>>> heartbeatAllocator(heap);
    O1HeapAllocator<TaskSendNodePortList<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>>> taskAllocator(heap);

    LoopardAdapter loopard1, loopard2;
    loopard1.memory_allocate = loopardMemoryAllocate;
    loopard1.memory_free = loopardMemoryFree;
    loopard2.memory_allocate = loopardMemoryAllocate;
    loopard2.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal1(&loopard1), loopard_cyphal2(&loopard2);
    loopard_cyphal1.setNodeID(id1);
    loopard_cyphal2.setNodeID(id2);

    std::tuple<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>> adapters(loopard_cyphal1, loopard_cyphal2);

    RegistrationManager registration_manager;
    auto heartbeat_task = allocate_shared_custom<TaskSendHeartBeat<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>>>(
        heartbeatAllocator, 1000, 0, 0, adapters);
    registration_manager.add(heartbeat_task);

    auto task_sendnodeportlist = allocate_shared_custom<TaskSendNodePortList<Cyphal<LoopardAdapter>, Cyphal<LoopardAdapter>>>(
        taskAllocator, &registration_manager, 1000, 0, 0, adapters);
    diagnostics = o1heapGetDiagnostics(heap);
    CHECK(diagnostics.allocated > allocated);
    REQUIRE(task_sendnodeportlist.use_count() == 1);

    registration_manager.add(task_sendnodeportlist);
    REQUIRE(registration_manager.containsTask(task_sendnodeportlist));
    REQUIRE(task_sendnodeportlist.use_count() == 2);

    registration_manager.remove(task_sendnodeportlist);
    REQUIRE(! registration_manager.containsTask(task_sendnodeportlist));    
    REQUIRE(task_sendnodeportlist.use_count() == 1);
    task_sendnodeportlist.reset();

    registration_manager.remove(heartbeat_task);
    heartbeat_task.reset();

    diagnostics = o1heapGetDiagnostics(heap);
    CHECK(diagnostics.allocated == allocated);
}