#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "TaskSendNodePortList.hpp"
#include "Task.hpp"
#include "cyphal.hpp"
#include <tuple>
#include <memory>
#include "loopard_adapter.hpp"
#include "canard_adapter.hpp"
#include "RegistrationManager.hpp"
#include "SubscriptionManager.hpp"
#include "ServiceManager.hpp"
#include "Allocator.hpp"
#include "TaskCheckMemory.hpp"
#include "TaskBlinkLED.hpp"
#include "TaskSendHeartBeat.hpp"
#include "TaskSendNodePortList.hpp"
#include "TaskSubscribeNodePortList.hpp"
#include "cyphal_subscriptions.hpp"

#include "nunavut/support/serialization.h"

#ifdef __x86_64__
#include "mock_hal.h" // Include the mock HAL header
#endif

void *loopardMemoryAllocate(size_t amount) { return static_cast<void *>(malloc(amount)); };
void loopardMemoryFree(void *pointer) { free(pointer); };

void *canardMemoryAllocate(CanardInstance */*ins*/, size_t amount) { return static_cast<void *>(malloc(amount)); };
void canardMemoryFree(CanardInstance */*ins*/, void *pointer) { free(pointer); };

TEST_CASE("TaskSendNodePortList: handleTask publishes NodePortList")
{
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
    REQUIRE(transfer1.payload_size <= uavcan_node_port_List_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_);

    uavcan_node_port_List_1_0 received_port_list1;
    size_t deserialized_size1 = transfer1.payload_size;
    int8_t deserialization_result1 = uavcan_node_port_List_1_0_deserialize_(&received_port_list1, static_cast<const uint8_t *>(transfer1.payload), &deserialized_size1);

    REQUIRE(deserialization_result1 >= 0);
    REQUIRE(received_port_list1.publishers.sparse_list.count == 2);
    REQUIRE(received_port_list1.subscribers.sparse_list.count == 0);

    // The first publisher should be Heartbeat
    REQUIRE(received_port_list1.publishers.sparse_list.elements[0].value == uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
    REQUIRE(received_port_list1.publishers.sparse_list.elements[1].value == uavcan_node_port_List_1_0_FIXED_PORT_ID_);

    delete[] static_cast<uint8_t *>(transfer1.payload);

    // Verify the content of the published message on the second adapter (similar to first adapter)
    CyphalTransfer transfer2 = loopard2.buffer.pop();
    REQUIRE(transfer2.metadata.port_id == uavcan_node_port_List_1_0_FIXED_PORT_ID_);
    REQUIRE(transfer2.metadata.transfer_kind == CyphalTransferKindMessage);
    REQUIRE(transfer2.metadata.remote_node_id == id2);
    REQUIRE(transfer2.payload_size <= uavcan_node_port_List_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_);

    uavcan_node_port_List_1_0 received_port_list2;
    size_t deserialized_size2 = transfer2.payload_size;
    int8_t deserialization_result2 = uavcan_node_port_List_1_0_deserialize_(&received_port_list2, static_cast<const uint8_t *>(transfer2.payload), &deserialized_size2);

    REQUIRE(deserialization_result2 >= 0);
    REQUIRE(received_port_list2.publishers.sparse_list.count == 2);
    REQUIRE(received_port_list2.subscribers.sparse_list.count == 0); // Expecting one subscriber

    // The first publisher should be Heartbeat
    REQUIRE(received_port_list2.publishers.sparse_list.elements[0].value == uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
    REQUIRE(received_port_list2.publishers.sparse_list.elements[1].value == uavcan_node_port_List_1_0_FIXED_PORT_ID_);
    delete[] static_cast<uint8_t *>(transfer2.payload);
}

TEST_CASE("TaskSendNodePortList: snippet to registration with std::alloc")
{
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
    REQUIRE(!registration_manager.containsTask(task_sendnodeportlist));
    REQUIRE(task_sendnodeportlist.use_count() == 1);

    registration_manager.remove(task_sendheartbeat);
}

TEST_CASE("TaskSendNodePortList: snippet to registration with O1HeapAllocator")
{
    constexpr CyphalNodeID id1 = 11;
    constexpr CyphalNodeID id2 = 12;

    char buffer[4192] __attribute__((aligned(256)));
    O1HeapInstance *heap = o1heapInit(buffer, 4192);
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
    REQUIRE(!registration_manager.containsTask(task_sendnodeportlist));
    REQUIRE(task_sendnodeportlist.use_count() == 1);
    task_sendnodeportlist.reset();

    registration_manager.remove(heartbeat_task);
    heartbeat_task.reset();

    diagnostics = o1heapGetDiagnostics(heap);
    CHECK(diagnostics.allocated == allocated);
}

TEST_CASE("TaskSendNodePortList: main loop snippet")
{
    constexpr CyphalNodeID cyphal_node_id = 11;
    GPIO_TypeDef GPIOC;
    uint16_t LED1_Pin = 1;

    char buffer[4192] __attribute__((aligned(256)));
    O1HeapInstance *heap = o1heapInit(buffer, 4192);
    REQUIRE(heap != nullptr);
    O1HeapAllocator<CanardRxTransfer> alloc(heap);
    
    LoopardAdapter loopard_adapter;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard_adapter);
    loopard_cyphal.setNodeID(cyphal_node_id);

    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);

    RegistrationManager registration_manager;
    SubscriptionManager subscription_manager;
    registration_manager.subscribe(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
    registration_manager.subscribe(uavcan_node_port_List_1_0_FIXED_PORT_ID_);
    registration_manager.subscribe(uavcan_diagnostic_Record_1_1_FIXED_PORT_ID_);
    registration_manager.publish(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
    registration_manager.publish(uavcan_node_port_List_1_0_FIXED_PORT_ID_);
    registration_manager.publish(uavcan_diagnostic_Record_1_1_FIXED_PORT_ID_);

    O1HeapAllocator<TaskSendHeartBeat<Cyphal<LoopardAdapter>>> alloc_TaskSendHeartBeat(heap);
    registration_manager.add(allocate_unique_custom<TaskSendHeartBeat<Cyphal<LoopardAdapter>>>(alloc_TaskSendHeartBeat, 1000, 100, 0, adapters));

    O1HeapAllocator<TaskSendNodePortList<Cyphal<LoopardAdapter>>> alloc_TaskSendNodePortList(heap);
    registration_manager.add(allocate_unique_custom<TaskSendNodePortList<Cyphal<LoopardAdapter>>>(alloc_TaskSendNodePortList, &registration_manager, 10000, 100, 0, adapters));

    O1HeapAllocator<TaskSubscribeNodePortList<Cyphal<LoopardAdapter>>> alloc_TaskSubscribeNodePortList(heap);
    registration_manager.add(allocate_unique_custom<TaskSubscribeNodePortList<Cyphal<LoopardAdapter>>>(alloc_TaskSubscribeNodePortList, &subscription_manager, 10000, 100, adapters));

    O1HeapAllocator<TaskBlinkLED> alloc_TaskBlinkLED(heap);
    registration_manager.add(allocate_unique_custom<TaskBlinkLED>(alloc_TaskBlinkLED, &GPIOC, LED1_Pin, 1000, 100));

    O1HeapAllocator<TaskCheckMemory> alloc_TaskCheckMemory(heap);
    registration_manager.add(allocate_unique_custom<TaskCheckMemory>(alloc_TaskCheckMemory, heap, 2000, 100));

    auto subscriptions = registration_manager.getSubscriptions();
    REQUIRE(subscriptions.size() == 3);
    CHECK(subscriptions.containsIf([](CyphalPortID port_id)
                                   { return port_id == uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_; }));
    CHECK(subscriptions.containsIf([](CyphalPortID port_id)
                                   { return port_id == uavcan_node_port_List_1_0_FIXED_PORT_ID_; }));
    CHECK(subscriptions.containsIf([](CyphalPortID port_id)
                                   { return port_id == uavcan_diagnostic_Record_1_1_FIXED_PORT_ID_; }));

    auto publications = registration_manager.getPublications();
    REQUIRE(publications.size() == 3);
    CHECK(publications.containsIf([](CyphalPortID port_id)
                                  { return port_id == uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_; }));
    CHECK(publications.containsIf([](CyphalPortID port_id)
                                  { return port_id == uavcan_node_port_List_1_0_FIXED_PORT_ID_; }));
    CHECK(publications.containsIf([](CyphalPortID port_id)
                                  { return port_id == uavcan_diagnostic_Record_1_1_FIXED_PORT_ID_; }));
}

TEST_CASE("TaskSendNodePortList: serialize deserialize Loopard")
{
    constexpr CyphalNodeID cyphal_node_id = 11;

    char buffer[4192] __attribute__((aligned(256)));
    O1HeapInstance *heap = o1heapInit(buffer, 4192);
    REQUIRE(heap != nullptr);
    O1HeapAllocator<CanardRxTransfer> alloc(heap);

    LoopardAdapter loopard_adapter;
    loopard_adapter.memory_allocate = loopardMemoryAllocate;
    loopard_adapter.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard_adapter);
    loopard_cyphal.setNodeID(cyphal_node_id);

    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);

    RegistrationManager registration_manager;
    SubscriptionManager subscription_manager;
    registration_manager.subscribe(1000);
    registration_manager.subscribe(1001);
    registration_manager.subscribe(1002);
    registration_manager.publish(2000);
    registration_manager.publish(2001);
    registration_manager.publish(2002);
    registration_manager.client(100);
    registration_manager.client(101);
    registration_manager.client(102);
    registration_manager.server(200);
    registration_manager.server(201);
    registration_manager.server(202);

    TaskSendNodePortList task = TaskSendNodePortList<Cyphal<LoopardAdapter>>(&registration_manager, 10000, 100, 0, adapters);
    task.handleTaskImpl();

    CyphalTransfer transfer = {};
    size_t payload_size = 0;
    CHECK(loopard_cyphal.cyphalRxReceive(nullptr, &payload_size, &transfer) == 1);
    CHECK(transfer.payload_size <= uavcan_node_port_List_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_);
    CHECK(transfer.payload_size == 160);
    CHECK(transfer.metadata.port_id <= uavcan_node_port_List_1_0_FIXED_PORT_ID_);

    uavcan_node_port_List_1_0 data;
    REQUIRE(uavcan_node_port_List_1_0_deserialize_(&data, static_cast<const uint8_t *>(transfer.payload), &transfer.payload_size) == 0);

    CHECK(data.publishers.sparse_list.count == 3);
    CHECK(data.subscribers.sparse_list.count == 3);
    CHECK(data.subscribers.sparse_list.elements[0].value == 1000);
    CHECK(data.subscribers.sparse_list.elements[1].value == 1001);
    CHECK(data.subscribers.sparse_list.elements[2].value == 1002);
    CHECK(data.publishers.sparse_list.count == 3);
    CHECK(data.publishers.sparse_list.elements[0].value == 2000);
    CHECK(data.publishers.sparse_list.elements[1].value == 2001);
    CHECK(data.publishers.sparse_list.elements[2].value == 2002);

    for(size_t i = 0; i < 512; ++i)
    {
        CHECK(nunavutGetBit(data.clients.mask_bitpacked_, sizeof(data.clients.mask_bitpacked_), i) == (i>=100 && i<=102));
        CHECK(nunavutGetBit(data.servers.mask_bitpacked_, sizeof(data.servers.mask_bitpacked_), i) == (i>=200 && i<=202));
    }
}

TEST_CASE("TaskSendNodePortList: nunavut serialize deserialize SparseList")
{
    uavcan_node_port_List_1_0 data1, data2;

    uavcan_node_port_SubjectIDList_1_0_select_sparse_list_(&data1.subscribers);
    data1.subscribers.sparse_list.elements[0].value = 1000;
    data1.subscribers.sparse_list.elements[1].value = 1001;
    data1.subscribers.sparse_list.elements[2].value = 1002;
    data1.subscribers.sparse_list.count = 3;

    uavcan_node_port_SubjectIDList_1_0_select_sparse_list_(&data1.publishers);
    data1.publishers.sparse_list.elements[0].value = 2000;
    data1.publishers.sparse_list.elements[1].value = 2001;
    data1.publishers.sparse_list.elements[2].value = 2002;
    data1.publishers.sparse_list.count = 3;

    memset(data1.servers.mask_bitpacked_, 0, 64);
    memset(data1.clients.mask_bitpacked_, 0, 64);

    constexpr size_t PAYLOAD_SIZE = uavcan_node_port_List_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    size_t payload_size1 = PAYLOAD_SIZE;
    uint8_t payload[PAYLOAD_SIZE];
    uavcan_node_port_List_1_0_serialize_(&data1, payload, &payload_size1);
    CHECK(payload_size1 <= PAYLOAD_SIZE);

    size_t payload_size2 = PAYLOAD_SIZE;
    int8_t result = uavcan_node_port_List_1_0_deserialize_(&data2, payload, &payload_size2);
    CHECK(result == 0);
    CHECK(payload_size2 <= PAYLOAD_SIZE);

    CHECK(payload_size1 == payload_size2);

    CHECK(data1.publishers.sparse_list.count == data2.publishers.sparse_list.count);
    CHECK(data1.subscribers.sparse_list.count == data2.subscribers.sparse_list.count);
    CHECK(data1.publishers.sparse_list.elements[0].value == data2.publishers.sparse_list.elements[0].value);
    CHECK(data1.publishers.sparse_list.elements[1].value == data2.publishers.sparse_list.elements[1].value);
    CHECK(data1.publishers.sparse_list.elements[2].value == data2.publishers.sparse_list.elements[2].value);
    CHECK(data1.subscribers.sparse_list.elements[0].value == data2.subscribers.sparse_list.elements[0].value);
    CHECK(data1.subscribers.sparse_list.elements[1].value == data2.subscribers.sparse_list.elements[1].value);
    CHECK(data1.subscribers.sparse_list.elements[2].value == data2.subscribers.sparse_list.elements[2].value);
    CHECK(data1.servers.mask_bitpacked_[0] == data2.servers.mask_bitpacked_[0]);
    CHECK(data1.servers.mask_bitpacked_[1] == data2.servers.mask_bitpacked_[1]);
    CHECK(data1.servers.mask_bitpacked_[2] == data2.servers.mask_bitpacked_[2]);
    CHECK(data1.clients.mask_bitpacked_[0] == data2.clients.mask_bitpacked_[0]);
    CHECK(data1.clients.mask_bitpacked_[1] == data2.clients.mask_bitpacked_[1]);
    CHECK(data1.clients.mask_bitpacked_[2] == data2.clients.mask_bitpacked_[2]);
}

TEST_CASE("TaskSendNodePortList: nunavut serialize deserialize MaskedList")
{
    uavcan_node_port_List_1_0 data1, data2;

    uavcan_node_port_SubjectIDList_1_0_select_mask_(&data1.subscribers);
    data1.subscribers.mask_bitpacked_[0] = 100;
    data1.subscribers.mask_bitpacked_[1] = 101;
    data1.subscribers.mask_bitpacked_[2] = 102;

    uavcan_node_port_SubjectIDList_1_0_select_mask_(&data1.publishers);
    data1.publishers.mask_bitpacked_[0] = 200;
    data1.publishers.mask_bitpacked_[1] = 201;
    data1.publishers.mask_bitpacked_[2] = 202;

    memset(data1.servers.mask_bitpacked_, 0x0f, 64);
    memset(data1.clients.mask_bitpacked_, 0xf0, 64);

    constexpr size_t PAYLOAD_SIZE = uavcan_node_port_List_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    size_t payload_size1 = PAYLOAD_SIZE;
    uint8_t payload[PAYLOAD_SIZE];
    uavcan_node_port_List_1_0_serialize_(&data1, payload, &payload_size1);
    CHECK(payload_size1 <= PAYLOAD_SIZE);

    size_t payload_size2 = PAYLOAD_SIZE;
    int8_t result = uavcan_node_port_List_1_0_deserialize_(&data2, payload, &payload_size2);
    CHECK(result == 0);
    CHECK(payload_size2 <= PAYLOAD_SIZE);

    CHECK(payload_size1 == payload_size2);

    CHECK(data1.publishers.mask_bitpacked_[0] == data2.publishers.mask_bitpacked_[0]);
    CHECK(data1.publishers.mask_bitpacked_[1] == data2.publishers.mask_bitpacked_[1]);
    CHECK(data1.publishers.mask_bitpacked_[2] == data2.publishers.mask_bitpacked_[2]);
    CHECK(data1.subscribers.mask_bitpacked_[0] == data2.subscribers.mask_bitpacked_[0]);
    CHECK(data1.subscribers.mask_bitpacked_[1] == data2.subscribers.mask_bitpacked_[1]);
    CHECK(data1.subscribers.mask_bitpacked_[2] == data2.subscribers.mask_bitpacked_[2]);
    CHECK(data1.servers.mask_bitpacked_[0] == data2.servers.mask_bitpacked_[0]);
    CHECK(data1.servers.mask_bitpacked_[1] == data2.servers.mask_bitpacked_[1]);
    CHECK(data1.servers.mask_bitpacked_[2] == data2.servers.mask_bitpacked_[2]);
    CHECK(data1.clients.mask_bitpacked_[0] == data2.clients.mask_bitpacked_[0]);
    CHECK(data1.clients.mask_bitpacked_[1] == data2.clients.mask_bitpacked_[1]);
    CHECK(data1.clients.mask_bitpacked_[2] == data2.clients.mask_bitpacked_[2]);
}
