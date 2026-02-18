#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "loopard_adapter.hpp"
#include "HeapAllocation.hpp"
#include "o1heap.h"
#include "ProcessRxQueue.hpp"
#include "RegistrationManager.hpp"
#include "SubscriptionManager.hpp"
#include "ServiceManager.hpp"
#include "CanTxQueueDrainer.hpp"

CanTxQueueDrainer::CanTxQueueDrainer(CanardAdapter* /*adapter*/, CAN_HandleTypeDef* /*hcan*/) {}

void CanTxQueueDrainer::drain() {}

void CanTxQueueDrainer::irq_safe_drain() {}

CanTxQueueDrainer tx_drainer{nullptr, nullptr};

// ------------------------------------------------------------
// Local heap adapter for SafeAllocator
// ------------------------------------------------------------
// static O1HeapInstance* heap_for_allocator = nullptr;

// struct LocalHeap {
//     static void* heapAllocate(void*, size_t amount) {
//         return o1heapAllocate(heap_for_allocator, amount);
//     }
//     static void heapFree(void*, void* ptr) {
//         o1heapFree(heap_for_allocator, ptr);
//     }
// };

using Heap = HeapAllocation<>;
CanardAdapter canard_adapter;
LoopardAdapter loopard_adapter;
SerardAdapter serard_adapter;
// ------------------------------------------------------------
// Utility
// ------------------------------------------------------------
void checkTransfers(const CyphalTransfer& t1, const CyphalTransfer& t2)
{
    CHECK(t1.metadata.port_id == t2.metadata.port_id);
    CHECK(t1.payload_size == t2.payload_size);
}

void checkPayloads(const char* payload1, const char* payload2, size_t size)
{
    CHECK(strncmp(payload1, payload2, size) == 0);
}

// ------------------------------------------------------------
// Mock Tasks
// ------------------------------------------------------------
constexpr CyphalPortID CYPHALPORT = 129;

class MockTask : public Task
{
public:
    MockTask(uint32_t interval, uint32_t tick, const CyphalTransfer& transfer)
        : Task(interval, tick), transfer_(transfer) {}

    void handleMessage(std::shared_ptr<CyphalTransfer> transfer) override {
        checkTransfers(transfer_, *transfer);
    }

    void handleTaskImpl() override {}

    void registerTask(RegistrationManager* manager, std::shared_ptr<Task> task) override {
        manager->subscribe(CYPHALPORT, task);
    }

    void unregisterTask(RegistrationManager*, std::shared_ptr<Task>) override {}

    CyphalTransfer transfer_;
};

class MockTaskFromBuffer : public TaskFromBuffer<CyphalBuffer32>
{
public:
    MockTaskFromBuffer(uint32_t interval, uint32_t tick, const CyphalTransfer& transfer)
        : TaskFromBuffer(interval, tick), transfer_(transfer) {}

    void handleTaskImpl() override {
        // fprintf(stderr, "MockTaskFromBuffer::handleTaskImpl called\n");
        auto t = buffer_.pop();
        CHECK(t.use_count() == 1);
        checkTransfers(transfer_, *t);
    }

    void registerTask(RegistrationManager* manager, std::shared_ptr<Task> task) override {
        manager->subscribe(CYPHALPORT, task);
    }

    void unregisterTask(RegistrationManager*, std::shared_ptr<Task>) override {}

    const CyphalBuffer32& getBuffer() const { return buffer_; } 

    CyphalTransfer transfer_;
};

// ------------------------------------------------------------
// Tests
// ------------------------------------------------------------

TEST_CASE("processTransfer no forwarding")
{
    Heap::initialize();
    
    auto adapters = std::make_tuple();

    CyphalTransfer transfer;
    transfer.metadata.priority = CyphalPriorityNominal;
    transfer.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer.metadata.port_id = 123;
    transfer.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.transfer_id = 0;

    constexpr char payload[] = "hello";
    transfer.payload_size = sizeof(payload);
    transfer.payload = Heap::heapAllocate(nullptr, sizeof(payload));
    memcpy(transfer.payload, payload, sizeof(payload));

    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    auto task = std::make_shared<MockTask>(10, 0, transfer);
    handlers.push(TaskHandler{123, task});

    ServiceManager service_manager(handlers);

    SafeAllocator<CyphalTransfer, Heap> alloc;
    LoopManager loop_manager(alloc);

    bool result = loop_manager.processTransfer(transfer, &service_manager, adapters);

    CHECK(result == true);
    CHECK(task.use_count() == 2);
}

TEST_CASE("processTransfer with LoopardAdapter")
{
    Heap::initialize();

    LoopardAdapter adapter;
    adapter.memory_allocate = Heap::loopardMemoryAllocate;
    adapter.memory_free = Heap::loopardMemoryDeallocate;
    Cyphal<LoopardAdapter> cyphal(&adapter);
    auto adapters = std::make_tuple(cyphal);

    CyphalTransfer transfer;
    transfer.metadata.priority = CyphalPriorityNominal;
    transfer.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer.metadata.port_id = 123;
    transfer.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.transfer_id = 0;

    constexpr char payload[] = "hello";
    transfer.payload_size = sizeof(payload);
    transfer.payload = Heap::heapAllocate(nullptr, sizeof(payload));
    memcpy(transfer.payload, payload, sizeof(payload));

    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    auto task = std::make_shared<MockTask>(10, 0, transfer);
    handlers.push(TaskHandler{123, task});

    ServiceManager service_manager(handlers);

    SafeAllocator<CyphalTransfer, Heap> alloc;
    LoopManager loop_manager(alloc);

    bool result = loop_manager.processTransfer(transfer, &service_manager, adapters);

    CHECK(result == true);
    CHECK(adapter.buffer.size() == 1);

    CyphalTransfer received;
    size_t frame_size = 0;
    int32_t rx = cyphal.cyphalRxReceive(nullptr, &frame_size, &received);
    CHECK(rx == 1);

    checkTransfers(transfer, received);
    checkPayloads(payload, static_cast<const char*>(received.payload), sizeof(payload));
}

void* canardMemoryAllocate(CanardInstance*, size_t amount) { return malloc(amount); }
void canardMemoryFree(CanardInstance*, void* ptr) { free(ptr); }

TEST_CASE("processTransfer with LoopardAdapter and CanardAdapter")
{
    Heap::initialize();

    LoopardAdapter loopard_adapter;
    loopard_adapter.memory_allocate = Heap::loopardMemoryAllocate;
    loopard_adapter.memory_free = Heap::loopardMemoryDeallocate;

    Cyphal<LoopardAdapter> loopard_cyphal(&loopard_adapter);

    CanardAdapter canard_adapter;
    canard_adapter.ins = canardInit(Heap::canardMemoryAllocate, Heap::canardMemoryDeallocate);
    canard_adapter.ins.node_id = 11;
    canard_adapter.que = canardTxInit(16, CANARD_MTU_CAN_CLASSIC);

    Cyphal<CanardAdapter> canard_cyphal(&canard_adapter);

    auto adapters = std::make_tuple(loopard_cyphal, canard_cyphal);

    CyphalTransfer transfer;
    transfer.metadata.priority = CyphalPriorityNominal;
    transfer.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer.metadata.port_id = 123;
    transfer.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.transfer_id = 0;

    constexpr char payload[] = "hello";
    transfer.payload_size = sizeof(payload);
    transfer.payload = Heap::heapAllocate(nullptr, sizeof(payload));
    memcpy(transfer.payload, payload, sizeof(payload));

    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    auto task = std::make_shared<MockTask>(10, 0, transfer);
    handlers.push(TaskHandler{123, task});

    ServiceManager service_manager(handlers);

    SafeAllocator<CyphalTransfer, Heap> alloc;
    LoopManager loop_manager(alloc);

    bool result = loop_manager.processTransfer(transfer, &service_manager, adapters);

    CHECK(result == true);
    CHECK(loopard_adapter.buffer.size() == 1);
    CHECK(canard_adapter.que.size > 0);

    // Loopard receive
    CyphalTransfer received_loopard;
    size_t frame_size_loopard = 0;
    int32_t rx_loopard = loopard_cyphal.cyphalRxReceive(nullptr, &frame_size_loopard, &received_loopard);
    CHECK(rx_loopard == 1);
    checkTransfers(transfer, received_loopard);
    checkTransfers(transfer, received_loopard);
    checkPayloads(payload, static_cast<const char*>(received_loopard.payload), sizeof(payload));

    // Canard receive
    CHECK(canard_cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 123, 100, 2000000) == 1);
    const CanardTxQueueItem* const_ptr = canardTxPeek(&canard_adapter.que);
    CHECK(const_ptr != nullptr);

    CanardTxQueueItem* ptr = canardTxPop(&canard_adapter.que, const_ptr);
    CHECK(ptr != nullptr);

    CyphalTransfer received_canard;
    CHECK(canard_cyphal.cyphalRxReceive(ptr->frame.extended_can_id,
                                        &ptr->frame.payload_size,
                                        (const uint8_t*)ptr->frame.payload,
                                        &received_canard) == 1);

    checkTransfers(transfer, received_canard);
    checkPayloads(payload, static_cast<const char*>(received_canard.payload), sizeof(payload));
}

TEST_CASE("CanProcessRxQueue with CanardAdapter and MockTask")
{
    constexpr CyphalPortID port_id = 123;
    constexpr CyphalNodeID node_id = 11;

    Heap::initialize();

    CanardAdapter canard_adapter;
    canard_adapter.ins = canardInit(Heap::canardMemoryAllocate, Heap::canardMemoryDeallocate);
    canard_adapter.ins.node_id = node_id;
    canard_adapter.que = canardTxInit(16, CANARD_MTU_CAN_CLASSIC);
    Cyphal<CanardAdapter> cyphal(&canard_adapter);

    auto adapters = std::make_tuple();

    CyphalTransfer transfer;
    transfer.metadata.priority = CyphalPriorityNominal;
    transfer.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer.metadata.port_id = port_id;
    transfer.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.transfer_id = 0;
    constexpr char payload[] = "hello";
    transfer.payload_size = sizeof(payload);
    transfer.payload = Heap::heapAllocate(nullptr, sizeof(payload));
    memcpy(transfer.payload, payload, sizeof(payload));

    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    auto task = std::make_shared<MockTask>(10, 0, transfer);
    handlers.push(TaskHandler{port_id, task});
    ServiceManager service_manager(handlers);

    CircularBuffer<CanRxFrame, 64> can_rx_buffer;

    CHECK(cyphal.cyphalTxPush(0, &transfer.metadata, transfer.payload_size, transfer.payload) == 1);

    auto diagnostics = Heap::getDiagnostics();
    size_t allocated_mem = diagnostics.allocated;

    SafeAllocator<CyphalTransfer, Heap> alloc;
    LoopManager loop_manager(alloc);
    loop_manager.CanProcessRxQueue(&cyphal, &service_manager, adapters, can_rx_buffer);

    CHECK(can_rx_buffer.size() == 0);
    diagnostics = Heap::getDiagnostics();
    CHECK(allocated_mem == diagnostics.allocated);
}

TEST_CASE("CanProcessRxQueue multiple frames")
{
    constexpr CyphalPortID port_id = 123;
    constexpr CyphalNodeID node_id = 11;

    Heap::initialize();

    CanardAdapter canard_adapter;
    canard_adapter.ins = canardInit(Heap::canardMemoryAllocate, Heap::canardMemoryDeallocate);
    canard_adapter.ins.node_id = node_id;
    canard_adapter.que = canardTxInit(16, CANARD_MTU_CAN_CLASSIC);
    Cyphal<CanardAdapter> cyphal(&canard_adapter);

    auto adapters = std::make_tuple();

    CyphalTransfer transfer1;
    transfer1.metadata.priority = CyphalPriorityNominal;
    transfer1.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer1.metadata.port_id = port_id;
    transfer1.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer1.metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    transfer1.metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    transfer1.metadata.transfer_id = 0;
    constexpr char payload1[] = "hello";
    transfer1.payload_size = sizeof(payload1);
    transfer1.payload = Heap::heapAllocate(nullptr, sizeof(payload1));
    memcpy(transfer1.payload, payload1, sizeof(payload1));

    CyphalTransfer transfer2;
    transfer2.metadata.priority = CyphalPriorityNominal;
    transfer2.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer2.metadata.port_id = port_id;
    transfer2.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer2.metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    transfer2.metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    transfer2.metadata.transfer_id = 1;
    constexpr char payload2[] = "world!";
    transfer2.payload_size = sizeof(payload2);
    transfer2.payload = Heap::heapAllocate(nullptr, sizeof(payload2));
    memcpy(transfer2.payload, payload2, sizeof(payload2));

    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    auto task1 = std::make_shared<MockTask>(10, 0, transfer1);
    handlers.push(TaskHandler{port_id, task1});
    auto task2 = std::make_shared<MockTask>(10, 0, transfer2);
    handlers.push(TaskHandler{port_id, task2});

    ServiceManager service_manager(handlers);

    CircularBuffer<CanRxFrame, 64> can_rx_buffer;

    CHECK(cyphal.cyphalTxPush(0, &transfer1.metadata, transfer1.payload_size, transfer1.payload) == 1);
    CHECK(cyphal.cyphalTxPush(0, &transfer2.metadata, transfer2.payload_size, transfer2.payload) == 1);

    auto diagnostics = Heap::getDiagnostics();
    size_t allocated_mem = diagnostics.allocated;

    SafeAllocator<CyphalTransfer, Heap> alloc;
    LoopManager loop_manager(alloc);
    loop_manager.CanProcessRxQueue(&cyphal, &service_manager, adapters, can_rx_buffer);

    CHECK(can_rx_buffer.size() == 0);
    diagnostics = Heap::getDiagnostics();
    CHECK(allocated_mem == diagnostics.allocated);
}

TEST_CASE("SerialProcessRxQueue with SerardAdapter and MockTask")
{
    constexpr CyphalPortID port_id = 123;
    constexpr CyphalNodeID node_id = 11;

    Heap::initialize();

    SerardAdapter serard_adapter;
    SerardMemoryResource serard_memory_resource = {&serard_adapter.ins, Heap::serardMemoryDeallocate, Heap::serardMemoryAllocate};
    serard_adapter.ins = serardInit(serard_memory_resource, serard_memory_resource);
    serard_adapter.ins.node_id = node_id;
    serard_adapter.user_reference = &serard_adapter.ins;
    serard_adapter.ins.user_reference = &serard_adapter.ins;
    serard_adapter.reass = serardReassemblerInit();
    serard_adapter.emitter = [](void*, uint8_t, const uint8_t*) -> bool { return true; };
    Cyphal<SerardAdapter> cyphal(&serard_adapter);

    auto adapters = std::make_tuple();

    CyphalTransfer transfer;
    transfer.metadata.priority = CyphalPriorityNominal;
    transfer.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer.metadata.port_id = port_id;
    transfer.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.transfer_id = 0;
    constexpr char payload[] = "hello";
    transfer.payload_size = sizeof(payload);
    transfer.payload = Heap::heapAllocate(nullptr, sizeof(payload));
    memcpy(transfer.payload, payload, sizeof(payload));

    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    auto task = std::make_shared<MockTask>(10, 0, transfer);
    handlers.push(TaskHandler{port_id, task});
    ServiceManager service_manager(handlers);

    CircularBuffer<SerialFrame, 4> serial_rx_buffer;

    CHECK(cyphal.cyphalTxPush(0, &transfer.metadata, transfer.payload_size, transfer.payload) == 1);

    auto diagnostics = Heap::getDiagnostics();
    size_t allocated_mem = diagnostics.allocated;

    SafeAllocator<CyphalTransfer, Heap> alloc;
    LoopManager loop_manager(alloc);
    loop_manager.SerialProcessRxQueue(&cyphal, &service_manager, adapters, serial_rx_buffer);

    CHECK(serial_rx_buffer.size() == 0);
    diagnostics = Heap::getDiagnostics();
    CHECK(allocated_mem == diagnostics.allocated);
}

TEST_CASE("SerialProcessRxQueue multiple frames with Serard")
{
    constexpr CyphalPortID port_id = 123;
    constexpr CyphalNodeID node_id = 11;

    Heap::initialize();
    
    SerardAdapter serard_adapter;
    SerardMemoryResource serard_memory_resource = {&serard_adapter.ins, Heap::serardMemoryDeallocate, Heap::serardMemoryAllocate};
    serard_adapter.ins = serardInit(serard_memory_resource, serard_memory_resource);
    serard_adapter.ins.node_id = node_id;
    serard_adapter.user_reference = &serard_adapter.ins;
    serard_adapter.ins.user_reference = &serard_adapter.ins;
    serard_adapter.reass = serardReassemblerInit();
    serard_adapter.emitter = [](void*, uint8_t, const uint8_t*) -> bool { return true; };
    Cyphal<SerardAdapter> cyphal(&serard_adapter);

    auto adapters = std::make_tuple();

    CyphalTransfer transfer1;
    transfer1.metadata.priority = CyphalPriorityNominal;
    transfer1.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer1.metadata.port_id = port_id;
    transfer1.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer1.metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    transfer1.metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    transfer1.metadata.transfer_id = 0;
    constexpr char payload1[] = "hello";
    transfer1.payload_size = sizeof(payload1);
    transfer1.payload = Heap::heapAllocate(nullptr, sizeof(payload1));
    memcpy(transfer1.payload, payload1, sizeof(payload1));

    CyphalTransfer transfer2;
    transfer2.metadata.priority = CyphalPriorityNominal;
    transfer2.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer2.metadata.port_id = port_id;
    transfer2.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer2.metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    transfer2.metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    transfer2.metadata.transfer_id = 1;
    constexpr char payload2[] = "world!";
    transfer2.payload_size = sizeof(payload2);
    transfer2.payload = Heap::heapAllocate(nullptr, sizeof(payload2));
    memcpy(transfer2.payload, payload2, sizeof(payload2));

    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    auto task1 = std::make_shared<MockTask>(10, 0, transfer1);
    handlers.push(TaskHandler{port_id, task1});
    auto task2 = std::make_shared<MockTask>(10, 0, transfer2);
    handlers.push(TaskHandler{port_id, task2});

    ServiceManager service_manager(handlers);

    CircularBuffer<SerialFrame, 4> serial_rx_buffer;

    CHECK(cyphal.cyphalTxPush(0, &transfer1.metadata, transfer1.payload_size, transfer1.payload) == 1);
    CHECK(cyphal.cyphalTxPush(0, &transfer2.metadata, transfer2.payload_size, transfer2.payload) == 1);

    auto diagnostics = Heap::getDiagnostics();
    size_t allocated_mem = diagnostics.allocated;

    SafeAllocator<CyphalTransfer, Heap> alloc;
    LoopManager loop_manager(alloc);
    loop_manager.SerialProcessRxQueue(&cyphal, &service_manager, adapters, serial_rx_buffer);

    CHECK(serial_rx_buffer.size() == 0);
    diagnostics = Heap::getDiagnostics();
    CHECK(allocated_mem == diagnostics.allocated);
}

TEST_CASE("LoopProcessRxQueue with LoopardAdapter and MockTask")
{
    constexpr CyphalPortID port_id = 123;

    Heap::initialize();

    auto diagnostics = Heap::getDiagnostics();
    size_t allocated_mem = diagnostics.allocated;

    LoopardAdapter adapter;
    adapter.memory_allocate = Heap::loopardMemoryAllocate;
    adapter.memory_free = Heap::loopardMemoryDeallocate;

    Cyphal<LoopardAdapter> cyphal(&adapter);
    auto adapters = std::make_tuple();

    CyphalTransfer transfer;
    transfer.metadata.priority = CyphalPriorityNominal;
    transfer.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer.metadata.port_id = port_id;
    transfer.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.transfer_id = 0;
    constexpr char payload[] = "hello";
    transfer.payload_size = sizeof(payload);
    transfer.payload = Heap::heapAllocate(nullptr, sizeof(payload));
    memcpy(transfer.payload, payload, sizeof(payload));

    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    auto task = std::make_shared<MockTask>(10, 0, transfer);
    handlers.push(TaskHandler{port_id, task});
    ServiceManager service_manager(handlers);

    adapter.buffer.push(transfer);

    SafeAllocator<CyphalTransfer, Heap> alloc;
    LoopManager loop_manager(alloc);
    loop_manager.LoopProcessRxQueue(&cyphal, &service_manager, adapters);

    CHECK(adapter.buffer.size() == 0);
    diagnostics = Heap::getDiagnostics();
    CHECK(allocated_mem == diagnostics.allocated);
}

TEST_CASE("LoopProcessRxQueue multiple frames with LoopardAdapter")
{
    constexpr CyphalPortID port_id1 = 123;
    constexpr CyphalPortID port_id2 = 124;

    Heap::initialize();

    auto diagnostics = Heap::getDiagnostics();
    size_t allocated_mem = diagnostics.allocated;

    LoopardAdapter adapter;
    adapter.memory_allocate = Heap::loopardMemoryAllocate;
    adapter.memory_free = Heap::loopardMemoryDeallocate;

    Cyphal<LoopardAdapter> cyphal(&adapter);
    auto adapters = std::make_tuple();

    CyphalTransfer transfer1;
    transfer1.metadata.priority = CyphalPriorityNominal;
    transfer1.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer1.metadata.port_id = port_id1;
    transfer1.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer1.metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    transfer1.metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    transfer1.metadata.transfer_id = 0;
    constexpr char payload1[] = "hello";
    transfer1.payload_size = sizeof(payload1);
    transfer1.payload = Heap::heapAllocate(nullptr, sizeof(payload1));
    memcpy(transfer1.payload, payload1, sizeof(payload1));

    CyphalTransfer transfer2;
    transfer2.metadata.priority = CyphalPriorityNominal;
    transfer2.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer2.metadata.port_id = port_id2;
    transfer2.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer2.metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    transfer2.metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    transfer2.metadata.transfer_id = 1;
    constexpr char payload2[] = "world!";
    transfer2.payload_size = sizeof(payload2);
    transfer2.payload = Heap::heapAllocate(nullptr, sizeof(payload2));
    memcpy(transfer2.payload, payload2, sizeof(payload2));

    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    auto task1 = std::make_shared<MockTask>(0, 0, transfer1);
    handlers.push(TaskHandler{port_id1, task1});
    auto task2 = std::make_shared<MockTask>(0, 0, transfer2);
    handlers.push(TaskHandler{port_id2, task2});

    ServiceManager service_manager(handlers);

    adapter.buffer.push(transfer1);
    adapter.buffer.push(transfer2);

    SafeAllocator<CyphalTransfer, Heap> alloc;
    LoopManager loop_manager(alloc);
    loop_manager.LoopProcessRxQueue(&cyphal, &service_manager, adapters);

    CHECK(adapter.buffer.size() == 0);
    diagnostics = Heap::getDiagnostics();
    CHECK(allocated_mem == diagnostics.allocated);
}

TEST_CASE("Full Loop Test with LoopardAdapter and MockTask")
{
    constexpr CyphalPortID port_id = 123;

    Heap::initialize();

    LoopardAdapter adapter;
    adapter.memory_allocate = Heap::loopardMemoryAllocate;
    adapter.memory_free = Heap::loopardMemoryDeallocate;

    Cyphal<LoopardAdapter> cyphal(&adapter);
    auto adapters = std::make_tuple();

    CyphalTransfer transfer;
    transfer.metadata.priority = CyphalPriorityNominal;
    transfer.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer.metadata.port_id = port_id;
    transfer.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.transfer_id = 0;
    constexpr char payload[] = "hello";
    transfer.payload_size = sizeof(payload);
    transfer.payload = Heap::heapAllocate(nullptr, sizeof(payload));
    memcpy(transfer.payload, payload, sizeof(payload));

    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    auto task1 = std::make_shared<MockTask>(10, 0, transfer);
    auto task2 = std::make_shared<MockTask>(10, 0, transfer);
    handlers.push(TaskHandler{port_id, task1});
    handlers.push(TaskHandler{port_id, task2});
    ServiceManager service_manager(handlers);

    auto diagnostics = Heap::getDiagnostics();
    size_t allocated_mem = diagnostics.allocated;

    SafeAllocator<CyphalTransfer, Heap> alloc;
    LoopManager loop_manager(alloc);

    for (int i = 0; i < 13; ++i)
    {
        diagnostics = Heap::getDiagnostics();
        CHECK(diagnostics.allocated == allocated_mem);
        allocated_mem = diagnostics.allocated;

        transfer.payload_size = sizeof(payload);
        transfer.payload = Heap::heapAllocate(nullptr, sizeof(payload));
        memcpy(transfer.payload, payload, sizeof(payload));
        adapter.buffer.push(transfer);

        loop_manager.LoopProcessRxQueue(&cyphal, &service_manager, adapters);

        diagnostics = Heap::getDiagnostics();
        CHECK(diagnostics.allocated == 64);
        allocated_mem = diagnostics.allocated;
    }
}

TEST_CASE("Full Loop Test with LoopardAdapter and MockTaskFromBuffer")
{
    constexpr CyphalPortID port_id = 123;

    Heap::initialize();

    LoopardAdapter adapter;
    adapter.memory_allocate = Heap::loopardMemoryAllocate;
    adapter.memory_free = Heap::loopardMemoryDeallocate;

    Cyphal<LoopardAdapter> cyphal(&adapter);
    auto adapters = std::make_tuple();

    CyphalTransfer transfer1;
    transfer1.metadata.priority = CyphalPriorityNominal;
    transfer1.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer1.metadata.port_id = port_id;
    transfer1.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer1.metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    transfer1.metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    transfer1.metadata.transfer_id = 0;
    constexpr char payload[] = "hello";

    CyphalTransfer transfer2 = transfer1;
    transfer2.payload_size = sizeof(payload);
    transfer2.payload = Heap::heapAllocate(nullptr, sizeof(payload));
    memcpy(transfer2.payload, payload, sizeof(payload));

    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    auto task1 = std::make_shared<MockTaskFromBuffer>(10, 0, transfer2);
    handlers.push(TaskHandler{port_id, task1});
    ServiceManager service_manager(handlers);

    SafeAllocator<CyphalTransfer, Heap> alloc;
    LoopManager loop_manager(alloc);

    auto diagnostics = Heap::getDiagnostics();
    size_t allocated_mem = diagnostics.allocated;
    size_t initial_allocated = allocated_mem;

    constexpr int NUM_ITERATIONS = 13;
    for (int i = 0; i < NUM_ITERATIONS; ++i)
    {
        fprintf(stderr, "---- Loop iteration %d ----\n", i);
        HAL_SetTick(HAL_GetTick() + 100);
        transfer1.payload_size = sizeof(payload);
        transfer1.payload = Heap::heapAllocate(nullptr, sizeof(payload));
        memcpy(transfer1.payload, payload, sizeof(payload));
        adapter.buffer.push(transfer1);
        CHECK(adapter.buffer.size() == 1);

        diagnostics = Heap::getDiagnostics();
        CHECK(diagnostics.allocated - allocated_mem == 64);
        allocated_mem = diagnostics.allocated;

        loop_manager.LoopProcessRxQueue(&cyphal, &service_manager, adapters);

        diagnostics = Heap::getDiagnostics();
        CHECK(diagnostics.allocated - allocated_mem == 128);
        allocated_mem = diagnostics.allocated;
    }

    for (int i = 0; i < NUM_ITERATIONS; ++i)
    {
        fprintf(stderr, "---- HandleTask iteration %d ----\n", i);
        task1->handleTaskImpl();
        CHECK(task1->getBuffer().size() == NUM_ITERATIONS - i - 1);

        diagnostics = Heap::getDiagnostics();
        CHECK(allocated_mem - diagnostics.allocated == 192);
        allocated_mem = diagnostics.allocated;

    }
    diagnostics = Heap::getDiagnostics();
    CHECK(diagnostics.allocated == initial_allocated);
}
