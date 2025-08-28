#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "loopard_adapter.hpp"
#include "Allocator.hpp"
#include "o1heap.h"
#include "ProcessRxQueue.hpp"
#include "RegistrationManager.hpp"
#include "SubscriptionManager.hpp"
#include "ServiceManager.hpp"


void checkTransfers(const CyphalTransfer transfer1, const CyphalTransfer transfer2)
{
    CHECK(transfer1.metadata.port_id == transfer2.metadata.port_id);
    CHECK(transfer1.payload_size == transfer2.payload_size);
    CHECK(strncmp(static_cast<const char *>(transfer1.payload), static_cast<const char *>(transfer2.payload), transfer1.payload_size) == 0);
}

// Mock Task class that interacts with the ServiceManager
constexpr CyphalPortID CYPHALPORT = 129; 

class MockTask : public Task
{
public:
    MockTask(uint32_t interval, uint32_t tick, const CyphalTransfer transfer) : Task(interval, tick), transfer_(transfer) {}
    ~MockTask() override {}

    void handleMessage(std::shared_ptr<CyphalTransfer> transfer) override
    {
        checkTransfers(transfer_, *transfer);
    }

    void handleTaskImpl() override {}
    void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override
    {
        manager->subscribe(CYPHALPORT, task);
    }
    void unregisterTask(RegistrationManager */*manager*/, std::shared_ptr<Task> /*task*/) override {}

    CyphalTransfer transfer_;
};

class MockTaskFromBuffer : public TaskFromBuffer
{
public:
MockTaskFromBuffer(uint32_t interval, uint32_t tick, const CyphalTransfer transfer) : TaskFromBuffer(interval, tick), transfer_(transfer) {}
    ~MockTaskFromBuffer() override {}

    void handleTaskImpl() override 
    {
        CHECK(buffer_.size() == 1);

        for(size_t i=0; i<buffer_.size(); ++i)
        {
            auto transfer = buffer_.pop();
            checkTransfers(transfer_, *transfer);
        }
    }
    void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override
    {
        manager->subscribe(CYPHALPORT, task);
    }
    void unregisterTask(RegistrationManager */*manager*/, std::shared_ptr<Task> /*task*/) override {}

    CyphalTransfer transfer_;
};


TEST_CASE("processTransfer no forwarding")
{
    // Setup
    constexpr CyphalPortID port_id = 123;

    char buffer[4192] __attribute__((aligned(256)));
    O1HeapInstance *o1heap = o1heapInit(buffer, 4192);
    O1HeapAllocator<CyphalTransfer> alloc(o1heap);

    auto adapters = std::make_tuple();

    CyphalTransfer transfer;
    transfer.metadata.priority = CyphalPriorityNominal;
    transfer.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer.metadata.port_id = port_id;
    transfer.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.transfer_id = 0;
    constexpr char payload[] = "hello";
    transfer.payload_size = sizeof(payload);
    transfer.payload = o1heapAllocate(o1heap, sizeof(payload));
    memcpy(transfer.payload, payload, sizeof(payload));

    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    std::shared_ptr<MockTask> task = std::make_shared<MockTask>(10, 0, transfer);
    CHECK(task.use_count() == 1);
    handlers.push(TaskHandler{port_id, task});
    ServiceManager service_manager(handlers);
    CHECK(task.use_count() == 2);

    // Exercise
    O1HeapAllocator<CyphalTransfer> allocator(o1heap);
    LoopManager loop_manager(allocator);
    bool result = loop_manager.processTransfer(transfer, &service_manager, adapters);

    // Verify processTransfer was successful
    CHECK(result == true);
    CHECK(task.use_count() == 2);
}

void *loopardMemoryAllocate(size_t amount) { return static_cast<void *>(malloc(amount)); };
void loopardMemoryFree(void *pointer) { free(pointer); };

TEST_CASE("processTransfer with LoopardAdapter")
{
    // Setup
    constexpr CyphalPortID port_id = 123;

    char buffer[4192] __attribute__((aligned(256)));
    O1HeapInstance *o1heap = o1heapInit(buffer, 4192);

    LoopardAdapter adapter;
    adapter.memory_allocate = loopardMemoryAllocate;
    adapter.memory_free = loopardMemoryFree;

    Cyphal<LoopardAdapter> cyphal(&adapter);
    auto adapters = std::make_tuple(cyphal);

    CyphalTransfer transfer;
    transfer.metadata.priority = CyphalPriorityNominal;
    transfer.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer.metadata.port_id = port_id;
    transfer.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.transfer_id = 0;
    constexpr char payload[] = "hello";
    transfer.payload_size = sizeof(payload);
    transfer.payload = o1heapAllocate(o1heap, sizeof(payload));
    memcpy(transfer.payload, payload, sizeof(payload));

    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    std::shared_ptr<MockTask> task = std::make_shared<MockTask>(10, 0, transfer);
    CHECK(task.use_count() == 1);
    handlers.push(TaskHandler{port_id, task});
    ServiceManager service_manager(handlers);
    CHECK(task.use_count() == 2);

    // Exercise
    O1HeapAllocator<CyphalTransfer> allocator(o1heap);
    LoopManager loop_manager(allocator);
    bool result = loop_manager.processTransfer(transfer, &service_manager, adapters);

    // Verify processTransfer was successful
    CHECK(result == true);
    CHECK(adapter.buffer.size() == 1); // Ensure the transfer was pushed to the adapter's buffer

    // Verify data received in LoopardAdapter, you can use the function to check if the transfer was received and pop it
    CyphalTransfer received_transfer;
    size_t frame_size = 0;
    int32_t rx_result = cyphal.cyphalRxReceive(nullptr, &frame_size, &received_transfer);

    CHECK(rx_result == 1); //  Check if a transfer was received.
    transfer.payload_size = sizeof(payload);
    transfer.payload = o1heapAllocate(o1heap, sizeof(payload));
    memcpy(transfer.payload, payload, sizeof(payload));
    checkTransfers(transfer, received_transfer);

    // After popping the transfer, the buffer should be empty
    CyphalTransfer received_transfer2;
    rx_result = cyphal.cyphalRxReceive(nullptr, &frame_size, &received_transfer2);
    CHECK(rx_result == 0);
    CHECK(task.use_count() == 2);
}

void *canardMemoryAllocate(CanardInstance */*ins*/, size_t amount) { return static_cast<void *>(malloc(amount)); };
void canardMemoryFree(CanardInstance */*ins*/, void *pointer) { free(pointer); };

TEST_CASE("processTransfer with LoopardAdapter and CanardAdapter")
{
    // Setup
    constexpr CyphalPortID port_id = 123;
    constexpr CyphalPortID node_id = 11;

    char buffer[4192] __attribute__((aligned(256)));
    O1HeapInstance *o1heap = o1heapInit(buffer, 4192);

    LoopardAdapter loopard_adapter;
    loopard_adapter.memory_allocate = loopardMemoryAllocate;
    loopard_adapter.memory_free = loopardMemoryFree;

    Cyphal<LoopardAdapter> loopard_cyphal(&loopard_adapter);

    CanardAdapter canard_adapter;
    canard_adapter.ins = canardInit(canardMemoryAllocate, canardMemoryFree);
    canard_adapter.ins.node_id = node_id;
    canard_adapter.que = canardTxInit(16, CANARD_MTU_CAN_CLASSIC);
    Cyphal<CanardAdapter> canard_cyphal(&canard_adapter);

    auto adapters = std::make_tuple(loopard_cyphal, canard_cyphal);

    CyphalTransfer transfer;
    transfer.metadata.priority = CyphalPriorityNominal;
    transfer.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer.metadata.port_id = port_id;
    transfer.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.transfer_id = 0;
    constexpr char payload[] = "hello";
    transfer.payload_size = sizeof(payload);
    transfer.payload = o1heapAllocate(o1heap, sizeof(payload));
    memcpy(transfer.payload, payload, sizeof(payload));

    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    std::shared_ptr<MockTask> task = std::make_shared<MockTask>(10, 0, transfer);
    CHECK(task.use_count() == 1);
    handlers.push(TaskHandler{port_id, task});
    ServiceManager service_manager(handlers);
    CHECK(task.use_count() == 2);

    // Exercise
    O1HeapAllocator<CyphalTransfer> allocator(o1heap);
    LoopManager loop_manager(allocator);
    bool result = loop_manager.processTransfer(transfer, &service_manager, adapters);

    // Verify processTransfer was successful
    CHECK(result == true);
    CHECK(loopard_adapter.buffer.size() == 1); // Ensure the transfer was pushed to the loopard adapter's buffer
    CHECK(canard_adapter.que.size > 0);        // Ensure a frame was queued in canard adapter

    // Verify data received in LoopardAdapter
    CyphalTransfer received_transfer_loopard;
    size_t frame_size_loopard = 0;
    int32_t rx_result_loopard = loopard_cyphal.cyphalRxReceive(nullptr, &frame_size_loopard, &received_transfer_loopard);
    CHECK(rx_result_loopard == 1); //  Check if a transfer was received.
    transfer.payload_size = sizeof(payload);
    transfer.payload = o1heapAllocate(o1heap, sizeof(payload));
    memcpy(transfer.payload, payload, sizeof(payload));
    checkTransfers(transfer, received_transfer_loopard);

    // Verify data received in CanardAdapter
    CHECK(canard_cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 123, 100, 2000000) == 1);
    const CanardTxQueueItem *const const_ptr = canardTxPeek(&canard_adapter.que);
    CHECK(const_ptr != nullptr);
    CanardTxQueueItem *ptr = canardTxPop(&canard_adapter.que, const_ptr);
    CHECK(ptr != nullptr);

    CyphalTransfer received_transfer_canard;
    CHECK(canard_cyphal.cyphalRxReceive(ptr->frame.extended_can_id, &ptr->frame.payload_size, static_cast<const uint8_t *>(ptr->frame.payload), &received_transfer_canard) == 1);
    checkTransfers(transfer, received_transfer_canard);

    CHECK(task.use_count() == 2);
}

TEST_CASE("CanProcessRxQueue with CanardAdapter and MockTask")
{
    // Setup
    constexpr CyphalPortID port_id = 123;
    constexpr CyphalNodeID node_id = 11;

    char buffer[4192] __attribute__((aligned(256)));
    O1HeapInstance *o1heap = o1heapInit(buffer, 4192);

    CanardAdapter canard_adapter;
    canard_adapter.ins = canardInit(canardMemoryAllocate, canardMemoryFree);
    canard_adapter.ins.node_id = node_id;
    canard_adapter.que = canardTxInit(16, CANARD_MTU_CAN_CLASSIC);
    Cyphal<CanardAdapter> cyphal(&canard_adapter);

    auto adapters = std::make_tuple();

    CyphalTransfer transfer;
    transfer.metadata.priority = CyphalPriorityNominal;
    transfer.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer.metadata.port_id = port_id;
    transfer.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.transfer_id = 0;
    constexpr char payload[] = "hello";
    transfer.payload_size = sizeof(payload);
    transfer.payload = o1heapAllocate(o1heap, sizeof(payload));
    memcpy(transfer.payload, payload, sizeof(payload));

    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    std::shared_ptr<MockTask> task = std::make_shared<MockTask>(10, 0, transfer);
    CHECK(task.use_count() == 1);
    handlers.push(TaskHandler{port_id, task});
    ServiceManager service_manager(handlers);
    CHECK(task.use_count() == 2);

    CircularBuffer<CanRxFrame, 64> can_rx_buffer;

    // Exercise
    CHECK(cyphal.cyphalTxPush(0, &transfer.metadata, transfer.payload_size, transfer.payload) == 1);
    O1HeapAllocator<CyphalTransfer> allocator(o1heap);
    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(o1heap);
    size_t allocated_mem = diagnostics.allocated;

    LoopManager loop_manager(allocator);
    loop_manager.CanProcessRxQueue(&cyphal, &service_manager, adapters, can_rx_buffer);

    // Verify processTransfer was successful, and the data in the can_rx_buffer has been cleared
    CHECK(can_rx_buffer.size() == 0);
    CHECK(task.use_count() == 2);
    diagnostics = o1heapGetDiagnostics(o1heap);
    CHECK(allocated_mem == diagnostics.allocated);
}

TEST_CASE("CanProcessRxQueue multiple frames")
{
    // Setup
    constexpr CyphalPortID port_id = 123;
    constexpr CyphalNodeID node_id = 11;

    char buffer[4192] __attribute__((aligned(256)));
    O1HeapInstance *o1heap = o1heapInit(buffer, 4192);

    CanardAdapter canard_adapter;
    canard_adapter.ins = canardInit(canardMemoryAllocate, canardMemoryFree);
    canard_adapter.ins.node_id = node_id;
    canard_adapter.que = canardTxInit(16, CANARD_MTU_CAN_CLASSIC);
    Cyphal<CanardAdapter> cyphal(&canard_adapter);

    auto adapters = std::make_tuple();

    CyphalTransfer transfer1;
    transfer1.metadata.priority = CyphalPriorityNominal;
    transfer1.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer1.metadata.port_id = port_id;
    transfer1.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer1.metadata.transfer_id = 0;
    constexpr char payload1[] = "hello";
    transfer1.payload_size = sizeof(payload1);
    transfer1.payload = o1heapAllocate(o1heap, sizeof(payload1));
    memcpy(transfer1.payload, payload1, sizeof(payload1));

    CyphalTransfer transfer2;
    transfer2.metadata.priority = CyphalPriorityNominal;
    transfer2.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer2.metadata.port_id = port_id;
    transfer2.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer2.metadata.transfer_id = 1;
    constexpr char payload2[] = "world!";
    transfer2.payload_size = sizeof(payload2);
    transfer2.payload = o1heapAllocate(o1heap, sizeof(payload2));
    memcpy(transfer2.payload, payload2, sizeof(payload2));

    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    std::shared_ptr<MockTask> task1 = std::make_shared<MockTask>(10, 0, transfer1);
    CHECK(task1.use_count() == 1);
    handlers.push(TaskHandler{port_id, task1});

    std::shared_ptr<MockTask> task2 = std::make_shared<MockTask>(10, 0, transfer2);
    CHECK(task2.use_count() == 1);
    handlers.push(TaskHandler{port_id, task2});

    ServiceManager service_manager(handlers);
    CHECK(task1.use_count() == 2);
    CHECK(task2.use_count() == 2);

    CircularBuffer<CanRxFrame, 64> can_rx_buffer;

    // Exercise
    CHECK(cyphal.cyphalTxPush(0, &transfer1.metadata, transfer1.payload_size, transfer1.payload) == 1);
    CHECK(cyphal.cyphalTxPush(0, &transfer2.metadata, transfer2.payload_size, transfer2.payload) == 1);
    O1HeapAllocator<CyphalTransfer> allocator(o1heap);
    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(o1heap);
    size_t allocated_mem = diagnostics.allocated;

    LoopManager loop_manager(allocator);
    loop_manager.CanProcessRxQueue(&cyphal, &service_manager, adapters, can_rx_buffer);

    // Verify processTransfer was successful, and the data in the can_rx_buffer has been cleared
    CHECK(can_rx_buffer.size() == 0);
    CHECK(task1.use_count() == 2);
    CHECK(task2.use_count() == 2);

    diagnostics = o1heapGetDiagnostics(o1heap);
    CHECK(allocated_mem == diagnostics.allocated);
}

void *serardMemoryAllocate(void *const /*user_reference*/, const size_t size) { return static_cast<void *>(malloc(size)); };
void serardMemoryDeallocate(void *const /*user_reference*/, const size_t /*size*/, void *const pointer) { free(pointer); };

TEST_CASE("SerialProcessRxQueue with SerardAdapter and MockTask")
{
    // Setup
    constexpr CyphalPortID port_id = 123;
    constexpr CyphalNodeID node_id = 11;

    char buffer[4192] __attribute__((aligned(256)));
    O1HeapInstance *o1heap = o1heapInit(buffer, 4192);

    SerardAdapter serard_adapter;
    struct SerardMemoryResource serard_memory_resource = {&serard_adapter.ins, serardMemoryDeallocate, serardMemoryAllocate};
    serard_adapter.ins = serardInit(serard_memory_resource, serard_memory_resource);
    serard_adapter.ins.node_id = node_id;
    serard_adapter.user_reference = &serard_adapter.ins;
    serard_adapter.ins.user_reference = &serard_adapter.ins;
    serard_adapter.reass = serardReassemblerInit();
    serard_adapter.emitter = [](void *, uint8_t, const uint8_t *) -> bool
    { return true; };
    Cyphal<SerardAdapter> cyphal(&serard_adapter);

    auto adapters = std::make_tuple();

    CyphalTransfer transfer;
    transfer.metadata.priority = CyphalPriorityNominal;
    transfer.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer.metadata.port_id = port_id;
    transfer.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.transfer_id = 0;
    constexpr char payload[] = "hello";
    transfer.payload_size = sizeof(payload);
    transfer.payload = o1heapAllocate(o1heap, sizeof(payload));
    memcpy(transfer.payload, payload, sizeof(payload));

    ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    std::shared_ptr<MockTask> task = std::make_shared<MockTask>(10, 0, transfer);
    CHECK(task.use_count() == 1);
    handlers.push(TaskHandler{port_id, task});
    ServiceManager service_manager(handlers);
    CHECK(task.use_count() == 2);

    CircularBuffer<SerialFrame, 4> serial_rx_buffer; // Even though it's for CAN, we still need it for the template function

    // Exercise
    CHECK(cyphal.cyphalTxPush(0, &transfer.metadata, transfer.payload_size, transfer.payload) == 1);
    O1HeapAllocator<CyphalTransfer> allocator(o1heap);
    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(o1heap);
    size_t allocated_mem = diagnostics.allocated;

    LoopManager loop_manager(allocator);
    loop_manager.SerialProcessRxQueue(&cyphal, &service_manager, adapters, serial_rx_buffer);

    // Verify processTransfer was successful, and the data in the can_rx_buffer has been cleared (should not be affected)
    CHECK(serial_rx_buffer.size() == 0);
    CHECK(task.use_count() == 2);
    diagnostics = o1heapGetDiagnostics(o1heap);
    CHECK(allocated_mem == diagnostics.allocated);
}

TEST_CASE("SerialProcessRxQueue multiple frames with Serard")
{
    // Setup
    constexpr CyphalPortID port_id = 123;
    constexpr CyphalNodeID node_id = 11;

    char buffer[4192] __attribute__((aligned(256)));
    O1HeapInstance *o1heap = o1heapInit(buffer, 4192);

    SerardAdapter serard_adapter;
    struct SerardMemoryResource serard_memory_resource = {&serard_adapter.ins, serardMemoryDeallocate, serardMemoryAllocate};
    serard_adapter.ins = serardInit(serard_memory_resource, serard_memory_resource);
    serard_adapter.ins.node_id = node_id;
    serard_adapter.user_reference = &serard_adapter.ins;
    serard_adapter.ins.user_reference = &serard_adapter.ins;
    serard_adapter.reass = serardReassemblerInit();
    serard_adapter.emitter = [](void *, uint8_t, const uint8_t *) -> bool
    { return true; };
    Cyphal<SerardAdapter> cyphal(&serard_adapter);

    auto adapters = std::make_tuple();

    CyphalTransfer transfer1;
    transfer1.metadata.priority = CyphalPriorityNominal;
    transfer1.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer1.metadata.port_id = port_id;
    transfer1.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer1.metadata.transfer_id = 0;
    constexpr char payload1[] = "hello";
    transfer1.payload_size = sizeof(payload1);
    transfer1.payload = o1heapAllocate(o1heap, sizeof(payload1));
    memcpy(transfer1.payload, payload1, sizeof(payload1));

    CyphalTransfer transfer2;
    transfer2.metadata.priority = CyphalPriorityNominal;
    transfer2.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer2.metadata.port_id = port_id;
    transfer2.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer2.metadata.transfer_id = 1;
    constexpr char payload2[] = "world!";
    transfer2.payload_size = sizeof(payload2);
    transfer2.payload = o1heapAllocate(o1heap, sizeof(payload2));
    memcpy(transfer2.payload, payload2, sizeof(payload2));

    ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    std::shared_ptr<MockTask> task1 = std::make_shared<MockTask>(10, 0, transfer1);
    CHECK(task1.use_count() == 1);
    handlers.push(TaskHandler{port_id, task1});

    std::shared_ptr<MockTask> task2 = std::make_shared<MockTask>(10, 0, transfer2);
    CHECK(task2.use_count() == 1);
    handlers.push(TaskHandler{port_id, task2});

    ServiceManager service_manager(handlers);
    CHECK(task1.use_count() == 2);
    CHECK(task2.use_count() == 2);

    CircularBuffer<SerialFrame, 4> serial_rx_buffer; // Even though it's for CAN, we still need it for the template function

    // Exercise
    CHECK(cyphal.cyphalTxPush(0, &transfer1.metadata, transfer1.payload_size, transfer1.payload) == 1);
    CHECK(cyphal.cyphalTxPush(0, &transfer2.metadata, transfer2.payload_size, transfer2.payload) == 1);
    O1HeapAllocator<CyphalTransfer> allocator(o1heap);
    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(o1heap);
    size_t allocated_mem = diagnostics.allocated;

    LoopManager loop_manager(allocator);
    loop_manager.SerialProcessRxQueue(&cyphal, &service_manager, adapters, serial_rx_buffer);

    // Verify processTransfer was successful, and the data in the can_rx_buffer has been cleared (should not be affected)
    CHECK(serial_rx_buffer.size() == 0);
    CHECK(task1.use_count() == 2);
    CHECK(task2.use_count() == 2);
    diagnostics = o1heapGetDiagnostics(o1heap);
    CHECK(allocated_mem == diagnostics.allocated);
}

TEST_CASE("LoopProcessRxQueue with LoopardAdapter and MockTask")
{
    // Setup
    constexpr CyphalPortID port_id = 123;

    char buffer[4192] __attribute__((aligned(256)));
    O1HeapInstance *o1heap = o1heapInit(buffer, 4192);
    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(o1heap);
    size_t allocated_mem = diagnostics.allocated;

    LoopardAdapter adapter;
    Cyphal<LoopardAdapter> cyphal(&adapter);
    auto adapters = std::make_tuple();

    CyphalTransfer transfer;
    transfer.metadata.priority = CyphalPriorityNominal;
    transfer.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer.metadata.port_id = port_id;
    transfer.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.transfer_id = 0;
    constexpr char payload[] = "hello";
    transfer.payload_size = sizeof(payload);
    transfer.payload = o1heapAllocate(o1heap, sizeof(payload));
    memcpy(transfer.payload, payload, sizeof(payload));

    ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    std::shared_ptr<MockTask> task = std::make_shared<MockTask>(10, 0, transfer);
    CHECK(task.use_count() == 1);
    handlers.push(TaskHandler{port_id, task});
    ServiceManager service_manager(handlers);
    CHECK(task.use_count() == 2);

    // push to loop back, to mock a receive
    adapter.buffer.push(transfer);

    // Exercise
    O1HeapAllocator<CyphalTransfer> allocator(o1heap);
    LoopManager loop_manager(allocator);
    loop_manager.LoopProcessRxQueue(&cyphal, &service_manager, adapters);

    // Verify processTransfer was successful, and the data in the can_rx_buffer has been cleared
    CHECK(adapter.buffer.size() == 0);
    CHECK(task.use_count() == 2);
    diagnostics = o1heapGetDiagnostics(o1heap);
    CHECK(allocated_mem == diagnostics.allocated);
}

TEST_CASE("LoopProcessRxQueue multiple frames with LoopardAdapter")
{
    // Setup
    constexpr CyphalPortID port_id1 = 123;
    constexpr CyphalPortID port_id2 = 124;

    char buffer[4192] __attribute__((aligned(256)));
    O1HeapInstance *o1heap = o1heapInit(buffer, 4192);
    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(o1heap);
    size_t allocated_mem = diagnostics.allocated;

    LoopardAdapter adapter;
    Cyphal<LoopardAdapter> cyphal(&adapter);
    auto adapters = std::make_tuple();

    CyphalTransfer transfer1;
    transfer1.metadata.priority = CyphalPriorityNominal;
    transfer1.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer1.metadata.port_id = port_id1;
    transfer1.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer1.metadata.transfer_id = 0;
    constexpr char payload1[] = "hello";
    transfer1.payload_size = sizeof(payload1);
    transfer1.payload = o1heapAllocate(o1heap, sizeof(payload1));
    memcpy(transfer1.payload, payload1, sizeof(payload1));

    CyphalTransfer transfer2;
    transfer2.metadata.priority = CyphalPriorityNominal;
    transfer2.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer2.metadata.port_id = port_id2;
    transfer2.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer2.metadata.transfer_id = 1;
    constexpr char payload2[] = "world!";
    transfer2.payload_size = sizeof(payload2);
    transfer2.payload = o1heapAllocate(o1heap, sizeof(payload2));
    memcpy(transfer2.payload, payload2, sizeof(payload2));

    ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    std::shared_ptr<MockTask> task1 = std::make_shared<MockTask>(0, 0, transfer1);
    CHECK(task1.use_count() == 1);
    handlers.push(TaskHandler{port_id1, task1});

    std::shared_ptr<MockTask> task2 = std::make_shared<MockTask>(0, 0, transfer2);
    CHECK(task2.use_count() == 1);
    handlers.push(TaskHandler{port_id2, task2});

    ServiceManager service_manager(handlers);
    CHECK(task1.use_count() == 2);
    CHECK(task2.use_count() == 2);

    // push to loop back, to mock a receive
    adapter.buffer.push(transfer1);
    adapter.buffer.push(transfer2);

    // Exercise
    O1HeapAllocator<CyphalTransfer> allocator(o1heap);
    LoopManager loop_manager(allocator);
    loop_manager.LoopProcessRxQueue(&cyphal, &service_manager, adapters);

    // Verify processTransfer was successful, and the data in the can_rx_buffer has been cleared
    CHECK(adapter.buffer.size() == 0);
    CHECK(task1.use_count() == 2);
    CHECK(task2.use_count() == 2);
    diagnostics = o1heapGetDiagnostics(o1heap);
    CHECK(allocated_mem == diagnostics.allocated);
}

TEST_CASE("Full Loop Test with LoopardAdapter and MockTask")
{
    // Setup
    constexpr CyphalPortID port_id = 123;

    char buffer[4192] __attribute__((aligned(256)));
    O1HeapInstance *o1heap = o1heapInit(buffer, 4192);

    LoopardAdapter adapter;
    adapter.memory_allocate = loopardMemoryAllocate;
    adapter.memory_free = loopardMemoryFree;

    Cyphal<LoopardAdapter> cyphal(&adapter);
    auto adapters = std::make_tuple();

    CyphalTransfer transfer;
    transfer.metadata.priority = CyphalPriorityNominal;
    transfer.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer.metadata.port_id = port_id;
    transfer.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.transfer_id = 0;
    constexpr char payload[] = "hello";
    transfer.payload_size = sizeof(payload);
    transfer.payload = o1heapAllocate(o1heap, sizeof(payload));
    memcpy(transfer.payload, payload, sizeof(payload));

    ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    std::shared_ptr<MockTask> task1 = std::make_shared<MockTask>(10, 0, transfer);
    std::shared_ptr<MockTask> task2 = std::make_shared<MockTask>(10, 0, transfer);
    handlers.push(TaskHandler{port_id, task1});
    handlers.push(TaskHandler{port_id, task2});
    ServiceManager service_manager(handlers);

    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(o1heap);
    size_t allocated_mem = diagnostics.allocated;

    O1HeapAllocator<CyphalTransfer> allocator(o1heap);
    LoopManager loop_manager(allocator);
    for (int i = 0; i < 13; ++i)
    {
        diagnostics = o1heapGetDiagnostics(o1heap);
        CHECK(diagnostics.allocated == allocated_mem);
        allocated_mem = diagnostics.allocated;
        transfer.payload_size = sizeof(payload);
        transfer.payload = o1heapAllocate(o1heap, sizeof(payload));
        memcpy(transfer.payload, payload, sizeof(payload));    
        adapter.buffer.push(transfer);
        loop_manager.LoopProcessRxQueue(&cyphal, &service_manager, adapters);
        diagnostics = o1heapGetDiagnostics(o1heap);
        CHECK(diagnostics.allocated == 64);
        allocated_mem = diagnostics.allocated;        
    }
}

TEST_CASE("Full Loop Test with LoopardAdapter and MockTaskFromBuffer")
{
    // Setup
    constexpr CyphalPortID port_id = 123;

    char buffer[4192] __attribute__((aligned(256)));
    O1HeapInstance *o1heap = o1heapInit(buffer, 4192);

    LoopardAdapter adapter;
    adapter.memory_allocate = loopardMemoryAllocate;
    adapter.memory_free = loopardMemoryFree;

    Cyphal<LoopardAdapter> cyphal(&adapter);
    auto adapters = std::make_tuple();

    CyphalTransfer transfer1;
    transfer1.metadata.priority = CyphalPriorityNominal;
    transfer1.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer1.metadata.port_id = port_id;
    transfer1.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer1.metadata.transfer_id = 0;
    constexpr char payload[] = "hello";

    CyphalTransfer transfer2 = transfer1;
    transfer2.payload_size = sizeof(payload);
    transfer2.payload = o1heapAllocate(o1heap, sizeof(payload));
    memcpy(transfer2.payload, payload, sizeof(payload));    


    ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    std::shared_ptr<MockTaskFromBuffer> task1 = std::make_shared<MockTaskFromBuffer>(10, 0, transfer2);
    handlers.push(TaskHandler{port_id, task1});
    CHECK(handlers.size() == 1);
    ServiceManager service_manager(handlers);

    O1HeapAllocator<CyphalTransfer> allocator(o1heap);
    LoopManager loop_manager(allocator);

    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(o1heap);
    size_t allocated_mem = diagnostics.allocated;

    for (int i = 0; i < 13; ++i)
    {
        HAL_SetTick(HAL_GetTick() + 100);
        transfer1.payload_size = sizeof(payload);
        transfer1.payload = o1heapAllocate(o1heap, sizeof(payload));
        memcpy(transfer1.payload, payload, sizeof(payload));    
        adapter.buffer.push(transfer1);
        CHECK(adapter.buffer.size() == 1);
        diagnostics = o1heapGetDiagnostics(o1heap);
        CHECK(diagnostics.allocated == 128);
        allocated_mem = diagnostics.allocated;

        loop_manager.LoopProcessRxQueue(&cyphal, &service_manager, adapters); // adapters are for forwarding!
        diagnostics = o1heapGetDiagnostics(o1heap);
        CHECK(diagnostics.allocated != allocated_mem);
        allocated_mem = diagnostics.allocated;
		
        service_manager.handleServices();
        diagnostics = o1heapGetDiagnostics(o1heap);
        CHECK(diagnostics.allocated != allocated_mem);
        CHECK(diagnostics.allocated == 64);
        allocated_mem = diagnostics.allocated;
    }
}

TEST_CASE("CanProcessTxQueue with CanardAdapter")
{
    // Setup
    constexpr CyphalPortID port_id = 123;
    constexpr CyphalNodeID node_id = 11;

    char buffer[4192] __attribute__((aligned(256)));
    O1HeapInstance *o1heap = o1heapInit(buffer, 4092);

    CanardAdapter canard_adapter;
    canard_adapter.ins = canardInit(canardMemoryAllocate, canardMemoryFree);
    canard_adapter.ins.node_id = node_id;
    canard_adapter.que = canardTxInit(16, CANARD_MTU_CAN_CLASSIC);
    Cyphal<CanardAdapter> cyphal(&canard_adapter);

    CyphalTransfer transfer;
    transfer.metadata.priority = CyphalPriorityNominal;
    transfer.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer.metadata.port_id = port_id;
    transfer.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.transfer_id = 0;
    constexpr char payload[] = "hello_ehllo!!!";
    transfer.payload_size = sizeof(payload);
    transfer.payload = o1heapAllocate(o1heap, sizeof(payload));
    memcpy(transfer.payload, payload, sizeof(payload));

    clear_usb_tx_buffer();
    CHECK(get_can_tx_buffer_count() == 0); //Check tx buffer count.
    // Push a transfer into the Canard queue
    CHECK(canard_adapter.que.size == 0); // Ensure the queue has been emptied.
    CHECK(cyphal.cyphalTxPush(0, &transfer.metadata, transfer.payload_size, transfer.payload) == 3);
    CHECK(canard_adapter.que.size == 3); // Ensure the queue has been emptied.

    // Exercise
    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(o1heap);
    size_t allocated_mem = diagnostics.allocated;

    O1HeapAllocator<CyphalTransfer> allocator(o1heap);
    LoopManager loop_manager(allocator);
    
    CAN_HandleTypeDef hcan_mock; //Create a mock
    loop_manager.CanProcessTxQueue(&canard_adapter, &hcan_mock);

    // Verify that a message was added to the mock CAN hardware
    CHECK(canard_adapter.que.size == 0); // Ensure the queue has been emptied.
    CHECK(get_can_tx_buffer_count() == 3); //Check tx buffer count.
    diagnostics = o1heapGetDiagnostics(o1heap);
    CHECK(allocated_mem == diagnostics.allocated);
}