#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "Task.hpp" // Include your header file
#include "cyphal.hpp"

// Mock HAL functions for testing (x86)
#ifdef __x86_64__
#include "mock_hal.h"
#endif

// Mock Cyphal Transfer structure - using the one from cyphal.hpp now!

// Mock RegistrationManager
class RegistrationManager {
public:
    void register_task(std::shared_ptr<Task> task) {
        registered_tasks.push_back(task);
    }
    void unregister_task(std::shared_ptr<Task> task) {
        for (size_t i = 0; i < registered_tasks.size(); ++i) {
            if (registered_tasks[i] == task) {
                registered_tasks.erase(registered_tasks.begin() + i);
                return;
            }
        }
    }
    std::vector<std::shared_ptr<Task>> registered_tasks;
};

// Concrete Task implementations for testing
class ConcreteTask : public Task {
public:
    ConcreteTask(uint32_t interval, uint32_t tick) : Task(interval, tick), handleTaskImplCalled(false), handleMessageCalled(false), transfer_data(0) {}
    virtual ~ConcreteTask() {}

    void handleTaskImpl() override {
        handleTaskImplCalled = true;
    }

    void handleMessage(std::shared_ptr<CyphalTransfer> transfer) override {
        if (transfer && transfer->payload) {
            transfer_data = *static_cast<uint8_t*>(transfer->payload);
        }
        last_transfer = transfer;
        handleMessageCalled = true;
    }

    void registerTask(RegistrationManager* manager, std::shared_ptr<Task> task) override {
        manager->register_task(task);
    }

    void unregisterTask(RegistrationManager* manager, std::shared_ptr<Task> task) override {
        manager->unregister_task(task);
    }

    bool handleTaskImplCalled;
    std::shared_ptr<CyphalTransfer> last_transfer;
    bool handleMessageCalled;
    uint8_t transfer_data;
};


//Mock adapter and serialize functions
class MockAdapter
{
public:
    int8_t cyphalTxPush(CyphalMicrosecond timeout_us, const CyphalTransferMetadata* metadata, size_t payload_size, const uint8_t* payload)
    {
        timeout_us_ = timeout_us;
        metadata_ = *metadata;
        payload_size_ = payload_size;
        payload_ = payload;
        return 1; // Simulate success
    }

    CyphalMicrosecond timeout_us_;
    CyphalTransferMetadata metadata_;
    size_t payload_size_;
    const uint8_t* payload_;
};

int8_t serialize_mock(const void* const data, uint8_t* const payload, size_t* const payload_size)
{
    // In this mock, we just copy data into payload.
    // In a real implementation, we would serialize the struct into a byte stream.
    if (payload == nullptr) {
        *payload_size = 0;
        return 1; // Or perhaps return an error code
    }
    if (data == nullptr)
    {
        *payload_size = 0;
    }
    else
    {
        const int* int_data = static_cast<const int*>(data);
        payload[0] = *int_data;
        *payload_size = 1;
    }
    return 1;
}

// Mock serialization function that simulates failure
int8_t serialize_mock_fail(const void* const data, uint8_t* const payload, size_t* const payload_size) {
    (void)data; (void)payload; (void)payload_size;  // Suppress warnings
    return -1; // Simulate serialization failure
}


class ConcreteTaskWithPublication : public TaskWithPublication<MockAdapter> {
public:
    ConcreteTaskWithPublication(uint32_t interval, uint32_t tick, CyphalTransferID transfer_id, std::tuple<MockAdapter>& adapters) :
        TaskWithPublication(interval, tick, transfer_id, adapters), handleTaskImplCalled(false), data(0) {}

    void handleTaskImpl() override {
        uint8_t payload[10]; // Enough space for the example.  Adjust as needed.
        publish(sizeof(payload), payload, &data, &serialize_mock, 123);
        handleTaskImplCalled = true;
    }

    void handleMessage(std::shared_ptr<CyphalTransfer> /*transfer*/) override {
        // Not used in this test
    }

    void registerTask(RegistrationManager* manager, std::shared_ptr<Task> task) override {
        manager->register_task(task);
    }

    void unregisterTask(RegistrationManager* manager, std::shared_ptr<Task> task) override {
        manager->unregister_task(task);
    }
    MockAdapter& getAdapter() { return std::get<0>(adapters_); }
    const MockAdapter& getAdapter() const { return std::get<0>(adapters_); } // Add const version


    bool handleTaskImplCalled;
    int data;
};

class TaskFromBufferExpanded : public TaskFromBuffer {
public:
    TaskFromBufferExpanded(uint32_t interval, uint32_t tick) : TaskFromBuffer(interval, tick) {}

    void handleTaskImpl() override {
        // Implement the abstract method.  Can be empty if no action is required.
    }
    void registerTask(RegistrationManager* manager, std::shared_ptr<Task> task) override {
        manager->register_task(task);
    }

    void unregisterTask(RegistrationManager* manager, std::shared_ptr<Task> task) override {
        manager->unregister_task(task);
    }

    size_t getBufferSize() const { return buffer.size(); } // Expose buffer size

    std::shared_ptr<CyphalTransfer> popExpanded() { return buffer.pop(); } // Expose pop
};

class ConcreteTaskForServer : public TaskForServer<MockAdapter> {
public:
    ConcreteTaskForServer(uint32_t interval, uint32_t tick, std::tuple<MockAdapter>& adapters) :
        TaskForServer(interval, tick, adapters), handleTaskImplCalled(false), data(0) {}

    void handleTaskImpl() override {
        // For simplicity, just publish a fixed payload when the task is handled
        uint8_t payload[10];
        publish(sizeof(payload), payload, &data, &serialize_mock, 123, 45, 67); // Example values
        handleTaskImplCalled = true;
    }

    void handleMessage(std::shared_ptr<CyphalTransfer> transfer) override {
        // Store the transfer in the buffer
        buffer.push(transfer);
        handleMessageCalled = true;
    }

    void registerTask(RegistrationManager* manager, std::shared_ptr<Task> task) override {
        manager->register_task(task);
    }

    void unregisterTask(RegistrationManager* manager, std::shared_ptr<Task> task) override {
        manager->unregister_task(task);
    }

    MockAdapter& getAdapter() { return std::get<0>(adapters_); }
    const MockAdapter& getAdapter() const { return std::get<0>(adapters_); } // Add const version

    bool handleTaskImplCalled;
    bool handleMessageCalled;
    int data;
    CyphalBuffer buffer; // Need to declare the buffer here
};


TEST_CASE("HAL_GetTick") {
    HAL_SetTick(100);
    CHECK(HAL_GetTick() == 100);
    HAL_SetTick(200);
    CHECK(HAL_GetTick() == 200);
}

TEST_CASE("Task Initialization and Getters") {
    ConcreteTask task(100, 50);

    CHECK(task.getInterval() == 100);
    CHECK(task.getShift() == 50);
    CHECK(task.getLastTick() == 0);

    task.initialize(1000);
    CHECK(task.getLastTick() == 1050);
}

TEST_CASE("Task Setters") {
    ConcreteTask task(100, 50);

    task.setInterval(200);
    task.setShift(75);
    task.setLastTick(1500);

    CHECK(task.getInterval() == 200);
    CHECK(task.getShift() == 75);
    CHECK(task.getLastTick() == 1500);
}

TEST_CASE("Task Execution Check") {
    ConcreteTask task(100, 50);
    task.initialize(0);
    HAL_SetTick(0);

    // First check, should not execute since not enough time has passed
    HAL_SetTick(50);
    task.handleTask();
    CHECK(task.handleTaskImplCalled == false);

    // Second check, enough time has passed, should execute
    HAL_SetTick(151);
    task.handleTask();
    CHECK(task.handleTaskImplCalled == true);
}

TEST_CASE("Task Handle Message") {
    ConcreteTask task(100, 50);
    auto transfer = std::make_shared<CyphalTransfer>();
    uint8_t data = 42;
    transfer->payload = &data; // Simulate a payload. Important!
    transfer->payload_size = 1; // And its size

    task.handleMessage(transfer);

    REQUIRE(task.handleMessageCalled == true); //Use REQUIRE to stop if this test fails.
    CHECK(task.transfer_data == 42);
}

TEST_CASE("Task Registration") {
    RegistrationManager manager;
    auto task = std::make_shared<ConcreteTask>(100, 50);

    task->registerTask(&manager, task);
    REQUIRE(manager.registered_tasks.size() == 1);
    CHECK(manager.registered_tasks[0] == task);

    task->unregisterTask(&manager, task);
    CHECK(manager.registered_tasks.empty());
}

TEST_CASE("TaskWithPublication Execution") {
    std::tuple<MockAdapter> adapters;
    ConcreteTaskWithPublication task(100, 50, 0, adapters);
    task.initialize(0);
    HAL_SetTick(0);

    // First check, should not execute since not enough time has passed
    HAL_SetTick(50);
    task.handleTask();
    CHECK(task.handleTaskImplCalled == false);

    // Second check, enough time has passed, should execute
    HAL_SetTick(151);
    task.handleTask();
    CHECK(task.handleTaskImplCalled == true);
}

TEST_CASE("TaskWithPublication Publish") {
    std::tuple<MockAdapter> adapters;
    ConcreteTaskWithPublication task(100, 50, 0, adapters);
    task.initialize(0);
    HAL_SetTick(0);

    // Check if publish function calls adapter.cyphalTxPush
    task.handleTaskImpl(); // Call the internal handleTaskImpl which calls publish
    auto& adapter = task.getAdapter(); // Use the accessor method
    CHECK(adapter.timeout_us_ == 0);
    CHECK(adapter.metadata_.priority == CyphalPriorityNominal);
    CHECK(adapter.metadata_.port_id == 123);
    CHECK(adapter.payload_size_ == 1);
    CHECK(adapter.payload_[0] == 0); // Check the first byte of the payload
}

TEST_CASE("TaskWithPublication Publish - Serialization Failure") {
    std::tuple<MockAdapter> adapters;
    ConcreteTaskWithPublication task(100, 50, 0, adapters); // Or modify handleTaskImpl
    task.initialize(0);
    HAL_SetTick(0);

    // Override the serialize function pointer (using a lambda for simplicity):
     auto old_handleTaskImplCalled = task.handleTaskImplCalled; // Save the original state
    task.handleTaskImplCalled = false;
    // Create an anonymous class derived from ConcreteTaskWithPublication, override handleTaskImpl
      class ConcreteTaskWithPublicationOverride : public ConcreteTaskWithPublication {
      public:
          ConcreteTaskWithPublicationOverride(uint32_t interval, uint32_t tick, CyphalTransferID transfer_id, std::tuple<MockAdapter>& adapters) :
              ConcreteTaskWithPublication(interval, tick, transfer_id, adapters) {}

          void handleTaskImpl() override {
             uint8_t payload[10]; // Enough space for the example.  Adjust as needed.
             publish(sizeof(payload), payload, &data, &serialize_mock_fail, 123);
             handleTaskImplCalled = true;
          }
      };

      ConcreteTaskWithPublicationOverride taskOverride(100, 50, 0, adapters);
      taskOverride.initialize(0);
      HAL_SetTick(0);

      taskOverride.handleTaskImpl();  // Execute the task

    CHECK(task.handleTaskImplCalled == old_handleTaskImplCalled); //The original hasn't been called
    CHECK(taskOverride.handleTaskImplCalled == true); // But the override has.
    // How to check the log is tricky because of Logger.hpp - can inject a mock logger if needed.
    //  For now, manually verify in output.
}


TEST_CASE("TaskFromBuffer") {
    TaskFromBufferExpanded task(100, 50);

    // Create some mock CyphalTransfer objects
    auto transfer1 = std::make_shared<CyphalTransfer>();
    uint8_t data1 = 1;
    transfer1->payload = &data1;
    transfer1->payload_size = 1;
    transfer1->metadata.port_id = 10; // Set the port ID

    auto transfer2 = std::make_shared<CyphalTransfer>();
    uint8_t data2 = 2;
    transfer2->payload = &data2;
    transfer2->payload_size = 1;
    transfer2->metadata.port_id = 20; // Set the port ID

    auto transfer3 = std::make_shared<CyphalTransfer>();
    uint8_t data3 = 3;
    transfer3->payload = &data3;
    transfer3->payload_size = 1;
    transfer3->metadata.port_id = 30; // Set the port ID

    // Handle the messages (add them to the buffer)
    task.handleMessage(transfer1);
    task.handleMessage(transfer2);
    task.handleMessage(transfer3);

    REQUIRE(task.getBufferSize() == 3);

    auto popped_transfer1 = task.popExpanded();
    auto popped_transfer2 = task.popExpanded();
    auto popped_transfer3 = task.popExpanded();

    REQUIRE(*static_cast<uint8_t*>(popped_transfer1->payload) == 1); // First element
    REQUIRE(*static_cast<uint8_t*>(popped_transfer2->payload) == 2); // Second element
    REQUIRE(*static_cast<uint8_t*>(popped_transfer3->payload) == 3); // Third element
}

TEST_CASE("TaskFromBuffer - Buffer Overflow") {
    TaskFromBufferExpanded task(100, 50);

    // Fill the buffer to capacity, allocating unique memory for each payload
    std::vector<std::shared_ptr<uint8_t>> payload_data; // Store payload data to keep it alive

    for (size_t i = 0; i < CIRC_BUF_SIZE; ++i) {
        auto transfer = std::make_shared<CyphalTransfer>();
        auto data = std::make_shared<uint8_t>(static_cast<uint8_t>(i));  // Allocate unique memory
        transfer->payload = data.get();
        transfer->payload_size = 1;
        task.handleMessage(transfer);
        payload_data.push_back(data); // Keep the data alive
    }

    REQUIRE(task.getBufferSize() == CIRC_BUF_SIZE);

    // Attempt to push one more element (overflow)
    auto transferOverflow = std::make_shared<CyphalTransfer>();
    auto overflowData = std::make_shared<uint8_t>(CIRC_BUF_SIZE);  // Allocate unique memory for overflow
    transferOverflow->payload = overflowData.get();
    transferOverflow->payload_size = 1;
    task.handleMessage(transferOverflow);
    payload_data.push_back(overflowData); // Keep the data alive

    REQUIRE(task.getBufferSize() == CIRC_BUF_SIZE); // Should remain full

    // Pop and Check
    for (size_t i = 1; i < CIRC_BUF_SIZE; ++i) {
        auto popped_transfer = task.popExpanded();
        CHECK(*static_cast<uint8_t*>(popped_transfer->payload) == i);
    }
    auto popped_transfer = task.popExpanded();
    CHECK(*static_cast<uint8_t*>(popped_transfer->payload) == CIRC_BUF_SIZE);
}

TEST_CASE("TaskFromBuffer - Pop from Empty Buffer") {
    TaskFromBufferExpanded task(100, 50);
    REQUIRE(task.getBufferSize() == 0);
    auto popped_transfer = task.popExpanded(); // pop from an empty buffer
    CHECK(popped_transfer == nullptr); //Expected result
}

TEST_CASE("ConcreteTaskForServer Execution") {
    std::tuple<MockAdapter> adapters;
    ConcreteTaskForServer task(100, 50, adapters);
    task.initialize(0);
    HAL_SetTick(0);

    // First check, should not execute since not enough time has passed
    HAL_SetTick(50);
    task.handleTask();
    CHECK(task.handleTaskImplCalled == false);

    // Second check, enough time has passed, should execute
    HAL_SetTick(151);
    task.handleTask();
    CHECK(task.handleTaskImplCalled == true);
}

TEST_CASE("ConcreteTaskForServer Publish") {
    std::tuple<MockAdapter> adapters;
    ConcreteTaskForServer task(100, 50, adapters);
    task.initialize(0);
    HAL_SetTick(0);
    uint8_t transfer_id = 67;

    // Call handleTaskImpl, which should call publish
    task.handleTaskImpl();

    // Check that the adapter's cyphalTxPush was called with the correct arguments
    auto& adapter = task.getAdapter();
    CHECK(adapter.timeout_us_ == 0);
    CHECK(adapter.metadata_.priority == CyphalPriorityNominal);
    CHECK(adapter.metadata_.transfer_kind == CyphalTransferKindResponse);
    CHECK(adapter.metadata_.port_id == 123);  // Verify port ID
    CHECK(adapter.metadata_.remote_node_id == 45); // Verify node ID
    CHECK(adapter.metadata_.transfer_id == transfer_id);  // Verify transfer ID
    CHECK(adapter.payload_size_ == 1);
    CHECK(adapter.payload_[0] == 0);
}

TEST_CASE("ConcreteTaskForServer - handleMessage") {
    std::tuple<MockAdapter> adapters;
    ConcreteTaskForServer task(100, 50, adapters);

    auto transfer1 = std::make_shared<CyphalTransfer>();
    uint8_t data1 = 10;
    transfer1->payload = &data1;
    transfer1->payload_size = 1;

    auto transfer2 = std::make_shared<CyphalTransfer>();
    uint8_t data2 = 20;
    transfer2->payload = &data2;
    transfer2->payload_size = 1;

    task.handleMessage(transfer1);
    task.handleMessage(transfer2);

    CHECK(task.handleMessageCalled == true);
    CHECK(task.buffer.size() == 2);
}

TEST_CASE("ConcreteTaskForServer Publish - Serialization Failure") {
    std::tuple<MockAdapter> adapters;
    ConcreteTaskForServer task(100, 50, adapters);
    task.initialize(0);
    HAL_SetTick(0);

    // Create an anonymous class derived from ConcreteTaskForServer, override handleTaskImpl
    class ConcreteTaskForServerOverride : public ConcreteTaskForServer {
    public:
        ConcreteTaskForServerOverride(uint32_t interval, uint32_t tick, std::tuple<MockAdapter>& adapters) :
            ConcreteTaskForServer(interval, tick, adapters) {}

        void handleTaskImpl() override {
            uint8_t payload[10];
            publish(sizeof(payload), payload, &data, &serialize_mock_fail, 123, 45, 67);
            handleTaskImplCalled = true;
        }
    };

    ConcreteTaskForServerOverride taskOverride(100, 50, adapters);
    taskOverride.initialize(0);
    HAL_SetTick(0);

    taskOverride.handleTaskImpl();  // Execute the task
    CHECK(taskOverride.handleTaskImplCalled == true);

    // Add additional checks as needed to verify that the error is handled correctly.
    // For example, you could check that a log message was generated.
}

TEST_CASE("ConcreteTaskForServer Publish - Zero Payload") {
    std::tuple<MockAdapter> adapters;
    ConcreteTaskForServer task(100, 50, adapters);
    task.initialize(0);
    HAL_SetTick(0);

    class ConcreteTaskForServerOverride : public ConcreteTaskForServer {
    public:
        ConcreteTaskForServerOverride(uint32_t interval, uint32_t tick, std::tuple<MockAdapter>& adapters) :
            ConcreteTaskForServer(interval, tick, adapters) {}

        void handleTaskImpl() override {
            uint8_t *payload = nullptr;
            size_t payload_size = 0;
            publish(payload_size, payload, &data, &serialize_mock, 123, 45, 67);
            handleTaskImplCalled = true;
        }
    };

    ConcreteTaskForServerOverride taskOverride(100, 50, adapters);
    taskOverride.initialize(0);
    HAL_SetTick(0);

    taskOverride.handleTaskImpl();  // Execute the task
    auto& adapter = taskOverride.getAdapter(); // Use the accessor method
    CHECK(adapter.payload_size_ == 0);
    CHECK(taskOverride.handleTaskImplCalled == true);

}

TEST_CASE("ConcreteTaskForServer Registration") {
    RegistrationManager manager;
    std::tuple<MockAdapter> adapters;
    auto task = std::make_shared<ConcreteTaskForServer>(100, 50, adapters);

    task->registerTask(&manager, task);
    REQUIRE(manager.registered_tasks.size() == 1);
    CHECK(manager.registered_tasks[0] == task);

    task->unregisterTask(&manager, task);
    CHECK(manager.registered_tasks.empty());
}