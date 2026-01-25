#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "ServiceManager.hpp"
#include "RegistrationManager.hpp"
#include <memory>
#include <iostream> // For cout
#include "Task.hpp"
#include "cyphal.hpp"

class RegistrationManager;

// DummyAdapter
class DummyAdapter
{
public:
    DummyAdapter(int value) : value_(value) {}
    int getValue() const { return value_; }

private:
    int value_;
};

// Mock Task class that interacts with the RegistrationManager
class MockTask : public Task
{
public:
    MockTask(uint32_t interval, uint32_t tick) : Task(interval, tick), message_handled(false), task_handled(false) {}
    ~MockTask() override {}

    void handleMessage(std::shared_ptr<CyphalTransfer> transfer) override
    {
        message_handled = true;
        last_transfer = transfer;
    }

    void handleTaskImpl() override
    {
        task_handled = true;
    }

    // Correct override signature (now taking void* as the manager)
    void registerTask(RegistrationManager * /*manager*/, std::shared_ptr<Task> /*task*/) override {}
    void unregisterTask(RegistrationManager * /*manager*/, std::shared_ptr<Task> /*task*/) override {}

    bool message_handled;
    bool task_handled;
    std::shared_ptr<CyphalTransfer> last_transfer;
};

TEST_CASE("ServiceManager: Initialization")
{
    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    std::shared_ptr<MockTask> task1 = std::make_shared<MockTask>(10, 0);
    std::shared_ptr<MockTask> task2 = std::make_shared<MockTask>(20, 5);

    handlers.push(TaskHandler{100, task1});
    handlers.push(TaskHandler{200, task2});

    ServiceManager manager(handlers);

    uint32_t now = 1000;
    manager.initializeServices(now);

    CHECK(task1->getLastTick() == now);
    CHECK(task2->getLastTick() == now + 5);
}

TEST_CASE("ServiceManager: Handle Message")
{
    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    std::shared_ptr<MockTask> task1 = std::make_shared<MockTask>(10, 0);
    std::shared_ptr<MockTask> task2 = std::make_shared<MockTask>(20, 5);

    handlers.push(TaskHandler{100, task1});
    handlers.push(TaskHandler{200, task2});

    ServiceManager manager(handlers);

    // Create a dummy transfer
    CyphalTransferMetadata metadata = {
        CyphalPriorityNominal,
        CyphalTransferKindMessage,
        100, // Matching port ID
        CYPHAL_NODE_ID_UNSET,
        CYPHAL_NODE_ID_UNSET,
        CYPHAL_NODE_ID_UNSET,
        1,
    };
    auto transfer = std::make_shared<CyphalTransfer>();
    transfer->metadata = metadata;

    manager.handleMessage(transfer);

    CHECK(task1->message_handled == true);
    CHECK(task1->last_transfer->metadata.port_id == 100);
    CHECK(task2->message_handled == false);

    // Send another transfer to task2

    CyphalTransferMetadata metadata2 = {
        CyphalPriorityNominal,
        CyphalTransferKindMessage,
        200, // Matching port ID
        CYPHAL_NODE_ID_UNSET,
        CYPHAL_NODE_ID_UNSET,
        CYPHAL_NODE_ID_UNSET,
        1,
    };
    auto transfer2 = std::make_shared<CyphalTransfer>();
    transfer2->metadata = metadata2;

    task1->message_handled = false; // reset flag

    manager.handleMessage(transfer2);

    CHECK(task1->message_handled == false);
    CHECK(task2->message_handled == true);
    CHECK(task2->last_transfer->metadata.port_id == 200);

    // Send another transfer to port id not in handler
    CyphalTransferMetadata metadata3 = {
        CyphalPriorityNominal,
        CyphalTransferKindMessage,
        300, // Matching port ID
        CYPHAL_NODE_ID_UNSET,
        CYPHAL_NODE_ID_UNSET,
        CYPHAL_NODE_ID_UNSET,
        1,
    };
    auto transfer3 = std::make_shared<CyphalTransfer>();
    transfer3->metadata = metadata3;

    task1->message_handled = false; // reset flag
    task2->message_handled = false; // reset flag

    manager.handleMessage(transfer3);

    CHECK(task1->message_handled == false);
    CHECK(task2->message_handled == false);
}

TEST_CASE("ServiceManager: Handle Services")
{
    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers;
    std::shared_ptr<MockTask> task1 = std::make_shared<MockTask>(10, 0);
    std::shared_ptr<MockTask> task2 = std::make_shared<MockTask>(20, 5);

    handlers.push(TaskHandler{100, task1});
    handlers.push(TaskHandler{200, task2});

    ServiceManager manager(handlers);

    // *IMPORTANT*: Initialize the services *before* calling handleServices
    manager.initializeServices(0); // Initialize with a starting tick of 0

    // Advance the tick to ensure the tasks are ready to run
    uint32_t now = 100; // Advance the tick
    HAL_SetTick(now);   // Set the mock tick

    manager.handleServices();

    CHECK(task1->task_handled == true);
    CHECK(task2->task_handled == true);

    // Add debugging checks if the tests fail
    if (!task1->task_handled)
    {
        std::cout << "Task1 not handled: last_tick=" << task1->getLastTick() << ", interval=" << task1->getInterval() << ", now=" << HAL_GetTick() << std::endl;
    }
    if (!task2->task_handled)
    {
        std::cout << "Task2 not handled: last_tick=" << task2->getLastTick() << ", interval=" << task2->getInterval() << ", now=" << HAL_GetTick() << std::endl;
    }
}

TEST_CASE("ServiceManager: No tasks in handler")
{
    ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> handlers; // empty handler
    ServiceManager manager(handlers);

    manager.initializeServices(1000);
    manager.handleServices();
    // handleMessage tested with empty handler in previous test case
}