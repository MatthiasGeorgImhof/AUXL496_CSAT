#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "RegistrationManager.hpp"
#include "Task.hpp"
#include "cyphal.hpp"
#include <memory>
#include <iostream>
#include <tuple>
#include <vector>

// Define some dummy adapter types (replace with your actual adapters)
class DummyAdapter
{
public:
    DummyAdapter(int value) : value_(value), cyphalRxSubscribeCallCount(0), cyphalRxUnsubscribeCallCount(0), lastTransferKind(CyphalTransferKind::CyphalTransferKindMessage), lastPortID(0), lastExtent(0), Timeout(0) {} // Corrected here
    int getValue() const { return value_; }
    int8_t cyphalRxSubscribe(const CyphalTransferKind transfer_kind,
                             const CyphalPortID port_id,
                             const size_t extent,
                             const CyphalMicrosecond transfer_id_timeout_usec)
    {
        cyphalRxSubscribeCallCount++;
        lastTransferKind = transfer_kind;
        lastPortID = port_id;
        lastExtent = extent;
        Timeout = transfer_id_timeout_usec; // Corrected here
        return 1;                           // Dummy implementation
    }

    int8_t cyphalRxUnsubscribe(const CyphalTransferKind transfer_kind,
                               const CyphalPortID port_id)
    {
        cyphalRxUnsubscribeCallCount++;
        lastTransferKind = transfer_kind;
        lastPortID = port_id;
        lastExtent = 0;
        Timeout = 0; // Corrected here
        return 1;    // Dummy implementation
    }

private:
    int value_;
public:
    int cyphalRxSubscribeCallCount;
    int cyphalRxUnsubscribeCallCount;
    CyphalTransferKind lastTransferKind;
    CyphalPortID lastPortID;
    size_t lastExtent;
    size_t Timeout;

};

// Mock Task class that interacts with the RegistrationManager
class MockTask : public Task
{
public:
    MockTask(uint32_t interval, uint32_t tick, CyphalPortID port_id)
        : Task(interval, tick), port_id_(port_id), registered_(false), unregistered_(false) {}
    ~MockTask() override {}

    void handleMessage(std::shared_ptr<CyphalTransfer> /*transfer*/) override {}
    void handleTaskImpl() override {}

    // Override the non-template virtual functions
    void registerTask(RegistrationManager *manager_ptr, std::shared_ptr<Task> task) override
    {
        RegistrationManager *manager = static_cast<RegistrationManager *>(manager_ptr);

        manager->subscribe(port_id_, task);
        manager->publish(port_id_, task);

        registered_ = true;
    }

    void unregisterTask(RegistrationManager *manager_ptr, std::shared_ptr<Task> task) override
    {
        RegistrationManager *manager = static_cast<RegistrationManager *>(manager_ptr);

        manager->unsubscribe(port_id_, task);
        manager->unpublish(port_id_, task);

        unregistered_ = true;
    }

    bool isRegistered() const { return registered_; }
    bool isUnregistered() const { return unregistered_; }

private:
    CyphalPortID port_id_;
    bool registered_;
    bool unregistered_;
};

class MockTaskArray : public Task
{
public:
    MockTaskArray(uint32_t interval, uint32_t tick, std::vector<CyphalPortID> port_ids)
        : Task(interval, tick), port_ids_(port_ids), registered_(false), unregistered_(false) {}
    ~MockTaskArray() override {}

    void handleMessage(std::shared_ptr<CyphalTransfer> /*transfer*/) override {}
    void handleTaskImpl() override {}

    // Override the non-template virtual functions
    void registerTask(RegistrationManager *manager_ptr, std::shared_ptr<Task> task) override
    {
        RegistrationManager *manager = static_cast<RegistrationManager *>(manager_ptr);

        for (CyphalPortID port_id_ : port_ids_)
        {
            manager->subscribe(port_id_, task);
            manager->publish(port_id_, task);
        }

        registered_ = true;
    }

    void unregisterTask(RegistrationManager *manager_ptr, std::shared_ptr<Task> task) override
    {
        RegistrationManager *manager = static_cast<RegistrationManager *>(manager_ptr);

        for (CyphalPortID port_id_ : port_ids_)
        {
            manager->unsubscribe(port_id_, task);
            manager->unpublish(port_id_, task);
        }

        unregistered_ = true;
    }

    bool isRegistered() const { return registered_; }
    bool isUnregistered() const { return unregistered_; }

private:
    std::vector<CyphalPortID> port_ids_;
    bool registered_;
    bool unregistered_;
};

// Helper function to verify manager entries
template <typename T>
auto verifyManagerEntries = [](const ArrayList<T, RegistrationManager::NUM_SUBSCRIPTIONS> &entries, const T &expected_entry, size_t expected_size, bool expected_result)
{
    bool found = false;
    CHECK(entries.size() == expected_size);
    for (size_t i = 0; i < entries.size(); ++i)
    {
        if (entries[i] == expected_entry)
        {
            found = true;
            break;
        }
    }
    CHECK(found == expected_result);
};

auto verifyHandlerEntries = [](const ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> &handlers, const auto &task, CyphalPortID port_id, size_t expected_size, bool expected_result)
{
    bool found = false;
    CHECK(handlers.size() == expected_size);
    for (size_t i = 0; i < handlers.size(); ++i)
    {
        if (handlers[i].port_id == port_id && handlers[i].task == task)
        {
            found = true;
            break;
        }
    }
    CHECK(found == expected_result);
};

TEST_CASE("RegistrationManager: Register and Unregister Task")
{
    // Create a RegistrationManager with a dummy adapter
    RegistrationManager manager; // Pass the instance to the constructor
    CyphalPortID port_id = 123;
    std::shared_ptr<MockTask> task = std::make_shared<MockTask>(100, 0, port_id);

    // Initially, task should not be registered
    CHECK(task->isRegistered() == false);
    CHECK(task->isUnregistered() == false);

    // Register the task
    task->registerTask(&manager, task);

    // Verify that the subscription and handler are added to the manager
    {
        const ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> &handlers = manager.getHandlers(); // Use the getter
        verifyHandlerEntries(handlers, task, port_id, 1, true);

        CyphalPortID expected_subscription = port_id;
        const ArrayList<CyphalPortID, RegistrationManager::NUM_SUBSCRIPTIONS> &subscriptions = manager.getSubscriptions(); // Use the getter
        verifyManagerEntries<CyphalPortID>(subscriptions, expected_subscription, 1, true);                                 // Explicit template argument

        CyphalPortID expected_publication = port_id;
        const ArrayList<CyphalPortID, RegistrationManager::NUM_PUBLICATIONS> &publications = manager.getPublications(); // Use the getter
        verifyManagerEntries<CyphalPortID>(publications, expected_publication, 1, true);                                // Explicit template argument
    }

    // Now, the task should be registered (MockTask's registerTask *is* now calling subscribe)
    CHECK(task->isRegistered() == true);
    CHECK(task->isUnregistered() == false);

    // Unregister the task
    task->unregisterTask(&manager, task);

    // Now, the task should be unregistered
    CHECK(task->isRegistered() == true); // registered stays as true since the variable is not changed in the task class.
    CHECK(task->isUnregistered() == true);

    // Verify that the subscription and handler are removed from the manager
    {
        const ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> &handlers = manager.getHandlers(); // Use the getter
        verifyHandlerEntries(handlers, task, port_id, 0, false);

        CyphalPortID expected_subscription = port_id;
        const ArrayList<CyphalPortID, RegistrationManager::NUM_SUBSCRIPTIONS> &subscriptions = manager.getSubscriptions(); // Use the getter
        verifyManagerEntries<CyphalPortID>(subscriptions, expected_subscription, 0, false);                                // Explicit template argument

        CyphalPortID expected_publication = port_id;
        const ArrayList<CyphalPortID, RegistrationManager::NUM_PUBLICATIONS> &publications = manager.getPublications(); // Use the getter
        verifyManagerEntries<CyphalPortID>(publications, expected_publication, 0, false);                               // Explicit template argument
    }
}

TEST_CASE("RegistrationManager: Multiple Registrations - Same Task, Same Port")
{
    RegistrationManager manager;
    CyphalPortID port_id = 123;
    std::shared_ptr<MockTask> task = std::make_shared<MockTask>(100, 0, port_id);

    // Register the task multiple times with the same port ID
    task->registerTask(&manager, task);
    task->registerTask(&manager, task);
    task->registerTask(&manager, task);

    // Verify that only one subscription/publication exists for the task and port
    const ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> &handlers = manager.getHandlers();
    verifyHandlerEntries(handlers, task, port_id, 1, true);

    CyphalPortID expected_subscription = port_id;
    const ArrayList<CyphalPortID, RegistrationManager::NUM_SUBSCRIPTIONS> &subscriptions = manager.getSubscriptions();
    verifyManagerEntries<CyphalPortID>(subscriptions, expected_subscription, 1, true); // Explicit template argument

    CyphalPortID expected_publication = port_id;
    const ArrayList<CyphalPortID, RegistrationManager::NUM_PUBLICATIONS> &publications = manager.getPublications();
    verifyManagerEntries<CyphalPortID>(publications, expected_publication, 1, true); // Explicit template argument

    // Unregister the task once
    task->unregisterTask(&manager, task);

    // Verify that subscription/publication are removed
    const ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> &handlers_unregistered = manager.getHandlers();
    verifyHandlerEntries(handlers_unregistered, task, port_id, 0, false);

    const ArrayList<CyphalPortID, RegistrationManager::NUM_SUBSCRIPTIONS> &subscriptions_unregistered = manager.getSubscriptions();
    verifyManagerEntries<CyphalPortID>(subscriptions_unregistered, expected_subscription, 0, false); // Explicit template argument

    const ArrayList<CyphalPortID, RegistrationManager::NUM_PUBLICATIONS> &publications_unregistered = manager.getPublications();
    verifyManagerEntries<CyphalPortID>(publications_unregistered, expected_publication, 0, false); // Explicit template argument
}

TEST_CASE("RegistrationManager: Multiple Registrations - Same Task, Different Ports")
{
    RegistrationManager manager;
    CyphalPortID port_id1 = 123;
    CyphalPortID port_id2 = 456;
    std::shared_ptr<MockTaskArray> task = std::make_shared<MockTaskArray>(100, 0, std::vector<CyphalPortID>({port_id1, port_id2}));

    // Register the task with the first port ID
    task->registerTask(&manager, task);

    // Verify both subscriptions exist
    const ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> &handlers = manager.getHandlers();
    verifyHandlerEntries(handlers, task, port_id1, 2, true);
    CHECK(handlers.size() == 2);

    const ArrayList<CyphalPortID, RegistrationManager::NUM_SUBSCRIPTIONS> &subscriptions = manager.getSubscriptions();
    verifyManagerEntries<CyphalPortID>(subscriptions, port_id1, 2, true); // Explicit template argument
    verifyManagerEntries<CyphalPortID>(subscriptions, port_id2, 2, true); // Explicit template argument
    CHECK(subscriptions.size() == 2);

    const ArrayList<CyphalPortID, RegistrationManager::NUM_PUBLICATIONS> &publications = manager.getPublications();
    verifyManagerEntries<CyphalPortID>(publications, port_id1, 2, true); // Explicit template argument
    verifyManagerEntries<CyphalPortID>(publications, port_id2, 2, true); // Explicit template argument
    CHECK(publications.size() == 2);

    // Unregister both tasks
    task->unregisterTask(&manager, task);

    const ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> &handlers_unregistered = manager.getHandlers();
    verifyHandlerEntries(handlers_unregistered, task, port_id1, 0, false);
    CHECK(handlers_unregistered.size() == 0);

    const ArrayList<CyphalPortID, RegistrationManager::NUM_SUBSCRIPTIONS> &subscriptions_unregistered = manager.getSubscriptions();
    verifyManagerEntries<CyphalPortID>(subscriptions_unregistered, port_id1, 0, false); // Explicit template argument
    verifyManagerEntries<CyphalPortID>(subscriptions_unregistered, port_id2, 0, false); // Explicit template argument
    CHECK(subscriptions_unregistered.size() == 0);

    const ArrayList<CyphalPortID, RegistrationManager::NUM_PUBLICATIONS> &publications_unregistered = manager.getPublications();
    verifyManagerEntries<CyphalPortID>(publications_unregistered, port_id1, 0, false); // Explicit template argument
    verifyManagerEntries<CyphalPortID>(publications_unregistered, port_id2, 0, false); // Explicit template argument
    CHECK(publications_unregistered.size() == 0);
}

TEST_CASE("RegistrationManager: Multiple Registrations - Different Tasks, Same Port")
{
    RegistrationManager manager;
    CyphalPortID port_id = 123;
    std::shared_ptr<MockTask> task1 = std::make_shared<MockTask>(100, 0, port_id);
    std::shared_ptr<MockTask> task2 = std::make_shared<MockTask>(200, 0, port_id);

    // Register both tasks with the same port ID
    task1->registerTask(&manager, task1);
    task2->registerTask(&manager, task2);

    // Verify that two subscriptions exist
    const ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> &handlers = manager.getHandlers();
    verifyHandlerEntries(handlers, task1, port_id, 2, true);
    verifyHandlerEntries(handlers, task2, port_id, 2, true);
    CHECK(handlers.size() == 2);

    const ArrayList<CyphalPortID, RegistrationManager::NUM_SUBSCRIPTIONS> &subscriptions = manager.getSubscriptions();
    verifyManagerEntries<CyphalPortID>(subscriptions, port_id, 1, true); // Explicit template argument
    CHECK(subscriptions.size() == 1);

    const ArrayList<CyphalPortID, RegistrationManager::NUM_PUBLICATIONS> &publications = manager.getPublications();
    verifyManagerEntries<CyphalPortID>(publications, port_id, 1, true); // Explicit template argument
    CHECK(publications.size() == 1);

    // Unregister the first task
    task1->unregisterTask(&manager, task1);

    // Verify task1 is not in handlers, task2 still is, subscription still exists
    const ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> &handlers_unregistered1 = manager.getHandlers();
    verifyHandlerEntries(handlers_unregistered1, task1, port_id, 1, false);
    verifyHandlerEntries(handlers_unregistered1, task2, port_id, 1, true);
    CHECK(handlers_unregistered1.size() == 1);

    const ArrayList<CyphalPortID, RegistrationManager::NUM_SUBSCRIPTIONS> &subscriptions_unregistered1 = manager.getSubscriptions();
    verifyManagerEntries<CyphalPortID>(subscriptions_unregistered1, port_id, 1, true); // Explicit template argument
    CHECK(subscriptions_unregistered1.size() == 1);

    const ArrayList<CyphalPortID, RegistrationManager::NUM_PUBLICATIONS> &publications_unregistered1 = manager.getPublications();
    verifyManagerEntries<CyphalPortID>(publications_unregistered1, port_id, 1, true); // Explicit template argument
    CHECK(publications_unregistered1.size() == 1);

    // Unregister the second task
    task2->unregisterTask(&manager, task2);

    // Verify subscription is removed
    const ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> &handlers_unregistered2 = manager.getHandlers();
    verifyHandlerEntries(handlers_unregistered2, task1, port_id, 0, false);
    verifyHandlerEntries(handlers_unregistered2, task2, port_id, 0, false);
    CHECK(handlers_unregistered2.size() == 0);

    const ArrayList<CyphalPortID, RegistrationManager::NUM_SUBSCRIPTIONS> &subscriptions_unregistered2 = manager.getSubscriptions();
    verifyManagerEntries<CyphalPortID>(subscriptions_unregistered2, port_id, 0, false); // Explicit template argument
    CHECK(subscriptions_unregistered2.size() == 0);

    const ArrayList<CyphalPortID, RegistrationManager::NUM_PUBLICATIONS> &publications_unregistered2 = manager.getPublications();
    verifyManagerEntries<CyphalPortID>(publications_unregistered2, port_id, 0, false); // Explicit template argument
    CHECK(publications_unregistered2.size() == 0);
}

TEST_CASE("RegistrationManager: Multiple Registrations - Different Task, Different Ports")
{
    RegistrationManager manager;
    CyphalPortID port_id1 = 123;
    CyphalPortID port_id2 = 456;
    std::shared_ptr<MockTask> task = std::make_shared<MockTask>(100, 0, port_id1);

    // Register the task with the first port ID
    task->registerTask(&manager, task);

    // Create a new task with the second port and register
    std::shared_ptr<MockTask> task2 = std::make_shared<MockTask>(100, 0, port_id2);
    task2->registerTask(&manager, task2);

    // Verify both subscriptions exist
    const ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> &handlers = manager.getHandlers();
    verifyHandlerEntries(handlers, task, port_id1, 2, true);
    verifyHandlerEntries(handlers, task2, port_id2, 2, true);
    CHECK(handlers.size() == 2);

    const ArrayList<CyphalPortID, RegistrationManager::NUM_SUBSCRIPTIONS> &subscriptions = manager.getSubscriptions();
    verifyManagerEntries<CyphalPortID>(subscriptions, port_id1, 2, true);
    verifyManagerEntries<CyphalPortID>(subscriptions, port_id2, 2, true);
    CHECK(subscriptions.size() == 2);

    const ArrayList<CyphalPortID, RegistrationManager::NUM_PUBLICATIONS> &publications = manager.getPublications();
    verifyManagerEntries<CyphalPortID>(publications, port_id1, 2, true);
    verifyManagerEntries<CyphalPortID>(publications, port_id2, 2, true);
    CHECK(publications.size() == 2);

    //Unregister both tasks
    task->unregisterTask(&manager, task);
    task2->unregisterTask(&manager, task2);

    const ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> &handlers_unregistered = manager.getHandlers();
    verifyHandlerEntries(handlers_unregistered, task, port_id1, 0, false);
    verifyHandlerEntries(handlers_unregistered, task2, port_id2, 0, false);
    CHECK(handlers_unregistered.size() == 0);

    const ArrayList<CyphalPortID, RegistrationManager::NUM_SUBSCRIPTIONS> &subscriptions_unregistered = manager.getSubscriptions();
    verifyManagerEntries<CyphalPortID>(subscriptions_unregistered, port_id1, 0, false);
    verifyManagerEntries<CyphalPortID>(subscriptions_unregistered, port_id2, 0, false);
    CHECK(subscriptions_unregistered.size() == 0);

    const ArrayList<CyphalPortID, RegistrationManager::NUM_PUBLICATIONS> &publications_unregistered = manager.getPublications();
    verifyManagerEntries<CyphalPortID>(publications_unregistered, port_id1, 0, false);
    verifyManagerEntries<CyphalPortID>(publications_unregistered, port_id2, 0, false);
    CHECK(publications_unregistered.size() == 0);
}

TEST_CASE("RegistrationManager: Exceeding Capacity")
{
    RegistrationManager manager;
    std::array<std::shared_ptr<MockTask>, RegistrationManager::NUM_TASK_HANDLERS + 1> tasks;

    // Create more tasks than the handler capacity
    for (size_t i = 0; i < tasks.size(); ++i)
    {
        tasks[i] = std::make_shared<MockTask>(100, 0, static_cast<CyphalPortID>(i + 1));
        tasks[i]->registerTask(&manager, tasks[i]);
    }

    // Verify the handler list is full
    const ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> &handlers = manager.getHandlers();
    CHECK(handlers.size() == RegistrationManager::NUM_TASK_HANDLERS);

    // Verify that subscriptions and publications lists also reach their capacity
    const ArrayList<CyphalPortID, RegistrationManager::NUM_SUBSCRIPTIONS> &subscriptions = manager.getSubscriptions();
    CHECK(subscriptions.size() == RegistrationManager::NUM_SUBSCRIPTIONS);

    const ArrayList<CyphalPortID, RegistrationManager::NUM_PUBLICATIONS> &publications = manager.getPublications();
    CHECK(publications.size() == RegistrationManager::NUM_PUBLICATIONS);
}

constexpr CyphalPortID CYPHALPORT = 129;

void checkTransfers(const CyphalTransfer transfer1, const CyphalTransfer transfer2)
{
    CHECK(transfer1.metadata.port_id == transfer2.metadata.port_id);
    CHECK(transfer1.payload_size == transfer2.payload_size);
    CHECK(strncmp(static_cast<const char *>(transfer1.payload), static_cast<const char *>(transfer2.payload), transfer1.payload_size) == 0);
}

class BasicTaskFromBuffer : public TaskFromBuffer
{
public:
    BasicTaskFromBuffer(uint32_t interval, uint32_t tick, const CyphalTransfer transfer) : TaskFromBuffer(interval, tick), transfer_(transfer) {}
    ~BasicTaskFromBuffer() override {}

    void handleTaskImpl() override
    {
        CHECK(buffer_.size() == 1);

        for (size_t i = 0; i < buffer_.size(); ++i)
        {
            auto transfer = buffer_.pop();
            checkTransfers(transfer_, *transfer);
        }
    }

    void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
    {
        manager->subscribe(CYPHALPORT, task);
        manager->publish(CYPHALPORT, task);
    }

    void unregisterTask(RegistrationManager */*manager*/, std::shared_ptr<Task> /*task*/) {}

    CyphalTransfer transfer_;
};

TEST_CASE("RegistrationManager: TaskFromBuffer")
{
    constexpr CyphalPortID port_id = 123;
    CyphalTransfer transfer;
    transfer.metadata.priority = CyphalPriorityNominal;
    transfer.metadata.transfer_kind = CyphalTransferKindMessage;
    transfer.metadata.port_id = port_id;
    transfer.metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    transfer.metadata.transfer_id = 0;
    transfer.payload_size = 5;
    char payload[6] = "hello";
    transfer.payload = payload;

    RegistrationManager manager;
    std::shared_ptr<BasicTaskFromBuffer> basic_task_buffer = std::make_shared<BasicTaskFromBuffer>(100, 0, transfer);
    HAL_SetTick(1000);

    basic_task_buffer->registerTask(&manager, basic_task_buffer);
    const ArrayList<::TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> &handlers = manager.getHandlers();
    CHECK(handlers.size() == 1);

    std::shared_ptr<CyphalTransfer> transfer_ptr = std::make_shared<CyphalTransfer>(transfer);
    CHECK(transfer_ptr.use_count() == 1);
    for (auto handler : handlers)
        handler.task->handleMessage(transfer_ptr);
    CHECK(transfer_ptr.use_count() == 2);
    for (auto handler : handlers)
        handler.task->handleTask();
    CHECK(transfer_ptr.use_count() == 1);
}