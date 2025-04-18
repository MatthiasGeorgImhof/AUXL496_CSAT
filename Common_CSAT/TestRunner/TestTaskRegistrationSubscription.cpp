#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <memory>
#include <tuple>
#include <vector>

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "SubscriptionManager.hpp"
#include "cyphal.hpp"
#include "cyphal_subscriptions.hpp"
#include "mock_hal.h"

// Mock HAL for testing
namespace mock_hal {
    uint32_t tick = 0;
    uint32_t HAL_GetTick() { return tick; }
    void HAL_SetTick(uint32_t t) { tick = t; }
}

// Mock Cyphal Adapter for testing
class MockCyphalAdapter {
public:
    int8_t cyphalTxPush(CyphalMicrosecond, const CyphalTransferMetadata *metadata, size_t size, const uint8_t *data) {
        last_tx_metadata = *metadata;
        last_tx_data.assign(data, data + size);
        return 1; // Simulate success
    }
    int8_t cyphalRxSubscribe(CyphalTransferKind transfer_kind, CyphalPortID port_id, size_t extent, CyphalMicrosecond timeout) {
        (void)transfer_kind;
        (void)port_id;
        (void)extent;
        (void)timeout;
        return 1; // Simulate success
    }
    int8_t cyphalRxUnsubscribe(CyphalTransferKind transfer_kind, CyphalPortID port_id) {
        (void)transfer_kind;
        (void)port_id;
        return 1; // Simulate success
    }

    CyphalTransferMetadata last_tx_metadata;
    std::vector<uint8_t> last_tx_data;
};

// Mock TaskFromBuffer implementation
class MockTaskFromBuffer : public TaskFromBuffer {
public:
    MockTaskFromBuffer(uint32_t interval, uint32_t tick) : TaskFromBuffer(interval, tick) {}

    void handleMessage(std::shared_ptr<CyphalTransfer> /*transfer*/) override {} // No-op

    void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override {
        if (manager) {
            manager->subscribe(port_id_, task);  // Subscribe
        }
    }

    void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override {
        if (manager) {
            manager->unsubscribe(port_id_, task); // Unsubscribe
        }
    }

protected:
    void handleTaskImpl() override {} // No-op

public:
    CyphalPortID port_id_ = 0;              // Port ID to use for publishing
};

// Mock TaskForServer implementation
class MockTaskForServer : public TaskForServer<MockCyphalAdapter> {
public:
    MockTaskForServer(uint32_t interval, uint32_t tick, std::tuple<MockCyphalAdapter>& adapters)
        : TaskForServer(interval, tick, adapters) {}

protected:
    void handleTaskImpl() override {}

public:
    void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override {
        if (manager) {
            manager->server(port_id_, task); // Register as server
        }
    }

    void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override {
        if (manager) {
            manager->unserver(port_id_, task); // Unregister as server
        }
    }

public:
    CyphalPortID port_id_ = 0; // Port ID for server
};

// Mock TaskForClient implementation
class MockTaskForClient : public TaskForClient<MockCyphalAdapter> {
public:
    MockTaskForClient(uint32_t interval, uint32_t tick, CyphalNodeID server_node_id, CyphalTransferID transfer_id, std::tuple<MockCyphalAdapter>& adapters)
        : TaskForClient(interval, tick, server_node_id, transfer_id, adapters) {}

    void handleTaskImpl() override {}

protected:
    void handleMessage(std::shared_ptr<CyphalTransfer> /*transfer*/) override {}

public:
    void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override {
        // Client tasks *publish* requests and *subscribe* to responses.
        if (manager) {
            manager->client(port_id_, task);
        }
    }

    void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override {
        if (manager) {
            manager->unclient(port_id_, task);
        }
    }

public:
    CyphalPortID port_id_ = 0; // Port ID for client requests
};


TEST_CASE("TaskFromBuffer Registration and Subscription") {
    constexpr uint32_t interval = 100;
    constexpr uint32_t tick = 0;
    constexpr CyphalPortID port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_;

    const CyphalSubscription* subscription = findMessageByPortIdRuntime(port_id);
    REQUIRE(subscription != nullptr);

    MockCyphalAdapter adapter;
    std::tuple<MockCyphalAdapter> adapters(adapter);

    std::shared_ptr<MockTaskFromBuffer> task = std::make_shared<MockTaskFromBuffer>(interval, tick);
    task->port_id_ = port_id;

    RegistrationManager registration_manager;
    registration_manager.add(task);
    REQUIRE(registration_manager.getHandlers().size() == 1);
    REQUIRE(registration_manager.getSubscriptions().size() == 1);
    REQUIRE(registration_manager.getPublications().size() == 0);
    REQUIRE(registration_manager.getServers().size() == 0);
    REQUIRE(registration_manager.getClients().size() == 0);
    REQUIRE(registration_manager.getHandlers()[0].task == task);
    REQUIRE(registration_manager.getHandlers()[0].port_id == port_id);

    SubscriptionManager subscription_manager;
    REQUIRE(subscription_manager.getSubscriptions().size() == 0);
    subscription_manager.subscribe<SubscriptionManager::MessageTag>(registration_manager.getSubscriptions(), adapters);
    subscription_manager.subscribe<SubscriptionManager::RequestTag>(registration_manager.getServers(), adapters);
    subscription_manager.subscribe<SubscriptionManager::ResponseTag>(registration_manager.getClients(), adapters);

    REQUIRE(subscription_manager.getSubscriptions().size() == 1);
    REQUIRE(subscription_manager.getSubscriptions()[0]->port_id == subscription->port_id);
    REQUIRE(subscription_manager.getSubscriptions()[0]->extent == subscription->extent);
    REQUIRE(subscription_manager.getSubscriptions()[0]->transfer_kind == subscription->transfer_kind);
    REQUIRE(subscription_manager.getSubscriptions()[0] == subscription);
}

TEST_CASE("uavcan_node_GetInfo_1_0_FIXED_PORT_ID_ sanity check")
{
    constexpr CyphalPortID port_id = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_;
    REQUIRE(findRequestByPortIdRuntime(port_id) != findResponseByPortIdRuntime(port_id));
}

TEST_CASE("TaskForServer Registration and Subscription") {
    constexpr uint32_t interval = 50;
    constexpr uint32_t tick = 0;
    constexpr CyphalPortID port_id = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_;

    const CyphalSubscription* subscription = findRequestByPortIdRuntime(port_id);
    REQUIRE(subscription != nullptr);
    REQUIRE(subscription != findResponseByPortIdRuntime(port_id));

    MockCyphalAdapter adapter;
    std::tuple<MockCyphalAdapter> adapters(adapter);

    std::shared_ptr<MockTaskForServer> task = std::make_shared<MockTaskForServer>(interval, tick, adapters);
    task->port_id_ = port_id;

    RegistrationManager registration_manager;
    registration_manager.add(task);
    REQUIRE(registration_manager.getHandlers().size() == 1);
    REQUIRE(registration_manager.getSubscriptions().size() == 0);
    REQUIRE(registration_manager.getPublications().size() == 0);
    REQUIRE(registration_manager.getServers().size() == 1);
    REQUIRE(registration_manager.getClients().size() == 0);
    REQUIRE(registration_manager.getHandlers()[0].task == task);
    REQUIRE(registration_manager.getHandlers()[0].port_id == port_id);


	SubscriptionManager subscription_manager;
    REQUIRE(subscription_manager.getSubscriptions().size() == 0);
    subscription_manager.subscribe<SubscriptionManager::MessageTag>(registration_manager.getSubscriptions(), adapters);
    subscription_manager.subscribe<SubscriptionManager::RequestTag>(registration_manager.getServers(), adapters);
    subscription_manager.subscribe<SubscriptionManager::ResponseTag>(registration_manager.getClients(), adapters);

    REQUIRE(subscription_manager.getSubscriptions().size() == 1);
    REQUIRE(subscription_manager.getSubscriptions()[0]->port_id == subscription->port_id);
    REQUIRE(subscription_manager.getSubscriptions()[0]->extent == subscription->extent);
    REQUIRE(subscription_manager.getSubscriptions()[0]->transfer_kind == subscription->transfer_kind);
    REQUIRE(subscription_manager.getSubscriptions()[0] == subscription);
}

TEST_CASE("TaskForClient Registration and Subscription") {
    constexpr uint32_t interval = 75;
    constexpr uint32_t tick = 0;
    constexpr CyphalPortID port_id = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_;
    constexpr CyphalNodeID server_node_id = 1;
    constexpr CyphalTransferID transfer_id = 0; 

    const CyphalSubscription* subscription = findResponseByPortIdRuntime(port_id);
    REQUIRE(subscription != nullptr);
    REQUIRE(subscription != findRequestByPortIdRuntime(port_id));

    MockCyphalAdapter adapter;
    std::tuple<MockCyphalAdapter> adapters(adapter);

    std::shared_ptr<MockTaskForClient> task = std::make_shared<MockTaskForClient>(interval, tick, server_node_id, transfer_id, adapters);
    task->port_id_ = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_;

    RegistrationManager registration_manager;
    registration_manager.add(task);
    REQUIRE(registration_manager.getHandlers().size() == 1);
    REQUIRE(registration_manager.getSubscriptions().size() == 0);
    REQUIRE(registration_manager.getPublications().size() == 0);
    REQUIRE(registration_manager.getServers().size() == 0);
    REQUIRE(registration_manager.getClients().size() == 1);
    REQUIRE(registration_manager.getHandlers()[0].task == task);
    REQUIRE(registration_manager.getHandlers()[0].port_id == port_id);

	SubscriptionManager subscription_manager;
    REQUIRE(subscription_manager.getSubscriptions().size() == 0);
    subscription_manager.subscribe<SubscriptionManager::MessageTag>(registration_manager.getSubscriptions(), adapters);
    subscription_manager.subscribe<SubscriptionManager::RequestTag>(registration_manager.getServers(), adapters);
    subscription_manager.subscribe<SubscriptionManager::ResponseTag>(registration_manager.getClients(), adapters);

    REQUIRE(subscription_manager.getSubscriptions().size() == 1);
    REQUIRE(subscription_manager.getSubscriptions()[0]->port_id == subscription->port_id);
    REQUIRE(subscription_manager.getSubscriptions()[0]->extent == subscription->extent);
    REQUIRE(subscription_manager.getSubscriptions()[0]->transfer_kind == subscription->transfer_kind);
    REQUIRE(subscription_manager.getSubscriptions()[0] == subscription);
}