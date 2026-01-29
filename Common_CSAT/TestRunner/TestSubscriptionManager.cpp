#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "SubscriptionManager.hpp"
#include "RegistrationManager.hpp"
#include "cyphal.hpp"
#include "cyphal_subscriptions.hpp"
#include <memory>
#include <iostream>
#include <tuple>
#include <vector>

// Define a dummy adapter type (replace with your actual adapter)
class DummyAdapter
{
public:
    DummyAdapter(int value) : value_(value), cyphalRxSubscribeCallCount(0), cyphalRxUnsubscribeCallCount(0), lastTransferKind(CyphalTransferKind::CyphalTransferKindMessage), lastPortID(0), lastExtent(0), lastTimeout(0) {}
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
        lastTimeout = transfer_id_timeout_usec;
        return 1; // Dummy implementation
    }

    int8_t cyphalRxUnsubscribe(const CyphalTransferKind transfer_kind,
                               const CyphalPortID port_id)
    {
        cyphalRxUnsubscribeCallCount++;
        lastTransferKind = transfer_kind;
        lastPortID = port_id;
        lastExtent = 0;
        lastTimeout = 0;
        return 1; // Dummy implementation
    }

    void resetCounts() {
        cyphalRxSubscribeCallCount = 0;
        cyphalRxUnsubscribeCallCount = 0;
        lastTransferKind = CyphalTransferKind::CyphalTransferKindMessage;
        lastPortID = 0;
        lastExtent = 0;
        lastTimeout = 0;
    }

private:
    int value_;

public:
    int cyphalRxSubscribeCallCount;
    int cyphalRxUnsubscribeCallCount;
    CyphalTransferKind lastTransferKind;
    CyphalPortID lastPortID;
    size_t lastExtent;
    CyphalMicrosecond lastTimeout;

};

// Helper function to create adapters
template <typename... Ts>
auto createAdapters(Ts&... adapters) {
    return std::tuple<Ts&...>(adapters...);
}

TEST_CASE("SubscriptionManager: Subscribe and Unsubscribe Single Message Port")
{
    SubscriptionManager manager;
    DummyAdapter adapter1(42), adapter2(43);
    auto adapters = createAdapters(adapter1, adapter2);
    const CyphalPortID port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_;

    manager.subscribe<SubscriptionManager::MessageTag>(port_id, adapters);

    const auto &subscriptions = manager.getSubscriptions();
    CHECK(subscriptions.size() == 1);
    CHECK(subscriptions[0]->port_id == port_id);
    CHECK(adapter1.cyphalRxSubscribeCallCount == 1);
    CHECK(adapter2.cyphalRxSubscribeCallCount == 1);

    manager.unsubscribe<SubscriptionManager::MessageTag>(port_id, adapters);
    CHECK(subscriptions.size() == 0);
    CHECK(adapter1.cyphalRxUnsubscribeCallCount == 1);
    CHECK(adapter2.cyphalRxUnsubscribeCallCount == 1);
}

TEST_CASE("SubscriptionManager: Subscribe and Unsubscribe Single Request Port")
{
    SubscriptionManager manager;
    DummyAdapter adapter1(42), adapter2(43);
    auto adapters = createAdapters(adapter1, adapter2);
    const CyphalPortID port_id = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_;

    manager.subscribe<SubscriptionManager::RequestTag>(port_id, adapters);

    const auto &subscriptions = manager.getSubscriptions();
    CHECK(subscriptions.size() == 1);
    CHECK(subscriptions[0]->port_id == port_id);
    CHECK(adapter1.cyphalRxSubscribeCallCount == 1);
    CHECK(adapter2.cyphalRxSubscribeCallCount == 1);

    manager.unsubscribe<SubscriptionManager::RequestTag>(port_id, adapters);
    CHECK(subscriptions.size() == 0);
    CHECK(adapter1.cyphalRxUnsubscribeCallCount == 1);
    CHECK(adapter2.cyphalRxUnsubscribeCallCount == 1);
}

TEST_CASE("SubscriptionManager: Subscribe and Unsubscribe Single Response Port")
{
    SubscriptionManager manager;
    DummyAdapter adapter1(42), adapter2(43);
    auto adapters = createAdapters(adapter1, adapter2);
    const CyphalPortID port_id = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_;

    manager.subscribe<SubscriptionManager::ResponseTag>(port_id, adapters);

    const auto &subscriptions = manager.getSubscriptions();
    CHECK(subscriptions.size() == 1);
    CHECK(subscriptions[0]->port_id == port_id);
    CHECK(adapter1.cyphalRxSubscribeCallCount == 1);
    CHECK(adapter2.cyphalRxSubscribeCallCount == 1);

    manager.unsubscribe<SubscriptionManager::ResponseTag>(port_id, adapters);
    CHECK(subscriptions.size() == 0);
    CHECK(adapter1.cyphalRxUnsubscribeCallCount == 1);
    CHECK(adapter2.cyphalRxUnsubscribeCallCount == 1);
}

TEST_CASE("SubscriptionManager: Subscribe and Unsubscribe List of Message Ports")
{
    SubscriptionManager manager;
    DummyAdapter adapter1(42), adapter2(43);
    auto adapters = createAdapters(adapter1, adapter2);

    std::vector<CyphalPortID> port_ids = {
        uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
        uavcan_node_port_List_1_0_FIXED_PORT_ID_};

    manager.subscribe<SubscriptionManager::MessageTag, std::vector<CyphalPortID>>(port_ids, adapters);

    const auto &subscriptions = manager.getSubscriptions();
    CHECK(subscriptions.size() == 2);
    CHECK(subscriptions[0]->port_id == port_ids[0]);
    CHECK(subscriptions[1]->port_id == port_ids[1]);
    CHECK(adapter1.cyphalRxSubscribeCallCount == 2);
    CHECK(adapter2.cyphalRxSubscribeCallCount == 2);

    manager.unsubscribe<SubscriptionManager::MessageTag, std::vector<CyphalPortID>>(port_ids, adapters);
    CHECK(subscriptions.size() == 0);
    CHECK(adapter1.cyphalRxUnsubscribeCallCount == 2);
    CHECK(adapter2.cyphalRxUnsubscribeCallCount == 2);
}

TEST_CASE("SubscriptionManager: Subscribe to non existent port")
{
    SubscriptionManager manager;
    DummyAdapter adapter1(42), adapter2(43);
    auto adapters = createAdapters(adapter1, adapter2);
    const CyphalPortID port_id = 65535;  // Non-existent port

    manager.subscribe<SubscriptionManager::MessageTag>(port_id, adapters);

    const auto &subscriptions = manager.getSubscriptions();
    CHECK(subscriptions.size() == 0);
    CHECK(adapter1.cyphalRxSubscribeCallCount == 0);
    CHECK(adapter2.cyphalRxSubscribeCallCount == 0);

    manager.unsubscribe<SubscriptionManager::MessageTag>(port_id, adapters);
    CHECK(subscriptions.size() == 0);
    CHECK(adapter1.cyphalRxUnsubscribeCallCount == 0);
    CHECK(adapter2.cyphalRxUnsubscribeCallCount == 0);
}

TEST_CASE("SubscriptionManager: Multiple Subscriptions and Unsubscriptions") {
    SubscriptionManager manager;
    DummyAdapter adapter1(42), adapter2(43);
    auto adapters = createAdapters(adapter1, adapter2);
    const CyphalPortID heartbeat_port = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_;
    const CyphalPortID getinfo_port = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_;

    // Subscribe to Heartbeat
    manager.subscribe<SubscriptionManager::MessageTag>(heartbeat_port, adapters);
    CHECK(manager.getSubscriptions().size() == 1);
    CHECK(adapter1.cyphalRxSubscribeCallCount == 1);
    CHECK(adapter2.cyphalRxSubscribeCallCount == 1);

    // Subscribe to GetInfo (Request)
    manager.subscribe<SubscriptionManager::RequestTag>(getinfo_port, adapters);
    CHECK(manager.getSubscriptions().size() == 2);
    CHECK(adapter1.cyphalRxSubscribeCallCount == 2);
    CHECK(adapter2.cyphalRxSubscribeCallCount == 2);

    // Unsubscribe from Heartbeat
    manager.unsubscribe<SubscriptionManager::MessageTag>(heartbeat_port, adapters);
    CHECK(manager.getSubscriptions().size() == 1);
    CHECK(adapter1.cyphalRxUnsubscribeCallCount == 1);
    CHECK(adapter2.cyphalRxUnsubscribeCallCount == 1);

    // Unsubscribe from GetInfo
    manager.unsubscribe<SubscriptionManager::RequestTag>(getinfo_port, adapters);
    CHECK(manager.getSubscriptions().size() == 0);
    CHECK(adapter1.cyphalRxUnsubscribeCallCount == 2);
    CHECK(adapter2.cyphalRxUnsubscribeCallCount == 2);
}

TEST_CASE("SubscriptionManager: Correct Transfer Kind, Extent and Timeout are passed to adapters") {
    SubscriptionManager manager;
    DummyAdapter adapter1(42), adapter2(43);
    auto adapters = createAdapters(adapter1, adapter2);
    const CyphalPortID heartbeat_port = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_;

    // Subscribe to Heartbeat
    manager.subscribe<SubscriptionManager::MessageTag>(heartbeat_port, adapters);

    // Validate that the subscription information was correctly passed to the adapters
    CHECK(adapter1.lastPortID == heartbeat_port);
    CHECK(adapter1.lastExtent == uavcan_node_Heartbeat_1_0_EXTENT_BYTES_);
    CHECK(adapter1.lastTransferKind == CyphalTransferKindMessage);
    CHECK(adapter1.lastTimeout == 1000);  // As defined in your subscribe method

    CHECK(adapter2.lastPortID == heartbeat_port);
    CHECK(adapter2.lastExtent == uavcan_node_Heartbeat_1_0_EXTENT_BYTES_);
    CHECK(adapter2.lastTransferKind == CyphalTransferKindMessage);
    CHECK(adapter2.lastTimeout == 1000);

    // Unsubscribe from Heartbeat and then reset state in adapter
    manager.unsubscribe<SubscriptionManager::MessageTag>(heartbeat_port, adapters);

    //Reset the state in the adapter
    adapter1.resetCounts();
    adapter2.resetCounts();

    const CyphalPortID getinfo_port = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_;
    // Subscribe to GetInfo (Request)
    manager.subscribe<SubscriptionManager::RequestTag>(getinfo_port, adapters);

    CHECK(adapter1.lastPortID == getinfo_port);
    CHECK(adapter1.lastExtent == uavcan_node_GetInfo_Request_1_0_EXTENT_BYTES_);
    CHECK(adapter1.lastTransferKind == CyphalTransferKindRequest);
    CHECK(adapter1.lastTimeout == 1000);  // As defined in your subscribe method

    CHECK(adapter2.lastPortID == getinfo_port);
    CHECK(adapter2.lastExtent == uavcan_node_GetInfo_Request_1_0_EXTENT_BYTES_);
    CHECK(adapter2.lastTransferKind == CyphalTransferKindRequest);
    CHECK(adapter2.lastTimeout == 1000);
}

TEST_CASE("SubscriptionManager: Subscribe and Unsubscribe using CyphalSubscription* directly")
{
    SubscriptionManager manager;
    DummyAdapter adapter1(42), adapter2(43);
    auto adapters = createAdapters(adapter1, adapter2);

    // Get the CyphalSubscription for Heartbeat message
    const CyphalSubscription* heartbeat_subscription = findMessageByPortIdRuntime(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
    REQUIRE(heartbeat_subscription != nullptr); // Ensure the subscription is found

    // Subscribe using the CyphalSubscription*
    manager.subscribe(heartbeat_subscription, adapters);

    const auto &subscriptions = manager.getSubscriptions();
    CHECK(subscriptions.size() == 1);
    CHECK(subscriptions[0] == heartbeat_subscription);
    CHECK(adapter1.cyphalRxSubscribeCallCount == 1);
    CHECK(adapter2.cyphalRxSubscribeCallCount == 1);
    CHECK(adapter1.lastPortID == heartbeat_subscription->port_id);
    CHECK(adapter2.lastPortID == heartbeat_subscription->port_id);


    // Unsubscribe using the CyphalSubscription*
    manager.unsubscribe(heartbeat_subscription, adapters);

    CHECK(subscriptions.size() == 0);
    CHECK(adapter1.cyphalRxUnsubscribeCallCount == 1);
    CHECK(adapter2.cyphalRxUnsubscribeCallCount == 1);
    CHECK(adapter1.lastPortID == heartbeat_subscription->port_id);
    CHECK(adapter2.lastPortID == heartbeat_subscription->port_id);

}

TEST_CASE("SubscriptionManager: Subscribe and Unsubscribe using CyphalSubscription* when subscription not found")
{
    SubscriptionManager manager;
    DummyAdapter adapter1(42), adapter2(43);
    auto adapters = createAdapters(adapter1, adapter2);

    //Craft a CyphalSubscription that will not be found by the manager.
    CyphalSubscription bad_sub{65000, 100, CyphalTransferKindMessage};

    // Subscribe using the CyphalSubscription*
    manager.subscribe(&bad_sub, adapters);

    const auto &subscriptions = manager.getSubscriptions();
    CHECK(subscriptions.size() == 1);
    CHECK(subscriptions[0] == &bad_sub);
    CHECK(adapter1.cyphalRxSubscribeCallCount == 1);
    CHECK(adapter2.cyphalRxSubscribeCallCount == 1);
    CHECK(adapter1.lastPortID == bad_sub.port_id);
    CHECK(adapter2.lastPortID == bad_sub.port_id);

    // Unsubscribe using the CyphalSubscription*
    manager.unsubscribe(&bad_sub, adapters);

    CHECK(subscriptions.size() == 0);
    CHECK(adapter1.cyphalRxUnsubscribeCallCount == 1);
    CHECK(adapter2.cyphalRxUnsubscribeCallCount == 1);
    CHECK(adapter1.lastPortID == bad_sub.port_id);
    CHECK(adapter2.lastPortID == bad_sub.port_id);
}

TEST_CASE("SubscriptionManager: Subscribe and Unsubscribe Message")
{
    SubscriptionManager manager;
    DummyAdapter adapter1(42), adapter2(43);
    auto adapters = createAdapters(adapter1, adapter2);
    constexpr CyphalPortID port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_;

    const CyphalSubscription* subscription = findMessageByPortIdRuntime(port_id);
    REQUIRE(subscription == findMessageByPortIdCompileTime<port_id>());
    REQUIRE(subscription != nullptr); // Ensure the subscription is found

    // Subscribe using the CyphalSubscription*
    manager.subscribe(subscription, adapters);
    const auto &subscriptions = manager.getSubscriptions();
    CHECK(subscriptions.size() == 1);
    CHECK(subscriptions[0] == subscription);
    CHECK(subscriptions[0]->port_id == subscription->port_id);
    CHECK(subscriptions[0]->extent == subscription->extent);
    CHECK(subscriptions[0]->extent == uavcan_node_Heartbeat_1_0_EXTENT_BYTES_);
    CHECK(subscriptions[0]->transfer_kind == subscription->transfer_kind);
    
    // Unsubscribe using the CyphalSubscription*
    manager.unsubscribe(subscription, adapters);
    CHECK(subscriptions.size() == 0);

    manager.subscribe<SubscriptionManager::MessageTag>(port_id, adapters);
    const auto &subscriptions_ = manager.getSubscriptions();
    CHECK(subscriptions_.size() == 1);
    CHECK(subscriptions_[0] == subscription);
    CHECK(subscriptions_[0]->port_id == subscription->port_id);
    CHECK(subscriptions_[0]->extent == subscription->extent);
    CHECK(subscriptions_[0]->extent == uavcan_node_Heartbeat_1_0_EXTENT_BYTES_);
    CHECK(subscriptions_[0]->transfer_kind == subscription->transfer_kind);
}

TEST_CASE("SubscriptionManager: Subscribe and Unsubscribe Request")
{
    SubscriptionManager manager;
    DummyAdapter adapter1(42), adapter2(43);
    auto adapters = createAdapters(adapter1, adapter2);
    constexpr CyphalPortID port_id = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_;

    const CyphalSubscription* subscription = findRequestByPortIdRuntime(port_id);
    REQUIRE(subscription == findRequestByPortIdCompileTime<port_id>());
    REQUIRE(subscription != nullptr); // Ensure the subscription is found

    // Subscribe using the CyphalSubscription*
    manager.subscribe(subscription, adapters);
    const auto &subscriptions = manager.getSubscriptions();
    CHECK(subscriptions.size() == 1);
    CHECK(subscriptions[0] == subscription);
    CHECK(subscriptions[0]->port_id == subscription->port_id);
    CHECK(subscriptions[0]->extent == subscription->extent);
    CHECK(subscriptions[0]->extent == uavcan_node_GetInfo_Request_1_0_EXTENT_BYTES_);
    CHECK(subscriptions[0]->transfer_kind == subscription->transfer_kind);
    
    // Unsubscribe using the CyphalSubscription*
    manager.unsubscribe(subscription, adapters);
    CHECK(subscriptions.size() == 0);

    manager.subscribe<SubscriptionManager::RequestTag>(port_id, adapters);
    const auto &subscriptions_ = manager.getSubscriptions();
    CHECK(subscriptions_.size() == 1);
    CHECK(subscriptions_[0] == subscription);
    CHECK(subscriptions_[0]->port_id == subscription->port_id);
    CHECK(subscriptions_[0]->extent == subscription->extent);
    CHECK(subscriptions_[0]->extent == uavcan_node_GetInfo_Request_1_0_EXTENT_BYTES_);
    CHECK(subscriptions_[0]->transfer_kind == subscription->transfer_kind);
}

TEST_CASE("SubscriptionManager: Subscribe and Unsubscribe Response")
{
    SubscriptionManager manager;
    DummyAdapter adapter1(42), adapter2(43);
    auto adapters = createAdapters(adapter1, adapter2);
    constexpr CyphalPortID port_id = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_;

    const CyphalSubscription* subscription = findResponseByPortIdRuntime(port_id);
    REQUIRE(subscription == findResponseByPortIdCompileTime<port_id>());
    REQUIRE(subscription != nullptr); // Ensure the subscription is found

    // Subscribe using the CyphalSubscription*
    manager.subscribe(subscription, adapters);
    const auto &subscriptions = manager.getSubscriptions();
    CHECK(subscriptions.size() == 1);
    CHECK(subscriptions[0] == subscription);
    CHECK(subscriptions[0]->port_id == subscription->port_id);
    CHECK(subscriptions[0]->extent == subscription->extent);
    CHECK(subscriptions[0]->extent == uavcan_node_GetInfo_Response_1_0_EXTENT_BYTES_);
    CHECK(subscriptions[0]->transfer_kind == subscription->transfer_kind);
    
    // Unsubscribe using the CyphalSubscription*
    manager.unsubscribe(subscription, adapters);
    CHECK(subscriptions.size() == 0);

    manager.subscribe<SubscriptionManager::ResponseTag>(port_id, adapters);
    const auto &subscriptions_ = manager.getSubscriptions();
    CHECK(subscriptions_.size() == 1);
    CHECK(subscriptions_[0] == subscription);
    CHECK(subscriptions_[0]->port_id == subscription->port_id);
    CHECK(subscriptions_[0]->extent == subscription->extent);
    CHECK(subscriptions_[0]->extent == uavcan_node_GetInfo_Response_1_0_EXTENT_BYTES_);
    CHECK(subscriptions_[0]->transfer_kind == subscription->transfer_kind);
}

