#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "SubscriptionManager.hpp"
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

private:
    int value_;

    public:
    int cyphalRxSubscribeCallCount;
    int cyphalRxUnsubscribeCallCount;
    CyphalTransferKind lastTransferKind;
    CyphalPortID lastPortID;
    size_t lastExtent;
    size_t lastTimeout;

};

TEST_CASE("SubscriptionManager: Subscribe and Unsubscribe Single Port")
{
    // Create a SubscriptionManager
    SubscriptionManager manager;

    // Create dummy adapters
    DummyAdapter adapter1(42);
    DummyAdapter adapter2(43);
    std::tuple<DummyAdapter &, DummyAdapter &> adapters(adapter1, adapter2);

    // Get a port_id from cyphal_subscriptions
    const CyphalPortID port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_;

    // Subscribe to the port
    manager.subscribe(port_id, adapters);

    // Verify that the subscription was added
    const auto &subscriptions = manager.getSubscriptions();
    REQUIRE(subscriptions.size() == 1);
    REQUIRE(subscriptions[0]->port_id == port_id);

    REQUIRE(adapter1.cyphalRxSubscribeCallCount == 1);
    REQUIRE(adapter1.lastPortID == port_id);

    REQUIRE(adapter2.cyphalRxSubscribeCallCount == 1);
    REQUIRE(adapter2.lastPortID == port_id);

    // Unsubscribe from the port
    manager.unsubscribe(port_id, adapters);

    // Verify that the subscription was removed
    REQUIRE(subscriptions.size() == 0);

    REQUIRE(adapter1.cyphalRxUnsubscribeCallCount == 1);
    REQUIRE(adapter1.lastPortID == port_id);

    REQUIRE(adapter2.cyphalRxUnsubscribeCallCount == 1);
    REQUIRE(adapter2.lastPortID == port_id);
}

TEST_CASE("SubscriptionManager: Subscribe and Unsubscribe List of Ports")
{
    // Create a SubscriptionManager
    SubscriptionManager manager;

    // Create dummy adapters
    DummyAdapter adapter1(42);
    DummyAdapter adapter2(43);
    std::tuple<DummyAdapter &, DummyAdapter &> adapters(adapter1, adapter2);

    // Create a list of port IDs
    std::vector<CyphalPortID> port_ids = {
        uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
        uavcan_node_port_List_1_0_FIXED_PORT_ID_};

    // Subscribe to the list of ports
    manager.subscribe(port_ids, adapters);

    // Verify that the subscriptions were added
    const auto &subscriptions = manager.getSubscriptions();
    REQUIRE(subscriptions.size() == 2);
    REQUIRE(subscriptions[0]->port_id == port_ids[0]);
    REQUIRE(subscriptions[1]->port_id == port_ids[1]);

    REQUIRE(adapter1.cyphalRxSubscribeCallCount == 2);
    REQUIRE(adapter2.cyphalRxSubscribeCallCount == 2);

    // Unsubscribe from the list of ports
    manager.unsubscribe(port_ids, adapters);

    // Verify that the subscriptions were removed
    REQUIRE(subscriptions.size() == 0);

    REQUIRE(adapter1.cyphalRxUnsubscribeCallCount == 2);
    REQUIRE(adapter2.cyphalRxUnsubscribeCallCount == 2);
}

TEST_CASE("SubscriptionManager: Subscribe to non existent port")
{
    // Create a SubscriptionManager
    SubscriptionManager manager;

    // Create dummy adapters
    DummyAdapter adapter1(42);
    DummyAdapter adapter2(43);
    std::tuple<DummyAdapter &, DummyAdapter &> adapters(adapter1, adapter2);

    // Get a port_id from cyphal_subscriptions
    const CyphalPortID port_id = 65535;

    // Subscribe to the port
    manager.subscribe(port_id, adapters);

    // Verify that the subscription was added
    const auto &subscriptions = manager.getSubscriptions();
    REQUIRE(subscriptions.size() == 0);

    REQUIRE(adapter1.cyphalRxSubscribeCallCount == 0);
    REQUIRE(adapter1.lastPortID == 0);

    REQUIRE(adapter2.cyphalRxSubscribeCallCount == 0);
    REQUIRE(adapter2.lastPortID == 0);

    // Unsubscribe from the port
    manager.unsubscribe(port_id, adapters);

    // Verify that the subscription was removed
    REQUIRE(subscriptions.size() == 0);

    REQUIRE(adapter1.cyphalRxUnsubscribeCallCount == 0);
    REQUIRE(adapter1.lastPortID == 0);

    REQUIRE(adapter2.cyphalRxUnsubscribeCallCount == 0);
    REQUIRE(adapter2.lastPortID == 0);
}