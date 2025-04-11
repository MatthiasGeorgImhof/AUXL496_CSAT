#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <cyphal_subscriptions.hpp>
#include "cyphal.hpp"
#include "nunavut_assert.h"
#include "uavcan/node/port/List_1_0.h"
#include "uavcan/node/Heartbeat_1_0.h"


TEST_CASE("findByPortIdCompileTime - Found") {
    constexpr CyphalSubscription const* result = findByPortIdCompileTime(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
    REQUIRE(result != nullptr);
    REQUIRE(result->port_id == uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
    REQUIRE(result->extent == uavcan_node_Heartbeat_1_0_EXTENT_BYTES_);
    REQUIRE(result->transfer_kind == CyphalTransferKindMessage);
}

TEST_CASE("findByPortIdCompileTime - Not Found") {
    constexpr CyphalSubscription const* result = findByPortIdCompileTime(999); // Non-existent port ID
    REQUIRE(result == nullptr);
}

TEST_CASE("findByPortIdCompileTime - Compile time assert") {
    constexpr CyphalSubscription const* result = findByPortIdCompileTime(uavcan_node_port_List_1_0_FIXED_PORT_ID_);
    REQUIRE(result != nullptr);
    // Use static_assert to ensure result is valid at compile time
    static_assert(result->port_id == uavcan_node_port_List_1_0_FIXED_PORT_ID_, "Port ID mismatch at compile time!");
    static_assert(result->extent == uavcan_node_port_List_1_0_EXTENT_BYTES_, "Extent Bytes mismatch at compile time!");

}

TEST_CASE("findByPortIdRuntime - Found") {
    CyphalSubscription const* result = findByPortIdRuntime(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
    REQUIRE(result != nullptr);
    REQUIRE(result->port_id == uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
    REQUIRE(result->extent == uavcan_node_Heartbeat_1_0_EXTENT_BYTES_);
    REQUIRE(result->transfer_kind == CyphalTransferKindMessage);
}

TEST_CASE("findByPortIdRuntime - Not Found") {
    CyphalSubscription const* result = findByPortIdRuntime(999); // Non-existent port ID
    REQUIRE(result == nullptr);
}

TEST_CASE("findByPortIdRuntime - Different Port ID") {
    CyphalSubscription const* result = findByPortIdRuntime(uavcan_node_port_List_1_0_FIXED_PORT_ID_);
    REQUIRE(result != nullptr);
    REQUIRE(result->port_id == uavcan_node_port_List_1_0_FIXED_PORT_ID_);
    REQUIRE(result->extent == uavcan_node_port_List_1_0_EXTENT_BYTES_);
    REQUIRE(result->transfer_kind == CyphalTransferKindMessage);
}