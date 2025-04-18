#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <cyphal_subscriptions.hpp>
#include "cyphal.hpp"
#include "nunavut_assert.h"
#include "uavcan/node/port/List_1_0.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/node/GetInfo_1_0.h"

TEST_CASE("findMessageByPortIdCompileTime - Found")
{
    constexpr CyphalSubscription const *result = findMessageByPortIdCompileTime<uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_>();
    REQUIRE(result != nullptr);
    CHECK(result->port_id == uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
    CHECK(result->extent == uavcan_node_Heartbeat_1_0_EXTENT_BYTES_);
    CHECK(result->transfer_kind == CyphalTransferKindMessage);
}

TEST_CASE("findMessageByPortIdCompileTime - Not Found")
{
    constexpr CyphalSubscription const *result = findMessageByPortIdCompileTime<999>(); // Non-existent port ID
    REQUIRE(result == nullptr);
}

TEST_CASE("findMessageByPortIdCompileTime - Compile time assert")
{
    constexpr CyphalSubscription const *result = findMessageByPortIdCompileTime<uavcan_node_port_List_1_0_FIXED_PORT_ID_>();
    REQUIRE(result != nullptr);
    static_assert(result->port_id == uavcan_node_port_List_1_0_FIXED_PORT_ID_, "Port ID mismatch at compile time!");
    static_assert(result->extent == uavcan_node_port_List_1_0_EXTENT_BYTES_, "Extent Bytes mismatch at compile time!");
}

TEST_CASE("findMessageByPortIdRuntime - Found")
{
    CyphalSubscription const *result = findMessageByPortIdRuntime(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
    REQUIRE(result != nullptr);
    CHECK(result->port_id == uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
    CHECK(result->extent == uavcan_node_Heartbeat_1_0_EXTENT_BYTES_);
    CHECK(result->transfer_kind == CyphalTransferKindMessage);
}

TEST_CASE("findMessageByPortIdRuntime - Not Found")
{
    CyphalSubscription const *result = findMessageByPortIdRuntime(999); // Non-existent port ID
    REQUIRE(result == nullptr);
}

TEST_CASE("findMessageByPortIdRuntime - Different Port ID")
{
    CyphalSubscription const *result = findMessageByPortIdRuntime(uavcan_node_port_List_1_0_FIXED_PORT_ID_);
    REQUIRE(result != nullptr);
    CHECK(result->port_id == uavcan_node_port_List_1_0_FIXED_PORT_ID_);
    CHECK(result->extent == uavcan_node_port_List_1_0_EXTENT_BYTES_);
    CHECK(result->transfer_kind == CyphalTransferKindMessage);
}

TEST_CASE("findRequestByPortIdCompileTime - Found")
{
    constexpr CyphalSubscription const *result = findRequestByPortIdCompileTime<uavcan_node_GetInfo_1_0_FIXED_PORT_ID_>();
    REQUIRE(result != nullptr);
    CHECK(result->port_id == uavcan_node_GetInfo_1_0_FIXED_PORT_ID_);
    CHECK(result->extent == uavcan_node_GetInfo_Request_1_0_EXTENT_BYTES_);
    CHECK(result->transfer_kind == CyphalTransferKindRequest);
}

TEST_CASE("findResponseByPortIdCompileTime - Found")
{
    constexpr CyphalSubscription const *result = findResponseByPortIdCompileTime<uavcan_node_GetInfo_1_0_FIXED_PORT_ID_>();
    REQUIRE(result != nullptr);
    CHECK(result->port_id == uavcan_node_GetInfo_1_0_FIXED_PORT_ID_);
    CHECK(result->extent == uavcan_node_GetInfo_Response_1_0_EXTENT_BYTES_);
    CHECK(result->transfer_kind == CyphalTransferKindResponse);
}

TEST_CASE("findRequestByPortIdRuntime - Found")
{
    CyphalSubscription const *result = findRequestByPortIdRuntime(uavcan_node_GetInfo_1_0_FIXED_PORT_ID_);
    REQUIRE(result != nullptr);
    CHECK(result->port_id == uavcan_node_GetInfo_1_0_FIXED_PORT_ID_);
    CHECK(result->extent == uavcan_node_GetInfo_Request_1_0_EXTENT_BYTES_);
    CHECK(result->transfer_kind == CyphalTransferKindRequest);
}

TEST_CASE("findResponseByPortIdRuntime - Found")
{
    CyphalSubscription const *result = findResponseByPortIdRuntime(uavcan_node_GetInfo_1_0_FIXED_PORT_ID_);
    REQUIRE(result != nullptr);
    CHECK(result->port_id == uavcan_node_GetInfo_1_0_FIXED_PORT_ID_);
    CHECK(result->extent == uavcan_node_GetInfo_Response_1_0_EXTENT_BYTES_);
    CHECK(result->transfer_kind == CyphalTransferKindResponse);
}

TEST_CASE("findRequestByPortIdCompileTime - Not Found")
{
    constexpr CyphalSubscription const *result = findRequestByPortIdCompileTime<999>(); // Non-existent port ID
    REQUIRE(result == nullptr);
}

TEST_CASE("findResponseByPortIdCompileTime - Not Found")
{
    constexpr CyphalSubscription const *result = findResponseByPortIdCompileTime<999>(); // Non-existent port ID
    REQUIRE(result == nullptr);
}

TEST_CASE("findRequestByPortIdRuntime - Not Found")
{
    CyphalSubscription const *result = findRequestByPortIdRuntime(999); // Non-existent port ID
    REQUIRE(result == nullptr);
}

TEST_CASE("findResponseByPortIdRuntime - Not Found")
{
    CyphalSubscription const *result = findResponseByPortIdRuntime(999); // Non-existent port ID
    REQUIRE(result == nullptr);
}