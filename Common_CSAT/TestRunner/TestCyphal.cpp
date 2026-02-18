#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "BoxSet.hpp"
#include "cyphal.hpp"
#include "canard_adapter.hpp"
#include "serard_adapter.hpp"
#include "udpard_adapter.hpp"
#include "loopard_adapter.hpp"
#include <iostream>
#include <cstdint>
#include <vector>
#include <cstring>
#include <algorithm>

#ifdef __x86_64__
#include "mock_hal.h" // Include the mock HAL header
#endif

void *canardMemoryAllocate(CanardInstance * /*ins*/, size_t amount) { return static_cast<void *>(malloc(amount)); };
void canardMemoryFree(CanardInstance * /*ins*/, void *pointer) { free(pointer); };

CanTxQueueDrainer::CanTxQueueDrainer(CanardAdapter* /*adapter*/, CAN_HandleTypeDef* /*hcan*/) {}

void CanTxQueueDrainer::drain() {}

void CanTxQueueDrainer::irq_safe_drain() {}

CanTxQueueDrainer tx_drainer{nullptr, nullptr};

TEST_CASE("Canard Adapter")
{
    CanardAdapter adapter;
    adapter.ins = canardInit(canardMemoryAllocate, canardMemoryFree);
    adapter.ins.node_id = 11;
    adapter.que = canardTxInit(16, CANARD_MTU_CAN_CLASSIC);
    Cyphal<CanardAdapter> cyphal(&adapter);

    CyphalTransferMetadata metadata;
    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id = 0;

    const char payload[] = "hello";
    size_t payload_size = sizeof(payload);

    SUBCASE("cyphalTxPush")
    {
        CHECK(cyphal.cyphalTxPush(0, &metadata, payload_size, payload) == 1);
    }

    SUBCASE("getNodeID and setNodeID")
    {
        CHECK(cyphal.getNodeID() == 11);
        cyphal.setNodeID(22);
        CHECK(cyphal.getNodeID() == 22);
    }

    SUBCASE("cyphalTxForward")
    {
        CanardAdapter adapter2;
        adapter2.ins = canardInit(canardMemoryAllocate, canardMemoryFree);
        adapter2.ins.node_id = 11;
        adapter2.que = canardTxInit(16, CANARD_MTU_CAN_CLASSIC);

        Cyphal<CanardAdapter> cyphal2(&adapter2);
        const char payload[] = "hello";
        size_t payload_size = sizeof(payload);

        auto res = cyphal2.cyphalTxForward(0, &metadata, payload_size, payload, 33);
        CHECK(res == 1);
    }

    REQUIRE(adapter.subscriptions.size() == 0);
    SUBCASE("cyphalRxSubscribe and Unsubscribe")
    {

        CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 42, 100, 2000000) == 1);
        CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 42, 100, 2000000) == 0);
        CHECK(cyphal.cyphalRxUnsubscribe(CyphalTransferKindMessage, 42) == 1);
        CHECK(cyphal.cyphalRxUnsubscribe(CyphalTransferKindMessage, 42) == 0);
        CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 43, 100, 2000000) == 1);
        CHECK(cyphal.cyphalRxUnsubscribe(CyphalTransferKindMessage, 43) == 1);
        CHECK(cyphal.cyphalRxUnsubscribe(CyphalTransferKindMessage, 43) == 0);
    }
    REQUIRE(adapter.subscriptions.size() == 0);

    SUBCASE("cyphalRxSubscribe returns negative on full boxset")
    {
        adapter.subscriptions.clear();
        for (CyphalPortID i = 0; i < CanardAdapter::SUBSCRIPTIONS; ++i)
        {
            CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, i, 100, 2000000) == 1);
        }
        REQUIRE(adapter.subscriptions.is_full());
        CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, CanardAdapter::SUBSCRIPTIONS + 1, 100, 2000000) == -2);
    }
}

TEST_CASE("Canard Send Receive")
{
    CanardAdapter adapter;
    adapter.ins = canardInit(canardMemoryAllocate, canardMemoryFree);
    adapter.ins.node_id = 11;
    adapter.que = canardTxInit(16, CANARD_MTU_CAN_CLASSIC);
    Cyphal<CanardAdapter> cyphal(&adapter);

    CyphalTransferMetadata metadata;
    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id = 0;

    const char payload1[] = "hello";
    size_t payload_size1 = sizeof(payload1);
    CHECK(cyphal.cyphalTxPush(0, &metadata, payload_size1, payload1) == 1);

    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id++;

    const char payload2[] = "ehllo ehllo ehllo";
    size_t payload_size2 = sizeof(payload2);
    CHECK(cyphal.cyphalTxPush(0, &metadata, payload_size2, payload2) == 3);

    CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 123, 100, 2000000) == 1);

    const CanardTxQueueItem *const const_ptr = canardTxPeek(&adapter.que);
    CHECK(const_ptr != nullptr);
    CanardTxQueueItem *ptr = canardTxPop(&adapter.que, const_ptr);
    CHECK(ptr != nullptr);

    CyphalTransfer transfer;

    CHECK(cyphal.cyphalRxReceive(ptr->frame.extended_can_id, &ptr->frame.payload_size, static_cast<const uint8_t *>(ptr->frame.payload), &transfer) == 1);
    CHECK(strncmp(payload1, static_cast<const char *>(transfer.payload), 5) == 0);

    const CanardTxQueueItem *const const_ptr2 = canardTxPeek(&adapter.que);
    CHECK(const_ptr2 != nullptr);
    ptr = canardTxPop(&adapter.que, const_ptr2);
    CHECK(ptr != nullptr);
    CHECK(cyphal.cyphalRxReceive(ptr->frame.extended_can_id, &ptr->frame.payload_size, static_cast<const uint8_t *>(ptr->frame.payload), &transfer) == 0);

    const CanardTxQueueItem *const const_ptr3 = canardTxPeek(&adapter.que);
    CHECK(const_ptr3 != nullptr);
    ptr = canardTxPop(&adapter.que, const_ptr3);
    CHECK(ptr != nullptr);
    CHECK(cyphal.cyphalRxReceive(ptr->frame.extended_can_id, &ptr->frame.payload_size, static_cast<const uint8_t *>(ptr->frame.payload), &transfer) == 0);

    const CanardTxQueueItem *const const_ptr4 = canardTxPeek(&adapter.que);
    CHECK(const_ptr4 != nullptr);
    ptr = canardTxPop(&adapter.que, const_ptr4);
    CHECK(ptr != nullptr);
    CHECK(cyphal.cyphalRxReceive(ptr->frame.extended_can_id, &ptr->frame.payload_size, static_cast<const uint8_t *>(ptr->frame.payload), &transfer) == 1);
    CHECK(strncmp(payload2, static_cast<const char *>(transfer.payload), 18) == 0);
}

TEST_CASE("Canard Send Receive Large")
{
    CAN_HandleTypeDef hcan;
    constexpr CyphalPortID port_id = 123;
    constexpr CyphalNodeID node_id = 11;

    CanardAdapter adapter;
    adapter.ins = canardInit(canardMemoryAllocate, canardMemoryFree);
    adapter.ins.node_id = node_id;
    adapter.que = canardTxInit(64, CANARD_MTU_CAN_CLASSIC);
    Cyphal<CanardAdapter> cyphal(&adapter);

    CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, port_id, 512, 2000000) == 1);

    CyphalTransferMetadata metadata;
    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = port_id;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id = 0;

    char payload[256];
    for (size_t i = 0; i < sizeof(payload); ++i)
    {
        payload[i] = static_cast<char>(i);
    }
    size_t payload_size = sizeof(payload);
    CHECK(cyphal.cyphalTxPush(0, &metadata, payload_size, payload) == 37);

    for (const CanardTxQueueItem *ti = NULL; (ti = canardTxPeek(&adapter.que)) != NULL;)
    {
        CAN_TxHeaderTypeDef header;
        header.ExtId = ti->frame.extended_can_id;
        header.DLC = static_cast<uint8_t>(ti->frame.payload_size);
        header.RTR = CAN_RTR_DATA;
        header.IDE = CAN_ID_EXT;
        uint32_t mailbox;
        CHECK(HAL_CAN_AddTxMessage(&hcan, &header, static_cast<uint8_t *>(const_cast<void *>(ti->frame.payload)), &mailbox) == HAL_OK);
        adapter.ins.memory_free(&adapter.ins, canardTxPop(&adapter.que, ti));
    }

    move_can_tx_to_rx();

    CyphalTransfer transfer;
    for (int i = 0; i < 37; ++i)
    {
        CAN_RxHeaderTypeDef header;
        uint8_t data[8];
        CHECK(HAL_CAN_GetRxMessage(&hcan, 0, &header, data) == HAL_OK);
        size_t data_size = header.DLC;
        int32_t result = cyphal.cyphalRxReceive(header.ExtId, &data_size, data, &transfer);
        if (i < 36)
            CHECK(result == 0);
        else
            CHECK(result == 1);
    }
    CHECK(transfer.payload_size == 256);
    CHECK(transfer.metadata.port_id == port_id);
    CHECK(transfer.metadata.remote_node_id == node_id);
    CHECK(transfer.metadata.transfer_kind == CyphalTransferKindMessage);
    CHECK(transfer.metadata.transfer_id == 0);
    CHECK(transfer.metadata.priority == CyphalPriorityNominal);
    CHECK(transfer.payload != nullptr);
    CHECK(memcmp(transfer.payload, payload, sizeof(payload)) == 0);
    adapter.ins.memory_free(&adapter.ins, transfer.payload);
}

TEST_CASE("Canard Send Forward Receive")
{
    constexpr CyphalNodeID my_id = 11;
    constexpr CyphalNodeID forward_id = 22;

    CanardAdapter adapter;
    adapter.ins = canardInit(canardMemoryAllocate, canardMemoryFree);
    adapter.ins.node_id = my_id;
    adapter.que = canardTxInit(16, CANARD_MTU_CAN_CLASSIC);
    Cyphal<CanardAdapter> cyphal(&adapter);
    CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 123, 100, 2000000) == 1);

    CyphalTransfer transfer1, transfer2;

    CyphalTransferMetadata metadata;
    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = forward_id;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id = 0;

    const char payload1[] = "hello";
    size_t payload_size1 = sizeof(payload1);
    auto res = cyphal.cyphalTxForward(0, &metadata, payload_size1, payload1, 0);
    CHECK(res == 1);
    const CanardTxQueueItem *const const_ptr1 = canardTxPeek(&adapter.que);
    CanardTxQueueItem *ptr1 = canardTxPop(&adapter.que, const_ptr1);
    CHECK(cyphal.cyphalRxReceive(ptr1->frame.extended_can_id, &ptr1->frame.payload_size, static_cast<const uint8_t *>(ptr1->frame.payload), &transfer1) == 1);
    CHECK(strncmp(payload1, static_cast<const char *>(transfer1.payload), 5) == 0);
    CHECK(transfer1.metadata.remote_node_id == forward_id);

    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = forward_id;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id++;

    const char payload2[] = "ehllo";
    size_t payload_size2 = sizeof(payload2);
    CHECK(cyphal.cyphalTxPush(0, &metadata, payload_size2, payload2) == 1);
    const CanardTxQueueItem *const const_ptr2 = canardTxPeek(&adapter.que);
    CanardTxQueueItem *ptr2 = canardTxPop(&adapter.que, const_ptr2);
    CHECK(cyphal.cyphalRxReceive(ptr2->frame.extended_can_id, &ptr2->frame.payload_size, static_cast<const uint8_t *>(ptr2->frame.payload), &transfer2) == 1);
    CHECK(strncmp(payload2, static_cast<const char *>(transfer2.payload), 5) == 0);
    CHECK(transfer2.metadata.remote_node_id == my_id);
}

void *serardMemoryAllocate(void *const /*user_reference*/, const size_t size) { return static_cast<void *>(malloc(size)); };
void serardMemoryDeallocate(void *const /*user_reference*/, const size_t /*size*/, void *const pointer) { free(pointer); };

TEST_CASE("Serard Basics")
{
    CHECK(cyphalNodeIdToSerard(CYPHAL_NODE_ID_UNSET) == SERARD_NODE_ID_UNSET);
    CHECK(cyphalNodeIdToSerard(123) == 123);
    CHECK(serardNodeIdToCyphal(SERARD_NODE_ID_UNSET) == CYPHAL_NODE_ID_UNSET);
    CHECK(serardNodeIdToCyphal(123) == 123);
    CHECK(serardNodeIdToCyphal(0x1122) == 0x0022);

    SUBCASE("serard cyphal serard")
    {
        SerardTransferMetadata metadata{
            SerardPriorityNominal,
            SerardTransferKindMessage,
            123,
            SERARD_NODE_ID_UNSET,
            SERARD_NODE_ID_UNSET,
            SERARD_NODE_ID_UNSET,
            11};
        SerardTransferMetadata translated = cyphalMetadataToSerard(serardMetadataToCyphal(metadata));
        CHECK((metadata.priority == translated.priority && metadata.transfer_kind == translated.transfer_kind && metadata.port_id == translated.port_id && metadata.remote_node_id == translated.remote_node_id && metadata.transfer_id == translated.transfer_id));
    }
    SUBCASE("cyphal serard cyphal")
    {
        CyphalTransferMetadata metadata{
            CyphalPriorityNominal,
            CyphalTransferKindMessage,
            123,
            CYPHAL_NODE_ID_UNSET,
            CYPHAL_NODE_ID_UNSET,
            CYPHAL_NODE_ID_UNSET,
            11};
        CyphalTransferMetadata translated = serardMetadataToCyphal(cyphalMetadataToSerard(metadata));
        CHECK((metadata.priority == translated.priority && metadata.transfer_kind == translated.transfer_kind && metadata.port_id == translated.port_id && metadata.remote_node_id == translated.remote_node_id && metadata.transfer_id == translated.transfer_id));
    }
}

TEST_CASE("Serard Adapter")
{
    SerardAdapter adapter;
    struct SerardMemoryResource serard_memory_resource = {&adapter.ins, serardMemoryDeallocate, serardMemoryAllocate};
    adapter.ins = serardInit(serard_memory_resource, serard_memory_resource);
    adapter.ins.node_id = 11; // Set initial node ID
    adapter.emitter = [](void *, uint8_t, const uint8_t *) -> bool
    { return true; };
    Cyphal<SerardAdapter> cyphal(&adapter);

    CyphalTransferMetadata metadata;
    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id = 0;

    const char payload[] = "hello";
    size_t payload_size = sizeof(payload);

    SUBCASE("cyphalTxPush")
    {
        CHECK(cyphal.cyphalTxPush(0, &metadata, payload_size, payload) == 1);
    }

    SUBCASE("getNodeID and setNodeID")
    {
        CHECK(cyphal.getNodeID() == 11);
        cyphal.setNodeID(22);
        CHECK(cyphal.getNodeID() == 22);
    }

    SUBCASE("cyphalTxForward")
    {
        SerardAdapter adapter2;
        struct SerardMemoryResource serard_memory_resource2 = {&adapter2.ins, serardMemoryDeallocate, serardMemoryAllocate};
        adapter2.ins = serardInit(serard_memory_resource2, serard_memory_resource2);
        adapter2.emitter = [](void *, uint8_t, const uint8_t *) -> bool
        { return true; };

        Cyphal<SerardAdapter> cyphal2(&adapter2);

        const char payload[] = "hello";
        size_t payload_size = sizeof(payload);
        CHECK(cyphal2.cyphalTxForward(0, &metadata, payload_size, payload, 33) == 1); // remote node 33
    }

    REQUIRE(adapter.subscriptions.size() == 0);
    SUBCASE("cyphalRxSubscribe and Unsubscribe")
    {
        CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 42, 100, 2000000) == 1);
        CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 42, 100, 2000000) == 0);
        CHECK(cyphal.cyphalRxUnsubscribe(CyphalTransferKindMessage, 42) == 1);
        CHECK(cyphal.cyphalRxUnsubscribe(CyphalTransferKindMessage, 42) == 0);
        CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 43, 100, 2000000) == 1);
        CHECK(cyphal.cyphalRxUnsubscribe(CyphalTransferKindMessage, 43) == 1);
    }
    REQUIRE(adapter.subscriptions.size() == 0);

    SUBCASE("cyphalRxSubscribe returns negative on full boxset")
    {
        adapter.subscriptions.clear();
        for (CyphalPortID i = 0; i < SerardAdapter::SUBSCRIPTIONS; ++i)
        {
            CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, i, 100, 2000000) == 1);
        }
        REQUIRE(adapter.subscriptions.is_full());
        CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, SerardAdapter::SUBSCRIPTIONS + 1, 100, 2000000) == -2);
    }
}

static std::vector<uint8_t> rxtx_buffer;
bool emit(void * /*user_reference*/, uint8_t size, const uint8_t *data)
{
    for (size_t i = 0; i < size; ++i)
        rxtx_buffer.push_back(data[i]);
    return true;
}

TEST_CASE("Cyphal<serard_adapter> Send Receive")
{
    rxtx_buffer.clear();

    SerardAdapter adapter;
    struct SerardMemoryResource serard_memory_resource = {&adapter.ins, serardMemoryDeallocate, serardMemoryAllocate};
    adapter.ins = serardInit(serard_memory_resource, serard_memory_resource);
    adapter.ins.node_id = 11;
    adapter.user_reference = &adapter.ins;
    adapter.ins.user_reference = &adapter.ins;
    adapter.reass = serardReassemblerInit();
    adapter.emitter = emit;
    Cyphal<SerardAdapter> cyphal(&adapter);

    auto res1 = cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 123, 128, 0);
    CHECK(res1 == 1);

    CyphalTransferMetadata metadata;
    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id = 0;
    const char payload1[] = "hello\0";
    CHECK(cyphal.cyphalTxPush(0, &metadata, strlen(payload1), payload1) == 1);
    CHECK(rxtx_buffer.size() == 36);

    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id++;
    const char payload2[] = "ehllo\0";
    CHECK(cyphal.cyphalTxPush(0, &metadata, strlen(payload2), payload2) == 1);
    CHECK(rxtx_buffer.size() == 72);

    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id++;
    const char payload3[] = "bonjour\0";
    CHECK(cyphal.cyphalTxPush(0, &metadata, strlen(payload3), payload3) == 1);
    CHECK(rxtx_buffer.size() == 110);

    CyphalTransfer transfer;
    size_t shift = 0;
    size_t in_out = rxtx_buffer.size();

    shift = rxtx_buffer.size() - in_out;
    CHECK(cyphal.cyphalRxReceive(&in_out, rxtx_buffer.data() + shift, &transfer) == 1);
    CHECK(strncmp(payload1, static_cast<const char *>(transfer.payload), 5) == 0);
    CHECK(in_out == 74);

    shift = rxtx_buffer.size() - in_out;
    CHECK(cyphal.cyphalRxReceive(&in_out, rxtx_buffer.data() + shift, &transfer) == 1);
    CHECK(strncmp(payload2, static_cast<const char *>(transfer.payload), 5) == 0);
    CHECK(in_out == 38);

    shift = rxtx_buffer.size() - in_out;
    CHECK(cyphal.cyphalRxReceive(&in_out, rxtx_buffer.data() + shift, &transfer) == 1);
    CHECK(strncmp(payload3, static_cast<const char *>(transfer.payload), 7) == 0);
    CHECK(in_out == 0);
}

TEST_CASE("Cyphal<serard_adapter> Send Forward Receive")
{
    rxtx_buffer.clear();
    CyphalNodeID my_id = 11;
    CyphalNodeID forward_id = 22;

    SerardAdapter adapter;
    struct SerardMemoryResource serard_memory_resource = {&adapter.ins, serardMemoryDeallocate, serardMemoryAllocate};
    adapter.ins = serardInit(serard_memory_resource, serard_memory_resource);
    adapter.ins.node_id = my_id;
    adapter.user_reference = &adapter.ins;
    adapter.ins.user_reference = &adapter.ins;
    adapter.reass = serardReassemblerInit();
    adapter.emitter = emit;
    Cyphal<SerardAdapter> cyphal(&adapter);

    auto res1 = cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 123, 128, 0);
    CHECK(res1 == 1);

    CyphalTransferMetadata metadata;
    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id = 0;
    const char payload1[] = "hello\0";
    CHECK(cyphal.cyphalTxPush(0, &metadata, strlen(payload1), payload1) == 1);
    CHECK(rxtx_buffer.size() == 36);

    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = forward_id;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id++;
    const char payload2[] = "ehllo\0";
    CHECK(cyphal.cyphalTxForward(0, &metadata, strlen(payload2), payload2, forward_id) == 1);
    CHECK(rxtx_buffer.size() == 72);

    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id++;
    const char payload3[] = "bonjour\0";
    CHECK(cyphal.cyphalTxPush(0, &metadata, strlen(payload3), payload3) == 1);
    CHECK(rxtx_buffer.size() == 110);

    CyphalTransfer transfer;
    size_t shift = 0;
    size_t in_out = rxtx_buffer.size();

    shift = rxtx_buffer.size() - in_out;
    CHECK(cyphal.cyphalRxReceive(&in_out, rxtx_buffer.data() + shift, &transfer) == 1);
    CHECK(strncmp(payload1, static_cast<const char *>(transfer.payload), 5) == 0);
    CHECK(in_out == 74);
    CHECK(transfer.metadata.remote_node_id == my_id);

    shift = rxtx_buffer.size() - in_out;
    CHECK(cyphal.cyphalRxReceive(&in_out, rxtx_buffer.data() + shift, &transfer) == 1);
    CHECK(strncmp(payload2, static_cast<const char *>(transfer.payload), 5) == 0);
    CHECK(in_out == 38);
    CHECK(transfer.metadata.remote_node_id == forward_id);

    shift = rxtx_buffer.size() - in_out;
    CHECK(cyphal.cyphalRxReceive(&in_out, rxtx_buffer.data() + shift, &transfer) == 1);
    CHECK(strncmp(payload3, static_cast<const char *>(transfer.payload), 7) == 0);
    CHECK(in_out == 0);
    CHECK(transfer.metadata.remote_node_id == my_id);
}

void *udpardMemoryAllocate(void *const /*user_reference*/, const size_t size) { return static_cast<void *>(malloc(size)); };
void udpardMemoryDeallocate(void *const /*user_reference*/, const size_t /*size*/, void *const pointer) { free(pointer); };

TEST_CASE("Udpard Adapter")
{
    UdpardAdapter adapter;
    struct UdpardMemoryDeleter udpard_memory_deleter = {&adapter.ins, udpardMemoryDeallocate};
    struct UdpardMemoryResource udpard_memory_resource = {&adapter.ins, udpardMemoryDeallocate, udpardMemoryAllocate};
    UdpardNodeID local_node_id = 11;
    Cyphal<UdpardAdapter> cyphal(&adapter);
    udpardTxInit(&adapter.ins, &local_node_id, 100, udpard_memory_resource);
    adapter.memory_resources = {udpard_memory_resource, udpard_memory_resource, udpard_memory_deleter};

    UdpardNodeID initial_node_id = 11;
    udpardTxInit(&adapter.ins, &initial_node_id, 100, udpard_memory_resource);

    CyphalTransferMetadata metadata;
    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id = 0;

    const char payload[] = "hello";
    size_t payload_size = sizeof(payload);

    SUBCASE("cyphalTxPush")
    {
        CHECK(cyphal.cyphalTxPush(0, &metadata, payload_size, payload) == 1);
    }

    SUBCASE("getNodeID and setNodeID")
    {
        UdpardNodeID new_node_id = 22;
        const UdpardNodeID *current_node_id = cyphal.getNodeID();
        CHECK(*current_node_id == 11);

        cyphal.setNodeID(&new_node_id);
        current_node_id = cyphal.getNodeID();
        CHECK(*current_node_id == 22);
    }

    SUBCASE("cyphalTxForward")
    {
        UdpardAdapter adapter2;
        struct UdpardMemoryDeleter udpard_memory_deleter2 = {&adapter2.ins, udpardMemoryDeallocate};
        struct UdpardMemoryResource udpard_memory_resource2 = {&adapter2.ins, udpardMemoryDeallocate, udpardMemoryAllocate};
        UdpardNodeID local_node_id2 = 11;

        Cyphal<UdpardAdapter> cyphal2(&adapter2);
        udpardTxInit(&adapter2.ins, &local_node_id2, 100, udpard_memory_resource2);
        adapter2.memory_resources = {udpard_memory_resource2, udpard_memory_resource2, udpard_memory_deleter2};
        UdpardNodeID initial_node_id2 = 11;
        udpardTxInit(&adapter2.ins, &initial_node_id2, 100, udpard_memory_resource2);

        const char payload[] = "hello";
        size_t payload_size = sizeof(payload);
        CHECK(cyphal2.cyphalTxForward(0, &metadata, payload_size, payload, 33) == 1);
    }

    REQUIRE(adapter.subscriptions.size() == 0);
    SUBCASE("cyphalRxSubscribe and Unsubscribe")
    {
        CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 42, 100, 2000000) == 1);
        CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 42, 100, 2000000) == 1);
        CHECK(cyphal.cyphalRxUnsubscribe(CyphalTransferKindMessage, 42) == 1);
        CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 43, 100, 2000000) == 1);
        CHECK(cyphal.cyphalRxUnsubscribe(CyphalTransferKindMessage, 43) == 1);
    }
    REQUIRE(adapter.subscriptions.size() == 0);

    SUBCASE("cyphalRxSubscribe returns negative on full boxset")
    {
        adapter.subscriptions.clear();
        for (CyphalPortID i = 0; i < UdpardAdapter::SUBSCRIPTIONS; ++i)
        {
            CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, i, 100, 2000000) == 1);
        }
        CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, UdpardAdapter::SUBSCRIPTIONS + 1, 100, 2000000) == -4);
    }
}

TEST_CASE("Udpard Adapter Send and Receive")
{
    UdpardAdapter adapter;
    struct UdpardMemoryDeleter udpard_memory_deleter = {&adapter.ins, udpardMemoryDeallocate};
    struct UdpardMemoryResource udpard_memory_resource = {&adapter.ins, udpardMemoryDeallocate, udpardMemoryAllocate};
    UdpardNodeID local_node_id = 11;
    Cyphal<UdpardAdapter> cyphal(&adapter);
    udpardTxInit(&adapter.ins, &local_node_id, 100, udpard_memory_resource);
    adapter.memory_resources = {udpard_memory_resource, udpard_memory_resource, udpard_memory_deleter};
    UdpardNodeID initial_node_id = 11;
    udpardTxInit(&adapter.ins, &initial_node_id, 100, udpard_memory_resource);

    CyphalTransferMetadata metadata;
    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id = 13;

    const char payload[] = "hello";
    size_t payload_size = sizeof(payload);
    CHECK(cyphal.cyphalTxPush(0, &metadata, payload_size, payload) == 1);

    CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 123, 100, 2000000) == 1);

    const UdpardTxItem *const_ptr = udpardTxPeek(&adapter.ins);
    CHECK(const_ptr != nullptr);
    UdpardTxItem *ptr = udpardTxPop(&adapter.ins, const_ptr);
    CHECK(ptr != nullptr);

    CyphalTransfer transfer;
    cyphal.cyphalRxReceive(&ptr->datagram_payload.size, static_cast<uint8_t *>(ptr->datagram_payload.data), &transfer);
    CHECK(transfer.payload_size != 0);
    CHECK(strncmp(payload, static_cast<const char *>(transfer.payload), 5) == 0);
}

TEST_CASE("Udpard Adapter Forward Send and Receive")
{
    constexpr CyphalNodeID my_id = 11;
    constexpr CyphalNodeID forward_id = 22;

    UdpardAdapter adapter;
    struct UdpardMemoryDeleter udpard_memory_deleter = {&adapter.ins, udpardMemoryDeallocate};
    struct UdpardMemoryResource udpard_memory_resource = {&adapter.ins, udpardMemoryDeallocate, udpardMemoryAllocate};
    UdpardNodeID local_node_id = my_id;
    Cyphal<UdpardAdapter> cyphal(&adapter);
    udpardTxInit(&adapter.ins, &local_node_id, 100, udpard_memory_resource);
    adapter.memory_resources = {udpard_memory_resource, udpard_memory_resource, udpard_memory_deleter};

    CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 123, 100, 2000000) == 1);

    CyphalTransferMetadata metadata;
    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id = 13;

    const char payload1[] = "hello";
    size_t payload_size1 = sizeof(payload1);
    CHECK(cyphal.cyphalTxForward(0, &metadata, payload_size1, payload1, forward_id) == 1);

    const UdpardTxItem *const_ptr1 = udpardTxPeek(&adapter.ins);
    CHECK(const_ptr1 != nullptr);
    UdpardTxItem *ptr1 = udpardTxPop(&adapter.ins, const_ptr1);
    CHECK(ptr1 != nullptr);

    CyphalTransfer transfer1;
    cyphal.cyphalRxReceive(&ptr1->datagram_payload.size, static_cast<uint8_t *>(ptr1->datagram_payload.data), &transfer1);
    CHECK(transfer1.payload_size != 0);
    CHECK(strncmp(payload1, static_cast<const char *>(transfer1.payload), 5) == 0);
    CHECK(transfer1.metadata.remote_node_id == forward_id);

    const char payload2[] = "ehllo";
    size_t payload_size2 = sizeof(payload2);
    CHECK(cyphal.cyphalTxPush(0, &metadata, payload_size2, payload2) == 1);

    const UdpardTxItem *const_ptr2 = udpardTxPeek(&adapter.ins);
    CHECK(const_ptr2 != nullptr);
    UdpardTxItem *ptr2 = udpardTxPop(&adapter.ins, const_ptr2);
    CHECK(ptr2 != nullptr);

    CyphalTransfer transfer2;
    cyphal.cyphalRxReceive(&ptr2->datagram_payload.size, static_cast<uint8_t *>(ptr2->datagram_payload.data), &transfer2);
    CHECK(transfer2.payload_size != 0);
    CHECK(strncmp(payload2, static_cast<const char *>(transfer2.payload), 5) == 0);
    CHECK(transfer2.metadata.remote_node_id == my_id);
}

void *loopardMemoryAllocate(size_t amount) { return static_cast<void *>(malloc(amount)); };
void loopardMemoryFree(void *pointer) { free(pointer); };

TEST_CASE("Loopard Adapter")
{
    LoopardAdapter adapter;
    adapter.memory_allocate = loopardMemoryAllocate;
    adapter.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> cyphal(&adapter);

    CyphalTransferMetadata metadata;
    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id = 0;

    const char payload[] = "hello";
    size_t payload_size = sizeof(payload);

    SUBCASE("cyphalTxPush: Successful push")
    {
        CHECK(cyphal.cyphalTxPush(0, &metadata, payload_size, payload) == 1);
        REQUIRE(adapter.buffer.size() == 1);
        const auto &transfer = adapter.buffer.peek();
        CHECK(transfer.metadata.port_id == metadata.port_id);
        CHECK(transfer.metadata.priority == metadata.priority);
        CHECK(transfer.payload_size == payload_size);
        CHECK(std::memcmp(transfer.payload, payload, payload_size) == 0);
    }
    SUBCASE("cyphalTxPush: returns 0 when buffer full")
    {
        // fill the buffer
        for (size_t i = 0; i < LoopardAdapter::BUFFER; ++i)
        {
            CHECK(cyphal.cyphalTxPush(0, &metadata, payload_size, payload) == 1);
        }
        CHECK(cyphal.cyphalTxPush(0, &metadata, payload_size, payload) == 0);
    }

    SUBCASE("cyphalRxSubscribe and Unsubscribe")
    {

        CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 42, 100, 2000000) == 1);
        CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 42, 100, 2000000) == 1);
        CHECK(cyphal.cyphalRxUnsubscribe(CyphalTransferKindMessage, 42) == 1);
        CHECK(cyphal.cyphalRxUnsubscribe(CyphalTransferKindMessage, 42) == 1);
        CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 43, 100, 2000000) == 1);
        CHECK(cyphal.cyphalRxUnsubscribe(CyphalTransferKindMessage, 43) == 1);
    }

    REQUIRE(adapter.subscriptions.size() == 0);
    SUBCASE("cyphalRxSubscribe returns negative on full boxset")
    {
        adapter.subscriptions.clear();
        for (CyphalPortID i = 0; i < LoopardAdapter::SUBSCRIPTIONS; ++i)
        {
            CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, i, 100, 2000000) == 1);
        }
        REQUIRE(adapter.subscriptions.is_full());
        CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, LoopardAdapter::SUBSCRIPTIONS + 1, 100, 2000000) == 1);
    }
}

TEST_CASE("Loopard Send Receive")
{
    LoopardAdapter adapter;
    adapter.memory_allocate = loopardMemoryAllocate;
    adapter.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> cyphal(&adapter);

    CyphalTransferMetadata metadata;
    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id = 0;

    const char payload1[] = "hello";
    size_t payload_size1 = sizeof(payload1);

    CHECK(cyphal.cyphalTxPush(0, &metadata, payload_size1, payload1) == 1);
    REQUIRE(adapter.buffer.size() == 1);

    const char payload2[] = "ehllo ";
    size_t payload_size2 = sizeof(payload2);
    metadata.transfer_id++;
    CHECK(cyphal.cyphalTxPush(0, &metadata, payload_size2, payload2) == 1);

    CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 123, 100, 2000000) == 1);

    CyphalTransfer transfer = {};
    size_t inout_payload_size = 0;
    CHECK(cyphal.cyphalRxReceive(nullptr, &inout_payload_size, &transfer) == 2);
    CHECK(transfer.payload_size == payload_size1);
    CHECK(strncmp(payload1, static_cast<const char *>(transfer.payload), 5) == 0);
    CHECK(cyphal.cyphalRxReceive(nullptr, &inout_payload_size, &transfer) == 1);
    CHECK(transfer.payload_size == payload_size2);
    CHECK(strncmp(payload2, static_cast<const char *>(transfer.payload), 5) == 0);
    CHECK(cyphal.cyphalRxReceive(nullptr, &inout_payload_size, &transfer) == 0);
}

TEST_CASE("Loopard Forward Send Receive")
{
    CyphalNodeID my_id = 11;
    CyphalNodeID forward_id = 22;

    LoopardAdapter adapter;
    adapter.memory_allocate = loopardMemoryAllocate;
    adapter.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> cyphal(&adapter);
    adapter.node_id = my_id;

    CyphalTransferMetadata metadata;
    metadata.priority = CyphalPriorityNominal;
    metadata.transfer_kind = CyphalTransferKindMessage;
    metadata.port_id = 123;
    metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
    metadata.transfer_id = 0;

    const char payload1[] = "hello";
    size_t payload_size1 = sizeof(payload1);

    CHECK(cyphal.cyphalTxForward(0, &metadata, payload_size1, payload1, forward_id) == 1);
    REQUIRE(adapter.buffer.size() == 1);

    const char payload2[] = "ehllo ";
    size_t payload_size2 = sizeof(payload2);
    metadata.transfer_id++;
    CHECK(cyphal.cyphalTxPush(0, &metadata, payload_size2, payload2) == 1);

    CHECK(cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, 123, 100, 2000000) == 1);

    CyphalTransfer transfer = {};
    size_t inout_payload_size = 0;
    CHECK(cyphal.cyphalRxReceive(nullptr, &inout_payload_size, &transfer) == 2);
    CHECK(transfer.payload_size == payload_size1);
    CHECK(strncmp(payload1, static_cast<const char *>(transfer.payload), 5) == 0);
    CHECK(transfer.metadata.remote_node_id == forward_id);

    CHECK(cyphal.cyphalRxReceive(nullptr, &inout_payload_size, &transfer) == 1);
    CHECK(transfer.payload_size == payload_size2);
    CHECK(strncmp(payload2, static_cast<const char *>(transfer.payload), 5) == 0);
    CHECK(cyphal.cyphalRxReceive(nullptr, &inout_payload_size, &transfer) == 0);
    CHECK(transfer.metadata.remote_node_id == my_id);
}

template <class A1, class A2, class A3, class A4>
class TestClass
{
private:
    std::tuple<A1, A2, A3, A4> adapters_;

public:
    TestClass(std::tuple<A1, A2, A3, A4> adapters) : adapters_(adapters) {}

    int8_t txpush_unroll(const size_t frame_size, void const *frame)
    {
        CyphalTransferMetadata metadata;
        metadata.priority = CyphalPriorityNominal;
        metadata.transfer_kind = CyphalTransferKindMessage;
        metadata.port_id = 123;
        metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
        metadata.source_node_id = CYPHAL_NODE_ID_UNSET;
        metadata.destination_node_id = CYPHAL_NODE_ID_UNSET;
        metadata.transfer_id = 0;

        int32_t res0 = std::get<0>(adapters_).cyphalTxPush(0, &metadata, frame_size, frame);
        int32_t res1 = std::get<1>(adapters_).cyphalTxPush(0, &metadata, frame_size, frame);
        int32_t res2 = std::get<2>(adapters_).cyphalTxPush(0, &metadata, frame_size, frame);
        int32_t res3 = std::get<3>(adapters_).cyphalTxPush(0, &metadata, frame_size, frame);
        return (res0 > 0) && (res1 > 0) && (res2 > 0) && (res3 > 0);
    }

    int8_t txpush_loop(const size_t frame_size, void const *frame)
    {
        CyphalTransferMetadata metadata;
        metadata.priority = CyphalPriorityNominal;
        metadata.transfer_kind = CyphalTransferKindMessage;
        metadata.port_id = 123;
        metadata.remote_node_id = CYPHAL_NODE_ID_UNSET;
        metadata.transfer_id = 0;

        bool all_successful = true;
        std::apply([&](auto &...adapter)
                   { ([&]()
                      {
                int32_t res = adapter.cyphalTxPush(0, &metadata, frame_size, frame);
                all_successful = all_successful && (res > 0); }(), ...); }, adapters_);
        return all_successful;
    }
};

TEST_CASE("All Combined Unroll")
{
    rxtx_buffer.clear();

    LoopardAdapter loopard_adapter;
    loopard_adapter.memory_allocate = loopardMemoryAllocate;
    loopard_adapter.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard_adapter);

    UdpardAdapter udpard_adapter;
    struct UdpardMemoryDeleter udpard_memory_deleter = {&udpard_adapter.ins, udpardMemoryDeallocate};
    struct UdpardMemoryResource udpard_memory_resource = {&udpard_adapter.ins, udpardMemoryDeallocate, udpardMemoryAllocate};
    UdpardNodeID local_node_id = 11;
    Cyphal<UdpardAdapter> udpard_cyphal(&udpard_adapter);
    udpardTxInit(&udpard_adapter.ins, &local_node_id, 100, udpard_memory_resource);
    udpard_adapter.memory_resources = {udpard_memory_resource, udpard_memory_resource, udpard_memory_deleter};

    SerardAdapter serard_adapter;
    struct SerardMemoryResource serard_memory_resource = {&serard_adapter.ins, serardMemoryDeallocate, serardMemoryAllocate};
    serard_adapter.ins = serardInit(serard_memory_resource, serard_memory_resource);
    serard_adapter.ins.node_id = 11;
    serard_adapter.user_reference = &serard_adapter.ins;
    serard_adapter.ins.user_reference = &serard_adapter.ins;
    serard_adapter.reass = serardReassemblerInit();
    serard_adapter.emitter = emit;
    Cyphal<SerardAdapter> serard_cyphal(&serard_adapter);

    CanardAdapter canard_adapter;
    canard_adapter.ins = canardInit(canardMemoryAllocate, canardMemoryFree);
    canard_adapter.ins.node_id = 11;
    canard_adapter.que = canardTxInit(16, CANARD_MTU_CAN_CLASSIC);
    Cyphal<CanardAdapter> canard_cyphal(&canard_adapter);

    auto adapters = TestClass(std::tuple<Cyphal<LoopardAdapter>, Cyphal<UdpardAdapter>, Cyphal<SerardAdapter>, Cyphal<CanardAdapter>>{loopard_cyphal, udpard_cyphal, serard_cyphal, canard_cyphal});

    const char frame[] = "common message";
    size_t frame_size = sizeof(frame);
    CHECK(adapters.txpush_unroll(frame_size, frame) == 1);

    CHECK(loopard_adapter.buffer.size() == 1);
    CHECK(udpard_adapter.ins.queue_size == 1);
    CHECK(canard_adapter.que.size > 0);
    CHECK(rxtx_buffer.size() > 0);
}

TEST_CASE("All Combined Loop")
{
    rxtx_buffer.clear();

    LoopardAdapter loopard_adapter;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard_adapter);

    UdpardAdapter udpard_adapter;
    struct UdpardMemoryDeleter udpard_memory_deleter = {&udpard_adapter.ins, udpardMemoryDeallocate};
    struct UdpardMemoryResource udpard_memory_resource = {&udpard_adapter.ins, udpardMemoryDeallocate, udpardMemoryAllocate};
    UdpardNodeID local_node_id = 11;
    Cyphal<UdpardAdapter> udpard_cyphal(&udpard_adapter);
    udpardTxInit(&udpard_adapter.ins, &local_node_id, 100, udpard_memory_resource);
    udpard_adapter.memory_resources = {udpard_memory_resource, udpard_memory_resource, udpard_memory_deleter};

    SerardAdapter serard_adapter;
    struct SerardMemoryResource serard_memory_resource = {&serard_adapter.ins, serardMemoryDeallocate, serardMemoryAllocate};
    serard_adapter.ins = serardInit(serard_memory_resource, serard_memory_resource);
    serard_adapter.ins.node_id = 11;
    serard_adapter.user_reference = &serard_adapter.ins;
    serard_adapter.ins.user_reference = &serard_adapter.ins;
    serard_adapter.reass = serardReassemblerInit();
    serard_adapter.emitter = emit;
    Cyphal<SerardAdapter> serard_cyphal(&serard_adapter);

    CanardAdapter canard_adapter;
    canard_adapter.ins = canardInit(canardMemoryAllocate, canardMemoryFree);
    canard_adapter.ins.node_id = 11;
    canard_adapter.que = canardTxInit(16, CANARD_MTU_CAN_CLASSIC);
    Cyphal<CanardAdapter> canard_cyphal(&canard_adapter);

    auto adapters = TestClass(std::tuple<Cyphal<LoopardAdapter>, Cyphal<UdpardAdapter>, Cyphal<SerardAdapter>, Cyphal<CanardAdapter>>{loopard_cyphal, udpard_cyphal, serard_cyphal, canard_cyphal});

    const char frame[] = "common message";
    size_t frame_size = sizeof(frame);
    CHECK(adapters.txpush_loop(frame_size, frame) == 1);

    CHECK(loopard_adapter.buffer.size() == 1);
    CHECK(udpard_adapter.ins.queue_size == 1);
    CHECK(canard_adapter.que.size > 0);
    CHECK(rxtx_buffer.size() > 0);
}