#pragma once

#include "cyphal.hpp"
#include "serard.h"

#include "BoxSet.hpp"

static_assert(CYPHAL_NODE_ID_UNSET != SERARD_NODE_ID_UNSET, "CYPHAL_NODE_ID_UNSET != SERARD_NODE_ID_UNSET");
static_assert(sizeof(CyphalTransferMetadata) != sizeof(SerardTransferMetadata), "metadata sizes should differ");
// static_assert(sizeof(CyphalTransfer) == sizeof(SerardRxTransfer), "rxtransfer sizes differ");

struct SerardAdapter
{
    static constexpr uint8_t SUBSCRIPTIONS = 32;
    struct Serard ins;
    struct SerardReassembler reass;
    SerardTxEmit emitter;
    void *user_reference;
    BoxSet<SerardRxSubscription, SUBSCRIPTIONS> subscriptions;
};

inline SerardNodeID cyphalNodeIdToSerard(const CyphalNodeID node_id) { return node_id == CYPHAL_NODE_ID_UNSET ? SERARD_NODE_ID_UNSET : static_cast<SerardNodeID>(node_id); }
inline CyphalNodeID serardNodeIdToCyphal(const SerardNodeID node_id) { return static_cast<CyphalNodeID>(node_id & CYPHAL_NODE_ID_UNSET); }
inline SerardTransferID cyphalTransferIdToSerard(const CyphalTransferID transfer_id) { return static_cast<SerardTransferID>(transfer_id); }
inline CyphalTransferID serardTransferIdToCyphal(const SerardTransferID transfer_id) { return static_cast<CyphalTransferID>(transfer_id); }

inline SerardTransferMetadata cyphalMetadataToSerard(const CyphalTransferMetadata metadata)
{
    return SerardTransferMetadata{
        static_cast<SerardPriority>(metadata.priority),
        static_cast<SerardTransferKind>(metadata.transfer_kind),
        static_cast<SerardPortID>(metadata.port_id),
        cyphalNodeIdToSerard(metadata.remote_node_id),
        cyphalNodeIdToSerard(metadata.source_node_id),
        cyphalNodeIdToSerard(metadata.destination_node_id),
        cyphalTransferIdToSerard(metadata.transfer_id)};
}

inline CyphalTransferMetadata serardMetadataToCyphal(const SerardTransferMetadata metadata)
{
    return CyphalTransferMetadata{
        static_cast<CyphalPriority>(metadata.priority),
        static_cast<CyphalTransferKind>(metadata.transfer_kind),
        static_cast<CyphalPortID>(metadata.port_id),
        serardNodeIdToCyphal(metadata.remote_node_id),
        serardNodeIdToCyphal(metadata.source_node_id),
        serardNodeIdToCyphal(metadata.destination_node_id),
        serardTransferIdToCyphal(metadata.transfer_id)};
}

inline void cyphalMetadataToSerard(const CyphalTransferMetadata *cyphal, SerardTransferMetadata *serard)
{
    serard->priority = static_cast<SerardPriority>(cyphal->priority);
    serard->transfer_kind = static_cast<SerardTransferKind>(cyphal->transfer_kind);
    serard->port_id = static_cast<SerardPortID>(cyphal->port_id);
    serard->remote_node_id = cyphalNodeIdToSerard(cyphal->remote_node_id);
    serard->source_node_id = cyphalNodeIdToSerard(cyphal->source_node_id);
    serard->destination_node_id = cyphalNodeIdToSerard(cyphal->destination_node_id);
    serard->transfer_id = cyphalTransferIdToSerard(cyphal->transfer_id);
}

inline void cyphalTransferToSerard(const CyphalTransfer *cyphal, SerardRxTransfer *serard)
{
    cyphalMetadataToSerard(&cyphal->metadata, &serard->metadata);
    serard->payload = cyphal->payload;
    serard->payload_size = cyphal->payload_size;
    serard->timestamp_usec = cyphal->timestamp_usec;
}

inline void serardMetadataToCyphal(const SerardTransferMetadata *serard, CyphalTransferMetadata *cyphal)
{
    cyphal->priority = static_cast<CyphalPriority>(serard->priority);
    cyphal->transfer_kind = static_cast<CyphalTransferKind>(serard->transfer_kind);
    cyphal->port_id = serard->port_id;
    cyphal->remote_node_id = serardNodeIdToCyphal(serard->remote_node_id);
    cyphal->source_node_id = serardNodeIdToCyphal(serard->source_node_id);
    cyphal->destination_node_id = serardNodeIdToCyphal(serard->destination_node_id);
    cyphal->transfer_id = serardTransferIdToCyphal(serard->transfer_id);
}

inline void serardTransferToCyphal(const SerardRxTransfer *serard, CyphalTransfer *cyphal)
{
    serardMetadataToCyphal(&serard->metadata, &cyphal->metadata);
    cyphal->payload = serard->payload;
    cyphal->payload_size = serard->payload_size;
    cyphal->timestamp_usec = serard->timestamp_usec;
}

template <>
class Cyphal<SerardAdapter>
{
private:
    SerardAdapter *adapter_;

public:
    Cyphal(SerardAdapter *adapter) : adapter_(adapter) {}

    int32_t cyphalTxPush(const CyphalMicrosecond /*tx_deadline_usec*/,
                         const CyphalTransferMetadata *const metadata,
                         const size_t payload_size,
                         const void *const payload)
    {
        SerardTransferMetadata serard_metadata = cyphalMetadataToSerard(*metadata);
        log(LOG_LEVEL_DEBUG, "serardTxPush at %08u: %3d -> %3d (%4d %3d)\r\n", HAL_GetTick(),
        		metadata->source_node_id, metadata->destination_node_id, metadata->port_id, metadata->transfer_id);

        return serardTxPush(&adapter_->ins, &serard_metadata, payload_size, payload, adapter_->user_reference, adapter_->emitter);
    }

    inline CanardNodeID getNodeID() const { return serardNodeIdToCyphal(adapter_->ins.node_id); }
    inline void setNodeID(const CanardNodeID node_id) { adapter_->ins.node_id = node_id; }

    int32_t cyphalTxForward(const CyphalMicrosecond tx_deadline_usec,
                            const CyphalTransferMetadata *const metadata,
                            const size_t payload_size,
                            const void *const payload,
                            const CyphalNodeID /*node_id*/)
    {
        CyphalTransferMetadata metadata_{*metadata};
        metadata_.remote_node_id = metadata_.destination_node_id;

        CanardNodeID node_id_ = getNodeID();
        setNodeID(metadata_.source_node_id);

        int32_t res{};
        res = cyphalTxPush(tx_deadline_usec, &metadata_, payload_size, payload);
        (void)tx_deadline_usec;
        (void)payload_size;
        (void)payload;

        setNodeID(node_id_);
        log(LOG_LEVEL_DEBUG, "serardTxForward at %08u: %3d -> %3d (%4d %3d)\r\n", HAL_GetTick(),
        		metadata->source_node_id, metadata->destination_node_id, metadata->port_id, metadata->transfer_id);
        return res;
    }

    int8_t cyphalRxSubscribe(const CyphalTransferKind transfer_kind,
                             const CyphalPortID port_id,
                             const size_t extent,
                             const CyphalMicrosecond transfer_id_timeout_usec)
    {
        SerardRxSubscription stub{};
        stub.port_id = port_id;
        SerardRxSubscription *subscription = adapter_->subscriptions.find_or_create(stub, [port_id](const SerardRxSubscription &a, const SerardRxSubscription &b)
                                                                                    { return a.port_id == b.port_id; });
        return serardRxSubscribe(&adapter_->ins, static_cast<SerardTransferKind>(transfer_kind), port_id, extent, transfer_id_timeout_usec, subscription);
    }

    int8_t cyphalRxUnsubscribe(const CyphalTransferKind transfer_kind,
                               const CyphalPortID port_id)
    {
        SerardRxSubscription stub{};
        stub.port_id = port_id;
        auto it = adapter_->subscriptions.find(stub, [port_id](const SerardRxSubscription &a, const SerardRxSubscription &b)
                                               { return a.port_id == b.port_id; });
        auto result = serardRxUnsubscribe(&adapter_->ins, static_cast<SerardTransferKind>(transfer_kind), port_id);
        if (it)
            adapter_->subscriptions.remove(it);
        return result;
    }

    int32_t cyphalRxReceive(size_t *frame_size, const uint8_t *const frame, CyphalTransfer *out_transfer)
    {
        SerardRxSubscription *sub;
        struct SerardRxTransfer serard_transfer{};
        int8_t result = serardRxAccept(&adapter_->ins, &adapter_->reass, 0, frame_size, frame, &serard_transfer, &sub);
        out_transfer->metadata.priority = static_cast<CyphalPriority>(serard_transfer.metadata.priority);
        out_transfer->metadata.transfer_kind = static_cast<CyphalTransferKind>(serard_transfer.metadata.transfer_kind);
        out_transfer->metadata.port_id = serard_transfer.metadata.port_id;
        out_transfer->metadata.remote_node_id = serardNodeIdToCyphal(serard_transfer.metadata.remote_node_id);
        out_transfer->metadata.source_node_id = serardNodeIdToCyphal(serard_transfer.metadata.source_node_id);
        out_transfer->metadata.destination_node_id = serardNodeIdToCyphal(serard_transfer.metadata.destination_node_id);
        out_transfer->metadata.transfer_id = serardTransferIdToCyphal(serard_transfer.metadata.transfer_id);
        out_transfer->payload = serard_transfer.payload;
        out_transfer->payload_size = serard_transfer.payload_size;
        out_transfer->timestamp_usec = serard_transfer.timestamp_usec;
        return result;
    }
};

#include "cyphal_adapter_api.hpp"

// Call the checks *after* the class definition
static_assert((checkCyphalAdapterAPI<SerardAdapter>(), true), "SerardAdapter fails API check");
