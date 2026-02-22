#pragma once

#include "Logger.hpp"

#include "cyphal.hpp"
#include "canard.h"

#include <cstring>
#include "BoxSet.hpp"
#include "CanTxQueueDrainer.hpp"

extern CanTxQueueDrainer tx_drainer;

static_assert(CYPHAL_NODE_ID_UNSET == CANARD_NODE_ID_UNSET, "unset lengths differ");
static_assert(sizeof(CyphalTransferMetadata) == sizeof(CanardTransferMetadata), "metadata sizes differ");
static_assert(sizeof(CyphalTransfer) == sizeof(CanardRxTransfer), "rxtransfer sizes differ");

inline void cyphalMetadataToCanard(const CyphalTransferMetadata *cyphal, CanardTransferMetadata *canard)
{
    memcpy(canard, cyphal, sizeof(CyphalTransferMetadata));
}

inline void cyphalTransferToCanard(const CyphalTransfer *cyphal, CanardRxTransfer *canard)
{
    memcpy(canard, cyphal, sizeof(CyphalTransfer));
}

inline void canardMetadataToCyphal(const CanardTransferMetadata *canard, CyphalTransferMetadata *cyphal)
{
    memcpy(cyphal, canard, sizeof(CanardTransferMetadata));
}

inline void canardTransferToCyphal(const CanardRxTransfer *canard, CyphalTransfer *cyphal)
{
    memcpy(cyphal, canard, sizeof(CanardRxTransfer));
}

struct CanardAdapter
{
    static constexpr uint8_t SUBSCRIPTIONS = 32;
    CanardInstance ins;
    CanardTxQueue que;
    BoxSet<CanardRxSubscription, SUBSCRIPTIONS> subscriptions;
};

template <>
class Cyphal<CanardAdapter>
{
private:
    CanardAdapter *adapter_;

public:
    Cyphal(CanardAdapter *adapter) : adapter_(adapter) {}

    int32_t cyphalTxPush(const CyphalMicrosecond tx_deadline_usec,
                         const CyphalTransferMetadata *const metadata,
                         const size_t payload_size,
                         const void *const payload)
    {
        log(LOG_LEVEL_INFO, "canardTxPush at %08u: %3d -> %3d (%4d %3d)\r\n", HAL_GetTick(),
        		metadata->source_node_id, metadata->destination_node_id, metadata->port_id, metadata->transfer_id);


    	auto res = canardTxPush(&adapter_->que, &adapter_->ins, tx_deadline_usec, reinterpret_cast<const CanardTransferMetadata *>(metadata), payload_size, payload);
        tx_drainer.irq_safe_drain();
        return res;
    }

    inline CanardNodeID getNodeID() const { return adapter_->ins.node_id; }
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

        setNodeID(node_id_);
        log(LOG_LEVEL_INFO, "canardTxForward at %08u: %3d -> %3d (%4d %3d)\r\n", HAL_GetTick(),
        		metadata->source_node_id, metadata->destination_node_id, metadata->port_id, metadata->transfer_id);
        return res;
    }

    int8_t cyphalRxSubscribe(const CyphalTransferKind transfer_kind,
                             const CyphalPortID port_id,
                             const size_t extent,
                             const CyphalMicrosecond transfer_id_timeout_usec)
    {
        CanardRxSubscription stub = {};
        stub.port_id = port_id;
        CanardRxSubscription *subscription = adapter_->subscriptions.find_or_create(stub, [](const CanardRxSubscription &a, const CanardRxSubscription &b)
                                                                                    { return a.port_id == b.port_id; });
        return canardRxSubscribe(&adapter_->ins, static_cast<CanardTransferKind>(transfer_kind), port_id, extent, transfer_id_timeout_usec, subscription);
    }

    int8_t cyphalRxUnsubscribe(const CyphalTransferKind transfer_kind,
                               const CyphalPortID port_id)
    {
        CanardRxSubscription stub = {};
        stub.port_id = port_id;
        CanardRxSubscription *subscription = adapter_->subscriptions.find(stub, [](const CanardRxSubscription &a, const CanardRxSubscription &b)
                                                                          { return a.port_id == b.port_id; });
        auto result = canardRxUnsubscribe(&adapter_->ins, static_cast<CanardTransferKind>(transfer_kind), port_id);
        if (subscription)
            adapter_->subscriptions.remove(subscription);
        return result;
    }

    int32_t cyphalRxReceive(uint32_t extended_can_id, const size_t *frame_size, const uint8_t *const frame, CyphalTransfer *out_transfer)
    {
        CanardFrame canard_frame = {extended_can_id, *frame_size, frame};
        return canardRxAccept(&adapter_->ins, 0, &canard_frame, 0, reinterpret_cast<CanardRxTransfer *>(out_transfer), nullptr);
    }
};

#include "cyphal_adapter_api.hpp"

// Call the checks *after* the class definition
static_assert((checkCyphalAdapterAPI<CanardAdapter>(), true), "CanardAdapter fails API check");
