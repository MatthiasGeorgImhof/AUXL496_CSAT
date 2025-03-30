#pragma once

#include <memory>
#include <cstring>

#include "cyphal.hpp"

#include "BoxSet.hpp"
#include "CircularBuffer.hpp"

#ifdef __cplusplus
extern "C" {
#endif

typedef void* (*LoopardMemoryAllocate)(size_t amount);
typedef void (*LoopardMemoryFree)(void* pointer);
#ifdef __cplusplus
}
#endif


struct LoopardAdapter
{
    static constexpr uint8_t SUBSCRIPTIONS = 32;
    static constexpr size_t BUFFER = 32;
    CircularBuffer<CyphalTransfer, BUFFER> buffer;
    BoxSet<uint8_t, SUBSCRIPTIONS> subscriptions;
    CyphalNodeID node_id = CYPHAL_NODE_ID_UNSET;

    LoopardMemoryAllocate memory_allocate;
    LoopardMemoryFree     memory_free;
};

template <>
class Cyphal<LoopardAdapter>
{
private:
    LoopardAdapter *adapter_;

public:
    Cyphal<LoopardAdapter>() = delete;
    Cyphal<LoopardAdapter>(LoopardAdapter *adapter) : adapter_(adapter) {}

    int32_t cyphalTxPush(const CyphalMicrosecond tx_deadline_usec,
                         const CyphalTransferMetadata *const metadata,
                         const size_t payload_size,
                         const void *const payload)
    {
        if (adapter_->buffer.is_full())
            return 0;
        const_cast<CyphalTransferMetadata*>(metadata)->remote_node_id = adapter_->node_id;
        CyphalTransfer transfer =
            {
                .metadata = *metadata,
                .timestamp_usec = 0,
                .payload_size = payload_size,
                .payload = adapter_->memory_allocate(payload_size)
            };
        std::memcpy(transfer.payload, payload, payload_size);
        adapter_->buffer.push(transfer);
        return 1;
    }

    inline CyphalNodeID getNodeID() const { return adapter_->node_id; }
    inline void setNodeID(const CyphalNodeID node_id) { adapter_->node_id = node_id; }

    int32_t cyphalTxForward(const CyphalMicrosecond tx_deadline_usec,
                            const CyphalTransferMetadata *const metadata,
                            const size_t payload_size,
                            const void *const payload,
                            const CyphalNodeID node_id)
    {
        CyphalNodeID node_id_ = getNodeID();
        setNodeID(node_id);
        cyphalTxPush(tx_deadline_usec, metadata, payload_size, payload);
        setNodeID(node_id_);
        return 1;
    }

    int8_t cyphalRxSubscribe(const CyphalTransferKind transfer_kind,
                             const CyphalPortID port_id,
                             const size_t extent,
                             const CyphalMicrosecond transfer_id_timeout_usec)
    {
        adapter_->subscriptions.find_or_create(port_id, [](const uint8_t &a, const uint8_t &b)
                                                                       { return a == b; });
        return 1;
    }

    int8_t cyphalRxUnsubscribe(const CyphalTransferKind transfer_kind,
                               const CyphalPortID port_id)
    {
        uint8_t *subscription = adapter_->subscriptions.find(port_id, [](const uint8_t &a, const uint8_t &b)
                                                             { return a == b; });
        if (subscription)
            adapter_->subscriptions.remove(subscription);
        return 1;
    }

    int32_t cyphalRxReceive(const uint8_t *const frame, size_t *frame_size, CyphalTransfer *out_transfer)
    {
        if (adapter_->buffer.is_empty())
            return 0;
        *out_transfer = adapter_->buffer.pop();
        return (adapter_->buffer.is_empty() ? 1 : 2);
    }
};

#include "cyphal_adapter_api.hpp"

// Call the checks *after* the class definition
static_assert((checkCyphalAdapterAPI<LoopardAdapter>(), true), "LoopardAdapter fails API check");