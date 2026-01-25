#include "cyphal.hpp"
#include "udpard.h"

#include "BoxSet.hpp"

static_assert(CYPHAL_NODE_ID_UNSET != UDPARD_NODE_ID_UNSET, "CYPHAL_NODE_ID_UNSET != UDPARD_NODE_ID_UNSET");
static_assert(sizeof(CyphalTransfer) != sizeof(UdpardRxTransfer), "sizeof(CyphalTransfer) != sizeof(UdpardRxTransfer)");

struct UdpardPortSubscription
{
    UdpardPortID port_id;
    UdpardRxSubscription subscription;
};

struct UdpardAdapter
{
    static constexpr uint8_t SUBSCRIPTIONS = 32;
    struct UdpardTx ins;
    UdpardRxMemoryResources memory_resources;
    void *user_transfer_reference;
    BoxSet<UdpardPortSubscription, SUBSCRIPTIONS> subscriptions;
};

struct UdpardHeader
{
    uint8_t version;
    uint8_t priority;
    uint16_t source_node_id;
    uint16_t destination_node_id;
    uint16_t data_specifier_snm;
    uint64_t transfer_id;
    uint32_t frame_index_eot;
    uint16_t user_data;
    uint8_t header_crc16_big_endian[2];
};

inline UdpardNodeID cyphalNodeIdToUdpard(const CyphalNodeID node_id)
{
    return node_id == CYPHAL_NODE_ID_UNSET ? UDPARD_NODE_ID_UNSET : static_cast<UdpardNodeID>(node_id);
}
inline CyphalNodeID udpardNodeIdToCyphal(const UdpardNodeID node_id) { return static_cast<CyphalNodeID>(node_id & CYPHAL_NODE_ID_UNSET); }
inline UdpardTransferID cyphalTransferIdToUdpard(const CyphalTransferID transfer_id) { return static_cast<UdpardTransferID>(transfer_id); }
inline CyphalTransferID udpardTransferIdToCyphal(const UdpardTransferID transfer_id) { return static_cast<CyphalTransferID>(transfer_id); }

inline void udpartToCyphalMetadata(const UdpardRxTransfer *udpard, const UdpardHeader *header, CyphalTransferMetadata *cyphal)
{
    cyphal->priority = static_cast<CyphalPriority>(udpard->priority);
    cyphal->transfer_kind = CyphalTransferKindMessage;
    cyphal->port_id = header->destination_node_id;
    cyphal->remote_node_id = udpardNodeIdToCyphal(udpard->source_node_id);
    cyphal->source_node_id = udpardNodeIdToCyphal(udpard->source_node_id);
    cyphal->destination_node_id = udpardNodeIdToCyphal(header->destination_node_id);
    cyphal->transfer_id = udpardTransferIdToCyphal(udpard->transfer_id);
}

inline void udpartToCyphalTransfer(const UdpardRxTransfer *udpard, const UdpardHeader *header, CyphalTransfer *cyphal)
{
    udpartToCyphalMetadata(udpard, header, &cyphal->metadata);
    cyphal->payload = static_cast<uint8_t *>(const_cast<void *>(udpard->payload.view.data));
    cyphal->payload_size = udpard->payload_size;
    cyphal->timestamp_usec = udpard->timestamp_usec;
}

template <>
class Cyphal<UdpardAdapter>
{
private:
    UdpardAdapter *adapter_;

public:
    Cyphal(UdpardAdapter *adapter) : adapter_(adapter) {}

    int32_t cyphalTxPush(const CyphalMicrosecond tx_deadline_usec,
                         const CyphalTransferMetadata *const metadata,
                         const size_t payload_size,
                         const void *const payload)
    {
        struct UdpardPayload payload_ = {payload_size, payload};
        return udpardTxPublish(&adapter_->ins, tx_deadline_usec, static_cast<UdpardPriority>(metadata->priority), static_cast<UdpardPortID>(metadata->port_id), cyphalTransferIdToUdpard(metadata->transfer_id), payload_, adapter_->user_transfer_reference);
    }

    inline const UdpardNodeID* getNodeID() { return adapter_->ins.local_node_id; }
    inline void setNodeID(const UdpardNodeID *node_id) { adapter_->ins.local_node_id = node_id; }

    int32_t cyphalTxForward(const CyphalMicrosecond tx_deadline_usec,
                         const CyphalTransferMetadata *const metadata,
                         const size_t payload_size,
                         const void *const payload,
                         const CyphalNodeID node_id)
    {
        const UdpardNodeID *node_id_ = getNodeID();
        UdpardNodeID forward_node_id = cyphalNodeIdToUdpard(node_id);
        setNodeID(&forward_node_id);
        int32_t res =  cyphalTxPush(tx_deadline_usec, metadata, payload_size, payload);
        setNodeID(node_id_);
        return res;
    }

    int8_t cyphalRxSubscribe(const CyphalTransferKind /*transfer_kind*/,
                             const CyphalPortID port_id,
                             const size_t extent,
                             const CyphalMicrosecond /*transfer_id_timeout_usec*/)
    {
        UdpardPortSubscription stub = {port_id, {}};
        UdpardPortSubscription *subscription = adapter_->subscriptions.find_or_create(stub, [](const UdpardPortSubscription &a, const UdpardPortSubscription &b)
                                                                                      { return a.port_id == b.port_id; });
        if (!subscription)
            return -4;
        int_fast8_t result = udpardRxSubscriptionInit(&subscription->subscription, port_id, extent, adapter_->memory_resources);
        return result >= 0 ? 1 : result;
    }

    int8_t cyphalRxUnsubscribe(const CyphalTransferKind /*transfer_kind*/,
                               const CyphalPortID port_id)
    {
        UdpardPortSubscription stub = {port_id, {}};
        UdpardPortSubscription *subscription = adapter_->subscriptions.find_or_create(stub, [](const UdpardPortSubscription &a, const UdpardPortSubscription &b)
                                                                                      { return a.port_id == b.port_id; });
        if (!subscription)
            return 0;

        udpardRxSubscriptionFree(&subscription->subscription);
        adapter_->subscriptions.remove(subscription);
        return 1;
    }

    int32_t cyphalRxReceive(size_t *frame_size, const uint8_t *const frame, CyphalTransfer *out_transfer)
    {
        const UdpardHeader *header = reinterpret_cast<const UdpardHeader *>(frame);
        struct UdpardPortSubscription stub = {header->data_specifier_snm, {}};
        UdpardPortSubscription *subpair = adapter_->subscriptions.find(stub, [](const UdpardPortSubscription &a, const UdpardPortSubscription &b)
                                                                       { return a.port_id == b.port_id; });
        UdpardMutablePayload payload = {*frame_size, static_cast<void *>(const_cast<uint8_t *>(frame))};

        UdpardRxTransfer udpard_transfer;
        udpardRxSubscriptionReceive(&subpair->subscription, 0, payload, 0, &udpard_transfer);

        out_transfer->metadata.priority = static_cast<CyphalPriority>(udpard_transfer.priority);
        out_transfer->metadata.transfer_kind = CyphalTransferKindMessage;
        out_transfer->metadata.port_id = header->destination_node_id;
        out_transfer->metadata.remote_node_id = udpardNodeIdToCyphal(udpard_transfer.source_node_id);
        out_transfer->metadata.transfer_id = udpardTransferIdToCyphal(udpard_transfer.transfer_id);
        out_transfer->payload = static_cast<uint8_t *>(const_cast<void *>(udpard_transfer.payload.view.data));
        out_transfer->payload_size = udpard_transfer.payload_size;
        out_transfer->timestamp_usec = udpard_transfer.timestamp_usec;
        return 1;
    }
};

#include "cyphal_adapter_api.hpp"

// Call the checks *after* the class definition
static_assert((checkCyphalAdapterAPI<UdpardAdapter>(), true), "UdpardAdapter fails API check");
