#include "cyphal.hpp"

CyphalTransfer createTransfer(size_t payload_size, uint8_t *payload, void *data,
                              int8_t (*serialize)(const void *const, uint8_t *const, size_t *const),
                              CyphalTransferMetadata metadata)
{
    (void) serialize(data, payload, &payload_size);

    return CyphalTransfer{
        .metadata = metadata,
        .timestamp_usec = 0,
        .payload_size = payload_size,
        .payload = payload};
}

CyphalTransfer createTransfer(size_t payload_size, uint8_t *payload, void *data,
                              int8_t (*serialize)(const void *const, uint8_t *const, size_t *const),
                              CyphalPortID port_id, CyphalTransferKind transfer_kind, CyphalNodeID node_id, CyphalTransferID transfer_id)
{
    CyphalTransferMetadata metadata =
        {
            CyphalPriorityNominal,
            static_cast<CyphalTransferKind>(transfer_kind), // Use the transfer_kind argument
            port_id,
            node_id,
			CYPHAL_NODE_ID_UNSET,
			CYPHAL_NODE_ID_UNSET,
            transfer_id,
        };

    return createTransfer(payload_size, payload, data, serialize, metadata);
}

void unpackTransfer(const CyphalTransfer *transfer, int8_t (*deserialize)(uint8_t *data, const uint8_t *payload, size_t *payload_size), uint8_t *data)
{
    size_t payload_size = transfer->payload_size;
    (void) deserialize(data, static_cast<const uint8_t *>(transfer->payload), &payload_size);
}

CyphalTransferID wrap_transfer_id(CyphalTransferID id)
{
	return id & 0x1f;
}
