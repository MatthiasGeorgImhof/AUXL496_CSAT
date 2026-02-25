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

#
#
#

bool is_service(uint32_t ext_id) { return ext_id & 0x2000; }

CyphalHeader parse_message_header(uint32_t header)
{
	return CyphalHeader {
		.is_service = is_service(header),
		.port_id = static_cast<CyphalPortID>((header>>8) & 8191),
		.source_id = static_cast<CyphalNodeID>(header & 127),
		.destination_id = 255
	};
}

CyphalHeader parse_service_header(uint32_t header)
{
	return CyphalHeader {
	.is_service = is_service(header),
	.port_id = static_cast<CyphalPortID>((header>>14) & 511),
	.source_id = static_cast<CyphalNodeID>(header & 127),
	.destination_id = static_cast<CyphalNodeID>((header>>7) & 127)
	};
}

CyphalHeader parse_header(uint32_t header)
{
	if (is_service(header))
		return parse_service_header(header);
	return parse_message_header(header);
}

CyphalTransferID wrap_transfer_id(CyphalTransferID id)
{
	return id & 0x1f;
}

