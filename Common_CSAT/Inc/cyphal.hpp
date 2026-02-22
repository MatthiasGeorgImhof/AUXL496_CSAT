#pragma once

#include <cstdint>
#include <cstddef>

#define CYPHAL_ERROR_ARGUMENT 2
#define CYPHAL_ERROR_MEMORY 3
#define CYPHAL_ERROR_CAPACITY 4
#define CYPHAL_ERROR_ANONYMOUS 5

#define CYPHAL_NODE_ID_UNSET 0xFFU
#define CYPHAL_DEFAULT_TRANSFER_ID_TIMEOUT_USEC 2000000UL

typedef enum
{
    CyphalPriorityExceptional = 0,
    CyphalPriorityImmediate = 1,
    CyphalPriorityFast = 2,
    CyphalPriorityHigh = 3,
    CyphalPriorityNominal = 4, ///< Nominal priority level should be the default.
    CyphalPriorityLow = 5,
    CyphalPrioritySlow = 6,
    CyphalPriorityOptional = 7,
} CyphalPriority;

typedef enum
{
    CyphalTransferKindMessage = 0,  ///< Multicast, from publisher to all subscribers.
    CyphalTransferKindResponse = 1, ///< Point-to-point, from server to client.
    CyphalTransferKindRequest = 2,  ///< Point-to-point, from client to server.
} CyphalTransferKind;
#define CYPHAL_NUM_TRANSFER_KINDS 3

typedef uint64_t CyphalMicrosecond;
typedef uint16_t CyphalPortID;
typedef uint8_t CyphalNodeID;
typedef uint8_t CyphalTransferID;

struct CyphalForwardRange
{
	CyphalNodeID start_id;
	CyphalNodeID end_id;
};

typedef struct
{
    CyphalPriority priority;
    CyphalTransferKind transfer_kind;
    CyphalPortID port_id;
    CyphalNodeID remote_node_id;
    CyphalNodeID source_node_id;
    CyphalNodeID destination_node_id;
    CyphalTransferID transfer_id;
} CyphalTransferMetadata;

typedef struct CyphalTransfer
{
    CyphalTransferMetadata metadata;
    CyphalMicrosecond timestamp_usec;
    size_t payload_size;
    void *payload;
} CyphalTransfer;

typedef struct CyphalSubscription
{
    CyphalPortID port_id;
    size_t extent;
    CyphalTransferKind transfer_kind;
} CyphalSubscription;

typedef CyphalSubscription CyphalPublication;

CyphalTransfer createTransfer(size_t payload_size, uint8_t *payload, void *data,
                              int8_t (*serialize)(const void *const, uint8_t *const, size_t *const),
                              CyphalTransferMetadata metadata);

CyphalTransfer createTransfer(size_t payload_size, uint8_t *payload, void *data,
                              int8_t (*serialize)(const void *const, uint8_t *const, size_t *const),
                              CyphalPortID port_id,
                              CyphalTransferKind transfer_kind = CyphalTransferKindMessage,
                              CyphalNodeID node_id = CYPHAL_NODE_ID_UNSET,
                              CyphalTransferID transfer_id = 0);

void unpackTransfer(const CyphalTransfer *transfer, int8_t (*deserialize)(uint8_t *data, const uint8_t *payload, size_t *payload_size), uint8_t *data);

CyphalTransferID wrap_transfer_id(CyphalTransferID id);

template <typename Adapter>
class Cyphal;
