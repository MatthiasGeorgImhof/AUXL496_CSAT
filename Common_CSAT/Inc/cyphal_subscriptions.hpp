#ifndef CYPHAL_SUBSCRIPTIONS_HPP_
#define CYPHAL_SUBSCRIPTIONS_HPP_

#include <array>
#include <algorithm>
#include <utility>
#include <type_traits>
#include <initializer_list>

#include "cyphal.hpp"
#include "nunavut_assert.h"
#include "uavcan/node/port/List_1_0.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/node/GetInfo_1_0.h"
#include "uavcan/diagnostic/Record_1_1.h"
#include "uavcan/file/Read_1_1.h"
#include "uavcan/file/Write_1_1.h"
#include "uavcan/time/Synchronization_1_0.h"
#include "uavcan/time/GetSynchronizationMasterInfo_0_1.h"

#include "_4111Spyglass.h"
#include "_4111spyglass/sat/sensor/Magnetometer_0_1.h"
#include "_4111spyglass/sat/sensor/GNSS_0_1.h"
#include "_4111spyglass/sat/solution/OrientationSolution_0_1.h"
#include "_4111spyglass/sat/solution/PositionSolution_0_1.h"

constexpr static std::array CYPHAL_MESSAGES =
{
CyphalSubscription{uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, uavcan_node_Heartbeat_1_0_EXTENT_BYTES_, CyphalTransferKindMessage},
CyphalSubscription{uavcan_node_port_List_1_0_FIXED_PORT_ID_, uavcan_node_port_List_1_0_EXTENT_BYTES_, CyphalTransferKindMessage},
CyphalSubscription{uavcan_diagnostic_Record_1_1_FIXED_PORT_ID_, uavcan_diagnostic_Record_1_1_EXTENT_BYTES_, CyphalTransferKindMessage},
CyphalSubscription{uavcan_time_Synchronization_1_0_FIXED_PORT_ID_, uavcan_time_Synchronization_1_0_EXTENT_BYTES_, CyphalTransferKindMessage},
CyphalSubscription{uavcan_time_GetSynchronizationMasterInfo_0_1_FIXED_PORT_ID_, uavcan_time_GetSynchronizationMasterInfo_Request_0_1_EXTENT_BYTES_, CyphalTransferKindMessage},
CyphalSubscription{_4111spyglass_sat_sensor_Magnetometer_0_1_PORT_ID_, _4111spyglass_sat_sensor_Magnetometer_0_1_EXTENT_BYTES_, CyphalTransferKindMessage},
CyphalSubscription{_4111spyglass_sat_sensor_GNSS_0_1_PORT_ID_, _4111spyglass_sat_sensor_GNSS_0_1_EXTENT_BYTES_, CyphalTransferKindMessage},
CyphalSubscription{_4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_, _4111spyglass_sat_solution_OrientationSolution_0_1_EXTENT_BYTES_, CyphalTransferKindMessage},
CyphalSubscription{_4111spyglass_sat_solution_PositionSolution_0_1_PORT_ID_, _4111spyglass_sat_solution_PositionSolution_0_1_EXTENT_BYTES_, CyphalTransferKindMessage},
};

constexpr static std::array CYPHAL_REQUESTS =
{
CyphalSubscription{uavcan_node_GetInfo_1_0_FIXED_PORT_ID_, uavcan_node_GetInfo_Request_1_0_EXTENT_BYTES_, CyphalTransferKindRequest},
CyphalSubscription{uavcan_file_Write_1_1_FIXED_PORT_ID_, uavcan_file_Write_Request_1_1_EXTENT_BYTES_, CyphalTransferKindRequest},
CyphalSubscription{uavcan_file_Read_1_1_FIXED_PORT_ID_, uavcan_file_Read_Request_1_1_EXTENT_BYTES_, CyphalTransferKindRequest}
};

constexpr static std::array CYPHAL_RESPONSES =
{
CyphalSubscription{uavcan_node_GetInfo_1_0_FIXED_PORT_ID_, uavcan_node_GetInfo_Response_1_0_EXTENT_BYTES_, CyphalTransferKindResponse},
CyphalSubscription{uavcan_file_Write_1_1_FIXED_PORT_ID_, uavcan_file_Write_Response_1_1_EXTENT_BYTES_, CyphalTransferKindResponse},
CyphalSubscription{uavcan_file_Read_1_1_FIXED_PORT_ID_, uavcan_file_Read_Response_1_1_EXTENT_BYTES_, CyphalTransferKindResponse}
};

// Template function for compile-time lookup of boolean values
template <size_t Size, CyphalPortID port_id>
consteval bool containsPortIdCompileTime(const std::array<CyphalSubscription, Size>& arr) {
    for (auto const &item : arr) {
        if (item.port_id == port_id) {
            return true;
        }
    }
    return false;
}

// Template function for compile-time lookup
template <size_t Size, CyphalPortID port_id>
consteval CyphalSubscription const *findByPortIdCompileTime(const std::array<CyphalSubscription, Size>& arr) {
    for (auto const &item : arr) {
        if (item.port_id == port_id) {
            return &item;
        }
    }
    return nullptr;
}

// Template function for runtime lookup
template <size_t Size>
CyphalSubscription const *findByPortIdRuntime(const std::array<CyphalSubscription, Size>& arr, CyphalPortID port_id) {
    for (auto const &item : arr) {
        if (item.port_id == port_id) {
            return &item;
        }
    }
    return nullptr;
}


// Convenience functions for specific arrays.  These remove the need to explicitly state the array size.

// Compile-time lookup for CYPHAL_MESSAGES
template <CyphalPortID port_id>
consteval bool containsMessageByPortIdCompileTime() {
    return containsPortIdCompileTime<CYPHAL_MESSAGES.size(), port_id>(CYPHAL_MESSAGES);
}

template <CyphalPortID port_id>
consteval CyphalSubscription const *findMessageByPortIdCompileTime() {
    return findByPortIdCompileTime<CYPHAL_MESSAGES.size(), port_id>(CYPHAL_MESSAGES);
}

// Runtime lookup for CYPHAL_MESSAGES
CyphalSubscription const *findMessageByPortIdRuntime(CyphalPortID port_id) {
    return findByPortIdRuntime<CYPHAL_MESSAGES.size()>(CYPHAL_MESSAGES, port_id);
}

// Compile-time lookup for CYPHAL_REQUESTS
template <CyphalPortID port_id>
consteval CyphalSubscription const *findRequestByPortIdCompileTime() {
    return findByPortIdCompileTime<CYPHAL_REQUESTS.size(), port_id>(CYPHAL_REQUESTS);
}

// Runtime lookup for CYPHAL_REQUESTS
CyphalSubscription const *findRequestByPortIdRuntime(CyphalPortID port_id) {
    return findByPortIdRuntime<CYPHAL_REQUESTS.size()>(CYPHAL_REQUESTS, port_id);
}

// Compile-time lookup for CYPHAL_RESPONSES
template <CyphalPortID port_id>
consteval CyphalSubscription const *findResponseByPortIdCompileTime() {
    return findByPortIdCompileTime<CYPHAL_RESPONSES.size(), port_id>(CYPHAL_RESPONSES);
}

// Runtime lookup for CYPHAL_RESPONSES
CyphalSubscription const *findResponseByPortIdRuntime(CyphalPortID port_id) {
    return findByPortIdRuntime<CYPHAL_RESPONSES.size()>(CYPHAL_RESPONSES, port_id);
}

#endif // CYPHAL_SUBSCRIPTIONS_HPP_
