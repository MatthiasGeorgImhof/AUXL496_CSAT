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
#include "uavcan/diagnostic/Record_1_1.h"
#include "uavcan/file/Read_1_1.h"
#include "uavcan/file/Write_1_1.h"
#include "uavcan/time/Synchronization_1_0.h"
#include "uavcan/time/GetSynchronizationMasterInfo_0_1.h"

#include "_4111Spyglass.h"
#include "_4111spyglass/sat/sensor/Magnetometer_0_1.h"
#include "_4111spyglass/sat/sensor/GNSS_0_1.h"


constexpr static std::array CYPHAL_MESSAGES =
{
    CyphalSubscription{uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, uavcan_node_Heartbeat_1_0_EXTENT_BYTES_, CyphalTransferKindMessage},
    CyphalSubscription{uavcan_node_port_List_1_0_FIXED_PORT_ID_, uavcan_node_port_List_1_0_EXTENT_BYTES_, CyphalTransferKindMessage},
    CyphalSubscription{uavcan_diagnostic_Record_1_1_FIXED_PORT_ID_, uavcan_diagnostic_Record_1_1_EXTENT_BYTES_, CyphalTransferKindMessage},
    CyphalSubscription{uavcan_time_Synchronization_1_0_FIXED_PORT_ID_, uavcan_time_Synchronization_1_0_EXTENT_BYTES_, CyphalTransferKindMessage},
    CyphalSubscription{uavcan_time_GetSynchronizationMasterInfo_0_1_FIXED_PORT_ID_, uavcan_time_GetSynchronizationMasterInfo_Request_0_1_EXTENT_BYTES_, CyphalTransferKindMessage},
    CyphalSubscription{_4111spyglass_sat_sensor_Magnetometer_0_1_PORT_ID_, _4111spyglass_sat_sensor_Magnetometer_0_1_EXTENT_BYTES_, CyphalTransferKindMessage},
    CyphalSubscription{_4111spyglass_sat_sensor_GNSS_0_1_PORT_ID_, _4111spyglass_sat_sensor_GNSS_0_1_EXTENT_BYTES_, CyphalTransferKindMessage}
};

constexpr CyphalSubscription const *findByPortIdCompileTime(CyphalPortID port_id)
{
    for (auto const &item : CYPHAL_MESSAGES)
    {
        if (item.port_id == port_id)
        {
            return &item;
        }
    }
    return nullptr;
}

CyphalSubscription const *findByPortIdRuntime(CyphalPortID port_id)
{
    for (auto const &item : CYPHAL_MESSAGES)
    {
        if (item.port_id == port_id)
        {
            return &item;
        }
    }
    return nullptr;
}

#endif // CYPHAL_SUBSCRIPTIONS_HPP_