#ifndef INC_TASKSENDHEARTBEAT_HPP_
#define INC_TASKSENDHEARTBEAT_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"

#include "nunavut_assert.h"
#include "uavcan/node/Heartbeat_1_0.h"

template <typename... Adapters>
class TaskSendHeartBeat : public TaskWithPublication<Adapters...>
{
public:
    TaskSendHeartBeat() = delete;
    TaskSendHeartBeat(uint32_t interval, uint32_t tick, CyphalTransferID transfer_id, std::tuple<Adapters...> &adapters) : TaskWithPublication<Adapters...>(interval, tick, transfer_id, adapters) {}

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;
    virtual void handleMessage(std::shared_ptr<CyphalTransfer> /*transfer*/) override {}

private:
    CyphalSubscription createSubscription();
};

template <typename... Adapters>
void TaskSendHeartBeat<Adapters...>::handleTaskImpl()
{
    // @formatter:off
    uavcan_node_Heartbeat_1_0 data = {
        .uptime = HAL_GetTick() / 1024,
        .health = {uavcan_node_Health_1_0_NOMINAL},
        .mode = {uavcan_node_Mode_1_0_OPERATIONAL},
        .vendor_specific_status_code = 0};
    // @formatter:on
    constexpr size_t PAYLOAD_SIZE = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t payload[PAYLOAD_SIZE];

    TaskWithPublication<Adapters...>::publish(PAYLOAD_SIZE, payload, &data,
                                              reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(uavcan_node_Heartbeat_1_0_serialize_),
                                              uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
}

template <typename... Adapters>
CyphalSubscription TaskSendHeartBeat<Adapters...>::createSubscription()
{
    return CyphalSubscription{uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, uavcan_node_Heartbeat_1_0_EXTENT_BYTES_, CyphalTransferKindMessage};
}

template <typename... Adapters>
void TaskSendHeartBeat<Adapters...>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->publish(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, task);
}

template <typename... Adapters>
void TaskSendHeartBeat<Adapters...>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unpublish(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, task);
}

#endif /* INC_TASKSENDHEARTBEAT_HPP_ */