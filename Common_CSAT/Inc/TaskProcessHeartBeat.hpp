/*
 * TaskSendHeartBeat.hpp
 *
 *  Created on: Dec 4, 2024
 *      Author: mgi
 */

#ifndef INC_TASKPROCESSHEARTBEAT_HPP_
#define INC_TASKPROCESSHEARTBEAT_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "Logger.hpp"

#include "nunavut_assert.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "nunavut/support/serialization.h"


template <typename... Adapters>
class TaskProcessHeartBeat : public TaskFromBuffer
{
public:
    TaskProcessHeartBeat() = delete;
    TaskProcessHeartBeat(uint32_t interval, uint32_t tick, std::tuple<Adapters...>& adapters) : TaskFromBuffer(interval, tick), adapters_(adapters) {}

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;

private:
    std::tuple<Adapters...>& adapters_;
};

template <typename... Adapters>
void TaskProcessHeartBeat<Adapters...>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->subscribe(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, task);
}

template <typename... Adapters>
void TaskProcessHeartBeat<Adapters...>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unsubscribe(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, task);
}

template <typename... Adapters>
void TaskProcessHeartBeat<Adapters...>::handleTaskImpl()
{
    // Process all transfers in the buffer
	size_t count = buffer_.size();
    for (size_t i = 0; i < count; ++i)
    {
        std::shared_ptr<CyphalTransfer> transfer = buffer_.pop();
        size_t payload_size = transfer->payload_size;
        uavcan_node_Heartbeat_1_0 heart_beat;
        uavcan_node_Heartbeat_1_0_deserialize_(&heart_beat, (const uint8_t *) transfer->payload, &payload_size);
		log(LOG_LEVEL_DEBUG, "TaskProcessHeartBeat %d: %d\r\n", transfer->metadata.remote_node_id, heart_beat.uptime);
    }
}

static_assert(containsMessageByPortIdCompileTime<uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_>(),
              "uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_ must be in CYPHAL_MESSAGES");

#endif /* INC_TASKPROCESSHEARTBEAT_HPP_ */
