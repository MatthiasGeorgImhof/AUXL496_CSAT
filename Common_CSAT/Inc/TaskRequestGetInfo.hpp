#ifndef INC_TASKREQUESTGETINFO_HPP_
#define INC_TASKREQUESTGETINFO_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "Logger.hpp"

#include "nunavut_assert.h"
#include "uavcan/node/GetInfo_1_0.h"
#include <cstring>

template <typename... Adapters>
class TaskRequestGetInfo : public TaskForClient<Adapters...>
{
public:
    TaskRequestGetInfo() = delete;
    TaskRequestGetInfo(uint32_t interval, uint32_t tick, CyphalNodeID node_id, CyphalTransferID transfer_id, std::tuple<Adapters...> &adapters) : TaskForClient<Adapters...>(interval, tick, node_id, transfer_id, adapters) {}

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;
};

template <typename T>
consteval T consteval_max(const T& a, const T& b) {
    return (a > b) ? a : b;
}

template <typename... Adapters>
void TaskRequestGetInfo<Adapters...>::handleTaskImpl()
{
    if (TaskForClient<Adapters...>::buffer_.is_empty())
    {
        uavcan_node_GetInfo_Request_1_0 data = {};
        constexpr size_t PAYLOAD_SIZE = consteval_max<size_t>(uavcan_node_GetInfo_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_, size_t(1));
        uint8_t payload[PAYLOAD_SIZE];

        TaskForClient<Adapters...>::publish(PAYLOAD_SIZE, payload, &data,
                                            reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(uavcan_node_GetInfo_Request_1_0_serialize_),
                                            uavcan_node_GetInfo_1_0_FIXED_PORT_ID_, TaskRequestGetInfo<Adapters...>::node_id_);
        log(LOG_LEVEL_DEBUG, "TaskRequestGetInfo: sent request\r\n");

        return;
    }

    size_t count = TaskForClient<Adapters...>::buffer_.size();
    for (size_t i = 0; i < count; ++i)
    {
        std::shared_ptr<CyphalTransfer> transfer = TaskForClient<Adapters...>::buffer_.pop();
        if (transfer->metadata.remote_node_id != TaskRequestGetInfo<Adapters...>::node_id_ ||
        	transfer->metadata.transfer_kind != CyphalTransferKindResponse	)
        {
            log(LOG_LEVEL_ERROR, "TaskRequestGetInfo Error: %4d %4d %4d\r\n",
            		TaskRequestGetInfo<Adapters...>::node_id_,
					transfer->metadata.remote_node_id,
					transfer->metadata.transfer_kind);
            return;
        }
        
        uavcan_node_GetInfo_Response_1_0 data;
        size_t payload_size = transfer->payload_size;

        int8_t deserialization_result = uavcan_node_GetInfo_Response_1_0_deserialize_(&data, static_cast<const uint8_t *>(transfer->payload), &payload_size);
        if (deserialization_result < 0)
        {
            log(LOG_LEVEL_ERROR, "TaskRequestGetInfo: Deserialization Error\r\n");
            return;
        }

        log(LOG_LEVEL_DEBUG, "TaskRequestGetInfo: received response from %4d\r\n", transfer->metadata.remote_node_id);
    }
}

template <typename... Adapters>
void TaskRequestGetInfo<Adapters...>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->client(uavcan_node_GetInfo_1_0_FIXED_PORT_ID_, task);
}

template <typename... Adapters>
void TaskRequestGetInfo<Adapters...>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unclient(uavcan_node_GetInfo_1_0_FIXED_PORT_ID_, task);
}

#endif /* INC_TASKREQUESTGETINFO_HPP_ */
