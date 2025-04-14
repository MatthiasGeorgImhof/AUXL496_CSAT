#ifndef INC_TASKRESPONDGETINFO_HPP_
#define INC_TASKRESPONDGETINFO_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "Logger.hpp"

#include "nunavut_assert.h"
#include "uavcan/node/GetInfo_1_0.h"
#include <cstring>

template <typename... Adapters>
class TaskRespondGetInfo : public TaskForServer<Adapters...>
{
public:
    TaskRespondGetInfo() = delete;
    TaskRespondGetInfo(uint8_t unique_id[16], char name[50], uint32_t interval, uint32_t tick, std::tuple<Adapters...> &adapters) : TaskForServer<Adapters...>(interval, tick, adapters),  unique_id_{}, name_{} {
        std::memcpy(unique_id_, unique_id, 16);
        std::memcpy(name_, name, 50);
    }

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;

protected:
    uint8_t unique_id_[16];
    char name_[50];
};

template <typename... Adapters>
void TaskRespondGetInfo<Adapters...>::handleTaskImpl()
{
    if (TaskForServer<Adapters...>::buffer.is_empty())
    {
        log(LOG_LEVEL_TRACE, "TaskRespondGetInfo: empty buffer\r\n");
        return;
    }

    size_t count = TaskForServer<Adapters...>::buffer.size();
    for (size_t i = 0; i < count; ++i)
    {
        std::shared_ptr<CyphalTransfer> transfer = TaskForServer<Adapters...>::buffer.pop();
        if (transfer->metadata.transfer_kind != CyphalTransferKindRequest)
            return;

        uavcan_node_GetInfo_Response_1_0 data = {
            .protocol_version = {uavcan_node_Version_1_0{.major = 1, .minor = 0}},
            .hardware_version = {uavcan_node_Version_1_0{.major = 1, .minor = 0}},
            .software_version = {uavcan_node_Version_1_0{.major = 1, .minor = 0}},
            .software_vcs_revision_id = 0xc5ad8c7dll,
            .unique_id = {}, 
            .name = {},
            .software_image_crc = {},
            .certificate_of_authenticity = {}
        };
        std::memcpy(data.unique_id, unique_id_, sizeof(data.unique_id));
        std::memcpy(data.name.elements, name_, sizeof(data.name.elements));
        data.name.count = std::strlen(name_);


        constexpr size_t PAYLOAD_SIZE = uavcan_node_GetInfo_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
        uint8_t payload[PAYLOAD_SIZE];

        TaskForServer<Adapters...>::publish(PAYLOAD_SIZE, payload, &data,
                                            reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(uavcan_node_GetInfo_Response_1_0_serialize_),
                                            uavcan_node_GetInfo_1_0_FIXED_PORT_ID_, transfer->metadata.remote_node_id, transfer->metadata.transfer_id);
        log(LOG_LEVEL_TRACE, "TaskRespondGetInfo: respond\r\n");
    }
}

template <typename... Adapters>
void TaskRespondGetInfo<Adapters...>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->subscribe(uavcan_node_GetInfo_1_0_FIXED_PORT_ID_, task);
}

template <typename... Adapters>
void TaskRespondGetInfo<Adapters...>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unsubscribe(uavcan_node_GetInfo_1_0_FIXED_PORT_ID_, task);
}

#endif /* INC_TASKRESPONDGETINFO_HPP_ */