#ifndef __TASKRESPONDWRITE_HPP_
#define __TASKRESPONDWRITE_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "InputOutputStream.hpp"

#include "nunavut_assert.h"
#include "uavcan/file/Write_1_1.h"
#include "uavcan/file/Error_1_0.h"

#include <cstring>

template <OutputStreamConcept Stream, typename... Adapters>
class TaskRespondWrite : public TaskForServer<CyphalBuffer8, Adapters...>
{
public:
    TaskRespondWrite() = delete;
    TaskRespondWrite(Stream &stream, uint32_t interval, uint32_t tick, std::tuple<Adapters...> &adapters) : 
    TaskForServer<CyphalBuffer8, Adapters...>(interval, tick, adapters), stream_(stream) {}

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;

private:
    Stream stream_;

};

template <OutputStreamConcept Stream, typename... Adapters>
void TaskRespondWrite<Stream, Adapters...>::handleTaskImpl()
{
    if (TaskForServer<CyphalBuffer8, Adapters...>::buffer_.is_empty())
    {
        log(LOG_LEVEL_TRACE, "TaskRespondWrite: empty buffer\r\n");
        return;
    }

    log(LOG_LEVEL_INFO, "TaskRespondWrite: received request\r\n");
    size_t count = TaskForServer<CyphalBuffer8, Adapters...>::buffer_.size();
    for (size_t i = 0; i < count; ++i)
    {
        std::shared_ptr<CyphalTransfer> transfer = TaskForServer<CyphalBuffer8, Adapters...>::buffer_.pop();
        if (transfer->metadata.transfer_kind != CyphalTransferKindRequest)
            continue;

        uavcan_file_Write_Request_1_1 request;
        size_t payload_size = transfer->payload_size;
        int8_t deserialization_result = uavcan_file_Write_Request_1_1_deserialize_(&request, static_cast<const uint8_t *>(transfer->payload), &payload_size);
        if (deserialization_result < 0)
        {
            log(LOG_LEVEL_ERROR, "TaskRequestGetInfo: Deserialization Error\r\n");
        }
        stream_.output(request.data.value.elements, request.data.value.count);

        uavcan_file_Write_Response_1_1 response;
        response._error.value = (deserialization_result < 0) ? uavcan_file_Error_1_0_IO_ERROR : uavcan_file_Error_1_0_OK;
        
        constexpr size_t PAYLOAD_SIZE = uavcan_file_Write_Response_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
        uint8_t payload[PAYLOAD_SIZE];

        TaskForServer<CyphalBuffer8, Adapters...>::publish(PAYLOAD_SIZE, payload, &response,
                                            reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(uavcan_file_Write_Response_1_1_serialize_),
                                            uavcan_file_Write_1_1_FIXED_PORT_ID_, transfer->metadata.remote_node_id, transfer->metadata.transfer_id);
        log(LOG_LEVEL_INFO, "TaskRespondWrite: sent response\r\n");
    }
}

template <OutputStreamConcept Stream, typename... Adapters>
void TaskRespondWrite<Stream, Adapters...>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->server(uavcan_file_Write_1_1_FIXED_PORT_ID_, task);
}

template <OutputStreamConcept Stream, typename... Adapters>
void TaskRespondWrite<Stream, Adapters...>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unserver(uavcan_file_Write_1_1_FIXED_PORT_ID_, task);
}

#endif // __TASKRESPONDWRITE_HPP_