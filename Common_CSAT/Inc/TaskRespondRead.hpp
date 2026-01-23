#ifndef __TASKRESPONDREAD_HPP_
#define __TASKRESPONDREAD_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "InputOutputStream.hpp" // For NAME_LENGTH, Error, etc.
#include "FileAccess.hpp"
#include "nunavut_assert.h"
#include "uavcan/file/Read_1_1.h"
#include "uavcan/file/Error_1_0.h"


template <FileAccessConcept Accessor, typename... Adapters>
class TaskRespondRead : public TaskForServer<CyphalBuffer8, Adapters...>
{
public:
    TaskRespondRead() = delete;
    TaskRespondRead(Accessor& accessor, uint32_t interval, uint32_t tick, std::tuple<Adapters...> &adapters)
        : TaskForServer<CyphalBuffer8, Adapters...>(interval, tick, adapters), accessor_(accessor) {}

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;

protected:
    bool respond();

protected:
    Accessor& accessor_;
};

template <FileAccessConcept Accessor, typename... Adapters>
void TaskRespondRead<Accessor, Adapters...>::handleTaskImpl()
{
    (void)respond();
}

template <FileAccessConcept Accessor, typename... Adapters>
bool TaskRespondRead<Accessor, Adapters...>::respond()
{
    if (TaskForServer<CyphalBuffer8, Adapters...>::buffer_.is_empty())
        return true;

    std::shared_ptr<CyphalTransfer> transfer = TaskForServer<CyphalBuffer8, Adapters...>::buffer_.pop();
    if (transfer->metadata.transfer_kind != CyphalTransferKindRequest)
    {
        log(LOG_LEVEL_ERROR, "TaskRespondRead: Expected Request transfer kind\r\n");
        return false;
    }

    uavcan_file_Read_Request_1_1 request_data;
    size_t payload_size = transfer->payload_size;
    int8_t deserialization_result = uavcan_file_Read_Request_1_1_deserialize_(&request_data, static_cast<const uint8_t *>(transfer->payload), &payload_size);
    if (deserialization_result < 0)
    {
        log(LOG_LEVEL_ERROR, "TaskRespondRead: Deserialization Error\r\n");
        return false;
    }

    uavcan_file_Read_Response_1_1 response_data = {}; // Initialize

    // Read from the file
    size_t bytes_to_read = uavcan_primitive_Unstructured_1_0_value_ARRAY_CAPACITY_;
    bool read_ok = accessor_.read(convertPath(request_data.path.path.elements, request_data.path.path.count), request_data.offset, response_data.data.value.elements, bytes_to_read);

    if (read_ok)
    {
        response_data.data.value.count = bytes_to_read;
        response_data._error.value = uavcan_file_Error_1_0_OK;
    }
    else
    {
        response_data.data.value.count = 0; // No data
        response_data._error.value = uavcan_file_Error_1_0_IO_ERROR;
    }

    // Serialize and publish the response
    constexpr size_t PAYLOAD_SIZE = uavcan_file_Read_Response_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t payload[PAYLOAD_SIZE];
    TaskForServer<CyphalBuffer8, Adapters...>::publish(PAYLOAD_SIZE, payload, &response_data,
                                         reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(uavcan_file_Read_Response_1_1_serialize_),
                                         transfer->metadata.port_id, transfer->metadata.remote_node_id, transfer->metadata.transfer_id);

    log(LOG_LEVEL_DEBUG, "TaskRespondRead: Sent response for path '%s', offset %zu, size %zu\r\n", request_data.path.path.elements, request_data.offset, response_data.data.value.count);

    return true;
}

template <FileAccessConcept Accessor, typename... Adapters>
void TaskRespondRead<Accessor, Adapters...>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->server(uavcan_file_Read_1_1_FIXED_PORT_ID_, task);
}

template <FileAccessConcept Accessor, typename... Adapters>
void TaskRespondRead<Accessor, Adapters...>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unserver(uavcan_file_Read_1_1_FIXED_PORT_ID_, task);
}

#endif // __TASKRESPONDREAD_HPP_