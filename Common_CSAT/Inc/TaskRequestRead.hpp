#ifndef __TASKREQUESTREAD_HPP_
#define __TASKREQUESTREAD_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "InputOutputStream.hpp"
#include "FileSource.hpp"

#include "nunavut_assert.h"
#include "uavcan/file/Read_1_1.h"
#include "uavcan/file/Error_1_0.h"

template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
class TaskRequestRead : public TaskForClient<CyphalBuffer8, Adapters...>
{
public:
    enum State
    {
        IDLE,
        SEND_REQUEST,
        RESEND_REQUEST,
        WAIT_RESPONSE,
    };

public:
    TaskRequestRead() = delete;
    TaskRequestRead(FileSource &source, OutputStream &output, uint32_t interval, uint32_t tick, CyphalNodeID node_id, CyphalTransferID transfer_id, std::tuple<Adapters...> &adapters)
        : TaskForClient<CyphalBuffer8, Adapters...>(interval, tick, node_id, transfer_id, adapters),
          source_(source),
          output_(output),
          state_(IDLE)
    {
    }

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;

protected:
    bool request();
    bool respond();

protected:
    FileSource &source_;
    OutputStream &output_;
    State state_;
    std::unique_ptr<uavcan_file_Read_Request_1_1> request_data_;
};

template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
void TaskRequestRead<FileSource, OutputStream, Adapters...>::handleTaskImpl()
{
    (void)respond();
    (void)request();
}

template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
bool TaskRequestRead<FileSource, OutputStream, Adapters...>::respond()
{
    if (TaskForClient<CyphalBuffer8, Adapters...>::buffer_.is_empty())
        return true;

    std::shared_ptr<CyphalTransfer> transfer = TaskForClient<CyphalBuffer8, Adapters...>::buffer_.pop();
    if (transfer->metadata.transfer_kind != CyphalTransferKindResponse)
    {
        log(LOG_LEVEL_ERROR, "TaskRequestRead: Expected Response transfer kind\r\n");
        return false;
    }

    uavcan_file_Read_Response_1_1 response_data;
    size_t payload_size = transfer->payload_size;
    int8_t deserialization_result = uavcan_file_Read_Response_1_1_deserialize_(&response_data, static_cast<const uint8_t *>(transfer->payload), &payload_size);
    if (deserialization_result < 0)
    {
        log(LOG_LEVEL_ERROR, "TaskRequestRead: Deserialization Error\r\n");
        state_ = RESEND_REQUEST; // Or some other error state
        return false;
    }

    log(LOG_LEVEL_DEBUG, "TaskRequestRead: Received response\r\n");

    if (response_data._error.value != uavcan_file_Error_1_0_OK)
    {
        log(LOG_LEVEL_ERROR, "TaskRequestRead: Server returned error: %d\r\n", response_data._error.value);
        state_ = RESEND_REQUEST;
        return false;
    }

    if (!output_.output(response_data.data.value.elements, response_data.data.value.count))
    {
        log(LOG_LEVEL_ERROR, "TaskRequestRead: OutputStream returned error\r\n");
        state_ = RESEND_REQUEST; // Error
        return false;
    }

    if (response_data.data.value.count == 0)
    {
        log(LOG_LEVEL_INFO, "TaskRequestRead: End of file reached\r\n");
        state_ = IDLE;
        output_.finalize();
        return true;
    }

    state_ = SEND_REQUEST;
    source_.setOffset(source_.offset() + response_data.data.value.count);
    return true;
}

template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
bool TaskRequestRead<FileSource, OutputStream, Adapters...>::request()
{
    if (state_ == WAIT_RESPONSE)
        return true;

    if (!TaskForClient<CyphalBuffer8, Adapters...>::buffer_.is_empty())
        return true;

    if (state_ == IDLE)
    {
        state_ = SEND_REQUEST;
    }

    if (state_ == SEND_REQUEST)
    {
        request_data_ = std::make_unique<uavcan_file_Read_Request_1_1>();
        request_data_->offset = source_.offset();
        std::array<char, NAME_LENGTH> path = source_.getPath();
        request_data_->path.path.count = path.size();
        std::memcpy(request_data_->path.path.elements, path.data(), path.size());
    }

    constexpr size_t PAYLOAD_SIZE = uavcan_file_Read_Request_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t payload[PAYLOAD_SIZE];
    TaskForClient<CyphalBuffer8, Adapters...>::publish(PAYLOAD_SIZE, payload, request_data_.get(),
                                        reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(uavcan_file_Read_Request_1_1_serialize_),
                                        uavcan_file_Read_1_1_FIXED_PORT_ID_, TaskForClient<CyphalBuffer8, Adapters...>::node_id_);
    log(LOG_LEVEL_DEBUG, "TaskRequestRead: Sent request for offset %zu, path '%s'\r\n", request_data_->offset, request_data_->path.path.elements);

    state_ = WAIT_RESPONSE;
    return true;
}

template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
void TaskRequestRead<FileSource, OutputStream, Adapters...>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->client(uavcan_file_Read_1_1_FIXED_PORT_ID_, task);
}

template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
void TaskRequestRead<FileSource, OutputStream, Adapters...>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unclient(uavcan_file_Read_1_1_FIXED_PORT_ID_, task);
}

#endif // __TASKREQUESTREAD_HPP_