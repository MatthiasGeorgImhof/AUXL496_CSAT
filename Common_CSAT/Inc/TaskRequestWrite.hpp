#ifndef __TASKREQUESTWRITE_HPP_
#define __TASKREQUESTWRITE_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "InputOutputStream.hpp"

#include "nunavut_assert.h"
#include "uavcan/file/Write_1_1.h"
#include "uavcan/file/Error_1_0.h"

#include <cstring>

template <InputStreamConcept InputStream, typename... Adapters>
class TaskRequestWrite : public TaskForClient<CyphalBuffer8, Adapters...>, private TaskPacing
{
public:
    enum State
    {
        IDLE = 0,
        SEND_INIT = 1,       // Initial state: Nothing is initialized
        WAIT_INIT = 2,       // waiting for initialization ok
        RESEND_INIT = 3,     // if init failed, there might be retry, also might be the start of a stream
        SEND_TRANSFER = 4,   // Ready to pull a chunk from stream
        WAIT_TRANSFER = 5,   // Sent a chunk, awaiting an OK
        RESEND_TRANSFER = 6, // chunk delivery failed and we need to send the same again
        SEND_DONE = 7,       // stream is emtpy, need to signal we are done
        WAIT_DONE = 8,       // Waiting for the final OK to complete the transmission
        RESEND_DONE = 9      // We have failed and might need to send this over and over
    };

public:
    TaskRequestWrite() = delete;
    TaskRequestWrite(InputStream &stream,
                     uint32_t sleep_interval,
                     uint32_t operate_interval,
                     uint32_t tick,
                     CyphalNodeID node_id,
                     CyphalTransferID transfer_id,
                     std::tuple<Adapters...> &adapters)
        : TaskForClient<CyphalBuffer8, Adapters...>(sleep_interval, tick, node_id, transfer_id, adapters),
          TaskPacing(sleep_interval, operate_interval),
          stream_(stream)
        , state_(IDLE)
        , total_size_(0)
        , offset_(0)
        , name_{}
        , data_(nullptr)
    {
    }

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;

protected:
    bool request();
    bool respond();
    void reset();

protected:
    InputStream &stream_;
    State state_;
    size_t total_size_;
    size_t offset_;
    std::array<char, NAME_LENGTH> name_;
    std::unique_ptr<uavcan_file_Write_Request_1_1> data_;
};

template <InputStreamConcept InputStream, typename... Adapters>
void TaskRequestWrite<InputStream, Adapters...>::reset()
{
    state_ = IDLE;
    total_size_ = 0;
    offset_ = 0;

    name_ = {};
    data_ = nullptr;
    TaskPacing::sleep(*this);
}

template <InputStreamConcept InputStream, typename... Adapters>
void TaskRequestWrite<InputStream, Adapters...>::handleTaskImpl()
{
    (void)respond();
    (void)request();
}

template <InputStreamConcept InputStream, typename... Adapters>
bool TaskRequestWrite<InputStream, Adapters...>::respond()
{
    log(LOG_LEVEL_DEBUG, "TaskRequestWrite: respond in state %d\r\n", state_);

    if (TaskForClient<CyphalBuffer8, Adapters...>::buffer_.is_empty())
        return false;

    std::shared_ptr<CyphalTransfer> transfer = TaskForClient<CyphalBuffer8, Adapters...>::buffer_.pop();
    if (transfer->metadata.transfer_kind != CyphalTransferKindResponse)
    {
        log(LOG_LEVEL_ERROR, "TaskRequestWrite: Expected Response transfer kind\r\n");
        return false;
    }

    uavcan_file_Write_Response_1_1 data;
    size_t payload_size = transfer->payload_size;

    int8_t deserialization_result =
        uavcan_file_Write_Response_1_1_deserialize_(&data,
                                                     static_cast<const uint8_t *>(transfer->payload),
                                                     &payload_size);
    if (deserialization_result < 0)
    {
        log(LOG_LEVEL_ERROR, "TaskRequestWrite: Deserialization Error\r\n");
        return false;
    }

    log(LOG_LEVEL_DEBUG, "TaskRequestWrite: received response %d in state %d\r\n", data._error, state_);

    if (data._error.value == uavcan_file_Error_1_0_OK)
    {
        if (state_ == WAIT_INIT)
        {
            state_ = SEND_TRANSFER;
        }
        else if (state_ == WAIT_TRANSFER)
        {
            state_ = SEND_TRANSFER;
        }
        else if (state_ == WAIT_DONE)
        {
            stream_.finalize();
            reset();
        }
    }
    else
    {
        if (state_ == WAIT_INIT)
        {
            state_ = RESEND_INIT;
        }
        if (state_ == WAIT_TRANSFER)
        {
            state_ = RESEND_TRANSFER;
        }
        else if (state_ == WAIT_DONE)
        {
            state_ = RESEND_DONE;
        }
    }

    return true;
}

template <InputStreamConcept InputStream, typename... Adapters>
bool TaskRequestWrite<InputStream, Adapters...>::request()
{
    log(LOG_LEVEL_DEBUG, "TaskRequestWrite: request in state %d\r\n", state_);

    if (state_ == WAIT_INIT || state_ == WAIT_TRANSFER || state_ == WAIT_DONE)
        return false;

    if (!TaskForClient<CyphalBuffer8, Adapters...>::buffer_.is_empty()) // Should have been emtpied by respond
        return false;

    if (!stream_.is_empty() && state_ == IDLE)
    {
        state_ = SEND_INIT;
        log(LOG_LEVEL_DEBUG, "TaskRequestWrite: data available\r\n");
        TaskPacing::operate(*this);
    }

    if (!data_)
    {
        data_ = std::make_unique<uavcan_file_Write_Request_1_1>();
    }

    log(LOG_LEVEL_DEBUG, "TaskRequestWrite: request in state: %d\r\n", state_);
    switch (state_)
    {
    case SEND_INIT:
    {
        size_t current_size{0};
        stream_.initialize(data_->data.value.elements, current_size);
        name_ = stream_.name();
        total_size_ = stream_.size();

        data_->offset = offset_;
        data_->data.value.count = current_size;
        data_->path.path.count = NAME_LENGTH;
        std::memcpy(data_->path.path.elements, name_.data(), NAME_LENGTH);
        offset_ += current_size;
    }
    [[fallthrough]];
    case RESEND_INIT:
        state_ = WAIT_INIT;
        break;

    case SEND_TRANSFER:
    {
        size_t current_size{uavcan_primitive_Unstructured_1_0_value_ARRAY_CAPACITY_};
        stream_.getChunk(data_->data.value.elements, current_size);
        if (current_size == 0) 
        { 
            state_ = SEND_DONE; 
            goto send_done_label; 
        }

    	data_->offset = offset_;
    	data_->data.value.count = current_size;
        data_->path.path.count = NAME_LENGTH;
        std::memcpy(data_->path.path.elements, name_.data(), NAME_LENGTH);
        offset_ += current_size;
    }
    [[fallthrough]];
    case RESEND_TRANSFER:
        state_ = WAIT_TRANSFER;
        break;

    send_done_label:
    case SEND_DONE:
        data_->offset = offset_;
        data_->data.value.count = 0;
        data_->path.path.count = NAME_LENGTH;
        std::memcpy(data_->path.path.elements, name_.data(), NAME_LENGTH);
        state_ = WAIT_DONE;
        break;

    case RESEND_DONE:
        // Re-send the same zero-size packet
        state_ = WAIT_DONE;
        break;

    default:
        return false; // catch it all
    }

    constexpr size_t PAYLOAD_SIZE = uavcan_file_Write_Request_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t payload[PAYLOAD_SIZE];
    TaskForClient<CyphalBuffer8, Adapters...>::publish(
        PAYLOAD_SIZE,
        payload,
        data_.get(),
        reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(
            uavcan_file_Write_Request_1_1_serialize_),
        uavcan_file_Write_1_1_FIXED_PORT_ID_,
        TaskForClient<CyphalBuffer8, Adapters...>::node_id_);
    log(LOG_LEVEL_DEBUG, "TaskRequestWrite: sent request\r\n");
    return true;
}

template <InputStreamConcept InputStream, typename... Adapters>
void TaskRequestWrite<InputStream, Adapters...>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->client(uavcan_file_Write_1_1_FIXED_PORT_ID_, task);
}

template <InputStreamConcept InputStream, typename... Adapters>
void TaskRequestWrite<InputStream, Adapters...>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unclient(uavcan_file_Write_1_1_FIXED_PORT_ID_, task);
}

#endif // __TASKREQUESTWRITE_HPP_
