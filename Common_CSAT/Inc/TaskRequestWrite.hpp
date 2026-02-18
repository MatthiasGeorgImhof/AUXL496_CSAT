#ifndef __TASKREQUESTWRITE_HPP_
#define __TASKREQUESTWRITE_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "InputOutputStream.hpp"
#include "heapallocation.hpp"

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

    constexpr static unsigned int MAX_CHUNK_SIZE = 256U;

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
		  stream_(stream),
		  state_(IDLE), total_size_(0), offset_(0), timeout_(0), last_transfer_id_{0}, name_{}, values_{}, num_values_(0)
    {
    }

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;

	virtual void update(uint32_t now) override;

protected:
    bool request();
    bool respond();
    void reset();

    template <typename T, typename... Args>
    auto make_on_local_heap(Args&&... args)
    {
        static SafeAllocator<T, LocalHeap> alloc;
        return alloc_unique_custom<T, LocalHeap>(alloc, std::forward<Args>(args)...);
    }

protected:
    using ValueBuffer = std::array<uint8_t, uavcan_primitive_Unstructured_1_0_value_ARRAY_CAPACITY_>;
    using ValueAlloc  = SafeAllocator<ValueBuffer, LocalHeap>;
    using ValuePtr    = std::unique_ptr<ValueBuffer, ValueAlloc::Deletor>;

protected:
    InputStream &stream_;
    State state_;
    size_t total_size_;
    size_t offset_;
    uint32_t timeout_;
    CyphalTransferID last_transfer_id_;
    std::array<char, NAME_LENGTH> name_;
    ValuePtr values_;
    size_t num_values_;
};

template <InputStreamConcept InputStream, typename... Adapters>
void TaskRequestWrite<InputStream, Adapters...>::reset()
{
    state_ = IDLE;
    total_size_ = 0;
    offset_ = 0;

    name_ = {};
    TaskPacing::sleep(*this);
}

template <InputStreamConcept InputStream, typename... Adapters>
void TaskRequestWrite<InputStream, Adapters...>::handleTaskImpl()
{
    if (values_ == nullptr)
    {
    	values_ = make_on_local_heap<ValueBuffer>();
    }
	(void)respond();
    (void)request();
}

template <InputStreamConcept InputStream, typename... Adapters>
bool TaskRequestWrite<InputStream, Adapters...>::respond()
{
    log(LOG_LEVEL_DEBUG, "TaskRequestWrite: respond in state %d\r\n", state_);

    if (TaskForClient<CyphalBuffer8, Adapters...>::buffer_.is_empty())
    {
    	if (HAL_GetTick() < timeout_)
    		return true;

    	switch(state_)
    	{
    	case WAIT_INIT:
    	{
    		state_ = RESEND_INIT;
    		break;
    	}
    	case WAIT_TRANSFER:
    	{
    		state_ = RESEND_TRANSFER;
            break;
    	}
        case WAIT_DONE:
        {
        	state_ = RESEND_DONE;
        	break;
        }
        default:
        {
    	}
    	}
        return false;
    }

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

    log(LOG_LEVEL_DEBUG, "TaskRequestWrite: received response %d in state %d for transfer_id %d \r\n", data._error, state_, transfer->metadata.transfer_id);

    if (data._error.value == uavcan_file_Error_1_0_OK)
    {
        switch(state_)
        {
        case WAIT_INIT:
            state_ = SEND_TRANSFER;
            break;
        case WAIT_TRANSFER:

            state_ = SEND_TRANSFER;
            break;

        case WAIT_DONE:
            stream_.finalize();
            reset();
            break;
        default:
        	return false;
        }
    }
    else
    {
    	switch(state_)
    	{
    	case WAIT_INIT:
        	state_ = RESEND_INIT;
    		break;
    	case WAIT_TRANSFER:
            state_ = RESEND_TRANSFER;
            break;
        case WAIT_DONE:
        	state_ = RESEND_DONE;
        	break;
        default:
        	return false;
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

    auto data = make_on_local_heap<uavcan_file_Write_Request_1_1>();

    log(LOG_LEVEL_DEBUG, "TaskRequestWrite: request in state %d\r\n", state_);
    switch (state_)
    {
    case SEND_INIT:
    {
        stream_.initialize(values_->data(), num_values_);
        name_ = stream_.name();
        total_size_ = stream_.size();

        data->offset = offset_;
        data->data.value.count = num_values_;
        data->path.path.count = NAME_LENGTH;
        std::memcpy(data->path.path.elements, name_.data(), NAME_LENGTH);
        std::memcpy(data->data.value.elements, values_->data(), num_values_);
        offset_ += num_values_;
        state_ = WAIT_INIT;
        break;
    }
    case RESEND_INIT:
        data->offset = offset_ - num_values_;
        data->data.value.count = num_values_;
        data->path.path.count = NAME_LENGTH;
        std::memcpy(data->path.path.elements, name_.data(), NAME_LENGTH);
        std::memcpy(data->data.value.elements, values_->data(), num_values_);
    	state_ = WAIT_INIT;
        break;

    case SEND_TRANSFER:
    {
    	num_values_ = std::min(MAX_CHUNK_SIZE, uavcan_primitive_Unstructured_1_0_value_ARRAY_CAPACITY_);
        stream_.getChunk(values_->data(), num_values_);
        if (num_values_ == 0)
        { 
            state_ = SEND_DONE; 
            goto send_done_label; 
        }

    	data->offset = offset_;
    	data->data.value.count = num_values_;
        data->path.path.count = NAME_LENGTH;
        std::memcpy(data->path.path.elements, name_.data(), NAME_LENGTH);
        std::memcpy(data->data.value.elements, values_->data(), num_values_);
        offset_ += num_values_;
        state_ = WAIT_TRANSFER;
        break;
    }
    case RESEND_TRANSFER:
    	data->offset = offset_ - num_values_;
    	data->data.value.count = num_values_;
        data->path.path.count = NAME_LENGTH;
        std::memcpy(data->path.path.elements, name_.data(), NAME_LENGTH);
        std::memcpy(data->data.value.elements, values_->data(), num_values_);
    	state_ = WAIT_TRANSFER;
        break;

    send_done_label:
    case SEND_DONE:
        data->offset = offset_;
        data->data.value.count = 0;
        data->path.path.count = NAME_LENGTH;
        std::memcpy(data->path.path.elements, name_.data(), NAME_LENGTH);
        state_ = WAIT_DONE;
        break;

    case RESEND_DONE:
        data->offset = offset_;
        data->data.value.count = 0;
        data->path.path.count = NAME_LENGTH;
        std::memcpy(data->path.path.elements, name_.data(), NAME_LENGTH);
    	state_ = WAIT_DONE;
        break;

    default:
        return false; // catch it all
    }

    last_transfer_id_ = this->transfer_id_;
    constexpr size_t PAYLOAD_SIZE = uavcan_file_Write_Request_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t payload[PAYLOAD_SIZE];
    TaskForClient<CyphalBuffer8, Adapters...>::publish(
        PAYLOAD_SIZE,
        payload,
        data.get(),
        reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(
            uavcan_file_Write_Request_1_1_serialize_),
        uavcan_file_Write_1_1_FIXED_PORT_ID_,
        TaskForClient<CyphalBuffer8, Adapters...>::node_id_);
    log(LOG_LEVEL_DEBUG, "TaskRequestWrite: sent request with %d bytes at offset %d and transfer_id %d\r\n", data->data.value.count, offset_- data->data.value.count, this->transfer_id_);
    timeout_ = HAL_GetTick() + 100 * Task::interval_;
    ++this->transfer_id_;
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

template <InputStreamConcept InputStream, typename... Adapters>
void TaskRequestWrite<InputStream, Adapters...>::update(uint32_t now)
{
	Task::update(now);
}


#endif // __TASKREQUESTWRITE_HPP_
