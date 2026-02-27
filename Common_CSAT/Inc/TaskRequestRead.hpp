#ifndef __TASKREQUESTREAD_HPP_
#define __TASKREQUESTREAD_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "InputOutputStream.hpp"
#include "FileSource.hpp"
#include "heapallocation.hpp"

#include "nunavut_assert.h"
#include "uavcan/file/Read_1_1.h"
#include "uavcan/file/Error_1_0.h"

template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
class TaskRequestRead : public TaskForClient<CyphalBuffer8, Adapters...>, private TaskPacing
{
public:
    enum State
    {
        IDLE,
        SEND_REQUEST,
        RESEND_REQUEST,
        WAIT_RESPONSE,
    };

    struct ReadState
    {
        State state;
        size_t offset;
        CyphalTransferID last_transfer_id;
    };

    enum TransferIDState
    {
        OK = 0,
        STALE = 1,
        FUTURE = 2
    };

public:
    TaskRequestRead() = delete;
    TaskRequestRead(FileSource &source, OutputStream &output, uint32_t sleep_interval, uint32_t operate_interval,
                    uint32_t tick, CyphalNodeID node_id, CyphalTransferID transfer_id, std::tuple<Adapters...> &adapters)
        : TaskForClient<CyphalBuffer8, Adapters...>(sleep_interval, tick, node_id, transfer_id, adapters), TaskPacing(sleep_interval, operate_interval),
          source_(source), output_(output), read_state_(IDLE, 0, 0), values_{}
    {
    }

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;

    virtual void update(uint32_t now) override;

protected:
    bool no_response_available() const;
    TransferIDState validate_transfer_id(const std::shared_ptr<CyphalTransfer> &t) const;
    bool validate_state_for_response() const;
    bool validate_response(const std::shared_ptr<CyphalTransfer> &t);

    bool request();
    bool respond();

    void reset();
    bool reset_and_fail();

    template <typename T, typename... Args>
    auto make_on_local_heap(Args &&...args)
    {
        static SafeAllocator<T, LocalHeap> alloc;
        return alloc_unique_custom<T, LocalHeap>(alloc, std::forward<Args>(args)...);
    }

protected:
    using ValueBuffer = std::array<uint8_t, uavcan_primitive_Unstructured_1_0_value_ARRAY_CAPACITY_>;
    using ValueAlloc = SafeAllocator<ValueBuffer, LocalHeap>;
    using ValuePtr = std::unique_ptr<ValueBuffer, ValueAlloc::Deletor>;

protected:
    FileSource &source_;
    OutputStream &output_;
    ReadState read_state_;
    ValuePtr values_;
};

template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
bool TaskRequestRead<FileSource, OutputStream, Adapters...>::no_response_available() const
{
    return TaskForClient<CyphalBuffer8, Adapters...>::buffer_.is_empty();
}

template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
TaskRequestRead<FileSource, OutputStream, Adapters...>::TransferIDState TaskRequestRead<FileSource, OutputStream, Adapters...>::validate_transfer_id(const std::shared_ptr<CyphalTransfer> &t) const
{
    const uint8_t expected = read_state_.last_transfer_id;
    const uint8_t received = t->metadata.transfer_id;
    const uint8_t delta = (received - expected) & 31; // 5-bit cyclic distance

    if (delta == 0)
    {
        // Correct TID
        return TransferIDState::OK;
    }

    if (delta < 16)
    {
        // FUTURE TID → server jumped ahead → fatal
        log(LOG_LEVEL_ERROR, "TaskRequestRead: FUTURE transfer-ID: expected %d, got %d (delta=%d)\r\n", expected, received, delta);
        return TransferIDState::FUTURE; // fatal
    }

    // OLD TID → stale/duplicate → ignore
    log(LOG_LEVEL_DEBUG, "TaskRequestRead: stale transfer-ID: expected %d, got %d (delta=%d)\r\n", expected, received, delta);

    // Caller must IGNORE this response
    return TransferIDState::STALE;
}

template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
bool TaskRequestRead<FileSource, OutputStream, Adapters...>::validate_response(const std::shared_ptr<CyphalTransfer> &t)
{
    // Only accept CyphalTransferKindResponse
    if (t->metadata.transfer_kind != CyphalTransferKindResponse)
    {
        log(LOG_LEVEL_DEBUG, "TaskRequestRead: ignoring non-response transfer kind %d\r\n", t->metadata.transfer_kind);
        return false; // discard silently
    }

    // Only accept responses from our server
    if (t->metadata.remote_node_id != TaskForClient<CyphalBuffer8, Adapters...>::node_id_)
    {
        log(LOG_LEVEL_DEBUG, "TaskRequestRead: ignoring response from node %d\r\n", t->metadata.remote_node_id);
        return false;
    }

    return true;
}

template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
bool TaskRequestRead<FileSource, OutputStream, Adapters...>::validate_state_for_response() const
{
    switch (read_state_.state)
    {
    case WAIT_RESPONSE:
        return true;
    default:
        log(LOG_LEVEL_ERROR, "TaskRequestRead: Response received in invalid state %d\r\n", read_state_.state);
        return false;
    }
}

template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
void TaskRequestRead<FileSource, OutputStream, Adapters...>::reset()
{
    while (!TaskForClient<CyphalBuffer8, Adapters...>::buffer_.is_empty())
    {
        TaskForClient<CyphalBuffer8, Adapters...>::buffer_.pop();
    }
    read_state_ = ReadState{IDLE, 0, 0};
    this->transfer_id_ = wrap_transfer_id(this->transfer_id_ + 1);
    TaskPacing::sleep(*this);

    log(LOG_LEVEL_WARNING, "TaskRequestRead: reset, transfer_id  %d -> %d\r\n", read_state_.last_transfer_id, this->transfer_id_);
}

template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
bool TaskRequestRead<FileSource, OutputStream, Adapters...>::reset_and_fail()
{
    reset();
    return false;
}

template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
void TaskRequestRead<FileSource, OutputStream, Adapters...>::handleTaskImpl()
{
    if (values_ == nullptr)
    {
        values_ = make_on_local_heap<ValueBuffer>();
    }

    (void)respond();
    (void)request();
}

template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
bool TaskRequestRead<FileSource, OutputStream, Adapters...>::respond()
{
    if (no_response_available())
    {
        log(LOG_LEVEL_INFO, "TaskRequestRead: respond() no response available, state=%d offset=%u\r\n",
            read_state_.state, static_cast<unsigned>(read_state_.offset));
        return true;
    }

    while (!no_response_available())
    {
        auto transfer = TaskForClient<CyphalBuffer8, Adapters...>::buffer_.pop();
        log(LOG_LEVEL_DEBUG,
            "TaskRequestRead: respond() got transfer: tid=%u kind=%u size=%u\r\n",
            transfer->metadata.transfer_id,
            transfer->metadata.transfer_kind,
            transfer->payload_size);

        // Ignore unrelated messages (wrong kind, wrong port-ID)
        if (!validate_response(transfer))
            continue;

        auto tid_state = validate_transfer_id(transfer);
        switch (tid_state)
        {
        case TransferIDState::OK:
            break;
        case TransferIDState::STALE:
            continue;
        case TransferIDState::FUTURE:
            return reset_and_fail();
        default:
            break;
        }

        if (!validate_state_for_response())
            return reset_and_fail();

        uavcan_file_Read_Response_1_1 response_data;
        size_t payload_size = transfer->payload_size;
        int8_t res = uavcan_file_Read_Response_1_1_deserialize_(
            &response_data,
            static_cast<const uint8_t *>(transfer->payload),
            &payload_size);

        if (res < 0)
        {
            log(LOG_LEVEL_ERROR, "TaskRequestRead: deserialization error res=%d\r\n", res);
            read_state_.state = RESEND_REQUEST;
            return false;
        }

        log(LOG_LEVEL_DEBUG,
            "TaskRequestRead: response OK, error=%d count=%u\r\n",
            response_data._error.value,
            response_data.data.value.count);

        if (response_data._error.value != uavcan_file_Error_1_0_OK)
        {
            log(LOG_LEVEL_ERROR, "TaskRequestRead: server error=%d\r\n", response_data._error.value);
            read_state_.state = RESEND_REQUEST;
            return false;
        }

        // Log chunk data in hex
        {
            constexpr size_t BUF = 1024;
            char hexbuf[BUF];
            uchar_buffer_to_hex(
                reinterpret_cast<const unsigned char *>(response_data.data.value.elements),
                response_data.data.value.count,
                hexbuf,
                BUF);

            log(LOG_LEVEL_INFO,
                "TaskRequestRead: chunk size=%u data=%s\r\n",
                response_data.data.value.count,
                hexbuf);
        }

        if (!output_.output(response_data.data.value.elements, response_data.data.value.count))
        {
            log(LOG_LEVEL_ERROR, "TaskRequestRead: OutputStream error\r\n");
            read_state_.state = RESEND_REQUEST;
            return false;
        }

        if (response_data.data.value.count == 0)
        {
            log(LOG_LEVEL_INFO, "TaskRequestRead: EOF reached, finalizing\r\n");
            output_.finalize();
            reset();
            return true;
        }

        log(LOG_LEVEL_DEBUG,
            "TaskRequestRead: advancing offset %u -> %u\r\n",
            static_cast<unsigned>(read_state_.offset),
            static_cast<unsigned>(read_state_.offset + response_data.data.value.count));

        read_state_.offset += response_data.data.value.count;
        read_state_.state = SEND_REQUEST;
        return true;
    }
    return true;
}

template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
bool TaskRequestRead<FileSource, OutputStream, Adapters...>::request()
{
    log(LOG_LEVEL_DEBUG,
        "TaskRequestRead: request() state=%d offset=%u buffer_empty=%d tid=%u\r\n",
        read_state_.state,
        static_cast<unsigned>(read_state_.offset),
        TaskForClient<CyphalBuffer8, Adapters...>::buffer_.is_empty(),
        this->transfer_id_);

    if (!TaskForClient<CyphalBuffer8, Adapters...>::buffer_.is_empty())
    {
        log(LOG_LEVEL_INFO, "TaskRequestRead: request() RX buffer not empty, skipping\r\n");
        return true;
    }

    auto request_data = make_on_local_heap<uavcan_file_Read_Request_1_1>();
    switch (read_state_.state)
    {
    case IDLE:
        log(LOG_LEVEL_DEBUG, "TaskRequestRead: IDLE -> SEND_REQUEST\r\n");
        read_state_.state = SEND_REQUEST;
        TaskPacing::operate(*this);
        [[fallthrough]];
    case SEND_REQUEST:
        [[fallthrough]];
    case RESEND_REQUEST:
        request_data->offset = read_state_.offset;
        request_data->path.path.count = source_.getPathLength();
        std::memcpy(request_data->path.path.elements, source_.getPath().data(), source_.getPathLength());
        break;
    case WAIT_RESPONSE:
        return true; // waiting for response
    default:
        log(LOG_LEVEL_ERROR, "TaskRequestRead: request() unknown state %d\r\n", read_state_.state);
        return false;
    }

    read_state_.last_transfer_id = wrap_transfer_id(this->transfer_id_);
    constexpr size_t PAYLOAD_SIZE = uavcan_file_Read_Request_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t payload[PAYLOAD_SIZE];
    TaskForClient<CyphalBuffer8, Adapters...>::publish(PAYLOAD_SIZE, payload, request_data.get(),
                                                       reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(uavcan_file_Read_Request_1_1_serialize_),
                                                       uavcan_file_Read_1_1_FIXED_PORT_ID_, TaskForClient<CyphalBuffer8, Adapters...>::node_id_);
    log(LOG_LEVEL_DEBUG, "TaskRequestRead: Sent request for offset %d, path '%.*s' of length %d\r\n", request_data->offset,
        request_data->path.path.count, request_data->path.path.elements, request_data->path.path.count);

    this->transfer_id_ = wrap_transfer_id(this->transfer_id_ + 1);
    read_state_.state = WAIT_RESPONSE;

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

template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
void TaskRequestRead<FileSource, OutputStream, Adapters...>::update(uint32_t now)
{
    Task::update(now);
}

#endif // __TASKREQUESTREAD_HPP_
