#ifndef __TASKPUSHWRITE_HPP_
#define __TASKPUSHWRITE_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "InputOutputStream.hpp"

#include "nunavut_assert.h"
#include "uavcan/file/Write_1_1.h"
#include "uavcan/file/Error_1_0.h"

#include <cstring>
#include <cstdint>

template <typename... Adapters>
class TaskPushWrite : public TaskForClient<CyphalBuffer8, Adapters...>
{
public:
    TaskPushWrite() = delete;

    TaskPushWrite(uint32_t interval,
                  uint32_t tick,
                  CyphalNodeID node_id,
                  CyphalTransferID transfer_id,
                  std::tuple<Adapters...>& adapters)
        : TaskForClient<CyphalBuffer8, Adapters...>(interval, tick, node_id, transfer_id, adapters)
    {}

    virtual void registerTask(RegistrationManager* manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager* manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;

    virtual void update(uint32_t now) override;

protected:
    bool request();
    bool respond();

    // Fill provided buffer with formatted name
    void formatValues(char (&out)[19], uint64_t u64, uint8_t u8);

private:
    static constexpr size_t NAME_LENGTH = 19; // 16 + 1 + 2

    uavcan_file_Write_Request_1_1 data;
    static constexpr size_t PAYLOAD_SIZE = uavcan_file_Write_Request_1_1_EXTENT_BYTES_;
    uint8_t payload[PAYLOAD_SIZE];
};

template <typename... Adapters>
void TaskPushWrite<Adapters...>::handleTaskImpl()
{
    (void)request();
    (void)respond();
}

template <typename... Adapters>
void TaskPushWrite<Adapters...>::formatValues(char (&result)[19], uint64_t u64, uint8_t u8)
{
    static constexpr size_t NAME_LENGTH = 19;
    static constexpr char hex_digits[] = "0123456789abcdef";

    // 16 hex chars for u64
    for (size_t i = 0; i < 16; ++i)
    {
        result[15 - i] = hex_digits[u64 & 0x0f];
        u64 >>= 4;
    }

    // separator
    result[16] = '_';

    // 2 hex chars for u8
    for (size_t i = 0; i < 2; ++i)
    {
        result[18 - i] = hex_digits[u8 & 0x0f];
        u8 >>= 4;
    }

    static_assert((2 * sizeof(u64) + 2 * sizeof(u8) + 1) == NAME_LENGTH,
                  "formatValues, invalid NAME_LENGTH");
    static_assert((2 * sizeof(u64) + 2 * sizeof(u8) + 1) == sizeof(result),
                  "formatValues, invalid result length");
}

template <typename... Adapters>
bool TaskPushWrite<Adapters...>::request()
{
    log(LOG_LEVEL_DEBUG, "TaskPushWrite: request\r\n");

    char name[NAME_LENGTH] = {};
    formatValues(name, HAL_GetTick(), 0x12);

    data.path.path.count = NAME_LENGTH;
    std::memcpy(data.path.path.elements, name, NAME_LENGTH);

//    constexpr char buffer[] = "Example data to write via UAVCAN file Write service. Though there should be more, much more. We write until we cannot write anymore!!!!";
    char buffer[160];
    for(size_t i=0; i<sizeof(buffer); ++i) buffer[i] = i;
    data.data.value.count = sizeof(buffer);
    std::memcpy(data.data.value.elements, buffer, sizeof(buffer));

    TaskForClient<CyphalBuffer8, Adapters...>::publish(
        PAYLOAD_SIZE, payload, static_cast<void*>(&data),
        reinterpret_cast<int8_t (*)(const void* const, uint8_t* const, size_t* const)>(
            uavcan_file_Write_Request_1_1_serialize_),
        uavcan_file_Write_1_1_FIXED_PORT_ID_,
        TaskForClient<CyphalBuffer8, Adapters...>::node_id_);

    log(LOG_LEVEL_DEBUG, "TaskPushWrite: sent request with transfer_id %d\r\n", this->transfer_id_);
    this->transfer_id_ = (this->transfer_id_ + 1) & 0x1f;
    return true;
}

template <typename... Adapters>
bool TaskPushWrite<Adapters...>::respond()
{
    auto count = TaskForClient<CyphalBuffer8, Adapters...>::buffer_.size();
    for (size_t i = 0; i < count; ++i)
    {
        std::shared_ptr<CyphalTransfer> transfer = TaskForClient<CyphalBuffer8, Adapters...>::buffer_.pop();
        log(LOG_LEVEL_DEBUG, "TaskPushWrite: respond received transfer_id %d\r\n", transfer->metadata.transfer_id);
        (void)transfer; // TODO: inspect status/error if needed
    }
    return true;
}

template <typename... Adapters>
void TaskPushWrite<Adapters...>::registerTask(RegistrationManager* manager, std::shared_ptr<Task> task)
{
    manager->client(uavcan_file_Write_1_1_FIXED_PORT_ID_, task);
}

template <typename... Adapters>
void TaskPushWrite<Adapters...>::unregisterTask(RegistrationManager* manager, std::shared_ptr<Task> task)
{
    manager->unclient(uavcan_file_Write_1_1_FIXED_PORT_ID_, task);
}

template <typename... Adapters>
void TaskPushWrite<Adapters...>::update(uint32_t now)
{
    Task::update(now);
}

#endif // __TASKPUSHWRITE_HPP_
