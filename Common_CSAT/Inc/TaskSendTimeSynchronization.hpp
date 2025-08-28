#ifndef INC_TASKSENDTIMESYNCHRONIZATION_HPP_
#define INC_TASKSENDTIMESYNCHRONIZATION_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "TimeUtils.hpp"

#ifdef __arm__
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "utilities.h"
#elif __x86_64__
#include "mock_hal.h"
#endif

#include "nunavut_assert.h"
#include "uavcan/time/Synchronization_1_0.h"

template <typename... Adapters>
class TaskSendTimeSynchronization : public TaskWithPublication<Adapters...>
{
public:
    TaskSendTimeSynchronization() = delete;
    TaskSendTimeSynchronization(RTC_HandleTypeDef *hrtc, uint32_t interval, uint32_t tick, CyphalTransferID transfer_id, std::tuple<Adapters...> &adapters) : TaskWithPublication<Adapters...>(interval, tick, transfer_id, adapters), hrtc_(hrtc), previous_microseconds_(0) {}

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;
    virtual void handleMessage(std::shared_ptr<CyphalTransfer> /*transfer*/) override {}

private:
    CyphalSubscription createSubscription();

private:
    RTC_HandleTypeDef *hrtc_;
    uint64_t previous_microseconds_;
};

template <typename... Adapters>
void TaskSendTimeSynchronization<Adapters...>::handleTaskImpl()
{
    struct TimeUtils::RTCDateTimeSubseconds rtc;
    HAL_RTC_GetTime(hrtc_, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc_, &rtc.date, RTC_FORMAT_BIN);

    uavcan_time_Synchronization_1_0 data = {
        .previous_transmission_timestamp_microsecond = previous_microseconds_};

    constexpr size_t PAYLOAD_SIZE = uavcan_time_Synchronization_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t payload[PAYLOAD_SIZE];

    TaskWithPublication<Adapters...>::publish(PAYLOAD_SIZE, payload, &data,
                                              reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(uavcan_time_Synchronization_1_0_serialize_),
                                              uavcan_time_Synchronization_1_0_FIXED_PORT_ID_);
    std::chrono::milliseconds milliseconds = TimeUtils::from_rtc(rtc, hrtc_->Init.SynchPrediv);
    previous_microseconds_ = 1000U * static_cast<uint64_t>(milliseconds.count());
}

template <typename... Adapters>
CyphalSubscription TaskSendTimeSynchronization<Adapters...>::createSubscription()
{
    return CyphalSubscription{uavcan_time_Synchronization_1_0_FIXED_PORT_ID_, uavcan_time_Synchronization_1_0_EXTENT_BYTES_, CyphalTransferKindMessage};
}

template <typename... Adapters>
void TaskSendTimeSynchronization<Adapters...>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->publish(uavcan_time_Synchronization_1_0_FIXED_PORT_ID_, task);
}

template <typename... Adapters>
void TaskSendTimeSynchronization<Adapters...>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unpublish(uavcan_time_Synchronization_1_0_FIXED_PORT_ID_, task);
}

#endif /* INC_TASKSENDTIMESYNCHRONIZATION_HPP_ */
