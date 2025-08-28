#ifndef INC_TASKPROCESSTIMESYNCHRONIZATION_HPP_
#define INC_TASKPROCESSTIMESYNCHRONIZATION_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "TimeUtils.hpp"

#include "cyphal_subscriptions.hpp"

#ifdef __arm__
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "utilities.h"
#elif __x86_64__
#include "mock_hal.h"
#endif

#include "nunavut_assert.h"
#include "uavcan/time/Synchronization_1_0.h"

class TaskProcessTimeSynchronization : public Task
{
public:
    TaskProcessTimeSynchronization() = delete;
    TaskProcessTimeSynchronization(RTC_HandleTypeDef *hrtc, uint32_t interval, uint32_t tick) : Task(interval, tick), hrtc_(hrtc), previous_millisecond_(0) {}

    virtual void handleMessage(std::shared_ptr<CyphalTransfer> transfer) override;

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;

    virtual void handleTaskImpl() override;

protected:
    RTC_HandleTypeDef *hrtc_;
    uint32_t previous_millisecond_;
};

void TaskProcessTimeSynchronization::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->subscribe(uavcan_time_Synchronization_1_0_FIXED_PORT_ID_, task);
}

void TaskProcessTimeSynchronization::handleTaskImpl() {}

void TaskProcessTimeSynchronization::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unsubscribe(uavcan_time_Synchronization_1_0_FIXED_PORT_ID_, task);
}

void TaskProcessTimeSynchronization::handleMessage(std::shared_ptr<CyphalTransfer> transfer)
{
    uint32_t previous_millisecond = this->previous_millisecond_;
    uavcan_time_Synchronization_1_0 time_synchronization;
    size_t size = transfer->payload_size;

    uavcan_time_Synchronization_1_0_deserialize_(&time_synchronization, (const uint8_t *)transfer->payload, &size);
    
    uint32_t current_millisecond = HAL_GetTick();
    TimeUtils::epoch_duration d = TimeUtils::from_uint64(time_synchronization.previous_transmission_timestamp_microsecond / 1000 + (current_millisecond - previous_millisecond));
    TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(d, hrtc_->Init.SynchPrediv);
    this->previous_millisecond_ = current_millisecond;
    if (previous_millisecond == 0) return; // Skip the first call to avoid setting the RTC time with an invalid previous_millisecond_

    HAL_RTC_SetTime(hrtc_, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(hrtc_, &rtc.date, RTC_FORMAT_BIN);
    HAL_RTCEx_SetSynchroShift(hrtc_,RTC_SHIFTADD1S_RESET, rtc.time.SubSeconds);
}

static_assert(containsMessageByPortIdCompileTime<uavcan_time_Synchronization_1_0_FIXED_PORT_ID_>(),
              "uavcan_time_Synchronization_1_0_FIXED_PORT_ID_ must be in CYPHAL_MESSAGES");


#endif /* INC_TASKPROCESSTIMESYNCHRONIZATION_HPP_ */
