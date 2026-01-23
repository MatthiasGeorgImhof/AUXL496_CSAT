#ifndef INC_TASKSETRTC_HPP_
#define INC_TASKSETRTC_HPP_

#include <optional>
#include <cstdint>

#ifdef __arm__
#include "usb_device.h"
#include "usbd_cdc_if.h"
#elif __x86_64__
#include "mock_hal.h"
#endif

#include "o1heap.h"
#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "GNSSCore.hpp"
#include "TimeUtils.hpp"

template <typename GNSS>
class TaskSetRTC : public Task {
public:
	TaskSetRTC() = delete;
	TaskSetRTC(GNSS &gnss, RTC_HandleTypeDef *hrtc, uint32_t interval, uint32_t tick) :
		Task(interval, tick), hrtc_(hrtc), gnss_(gnss) {}

	virtual void handleMessage(std::shared_ptr<CyphalTransfer> /*transfer*/) override {};

	virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
	virtual void unregisterTask(RegistrationManager* manager, std::shared_ptr<Task> task) override;

    virtual void handleTaskImpl() override;

private:
    RTC_HandleTypeDef *hrtc_;
    GNSS &gnss_;
};

template <typename GNSS>
void TaskSetRTC<GNSS>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->subscribe(PURE_HANDLER, task);
}

template <typename GNSS>
void TaskSetRTC<GNSS>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unsubscribe(PURE_HANDLER, task);
}

template <typename GNSS>
void TaskSetRTC<GNSS>::handleTaskImpl()
{
    {
        std::optional<UTCTime> utctime = gnss_.getNavTimeUTC();
        if (!utctime.has_value())
        {
            return;
        }

        TimeUtils::epoch_duration duration = TimeUtils::to_epoch_duration(utctime.value().year, utctime.value().month, utctime.value().day,
                                                                          utctime.value().hour, utctime.value().min, utctime.value().sec, utctime.value().nano);

        struct TimeUtils::RTCDateTimeSubseconds rtc = TimeUtils::to_rtc(duration, hrtc_->Init.SynchPrediv);
        HAL_RTC_SetTime(hrtc_, &rtc.time, RTC_FORMAT_BIN);
        HAL_RTC_SetDate(hrtc_, &rtc.date, RTC_FORMAT_BIN);
        HAL_RTCEx_SetSynchroShift(hrtc_, RTC_SHIFTADD1S_RESET, rtc.time.SubSeconds);
    }
}

#endif /* INC_TASKSETRTC_HPP_ */
