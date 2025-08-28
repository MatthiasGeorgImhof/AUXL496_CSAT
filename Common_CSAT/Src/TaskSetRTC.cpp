/*
 * TaskSetRTC.cpp
 *
 *  Created on: Dec 4, 2024
 *      Author: mgi
 */

#include <TaskSetRTC.hpp>
#include "ServiceManager.hpp"
#include "TimeUtils.hpp"
#ifdef __arm__
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "utilities.h"
#elif __x86_64__
#include "mock_hal.h"
#endif

void TaskSetRTC::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->subscribe(PURE_HANDLER, task);
}

void TaskSetRTC::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unsubscribe(PURE_HANDLER, task);
}

void TaskSetRTC::handleTaskImpl()
{
    {
        std::optional<UTCTime> utctime = gnss_.GetNavTimeUTC();
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
