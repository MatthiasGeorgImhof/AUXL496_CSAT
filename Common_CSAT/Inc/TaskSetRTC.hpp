#ifndef INC_TASKSETRTC_HPP_
#define INC_TASKSETRTC_HPP_

#include <stdint.h>
#include "o1heap.h"

#ifdef __arm__
#include "usb_device.h"
#include "usbd_cdc_if.h"
#elif __x86_64__
#include "mock_hal.h"
#endif

#include "Task.hpp"
#include "GNSS.hpp"

class TaskSetRTC : public Task {
public:
	TaskSetRTC() = delete;
	TaskSetRTC(UART_HandleTypeDef *huart_handle, RTC_HandleTypeDef *hrtc, uint32_t interval, uint32_t tick) :
		Task(interval, tick), hrtc_(hrtc), gnss_(huart_handle) {}

	virtual void handleMessage(std::shared_ptr<CyphalTransfer> /*transfer*/) override {};

	virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
	virtual void unregisterTask(RegistrationManager* manager, std::shared_ptr<Task> task) override;

    virtual void handleTaskImpl() override;

private:
    RTC_HandleTypeDef *hrtc_;
    GNSS gnss_;
};

#endif /* INC_TASKSETRTC_HPP_ */
