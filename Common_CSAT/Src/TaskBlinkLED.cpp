#include "TaskBlinkLED.hpp"
#include "RegistrationManager.hpp"
#include "cyphal.hpp"

#ifdef __arm__
#include "usbd_cdc_if.h"
#endif
#ifdef __x86_64__
#include "mock_hal.h"
#endif

void TaskBlinkLED::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
	manager->subscribe(PURE_HANDLER, task);
}

void TaskBlinkLED::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
	manager->unsubscribe(PURE_HANDLER, task);
}

void TaskBlinkLED::handleTaskImpl()
{
	HAL_GPIO_TogglePin(GPIO_, pins_);
}
