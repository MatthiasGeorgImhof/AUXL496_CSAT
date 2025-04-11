#pragma once

#include <stdint.h>
#include "Task.hpp"

class RegistrationManager;

class TaskBlinkLED : public Task {
public:
TaskBlinkLED() = delete;
TaskBlinkLED(GPIO_TypeDef* GPIO, uint16_t pins, uint32_t interval, uint32_t tick) : Task(interval, tick), GPIO_(GPIO), pins_(pins) {}

	virtual void handleMessage(std::shared_ptr<CyphalTransfer> /*transfer*/) override {};

	virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
	virtual void unregisterTask(RegistrationManager* manager, std::shared_ptr<Task> task) override;

    virtual void handleTaskImpl() override;

protected:
GPIO_TypeDef* GPIO_;
uint16_t pins_;
};
