/*
 * Task.hpp
 *
 *  Created on: Dec 4, 2024
 *      Author: mgi
 */

#ifndef INC_TASKCHECKMEMORY_HPP_
#define INC_TASKCHECKMEMORY_HPP_

#include <stdint.h>
#include "o1heap.h"
#include "Task.hpp"

class RegistrationManager;

class TaskCheckMemory : public Task {
public:
	TaskCheckMemory() = delete;
	TaskCheckMemory(O1HeapInstance *o1heap, uint32_t interval, uint32_t tick) : Task(interval, tick), o1heap(o1heap) {}

	virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
	virtual void unregisterTask(RegistrationManager* manager, std::shared_ptr<Task> task) override;

    virtual void handleTaskImpl() override;

protected:
    O1HeapInstance *o1heap;
};

#endif /* INC_TASKCHECKMEMORY_HPP_ */
