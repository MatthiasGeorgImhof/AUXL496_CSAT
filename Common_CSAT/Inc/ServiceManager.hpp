/*
 * ServiceManager.hpp
 *
 *  Created on: Dec 31, 2024
 *      Author: mgi
 */

#ifndef INC_SERVICEMANAGER_HPP_
#define INC_SERVICEMANAGER_HPP_

#include <memory>
#include "cstdint"

#include "ArrayList.hpp"
#include "Task.hpp"

class ServiceManager {
public:
	ServiceManager () = delete;
	ServiceManager(const ArrayList<TaskHandler, NUM_TASK_HANDLERS> &handlers): handlers_(handlers) {};

	void initializeServices(uint32_t now) const;
	void handleMessage(std::shared_ptr<CyphalTransfer> transfer) const;
	void handleServices() const;

    inline const ArrayList<TaskHandler, NUM_TASK_HANDLERS> &getHandlers() const { return handlers_; };

private:
	const ArrayList<TaskHandler, NUM_TASK_HANDLERS> &handlers_;
};

#endif /* INC_SERVICEMANAGER_HPP_ */
