#ifndef _TASKCHECKTXQUEUE_HPP_
#define _TASKCHECKTXQUEUE_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "canard_adapter.hpp"

class TaskCheckTxQueue : public Task
{
public:
    TaskCheckTxQueue(uint32_t interval, uint32_t tick, CanardAdapter& adapter)
        : Task(interval, tick)
        , canard_adapter(adapter)
    {}

    virtual void registerTask(RegistrationManager* manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager* manager, std::shared_ptr<Task> task) override;

    virtual void handleTaskImpl() override;

protected:
    CanardAdapter& canard_adapter;
};

inline void TaskCheckTxQueue::handleTaskImpl()
{
	log(LOG_LEVEL_DEBUG, "TaskCheckTxQueue queue capacity %d size %d\r\n", canard_adapter.que.capacity, canard_adapter.que.size);
    tx_drainer.irq_safe_drain();
}

inline void TaskCheckTxQueue::registerTask(RegistrationManager* manager, std::shared_ptr<Task> task)
{
    manager->subscribe(PURE_HANDLER, task);
}

inline void TaskCheckTxQueue::unregisterTask(RegistrationManager* manager, std::shared_ptr<Task> task)
{
    manager->unsubscribe(PURE_HANDLER, task);
}

#endif // _TASKCHECKTXQUEUE_HPP_
