#ifndef INC_TASKSUBSCRIBENODEPORTLIST_HPP_
#define INC_TASKSUBSCRIBENODEPORTLIST_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "SubscriptionManager.hpp"
#include "Logger.hpp"

#include "nunavut_assert.h"
#include "uavcan/node/port/List_1_0.h"

template <typename... Adapters>
class TaskSubscribeNodePortList : public TaskFromBuffer
{
public:
    TaskSubscribeNodePortList() = delete;
    TaskSubscribeNodePortList(SubscriptionManager *subscription_manager, uint32_t interval, uint32_t tick, std::tuple<Adapters...>& adapters) : TaskFromBuffer(interval, tick), adapters_(adapters), subscription_manager_(subscription_manager) {}

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;

private:
    std::tuple<Adapters...>& adapters_;
    SubscriptionManager *subscription_manager_;
};

template <typename... Adapters>
void TaskSubscribeNodePortList<Adapters...>::handleTaskImpl()
{
    char subscribers[256] = {};
	auto subscriptions = subscription_manager_->getSubscriptions();
	for(auto sub: subscriptions)
	{
        char vstring[16] = {};
        sprintf(vstring, "% 5d", sub->port_id);
        strcat(subscribers, vstring);
	}

	log(LOG_LEVEL_DEBUG, "TaskSubscribeNodePortList: %d %d ( %s)\r\n", buffer.size(), subscriptions.size(), subscribers);

    // Check if the buffer is empty
    if (buffer.is_empty())
    {
        return;
    }

    // Process all transfers in the buffer
    // Use a loop to process all items in the buffer
	size_t count = buffer.size();
    for (size_t i = 0; i < count; ++i)
    {
        std::shared_ptr<CyphalTransfer> transfer = buffer.pop();
        size_t payload_size = transfer->payload_size;
        uavcan_node_port_List_1_0 data;  // Declare data here
        int8_t result = uavcan_node_port_List_1_0_deserialize_(&data, static_cast<const uint8_t*>(transfer->payload), &payload_size); // Pass the address
        if (result != 0)
        {
        	log(LOG_LEVEL_DEBUG, "TaskSubscribeNodePortList: deserialization\r\n");
        	return;
        }

        char publishers[256] = {};
        char subscribers[256] = {};

        // Iterate over publishers and subscribers and call subscribe.
        for (size_t j = 0; j < data.publishers.sparse_list.count; ++j)
        {
            subscription_manager_->subscribe(data.publishers.sparse_list.elements[j].value, adapters_);
            char vstring[16] = {};
            sprintf(vstring, "% 5d", data.publishers.sparse_list.elements[j].value);
            strcat(publishers, vstring);
        }
        for (size_t j = 0; j < data.subscribers.sparse_list.count; ++j)
        {
            subscription_manager_->subscribe(data.subscribers.sparse_list.elements[j].value, adapters_);
            char vstring[16] = {};
            sprintf(vstring, "% 5d", data.subscribers.sparse_list.elements[j].value);
            strcat(subscribers, vstring);
        }
        log(LOG_LEVEL_DEBUG, "TaskSubscribeNodePortList: (%s ) (%s )\r\n", publishers, subscribers);

    }
}

template <typename... Adapters>
void TaskSubscribeNodePortList<Adapters...>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->subscribe(uavcan_node_port_List_1_0_FIXED_PORT_ID_, task);
}

template <typename... Adapters>
void TaskSubscribeNodePortList<Adapters...>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unsubscribe(uavcan_node_port_List_1_0_FIXED_PORT_ID_, task);
}

#endif /* INC_TASKSUBSCRIBENODEPORTLIST_HPP_ */
