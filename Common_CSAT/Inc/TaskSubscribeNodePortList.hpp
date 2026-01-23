#ifndef INC_TASKSUBSCRIBENODEPORTLIST_HPP_
#define INC_TASKSUBSCRIBENODEPORTLIST_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "SubscriptionManager.hpp"
#include "Logger.hpp"

#include "nunavut_assert.h"
#include "uavcan/node/port/List_1_0.h"
#include "nunavut/support/serialization.h"

template <typename... Adapters>
class TaskSubscribeNodePortList : public TaskFromBuffer<CyphalBuffer8>
{
public:
    TaskSubscribeNodePortList() = delete;
    TaskSubscribeNodePortList(SubscriptionManager *subscription_manager, uint32_t interval, uint32_t tick, std::tuple<Adapters...>& adapters) : TaskFromBuffer<CyphalBuffer8>(interval, tick), adapters_(adapters), subscription_manager_(subscription_manager) {}

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
    if (buffer_.is_empty())
    {
        log(LOG_LEVEL_TRACE, "TaskSubscribeNodePortList: empty buffer\r\n");
        return;
    }

    // Process all transfers in the buffer
	size_t count = buffer_.size();
    for (size_t i = 0; i < count; ++i)
    {
        std::shared_ptr<CyphalTransfer> transfer = buffer_.pop();
        log(LOG_LEVEL_DEBUG, "TaskSubscribeNodePortList message i=%d with %d owners,  current buffer size=%d of %d\r\n", i, transfer.use_count(), buffer_.size(), buffer_.capacity());

        size_t payload_size = transfer->payload_size;
        uavcan_node_port_List_1_0 data;  // Declare data here
        int8_t result = uavcan_node_port_List_1_0_deserialize_(&data, static_cast<const uint8_t*>(transfer->payload), &payload_size); // Pass the address
        if (result != 0)
        {
        	log(LOG_LEVEL_ERROR, "TaskSubscribeNodePortList: deserialization error\r\n");
        	continue;
        }

         char substring[64];
         char pubstring[64];
         char clistring[64];
         char servstring[64];

         for (size_t j = 0; j < data.publishers.sparse_list.count; ++j)
         {
             subscription_manager_->subscribe<SubscriptionManager::MessageTag>(data.publishers.sparse_list.elements[j].value, adapters_);
             snprintf(substring, sizeof(substring), "%d ", data.publishers.sparse_list.elements[j].value);
         }
         for (size_t j = 0; j < data.subscribers.sparse_list.count; ++j)
         {
             subscription_manager_->subscribe<SubscriptionManager::MessageTag>(data.subscribers.sparse_list.elements[j].value, adapters_);
             snprintf(pubstring, sizeof(pubstring), "%d ", data.subscribers.sparse_list.elements[j].value);
         }
         for(const CyphalSubscription &subscription : CYPHAL_REQUESTS)
         {
             if (nunavutGetBit(data.clients.mask_bitpacked_, sizeof(data.clients.mask_bitpacked_), subscription.port_id))
             {
                 subscription_manager_->subscribe(&subscription, adapters_);
                 snprintf(clistring, sizeof(clistring), "%d ", subscription.port_id);
             }
         }
         for(const CyphalSubscription &subscription : CYPHAL_RESPONSES)
         {
             if (nunavutGetBit(data.servers.mask_bitpacked_, sizeof(data.servers.mask_bitpacked_), subscription.port_id))
             {
                 subscription_manager_->subscribe(&subscription, adapters_);
                 snprintf(servstring, sizeof(servstring), "%d ", subscription.port_id);
             }
         }

         log(LOG_LEVEL_DEBUG, "TaskSubscribeNodePortList %d ( %s) ( %s) ( %s) ( %s)\r\n", transfer->metadata.remote_node_id, substring, pubstring, clistring, servstring);
    }

    log(LOG_LEVEL_DEBUG, "TaskSubscribeNodePortList end of handleTaskImpl\r\n");
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
