#ifndef INC_TASKSENDNODEPORTLIST_HPP_
#define INC_TASKSENDNODEPORTLIST_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "Logger.hpp"

#include <cstddef>

#include "nunavut_assert.h"
#include "uavcan/node/port/List_1_0.h"

template <typename... Adapters>
class TaskSendNodePortList : public TaskWithPublication<Adapters...>
{
public:
    TaskSendNodePortList() = delete;
    TaskSendNodePortList(const RegistrationManager *registration_manager, uint32_t interval, uint32_t tick, CyphalTransferID transfer_id, std::tuple<Adapters...> &adapters) : 
    TaskWithPublication<Adapters...>(interval, tick, transfer_id, adapters), registration_manager_(registration_manager) {}

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;
    virtual void handleMessage(std::shared_ptr<CyphalTransfer> /*transfer*/) override {}

private:
    CyphalSubscription createSubscription();
    CyphalPublication createPublication();

private:
    const RegistrationManager *registration_manager_;
};

template <typename... Adapters>
void TaskSendNodePortList<Adapters...>::handleTaskImpl()
{
	uavcan_node_port_List_1_0 data;
    
    char substring[64];
    char pubstring[64];
    char clistring[64];
    char servstring[64];

    size_t subscription_size = registration_manager_->getSubscriptions().size();
    data.subscribers.sparse_list.count = subscription_size;
    for(uint16_t i=0; i<subscription_size; ++i)
    {
    	data.subscribers.sparse_list.elements[i].value = registration_manager_->getSubscriptions()[i];
        snprintf(substring, sizeof(substring), "%d", registration_manager_->getSubscriptions()[i]);
    }
    uavcan_node_port_SubjectIDList_1_0_select_sparse_list_(&data.subscribers);

    size_t publication_size = registration_manager_->getPublications().size();
    data.publishers.sparse_list.count = publication_size;
    for(uint16_t i=0; i<publication_size; ++i)
    {
    	data.publishers.sparse_list.elements[i].value = registration_manager_->getPublications()[i];
        snprintf(pubstring, sizeof(pubstring), "%d", registration_manager_->getPublications()[i]);
    }
    uavcan_node_port_SubjectIDList_1_0_select_sparse_list_(&data.publishers);
    

    memset(data.clients.mask_bitpacked_, 0, sizeof(data.clients.mask_bitpacked_));
    size_t client_size = registration_manager_->getClients().size();
    for(uint16_t i=0; i<client_size; ++i)
    {
        nunavutSetBit(data.clients.mask_bitpacked_, sizeof(data.clients.mask_bitpacked_), registration_manager_->getClients()[i], true);
        snprintf(clistring, sizeof(clistring), "%d", registration_manager_->getClients()[i]);
    }

    memset(data.servers.mask_bitpacked_, 0, sizeof(data.servers.mask_bitpacked_));
    size_t server_size = registration_manager_->getServers().size();
    for(uint16_t i=0; i<server_size; ++i)
    {
        nunavutSetBit(data.servers.mask_bitpacked_, sizeof(data.servers.mask_bitpacked_), registration_manager_->getServers()[i], true);
        snprintf(substring, sizeof(substring), "%d", registration_manager_->getServers()[i]);
    }

    log(LOG_LEVEL_DEBUG, "TaskSendNodePortList ( %s) ( %s) ( %s) ( %s)\r\n", substring, pubstring, clistring, servstring);
    

    constexpr size_t PAYLOAD_SIZE = uavcan_node_port_List_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t payload[PAYLOAD_SIZE];

    TaskWithPublication<Adapters...>::publish(PAYLOAD_SIZE, payload, &data,
                                              reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(uavcan_node_port_List_1_0_serialize_),
                                              uavcan_node_port_List_1_0_FIXED_PORT_ID_);
}

template <typename... Adapters>
CyphalSubscription TaskSendNodePortList<Adapters...>::createSubscription()
{
    return CyphalSubscription{uavcan_node_port_List_1_0_FIXED_PORT_ID_, uavcan_node_port_List_1_0_EXTENT_BYTES_, CyphalTransferKindMessage};
}

template <typename... Adapters>
CyphalPublication TaskSendNodePortList<Adapters...>::createPublication()
{
    return CyphalPublication{uavcan_node_port_List_1_0_FIXED_PORT_ID_, uavcan_node_port_List_1_0_EXTENT_BYTES_, CyphalTransferKindMessage};
}

template <typename... Adapters>
void TaskSendNodePortList<Adapters...>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->publish(uavcan_node_port_List_1_0_FIXED_PORT_ID_, task);
}

template <typename... Adapters>
void TaskSendNodePortList<Adapters...>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unpublish(uavcan_node_port_List_1_0_FIXED_PORT_ID_, task);
}

#endif /* INC_TASKSENDNODEPORTLIST_HPP_ */
