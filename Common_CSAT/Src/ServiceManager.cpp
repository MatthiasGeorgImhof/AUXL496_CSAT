/*
 * ServiceManager.cpp
 *
 *  Created on: Dec 31, 2024
 *      Author: mgi
 */

#include "ServiceManager.hpp"
#include <algorithm>
#include "Allocator.hpp"

void ServiceManager::initializeServices(uint32_t now) const
{
	for(auto handler : handlers_)
	{
		handler.task->initialize(now);
	}
}

void ServiceManager::handleMessage(std::shared_ptr<CyphalTransfer> transfer) const
{
	log(LOG_LEVEL_DEBUG, "ServiceManager::handleMessage %4d %2d %4d\r\n",
			transfer->metadata.remote_node_id, transfer->metadata.transfer_kind, transfer->metadata.port_id);
	for(auto handler : handlers_)
	{
		if (handler.port_id == transfer->metadata.port_id)
		{
			handler.task->handleMessage(transfer);
		}
	}
}

void ServiceManager::handleServices() const
{
	for(auto handler : handlers_)
	{
		handler.task->handleTask();
	}
}
