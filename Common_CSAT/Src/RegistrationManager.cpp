#include "RegistrationManager.hpp"

/**
 * @brief Subscribes a task to a Cyphal port using the provided adapters.
 * @param subscription The Cyphal subscription details.
 */
void RegistrationManager::subscribe(const CyphalPortID port_id, std::shared_ptr<Task> task)
{
    TaskHandler handler = {port_id, task};
    handlers_.pushOrReplace(handler, [&](const TaskHandler &existing_handler, const TaskHandler &new_handler)
                            { return existing_handler.port_id == new_handler.port_id && existing_handler.task == new_handler.task; });
    subscribe(port_id);
}

void RegistrationManager::subscribe(const CyphalPortID port_id)
{
    if (is_valid(port_id))
    {
        subscriptions_.pushOrReplace(port_id, [&](const CyphalPortID &existing_port_id, const CyphalPortID &new_port_id)
                                     { return existing_port_id == new_port_id; });
    }
}

/**
 * @brief Unsubscribes a task from a Cyphal port using the provided adapters.
 * @param subscription The Cyphal subscription details.
 */
void RegistrationManager::unsubscribe(const CyphalPortID port_id, std::shared_ptr<Task> task)
{
    handlers_.removeIf([&](const TaskHandler &handler)
                       { return handler.port_id == port_id && handler.task == task; });

    if (!handlers_.containsIf([&](const TaskHandler &handler)
                              { return handler.port_id == port_id; }))
    {
        subscriptions_.removeIf([&](const CyphalPortID &port_id_)
                                { return port_id_ == port_id; });
    }
}

/**
 * @brief publishes a task to a Cyphal port using the provided adapters.
 * @param publication The Cyphal publication details.
 */
void RegistrationManager::publish(const CyphalPortID port_id, std::shared_ptr<Task> task)
{
    TaskHandler handler = {port_id, task};
    handlers_.pushOrReplace(handler, [&](const TaskHandler &existing_handler, const TaskHandler &new_handler)
                            { return existing_handler.port_id == new_handler.port_id && existing_handler.task == new_handler.task; });
    publish(port_id);
}

void RegistrationManager::publish(const CyphalPortID port_id)
{
    if (is_valid(port_id))
    {
        publications_.pushOrReplace(port_id, [&](const CyphalPortID &existing_port_id, const CyphalPortID &new_port_id)
                                    { return existing_port_id == new_port_id; });
    }
}

/**
 * @brief Unpublishes a task from a Cyphal port using the provided adapters.
 * @param publication The Cyphal publication details.
 */
void RegistrationManager::unpublish(const CyphalPortID port_id, std::shared_ptr<Task> task)
{
    handlers_.removeIf([&](const TaskHandler &handler)
                       { return handler.port_id == port_id && handler.task == task; });
    if (!handlers_.containsIf([&](const TaskHandler &handler)
                              { return handler.port_id == port_id; }))
    {
        publications_.removeIf([&](const CyphalPortID &port_id_)
                               { return port_id_ == port_id; });
    }
}