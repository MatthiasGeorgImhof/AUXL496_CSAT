#include "RegistrationManager.hpp"

/**
 * @brief Subscribes a task to a Cyphal port.
 * @param port_id The Cyphal port ID to subscribe to.
 * @param task The task to be associated with the port.
 */
void RegistrationManager::subscribe(const CyphalPortID port_id, std::shared_ptr<Task> task)
{
    TaskHandler handler = {port_id, task};
    handlers_.pushOrReplace(handler, [&](const TaskHandler &existing_handler, const TaskHandler &new_handler)
                            { return existing_handler.port_id == new_handler.port_id && existing_handler.task == new_handler.task; });
    subscribe(port_id);
}

/**
 * @brief Subscribes to a Cyphal port ID.
 * @param port_id The Cyphal port ID to subscribe to.
 */
void RegistrationManager::subscribe(const CyphalPortID port_id)
{
    if (is_valid(port_id))
    {
        subscriptions_.pushOrReplace(port_id, [&](const CyphalPortID &existing_port_id, const CyphalPortID &new_port_id)
                                     { return existing_port_id == new_port_id; });
    }
}

/**
 * @brief Unsubscribes a task from a Cyphal port.
 * @param port_id The Cyphal port ID to unsubscribe from.
 * @param task The task to be disassociated from the port.
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
 * @brief Associates a task with a Cyphal port for publishing.
 * @param port_id The Cyphal port ID to publish to.
 * @param task The task to be associated with the port.
 */
void RegistrationManager::publish(const CyphalPortID port_id, std::shared_ptr<Task> task)
{
    TaskHandler handler = {port_id, task};
    handlers_.pushOrReplace(handler, [&](const TaskHandler &existing_handler, const TaskHandler &new_handler)
                            { return existing_handler.port_id == new_handler.port_id && existing_handler.task == new_handler.task; });
    publish(port_id);
}

/**
 * @brief Registers a Cyphal port ID for publishing.
 * @param port_id The Cyphal port ID to register.
 */
void RegistrationManager::publish(const CyphalPortID port_id)
{
    if (is_valid(port_id))
    {
        publications_.pushOrReplace(port_id, [&](const CyphalPortID &existing_port_id, const CyphalPortID &new_port_id)
                                    { return existing_port_id == new_port_id; });
    }
}

/**
 * @brief Removes the association of a task with a Cyphal port for publishing.
 * @param port_id The Cyphal port ID to unpublish from.
 * @param task The task to be disassociated from the port.
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

/**
 * @brief Associates a task with a Cyphal port for client requests.
 * @param port_id The Cyphal port ID to use for client requests.
 * @param task The task to be associated with the port.
 */
void RegistrationManager::client(const CyphalPortID port_id, std::shared_ptr<Task> task)
{
    TaskHandler handler = {port_id, task};
    handlers_.pushOrReplace(handler, [&](const TaskHandler &existing_handler, const TaskHandler &new_handler)
                            { return existing_handler.port_id == new_handler.port_id && existing_handler.task == new_handler.task; });
    client(port_id);
}

/**
 * @brief Registers a Cyphal port ID as a client.
 * @param port_id The Cyphal port ID to register as a client.
 */
void RegistrationManager::client(const CyphalPortID port_id)
{
     if (is_valid(port_id))
    {
        clients_.pushOrReplace(port_id, [&](const CyphalPortID &existing_port_id, const CyphalPortID &new_port_id)
                                    { return existing_port_id == new_port_id; });
    }
}

/**
 * @brief Removes the association of a task with a Cyphal port for client requests.
 * @param port_id The Cyphal port ID to unclient from.
 * @param task The task to be disassociated from the port.
 */
void RegistrationManager::unclient(const CyphalPortID port_id, std::shared_ptr<Task> task)
{
    handlers_.removeIf([&](const TaskHandler &handler)
                       { return handler.port_id == port_id && handler.task == task; });
     if (!handlers_.containsIf([&](const TaskHandler &handler)
                              { return handler.port_id == port_id; }))
    {
        clients_.removeIf([&](const CyphalPortID &port_id_)
                               { return port_id_ == port_id; });
    }
}

/**
 * @brief Associates a task with a Cyphal port for server responses.
 * @param port_id The Cyphal port ID to use for server responses.
 * @param task The task to be associated with the port.
 */
void RegistrationManager::server(const CyphalPortID port_id, std::shared_ptr<Task> task)
{
    TaskHandler handler = {port_id, task};
    handlers_.pushOrReplace(handler, [&](const TaskHandler &existing_handler, const TaskHandler &new_handler)
                            { return existing_handler.port_id == new_handler.port_id && existing_handler.task == new_handler.task; });
    server(port_id);
}

/**
 * @brief Registers a Cyphal port ID as a server.
 * @param port_id The Cyphal port ID to register as a server.
 */
void RegistrationManager::server(const CyphalPortID port_id)
{
    if (is_valid(port_id))
    {
        servers_.pushOrReplace(port_id, [&](const CyphalPortID &existing_port_id, const CyphalPortID &new_port_id)
                                    { return existing_port_id == new_port_id; });
    }
}

/**
 * @brief Removes the association of a task with a Cyphal port for server responses.
 * @param port_id The Cyphal port ID to unserver from.
 * @param task The task to be disassociated from the port.
 */
void RegistrationManager::unserver(const CyphalPortID port_id, std::shared_ptr<Task> task)
{
    handlers_.removeIf([&](const TaskHandler &handler)
                       { return handler.port_id == port_id && handler.task == task; });
    if (!handlers_.containsIf([&](const TaskHandler &handler)
                              { return handler.port_id == port_id; }))
    {
        servers_.removeIf([&](const CyphalPortID &port_id_)
                               { return port_id_ == port_id; });
    }
}