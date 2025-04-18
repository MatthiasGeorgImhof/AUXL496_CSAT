#ifndef INC_REGISTRATIONMANAGER_HPP_
#define INC_REGISTRATIONMANAGER_HPP_

#include <memory>
#include "cstdint"
#include <tuple>
#include <algorithm>
#include <type_traits>
#include "ArrayList.hpp"
#include "Task.hpp"
#include "cyphal.hpp"

/**
 * @brief Manages the registration and unregistration of tasks, and their subscriptions
 *        to Cyphal ports.  It also handles the interaction with the Cyphal adapters.
 */
class RegistrationManager
{
public:
    static constexpr size_t NUM_SUBSCRIPTIONS = 16;
    static constexpr size_t NUM_PUBLICATIONS = 16;
    static constexpr size_t NUM_CLIENT_SERVERS = 8;
    static constexpr size_t NUM_TASK_HANDLERS = 32;

    RegistrationManager() = default;
    RegistrationManager(const RegistrationManager &) = delete;
    RegistrationManager &operator=(const RegistrationManager &) = delete;

    /**
     * @brief Adds a task to the registration manager, triggering its registration.
     * @param task A shared pointer to the task to be added.
     */
    inline void add(std::shared_ptr<Task> task);

    /**
     * @brief Removes a task from the registration manager, triggering its unregistration.
     * @param task A shared pointer to the task to be removed.
     */
    inline void remove(std::shared_ptr<Task> task);

    /**
     * @brief Subscribes a task to a Cyphal port using the provided adapters.
     * @tparam Adapters Variadic template parameter representing the Cyphal adapter types.
     * @param subscription The Cyphal subscription details.
     * @param task A shared pointer to the task to be subscribed.
     */
    void subscribe(const CyphalPortID port_id, std::shared_ptr<Task> task);
    void subscribe(const CyphalPortID port_id);

    /**
     * @brief Unsubscribes a task from a Cyphal port using the provided adapters.
     * @tparam Adapters Variadic template parameter representing the Cyphal adapter types.
     * @param subscription The Cyphal subscription details.
     * @param task A shared pointer to the task to be unsubscribed.
     */
    void unsubscribe(const CyphalPortID port_id, std::shared_ptr<Task> task);

    /**
     * @brief publishes a task to a Cyphal port using the provided adapters.
     * @tparam Adapters Variadic template parameter representing the Cyphal adapter types.
     * @param publication The Cyphal subscription details.
     * @param task A shared pointer to the task to be published.
     */
    void publish(const CyphalPortID port_id, std::shared_ptr<Task> task);
    void publish(const CyphalPortID port_id);

    /**
     * @brief Unpublishes a task from a Cyphal port using the provided adapters.
     * @tparam Adapters Variadic template parameter representing the Cyphal adapter types.
     * @param publication The Cyphal subscription details.
     * @param task A shared pointer to the task to be unpublished.
     */
    void unpublish(const CyphalPortID port_id, std::shared_ptr<Task> task);

    /**
     * @brief clientes a task to a Cyphal port using the provided adapters.
     * @tparam Adapters Variadic template parameter representing the Cyphal adapter types.
     * @param publication The Cyphal subscription details.
     * @param task A shared pointer to the task to be cliented.
     */
    void client(const CyphalPortID port_id, std::shared_ptr<Task> task);
    void client(const CyphalPortID port_id);

    /**
     * @brief Unclients a task from a Cyphal port using the provided adapters.
     * @tparam Adapters Variadic template parameter representing the Cyphal adapter types.
     * @param publication The Cyphal subscription details.
     * @param task A shared pointer to the task to be uncliented.
     */
    void unclient(const CyphalPortID port_id, std::shared_ptr<Task> task);

    /**
     * @brief serves a task to a Cyphal port using the provided adapters.
     * @tparam Adapters Variadic template parameter representing the Cyphal adapter types.
     * @param publication The Cyphal subscription details.
     * @param task A shared pointer to the task to be servered.
     */
    void server(const CyphalPortID port_id, std::shared_ptr<Task> task);
    void server(const CyphalPortID port_id);

    /**
     * @brief Unserves a task from a Cyphal port using the provided adapters.
     * @tparam Adapters Variadic template parameter representing the Cyphal adapter types.
     * @param publication The Cyphal subscription details.
     * @param task A shared pointer to the task to be unservered.
     */
    void unserver(const CyphalPortID port_id, std::shared_ptr<Task> task);

    /**
     * @brief Gets the list of task handlers.
     * @return A const reference to the ArrayList of task handlers.
     */
    inline const ArrayList<TaskHandler, NUM_TASK_HANDLERS> &getHandlers() const;

    /**
     * @brief Gets the list of Cyphal subscriptions.
     * @return A const reference to the ArrayList of Cyphal subscriptions.
     */
    inline const ArrayList<CyphalPortID, NUM_SUBSCRIPTIONS> &getSubscriptions() const;
    inline const ArrayList<CyphalPortID, NUM_PUBLICATIONS> &getPublications() const;
    inline const ArrayList<CyphalPortID, NUM_CLIENT_SERVERS> &getClients() const;
    inline const ArrayList<CyphalPortID, NUM_CLIENT_SERVERS> &getServers() const;

    inline bool containsTask(const std::shared_ptr<Task> &task) const;

private:
    /**
     * @brief List of task handlers.
     */
    ArrayList<TaskHandler, NUM_TASK_HANDLERS> handlers_;

    /**
     * @brief List of Cyphal subscriptions.
     */
    ArrayList<CyphalPortID, NUM_SUBSCRIPTIONS> subscriptions_;
    ArrayList<CyphalPortID, NUM_PUBLICATIONS> publications_;
    ArrayList<CyphalPortID, NUM_CLIENT_SERVERS> clients_;
    ArrayList<CyphalPortID, NUM_CLIENT_SERVERS> servers_;
};


/**
 * @brief Adds a task to the registration manager, triggering its registration.
 * @param task A shared pointer to the task to be added.
 */
inline void RegistrationManager::add(std::shared_ptr<Task> task)
{
    task->registerTask(this, task);
}

/**
 * @brief Removes a task from the registration manager, triggering its unregistration.
 * @param task A shared pointer to the task to be removed.
 */
inline void RegistrationManager::remove(std::shared_ptr<Task> task)
{
    task->unregisterTask(this, task);
}

inline bool RegistrationManager::containsTask(const std::shared_ptr<Task> &task) const
{
    return std::any_of(handlers_.begin(), handlers_.end(),
                       [&](const auto &handler)
                       {
                           return handler.task == task; // Compare the shared_ptr directly
                       });
}

/**
 * @brief Gets the list of task handlers.
 * @return A const reference to the ArrayList of task handlers.
 */
inline const ArrayList<TaskHandler, RegistrationManager::NUM_TASK_HANDLERS> &RegistrationManager::getHandlers() const { return handlers_; }

/**
 * @brief Gets the list of Cyphal subscriptions.
 * @return A const reference to the ArrayList of Cyphal subscriptions.
 */
inline const ArrayList<CyphalPortID, RegistrationManager::NUM_SUBSCRIPTIONS> &RegistrationManager::getSubscriptions() const { return subscriptions_; }
inline const ArrayList<CyphalPortID, RegistrationManager::NUM_PUBLICATIONS> &RegistrationManager::getPublications() const { return publications_; }
inline const ArrayList<CyphalPortID, RegistrationManager::NUM_CLIENT_SERVERS> &RegistrationManager::getClients() const { return clients_; }
inline const ArrayList<CyphalPortID, RegistrationManager::NUM_CLIENT_SERVERS> &RegistrationManager::getServers() const { return servers_; }

#endif