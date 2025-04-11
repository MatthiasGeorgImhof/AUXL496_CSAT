#ifndef INC_SubscriptionManager_HPP_
#define INC_SubscriptionManager_HPP_

#include <memory>
#include "cstdint"
#include <tuple>
#include <algorithm>
#include <type_traits>
#include "ArrayList.hpp"
#include "Task.hpp"
#include "cyphal.hpp"
#include "cyphal_subscriptions.hpp"

/**
 * @brief Manages subscriptions to Cyphal ports.  It also handles the interaction with the Cyphal adapters.
 */
class SubscriptionManager
{
public:
    static constexpr size_t NUM_SUBSCRIPTIONS = 16;
    static constexpr size_t NUM_PUBLICATIONS = 16;

    SubscriptionManager() = default;
    SubscriptionManager(const SubscriptionManager &) = delete;
    SubscriptionManager &operator=(const SubscriptionManager &) = delete;

    /**
     * @brief Subscribes a task to a Cyphal port using the provided adapters.
     * @tparam Adapters Variadic template parameter representing the Cyphal adapter types.
     * @param subscription The Cyphal subscription details.
     * @param adapters A tuple containing the Cyphal adapter instances.
     */
    template <typename List, typename... Adapters>
    void subscribe(const List &port_ids, std::tuple<Adapters...> &adapters);

    /**
     * @brief Subscribes a task to a Cyphal port using the provided adapters.
     * @tparam Adapters Variadic template parameter representing the Cyphal adapter types.
     * @param subscription The Cyphal subscription details.
     * @param adapters A tuple containing the Cyphal adapter instances.
     */
    template <typename... Adapters>
    void subscribe(const CyphalPortID port_id, std::tuple<Adapters...> &adapters);

    /**
     * @brief Unsubscribes a task from a Cyphal port using the provided adapters.
     * @tparam Adapters Variadic template parameter representing the Cyphal adapter types.
     * @param subscription The Cyphal subscription details.
     * @param adapters A tuple containing the Cyphal adapter instances.
     */
    template <typename List, typename... Adapters>

    void unsubscribe(const List &port_ids, std::tuple<Adapters...> &adapters);
    /**
     * @brief Unsubscribes a task from a Cyphal port using the provided adapters.
     * @tparam Adapters Variadic template parameter representing the Cyphal adapter types.
     * @param subscription The Cyphal subscription details.
     * @param adapters A tuple containing the Cyphal adapter instances.
     */
    template <typename... Adapters>
    void unsubscribe(const CyphalPortID port_id, std::tuple<Adapters...> &adapters);

    /**
     * @brief Gets the list of Cyphal subscriptions.
     * @return A const reference to the ArrayList of Cyphal subscriptions.
     */
    inline const ArrayList<const CyphalSubscription *, NUM_SUBSCRIPTIONS> &getSubscriptions() const;

private:
    /**
     * @brief List of Cyphal subscriptions.
     */
    ArrayList<const CyphalSubscription *, NUM_SUBSCRIPTIONS> subscriptions_;
};

/**
 * @brief Subscribes a task to a Cyphal port using the provided adapters.
 * @tparam Adapters Variadic template parameter representing the Cyphal adapter types.
 * @param subscription The Cyphal subscription details.
 */

template <typename List, typename... Adapters>
void SubscriptionManager::subscribe(const List &port_ids, std::tuple<Adapters...> &adapters)
{
    for (auto port_id : port_ids)
        subscribe(port_id, adapters);
}

template <typename... Adapters>
void SubscriptionManager::subscribe(const CyphalPortID port_id, std::tuple<Adapters...> &adapters)
{
    const CyphalSubscription *subscription = findByPortIdRuntime(port_id);
    if (subscription == nullptr)
        return;

    subscriptions_.push(subscription);

    bool all_successful = true;
    std::apply([&](auto &...adapter)
               { ([&]()
                  {
            int8_t res = (adapter.cyphalRxSubscribe(subscription->transfer_kind, subscription->port_id, subscription->extent, 1000) > 0);
            all_successful = all_successful && res; }(), ...); }, adapters);
}

/**
 * @brief Unsubscribes a task from a Cyphal port using the provided adapters.
 * @tparam Adapters Variadic template parameter representing the Cyphal adapter types.
 * @param subscription The Cyphal subscription details.
 */
template <typename List, typename... Adapters>
void SubscriptionManager::unsubscribe(const List &port_ids, std::tuple<Adapters...> &adapters)
{
    for (auto port_id : port_ids)
        unsubscribe(port_id, adapters);
}

template <typename... Adapters>
void SubscriptionManager::unsubscribe(const CyphalPortID port_id, std::tuple<Adapters...> &adapters)
{
    const CyphalSubscription *subscription = findByPortIdRuntime(port_id);
    if (subscription == nullptr)
        return;

    subscriptions_.removeIf([&](const CyphalSubscription *sub)
                            { return sub == subscription; });

    bool all_successful = true;
    std::apply([&](auto &...adapter)
               { ([&]()
                  {
            int8_t res = (adapter.cyphalRxUnsubscribe(subscription->transfer_kind, subscription->port_id) > 0);
            all_successful = all_successful && res; }(), ...); }, adapters);
}

/**
 * @brief Gets the list of Cyphal subscriptions.
 * @return A const reference to the ArrayList of Cyphal subscriptions.
 */
inline const ArrayList<const CyphalSubscription *, SubscriptionManager::NUM_SUBSCRIPTIONS> &SubscriptionManager::getSubscriptions() const { return subscriptions_; }

#endif