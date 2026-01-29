#ifndef INC_SubscriptionManager_HPP_
#define INC_SubscriptionManager_HPP_

#include <memory>
#include "cstdint"
#include <tuple>
#include <algorithm>
#include <type_traits>
#include "ArrayList.hpp"
#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "cyphal.hpp"
#include "cyphal_subscriptions.hpp"

/**
 * @brief Manages subscriptions to Cyphal ports and interacts with Cyphal adapters.
 */
class SubscriptionManager
{
public:
    static constexpr size_t NUM_SUBSCRIPTIONS = 16;
    static constexpr size_t NUM_PUBLICATIONS = 16;

    struct MessageTag {};
    struct RequestTag {};
    struct ResponseTag {};

public:
    SubscriptionManager() = default;
    SubscriptionManager(const SubscriptionManager &) = delete;
    SubscriptionManager &operator=(const SubscriptionManager &) = delete;

    template <typename... Adapters>
    void subscribeAll(const RegistrationManager& reg, std::tuple<Adapters...>& adapters);

    /**
     * @brief Subscribes to a single Cyphal port using the provided adapters.
     * @tparam Adapters  Variadic template parameter representing the Cyphal adapter types.
     * @param subscription     The Cyphal subscription to subscribe to.
     * @param adapters    A tuple containing the Cyphal adapter instances.
     */
    template <typename... Adapters>
    void subscribe(const CyphalSubscription *subscription, std::tuple<Adapters...> &adapters);

    /**
     * @brief Subscribes to Cyphal ports using the provided adapters.
     * @tparam Tag  Cyphal messages array type (MessageTag, RequestTag, ResponseTag).
     * @tparam PortIDs   Container type of port IDs (e.g., std::vector).
     * @tparam Adapters  Variadic template parameter representing the Cyphal adapter types.
     * @param port_ids    Container of Cyphal port IDs to subscribe to.
     * @param adapters    A tuple containing the Cyphal adapter instances.
     */
    template <typename Tag, typename PortIDs, typename... Adapters>
    void subscribe(const PortIDs &port_ids, std::tuple<Adapters...> &adapters);

    /**
     * @brief Subscribes to a single Cyphal port using the provided adapters.
     * @tparam Tag  Cyphal messages array type (MessageTag, RequestTag, ResponseTag).
     * @tparam Adapters  Variadic template parameter representing the Cyphal adapter types.
     * @param port_id     The Cyphal port ID to subscribe to.
     * @param adapters    A tuple containing the Cyphal adapter instances.
     */
    template <typename Tag, typename... Adapters>
    void subscribe(const CyphalPortID port_id, std::tuple<Adapters...> &adapters);

    /**
     * @brief Unsubscribes from a single Cyphal port using the provided adapters.
     * @tparam Adapters  Variadic template parameter representing the Cyphal adapter types.
     * @param subscription     The Cyphal subscription to unsubscribe from.
     * @param adapters    A tuple containing the Cyphal adapter instances.
     */
    template <typename... Adapters>
    void unsubscribe(const CyphalSubscription *subscription, std::tuple<Adapters...> &adapters);

    /**
     * @brief Unsubscribes from Cyphal ports using the provided adapters.
     * @tparam Tag  Cyphal messages array type (MessageTag, RequestTag, ResponseTag).
     * @tparam PortIDs   Container type of port IDs (e.g., std::vector).
     * @tparam Adapters  Variadic template parameter representing the Cyphal adapter types.
     * @param port_ids    Container of Cyphal port IDs to unsubscribe from.
     * @param adapters    A tuple containing the Cyphal adapter instances.
     */
    template <typename Tag, typename PortIDs, typename... Adapters>
    void unsubscribe(const PortIDs &port_ids, std::tuple<Adapters...> &adapters);

    /**
     * @brief Unsubscribes from a single Cyphal port using the provided adapters.
     * @tparam Tag  Cyphal messages array type (MessageTag, RequestTag, ResponseTag).
     * @tparam Adapters  Variadic template parameter representing the Cyphal adapter types.
     * @param port_id     The Cyphal port ID to unsubscribe from.
     */
    template <typename Tag, typename... Adapters>
    void unsubscribe(const CyphalPortID port_id, std::tuple<Adapters...> &adapters);

    /**
     * @brief Gets the list of Cyphal subscriptions.
     * @return A const reference to the ArrayList of Cyphal subscriptions.
     */
    inline const ArrayList<const CyphalSubscription *, NUM_SUBSCRIPTIONS> &getSubscriptions() const { return subscriptions_; }

private:
    ArrayList<const CyphalSubscription *, NUM_SUBSCRIPTIONS> subscriptions_;
};

/**
 * @brief Subscribes to Cyphal ports using the provided adapters.
 */
template <typename... Adapters>
void SubscriptionManager::subscribe(const CyphalSubscription *subscription, std::tuple<Adapters...> &adapters)
{
    if (subscriptions_.full()) {
        return;
    }

    subscriptions_.push(subscription);
    bool all_successful = true;
    std::apply([&](auto &...adapter)
               { (([&]()
                   {
                 int8_t res = (adapter.cyphalRxSubscribe(subscription->transfer_kind, subscription->port_id, subscription->extent, 1000) > 0);
                 all_successful = all_successful && res; }()),
                  ...); }, adapters);
}

template <typename Tag, typename PortIDs, typename... Adapters>
void SubscriptionManager::subscribe(const PortIDs &port_ids, std::tuple<Adapters...> &adapters)
{
    static_assert(std::is_same_v<Tag, SubscriptionManager::MessageTag> ||
        std::is_same_v<Tag, SubscriptionManager::RequestTag> ||
        std::is_same_v<Tag, SubscriptionManager::ResponseTag>,
        "Invalid tag type. Tag must be MessageTag, RequestTag, or ResponseTag.");
    
    for (auto port_id : port_ids)
    {
        subscribe<Tag>(port_id, adapters);
    }
}

template <typename Tag, typename... Adapters>
void SubscriptionManager::subscribe(const CyphalPortID port_id, std::tuple<Adapters...> &adapters)
{
    const CyphalSubscription *subscription = nullptr;
    static_assert(std::is_same_v<Tag, SubscriptionManager::MessageTag> ||
        std::is_same_v<Tag, SubscriptionManager::RequestTag> ||
        std::is_same_v<Tag, SubscriptionManager::ResponseTag>,
        "Invalid tag type. Tag must be MessageTag, RequestTag, or ResponseTag.");

    if constexpr (std::is_same_v<Tag, SubscriptionManager::MessageTag>)
    {
        subscription = findMessageByPortIdRuntime(port_id);
    }
    else if constexpr (std::is_same_v<Tag, SubscriptionManager::RequestTag>)
    {
        subscription = findRequestByPortIdRuntime(port_id);
    }
    else if constexpr (std::is_same_v<Tag, SubscriptionManager::ResponseTag>)
    {
        subscription = findResponseByPortIdRuntime(port_id);
    }

    if (subscription == nullptr)
    {
        return;
    }
    subscribe(subscription, adapters);
}

/**
 * @brief Unsubscribes from Cyphal ports using the provided adapters.
 */
template <typename... Adapters>
void SubscriptionManager::unsubscribe(const CyphalSubscription *subscription, std::tuple<Adapters...> &adapters)
{
    bool all_successful = true;
    std::apply([&](auto &...adapter)
               { (([&]()
                   {
                int8_t res = (adapter.cyphalRxUnsubscribe(subscription->transfer_kind, subscription->port_id) > 0);
                all_successful = all_successful && res; }()),
                  ...); }, adapters);

    subscriptions_.removeIf([&](const CyphalSubscription *sub)
                             { return sub == subscription; });
}

template <typename Tag, typename PortIDs, typename... Adapters>
void SubscriptionManager::unsubscribe(const PortIDs &port_ids, std::tuple<Adapters...> &adapters)
{
    static_assert(std::is_same_v<Tag, SubscriptionManager::MessageTag> ||
        std::is_same_v<Tag, SubscriptionManager::RequestTag> ||
        std::is_same_v<Tag, SubscriptionManager::ResponseTag>,
        "Invalid tag type. Tag must be MessageTag, RequestTag, or ResponseTag.");

    for (auto port_id : port_ids)
    {
        unsubscribe<Tag>(port_id, adapters);
    }
}

template <typename Tag, typename... Adapters>
void SubscriptionManager::unsubscribe(const CyphalPortID port_id, std::tuple<Adapters...> &adapters)
{
    const CyphalSubscription *subscription = nullptr;
    static_assert(std::is_same_v<Tag, SubscriptionManager::MessageTag> ||
        std::is_same_v<Tag, SubscriptionManager::RequestTag> ||
        std::is_same_v<Tag, SubscriptionManager::ResponseTag>,
        "Invalid tag type. Tag must be MessageTag, RequestTag, or ResponseTag.");

    if constexpr (std::is_same_v<Tag, SubscriptionManager::MessageTag>)
    {
        subscription = findMessageByPortIdRuntime(port_id);
    }
    else if constexpr (std::is_same_v<Tag, SubscriptionManager::RequestTag>)
    {
        subscription = findRequestByPortIdRuntime(port_id);
    }
    else if constexpr (std::is_same_v<Tag, SubscriptionManager::ResponseTag>)
    {
        subscription = findResponseByPortIdRuntime(port_id);
    }

    if (subscription == nullptr)
    {
        return;
    }
    unsubscribe(subscription, adapters);
}

template <typename... Adapters>
void SubscriptionManager::subscribeAll(const RegistrationManager& reg, std::tuple<Adapters...>& adapters)
{
    // Subscribe to all message ports
    subscribe<SubscriptionManager::MessageTag>(reg.getSubscriptions(), adapters);

    // Subscribe to all request ports (servers receive requests)
    subscribe<SubscriptionManager::RequestTag>(reg.getServers(), adapters);

    // Subscribe to all response ports (clients receive responses)
    subscribe<SubscriptionManager::ResponseTag>(reg.getClients(), adapters);
}


#endif // INC_SubscriptionManager_HPP_
