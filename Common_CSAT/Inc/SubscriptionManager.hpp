#ifndef INC_SubscriptionManager_HPP_
#define INC_SubscriptionManager_HPP_

#include <cstdint>
#include <cstddef>
#include <memory>
#include <tuple>
#include <algorithm>
#include <type_traits>

#include "ArrayList.hpp"
#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "cyphal.hpp"
#include "cyphal_subscriptions.hpp"

class SubscriptionManager
{
public:
    static constexpr size_t NUM_SUBSCRIPTIONS = 16;

    struct MessageTag {};
    struct RequestTag {};
    struct ResponseTag {};

public:
    SubscriptionManager() = default;
    SubscriptionManager(const SubscriptionManager&) = delete;
    SubscriptionManager& operator=(const SubscriptionManager&) = delete;

    template <typename... Adapters>
    void subscribeAll(const RegistrationManager& reg, std::tuple<Adapters...>& adapters);

    template <typename... Adapters>
    void subscribe(const CyphalSubscription* subscription, std::tuple<Adapters...>& adapters);

    // MULTI‑PORT OVERLOAD — only enabled for containers
    template <typename Tag, typename PortIDs, typename... Adapters>
    std::enable_if_t<!std::is_integral_v<PortIDs>, void>
    subscribe(const PortIDs& port_ids, std::tuple<Adapters...>& adapters)
    {
        for (auto port_id : port_ids)
            subscribe<Tag>(port_id, adapters);
    }

    // SINGLE‑PORT OVERLOAD — only enabled for integral CyphalPortID
    template <typename Tag, typename... Adapters>
    std::enable_if_t<std::is_integral_v<CyphalPortID>, void>
    subscribe(const CyphalPortID port_id, std::tuple<Adapters...>& adapters)
    {
        const CyphalSubscription* subscription = nullptr;

        if constexpr (std::is_same_v<Tag, MessageTag>)
            subscription = findMessageByPortIdRuntime(port_id);
        else if constexpr (std::is_same_v<Tag, RequestTag>)
            subscription = findRequestByPortIdRuntime(port_id);
        else if constexpr (std::is_same_v<Tag, ResponseTag>)
            subscription = findResponseByPortIdRuntime(port_id);

        if (subscription)
            subscribe(subscription, adapters);
    }

    template <typename... Adapters>
    void unsubscribe(const CyphalSubscription* subscription, std::tuple<Adapters...>& adapters);

    // MULTI‑PORT OVERLOAD — only enabled for containers
    template <typename Tag, typename PortIDs, typename... Adapters>
    std::enable_if_t<!std::is_integral_v<PortIDs>, void>
    unsubscribe(const PortIDs& port_ids, std::tuple<Adapters...>& adapters)
    {
        for (auto port_id : port_ids)
            unsubscribe<Tag>(port_id, adapters);
    }

    // SINGLE‑PORT OVERLOAD — only enabled for integral CyphalPortID
    template <typename Tag, typename... Adapters>
    std::enable_if_t<std::is_integral_v<CyphalPortID>, void>
    unsubscribe(const CyphalPortID port_id, std::tuple<Adapters...>& adapters)
    {
        const CyphalSubscription* subscription = nullptr;

        if constexpr (std::is_same_v<Tag, MessageTag>)
            subscription = findMessageByPortIdRuntime(port_id);
        else if constexpr (std::is_same_v<Tag, RequestTag>)
            subscription = findRequestByPortIdRuntime(port_id);
        else if constexpr (std::is_same_v<Tag, ResponseTag>)
            subscription = findResponseByPortIdRuntime(port_id);

        if (subscription)
            unsubscribe(subscription, adapters);
    }

    const ArrayList<const CyphalSubscription*, NUM_SUBSCRIPTIONS>&
    getSubscriptions() const { return subscriptions_; }

private:
    ArrayList<const CyphalSubscription*, NUM_SUBSCRIPTIONS> subscriptions_;
};

// -----------------------------------------------------------------------------
// IMPLEMENTATIONS
// -----------------------------------------------------------------------------

template <typename... Adapters>
void SubscriptionManager::subscribe(const CyphalSubscription* subscription,
                                    std::tuple<Adapters...>& adapters)
{
    if (subscriptions_.full())
        return;

    subscriptions_.push(subscription);

    std::apply([&](auto&... adapter) {
        ((adapter.cyphalRxSubscribe(subscription->transfer_kind,
                                    subscription->port_id,
                                    subscription->extent,
                                    1000)), ...);
    }, adapters);
}

template <typename... Adapters>
void SubscriptionManager::unsubscribe(const CyphalSubscription* subscription,
                                      std::tuple<Adapters...>& adapters)
{
    std::apply([&](auto&... adapter) {
        ((adapter.cyphalRxUnsubscribe(subscription->transfer_kind,
                                      subscription->port_id)), ...);
    }, adapters);

    subscriptions_.removeIf([&](auto* s) { return s == subscription; });
}

template <typename... Adapters>
void SubscriptionManager::subscribeAll(const RegistrationManager& reg,
                                       std::tuple<Adapters...>& adapters)
{
    subscribe<MessageTag>(reg.getSubscriptions(), adapters);
    subscribe<RequestTag>(reg.getServers(), adapters);
    subscribe<ResponseTag>(reg.getClients(), adapters);
}

#endif // INC_SubscriptionManager_HPP_
