#ifndef INC_TASKPOSITIONSERVICE_HPP_
#define INC_TASKPOSITIONSERVICE_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "Logger.hpp"
#include "au.hpp"

#include "nunavut_assert.h"
#include "_4111Spyglass.h"
#include "_4111spyglass/sat/model/PositionVelocity_0_1.h"
#include <cstring>
#include <chrono>
#include <cstdint>
#include <array>

template <typename Tracker, typename... Adapters>
class TaskPositionService : public TaskWithPublication<Adapters...>
{
public:
    TaskPositionService() = delete;
    TaskPositionService(Tracker &tracker, uint32_t interval, uint32_t tick, CyphalTransferID transfer_id, std::tuple<Adapters...> &adapters) : TaskWithPublication<Adapters...>(interval, tick, transfer_id, adapters), tracker_(tracker) {}

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;

private:
    Tracker &tracker_;
};

template <typename Tracker, typename... Adapters>
void TaskPositionService<Tracker, Adapters...>::handleTaskImpl()
{
    std::array<au::QuantityF<au::MetersInEcefFrame>, 3> r;
    std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> v;
    au::QuantityU64<au::Milli<au::Seconds>> timestamp;
    if (!tracker_.predict(r, v, timestamp))
    {
        return;
    };

    _4111spyglass_sat_model_PositionVelocity_0_1 data;
    data.timestamp.microsecond = timestamp.in(au::micro(au::seconds));
    std::transform(std::begin(r), std::end(r), std::begin(data.position_m), [](const auto &item) { return item.in(au::meters * au::ecefs); });
    std::transform(std::begin(v), std::end(v), std::begin(data.velocity_ms), [](const auto &item) { return item.in(au::meters * au::ecefs / au::second); });

    constexpr size_t PAYLOAD_SIZE = _4111spyglass_sat_model_PositionVelocity_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t payload[PAYLOAD_SIZE];

    TaskPositionService<Tracker, Adapters...>::publish(PAYLOAD_SIZE, payload, &data,
                                   reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(_4111spyglass_sat_model_PositionVelocity_0_1_serialize_),
                                   _4111spyglass_sat_model_PositionVelocity_0_1_PORT_ID_);
}

template <typename Tracker, typename... Adapters>
void TaskPositionService<Tracker, Adapters...>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->publish(_4111spyglass_sat_model_PositionVelocity_0_1_PORT_ID_, task);
}

template <typename Tracker, typename... Adapters>
void TaskPositionService<Tracker, Adapters...>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unpublish(_4111spyglass_sat_model_PositionVelocity_0_1_PORT_ID_, task);
}

#endif // INC_TASKPOSITIONSERVICE_HPP_
