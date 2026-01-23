#ifndef INC_TASKPOSITIONSERVICE_HPP_
#define INC_TASKPOSITIONSERVICE_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "Logger.hpp"
#include "au.hpp"
#include "PositionService.hpp"

#include "nunavut_assert.h"
#include "_4111Spyglass.h"
#include "_4111spyglass/sat/solution/PositionSolution_0_1.h"
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
    PositionSolution solution = tracker_.predict();

    _4111spyglass_sat_solution_PositionSolution_0_1 data;
    data.timestamp.microsecond = solution.timestamp.in(au::micro(au::seconds));
    data.position_ecef.meter[0] = solution.position[0].in(au::metersInEcefFrame);
    data.position_ecef.meter[1] = solution.position[1].in(au::metersInEcefFrame);
    data.position_ecef.meter[2] = solution.position[2].in(au::metersInEcefFrame);
    data.velocity_ecef.meter_per_second[0] = solution.velocity[0].in(au::metersPerSecondInEcefFrame);
    data.velocity_ecef.meter_per_second[1] = solution.velocity[1].in(au::metersPerSecondInEcefFrame);
    data.velocity_ecef.meter_per_second[2] = solution.velocity[2].in(au::metersPerSecondInEcefFrame);
    data.acceleration_ecef.meter_per_second_per_second[0] = solution.acceleration[0].in(au::metersPerSecondSquaredInEcefFrame);
    data.acceleration_ecef.meter_per_second_per_second[1] = solution.acceleration[1].in(au::metersPerSecondSquaredInEcefFrame);
    data.acceleration_ecef.meter_per_second_per_second[2] = solution.acceleration[2].in(au::metersPerSecondSquaredInEcefFrame);
    data.valid_position = solution.has_valid(PositionSolution::Validity::POSITION);
    data.valid_velocity = solution.has_valid(PositionSolution::Validity::VELOCITY);
    data.valid_acceleration = solution.has_valid(PositionSolution::Validity::ACCELERATION);

    log(LOG_LEVEL_DEBUG, "TaskPositionService %f %f %f\r\n", data.position_ecef.meter[0], data.position_ecef.meter[1], data.position_ecef.meter[2]);

    constexpr size_t PAYLOAD_SIZE = _4111spyglass_sat_solution_PositionSolution_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t payload[PAYLOAD_SIZE];

    TaskPositionService<Tracker, Adapters...>::publish(PAYLOAD_SIZE, payload, &data,
                                                       reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(_4111spyglass_sat_solution_PositionSolution_0_1_serialize_),
                                                       _4111spyglass_sat_solution_PositionSolution_0_1_PORT_ID_);
}

template <typename Tracker, typename... Adapters>
void TaskPositionService<Tracker, Adapters...>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->publish(_4111spyglass_sat_solution_PositionSolution_0_1_PORT_ID_, task);
}

template <typename Tracker, typename... Adapters>
void TaskPositionService<Tracker, Adapters...>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unpublish(_4111spyglass_sat_solution_PositionSolution_0_1_PORT_ID_, task);
}

#endif // INC_TASKPOSITIONSERVICE_HPP_
