// TaskMagnetorquer.cpp

#pragma once

#include <Task.hpp>
#include "RegistrationManager.hpp"
#include "OrientationService.hpp"
#include "MagnetorquerHardwareInterface.hpp"
#include "Logger.hpp"

#include "au.hpp"
#include "cyphal_subscriptions.hpp"
#include "nunavut_assert.h"
#include "_4111Spyglass.h"
#include "_4111spyglass/sat/solution/OrientationSolution_0_1.h"
#include "_4111spyglass/sat/solution/PositionSolution_0_1.h"

using TaskMagnetorquerBase = TaskFromBuffer<CyphalBuffer2>;
template <typename... Adapters>
class TaskMagnetorquer : public TaskMagnetorquerBase
{
public:
    TaskMagnetorquer() = delete;
    TaskMagnetorquer(const MagnetorquerSystem::Config &torquer_config, uint32_t interval, uint32_t tick, std::tuple<Adapters...>& adapters) : TaskMagnetorquerBase(interval, tick), adapters_(adapters), torquer_(torquer_config) {}

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;

    void applyMagneTorquer(std::shared_ptr<CyphalTransfer> &transfer, size_t &payload_size);
    void getQDesired(std::shared_ptr<CyphalTransfer> &transfer, size_t &payload_size);

private:
    std::tuple<Adapters...>& adapters_;
    MagnetorquerSystem torquer_;
    Eigen::Quaternionf q_desired_;
    bool valid_q_desired_ = false;
};

template <typename... Adapters>
void TaskMagnetorquer<Adapters...>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->subscribe(_4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_, task);
    manager->subscribe(_4111spyglass_sat_solution_PositionSolution_0_1_PORT_ID_, task);
}

template <typename... Adapters>
void TaskMagnetorquer<Adapters...>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unsubscribe(_4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_, task);
    manager->unsubscribe(_4111spyglass_sat_solution_PositionSolution_0_1_PORT_ID_, task);
}

template <typename... Adapters>
void TaskMagnetorquer<Adapters...>::handleTaskImpl()
{
	size_t count = this->buffer_.size();
    for (size_t i = 0; i < count; ++i)
    {
        std::shared_ptr<CyphalTransfer> transfer = this->buffer_.pop();
        size_t payload_size = transfer->payload_size;
        if (transfer->metadata.port_id == _4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_)
        {
            this->applyMagneTorquer(transfer, payload_size);
        }
        else if (transfer->metadata.port_id == _4111spyglass_sat_solution_PositionSolution_0_1_PORT_ID_)
        {
            this->getQDesired(transfer, payload_size);
        }
    }
}

template <typename... Adapters>
void TaskMagnetorquer<Adapters...>::applyMagneTorquer(std::shared_ptr<CyphalTransfer> &transfer, size_t &payload_size)
{
    if (! valid_q_desired_) return;
    _4111spyglass_sat_solution_OrientationSolution_0_1 solution{};
    auto result = _4111spyglass_sat_solution_OrientationSolution_0_1_deserialize_(&solution, (const uint8_t *)transfer->payload, &payload_size);
    log(LOG_LEVEL_DEBUG, "TaskMagnetorquer %d: %d\r\n", transfer->metadata.remote_node_id);

    if (result == NUNAVUT_SUCCESS)
    {
        MagneticField B_body(solution.magnetic_field_body.tesla[0], solution.magnetic_field_body.tesla[1], solution.magnetic_field_body.tesla[2]);
        AngularVelocity omega_measured(solution.angular_velocity_ned.radian_per_second[0], solution.angular_velocity_ned.radian_per_second[1], solution.angular_velocity_ned.radian_per_second[2]);
        Eigen::Quaternionf q_current(solution.quaternion_ned.wxyz[0],
                                     solution.quaternion_ned.wxyz[1],
                                     solution.quaternion_ned.wxyz[2],
                                     solution.quaternion_ned.wxyz[3]);
        torquer_.apply(q_current, omega_measured, q_desired_, B_body);
    }
    else
    {
        log(LOG_LEVEL_ERROR, "TaskMagnetorquer: malformed OrientationSolution payload\r\n");
    }
}

template <typename... Adapters>
void TaskMagnetorquer<Adapters...>::getQDesired(std::shared_ptr<CyphalTransfer> &transfer, size_t &payload_size)
{
    _4111spyglass_sat_solution_PositionSolution_0_1 solution{};
    auto result = _4111spyglass_sat_solution_PositionSolution_0_1_deserialize_(&solution, (const uint8_t *)transfer->payload, &payload_size);
    log(LOG_LEVEL_DEBUG, "TaskMagnetorquer %d: %d\r\n", transfer->metadata.remote_node_id);

    if (result == NUNAVUT_SUCCESS)
    {
        q_desired_ = LVLHAttitudeTarget::computeDesiredAttitudeECEF(
            {au::make_quantity<au::MetersInEcefFrame>(solution.position_ecef.meter[0]),
             au::make_quantity<au::MetersInEcefFrame>(solution.position_ecef.meter[1]),
             au::make_quantity<au::MetersInEcefFrame>(solution.position_ecef.meter[2])},
            {au::make_quantity<au::MetersPerSecondInEcefFrame>(solution.velocity_ecef.meter_per_second[0]),
             au::make_quantity<au::MetersPerSecondInEcefFrame>(solution.velocity_ecef.meter_per_second[0]),
             au::make_quantity<au::MetersPerSecondInEcefFrame>(solution.velocity_ecef.meter_per_second[0])});
        valid_q_desired_ = true;
        log(LOG_LEVEL_INFO, "TaskMagnetorquer: updated q_desired\r\n"); 
    }
    else
    {
        log(LOG_LEVEL_ERROR, "TaskMagnetorquer: malformed OrientationSolution payload\r\n");
    }
}

static_assert(containsMessageByPortIdCompileTime<_4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_>(),
              "_4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_ must be in CYPHAL_MESSAGES");

static_assert(containsMessageByPortIdCompileTime<_4111spyglass_sat_solution_PositionSolution_0_1_PORT_ID_>(),
              "_4111spyglass_sat_solution_PositionSolution_0_1_PORT_ID_ must be in CYPHAL_MESSAGES");
