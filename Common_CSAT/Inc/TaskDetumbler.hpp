// TaskDetumbler.hpp

#pragma once

#include <Task.hpp>
#include "RegistrationManager.hpp"
#include "OrientationService.hpp"
#include "MagneticBDotController.hpp"
#include "Logger.hpp"

#include "au.hpp"
#include "cyphal_subscriptions.hpp"
#include "nunavut_assert.h"
#include "_4111Spyglass.h"
#include "_4111spyglass/sat/solution/OrientationSolution_0_1.h"


using TaskDetumblerBase = TaskFromBuffer<CyphalBuffer1>;
template <typename... Adapters>
class TaskDetumbler : public TaskDetumblerBase
{
public:
    TaskDetumbler() = delete;
    TaskDetumbler(const DetumblerSystem::Config &detumbler_config, uint32_t interval, uint32_t tick, std::tuple<Adapters...>& adapters) : TaskDetumblerBase(interval, tick), adapters_(adapters), detumbler_(detumbler_config) {}

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;

private:
    std::tuple<Adapters...>& adapters_;
    DetumblerSystem detumbler_;
};

template <typename... Adapters>
void TaskDetumbler<Adapters...>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->subscribe(_4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_, task);
}

template <typename... Adapters>
void TaskDetumbler<Adapters...>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unsubscribe(_4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_, task);
}

template <typename... Adapters>
void TaskDetumbler<Adapters...>::handleTaskImpl()
{
    if (this->buffer_.size() == 1)
    {
        std::shared_ptr<CyphalTransfer> transfer = this->buffer_.pop();
        size_t payload_size = transfer->payload_size;
        _4111spyglass_sat_solution_OrientationSolution_0_1 solution{};
        auto result = _4111spyglass_sat_solution_OrientationSolution_0_1_deserialize_(&solution, (const uint8_t *) transfer->payload, &payload_size);
		log(LOG_LEVEL_DEBUG, "TaskDetumbler %d: %d\r\n", transfer->metadata.remote_node_id);

        if (result==NUNAVUT_SUCCESS)
        {
            MagneticField B_body(solution.magnetic_field_body.tesla[0], solution.magnetic_field_body.tesla[1], solution.magnetic_field_body.tesla[2]);
            auto timestamp = au::make_quantity<au::Milli<au::Seconds>>(solution.timestamp.microsecond/1000);
            detumbler_.apply(B_body, timestamp);
        }
        else
        {
            log(LOG_LEVEL_ERROR, "TaskDetumbler: malformed OrientationSolution payload\r\n");
        }
    }
}

static_assert(containsMessageByPortIdCompileTime<_4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_>(),
              "_4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_ must be in CYPHAL_MESSAGES");

