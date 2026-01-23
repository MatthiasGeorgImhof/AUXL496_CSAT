#ifndef INC_TASKORIENTATIONSERVICE_HPP_
#define INC_TASKORIENTATIONSERVICE_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "OrientationService.hpp"
#include "Logger.hpp"
#include "au.hpp"

#include "nunavut_assert.h"
#include "_4111Spyglass.h"
#include "_4111spyglass/sat/solution/OrientationSolution_0_1.h"
#include <cmath>
#include <cstring>
#include <chrono>
#include <cstdint>
#include <array>
#include <numbers>

template <typename Tracker, typename... Adapters>
class TaskOrientationService : public TaskWithPublication<Adapters...>
{
public:
    TaskOrientationService() = delete;
    TaskOrientationService(Tracker &tracker, uint32_t interval, uint32_t tick, CyphalTransferID transfer_id, std::tuple<Adapters...> &adapters) : TaskWithPublication<Adapters...>(interval, tick, transfer_id, adapters), tracker_(tracker) {}

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;

private:
    Tracker &tracker_;
};

template <typename Tracker, typename... Adapters>
void TaskOrientationService<Tracker, Adapters...>::handleTaskImpl()
{
    OrientationSolution solution = tracker_.predict();
    _4111spyglass_sat_solution_OrientationSolution_0_1 data{};
    data.timestamp.microsecond = solution.timestamp.in(au::micro(au::seconds));

    if (solution.validity_flags & static_cast<uint8_t>(OrientationSolution::Validity::QUATERNION))
    {
        data.quaternion_ned.wxyz[0] = solution.q[0];
        data.quaternion_ned.wxyz[1] = solution.q[1];
        data.quaternion_ned.wxyz[2] = solution.q[2];
        data.quaternion_ned.wxyz[3] = solution.q[3];
        data.valid_quaternion = true;

        auto orientation = getEulerAngles(solution.q);
        log(LOG_LEVEL_DEBUG, "TaskOrientationService %f %f %f\r\n", orientation[0].in(au::degreesInNedFrame), orientation[1].in(au::degreesInNedFrame), orientation[2].in(au::degreesInNedFrame));
        data.yaw_ned.radian = orientation[0].in(au::radiansInNedFrame);
        data.pitch_ned.radian = orientation[1].in(au::radiansInNedFrame);
        data.roll_ned.radian = orientation[2].in(au::radiansInNedFrame);
        data.valid_yaw_pitch_roll = true;
    }

    if (solution.validity_flags & static_cast<uint8_t>(OrientationSolution::Validity::ANGULAR_VELOCITY))
    {
        data.angular_velocity_ned.radian_per_second[0] = solution.angular_velocity[0].in(au::radiansPerSecondInBodyFrame);
        data.angular_velocity_ned.radian_per_second[1] = solution.angular_velocity[1].in(au::radiansPerSecondInBodyFrame);
        data.angular_velocity_ned.radian_per_second[2] = solution.angular_velocity[2].in(au::radiansPerSecondInBodyFrame);
        data.valid_angular_velocity = true;
    }

    if (solution.validity_flags & static_cast<uint8_t>(OrientationSolution::Validity::MAGNETIC_FIELD))
    {
        data.magnetic_field_body.tesla[0] = solution.magnetic_field[0].in(au::micro(au::teslaInBodyFrame));
        data.magnetic_field_body.tesla[1] = solution.magnetic_field[1].in(au::micro(au::teslaInBodyFrame));
        data.magnetic_field_body.tesla[2] = solution.magnetic_field[2].in(au::micro(au::teslaInBodyFrame));
        data.valid_magnetic_field = true;
    }

    constexpr size_t PAYLOAD_SIZE = _4111spyglass_sat_solution_OrientationSolution_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t payload[PAYLOAD_SIZE];

    TaskOrientationService<Tracker, Adapters...>::publish(PAYLOAD_SIZE, payload, &data,
                                                          reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(_4111spyglass_sat_solution_OrientationSolution_0_1_serialize_),
                                                          _4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_);
}

template <typename Tracker, typename... Adapters>
void TaskOrientationService<Tracker, Adapters...>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->publish(_4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_, task);
}

template <typename Tracker, typename... Adapters>
void TaskOrientationService<Tracker, Adapters...>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unpublish(_4111spyglass_sat_solution_OrientationSolution_0_1_PORT_ID_, task);
}

#endif // INC_TASKORIENTATIONSERVICE_HPP_
