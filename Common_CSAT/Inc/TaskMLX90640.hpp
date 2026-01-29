#ifndef INC_TASKMLX90640_HPP_
#define INC_TASKMLX90640_HPP_

#include <cstdint>
#include <memory>

#include "Logger.hpp"
#include "Task.hpp"
#include "MLX90640.hpp"
#include "PowerSwitch.hpp"
#include "RegistrationManager.hpp"
#include "ImageBufferConcept.hpp"
#include "Trigger.hpp"

// ─────────────────────────────────────────────
// MLX90640 Task State Machine
// ─────────────────────────────────────────────
enum class MLXState : uint8_t
{
    Off = 0,      // Cold start: sensor never powered yet
    PoweringOn,   // PowerSwitch.on() has been called
    BootDelay,    // Waiting for MLX90640 internal boot time
    Initializing, // wakeUp(), chess mode, refresh rate, etc.

    WaitCompleteFrame, // Wait a full refresh cycle (both subpages)
    WaitForReadyA,     // Waiting for NEW_DATA for subpage A
    ReadSubpageA,      // Reading subpage A

    WaitForReadyB, // Waiting for NEW_DATA for subpage B
    ReadSubpageB,  // Reading subpage B

    FrameComplete, // Both subpages acquired, frame ready

    ShuttingDown, // Putting MLX90640 into sleep mode
    PoweringOff,  // PowerSwitch.off() has been called

    Waiting, // Warm-start state, waiting for trigger
    Error    // Terminal/error state
};

// ─────────────────────────────────────────────
// MLX90640 Acquisition Modes
// ─────────────────────────────────────────────
enum class MLXMode : uint8_t
{
    OneShot,   // Acquire exactly one frame
    Burst,     // Acquire N frames
    Continuous // Acquire frames indefinitely
};

// ─────────────────────────────────────────────
// TaskMLX90640
// ─────────────────────────────────────────────
template <typename PowerSwitchT, typename MLXT, ImageBufferConcept ImageBufferT, typename TriggerT = OnceTrigger>
class TaskMLX90640 : public Task, private TaskPacing
{
public:
    TaskMLX90640(PowerSwitchT &pwr, CIRCUITS circuit, MLXT &mlx, ImageBufferT &buffer, TriggerT &trigger,
                 MLXMode mode, uint32_t burstCount, uint32_t sleep_interval, uint32_t operate_interval, uint32_t tick)
        : Task(sleep_interval, tick), TaskPacing(sleep_interval, operate_interval),
          power_(pwr),
          circuit_(circuit),
          sensor_(mlx),
          image_buffer_(buffer),
          trigger_(trigger),
          t0_(0),
          state_(MLXState::Off),
          mode_(mode),
          burstCount_(burstCount),
          burstRemaining_(burstCount),
          spA_(-1),
          spB_(-1)
    {
    }

    virtual ~TaskMLX90640() = default;

    void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override
    {
        manager->subscribe(PURE_HANDLER, task);
    }
    void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override
    {
        manager->unsubscribe(PURE_HANDLER, task);
    }

    MLXState getState() const { return state_; }
    MLXMode getMode() const { return mode_; }
    uint32_t getBurstRemaining() const { return burstRemaining_; }

protected:
    void handleTaskImpl() override
    {
        log(LOG_LEVEL_DEBUG, "TaskMLX90640 handled in state %d\r\n", static_cast<uint8_t>(state_));

        switch (state_)
        {
        case MLXState::Off:
            stateOff();
            break;
        case MLXState::PoweringOn:
            statePoweringOn();
            break;
        case MLXState::BootDelay:
            stateBootDelay();
            break;
        case MLXState::Initializing:
            stateInitialize();
            break;
        case MLXState::WaitCompleteFrame:
            stateWaitCompleteFrame();
            break;
        case MLXState::WaitForReadyA:
            stateWaitingForReadyA();
            break;
        case MLXState::ReadSubpageA:
            stateReadSubpageA();
            break;
        case MLXState::WaitForReadyB:
            stateWaitForReadyB();
            break;
        case MLXState::ReadSubpageB:
            stateReadSubpageB();
            break;
        case MLXState::FrameComplete:
            stateFrameComplete();
            break;
        case MLXState::ShuttingDown:
            stateShuttingDown();
            break;
        case MLXState::PoweringOff:
            statePoweringOff();
            break;
        case MLXState::Waiting:
            stateWaiting();
            break;
        case MLXState::Error:
            stateError();
            break;
        }
    }

private:
    // ─────────────────────────────────────────────
    // Trigger logic: Only Off and Waiting may start
    // ─────────────────────────────────────────────
    bool shouldStart()
    {
        if (state_ != MLXState::Off && state_ != MLXState::Waiting)
            return false;

        return trigger_.trigger();
    }

    // ─────────────────────────────────────────────
    // Shared start logic for Off and Waiting
    // ─────────────────────────────────────────────
    void tryStart()
    {
        if (!shouldStart())
            return;

        if (!power_.on(circuit_))
        {
            log(LOG_LEVEL_ERROR, "TaskMLX90640: power_.on() failed\r\n");
            state_ = MLXState::Error;
            return;
        }

        t0_ = HAL_GetTick();
        burstRemaining_ = burstCount_;
        state_ = MLXState::PoweringOn;
    }

    // ─────────────────────────────────────────────
    // State handlers
    // ─────────────────────────────────────────────
    void stateOff() { tryStart(); }
    void stateWaiting() { tryStart(); }

    void stateError()
    {
        // Terminal state: do nothing
    }

    void statePoweringOn()
    {
        state_ = MLXState::BootDelay;
    }

    void stateBootDelay()
    {
        if (HAL_GetTick() - t0_ >= TASK_BOOT_DELAY_MS)
            state_ = MLXState::Initializing;
    }

    void stateInitialize()
    {
        if (sensor_.wakeUp(REFRESH_RATE))
        {
            state_ = MLXState::WaitCompleteFrame;
            TaskPacing::operate(*this);
            t0_ = HAL_GetTick();
        }
        else
        {
            log(LOG_LEVEL_ERROR, "TaskMLX90640: wakeUp() failed\r\n");
            state_ = MLXState::Error;
        }
    }

    void stateWaitCompleteFrame()
    {
        if ((HAL_GetTick() - t0_) >= REFRESH_INTERVAL)
            state_ = MLXState::ReadSubpageA;
    }

    void stateWaitingForReadyA()
    {
        if (((HAL_GetTick() - t0_) >= REFRESH_INTERVAL_2) && sensor_.isReady())
        {
            state_ = MLXState::ReadSubpageA;
        }
    }

    void stateReadSubpageA()
    {
        if (sensor_.readSubpage(subA_, spA_))
        {
            // spA_ is now 0 or 1 (or something bogus)
            if (spA_ != 0 && spA_ != 1)
            {
                log(LOG_LEVEL_ERROR, "TaskMLX90640: readSubpage A returned invalid subpage=%d\r\n", spA_);
                state_ = MLXState::Error;
                return;
            }

            // We have one subpage stored; wait for the other one.
            state_ = MLXState::WaitForReadyB;
            t0_ = HAL_GetTick();
        }
        else
        {
            log(LOG_LEVEL_ERROR, "TaskMLX90640: readSubpage A failed\r\n");
            state_ = MLXState::Error;
        }
    }

    void stateWaitForReadyB()
    {
        if (((HAL_GetTick() - t0_) >= REFRESH_INTERVAL_2) && sensor_.isReady())
        {
            state_ = MLXState::ReadSubpageB;
        }
    }

    void stateReadSubpageB()
    {
        int new_sp = -1;
        if (sensor_.readSubpage(subB_, new_sp))
        {
            if (new_sp != 0 && new_sp != 1)
            {
                log(LOG_LEVEL_ERROR, "TaskMLX90640: readSubpage B returned invalid subpage=%d\r\n", new_sp);
                state_ = MLXState::Error;
                return;
            }

            if (new_sp == spA_)
            {
                // We got the same subpage as before:
                // treat this as the start of a new frame, and keep this as the new A.
                log(LOG_LEVEL_WARNING,
                    "TaskMLX90640: same subpage twice (sp=%d) - restarting pair\r\n",
                    new_sp);

                // Move B -> A, just to reuse the buffers.
                std::memcpy(subA_, subB_, sizeof(subA_));
                spA_ = new_sp;

                // Wait for the other subpage again.
                state_ = MLXState::WaitForReadyB;
                t0_ = HAL_GetTick();
                return;
            }

            // We finally have two different subpages in this frame: spA_ and new_sp.
            spB_ = new_sp;
            state_ = MLXState::FrameComplete;
            t0_ = HAL_GetTick();
        }
        else
        {
            log(LOG_LEVEL_ERROR, "TaskMLX90640: readSubpage B failed\r\n");
            state_ = MLXState::Error;
        }
    }

    void stateFrameComplete()
    {
        if (spA_ != spB_ && spA_ >= 0 && spB_ >= 0)
        {
            if (spA_ == 0)
                sensor_.createFrame(subA_, subB_, frame_);
            else
                sensor_.createFrame(subB_, subA_, frame_);

            publishFrame();
        }
        else
        {
            log(LOG_LEVEL_WARNING,
                "TaskMLX90640: invalid subpage pair spA=%d spB=%d - retrying\r\n",
                spA_, spB_);

            // Reset subpage tracking and try again.
            spA_ = -1;
            spB_ = -1;

            state_ = MLXState::WaitForReadyA;
            t0_    = HAL_GetTick();
            return;
        }

        if (mode_ == MLXMode::OneShot)
        {
            state_ = MLXState::ShuttingDown;
        }
        else if (mode_ == MLXMode::Burst)
        {
            if (burstRemaining_ > 0)
                burstRemaining_--;

            if (burstRemaining_ == 0)
                state_ = MLXState::ShuttingDown;
            else
                state_ = MLXState::WaitForReadyA;
        }
        else // Continuous
        {
            state_ = MLXState::WaitForReadyA;
        }
    }

    void stateShuttingDown()
    {
        sensor_.sleep();
        state_ = MLXState::PoweringOff;
        TaskPacing::sleep(*this);
    }

    void statePoweringOff()
    {
        power_.off(circuit_);
        state_ = MLXState::Waiting; // Successful cycle → Waiting
    }

    void publishFrame()
    {
        ImageMetadata meta{};
        meta.timestamp = HAL_GetTick();
        meta.payload_size = MLX90640_FRAME_WORDS * sizeof(uint16_t);
        meta.latitude = 0.0f;
        meta.longitude = 0.0f;
        meta.producer = METADATA_PRODUCER::CAMERA_1;
        meta.format = METADATA_FORMAT::UNKN;

        log(LOG_LEVEL_INFO, "MLX90640: Publishing frame to ImageBuffer\r\n");
        if (image_buffer_.add_image(meta) != ImageBufferError::NO_ERROR)
        {
            log(LOG_LEVEL_ERROR, "MLX90640: add_image() failed\r\n");
            return;
        }

        const uint8_t *bytes = reinterpret_cast<const uint8_t *>(frame_);
        size_t remaining = meta.payload_size;

        while (remaining > 0)
        {
            size_t chunk = remaining;
            if (image_buffer_.add_data_chunk(bytes, chunk) != ImageBufferError::NO_ERROR)
            {
                log(LOG_LEVEL_ERROR, "MLX90640: add_data_chunk() failed\r\n");
                return;
            }
            bytes += chunk;
            remaining -= chunk;
        }

        if (image_buffer_.push_image() != ImageBufferError::NO_ERROR)
        {
            log(LOG_LEVEL_ERROR, "MLX90640: push_image() failed\r\n");
            return;
        }

        log(LOG_LEVEL_DEBUG, "MLX90640: frame stored in ImageBuffer\r\n");
    }

private:
    PowerSwitchT &power_;
    CIRCUITS circuit_;
    MLXT &sensor_;
    ImageBufferT &image_buffer_;
    TriggerT &trigger_;

    uint32_t t0_;

    MLXState state_;
    MLXMode mode_;
    uint32_t burstCount_;
    uint32_t burstRemaining_;

    uint16_t subA_[MLX90640_SUBPAGE_WORDS];
    uint16_t subB_[MLX90640_SUBPAGE_WORDS];
    uint16_t frame_[MLX90640_FRAME_WORDS];
    int spA_;
    int spB_;

    constexpr static MLX90640_RefreshRate REFRESH_RATE = MLX90640_RefreshRate::Hz4;
    constexpr static uint32_t REFRESH_INTERVAL = getRefreshIntervalMs(REFRESH_RATE);
    constexpr static uint32_t REFRESH_INTERVAL_2 = REFRESH_INTERVAL/2;
    constexpr static uint32_t TASK_BOOT_DELAY_MS = MLX90640_BOOT_TIME_MS;
};

#endif /* INC_TASKMLX90640_HPP_ */
