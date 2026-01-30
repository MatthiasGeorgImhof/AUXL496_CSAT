#ifndef INC_TASKSYNTHETICIMAGEGENERATOR_HPP_
#define INC_TASKSYNTHETICIMAGEGENERATOR_HPP_

#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "ImageBufferConcept.hpp"
#include "Trigger.hpp"
#include "Logger.hpp"

template <ImageBufferConcept ImageBufferT, typename TriggerT = OnceTrigger>
class TaskSyntheticImageGenerator : public Task
{
public:
    TaskSyntheticImageGenerator(ImageBufferT& buffer,
                                TriggerT trigger,
                                uint32_t payload_length,
                                uint32_t interval,
                                uint32_t tick)
        : Task(interval, tick),
          buffer_(buffer),
          trigger_(trigger),
          payload_length_(payload_length)
    {
        payload_.resize(payload_length_);
        for (uint32_t i = 0; i < payload_length_; i++)
            payload_[i] = static_cast<uint8_t>(i);
    }

    void registerTask(RegistrationManager* manager, std::shared_ptr<Task> task) override
    {
        manager->subscribe(PURE_HANDLER, task);
    }

    void unregisterTask(RegistrationManager* manager, std::shared_ptr<Task> task) override
    {
        manager->unsubscribe(PURE_HANDLER, task);
    }

protected:
    void handleTaskImpl() override
    {
        // Only generate a frame when the trigger fires
        if (!trigger_.trigger())
            return;

        publishSyntheticImage();
    }

private:
    void publishSyntheticImage()
    {
        // Respect buffer capacity contract
        log(LOG_LEVEL_DEBUG, "TaskSyntheticImageGenerator::publishSyntheticImage %d\r\n", HAL_GetTick());
    	if (!buffer_.has_room_for(payload_length_))
            return;

        ImageMetadata meta{};
        meta.timestamp    = HAL_GetTick();
        meta.payload_size = payload_length_;
        meta.latitude     = 0.0f;
        meta.longitude    = 0.0f;
        meta.producer     = METADATA_PRODUCER::CAMERA_1;
        meta.format       = METADATA_FORMAT::UNKN;

        if (buffer_.add_image(meta) != ImageBufferError::NO_ERROR)
            return;

        const uint8_t* bytes = payload_.data();
        size_t remaining = payload_length_;

        while (remaining > 0)
        {
            size_t chunk = remaining;
            if (buffer_.add_data_chunk(bytes, chunk) != ImageBufferError::NO_ERROR)
                return;

            bytes += chunk;
            remaining -= chunk;
        }

        buffer_.push_image();
        log(LOG_LEVEL_DEBUG, "TaskSyntheticImageGenerator::publishSyntheticImage pushed image\r\n");
    }

private:
    ImageBufferT& buffer_;
    TriggerT trigger_;
    uint32_t payload_length_;
    std::vector<uint8_t> payload_;
};

#endif /* INC_TASKSYNTHETICIMAGEGENERATOR_HPP_ */
