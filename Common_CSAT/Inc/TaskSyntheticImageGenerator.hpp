#ifndef INC_TASKSYNTHETICIMAGEGENERATOR_HPP_
#define INC_TASKSYNTHETICIMAGEGENERATOR_HPP_

#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "imagebuffer/metadata.hpp"
#include "ImageBuffer.hpp"

template <typename ImageBufferT>
class TaskSyntheticImageGenerator : public Task
{
public:
    TaskSyntheticImageGenerator(ImageBufferT& buffer,
                                uint32_t payload_length,
                                uint32_t interval,
                                uint32_t tick)
        : Task(interval, tick),
          buffer_(buffer),
          payload_length_(payload_length),
          published_(false)
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
        if (published_)
            return;

        publishSyntheticImage();
        published_ = true;
    }

private:
    void publishSyntheticImage()
    {
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
    }

private:
    ImageBufferT& buffer_;
    uint32_t payload_length_;
    bool published_;
    std::vector<uint8_t> payload_;
};

#endif /* INC_TASKSYNTHETICIMAGEGENERATOR_HPP_ */
