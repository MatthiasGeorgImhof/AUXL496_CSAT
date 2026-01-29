#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "TaskSyntheticImageGenerator.hpp"
#include "TrivialImageBuffer.hpp"
#include "InputOutputStream.hpp"
#include "RegistrationManager.hpp"
#include "Trigger.hpp"

#include "mock_hal.h"

// Stub RegistrationManager methods
void RegistrationManager::subscribe(const CyphalPortID, std::shared_ptr<Task>) {}
void RegistrationManager::subscribe(const CyphalPortID) {}
void RegistrationManager::unsubscribe(const CyphalPortID, std::shared_ptr<Task>) {}
void RegistrationManager::publish(const CyphalPortID, std::shared_ptr<Task>) {}
void RegistrationManager::publish(const CyphalPortID) {}
void RegistrationManager::unpublish(const CyphalPortID, std::shared_ptr<Task>) {}
void RegistrationManager::client(const CyphalPortID, std::shared_ptr<Task>) {}
void RegistrationManager::client(const CyphalPortID) {}
void RegistrationManager::unclient(const CyphalPortID, std::shared_ptr<Task>) {}
void RegistrationManager::server(const CyphalPortID, std::shared_ptr<Task>) {}
void RegistrationManager::server(const CyphalPortID) {}
void RegistrationManager::unserver(const CyphalPortID, std::shared_ptr<Task>) {}

using Buffer = TrivialImageBuffer<8192>;

// ------------------------------------------------------------
// Test: Synthetic task produces one image and stream reads it
// ------------------------------------------------------------
TEST_CASE("SyntheticImageGenerator basic pipeline")
{
    HAL_SetTick(0);

    // 1. Buffer
    Buffer buf;

    // 2. Synthetic task: generate payload {0,1,2,...,15}
    constexpr uint32_t N = 16;
    OnceTrigger trig;   // <-- NEW: required trigger

    TaskSyntheticImageGenerator<Buffer, OnceTrigger> gen(
        buf,
        trig,       // <-- NEW: trigger argument
        N,          // payload length
        0,          // interval
        0           // tick
    );

    // 3. Before running task: buffer must be empty
    CHECK(buf.is_empty());

    // 4. Run the synthetic task once → produces exactly one image
    gen.handleTask();
    CHECK(!buf.is_empty());

    // 5. Stream adapter
    ImageInputStream<Buffer> stream(buf);
    CHECK(!stream.is_empty());

    // 6. Read metadata via initialize()
    uint8_t meta_buf[sizeof(ImageMetadata)];
    size_t meta_size = sizeof(meta_buf);

    CHECK(stream.initialize(meta_buf, meta_size));
    CHECK(meta_size == sizeof(ImageMetadata));

    ImageMetadata meta{};
    std::memcpy(&meta, meta_buf, sizeof(ImageMetadata));

    CHECK(meta.payload_size == N);

    // 7. Read payload in chunks
    uint8_t readback[N];
    size_t offset = 0;

    while (offset < N)
    {
        uint8_t chunk[8];
        size_t chunk_size = sizeof(chunk);

        CHECK(stream.getChunk(chunk, chunk_size));

        for (size_t i = 0; i < chunk_size; i++)
            readback[offset + i] = chunk[i];

        offset += chunk_size;
    }

    // 8. Finalize (size=0)
    size_t zero = 0;
    CHECK(stream.getChunk(nullptr, zero));

    // 9. Validate payload content
    for (uint32_t i = 0; i < N; i++)
        CHECK(readback[i] == static_cast<uint8_t>(i));

    // 10. Stream and buffer should now be empty
    CHECK(stream.is_empty());
    CHECK(buf.is_empty());
}
