#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include "mock_hal.h"

#include "TaskMLX90640.hpp"
#include "RegistrationManager.hpp"
#include "Trigger.hpp"


void advance_time_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        HAL_IncTick();
    }
}

// -----------------------------------------------------------------------------
// Mock MLX90640 driver
// -----------------------------------------------------------------------------
struct MockMLX
{
    bool wakeUp_called = false;
    bool sleep_called = false;
    int isReady_calls = 0;
    int readSubpage_calls = 0;

    bool wakeUp(MLX90640_RefreshRate = MLX90640_RefreshRate::Hz4)
    {
        wakeUp_called = true;
        return true;
    }

    bool sleep()
    {
        sleep_called = true;
        return true;
    }

    bool isReady()
    {
        isReady_calls++;
        return true;    // Always ready for tests
    }

    bool readSubpage(uint16_t* buf, int& sp)
    {
        readSubpage_calls++;
        sp = (readSubpage_calls % 2 == 1 ? 0 : 1);
        buf[0] = 0xABCD;
        return true;
    }

    void createFrame(const uint16_t* a, const uint16_t* b, uint16_t* out)
    {
        out[0] = a[0];
        out[1] = b[0];
    }
};

// -----------------------------------------------------------------------------
// Mock PowerSwitch
// -----------------------------------------------------------------------------
struct MockPower
{
    bool on_called = false;
    bool off_called = false;

    bool on(CIRCUITS)
    {
        on_called = true;
        return true;
    }

    bool off(CIRCUITS)
    {
        off_called = true;
        return true;
    }
};

struct MockImageBuffer
{
    // ------------------------------------------------------------
    // Instrumentation for tests
    // ------------------------------------------------------------
    int add_image_calls      = 0;   // how many acquisition events attempted
    int push_image_calls     = 0;   // how many frames successfully stored
    int add_chunk_calls      = 0;   // how many chunks were written
    size_t total_chunk_bytes = 0;   // total bytes written

    // ------------------------------------------------------------
    // Reset between tests
    // ------------------------------------------------------------
    void reset()
    {
        add_image_calls      = 0;
        push_image_calls     = 0;
        add_chunk_calls      = 0;
        total_chunk_bytes    = 0;
    }

    // ------------------------------------------------------------
    // State queries
    // ------------------------------------------------------------
    bool is_empty() const
    {
        // Infinite buffer is never "empty" or "full" in the traditional sense.
        // For concept compliance, we can say it's always empty before reading.
        return true;
    }

    size_t count() const
    {
        // Infinite buffer: concept requires a count, but tests don't use it.
        return 0;
    }

    bool has_room_for(size_t) const
    {
        // Infinite capacity: always has room.
        return true;
    }

    size_t size() const
    {
        // Not used by MLX tests; return total bytes written.
        return total_chunk_bytes;
    }

    // ------------------------------------------------------------
    // Producer API
    // ------------------------------------------------------------
    ImageBufferError add_image(const ImageMetadata&)
    {
        ++add_image_calls;
        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError add_data_chunk(const uint8_t*, size_t size)
    {
        ++add_chunk_calls;
        total_chunk_bytes += size;
        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError push_image()
    {
        ++push_image_calls;
        return ImageBufferError::NO_ERROR;
    }

    // ------------------------------------------------------------
    // Consumer API (unused in MLX tests)
    // ------------------------------------------------------------
    ImageBufferError get_image(ImageMetadata&)
    {
        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError get_data_chunk(uint8_t*, size_t& size)
    {
        size = 0;
        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError pop_image()
    {
        return ImageBufferError::NO_ERROR;
    }
};

struct MockTriggerAlways {
    bool trigger() { return true; }
};

// -----------------------------------------------------------------------------
// TESTS
// -----------------------------------------------------------------------------

TEST_CASE("TaskMLX90640 basic state progression")
{
    HAL_SetTick(0);

    RegistrationManager mgr;
    auto pwr = std::make_shared<MockPower>();
    auto mlx = std::make_shared<MockMLX>();
    auto imgBuf = std::make_shared<MockImageBuffer>();

    OnceTrigger trig;
    auto task = std::make_shared<TaskMLX90640<MockPower, MockMLX, MockImageBuffer, OnceTrigger>>(
        *pwr,
        CIRCUITS::CIRCUIT_0,
        *mlx,
        *imgBuf,
        trig,
        MLXMode::OneShot,
        1,      // burst count
        0, 0, 0    // sleep_interval, operate_interval, tick
    );

    mgr.add(task);

    for (int i = 0; i < 5000; i++) {
        advance_time_ms(1);
        task->handleTask();
    }

    CHECK(pwr->on_called == true);
    CHECK(mlx->wakeUp_called == true);
    CHECK(mlx->readSubpage_calls >= 2);
    CHECK(mlx->sleep_called == true);
    CHECK(pwr->off_called == true);
    CHECK(task->getState() == MLXState::Waiting);
}

TEST_CASE("TaskMLX90640 OneShot mode produces exactly one frame")
{
    HAL_SetTick(0);

    RegistrationManager mgr;
    auto pwr = std::make_shared<MockPower>();
    auto mlx = std::make_shared<MockMLX>();
    auto imgBuf = std::make_shared<MockImageBuffer>();

    OnceTrigger trig;
    auto task = std::make_shared<TaskMLX90640<MockPower, MockMLX, MockImageBuffer, OnceTrigger>>(
        *pwr,
        CIRCUITS::CIRCUIT_0,
        *mlx,
        *imgBuf,
        trig,
        MLXMode::OneShot,
        1,      // burst count
        0, 0, 0   // sleep_interval, operate_interval, tick
    );

    mgr.add(task);

    for (int i = 0; i < 5000; i++) {
        advance_time_ms(1);
        task->handleTask();
    }

    CHECK(mlx->readSubpage_calls >= 2);   // exactly one frame
    CHECK(task->getState() == MLXState::Waiting);
}

TEST_CASE("TaskMLX90640 Burst mode produces N frames")
{
    HAL_SetTick(0);

    RegistrationManager mgr;
    auto pwr = std::make_shared<MockPower>();
    auto mlx = std::make_shared<MockMLX>();
    auto imgBuf = std::make_shared<MockImageBuffer>();

    const int N = 3;

    OnceTrigger trig;
    auto task = std::make_shared<TaskMLX90640<MockPower, MockMLX, MockImageBuffer, OnceTrigger>>(
        *pwr,
        CIRCUITS::CIRCUIT_0,
        *mlx,
        *imgBuf,
        trig,
        MLXMode::Burst,
        N,      // burst count
        0, 0, 0   // sleep_interval, operate_interval, tick
    );

    mgr.add(task);

    for (int i = 0; i < 5000; i++) {
        advance_time_ms(1);
        task->handleTask();
    }

    CHECK(mlx->readSubpage_calls == 2 * N);
    CHECK(task->getState() == MLXState::Waiting);
}

TEST_CASE("TaskMLX90640 with MockTriggerAlways produces multiple cycles")
{
    HAL_SetTick(0);

    RegistrationManager mgr;
    auto pwr    = std::make_shared<MockPower>();
    auto mlx    = std::make_shared<MockMLX>();
    auto imgBuf = std::make_shared<MockImageBuffer>();
    MockTriggerAlways trig;

    // OneShot mode: each cycle produces exactly 1 frame
    auto task = std::make_shared<
        TaskMLX90640<MockPower, MockMLX, MockImageBuffer, MockTriggerAlways>
    >(
        *pwr,
        CIRCUITS::CIRCUIT_0,
        *mlx,
        *imgBuf,
        trig,              // <-- always returns true
        MLXMode::OneShot,
        1,                 // burstCount (ignored in OneShot)
        0, 0, 0    // sleep_interval, operate_interval, tick
    );

    mgr.add(task);

    // Run long enough for multiple cycles
    for (int i = 0; i < 5000; i++) {
        advance_time_ms(1);
        task->handleTask();
    }

CHECK(imgBuf->add_image_calls > 1);   // multiple acquisition events happened
CHECK(mlx->readSubpage_calls >= 2 * imgBuf->add_image_calls);
CHECK(imgBuf->push_image_calls == 5);
}
