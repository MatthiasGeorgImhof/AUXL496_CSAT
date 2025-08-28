#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "mock_hal.h" // No conditional include needed

#include "TaskCheckMemory.hpp"
#include "o1heap.h"
#include "Task.hpp" // Include Task.hpp for TaskWithPublication etc.
#include "RegistrationManager.hpp"

// Mock RegistrationManager
class MockRegistrationManager : public RegistrationManager
{
public:
    template <typename... Adapters>
    void subscribe(const CyphalSubscription &subscription, std::shared_ptr<Task> task, std::tuple<Adapters...> &adapters)
    {
        subscribed_ = true;
    }

    template <typename... Adapters>
    void unsubscribe(const CyphalSubscription &subscription, std::shared_ptr<Task> task, std::tuple<Adapters...> &adapters)
    {
        subscribed_ = false;
    }

    bool
    isSubscribed() const
    {
        return subscribed_;
    }

private:
    bool subscribed_ = false;
};

TEST_CASE("TaskCheckMemory")
{
    // Create an O1Heap instance (you might need to adjust parameters)
    uint8_t buffer[4096] __attribute__ ((aligned (256)));
    O1HeapInstance *heap = o1heapInit((O1HeapInstance *)buffer, sizeof(buffer));
    CHECK(heap != nullptr);

    // Create a TaskCheckMemory instance
    TaskCheckMemory task(heap, 100, 0); // Interval = 100, Shift = 0

    // Create Registration Manager
    MockRegistrationManager manager;
    std::shared_ptr<Task> task_ptr = std::make_shared<TaskCheckMemory>(task);
    task.registerTask(&manager, task_ptr);
    // Initial state check (before handleTask is called) - you might need more
    // specific checks depending on your O1Heap initialization.
    O1HeapDiagnostics diagnostic_before = o1heapGetDiagnostics(heap);

    // Simulate time passing and handle the task
    for (uint32_t i = 0; i < 101; ++i)
    {
        HAL_SetTick(i);
        task.handleTask();
    }

    // Check that handleTaskImpl() was called after interval has passed
    O1HeapDiagnostics diagnostic_after = o1heapGetDiagnostics(heap);
    CHECK(diagnostic_before.allocated == diagnostic_after.allocated);
}