#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "Trigger.hpp"
#include "mock_hal.h"

// -----------------------------------------------------------------------------
// TriggerConcept
// -----------------------------------------------------------------------------
TEST_CASE("TriggerConcept accepts valid trigger types")
{
    CHECK(TriggerConcept<ManualTrigger>);
    CHECK(TriggerConcept<OnceTrigger>);
    CHECK(TriggerConcept<PeriodicTrigger>);
}

// -----------------------------------------------------------------------------
// ManualTrigger
// -----------------------------------------------------------------------------
TEST_CASE("ManualTrigger basic behavior")
{
    ManualTrigger t;

    CHECK(t.trigger() == false);   // nothing pending

    t.fire();
    CHECK(t.trigger() == true);    // first call consumes pending

    CHECK(t.trigger() == false);   // now empty again

    t.fire();
    t.fire();                      // multiple fires still produce one true
    CHECK(t.trigger() == true);
    CHECK(t.trigger() == false);
}

// -----------------------------------------------------------------------------
// OnceTrigger
// -----------------------------------------------------------------------------
TEST_CASE("OnceTrigger triggers exactly once")
{
    OnceTrigger t;

    CHECK(t.trigger() == true);    // first time true
    CHECK(t.trigger() == false);   // then always false
    CHECK(t.trigger() == false);
}

// -----------------------------------------------------------------------------
// PeriodicTrigger
// -----------------------------------------------------------------------------
TEST_CASE("PeriodicTrigger fires at correct intervals")
{
    HAL_SetTick(0);

    PeriodicTrigger t{ .interval_ms = 10, .next_time = 0 };

    // At t=0 → should fire
    CHECK(t.trigger() == true);

    // Immediately again → should NOT fire
    CHECK(t.trigger() == false);

    // Advance to t=9 → still not enough
    HAL_SetTick(9);
    CHECK(t.trigger() == false);

    // At t=10 → should fire
    HAL_SetTick(10);
    CHECK(t.trigger() == true);

    // Next fire at t=20
    HAL_SetTick(19);
    CHECK(t.trigger() == false);

    HAL_SetTick(20);
    CHECK(t.trigger() == true);
}
