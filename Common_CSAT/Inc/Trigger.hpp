#ifndef __TRIGGER_HPP__
#define __TRIGGER_HPP__

#include <cstdint>
#include <concepts>

#ifdef __arm__
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_rcc.h"
#include "stm32l4xx_hal_rtc.h"
#endif
#ifdef __x86_64__
#include "mock_hal.h"
#endif

template <typename T>
concept TriggerConcept = requires(T t) {
    { t.trigger() } -> std::convertible_to<bool>;
};

struct ManualTrigger {
    bool pending = false;

    void fire() { pending = true; }

    bool trigger() {
        if (pending) {
            pending = false;
            return true;
        }
        return false;
    }
};

struct OnceTrigger {
    bool triggered = false;
    
    bool trigger() {
        if (!triggered) {
            triggered = true;
            return true;
        }
        return false;
    }
};

struct PeriodicTrigger {
    uint32_t interval_ms;
    uint32_t next_time = 0;

    bool trigger() {
        uint32_t now = HAL_GetTick();
        if (now >= next_time) {
            next_time = now + interval_ms;
            return true;
        }
        return false;
    }
};

struct ContinuousTrigger {
	bool trigger() {
		return true;
	}
};

#endif // __TRIGGER_HPP__
