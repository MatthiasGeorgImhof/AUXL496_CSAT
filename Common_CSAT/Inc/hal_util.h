#ifndef HAL_UTILITIES_H
#define HAL_UTILITIES_H

#ifdef __x86_64__
#include "mock_hal.h"
#endif /* __x86_64__ */

//
// https://stackoverflow.com/questions/69391709/is-there-any-way-to-make-a-delay-in-microseconds-in-stm32-cubeide
//

uint32_t gSysTickLoad = 0;
uint32_t gTicksPerMicrosecondFloor = 0;
uint32_t gTicksPerMicrosecondMod1000 = 0;

void DELAY_init(void)
{
  //== Prep for DELAY_microseconds() ==

  // SysTick->LOAD gets set to the number of ticks in a millisecond, in HAL_SYSTICK_Config(), which
  // is called from HAL_Init(). So we don't need to determine or assume clock frequency, and can
  // instead just work backwards from there.
  gSysTickLoad = SysTick->LOAD;
  gTicksPerMicrosecondFloor = (gSysTickLoad + 1) / 1000;
  // This will be zero unless the clock is not an even number of MHz.
  gTicksPerMicrosecondMod1000 = (gSysTickLoad + 1) % 1000;
}

// void inline __attribute__((always_inline)) DELAY_microseconds(uint16_t us)
void DELAY_microseconds(uint16_t us)
{
  uint32_t startTick = SysTick->VAL; // get start tick as soon as possible

  // The asserts and division that follow dictate the minimum possible delay (smallest
  // supported "us" parameter). They provide sanity checks and accuracy as a trade-off.
  // assert(gSysTickLoad == SysTick->LOAD); //make sure DELAY_init got called and nothing has changed since
  // assert(us < 1000); //otherwise, infinite loop!

  if (us > 1000)
    us = 1000; // limit to 1000us (1ms) max, to avoid infinite loop

  uint32_t delayTicks = gTicksPerMicrosecondFloor * us;
  delayTicks += (500 + gTicksPerMicrosecondMod1000 * us) / 1000; // Probably a nop

  while (1)
  {
#ifdef __x86_64__
    // Simulate the SysTick interrupt and tick increment
    HAL_IncTick();
#endif /* __x86_64__ */

    // Get the current tick value
    uint32_t currentTicks = SysTick->VAL;
    // Handle SysTick->VAL hitting zero and resetting back to SysTick->LOAD.
    uint32_t elapsedTicks = currentTicks < startTick ? startTick - currentTicks : gSysTickLoad + startTick - currentTicks;

    if (elapsedTicks >= delayTicks)
      break;
  }
}

#endif /* HAL_UTILITIES_H */
