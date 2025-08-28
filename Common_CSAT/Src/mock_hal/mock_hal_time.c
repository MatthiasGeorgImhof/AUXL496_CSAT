#ifdef __x86_64__

#include "mock_hal/mock_hal_time.h"

//------------------------------------------------------------------------------
//  GLOBAL MOCKED VARIABLES - State
//------------------------------------------------------------------------------

//--- General Mock Variables ---
uint32_t current_tick = 0;

SysTick_Type sys_tick; //Mock SysTick register structure
SysTick_Type* SysTick = &sys_tick; //Mock SysTick pointer
//------------------------------------------------------------------------------

void HAL_Delay(uint32_t Delay)
{
    uint32_t startTick = HAL_GetTick();
    uint32_t targetTick = startTick + Delay;
    while (HAL_GetTick() < targetTick)
    {
        HAL_IncTick(); // Simulate the SysTick interrupt and tick increment
    }
}

uint32_t HAL_GetTick(void)
{
    return current_tick;
}

void HAL_SetTick(uint32_t tick)
{
    current_tick = tick;
}

void set_current_tick(uint32_t tick){
    current_tick = tick;
    SysTick->VAL = 0; // Set VAL to zero so its synchronised correctly
}

HAL_StatusTypeDef HAL_SYSTICK_Config(uint32_t ticks)
{
  //Mock implementation, record number of ticks in reload register
  SysTick->LOAD = ticks;
  SysTick->VAL = 0; //Initialise VAL to 0 at config

  return HAL_OK;
}

HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority)
{
  //Mock implementation
  (void)TickPriority; //Not needed
  return HAL_OK;
}

void HAL_IncTick(void)
{
    //Simulate systick counting down and rolling over

    if (SysTick->VAL > 0)
    {
        SysTick->VAL--;
    }
    else
    {
        current_tick++; // increment global system tick counter
        SysTick->VAL = SysTick->LOAD; //Reset to LOAD value AFTER incrementing HAL_GetTick()
    }
}

#endif /*__x86_64__*/