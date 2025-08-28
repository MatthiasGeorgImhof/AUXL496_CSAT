#ifndef HS_CLOCK_SWITCH_H
#define HS_CLOCK_SWITCH_H

#ifdef __x86_64__
#include "mock_hal.h"
#else
#include "stm32l4xx_hal.h"
#endif

class HSClockSwitch
{
public:
    HSClockSwitch();

    ~HSClockSwitch() {}

    HAL_StatusTypeDef switchToHSE();

    HAL_StatusTypeDef switchToHSI();

private:
    HAL_StatusTypeDef configureHSE();
    HAL_StatusTypeDef configureHSI();
    HAL_StatusTypeDef selectClockSource(uint32_t clockSource);
};

class HSClockSwitchWithEnable : private HSClockSwitch
{
public:
    HSClockSwitchWithEnable(GPIO_TypeDef* GPIO, uint16_t pins);
    
    HSClockSwitchWithEnable() = delete;

    ~HSClockSwitchWithEnable() {}

    HAL_StatusTypeDef switchToHSE();

    HAL_StatusTypeDef switchToHSI();

private:
    GPIO_TypeDef* GPIO_;
    uint16_t pins_;
};

#endif // HS_CLOCK_SWITCH_H
