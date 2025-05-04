#ifndef _CAMERA_SWITCH_H_
#define _CAMERA_SWITCH_H_

#include "mock_hal.h"

enum class CameraSwitchState : uint8_t {
    OFF = 0,
    A = 1,
    B = 2,
    C = 3,
};

class CameraSwitch {
public:
    CameraSwitch(GPIO_TypeDef* gpio1, uint16_t pin1, GPIO_TypeDef* gpio0, uint16_t pin0) : 
        gpio1_{gpio1}, gpio0_{gpio0}, pin1_{pin1}, pin0_{pin0}, state_{CameraSwitchState::OFF}
    {
        setState(state_);
    }

    void setState(CameraSwitchState state)
    {
        GPIO_PinState pin1_state = (state == CameraSwitchState::B || state == CameraSwitchState::C) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        GPIO_PinState pin0_state = (state == CameraSwitchState::B || state == CameraSwitchState::A) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(const_cast<GPIO_TypeDef*>(gpio1_), pin1_, pin1_state);
        HAL_GPIO_WritePin(const_cast<GPIO_TypeDef*>(gpio0_), pin0_, pin0_state);
    }

private:
const GPIO_TypeDef *gpio1_, *gpio0_;    
const uint16_t pin1_, pin0_;    
CameraSwitchState state_;
};

#endif /* _CAMERA_SWITCH_H_ */
