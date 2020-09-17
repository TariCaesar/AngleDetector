#include "motor.h"

int32_t MotorInit(){
    if(!LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_GPIOA)) {
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    }

    LL_GPIO_InitTypeDef motorGpioInit;
    motorGpioInit.Pin = LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6;
    motorGpioInit.Mode = LL_GPIO_MODE_OUTPUT;
    motorGpioInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    motorGpioInit.Speed = LL_GPIO_SPEED_FREQ_LOW;
    motorGpioInit.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(GPIOA, &motorGpioInit);

    //Turn on motor for calibration caution
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_ALL);

    return 0;
}

int32_t MotorTurnOn(int32_t motorIndex){
    switch(motorIndex){
        case 0:
            LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
            break;
        case 1:
            LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
            break;
        case 2:
            LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
            break;
        default:
            return -1;
    }
    return 0;
}

int32_t MotorTurnOff(int32_t motorIndex){
    switch(motorIndex){
        case 0:
            LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
            break;
        case 1:
            LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
            break;
        case 2:
            LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
            break;
        default:
            return -1;
    }
    return 0;
}
