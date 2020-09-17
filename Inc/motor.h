#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"

int32_t MotorInit();
int32_t MotorTurnOn(int32_t motorIndex);
int32_t MotorTurnOff(int32_t motorIndex);

#endif
