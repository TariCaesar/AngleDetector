#ifndef SYSTEM_H
#define SYSTEM_H

#include "stm32f1xx.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_utils.h"
#include "dmp_user.h"
#include "i2c.h"

int32_t SysClkInit();
int32_t SysTickEnable();

#endif
