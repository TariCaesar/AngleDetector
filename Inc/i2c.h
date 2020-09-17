#ifndef I2C_H
#define I2C_H

#include "stm32f1xx.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_i2c.h"
#include "mystdio.h"

#define I2C_REQUEST_WRITE 0xfe
#define I2C_REQUEST_READ 0x01

void I2cDelay(int32_t ms);
int32_t I2cGetMs();
void SetI2CTarget(I2C_TypeDef* I2CxTarget);

int32_t I2cInit();

uint8_t I2cWriteByte(uint8_t deviceAddr, uint8_t regAddr, uint8_t data);
//i2cWriteWord first transmit high byte and then low byte
uint32_t I2cWrite(uint8_t deviceAddr, uint8_t regAddr, uint8_t* dataSrc, uint32_t dataSize);

uint32_t I2cReadByte(uint8_t deviceAddr, uint8_t regAddr, uint8_t *dataDst);
uint32_t I2cReadHalfWord(uint8_t deviceAddr, uint8_t regAddr, uint16_t* dataDst);
uint32_t I2cRead(uint8_t deviceAddr, uint8_t regAddr, uint8_t* dataDst, uint32_t dataSize);

#endif
