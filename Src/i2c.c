#include "i2c.h"

static I2C_TypeDef* I2Cx = I2C1;
static uint8_t deviceAddr = 0x68;

int32_t I2cGetMs(){return 36000;};

void I2cDelay(int32_t ms){
    int32_t i;
    for(i = 0; i < 36000 * ms; ++i);
}

void SetI2CTarget(I2C_TypeDef* I2CxTarget){
    I2Cx = I2CxTarget;
}


int32_t I2cInit(){
    //enable i2c clock
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

    //check the GPIOB clock status
    //if not open, open it
    if(!LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_GPIOB)) {
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
    }

    //GPIO AF configuration
    //I2C1 SCL is PB6, I2C1 SDA is PB7, configure into af output
    LL_GPIO_InitTypeDef i2c1GpioInit;
    i2c1GpioInit.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
    i2c1GpioInit.Mode = LL_GPIO_MODE_ALTERNATE;
    i2c1GpioInit.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    i2c1GpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    i2c1GpioInit.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(GPIOB, &i2c1GpioInit);

    //GPIO AF configuration
    //I2C2 SCL is PB10, I2C2 SDA is PB11, configure into af output
    LL_GPIO_InitTypeDef i2c2GpioInit;
    i2c2GpioInit.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_11;
    i2c2GpioInit.Mode = LL_GPIO_MODE_ALTERNATE;
    i2c2GpioInit.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    i2c2GpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    i2c2GpioInit.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(GPIOB, &i2c2GpioInit);


    LL_I2C_DeInit(I2C1);
    LL_I2C_InitTypeDef i2c1Init;
    LL_I2C_StructInit(&i2c1Init);
    i2c1Init.ClockSpeed = 200000u;
    i2c1Init.PeripheralMode = LL_I2C_MODE_I2C;
    i2c1Init.OwnAddress1 = 0x51;
    i2c1Init.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
    i2c1Init.TypeAcknowledge = LL_I2C_ACK;
    LL_I2C_Init(I2C1, &i2c1Init);

    LL_I2C_DeInit(I2C2);
    LL_I2C_InitTypeDef i2c2Init;
    LL_I2C_StructInit(&i2c2Init);
    i2c2Init.ClockSpeed = 200000u;
    i2c2Init.PeripheralMode = LL_I2C_MODE_I2C;
    i2c2Init.OwnAddress1 = 0x52;
    i2c2Init.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
    i2c2Init.TypeAcknowledge = LL_I2C_ACK;
    LL_I2C_Init(I2C2, &i2c2Init);

    return 0;
}

uint8_t I2cWriteByte(uint8_t deviceAddr, uint8_t regAddr, uint8_t data){
    LL_I2C_Enable(I2Cx);

    if(I2Cx != I2C1 && I2Cx != I2C2)return 1;
    
    while(LL_I2C_IsActiveFlag_BUSY(I2Cx))continue;

    //Generate Start Signal, and wait for it complete
    LL_I2C_GenerateStartCondition(I2Cx);

    while(!LL_I2C_IsActiveFlag_SB(I2Cx))continue;

    LL_I2C_TransmitData8(I2Cx, (deviceAddr << 1) & I2C_REQUEST_WRITE);
    while(!LL_I2C_IsActiveFlag_ADDR(I2Cx))continue;
    LL_I2C_ClearFlag_ADDR(I2Cx);

    LL_I2C_TransmitData8(I2Cx, regAddr);
    while(!LL_I2C_IsActiveFlag_BTF(I2Cx))continue;

    LL_I2C_TransmitData8(I2Cx, data);
    //wait until the last transfer complete
    while(!LL_I2C_IsActiveFlag_BTF(I2Cx))continue;

    LL_I2C_GenerateStopCondition(I2Cx);
    //Wait until bus release
    while(LL_I2C_IsActiveFlag_BUSY(I2Cx))continue;

    LL_I2C_Disable(I2Cx);
    return 0;
}

uint32_t I2cWrite(uint8_t deviceAddr, uint8_t regAddr, uint8_t* dataSrc, uint32_t dataSize){
    LL_I2C_Enable(I2Cx);

    if(I2Cx != I2C1 && I2Cx != I2C2)return 1;
    if(LL_I2C_IsActiveFlag_BUSY(I2Cx))return 2;

    //Generate Start Signal, and wait for it complete
    LL_I2C_GenerateStartCondition(I2Cx);
    while(!LL_I2C_IsActiveFlag_SB(I2Cx))continue;

    LL_I2C_TransmitData8(I2Cx, (deviceAddr << 1) & I2C_REQUEST_WRITE);
    while(!LL_I2C_IsActiveFlag_ADDR(I2Cx))continue;
    LL_I2C_ClearFlag_ADDR(I2Cx);

    LL_I2C_TransmitData8(I2Cx, regAddr);
    while(!LL_I2C_IsActiveFlag_BTF(I2Cx))continue;
    uint32_t i;
    for(i = 0; i < dataSize; ++i){
        LL_I2C_TransmitData8(I2Cx, *(dataSrc++));
        while(!LL_I2C_IsActiveFlag_BTF(I2Cx))continue;
    }

    LL_I2C_GenerateStopCondition(I2Cx);
    //Wait until bus release
    while(LL_I2C_IsActiveFlag_BUSY(I2Cx))continue;

    LL_I2C_Disable(I2Cx);
    return 0;
}


uint32_t I2cReadByte(uint8_t deviceAddr, uint8_t regAddr, uint8_t* dataDst){
    if(I2cRead(deviceAddr, regAddr, dataDst, 1))return 1;
    else return 0;
}

uint32_t I2cReadHalfWord(uint8_t deviceAddr, uint8_t regAddr, uint16_t* dataDst){
    if(I2cRead(deviceAddr, regAddr, (uint8_t*)dataDst, 2))return 1;
    else return 0;
}

uint32_t I2cRead(uint8_t deviceAddr, uint8_t regAddr, uint8_t* dataDst, uint32_t dataSize){
    LL_I2C_Enable(I2Cx);
    if(LL_I2C_IsActiveFlag_BUSY(I2Cx))return 1;

    LL_I2C_GenerateStartCondition(I2Cx);
    while(!LL_I2C_IsActiveFlag_SB(I2Cx))continue;

    LL_I2C_TransmitData8(I2Cx, (deviceAddr << 1) & I2C_REQUEST_WRITE);
    while(!LL_I2C_IsActiveFlag_ADDR(I2Cx))continue;
    LL_I2C_ClearFlag_ADDR(I2Cx);

    LL_I2C_TransmitData8(I2Cx, regAddr);

    //wait until the last transfer complete
    while(!LL_I2C_IsActiveFlag_BTF(I2Cx))continue;
    //Generate Start Signal, and wait for it complete

    LL_I2C_GenerateStartCondition(I2Cx);
    while(!LL_I2C_IsActiveFlag_SB(I2Cx))continue;

    //Transfer device Addr for read, and wait for it complete
    LL_I2C_TransmitData8(I2Cx, (deviceAddr << 1) | I2C_REQUEST_READ);
    while(!LL_I2C_IsActiveFlag_ADDR(I2Cx))continue;

    if(dataSize == 1){
        //lock scl
        LL_GPIO_SetPinMode(GPIOB, (I2Cx == I2C1)? LL_GPIO_PIN_6: LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
        LL_GPIO_ResetOutputPin(GPIOB, (I2Cx == I2C1)? LL_GPIO_PIN_6: LL_GPIO_PIN_10);
        LL_I2C_ClearFlag_ADDR(I2Cx);
        LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
        LL_I2C_GenerateStopCondition(I2Cx);
        //release scl
        LL_GPIO_SetPinMode(GPIOB, (I2Cx == I2C1)? LL_GPIO_PIN_6: LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);

        while(!LL_I2C_IsActiveFlag_RXNE(I2Cx))continue;
        *dataDst = LL_I2C_ReceiveData8(I2Cx);
    }
    else if(dataSize == 2){
        LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);
        LL_I2C_EnableBitPOS(I2Cx);

        //lock scl
        LL_GPIO_SetPinMode(GPIOB, (I2Cx == I2C1)? LL_GPIO_PIN_6: LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
        LL_GPIO_ResetOutputPin(GPIOB, (I2Cx == I2C1)? LL_GPIO_PIN_6: LL_GPIO_PIN_10);
        LL_I2C_ClearFlag_ADDR(I2Cx);
        LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
        //release scl
        LL_GPIO_SetPinMode(GPIOB, (I2Cx == I2C1)? LL_GPIO_PIN_6: LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);

        while(!LL_I2C_IsActiveFlag_BTF(I2Cx))continue;
        //lock scl
        LL_GPIO_SetPinMode(GPIOB, (I2Cx == I2C1)? LL_GPIO_PIN_6: LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
        LL_GPIO_ResetOutputPin(GPIOB, (I2Cx == I2C1)? LL_GPIO_PIN_6: LL_GPIO_PIN_10);
        LL_I2C_DisableBitPOS(I2Cx);
        LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);

        LL_I2C_GenerateStopCondition(I2Cx);
        *(dataDst++) = LL_I2C_ReceiveData8(I2Cx);
        //release scl
        LL_GPIO_SetPinMode(GPIOB, (I2Cx == I2C1)? LL_GPIO_PIN_6: LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);

        while(!LL_I2C_IsActiveFlag_RXNE(I2Cx))continue;
        *dataDst = LL_I2C_ReceiveData8(I2Cx);
    }
    else{
        //Transfer device Addr for read, and wait for it complete
        LL_I2C_ClearFlag_ADDR(I2Cx);
        LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);

        uint32_t i = dataSize;
        while(i > 3){
            while(!LL_I2C_IsActiveFlag_BTF(I2Cx))continue;
            *(dataDst++) = LL_I2C_ReceiveData8(I2Cx);
            --i;
        }
        while(!LL_I2C_IsActiveFlag_BTF(I2Cx))continue;
        //lock scl
        LL_GPIO_SetPinMode(GPIOB, (I2Cx == I2C1)? LL_GPIO_PIN_6: LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
        LL_GPIO_ResetOutputPin(GPIOB, (I2Cx == I2C1)? LL_GPIO_PIN_6: LL_GPIO_PIN_10);
        LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
        //release scl
        LL_GPIO_SetPinMode(GPIOB, (I2Cx == I2C1)? LL_GPIO_PIN_6: LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
        *(dataDst++) = LL_I2C_ReceiveData8(I2Cx);

        while(!LL_I2C_IsActiveFlag_BTF(I2Cx))continue;
        //lock scl
        LL_GPIO_SetPinMode(GPIOB, (I2Cx == I2C1)? LL_GPIO_PIN_6: LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
        LL_GPIO_ResetOutputPin(GPIOB, (I2Cx == I2C1)? LL_GPIO_PIN_6: LL_GPIO_PIN_10);
        LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);
        LL_I2C_GenerateStopCondition(I2Cx);
        *(dataDst++) = LL_I2C_ReceiveData8(I2Cx);
        //release scl
        LL_GPIO_SetPinMode(GPIOB, (I2Cx == I2C1)? LL_GPIO_PIN_6: LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);

        *(dataDst++) = LL_I2C_ReceiveData8(I2Cx);
    }

    //Wait until bus release
    while(LL_I2C_IsActiveFlag_BUSY(I2Cx))continue;

    LL_I2C_Disable(I2Cx);
    return 0;
}
