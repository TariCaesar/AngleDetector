#include "usart.h"

#ifdef USART_USE_USART1
static volatile UsartRxBuffer usart1RxBuffer;
static volatile UsartTxBuffer usart1TxBuffer;
#endif

#ifdef USART_USE_USART2
static volatile UsartRxBuffer usart2RxBuffer;
static volatile UsartTxBuffer usart2TxBuffer;
#endif

int32_t UsartInit()
{
#ifdef USART_USE_USART1
    //check the GPIOA clock status
    //if not open, open it
    if(!LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_GPIOA)) {
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    }

    //USART1 Tx is PA9, Rx is PA10 
    LL_GPIO_InitTypeDef usart1GpioInit;
    usart1GpioInit.Pin = LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
    usart1GpioInit.Mode = LL_GPIO_MODE_ALTERNATE;
    usart1GpioInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    usart1GpioInit.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
    usart1GpioInit.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(GPIOA, &usart1GpioInit);

    //Enable usart clock
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

    LL_USART_DeInit(USART1);
    //init usart
    LL_USART_InitTypeDef usart1Init;
    //Set usart1 baudrate to 115200
    usart1Init.BaudRate = 115200u;
    //Set usart1 frame format 8b data, 1b stop, no parity
    usart1Init.DataWidth = LL_USART_DATAWIDTH_8B;
    usart1Init.StopBits = LL_USART_STOPBITS_1;
    usart1Init.Parity = LL_USART_PARITY_NONE;
    //only enable RX as we'll enable TX later
    usart1Init.TransferDirection = LL_USART_DIRECTION_RX;
    //Disable hardware flow control
    usart1Init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    //Set oversampling to 16
    usart1Init.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART1, &usart1Init);

    //Set usart into asysn mode
    LL_USART_ConfigAsyncMode(USART1);

    //Init usart buffer
    usart1RxBuffer.head = 0;
    usart1TxBuffer.head = 0;
    usart1TxBuffer.tail = 0;
    usart1TxBuffer.tailNext = 0;

    //Init interrupt
    LL_USART_ClearFlag_RXNE(USART1);
    LL_USART_EnableIT_RXNE(USART1);
    LL_USART_ClearFlag_TC(USART1);
    LL_USART_EnableIT_TC(USART1);
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(2, 2, 1));
    NVIC_EnableIRQ(USART1_IRQn);
    
#ifdef USART_USE_DMA
    //Enable dma clock
    if(!LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_DMA1)) {
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    }
#endif

    //Enable Usart
    LL_USART_Enable(USART1);
#endif

#ifdef USART_USE_USART2

    //check the GPIOA clock status
    //if not open, open it
    if(!LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_GPIOA)) {
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    }

    //USART2 Tx is PA2, Rx is PA3
    LL_GPIO_InitTypeDef usart2GpioInit;
    usart2GpioInit.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
    usart2GpioInit.Mode = LL_GPIO_MODE_ALTERNATE;
    usart2GpioInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    usart2GpioInit.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
    usart2GpioInit.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(GPIOA, &usart2GpioInit);

    //Enable usart clock
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

    LL_USART_DeInit(USART2);
    //init usart
    LL_USART_InitTypeDef usart2Init;
    //Set usart1 baudrate to 115200
    usart2Init.BaudRate = 115200u;
    //Set usart1 frame format 8b data, 1b stop, no parity
    usart2Init.DataWidth = LL_USART_DATAWIDTH_8B;
    usart2Init.StopBits = LL_USART_STOPBITS_1;
    usart2Init.Parity = LL_USART_PARITY_NONE;
    //only enable RX as we'll enable TX later
    usart2Init.TransferDirection = LL_USART_DIRECTION_RX;
    //Disable hardware flow control
    usart2Init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    //Set oversampling to 16
    usart2Init.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART2, &usart2Init);

    //Set usart into asysn mode
    LL_USART_ConfigAsyncMode(USART2);

    //Init usart2 buffer
    usart2RxBuffer.head = 0;
    usart2TxBuffer.head = 0;
    usart2TxBuffer.tail = 0;
    usart2TxBuffer.tailNext = 0;

    //Init interrupt
    LL_USART_ClearFlag_RXNE(USART2);
    LL_USART_EnableIT_RXNE(USART2);
    LL_USART_ClearFlag_TC(USART2);
    LL_USART_EnableIT_TC(USART2);;
    NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(2, 2, 1));
    NVIC_EnableIRQ(USART2_IRQn);

#ifdef USART_USE_DMA
    //Enable dma clock
    if(!LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_DMA1)) {
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    }
#endif

    //Enable Usart
    LL_USART_Enable(USART2);
#endif

    return 0;
}

static int32_t UsartTransmitTrigger(USART_TypeDef* usartTarget)
{
    //if buffer remain less than USART_TX_DMA_THRESHOLD, use single transimit
    //otherwise, use dma
    uint32_t txTransmitRemain;
    UsartTxBuffer* txBuffer = 0;

#ifdef USART_USE_USART1
    if(usartTarget == USART1)txBuffer = &usart1TxBuffer;
#endif
#ifdef USART_USE_USART2 
    if(usartTarget == USART2)txBuffer = &usart2TxBuffer;
#endif
    if(txBuffer == 0)return 1;

    txTransmitRemain =
        (txBuffer->head < txBuffer->tail) ? (txBuffer->head + USART_TX_BUFFER_SIZE - txBuffer->tail) : (txBuffer->head - txBuffer->tail);

    if(txTransmitRemain == 0) return 2;

#ifdef USART_USE_DMA
    if(txTransmitRemain < USART_TX_DMA_THRESHOLD) {
        txBuffer->tailNext = (txBuffer->tail + 1) % USART_TX_BUFFER_SIZE;
        LL_USART_TransmitData8(usartTarget, txBuffer->data[txBuffer->tail]);
    }
    else {
        //Configre DMA
        LL_USART_EnableDMAReq_TX(usartTarget);
        LL_DMA_InitTypeDef usartTxDmaInit;
        usartTxDmaInit.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
        usartTxDmaInit.PeriphOrM2MSrcAddress = (uint32_t)(&(usartTarget->DR));
        usartTxDmaInit.MemoryOrM2MDstAddress = (uint32_t)(txBuffer->data + txBuffer->tail);
        usartTxDmaInit.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
        usartTxDmaInit.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
        usartTxDmaInit.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
        usartTxDmaInit.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
        usartTxDmaInit.Mode = LL_DMA_MODE_NORMAL;
        usartTxDmaInit.Priority = LL_DMA_PRIORITY_MEDIUM;
        if(txBuffer->head < txBuffer->tail) {
            usartTxDmaInit.NbData = USART_TX_BUFFER_SIZE - txBuffer->tail;
            txBuffer->tailNext = 0;
        }
        else {
            usartTxDmaInit.NbData = txBuffer->head - txBuffer->tail;
            txBuffer->tailNext = txBuffer->head;
        }
        uint32_t usartDmaChannel = 
            (usartTarget == USART1)? LL_DMA_CHANNEL_4: 
            (usartTarget == USART2)? LL_DMA_CHANNEL_7:
            0xffffffff;
        LL_DMA_Init(DMA1, usartDmaChannel, &usartTxDmaInit);
        //Enable Channl for DMA1
        LL_DMA_EnableChannel(DMA1, usartDmaChannel);
    }
#else
    txBuffer->tailNext = (txBuffer->tail + 1) % USART_TX_BUFFER_SIZE;
    LL_USART_TransmitData8(usartTarget, txBuffer->data[txBuffer->tail]);
#endif
    //Clear USART1 TXE Interrupt flag
    LL_USART_ClearFlag_TC(usartTarget);
    //Enable USART1 Tx
    LL_USART_EnableDirectionTx(usartTarget);
    return 0;
}

uint32_t UsartReceiveData(uint8_t* addrDst, uint32_t size, USART_TypeDef* usartTarget)
{
    uint32_t i, rxReceiveRemain;
    UsartRxBuffer* rxBuffer = 0;

#ifdef USART_USE_USART1
    if(usartTarget == USART1)rxBuffer = &usart1TxBuffer;
#endif
#ifdef USART_USE_USART2 
    if(usartTarget == USART2)rxBuffer = &usart2TxBuffer;
#endif
    if(rxBuffer == 0)return 0;
    

    rxReceiveRemain =
        (rxBuffer->head < rxBuffer->tail) ? (rxBuffer->head + USART_RX_BUFFER_SIZE - rxBuffer->tail) : (rxBuffer->head - rxBuffer->tail);
    if(rxReceiveRemain < size) size = rxReceiveRemain;
    for(i = 0; i < size; ++i) {
        addrDst[i] = rxBuffer->data[rxBuffer->tail];
        rxBuffer->tail = (rxBuffer->tail + 1) % USART_RX_BUFFER_SIZE;
    }
    return size;
}

uint32_t UsartTransmitData(uint8_t* addrSrc, uint32_t size, USART_TypeDef* usartTarget)
{
    uint32_t i, txBufferRemain;
    UsartTxBuffer* txBuffer;

#ifdef USART_USE_USART1
    if(usartTarget == USART1)txBuffer = &usart1TxBuffer;
#endif
#ifdef USART_USE_USART2 
    if(usartTarget == USART2)txBuffer = &usart2TxBuffer;
#endif
    if(txBuffer == 0)return 1;

    //check if buffer is large enough
    txBufferRemain =
        (txBuffer->head < txBuffer->tail) ? (txBuffer->tail - txBuffer->head - 1) : (USART_TX_BUFFER_SIZE - txBuffer->head + txBuffer->tail - 1);
    if(txBufferRemain < size) size = txBufferRemain;
    for(i = 0; i < size; ++i) {
        txBuffer->data[txBuffer->head] = addrSrc[i];
        txBuffer->head = (txBuffer->head + 1) % USART_TX_BUFFER_SIZE;
    }
    //check if Usart Tx is enable
    //if so, it means dma is running
    if(!(LL_USART_GetTransferDirection(usartTarget) & LL_USART_DIRECTION_TX))UsartTransmitTrigger(usartTarget);
    return size;
}

#ifdef USART_USE_USART1
void USART1_IRQHandler()
{
    if(LL_USART_IsActiveFlag_TC(USART1)) {
        LL_USART_ClearFlag_TC(USART1);
#ifdef USART_USE_DMA
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
#endif
        usart1TxBuffer.tail = usart1TxBuffer.tailNext;
        if(usart1TxBuffer.tail == usart1TxBuffer.head) {
            //Transmit complete
            LL_USART_DisableDMAReq_TX(USART1);
            LL_USART_DisableDirectionTx(USART1);
        }
        else {
            UsartTransmitTrigger(USART1);
        }
    }
    else if(LL_USART_IsActiveFlag_RXNE(USART1)) {
        usart1RxBuffer.data[usart1RxBuffer.head] = LL_USART_ReceiveData8(USART1);
        usart1RxBuffer.head = (usart1RxBuffer.head + 1) % USART_RX_BUFFER_SIZE;
        //if rxBuffer is full, throw the oldest data
        if(usart1RxBuffer.head == usart1RxBuffer.tail) {
            usart1RxBuffer.tail = (usart1RxBuffer.tail + 1) % USART_RX_BUFFER_SIZE;
        }
        //clear the interrupt flag
        LL_USART_ClearFlag_RXNE(USART1);
    }
}
#endif

#ifdef USART_USE_USART2
void USART2_IRQHandler()
{
    if(LL_USART_IsActiveFlag_TC(USART2)) {
        LL_USART_ClearFlag_TC(USART2);
#ifdef USART_USE_DMA
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_7);
#endif
        usart2TxBuffer.tail = usart2TxBuffer.tailNext;
        if(usart2TxBuffer.tail == usart2TxBuffer.head) {
            //Transmit complete
            LL_USART_DisableDMAReq_TX(USART2);
            LL_USART_DisableDirectionTx(USART2);
        }
        else {
            UsartTransmitTrigger(USART2);
        }
    }
    else if(LL_USART_IsActiveFlag_RXNE(USART2)) {
        usart2RxBuffer.data[usart2RxBuffer.head] = LL_USART_ReceiveData8(USART2);
        usart2RxBuffer.head = (usart2RxBuffer.head + 1) % USART_RX_BUFFER_SIZE;
        //if rxBuffer is full, throw the oldest data
        if(usart2RxBuffer.head == usart2RxBuffer.tail) {
            usart2RxBuffer.tail = (usart2RxBuffer.tail + 1) % USART_RX_BUFFER_SIZE;
        }
        //clear the interrupt flag
        LL_USART_ClearFlag_RXNE(USART2);
    }
}
#endif
