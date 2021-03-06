#ifndef USART_H
#define USART_H

#include "stm32f1xx.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"

//due to our full judgement alogrithm, buffer size is 1 less than them
#define USART_RX_BUFFER_SIZE 1024
#define USART_TX_BUFFER_SIZE 1024
#define USART_TX_DMA_THRESHOLD 4

typedef struct{
    uint8_t data[USART_RX_BUFFER_SIZE];
    uint32_t head;
    uint32_t tail;
}UsartRxBuffer;

typedef struct{
    uint8_t data[USART_TX_BUFFER_SIZE];
    uint32_t head;
    uint32_t tail;
    uint32_t tailNext;
}UsartTxBuffer;

int32_t UsartInit();
int32_t UsartSetMystdioHandler(USART_TypeDef* usartTarget);

uint32_t UsartReceiveData(uint8_t* addrDst, uint32_t size, USART_TypeDef* usartTarget);
uint32_t UsartTransmitData(uint8_t* addrSrc, uint32_t size, USART_TypeDef* usartTarget);
#endif
