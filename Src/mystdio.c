#include "mystdio.h"

static uint32_t Usart1TransmitHandler(uint8_t* addrSrc, uint32_t size){
    return UsartTransmitData(addrSrc, size, USART1);
}

static uint32_t Usart2TransmitHandler(uint8_t* addrSrc, uint32_t size){
    return UsartTransmitData(addrSrc, size, USART2);
}

static uint32_t Usart1ReceiveHandler(uint8_t* addrDst, uint32_t size){
    return UsartReceiveData(addrDst, size, USART1);
}

static uint32_t Usart2ReceiveHandler(uint8_t* addrDst, uint32_t size){
    return UsartReceiveData(addrDst, size, USART2);
}

static uint32_t (*mystdioTransmitHandler)(uint8_t*, uint32_t) = Usart1TransmitHandler;
static uint32_t (*mystdioReceiveHandler)(uint8_t*, uint32_t) = Usart1ReceiveHandler;

int32_t SetMystdioTransimitHandler(uint32_t (*handler)(uint8_t*, uint32_t)){
    mystdioTransmitHandler = handler;
    return 0;
}

int32_t SetMystdioReceiveHandler(uint32_t (*handler)(uint8_t*, uint32_t)){
    mystdioReceiveHandler = handler;
    return 0;
}

int32_t _write(int32_t ch, uint8_t* pBuffer, int32_t size)
{
    int32_t sizeSent = 0;
    sizeSent += mystdioTransmitHandler(pBuffer, size);
    while(sizeSent < size) {
        sizeSent += mystdioTransmitHandler(pBuffer + sizeSent, size - sizeSent);
    };
    return size;
}

int32_t _read(int32_t ch, uint8_t* pBuffer, int32_t size)
{
    int32_t sizeReceived = 0;
    sizeReceived += mystdioReceiveHandler(pBuffer, size);
    while(sizeReceived < size) {
        sizeReceived += mystdioReceiveHandler(pBuffer + sizeReceived, size - sizeReceived);
    };
    return size;
}
