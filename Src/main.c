#include "main.h"

int main()
{
    //Init SysClk
    SysClkInit();
    //Set priorityGroup to 2 for all the system
    NVIC_SetPriorityGrouping(2);

    //Init peripheral
    if(UsartInit())NVIC_SystemReset();
    printf("Usart initialization succeed!\n");

    if(mpu_dmp_init())NVIC_SystemReset();
    printf("Mpu initialization succeed!\n");

    //Enable mpu update
    SysTickEnable();

    while(1){
    }
    return 0;
}

