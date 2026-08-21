#include "key.h"
#include "delay.h"
#include "main.h"
#include "bsp.h"


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    delay_ms(100);
    if(HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin)==0){

    }
}

