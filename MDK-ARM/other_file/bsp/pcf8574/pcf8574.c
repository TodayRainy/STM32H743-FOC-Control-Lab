#include "pcf8574.h"
#include "i2c.h"

#include <stdio.h>


static void PCF8574_WriteOneBit(uint8_t DataToWrite)
{
    HAL_I2C_Master_Transmit(&hi2c2, PCF8574_ADDR, &DataToWrite, 1, HAL_MAX_DELAY); //写入新的数据
}

void PCF8574_Init(void)
{
    if(HAL_OK==HAL_I2C_IsDeviceReady(&hi2c2, PCF8574_ADDR, 10, HAL_MAX_DELAY)){
        printf("PCF8574 存在\r\n");
    }else{
        printf("PCF8574 不存在\r\n");
    }

    PCF8574_WriteOneBit(0xFF);      /*全部输出高*/
}


//设置PCF8574某个IO的高低电平
//bit:要设置的IO编号,0~7
//sta:IO的状态;0或1
void PCF8574_WriteBit(uint8_t bit,uint8_t sta)
{
    uint8_t data;
    HAL_I2C_Master_Receive(&hi2c2, PCF8574_ADDR, &data, 1, HAL_MAX_DELAY);  //先读出原来的设置
    if(sta==0)data&=~(1<<bit);     
    else data|=1<<bit;
    HAL_I2C_Master_Transmit(&hi2c2, PCF8574_ADDR, &data, 1, HAL_MAX_DELAY); //写入新的数据
}

void PCF8574_IRQHandler(void)
{
    
}











