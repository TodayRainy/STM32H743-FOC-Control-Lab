#include "bsp.hpp"
#include <stdio.h>



/**********************************ADC中断***************************************** */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC3) { 
        adc3_dma->stop();
        adc3_dma->calculate();
        adc3_dma->start();

        // printf("HAL_ADC3_ConvCpltCallback: %d\r\n", HAL_ADC_GetValue(hadc));
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC3) {
        // printf("HAL_ADC3_ErrorCallback: %d\r\n", HAL_ADC_GetError(hadc));
        // adc3_dma->stop();
        // adc3_dma->start();
    }
}

#pragma arm section code  = "RAMCODE"

FDCAN_RxHeaderTypeDef FDCAN1_RxHeader;
uint32_t can_cnt = 0;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
 
    uint8_t i=0;
    uint8_t rxdata[8];
    if((RxFifo0ITs&FDCAN_IT_RX_FIFO0_NEW_MESSAGE)!=RESET)   //FIFO1新数据中断
    {
        //提取FIFO0中接收到的数据
        HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &FDCAN1_RxHeader, rxdata);
        // if(can1){
            can1->send_msg(0x18, CAN_STD, rxdata, FDCAN1_RxHeader.DataLength>>16);
        // }

        printf("id:%#x\r\n",FDCAN1_RxHeader.Identifier);
        printf("len:%d\r\n",FDCAN1_RxHeader.DataLength>>16);
        for(i=0;i<8;i++)
        printf("rxdata[%d]:%d\r\n",i,rxdata[i]);
        can_cnt++;
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
}

#pragma arm section






