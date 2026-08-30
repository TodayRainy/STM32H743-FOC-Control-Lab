#include "bsp.hpp"

#include "main.h"
#include "tim.h"
#include "spi.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "foc_app.hpp"

/*USB初始化*/
extern "C" void MX_USB_Device_Init(void);

adc_dma     *adc3_dma           = NULL;
uint16_t    adc3_dmabuf[2*100];
w25qxx      *w25q64             = NULL;
mt6835gt    *mt6835gt_0         = NULL;
mt6701acd   *mt6701acd_0        = NULL;
bsp_can     *can1               = NULL;


#include "foc_hw.hpp"

extern "C" void BSP_Init(void)
{
    delay_init(480);

    Cache_Enable();                                     //打开L1-Cache

    MPU_Memory_Protection();

    HAL_GPIO_WritePin(MT6835_RST_GPIO_Port, MT6835_RST_Pin, GPIO_PIN_RESET);

    adc3_dma = new adc_dma(&hadc3, adc3_dmabuf, 2, 50);
    
    w25q64 = new w25qxx(&hspi4, SPI4_CS_GPIO_Port, SPI4_CS_Pin);


    mt6835gt_0 = new mt6835gt(&hspi2, SPI2_CS_GPIO_Port, SPI2_CS_Pin);

    mt6701acd_0 = new mt6701acd(&hi2c3, MT6701ACD_I2CADDR);

    printf("C++ BSP_Init: OK\r\n");
    //  while(1){
    //      float angle = 0;
    // //     mt6835gt_0->read_angle(angle);
    //       mt6701acd_0->read_angle(angle);

    //      printf("angle: %f 方向: %d\r\n", angle, mt6835gt_0->read_dir());
    //      HAL_Delay(50);
    // }

    // while (1);
    


    // HAL_TIM_Base_Start_IT(&htim1);
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    // HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    // HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    // HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    // A_H_SET_PWM(6000.0f *0.7f);
    // B_H_SET_PWM(6000.0f *0.5f);
    // C_H_SET_PWM(6000.0f *0.3f);

   // norflash_init();

    
    
    MX_USB_Device_Init();

    can1 = new bsp_can(&hfdcan1);

    FOC_Init();

    // uint8_t msg[8] = {};
    // while(1){
    //     msg[0]++;
    //     can1->send_msg(0x18, CAN_STD, msg, 8);
    //     HAL_Delay(1000);
    // }

    printf("C++ BSP_Init: OK\r\n");



    
}









