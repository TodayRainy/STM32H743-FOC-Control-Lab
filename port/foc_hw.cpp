/**
 ****************************************************************************************************
 * @file        foc_hw.cpp
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.1
 * @date        2026-03-27
 *
 * @license
 * Copyright (c) 2026 哔哩哔哩-Rebron大侠
 *
 * This software is provided for educational and research purposes only.
 *
 * Permission is hereby granted to use, copy, modify, and distribute this
 * software for NON-COMMERCIAL purposes only.
 *
 * Commercial use is strictly prohibited without explicit written permission
 * from the author.
 *
 * Commercial use includes, but is not limited to:
 * - Using this software in commercial products
 * - Selling software or hardware containing this code
 * - Providing paid services based on this software
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
 *
 * 本软件仅供学习与研究使用。
 *
 * 未经作者书面许可，禁止用于任何商业用途，包括但不限于：
 * - 用于商业产品
 * - 销售包含本代码的软硬件
 * - 基于本软件提供收费服务
 *
 * 作者不对任何使用行为负责。
 ****************************************************************************************************
 */



#include "foc_hw.hpp"
#include "foc_motor.hpp"
#include "foc_math.hpp"
#include "foc_encoder_math.hpp"

#include "bsp.hpp"
#include <stdio.h>

#pragma arm section code  = "RAMCODE"


/***************************************PWM硬件相关**************************************/
#include "tim.h"
/**
 * @brief PWM硬件初始化
 * 
 */
void FOC_PWM_HW_Init(void)
{
    A_H_SET_PWM(0);
    B_H_SET_PWM(0);
    C_H_SET_PWM(0);

    /* Keep the power stage disabled until a mode transition arms it. */
    FOC_PWM_HW_ON_OFF(false, false, false, false, false, false);
}

void FOC_PWM_HW_StartControl(void)
{
    /* TIM1 also drives the injected ADC trigger. It must start only after
       every object referenced by the control ISR has been assigned. */
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
}

void FOC_PWM_HW_DeInit(void)
{   
    HAL_TIM_Base_Stop_IT(&htim1);
    HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);

    FOC_PWM_HW_ON_OFF(false,false,false,false,false,false);       /*关闭硬件PWM*/      
}


/**
 * @brief PWM硬件通道开关
 * 
 * @param A     A相上管通道
 * @param A_N   A相下管通道
 * @param B     B相上管通道
 * @param B_N   B相下管通道
 * @param C     C相下管通道
 * @param C_N   C相下管通道
 */
void FOC_PWM_HW_ON_OFF(bool A, bool A_N, bool B, bool B_N, bool C, bool C_N)
{
    if(A)   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    else    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

    if(A_N) HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    else    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    if(B)   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    else    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);

    if(B_N) HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    else    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

    if(C)   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    else    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

    if(C_N) HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    else    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);


}






/***************************************ABZ编码器硬件相关**************************************/
void FOC_ABZENC_HW_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);         /*开启定时器编码器模式*/
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);             /*开启定时器捕获, 捕获编码器Z相*/
}

void FOC_ABZENC_HW_DeInit(void)
{
    HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);          /*开启定时器编码器模式*/
    HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_4);              /*开启定时器捕获, 捕获编码器Z相*/
}

/**
 * @brief ABZ编码器获取角度
 * 
 * @param enc 
 * @return float 返回机械角度
 */
float FOC_ABZENC_HW_GetMechAngle(void)
{
    const uint32_t tick = __HAL_TIM_GET_COUNTER(&htim3);
    return foc_encoder_count_to_mech_angle(tick, htim3.Init.Period);
}


/**
 * @brief 获取编码器计数方向 0递增 1递减
 * 
 * @param enc 
 * @return int8_t 返回1:向上计数  返回-1:向下计数
 */
int8_t FOC_ABZENC_HW_DIR(void)
{
    int8_t dir = 1;
    if(0==READ_BIT(TIM3->CR1, TIM_CR1_DIR)){
        dir = 1;
    }else{
        dir = -1;
    }
    return dir;
}

bool trigger_z = false;         /*Z相触发标志*/
uint32_t z_over_cnt = 0;        /*Z相触发次数*/
/**
 * @brief ABZ编码器中断中调用
 * 
 * @param enc 
 */
void FOC_ABZENC_HW_IRQHandler(void)
{    
    /*定位Z轴*/
    if(FOC_ABZENC_HW_DIR()==1){
        __HAL_TIM_SET_COUNTER(&htim3, 0);                       
    }else{
        __HAL_TIM_SET_COUNTER(&htim3, htim3.Init.Period);       
    }

    /*也可以直接定位到中间*/
    // __HAL_TIM_SET_COUNTER(&htim3, htim3.Init.Period/2);

    /*定时器溢出了*/
    if(trigger_z==true){
        z_over_cnt += 1;
    }else{
        trigger_z = true;
        printf("ABZ编码器已对齐过一次Z轴: OK\r\n");
    }
}


/***************************************ABS编码器硬件相关**************************************/
void FOC_ABSENC_HW_Init(void)
{

}

void FOC_ABSENC_HW_DeInit(void)
{

}

/**
 * @brief 获取ABS编码器机械角度
 * 
 * @return float 机械角度
 */
float FOC_ABSENC_HW_GetMechAngle(void)
{
    float angle = 0;
    mt6835gt_0->read_angle(angle);

    return angle;
}


/***************************************时间相关**************************************/
void FOC_TIME_HW_Init(void)
{
    HAL_TIM_Base_Start(&htim7); 
}

void FOC_TIME_HW_DeInit(void)
{
    HAL_TIM_Base_Stop(&htim7); 
}


uint32_t FOC_TIME_GetCount(void)
{
    uint32_t tick = __HAL_TIM_GET_COUNTER(&htim7);

    return tick;
}

/*获取定时器的时间*/
float FOC_TIME_GetSecond(void)
{
    float second = 0;
    float htim7_freq = 240000000/(htim7.Init.Prescaler+1);

    float tick = __HAL_TIM_GET_COUNTER(&htim7);

    second = tick/htim7_freq;

    return second;
}

void FOC_TIME_SetCount(uint32_t tick)
{
    __HAL_TIM_SET_COUNTER(&htim7, tick);
}


/***************************************电流采样相关**************************************/
#include "adc.h"
/**
 * @brief 电流部分的硬件初始化
 * 
 */
void FOC_CURRENT_HW_Init(void)
{   
    /*ADC3用于母线电压和电位计采集*/
    HAL_ADCEx_Calibration_Start(&hadc1, LL_ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_Delay(100);
    HAL_ADCEx_Calibration_Start(&hadc2, LL_ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_Delay(100);
    // HAL_ADCEx_Calibration_Start(&hadc3, LL_ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    // HAL_Delay(100);

    /*注入通道方式*/
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    // HAL_ADCEx_InjectedStart_IT(&hadc3);
}

/**
 * @brief 电流部分的硬件复位
 * 
 */
void FOC_CURRENT_HW_DeInit(void)
{
    HAL_ADCEx_InjectedStop_IT(&hadc1);
    HAL_ADCEx_InjectedStop_IT(&hadc2);
    // HAL_ADCEx_InjectedStop_IT(&hadc3);
}



/*电阻采样*/
float Resist_Calc_I(float mcu_ad)
{
    /*运放放大6倍后抬升1.65V, 5m欧电阻采样*/
    float Uadc = mcu_ad/4096.0f *3.3f - 1.65f;
    float I = Uadc/6.0f/0.005f;
    return -I;
}

/*电流传感器采样*/
float CC6921SO_Calc_I(uint32_t mcu_ad)
{
    float Uout;
    float Uadc;     
    float I;

    Uadc = (float)mcu_ad/4096.0f *3.3f;

    Uout = Uadc *3.0f /2.0f;   

    I = (Uout-2.5f)/0.02f;       /*电流计算  Uout = I*0.04*+0.25 */
    return I;
}

void FOC_CURRENT_REFRESH(abc_t &mcu_ad, abc_t &ad_offset, abc_t &Iabc)
{
    mcu_ad.a = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) - ad_offset.a;
    mcu_ad.b = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1) - ad_offset.b;
    // mcu_ad.c = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1) - ad_offset.c;

    /*计算三相电流*/
    Iabc.a = Resist_Calc_I(mcu_ad.a);
    Iabc.b = Resist_Calc_I(mcu_ad.b);
    // Iabc.c = Resist_Calc_I(mcu_ad.c);
    Iabc.c = -(Iabc.a + Iabc.b);
}


/***************************************中断相关**************************************/
#include "foc_app.hpp"
/*定时器周期中断*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ 
}

/*定时器捕获中断*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM4){
        /*触发了编码器Z相*/
        FOC_ABZENC_HW_IRQHandler();
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin==ENC_Z_Pin){
        /*触发了编码器Z相*/

    }
}


/*****************ADC************************* */
static bool adc_flag[3] = {0};
/*ADC注入中断*/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc==&hadc1){
        adc_flag[0] = true;
    }else if(hadc==&hadc2){
        adc_flag[1] = true;
    }else if(hadc==&hadc3){
        adc_flag[2] = true;
    }

    adc_flag[2] = true;
    if(adc_flag[0]==true&&adc_flag[1]==true&&adc_flag[2]==true){

        adc_flag[0] = false;
        adc_flag[1] = false;
        adc_flag[2] = false;
        
        FOC_RUN();
    }
}


/*输出捕获中断*/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{   
     if(htim->Instance==TIM1){
        FOC_SPEED_CALC();
        FOC_SPEED_CTRL();
        FOC_POS_CTRL();
    }
}







#pragma arm section








