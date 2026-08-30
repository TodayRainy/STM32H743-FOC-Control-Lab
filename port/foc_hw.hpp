/**
 ****************************************************************************************************
 * @file        foc_hw.hpp
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


#ifndef __FOC_HW_H
#define __FOC_HW_H

#include <stdbool.h>
#include <stdint.h>

#include <tim.h>

/*PWM接口 根据PWM模式来决定输出值是否要互补*/
#if 1
/*PWM模式2*/
#define A_H_SET_PWM(x)       (htim1.Instance->CCR3 =htim1.Instance->ARR-x)
#define B_H_SET_PWM(x)       (htim1.Instance->CCR2 =htim1.Instance->ARR-x)
#define C_H_SET_PWM(x)       (htim1.Instance->CCR1 =htim1.Instance->ARR-x)
#else
/*PWM模式1*/
#define A_H_SET_PWM(x)       (htim1.Instance->CCR3 =x)
#define B_H_SET_PWM(x)       (htim1.Instance->CCR2 =x)
#define C_H_SET_PWM(x)       (htim1.Instance->CCR1 =x)
#endif
#define U_L_SET_PWM(x)       (x)
#define V_L_SET_PWM(x)       (x)
#define W_L_SET_PWM(x)       (x)


/*霍尔接口*/
#define HALL_U_GET             HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin)
#define HALL_V_GET             HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin)
#define HALL_W_GET             HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin) 

void FOC_PWM_HW_Init(void);
void FOC_PWM_HW_StartControl(void);
void FOC_PWM_HW_DeInit(void);
void FOC_PWM_HW_ON_OFF(bool A, bool A_N, bool B, bool B_N, bool C, bool C_N);

#include <foc_abzenc.hpp>
void FOC_ABZENC_HW_Init(void);
void FOC_ABZENC_HW_DeInit(void);
void FOC_ABZENC_HW_IRQHandler(void);
float FOC_ABZENC_HW_GetMechAngle(void);
uint32_t FOC_ABZENC_HW_GetCount(void);
uint32_t FOC_ABZENC_HW_GetCountsPerRevolution(void);
int8_t FOC_ABZENC_HW_DIR(void);


#include <foc_current.hpp>
void FOC_CURRENT_HW_Init(void);
void FOC_CURRENT_HW_DeInit(void);
void FOC_CURRENT_REFRESH(abc_t &mcu_ad, abc_t &ad_offset, abc_t &Iabc);





#include <foc_absenc.hpp>
float FOC_ABSENC_HW_GetMechAngle(void);

#include <foc_time.hpp>
uint32_t FOC_TIME_GetCount(void);
float FOC_TIME_GetSecond(void);
void FOC_TIME_SetCount(uint32_t tick);
void FOC_TIME_HW_Init(void);
void FOC_TIME_HW_DeInit(void);

#endif
