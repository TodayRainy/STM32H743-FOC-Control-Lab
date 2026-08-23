/**
 ****************************************************************************************************
 * @file        foc_pwm.hpp
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


#ifndef _FOC_PWM_H
#define _FOC_PWM_H


#include "foc_type.h"
#include <stdint.h>





/*PWM发波模式*/
typedef enum{
    SPWM_MODE = 0,
    SVPWM_MODE,
    TEST_MODE,
}PWM_MODE_t;


class foc_pwm
{
private:

    /* data */
    uint8_t sector;                     /* 当前扇区*/
    uint8_t dtc_state;

    float ratio_max;                    /* 最大调制比 */
    float svpwm_val1;
    float svpwm_val2;

    abc_t voltage_drop;


    inline uint8_t sector_get(alphabeta_t U_alphabeta);
    inline uint8_t sector_get_atan(alphabeta_t U_alphabeta);

    inline void vector_time_calc(uint8_t sector, alphabeta_t alphabeta, float Tpwm, float Udc);

    inline qd_t voltage_limit(qd_t U_qd, float Udc, PWM_MODE_t mode);
    inline void svpwm(qd_t U_qd, float elec_angle, float Udc);
    inline void spwm(qd_t U_qd, float elec_angle, float Udc);
    
    

public:
    foc_pwm(uint32_t Tpwm);
    ~foc_pwm();

    qd_t run(qd_t Uqd, float elec_angle, float Udc, PWM_MODE_t mode);

    qd_t run(qd_t Uqd, float elec_angle, abc_t Iabc, float Udc, PWM_MODE_t mode, float Ts);

    uint32_t Tpwm;                              /* 定时器的周期值, 中央对齐方式*/
    uint32_t Tpwm_half;                         /* 定时器的周期值一半，实际的最大值*/
    float voltage_scale;                        /* 实际电压矢量/请求电压矢量 */
    alphabeta_t Debug_Ualphabeta;               /* 输出的Uαβ*/
    abc_t Debug_Uabc;                           /* 输出三相Uabc*/
    abc_t Debug_Tabc;                           /* 周期的占空比*/
};













#endif
















