/**
 ****************************************************************************************************
 * @file        foc_math.cpp
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

#include "foc_math.hpp"

#include <math.h>
#include <stdlib.h>


#pragma arm section code  = "RAMCODE"

/**
 * @brief 符号判断
 *
 * @param val
 * @return int8_t
 */
int8_t FOC_SIGN_Judgment(float val)
{
    if(val>=0)return 1;
    else return 0;
}

#if 0
/**
 * @brief 克拉克变换
 *
 * @param input 三相输入
 * @return alphabeta_t 返回αβ轴
 */
alphabeta_t FOC_Clarke(abc_t input)
{
    alphabeta_t output = {0};

    /* qIalpha = qIas*/
    /*qIbeta = -(2*qIbs+qIas)/sqrt(3)*/
#if 0
    /*没有等幅值变换*/
    output.alpha = 1.5f * input.a;
    output.beta = SQRT_3 *(0.5f*input.a + input.b);
#else
    /*等幅值变换 */
    output.alpha = input.a;
    output.beta = ((input.a + 2.0f*input.b)/SQRT_3);
#endif
    return output;
}


/**
 * @brief 帕克变换
 *
 * @param input αβ轴输入
 * @param elec_angle 电角度
 * @return qd_t 返回qd轴
 */
qd_t Foc_Park(alphabeta_t input, float elec_angle)
{
    qd_t output = {0};

    float _cos = arm_cos_f32(elec_angle);
    float _sin = arm_sin_f32(elec_angle);

    output.q = (_cos*input.beta -  _sin*input.alpha);
    output.d = (_cos*input.alpha + _sin*input.beta);

    return output;
}

/**
 * @brief 帕克逆变换
 *
 * @param input qd轴
 * @param elec_angle 电角度
 * @return alphabeta_t 返回αβ轴
 */
alphabeta_t FOC_Rev_Park(qd_t input, float elec_angle)
{
    alphabeta_t output = {0};

    float _cos = arm_cos_f32(elec_angle);
    float _sin = arm_sin_f32(elec_angle);

    /*派克逆变换
    Iα = Id*cosθ - Iq*sinθ
    Iβ = Iq*cosθ + Id*sinθ 我们取Id为0的情况*/
    output.alpha = (_cos* input.d - _sin* input.q);
    output.beta  = (_cos* input.q + _sin* input.d);

    return output;
}

/*克拉克逆变换*/

/**
 * @brief 克拉克逆变换
 *
 * @param input αβ轴
 * @return abc_t 返回三相
 */
abc_t FOC_Rev_Clarke(alphabeta_t input)
{
    abc_t output = {0};
/*
    Ia = Iα;
    Ib = (√3*Iβ-Iα)/2
    Ic = (-Iα-√3*Iβ)/2
*/
    output.a = input.alpha;
    output.b = (SQRT_3*(input.beta) -input.alpha)/2.0f;
    output.c = (-input.alpha - SQRT_3*(input.beta))/2.0f;

    // output.a /= 1.5f;
    // output.b /= 1.5f;
    // output.c /= 1.5f;


    return output;
}


/**
 * @brief 限制角度在0-2PI之间
 *
 * @param elec_angle 输入角度
 * @return float 返回角度
 */
float Limit_Angle(float elec_angle)
{
    #if 1
    while(elec_angle>_2PI||elec_angle<0){
        if(elec_angle>_2PI){
            elec_angle -= _2PI;
        }else if(elec_angle<0){
            elec_angle += _2PI;
        }
    }
    #else

    elec_angle = fmod(elec_angle, _2PI);

    if(elec_angle<0){
        elec_angle += _2PI;
    }
    #endif

    return elec_angle;
}

#endif

/**
 * @brief 计算电角度差
 *
 * @param agnle1 角度1
 * @param angle2 角度2
 * @return float  返回角度差
 */
float Angle_Err(float agnle1, float angle2)
{
    float err = Limit_Angle(agnle1 - angle2 + _2PI);

    if (err > PI)           // 限制误差到 [-π, π]
        err -= _2PI;
    return err;
}


/*速度转换*/
float Speed_Transf(float target_speed, speed_t speed_data,SPD_TYPE_t type)
{
    float pole_pairs = speed_data.pole_pairs;
    float gear_ratio = speed_data.gear_ratio;

    /*所有的速度要改成电角速度进行计算*/
    switch (type){
        /*弧度/秒*/
        case ELEC_SPD_RAD_S:
            break;
        case MECH_SPD_RAD_S:
            target_speed = target_speed*pole_pairs;
            break;
        case OUT_SPD_RAD_S:
            target_speed = target_speed*pole_pairs*gear_ratio;
            break;

        /*圈数/秒*/
        case ELEC_SPD_R_S:
            target_speed = target_speed/_2PI;
            break;
        case MECH_SPD_R_S:
            target_speed = target_speed*pole_pairs/_2PI;
            break;
        case OUT_SPD_R_S:
            target_speed = target_speed*pole_pairs*gear_ratio/_2PI;
            break;

        /*圈数/分*/
        case ELEC_SPD_R_M:
            target_speed = target_speed/_2PI*60.0f;
            break;
        case MECH_SPD_R_M:
            target_speed = target_speed*pole_pairs/_2PI*60.0f;
            break;
        case OUT_SPD_R_M:
            target_speed = target_speed*pole_pairs*gear_ratio/_2PI*60.0f;
            break;
        default:
            break;
    }

    return target_speed;
}


/*快速反正切*/
float utils_fast_atan2(float y, float x) {
    if (x == 0.0f && y == 0.0f) {
        return 0.0f;
    }

    const float abs_y = fabsf(y) + 1e-20f;
    const float a = 0.1963f;
    const float b = 0.9817f;
    const float pi_4 = 0.7853981634f;
    const float pi_34 = 2.3561944902f;

    float r, angle;

    if (x >= 0.0f) {
        r = (x - abs_y) / (x + abs_y);
        angle = (a * r * r - b) * r + pi_4;
    } else {
        r = (x + abs_y) / (abs_y - x);
        angle = (a * r * r - b) * r + pi_34;
    }

    return (y < 0.0f) ? -angle : angle;
}




















#pragma arm section

