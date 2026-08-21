/**
 ****************************************************************************************************
 * @file        foc_filter.cpp
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
#include "foc_filter.hpp"
#include "foc_math.hpp"



#pragma arm section code  = "RAMCODE"


foc_pll::foc_pll(float kp, float ki)
{
    this->kp = kp;
    this->ki = ki;

    this->p_term = 0;
    this->i_term = 0;
    this->err = 0;
    this->omega = 0;
    this->omega_f = 0;
    this->omega_max = 0;
    this->angle = 0;
    this->angle_last = 0;

    movefilter = new foc_movefilter<float>(100);
}

foc_pll::~foc_pll()
{
    delete(movefilter);
}

void foc_pll::clear(void)
{
    movefilter->clear();

    this->p_term = 0;
    this->i_term = 0;
    this->err = 0;
    this->omega = 0;
    this->omega_f = 0;
    this->omega_max = 0;
    this->angle = 0;
    this->angle_last = 0;
}

/*滑模锁相环*/
float foc_pll::calc_smo(float alpha, float beta, float Ts)
{
    float _cos, _sin;
    
    _sin = arm_sin_f32(angle);
    _cos = arm_cos_f32(angle);

    err = -alpha * _cos - beta * _sin;

    p_term = kp *err;
    i_term += ki * err * Ts; 

    omega = p_term+i_term;

    omega_f = omega_f*0.9f+omega*0.1f;

    angle += omega_f*Ts;

    angle = Limit_Angle(angle);

    movefilter->fill(omega_f);

    return angle;
}

/*磁链锁相环*/
float foc_pll::calc_flux(float alpha, float beta, float Ts)
{
    float _cos, _sin; 

    _sin = arm_sin_f32(angle);
    _cos = arm_cos_f32(angle);

    err = -alpha * _sin + beta * _cos;

    p_term = kp *err;
    i_term += ki * err;

    omega = p_term+i_term;

    omega_f = omega_f*0.9f+omega*0.1f;
    // omega_f = omega;

    angle += omega_f*Ts;

    angle = Limit_Angle(angle);

    movefilter->fill(omega_f);
    return angle;
}

/*高频注入锁相环*/
float foc_pll::calc_hfi(float alpha, float beta, float Ts)
{
    float _cos, _sin;

    _sin = arm_sin_f32(angle);
    _cos = arm_cos_f32(angle);

    err = -alpha * _sin + beta * _cos;
    p_term = kp *err;
    i_term += ki * err;

    omega = p_term+i_term;

    omega_f = omega_f*0.9f+omega*0.1f;

    angle += omega_f*Ts;

    angle = Limit_Angle(angle);

    movefilter->fill(omega_f);
    return angle;
}


#pragma arm section



