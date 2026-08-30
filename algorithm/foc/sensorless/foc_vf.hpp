/**
 ****************************************************************************************************
 * @file        foc_vf.hpp
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

#ifndef _FOC_VF_H
#define _FOC_VF_H

#include "foc_type.h"
#include "foc_filter.hpp"
#include "foc_speed.hpp"

class foc_vf
{
private:
    /* data */
    qd_t Uqd;
    float current_elec_speed;

public:
    float step;                 /*单次步进*/
    float k;                    /*v/f比例系数*/
    float Uout;
    float acc;                  /*加速度*/
    float freq_th_h;            /*高频阈值*/
    float freq_th_l;            /*低频阈值*/
    foc_speed *spd;

public:
    foc_vf(float k, float freq_th_h, float freq_th_l, uint32_t pole_pairs, uint32_t gear_ratio);
    ~foc_vf();

    void SetElecSpeed(qd_t Uqd, float target_elec_speed, float Ts);
    float ElecAngleCalc(qd_t Uqd, float Ts);
    float ElecSpeedCalc(void);




    
};






#endif


