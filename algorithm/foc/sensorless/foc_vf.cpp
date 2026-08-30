/**
 ****************************************************************************************************
 * @file        foc_vf.cpp
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

#include "foc_vf.hpp"
#include "foc_math.hpp"

 
#pragma arm section code  = "RAMCODE"


foc_vf::foc_vf(float k, float freq_th_h, float freq_th_l, uint32_t pole_pairs, uint32_t gear_ratio)
{
    /*补齐漏初始化的成员: 对象由 new 分配, C++ 不会自动清零*/
    this->Uout = 0;

    memset(&this->spd, 0x00, sizeof(this->spd));

    this->k                     = k;
    this->step                  = 0.0f;
    this->current_elec_speed    = 0.0f;
    this->acc                   = 100.0f;
    this->freq_th_h             = freq_th_h;
    this->freq_th_l             = freq_th_l;

    spd = new foc_speed(pole_pairs, gear_ratio, 0);

}

foc_vf::~foc_vf()
{
    delete(spd);
}

/*设置电角速度*/
void foc_vf::SetElecSpeed(qd_t Uqd, float target_elec_speed, float Ts)
{
    float delta = acc * Ts;

    if (current_elec_speed < target_elec_speed) {
        current_elec_speed += delta;
        if (current_elec_speed > target_elec_speed)
            current_elec_speed = target_elec_speed;
    } else {
        current_elec_speed -= delta;
        if (current_elec_speed < target_elec_speed)
            current_elec_speed = target_elec_speed;
    }
}


/**
 * @brief VF角度计算
 * 
 * @return float 电角度
 */
float foc_vf::ElecAngleCalc(qd_t Uqd, float Ts)
{
    float elec_angle;
    float freq_hz;
    float voltage;

    
    step = current_elec_speed * Ts;
    // 修正1：取绝对频率
    freq_hz = fabsf(current_elec_speed) / _2PI;
    voltage = k * freq_hz;
    // 修正2：低频提升（关键！）
    if (freq_hz < freq_th_l) {
        voltage += 1.00f * k *freq_th_l* ((freq_th_l - freq_hz) / freq_th_l);
    }
    // 修正3：弱磁处理（频率>50Hz时电压不再增加）
    if (freq_hz > freq_th_h) {
        voltage = k * freq_th_h;
    }
    if(step==0)voltage = 0;
    

    /*角度计算*/
    elec_angle = Limit_Angle(spd->speed_data.elec_angle+step);
    spd->AngleRefresh(elec_angle);
    spd->SpeedCalc(Ts);

    
    Uout = voltage;
    return elec_angle;
}

/**
 * @brief           VF速度计算
 * 
 * @param t         周期时间
 * @return float    返回速度 rad/s
 */
float foc_vf::ElecSpeedCalc(void)
{
    return spd->speed_data.elec_speed;
}


#pragma arm section
