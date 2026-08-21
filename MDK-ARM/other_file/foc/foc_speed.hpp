/**
 ****************************************************************************************************
 * @file        foc_speed.hpp
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

#ifndef _FOC_SPEED_HPP
#define _FOC_SPEED_HPP


#include "foc_filter.hpp"

class foc_speed
{

public:
    speed_t speed_data;
private:
    /* data */
    float elec_angle_pre;
    float mech_angle_pre;
    float out_angle_pre;

    
    float delta[3];             /*角度差, 电角度，机械角度，输出轴角度*/

    foc_movefilter<float> *movefilter;
public:
    foc_speed(uint32_t pole_pairs, uint32_t gear_ratio, float offset);
    ~foc_speed();

    void clear(void);

    /*初始化上一个角度*/
    void SetAnglePre(float elec_angle_pre, float mech_angle_pre, float out_angle_pre);   

    /*更新电角度*/
    void AngleRefresh(float elec_angle);
    /*速度更新*/
    void SpeedRefresh(float elec_speed);

    /*速度计算 需要更新了角度之后才能计算, 如果不用角度差的方法计算速度, 可以只使用速度更新接口*/
    void SpeedCalc(float Ts);

};



















#endif
