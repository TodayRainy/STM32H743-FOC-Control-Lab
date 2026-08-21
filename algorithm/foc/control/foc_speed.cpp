/**
 ****************************************************************************************************
 * @file        foc_speed.cpp
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

#include "foc_speed.hpp"
#include "foc_math.hpp"


#pragma arm section code  = "RAMCODE"


foc_speed::foc_speed(uint32_t pole_pairs, uint32_t gear_ratio, float offset)
{
    movefilter = new foc_movefilter<float>(500);
    movefilter->clear();

    memset(&speed_data, 0x00, sizeof(speed_data));

    this->speed_data.pole_pairs  = pole_pairs;
    this->speed_data.gear_ratio  = gear_ratio;
    this->speed_data.offset      = offset;

    this->elec_angle_pre    = 0;
    this->mech_angle_pre    = 0;
    this->out_angle_pre     = 0;

   
}

foc_speed::~foc_speed()
{
    delete(movefilter);
}

/*设置上一个电角度*/
void foc_speed::SetAnglePre(float elec_angle_pre, float mech_angle_pre, float out_angle_pre)
{
    this->elec_angle_pre    = elec_angle_pre;
    this->mech_angle_pre    = mech_angle_pre;
    this->out_angle_pre     = out_angle_pre;
}

void foc_speed::clear(void)
{
    movefilter->clear();
}

/*更新电角度*/
void foc_speed::AngleRefresh(float elec_angle)
{
    float mech_angle    = 0;                                /*机械角度*/
    float out_angle     = 0;

    float delta[3] = {0};                                   /* 角度差 */

    int32_t pole_pairs    = speed_data.pole_pairs;
    int32_t gear_ratio    = speed_data.gear_ratio;

    /*计算电角度*/
    elec_angle = Limit_Angle(elec_angle);
    delta[0] = elec_angle - elec_angle_pre;
    if (delta[0] > PI){
        delta[0] -= _2PI;
        speed_data.elec_round--;
    }
    if (delta[0] < -PI){
        delta[0] += _2PI;
        speed_data.elec_round++;
    }
    elec_angle_pre = elec_angle;

    /*圈数限制*/
    if(speed_data.elec_round > (pole_pairs-1)){
        speed_data.elec_round = 0;
    }
    if(speed_data.elec_round < 0){
        speed_data.elec_round = (pole_pairs-1);
    }

    /*计算机械角度*/
    mech_angle = Limit_Angle((elec_angle+((float)speed_data.elec_round*_2PI))/(float)pole_pairs);
    delta[1] = mech_angle - mech_angle_pre;
    if (delta[1] > PI) speed_data.mech_round--;        
    if (delta[1] < -PI) speed_data.mech_round++;
    mech_angle_pre = mech_angle;
    /*圈数限制*/
        if(speed_data.mech_round > (gear_ratio-1)){
        speed_data.mech_round = 0;
    }
    if(speed_data.mech_round < 0){
        speed_data.mech_round = (gear_ratio-1);
    }

    /*计算输出轴角度*/
    out_angle = Limit_Angle((mech_angle+(speed_data.mech_round*_2PI))/(float)gear_ratio);
    delta[2] = out_angle - out_angle_pre;
    if (delta[2] > PI) speed_data.out_round--;        
    if (delta[2] < -PI) speed_data.out_round++;
    out_angle_pre = out_angle;

    

    speed_data.elec_angle    = elec_angle;
    speed_data.mech_angle    = mech_angle;
    speed_data.out_angle     = out_angle;

    for (size_t i = 0; i < 3; i++){
        this->delta[i] = delta[i];
    }
}

/*速度计算 这个函数要紧跟AngleRefresh执行*/
void foc_speed::SpeedCalc(float Ts)
{
#if 1
    float elec_speed = 0;
    movefilter->fill(delta[0]/Ts);
    elec_speed = movefilter->calc();

    SpeedRefresh(elec_speed);
#else
    /*电角速度计算*/
    
    speed_data.elec_speed = delta[0]/Ts;
    /*机械速度计算*/
    speed_data.mech_speed = delta[1]/Ts*60.0f/_2PI;
    /*输出轴速度计算*/
    speed_data.out_speed = delta[2]/Ts*60.0f/_2PI;
    // SpeedRefresh(speed.elec_speed);

#endif


}

/*速度更新*/
void foc_speed::SpeedRefresh(float elec_speed)
{
    float pole_pairs        = speed_data.pole_pairs;
    float gear_ratio        = speed_data.gear_ratio;

    speed_data.elec_speed   = elec_speed;
    speed_data.mech_speed   = speed_data.elec_speed/pole_pairs;
    speed_data.out_speed    = speed_data.mech_speed/gear_ratio;
}


#pragma arm section
