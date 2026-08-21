/**
 ****************************************************************************************************
 * @file        foc_ctrl.cpp
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

#include "foc_ctrl.hpp"
#include "foc_math.hpp"

#pragma arm section code  = "RAMCODE"


foc_track::foc_track(float threshold)
{
    this->state     = 0;
    this->fix_obj   = 0;
    this->phase_val = 0;
    this->step      = 0;

    this->threshold = threshold;

}

foc_track::~foc_track()
{
}

/*T法曲线*/
inline float foc_track::T(float new_obj, float val, float track_ts, float Ts)
{

    if( fabsf(new_obj - fix_obj) > threshold){
        fix_obj = new_obj;
        phase_val = val;
        state = 1;
    }

    switch (state){
        case 0:                 /*稳定状态*/
            phase_val = new_obj;
            fix_obj = new_obj;
            /* code */
            break;
        case 1:                 /*计算变化步态*/
            step = (fix_obj-val)/track_ts*Ts;
            phase_val = val;
            state = 2;
            break;
        case 2:
            /*按照周期叠加步态*/
            phase_val += step;
            if(step>0){
                if(phase_val>new_obj){
                    phase_val = new_obj;
                    state = 0;
                }
            }else{
                if(phase_val<new_obj){
                    phase_val = new_obj;
                    state = 0;
                }
            }
            break;
        default:
            break;
    }

    return phase_val; 
}


float foc_track::run(float new_obj, float val, float track_ts, float Ts, TRACK_MODE_t mode)
{
    float phase_val = 0; 

    switch (mode){
        case TRACK_MODE_NONE:
            phase_val = new_obj;
            break;
         case TRACK_MODE_T:
            phase_val = T(new_obj, val, track_ts, Ts);
            break;
        default:
            break;
    }

    return phase_val;
}

foc_spd_ctrl::foc_spd_ctrl(float kp, float ki, float ref_limit, float i_term_limit, float out_limit)
{
    /*补齐漏初始化的成员: 对象由 new 分配, C++ 不会自动清零*/
    this->Ts = 0;

    this->state = 0;

    this->out.q = 0;
    this->out.d = 0;
    spd_pid = new foc_pid(kp, ki, 0, ref_limit, i_term_limit, out_limit);
    track   = new foc_track(out_limit*0.01f);

    run(0, 0, TRACK_MODE_NONE, 0, 0);
}

foc_spd_ctrl::~foc_spd_ctrl()
{
    delete(spd_pid);
    delete(track);
}

void foc_spd_ctrl::run(float target_speed, float current_speed, TRACK_MODE_t mode, float accel_ts, float Ts)
{
    this->Ts = Ts;

    target_speed = track->run(target_speed, current_speed, accel_ts, Ts, mode);

    out.q = spd_pid->pid(target_speed, current_speed, Ts);
    out.d = 0.00001f;                                   /*不能设置为0, 为了避免电流环pid函数里进入退饱和*/
}

qd_t foc_spd_ctrl::pid(void)
{
    return out;
}

void foc_spd_ctrl::clear(void)
{
    spd_pid->clear();
}

foc_pos_ctrl::foc_pos_ctrl(float kp, float ki, float kd, float ref_limit, float i_term_limit, float out_limit,
                            TC_POS_MODE_t pos_mode, TC_ROTATE_MODE_t rotate_mode)
{
    /*补齐漏初始化的成员: 对象由 new 分配, C++ 不会自动清零*/
    this->out = 0;

    state = 0;
    last_ref_pos = 0;

    this->pos_mode = pos_mode;
    this->rotate_mode = rotate_mode;

    pos_pid = new foc_pid(kp, ki, kd, ref_limit, i_term_limit, out_limit);
}

foc_pos_ctrl::~foc_pos_ctrl()
{
    delete(pos_pid);
}

float foc_pos_ctrl::dif_calc(float ref_pos, float fb_multi_pos, float fb_single_pos, TC_ROTATE_MODE_t rotate_mode)
{
    float dif = 0;
     switch(pos_mode){
        case TC_POS_CONTINUOUS:             /*连续模式，直接做差*/
            dif = ref_pos - fb_multi_pos;
            break;
        case TC_POS_WRAPPED:                /*环绕模式 按照顺时针逆时针进行计算*/

            ref_pos = Limit_Angle(ref_pos);
            fb_single_pos = Limit_Angle(fb_single_pos);

            switch (rotate_mode){
                case TC_ROTATE_INCREASE:
                    if(ref_pos>=fb_single_pos){
                        dif = ref_pos-fb_single_pos;
                    }else{
                        dif = ref_pos + (_2PI-fb_single_pos);
                    }
                    break;
                case TC_ROTATE_DECREASE:
                    if(ref_pos<=fb_single_pos){
                        dif = ref_pos-fb_single_pos;
                    }else{
                        dif = -(fb_single_pos + (_2PI-ref_pos));
                    }
                    break;
                case TC_ROTATE_SHORTEST_PATH:
                    dif = ref_pos - fb_single_pos;
                    if(dif>PI){
                        dif -= _2PI;
                    }else if(dif<-PI){
                        dif += _2PI;
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }

    return dif;
}

float foc_pos_ctrl::run(float ref_pos, float fb_multi_pos, float fb_single_pos, TC_ROTATE_MODE_t rotate_mode, float Ts)
{
    float dif = 0;

    if(fabsf(ref_pos-last_ref_pos)>0.5f){
        pos_pid->clear();
        state = 0;
    }
    last_ref_pos = ref_pos;

    switch (state){
        case 0:         /*初始化操作*/
            dif = dif_calc(ref_pos, fb_multi_pos, fb_single_pos, rotate_mode);
            state = 1;
            break;
        case 1:         /*运行中*/
            dif = dif_calc(ref_pos, fb_multi_pos, fb_single_pos, rotate_mode);
            if(fabsf(dif)<0.5f){            
                state = 2;
            }
            break;
        case 2:         /*到达目标位置，进行最短路径方式跟踪*/
            dif = dif_calc(ref_pos, fb_multi_pos, fb_single_pos, TC_ROTATE_SHORTEST_PATH);
            break;
        default:
            break;
    }

    // out = pos_pid->pi(dif, Ts);
    out = pos_pid->pid(dif, Ts);

    

    return out;
}

void foc_pos_ctrl::clear(void)
{
    pos_pid->clear();
}

#pragma arm section

