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


foc_track::foc_track()
{
    clear();
}

foc_track::~foc_track()
{
}

/*恒定变化率的线性轨迹*/
inline float foc_track::ramp(float new_obj, float val, float max_rate, float Ts)
{
    float delta;
    float max_step;

    if(initialized == 0U){
        phase_val = val;
        initialized = 1U;
    }

    delta = new_obj-phase_val;
    max_step = max_rate*Ts;
    if(delta>max_step){
        phase_val += max_step;
    }else if(delta<-max_step){
        phase_val -= max_step;
    }else{
        phase_val = new_obj;
    }

    return phase_val;
}


float foc_track::run(float new_obj, float val, float max_rate, float Ts, TRACK_MODE_t mode)
{
    switch (mode){
        case TRACK_MODE_NONE:
            this->phase_val = new_obj;
            initialized = 1U;
            break;
        case TRACK_MODE_RAMP:
            (void)ramp(new_obj, val, max_rate, Ts);
            break;
        default:
            break;
    }

    return this->phase_val;
}

void foc_track::clear(void)
{
    initialized = 0U;
    phase_val = 0.0f;
}

foc_spd_ctrl::foc_spd_ctrl(float kp, float ki, float ref_limit, float i_term_limit, float out_limit)
{
    /*补齐漏初始化的成员: 对象由 new 分配, C++ 不会自动清零*/
    this->Ts = 0;

    this->state = 0;

    this->out.q = 0;
    this->out.d = 0;
    this->target_ref = 0;
    this->track_mode = TRACK_MODE_NONE;
    this->accel_limit = 0;
    spd_pid = new foc_pid(kp, ki, 0, ref_limit, i_term_limit, out_limit);
    track   = new foc_track();

    run(0, 0, 0);
}

foc_spd_ctrl::~foc_spd_ctrl()
{
    delete(spd_pid);
    delete(track);
}

void foc_spd_ctrl::run(float target_speed, float current_speed, float Ts)
{
    this->Ts = Ts;

    target_ref = track->run(target_speed, current_speed, accel_limit, Ts, track_mode);

    out.q = spd_pid->pid(target_ref, current_speed, Ts);
    out.d = 0.00001f;                                   /*不能设置为0, 为了避免电流环pid函数里进入退饱和*/
}

void foc_spd_ctrl::set_gains(float kp, float ki)
{
    spd_pid->set_gains(kp, ki, 0.0f);
}

void foc_spd_ctrl::set_track(TRACK_MODE_t mode, float accel_limit)
{
    this->track_mode = mode;
    this->accel_limit = accel_limit;
    track->clear();
}

qd_t foc_spd_ctrl::pid(void)
{
    return out;
}

void foc_spd_ctrl::clear(void)
{
    spd_pid->clear();
    track->clear();
    target_ref = 0.0f;
}

float foc_spd_ctrl::reference(void) const
{
    return target_ref;
}

foc_pos_ctrl::foc_pos_ctrl(float kp, float ki, float kd, float ref_limit, float i_term_limit, float out_limit,
                            TC_POS_MODE_t pos_mode, TC_ROTATE_MODE_t rotate_mode)
{
    this->out = 0;
    this->target_ref = 0;
    this->pos_mode = pos_mode;
    this->rotate_mode = rotate_mode;
    this->track_mode = TRACK_MODE_NONE;
    this->velocity_limit = 0;

    pos_pid = new foc_pid(kp, ki, kd, ref_limit, i_term_limit, out_limit);
    track = new foc_track();
}

foc_pos_ctrl::~foc_pos_ctrl()
{
    delete(pos_pid);
    delete(track);
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
    target_ref = track->run(ref_pos, fb_multi_pos, velocity_limit, Ts, track_mode);
    const float dif = dif_calc(target_ref, fb_multi_pos, fb_single_pos, rotate_mode);
    out = pos_pid->pid(dif, Ts);
    return out;
}

void foc_pos_ctrl::set_gains(float kp, float ki, float kd)
{
    pos_pid->set_gains(kp, ki, kd);
}

void foc_pos_ctrl::set_track(TRACK_MODE_t mode, float velocity_limit)
{
    this->track_mode = mode;
    this->velocity_limit = velocity_limit;
    track->clear();
}

void foc_pos_ctrl::clear(void)
{
    pos_pid->clear();
    track->clear();
    out = 0.0f;
    target_ref = 0.0f;
}

float foc_pos_ctrl::reference(void) const
{
    return target_ref;
}

#pragma arm section
