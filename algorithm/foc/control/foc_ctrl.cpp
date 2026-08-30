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
#include <math.h>

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

/*限速度、限加速度的位置轨迹；短行程自然退化为三角形速度轨迹*/
inline float foc_track::trapezoid(float new_obj, float val, float max_rate,
                                  float max_acceleration, float Ts)
{
    float delta;
    float desired_rate;
    float next_rate;
    float next_phase;
    float max_rate_step;
    float direction;
    float directed_rate;
    float stopping_distance;

    if(initialized == 0U){
        phase_val = val;
        rate_val = 0.0f;
        command_val = new_obj;
        braking = 0U;
        initialized = 1U;
    }

    if((Ts <= 0.0f) || (max_rate <= 0.0f) || (max_acceleration <= 0.0f)){
        phase_val = new_obj;
        rate_val = 0.0f;
        return phase_val;
    }

    if(new_obj != command_val){
        command_val = new_obj;
        braking = 0U;
    }

    delta = new_obj-phase_val;
    direction = (delta >= 0.0f) ? 1.0f : -1.0f;
    directed_rate = direction*rate_val;

    if(braking != 0U){
        max_rate_step = brake_acceleration*Ts;
        if(fabsf(rate_val) <= max_rate_step){
            phase_val = new_obj;
            rate_val = 0.0f;
            braking = 0U;
        }else{
            next_rate = rate_val
                      - ((rate_val > 0.0f) ? max_rate_step : -max_rate_step);
            phase_val += 0.5f*(rate_val+next_rate)*Ts;
            rate_val = next_rate;
        }
        return phase_val;
    }

    if((rate_val == 0.0f)
    && (fabsf(delta) <= 0.5f*max_acceleration*Ts*Ts)){
        phase_val = new_obj;
        return phase_val;
    }

    stopping_distance = (directed_rate > 0.0f)
                      ? directed_rate*directed_rate/(2.0f*max_acceleration)
                      : 0.0f;

    /*提前一个离散步进入制动段，再用固定减速度精确收口到零速。*/
    if((directed_rate > 0.0f)
    && (fabsf(delta) <= stopping_distance
                          + directed_rate*Ts
                          + 0.5f*max_acceleration*Ts*Ts)){
        brake_acceleration = directed_rate*directed_rate
                           / (2.0f*fabsf(delta));
        brake_acceleration = LIMIT_RANGE(brake_acceleration, 0.0f,
                                         max_acceleration);
        braking = 1U;
        max_rate_step = brake_acceleration*Ts;
        if(fabsf(rate_val) <= max_rate_step){
            phase_val = new_obj;
            rate_val = 0.0f;
            braking = 0U;
        }else{
            next_rate = rate_val
                      - ((rate_val > 0.0f) ? max_rate_step : -max_rate_step);
            phase_val += 0.5f*(rate_val+next_rate)*Ts;
            rate_val = next_rate;
        }
        return phase_val;
    }

    desired_rate = direction*max_rate;

    max_rate_step = max_acceleration*Ts;
    next_rate = rate_val
              + LIMIT_RANGE(desired_rate-rate_val, -max_rate_step, max_rate_step);
    next_phase = phase_val + 0.5f*(rate_val+next_rate)*Ts;
    phase_val = next_phase;
    rate_val = next_rate;

    return phase_val;
}

void foc_track::scurve_plan(float new_obj, float val, float max_rate,
                            float max_acceleration, float max_jerk)
{
    static const float jerk_sign[7] = {
        1.0f, 0.0f, -1.0f, 0.0f, -1.0f, 0.0f, 1.0f
    };
    const float distance = fabsf(new_obj-val);
    const float direction = (new_obj >= val) ? 1.0f : -1.0f;
    const float time_to_acceleration = max_acceleration/max_jerk;
    const float velocity_at_acceleration =
        max_acceleration*time_to_acceleration;
    float jerk_time;
    float acceleration_time = 0.0f;
    float cruise_time = 0.0f;

    command_val = new_obj;
    phase_val = val;
    rate_val = 0.0f;
    acceleration_val = 0.0f;
    scurve_elapsed = 0.0f;
    scurve_pending = 0U;

    if(distance <= 1.0e-7f){
        phase_val = new_obj;
        scurve_active = 0U;
        scurve_total_time = 0.0f;
        return;
    }

    if(max_rate >= velocity_at_acceleration){
        const float acceleration_to_velocity =
            max_rate/max_acceleration-time_to_acceleration;
        const float distance_to_velocity = max_rate
            *(2.0f*time_to_acceleration+acceleration_to_velocity);
        const float distance_to_acceleration = 2.0f*max_acceleration
            *time_to_acceleration*time_to_acceleration;

        if(distance >= distance_to_velocity){
            jerk_time = time_to_acceleration;
            acceleration_time = acceleration_to_velocity;
            cruise_time = (distance-distance_to_velocity)/max_rate;
        }else if(distance >= distance_to_acceleration){
            jerk_time = time_to_acceleration;
            acceleration_time = 0.5f*(
                sqrtf(time_to_acceleration*time_to_acceleration
                      + 4.0f*distance/max_acceleration)
                - 3.0f*time_to_acceleration
            );
        }else{
            jerk_time = cbrtf(distance/(2.0f*max_jerk));
        }
    }else{
        const float time_to_velocity = sqrtf(max_rate/max_jerk);
        const float distance_to_velocity = 2.0f*max_rate*time_to_velocity;
        jerk_time = time_to_velocity;
        if(distance >= distance_to_velocity){
            cruise_time = (distance-distance_to_velocity)/max_rate;
        }else{
            jerk_time = cbrtf(distance/(2.0f*max_jerk));
        }
    }

    scurve_segment_duration[0] = jerk_time;
    scurve_segment_duration[1] = acceleration_time;
    scurve_segment_duration[2] = jerk_time;
    scurve_segment_duration[3] = cruise_time;
    scurve_segment_duration[4] = jerk_time;
    scurve_segment_duration[5] = acceleration_time;
    scurve_segment_duration[6] = jerk_time;
    scurve_jerk = direction*max_jerk;

    float time = 0.0f;
    float position = val;
    float velocity = 0.0f;
    float acceleration = 0.0f;
    for(uint8_t segment = 0U; segment < 7U; ++segment){
        const float duration = scurve_segment_duration[segment];
        const float jerk = scurve_jerk*jerk_sign[segment];
        scurve_segment_time[segment] = time;
        scurve_segment_position[segment] = position;
        scurve_segment_rate[segment] = velocity;
        scurve_segment_acceleration[segment] = acceleration;
        position += velocity*duration
                  + 0.5f*acceleration*duration*duration
                  + jerk*duration*duration*duration/6.0f;
        velocity += acceleration*duration
                  + 0.5f*jerk*duration*duration;
        acceleration += jerk*duration;
        time += duration;
    }
    scurve_total_time = time;
    scurve_active = 1U;
}

void foc_track::scurve_evaluate(float time)
{
    static const float jerk_sign[7] = {
        1.0f, 0.0f, -1.0f, 0.0f, -1.0f, 0.0f, 1.0f
    };

    if(time >= scurve_total_time){
        phase_val = command_val;
        rate_val = 0.0f;
        acceleration_val = 0.0f;
        return;
    }

    for(uint8_t segment = 0U; segment < 7U; ++segment){
        const float end = scurve_segment_time[segment]
                        + scurve_segment_duration[segment];
        if((time <= end) || (segment == 6U)){
            const float local_time = time-scurve_segment_time[segment];
            const float jerk = scurve_jerk*jerk_sign[segment];
            phase_val = scurve_segment_position[segment]
                      + scurve_segment_rate[segment]*local_time
                      + 0.5f*scurve_segment_acceleration[segment]
                        *local_time*local_time
                      + jerk*local_time*local_time*local_time/6.0f;
            rate_val = scurve_segment_rate[segment]
                     + scurve_segment_acceleration[segment]*local_time
                     + 0.5f*jerk*local_time*local_time;
            acceleration_val = scurve_segment_acceleration[segment]
                             + jerk*local_time;
            return;
        }
    }
}

inline float foc_track::scurve(float new_obj, float val, float max_rate,
                               float max_acceleration, float max_jerk,
                               float Ts)
{
    if(initialized == 0U){
        phase_val = val;
        rate_val = 0.0f;
        acceleration_val = 0.0f;
        command_val = val;
        initialized = 1U;
    }

    if((Ts <= 0.0f) || (max_rate <= 0.0f)
    || (max_acceleration <= 0.0f) || (max_jerk <= 0.0f)){
        phase_val = new_obj;
        rate_val = 0.0f;
        acceleration_val = 0.0f;
        command_val = new_obj;
        scurve_active = 0U;
        scurve_pending = 0U;
        return phase_val;
    }

    if(scurve_active != 0U){
        if(new_obj != command_val){
            scurve_pending_command = new_obj;
            scurve_pending = 1U;
        }else{
            scurve_pending = 0U;
        }

        scurve_elapsed += Ts;
        scurve_evaluate(scurve_elapsed);
        if(scurve_elapsed >= scurve_total_time){
            scurve_active = 0U;
            if(scurve_pending != 0U){
                const float pending = scurve_pending_command;
                scurve_pending = 0U;
                scurve_plan(pending, phase_val, max_rate,
                            max_acceleration, max_jerk);
            }
        }
        return phase_val;
    }

    if(new_obj != phase_val){
        scurve_plan(new_obj, phase_val, max_rate,
                    max_acceleration, max_jerk);
        scurve_elapsed = (Ts < scurve_total_time) ? Ts : scurve_total_time;
        scurve_evaluate(scurve_elapsed);
    }else{
        rate_val = 0.0f;
        acceleration_val = 0.0f;
    }
    return phase_val;
}


float foc_track::run(float new_obj, float val, float max_rate, float Ts,
                     TRACK_MODE_t mode, float max_acceleration,
                     float max_jerk)
{
    const float previous = (initialized != 0U) ? phase_val : val;
    const float previous_rate = rate_val;

    switch (mode){
        case TRACK_MODE_NONE:
            this->phase_val = new_obj;
            initialized = 1U;
            rate_val = 0.0f;
            acceleration_val = 0.0f;
            break;
        case TRACK_MODE_RAMP:
            (void)ramp(new_obj, val, max_rate, Ts);
            rate_val = (Ts > 0.0f) ? (phase_val-previous)/Ts : 0.0f;
            acceleration_val = (Ts > 0.0f)
                             ? (rate_val-previous_rate)/Ts : 0.0f;
            break;
        case TRACK_MODE_TRAPEZOID:
            (void)trapezoid(new_obj, val, max_rate, max_acceleration, Ts);
            acceleration_val = (Ts > 0.0f)
                             ? (rate_val-previous_rate)/Ts : 0.0f;
            break;
        case TRACK_MODE_SCURVE:
            (void)scurve(new_obj, val, max_rate, max_acceleration,
                         max_jerk, Ts);
            break;
        default:
            rate_val = 0.0f;
            acceleration_val = 0.0f;
            break;
    }

    return this->phase_val;
}

float foc_track::rate(void) const
{
    return rate_val;
}

float foc_track::acceleration(void) const
{
    return acceleration_val;
}

void foc_track::clear(void)
{
    initialized = 0U;
    braking = 0U;
    phase_val = 0.0f;
    rate_val = 0.0f;
    acceleration_val = 0.0f;
    command_val = 0.0f;
    brake_acceleration = 0.0f;
    scurve_active = 0U;
    scurve_pending = 0U;
    scurve_pending_command = 0.0f;
    scurve_elapsed = 0.0f;
    scurve_total_time = 0.0f;
    scurve_jerk = 0.0f;
    for(uint8_t segment = 0U; segment < 7U; ++segment){
        scurve_segment_time[segment] = 0.0f;
        scurve_segment_duration[segment] = 0.0f;
        scurve_segment_position[segment] = 0.0f;
        scurve_segment_rate[segment] = 0.0f;
        scurve_segment_acceleration[segment] = 0.0f;
    }
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

void foc_spd_ctrl::set_output_limit(float out_limit)
{
    spd_pid->set_output_limit(out_limit);
    out.q = LIMIT_RANGE(out.q, -out_limit, out_limit);
}

void foc_spd_ctrl::set_track(TRACK_MODE_t mode, float accel_limit)
{
    this->track_mode = mode;
    this->accel_limit = accel_limit;
    track->clear();
}

void foc_spd_ctrl::track_output(float applied_iq)
{
    spd_pid->track_output(applied_iq);
    out.q = applied_iq;
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
    this->acceleration_limit = 0;
    this->jerk_limit = 0;
    this->velocity_feedforward_gain = 0.0f;
    this->output_limit = out_limit;

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
    target_ref = track->run(ref_pos, fb_multi_pos, velocity_limit, Ts,
                            track_mode, acceleration_limit, jerk_limit);
    const float dif = dif_calc(target_ref, fb_multi_pos, fb_single_pos, rotate_mode);
    out = pos_pid->pid(dif, Ts)
        + velocity_feedforward_gain*track->rate();
    out = LIMIT_RANGE(out, -output_limit, output_limit);
    return out;
}

void foc_pos_ctrl::set_gains(float kp, float ki, float kd)
{
    pos_pid->set_gains(kp, ki, kd);
}

void foc_pos_ctrl::set_output_limit(float out_limit)
{
    output_limit = out_limit;
    pos_pid->set_output_limit(out_limit);
    out = LIMIT_RANGE(out, -out_limit, out_limit);
}

void foc_pos_ctrl::set_velocity_feedforward(float gain)
{
    velocity_feedforward_gain = gain;
}

void foc_pos_ctrl::set_track(TRACK_MODE_t mode, float velocity_limit,
                             float acceleration_limit, float jerk_limit)
{
    this->track_mode = mode;
    this->velocity_limit = velocity_limit;
    this->acceleration_limit = acceleration_limit;
    this->jerk_limit = jerk_limit;
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
