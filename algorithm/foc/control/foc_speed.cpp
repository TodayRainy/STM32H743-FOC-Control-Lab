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

#include <cstring>


#pragma arm section code  = "RAMCODE"


foc_speed::foc_speed(uint32_t pole_pairs, uint32_t gear_ratio, float offset)
{
    movefilter = new foc_movefilter<float>(500);
    win_movefilter = new foc_movefilter<float>(40);       /* 1 ms at 40 kHz. */
    movefilter->clear();
    win_movefilter->clear();

    memset(&speed_data, 0x00, sizeof(speed_data));

    this->speed_data.pole_pairs  = pole_pairs;
    this->speed_data.gear_ratio  = gear_ratio;
    this->speed_data.offset      = offset;

    this->elec_angle_pre    = 0;
    this->mech_angle_pre    = 0;
    this->out_angle_pre     = 0;

    this->estimator_mode          = FOC_SPEED_ESTIMATOR_BASE;
    this->estimator_sample_period = 0.0f;
    this->estimator_position      = 0.0f;
    this->base_speed              = 0.0f;
    this->win_iir_speed           = 0.0f;
    this->win_iir_alpha           = 0.0f;
    this->pll_position            = 0.0f;
    this->pll_speed               = 0.0f;
    this->pll_kp                  = 0.0f;
    this->pll_ki                  = 0.0f;
    this->period_started          = 0U;
    this->period_direction        = 0;
    this->period_counts           = 0;
    this->period_elapsed          = 0.0f;
    this->period_no_edge_time     = 0.0f;
    this->period_speed            = 0.0f;
    this->period_iir_speed        = 0.0f;
    this->period_iir_alpha        = 0.0f;
    this->fused_speed             = 0.0f;

   
}

foc_speed::~foc_speed()
{
    delete(movefilter);
    delete(win_movefilter);
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
    win_movefilter->clear();
    estimator_position = 0.0f;
    base_speed = 0.0f;
    win_iir_speed = 0.0f;
    pll_position = 0.0f;
    pll_speed = 0.0f;
    period_started = 0U;
    period_direction = 0;
    period_counts = 0;
    period_elapsed = 0.0f;
    period_no_edge_time = 0.0f;
    period_speed = 0.0f;
    period_iir_speed = 0.0f;
    fused_speed = 0.0f;
    SpeedRefresh(0.0f);
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
    estimator_position += delta[0];

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
void foc_speed::SpeedCalc(float Ts, int32_t encoder_count_delta,
                          uint32_t encoder_counts_per_revolution)
{
    const float instant_speed = delta[0]/Ts;

    if(estimator_sample_period != Ts){
        const float win_cutoff_rad_s = _2PI*150.0f;
        const float pll_natural_rad_s = _2PI*200.0f;
        estimator_sample_period = Ts;
        win_iir_alpha = expf(-win_cutoff_rad_s*Ts);
        period_iir_alpha = win_iir_alpha;
        pll_kp = 2.0f*pll_natural_rad_s;
        pll_ki = pll_natural_rad_s*pll_natural_rad_s;
    }

    movefilter->fill(instant_speed);
    base_speed = movefilter->calc();

    win_movefilter->fill(instant_speed);
    win_iir_speed = win_iir_alpha*win_iir_speed
                    + (1.0f-win_iir_alpha)*win_movefilter->calc();

    pll_position += Ts*pll_speed;
    const float pll_error = estimator_position-pll_position;
    pll_position += Ts*pll_kp*pll_error;
    pll_speed += Ts*pll_ki*pll_error;

    if(encoder_counts_per_revolution > 0U){
        const int8_t direction = (encoder_count_delta > 0) ? 1
                               : ((encoder_count_delta < 0) ? -1 : 0);

        period_no_edge_time += Ts;
        if(period_started != 0U){
            period_elapsed += Ts;
        }

        if(direction != 0){
            period_no_edge_time = 0.0f;
            if((period_started == 0U) || (direction != period_direction)){
                /* The first edge establishes a time origin. A reversal must not
                 * reuse the previous direction's held period estimate. */
                period_started = 1U;
                period_direction = direction;
                period_counts = 0;
                period_elapsed = 0.0f;
                period_speed = 0.0f;
                period_iir_speed = 0.0f;
            }else{
                period_counts += encoder_count_delta;
                if((period_counts >= 4) || (period_counts <= -4)){
                    const float count_angle = _2PI
                        *(float)speed_data.pole_pairs
                        /(float)encoder_counts_per_revolution;
                    if(period_elapsed > 0.0f){
                        period_speed = (float)period_counts*count_angle
                                     / period_elapsed;
                    }
                    period_counts = 0;
                    period_elapsed = 0.0f;
                }
            }
        }

        /* Ten milliseconds corresponds to 0.68 count at 1 rpm and
         * decisively removes a stale period after a stop. */
        if(period_no_edge_time >= 0.010f){
            period_started = 0U;
            period_direction = 0;
            period_counts = 0;
            period_elapsed = 0.0f;
            period_speed = 0.0f;
        }

        period_iir_speed = period_iir_alpha*period_iir_speed
                         + (1.0f-period_iir_alpha)*period_speed;

        const float win_mech_rpm = fabsf(win_iir_speed)
            /(float)speed_data.pole_pairs*60.0f/_2PI;
        const float win_weight = LIMIT_RANGE(
            (win_mech_rpm-100.0f)/(250.0f-100.0f), 0.0f, 1.0f
        );
        fused_speed = period_iir_speed
                    + win_weight*(win_iir_speed-period_iir_speed);
    }

    SpeedRefresh(EstimatorSpeedGet(estimator_mode));
}

void foc_speed::EstimatorModeSet(foc_speed_estimator_mode_t mode)
{
    estimator_mode = mode;
}

foc_speed_estimator_mode_t foc_speed::EstimatorModeGet(void) const
{
    return estimator_mode;
}

float foc_speed::EstimatorSpeedGet(foc_speed_estimator_mode_t mode) const
{
    switch(mode){
        case FOC_SPEED_ESTIMATOR_WIN_IIR:
            return win_iir_speed;
        case FOC_SPEED_ESTIMATOR_PLL:
            return pll_speed;
        case FOC_SPEED_ESTIMATOR_FUSED:
            return fused_speed;
        case FOC_SPEED_ESTIMATOR_BASE:
        default:
            return base_speed;
    }
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
