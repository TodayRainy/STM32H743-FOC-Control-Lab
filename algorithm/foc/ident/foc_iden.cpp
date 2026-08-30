/**
 ****************************************************************************************************
 * @file        foc_iden.cpp
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

#include "foc_iden.hpp"
#include "foc_math.hpp"

#pragma arm section code  = "RAMCODE"

#define debug   0

foc_offline_iden::foc_offline_iden(float Is_max, float elec_speed_l, float elec_speed_h,
                            float Rs, float Lq, float Ld)
{
    /*补齐漏初始化的辨识中间量: 对象由 new 分配, C++ 不会自动清零*/
    this->step           = 0;
    this->Is             = 0;
    this->Is_err         = 0;
    this->V_pulse        = 0;
    this->injection_half_cnt = 0;
    this->injection_sample_cnt = 0;
    this->injection_measure_samples = 0;
    this->injection_settle_samples = 0;
    this->injection_sign = 0;
    this->injection_cos = 0;
    this->injection_sin = 0;
    this->injection_rotate_cos = 0;
    this->injection_rotate_sin = 0;
    this->voltage_dft_cos = 0;
    this->voltage_dft_sin = 0;
    this->current_dft_cos = 0;
    this->current_dft_sin = 0;
    this->elec_speed_ref = 0;
    this->elec_speed_set = 0;
    this->flux_point     = 0;
    this->flux_speed_sum = 0;
    this->Vs_max         = 0;
    this->Lqd_dif        = 0;
    this->flux           = 0;
    this->inverter_voltage_offset = 0;
    this->Rs_rmse        = -1.0f;
    this->Rs_sweep_forward = 0;
    this->Rs_sweep_reverse = 0;
    this->Rs_sweep_delta = -1.0f;
    this->voltage_offset_forward = 0;
    this->voltage_offset_reverse = 0;
    this->voltage_offset_delta = -1.0f;
    this->Rs_holdout_rmse = -1.0f;
    this->Ld_eq_resistance = 0;
    this->Lq_eq_resistance = 0;
    this->Ld_current_rms = 0;
    this->Lq_current_rms = 0;
    this->Ld_high = 0;
    this->Lq_high = 0;
    this->Ld_amplitude_delta = -1.0f;
    this->Lq_amplitude_delta = -1.0f;
    this->Ld_high_current_rms = 0;
    this->Lq_high_current_rms = 0;
    this->flux_voltage_intercept = 0;
    this->flux_rmse      = -1.0f;
    this->flux_holdout_rmse = -1.0f;
    this->flux_holdout_max_error = -1.0f;

    this->state             = 0;
    this->cnt               = 0;
    this->Vs_avg            = 0;
    this->Is_avg            = 0;
    this->Is_err_thres      = 0.17f;
    this->Vs_step           = 0.0001f;

    this->Is_max            = Is_max;
    this->elec_speed_l      = elec_speed_l;
    this->elec_speed_h      = elec_speed_h;
    this->Rs                = Rs;
    this->Lq                = Lq;
    this->Ld                = Ld;
    this->Ls                = (Lq+Ld)*0.5f;

    for(uint32_t i=0U;i<RS_IDEN_POINT_COUNT;i++){
        Is_steps_ref[i] = 0;
        Vs_steps_ref[i] = 0;
        Is_steps[i] = 0;
        Vs_steps[i] = 0;
    }
    memset(flux_speed, 0x00, sizeof(flux_speed));
    memset(flux_uq, 0x00, sizeof(flux_uq));
    memset(flux_iq, 0x00, sizeof(flux_iq));
    memset(flux_id, 0x00, sizeof(flux_id));
    memset(&flux_voltage_sum, 0x00, sizeof(flux_voltage_sum));
    memset(&flux_current_sum, 0x00, sizeof(flux_current_sum));
}

foc_offline_iden::~foc_offline_iden()
{
}

void foc_offline_iden::ResultGet(foc_iden_result_t &result) const
{
    result.rs = Rs;
    result.inverter_voltage_offset = inverter_voltage_offset;
    result.rs_rmse = Rs_rmse;
    result.rs_sweep_forward = Rs_sweep_forward;
    result.rs_sweep_reverse = Rs_sweep_reverse;
    result.rs_sweep_delta = Rs_sweep_delta;
    result.voltage_offset_forward = voltage_offset_forward;
    result.voltage_offset_reverse = voltage_offset_reverse;
    result.voltage_offset_delta = voltage_offset_delta;
    result.rs_holdout_rmse = Rs_holdout_rmse;
    result.rs_point_count = RS_IDEN_POINT_COUNT;
    for(uint32_t n = 0U; n < RS_IDEN_POINT_COUNT; n++){
        result.rs_current[n] = Is_steps[n];
        result.rs_voltage[n] = Vs_steps[n];
    }
    result.ld = Ld;
    result.lq = Lq;
    result.ls = Ls;
    result.ld_eq_resistance = Ld_eq_resistance;
    result.lq_eq_resistance = Lq_eq_resistance;
    result.ld_current_rms = Ld_current_rms;
    result.lq_current_rms = Lq_current_rms;
    result.ld_high = Ld_high;
    result.lq_high = Lq_high;
    result.ld_amplitude_delta = Ld_amplitude_delta;
    result.lq_amplitude_delta = Lq_amplitude_delta;
    result.ld_high_current_rms = Ld_high_current_rms;
    result.lq_high_current_rms = Lq_high_current_rms;
    result.flux = flux;
    result.flux_voltage_intercept = flux_voltage_intercept;
    result.flux_rmse = flux_rmse;
    result.flux_holdout_rmse = flux_holdout_rmse;
    result.flux_holdout_max_error = flux_holdout_max_error;
}

/* Two opposite-order sweeps separate time drift from the Rs/Voffset fit.
 * The final four currents are excluded from fitting and validate prediction. */
bool foc_offline_iden::RsIden(qd_t &Uqd, qd_t Iqd, uint32_t samps, float Ts)
{
    bool ret = false;
    const float sampled_voltage = Uqd.d;
    static const float current_fraction[RS_IDEN_POINT_COUNT] = {
        -1.0f/3.0f,  1.0f/3.0f, -2.0f/3.0f,  2.0f/3.0f, -1.0f,  1.0f,
         1.0f,      -1.0f,       2.0f/3.0f, -2.0f/3.0f,  1.0f/3.0f, -1.0f/3.0f,
        -0.5f,       0.5f,      -5.0f/6.0f,  5.0f/6.0f
    };

    /* Keep regulating during settling and averaging. Freezing voltage after
     * the first threshold crossing allowed current to drift between points. */
    if((state >= 1U) && (state <= 3U)){
        Is = Iqd.d;
        Is_err = Is - Is_steps_ref[step];
        if(Is < Is_steps_ref[step]){
            Vs_steps_ref[step] += Vs_step;
        }else{
            Vs_steps_ref[step] -= Vs_step;
        }
        if(Vs_steps_ref[step] > 0.75f)Vs_steps_ref[step] = 0.75f;
        if(Vs_steps_ref[step] < -0.75f)Vs_steps_ref[step] = -0.75f;
        Uqd.d = Vs_steps_ref[step];
        Uqd.q = 0.0f;
    }

    switch(state){
        case 0:
            cnt = 0;
            step = 0;
            Vs_avg = 0;
            Is_avg = 0;
            Vs_max = 0;
            Rs_rmse = -1.0f;
            Rs_sweep_delta = -1.0f;
            voltage_offset_delta = -1.0f;
            Rs_holdout_rmse = -1.0f;
            for(uint32_t i = 0U; i < RS_IDEN_POINT_COUNT; i++){
                Is_steps_ref[i] = Is_max * current_fraction[i];
                Vs_steps_ref[i] = 0;
                Vs_steps[i] = 0;
                Is_steps[i] = 0;
            }
            state = 1;
            break;
        case 1:
            cnt = (fabsf(Is_err) < Is_err_thres) ? (cnt + 1U) : 0U;
            if(cnt > 5U){
                cnt = 0;
                Vs_avg = 0;
                Is_avg = 0;
                state = 2;
            }
            break;
        case 2:
            if(++cnt >= (uint32_t)(0.01f/Ts)){
                cnt = 0;
                state = 3;
            }
            break;
        case 3:
            Vs_avg += sampled_voltage;
            Is_avg += Iqd.d;
            if(++cnt >= samps){
                cnt = 0;
                Vs_steps[step] = Vs_avg/(float)samps;
                Is_steps[step] = Is_avg/(float)samps;
                if(Vs_max < fabsf(Vs_steps[step]))Vs_max = fabsf(Vs_steps[step]);
                step++;
                if(step < RS_IDEN_POINT_COUNT){
                    state = 1;
                }else{
                    state = 4;
                    step = 0;
                }
            }
            break;
        case 4:
        {
            foc_rs_fit_t fit_forward = {};
            foc_rs_fit_t fit_reverse = {};
            foc_rs_fit_t fit_all = {};
            const bool forward_valid = foc_iden_fit_rs(
                Is_steps, Vs_steps, RS_IDEN_SWEEP_POINT_COUNT, &fit_forward);
            const bool reverse_valid = foc_iden_fit_rs(
                &Is_steps[RS_IDEN_SWEEP_POINT_COUNT],
                &Vs_steps[RS_IDEN_SWEEP_POINT_COUNT],
                RS_IDEN_SWEEP_POINT_COUNT, &fit_reverse);
            const bool fit_valid = foc_iden_fit_rs(
                Is_steps, Vs_steps, RS_IDEN_FIT_POINT_COUNT, &fit_all);

            if(forward_valid && reverse_valid){
                Rs_sweep_forward = fit_forward.resistance;
                Rs_sweep_reverse = fit_reverse.resistance;
                const float rs_mean = 0.5f * (fit_forward.resistance
                                            + fit_reverse.resistance);
                Rs_sweep_delta = fabsf(fit_forward.resistance
                                      - fit_reverse.resistance) / rs_mean;
                voltage_offset_forward = fit_forward.voltage_offset;
                voltage_offset_reverse = fit_reverse.voltage_offset;
                voltage_offset_delta = fabsf(fit_forward.voltage_offset
                                            - fit_reverse.voltage_offset);
            }
            if(fit_valid){
                Rs = fit_all.resistance;
                inverter_voltage_offset = fit_all.voltage_offset;
                Rs_rmse = fit_all.rmse;
                Rs_holdout_rmse = foc_iden_rs_prediction_rmse(
                    &Is_steps[RS_IDEN_FIT_POINT_COUNT],
                    &Vs_steps[RS_IDEN_FIT_POINT_COUNT],
                    RS_IDEN_HOLDOUT_POINT_COUNT, Rs, inverter_voltage_offset);
            }
            Uqd.q = 0;
            Uqd.d = 0;
            cnt = 0;
            state = 0;
            ret = true;
            break;
        }
        default:
            break;
    }
    return ret;
}

void foc_offline_iden::LsAxisInit(qd_t &Uqd, bool q_axis, float injection_voltage,
                                  uint8_t half_period_samples,
                                  uint32_t measure_cycles, float Ts)
{
    if(half_period_samples < 2U)half_period_samples = 2U;
    if(measure_cycles < 8U)measure_cycles = 8U;

    V_pulse = fabsf(injection_voltage);

    injection_half_cnt = 0U;
    injection_sample_cnt = 0U;
    injection_settle_samples = 8U * 2U * half_period_samples;
    injection_measure_samples = measure_cycles * 2U * half_period_samples;
    injection_sign = 1.0f;
    injection_cos = 1.0f;
    injection_sin = 0.0f;

    const float angle_step = PI / (float)half_period_samples;
    injection_rotate_cos = cosf(angle_step);
    injection_rotate_sin = sinf(angle_step);
    voltage_dft_cos = 0.0f;
    voltage_dft_sin = 0.0f;
    current_dft_cos = 0.0f;
    current_dft_sin = 0.0f;

    Uqd.q = q_axis ? V_pulse : 0.0f;
    Uqd.d = q_axis ? 0.0f : V_pulse;
    (void)Ts;
}

bool foc_offline_iden::LsAxisRun(qd_t &Uqd, qd_t Iqd, bool q_axis,
                                 uint8_t half_period_samples, float Ts,
                                 foc_impedance_fit_t &fit)
{
    const float voltage = q_axis ? Uqd.q : Uqd.d;
    const float current = q_axis ? Iqd.q : Iqd.d;

    if(injection_sample_cnt >= injection_settle_samples){
        voltage_dft_cos += voltage * injection_cos;
        voltage_dft_sin += voltage * injection_sin;
        current_dft_cos += current * injection_cos;
        current_dft_sin += current * injection_sin;
    }

    const float next_cos = injection_cos * injection_rotate_cos
                         - injection_sin * injection_rotate_sin;
    injection_sin = injection_sin * injection_rotate_cos
                  + injection_cos * injection_rotate_sin;
    injection_cos = next_cos;
    injection_sample_cnt++;

    if(++injection_half_cnt >= half_period_samples){
        injection_half_cnt = 0U;
        injection_sign = -injection_sign;
    }

    Uqd.q = q_axis ? injection_sign * V_pulse : 0.0f;
    Uqd.d = q_axis ? 0.0f : injection_sign * V_pulse;

    if(injection_sample_cnt < (injection_settle_samples + injection_measure_samples)){
        return false;
    }

    const float omega = PI / ((float)half_period_samples * Ts);
    (void)foc_iden_fit_impedance(voltage_dft_cos, voltage_dft_sin,
                                 current_dft_cos, current_dft_sin,
                                 injection_measure_samples, omega, &fit);
    Uqd.q = 0.0f;
    Uqd.d = 0.0f;
    return true;
}

/* Zero-speed square-wave injection. Fundamental DFT rejects DC offsets and
 * the square wave harmonics; the real and imaginary impedance components
 * provide an independent quality check for each axis. */
bool foc_offline_iden::LsIden(qd_t &Uqd, qd_t Iqd,
                              float voltage_low, float voltage_high,
                              uint8_t prescale, uint32_t samps, float Ts)
{
    bool ret = false;
    foc_impedance_fit_t fit = {};

    switch(state){
        case 0:
            Ld_amplitude_delta = -1.0f;
            Lq_amplitude_delta = -1.0f;
            LsAxisInit(Uqd, false, voltage_low, prescale, samps, Ts);
            state = 1;
            break;
        case 1:
            if(LsAxisRun(Uqd, Iqd, false, prescale, Ts, fit)){
                if(fit.inductance > 0.0f){
                    Ld = fit.inductance;
                    Ld_eq_resistance = fit.resistance;
                    Ld_current_rms = fit.current_rms;
                }else{
                    Ld_current_rms = -1.0f;
                }
                LsAxisInit(Uqd, true, voltage_low, prescale, samps, Ts);
                state = 2;
            }
            break;
        case 2:
            if(LsAxisRun(Uqd, Iqd, true, prescale, Ts, fit)){
                if(fit.inductance > 0.0f){
                    Lq = fit.inductance;
                    Lq_eq_resistance = fit.resistance;
                    Lq_current_rms = fit.current_rms;
                }else{
                    Lq_current_rms = -1.0f;
                }
                LsAxisInit(Uqd, false, voltage_high, prescale, samps, Ts);
                state = 3;
            }
            break;
        case 3:
            if(LsAxisRun(Uqd, Iqd, false, prescale, Ts, fit)){
                if(fit.inductance > 0.0f){
                    Ld_high = fit.inductance;
                    Ld_high_current_rms = fit.current_rms;
                }else{
                    Ld_high = 0.0f;
                    Ld_high_current_rms = -1.0f;
                }
                LsAxisInit(Uqd, true, voltage_high, prescale, samps, Ts);
                state = 4;
            }
            break;
        case 4:
            if(LsAxisRun(Uqd, Iqd, true, prescale, Ts, fit)){
                if(fit.inductance > 0.0f){
                    Lq_high = fit.inductance;
                    Lq_high_current_rms = fit.current_rms;
                }else{
                    Lq_high = 0.0f;
                    Lq_high_current_rms = -1.0f;
                }
                if((Ld > 1.0e-9f) && (Ld_high > 0.0f)){
                    Ld_amplitude_delta = fabsf(Ld_high - Ld) / Ld;
                }
                if((Lq > 1.0e-9f) && (Lq_high > 0.0f)){
                    Lq_amplitude_delta = fabsf(Lq_high - Lq) / Lq;
                }
                state = 5;
            }
            break;
        case 5:
            Lqd_dif = Lq - Ld;
            Ls = (Lq + Ld) * 0.5f;
            state = 0;
            ret = true;
            break;
        default:
            break;
    }

    return ret;
}

/* Multi-speed, bidirectional flux regression. Actual encoder speed is used
 * instead of assuming that the speed reference has been reached. */
bool foc_offline_iden::FsIden(qd_t &Uqd, qd_t Iqd, float measured_elec_speed,
                              float &target_elec_speed,
                              float elec_speed_acc, float Ts)
{
    bool ret = false;
    const float point_speed[FLUX_IDEN_POINT_COUNT] = {
        fabsf(elec_speed_l), 0.5f*(fabsf(elec_speed_l) + fabsf(elec_speed_h)),
        fabsf(elec_speed_h), -fabsf(elec_speed_l),
        -0.5f*(fabsf(elec_speed_l) + fabsf(elec_speed_h)), -fabsf(elec_speed_h)
    };

    switch(state){
        case 0:
            cnt = 0U;
            flux_point = 0U;
            elec_speed_set = measured_elec_speed;
            elec_speed_ref = point_speed[0];
            flux_rmse = -1.0f;
            flux_holdout_rmse = -1.0f;
            flux_holdout_max_error = -1.0f;
            memset(flux_speed, 0x00, sizeof(flux_speed));
            memset(flux_uq, 0x00, sizeof(flux_uq));
            memset(flux_iq, 0x00, sizeof(flux_iq));
            memset(flux_id, 0x00, sizeof(flux_id));
            state = 1;
            break;
        case 1:                         /*有斜率限制地移动到下一个速度点*/
            elec_speed_ref = point_speed[flux_point];
            if(elec_speed_set < elec_speed_ref){
                elec_speed_set += elec_speed_acc * Ts;
                if(elec_speed_set > elec_speed_ref)elec_speed_set = elec_speed_ref;
            }else if(elec_speed_set > elec_speed_ref){
                elec_speed_set -= elec_speed_acc * Ts;
                if(elec_speed_set < elec_speed_ref)elec_speed_set = elec_speed_ref;
            }else{
                cnt = 0U;
                state = 2;
            }
            break;
        case 2:                         /*等待速度环稳定*/
            if(++cnt >= (uint32_t)(1.0f / Ts)){
                cnt = 0U;
                flux_voltage_sum.q = 0.0f;
                flux_voltage_sum.d = 0.0f;
                flux_current_sum.q = 0.0f;
                flux_current_sum.d = 0.0f;
                flux_speed_sum = 0.0f;
                state = 3;
            }
            break;
        case 3:                         /*采集0.5秒稳态数据*/
            flux_voltage_sum.q += Uqd.q;
            flux_voltage_sum.d += Uqd.d;
            flux_current_sum.q += Iqd.q;
            flux_current_sum.d += Iqd.d;
            flux_speed_sum += measured_elec_speed;
            if(++cnt >= (uint32_t)(0.5f / Ts)){
                const float inv_count = 1.0f / (float)cnt;
                flux_uq[flux_point] = flux_voltage_sum.q * inv_count;
                flux_iq[flux_point] = flux_current_sum.q * inv_count;
                flux_id[flux_point] = flux_current_sum.d * inv_count;
                flux_speed[flux_point] = flux_speed_sum * inv_count;
                cnt = 0U;
                flux_point++;
                if(flux_point < FLUX_IDEN_POINT_COUNT){
                    state = 1;
                }else{
                    state = 4;
                }
            }
            break;
        case 4:                         /*所有采样完成后回到零速*/
            if(elec_speed_set > 0.0f){
                elec_speed_set -= elec_speed_acc * Ts;
                if(elec_speed_set < 0.0f)elec_speed_set = 0.0f;
            }else if(elec_speed_set < 0.0f){
                elec_speed_set += elec_speed_acc * Ts;
                if(elec_speed_set > 0.0f)elec_speed_set = 0.0f;
            }else{
                state = 5;
            }
            break;
        case 5:
        {
            float corrected_voltage[FLUX_IDEN_POINT_COUNT] = {};
            for(uint32_t n = 0U; n < FLUX_IDEN_POINT_COUNT; n++){
                float offset = 0.0f;
                if(flux_iq[n] > 0.02f)offset = inverter_voltage_offset;
                if(flux_iq[n] < -0.02f)offset = -inverter_voltage_offset;
                corrected_voltage[n] = flux_uq[n] - Rs * flux_iq[n]
                                      - flux_speed[n] * Ld * flux_id[n]
                                      - offset;
            }

            const float fit_speed[FLUX_FIT_POINT_COUNT] = {
                flux_speed[0], flux_speed[2], flux_speed[3], flux_speed[5]
            };
            const float fit_voltage[FLUX_FIT_POINT_COUNT] = {
                corrected_voltage[0], corrected_voltage[2],
                corrected_voltage[3], corrected_voltage[5]
            };
            const float holdout_speed[2] = {flux_speed[1], flux_speed[4]};
            const float holdout_voltage[2] = {
                corrected_voltage[1], corrected_voltage[4]
            };

            foc_flux_fit_t fit = {};
            if(foc_iden_fit_flux(fit_speed, fit_voltage,
                                 FLUX_FIT_POINT_COUNT, &fit)){
                flux = fit.flux;
                flux_voltage_intercept = fit.voltage_intercept;
                flux_rmse = fit.rmse;
                flux_holdout_rmse = foc_iden_flux_prediction_rmse(
                    holdout_speed, holdout_voltage, 2U, flux,
                    flux_voltage_intercept, &flux_holdout_max_error);
            }
            state = 0;
            ret = true;
            break;
        }
        default:
            break;
    }

    target_elec_speed = elec_speed_set;
    return ret;
}



#pragma arm section


















