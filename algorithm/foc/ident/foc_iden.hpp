/**
 ****************************************************************************************************
 * @file        foc_iden.hpp
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


#ifndef _FOC_IDEN_HPP
#define _FOC_IDEN_HPP




#include "foc_type.h"
#include "foc_filter.hpp"
#include "foc_iden_math.hpp"
#include <stdio.h>

#define RS_IDEN_SWEEP_POINT_COUNT   6U
#define RS_IDEN_FIT_POINT_COUNT     12U
#define RS_IDEN_HOLDOUT_POINT_COUNT 4U
#define RS_IDEN_POINT_COUNT         16U
#define FLUX_FIT_POINT_COUNT        4U
#define FLUX_IDEN_POINT_COUNT       6U

typedef struct
{
    float rs;
    float inverter_voltage_offset;
    float rs_rmse;
    float rs_sweep_forward;
    float rs_sweep_reverse;
    float rs_sweep_delta;
    float voltage_offset_forward;
    float voltage_offset_reverse;
    float voltage_offset_delta;
    float rs_holdout_rmse;
    uint8_t rs_point_count;
    float rs_current[RS_IDEN_POINT_COUNT];
    float rs_voltage[RS_IDEN_POINT_COUNT];
    float ld;
    float lq;
    float ls;
    float ld_eq_resistance;
    float lq_eq_resistance;
    float ld_current_rms;
    float lq_current_rms;
    float ld_high;
    float lq_high;
    float ld_amplitude_delta;
    float lq_amplitude_delta;
    float ld_high_current_rms;
    float lq_high_current_rms;
    float flux;
    float flux_voltage_intercept;
    float flux_rmse;
    float flux_holdout_rmse;
    float flux_holdout_max_error;
} foc_iden_result_t;

class foc_offline_iden
{
private:
    /* data */
    uint8_t state;                                  /*状态机标志*/
    uint32_t cnt;                                   /*计数器*/
    

    /*电阻辨识变量*/
    uint8_t step;                                   /*阶段*/
    float Vs_step;                                  /*电压步态*/
    float Is_steps_ref[RS_IDEN_POINT_COUNT];        /*电流阶梯参考值*/
    float Vs_steps_ref[RS_IDEN_POINT_COUNT];        /*电压阶梯参考值*/
    float Vs_steps[RS_IDEN_POINT_COUNT];            /*电压阶梯步态*/
    float Is_steps[RS_IDEN_POINT_COUNT];            /*电流阶梯值*/
   
    float Vs_avg;                                   /*定子电压平均值*/
    float Is_avg;                                   /*定子电流平均值*/
    float Is_err_thres;                             /*电流误差阈值*/
    float Is;
    float Is_err;
    
    /*电感辨识*/
    float V_pulse;                                  /*脉冲电压*/
    uint32_t injection_half_cnt;
    uint32_t injection_sample_cnt;
    uint32_t injection_measure_samples;
    uint32_t injection_settle_samples;
    float injection_sign;
    float injection_cos;
    float injection_sin;
    float injection_rotate_cos;
    float injection_rotate_sin;
    float voltage_dft_cos;
    float voltage_dft_sin;
    float current_dft_cos;
    float current_dft_sin;
    

    /*磁链辨识*/
    /*机械速度，目标值，设定值，加速度值*/
    
    float elec_speed_ref;
    float elec_speed_set;
    uint8_t flux_point;
    float flux_speed[FLUX_IDEN_POINT_COUNT];
    float flux_uq[FLUX_IDEN_POINT_COUNT];
    float flux_iq[FLUX_IDEN_POINT_COUNT];
    float flux_id[FLUX_IDEN_POINT_COUNT];
    qd_t flux_voltage_sum;
    qd_t flux_current_sum;
    float flux_speed_sum;

    void LsAxisInit(qd_t &Uqd, bool q_axis, float injection_voltage,
                    uint8_t half_period_samples, uint32_t measure_cycles, float Ts);
    bool LsAxisRun(qd_t &Uqd, qd_t Iqd, bool q_axis,
                   uint8_t half_period_samples, float Ts,
                   foc_impedance_fit_t &fit);
public:
    float Is_max;
    float Vs_max;  
    float_t Rs;
    float Lq;
    float Ld;
    float Ls;
    float Lqd_dif;
    float inverter_voltage_offset;
    float Rs_rmse;
    float Rs_sweep_forward;
    float Rs_sweep_reverse;
    float Rs_sweep_delta;
    float voltage_offset_forward;
    float voltage_offset_reverse;
    float voltage_offset_delta;
    float Rs_holdout_rmse;
    float Ld_eq_resistance;
    float Lq_eq_resistance;
    float Ld_current_rms;
    float Lq_current_rms;
    float Ld_high;
    float Lq_high;
    float Ld_amplitude_delta;
    float Lq_amplitude_delta;
    float Ld_high_current_rms;
    float Lq_high_current_rms;

    float elec_speed_h;
    float elec_speed_l;
    float flux;
    float flux_voltage_intercept;
    float flux_rmse;
    float flux_holdout_rmse;
    float flux_holdout_max_error;
public:
    foc_offline_iden(float Is_max, float elec_speed_l, float elec_speed_h,
                    float Rs, float Lq, float Ld);
    ~foc_offline_iden();
    void ResultGet(foc_iden_result_t &result) const;

    /*电阻辨识 用额定电流来辨识*/
    bool RsIden(qd_t &Uqd, qd_t Iqd, uint32_t samps, float Ts);

    /* prescale is the half-period sample count; samps is the measured cycle count. */
    bool LsIden(qd_t &Uqd, qd_t Iqd, float voltage_low, float voltage_high,
                uint8_t prescale, uint32_t samps, float Ts);
    
    bool FsIden(qd_t &Uqd, qd_t Iqd, float measured_elec_speed,
                float &target_elec_speed, float elec_speed_acc, float Ts);

    speed_t speed;
};




































#endif
