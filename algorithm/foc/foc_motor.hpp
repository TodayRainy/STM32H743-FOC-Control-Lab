/**
 ****************************************************************************************************
 * @file        foc_motor.hpp
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

#ifndef _FOC_MOTOR_HPP
#define _FOC_MOTOR_HPP

#include <stdint.h>


#include "foc_type.h"
#include "foc_current_feedforward.hpp"
#include "foc_pwm.hpp"
#include "foc_current.hpp"
#include "foc_pid.hpp"
#include "foc_vf.hpp"
#include "foc_absenc.hpp"
#include "foc_abzenc.hpp"
#include "foc_smo.hpp"
#include "foc_flux.hpp"
#include "foc_hfi.hpp"
#include "foc_ctrl.hpp"
#include "foc_time.hpp"
#include "foc_iden.hpp"


/*电机运行模式*/
typedef enum
{
    MODE_IDLE = 0,                      /*空闲状态*/
    MODE_POWER_STAGE_CHARGE,            /*功率级启动前预充*/

    MODE_CURRENT_CALIB,                 /*电流校准*/
    MODE_PHASEVOLT_CALIB,               /*相电压校准*/

    MODE_CLEAR,
    MODE_START,

    MODE_ABZENC_CALIB,                  /*ABZ编码器校准*/
    MODE_ABSENC_CALIB,                  /*绝对式编码器校准*/
    

    MODE_IDEN_RL,                       /*辨识电阻电感*/
    MODE_IDEN_F,                        /*辨识磁链*/

    MODE_OPEN_VOLT_RUN,                 /*电压开环运行*/
    MODE_TORQUE_RUN,                    /*力矩环运行*/
    MODE_SPEED_TORQUE_RUN,              /*速度环-力矩环运行*/
    MODE_POSITION_VOLT_RUN,             /*位置环-电压运行*/
    MODE_POSITION_TORQUE_RUN,           /*位置环-力矩环运行*/
    MODE_POSITION_SPEED_TORQUE_RUN,     /*位置环-速度环-力矩环运行*/

    MODE_CURRENT_CALIB_FAILED,          /*电流偏置校准失败，复位前保持关闭*/
    MODE_READY,                         /*电流校准完成，等待启动请求*/
}MOTOR_MODE_t;

/*角度生成器*/
typedef enum{
    ANGLE_GEN_VF = 0,                   /*VF计算角度*/
    ANGLE_GEN_ABZENC,                   /*ABZ编码器*/
    ANGLE_GEN_ABSENC,                   /*ABS编码器*/
    ANGLE_GEN_FLUX,                     /*磁链观测器*/
    ANGLE_GEN_SMO,                      /*滑模观测器*/
    ANGLE_GEN_HFI,                      /*高频注入*/
}ANGLE_GEN_t;

class foc_motor
{
private:
    /* data */
    uint8_t tabc_state;
    qd_t Uqd;                               /*输出Uqd*/
    qd_t Iqd;                               /*采集得到的Iqd*/
    qd_t Iqd_ref;                           /*目标Iqd*/
    float Ibus;
    foc_movefilter<float> *Ibus_filter;
public:
    speed_t speed_data;
    FOC_DEBUG_t debug;

private:
    float           target_out_pos;         /*输出轴角度*/
    float           out_multi_pos;          /*输出轴累积角度*/
    float           target_elec_speed;      /*目标电角速度*/
    float           max_elec_speed;         /*最大电角速度*/

    uint32_t        cnt;                    /*用于计数*/
    uint8_t         iden_state;             /*参数辨识状态*/
    uint32_t        enc_calib_state;        /*编码器校准状态*/
    uint32_t        spd_cnt;                /*速度环分频计数*/
    uint32_t        pos_cnt;                /*位置环分频计数*/
    uint32_t        protect_cnt;            /*保护机制分频计数*/
    float           vbus;                   /*供电电压*/
    float           Ts;                     /*周期*/
    float           Ts_test;
    float           Vd_bias;                /*D轴偏置电压*/
    qd_t            current_feedforward;
    float           motor_ld;
    float           motor_lq;
    float           motor_flux;
    float           speed_id_reference;
    volatile uint32_t current_feedforward_mode;
    bool            encoder_calibrated;

    volatile MOTOR_MODE_t m_mode;           /*电机模式*/
    MOTOR_MODE_t     requested_run_mode;    /*启动校准完成后进入的运行模式*/
    PWM_MODE_t      pwm_mode;               /*PWM模式*/
    ANGLE_GEN_t     angle_gen;              /*角度生成器选择*/

public:
    foc_pid         *pid_iq;
    foc_pid         *pid_id;

    
    foc_time        *time;
    foc_spd_ctrl    *spd_ctrl;
    foc_pos_ctrl    *pos_ctrl;
    foc_offline_iden *offline_iden;

    foc_vf          *vf;
    foc_abzenc      *abzenc;
    foc_absenc      *absenc;
    foc_smo         *smo;
    foc_flux        *flux;
    foc_hfi         *hfi;

    foc_current     *current;
    foc_pwm         *pwm;

    

private:

    bool abz_enc_calib(void);
    bool abs_enc_calib(void);

    bool iden_run_rl(void);
    bool iden_run_f(void);

    inline float angle_get(qd_t &Uqd, alphabeta_t &Ualphabeta, alphabeta_t &I_alphabeta, ANGLE_GEN_t angle_gen, float Ts);
    inline void current_loop_run(float elec_angle);
    inline void open_volt_run(void);
    inline void torque_run(void);
    inline void speed_torque_run(void);
    inline void pos_volt_run(void);
    inline void pos_torque_run(void);
    inline void pos_speed_torque_run(void);
public:
    foc_motor(qd_t Uqd, float max_speed, float vbus, float Ts, 
                MOTOR_MODE_t m_mode, PWM_MODE_t pwm_mode, ANGLE_GEN_t angle_gen);
    ~foc_motor();

    bool request_prepare(void);
    bool request_start(MOTOR_MODE_t run_mode = MODE_POSITION_SPEED_TORQUE_RUN);
    bool request_stop(void);
    MOTOR_MODE_t mode_get(void) const;

    void set_bus_voltage(float vbus);
    void set_voltage_limit_ratio(float ratio);
    void configure_current_feedforward(float ld, float lq, float flux);
    void set_current_feedforward_mode(CURRENT_FEEDFORWARD_MODE_t mode);
    void enable_current_feedforward(bool enabled);
    void set_speed_id_reference(float id);
    void set_current_reference(float iq, float id);
    void set_position_track(TRACK_MODE_t mode, float velocity_limit,
                            float acceleration_limit = 0.0f);
    void set_position_velocity_feedforward(float gain);
    float output_position(void) const;

    void run(void);
    void protect(void);
    void clear(void);
    void SpeedCalc(void);
    void SpeedCtrl(float rate);
    void PositionCtrl(float target_position);
    
    

};





















































#endif
