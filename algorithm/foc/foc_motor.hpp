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
#include "foc_field_weakening.hpp"
#include "foc_pwm.hpp"
#include "foc_current.hpp"
#include "foc_pid.hpp"
#include "foc_vf.hpp"
#include "foc_absenc.hpp"
#include "foc_abzenc.hpp"
#include "foc_smo.hpp"
#include "foc_flux.hpp"
#include "foc_hfi.hpp"
#include "foc_hfi_speed_startup.hpp"
#include "foc_hfi_flux_handoff.hpp"
#include "foc_sensorless_startup.hpp"
#include "foc_ctrl.hpp"
#include "foc_time.hpp"
#include "foc_fault.hpp"
#include "foc_iden.hpp"


/* Legacy mode values are kept as the serial/debug compatibility view. Runtime
 * ownership is split into lifecycle, operation and diagnostic dimensions. */
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

typedef enum
{
    FOC_LIFECYCLE_IDLE = 0,
    FOC_LIFECYCLE_CURRENT_CALIBRATION,
    FOC_LIFECYCLE_READY,
    FOC_LIFECYCLE_STARTING,
    FOC_LIFECYCLE_RUNNING,
    FOC_LIFECYCLE_FAULT_ACTIVE,
    FOC_LIFECYCLE_FAULT_LATCHED,
} FOC_LIFECYCLE_t;

typedef enum
{
    FOC_OPERATION_NONE = 0,
    FOC_OPERATION_IDENTIFY_RL,
    FOC_OPERATION_IDENTIFY_FLUX,
    FOC_OPERATION_OPEN_VOLTAGE,
    FOC_OPERATION_CURRENT,
    FOC_OPERATION_SPEED,
    FOC_OPERATION_POSITION_VOLTAGE,
    FOC_OPERATION_POSITION_CURRENT,
    FOC_OPERATION_POSITION_SPEED,
} FOC_OPERATION_t;

typedef enum
{
    FOC_STARTUP_POWER_STAGE_CHARGE = 0,
    FOC_STARTUP_CLEAR_CONTROL,
    FOC_STARTUP_SELECT_SOURCE,
    FOC_STARTUP_CALIBRATE_ABZ,
    FOC_STARTUP_CALIBRATE_ABS,
} FOC_STARTUP_PHASE_t;

typedef enum
{
    FOC_DIAGNOSTIC_NONE = 0,
    FOC_DIAGNOSTIC_HFI_ENCODER_SHADOW,
    FOC_DIAGNOSTIC_HFI_D_AXIS_SCAN,
    FOC_DIAGNOSTIC_HFI_TENSOR_SCAN,
} FOC_DIAGNOSTIC_t;

typedef enum
{
    FOC_POWER_STAGE_DISABLED = 0,
    FOC_POWER_STAGE_PRECHARGE,
    FOC_POWER_STAGE_ENABLED,
} FOC_POWER_STAGE_t;

/* Rotor position/speed source used by the control law. */
typedef enum{
    ANGLE_GEN_VF = 0,                   /*VF计算角度*/
    ANGLE_GEN_ABZENC,                   /*ABZ编码器*/
    ANGLE_GEN_ABSENC,                   /*ABS编码器*/
    ANGLE_GEN_FLUX,                     /*磁链观测器*/
    ANGLE_GEN_SMO,                      /*滑模观测器*/
    ANGLE_GEN_HFI,                      /*高频注入*/
    ANGLE_GEN_SENSORLESS,               /*I/f启动并切换到有源磁链*/
}ROTOR_SOURCE_t;

/* Temporary source compatibility for existing application construction. */
typedef ROTOR_SOURCE_t ANGLE_GEN_t;

class foc_motor
{
private:
    typedef enum
    {
        FOC_REQUEST_NONE = 0,
        FOC_REQUEST_PREPARE,
        FOC_REQUEST_START,
        FOC_REQUEST_START_SENSORLESS,
        FOC_REQUEST_START_HFI,
        FOC_REQUEST_STOP,
    } FOC_REQUEST_t;

    typedef enum
    {
        HFI_REQUEST_NONE = 0,
        HFI_REQUEST_START_ENCODER_SHADOW,
        HFI_REQUEST_START_D_AXIS_SCAN,
        HFI_REQUEST_START_TENSOR_SCAN,
        HFI_REQUEST_ACTIVATE_D_AXIS_TRACK,
        HFI_REQUEST_ACTIVATE_TENSOR_CONTROL,
        HFI_REQUEST_ENABLE_SPEED_CONTROL,
        HFI_REQUEST_STOP_SHADOW,
        HFI_REQUEST_FORCE_STOP,
    } HFI_REQUEST_t;

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
    volatile float  vbus;                   /*ISR更新，前台启动门槛读取*/
    float           Ts;                     /*周期*/
    float           Ts_test;
    float           Vd_bias;                /*D轴偏置电压*/
    qd_t            current_feedforward;
    float           motor_ld;
    float           motor_lq;
    float           motor_flux;
    float           speed_id_reference;
    float           speed_current_limit;
    float           voltage_request;
    foc_field_weakening field_weakening;
    foc_field_weakening_output_t field_weakening_output;
    foc_sensorless_startup sensorless_startup;
    foc_sensorless_startup_output_t sensorless_output;
    foc_hfi_speed_startup hfi_speed_startup;
    foc_hfi_speed_startup_output_t hfi_speed_startup_output;
    foc_hfi_flux_handoff hfi_flux_handoff;
    foc_hfi_flux_output_t hfi_flux_output;
    volatile float  sensorless_start_direction;
    qd_t            hfi_injection;
    alphabeta_t     hfi_applied_injection;
    float           hfi_speed_kp;
    float           hfi_speed_ki;
    float           flux_speed_kp;
    float           flux_speed_ki;
    float           hfi_current_limit;
    float           flux_current_limit;
    volatile FOC_DIAGNOSTIC_t diagnostic_mode;
    volatile HFI_REQUEST_t hfi_request;
    volatile float  hfi_request_voltage;
    volatile float  hfi_request_initial_offset;
    bool            hfi_speed_control_enabled;
    volatile uint32_t current_feedforward_mode;
    bool            encoder_calibrated;

    volatile FOC_LIFECYCLE_t lifecycle;
    FOC_STARTUP_PHASE_t startup_phase;
    volatile FOC_OPERATION_t operation;
    volatile FOC_OPERATION_t requested_operation;
    volatile FOC_REQUEST_t control_request;
    foc_fault_manager fault_manager;
    foc_fault_monitor fault_monitor;
    foc_fault_monitor_config_t fault_monitor_config;
    uint32_t transient_fault_bits;
    uint32_t control_ticks;
    bool fault_monitor_configured;
    PWM_MODE_t      pwm_mode;               /*PWM模式*/
    volatile ROTOR_SOURCE_t angle_gen;

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
    bool queue_hfi_request(HFI_REQUEST_t request, float injection_voltage,
                           float initial_offset);
    void apply_hfi_request(void);
    static FOC_OPERATION_t operation_from_mode(MOTOR_MODE_t mode);
    static MOTOR_MODE_t mode_from_operation(FOC_OPERATION_t operation);
    bool speed_operation_running(void) const;
    bool queue_start_request(FOC_OPERATION_t next_operation,
                             FOC_REQUEST_t request,
                             float request_parameter);
    void apply_control_request(void);
    void begin_start(void);
    void stop_normal(void);
    foc_fault_snapshot_t fault_snapshot(void) const;
    void trip_fault(uint32_t fault, bool transient);
    void stop_for_fault(void);
    void update_fault_monitor(bool power_control_active);

    inline float angle_get(qd_t &Uqd, alphabeta_t &Ualphabeta,
                           alphabeta_t &I_alphabeta,
                           ROTOR_SOURCE_t source, float Ts);
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
    bool request_sensorless_start(float direction);
    bool request_hfi_start(float injection_voltage);
    bool request_stop(void);
    bool request_fault_clear(void);
    bool start_hfi_shadow(float injection_voltage, float initial_offset);
    bool start_hfi_characterization(float injection_voltage,
                                    float initial_offset);
    bool start_hfi_tensor_characterization(float injection_voltage,
                                           float initial_offset);
    bool activate_hfi_d_axis_tracking(void);
    bool activate_hfi_control(void);
    bool stop_hfi_shadow(void);
    MOTOR_MODE_t mode_get(void) const;
    FOC_POWER_STAGE_t power_stage_get(void) const;
    FOC_LIFECYCLE_t lifecycle_get(void) const;
    FOC_OPERATION_t operation_get(void) const;
    ROTOR_SOURCE_t rotor_source_get(void) const;
    FOC_DIAGNOSTIC_t diagnostic_get(void) const;
    foc_fault_status_t fault_status_get(void) const;

    void set_bus_voltage(float vbus);
    void configure_fault_monitor(const foc_fault_monitor_config_t &config);
    void set_voltage_limit_ratio(float ratio);
    void configure_sensorless_startup(
        float align_current_a, float open_loop_current_a,
        float handoff_current_a, float current_taper_rate_a_s,
        float align_time_s, float acceleration_rad_s2,
        float handoff_speed_rad_s, float phase_gate_rad,
        float speed_gate_rad_s, float qualification_time_s,
        float blend_time_s, float qualification_timeout_s);
    void configure_hfi_flux_handoff(
        float hfi_to_flux_speed, float hfi_prepare_speed,
        float flux_to_hfi_speed, float hfi_speed_ceiling,
        float flux_speed_floor, float angle_gate_rad,
        float speed_gate_rad_s, float qualification_time_s,
        float blend_time_s, float hfi_kp, float hfi_ki,
        float flux_kp, float flux_ki,
        float hfi_current_limit, float flux_current_limit);
    void configure_hfi_speed_startup(
        float current_limit, float acceleration,
        float proportional_gain, float acceleration_target_speed,
        float closed_loop_entry_speed, float qualification_time);
    void configure_field_weakening(float rs, float ld, float lq, float flux,
                                   float kp, float ki, float current_limit,
                                   float target_utilization, float filter_alpha,
                                   float id_rate_limit);
    void set_field_weakening_mode(FIELD_WEAKENING_MODE_t mode);
    FIELD_WEAKENING_MODE_t field_weakening_mode_get(void) const;
    void configure_current_feedforward(float ld, float lq, float flux);
    void set_current_feedforward_mode(CURRENT_FEEDFORWARD_MODE_t mode);
    void enable_current_feedforward(bool enabled);
    void set_speed_id_reference(float id);
    void set_speed_current_limit(float limit);
    void set_current_reference(float iq, float id);
    void set_position_track(TRACK_MODE_t mode, float velocity_limit,
                            float acceleration_limit = 0.0f,
                            float jerk_limit = 0.0f);
    void set_position_velocity_feedforward(float gain);
    void set_speed_track(TRACK_MODE_t mode, float acceleration_limit);
    void set_speed_estimator_mode(foc_speed_estimator_mode_t mode);
    void set_speed_gains(float kp, float ki);
    void set_position_gains(float kp, float ki, float kd);
    void set_position_output_limit(float limit);
    void set_current_gains(bool q_axis, float kp, float ki);
    void clear_control_loops(void);
    void identification_result_get(foc_iden_result_t &result) const;
    void timing_clear(void);
    void timing_capture(void);
    float output_position(void) const;

    void run(void);
    void protect(void);
    void clear(void);
    void SpeedCalc(void);
    void SpeedCtrl(float rate);
    void PositionCtrl(float target_position);
};





















































#endif
