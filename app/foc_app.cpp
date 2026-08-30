/**
 ****************************************************************************************************
 * @file        foc_app.cpp
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


#include "foc_app.hpp"
#include "foc_hw.hpp"
#include "foc_type.h"
#include "debug.h"
#include "usbd_cdc_interface.h"

#include <bsp.hpp>


#include <stdio.h>
#include <stdint.h>
#include <math.h>


#pragma arm section code  = "RAMCODE"




FOC_DEBUG_t foc_debug = {};
volatile uint32_t foc_debug_seq = 0;
static volatile bool foc_host_speed_control = false;
static volatile float foc_speed_command_rpm = 0.0f;
static volatile bool foc_host_current_control = false;
static volatile bool foc_host_position_control = false;
static volatile bool foc_position_command_initialized = false;
static volatile float foc_position_origin_rad = 0.0f;
static volatile float foc_position_command_rad = 0.0f;
static volatile bool foc_current_sine_active = false;
static volatile bool foc_current_sine_q_axis = false;
static volatile float foc_current_sine_amplitude = 0.0f;
static volatile float foc_current_sine_value = 0.0f;
static volatile float foc_current_sine_quadrature = 1.0f;
static volatile float foc_current_sine_step = 0.0f;
static volatile float foc_current_cosine_step = 1.0f;
static volatile bool foc_speed_sine_active = false;
static volatile float foc_speed_sine_amplitude_rpm = 0.0f;
static volatile float foc_speed_sine_bias_rpm = 0.0f;
static volatile float foc_speed_sine_value = 0.0f;
static volatile float foc_speed_sine_quadrature = 1.0f;
static volatile float foc_speed_sine_step = 0.0f;
static volatile float foc_speed_cosine_step = 1.0f;
static float foc_control_sample_hz = 40000.0f;
static FOC_POWER_STAGE_t foc_applied_power_stage = FOC_POWER_STAGE_DISABLED;
static const char *foc_command_reason = "NONE";

static void FOC_ApplyPowerStage(FOC_POWER_STAGE_t command)
{
    if(command == foc_applied_power_stage){
        return;
    }

    switch(command){
        case FOC_POWER_STAGE_DISABLED:
            FOC_PWM_HW_ON_OFF(false, false, false, false, false, false);
            break;
        case FOC_POWER_STAGE_PRECHARGE:
            A_H_SET_PWM(0);
            B_H_SET_PWM(0);
            C_H_SET_PWM(0);
            FOC_PWM_HW_ON_OFF(true, true, true, true, true, true);
            break;
        case FOC_POWER_STAGE_ENABLED:
            if(foc_applied_power_stage == FOC_POWER_STAGE_DISABLED){
                FOC_PWM_HW_ON_OFF(true, true, true, true, true, true);
            }
            break;
    }
    foc_applied_power_stage = command;
}

/* ProDrone 3505 KV650. LEGACY is only the pre-stage1 voltage model retained
 * for A/B comparison; it is unrelated to the current/speed/position gains. */
#define M0_LEGACY_LQ                (53.17701f/1000000.0f)
#define M0_LEGACY_LD                (34.98624f/1000000.0f)
#define M0_LEGACY_FLUX              (0.90451f/1000.0f)
#define M0_MOTOR_KV                 (650.0f)

/* Stage-1 identified electrical model. */
#define M0_IDENTIFIED_RS            (144.818365f/1000.0f)
#define M0_IDENTIFIED_LD            (26.043582f/1000000.0f)
#define M0_IDENTIFIED_LQ            (40.343765f/1000000.0f)
#define M0_IDENTIFIED_FLUX          (0.90795034f/1000.0f)
#define M0_MODEL_TEST_ID_LIMIT      1.5f
#define M0_CURRENT_SINE_MAX_A       0.5f
#define M0_CURRENT_SINE_MIN_HZ      50.0f
#define M0_CURRENT_SINE_MAX_HZ      5000.0f
#define M0_SPEED_SINE_MAX_RPM       500.0f
#define M0_SPEED_SINE_MIN_HZ        1.0f
#define M0_SPEED_SINE_MAX_HZ        190.0f

/*3505 kv650*/
#define M0_POLE_PAIRS               10                                          /*极对数*/
#define M0_GEAR_RATIO               1                                           /*减速比*/
#define M0_MAX_SPEED_RPM            6500.0f
#define M0_MAX_SPEED                (M0_MAX_SPEED_RPM*_2PI/60.0f*M0_POLE_PAIRS) /*最大电角速度*/
#define M0_VBUS                     24.0f                                       /*母线电压*/
#define M0_DEFAULT_UQ               0.5f                                        /*默认Uq*/
#define M0_DEFAULT_UD               0.0f                                        /*默认Ud*/

/* Software diagnostics use only signals that exist on this board. These
 * thresholds are software defaults pending dedicated board fault injection. */
#define M0_FAULT_VBUS_UNDERVOLTAGE       18.0f
#define M0_FAULT_VBUS_UNDERVOLTAGE_CLEAR 19.0f
#define M0_FAULT_VBUS_OVERVOLTAGE        28.0f
#define M0_FAULT_VBUS_OVERVOLTAGE_CLEAR  27.0f
#define M0_FAULT_BUS_SET_TIME_S           0.020f
#define M0_FAULT_BUS_CLEAR_TIME_S         0.100f
#define M0_FAULT_OVERSPEED                (1.05f*M0_MAX_SPEED)
#define M0_FAULT_OVERSPEED_CLEAR          M0_MAX_SPEED
#define M0_FAULT_OVERSPEED_SET_TIME_S     0.020f
#define M0_FAULT_OVERSPEED_CLEAR_TIME_S   0.100f
#define M0_FAULT_DEADLINE_S               25.0e-6f
#define M0_FAULT_DEADLINE_CLEAR_S         23.0e-6f
#define M0_FAULT_DEADLINE_SET_TIME_S      0.001f
#define M0_FAULT_DEADLINE_CLEAR_TIME_S    0.005f
#define M0_WARNING_SPEED_ERROR            M0_RPM_TO_ELEC_RAD_S(500.0f)
#define M0_WARNING_SPEED_ERROR_CLEAR      M0_RPM_TO_ELEC_RAD_S(250.0f)
#define M0_WARNING_SPEED_REF_SETTLED      M0_RPM_TO_ELEC_RAD_S(25.0f)
#define M0_WARNING_POSITION_ERROR         (20.0f*PI/180.0f)
#define M0_WARNING_POSITION_ERROR_CLEAR   (10.0f*PI/180.0f)
#define M0_WARNING_POSITION_REF_SETTLED   (1.0f*PI/180.0f)
#define M0_WARNING_TRACK_SET_TIME_S        0.500f
#define M0_WARNING_TRACK_CLEAR_TIME_S      0.200f
/*辨识相关*/
#define M0_IDEN_IMAX                3.0f                                         /*阶段1 RL辨识点: ±1/±2/±3 A*/
#define M0_IDEN_SPEED_L             1000.0f                                     /*磁链辨识低速*/
#define M0_IDEN_SPEED_H             3000.0f                                     /*磁链辨识高速*/

/*VF*/
#define M0_VF_DEFAULT_K             (60.0f/M0_MOTOR_KV/M0_POLE_PAIRS/2*_2PI)
#define M0_VF_FREQ_HIGH             (25.0f)
#define M0_VF_FREQ_LOW              (10.0f)
/*ABZ编码器*/
#define M0_ABZENC_PHASE_OFFSET      -2.232439f
#define M0_ABZENC_INSTALL_DIR       1            /*编码器安装方向*/
/*ABS编码器*/
#define M0_ABSENC_PHASE_OFFSET      0.046832f
#define M0_ABSENC_INSTALL_DIR       -1            /*编码器安装方向*/
/*高频注入*/
#define M0_HFI_VOLT_REF             0.5f
#define M0_HFI_HALF_PERIOD_SAMPLES  2U              /*40 kHz FOC下为10 kHz方波*/
#define M0_HFI_DEMOD_FILTER_HZ      1000.0f
#define M0_HFI_PLL_HZ               100.0f
#define M0_HFI_D_AXIS_ACQUIRE_PLL_HZ 20.0f
#define M0_HFI_D_AXIS_TRACK_PLL_HZ   40.0f
#define M0_HFI_SPEED_LEAK_HZ         1.0f
#define M0_HFI_SPEED_FILTER_HZ       50.0f
#define M0_HFI_DEMOD_POLARITY        1.0f           /*D-070实际电压张量回归*/
#define M0_HFI_OUTPUT_ALIGNMENT_RAD  0.0f            /*启动定向后在线校准*/
#define M0_HFI_IPD_CURRENT_A          1.0f           /*D-075八位置IPD消除0.5 A对比重叠*/
#define M0_HFI_SPEED_KP                0.00160960136f
#define M0_HFI_SPEED_KI                0.012f
#define M0_HFI_CURRENT_LIMIT_A         1.5f
#define M0_HFI_ACCEL_CURRENT_LIMIT_A   0.50f
#define M0_HFI_ACCEL_RATE_RPM_S        250.0f
#define M0_HFI_ACCEL_KP                0.0016f
#define M0_HFI_ACCEL_TARGET_RPM        350.0f
#define M0_HFI_SPEED_LOOP_ENTRY_RPM    250.0f
#define M0_HFI_ACCEL_QUALIFY_TIME_S    0.020f

/*电流环PID*/
/*40K*/
/* D-032 measured-plant design: 925 Hz target, 80 deg phase margin.
 * Parallel PI units are Kp [V/A] and Ki [V/(A*s)]. */
#define M0_IQ_KP                    (0.251296400f)
#define M0_IQ_KI                    (1151.639999f)
#define M0_ID_KP                    (0.155978918f)
#define M0_ID_KI                    (1086.465606f)

#define M0_IQD_ITERM_LIMIT          50000000000.0f

#define M0_IQD_OUTLIMIT             15.0f                /*qd轴电流环 输出值限制*/
#define M0_IQD_REF_LIMIT            7.0f

/*速度环PID*/
/* D-043 engineering default: fc24/PM70, about 45-55 Hz measured closed-loop BW. */
#define M0_S_KP                     (0.00536533785f)
#define M0_S_KI                     (0.258523708f)
#define M0_S_ITERM_LIMIT            10000000.0f
#define M0_S_REF_LIMIT              M0_MAX_SPEED
#define M0_S_OUT_LIMIT              M0_IQD_REF_LIMIT

/* D-061 field weakening: 50 Hz stator-voltage feedback PI. The 200 Hz
 * voltage filter alpha is evaluated at the 4 kHz speed-control rate. */
#define M0_FW_TOTAL_CURRENT_A       5.0f
#define M0_FW_TARGET_UTILIZATION    0.95f
#define M0_FW_KP                    0.4f
#define M0_FW_KI                    2400.0f
#define M0_FW_VOLTAGE_FILTER_ALPHA  0.2695973f
#define M0_FW_ID_RATE_A_PER_S       500.0f

/* D-053/D-054 no-load position default; TRAP 180/900 with velocity
 * feedforward remains an optional faster trajectory. */
#define M0_POS_KP                       (376.991118f)
#define M0_POS_KI                       0
#define M0_POS_KD                       0.0f
#define M0_POS_ITERM_LIMIT              10000000.0f
#define M0_POS_REF_LIMIT                100.0f
#define M0_POS_OUT_LIMIT                M0_S_REF_LIMIT
#define M0_POS_DEFAULT_RATE_DEG_S       180.0f

/* D-066 sensorless shadow observers. Both run at the 40 kHz current-loop
 * rate while ABZ remains the only control angle. */
#define M0_ESMO_SLIDING_GAIN_V          12.0f
#define M0_ESMO_BOUNDARY_CURRENT_A      0.80f
#define M0_ESMO_EMF_FILTER_HZ           300.0f
#define M0_ESMO_PLL_HZ                  60.0f
#define M0_ACTIVE_FLUX_CONVERGENCE_HZ   100.0f
#define M0_ACTIVE_FLUX_PLL_HZ           200.0f
/* D-067 ABZ-shadow fit: observer-to-control-frame offset and residual
 * voltage/current-chain delay beyond the observer's half-sample term. */
#define M0_SENSORLESS_ALIGNMENT_RAD      0.048137647f
#define M0_SENSORLESS_EXTRA_DELAY_S      20.2898e-6f
#define M0_SENSORLESS_ALIGN_CURRENT_A     0.8f
#define M0_SENSORLESS_OPEN_CURRENT_A      1.0f
#define M0_SENSORLESS_HANDOFF_CURRENT_A   0.15f
#define M0_SENSORLESS_CURRENT_TAPER_A_S   5.0f
#define M0_SENSORLESS_ALIGN_TIME_S        0.20f
#define M0_SENSORLESS_ACCEL_RPM_S         3000.0f
#define M0_SENSORLESS_ACCEL_RAD_S2        (M0_SENSORLESS_ACCEL_RPM_S*_2PI/60.0f*M0_POLE_PAIRS)
#define M0_SENSORLESS_HANDOFF_RPM         1000.0f
#define M0_SENSORLESS_HANDOFF_RAD_S       (M0_SENSORLESS_HANDOFF_RPM*_2PI/60.0f*M0_POLE_PAIRS)
#define M0_SENSORLESS_PHASE_GATE_RAD      (60.0f*PI/180.0f)
#define M0_SENSORLESS_SPEED_GATE_RPM      100.0f
#define M0_SENSORLESS_SPEED_GATE_RAD_S    (M0_SENSORLESS_SPEED_GATE_RPM*_2PI/60.0f*M0_POLE_PAIRS)
#define M0_SENSORLESS_QUALIFY_TIME_S      0.020f
#define M0_SENSORLESS_BLEND_TIME_S        0.020f
#define M0_SENSORLESS_QUALIFY_TIMEOUT_S   0.50f

/* HFI的20 Hz d轴PLL实测可覆盖500 rpm，Active Flux已验证到500 rpm。
 * 500...650 rpm作为重叠区；700/450 rpm是观测器有效性停止边界。 */
#define M0_HFI_TO_FLUX_RPM             550.0f
#define M0_HFI_PREPARE_RPM             650.0f
#define M0_FLUX_TO_HFI_RPM             525.0f
#define M0_HFI_SPEED_CEILING_RPM       700.0f
#define M0_FLUX_SPEED_FLOOR_RPM        450.0f
#define M0_HFI_FLUX_ANGLE_GATE_RAD     (30.0f*PI/180.0f)
#define M0_HFI_FLUX_SPEED_GATE_RPM     100.0f
#define M0_HFI_FLUX_QUALIFY_TIME_S     0.020f
#define M0_HFI_FLUX_BLEND_TIME_S       0.020f
#define M0_RPM_TO_ELEC_RAD_S(rpm)      ((rpm)*_2PI/60.0f*M0_POLE_PAIRS)


/*要注意状态机在foc_motor构建过程中就启动了, 所有有关FOC的数据结构都在m0之中*/
foc_motor *m0 = NULL;

static uint8_t FOC_CommandAccepted(void)
{
    foc_command_reason = "NONE";
    return 1U;
}

static uint8_t FOC_CommandRejected(const char *reason)
{
    foc_command_reason = reason;
    return 0U;
}

static uint8_t FOC_CommandStartRejected(void)
{
    const FOC_LIFECYCLE_t lifecycle = m0->lifecycle_get();
    if((lifecycle == FOC_LIFECYCLE_FAULT_ACTIVE)
    || (lifecycle == FOC_LIFECYCLE_FAULT_LATCHED)){
        return FOC_CommandRejected("ACTIVE_FAULT");
    }
    if(lifecycle != FOC_LIFECYCLE_READY){
        return FOC_CommandRejected("NOT_READY");
    }
    if((adc3_dma->vbus <= M0_FAULT_VBUS_UNDERVOLTAGE_CLEAR)
    || (adc3_dma->vbus >= M0_FAULT_VBUS_OVERVOLTAGE_CLEAR)){
        return FOC_CommandRejected("BUS_VOLTAGE");
    }
    return FOC_CommandRejected("REQUEST_BUSY");
}

extern "C" const char *FOC_CommandLastReason(void)
{
    return foc_command_reason;
}

static const char *FOC_LifecycleName(FOC_LIFECYCLE_t lifecycle)
{
    switch(lifecycle){
        case FOC_LIFECYCLE_IDLE: return "IDLE";
        case FOC_LIFECYCLE_CURRENT_CALIBRATION: return "CURRENT_CALIBRATION";
        case FOC_LIFECYCLE_READY: return "READY";
        case FOC_LIFECYCLE_STARTING: return "STARTING";
        case FOC_LIFECYCLE_RUNNING: return "RUNNING";
        case FOC_LIFECYCLE_FAULT_ACTIVE: return "FAULT_ACTIVE";
        case FOC_LIFECYCLE_FAULT_LATCHED: return "FAULT_LATCHED";
    }
    return "UNKNOWN";
}

static const char *FOC_OperationName(FOC_OPERATION_t operation)
{
    switch(operation){
        case FOC_OPERATION_NONE: return "NONE";
        case FOC_OPERATION_IDENTIFY_RL: return "IDENTIFY_RL";
        case FOC_OPERATION_IDENTIFY_FLUX: return "IDENTIFY_FLUX";
        case FOC_OPERATION_OPEN_VOLTAGE: return "OPEN_VOLTAGE";
        case FOC_OPERATION_CURRENT: return "CURRENT";
        case FOC_OPERATION_SPEED: return "SPEED";
        case FOC_OPERATION_POSITION_VOLTAGE: return "POSITION_VOLTAGE";
        case FOC_OPERATION_POSITION_CURRENT: return "POSITION_CURRENT";
        case FOC_OPERATION_POSITION_SPEED: return "POSITION_SPEED";
    }
    return "UNKNOWN";
}

static const char *FOC_RotorSourceName(ROTOR_SOURCE_t source)
{
    switch(source){
        case ANGLE_GEN_VF: return "VF";
        case ANGLE_GEN_ABZENC: return "ABZ";
        case ANGLE_GEN_ABSENC: return "ABS";
        case ANGLE_GEN_FLUX: return "ACTIVE_FLUX";
        case ANGLE_GEN_SMO: return "SMO";
        case ANGLE_GEN_HFI: return "HFI";
        case ANGLE_GEN_SENSORLESS: return "SENSORLESS";
    }
    return "UNKNOWN";
}

static const char *FOC_DiagnosticName(FOC_DIAGNOSTIC_t diagnostic)
{
    switch(diagnostic){
        case FOC_DIAGNOSTIC_NONE: return "NONE";
        case FOC_DIAGNOSTIC_HFI_ENCODER_SHADOW: return "HFI_ENCODER_SHADOW";
        case FOC_DIAGNOSTIC_HFI_D_AXIS_SCAN: return "HFI_D_AXIS_SCAN";
        case FOC_DIAGNOSTIC_HFI_TENSOR_SCAN: return "HFI_TENSOR_SCAN";
    }
    return "UNKNOWN";
}

static const char *FOC_FaultName(uint32_t fault)
{
    switch(fault){
        case FOC_FAULT_NONE: return "NONE";
        case FOC_FAULT_CURRENT_OFFSET: return "CURRENT_OFFSET";
        case FOC_FAULT_SOFTWARE_OVERCURRENT: return "SOFTWARE_OVERCURRENT";
        case FOC_FAULT_BUS_UNDERVOLTAGE: return "BUS_UNDERVOLTAGE";
        case FOC_FAULT_BUS_OVERVOLTAGE: return "BUS_OVERVOLTAGE";
        case FOC_FAULT_OVERSPEED: return "OVERSPEED";
        case FOC_FAULT_SENSORLESS_STARTUP: return "SENSORLESS_STARTUP";
        case FOC_FAULT_ROTOR_SOURCE_LOST: return "ROTOR_SOURCE_LOST";
        case FOC_FAULT_CONTROL_DEADLINE: return "CONTROL_DEADLINE";
        case FOC_FAULT_SPEED_TRACKING: return "SPEED_TRACKING";
        case FOC_FAULT_POSITION_TRACKING: return "POSITION_TRACKING";
    }
    return "MULTIPLE_OR_UNKNOWN";
}


void FOC_Init(void)
{
    /*定时器周期值*/
    uint32_t htim1_period = htim1.Init.Period+1;
    /*定时器频率*/
    uint32_t htim1_freq = 240000000/(htim1.Init.Prescaler+1)/htim1_period;
    /*默认Uqd大小*/
    qd_t Uqd = {M0_DEFAULT_UQ, M0_DEFAULT_UD};
    /*FOC运行周期时间*/
    float Ts = 2.0f/(float)htim1_freq;
    /*FOC一个周期的PWM滴答值*/
    uint32_t Tpwm = 2*htim1_period;                                /*因为SVPWM时 定时器1是中央对齐的方式, 所以实际上的周期是两倍Period*/

    foc_control_sample_hz = (float)htim1_freq/2.0f;

    printf("\r\nDefault Uq:%.2fV, Ud:%.2fV, period:%.3fms, timer counts:%d\r\n", M0_DEFAULT_UQ,  M0_DEFAULT_UD, Ts*1000, Tpwm);

    /*创建电机实例*/
    m0 = new foc_motor(Uqd, M0_MAX_SPEED, M0_VBUS, Ts, MODE_IDLE, SVPWM_MODE, ANGLE_GEN_ABZENC);

    m0->vf      = new foc_vf(M0_VF_DEFAULT_K, M0_VF_FREQ_HIGH, M0_VF_FREQ_LOW, M0_POLE_PAIRS, M0_GEAR_RATIO);
    m0->absenc  = new foc_absenc(M0_POLE_PAIRS, M0_GEAR_RATIO, M0_ABSENC_PHASE_OFFSET, M0_ABSENC_INSTALL_DIR);
    m0->abzenc  = new foc_abzenc(M0_POLE_PAIRS, M0_GEAR_RATIO, M0_ABZENC_PHASE_OFFSET, M0_ABZENC_INSTALL_DIR);
    m0->set_speed_estimator_mode(FOC_SPEED_ESTIMATOR_WIN_IIR);
    m0->smo     = new foc_smo(
        M0_IDENTIFIED_RS, M0_IDENTIFIED_LQ,
        M0_ESMO_SLIDING_GAIN_V, M0_ESMO_BOUNDARY_CURRENT_A,
        M0_ESMO_EMF_FILTER_HZ, M0_ESMO_PLL_HZ,
        M0_POLE_PAIRS, M0_GEAR_RATIO, M0_SENSORLESS_ALIGNMENT_RAD,
        M0_SENSORLESS_EXTRA_DELAY_S
    );
    m0->flux    = new foc_flux(
        M0_IDENTIFIED_RS, M0_IDENTIFIED_LD, M0_IDENTIFIED_LQ,
        M0_IDENTIFIED_FLUX, M0_ACTIVE_FLUX_CONVERGENCE_HZ,
        M0_ACTIVE_FLUX_PLL_HZ, M0_POLE_PAIRS, M0_GEAR_RATIO,
        M0_SENSORLESS_ALIGNMENT_RAD, M0_SENSORLESS_EXTRA_DELAY_S
    );
    m0->hfi     = new foc_hfi(
        M0_IDENTIFIED_LD, M0_IDENTIFIED_LQ, M0_HFI_VOLT_REF,
        M0_HFI_HALF_PERIOD_SAMPLES, M0_HFI_DEMOD_FILTER_HZ,
        M0_HFI_PLL_HZ, M0_HFI_SPEED_LEAK_HZ,
        M0_HFI_SPEED_FILTER_HZ,
        M0_HFI_DEMOD_POLARITY,
        M0_HFI_OUTPUT_ALIGNMENT_RAD,
        M0_HFI_IPD_CURRENT_A,
        M0_POLE_PAIRS, M0_GEAR_RATIO, 0.0f,
        M0_HFI_D_AXIS_ACQUIRE_PLL_HZ,
        M0_HFI_D_AXIS_TRACK_PLL_HZ
    );

    m0->current         = new foc_current(4096.0f/2.0f, M0_IQD_REF_LIMIT*1.5f);
    m0->pid_iq          = new foc_pid(M0_IQ_KP, M0_IQ_KI, 0, M0_IQD_REF_LIMIT, M0_IQD_ITERM_LIMIT, M0_IQD_OUTLIMIT);
    m0->pid_id          = new foc_pid(M0_ID_KP, M0_ID_KI, 0, M0_IQD_REF_LIMIT, M0_IQD_ITERM_LIMIT, M0_IQD_OUTLIMIT);
    m0->spd_ctrl        = new foc_spd_ctrl(M0_S_KP, M0_S_KI, M0_S_REF_LIMIT, M0_S_ITERM_LIMIT, M0_S_OUT_LIMIT);
    m0->pos_ctrl        = new foc_pos_ctrl(M0_POS_KP, M0_POS_KI, M0_POS_KD, M0_POS_REF_LIMIT, M0_POS_ITERM_LIMIT, M0_POS_OUT_LIMIT,
                                TC_POS_CONTINUOUS, TC_ROTATE_SHORTEST_PATH);
    /* D-059 reset default: Stage-1 identified motor model with full dq feedforward. */
    m0->configure_current_feedforward(
        M0_IDENTIFIED_LD, M0_IDENTIFIED_LQ, M0_IDENTIFIED_FLUX
    );
    m0->configure_sensorless_startup(
        M0_SENSORLESS_ALIGN_CURRENT_A, M0_SENSORLESS_OPEN_CURRENT_A,
        M0_SENSORLESS_HANDOFF_CURRENT_A, M0_SENSORLESS_CURRENT_TAPER_A_S,
        M0_SENSORLESS_ALIGN_TIME_S, M0_SENSORLESS_ACCEL_RAD_S2,
        M0_SENSORLESS_HANDOFF_RAD_S, M0_SENSORLESS_PHASE_GATE_RAD,
        M0_SENSORLESS_SPEED_GATE_RAD_S, M0_SENSORLESS_QUALIFY_TIME_S,
        M0_SENSORLESS_BLEND_TIME_S, M0_SENSORLESS_QUALIFY_TIMEOUT_S
    );
    m0->configure_hfi_flux_handoff(
        M0_RPM_TO_ELEC_RAD_S(M0_HFI_TO_FLUX_RPM),
        M0_RPM_TO_ELEC_RAD_S(M0_HFI_PREPARE_RPM),
        M0_RPM_TO_ELEC_RAD_S(M0_FLUX_TO_HFI_RPM),
        M0_RPM_TO_ELEC_RAD_S(M0_HFI_SPEED_CEILING_RPM),
        M0_RPM_TO_ELEC_RAD_S(M0_FLUX_SPEED_FLOOR_RPM),
        M0_HFI_FLUX_ANGLE_GATE_RAD,
        M0_RPM_TO_ELEC_RAD_S(M0_HFI_FLUX_SPEED_GATE_RPM),
        M0_HFI_FLUX_QUALIFY_TIME_S, M0_HFI_FLUX_BLEND_TIME_S,
        M0_HFI_SPEED_KP, M0_HFI_SPEED_KI, M0_S_KP, M0_S_KI,
        M0_HFI_CURRENT_LIMIT_A, M0_S_OUT_LIMIT
    );
    m0->configure_hfi_speed_startup(
        M0_HFI_ACCEL_CURRENT_LIMIT_A,
        M0_RPM_TO_ELEC_RAD_S(M0_HFI_ACCEL_RATE_RPM_S),
        M0_HFI_ACCEL_KP,
        M0_RPM_TO_ELEC_RAD_S(M0_HFI_ACCEL_TARGET_RPM),
        M0_RPM_TO_ELEC_RAD_S(M0_HFI_SPEED_LOOP_ENTRY_RPM),
        M0_HFI_ACCEL_QUALIFY_TIME_S
    );
    m0->enable_current_feedforward(true);
    m0->set_speed_current_limit(M0_S_OUT_LIMIT);
    m0->configure_field_weakening(
        M0_IDENTIFIED_RS, M0_IDENTIFIED_LD, M0_IDENTIFIED_LQ,
        M0_IDENTIFIED_FLUX, M0_FW_KP, M0_FW_KI,
        M0_FW_TOTAL_CURRENT_A, M0_FW_TARGET_UTILIZATION,
        M0_FW_VOLTAGE_FILTER_ALPHA, M0_FW_ID_RATE_A_PER_S
    );
    /* D-064 reset default: identified voltage-ellipse feedforward with
     * stator-voltage feedback trim. It remains inactive below the boundary. */
    m0->set_field_weakening_mode(FIELD_WEAKENING_MODE_HYBRID);
    m0->set_position_track(TRACK_MODE_RAMP, M0_POS_DEFAULT_RATE_DEG_S*PI/180.0f);

    foc_fault_monitor_config_t fault_config = {};
    fault_config.bus_undervoltage = M0_FAULT_VBUS_UNDERVOLTAGE;
    fault_config.bus_undervoltage_clear = M0_FAULT_VBUS_UNDERVOLTAGE_CLEAR;
    fault_config.bus_overvoltage = M0_FAULT_VBUS_OVERVOLTAGE;
    fault_config.bus_overvoltage_clear = M0_FAULT_VBUS_OVERVOLTAGE_CLEAR;
    fault_config.bus_set_time = M0_FAULT_BUS_SET_TIME_S;
    fault_config.bus_clear_time = M0_FAULT_BUS_CLEAR_TIME_S;
    fault_config.overspeed = M0_FAULT_OVERSPEED;
    fault_config.overspeed_clear = M0_FAULT_OVERSPEED_CLEAR;
    fault_config.overspeed_set_time = M0_FAULT_OVERSPEED_SET_TIME_S;
    fault_config.overspeed_clear_time = M0_FAULT_OVERSPEED_CLEAR_TIME_S;
    fault_config.deadline = M0_FAULT_DEADLINE_S;
    fault_config.deadline_clear = M0_FAULT_DEADLINE_CLEAR_S;
    fault_config.deadline_set_time = M0_FAULT_DEADLINE_SET_TIME_S;
    fault_config.deadline_clear_time = M0_FAULT_DEADLINE_CLEAR_TIME_S;
    fault_config.speed_tracking_error = M0_WARNING_SPEED_ERROR;
    fault_config.speed_tracking_clear = M0_WARNING_SPEED_ERROR_CLEAR;
    fault_config.speed_reference_settled = M0_WARNING_SPEED_REF_SETTLED;
    fault_config.speed_tracking_set_time = M0_WARNING_TRACK_SET_TIME_S;
    fault_config.speed_tracking_clear_time = M0_WARNING_TRACK_CLEAR_TIME_S;
    fault_config.position_tracking_error = M0_WARNING_POSITION_ERROR;
    fault_config.position_tracking_clear = M0_WARNING_POSITION_ERROR_CLEAR;
    fault_config.position_reference_settled = M0_WARNING_POSITION_REF_SETTLED;
    fault_config.position_tracking_set_time = M0_WARNING_TRACK_SET_TIME_S;
    fault_config.position_tracking_clear_time = M0_WARNING_TRACK_CLEAR_TIME_S;
    m0->configure_fault_monitor(fault_config);

    m0->time            = new foc_time();
    m0->offline_iden    = new foc_offline_iden(M0_IDEN_IMAX, M0_IDEN_SPEED_L, M0_IDEN_SPEED_H,
                                               M0_IDENTIFIED_RS, M0_IDENTIFIED_LQ, M0_IDENTIFIED_LD);


    /*所有控制对象创建完成后，再启动定时器和FOC状态机*/
    m0->pwm             = new foc_pwm(Tpwm);
    Debug_ScopeInit(htim1_freq/2U);
    (void)m0->request_prepare();
    FOC_PWM_HW_StartControl();
}

void FOC_RUN(void)
{
    m0->timing_clear();
    const float measured_vbus = adc3_dma->vbus;
    m0->set_bus_voltage(measured_vbus);

    /*更新电流*/
    FOC_CURRENT_REFRESH(m0->current->mcu_ad, m0->current->ad_offset, m0->current->Iabc);
    if(foc_current_sine_active && foc_host_current_control
       && (m0->mode_get() == MODE_TORQUE_RUN)){
        const float sine = foc_current_sine_value;
        const float quadrature = foc_current_sine_quadrature;
        const float reference = foc_current_sine_amplitude*sine;
        if(foc_current_sine_q_axis){
            m0->set_current_reference(reference, 0.0f);
        }else{
            m0->set_current_reference(0.0f, reference);
        }
        foc_current_sine_value = sine*foc_current_cosine_step
                               + quadrature*foc_current_sine_step;
        foc_current_sine_quadrature = quadrature*foc_current_cosine_step
                                    - sine*foc_current_sine_step;
    }
    if(foc_speed_sine_active && foc_host_speed_control
       && (m0->mode_get() == MODE_SPEED_TORQUE_RUN)){
        const float sine = foc_speed_sine_value;
        const float quadrature = foc_speed_sine_quadrature;
        foc_speed_command_rpm = foc_speed_sine_bias_rpm
                              + foc_speed_sine_amplitude_rpm*sine;
        foc_speed_sine_value = sine*foc_speed_cosine_step
                             + quadrature*foc_speed_sine_step;
        foc_speed_sine_quadrature = quadrature*foc_speed_cosine_step
                                  - sine*foc_speed_sine_step;
    }
    m0->run();
    FOC_ApplyPowerStage(m0->power_stage_get());

    /* Keep all control-loop scheduling on the ADC/FOC timebase. Each method
     * retains its established decimation counter. Updated references are
     * consumed by the next current-loop sample. */
    FOC_SPEED_CALC();
    FOC_SPEED_CTRL();
    FOC_POS_CTRL();

    ++foc_debug_seq;
    __DMB();
    foc_debug = m0->debug;
    foc_debug.vbus = measured_vbus;
    __DMB();
    ++foc_debug_seq;
    Debug_ScopeSample(&foc_debug);
    /* Evaluate the complete ADC/FOC callback on the following sample. */
    m0->timing_capture();
}

void FOC_SPEED_CALC(void)
{
    m0->SpeedCalc();
}

void FOC_SPEED_CTRL(void)
{
    float speed_rate = 0;

    if(foc_host_speed_control){
        speed_rate = foc_speed_command_rpm/M0_MAX_SPEED_RPM;
    }else{
        int32_t adc_pot = adc3_dma->adc_pot;
        speed_rate = (float)adc_pot/100.0f;
    }

    m0->SpeedCtrl(speed_rate);
}

extern "C" uint8_t FOC_CommandStartSpeed(void)
{
    if(m0 == NULL){
        return FOC_CommandRejected("NOT_INITIALIZED");
    }

    if(m0->request_start(MODE_SPEED_TORQUE_RUN) == false){
        return FOC_CommandStartRejected();
    }

    foc_speed_command_rpm = 0.0f;
    foc_speed_sine_active = false;
    foc_speed_sine_amplitude_rpm = 0.0f;
    foc_speed_sine_bias_rpm = 0.0f;
    foc_host_speed_control = true;
    foc_host_current_control = false;
    foc_host_position_control = false;
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandStartCurrent(void)
{
    if(m0 == NULL){
        return FOC_CommandRejected("NOT_INITIALIZED");
    }

    if(m0->request_start(MODE_TORQUE_RUN) == false){
        return FOC_CommandStartRejected();
    }

    foc_current_sine_active = false;
    foc_speed_sine_active = false;
    foc_speed_sine_amplitude_rpm = 0.0f;
    foc_speed_sine_bias_rpm = 0.0f;
    m0->set_current_reference(0.0f, 0.0f);
    foc_host_speed_control = false;
    foc_host_current_control = true;
    foc_host_position_control = false;
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandStartPosition(void)
{
    if(m0 == NULL){
        return FOC_CommandRejected("NOT_INITIALIZED");
    }

    if(m0->request_start(MODE_POSITION_SPEED_TORQUE_RUN) == false){
        return FOC_CommandStartRejected();
    }

    /* Position tracking owns the cascade reference trajectory. */
    m0->set_speed_track(TRACK_MODE_NONE, 0.0f);
    foc_speed_sine_active = false;
    foc_speed_sine_amplitude_rpm = 0.0f;
    foc_speed_sine_bias_rpm = 0.0f;

    foc_position_origin_rad = 0.0f;
    foc_position_command_rad = 0.0f;
    foc_position_command_initialized = false;
    foc_host_speed_control = false;
    foc_host_current_control = false;
    foc_host_position_control = true;
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandStartIdentificationRL(void)
{
    if((m0 == NULL) || (m0->request_start(MODE_IDEN_RL) == false)){
        return 0U;
    }

    foc_host_speed_control = false;
    foc_host_current_control = false;
    foc_host_position_control = false;
    return 1U;
}

extern "C" uint8_t FOC_CommandStartIdentificationFlux(void)
{
    if((m0 == NULL) || (m0->request_start(MODE_IDEN_F) == false)){
        return 0U;
    }

    foc_host_speed_control = false;
    foc_host_current_control = false;
    foc_host_position_control = false;
    return 1U;
}

extern "C" uint8_t FOC_CommandPrintIdentification(void)
{
    if((m0 == NULL) || (m0->mode_get() != MODE_READY)){
        return 0U;
    }

    foc_iden_result_t result = {};
    m0->identification_result_get(result);
    usb_printf("IDENT_RL,Rs=%.8g,Voff=%.8g,RsRmse=%.8g,Ld=%.8g,Lq=%.8g,Ls=%.8g,"
               "Rd=%.8g,Rq=%.8g,IdRms=%.8g,IqRms=%.8g\r\n",
               result.rs, result.inverter_voltage_offset, result.rs_rmse,
               result.ld, result.lq, result.ls,
               result.ld_eq_resistance, result.lq_eq_resistance,
               result.ld_current_rms, result.lq_current_rms);
    usb_printf("IDENT_RS_DIAG,RsF=%.8g,RsR=%.8g,RsDelta=%.8g,VoffF=%.8g,VoffR=%.8g,"
               "VoffDelta=%.8g,RsHoldoutRmse=%.8g\r\n",
               result.rs_sweep_forward, result.rs_sweep_reverse,
               result.rs_sweep_delta, result.voltage_offset_forward,
               result.voltage_offset_reverse, result.voltage_offset_delta,
               result.rs_holdout_rmse);
    usb_printf("IDENT_LCHECK,LdHigh=%.8g,LqHigh=%.8g,LdDelta=%.8g,LqDelta=%.8g,"
               "IdHighRms=%.8g,IqHighRms=%.8g\r\n",
               result.ld_high, result.lq_high, result.ld_amplitude_delta,
               result.lq_amplitude_delta, result.ld_high_current_rms,
               result.lq_high_current_rms);
    usb_printf("IDENT_FLUX,Flux=%.8g,FluxIntercept=%.8g,FluxRmse=%.8g,"
               "FluxHoldoutRmse=%.8g,FluxHoldoutMax=%.8g\r\n",
               result.flux, result.flux_voltage_intercept, result.flux_rmse,
               result.flux_holdout_rmse, result.flux_holdout_max_error);
    return 1U;
}

extern "C" uint8_t FOC_CommandPrintIdentificationPoints(void)
{
    if((m0 == NULL) || (m0->mode_get() != MODE_READY)){
        return 0U;
    }

    foc_iden_result_t result = {};
    m0->identification_result_get(result);
    for(uint32_t start = 0U; start < result.rs_point_count; start += 4U){
        usb_printf("IDENT_RS_POINTS,%u=%.8g:%.8g,%u=%.8g:%.8g,"
                   "%u=%.8g:%.8g,%u=%.8g:%.8g\r\n",
                   start, result.rs_current[start], result.rs_voltage[start],
                   start + 1U, result.rs_current[start + 1U], result.rs_voltage[start + 1U],
                   start + 2U, result.rs_current[start + 2U], result.rs_voltage[start + 2U],
                   start + 3U, result.rs_current[start + 3U], result.rs_voltage[start + 3U]);
    }
    return 1U;
}

extern "C" uint8_t FOC_CommandSetSpeed(float speed_rpm)
{
    if(foc_host_speed_control == false){
        return FOC_CommandRejected("WRONG_OPERATION");
    }
    if(!isfinite(speed_rpm) || (speed_rpm > M0_MAX_SPEED_RPM)
       || (speed_rpm < -M0_MAX_SPEED_RPM)){
        return FOC_CommandRejected("OUT_OF_RANGE");
    }

    foc_speed_sine_active = false;
    foc_speed_sine_amplitude_rpm = 0.0f;
    foc_speed_sine_bias_rpm = 0.0f;
    foc_speed_command_rpm = speed_rpm;
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandSetSpeedSine(float amplitude_rpm,
                                             float frequency_hz)
{
    float phase_step;

    if((m0 == NULL) || (foc_host_speed_control == false)
       || (m0->mode_get() != MODE_SPEED_TORQUE_RUN)
       || !isfinite(amplitude_rpm) || !isfinite(frequency_hz)
       || (amplitude_rpm <= 0.0f) || (amplitude_rpm > M0_SPEED_SINE_MAX_RPM)
       || ((fabsf(foc_speed_command_rpm) + amplitude_rpm) > M0_MAX_SPEED_RPM)
       || (frequency_hz < M0_SPEED_SINE_MIN_HZ)
       || (frequency_hz > M0_SPEED_SINE_MAX_HZ)){
        return 0U;
    }

    phase_step = _2PI*frequency_hz/foc_control_sample_hz;
    foc_speed_sine_active = false;
    foc_speed_sine_amplitude_rpm = amplitude_rpm;
    foc_speed_sine_bias_rpm = foc_speed_command_rpm;
    foc_speed_sine_value = 0.0f;
    foc_speed_sine_quadrature = 1.0f;
    foc_speed_sine_step = sinf(phase_step);
    foc_speed_cosine_step = cosf(phase_step);
    m0->set_speed_track(TRACK_MODE_NONE, 0.0f);
    foc_speed_sine_active = true;
    return 1U;
}

extern "C" uint8_t FOC_CommandStopSpeedSine(void)
{
    if((m0 == NULL) || (foc_host_speed_control == false)){
        return 0U;
    }

    foc_speed_sine_active = false;
    foc_speed_sine_amplitude_rpm = 0.0f;
    foc_speed_sine_bias_rpm = 0.0f;
    foc_speed_command_rpm = 0.0f;
    return 1U;
}

extern "C" uint8_t FOC_CommandSetSpeedOutputLimit(float limit_a)
{
    if(m0 == NULL){
        return FOC_CommandRejected("NOT_INITIALIZED");
    }
    if(m0->mode_get() != MODE_READY){
        return FOC_CommandRejected("NOT_READY");
    }
    if(!isfinite(limit_a) || (limit_a <= 0.0f)
       || (limit_a > M0_IQD_REF_LIMIT)){
        return FOC_CommandRejected("OUT_OF_RANGE");
    }

    m0->set_speed_current_limit(limit_a);
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandSetSpeedEstimator(uint8_t mode)
{
    if(m0 == NULL){
        return FOC_CommandRejected("NOT_INITIALIZED");
    }
    if(m0->mode_get() != MODE_READY){
        return FOC_CommandRejected("NOT_READY");
    }
    if(mode > (uint8_t)FOC_SPEED_ESTIMATOR_FUSED){
        return FOC_CommandRejected("OUT_OF_RANGE");
    }

    m0->set_speed_estimator_mode((foc_speed_estimator_mode_t)mode);
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandSetCurrent(float iq, float id)
{
    if(m0 == NULL){
        return FOC_CommandRejected("NOT_INITIALIZED");
    }
    if(foc_host_current_control == false){
        return FOC_CommandRejected("WRONG_OPERATION");
    }
    if(!isfinite(iq) || !isfinite(id)
       || (hypotf(iq, id) > M0_IQD_REF_LIMIT)){
        return FOC_CommandRejected("OUT_OF_RANGE");
    }

    foc_current_sine_active = false;
    m0->set_current_reference(iq, id);
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandSetCurrentSine(uint8_t q_axis,
                                                float amplitude,
                                                float frequency_hz)
{
    float phase_step;

    if((m0 == NULL) || (foc_host_current_control == false)
       || (m0->mode_get() != MODE_TORQUE_RUN)
       || !isfinite(amplitude) || !isfinite(frequency_hz)
       || (amplitude <= 0.0f) || (amplitude > M0_CURRENT_SINE_MAX_A)
       || (frequency_hz < M0_CURRENT_SINE_MIN_HZ)
       || (frequency_hz > M0_CURRENT_SINE_MAX_HZ)){
        return 0U;
    }

    phase_step = _2PI*frequency_hz/foc_control_sample_hz;
    foc_current_sine_active = false;
    foc_current_sine_q_axis = (q_axis != 0U);
    foc_current_sine_amplitude = amplitude;
    foc_current_sine_value = 0.0f;
    foc_current_sine_quadrature = 1.0f;
    foc_current_sine_step = sinf(phase_step);
    foc_current_cosine_step = cosf(phase_step);
    m0->set_current_reference(0.0f, 0.0f);
    foc_current_sine_active = true;
    return 1U;
}

extern "C" uint8_t FOC_CommandStopCurrentSine(void)
{
    if((m0 == NULL) || (foc_host_current_control == false)){
        return 0U;
    }

    foc_current_sine_active = false;
    foc_current_sine_q_axis = false;
    foc_current_sine_amplitude = 0.0f;
    m0->set_current_reference(0.0f, 0.0f);
    return 1U;
}

extern "C" uint8_t FOC_CommandSetPosition(float position_deg)
{
    const float position_rad = position_deg*PI/180.0f;

    if(m0 == NULL){
        return FOC_CommandRejected("NOT_INITIALIZED");
    }
    if(foc_host_position_control == false){
        return FOC_CommandRejected("WRONG_OPERATION");
    }
    if(foc_position_command_initialized == false){
        return FOC_CommandRejected("POSITION_NOT_READY");
    }
    if(!isfinite(position_deg) || (fabsf(position_rad) > M0_POS_REF_LIMIT)){
        return FOC_CommandRejected("OUT_OF_RANGE");
    }

    foc_position_command_rad = position_rad;
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandSetSpeedPI(float kp, float ki)
{
    if((m0 == NULL) || !isfinite(kp) || !isfinite(ki) || (kp < 0.0f) || (ki < 0.0f)){
        return 0U;
    }

    m0->set_speed_gains(kp, ki);
    return 1U;
}

extern "C" uint8_t FOC_CommandSetPositionPI(float kp, float ki, float kd)
{
    if((m0 == NULL) || !isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
       (kp < 0.0f) || (ki < 0.0f) || (kd < 0.0f)){
        return 0U;
    }

    m0->set_position_gains(kp, ki, kd);
    return 1U;
}

extern "C" uint8_t FOC_CommandSetPositionSpeedLimit(float limit_rpm)
{
    if((m0 == NULL) || (m0->mode_get() != MODE_READY) ||
       !isfinite(limit_rpm) || (limit_rpm <= 0.0f) ||
       (limit_rpm > M0_MAX_SPEED_RPM)){
        return 0U;
    }

    m0->set_position_output_limit(
        limit_rpm*_2PI/60.0f*M0_POLE_PAIRS
    );
    return 1U;
}

extern "C" uint8_t FOC_CommandSetIqPI(float kp, float ki)
{
    if((m0 == NULL) || !isfinite(kp) || !isfinite(ki) || (kp < 0.0f) || (ki < 0.0f)){
        return 0U;
    }

    m0->set_current_gains(true, kp, ki);
    return 1U;
}

extern "C" uint8_t FOC_CommandSetIdPI(float kp, float ki)
{
    if((m0 == NULL) || !isfinite(kp) || !isfinite(ki) || (kp < 0.0f) || (ki < 0.0f)){
        return 0U;
    }

    m0->set_current_gains(false, kp, ki);
    return 1U;
}

extern "C" uint8_t FOC_CommandClearPI(void)
{
    if(m0 == NULL){
        return 0U;
    }

    m0->clear_control_loops();
    m0->set_speed_id_reference(0.0f);
    m0->set_field_weakening_mode(m0->field_weakening_mode_get());
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandStartHfi(void)
{
    if(m0 == NULL){
        return FOC_CommandRejected("NOT_INITIALIZED");
    }

    if(m0->request_hfi_start(M0_HFI_VOLT_REF) == false){
        return FOC_CommandStartRejected();
    }

    foc_speed_command_rpm = 0.0f;
    foc_speed_sine_active = false;
    foc_speed_sine_amplitude_rpm = 0.0f;
    foc_speed_sine_bias_rpm = 0.0f;
    foc_host_speed_control = true;
    foc_host_current_control = false;
    foc_host_position_control = false;
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandStartSensorless(float speed_rpm)
{
    if(m0 == NULL){
        return FOC_CommandRejected("NOT_INITIALIZED");
    }
    if(!isfinite(speed_rpm) || (speed_rpm == 0.0f)
    || (fabsf(speed_rpm) > M0_MAX_SPEED_RPM)){
        return FOC_CommandRejected("OUT_OF_RANGE");
    }

    if(m0->request_sensorless_start(speed_rpm) == false){
        return FOC_CommandStartRejected();
    }

    m0->set_speed_track(TRACK_MODE_RAMP, M0_SENSORLESS_ACCEL_RAD_S2);
    foc_speed_command_rpm = speed_rpm;
    foc_speed_sine_active = false;
    foc_speed_sine_amplitude_rpm = 0.0f;
    foc_speed_sine_bias_rpm = 0.0f;
    foc_host_speed_control = true;
    foc_host_current_control = false;
    foc_host_position_control = false;
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandStartHfiShadow(float injection_voltage,
                                                float initial_offset_deg)
{
    if((m0 == NULL) || !isfinite(injection_voltage)
    || !isfinite(initial_offset_deg)
    || (injection_voltage < 0.1f) || (injection_voltage > 1.0f)
    || (fabsf(initial_offset_deg) > 80.0f)){
        return 0U;
    }

    return m0->start_hfi_shadow(
        injection_voltage, initial_offset_deg*PI/180.0f
    ) ? 1U : 0U;
}

extern "C" uint8_t FOC_CommandStopHfiShadow(void)
{
    return ((m0 != NULL) && m0->stop_hfi_shadow()) ? 1U : 0U;
}

extern "C" uint8_t FOC_CommandStartHfiCharacterization(
    float injection_voltage, float initial_offset_deg)
{
    if((m0 == NULL) || !isfinite(injection_voltage)
    || !isfinite(initial_offset_deg)
    || (injection_voltage < 0.1f) || (injection_voltage > 1.0f)
    || (fabsf(initial_offset_deg) > 80.0f)){
        return 0U;
    }

    return m0->start_hfi_characterization(
        injection_voltage, initial_offset_deg*PI/180.0f
    ) ? 1U : 0U;
}

extern "C" uint8_t FOC_CommandStartHfiTensorCharacterization(
    float injection_voltage, float initial_offset_deg)
{
    if((m0 == NULL) || !isfinite(injection_voltage)
    || !isfinite(initial_offset_deg)
    || (injection_voltage < 0.1f) || (injection_voltage > 1.0f)
    || (fabsf(initial_offset_deg) > 80.0f)){
        return 0U;
    }

    return m0->start_hfi_tensor_characterization(
        injection_voltage, initial_offset_deg*PI/180.0f
    ) ? 1U : 0U;
}

extern "C" uint8_t FOC_CommandActivateHfiControl(void)
{
    return ((m0 != NULL) && m0->activate_hfi_control()) ? 1U : 0U;
}

extern "C" uint8_t FOC_CommandActivateHfiDAxisTracking(void)
{
    return ((m0 != NULL) && m0->activate_hfi_d_axis_tracking()) ? 1U : 0U;
}

extern "C" uint8_t FOC_CommandSetSpeedStep(void)
{
    if(m0 == NULL){
        return 0U;
    }

    m0->set_speed_track(TRACK_MODE_NONE, 0.0f);
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandSetSpeedRamp(float accel_rpm_per_sec)
{
    float accel_elec;

    if((m0 == NULL) || !isfinite(accel_rpm_per_sec) || (accel_rpm_per_sec <= 0.0f)){
        return 0U;
    }

    accel_elec = accel_rpm_per_sec*_2PI/60.0f*M0_POLE_PAIRS;
    m0->set_speed_track(TRACK_MODE_RAMP, accel_elec);
    return 1U;
}

extern "C" uint8_t FOC_CommandSetPositionStep(void)
{
    if(m0 == NULL){
        return 0U;
    }

    m0->set_position_track(TRACK_MODE_NONE, 0.0f);
    return 1U;
}

extern "C" uint8_t FOC_CommandSetPositionRamp(float velocity_deg_per_sec)
{
    if((m0 == NULL) || !isfinite(velocity_deg_per_sec) || (velocity_deg_per_sec <= 0.0f)){
        return 0U;
    }

    m0->set_position_track(TRACK_MODE_RAMP, velocity_deg_per_sec*PI/180.0f);
    return 1U;
}

extern "C" uint8_t FOC_CommandSetPositionTrapezoid(float velocity_deg_per_sec,
                                                    float acceleration_deg_per_sec2)
{
    if((m0 == NULL)
    || !isfinite(velocity_deg_per_sec) || (velocity_deg_per_sec <= 0.0f)
    || !isfinite(acceleration_deg_per_sec2) || (acceleration_deg_per_sec2 <= 0.0f)){
        return 0U;
    }

    m0->set_position_track(
        TRACK_MODE_TRAPEZOID,
        velocity_deg_per_sec*PI/180.0f,
        acceleration_deg_per_sec2*PI/180.0f
    );
    return 1U;
}

extern "C" uint8_t FOC_CommandSetPositionSCurve(float velocity_deg_per_sec,
                                                  float acceleration_deg_per_sec2,
                                                  float jerk_deg_per_sec3)
{
    if((m0 == NULL)
    || !isfinite(velocity_deg_per_sec) || (velocity_deg_per_sec <= 0.0f)
    || !isfinite(acceleration_deg_per_sec2) || (acceleration_deg_per_sec2 <= 0.0f)
    || !isfinite(jerk_deg_per_sec3) || (jerk_deg_per_sec3 <= 0.0f)){
        return 0U;
    }

    m0->set_position_track(
        TRACK_MODE_SCURVE,
        velocity_deg_per_sec*PI/180.0f,
        acceleration_deg_per_sec2*PI/180.0f,
        jerk_deg_per_sec3*PI/180.0f
    );
    return 1U;
}

extern "C" uint8_t FOC_CommandSetPositionFeedforward(uint8_t enabled)
{
    if((m0 == NULL) || (m0->mode_get() != MODE_READY)){
        return 0U;
    }

    m0->set_position_velocity_feedforward(
        (enabled != 0U) ? M0_POLE_PAIRS : 0.0f
    );
    return 1U;
}

extern "C" uint8_t FOC_CommandSetFeedforward(uint8_t enabled)
{
    if(m0 == NULL){
        return 0U;
    }

    m0->enable_current_feedforward(enabled != 0U);
    return 1U;
}

extern "C" uint8_t FOC_CommandSetFeedforwardMode(uint8_t mode)
{
    if(m0 == NULL){
        return FOC_CommandRejected("NOT_INITIALIZED");
    }
    if(mode > (uint8_t)CURRENT_FF_MODE_BEMF){
        return FOC_CommandRejected("OUT_OF_RANGE");
    }

    m0->set_current_feedforward_mode((CURRENT_FEEDFORWARD_MODE_t)mode);
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandSetFeedforwardModel(uint8_t identified)
{
    if(m0 == NULL){
        return FOC_CommandRejected("NOT_INITIALIZED");
    }
    if(m0->mode_get() != MODE_READY){
        return FOC_CommandRejected("NOT_READY");
    }

    if(identified != 0U){
        m0->configure_current_feedforward(
            M0_IDENTIFIED_LD, M0_IDENTIFIED_LQ, M0_IDENTIFIED_FLUX);
    }else{
        m0->configure_current_feedforward(
            M0_LEGACY_LD, M0_LEGACY_LQ, M0_LEGACY_FLUX);
    }
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandSetSpeedId(float id)
{
    if((m0 == NULL) || (foc_host_speed_control == false)
       || (m0->mode_get() != MODE_SPEED_TORQUE_RUN)
       || (m0->field_weakening_mode_get() != FIELD_WEAKENING_MODE_OFF)
       || !isfinite(id) || (fabsf(id) > M0_MODEL_TEST_ID_LIMIT)){
        return 0U;
    }

    m0->set_speed_id_reference(id);
    return 1U;
}

extern "C" uint8_t FOC_CommandSetFieldWeakeningMode(uint8_t mode)
{
    if(m0 == NULL){
        return FOC_CommandRejected("NOT_INITIALIZED");
    }
    if(m0->mode_get() != MODE_READY){
        return FOC_CommandRejected("NOT_READY");
    }
    if(mode > (uint8_t)FIELD_WEAKENING_MODE_HYBRID){
        return FOC_CommandRejected("OUT_OF_RANGE");
    }

    m0->set_field_weakening_mode((FIELD_WEAKENING_MODE_t)mode);
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandSetVoltageLimit(float ratio)
{
    const MOTOR_MODE_t mode = (m0 != NULL) ? m0->mode_get() : MODE_IDLE;

    if(m0 == NULL){
        return FOC_CommandRejected("NOT_INITIALIZED");
    }
    if((mode != MODE_READY)
    && ((mode != MODE_SPEED_TORQUE_RUN) || (foc_host_speed_control == false))){
        return FOC_CommandRejected("WRONG_OPERATION");
    }
    if(!isfinite(ratio) || (ratio <= 0.0f) || (ratio > 1.0f)){
        return FOC_CommandRejected("OUT_OF_RANGE");
    }

    m0->set_voltage_limit_ratio(ratio);
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandPrintStatus(void)
{
    if(m0 == NULL){
        return 0U;
    }

    FOC_LIFECYCLE_t lifecycle;
    FOC_OPERATION_t operation;
    ROTOR_SOURCE_t source;
    FOC_DIAGNOSTIC_t diagnostic;
    MOTOR_MODE_t mode;
    foc_fault_status_t fault;
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    lifecycle = m0->lifecycle_get();
    operation = m0->operation_get();
    source = m0->rotor_source_get();
    diagnostic = m0->diagnostic_get();
    mode = m0->mode_get();
    fault = m0->fault_status_get();
    __DMB();
    if(interrupt_mask == 0U){
        __enable_irq();
    }
    usb_printf(
        "STATUS,Lifecycle=%s,Operation=%s,Source=%s,Diagnostic=%s,"
        "Mode=%u,FaultActive=0x%08lX,FaultLatched=0x%08lX\r\n",
        FOC_LifecycleName(lifecycle),
        FOC_OperationName(operation),
        FOC_RotorSourceName(source),
        FOC_DiagnosticName(diagnostic),
        (unsigned)mode,
        (unsigned long)fault.active_bits,
        (unsigned long)fault.latched_bits
    );
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandPrintFault(void)
{
    if(m0 == NULL){
        return 0U;
    }

    foc_fault_status_t fault;
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    fault = m0->fault_status_get();
    __DMB();
    if(interrupt_mask == 0U){
        __enable_irq();
    }
    const foc_fault_snapshot_t &sample = fault.first_snapshot;
    usb_printf(
        "FAULT,Active=0x%08lX,Latched=0x%08lX,First=%s,TimeMs=%lu,"
        "Lifecycle=%lu,Operation=%lu,Source=%lu,Iq=%.4f,Id=%.4f,"
        "Speed=%.4f,Target=%.4f,Vbus=%.3f,Scale=%.5f\r\n",
        (unsigned long)fault.active_bits,
        (unsigned long)fault.latched_bits,
        FOC_FaultName(fault.first_fault),
        (unsigned long)sample.timestamp_ms,
        (unsigned long)sample.lifecycle,
        (unsigned long)sample.operation,
        (unsigned long)sample.rotor_source,
        sample.iq, sample.id, sample.speed, sample.target_speed,
        sample.vbus, sample.voltage_scale
    );
    return FOC_CommandAccepted();
}

extern "C" uint8_t FOC_CommandFaultClear(void)
{
    if(m0 == NULL){
        return 0U;
    }

    foc_fault_status_t fault;
    FOC_LIFECYCLE_t lifecycle;
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    const bool cleared = m0->request_fault_clear();
    fault = m0->fault_status_get();
    lifecycle = m0->lifecycle_get();
    __DMB();
    if(interrupt_mask == 0U){
        __enable_irq();
    }

    if(cleared){
        usb_printf("FAULT_CLEAR,OK\r\n");
        return 1U;
    }

    usb_printf("FAULT_CLEAR,REJECT,Active=0x%08lX,Lifecycle=%s\r\n",
               (unsigned long)fault.active_bits,
               FOC_LifecycleName(lifecycle));
    return 0U;
}

extern "C" void FOC_ServiceFaultReport(void)
{
    static uint32_t reported_trip_bits = 0U;

    if(m0 == NULL){
        return;
    }
    foc_fault_status_t fault;
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    fault = m0->fault_status_get();
    __DMB();
    if(interrupt_mask == 0U){
        __enable_irq();
    }
    const uint32_t trip_bits = fault.latched_bits & FOC_FAULT_TRIP_MASK;
    const uint32_t new_bits = trip_bits & ~reported_trip_bits;
    if(new_bits != 0U){
        const foc_fault_snapshot_t &sample = fault.first_snapshot;
        if((new_bits & FOC_FAULT_SOFTWARE_OVERCURRENT) != 0U){
            usb_printf("FAULT_OVERCURRENT,Iq=%.4f,Id=%.4f\r\n",
                       sample.iq, sample.id);
        }
        usb_printf(
            "FAULT_EVENT,New=0x%08lX,First=%s,TimeMs=%lu,"
            "Iq=%.4f,Id=%.4f,Speed=%.4f,Vbus=%.3f\r\n",
            (unsigned long)new_bits, FOC_FaultName(fault.first_fault),
            (unsigned long)sample.timestamp_ms, sample.iq, sample.id,
            sample.speed, sample.vbus
        );
    }
    reported_trip_bits = trip_bits;
}

extern "C" uint8_t FOC_CommandStop(void)
{
    uint8_t stopped;

    if(m0 == NULL){
        return FOC_CommandRejected("NOT_INITIALIZED");
    }

    foc_speed_command_rpm = 0.0f;
    foc_speed_sine_active = false;
    foc_speed_sine_amplitude_rpm = 0.0f;
    foc_speed_sine_bias_rpm = 0.0f;
    foc_current_sine_active = false;
    foc_current_sine_amplitude = 0.0f;
    m0->set_speed_id_reference(0.0f);
    m0->set_current_reference(0.0f, 0.0f);
    stopped = m0->request_stop() ? 1U : 0U;
    foc_host_speed_control = false;
    foc_host_current_control = false;
    foc_host_position_control = false;
    foc_position_command_initialized = false;
    foc_position_origin_rad = 0.0f;
    foc_position_command_rad = 0.0f;
    if(stopped != 0U){
        return FOC_CommandAccepted();
    }
    const FOC_LIFECYCLE_t lifecycle = m0->lifecycle_get();
    return FOC_CommandRejected(
        ((lifecycle == FOC_LIFECYCLE_FAULT_ACTIVE)
      || (lifecycle == FOC_LIFECYCLE_FAULT_LATCHED))
            ? "ACTIVE_FAULT" : "INVALID_STATE"
    );
}

void FOC_POS_CTRL(void)
{
    float target_position;

    if(foc_host_position_control){
        if(m0->mode_get() != MODE_POSITION_SPEED_TORQUE_RUN){
            return;
        }
        if(foc_position_command_initialized == false){
            foc_position_origin_rad = m0->output_position();
            foc_position_command_rad = 0.0f;
            foc_position_command_initialized = true;
        }
        target_position = foc_position_origin_rad + foc_position_command_rad;
    }else{
        float pos_rate = (float)adc3_dma->adc_pot/100.0f;
        if(pos_rate > 0.9f) pos_rate = 0.9f;
        if(pos_rate < -0.9f) pos_rate = -0.9f;
        target_position = _2PI*pos_rate;
    }

    m0->PositionCtrl(target_position);
}




#pragma arm section











