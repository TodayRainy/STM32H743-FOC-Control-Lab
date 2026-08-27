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

#if 0
/*SPEED电机*/
#define Rs          (55.17068f/1000.0f)        
#define Lq          (9.79341f/1000000.0f)      
#define Ld          (7.17162f/1000000.0f)       
#define Ls          (8.48252f/1000000.0f)       
#define FLUX        (0.47582f /1000.0f)          
#define KV          (1700.0f)                   /* KV值*/



/*酷飞电机 1960 参数*/
// #define Rs          (53.49573f/1000.0f)         /* 70.24f mΩ */
// #define Lq          (10.18956f/1000000.0f)      /* 4.87f uH */
// #define Ld          (6.93781f/1000000.0f)       /* 4.87f uH */
// #define Ls          (8.56369f/1000000.0f)       /* 4.87f uH */
// #define FLUX        (0.41988f /1000.0f)          /* 磁通: 0.613f mWb */
// #define KV          (1700.0f)                   /* KV值*/


/*SPEED 2207 kv1700*/
#define M0_POLE_PAIRS               7
#define M0_GEAR_RATIO               1              /*减速比*/
#define M0_MAX_SPEED_RPM            10000.0f
#define M0_MAX_SPEED                (M0_MAX_SPEED_RPM*_2PI/60.0f*M0_POLE_PAIRS) /*要输出电角速度*/
#define M0_VBUS                     24.0f
#define M0_DEFAULT_UQ               0.5f
#define M0_DEFAULT_UD               0.0f
/*辨识相关*/
#define M0_IDEN_IMAX                12.8f
#define M0_IDEN_SPEED_L             1000.0f
#define M0_IDEN_SPEED_H             3000.0f

/*VF*/
#define M0_VF_DEFAULT_K             (60.0f/KV/M0_POLE_PAIRS*_2PI)
// #define M0_VF_DEFAULT_K             0.0570459999f/2
#define M0_VF_FREQ_HIGH             (35.0f)
#define M0_VF_FREQ_LOW              (10.0f)
/*ABZ编码器*/
#define M0_ABZENC_PHASE_OFFSET      -2.271589f
#define M0_ABZENC_INSTALL_DIR       1               /*编码器安装方向*/
/*ABS编码器*/
#define M0_ABSENC_PHASE_OFFSET      0.568693161f
#define M0_ABSENC_INSTALL_DIR       -1              /*编码器安装方向*/
/*滑模观测器*/
#define M0_SMO_K                    0.5f
/*高频注入*/
#define M0_HFI_VOLT_REF             1.0f
#define M0_HFI_VOLT_BIAS            0.5f
#define M0_HFI_DIV                  1               /*分频系数 高频注入频率 = 基频/M0_HFI_DIV/2*/ 

/*电流环PID*/
// #define M0_IQ_KP                    (Ls*_2PI*1000.0f)            /*有感参数*/
// #define M0_IQ_KI                    (Rs/_2PI*1000.0f)
// #define M0_ID_KP                    (M0_IQ_KP*2)
// #define M0_ID_KI                    (M0_IQ_KI*2)

/*40K*/
#define M0_IQ_KP                    (Ls*_2PI*2000.0f)     
#define M0_IQ_KI                    (Rs/_2PI*2000.0f)
#define M0_ID_KP                    (M0_IQ_KP)
#define M0_ID_KI                    (M0_IQ_KI)

#define M0_IQD_ITERM_LIMIT          50000000000.0f

#define M0_IQD_OUTLIMIT             10.0f               /*qd轴电流环 输出值限制*/
#define M0_HFI_IQD_OUTLIMIT         1.5f                /*qd轴电流环 输出值限制-高频状态  该电机Uq 0.7V 空载大概2500r/min*/
#define M0_IQD_REF_LIMIT            10.0f

/*速度环PID*/
#define M0_S_KP                     (0.0112499995f)
#define M0_S_KI                     (0.303750038f)

#define M0_S_ITERM_LIMIT            10000000.0f  
#define M0_S_REF_LIMIT              M0_MAX_SPEED
#define M0_S_OUT_LIMIT              M0_IQD_REF_LIMIT

/*位置环PID */
#if 1   
/*位置环-力矩环*/
#define M0_POS_KP                     (50.0f)
#define M0_POS_KI                     (M0_POS_KP*0.5f)
#define M0_POS_KD                     100.0f
#define M0_POS_ITERM_LIMIT            10000000.0f  
#define M0_POS_REF_LIMIT              100.0f
#define M0_POS_OUT_LIMIT              M0_S_REF_LIMIT
#else
/*位置环-速度环-力矩环*/
#define M0_POS_KP                     (500.0f)
#define M0_POS_KI                     (M0_POS_KP*0.5f)
#define M0_POS_ITERM_LIMIT            10000000.0f  
#define M0_POS_REF_LIMIT              100.0f
#define M0_POS_OUT_LIMIT              M0_S_REF_LIMIT

#endif

#endif


#if 1
/*ProDrone 3505电机 kv650*/
#define Rs          (101.02932f/1000.0f)         
#define Lq          (53.17701f/1000000.0f)       
#define Ld          (34.98624f/1000000.0f)       
#define Ls          (44.08162f/1000000.0f)       
#define FLUX        (0.90451f/1000.0f)         
#define KV          (650.0f)

/* Stage-1 candidate model. It feeds optional dq feedforward and seeds a
 * standalone flux identification; legacy PI and observer parameters stay fixed. */
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
/*辨识相关*/
#define M0_IDEN_IMAX                3.0f                                         /*阶段1 RL辨识点: ±1/±2/±3 A*/
#define M0_IDEN_SPEED_L             1000.0f                                     /*磁链辨识低速*/
#define M0_IDEN_SPEED_H             3000.0f                                     /*磁链辨识高速*/

/*VF*/
#define M0_VF_DEFAULT_K             (60.0f/KV/M0_POLE_PAIRS/2*_2PI)
#define M0_VF_FREQ_HIGH             (25.0f)
#define M0_VF_FREQ_LOW              (10.0f)
/*ABZ编码器*/
#define M0_ABZENC_PHASE_OFFSET      -2.232439f
#define M0_ABZENC_INSTALL_DIR       1            /*编码器安装方向*/
/*ABS编码器*/
#define M0_ABSENC_PHASE_OFFSET      0.046832f
#define M0_ABSENC_INSTALL_DIR       -1            /*编码器安装方向*/
/*滑模观测器*/
#define M0_SMO_K                    0.5f
/*高频注入*/
#define M0_HFI_VOLT_REF             1.0f
#define M0_HFI_VOLT_BIAS            0.5f
#define M0_HFI_DIV                  2               /*分频系数 高频注入频率 = 基频/M0_HFI_DIV/2*/ 

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
#define M0_HFI_IQD_OUTLIMIT         1.5f                /*qd轴电流环 输出值限制-高频状态  该电机Uq 0.7V 空载大概2500r/min*/
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

/*位置环PID 这个电机目前直接输出Uq的位置环效果是最好的，但是要注意电源限流，防止烧坏了电机，响应比较快，但无法做ST曲线，也无法控制电流*/

/*位置环-电压开环*/
// #define M0_POS_KP                     (2.0f)
// #define M0_POS_KI                       10
// #define M0_POS_ITERM_LIMIT            10000000.0f  
// #define M0_POS_REF_LIMIT              100.0f
// #define M0_POS_OUT_LIMIT              5.0f

// /*位置环-力矩环*/
// #define M0_POS_KP                     (5.0f)
// // #define M0_POS_KI                     (M0_POS_KP*0.5f)
// #define M0_POS_KI                     0
// #define M0_POS_KD                     0.2f
// #define M0_POS_ITERM_LIMIT            10000000.0f  
// #define M0_POS_REF_LIMIT              100.0f
// #define M0_POS_OUT_LIMIT              M0_S_REF_LIMIT

/*位置环-速度环-力矩环 用位置环的时候最好把电流环的最大Iqref限制的小一点*/
// #undef M0_IQD_REF_LIMIT
// #define M0_IQD_REF_LIMIT                15.0f
/* D-053/D-054 no-load position default; TRAP 180/900 with velocity
 * feedforward remains an optional faster trajectory. */
#define M0_POS_KP                       (376.991118f)
#define M0_POS_KI                       0
#define M0_POS_KD                       0.0f
#define M0_POS_ITERM_LIMIT              10000000.0f  
#define M0_POS_REF_LIMIT                100.0f
#define M0_POS_OUT_LIMIT                M0_S_REF_LIMIT
#define M0_POS_DEFAULT_RATE_DEG_S       180.0f

#endif


/*要注意状态机在foc_motor构建过程中就启动了, 所有有关FOC的数据结构都在m0之中*/
foc_motor *m0 = NULL;


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

    printf("\r\n默认Uq:%.2fV, Ud:%.2fV, 周期时间:%.3f毫秒, 周期计数值:%d\r\n", M0_DEFAULT_UQ,  M0_DEFAULT_UD, Ts*1000, Tpwm);

    /*创建电机实例*/
    m0 = new foc_motor(Uqd, M0_MAX_SPEED, M0_VBUS, Ts, MODE_IDLE, SVPWM_MODE, ANGLE_GEN_ABZENC);
    
    m0->vf      = new foc_vf(M0_VF_DEFAULT_K, M0_VF_FREQ_HIGH, M0_VF_FREQ_LOW, M0_POLE_PAIRS, M0_GEAR_RATIO);
    m0->absenc  = new foc_absenc(M0_POLE_PAIRS, M0_GEAR_RATIO, M0_ABSENC_PHASE_OFFSET, M0_ABSENC_INSTALL_DIR);
    m0->abzenc  = new foc_abzenc(M0_POLE_PAIRS, M0_GEAR_RATIO, M0_ABZENC_PHASE_OFFSET, M0_ABZENC_INSTALL_DIR);
    m0->abzenc->spd->EstimatorModeSet(FOC_SPEED_ESTIMATOR_WIN_IIR);
    m0->smo     = new foc_smo(Rs, Lq, Ld, M0_SMO_K, M0_POLE_PAIRS, M0_GEAR_RATIO, 0);
    m0->flux    = new foc_flux(Rs, Ls, Lq, Ld, FLUX, 9000000, M0_POLE_PAIRS, M0_GEAR_RATIO, 0);
    m0->hfi     = new foc_hfi(M0_HFI_VOLT_REF, M0_HFI_VOLT_BIAS, M0_HFI_DIV, M0_POLE_PAIRS, M0_GEAR_RATIO, 0);

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
    const float measured_vbus = adc3_dma->vbus;
    if(measured_vbus > 1.0f){
        m0->set_bus_voltage(measured_vbus);
    }

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

    ++foc_debug_seq;
    __DMB();
    foc_debug = m0->debug;
    foc_debug.vbus = measured_vbus;
    __DMB();
    ++foc_debug_seq;
    Debug_ScopeSample(&foc_debug);
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
        return 0U;
    }

    foc_speed_command_rpm = 0.0f;
    foc_speed_sine_active = false;
    foc_speed_sine_amplitude_rpm = 0.0f;
    foc_speed_sine_bias_rpm = 0.0f;
    if(m0->request_start(MODE_SPEED_TORQUE_RUN) == false){
        return 0U;
    }

    foc_host_speed_control = true;
    foc_host_current_control = false;
    foc_host_position_control = false;
    return 1U;
}

extern "C" uint8_t FOC_CommandStartCurrent(void)
{
    if(m0 == NULL){
        return 0U;
    }

    foc_current_sine_active = false;
    foc_speed_sine_active = false;
    foc_speed_sine_amplitude_rpm = 0.0f;
    foc_speed_sine_bias_rpm = 0.0f;
    m0->set_current_reference(0.0f, 0.0f);
    if(m0->request_start(MODE_TORQUE_RUN) == false){
        return 0U;
    }

    foc_host_speed_control = false;
    foc_host_current_control = true;
    foc_host_position_control = false;
    return 1U;
}

extern "C" uint8_t FOC_CommandStartPosition(void)
{
    if(m0 == NULL){
        return 0U;
    }

    if(m0->request_start(MODE_POSITION_SPEED_TORQUE_RUN) == false){
        return 0U;
    }

    /* Position tracking owns the cascade reference trajectory. */
    m0->spd_ctrl->set_track(TRACK_MODE_NONE, 0.0f);
    foc_speed_sine_active = false;
    foc_speed_sine_amplitude_rpm = 0.0f;
    foc_speed_sine_bias_rpm = 0.0f;

    foc_position_origin_rad = 0.0f;
    foc_position_command_rad = 0.0f;
    foc_position_command_initialized = false;
    foc_host_speed_control = false;
    foc_host_current_control = false;
    foc_host_position_control = true;
    return 1U;
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
    if((m0 == NULL) || (m0->offline_iden == NULL)
       || (m0->mode_get() != MODE_READY)){
        return 0U;
    }

    foc_iden_result_t result = {};
    m0->offline_iden->ResultGet(result);
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
    if((m0 == NULL) || (m0->offline_iden == NULL)
       || (m0->mode_get() != MODE_READY)){
        return 0U;
    }

    foc_iden_result_t result = {};
    m0->offline_iden->ResultGet(result);
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
    if((foc_host_speed_control == false) ||
       (speed_rpm > M0_MAX_SPEED_RPM) ||
       (speed_rpm < -M0_MAX_SPEED_RPM)){
        return 0U;
    }

    foc_speed_sine_active = false;
    foc_speed_sine_amplitude_rpm = 0.0f;
    foc_speed_sine_bias_rpm = 0.0f;
    foc_speed_command_rpm = speed_rpm;
    return 1U;
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
    m0->spd_ctrl->set_track(TRACK_MODE_NONE, 0.0f);
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
    if((m0 == NULL) || (m0->mode_get() != MODE_READY)
       || !isfinite(limit_a) || (limit_a <= 0.0f)
       || (limit_a > M0_IQD_REF_LIMIT)){
        return 0U;
    }

    m0->set_speed_current_limit(limit_a);
    return 1U;
}

extern "C" uint8_t FOC_CommandSetSpeedEstimator(uint8_t mode)
{
    if((m0 == NULL) || (m0->abzenc == NULL)
       || (m0->mode_get() != MODE_READY)
       || (mode > (uint8_t)FOC_SPEED_ESTIMATOR_PLL)){
        return 0U;
    }

    m0->abzenc->spd->EstimatorModeSet((foc_speed_estimator_mode_t)mode);
    return 1U;
}

extern "C" uint8_t FOC_CommandSetCurrent(float iq, float id)
{
    if((m0 == NULL) || (foc_host_current_control == false) ||
       !isfinite(iq) || !isfinite(id) ||
       (hypotf(iq, id) > M0_IQD_REF_LIMIT)){
        return 0U;
    }

    foc_current_sine_active = false;
    m0->set_current_reference(iq, id);
    return 1U;
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

    if((m0 == NULL) || (foc_host_position_control == false) ||
       (foc_position_command_initialized == false) ||
       !isfinite(position_deg) || (fabsf(position_rad) > M0_POS_REF_LIMIT)){
        return 0U;
    }

    foc_position_command_rad = position_rad;
    return 1U;
}

extern "C" uint8_t FOC_CommandSetSpeedPI(float kp, float ki)
{
    if((m0 == NULL) || !isfinite(kp) || !isfinite(ki) || (kp < 0.0f) || (ki < 0.0f)){
        return 0U;
    }

    m0->spd_ctrl->set_gains(kp, ki);
    return 1U;
}

extern "C" uint8_t FOC_CommandSetPositionPI(float kp, float ki, float kd)
{
    if((m0 == NULL) || !isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
       (kp < 0.0f) || (ki < 0.0f) || (kd < 0.0f)){
        return 0U;
    }

    m0->pos_ctrl->set_gains(kp, ki, kd);
    return 1U;
}

extern "C" uint8_t FOC_CommandSetPositionSpeedLimit(float limit_rpm)
{
    if((m0 == NULL) || (m0->mode_get() != MODE_READY) ||
       !isfinite(limit_rpm) || (limit_rpm <= 0.0f) ||
       (limit_rpm > M0_MAX_SPEED_RPM)){
        return 0U;
    }

    m0->pos_ctrl->set_output_limit(
        limit_rpm*_2PI/60.0f*M0_POLE_PAIRS
    );
    return 1U;
}

extern "C" uint8_t FOC_CommandSetIqPI(float kp, float ki)
{
    if((m0 == NULL) || !isfinite(kp) || !isfinite(ki) || (kp < 0.0f) || (ki < 0.0f)){
        return 0U;
    }

    m0->pid_iq->set_gains(kp, ki, 0.0f);
    return 1U;
}

extern "C" uint8_t FOC_CommandSetIdPI(float kp, float ki)
{
    if((m0 == NULL) || !isfinite(kp) || !isfinite(ki) || (kp < 0.0f) || (ki < 0.0f)){
        return 0U;
    }

    m0->pid_id->set_gains(kp, ki, 0.0f);
    return 1U;
}

extern "C" uint8_t FOC_CommandClearPI(void)
{
    if(m0 == NULL){
        return 0U;
    }

    m0->spd_ctrl->clear();
    m0->pos_ctrl->clear();
    m0->pid_iq->clear();
    m0->pid_id->clear();
    m0->set_speed_id_reference(0.0f);
    m0->set_field_weakening_mode(m0->field_weakening_mode_get());
    return 1U;
}

extern "C" uint8_t FOC_CommandSetSpeedStep(void)
{
    if(m0 == NULL){
        return 0U;
    }

    m0->spd_ctrl->set_track(TRACK_MODE_NONE, 0.0f);
    return 1U;
}

extern "C" uint8_t FOC_CommandSetSpeedRamp(float accel_rpm_per_sec)
{
    float accel_elec;

    if((m0 == NULL) || !isfinite(accel_rpm_per_sec) || (accel_rpm_per_sec <= 0.0f)){
        return 0U;
    }

    accel_elec = accel_rpm_per_sec*_2PI/60.0f*M0_POLE_PAIRS;
    m0->spd_ctrl->set_track(TRACK_MODE_RAMP, accel_elec);
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
    if((m0 == NULL) || (mode > (uint8_t)CURRENT_FF_MODE_BEMF)){
        return 0U;
    }

    m0->set_current_feedforward_mode((CURRENT_FEEDFORWARD_MODE_t)mode);
    return 1U;
}

extern "C" uint8_t FOC_CommandSetFeedforwardModel(uint8_t identified)
{
    if((m0 == NULL) || (m0->mode_get() != MODE_READY)){
        return 0U;
    }

    if(identified != 0U){
        m0->configure_current_feedforward(
            M0_IDENTIFIED_LD, M0_IDENTIFIED_LQ, M0_IDENTIFIED_FLUX);
    }else{
        m0->configure_current_feedforward(Ld, Lq, FLUX);
    }
    return 1U;
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
    if((m0 == NULL) || (m0->mode_get() != MODE_READY)
       || (mode > (uint8_t)FIELD_WEAKENING_MODE_HYBRID)){
        return 0U;
    }

    m0->set_field_weakening_mode((FIELD_WEAKENING_MODE_t)mode);
    return 1U;
}

extern "C" uint8_t FOC_CommandSetVoltageLimit(float ratio)
{
    const MOTOR_MODE_t mode = (m0 != NULL) ? m0->mode_get() : MODE_IDLE;

    if((m0 == NULL)
       || ((mode != MODE_READY)
           && ((mode != MODE_SPEED_TORQUE_RUN)
               || (foc_host_speed_control == false)))
       || !isfinite(ratio) || (ratio <= 0.0f) || (ratio > 1.0f)){
        return 0U;
    }

    m0->set_voltage_limit_ratio(ratio);
    return 1U;
}

extern "C" void FOC_ServiceFaultReport(void)
{
    qd_t fault_current = {0.0f, 0.0f};

    if((m0 != NULL) && (m0->current != NULL)
       && m0->current->take_overcurrent_fault(&fault_current)){
        printf("FAULT_OVERCURRENT,Iq=%.4f,Id=%.4f\r\n",
               fault_current.q, fault_current.d);
    }
}

extern "C" uint8_t FOC_CommandStop(void)
{
    uint8_t stopped;

    if(m0 == NULL){
        return 0U;
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
    return stopped;
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

extern "C" void time_clear(void)
{
    m0->time->clear();
}












#pragma arm section











