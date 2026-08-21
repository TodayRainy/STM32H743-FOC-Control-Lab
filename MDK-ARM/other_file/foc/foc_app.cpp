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

#include <bsp.hpp>


#include <stdio.h>
#include <stdint.h>


#pragma arm section code  = "RAMCODE"




FOC_DEBUG_t foc_debug = {};

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
#define M0_MAX_SPEED                (10000.0f*_2PI/60.0f*M0_POLE_PAIRS)        /*要输出电角速度*/           
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
      
/*3505 kv650*/
#define M0_POLE_PAIRS               10                                          /*极对数*/
#define M0_GEAR_RATIO               1                                           /*减速比*/
#define M0_MAX_SPEED                (5000.0f*_2PI/60.0f*M0_POLE_PAIRS)          /*最大电角速度*/           
#define M0_VBUS                     24.0f                                       /*母线电压*/
#define M0_DEFAULT_UQ               0.5f                                        /*默认Uq*/
#define M0_DEFAULT_UD               0.0f                                        /*默认Ud*/
/*辨识相关*/
#define M0_IDEN_IMAX                10.8f                                        /*辨识电流*/
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
#define M0_IQ_KP                    (Ls*_2PI*2000.0f)            /*有感参数*/
#define M0_IQ_KI                    (Rs/_2PI*2000.0f)
#define M0_ID_KP                    (M0_IQ_KP)
#define M0_ID_KI                    (M0_IQ_KI)

#define M0_IQD_ITERM_LIMIT          50000000000.0f

#define M0_IQD_OUTLIMIT             15.0f                /*qd轴电流环 输出值限制*/
#define M0_HFI_IQD_OUTLIMIT         1.5f                /*qd轴电流环 输出值限制-高频状态  该电机Uq 0.7V 空载大概2500r/min*/
#define M0_IQD_REF_LIMIT            7.0f

/*速度环PID*/
/*40k*/
// #define M0_S_KP                     (0.015764175f)
// #define M0_S_KI                     (0.246037498f)
#define M0_S_KP                     (0.015764175f/10.0f)
#define M0_S_KI                     (0.246037498f/10.0f)
#define M0_S_ITERM_LIMIT            10000000.0f  
#define M0_S_REF_LIMIT              M0_MAX_SPEED
#define M0_S_OUT_LIMIT              M0_IQD_REF_LIMIT

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
#define M0_POS_KP                       (3125.0f)
#define M0_POS_KI                       0
#define M0_POS_KD                       25.0f
#define M0_POS_ITERM_LIMIT              10000000.0f  
#define M0_POS_REF_LIMIT                100.0f
#define M0_POS_OUT_LIMIT                M0_S_REF_LIMIT

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

    printf("\r\n默认Uq:%.2fV, Ud:%.2fV, 周期时间:%.3f毫秒, 周期计数值:%d\r\n", M0_DEFAULT_UQ,  M0_DEFAULT_UD, Ts*1000, Tpwm);

    /*创建电机实例*/
    m0 = new foc_motor(Uqd, M0_MAX_SPEED, M0_VBUS, Ts, MODE_IDLE_START, SVPWM_MODE, ANGLE_GEN_ABZENC);
    
    m0->vf      = new foc_vf(M0_VF_DEFAULT_K, M0_VF_FREQ_HIGH, M0_VF_FREQ_LOW, M0_POLE_PAIRS, M0_GEAR_RATIO);
    m0->absenc  = new foc_absenc(M0_POLE_PAIRS, M0_GEAR_RATIO, M0_ABSENC_PHASE_OFFSET, M0_ABSENC_INSTALL_DIR);
    m0->abzenc  = new foc_abzenc(M0_POLE_PAIRS, M0_GEAR_RATIO, M0_ABZENC_PHASE_OFFSET, M0_ABZENC_INSTALL_DIR);
    m0->smo     = new foc_smo(Rs, Lq, Ld, M0_SMO_K, M0_POLE_PAIRS, M0_GEAR_RATIO, 0);
    m0->flux    = new foc_flux(Rs, Ls, Lq, Ld, FLUX, 9000000, M0_POLE_PAIRS, M0_GEAR_RATIO, 0);
    m0->hfi     = new foc_hfi(M0_HFI_VOLT_REF, M0_HFI_VOLT_BIAS, M0_HFI_DIV, M0_POLE_PAIRS, M0_GEAR_RATIO, 0);

    m0->current         = new foc_current(4096.0f/2.0f, M0_IQD_REF_LIMIT*1.5f);
    m0->pid_iq          = new foc_pid(M0_IQ_KP, M0_IQ_KI, 0, M0_IQD_REF_LIMIT, M0_IQD_ITERM_LIMIT, M0_IQD_OUTLIMIT);
    m0->pid_id          = new foc_pid(M0_ID_KP, M0_ID_KI, 0, M0_IQD_REF_LIMIT, M0_IQD_ITERM_LIMIT, M0_IQD_OUTLIMIT);
    m0->spd_ctrl        = new foc_spd_ctrl(M0_S_KP, M0_S_KI, M0_S_REF_LIMIT, M0_S_ITERM_LIMIT, M0_S_OUT_LIMIT);
    m0->pos_ctrl        = new foc_pos_ctrl(M0_POS_KP, M0_POS_KI, M0_POS_KD, M0_POS_REF_LIMIT, M0_POS_ITERM_LIMIT, M0_POS_OUT_LIMIT, 
                                TC_POS_CONTINUOUS, TC_ROTATE_SHORTEST_PATH);

    m0->time            = new foc_time();
    m0->offline_iden    = new foc_offline_iden(M0_IDEN_IMAX, M0_IDEN_SPEED_L, M0_IDEN_SPEED_H, Rs, Lq, Ld);
    
    
    /*PWM模块要最后构建，因为构建pwm会启动FOC状态机运行，需要确保其他模块已经创建好*/
    m0->pwm             = new foc_pwm(Tpwm);
}

void FOC_RUN(void)
{
    /*更新电流*/
    FOC_CURRENT_REFRESH(m0->current->mcu_ad, m0->current->ad_offset, m0->current->Iabc);
    m0->run();
    foc_debug = m0->debug;
}

void FOC_SPEED_CALC(void)
{
    m0->SpeedCalc();   
}

void FOC_SPEED_CTRL(void)
{
    float speed_rate = 0;
#if 0
    int32_t adc_pot = adc3_dma->adc_pot-2000;
    speed_rate = (float)adc_pot/4000.0f;
#else
    int32_t adc_pot = adc3_dma->adc_pot;
    speed_rate = (float)adc_pot/100.0f;
#endif

    m0->SpeedCtrl(speed_rate);
}

void FOC_POS_CTRL(void)
{
    float pos_rate = 0;

    int32_t adc_pot = adc3_dma->adc_pot;
    pos_rate = (float)adc_pot/100.0f;
    
    m0->PossitionCtrl(pos_rate);
}

extern "C" void time_clear(void)
{
    m0->time->clear();
}












#pragma arm section











