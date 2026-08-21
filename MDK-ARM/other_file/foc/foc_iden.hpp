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
#include <stdio.h>

typedef enum
{
    id_Rs_init,
    id_Rs_volt,
    id_Rs_wait,
    id_Rs_samp,
    id_Rs_calc,
    id_Rs_end,
} id_Rs_state_e;

const float I_ref[4] = {-2.5f, -5, 2.5f, 5};


#define RS_IDEN_BUF_SIZE            10

class foc_offline_iden
{
private:
    /* data */
    uint8_t all_state;
    uint8_t state;                                  /*状态机标志*/
    uint32_t cnt;                                   /*计数器*/
    

    /*电阻辨识变量*/
    uint8_t step;                                   /*阶段*/
    float Vs_step;                                  /*电压步态*/
    float Is_steps_ref[RS_IDEN_BUF_SIZE];           /*电流阶梯参考值*/
    float Vs_steps_ref[RS_IDEN_BUF_SIZE];           /*电压阶梯参考值*/
    float Vs_steps[RS_IDEN_BUF_SIZE];               /*电压阶梯步态*/                 
    float Is_steps[RS_IDEN_BUF_SIZE];               /*电流阶梯值*/
   
    float Vs_avg;                                   /*定子电压平均值*/
    float Is_avg;                                   /*定子电流平均值*/
    float Is_err_thres;                             /*电流误差阈值*/
    float Is;
    float Is_err;
    
    /*电感辨识*/
    uint32_t prescale_cnt;                           /*分频计数*/
    
    qd_t Uqd_sum;
    qd_t Iqd_sum_p;
    qd_t Iqd_sum_n;
    qd_t Iqd_p_cnt;
    qd_t Iqd_n_cnt;
    uint32_t Iqd_cnt;
    float I_start;
    float I_end;
    float I_sum;
    float V_pulse;                                  /*脉冲电压*/
    

    /*磁链辨识*/
    qd_t Uqd_avg_l;                                 /*用于计算平均值*/
    qd_t Uqd_avg_h;
    qd_t Iqd_avg_l;
    qd_t Iqd_avg_h;

    qd_t Uqd_l;                                     /*低速电压*/
    qd_t Uqd_h;                                     /*高速电压*/
    qd_t Iqd_l;                                     /*低速电流*/
    qd_t Iqd_h;                                     /*高速电流*/

    float Vs_l;
    float Vs_h;
    float Is_l;
    float Is_h;

    float Vs_diff;
    float Is_diff;
    float Uq_dif;
    qd_t Iqd_dif;

    /*机械速度，目标值，设定值，加速度值*/
    
    float elec_speed_ref;
    float elec_speed_set;
    float elec_speed_acc;
public:
    float Is_max;
    float Vs_max;  
    float_t Rs;
    float Lq;
    float Ld;
    float Ls;
    float Lqd_dif;

    float elec_speed_h;
    float elec_speed_l;
    float flux;
public:
    foc_offline_iden(float Is_max, float elec_speed_l, float elec_speed_h,
                    float Rs, float Lq, float Ld);
    ~foc_offline_iden();

    /*电阻辨识 用额定电流来辨识*/
    bool RsIden(uint8_t Is_steps, qd_t &Uqd, qd_t Iqd, uint32_t samps, float Ts);

    bool LsIden(qd_t &Uqd, qd_t Iqd, uint8_t prescale, uint32_t samps, float Ts);
    
    bool FsIden(qd_t &Uqd, qd_t Iqd, float &elec_speed, float elec_speed_acc, float Ts);

    speed_t speed;
};




































#endif
