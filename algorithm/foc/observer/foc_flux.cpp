/**
 ****************************************************************************************************
 * @file        foc_flux.cpp
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


#include "foc_flux.hpp"
#include "foc_math.hpp"
#include <stdio.h>

#pragma arm section code  = "RAMCODE"


/*使用反正切*/
#define ATAN_ENABEL     1




foc_flux::foc_flux(float Rs, float Ls, float Lq, float Ld, float flux, float G,
                uint32_t pole_pairs, uint32_t gear_ratio, float offset)
{
    /*补齐漏初始化的成员。flux_alpha/flux_beta 是磁链观测器的积分器状态, 以 += 累加,
      未清零就直接积分会让预热循环从堆内存残留值起算*/
    this->flux_alpha = 0;
    this->flux_beta  = 0;
    this->err        = 0;
    this->eta_x1     = 0;
    this->eta_x2     = 0;
    this->eta_x1_dt  = 0;
    this->eta_x2_dt  = 0;
    this->Ls_I_alpha = 0;
    this->Ls_I_beta  = 0;

    this->Rs = Rs;
    this->Ls = Ls;
    this->Lq = Lq;
    this->Ld = Ld;
    this->flux = flux;
    this->G = G;
    this->flux_squared = flux*flux;

    /*磁链高速震荡的时候可以调大这两个参数*/

    /*酷飞电机*/
    // float kp = 4*PI  *1.5f *5 /2 *20;
    // float ki = 4*PI*PI   *5*5 /(2*2) *20;
    
    /*ProDrone电机*/                   
    float kp = 4*PI  *1.5f *5 /2 *2;
    float ki = 4*PI*PI   *5*5 /(2*2) *2;

    spd             = new foc_speed(pole_pairs, gear_ratio, 0);
    // pll             = new foc_pll(1000000.0f, 10000.0f);

    pll             = new foc_pll(1000000.0f, 10000.0f);
    flux_alpha_pid  = new foc_pid(kp, ki, 0, 50000000000, 100.0f, 100.0f);
    flux_beta_pid   = new foc_pid(kp, ki, 0, 50000000000, 100.0f, 100.0f);

    alphabeta_t tmp = {0.0f, 0.0f};
    spd->SetAnglePre(0, 0, 0);
    /*多几轮预计算，避免运行计算时速度突然毛刺很大*/
    for (size_t i = 0; i < 10000; i++){
        ElecAngleCalc(tmp, tmp, 1/20000.0f);
    }
    



}

foc_flux::~foc_flux()
{
    delete(pll);
    delete(flux_beta_pid);
    delete(flux_alpha_pid);
    delete(spd);
}

/*非线性磁链*/
inline float foc_flux::AngleCalc_nonelinear(alphabeta_t U_alphabeta, alphabeta_t I_alphabeta, float Ts)
{

    float elec_angle = 0.0f;
    float Rs = this->Rs;                                                    /*定子电阻*/

    float Ls_I_alpha = this->Ls * I_alphabeta.alpha;                        /*alpha电流和定子电感的乘积*/
    float Ls_I_beta = this->Ls * I_alphabeta.beta;                          /*beta电流和定子电感的乘积*/

    float x1 = this->flux_alpha;                                            /**/
    float x2 = this->flux_beta;

    float eta_x1 = x1-Ls_I_alpha;                                           /*观测的定子磁链*/
    float eta_x2 = x2-Ls_I_beta;

    float y1 = -Rs *I_alphabeta.alpha + U_alphabeta.alpha;                  /*(永磁体磁链+电流磁链)的微分*/
    float y2 = -Rs *I_alphabeta.beta + U_alphabeta.beta;

    float err = this->flux_squared - (eta_x1*eta_x1 + eta_x2*eta_x2);       /*误差状态收敛*/
    err = LIMIT_RANGE(err, -this->flux, this->flux);                        /*误差限制幅度*/

    float eta_x1_dt     = y1 + this->G * eta_x1 * err;                      /*(永磁体磁链+电流磁链)的微分+增益*/
    float eta_x2_dt     = y2 + this->G * eta_x2 * err;

    this->flux_alpha    += eta_x1_dt * Ts;                                  /*通过积分计算观测到的α轴的磁链*/
    this->flux_beta     += eta_x2_dt * Ts;                                  /*通过积分计算观测到的β轴的磁链*/

    /*参数保存*/
    this->err = err;
    this->eta_x1 = eta_x1;
    this->eta_x2 = eta_x2;
    this->eta_x1_dt = eta_x1_dt;
    this->eta_x2_dt = eta_x2_dt;
    this->Ls_I_alpha = Ls_I_alpha;
    this->Ls_I_beta = Ls_I_beta;


    float flux_alpha = this->flux_alpha-Ls_I_alpha;                         /*α轴的永磁体磁链*/
    float flux_beta = this->flux_beta-Ls_I_beta;                            /*β轴的永磁体磁链*/
    float dir = spd->speed_data.elec_speed>=0?1.0f:-1.0f;

    /*反正切计算电角度*/
    #if ATAN_ENABEL
    // elec_angle = atan2f(flux_beta, flux_alpha);
    elec_angle = utils_fast_atan2(flux_beta, flux_alpha);
    elec_angle = Limit_Angle(elec_angle + (dir*spd->speed_data.offset *PI));
    spd->AngleRefresh(elec_angle);
    spd->SpeedCalc(Ts);
    #else
    /*PLL求电角度*/
    elec_angle = pll->calc_flux(flux_alpha, flux_beta, Ts);
    elec_angle = Limit_Angle(elec_angle + (dir*spd->speed_data.offset *PI));
    spd->AngleRefresh(elec_angle);
    #endif

    

    return spd->speed_data.elec_angle;
}

/*电流磁链*/
inline void foc_flux::flux_I_cacl(qd_t Iqd)
{
    float elec_angle = spd->speed_data.elec_angle;
    float _cos = arm_cos_f32(elec_angle);
    float _sin = arm_sin_f32(elec_angle);

    qd_t flux_qd = {0};
    alphabeta_t flux_alphabeta = {0};
    flux_qd.d =  this->flux + this->Ld * Iqd.d;              /*d轴的定子磁链*/
    flux_qd.q =  this->Lq * Iqd.q;                           /*q轴的定子磁链*/

    flux_alphabeta.alpha    = flux_qd.d * _cos - flux_qd.q *_sin;
    flux_alphabeta.beta     = flux_qd.d * _sin + flux_qd.q *_cos;

    this->flux_I = flux_alphabeta;
}

/*电压磁链*/
inline void foc_flux::flux_V_cacl(alphabeta_t U_alphabeta, alphabeta_t I_alphabeta, float Ts)
{
    float elec_angle = 0.0f;
    float Rs = this->Rs;                                                    /*定子电阻*/

    float Ls_I_alpha = this->Lq * I_alphabeta.alpha;                        /*alpha电流和定子电感的乘积*/
    float Ls_I_beta = this->Lq * I_alphabeta.beta;                          /*beta电流和定子电感的乘积*/

    float y1 = -Rs *I_alphabeta.alpha + U_alphabeta.alpha;                  /*(永磁体磁链+电流磁链)的微分 也就是反电动势*/
    float y2 = -Rs *I_alphabeta.beta + U_alphabeta.beta;

    /*加上PID计算的增益*/
    y1 -= this->flux_alpha_pid->out;
    y2 -= this->flux_beta_pid->out;


    this->flux_V.alpha  += y1 * Ts;                                         /*通过积分计算观测到的α轴的总磁链*/
    this->flux_V.beta   += y2 * Ts;                                         /*通过积分计算观测到的β轴的总磁链*/

    /*计算PID增益*/
    flux_alpha_pid->pi(this->flux_V.alpha - this->flux_I.alpha, Ts);
    flux_beta_pid->pi(this->flux_V.beta - this->flux_I.beta, Ts);

    float flux_alpha = this->flux_V.alpha-Ls_I_alpha;                         /*α轴的永磁体磁链*/
    float flux_beta = this->flux_V.beta-Ls_I_beta;                            /*β轴的永磁体磁链*/

    float dir = spd->speed_data.elec_speed>=0?1.0f:-1.0f;
    /*反正切计算电角度*/
    #if ATAN_ENABEL
    // elec_angle = atan2f(flux_beta,  flux_alpha);
    elec_angle = utils_fast_atan2(flux_beta, flux_alpha);
    elec_angle = Limit_Angle(elec_angle + (dir*spd->speed_data.offset *PI));
    spd->AngleRefresh(elec_angle);
    spd->SpeedCalc(Ts);
    #else
    /*PLL求电角度*/
    elec_angle = pll->calc_flux(flux_alpha, flux_beta, Ts);
    elec_angle = Limit_Angle(elec_angle + (dir*spd->speed_data.offset *PI));
    spd->AngleRefresh(elec_angle);
    #endif

}

float foc_flux::ElecAngleCalc(alphabeta_t U_alphabeta, alphabeta_t I_alphabeta, float Ts)
{
    #if 1
    /*电压电流互补磁链*/
    float elec_angle = spd->speed_data.elec_angle;
    qd_t Iqd = {0};
    Iqd = Foc_Park(I_alphabeta, elec_angle);

    flux_I_cacl(Iqd);
    flux_V_cacl(U_alphabeta, I_alphabeta, Ts);


    #else 
    /*非线性磁链*/
    AngleCalc_nonelinear(U_alphabeta, I_alphabeta, Ts);
    #endif

    return spd->speed_data.elec_angle;
}

float foc_flux::ElecSpeedCalc(void)
{
#if !ATAN_ENABEL
    spd->SpeedRefresh(pll->movefilter->calc());
#endif
    return spd->speed_data.elec_speed;
}


void foc_flux::clear(void)
{
    spd->clear();
    pll->clear();
    flux_alpha_pid->clear();
    flux_beta_pid->clear();
}



















// #pragma arm section code  = "RAMCODE"
#pragma arm section






