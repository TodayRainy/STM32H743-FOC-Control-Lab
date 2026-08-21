/**
 ****************************************************************************************************
 * @file        foc_smo.cpp
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

#include "foc_smo.hpp"
#include "foc_math.hpp"


#pragma arm section code  = "RAMCODE"

/*使用反正切*/
#define ATAN_ENABEL     0

/**
 * @brief 饱和函数
 * 
 * @param s 
 * @param delta 
 * @return float 
 */
static float Sat(float s, float delta)
{
    if(s > delta)
        return 1;
    else if(s < -delta)
        return -1;
    else
        return s/delta;
}


foc_smo::foc_smo(float Rs, float Lq, float Ld, float k, uint32_t pole_pairs, uint32_t gear_ratio, float offset)
{
    this->Rs    = Rs;
    this->Lq    = Lq;
    this->Ld    = Ld;
    this->Ls    = (Lq+Ld)/2.0f;
    this->k     = k;
    this->lqd_dif = Ld-Lq;

    pll         = new foc_pll(2700.0f, 364975.0f);
    spd       = new foc_speed(pole_pairs, gear_ratio, offset);

    alphabeta_t tmp = {0.0f, 0.0f};
    spd->SetAnglePre(0, 0, 0);
    /*多做几轮预计算，避免运行计算时速度突然毛刺很大*/
    for (size_t i = 0; i < 10000; i++){
        ElecAngleCalc(tmp, tmp, 1/20000.0f);
    }
}

foc_smo::~foc_smo()
{
    delete(pll);
    delete(spd);
}

/****
 * 滑模公式推导
 * 
 */

 /*滑模计算电角度*/
float foc_smo::ElecAngleCalc(alphabeta_t U_alphabeta, alphabeta_t I_alphabeta, float Ts)
{
    float elec_angle = 0;

    int8_t dir = spd->speed_data.elec_speed>=0?1:-1;

    float ws = spd->speed_data.elec_speed;

    /* 估算的电流微分进行积分 */
    est_I.alpha += (U_alphabeta.alpha - est_I.alpha * Rs - (lqd_dif*ws) - est_E.alpha)/Ls *Ts;
    est_I.beta  += (U_alphabeta.beta - est_I.beta * Rs + (lqd_dif*ws) - est_E.beta)/Ls *Ts;
    // est_I.alpha += (U_alphabeta.alpha - est_I.alpha * Rs - est_E.alpha)/Ls *Ts;
    // est_I.beta  += (U_alphabeta.beta - est_I.beta * Rs - est_E.beta)/Ls *Ts;

    /*计算电流误差*/
    I_err.alpha = est_I.alpha - I_alphabeta.alpha;
    I_err.beta = est_I.beta - I_alphabeta.beta;

    /*修正估算的反电动势*/
    est_E.alpha = k * Sat(I_err.alpha, 0.5f);
    est_E.beta  = k * Sat(I_err.beta, 0.5f); 


    /*反正切计算电角度*/
    #if ATAN_ENABEL
    // elec_angle = atan2f(flux_beta, flux_alpha);
    elec_angle = utils_fast_atan2(est_E.beta, est_E.alpha);
    elec_angle = Limit_Angle(elec_angle + (dir*spd->speed_data.offset *PI));
    spd->AngleRefresh(elec_angle);
    spd->SpeedCalc(Ts);
    #else
    /*PLL求电角度*/
    elec_angle = pll->calc_smo(dir*est_E.alpha, dir*est_E.beta, Ts);
    elec_angle = Limit_Angle(elec_angle + (dir*spd->speed_data.offset *PI));
    spd->AngleRefresh(elec_angle);
    #endif



    return spd->speed_data.elec_angle;
}

/*电角速度计算*/
float foc_smo::ElecSpeedCalc(void)
{
    #if !ATAN_ENABEL
    spd->SpeedRefresh(pll->movefilter->calc());
#endif

    return spd->speed_data.elec_speed;

}

void foc_smo::clear(void)
{
    pll->clear();
    spd->clear();
}

#pragma arm section

