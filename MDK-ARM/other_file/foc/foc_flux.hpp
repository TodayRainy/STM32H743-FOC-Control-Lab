/**
 ****************************************************************************************************
 * @file        foc_flux.hpp
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

#ifndef _FOC_FLUX_HPP
#define _FOC_FLUX_HPP


#include "foc_type.h"
#include "foc_filter.hpp"
#include "foc_pid.hpp"
#include "foc_speed.hpp"



class foc_flux
{

private:
    foc_pll *pll;

    /*有效磁链的参数*/
    alphabeta_t flux_I;                         /* 电流型有效磁链*/
    alphabeta_t flux_V;                         /* 电压型有效磁链*/         
    foc_pid *flux_alpha_pid;
    foc_pid *flux_beta_pid;

    float flux;                                 /* 磁链大小*/
    float flux_squared;                         /* 磁链的平方*/
    float G;                                    /* 增益*/
    float Rs;                                   /* 相电阻 */
    float Ls;                                   /* 相电感 */
    float Lq;
    float Ld;
    
    float flux_alpha;                           /* 通过积分计算观测到的α轴的磁链*/
    float flux_beta;                            /* 通过积分计算观测到的β轴的磁链*/


    float err, eta_x1, eta_x2, eta_x1_dt, eta_x2_dt, Ls_I_alpha, Ls_I_beta;         /*用于调试的中间变量*/ 
public:
    foc_speed *spd;
private:
    /* data */
    inline void flux_I_cacl(qd_t Iqd);
    inline void flux_V_cacl(alphabeta_t U_alphabeta, alphabeta_t I_alphabeta, float Ts);
    inline float AngleCalc_nonelinear(alphabeta_t U_alphabeta, alphabeta_t I_alphabeta, float Ts);
public:
    foc_flux(float Rs, float Ls, float Lq, float Ld, float flux, float G,
                uint32_t pole_pairs, uint32_t gear_ratio, float offset);
    ~foc_flux();
    float ElecAngleCalc(alphabeta_t U_alphabeta, alphabeta_t I_alphabeta, float Ts);
    float ElecSpeedCalc(void);

    void clear(void);

};





















#endif
