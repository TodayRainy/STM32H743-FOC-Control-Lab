/**
 ****************************************************************************************************
 * @file        foc_smo.hpp
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

#ifndef _FOC_SMO_HPP
#define _FOC_SMO_HPP




#include "foc_type.h"
#include "foc_filter.hpp"
#include "foc_speed.hpp"

class foc_smo
{
private:
    /* data */
    float k;                                    /*滑模系数*/
    float Ls;                                   /*相电感*/
    float Rs;                                   /*定子电阻，也就是相电阻*/

    float Lq;
    float Ld;
    float lqd_dif;

    foc_pll *pll;

    alphabeta_t I_err;                          /*估算电流和实际电流的误差*/
    alphabeta_t est_I;                          /*估算的电流*/
    alphabeta_t est_E;                          /*估算的反电动势*/

public:
    foc_speed *spd;
public:
    foc_smo(float Rs, float Lq, float Ld, float k, uint32_t pole_pairs, uint32_t gear_ratio, float offset);
    ~foc_smo();

    void clear(void);
    
    float ElecAngleCalc(alphabeta_t U_alphabeta, alphabeta_t I_alphabeta, float Ts);

    float ElecSpeedCalc(void);

};
































#endif
