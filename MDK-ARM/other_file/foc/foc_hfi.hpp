/**
 ****************************************************************************************************
 * @file        foc_hfi.hpp
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


#ifndef _FOC_HFI_HPP
#define _FOC_HFI_HPP

#include "foc_type.h"
#include "foc_filter.hpp"
#include "foc_speed.hpp"
#include "arm_math.h"

#define FOC_HFI_FREQ_1K         1000
#define FOC_HFI_FREQ_5K         5000
#define FOC_HFI_FREQ_10K        10000
#define FOC_HFI_FREQ_20K        20000


class foc_hfi
{

private:
    float d_bias_vol;                   /*D轴偏置电压*/
    float hfi_vol_ref;                  /*高频注入电压参考值*/
    int8_t sign;

    uint32_t hfi_freq;                  /*注入频率*/
    uint32_t hfi_time;                  /*注入时间*/
    uint32_t hfi_time_ref;              /*注入参考时间*/

    qd_t Iqd_lf;                        /*qd轴低频电流*/
    qd_t Iqd_hf;                        /*qd轴高频电流*/
    qd_t Iqd_last;                      /*上次的qd电流*/
    
    alphabeta_t alphabeta_hlast;
    alphabeta_t alphabeta_last;

    foc_pll *pll;
    

public:
    float hfi_vol;                      /*高频注入电压值*/
    alphabeta_t alphabeta_h;            /*αβ轴高频电流*/
    alphabeta_t alphabeta_f;            /*αβ轴低频电流*/

    foc_speed *spd;

private:
    qd_t Extract_LF(qd_t Iqd, qd_t Iqd_last);
    qd_t Extract_HF(qd_t Iqd, qd_t Iqd_last);

    alphabeta_t Extract_LF(alphabeta_t Ialphabeta, alphabeta_t Ialphabeta_last);
    alphabeta_t Extract_HF(alphabeta_t Ialphabeta, alphabeta_t Ialphabeta_last);

public:
    foc_hfi(float hfi_vol_ref, float d_bias_vol, uint8_t div, uint32_t pole_pairs, uint32_t gear_ratio, float offset);
    ~foc_hfi();

    void clear(void);

    void inject(void);

    float ElecAngleCalc(alphabeta_t Ialphabeta, float Ts);
    float ElecSpeedCalc(void);

};








#endif
