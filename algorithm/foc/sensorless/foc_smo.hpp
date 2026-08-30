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
#include "foc_speed.hpp"
#include "foc_sensorless_observer.hpp"

class foc_smo
{
private:
    foc_esmo_observer observer;

public:
    foc_speed *spd;
public:
    foc_smo(float Rs, float Lq, float sliding_gain, float boundary_current,
            float emf_filter_hz, float pll_bandwidth_hz,
            uint32_t pole_pairs, uint32_t gear_ratio, float offset,
            float extra_delay_s);
    ~foc_smo();

    void clear(void);
    
    float ElecAngleCalc(alphabeta_t U_alphabeta, alphabeta_t I_alphabeta, float Ts);

    float ElecSpeedCalc(void);

    foc_sensorless_output_t output(void) const;

};
































#endif
