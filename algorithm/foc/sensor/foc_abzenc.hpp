/**
 ****************************************************************************************************
 * @file        foc_abzenc.hpp
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


#ifndef _FOC_ABZENC_H
#define _FOC_ABZENC_H


#include "foc_type.h"
#include "foc_speed.hpp"
#include "foc_filter.hpp"
#include <stdbool.h>

class foc_abzenc
{
private:
    /* data */
    float   zero_offset;                        /*零点偏移*/
    uint8_t calib_state;                        /*校准状态*/
    uint32_t calib_cnt;                         /*校准次数*/

    int32_t align_dir_cnt;                      /*方向计数*/
    int8_t install_dir;                         /*安装方向*/
    float offset_sum[2];

    int32_t over_count;
    uint32_t count_pre;
    uint8_t count_initialized;

public:
    foc_speed *spd;

public:
    foc_abzenc(uint32_t pole_pairs, uint32_t gear_ratio, float zero_offset, int8_t install_dir);
    ~foc_abzenc();

    void reset_calibration(void);

    /*方向校准*/
    bool DirCalib(float Ts);

    bool ElecAngleCalib(qd_t &Uqd, float Ts);
    float ElecAngleCalc(float Ts);

    float ElecSpeedCalc(void);

    void EstimatorModeSet(foc_speed_estimator_mode_t mode);

};






















#endif
