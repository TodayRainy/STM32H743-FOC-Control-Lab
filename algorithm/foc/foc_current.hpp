/**
 ****************************************************************************************************
 * @file        foc_current.hpp
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

#ifndef _FOC_CURRENT_H
#define _FOC_CURRENT_H


#include "foc_type.h"

#include <stdint.h>
#include <stdbool.h>

#define CALIBRATION_TIMES       1000U                   /*AD校准时采集次数*/
#define CALIBRATION_OFFSET_LIMIT 100.0f                 /*允许的最大零点偏置*/

typedef enum
{
    CURRENT_CALIB_IN_PROGRESS = 0,
    CURRENT_CALIB_DONE,
    CURRENT_CALIB_FAILED,
} CURRENT_CALIB_RESULT_t;

class foc_current
{
private:
    /* data */
    float       ad_mid;
    abc_t       ad_offset_temp;                         /*临时值*/
    uint32_t    ad_offset_cnt;                          /*偏置采集次数*/
    uint32_t    protect_cnt;
    float       Is_max;
public:
    abc_t       mcu_ad;                                 /*MCU端口的AD值*/
    abc_t       Iabc;                                   /*采集转化的三相电流值*/
    abc_t       ad_offset;                              /*电流AD偏置 ad采集值*/
public:
    foc_current(float ad_mid, float Is_max);
    ~foc_current();
    void offset_reset(void);
    CURRENT_CALIB_RESULT_t offset_calc(void);

    bool protect(qd_t Iqd);
};







#endif
