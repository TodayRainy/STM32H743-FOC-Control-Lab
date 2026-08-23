/**
 ****************************************************************************************************
 * @file        foc_current.cpp
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

#include "foc_current.hpp"
#include "foc_math.hpp"
#include "foc_motor.hpp"
#include "foc_hw.hpp"

#include <bsp.hpp>
#include <adc.h>


#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>


#pragma arm section code  = "RAMCODE"

foc_current::foc_current(float ad_mid, float Is_max)
{
    offset_reset();
    memset(&mcu_ad, 0x00, sizeof(mcu_ad));
    memset(&Iabc, 0x00, sizeof(Iabc));
    this->protect_cnt   = 0;
    this->ad_mid        = ad_mid;
    this->Is_max        = fabsf(Is_max);

    

    FOC_CURRENT_HW_Init();
}

foc_current::~foc_current()
{
    FOC_CURRENT_HW_DeInit();
}

void foc_current::offset_reset(void)
{
    memset(&ad_offset, 0x00, sizeof(ad_offset));
    memset(&ad_offset_temp, 0x00, sizeof(ad_offset_temp));
    this->ad_offset_cnt = 0;
}


/*计算偏差*/
CURRENT_CALIB_RESULT_t foc_current::offset_calc(void)
{
    this->ad_offset_temp.a += this->mcu_ad.a;
    this->ad_offset_temp.b += this->mcu_ad.b;
    this->ad_offset_cnt++;

    if(this->ad_offset_cnt < CALIBRATION_TIMES){
        return CURRENT_CALIB_IN_PROGRESS;
    }

    this->ad_offset.a = this->ad_offset_temp.a/CALIBRATION_TIMES - ad_mid;
    this->ad_offset.b = this->ad_offset_temp.b/CALIBRATION_TIMES - ad_mid;
    /* 两电阻采样没有 C 相 ADC，C 相电流由 A/B 相重构 */
    this->ad_offset.c = 0.0f;

    memset(&this->ad_offset_temp, 0x00, sizeof(this->ad_offset_temp));
    this->ad_offset_cnt = 0;

    if((fabsf(this->ad_offset.a) > CALIBRATION_OFFSET_LIMIT) ||
       (fabsf(this->ad_offset.b) > CALIBRATION_OFFSET_LIMIT)){
        return CURRENT_CALIB_FAILED;
    }

    return CURRENT_CALIB_DONE;
}


bool foc_current::protect(qd_t Iqd)
{
    bool ret = false;
    
    float Is = 0;
    arm_sqrt_f32(SQ(Iqd.q)+SQ(Iqd.d), &Is);
    if(Is>=Is_max){
        protect_cnt++;
    }else{
        protect_cnt = 0;
    }

    if(protect_cnt>=1000){
        printf("警告 电流过流: Iq:%.4f Id:%.4f\r\n", Iqd.q, Iqd.d);
        ret = true;
    }

    return ret;
}




































// #pragma arm section code  = "RAMCODE"

#pragma arm section

