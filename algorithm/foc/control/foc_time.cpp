/**
 ****************************************************************************************************
 * @file        foc_time.cpp
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

#include "foc_time.hpp"
#include "foc_hw.hpp"


#pragma arm section code  = "RAMCODE"

foc_time::foc_time()
{

    FOC_TIME_HW_Init();
}

foc_time::~foc_time()
{
    FOC_TIME_HW_DeInit();
}


/*获取间隔时间*/
float foc_time::get_interval_time(void)
{
    float Ts = 0;

    Ts = FOC_TIME_GetSecond();

    return Ts;
}

/*清除时间*/
void foc_time::clear(void)
{
    FOC_TIME_SetCount(0);
}

#pragma arm section
