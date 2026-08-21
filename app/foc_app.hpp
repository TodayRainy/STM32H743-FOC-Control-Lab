/**
 ****************************************************************************************************
 * @file        foc_app.hpp
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


#ifndef __FOC_APP_HPP
#define __FOC_APP_HPP


#include "foc_motor.hpp"
#include "foc_math.hpp"





extern void FOC_Init(void);

extern void FOC_App_Run(void);

void foc_key(void);



void FOC_Init_CPP(void);
void FOC_RUN(void);
void FOC_SPEED_CALC(void);
void FOC_SPEED_CTRL(void);
void FOC_POS_CTRL(void);



// extern foc_motor *m0;






#endif




