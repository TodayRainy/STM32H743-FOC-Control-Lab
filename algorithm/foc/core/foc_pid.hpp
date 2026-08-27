/**
 ****************************************************************************************************
 * @file        foc_pid.hpp
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


#ifndef _FOC_PID_HPP
#define _FOC_PID_HPP

#include <stdbool.h>




















class foc_pid
{
private:
    /* data */
    float  ref;                /* 设定目标 */
    float  ref_limit;
    float  fb;                 /* 反馈值 */

    float  kp;                 /* 比例常数 P */
    float  ki;                 /* 积分常数 I */
    float  kd;                 /* 微分常数 D */

    float  p_term;             /* 比例项 */
    float  i_term;             /* 积分项 */
    float  d_term;             /* 微分项 */

    float  i_term_limit;       /* 积分限制 */
    float  err;                /* Error[-1] */
    float  err_last;           /* Error[-1] */
    float  err_pre;            /* Error[-2] */


    float  out_limit;          /* 输出实际值限制 */
    float  out_min;            /*输出最小值*/
    float  out_max;            /*输出最大值*/


    bool Clear_SumError;       /*用于调试*/

public:
    float out;                /* 输出值 */

public:
    foc_pid(float kp, float ki, float kd, float ref_limit, float i_term_limit, float out_limit);
    ~foc_pid();

    

    float pi(float ref, float fb, float Ts);
    float pid(float ref, float fb, float Ts);

    float pi(float err, float Ts);
    float pid(float err, float Ts);

    void set_gains(float kp, float ki, float kd);
    void set_output_limit(float out_limit);
    void track_output(float applied_out);
    void clear(void);

};




















#endif
