/**
 ****************************************************************************************************
 * @file        foc_pid.cpp
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

#include "foc_pid.hpp"
#include "foc_math.hpp"


#include <stdio.h>
#include <math.h>


#pragma arm section code  = "RAMCODE"


foc_pid::foc_pid(float kp, float ki, float kd, float ref_limit, float i_term_limit, float out_limit)
{
    this->kp            = kp;
    this->ki            = ki;
    this->kd            = kd;
    this->ref_limit     = ref_limit;
    this->i_term_limit  = i_term_limit;
    this->out_limit     = out_limit;

    clear();
    
}

foc_pid::~foc_pid()
{
}

void foc_pid::set_gains(float kp, float ki, float kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}


void foc_pid::clear(void)
{

    this->ref           = 0;
    this->fb            = 0;
    this->p_term        = 0;
    this->i_term        = 0;
    this->d_term        = 0;
    this->err           = 0;
    this->err_last      = 0;
    this->err_pre       = 0;
    this->out_min       = 0;
    this->out_max       = 0;
    this->out           = 0;

    Clear_SumError = false;
}

void foc_pid::track_output(float applied_out)
{
    if(ki!=0){
        i_term += applied_out-out;
        i_term = LIMIT_RANGE(i_term, -i_term_limit, i_term_limit);
    }
    out = applied_out;
}



float foc_pid::pi(float ref, float fb, float Ts)
{
    float i_term_tmp = 0;
    float discharge = 0;

    ref = LIMIT_RANGE(ref, -ref_limit, ref_limit);
    this->ref = ref;
    this->fb = fb;
    err = ref - fb;

    /*比例项*/
    p_term = kp *err;

    /*积分项*/
    if(0==ki){
        i_term = 0;
    }else{
        i_term_tmp = ki *err*Ts;
        i_term += i_term_tmp;

        /*积分限制幅度*/
        if(i_term>i_term_limit){
            i_term = i_term_limit;
        }else if(i_term<-i_term_limit){
            i_term = -i_term_limit;
        }
        i_term = LIMIT_RANGE(i_term, -i_term_limit, i_term_limit); 
    }

    out = p_term + i_term;

    if(out>out_limit){
        discharge = out_limit - out;
    }else if(out<-out_limit){
        discharge = -out_limit - out;
    }

    out = LIMIT_RANGE(out, -out_limit, out_limit);           /*输出限制*/

    if(out>out_max){
        out_max = out;
    }
    if(out<out_min){
        out_min = out;
    }

    i_term += discharge;

    return out;
}

float foc_pid::pid(float ref, float fb, float Ts)
{


    float i_term_tmp = 0;
    float discharge = 0;
    float delta_err = 0;

    /* 参数合法性检查 */
    ref = LIMIT_RANGE(ref, -ref_limit, ref_limit);
    this->ref = ref;
    this->fb = fb;
    err = ref - fb;

    /*比例项*/
    p_term = kp *err;

    /*积分项*/
    if(0==ki){
        i_term = 0;
    }else{
        i_term_tmp = ki *err*Ts;
        i_term += i_term_tmp;

        /*积分限制幅度*/
        if(i_term>i_term_limit){
            i_term = i_term_limit;
        }else if(i_term<-i_term_limit){
            i_term = -i_term_limit;
        }
        i_term = LIMIT_RANGE(i_term, -i_term_limit, i_term_limit); 
    }

    /*微分项*/
    delta_err = err - err_last;
    err_last = err;
    if(Ts){
        d_term = kd * delta_err/Ts;
    }

    out = p_term + i_term + d_term;

    if(out>out_limit){
        discharge = out_limit - out;
    }else if(out<-out_limit){
        discharge = -out_limit - out;
    }

    out = LIMIT_RANGE(out, -out_limit, out_limit);           /*输出限制*/
    i_term += discharge;

    if(out>out_max){
        out_max = out;
    }
    if(out<out_min){
        out_min = out;
    }

    return out;
}

float foc_pid::pi(float err, float Ts)
{
    float i_term_tmp = 0;
    float discharge = 0;

    this->err = err;

    /*比例项*/
    p_term = kp *err;

    /*积分项*/
    if(0==ki){
        i_term = 0;
    }else{
        i_term_tmp = ki *err*Ts;
        i_term += i_term_tmp;

        /*积分限制幅度*/
        if(i_term>i_term_limit){
            i_term = i_term_limit;
        }else if(i_term<-i_term_limit){
            i_term = -i_term_limit;
        }
        i_term = LIMIT_RANGE(i_term, -i_term_limit, i_term_limit); 
    }

    out = p_term + i_term;

    if(out>out_limit){
        discharge = out_limit - out;
    }else if(out<-out_limit){
        discharge = -out_limit - out;
    }

    out = LIMIT_RANGE(out, -out_limit, out_limit);           /*输出限制*/
    i_term += discharge;

    if(out>out_max){
        out_max = out;
    }
    if(out<out_min){
        out_min = out;
    }

    return out;
}

float foc_pid::pid(float err, float Ts)
{
    float i_term_tmp = 0;
    float discharge = 0;
    float delta_err = 0;

    /* 参数合法性检查 */

    /*比例项*/
    p_term = kp *err;

    /*积分项*/
    if(0==ki){
        i_term = 0;
    }else{
        i_term_tmp = ki *err*Ts;
        i_term += i_term_tmp;

        /*积分限制幅度*/
        if(i_term>i_term_limit){
            i_term = i_term_limit;
        }else if(i_term<-i_term_limit){
            i_term = -i_term_limit;
        }
        i_term = LIMIT_RANGE(i_term, -i_term_limit, i_term_limit); 
    }

    /*微分项*/
    delta_err = err - err_last;
    err_last = err;
    d_term = kd * delta_err/Ts;
    

    out = p_term + i_term + d_term;

    if(out>out_limit){
        discharge = out_limit - out;
    }else if(out<-out_limit){
        discharge = -out_limit - out;
    }

    out = LIMIT_RANGE(out, -out_limit, out_limit);           /*输出限制*/
    i_term += discharge;

    if(out>out_max){
        out_max = out;
    }
    if(out<out_min){
        out_min = out;
    }

    return out;
}





#pragma arm section


