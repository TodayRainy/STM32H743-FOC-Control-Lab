/**
 ****************************************************************************************************
 * @file        foc_hfi.cpp
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

#include "foc_hfi.hpp"
#include "foc_math.hpp"



#pragma arm section code  = "RAMCODE"


foc_hfi::foc_hfi(float hfi_vol_ref, float d_bias_vol, uint8_t div, uint32_t pole_pairs, uint32_t gear_ratio, float offset)
{
    /*补齐漏初始化的成员: 对象由 new 分配, C++ 不会自动清零*/
    this->hfi_freq = 0;
    this->hfi_time = 0;

    this->hfi_vol_ref       = hfi_vol_ref;
    this->d_bias_vol        = d_bias_vol;
    this->hfi_time_ref =    div;
    this->hfi_vol           = hfi_vol_ref;

    this->sign = 1;


    pll = new foc_pll(135.0f, 9.23320198f);
    spd = new foc_speed(pole_pairs, gear_ratio, offset);
    
}

foc_hfi::~foc_hfi()
{
    delete(pll);
    delete(spd);
}

/*高频注入*/
void foc_hfi::inject(void)
{
    if(++hfi_time>=hfi_time_ref){
        sign = -sign;
        hfi_time = 0;
        hfi_vol = sign*hfi_vol_ref + d_bias_vol;    /*高频电压+d轴偏置电压*/
    }
}

/*提取低频*/
qd_t foc_hfi::Extract_LF(qd_t Iqd, qd_t Iqd_last)
{
    qd_t Iqd_lf = {0};

    Iqd_lf.q = (Iqd.q + Iqd_last.q)*0.5f;
    Iqd_lf.d = (Iqd.d + Iqd_last.d)*0.5f;
    return Iqd_lf;
}

/*提取高频*/
qd_t foc_hfi::Extract_HF(qd_t Iqd, qd_t Iqd_last)
{
    qd_t Iqd_hf = {0};

    Iqd_hf.q = (Iqd.q - Iqd_last.q)*0.5f;
    Iqd_hf.d = (Iqd.d - Iqd_last.d)*0.5f;

    return Iqd_hf;
}

/*提取低频*/
alphabeta_t foc_hfi::Extract_LF(alphabeta_t Ialphabeta, alphabeta_t Ialphabeta_last)
{
    alphabeta_t alphabeta_lf = {0};

    alphabeta_lf.alpha = (Ialphabeta.alpha + Ialphabeta_last.alpha)*0.5f;       /*两次的高频分量相+=0*/
    alphabeta_lf.beta = (Ialphabeta.beta + Ialphabeta_last.beta)*0.5f;

    return alphabeta_lf;
}

/*提取高频*/
alphabeta_t foc_hfi::Extract_HF(alphabeta_t Ialphabeta, alphabeta_t Ialphabeta_last)
{
    alphabeta_t alphabeta_hf = {0};                     /*高频分量*/

    alphabeta_hf.alpha = (Ialphabeta.alpha - Ialphabeta_last.alpha)*0.5f;       /*两次的高频分量相-= 2倍绝对值*/   
    alphabeta_hf.beta = (Ialphabeta.beta - Ialphabeta_last.beta)*0.5f;

    return alphabeta_hf;
}

/*电角度计算*/
float foc_hfi::ElecAngleCalc(alphabeta_t Ialphabeta, float Ts)
{
    float elec_angle = 0;      /*电角度*/

    alphabeta_t alphabeta_Ih = {0};     /*包络检测后的αβ*/

    int8_t sign = hfi_vol>0?1:-1;

    /*上一次的高频αβ轴分量*/
    alphabeta_hlast = alphabeta_h;

    /*提取这一次的高频αβ轴分量*/
    alphabeta_h = Extract_HF(Ialphabeta, alphabeta_last);
    /*提取这一次的低频分量*/
    alphabeta_f = Extract_LF(Ialphabeta, alphabeta_last);

    alphabeta_last = Ialphabeta;        /*保存这次加了激励的值*/

    /*这里求αβ轴电流的微分*/
    alphabeta_Ih.alpha = (alphabeta_h.alpha - alphabeta_hlast.alpha)*sign;
    alphabeta_Ih.beta = (alphabeta_h.beta - alphabeta_hlast.beta)*sign;

    #if 0
    /*反正切*/
    elec_angle = Limit_Angle(atan2f(alphabeta_Ih.beta, alphabeta_Ih.alpha));
    #else
    /*锁相环*/
    elec_angle = Limit_Angle(pll->calc_hfi(alphabeta_Ih.alpha, alphabeta_Ih.beta, Ts));
    #endif

    /*计算注入频率*/
    hfi_freq = 1.0f/Ts/hfi_time_ref/2.0f;

    inject();

    spd->AngleRefresh(elec_angle);

    return elec_angle;
}

/*电角速度计算*/
float foc_hfi::ElecSpeedCalc()
{
    spd->SpeedRefresh(pll->movefilter->calc());

    return spd->speed_data.elec_speed;
}


void foc_hfi::clear(void)
{
    spd->clear();
    pll->clear();
}


#pragma arm section
