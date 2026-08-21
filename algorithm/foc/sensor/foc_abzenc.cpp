/**
 ****************************************************************************************************
 * @file        foc_abzenc.cpp
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

#include "foc_abzenc.hpp"
#include "foc_math.hpp"
#include "foc_hw.hpp"

#include <stdio.h>



#pragma arm section code  = "RAMCODE"





foc_abzenc::foc_abzenc(uint32_t pole_pairs, uint32_t gear_ratio, float zero_offset, int8_t install_dir)
{
    /*补齐漏初始化的成员: 对象由 new 分配, C++ 不会自动清零*/
    this->over_count = 0;

/*初始化以后先计算一次速度，不然正式启动会抖动一下*/
    this->offset_sum[0]     = 0;
    this->offset_sum[1]     = 0;
    this->calib_state       = 0;
    this->calib_cnt         = 0;
    this->align_dir_cnt     = 0;
    this->install_dir       = install_dir;
    this->zero_offset       = zero_offset;

    spd = new foc_speed(pole_pairs, gear_ratio, 0);

    FOC_ABZENC_HW_Init();

    float mech_angle = FOC_ABZENC_HW_GetMechAngle();                                    /*机械角度*/
    float elec_angle = Limit_Angle(mech_angle*pole_pairs)+ spd->speed_data.offset+zero_offset;      /*电角度*/
    spd->SetAnglePre(elec_angle, mech_angle, 0);
    ElecAngleCalc(1.0f);

    
}

foc_abzenc::~foc_abzenc()
{
    delete(spd);
    FOC_ABZENC_HW_DeInit();  
}


/*方向校准*/
bool foc_abzenc::DirCalib(float Ts)
{
    bool ret = false;
    float elec_angle = ElecAngleCalc(Ts);

    switch(calib_state){
        case 0:
            calib_cnt               = 0;
            zero_offset             = 0;
            align_dir_cnt           = 0;
            calib_state             = 1;
            install_dir             = 1;
            break;
        case 1:
            if(spd->speed_data.elec_speed >0){
                align_dir_cnt++;
            }else{
                align_dir_cnt--;
            }

            if(++calib_cnt>(2.0f/Ts)){
                calib_cnt = 0;
                calib_state = 2;
                install_dir = align_dir_cnt>=0?1:-1;
                printf("编码器安装方向: %d\r\n", install_dir);
            }
            break;
        case 2:
            calib_cnt = 0;
            calib_state = 0;
            ret = true;
            break;
    }

    return ret;
}

/*电角度校准*/
bool foc_abzenc::ElecAngleCalib(qd_t &Uqd, float Ts)
{
    bool ret = false;
    float elec_angle = ElecAngleCalc(Ts);

    Uqd.q = 0;

    switch(calib_state){
        case 0:
            calib_cnt               = 0;
            zero_offset             = 0;
            offset_sum[0]           = 0;
            offset_sum[1]           = 0;
            calib_state             = 1;
            break;
        case 1:                 /*给正Uq偏移*/
            Uqd.q = Uqd.d*0.5f;
            if(++calib_cnt>(0.5f/Ts)){
                calib_cnt = 0;
                calib_state = 2;
            };
            break;
        case 2:                 /*不再给Uq，等待稳定*/
            if(++calib_cnt>(0.5f/Ts)){
                calib_cnt = 0;
                calib_state = 3;
            };
            break;
        case 3:                 /*累计电角度*/
            offset_sum[0] += elec_angle;    
            if(++calib_cnt>(0.5f/Ts)){
                offset_sum[0] /= (float)calib_cnt;

                calib_cnt = 0;
                calib_state = 4;
            }
            break;
        case 4:                 /*给负Uq偏移*/
            Uqd.q = -Uqd.d*0.5f;
            if(++calib_cnt>(0.5f/Ts)){
                calib_cnt = 0;
                calib_state = 5;
            };
            break;
        case 5:                 /*不再给Uq，等待稳定*/
            if(++calib_cnt>(0.5f/Ts)){
                calib_cnt = 0;
                calib_state = 6;
            };
            break;
        case 6:                 /*累计电角度*/
            offset_sum[1] += elec_angle;    
            if(++calib_cnt>(0.5f/Ts)){
                offset_sum[1] /= (float)calib_cnt;
                calib_cnt = 0;
                calib_state = 7;
            }
            break;
        case 7:
            zero_offset = (offset_sum[0]+offset_sum[1])*0.5f *(-install_dir);
            printf("校准结束: 编码器偏移值: %f\r\n\r\n", zero_offset);
            ret = true;
            calib_state = 0;
            calib_cnt = 0;

            Uqd.q = 0;
            Uqd.d = 0;
            
            break;
        default:
            calib_state = 0;
            calib_cnt = 0;
            break;     
    }

    return ret;
}

/*电角度计算*/
float foc_abzenc::ElecAngleCalc(float Ts)
{

    float elec_angle = 0;                               /*电角度*/
    float mech_angle = 0;                               /*机械角度*/

    float pole_pairs = spd->speed_data.pole_pairs;

    mech_angle = FOC_ABZENC_HW_GetMechAngle();
    elec_angle = Limit_Angle(mech_angle*pole_pairs + spd->speed_data.offset + zero_offset );

    if(-1==install_dir){
        elec_angle = _2PI-elec_angle;
    }

    spd->AngleRefresh(elec_angle);
    spd->SpeedCalc(Ts);

    return elec_angle;    
}

/*电角速度计算*/
float foc_abzenc::ElecSpeedCalc(void)
{
    return spd->speed_data.elec_speed;
}




#pragma arm section













