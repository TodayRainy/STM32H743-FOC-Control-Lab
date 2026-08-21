/**
 ****************************************************************************************************
 * @file        foc_iden.cpp
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

#include "foc_iden.hpp"
#include "foc_math.hpp"

#pragma arm section code  = "RAMCODE"

#define debug   0

foc_offline_iden::foc_offline_iden(float Is_max, float elec_speed_l, float elec_speed_h,
                            float Rs, float Lq, float Ld)
{
    this->all_state         = 0;
    this->state             = 0;
    this->cnt               = 0;
    this->Vs_avg            = 0;
    this->Is_avg            = 0;
    this->Is_err_thres      = 0.17f;   /*需要采样精度满足*/
    this->Vs_step           = 0.0001f;

    this->Is_max            = Is_max;
    this->elec_speed_l      = elec_speed_l;
    this->elec_speed_h      = elec_speed_h;
    this->Lq                = Lq;
    this->Ld                = Ld;
    this->Ls                = (Lq+Ld)*0.5f;

    for(int i=0;i<RS_IDEN_BUF_SIZE;i++){
        Is_steps_ref[i] = 0;
        Vs_steps_ref[i] = 0;
        Is_steps[i] = 0;
        Vs_steps[i] = 0;
    }
}

foc_offline_iden::~foc_offline_iden()
{
}

/*电阻辨识需要保持电机静止*/
bool foc_offline_iden::RsIden(uint8_t step_num, qd_t &Uqd, qd_t Iqd, uint32_t samps, float Ts)
{
    bool ret = false;
    float Rs            = 0;
    float Is_ref        = 0;
    float step_size     = 0;
    int8_t I_dir;                                                       /*电流方向*/ 

    if(step_num>RS_IDEN_BUF_SIZE)step_num=RS_IDEN_BUF_SIZE;       /*限制最大阶梯数*/
    else if(step_num%2)step_num -= 1;                             /*阶段偶数化*/

    switch(state){
        case 0:             /*初始化*/
            cnt = 0;
            step = 0;
            Vs_avg = 0;
            Is_avg = 0;
            step_size = Is_max/(step_num/2);

            for(int i=0;i<(step_num/2);i++){
                Is_steps_ref[i] = -step_size*(i+1);
                Is_steps_ref[i+step_num/2] = -Is_steps_ref[i];
            }

            for (size_t i = 0; i < step_num; i++){
                Vs_steps_ref[i] = 0;
            }
        
            state = 1;
            break;
        case 1:             /*计算实际定子电流和目标定子电流的差，采用逐次逼近方法 施加电压*/
            Is = sqrtf(SQ(Iqd.q)+SQ(Iqd.d));
            Is_err = Is - fabsf(Is_steps_ref[step]);
            I_dir = Is_steps_ref[step]>=0?1:-1;
        #if 0
            Is *= I_dir;
            if(Is>Is_steps_ref[step]){
                Vs_steps_ref[step] -= Vs_step;
            }else if(Is<Is_steps_ref[step]){
                Vs_steps_ref[step] += Vs_step;
            }
        #else
            Is_ref = fabsf(Is_steps_ref[step]);
            if(Is<Is_ref){
                Vs_steps_ref[step] += Vs_step*I_dir;
            }else{
                Vs_steps_ref[step] -= Vs_step*I_dir;
            }
        #endif

            cnt += (fabsf(Is_err)<Is_err_thres)? 1:0;
            if(cnt>5){
                cnt = 0;
                Vs_avg = 0;
                Is_avg = 0;
                state = 2;
            }

            Uqd.d = Vs_steps_ref[step];
            Uqd.q = 0;

            break;  
        case 2:     /*延时10毫秒, 等待稳定以后再采样*/
            if(++cnt>=(0.01f/Ts)){
                cnt = 0;
                state = 3;
            }
            break;
        case 3:     /*采样阶段*/
            Vs_avg += sqrtf(SQ(Uqd.q)+SQ(Uqd.d));
            Is_avg += sqrtf(SQ(Iqd.q)+SQ(Iqd.d));
 
            if(++cnt>=samps){
                cnt = 0;
                Vs_steps[step] = Vs_avg/(float)samps;
                Is_steps[step] = Is_avg/(float)samps;
                if(Vs_max<Vs_steps[step])Vs_max=Vs_steps[step];

                #if debug
                // printf("Vs_steps[%d]:【%.4f】  Is_steps:【%.4f】\r\n", step, Vs_steps[step], Is_steps[step]);
                printf("Vs_steps_ref: %f Is: %f ref_current: %f Is_err:%f  \r\n", Vs_steps[step], Is, Is_steps[step], Is_err);
                #endif
                
                step++;
                if(step<step_num){
                    state = 1;
                }else{
                    state = 4;
                    step = 0;
                }
            }
            break;
        case 4:     /*计算阶段*/
            Rs = 0;
            /*计算负阶段电压*/
            for(size_t i=0;i<(step_num/2-1);i++){
                Rs += (Vs_steps[i+1]-Vs_steps[i]) / (Is_steps[i+1]-Is_steps[i]);
                cnt++;
            }
            /*计算正阶段电压*/
            for (size_t i = (step_num/2); i < (step_num-1); i++){
                Rs += (Vs_steps[i+1]-Vs_steps[i]) / (Is_steps[i+1]-Is_steps[i]);
                cnt++;
            }

            Rs /= (float)cnt;
            this->Rs = Rs;
            #if debug
            printf("辨识电阻Rs: %fmΩ\r\n", this->Rs*1000);
            printf("最大辨识电压:%.4fV\r\n", Vs_max);
            #endif
            Uqd.q = 0;
            Uqd.d = 0;
            cnt = 0;
            state = 0;
            ret = true;
            break;
        default:
            break;
    }
    return ret;
}

/*电感辨识*/
bool foc_offline_iden::LsIden(qd_t &Uqd, qd_t Iqd, uint8_t prescale, uint32_t samps, float Ts)
{
    bool ret = false;

    float I_avg     = 0;
    float delta_i   = 0;
    float L_step    = 0;
    float Ld_neg    = 0;
    float Lq_pos    = 0;
    float Lq_neg    = 0;
    float Ld_pos    = 0;

    float delta_t   = prescale * Ts;


    switch (state){
        case 0:                 /* 初始化 */
            cnt             = 0;
            prescale_cnt    = 0;

            /*清零新加的累加器和计数器*/
            memset(&Iqd_sum_p, 0x00, sizeof(Iqd_sum_p));
            memset(&Iqd_sum_n, 0x00, sizeof(Iqd_sum_n));
            memset(&Iqd_p_cnt, 0x00, sizeof(Iqd_p_cnt));
            memset(&Iqd_n_cnt, 0x00, sizeof(Iqd_n_cnt));

            /* 临时变量在步长开始时重置，此处可不初始化，但置零无妨 */
            I_start = 0; 
            I_sum = 0; 
            Iqd_cnt = 0;
            Uqd.q = 0;
            
            V_pulse = -fabsf(Vs_max);   /*初始为负电压*/
            Uqd.d = V_pulse;
            state = 1;
            break;
        case 1:                 /* d轴电压脉冲 */
            if (++prescale_cnt < prescale) {
                /*步长内：累积电流，并记录起始电流（步长的第一个Ts）*/
                if (prescale_cnt == 1) {          // 步长开始
                    I_start = Iqd.d;
                    I_sum = 0;
                    Iqd_cnt = 0;
                }
                I_sum += Iqd.d;
                Iqd_cnt++;
                break;
            }

            I_sum += Iqd.d;
            Iqd_cnt++;
            prescale_cnt = 0;
            I_end = Iqd.d;
            I_avg = I_sum / Iqd_cnt;
            delta_i = I_end - I_start;


            /*计算阶段的电感值，计入累加*/
            if (V_pulse > 0) {                      /*正脉冲*/
                L_step = (fabsf(V_pulse) - Rs * I_avg) * delta_t / delta_i;
                Iqd_sum_p.d += L_step;
                Iqd_p_cnt.d++;
            } else {                                /*负脉冲*/
                L_step = (-fabsf(V_pulse) - Rs * I_avg) * delta_t / delta_i;
                Iqd_sum_n.d += L_step;
                Iqd_n_cnt.d++;
            }

            /*反向注入*/
            V_pulse = -V_pulse;
            Uqd.d = V_pulse;
            Uqd.q = 0;

            /*采样次数累积*/
            cnt++;
            if (cnt >= samps) {
                /*d轴脉冲结束，准备切换到q轴*/
                V_pulse = -fabsf(Vs_max);   /*复位为负电压*/
                cnt = 0;
                Uqd.d = 0;
                Uqd.q = V_pulse;
                state = 2;
            }
            break;

        case 2:                 /* q轴电压脉冲 */
            if (++prescale_cnt < prescale) {
                if (prescale_cnt == 1) {
                    I_start = Iqd.q;
                    I_sum = 0;
                    Iqd_cnt = 0;
                }
                I_sum += Iqd.q;
                Iqd_cnt++;
                break;
            }

            I_sum += Iqd.d;
            Iqd_cnt++;
            prescale_cnt = 0;
            I_end = Iqd.q;
            I_avg = I_sum / Iqd_cnt;
            delta_i = I_end - I_start;
            

            if (V_pulse > 0) {
                L_step = (fabsf(V_pulse) - Rs * I_avg) * delta_t / delta_i;
                Iqd_sum_p.q += L_step;
                Iqd_p_cnt.q++;
            } else {
                L_step = (-fabsf(V_pulse) - Rs * I_avg) * delta_t / delta_i;
                Iqd_sum_n.q += L_step;
                Iqd_n_cnt.q++;
            }

            /*反向注入*/
            V_pulse = -V_pulse;
            Uqd.q = V_pulse;
            Uqd.d = 0;

            cnt++;
            if (cnt >= samps) {
                V_pulse = -fabsf(Vs_max);
                cnt = 0;
                Uqd.d = 0;
                Uqd.q = 0;
                state = 3;
            }
            break;

        case 3:                 /* 计算qd轴电感 */
            // 对正负脉冲分别取平均
            Ld_pos = (Iqd_p_cnt.d > 0) ? (Iqd_sum_p.d / Iqd_p_cnt.d) : 0;
            Ld_neg = (Iqd_n_cnt.d > 0) ? (Iqd_sum_n.d / Iqd_n_cnt.d) : 0;
            Ld = (Ld_pos + Ld_neg) * 0.5f;

            Lq_pos = (Iqd_p_cnt.q > 0) ? (Iqd_sum_p.q / Iqd_p_cnt.q) : 0;
            Lq_neg = (Iqd_n_cnt.q > 0) ? (Iqd_sum_n.q / Iqd_n_cnt.q) : 0;
            Lq = (Lq_pos + Lq_neg) * 0.5f;

            Lqd_dif = Lq - Ld;
            Ls = (Lq + Ld) * 0.5f;

            #if debug
            printf("Ld: %.5f uH\r\n", Ld * 1e6f);
            printf("Lq: %.5f uH\r\n", Lq * 1e6f);
            printf("Ldif: %.5f uH\r\n", Lqd_dif * 1e6f);
            printf("Ls: %.5f uH\r\n", Ls * 1e6f);
            printf("V_pulse: %f, Iqd_cnt: %d\r\n", V_pulse, Iqd_cnt);
            #endif

            state = 0;
            ret = true;
            
            break;
        default:
            break;
    }

    return ret;
}

/*磁链辨识*/
bool foc_offline_iden::FsIden(qd_t &Uqd, qd_t Iqd, float &elec_speed, float elec_speed_acc, float Ts)
{
    bool ret = false;

    float we_dif = 0;           /*速度差: 弧度/秒*/
    float we_h = 0;
    float we_l = 0;

    switch(state){
        case 0:             /*初始化*/
            cnt = 0;
            Uqd_avg_l.q = 0.0f;
            Uqd_avg_l.d = 0.0f;
            Uqd_avg_h.q = 0.0f;
            Uqd_avg_h.d = 0.0f;
            Iqd_avg_l.q = 0.0f;
            Iqd_avg_l.d = 0.0f;
            Iqd_avg_h.q = 0.0f;
            Iqd_avg_h.d = 0.0f;

            Uqd_l.q = 0.0f;
            Uqd_l.d = 0.0f;
            Uqd_h.q = 0.0f;
            Uqd_h.d = 0.0f;
            Iqd_l.q = 0.0f;
            Iqd_l.d = 0.0f;
            Iqd_h.q = 0.0f;
            Iqd_h.d = 0.0f;

            Vs_l = 0.0f;
            Vs_h = 0.0f;
            Is_l = 0.0f;
            Is_h = 0.0f;

            
            elec_speed_ref = elec_speed;
            elec_speed_set = elec_speed;
            state = 1;

            // Uqd.q = 0.7f;
            // Uqd.q = Vs_max;
            break;
        case 1:         /*低速加速*/
            elec_speed_ref = elec_speed_l;
            if(elec_speed_ref>elec_speed_set){
                elec_speed_set += elec_speed_acc *Ts;
                if(elec_speed_set>elec_speed_ref){
                    elec_speed_set=elec_speed_ref;
                }
            }else if(elec_speed_ref<elec_speed_set){
                elec_speed_set -= elec_speed_acc *Ts;
                if(elec_speed_set<elec_speed_ref){
                    elec_speed_set=elec_speed_ref;
                }
            }else{
                state = 2;
            }
            break;
        case 2:         /*稳定等待*/
            if(++cnt>=2.0f/Ts){
                Uqd_avg_l.q = 0;
                Uqd_avg_l.d = 0;
                Iqd_avg_l.q = 0;
                Iqd_avg_l.d = 0;
                cnt = 0;
                state = 3;
            }
            break;
        case 3:     /*低速采样*/
            Uqd_avg_l.d += Uqd.d;
            Uqd_avg_l.q += Uqd.q;
            Iqd_avg_l.d += Iqd.d;
            Iqd_avg_l.q += Iqd.q;

            if(++cnt>=1.0f/Ts){
                Uqd_avg_l.d /= (float)cnt;
                Uqd_avg_l.q /= (float)cnt;
                Iqd_avg_l.d /= (float)cnt;
                Iqd_avg_l.q /= (float)cnt;

                Uqd_l.d = Uqd_avg_l.d;
                Uqd_l.q = Uqd_avg_l.q;
                Iqd_l.d = Iqd_avg_l.d;
                Iqd_l.q = Iqd_avg_l.q;

                Vs_l = sqrtf(SQ(Uqd_l.d)+SQ(Uqd_l.q));
                Is_l = sqrtf(SQ(Iqd_l.d)+SQ(Iqd_l.q));

                cnt = 0;
                state = 4;

                // Uqd.q = 1.0f;
                // Uqd.q = Vs_max*1.5f;
            }
            break;
        case 4:     /*高速加速*/
            elec_speed_ref = elec_speed_h;
            if(elec_speed_ref>elec_speed_set){
                elec_speed_set += elec_speed_acc *Ts;
                if(elec_speed_set>elec_speed_ref){
                    elec_speed_set = elec_speed_ref;
                }
            }else if(elec_speed_ref<elec_speed_set){
                elec_speed_set -= elec_speed_acc *Ts;
                if(elec_speed_set<elec_speed_ref){
                    elec_speed_set = elec_speed_ref;
                }
            }else{
                state = 5;
            }
            break;
        case 5:     /*稳定等待*/
            if(++cnt>=2.0f/Ts){
                Uqd_avg_h.q = 0;
                Uqd_avg_h.d = 0;
                Iqd_avg_h.q = 0;
                Iqd_avg_h.d = 0;
                cnt = 0;
                state = 6;
            }
            break;
        case 6:     /*高速采样*/

            Uqd_avg_h.d += Uqd.d;
            Uqd_avg_h.q += Uqd.q;
            Iqd_avg_h.d += Iqd.d;
            Iqd_avg_h.q += Iqd.q;
            if(++cnt>=1.0f/Ts){
                
                Uqd_avg_h.d /= (float)cnt;
                Uqd_avg_h.q /= (float)cnt;
                Iqd_avg_h.d /= (float)cnt;
                Iqd_avg_h.q /= (float)cnt;

                Uqd_h.d = Uqd_avg_h.d;
                Uqd_h.q = Uqd_avg_h.q;
                Iqd_h.d = Iqd_avg_h.d;
                Iqd_h.q = Iqd_avg_h.q;
            
                Vs_h = sqrtf(SQ(Uqd_h.d)+SQ(Uqd_h.q));
                Is_h = sqrtf(SQ(Iqd_h.d)+SQ(Iqd_h.q));

                cnt = 0;
                state = 7;
            }
            break;
        case 7: /*计算磁链*/  
            if(elec_speed_set>0){
                elec_speed_set -= elec_speed_acc *Ts;
                if(elec_speed_set<0){
                    elec_speed_set = 0;
                    state = 8;
                }
            }else if(elec_speed_set<0){
                elec_speed_set += elec_speed_acc *Ts;
                if(elec_speed_set>0){
                    elec_speed_set = 0;
                    state = 8;
                }
            }else{
                state = 8;
            }
            break;
        case 8:

            #if 1
            /*用q轴电流电压来辨识*/
            Uq_dif = Uqd_h.q - Uqd_l.q;
            Iqd_dif.q = Iqd_h.q - Iqd_l.q;
            Iqd_dif.d = Iqd_h.d - Iqd_l.d;

            we_l = elec_speed_l;
            we_h = elec_speed_h;
            we_dif = we_h - we_l;
            // flux = (Uq_dif-Rs*Iqd_dif.q - we_h*Ls*Iqd_h.d + we_l *Ls*Iqd_l.d)/we_dif;
            /*适用于内嵌式电机*/
            flux = (Uq_dif-Rs*Iqd_dif.q - we_h*Ld*Iqd_h.d + we_l *Ld*Iqd_l.d)/we_dif;

            #if debug
            printf("Uq_dif: %f\r\n", Uq_dif);
            printf("Iqd_dif.q %f\r\n", Iqd_dif.q);
            printf("Iqd_dif.d: %f\r\n", Iqd_dif.d);
            printf("we_l: %f\r\n", we_l);
            printf("we_h: %f\r\n", we_h);
            printf("we_dif: %f\r\n", we_dif);
            printf("Uqd_h.q: %f\r\n", Uqd_h.q);
            printf("Uqd_l.q: %f\r\n", Uqd_l.q);
            printf("Iqd_h.q: %f\r\n", Iqd_h.q);
            printf("Iqd_h.d: %f\r\n", Iqd_h.d);
            #endif

            #else   /*用定子电流电压来辨识*/
            Vs_diff = Vs_h - Vs_l;
            Is_diff = Is_h - Is_l;
            we_l = mec_speed_l/60.0f*_2PI*(float)this->speed.pole_pairs;
            we_h = mec_speed_h/60.0f*_2PI*(float)this->speed.pole_pairs;
            we_dif = we_h - we_l;
            flux = (Vs_diff- Rs*Is_diff)/we_dif - Is_diff*Ls;
            #endif

            #if debug
            printf("Vs_diff: %f\r\n", Vs_diff);
            printf("Is_diff %f\r\n", Is_diff);
            printf("we_dif: %f\r\n", we_dif);
            printf("we_l: %f\r\n", we_l);
            printf("we_h: %f\r\n", we_h);
            #endif

            state = 9;
            
            break;
        case 9:
            #if debug
            printf("辨识磁链: %.5fmWb\r\n", flux*1000);
            #endif
            state = 0;
            ret = true;
            break;
        default:
            break;
    }

    elec_speed = elec_speed_set;
    
    return ret;
}   



#pragma arm section

































