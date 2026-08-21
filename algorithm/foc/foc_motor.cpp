/**
 ****************************************************************************************************
 * @file        foc_motor.cpp
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

#include "foc_motor.hpp"
#include <foc_math.hpp>

#include <stdio.h>



#pragma arm section code  = "RAMCODE"



#include "foc_hw.hpp"

foc_motor::foc_motor(qd_t Uqd, float max_elec_speed, float vbus, float Ts,
                        MOTOR_MODE_t m_mode, PWM_MODE_t pwm_mode, ANGLE_GEN_t angle_gen)
{
    this->Uqd               = Uqd;
    this->max_elec_speed    = max_elec_speed;
    this->vbus              = vbus;
    this->Ts                = Ts;
    this->m_mode            = m_mode;
    this->pwm_mode          = pwm_mode;
    this->angle_gen         = angle_gen;

    this->iden_state        = 0;
    this->enc_calib_state   = 0;
    this->spd_cnt           = 0;
    this->pos_cnt           = 0;
    this->Vd_bias           = 0;

    this->target_elec_speed     = 0;
    this->target_out_pos = 0;
    

    this->Iqd_ref.q         = 0.0;
    this->Iqd_ref.d         = 0.00001f;

    Ibus_filter = new foc_movefilter<float>(100);

    tabc_state = 0;

    memset(&this->speed_data, 0x00, sizeof(this->speed_data));
    memset(&this->debug, 0x00, sizeof(this->debug));

}

foc_motor::~foc_motor()
{

}

/*电阻电感辨识*/
bool foc_motor::iden_run_rl(void)
{

    float elec_angle = 0;
    bool ret = false;
    abc_t Iabc = {0};
    alphabeta_t I_alphabeta = {0};

    /*更新电流*/
    Iabc = current->Iabc;
    /*克拉克变换*/
    I_alphabeta = FOC_Clarke(Iabc);

    // elec_angle = 0;
    // elec_angle = angle_get(Uqd, pwm->Debug_Ualphabeta, I_alphabeta, angle_gen, Ts);
    // Iqd = Foc_Park(I_alphabeta, elec_angle);

    switch (iden_state){
        case 0:
            if(++cnt>=(1.0f/Ts)){
                cnt = 0;
                iden_state = 1;
            }
            Uqd.q = 0;
            Uqd.d = 0;
            break;
        case 1:
            elec_angle = 0;
            Iqd = Foc_Park(I_alphabeta, elec_angle);
            if(true == offline_iden->RsIden(4, Uqd, Iqd, 1000, Ts)){
                iden_state = 2;
            }
            break;
        case 2:
            elec_angle = angle_get(Uqd, pwm->Debug_Ualphabeta, I_alphabeta, angle_gen, Ts);
            Iqd = Foc_Park(I_alphabeta, elec_angle);
            if(true == offline_iden->LsIden(Uqd, Iqd, 4, 1000, Ts)){
                iden_state = 3;
            }
            break;
        case 3:
            printf("\r\n*****电阻电感辨识完成*******\r\n");
            printf("辨识电流: %.5fA\r\n", offline_iden->Is_max);
            printf("辨识电压: %.5fV\r\n", offline_iden->Vs_max);
            printf("Rs:     %.5f mΩ\r\n", offline_iden->Rs*1000.0f);
            printf("Lq:     %.5f uH\r\n", offline_iden->Lq*1000.0f*1000.0f);
            printf("Ld:     %.5f uH\r\n", offline_iden->Ld*1000.0f*1000.0f);
            printf("Ls:     %.5f uH\r\n", offline_iden->Ls*1000.0f*1000.0f);
            printf("***********************\r\n");
            Uqd.q = 0;
            Uqd.d = 0;
            iden_state = 0;
            ret = true;
            break;
        default:
            break;
    }

    pwm->run(Uqd, elec_angle, vbus, SVPWM_MODE);
    return ret;
}

/*磁链辨识*/
bool foc_motor::iden_run_f(void)
{
    bool ret = false;

    switch (iden_state){
        case 0:                         /*初始化*/
            target_elec_speed = max_elec_speed/4;

            /*加速到一定的速度，或者用滑模辨识的时候已经切换到了滑模状态*/
            if(speed_data.elec_speed>target_elec_speed*0.5f){ 
                iden_state = 1;
            }
            break;
        case 1:
            if(true == offline_iden->FsIden(Uqd, Iqd, target_elec_speed, 500, Ts)){
                iden_state = 2;
            }
            break;
        case 2:
            printf("\r\n*****磁链辨识完成*******\r\n");
            printf("辨识速度区间: %.2frad/s ~ %.2frad/s\r\n", offline_iden->elec_speed_l, offline_iden->elec_speed_h);
            printf("Flux:   %.5f mWb\r\n", offline_iden->flux*1000.0f);
            printf("***********************\r\n");
            target_elec_speed = 0;
            Uqd.q = 0;
            Uqd.d = 0;

            iden_state = 0;
            ret = true;
            break;
        default:
            break;
    }

    /*滑模运行辨识*/
    // vf_smo_run();
    /*编码器运行辨识*/

    angle_gen = ANGLE_GEN_ABZENC;
    speed_torque_run();

    return ret;
}

/*编码器校准*/
extern bool trigger_z;                     /*Z相触发标志*/
extern uint32_t z_over_cnt;                /*Z相触发次数*/
bool foc_motor::abz_enc_calib(void)
{
    bool ret = false;
    float elec_angle = 0;

    /*强拖D轴*/
    vf->SetElecSpeed(Uqd, 50, Ts);
    elec_angle = vf->ElecAngleCalc(Uqd, Ts);
    Uqd.q = 0;
    Uqd.d = vf->Uout;

    switch (enc_calib_state){
        case 0:
            if(true==trigger_z){
                enc_calib_state = 1;
            }
            cnt = 0;
            break;
        case 1:                 /*校准安装方向*/
            if(true==abzenc->DirCalib(Ts)){
                enc_calib_state = 2;
            }
            break;
        case 2:                 /*强拖到0°左右*/
            if(fabsf(elec_angle)<(vf->step)){
                enc_calib_state = 3;
            }
            break;
        case 3:                 /*校准角度*/
            ret = abzenc->ElecAngleCalib(Uqd, Ts);
            Uqd.d *= 2;
            elec_angle = 0;
            break;
        default:
            enc_calib_state = 0;
            break;
    }

    pwm->run(Uqd, elec_angle, vbus, pwm_mode);

    if(true==ret){
        enc_calib_state = 0;

    }

    return ret;
}

/*编码器校准*/
bool foc_motor::abs_enc_calib(void)
{
    bool ret = false;
    float elec_angle;

    /*强拖D轴*/
    vf->SetElecSpeed(Uqd, 10, Ts);
    elec_angle = vf->ElecAngleCalc(Uqd, Ts);
    Uqd.q = 0;
    Uqd.d = vf->Uout;

    switch (enc_calib_state){
        case 0:                 
            enc_calib_state = 1;
            cnt = 0;
            break;
        case 1:                 /*校准安装方向*/
            if(true==absenc->DirCalib(Ts)){
                enc_calib_state = 2;
            }
            break;
        case 2:                 /*强拖到0°左右*/
            if(fabsf(elec_angle)<(vf->step*2)){
                enc_calib_state = 3;
            }
            break;
        case 3:                 /*校准角度*/
            ret = absenc->ElecAngleCalib(Uqd, Ts);
            Uqd.d *= 2;
            elec_angle = 0;
            break;
        default:
            enc_calib_state = 0;
            break;
    }

    pwm->run(Uqd, elec_angle, vbus, pwm_mode);

    if(true==ret){
        enc_calib_state = 0;
    }

    return ret;
}

/*开环电压运行*/
inline void foc_motor::open_volt_run(void)
{
    float elec_angle = 0;
    abc_t Iabc = {0};
    alphabeta_t I_alphabeta = {0};

    /*更新电流*/
    Iabc = current->Iabc;
    /*克拉克变换*/
    I_alphabeta = FOC_Clarke(Iabc);

    elec_angle = angle_get(Uqd, pwm->Debug_Ualphabeta, I_alphabeta, angle_gen, Ts);

    Iqd = Foc_Park(I_alphabeta, elec_angle);

    Uqd.d = Vd_bias;
    
    // pwm->run(Uqd, elec_angle, vbus, pwm_mode);
    pwm->run(Uqd, elec_angle, Iabc, vbus, pwm_mode, Ts);
}

/*力矩环运行*/
inline void foc_motor::torque_run(void)
{
    float elec_angle = 0;
    abc_t Iabc = {0};
    alphabeta_t I_alphabeta = {0};

    /*更新电流*/
    Iabc = current->Iabc;
    /*克拉克变换*/
    I_alphabeta = FOC_Clarke(Iabc);

    elec_angle = angle_get(Uqd, pwm->Debug_Ualphabeta, I_alphabeta, angle_gen, Ts);

    Iqd = Foc_Park(I_alphabeta, elec_angle);

    switch (angle_gen){
        case ANGLE_GEN_ABZENC:
        case ANGLE_GEN_ABSENC:
        case ANGLE_GEN_FLUX:
        case ANGLE_GEN_SMO:
            Uqd.q = pid_iq->pi(Iqd_ref.q, Iqd.q, Ts);
            Uqd.d = pid_id->pi(Iqd_ref.d, Iqd.d, Ts)+Vd_bias;
            break;
        default:
            break;
    }

    pwm->run(Uqd, elec_angle, vbus, pwm_mode);
}

/*速度环-力矩环运行*/
inline void foc_motor::speed_torque_run(void)
{

    float elec_angle = 0;
    abc_t Iabc = {0};
    alphabeta_t I_alphabeta = {0};

    /*更新电流*/
    Iabc = current->Iabc;
    /*克拉克变换*/
    I_alphabeta = FOC_Clarke(Iabc);

    elec_angle = angle_get(Uqd, pwm->Debug_Ualphabeta, I_alphabeta, angle_gen, Ts);

    Iqd = Foc_Park(I_alphabeta, elec_angle);

    /*速度环*/
    Iqd_ref = spd_ctrl->pid();

    Vd_bias = 0;
    switch (angle_gen){
        case ANGLE_GEN_ABZENC:
        case ANGLE_GEN_ABSENC:
        case ANGLE_GEN_FLUX:
        case ANGLE_GEN_SMO:
            Uqd.q = pid_iq->pi(Iqd_ref.q, Iqd.q, Ts);
            Uqd.d = pid_id->pi(Iqd_ref.d, Iqd.d, Ts)+Vd_bias;
            break;
        default:
            break;
    }
    
    pwm->run(Uqd, elec_angle, vbus, pwm_mode);
}

/*位置环-电压运行*/
inline void foc_motor::pos_volt_run(void)
{
    float elec_angle = 0;
    abc_t Iabc = {0};
    alphabeta_t I_alphabeta = {0};

    /*更新电流*/
    Iabc = current->Iabc;
    /*克拉克变换*/
    I_alphabeta = FOC_Clarke(Iabc);

    elec_angle = angle_get(Uqd, pwm->Debug_Ualphabeta, I_alphabeta, angle_gen, Ts);

    Iqd = Foc_Park(I_alphabeta, elec_angle);

    /*位置环*/
    Iqd_ref.q = pos_ctrl->pid();

    switch (angle_gen){
        case ANGLE_GEN_ABZENC:
        case ANGLE_GEN_ABSENC:
        case ANGLE_GEN_FLUX:
        case ANGLE_GEN_SMO:
            Uqd.q = Iqd_ref.q;
            Uqd.d = Iqd_ref.d;
            break;
        default:
            break;
    }
    
    pwm->run(Uqd, elec_angle, vbus, pwm_mode); 
}

/*位置环-力矩环运行*/
inline void foc_motor::pos_torque_run(void)
{
    float elec_angle = 0;
    abc_t Iabc = {0};
    alphabeta_t I_alphabeta = {0};

    /*更新电流*/
    Iabc = current->Iabc;
    /*克拉克变换*/
    I_alphabeta = FOC_Clarke(Iabc);

    elec_angle = angle_get(Uqd, pwm->Debug_Ualphabeta, I_alphabeta, angle_gen, Ts);

    Iqd = Foc_Park(I_alphabeta, elec_angle);

    /*位置环*/
    Iqd_ref.q = pos_ctrl->pid();

    switch (angle_gen){
        case ANGLE_GEN_ABZENC:
        case ANGLE_GEN_ABSENC:
        case ANGLE_GEN_FLUX:
        case ANGLE_GEN_SMO:
            Uqd.q = pid_iq->pi(Iqd_ref.q, Iqd.q, Ts);
            Uqd.d = pid_id->pi(Iqd_ref.d, Iqd.d, Ts)+Vd_bias;
            break;
        default:
            break;
    }
    
    pwm->run(Uqd, elec_angle, vbus, pwm_mode); 
}

/*位置环-速度环-力矩环运行*/
inline void foc_motor::pos_speed_torque_run(void)
{

    float elec_angle = 0;
    abc_t Iabc = {0};
    alphabeta_t I_alphabeta = {0};

    /*更新电流*/
    Iabc = current->Iabc;
    /*克拉克变换*/
    I_alphabeta = FOC_Clarke(Iabc);

    elec_angle = angle_get(Uqd, pwm->Debug_Ualphabeta, I_alphabeta, angle_gen, Ts);

    Iqd = Foc_Park(I_alphabeta, elec_angle);

    /*位置环*/
    target_elec_speed = pos_ctrl->pid();

    /*速度环*/
    Iqd_ref = spd_ctrl->pid();

    switch (angle_gen){
        case ANGLE_GEN_ABZENC:
        case ANGLE_GEN_ABSENC:
        case ANGLE_GEN_FLUX:
        case ANGLE_GEN_SMO:
            Uqd.q = pid_iq->pi(Iqd_ref.q, Iqd.q, Ts);
            Uqd.d = pid_id->pi(Iqd_ref.d, Iqd.d, Ts)+Vd_bias;
            break;
        default:
            break;
    }
    
    pwm->run(Uqd, elec_angle, vbus, pwm_mode);
}

/*角度获取*/
inline float foc_motor::angle_get(qd_t &Uqd, alphabeta_t &Ualphabeta, alphabeta_t &I_alphabeta, ANGLE_GEN_t angle_gen, float Ts)
{
    float elec_angle = 0;
    Vd_bias = 0;
    
    switch (angle_gen){
        case ANGLE_GEN_VF:              /*VF建议只在开环情况下运行*/
            elec_angle = vf->ElecAngleCalc(Uqd, Ts);
            speed_data = vf->spd->speed_data;
            vf->SetElecSpeed(Uqd, target_elec_speed, Ts);

            absenc->ElecAngleCalc(Ts);
            abzenc->ElecAngleCalc(Ts);
            smo->ElecAngleCalc(Ualphabeta, I_alphabeta, Ts);
            flux->ElecAngleCalc(Ualphabeta, I_alphabeta, Ts);
            Uqd.q = vf->Uout;
            break;
        case ANGLE_GEN_ABZENC:
            elec_angle = abzenc->ElecAngleCalc(Ts);
            speed_data = abzenc->spd->speed_data;
        
            if(false==trigger_z){
                elec_angle = vf->ElecAngleCalc(Uqd, Ts);
                vf->SetElecSpeed(Uqd, 50, Ts);
            }else{
                smo->ElecAngleCalc(Ualphabeta, I_alphabeta, Ts);
                flux->ElecAngleCalc(Ualphabeta, I_alphabeta, Ts);
            }

            break;
        case ANGLE_GEN_ABSENC:
            elec_angle = absenc->ElecAngleCalc(Ts);
            speed_data = absenc->spd->speed_data;
            break;
        case ANGLE_GEN_FLUX:
            elec_angle = flux->ElecAngleCalc(Ualphabeta, I_alphabeta, Ts);
            speed_data = flux->spd->speed_data;

            smo->ElecAngleCalc(Ualphabeta, I_alphabeta, Ts);
            abzenc->ElecAngleCalc(Ts);

            if(fabsf(speed_data.elec_speed)<500){
                // Vd_bias = 0.5f;
            }


            break;
        case ANGLE_GEN_SMO:
            elec_angle = smo->ElecAngleCalc(Ualphabeta, I_alphabeta, Ts);
            speed_data = smo->spd->speed_data;
            flux->ElecAngleCalc(Ualphabeta, I_alphabeta, Ts);
            abzenc->ElecAngleCalc(Ts);
            break;
        case ANGLE_GEN_HFI:
            elec_angle = hfi->ElecAngleCalc(I_alphabeta, Ts);
            I_alphabeta = hfi->alphabeta_f;
            Vd_bias = hfi->hfi_vol;
            speed_data = hfi->spd->speed_data;
           break;        
        default:
            break;
    }

    return elec_angle;
}

void foc_motor::clear(void)
{
    memset(&speed_data, 0x00, sizeof(speed_data));
    memset(&debug, 0x00, sizeof(debug));

    cnt = 0;
    iden_state = 0;
    enc_calib_state = 0;

    iden_state        = 0;
    enc_calib_state   = 0;
    spd_cnt           = 0;
    pos_cnt           = 0;
    Vd_bias           = 0;

    pid_iq->clear();
    pid_id->clear();
    spd_ctrl->clear();
    pos_ctrl->clear();
    smo->clear();
    flux->clear();
    time->clear();
}

void foc_motor::run(void)
{
    time->clear();

    switch(this->m_mode){
        case MODE_IDLE:                                                                             /*空闲状态*/
            FOC_PWM_HW_ON_OFF(false, false, false, false, false, false);                            /*关闭所有PWM通道*/
            // printf("IDLE\r\n");   
            // this->m_mode = MODE_IDLE_START;
            break;
        case MODE_IDLE_START:                                   
            FOC_PWM_HW_ON_OFF(true, true, true, true, true, true);                                  /*打开所有PWM通道，电容充电等待*/ 
            if(++cnt>1000){
                cnt = 0;
                this->m_mode = MODE_CURRENT_CALIB;
                printf("MODE_CHARGE\r\n"); 
            }
            break;
        case MODE_CURRENT_CALIB:                                                                    /*电流校准*/
            if(0==this->current->offset_calc()){
                this->m_mode = MODE_CLEAR;
                printf("IDLE_START\r\n"); 
            }
            break;
        case MODE_CLEAR:
            if((this->angle_gen==ANGLE_GEN_ABZENC) && trigger_z==false){
                open_volt_run();
            }else{
                clear();                            
                this->m_mode = MODE_START;                                                          /*跳转到启动*/
                printf("CLEAR\r\n"); 
            }
            break;
        
        case MODE_START:                                                                            /*****************在这里选择********************* */
            this->m_mode = MODE_ABZENC_CALIB;                                                         
            printf("START\r\n");    
            break;
        case MODE_IDEN_RL:
            if(true==iden_run_rl()){
                this->m_mode = MODE_IDLE;
            }
            break;
        case MODE_IDEN_F:
            if(true==iden_run_f()){
                this->m_mode = MODE_IDLE;
            }
            break;
    
        case MODE_ABZENC_CALIB:
            if(true==abz_enc_calib()){
                this->angle_gen = ANGLE_GEN_ABZENC;
                this->m_mode = MODE_POSITION_SPEED_TORQUE_RUN;
            }
            break;
        case MODE_ABSENC_CALIB:
            if(true==abs_enc_calib()){
                this->angle_gen = ANGLE_GEN_ABSENC;
                this->m_mode = MODE_POSITION_SPEED_TORQUE_RUN;
            }
            break;
            /*开环电压运行*/
        case MODE_OPEN_VOLT_RUN:
            open_volt_run();
            break;
        case MODE_TORQUE_RUN:
            torque_run();
            break;
        case MODE_SPEED_TORQUE_RUN:
            speed_torque_run();
            break;
        
        case MODE_POSITION_VOLT_RUN:
            pos_volt_run();
            break;
        case MODE_POSITION_TORQUE_RUN:
            pos_torque_run();
            break;
        case MODE_POSITION_SPEED_TORQUE_RUN:
            pos_speed_torque_run();
            break; 
    }

    

    /*保存调试信息*/
    this->debug.Ualphabeta  = pwm->Debug_Ualphabeta;
    this->debug.Uabc        = pwm->Debug_Uabc;
    this->debug.Tabc        = pwm->Debug_Tabc;
    this->debug.Iabc        = current->Iabc;
    this->debug.Iadc_ad     = current->mcu_ad;
    this->debug.Iqd         = Iqd;

    this->debug.vf_speed        = vf->spd->speed_data;
    this->debug.abzenc_speed    = abzenc->spd->speed_data;
    this->debug.absenc_speed    = absenc->spd->speed_data;
    this->debug.smo_speed       = smo->spd->speed_data;
    this->debug.flux_speed      = flux->spd->speed_data;
    this->debug.hfi_speed       = hfi->spd->speed_data;

    Ts_test = time->get_interval_time();

    /*保护逻辑*/
    protect();
}

void foc_motor::protect(void)
{
    switch(this->m_mode){
        case MODE_START:
        case MODE_IDEN_RL:
        case MODE_IDEN_F:
        case MODE_ABZENC_CALIB:
        case MODE_ABSENC_CALIB:
        case MODE_OPEN_VOLT_RUN:
        case MODE_TORQUE_RUN:
        case MODE_SPEED_TORQUE_RUN:
        case MODE_POSITION_VOLT_RUN:
        case MODE_POSITION_TORQUE_RUN:
        case MODE_POSITION_SPEED_TORQUE_RUN:
            if(++protect_cnt>=1){
                if(true==current->protect(this->debug.Iqd)){
                    this->m_mode = MODE_IDLE;
                }
                protect_cnt = 0;
            }
            break;
    }    
}

/*计算所有模块的速度*/
void foc_motor::SpeedCalc(void)
{
    vf->ElecSpeedCalc(); 
    abzenc->ElecSpeedCalc();
    absenc->ElecSpeedCalc();
    smo->ElecSpeedCalc();
    flux->ElecSpeedCalc();
    hfi->ElecSpeedCalc();
}

/*速度控制*/
void foc_motor::SpeedCtrl(float rate)
{
    /*速度环分频*/
    if(++spd_cnt>=10){
        if(rate>1)rate = 1;
        if(rate<-1)rate = -1;

        switch(m_mode){
            case MODE_SPEED_TORQUE_RUN:
                target_elec_speed = rate*max_elec_speed;
            case MODE_IDEN_F:
            case MODE_POSITION_SPEED_TORQUE_RUN:
                spd_ctrl->run(target_elec_speed, speed_data.elec_speed, TRACK_MODE_NONE, 5.0, Ts*spd_cnt);
            default:
                break;
        }
        spd_cnt = 0;
    }

}

/*位置控制*/
void foc_motor::PossitionCtrl(float rate)
{
    /*在线仿真、电位器控制、或者通信协议控制，修改target_out_pos*/
    switch(m_mode){
        case MODE_POSITION_VOLT_RUN:
        case MODE_POSITION_TORQUE_RUN:
        case MODE_POSITION_SPEED_TORQUE_RUN:
            if(++pos_cnt>100){
                if(rate>0.9f)rate = 0.9f;
                if(rate<-0.9f)rate = -0.9f;
                target_out_pos = _2PI *rate; /*  */
                out_multi_pos = speed_data.out_angle + speed_data.out_round*_2PI;                   /*计算多圈值*/
                // pos_ctrl->run(target_out_pos, pos_test, TC_ROTATE_SHORTEST_PATH, Ts*pos_cnt);    
                pos_ctrl->run(target_out_pos, out_multi_pos, speed_data.out_angle, TC_ROTATE_SHORTEST_PATH, Ts*pos_cnt);
                pos_cnt = 0;
            }
            break;
        default:
            break;      
    }
}


#pragma arm section

