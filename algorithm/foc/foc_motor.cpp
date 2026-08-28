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

#pragma arm section code  = "RAMCODE"



#define IDEN_CURRENT_ABORT_A        5.0f
#define IDEN_SPEED_ABORT_SCALE      1.25f

FOC_OPERATION_t foc_motor::operation_from_mode(MOTOR_MODE_t mode)
{
    switch(mode){
        case MODE_IDEN_RL: return FOC_OPERATION_IDENTIFY_RL;
        case MODE_IDEN_F: return FOC_OPERATION_IDENTIFY_FLUX;
        case MODE_OPEN_VOLT_RUN: return FOC_OPERATION_OPEN_VOLTAGE;
        case MODE_TORQUE_RUN: return FOC_OPERATION_CURRENT;
        case MODE_SPEED_TORQUE_RUN: return FOC_OPERATION_SPEED;
        case MODE_POSITION_VOLT_RUN: return FOC_OPERATION_POSITION_VOLTAGE;
        case MODE_POSITION_TORQUE_RUN: return FOC_OPERATION_POSITION_CURRENT;
        case MODE_POSITION_SPEED_TORQUE_RUN: return FOC_OPERATION_POSITION_SPEED;
        default: return FOC_OPERATION_NONE;
    }
}

MOTOR_MODE_t foc_motor::mode_from_operation(FOC_OPERATION_t operation)
{
    switch(operation){
        case FOC_OPERATION_IDENTIFY_RL: return MODE_IDEN_RL;
        case FOC_OPERATION_IDENTIFY_FLUX: return MODE_IDEN_F;
        case FOC_OPERATION_OPEN_VOLTAGE: return MODE_OPEN_VOLT_RUN;
        case FOC_OPERATION_CURRENT: return MODE_TORQUE_RUN;
        case FOC_OPERATION_SPEED: return MODE_SPEED_TORQUE_RUN;
        case FOC_OPERATION_POSITION_VOLTAGE: return MODE_POSITION_VOLT_RUN;
        case FOC_OPERATION_POSITION_CURRENT: return MODE_POSITION_TORQUE_RUN;
        case FOC_OPERATION_POSITION_SPEED: return MODE_POSITION_SPEED_TORQUE_RUN;
        case FOC_OPERATION_NONE:
        default: return MODE_READY;
    }
}

bool foc_motor::speed_operation_running(void) const
{
    return (lifecycle == FOC_LIFECYCLE_RUNNING)
        && (operation == FOC_OPERATION_SPEED);
}

foc_motor::foc_motor(qd_t Uqd, float max_elec_speed, float vbus, float Ts,
                        MOTOR_MODE_t m_mode, PWM_MODE_t pwm_mode, ANGLE_GEN_t angle_gen)
{
    this->Uqd               = Uqd;
    this->max_elec_speed    = max_elec_speed;
    this->vbus              = vbus;
    this->Ts                = Ts;
    this->operation         = operation_from_mode(m_mode);
    this->requested_operation = FOC_OPERATION_POSITION_SPEED;
    this->startup_phase     = FOC_STARTUP_POWER_STAGE_CHARGE;
    if(m_mode == MODE_CURRENT_CALIB){
        this->lifecycle = FOC_LIFECYCLE_CURRENT_CALIBRATION;
    }else if(m_mode == MODE_READY){
        this->lifecycle = FOC_LIFECYCLE_READY;
    }else if(m_mode == MODE_CURRENT_CALIB_FAILED){
        this->lifecycle = FOC_LIFECYCLE_CALIBRATION_FAILED;
    }else if(this->operation != FOC_OPERATION_NONE){
        this->lifecycle = FOC_LIFECYCLE_RUNNING;
    }else{
        this->lifecycle = FOC_LIFECYCLE_IDLE;
    }
    this->pwm_mode          = pwm_mode;
    this->angle_gen         = angle_gen;

    this->iden_state        = 0;
    this->enc_calib_state   = 0;
    this->spd_cnt           = 0;
    this->pos_cnt           = 0;
    this->Vd_bias           = 0;
    this->current_feedforward.q = 0;
    this->current_feedforward.d = 0;
    this->motor_ld          = 0;
    this->motor_lq          = 0;
    this->motor_flux        = 0;
    this->speed_id_reference = 0;
    this->speed_current_limit = 0;
    this->voltage_request = 0;
    this->sensorless_start_direction = 1.0f;
    this->hfi_injection.q = 0.0f;
    this->hfi_injection.d = 0.0f;
    this->diagnostic_mode = FOC_DIAGNOSTIC_NONE;
    this->hfi_request = HFI_REQUEST_NONE;
    this->hfi_request_voltage = 0.0f;
    this->hfi_request_initial_offset = 0.0f;
    memset(&this->field_weakening_output, 0x00,
           sizeof(this->field_weakening_output));
    memset(&this->sensorless_output, 0x00,
           sizeof(this->sensorless_output));
    this->current_feedforward_mode = (uint32_t)CURRENT_FF_MODE_OFF;
    this->encoder_calibrated = false;

    this->target_elec_speed     = 0;
    this->target_out_pos = 0;

    /*补齐原本漏初始化的成员: C++ 不会自动清零非静态成员,
      这些变量原先沉底为 new 出来的堆内存残留值*/
    this->Ibus              = 0;
    this->out_multi_pos     = 0;
    this->cnt               = 0;
    this->Ts_test           = 0;

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

    /* State 0 holds zero voltage for one second after encoder calibration.
     * Enable the identification-specific limit only after that settling time. */
    if((iden_state != 0U)
       && (hypotf(I_alphabeta.alpha, I_alphabeta.beta) > IDEN_CURRENT_ABORT_A)){
        offline_iden->Rs_rmse = -1.0f;
        offline_iden->Rs_holdout_rmse = -1.0f;
        offline_iden->Ld_current_rms = -1.0f;
        offline_iden->Lq_current_rms = -1.0f;
        offline_iden->Ld_amplitude_delta = -1.0f;
        offline_iden->Lq_amplitude_delta = -1.0f;
        Uqd.q = 0.0f;
        Uqd.d = 0.0f;
        iden_state = 0U;
        return true;
    }

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
            /* Use the measured rotor d axis so signed DC current produces
             * negligible torque on a freely rotating motor. */
            elec_angle = angle_get(Uqd, pwm->Debug_Ualphabeta,
                                   I_alphabeta, angle_gen, Ts);
            Iqd = Foc_Park(I_alphabeta, elec_angle);
            if(true == offline_iden->RsIden(Uqd, Iqd, 1000, Ts)){
                iden_state = (offline_iden->Rs_rmse >= 0.0f) ? 2 : 4;
            }
            break;
        case 2:
            elec_angle = angle_get(Uqd, pwm->Debug_Ualphabeta, I_alphabeta, angle_gen, Ts);
            Iqd = Foc_Park(I_alphabeta, elec_angle);
            /* 40 samples per half period gives 500 Hz injection at 40 kHz FOC. */
            if(true == offline_iden->LsIden(Uqd, Iqd, 0.25f, 0.35f,
                                            40, 64, Ts)){
                iden_state = 3;
            }
            break;
        case 3:
            Uqd.q = 0;
            Uqd.d = 0;
            iden_state = 0;
            ret = true;
            break;
        case 4:                         /*Rs拟合失败，安全结束本次辨识*/
            Uqd.q = 0;
            Uqd.d = 0;
            iden_state = 0;
            ret = true;
            break;
        default:
            break;
    }

    Uqd = pwm->run(Uqd, elec_angle, vbus, SVPWM_MODE);
    return ret;
}

/*磁链辨识*/
bool foc_motor::iden_run_f(void)
{
    bool ret = false;

    if((hypotf(Iqd.q, Iqd.d) > IDEN_CURRENT_ABORT_A)
       || (fabsf(speed_data.elec_speed)
           > offline_iden->elec_speed_h * IDEN_SPEED_ABORT_SCALE)){
        offline_iden->flux_rmse = -1.0f;
        offline_iden->flux_holdout_rmse = -1.0f;
        target_elec_speed = 0.0f;
        Uqd.q = 0.0f;
        Uqd.d = 0.0f;
        iden_state = 0U;
        return true;
    }

    switch (iden_state){
        case 0:                         /*由辨识器统一管理所有正反转速度点*/
            target_elec_speed = 0.0f;
            iden_state = 1;
            break;
        case 1:
            if(true == offline_iden->FsIden(Uqd, Iqd, speed_data.elec_speed,
                                            target_elec_speed, 500.0f, Ts)){
                iden_state = 2;
            }
            break;
        case 2:
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

    Uqd = pwm->run(Uqd, elec_angle, vbus, pwm_mode);

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

    Uqd = pwm->run(Uqd, elec_angle, vbus, pwm_mode);

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
    
    Uqd = pwm->run(Uqd, elec_angle, Iabc, vbus, pwm_mode, Ts);
}

inline void foc_motor::current_loop_run(float elec_angle)
{
    const float pi_q = pid_iq->pi(Iqd_ref.q, Iqd.q, Ts);
    const float pi_d = pid_id->pi(Iqd_ref.d, Iqd.d, Ts);
    const uint32_t feedforward_mode = current_feedforward_mode;

    current_feedforward = FOC_CurrentFeedforward(
        feedforward_mode, speed_data.elec_speed,
        motor_ld, motor_lq, motor_flux, Iqd
    );

    Uqd.q = pi_q + current_feedforward.q + hfi_injection.q;
    Uqd.d = pi_d + Vd_bias + current_feedforward.d + hfi_injection.d;
    voltage_request = hypotf(Uqd.q, Uqd.d);

    Uqd = pwm->run(Uqd, elec_angle, vbus, pwm_mode);
    pid_iq->track_output(Uqd.q-current_feedforward.q-hfi_injection.q);
    pid_id->track_output(Uqd.d-Vd_bias-current_feedforward.d-hfi_injection.d);
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
            current_loop_run(elec_angle);
            break;
        default:
            Uqd = pwm->run(Uqd, elec_angle, vbus, pwm_mode);
            break;
    }
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

    /* I/f owns the current reference until the active-flux handoff ends. */
    if((angle_gen == ANGLE_GEN_SENSORLESS)
    && (sensorless_startup.closed_loop() == false)){
        Iqd_ref = sensorless_output.current_reference;
    }else{
        Iqd_ref = spd_ctrl->pid();
        Iqd_ref.d = speed_id_reference;
    }

    Vd_bias = 0;
    switch (angle_gen){
        case ANGLE_GEN_ABZENC:
        case ANGLE_GEN_ABSENC:
        case ANGLE_GEN_FLUX:
        case ANGLE_GEN_SMO:
        case ANGLE_GEN_HFI:
        case ANGLE_GEN_SENSORLESS:
            current_loop_run(elec_angle);
            break;
        default:
            Uqd = pwm->run(Uqd, elec_angle, vbus, pwm_mode);
            break;
    }
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
    
    Uqd = pwm->run(Uqd, elec_angle, vbus, pwm_mode);
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
            current_loop_run(elec_angle);
            break;
        default:
            Uqd = pwm->run(Uqd, elec_angle, vbus, pwm_mode);
            break;
    }
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
            current_loop_run(elec_angle);
            break;
        default:
            Uqd = pwm->run(Uqd, elec_angle, vbus, pwm_mode);
            break;
    }
}

/*角度获取*/
inline float foc_motor::angle_get(qd_t &Uqd, alphabeta_t &Ualphabeta, alphabeta_t &I_alphabeta, ANGLE_GEN_t angle_gen, float Ts)
{
    float elec_angle = 0;
    Vd_bias = 0;
    hfi_injection.q = 0.0f;
    hfi_injection.d = 0.0f;
    
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
                /*必须把 VF 算出的电压写回 Uqd。
                  SetElecSpeed/ElecAngleCalc 的 Uqd 形参是值传递,
                  它们内部对 Uqd 的修改传不出来;
                  不写回则 Uqd 一直停在构造时的 M0_DEFAULT_UQ(0.5V),
                  调制深度仅 1.5%, 转子拖不动只会来回抽搏,
                  Z 相永不触发, 状态机卡在 MODE_CLEAR。

                  注: 必须写 q 轴而不是 d 轴。调用方 open_volt_run()
                  在 angle_get() 返回后立即执行 Uqd.d = Vd_bias (=0),
                  写 d 轴会被立即覆盖成 0, 变成完全不发波。
                  而 ANGLE_GEN_VF 分支同样是写 Uqd.q = vf->Uout。*/
                Uqd.q = vf->Uout;
            }else{
                smo->ElecAngleCalc(Ualphabeta, I_alphabeta, Ts);
                flux->ElecAngleCalc(Ualphabeta, I_alphabeta, Ts);
                if(diagnostic_mode != FOC_DIAGNOSTIC_NONE){
                    hfi->ElecAngleCalc(I_alphabeta, Ualphabeta, Ts);
                    hfi_injection = Foc_Park(
                        hfi->InjectionVoltage(), elec_angle
                    );
                }
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
            elec_angle = hfi->ElecAngleCalc(
                I_alphabeta, Ualphabeta, Ts
            );
            hfi_injection = Foc_Park(
                hfi->InjectionVoltage(), elec_angle
            );
            speed_data = hfi->speed_output();
            abzenc->ElecAngleCalc(Ts);
            break;
        case ANGLE_GEN_SENSORLESS:
        {
            const float observer_angle = flux->ElecAngleCalc(
                Ualphabeta, I_alphabeta, Ts
            );
            const foc_sensorless_output_t observer = flux->output();
            smo->ElecAngleCalc(Ualphabeta, I_alphabeta, Ts);
            abzenc->ElecAngleCalc(Ts);
            sensorless_output = sensorless_startup.update(
                observer_angle, observer.speed, Ts
            );
            elec_angle = sensorless_output.control_angle;
            speed_data = flux->spd->speed_data;
            speed_data.elec_angle = elec_angle;
            speed_data.elec_speed = sensorless_output.control_speed;
            vf->spd->AngleRefresh(sensorless_output.virtual_angle);
            vf->spd->SpeedRefresh(sensorless_output.virtual_speed);
            break;
        }
        default:
            break;
    }

    return elec_angle;
}

void foc_motor::clear(void)
{
    memset(&speed_data, 0x00, sizeof(speed_data));
    memset(&debug, 0x00, sizeof(debug));
    diagnostic_mode = FOC_DIAGNOSTIC_NONE;

    cnt = 0;
    iden_state = 0;
    enc_calib_state = 0;

    iden_state        = 0;
    enc_calib_state   = 0;
    spd_cnt           = 0;
    pos_cnt           = 0;
    Vd_bias           = 0;
    hfi_injection.q = 0.0f;
    hfi_injection.d = 0.0f;
    current_feedforward.q = 0;
    current_feedforward.d = 0;
    speed_id_reference = 0;
    voltage_request = 0;
    field_weakening.clear();
    memset(&field_weakening_output, 0x00, sizeof(field_weakening_output));
    field_weakening_output.iq_limit = speed_current_limit;
    target_elec_speed = 0;
    Iqd_ref.q          = 0;
    Iqd_ref.d          = 0.00001f;

    pid_iq->clear();
    pid_id->clear();
    spd_ctrl->clear();
    spd_ctrl->set_output_limit(speed_current_limit);
    pos_ctrl->clear();
    if(angle_gen == ANGLE_GEN_ABZENC){
        (void)abzenc->ElecAngleCalc(Ts);
    }else if(angle_gen == ANGLE_GEN_ABSENC){
        (void)absenc->ElecAngleCalc(Ts);
    }
    abzenc->spd->clear();
    absenc->spd->clear();
    smo->clear();
    flux->clear();
    hfi->clear();
    sensorless_startup.clear();
    if(angle_gen == ANGLE_GEN_SENSORLESS){
        sensorless_startup.start(sensorless_start_direction);
        sensorless_output = sensorless_startup.get_output();
    }
    time->clear();
}

bool foc_motor::request_prepare(void)
{
    if(this->lifecycle != FOC_LIFECYCLE_IDLE){
        return false;
    }

    this->current->offset_reset();
    this->lifecycle = FOC_LIFECYCLE_CURRENT_CALIBRATION;
    return true;
}

bool foc_motor::request_start(MOTOR_MODE_t run_mode)
{
    if(this->lifecycle != FOC_LIFECYCLE_READY){
        return false;
    }

    const FOC_OPERATION_t next_operation = operation_from_mode(run_mode);
    if((next_operation != FOC_OPERATION_IDENTIFY_RL) &&
       (next_operation != FOC_OPERATION_IDENTIFY_FLUX) &&
       (next_operation != FOC_OPERATION_CURRENT) &&
       (next_operation != FOC_OPERATION_SPEED) &&
       (next_operation != FOC_OPERATION_POSITION_SPEED)){
        return false;
    }

    this->cnt = 0;
    this->requested_operation = next_operation;
    this->startup_phase = FOC_STARTUP_POWER_STAGE_CHARGE;
    this->lifecycle = FOC_LIFECYCLE_STARTING;
    return true;
}

bool foc_motor::request_sensorless_start(float direction)
{
    if((this->lifecycle != FOC_LIFECYCLE_READY) || (direction == 0.0f)){
        return false;
    }

    this->sensorless_start_direction = (direction < 0.0f) ? -1.0f : 1.0f;
    this->angle_gen = ANGLE_GEN_SENSORLESS;
    return request_start(MODE_SPEED_TORQUE_RUN);
}

bool foc_motor::request_stop(void)
{
    if(this->lifecycle == FOC_LIFECYCLE_CALIBRATION_FAILED){
        return false;
    }

    this->cnt = 0;
    this->lifecycle = FOC_LIFECYCLE_READY;
    this->operation = FOC_OPERATION_NONE;
    this->startup_phase = FOC_STARTUP_POWER_STAGE_CHARGE;
    this->enc_calib_state = 0;
    this->abzenc->reset_calibration();
    this->sensorless_startup.clear();
    this->hfi_request = HFI_REQUEST_FORCE_STOP;
    if((this->angle_gen == ANGLE_GEN_SENSORLESS)
    || (this->angle_gen == ANGLE_GEN_HFI)){
        this->angle_gen = ANGLE_GEN_ABZENC;
    }
    return true;
}

MOTOR_MODE_t foc_motor::mode_get(void) const
{
    switch(this->lifecycle){
        case FOC_LIFECYCLE_IDLE:
            return MODE_IDLE;
        case FOC_LIFECYCLE_CURRENT_CALIBRATION:
            return MODE_CURRENT_CALIB;
        case FOC_LIFECYCLE_READY:
            return MODE_READY;
        case FOC_LIFECYCLE_CALIBRATION_FAILED:
            return MODE_CURRENT_CALIB_FAILED;
        case FOC_LIFECYCLE_RUNNING:
            return mode_from_operation(this->operation);
        case FOC_LIFECYCLE_STARTING:
            switch(this->startup_phase){
                case FOC_STARTUP_POWER_STAGE_CHARGE:
                    return MODE_POWER_STAGE_CHARGE;
                case FOC_STARTUP_CLEAR_CONTROL:
                    return MODE_CLEAR;
                case FOC_STARTUP_SELECT_SOURCE:
                    return MODE_START;
                case FOC_STARTUP_CALIBRATE_ABZ:
                    return MODE_ABZENC_CALIB;
                case FOC_STARTUP_CALIBRATE_ABS:
                    return MODE_ABSENC_CALIB;
            }
            break;
    }
    return MODE_IDLE;
}

FOC_POWER_STAGE_t foc_motor::power_stage_get(void) const
{
    if(this->lifecycle == FOC_LIFECYCLE_RUNNING){
        return FOC_POWER_STAGE_ENABLED;
    }
    if(this->lifecycle == FOC_LIFECYCLE_STARTING){
        return (this->startup_phase == FOC_STARTUP_POWER_STAGE_CHARGE)
            ? FOC_POWER_STAGE_PRECHARGE : FOC_POWER_STAGE_ENABLED;
    }
    return FOC_POWER_STAGE_DISABLED;
}

void foc_motor::set_bus_voltage(float vbus)
{
    this->vbus = vbus;
}

void foc_motor::set_voltage_limit_ratio(float ratio)
{
    this->pwm->set_voltage_limit_ratio(ratio);
}

bool foc_motor::queue_hfi_request(HFI_REQUEST_t request,
                                  float injection_voltage,
                                  float initial_offset)
{
    if(hfi_request != HFI_REQUEST_NONE){
        return false;
    }

    hfi_request_voltage = injection_voltage;
    hfi_request_initial_offset = initial_offset;
    hfi_request = request;
    return true;
}

void foc_motor::apply_hfi_request(void)
{
    const HFI_REQUEST_t request = hfi_request;
    if(request == HFI_REQUEST_NONE){
        return;
    }

    const float injection_voltage = hfi_request_voltage;
    const float initial_offset = hfi_request_initial_offset;
    hfi_request = HFI_REQUEST_NONE;

    switch(request){
        case HFI_REQUEST_START_ENCODER_SHADOW:
            if(speed_operation_running()
            && (angle_gen == ANGLE_GEN_ABZENC)){
                hfi->start_encoder_calibrated_shadow(
                    abzenc->spd->speed_data.elec_angle,
                    initial_offset, injection_voltage
                );
                diagnostic_mode = FOC_DIAGNOSTIC_HFI_ENCODER_SHADOW;
            }
            break;
        case HFI_REQUEST_START_D_AXIS_SCAN:
            if(speed_operation_running()
            && (angle_gen == ANGLE_GEN_ABZENC)){
                hfi->start_d_axis_scan(
                    abzenc->spd->speed_data.elec_angle+initial_offset,
                    injection_voltage
                );
                diagnostic_mode = FOC_DIAGNOSTIC_HFI_D_AXIS_SCAN;
            }
            break;
        case HFI_REQUEST_START_TENSOR_SCAN:
            if(speed_operation_running()
            && (angle_gen == ANGLE_GEN_ABZENC)){
                hfi->start_tensor_scan(
                    abzenc->spd->speed_data.elec_angle+initial_offset,
                    injection_voltage
                );
                diagnostic_mode = FOC_DIAGNOSTIC_HFI_TENSOR_SCAN;
            }
            break;
        case HFI_REQUEST_ACTIVATE_TENSOR_CONTROL:
            if(speed_operation_running()
            && (angle_gen == ANGLE_GEN_ABZENC)
            && (diagnostic_mode != FOC_DIAGNOSTIC_NONE)
            && hfi->tracking()){
                diagnostic_mode = FOC_DIAGNOSTIC_NONE;
                angle_gen = ANGLE_GEN_HFI;
                speed_data = hfi->speed_output();
            }
            break;
        case HFI_REQUEST_STOP_SHADOW:
            if(angle_gen != ANGLE_GEN_HFI){
                diagnostic_mode = FOC_DIAGNOSTIC_NONE;
                hfi_injection.q = 0.0f;
                hfi_injection.d = 0.0f;
                hfi->stop();
            }
            break;
        case HFI_REQUEST_FORCE_STOP:
            diagnostic_mode = FOC_DIAGNOSTIC_NONE;
            hfi_injection.q = 0.0f;
            hfi_injection.d = 0.0f;
            hfi->stop();
            break;
        case HFI_REQUEST_NONE:
        default:
            break;
    }
}

bool foc_motor::start_hfi_shadow(float injection_voltage,
                                 float initial_offset)
{
    if(!speed_operation_running()
    || (angle_gen != ANGLE_GEN_ABZENC)){
        return false;
    }

    return queue_hfi_request(HFI_REQUEST_START_ENCODER_SHADOW,
                             injection_voltage, initial_offset);
}

bool foc_motor::start_hfi_characterization(float injection_voltage,
                                            float initial_offset)
{
    if(!speed_operation_running()
    || (angle_gen != ANGLE_GEN_ABZENC)){
        return false;
    }

    return queue_hfi_request(HFI_REQUEST_START_D_AXIS_SCAN,
                             injection_voltage, initial_offset);
}

bool foc_motor::start_hfi_tensor_characterization(
    float injection_voltage, float initial_offset)
{
    if(!speed_operation_running()
    || (angle_gen != ANGLE_GEN_ABZENC)){
        return false;
    }

    return queue_hfi_request(HFI_REQUEST_START_TENSOR_SCAN,
                             injection_voltage, initial_offset);
}

bool foc_motor::activate_hfi_control(void)
{
    if(!speed_operation_running()
    || (angle_gen != ANGLE_GEN_ABZENC)
    || (diagnostic_mode == FOC_DIAGNOSTIC_NONE) || !hfi->tracking()){
        return false;
    }

    return queue_hfi_request(HFI_REQUEST_ACTIVATE_TENSOR_CONTROL,
                             0.0f, 0.0f);
}

bool foc_motor::stop_hfi_shadow(void)
{
    if(angle_gen == ANGLE_GEN_HFI){
        return false;
    }

    return queue_hfi_request(HFI_REQUEST_STOP_SHADOW, 0.0f, 0.0f);
}

void foc_motor::configure_sensorless_startup(
    float align_current_a, float open_loop_current_a,
    float handoff_current_a, float current_taper_rate_a_s,
    float align_time_s, float acceleration_rad_s2,
    float handoff_speed_rad_s, float phase_gate_rad,
    float speed_gate_rad_s, float qualification_time_s,
    float blend_time_s, float qualification_timeout_s)
{
    sensorless_startup.configure(
        align_current_a, open_loop_current_a,
        handoff_current_a, current_taper_rate_a_s, align_time_s,
        acceleration_rad_s2, handoff_speed_rad_s, phase_gate_rad,
        speed_gate_rad_s, qualification_time_s, blend_time_s,
        qualification_timeout_s
    );
}

void foc_motor::configure_field_weakening(
    float rs, float ld, float lq, float flux, float kp, float ki,
    float current_limit, float target_utilization, float filter_alpha,
    float id_rate_limit)
{
    field_weakening.configure(
        rs, ld, lq, flux, kp, ki, current_limit,
        target_utilization, filter_alpha, id_rate_limit
    );
}

void foc_motor::set_field_weakening_mode(FIELD_WEAKENING_MODE_t mode)
{
    field_weakening.set_mode(mode);
    speed_id_reference = 0.0f;
}

FIELD_WEAKENING_MODE_t foc_motor::field_weakening_mode_get(void) const
{
    return field_weakening.mode_get();
}

void foc_motor::configure_current_feedforward(float ld, float lq, float flux)
{
    this->motor_ld = ld;
    this->motor_lq = lq;
    this->motor_flux = flux;
}

void foc_motor::enable_current_feedforward(bool enabled)
{
    set_current_feedforward_mode(
        enabled ? CURRENT_FF_MODE_BOTH : CURRENT_FF_MODE_OFF
    );
}

void foc_motor::set_current_feedforward_mode(CURRENT_FEEDFORWARD_MODE_t mode)
{
    this->current_feedforward_mode = (uint32_t)mode;
}

void foc_motor::set_speed_id_reference(float id)
{
    this->speed_id_reference = id;
}

void foc_motor::set_speed_current_limit(float limit)
{
    speed_current_limit = limit;
    spd_ctrl->set_output_limit(limit);
}

void foc_motor::set_current_reference(float iq, float id)
{
    this->Iqd_ref.q = iq;
    this->Iqd_ref.d = id;
}

void foc_motor::set_position_track(TRACK_MODE_t mode, float velocity_limit,
                                   float acceleration_limit)
{
    pos_ctrl->set_track(mode, velocity_limit, acceleration_limit);
}

void foc_motor::set_position_velocity_feedforward(float gain)
{
    pos_ctrl->set_velocity_feedforward(gain);
}

void foc_motor::set_speed_track(TRACK_MODE_t mode, float acceleration_limit)
{
    spd_ctrl->set_track(mode, acceleration_limit);
}

void foc_motor::set_speed_estimator_mode(foc_speed_estimator_mode_t mode)
{
    abzenc->spd->EstimatorModeSet(mode);
}

void foc_motor::set_speed_gains(float kp, float ki)
{
    spd_ctrl->set_gains(kp, ki);
}

void foc_motor::set_position_gains(float kp, float ki, float kd)
{
    pos_ctrl->set_gains(kp, ki, kd);
}

void foc_motor::set_position_output_limit(float limit)
{
    pos_ctrl->set_output_limit(limit);
}

void foc_motor::set_current_gains(bool q_axis, float kp, float ki)
{
    (q_axis ? pid_iq : pid_id)->set_gains(kp, ki, 0.0f);
}

void foc_motor::clear_control_loops(void)
{
    spd_ctrl->clear();
    pos_ctrl->clear();
    pid_iq->clear();
    pid_id->clear();
}

void foc_motor::identification_result_get(foc_iden_result_t &result) const
{
    offline_iden->ResultGet(result);
}

bool foc_motor::take_overcurrent_fault(qd_t *fault_current)
{
    return current->take_overcurrent_fault(fault_current);
}

void foc_motor::timing_clear(void)
{
    time->clear();
}

float foc_motor::output_position(void) const
{
    if(angle_gen == ANGLE_GEN_ABZENC){
        return abzenc->spd->speed_data.out_angle + abzenc->spd->speed_data.out_round*_2PI;
    }
    if(angle_gen == ANGLE_GEN_ABSENC){
        return absenc->spd->speed_data.out_angle + absenc->spd->speed_data.out_round*_2PI;
    }
    return speed_data.out_angle + speed_data.out_round*_2PI;
}

void foc_motor::run(void)
{
    time->clear();
    apply_hfi_request();

    switch(this->lifecycle){
        case FOC_LIFECYCLE_IDLE:
        case FOC_LIFECYCLE_READY:
        case FOC_LIFECYCLE_CALIBRATION_FAILED:
            break;

        case FOC_LIFECYCLE_CURRENT_CALIBRATION:
        {
            const CURRENT_CALIB_RESULT_t result = this->current->offset_calc();
            if(result == CURRENT_CALIB_FAILED){
                this->lifecycle = FOC_LIFECYCLE_CALIBRATION_FAILED;
            }else if(result == CURRENT_CALIB_DONE){
                this->lifecycle = FOC_LIFECYCLE_READY;
            }
            break;
        }

        case FOC_LIFECYCLE_STARTING:
            switch(this->startup_phase){
                case FOC_STARTUP_POWER_STAGE_CHARGE:
                    if(++cnt > 1000){
                        cnt = 0;
                        this->startup_phase = FOC_STARTUP_CLEAR_CONTROL;
                    }
                    break;

                case FOC_STARTUP_CLEAR_CONTROL:
                    if((this->angle_gen == ANGLE_GEN_ABZENC) && !trigger_z){
                        open_volt_run();
                    }else{
                        clear();
                        this->startup_phase = FOC_STARTUP_SELECT_SOURCE;
                    }
                    break;

                case FOC_STARTUP_SELECT_SOURCE:
                    if((angle_gen == ANGLE_GEN_SENSORLESS)
                    || encoder_calibrated){
                        this->operation = this->requested_operation;
                        this->lifecycle = FOC_LIFECYCLE_RUNNING;
                    }else{
                        this->startup_phase = (angle_gen == ANGLE_GEN_ABSENC)
                            ? FOC_STARTUP_CALIBRATE_ABS
                            : FOC_STARTUP_CALIBRATE_ABZ;
                    }
                    break;

                case FOC_STARTUP_CALIBRATE_ABZ:
                    if(abz_enc_calib()){
                        this->angle_gen = ANGLE_GEN_ABZENC;
                        this->encoder_calibrated = true;
                        this->operation = this->requested_operation;
                        this->lifecycle = FOC_LIFECYCLE_RUNNING;
                    }
                    break;

                case FOC_STARTUP_CALIBRATE_ABS:
                    if(abs_enc_calib()){
                        this->angle_gen = ANGLE_GEN_ABSENC;
                        this->encoder_calibrated = true;
                        this->operation = this->requested_operation;
                        this->lifecycle = FOC_LIFECYCLE_RUNNING;
                    }
                    break;
            }
            break;

        case FOC_LIFECYCLE_RUNNING:
            switch(this->operation){
                case FOC_OPERATION_IDENTIFY_RL:
                    if(iden_run_rl()){
                        this->operation = FOC_OPERATION_NONE;
                        this->lifecycle = FOC_LIFECYCLE_READY;
                    }
                    break;
                case FOC_OPERATION_IDENTIFY_FLUX:
                    if(iden_run_f()){
                        this->operation = FOC_OPERATION_NONE;
                        this->lifecycle = FOC_LIFECYCLE_READY;
                    }
                    break;
                case FOC_OPERATION_OPEN_VOLTAGE:
                    open_volt_run();
                    break;
                case FOC_OPERATION_CURRENT:
                    torque_run();
                    break;
                case FOC_OPERATION_SPEED:
                    speed_torque_run();
                    if((angle_gen == ANGLE_GEN_SENSORLESS)
                    && sensorless_startup.failed()){
                        this->operation = FOC_OPERATION_NONE;
                        this->lifecycle = FOC_LIFECYCLE_READY;
                        this->angle_gen = ANGLE_GEN_ABZENC;
                    }
                    break;
                case FOC_OPERATION_POSITION_VOLTAGE:
                    pos_volt_run();
                    break;
                case FOC_OPERATION_POSITION_CURRENT:
                    pos_torque_run();
                    break;
                case FOC_OPERATION_POSITION_SPEED:
                    pos_speed_torque_run();
                    break;
                case FOC_OPERATION_NONE:
                    this->lifecycle = FOC_LIFECYCLE_READY;
                    break;
            }
            break;
    }

    

    /*保存调试信息*/
    this->debug.Uqd         = Uqd;
    this->debug.Iqd_ref     = Iqd_ref;
    this->debug.target_elec_speed = target_elec_speed;
    this->debug.speed_ref   = spd_ctrl->reference();
    this->debug.elec_speed  = speed_data.elec_speed;
    this->debug.voltage_scale = pwm->voltage_scale;
    this->debug.motor_mode  = (uint32_t)mode_get();
    this->debug.vbus        = vbus;
    this->debug.target_position = target_out_pos;
    this->debug.position_ref = pos_ctrl->reference();
    this->debug.position_feedback = out_multi_pos;
    this->debug.current_feedforward = current_feedforward;
    this->debug.current_feedforward_mode = current_feedforward_mode;
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
    this->debug.hfi_speed       = hfi->speed_output();
    const foc_hfi_output_t hfi_output = hfi->output();
    this->debug.hfi_phase_error = hfi_output.phase_error;
    this->debug.hfi_d_response  = hfi_output.d_response;
    this->debug.hfi_q_response  = hfi_output.q_response;

    Ts_test = time->get_interval_time();

    /*保护逻辑*/
    protect();
}

void foc_motor::protect(void)
{
    const bool startup_control_active =
        (this->lifecycle == FOC_LIFECYCLE_STARTING)
        && ((this->startup_phase == FOC_STARTUP_SELECT_SOURCE)
         || (this->startup_phase == FOC_STARTUP_CALIBRATE_ABZ)
         || (this->startup_phase == FOC_STARTUP_CALIBRATE_ABS));
    if((this->lifecycle == FOC_LIFECYCLE_RUNNING)
    || startup_control_active){
        if(current->protect(this->debug.Iqd)){
            this->operation = FOC_OPERATION_NONE;
            this->lifecycle = FOC_LIFECYCLE_IDLE;
        }
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

        if(this->lifecycle == FOC_LIFECYCLE_RUNNING){
            switch(this->operation){
            case FOC_OPERATION_SPEED:
                target_elec_speed = rate*max_elec_speed;
                if((angle_gen == ANGLE_GEN_SENSORLESS)
                && (sensorless_startup.closed_loop() == false)){
                    field_weakening.clear();
                    speed_id_reference = 0.0f;
                    spd_ctrl->set_output_limit(speed_current_limit);
                    spd_ctrl->track_output(
                        sensorless_output.current_reference.q
                    );
                }else{
                    field_weakening_output = field_weakening.run(
                        voltage_request,
                        pwm->get_voltage_limit(vbus, pwm_mode),
                        speed_data.elec_speed, Iqd.q, Ts*spd_cnt
                    );
                    speed_id_reference = field_weakening_output.id_ref;
                    spd_ctrl->set_output_limit(
                        (field_weakening_output.iq_limit < speed_current_limit)
                        ? field_weakening_output.iq_limit : speed_current_limit
                    );
                    spd_ctrl->run(target_elec_speed, speed_data.elec_speed,
                                  Ts*spd_cnt);
                }
                break;
            case FOC_OPERATION_IDENTIFY_FLUX:
            case FOC_OPERATION_POSITION_SPEED:
                spd_ctrl->set_output_limit(speed_current_limit);
                spd_ctrl->run(target_elec_speed, speed_data.elec_speed, Ts*spd_cnt);
                break;
            default:
                break;
            }
        }
        spd_cnt = 0;
    }

}

/*位置控制*/
void foc_motor::PositionCtrl(float target_position)
{
    if(this->lifecycle != FOC_LIFECYCLE_RUNNING){
        return;
    }

    switch(this->operation){
        case FOC_OPERATION_POSITION_VOLTAGE:
        case FOC_OPERATION_POSITION_CURRENT:
        case FOC_OPERATION_POSITION_SPEED:
            if(++pos_cnt>=100){
                target_out_pos = target_position;
                out_multi_pos = output_position();
                pos_ctrl->run(target_out_pos, out_multi_pos, speed_data.out_angle, TC_ROTATE_SHORTEST_PATH, Ts*pos_cnt);
                pos_cnt = 0;
            }
            break;
        default:
            break;      
    }
}


#pragma arm section
