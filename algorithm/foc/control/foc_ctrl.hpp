/**
 ****************************************************************************************************
 * @file        foc_ctrl.hpp
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

#ifndef _FOC_CTRL_HPP
#define _FOC_CTRL_HPP

#include "foc_type.h"
#include "foc_pid.hpp"
#include "foc_filter.hpp"
#include "foc_math.hpp"

/*轨迹模式*/
typedef enum{
    TRACK_MODE_NONE = 0,        /*无轨迹*/
    TRACK_MODE_RAMP,            /*线性加减速*/
}TRACK_MODE_t;


class foc_track
{
private:
    uint8_t initialized;
    float phase_val;        /*阶段值*/

    inline float ramp(float new_obj, float val, float max_rate, float Ts);

public:
    foc_track();
    ~foc_track();

    float run(float new_obj, float val, float max_rate, float Ts, TRACK_MODE_t mode);
    void clear(void);
};




class foc_spd_ctrl
{
private:
    /* data */
    uint8_t state;
    float Ts;

    qd_t out;
    float target_ref;
    speed_t speed_data;
    foc_pid *spd_pid;
    foc_track *track;
    TRACK_MODE_t track_mode;
    float accel_limit;

public:
    foc_spd_ctrl(float kp, float ki, float ref_limit, float i_term_limit, float out_limit);
    ~foc_spd_ctrl();

    void run(float target_speed, float current_speed, float Ts);

    void set_gains(float kp, float ki);
    void set_track(TRACK_MODE_t mode, float accel_limit);
    void clear(void);

    qd_t pid(void);
    float reference(void) const;
};




/*位置模式*/
typedef enum{
    TC_POS_CONTINUOUS = 0,          /*连续位置*/
    TC_POS_WRAPPED,                 /*环绕位置*/
}TC_POS_MODE_t;

/*旋转模式*/
typedef enum{
    TC_ROTATE_INCREASE = 0,         /*顺时针旋转*/
    TC_ROTATE_DECREASE,             /*逆时针旋转*/
    TC_ROTATE_SHORTEST_PATH,        /*最短路径旋转*/
}TC_ROTATE_MODE_t;


class foc_pos_ctrl
{
private:
    float out;
    float target_ref;
    TC_POS_MODE_t pos_mode;         /*位置模式*/
    TC_ROTATE_MODE_t rotate_mode;   /*旋转模式*/
    foc_pid *pos_pid;
    foc_track *track;
    TRACK_MODE_t track_mode;
    float velocity_limit;
private:
    /*位置差计算*/
    float dif_calc(float ref_pos, float fb_multi_pos, float fb_single_pos, TC_ROTATE_MODE_t rotate_mode);
public:
    foc_pos_ctrl(float kp, float ki, float kd, float ref_limit, float i_term_limit, float out_limit,
                            TC_POS_MODE_t pos_mode, TC_ROTATE_MODE_t rotate_mode);
    ~foc_pos_ctrl();
    float run(float ref_pos, float fb_multi_pos, float fb_single_pos,TC_ROTATE_MODE_t rotate_mode, float Ts);
    void set_gains(float kp, float ki, float kd);
    void set_track(TRACK_MODE_t mode, float velocity_limit);
    void clear(void);

    float reference(void) const;

    float pid(void){
        return out;
    }

};















































#endif


