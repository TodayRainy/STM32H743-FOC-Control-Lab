#ifndef _FOC_POS_HPP
#define _FOC_POS_HPP

#include "foc_type.h"
#include "foc_pid.hpp"
#include "foc_math.hpp"

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
    /* data */
    uint8_t state;

    float goal_pos;                 /*目标位置*/

    TC_POS_MODE_t pos_mode;
    TC_ROTATE_MODE_t rotate_mode;

    foc_pid *pos_pid;
public:
    float out;

private:
    float difangle_calc(float new_pos, float old_pos, TC_POS_MODE_t pos_mode, TC_ROTATE_MODE_t rotate_mode);


public:
    foc_pos_ctrl(float kp, float ki, float kd, float err_sum_limit, float ref_limit, float out_limit, float Ts);
    ~foc_pos_ctrl();

    void run(float target_pos, float current_pos);
    float pid(void);
};
































#endif
