#include "foc_pos_ctrl.hpp"


foc_pos_ctrl::foc_pos_ctrl(float kp, float ki, float kd, float err_sum_limit, float ref_limit, float out_limit, float Ts)
{
    this->pos_mode      = TC_POS_CONTINUOUS;
    this->rotate_mode   = TC_ROTATE_SHORTEST_PATH;

    pos_pid = new foc_pid(kp, ki, kd, err_sum_limit, ref_limit, out_limit, Ts);
}

foc_pos_ctrl::~foc_pos_ctrl()
{
}

/*计算角度差值*/
float foc_pos_ctrl::difangle_calc(float new_pos, float old_pos, TC_POS_MODE_t pos_mode, TC_ROTATE_MODE_t rotate_mode)
{
    float dif = 0;

    switch(pos_mode){
        case TC_POS_CONTINUOUS:
            dif = new_pos - old_pos;
            break;
        case TC_POS_WRAPPED:
            switch (rotate_mode){
                case TC_ROTATE_INCREASE:
                    if(new_pos>=old_pos){
                        dif = new_pos-old_pos;
                    }else{
                        dif = new_pos + (_2PI-old_pos);
                    }
                    break;
                case TC_ROTATE_DECREASE:
                    if(new_pos<=old_pos){
                        dif = new_pos-old_pos;
                    }else{
                        dif = -(old_pos + (_2PI-new_pos));
                    }
                    break;
                case TC_ROTATE_SHORTEST_PATH:
                    dif = new_pos - old_pos;
                    if(dif>PI){
                        dif -= _2PI;
                    }else if(dif<-PI){
                        dif += _2PI;
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }

    return dif;
}

void foc_pos_ctrl::run(float new_pos, float old_pos)
{
    float dif = 0;

    if(new_pos!=goal_pos){
        goal_pos = new_pos;
        state = 0;
    }

    switch (state){
        case 0:
            /* code */
            state = 1;
            break;
        case 1:                 /*运行中*/
            dif = difangle_calc(new_pos, old_pos, TC_POS_WRAPPED, TC_ROTATE_SHORTEST_PATH);
            if(fabsf(dif)<0.5f){
                state = 2;
            }
            break;
        case 2:                 /*跟踪状态*/
            dif = difangle_calc(new_pos, old_pos, TC_POS_WRAPPED, TC_ROTATE_SHORTEST_PATH);
            break;
        default:
            break;
    }

    out = pos_pid->Position_Ctrl(new_pos, old_pos);
    // out = pos_pid->pi(dif);
    // out = pos_pid->Increment_Ctrl(target_pos, current_pos);
}

float foc_pos_ctrl::pid(void)
{
    return out;
}












