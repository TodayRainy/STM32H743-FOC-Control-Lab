#ifndef FOC_DTC_HPP
#define FOC_DTC_HPP

#include "foc_math.hpp"


class foc_dtc
{
private:
    /* data */
    float deadTime[3];
    float hysteresis;                   /*电流滞环阈值（安培），例如 0.1*/
    float v_ref[3];                     /*补偿电压参考值*/

    float mos_r;                        /*二极管电阻*/

    /*计算补偿电压*/
    

public:
    foc_dtc(float deadTime_A, float deadTime_B, float deadTime_C, float hysteresis);
    ~foc_dtc();

    abc_t dtc_calc_v(abc_t Uabc, abc_t Iabc, float Udc, float Ts);
    abc_t dtc_calc_Tpwm(abc_t Uabc, abc_t Iabc, float Tpwm, float Udc, float Ts);


};


#endif
