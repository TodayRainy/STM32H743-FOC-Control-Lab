#include "foc_dtc.hpp"




foc_dtc::foc_dtc(float deadTime_A, float deadTime_B, float deadTime_C, float hysteresis)
{
    this->deadTime[0] = deadTime_A;
    this->deadTime[1] = deadTime_B;
    this->deadTime[2] = deadTime_C;

    this->hysteresis = hysteresis;
}

foc_dtc::~foc_dtc()
{
}

/**
 * @brief 平均电压法死区补偿，带滞环电流判断
 * @param Uabc 期望输出电压（相电压，相对于直流母线负端），单位V
 * @param Iabc 三相电流，单位A
 * @param Udc  直流母线电压，单位V
 * @param Ts   PWM开关周期，单位s (例如 1/40000 = 25e-6)
 * @return 补偿后的电压
 */
abc_t foc_dtc::dtc_calc_v(abc_t Uabc, abc_t Iabc, float Udc, float Ts)
{
    float vcomp[3] = {0.0f};   // 补偿电压

    for (size_t i = 0; i < 3; i++){
        v_ref[i] = (deadTime[i] / Ts) * Udc;
    }

    vcomp[0] = (fabsf(Iabc.a) > hysteresis) ? ((Iabc.a > 0.0f) ? v_ref[0] : -v_ref[0]) : 0.0f;
    vcomp[1] = (fabsf(Iabc.b) > hysteresis) ? ((Iabc.b > 0.0f) ? v_ref[1] : -v_ref[1]) : 0.0f;
    vcomp[2] = (fabsf(Iabc.c) > hysteresis) ? ((Iabc.c > 0.0f) ? v_ref[2] : -v_ref[2]) : 0.0f;

    // 叠加补偿
    Uabc.a += vcomp[0];
    Uabc.b += vcomp[1];
    Uabc.c += vcomp[2];

    // 限幅到安全范围 [0, Udc]
    Uabc.a = LIMIT_RANGE(Uabc.a, 0.0f, Udc);
    Uabc.b = LIMIT_RANGE(Uabc.b, 0.0f, Udc);
    Uabc.c = LIMIT_RANGE(Uabc.c, 0.0f, Udc);

    return Uabc;
}

/*返回PWM值*/
abc_t foc_dtc::dtc_calc_Tpwm(abc_t Tabc, abc_t Iabc, float Tpwm, float Udc, float Ts)
{
    float vcomp[3] = {};                 /*输出补偿电压数组 [3]*/

    /*误差电压幅值 Vdead = (Tdead / Tpwm) * Vdc*/
    for (size_t i = 0; i < 3; i++)
    {
        v_ref[i] = (deadTime[i] / Ts) * Udc;
    }
     
    /*根据电流方向计算补偿电压（滞环区内补偿为0）*/
    vcomp[0] = (fabsf(Iabc.a) > hysteresis) ? ((Iabc.a > 0.0f) ? v_ref[0] : -v_ref[0]) : 0.0f;
    vcomp[1] = (fabsf(Iabc.b) > hysteresis) ? ((Iabc.b > 0.0f) ? v_ref[1] : -v_ref[1]) : 0.0f;
    vcomp[2] = (fabsf(Iabc.c) > hysteresis) ? ((Iabc.c > 0.0f) ? v_ref[2] : -v_ref[2]) : 0.0f;
    
    Tabc.a = LIMIT_RANGE(Tabc.a + vcomp[0]/Udc *Tpwm, 0, Tpwm);
    Tabc.b = LIMIT_RANGE(Tabc.b + vcomp[1]/Udc *Tpwm, 0, Tpwm);
    Tabc.c = LIMIT_RANGE(Tabc.c + vcomp[2]/Udc *Tpwm, 0, Tpwm);

    return Tabc;
}







