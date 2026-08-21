/**
 ****************************************************************************************************
 * @file        foc_pwm.cpp
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


#include "foc_pwm.hpp"
#include "foc_hw.hpp"
#include "foc_math.hpp"
#include "stdlib.h"


#pragma arm section code  = "RAMCODE"


foc_pwm::foc_pwm(uint32_t Tpwm)
{
    /*补齐漏初始化的成员: 对象由 new 分配, C++ 不会自动清零*/
    this->sector     = 0;
    this->svpwm_val1 = 0;
    this->svpwm_val2 = 0;
   
    this->ratio_max = 0;
    this->Tpwm      = Tpwm;
    this->Tpwm_half = Tpwm/2;

    Debug_Uabc.a = 12;
    Debug_Uabc.b = 12;
    Debug_Uabc.c = 12;

    voltage_drop.a = 0.0166666675f;
    voltage_drop.b = 0.0299999993f;
    voltage_drop.c = 0.0439999998f;

    dtc_state = 0;


    FOC_PWM_HW_Init();
}

foc_pwm::~foc_pwm()
{
    
    FOC_PWM_HW_DeInit();

}

/*
扇区判断条件

扇区Ⅰ:
    Uα>0, Uβ>0; Uβ/Uα<√3;           最终结果: √3Uα-Uβ>0
扇区Ⅱ: 
    Uα>0, Uβ>0; Uβ/Uα>√3;           最终结果: √3Uα-Uβ<0
    Uα<0, Uβ>0; Uβ/Uα<-√3;          最终结果: -√3Uα-Uβ<0
扇区Ⅲ: 
    Uα<0, Uβ>0; Uβ/Uα>-√3;          最终结果: -√3Uα-Uβ>0
扇区Ⅳ: 
    Uα<0, Uβ<0; Uβ/Uα<√3;           最终结果: √3Uα-Uβ<0
扇区Ⅴ: 
    Uα<0, Uβ<0; Uβ/Uα>√3;           最终结果: √3Uα-Uβ>0
    Uα>0, Uβ<0; Uβ/Uα<-√3;          最终结果: -√3Uα-Uβ>0;
扇区Ⅵ: 
    Ua>0, Uβ<0; Uβ/Uα>-√3;          最终结果: -√3Uα-Uβ<0;

计算权重系数
U1 = Uβ;        U1>0 A=1; U1<0 A=0;
U2 = √3Uα-Uβ;   U2>0 B=1; U2<0 B=0;
U3 = -√3Uα-Uβ;  U3>0 C=1; U3<0 C=0;

扇区编号与权重系数之间的关系
扇区Ⅰ: A=1; B=1; C=0;   N=3
扇区Ⅱ: A=1; B=0; C=0;   N=1
扇区Ⅲ: A=1; B=0; C=1;   N=5
扇区Ⅳ: A=0; B=0; C=1;   N=4                        
扇区Ⅴ: A=0; B=1; C=1;   N=6
扇区Ⅵ: A=0; B=1; C=0;   N=2

*/

/**
 * @brief 扇区判断
 * 
 * @param U_alphabeta   αβ电压
 * @return uint8_t      返回扇区号
 */
inline uint8_t foc_pwm::sector_get(alphabeta_t U_alphabeta)
{
#if 0
    float A = U_alphabeta.beta;
    float B = SQRT_3*U_alphabeta.alpha - U_alphabeta.beta;
    float C = -SQRT_3*U_alphabeta.alpha - U_alphabeta.beta;

#else
    this->svpwm_val1 = U_alphabeta.alpha *SQRT_3_DIV_2;
    this->svpwm_val2 = U_alphabeta.beta/2.0f;

    float A = U_alphabeta.beta;
    float B = svpwm_val1 - svpwm_val2;
    float C = -svpwm_val1 - svpwm_val2;
#endif


    uint8_t N=0;
    uint8_t sector = 0;

    /*计算权重值*/
    if(A>0)N +=1;
    if(B>0)N +=2;
    if(C>0)N +=4;

    /*扇区编号和权重值映射*/
    switch(N){
        case 3:
            sector = 1;
            break;
        case 1:
            sector = 2;
            break;
        case 5:
            sector = 3;
            break;
        case 4:
            sector = 4;
            break;
        case 6:
            sector = 5;
            break;
        case 2:
            sector = 6;
            break;
    }
    return sector;
}

/**
 * @brief 通过反正切来判断扇区
 * 
 * @param U_alphabeta αβ电压
 * @return uint8_t 返回扇区号
 */
inline uint8_t foc_pwm::sector_get_atan(alphabeta_t U_alphabeta)
{
    uint8_t sector = 1;

    /* 反正切计算矢量角度*/
    float elec_angle = Limit_Angle(atan2f(U_alphabeta.beta, U_alphabeta.alpha));

    if(elec_angle>=0 && elec_angle<_PI_3){
        sector = 1;
    }else if(elec_angle>=_PI_3 && elec_angle<(2*_PI_3)){
        sector = 2;
    }else if(elec_angle>=(2*_PI_3) && elec_angle<(3*_PI_3)){
        sector = 3;
    }else if(elec_angle>=(3*_PI_3) && elec_angle<(4*_PI_3)){
        sector = 4;
    }else if(elec_angle>=(4*_PI_3) && elec_angle<(5*_PI_3)){
        sector = 5;
    }else{
        sector = 6;
    }

    return sector;
}


/*
矢量时间推导

U1 = (T1/Ts)*(2/3)*Udc;
U2 = (T2/Ts)*(2/3)*Udc;



扇区Ⅰ   权重值:3    U1矢量:100  U2矢量:110      
Uα = U1 + U2*cos60°;
Uβ = U2*sin60°;
T1 = -((√3*Ts)/Udc) *(-(√3/2)*Uα+(1/2)*β)
T2 = ((√3*Ts)/Udc) *Uβ

扇区Ⅱ   权重值:1    U1矢量:010  U2矢量:110      
Uα = U2*cos60° - U1*cos60°
Uβ = U2*cos30° + U1*cos30°
T1 = ((√3*Ts)/Udc) *( -(√3/2)*Uα+(1/2)*β)
T2 = ((√3*Ts)/Udc) *((√3/2)*Uα+(1/2)*β)

扇区Ⅲ   权重值:5    U1矢量:010  U2矢量:011 
Uα = -U1*cos60° - U2
Uβ = U1*sin60°
T1 = ((√3*Ts)/Udc) *Uβ
T2 = -((√3*Ts)/Udc) *((√3/2)*Uα+(1/2)*β)

扇区Ⅳ   权重值:4    U1矢量:001  U2矢量:011 
Uα = -U2 - U1*cos60°
Uβ = -U1*sin60°
T1 = -((√3*Ts)/Udc) *Uβ 
T2 = ((√3*Ts)/Udc) *(-(√3/2)*Uα+(1/2)*β)


扇区Ⅴ   权重值:6    U1矢量:001  U2矢量:101        
Uα = -U1*cos60° + U2*cos60°
Uβ = -U1*sin60° - U2*sin60°
T1 = -((√3*Ts)/Udc) *((√3/2)*Uα+(1/2)*β)
T2 = -((√3*Ts)/Udc) *(-(√3/2)*Uα+(1/2)*β)

扇区Ⅵ   权重值:2    U1矢量:100  U2矢量:101 
Uα = U2*cos60° + U1
Uβ = -U2*sin60°
T1 = ((√3*Ts)/Udc) *((√3/2)*Uα+(1/2)*β)
T2 = -((√3*Ts)/Udc) *Uβ 

*/

/**
 * @brief 矢量时间计算
 * 
 * @param sector 扇区
 * @param alphabeta αβ轴电压
 * @param Tpwm PWM周期值
 * @param Udc 母线电压
 */
inline void foc_pwm::vector_time_calc(uint8_t sector, alphabeta_t alphabeta, float Tpwm, float Udc)
{
    // Udc = Udc*1.5f;      

    float tmp = (float)Tpwm * SQRT_3/Udc;
    
#if 0
    float X = tmp *(alphabeta.beta);
    float Y = tmp *(alphabeta.alpha *SQRT_3_DIV_2 + alphabeta.beta/2.0f);
    float Z = tmp *(-alphabeta.alpha *SQRT_3_DIV_2 + alphabeta.beta/2.0f);
#else

    float X = tmp *(alphabeta.beta);
    float Y = tmp *(svpwm_val1 + svpwm_val2);
    float Z = tmp *(-svpwm_val1 + svpwm_val2);
#endif

    
    float T0 = 0, T1 = 0, T2 = 0;            /*0矢量的时间，以及扇区的两个相邻矢量的时间*/

    abc_t Tabc_0 = {0};
    abc_t Tabc_1 = {0};

    /*根据扇区号，是扇区号，不是权重值，计算相邻矢量时间*/
    switch(sector){
        case 1:         /*权重值N:3*/
            T1 = -Z;
            T2 = X;
            break;  
        case 2:         /*权重值N:1*/
            T1 = Z;
            T2 = Y;
            break;
        case 3:         /*权重值N:5*/
            T1 = X;
            T2 = -Y;
            break;
        case 4:         /*权重值N:4*/
            T1 = -X;
            T2 = Z;
            break;
        case 5:         /*权重值N:6*/
            T1 = -Y;
            T2 = -Z;
            break;
        case 6:         /*权重值N:2*/
            T1 = Y;
            T2 = -X;
            break;
    }
    
    /*过调制处理*/
    if(T1+T2>Tpwm){
        T1 = T1/(T1+T2)*(float)Tpwm;
        T2 = T2/(T1+T2)*(float)Tpwm;
    }

    if( ((T1+T2)/Tpwm)>ratio_max){
        ratio_max = (T1+T2)/Tpwm;
    }


    T0 = Tpwm-T1-T2;                            /*计算零矢量时间*/

    //到这里相邻两个矢量在一个Tpwm周期时间内，各自的时间T1, T2已经计算出来了
    Tabc_0.a = T0/4.0f;                         /*零矢量的PWM滴答数*/
    Tabc_0.b = Tabc_0.a + T1/2.0f;              /*滴答值第二的矢量*/
    Tabc_0.c = Tabc_0.b + T2/2.0f;              /*滴答值第三的矢量*/


    /*用七段式SVPWM方式，根据扇区来分配矢量*/
    switch (sector){
        case 1:                 /*权重值N:3*/
            Tabc_1.a = Tabc_0.a;
            Tabc_1.b = Tabc_0.b;
            Tabc_1.c = Tabc_0.c;
            break;
        case 2:                 /*权重值N:1*/
            Tabc_1.a = Tabc_0.b;
            Tabc_1.b = Tabc_0.a;
            Tabc_1.c = Tabc_0.c;
            break;
        case 3:                 /*权重值N:5*/
            Tabc_1.a = Tabc_0.c;
            Tabc_1.b = Tabc_0.a;
            Tabc_1.c = Tabc_0.b;
            break;
        case 4:                 /*权重值N:4*/
            Tabc_1.a = Tabc_0.c;
            Tabc_1.b = Tabc_0.b;
            Tabc_1.c = Tabc_0.a;
            break;
        case 5:                 /*权重值N:6*/
            Tabc_1.a = Tabc_0.b;
            Tabc_1.b = Tabc_0.c;
            Tabc_1.c = Tabc_0.a;

            break;
        case 6:                 /*权重值N:2*/
            Tabc_1.a = Tabc_0.a;
            Tabc_1.b = Tabc_0.c;
            Tabc_1.c = Tabc_0.b;
            break;
    }

   
    // /*将计数值填充到PWM寄存器*/
    // A_H_SET_PWM(Tabc_1.a);          
    // B_H_SET_PWM(Tabc_1.b);
    // C_H_SET_PWM(Tabc_1.c);

    Tpwm = Tpwm/2;
    this->Debug_Tabc        = Tabc_1;
    this->Debug_Uabc.a      = Tabc_1.a/Tpwm*Udc;
    this->Debug_Uabc.b      = Tabc_1.b/Tpwm*Udc;
    this->Debug_Uabc.c      = Tabc_1.c/Tpwm*Udc;
    
}



/**
 * @brief 输出svpwm
 * 
 * @param U_qd qd轴电压
 * @param elec_angle 电角度
 */
inline void foc_pwm::svpwm(qd_t U_qd, float elec_angle, float Udc)
{
    uint8_t sector;
    alphabeta_t input = {0};

    input = FOC_Rev_Park(U_qd, elec_angle);
    this->Debug_Ualphabeta  = input;        /*这个地方必须把反向之前的数值保存起来，不然会影响无感观测器的计算*/
    
    #if 1       /*矢量反向*/
    input.alpha = -input.alpha;
    input.beta = -input.beta;
    #endif

    /*获取扇区*/
    sector = sector_get(input);
    /*计算矢量时间*/
    vector_time_calc(sector, input, Tpwm, Udc);
}

/**
 * @brief 输出spwm
 * 
 * @param U_qd qd轴电压
 * @param elec_angle 电角度
 */
inline void foc_pwm::spwm(qd_t U_qd, float elec_angle, float Udc)
{
    abc_t output             = {0};
    alphabeta_t input        = {0};

    float half_power = Udc/2.0f;

    input = FOC_Rev_Park(U_qd, elec_angle);

    /*克拉克逆变换*/
    output = FOC_Rev_Clarke(input);
    
    output.a += half_power;
    output.b += half_power;
    output.c += half_power;

    // A_H_SET_PWM(output.a/Udc *Tpwm_half);                 
    // B_H_SET_PWM(output.b/Udc *Tpwm_half);
    // C_H_SET_PWM(output.c/Udc *Tpwm_half);

    this->Debug_Tabc.a = output.a/Udc *Tpwm_half;
    this->Debug_Tabc.b = output.b/Udc *Tpwm_half;
    this->Debug_Tabc.c = output.c/Udc *Tpwm_half;

    this->Debug_Ualphabeta = input;
    this->Debug_Uabc = output;
}


/**
 * @brief 运行PWM
 * 
 * @param Uqd           qd轴电压
 * @param elec_angle    电角度
 * @param mode          发波模式
 */
void foc_pwm::run(qd_t Uqd, float elec_angle, float Udc, PWM_MODE_t mode)
{
    switch(mode){
        case SPWM_MODE:
            spwm(Uqd, elec_angle, Udc);
            break;
        case SVPWM_MODE:
            svpwm(Uqd, elec_angle, Udc);
            break;
        case TEST_MODE:
            break;
    }

    abc_t Debug_Uabc = this->Debug_Uabc;

    /*输出到PWM寄存器*/
    // Debug_Uabc.a += voltage_drop.a;
    // Debug_Uabc.b += voltage_drop.b;
    // Debug_Uabc.c += voltage_drop.c;

    
    A_H_SET_PWM(Debug_Uabc.a/Udc*Tpwm_half);                 
    B_H_SET_PWM(Debug_Uabc.b/Udc*Tpwm_half);
    C_H_SET_PWM(Debug_Uabc.c/Udc*Tpwm_half);


}

void foc_pwm::run(qd_t Uqd, float elec_angle, abc_t Iabc, float Udc, PWM_MODE_t mode, float Ts)
{
    switch(mode){
        case SPWM_MODE:
            spwm(Uqd, elec_angle, Udc);
            break;
        case SVPWM_MODE:
            svpwm(Uqd, elec_angle, Udc);
            break;
        case TEST_MODE:
            break;
    }

    abc_t Debug_Uabc = this->Debug_Uabc;

    /*
    中间可以做些补偿之类的
    */

    A_H_SET_PWM(Debug_Uabc.a/Udc*Tpwm_half);                 
    B_H_SET_PWM(Debug_Uabc.b/Udc*Tpwm_half);
    C_H_SET_PWM(Debug_Uabc.c/Udc*Tpwm_half);
}


#pragma arm section
