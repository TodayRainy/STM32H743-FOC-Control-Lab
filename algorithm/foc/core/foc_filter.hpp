/**
 ****************************************************************************************************
 * @file        foc_filter.hpp
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

#ifndef _FOC_FILTER_HPP
#define _FOC_FILTER_HPP


#include "foc_type.h"
#include "arm_math.h"
#include "foc_pid.hpp"

#include <stdint.h>



#define MOVE_FILTER_BUF_MAXSIZE     500         /*滑动滤波最大区间*/
/*滑动滤波器*/
template <typename T>
class foc_movefilter
{
private:
    /* data */

    T *buf;
    T buf_sum;
    
    uint32_t size;
    uint32_t index;
    uint32_t fill_index;

    T val;

public:
    foc_movefilter(uint32_t size);
    ~foc_movefilter();

    void clear(void);
    void fill(T val);
    T calc(void);
};

template <typename T>
foc_movefilter<T>::foc_movefilter(uint32_t size)
{
    if(size>MOVE_FILTER_BUF_MAXSIZE)size=MOVE_FILTER_BUF_MAXSIZE;
    this->size = size;
    this->buf = new T[size];


    clear();

}
template <typename T>
foc_movefilter<T>::~foc_movefilter()
{
    delete(buf);
}

template <typename T>
void foc_movefilter<T>::clear(void)
{
    index = 0;
    fill_index = 0;
    buf_sum = 0;

    for(uint32_t i=0; i<size; i++){
        buf[i] = 0;
    }
}

template <typename T>
void foc_movefilter<T>::fill(T val)
{
    buf_sum -= buf[index];
    buf[index] = val;
    buf_sum += val;

    index++;
    if(index>=size){
        index = 0;
    }

    if(fill_index<size){
        fill_index++;
    }
}

template <typename T>
T foc_movefilter<T>::calc(void)
{
    T buf_sum = this->buf_sum;

    if(fill_index==0)return 0;

    if(fill_index < size){
        buf_sum = buf_sum/static_cast<T>(fill_index);
    }else{
        buf_sum = buf_sum/static_cast<T>(size);
    }
    
    val = buf_sum;

    return buf_sum;
}


class foc_pll
{
private:
    /* data */
    float kp;										/*锁相环PI控制器Kp*/
    float ki;										/*锁相环PI控制器Ki*/

    float p_term;                                   /*比例项*/
    float i_term;                                   /*积分项*/

    

    float err;                                      /*误差*/
    float omega;                                    /*电角速度*/
    float omega_f;

    float omega_max;
    
    float angle;                                    /*锁相环得到的角度*/
    float angle_last;						        /*上一次的电角度*/
public:
    foc_pll(float kp, float ki);
    ~foc_pll();

    foc_movefilter<float> *movefilter;

    void clear(void);

    float calc_smo(float alpha, float beta, float Ts);

    float calc_flux(float alpha, float beta, float Ts);

    float calc_hfi(float alpha, float beta, float Ts);

};





















#endif
