#ifndef _BSP_HPP
#define _BSP_HPP


#include "sys_m7.h"
#include "dsp/arm_math.h"

#include "mpu/mpu.hpp"

#include "mt6835gt/mt6835gt.hpp"
extern mt6835gt *mt6835gt_0;

#include "mt6701acd/mt6701acd.hpp"
extern mt6701acd *mt6701acd_0;


#include "w25qxx/w25qxx.hpp"
extern w25qxx *w25q64;

#include "adc_dma/adc_dma.hpp"
extern adc_dma *adc3_dma;

#include "can/bsp_can.hpp"
extern bsp_can *can1;

#endif





