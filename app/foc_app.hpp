/**
 ****************************************************************************************************
 * @file        foc_app.hpp
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


#ifndef __FOC_APP_HPP
#define __FOC_APP_HPP


#include "foc_motor.hpp"
#include "foc_math.hpp"





extern void FOC_Init(void);

extern void FOC_App_Run(void);

void foc_key(void);



void FOC_Init_CPP(void);
void FOC_RUN(void);
void FOC_SPEED_CALC(void);
void FOC_SPEED_CTRL(void);
void FOC_POS_CTRL(void);
extern "C" void FOC_ServiceFaultReport(void);

extern "C" uint8_t FOC_CommandStartSpeed(void);
extern "C" uint8_t FOC_CommandStartCurrent(void);
extern "C" uint8_t FOC_CommandStartPosition(void);
extern "C" uint8_t FOC_CommandStartIdentificationRL(void);
extern "C" uint8_t FOC_CommandStartIdentificationFlux(void);
extern "C" uint8_t FOC_CommandPrintIdentification(void);
extern "C" uint8_t FOC_CommandPrintIdentificationPoints(void);
extern "C" uint8_t FOC_CommandSetSpeed(float speed_rpm);
extern "C" uint8_t FOC_CommandSetSpeedSine(float amplitude_rpm,
                                             float frequency_hz);
extern "C" uint8_t FOC_CommandStopSpeedSine(void);
extern "C" uint8_t FOC_CommandSetSpeedOutputLimit(float limit_a);
extern "C" uint8_t FOC_CommandSetSpeedEstimator(uint8_t mode);
extern "C" uint8_t FOC_CommandSetCurrent(float iq, float id);
extern "C" uint8_t FOC_CommandSetCurrentSine(uint8_t q_axis,
                                                float amplitude,
                                                float frequency_hz);
extern "C" uint8_t FOC_CommandStopCurrentSine(void);
extern "C" uint8_t FOC_CommandSetPosition(float position_deg);
extern "C" uint8_t FOC_CommandSetSpeedPI(float kp, float ki);
extern "C" uint8_t FOC_CommandSetPositionPI(float kp, float ki, float kd);
extern "C" uint8_t FOC_CommandSetPositionSpeedLimit(float limit_rpm);
extern "C" uint8_t FOC_CommandSetIqPI(float kp, float ki);
extern "C" uint8_t FOC_CommandSetIdPI(float kp, float ki);
extern "C" uint8_t FOC_CommandClearPI(void);
extern "C" uint8_t FOC_CommandSetSpeedStep(void);
extern "C" uint8_t FOC_CommandSetSpeedRamp(float accel_rpm_per_sec);
extern "C" uint8_t FOC_CommandSetPositionStep(void);
extern "C" uint8_t FOC_CommandSetPositionRamp(float velocity_deg_per_sec);
extern "C" uint8_t FOC_CommandSetPositionTrapezoid(float velocity_deg_per_sec,
                                                     float acceleration_deg_per_sec2);
extern "C" uint8_t FOC_CommandSetPositionFeedforward(uint8_t enabled);
extern "C" uint8_t FOC_CommandSetFeedforward(uint8_t enabled);
extern "C" uint8_t FOC_CommandSetFeedforwardMode(uint8_t mode);
extern "C" uint8_t FOC_CommandSetFeedforwardModel(uint8_t identified);
extern "C" uint8_t FOC_CommandSetSpeedId(float id);
extern "C" uint8_t FOC_CommandSetFieldWeakeningMode(uint8_t mode);
extern "C" uint8_t FOC_CommandSetVoltageLimit(float ratio);
extern "C" uint8_t FOC_CommandStop(void);



// extern foc_motor *m0;






#endif




