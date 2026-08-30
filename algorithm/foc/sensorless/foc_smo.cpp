/**
 ****************************************************************************************************
 * @file        foc_smo.cpp
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.2
 * @date        2026-08-28
 *
 * @license
 * Copyright (c) 2026 哔哩哔哩-Rebron大侠
 * This software is provided for educational and research purposes only.
 ****************************************************************************************************
 */

#include "foc_smo.hpp"

#pragma arm section code = "RAMCODE"

foc_smo::foc_smo(float Rs, float Lq, float sliding_gain,
                 float boundary_current, float emf_filter_hz,
                 float pll_bandwidth_hz, uint32_t pole_pairs,
                 uint32_t gear_ratio, float offset,
                 float extra_delay_s)
    : observer(Rs, Lq, sliding_gain, boundary_current,
               emf_filter_hz, pll_bandwidth_hz, offset,
               extra_delay_s)
{
    spd = new foc_speed(pole_pairs, gear_ratio, offset);
}

foc_smo::~foc_smo()
{
    delete(spd);
}

float foc_smo::ElecAngleCalc(alphabeta_t U_alphabeta,
                             alphabeta_t I_alphabeta, float Ts)
{
    const foc_sensorless_output_t estimate = observer.update(
        U_alphabeta, I_alphabeta, Ts
    );
    spd->AngleRefresh(estimate.angle);
    return estimate.angle;
}

float foc_smo::ElecSpeedCalc(void)
{
    spd->SpeedRefresh(observer.get_output().speed);
    return spd->speed_data.elec_speed;
}

foc_sensorless_output_t foc_smo::output(void) const
{
    return observer.get_output();
}

void foc_smo::clear(void)
{
    observer.clear();
    spd->clear();
}

#pragma arm section
