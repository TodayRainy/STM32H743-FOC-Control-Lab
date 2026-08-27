#ifndef _FOC_ENCODER_MATH_HPP
#define _FOC_ENCODER_MATH_HPP

#include <stdint.h>

inline float foc_encoder_count_to_mech_angle(uint32_t count, uint32_t timer_period)
{
    const float two_pi = 6.28318530717958647692f;
    const float counts_per_revolution = static_cast<float>(timer_period + 1U);
    return static_cast<float>(count) * two_pi / counts_per_revolution;
}

#endif
