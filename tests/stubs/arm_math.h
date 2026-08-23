#ifndef TESTS_STUBS_ARM_MATH_H
#define TESTS_STUBS_ARM_MATH_H

#include <math.h>

#ifndef PI
#define PI 3.14159265358979f
#endif

static inline float arm_cos_f32(float value)
{
    return cosf(value);
}

static inline float arm_sin_f32(float value)
{
    return sinf(value);
}

#endif
