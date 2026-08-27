#ifndef _FOC_IDEN_MATH_HPP
#define _FOC_IDEN_MATH_HPP

#include <math.h>
#include <stdint.h>

typedef struct
{
    float resistance;
    float voltage_offset;
    float rmse;
} foc_rs_fit_t;

typedef struct
{
    float resistance;
    float inductance;
    float voltage_rms;
    float current_rms;
} foc_impedance_fit_t;

typedef struct
{
    float flux;
    float voltage_intercept;
    float rmse;
} foc_flux_fit_t;

/* Fit u = R * i + Voffset * sign(i). The offset captures the dominant
 * current-direction-dependent inverter voltage error without modifying the
 * existing dead-time implementation. */
static inline bool foc_iden_fit_rs(const float *current, const float *voltage,
                                   uint32_t count, foc_rs_fit_t *result)
{
    if((current == 0) || (voltage == 0) || (result == 0) || (count < 4U)){
        return false;
    }

    float sum_ii = 0.0f;
    float sum_is = 0.0f;
    float sum_iy = 0.0f;
    float sum_sy = 0.0f;
    uint32_t used = 0U;

    for(uint32_t n = 0U; n < count; n++){
        if(fabsf(current[n]) < 1.0e-4f){
            continue;
        }
        const float sign_i = current[n] > 0.0f ? 1.0f : -1.0f;
        sum_ii += current[n] * current[n];
        sum_is += current[n] * sign_i;
        sum_iy += current[n] * voltage[n];
        sum_sy += sign_i * voltage[n];
        used++;
    }

    const float determinant = sum_ii * (float)used - sum_is * sum_is;
    if((used < 4U) || (fabsf(determinant) < 1.0e-9f)){
        return false;
    }

    result->resistance = (sum_iy * (float)used - sum_sy * sum_is) / determinant;
    result->voltage_offset = (sum_ii * sum_sy - sum_is * sum_iy) / determinant;

    float error_sq = 0.0f;
    for(uint32_t n = 0U; n < count; n++){
        if(fabsf(current[n]) < 1.0e-4f){
            continue;
        }
        const float sign_i = current[n] > 0.0f ? 1.0f : -1.0f;
        const float estimate = result->resistance * current[n]
                             + result->voltage_offset * sign_i;
        const float error = voltage[n] - estimate;
        error_sq += error * error;
    }
    result->rmse = sqrtf(error_sq / (float)used);

    return (result->resistance > 0.0f) && (result->voltage_offset >= 0.0f);
}

static inline float foc_iden_rs_prediction_rmse(const float *current,
                                                const float *voltage,
                                                uint32_t count,
                                                float resistance,
                                                float voltage_offset)
{
    if((current == 0) || (voltage == 0) || (count == 0U)){
        return -1.0f;
    }

    float error_sq = 0.0f;
    uint32_t used = 0U;
    for(uint32_t n = 0U; n < count; n++){
        if(fabsf(current[n]) < 1.0e-4f){
            continue;
        }
        const float sign_i = current[n] > 0.0f ? 1.0f : -1.0f;
        const float estimate = resistance * current[n] + voltage_offset * sign_i;
        const float error = voltage[n] - estimate;
        error_sq += error * error;
        used++;
    }
    return used > 0U ? sqrtf(error_sq / (float)used) : -1.0f;
}

/* DFT coefficients use x = Xc*cos(theta) + Xs*sin(theta). */
static inline bool foc_iden_fit_impedance(float voltage_cos, float voltage_sin,
                                          float current_cos, float current_sin,
                                          uint32_t sample_count, float omega,
                                          foc_impedance_fit_t *result)
{
    if((result == 0) || (sample_count == 0U) || (omega <= 0.0f)){
        return false;
    }

    const float current_norm_sq = current_cos * current_cos
                                + current_sin * current_sin;
    if(current_norm_sq < 1.0e-12f){
        return false;
    }

    result->resistance = (voltage_cos * current_cos
                         + voltage_sin * current_sin) / current_norm_sq;
    const float reactance = (voltage_cos * current_sin
                            - voltage_sin * current_cos) / current_norm_sq;
    result->inductance = reactance / omega;

    const float rms_scale = 1.41421356237f / (float)sample_count;
    result->voltage_rms = rms_scale
                        * sqrtf(voltage_cos * voltage_cos + voltage_sin * voltage_sin);
    result->current_rms = rms_scale * sqrtf(current_norm_sq);

    return (result->inductance > 0.0f) && (result->current_rms > 1.0e-3f);
}

/* Fit y = flux * electrical_speed + intercept. y is the q-axis voltage after
 * subtracting copper drop, cross coupling and inverter offset. */
static inline bool foc_iden_fit_flux(const float *electrical_speed,
                                     const float *corrected_q_voltage,
                                     uint32_t count, foc_flux_fit_t *result)
{
    if((electrical_speed == 0) || (corrected_q_voltage == 0)
       || (result == 0) || (count < 2U)){
        return false;
    }

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_xx = 0.0f;
    float sum_xy = 0.0f;
    for(uint32_t n = 0U; n < count; n++){
        sum_x += electrical_speed[n];
        sum_y += corrected_q_voltage[n];
        sum_xx += electrical_speed[n] * electrical_speed[n];
        sum_xy += electrical_speed[n] * corrected_q_voltage[n];
    }

    const float determinant = (float)count * sum_xx - sum_x * sum_x;
    if(fabsf(determinant) < 1.0e-9f){
        return false;
    }

    result->flux = ((float)count * sum_xy - sum_x * sum_y) / determinant;
    result->voltage_intercept = (sum_y - result->flux * sum_x) / (float)count;

    float error_sq = 0.0f;
    for(uint32_t n = 0U; n < count; n++){
        const float estimate = result->flux * electrical_speed[n]
                             + result->voltage_intercept;
        const float error = corrected_q_voltage[n] - estimate;
        error_sq += error * error;
    }
    result->rmse = sqrtf(error_sq / (float)count);

    return result->flux > 0.0f;
}

static inline float foc_iden_flux_prediction_rmse(const float *electrical_speed,
                                                  const float *corrected_q_voltage,
                                                  uint32_t count, float flux,
                                                  float voltage_intercept,
                                                  float *max_abs_error)
{
    if((electrical_speed == 0) || (corrected_q_voltage == 0) || (count == 0U)){
        return -1.0f;
    }

    float error_sq = 0.0f;
    float max_error = 0.0f;
    for(uint32_t n = 0U; n < count; n++){
        const float estimate = flux * electrical_speed[n] + voltage_intercept;
        const float error = corrected_q_voltage[n] - estimate;
        const float abs_error = fabsf(error);
        error_sq += error * error;
        if(abs_error > max_error){
            max_error = abs_error;
        }
    }
    if(max_abs_error != 0){
        *max_abs_error = max_error;
    }
    return sqrtf(error_sq / (float)count);
}

#endif
