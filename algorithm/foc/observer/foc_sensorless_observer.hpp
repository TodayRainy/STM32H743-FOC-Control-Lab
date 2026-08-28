#ifndef _FOC_SENSORLESS_OBSERVER_HPP
#define _FOC_SENSORLESS_OBSERVER_HPP

#include "foc_math.hpp"

#include <math.h>

typedef struct
{
    float angle;
    float speed;
    float signal_magnitude;
    float phase_error;
} foc_sensorless_output_t;

/*
 * Extended sliding-mode observer for the medium/high-speed range.
 *
 * The stationary-frame model uses Lq, so the switching voltage estimates the
 * IPMSM extended EMF. The PLL tracks the filtered EMF vector; the rotor angle
 * is recovered by removing the signed pi/2 EMF lead and the known LPF lag.
 * Voltage is the previous PWM cycle's post-vector-limit alpha/beta command.
 */
class foc_esmo_observer
{
public:
    foc_esmo_observer(float rs, float lq, float sliding_gain,
                      float boundary_current, float emf_filter_hz,
                      float pll_bandwidth_hz,
                      float angle_alignment = 0.0f,
                      float extra_delay_s = 0.0f)
    {
        this->rs = rs;
        this->lq = lq;
        this->sliding_gain = sliding_gain;
        this->boundary_current = boundary_current;
        this->emf_filter_rad_s = _2PI*emf_filter_hz;
        this->angle_alignment = angle_alignment;
        this->extra_delay_s = extra_delay_s;

        const float pll_rad_s = _2PI*pll_bandwidth_hz;
        this->pll_kp = 2.0f*pll_rad_s;
        this->pll_ki = pll_rad_s*pll_rad_s;
        clear();
    }

    void clear(void)
    {
        est_current.alpha = 0.0f;
        est_current.beta = 0.0f;
        switching_voltage.alpha = 0.0f;
        switching_voltage.beta = 0.0f;
        filtered_emf.alpha = 0.0f;
        filtered_emf.beta = 0.0f;
        pll_angle = 0.0f;
        pll_speed = 0.0f;
        direction = 1.0f;
        phase_initialized = false;
        filter_sample_period = 0.0f;
        filter_alpha = 0.0f;
        output.angle = 0.0f;
        output.speed = 0.0f;
        output.signal_magnitude = 0.0f;
        output.phase_error = 0.0f;
    }

    foc_sensorless_output_t update(alphabeta_t voltage,
                                   alphabeta_t current, float sample_period)
    {
        const float inv_lq = 1.0f/lq;
        est_current.alpha += sample_period*inv_lq
                           *(voltage.alpha-rs*est_current.alpha
                             -switching_voltage.alpha);
        est_current.beta += sample_period*inv_lq
                          *(voltage.beta-rs*est_current.beta
                            -switching_voltage.beta);

        const float current_error_alpha = est_current.alpha-current.alpha;
        const float current_error_beta = est_current.beta-current.beta;
        switching_voltage.alpha = sliding_gain*sat(current_error_alpha);
        switching_voltage.beta = sliding_gain*sat(current_error_beta);

        if (filter_sample_period != sample_period) {
            filter_sample_period = sample_period;
            filter_alpha = 1.0f-expf(-emf_filter_rad_s*sample_period);
        }
        filtered_emf.alpha += filter_alpha
                            *(switching_voltage.alpha-filtered_emf.alpha);
        filtered_emf.beta += filter_alpha
                           *(switching_voltage.beta-filtered_emf.beta);

        const float magnitude = sqrtf(filtered_emf.alpha*filtered_emf.alpha
                                      +filtered_emf.beta*filtered_emf.beta);
        float phase_error = 0.0f;

        if (!phase_initialized && magnitude > 0.05f) {
            pll_angle = wrap_angle(atan2f(filtered_emf.beta,
                                          filtered_emf.alpha));
            phase_initialized = true;
        }

        if (phase_initialized && magnitude > 0.05f) {
            const float sine = arm_sin_f32(pll_angle);
            const float cosine = arm_cos_f32(pll_angle);
            phase_error = (-sine*filtered_emf.alpha
                           +cosine*filtered_emf.beta)/magnitude;
            pll_speed += pll_ki*phase_error*sample_period;
            pll_angle = wrap_angle(pll_angle
                                  +(pll_speed+pll_kp*phase_error)
                                   *sample_period);

            if (pll_speed > 5.0f) {
                direction = 1.0f;
            } else if (pll_speed < -5.0f) {
                direction = -1.0f;
            }
        }

        const float filter_lag = atan2f(pll_speed, emf_filter_rad_s);
        output.angle = wrap_angle(pll_angle-direction*_PI_2+filter_lag
                                  +angle_alignment
                                  -(0.5f*sample_period+extra_delay_s)
                                   *pll_speed);
        output.speed = pll_speed;
        output.signal_magnitude = magnitude;
        output.phase_error = phase_error;
        return output;
    }

    foc_sensorless_output_t get_output(void) const
    {
        return output;
    }

private:
    float sat(float value) const
    {
        if (value > boundary_current) {
            return 1.0f;
        }
        if (value < -boundary_current) {
            return -1.0f;
        }
        return value/boundary_current;
    }

    static float wrap_angle(float angle)
    {
        while (angle >= _2PI) {
            angle -= _2PI;
        }
        while (angle < 0.0f) {
            angle += _2PI;
        }
        return angle;
    }

    float rs;
    float lq;
    float sliding_gain;
    float boundary_current;
    float emf_filter_rad_s;
    float angle_alignment;
    float extra_delay_s;
    float pll_kp;
    float pll_ki;
    float pll_angle;
    float pll_speed;
    float direction;
    bool phase_initialized;
    float filter_sample_period;
    float filter_alpha;
    alphabeta_t est_current;
    alphabeta_t switching_voltage;
    alphabeta_t filtered_emf;
    foc_sensorless_output_t output;
};

/*
 * Nonlinear active-flux observer for a salient PMSM.
 *
 * psi_active = psi_stator - Lq*i =
 *              (psi_pm + (Ld-Lq)*id) * [cos(theta), sin(theta)]
 *
 * The radial correction follows the Ortega/ODrive nonlinear flux observer.
 * Its gain is expressed as a convergence bandwidth instead of an opaque raw
 * coefficient. A normalized type-II PLL supplies a smooth angle and speed.
 */
class foc_active_flux_observer
{
public:
    foc_active_flux_observer(float rs, float ld, float lq, float pm_flux,
                             float convergence_hz, float pll_bandwidth_hz,
                             float angle_alignment = 0.0f,
                             float extra_delay_s = 0.0f)
    {
        this->rs = rs;
        this->ld = ld;
        this->lq = lq;
        this->pm_flux = pm_flux;
        this->angle_alignment = angle_alignment;
        this->extra_delay_s = extra_delay_s;
        this->radial_gain = (_2PI*convergence_hz)/(pm_flux*pm_flux);

        const float pll_rad_s = _2PI*pll_bandwidth_hz;
        this->pll_kp = 2.0f*pll_rad_s;
        this->pll_ki = pll_rad_s*pll_rad_s;
        clear();
    }

    void clear(void)
    {
        stator_flux.alpha = pm_flux;
        stator_flux.beta = 0.0f;
        previous_current.alpha = 0.0f;
        previous_current.beta = 0.0f;
        current_initialized = false;
        pll_angle = 0.0f;
        pll_speed = 0.0f;
        output.angle = 0.0f;
        output.speed = 0.0f;
        output.signal_magnitude = pm_flux;
        output.phase_error = 0.0f;
    }

    foc_sensorless_output_t update(alphabeta_t voltage,
                                   alphabeta_t current, float sample_period)
    {
        if (!current_initialized) {
            previous_current = current;
            current_initialized = true;
        }

        alphabeta_t active_flux;
        active_flux.alpha = stator_flux.alpha-lq*current.alpha;
        active_flux.beta = stator_flux.beta-lq*current.beta;

        float magnitude = sqrtf(active_flux.alpha*active_flux.alpha
                                +active_flux.beta*active_flux.beta);
        float cosine = arm_cos_f32(pll_angle);
        float sine = arm_sin_f32(pll_angle);
        const float id_estimate = cosine*current.alpha+sine*current.beta;
        const float target_flux = pm_flux+(ld-lq)*id_estimate;
        const float magnitude_error = target_flux*target_flux
                                     -magnitude*magnitude;
        const float correction_scale = 0.5f*radial_gain*magnitude_error;
        const float current_mid_alpha = 0.5f
                                      *(current.alpha+previous_current.alpha);
        const float current_mid_beta = 0.5f
                                     *(current.beta+previous_current.beta);

        stator_flux.alpha += sample_period
                           *(voltage.alpha-rs*current_mid_alpha
                             +correction_scale*active_flux.alpha);
        stator_flux.beta += sample_period
                          *(voltage.beta-rs*current_mid_beta
                            +correction_scale*active_flux.beta);
        previous_current = current;

        active_flux.alpha = stator_flux.alpha-lq*current.alpha;
        active_flux.beta = stator_flux.beta-lq*current.beta;
        magnitude = sqrtf(active_flux.alpha*active_flux.alpha
                          +active_flux.beta*active_flux.beta);

        float phase_error = 0.0f;
        if (magnitude > 0.25f*pm_flux) {
            phase_error = (-sine*active_flux.alpha
                           +cosine*active_flux.beta)/magnitude;
            pll_speed += pll_ki*phase_error*sample_period;
            pll_angle = wrap_angle(pll_angle
                                  +(pll_speed+pll_kp*phase_error)
                                   *sample_period);
        }

        output.angle = wrap_angle(pll_angle+angle_alignment
                                  -(0.5f*sample_period+extra_delay_s)
                                   *pll_speed);
        output.speed = pll_speed;
        output.signal_magnitude = magnitude;
        output.phase_error = phase_error;
        return output;
    }

    foc_sensorless_output_t get_output(void) const
    {
        return output;
    }

private:
    static float wrap_angle(float angle)
    {
        while (angle >= _2PI) {
            angle -= _2PI;
        }
        while (angle < 0.0f) {
            angle += _2PI;
        }
        return angle;
    }

    float rs;
    float ld;
    float lq;
    float pm_flux;
    float angle_alignment;
    float extra_delay_s;
    float radial_gain;
    float pll_kp;
    float pll_ki;
    float pll_angle;
    float pll_speed;
    bool current_initialized;
    alphabeta_t stator_flux;
    alphabeta_t previous_current;
    foc_sensorless_output_t output;
};

#endif
