#ifndef _FOC_HFI_OBSERVER_HPP
#define _FOC_HFI_OBSERVER_HPP

#include "foc_math.hpp"

#include <math.h>
#include <stdint.h>

typedef struct
{
    float angle;
    float speed;
    float phase_error;
    float d_response;
    float q_response;
    alphabeta_t injection_voltage;
} foc_hfi_output_t;

/* These modes describe the current experimental HFI implementation. The
 * production d-axis tracking carrier and polarity detection are intentionally
 * not represented until their algorithms are implemented. */
typedef enum
{
    FOC_HFI_MODE_OFF = 0,
    FOC_HFI_MODE_D_AXIS_SCAN,
    FOC_HFI_MODE_TENSOR_SCAN,
    FOC_HFI_MODE_ENCODER_CALIBRATION,
    FOC_HFI_MODE_TENSOR_TRACK,
} foc_hfi_mode_t;

/*
 * Alternating-axis high-frequency injection observer for a salient PMSM.
 *
 * The carrier sequence is d+, d-, q+, q-. Each state lasts two 40 kHz
 * control samples; the current sample following the second applied pulse is
 * the usable ripple on this center-aligned PWM/ADC chain. Opposite polarities
 * are averaged to form the two columns of the incremental current-response
 * tensor.
 *
 * A real inductance tensor is symmetric. Therefore averaging its two cross
 * terms rejects the measured antisymmetric PWM/sampling leakage:
 *
 *   error = 0.5*atan2(B_qd + B_dq, B_dd - B_qq)
 *
 * The result is pi-periodic. The existing positive-d-current alignment
 * supplies initial rotor polarity before HFI is allowed to control the motor.
 */
class foc_hfi_observer
{
public:
    foc_hfi_observer(float ld, float lq, float injection_voltage,
                     uint32_t half_period_samples, float demod_filter_hz,
                     float pll_bandwidth_hz,
                     float pll_speed_leak_hz = 0.0f,
                     float speed_output_filter_hz = 0.0f,
                     float demod_polarity = 1.0f,
                     float output_alignment = 0.0f)
    {
        expected_saliency_sign = (1.0f/ld-1.0f/lq) >= 0.0f ? 1.0f : -1.0f;
        injection_amplitude = injection_voltage;
        this->half_period_samples = half_period_samples;
        demod_filter_rad_s = _2PI*demod_filter_hz;
        this->demod_polarity = demod_polarity;
        this->output_alignment = output_alignment;

        const float pll_rad_s = _2PI*pll_bandwidth_hz;
        pll_kp = 2.0f*pll_rad_s;
        pll_ki = pll_rad_s*pll_rad_s;
        pll_speed_leak_rad_s = _2PI*pll_speed_leak_hz;
        speed_output_filter_rad_s = _2PI*speed_output_filter_hz;
        clear();
    }

    void clear(void)
    {
        mode = FOC_HFI_MODE_OFF;
        bias_calibration_count = 0U;
        bias_calibration_sum = 0.0f;
        phase_error_bias = 0.0f;
        calibration_reference_angle = 0.0f;
        calibration_initial_offset = 0.0f;
        current_initialized = false;
        previous_current.q = 0.0f;
        previous_current.d = 0.0f;
        pending_response.q = 0.0f;
        pending_response.d = 0.0f;
        pending_voltage.q = 0.0f;
        pending_voltage.d = 0.0f;
        d_column.q = 0.0f;
        d_column.d = 0.0f;
        q_column.q = 0.0f;
        q_column.d = 0.0f;
        d_voltage_column.q = 0.0f;
        d_voltage_column.d = 0.0f;
        q_voltage_column.q = 0.0f;
        q_voltage_column.d = 0.0f;
        fixed_d_response = 0.0f;
        fixed_q_response = 0.0f;
        angle = 0.0f;
        speed = 0.0f;
        filtered_speed = 0.0f;
        next_state = 0U;
        applied_state = 4U;
        carrier_count = 0U;
        filter_sample_period = 0.0f;
        filter_alpha = 0.0f;
        output.angle = 0.0f;
        output.speed = 0.0f;
        output.phase_error = 0.0f;
        output.d_response = 0.0f;
        output.q_response = 0.0f;
        output.injection_voltage.alpha = 0.0f;
        output.injection_voltage.beta = 0.0f;
    }

    void start_tensor_tracking(float initial_angle)
    {
        clear();
        mode = FOC_HFI_MODE_TENSOR_TRACK;
        angle = wrap_angle(initial_angle-output_alignment);
        output.angle = wrap_angle(angle+output_alignment);
    }

    void start_d_axis_scan_carrier(float initial_carrier_angle)
    {
        clear();
        mode = FOC_HFI_MODE_D_AXIS_SCAN;
        angle = wrap_angle(initial_carrier_angle);
        output.angle = wrap_angle(angle+output_alignment);
    }

    void start_tensor_scan_carrier(float initial_carrier_angle)
    {
        clear();
        mode = FOC_HFI_MODE_TENSOR_SCAN;
        angle = wrap_angle(initial_carrier_angle);
        output.angle = wrap_angle(angle+output_alignment);
    }

    void start_encoder_calibrated_tensor_tracking(float reference_angle,
                                                  float initial_offset)
    {
        clear();
        mode = FOC_HFI_MODE_ENCODER_CALIBRATION;
        calibration_reference_angle = wrap_angle(reference_angle);
        calibration_initial_offset = initial_offset;
        angle = calibration_reference_angle;
        output.angle = angle;
    }

    void stop(void)
    {
        mode = FOC_HFI_MODE_OFF;
        applied_state = 4U;
        output.injection_voltage.alpha = 0.0f;
        output.injection_voltage.beta = 0.0f;
    }

    void set_injection_voltage(float voltage)
    {
        injection_amplitude = voltage;
    }

    bool running(void) const
    {
        return mode != FOC_HFI_MODE_OFF;
    }

    bool tracking(void) const
    {
        return mode == FOC_HFI_MODE_TENSOR_TRACK;
    }

    foc_hfi_mode_t mode_get(void) const
    {
        return mode;
    }

    foc_hfi_output_t update(alphabeta_t current,
                            alphabeta_t applied_voltage,
                            float sample_period)
    {
        if (mode == FOC_HFI_MODE_OFF) {
            output.injection_voltage.alpha = 0.0f;
            output.injection_voltage.beta = 0.0f;
            return output;
        }

        const qd_t estimated_current = park(current, angle);
        const qd_t estimated_voltage = park(applied_voltage, angle);
        float phase_error = 0.0f;
        const bool demod_sample = current_initialized
                               && (applied_state < state_count())
                               && (carrier_count == 0U);
        if (demod_sample) {
            const float carrier_sign = state_sign(applied_state);
            qd_t response;
            response.q = demod_polarity*carrier_sign
                        *(estimated_current.q-previous_current.q);
            response.d = demod_polarity*carrier_sign
                        *(estimated_current.d-previous_current.d);
            const qd_t voltage_response = {
                carrier_sign*estimated_voltage.q,
                carrier_sign*estimated_voltage.d,
            };

            if (mode == FOC_HFI_MODE_D_AXIS_SCAN) {
                update_filter(sample_period*2.0f*half_period_samples);
                fixed_q_response += filter_alpha
                                  *(response.q-fixed_q_response);
                fixed_d_response += filter_alpha
                                  *(response.d-fixed_d_response);
            } else {
                update_filter(sample_period*4.0f*half_period_samples);
                switch (applied_state) {
                    case 0U:                         /* d+ */
                    case 2U:                         /* q+ */
                        pending_response = response;
                        pending_voltage = voltage_response;
                        break;
                    case 1U:                         /* d- */
                    {
                        const qd_t pair = {
                            0.5f*(pending_response.q+response.q),
                            0.5f*(pending_response.d+response.d),
                        };
                        d_column.q += filter_alpha*(pair.q-d_column.q);
                        d_column.d += filter_alpha*(pair.d-d_column.d);
                        const qd_t voltage_pair = {
                            0.5f*(pending_voltage.q+voltage_response.q),
                            0.5f*(pending_voltage.d+voltage_response.d),
                        };
                        d_voltage_column.q += filter_alpha
                            *(voltage_pair.q-d_voltage_column.q);
                        d_voltage_column.d += filter_alpha
                            *(voltage_pair.d-d_voltage_column.d);
                        break;
                    }
                    case 3U:                         /* q- */
                    {
                        const qd_t pair = {
                            0.5f*(pending_response.q+response.q),
                            0.5f*(pending_response.d+response.d),
                        };
                        q_column.q += filter_alpha*(pair.q-q_column.q);
                        q_column.d += filter_alpha*(pair.d-q_column.d);
                        const qd_t voltage_pair = {
                            0.5f*(pending_voltage.q+voltage_response.q),
                            0.5f*(pending_voltage.d+voltage_response.d),
                        };
                        q_voltage_column.q += filter_alpha
                            *(voltage_pair.q-q_voltage_column.q);
                        q_voltage_column.d += filter_alpha
                            *(voltage_pair.d-q_voltage_column.d);

                        const float determinant =
                            q_voltage_column.q*d_voltage_column.d
                           -d_voltage_column.q*q_voltage_column.d;
                        if (fabsf(determinant) > 1.0e-4f) {
                            const float b_qq =
                                (q_column.q*d_voltage_column.d
                                -d_column.q*q_voltage_column.d)/determinant;
                            const float b_qd =
                                (-q_column.q*d_voltage_column.q
                                 +d_column.q*q_voltage_column.q)/determinant;
                            const float b_dq =
                                (q_column.d*d_voltage_column.d
                                -d_column.d*q_voltage_column.d)/determinant;
                            const float b_dd =
                                (-q_column.d*d_voltage_column.q
                                 +d_column.d*q_voltage_column.q)/determinant;
                            const float cross = expected_saliency_sign
                                *(b_qd+b_dq);
                            const float diagonal = expected_saliency_sign
                                *(b_dd-b_qq);
                            const float raw_phase_error =
                                0.5f*atan2f(cross, diagonal);
                            output.d_response = diagonal;
                            output.q_response = 0.5f*cross;
                            if (mode == FOC_HFI_MODE_ENCODER_CALIBRATION) {
                                if (bias_calibration_count
                                    >= bias_settling_updates) {
                                    bias_calibration_sum += raw_phase_error;
                                }
                                ++bias_calibration_count;
                                if (bias_calibration_count >=
                                    bias_settling_updates
                                   +bias_average_updates) {
                                    phase_error_bias = bias_calibration_sum
                                        /(float)bias_average_updates;
                                    mode = FOC_HFI_MODE_TENSOR_TRACK;
                                    speed = 0.0f;
                                    angle = wrap_angle(
                                        calibration_reference_angle
                                       +calibration_initial_offset
                                       -output_alignment
                                    );
                                    reset_tensor_state();
                                }
                            } else if (mode == FOC_HFI_MODE_TENSOR_TRACK) {
                                phase_error = wrap_saliency_error(
                                    raw_phase_error-phase_error_bias
                                );
                                const float pll_period = sample_period*4.0f
                                                       *half_period_samples;
                                speed *= expf(-pll_speed_leak_rad_s
                                             *pll_period);
                                speed += pll_ki*phase_error*pll_period;
                                angle = wrap_angle(
                                    angle+pll_kp*phase_error*pll_period
                                );
                            }
                        }
                        break;
                    }
                    default:
                        break;
                }
            }
        } else if (!current_initialized) {
            current_initialized = true;
        }
        previous_current = estimated_current;

        if (mode == FOC_HFI_MODE_TENSOR_TRACK) {
            angle = wrap_angle(angle+speed*sample_period);
            if (speed_output_filter_rad_s > 0.0f) {
                const float alpha = LIMIT_RANGE(
                    speed_output_filter_rad_s*sample_period, 0.0f, 1.0f
                );
                filtered_speed += alpha*(speed-filtered_speed);
            } else {
                filtered_speed = speed;
            }
        }

        make_injection(next_state);
        applied_state = next_state;
        ++carrier_count;
        if (carrier_count >= half_period_samples) {
            carrier_count = 0U;
            next_state = (next_state+1U)%state_count();
        }

        output.angle = wrap_angle(angle+output_alignment);
        output.speed = filtered_speed;
        output.phase_error = phase_error;
        if (mode == FOC_HFI_MODE_D_AXIS_SCAN) {
            output.d_response = fixed_d_response;
            output.q_response = fixed_q_response;
        }
        return output;
    }

    foc_hfi_output_t get_output(void) const
    {
        return output;
    }

private:
    uint32_t state_count(void) const
    {
        return (mode == FOC_HFI_MODE_D_AXIS_SCAN) ? 2U : 4U;
    }

    static float state_sign(uint32_t state)
    {
        return ((state & 1U) == 0U) ? 1.0f : -1.0f;
    }

    void reset_tensor_state(void)
    {
        current_initialized = false;
        pending_response.q = 0.0f;
        pending_response.d = 0.0f;
        pending_voltage.q = 0.0f;
        pending_voltage.d = 0.0f;
        d_column.q = 0.0f;
        d_column.d = 0.0f;
        q_column.q = 0.0f;
        q_column.d = 0.0f;
        d_voltage_column.q = 0.0f;
        d_voltage_column.d = 0.0f;
        q_voltage_column.q = 0.0f;
        q_voltage_column.d = 0.0f;
        next_state = 0U;
        applied_state = 4U;
        carrier_count = 0U;
    }

    void update_filter(float response_period)
    {
        if (filter_sample_period != response_period) {
            filter_sample_period = response_period;
            filter_alpha = 1.0f-expf(-demod_filter_rad_s*response_period);
        }
    }

    void make_injection(uint32_t state)
    {
        const float voltage = state_sign(state)*injection_amplitude;
        const float cosine = arm_cos_f32(angle);
        const float sine = arm_sin_f32(angle);
        if ((mode != FOC_HFI_MODE_D_AXIS_SCAN) && (state >= 2U)) {
            output.injection_voltage.alpha = -voltage*sine;
            output.injection_voltage.beta = voltage*cosine;
        } else {
            output.injection_voltage.alpha = voltage*cosine;
            output.injection_voltage.beta = voltage*sine;
        }
    }

    static qd_t park(alphabeta_t value, float reference_angle)
    {
        qd_t result;
        const float cosine = arm_cos_f32(reference_angle);
        const float sine = arm_sin_f32(reference_angle);
        result.q = cosine*value.beta-sine*value.alpha;
        result.d = cosine*value.alpha+sine*value.beta;
        return result;
    }

    static float wrap_angle(float value)
    {
        while (value >= _2PI) {
            value -= _2PI;
        }
        while (value < 0.0f) {
            value += _2PI;
        }
        return value;
    }

    static float wrap_saliency_error(float value)
    {
        while (value > PI/2.0f) {
            value -= PI;
        }
        while (value <= -PI/2.0f) {
            value += PI;
        }
        return value;
    }

    static const uint32_t bias_settling_updates = 64U;
    static const uint32_t bias_average_updates = 512U;

    float expected_saliency_sign;
    float injection_amplitude;
    uint32_t half_period_samples;
    float demod_filter_rad_s;
    float demod_polarity;
    float output_alignment;
    float pll_kp;
    float pll_ki;
    float pll_speed_leak_rad_s;
    float speed_output_filter_rad_s;
    float filtered_speed;
    foc_hfi_mode_t mode;
    uint32_t bias_calibration_count;
    float bias_calibration_sum;
    float phase_error_bias;
    float calibration_reference_angle;
    float calibration_initial_offset;
    bool current_initialized;
    qd_t previous_current;
    qd_t pending_response;
    qd_t pending_voltage;
    qd_t d_column;
    qd_t q_column;
    qd_t d_voltage_column;
    qd_t q_voltage_column;
    float fixed_d_response;
    float fixed_q_response;
    float angle;
    float speed;
    uint32_t next_state;
    uint32_t applied_state;
    uint32_t carrier_count;
    float filter_sample_period;
    float filter_alpha;
    foc_hfi_output_t output;
};

#endif
