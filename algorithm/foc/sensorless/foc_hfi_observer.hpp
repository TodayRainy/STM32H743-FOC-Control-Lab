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
    float d_current_reference;
    float polarity_contrast;
    bool axis_valid;
    bool polarity_valid;
    bool ready;
    alphabeta_t injection_voltage;
} foc_hfi_output_t;

typedef enum
{
    FOC_HFI_MODE_OFF = 0,
    FOC_HFI_MODE_D_AXIS_SCAN,
    FOC_HFI_MODE_TENSOR_SCAN,
    FOC_HFI_MODE_ENCODER_CALIBRATION,
    FOC_HFI_MODE_TENSOR_TRACK,
    FOC_HFI_MODE_AXIS_ACQUIRE,
    FOC_HFI_MODE_POLARITY_DETECTION,
    FOC_HFI_MODE_D_AXIS_TRACK,
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
 * The four-axis tensor is retained for axis acquisition and diagnostics. Once
 * its mean admittance and antisymmetric sampling leakage are known, tracking
 * is d-axis dominant: three estimated-d-axis bipolar pairs are followed by
 * one q-axis calibration pair. The sparse orthogonal pair refreshes the
 * measured PWM/sampling leakage; it is not a torque-current command. At
 * standstill a short +Id/0/-Id sequence compares saturation response and
 * resolves the remaining pi ambiguity before q-axis torque is enabled.
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
                     float output_alignment = 0.0f,
                     float polarity_current_a = 0.5f,
                     float d_axis_acquire_pll_bandwidth_hz = 20.0f,
                     float d_axis_track_pll_bandwidth_hz = 40.0f)
    {
        expected_saliency_sign = (1.0f/ld-1.0f/lq) >= 0.0f ? 1.0f : -1.0f;
        injection_amplitude = injection_voltage;
        this->half_period_samples = half_period_samples;
        demod_filter_rad_s = _2PI*demod_filter_hz;
        this->demod_polarity = demod_polarity;
        this->output_alignment = output_alignment;
        this->polarity_current_a = polarity_current_a;

        const float pll_rad_s = _2PI*pll_bandwidth_hz;
        pll_kp = 2.0f*pll_rad_s;
        pll_ki = pll_rad_s*pll_rad_s;
        const float d_axis_acquire_rad_s =
            _2PI*d_axis_acquire_pll_bandwidth_hz;
        d_axis_acquire_pll_kp = 2.0f*d_axis_acquire_rad_s;
        const float d_axis_track_rad_s =
            _2PI*d_axis_track_pll_bandwidth_hz;
        d_axis_track_pll_kp = 2.0f*d_axis_track_rad_s;
        d_axis_track_pll_ki = d_axis_track_rad_s*d_axis_track_rad_s;
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
        tensor_mean_admittance = 0.0f;
        tensor_antisymmetric_cross = 0.0f;
        tensor_d_admittance = 0.0f;
        tensor_saliency_admittance = 0.0f;
        d_axis_settling_count = 0U;
        d_axis_speed_tracking_enabled = false;
        speed_tracking_direction = 0.0f;
        fixed_d_response = 0.0f;
        fixed_q_response = 0.0f;
        fixed_voltage_response.q = 0.0f;
        fixed_voltage_response.d = 0.0f;
        angle = 0.0f;
        speed = 0.0f;
        filtered_speed = 0.0f;
        axis_sine_sum = 0.0f;
        axis_cosine_sum = 0.0f;
        positive_response_sum = 0.0f;
        negative_response_sum = 0.0f;
        polarity_stage = FOC_HFI_POLARITY_AXIS_SETTLE;
        stage_update_count = 0U;
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
        output.d_current_reference = 0.0f;
        output.polarity_contrast = 0.0f;
        output.axis_valid = false;
        output.polarity_valid = false;
        output.ready = false;
        output.injection_voltage.alpha = 0.0f;
        output.injection_voltage.beta = 0.0f;
    }

    void start_tensor_tracking(float initial_angle)
    {
        clear();
        mode = FOC_HFI_MODE_TENSOR_TRACK;
        angle = wrap_angle(initial_angle-output_alignment);
        output.angle = wrap_angle(angle+output_alignment);
        output.ready = true;
    }

    void start_sensorless(float initial_angle)
    {
        clear();
        mode = FOC_HFI_MODE_AXIS_ACQUIRE;
        angle = wrap_angle(initial_angle-output_alignment);
        output.angle = wrap_angle(angle+output_alignment);
    }

    bool start_d_axis_tracking(void)
    {
        if((mode != FOC_HFI_MODE_TENSOR_TRACK)
        || (tensor_mean_admittance <= 0.0f)){
            return false;
        }

        begin_d_axis_tracking(true);
        return true;
    }

    bool resume_d_axis_tracking(float initial_angle, float initial_speed)
    {
        if (tensor_mean_admittance <= 0.0f) {
            return false;
        }

        angle = wrap_angle(initial_angle-output_alignment);
        output.angle = wrap_angle(angle+output_alignment);
        begin_d_axis_tracking(true, initial_speed);
        return true;
    }

    bool enable_d_axis_speed_tracking(void)
    {
        if((mode != FOC_HFI_MODE_D_AXIS_TRACK) || !output.ready){
            return false;
        }

        speed = 0.0f;
        filtered_speed = 0.0f;
        d_axis_speed_tracking_enabled = true;
        return true;
    }

    void set_speed_tracking_direction(float direction)
    {
        if (direction == 0.0f) {
            speed_tracking_direction = 0.0f;
            return;
        }

        const float next_direction = (direction > 0.0f) ? 1.0f : -1.0f;
        if (next_direction != speed_tracking_direction) {
            speed = 0.0f;
            filtered_speed = 0.0f;
            speed_tracking_direction = next_direction;
        }
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
        output.d_current_reference = 0.0f;
        output.ready = false;
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
        return (mode == FOC_HFI_MODE_TENSOR_TRACK)
            || (mode == FOC_HFI_MODE_D_AXIS_TRACK);
    }

    bool ready(void) const
    {
        return output.ready;
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
        float phase_error = output.phase_error;
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
                fixed_voltage_response.q += filter_alpha
                    *(voltage_response.q-fixed_voltage_response.q);
                fixed_voltage_response.d += filter_alpha
                    *(voltage_response.d-fixed_voltage_response.d);
            } else if (mode == FOC_HFI_MODE_D_AXIS_TRACK) {
                const float response_period = sample_period*2.0f
                                             *half_period_samples;
                update_filter(response_period);
                if ((applied_state & 1U) == 0U) {
                    pending_response = response;
                    pending_voltage = voltage_response;
                } else {
                    const qd_t pair = {
                        0.5f*(pending_response.q+response.q),
                        0.5f*(pending_response.d+response.d),
                    };
                    const qd_t voltage_pair = {
                        0.5f*(pending_voltage.q+voltage_response.q),
                        0.5f*(pending_voltage.d+voltage_response.d),
                    };
                    float raw_phase_error;

                    if (applied_state < d_axis_q_refresh_state) {
                        d_column.q += filter_alpha*(pair.q-d_column.q);
                        d_column.d += filter_alpha*(pair.d-d_column.d);
                        d_voltage_column.q += filter_alpha
                            *(voltage_pair.q-d_voltage_column.q);
                        d_voltage_column.d += filter_alpha
                            *(voltage_pair.d-d_voltage_column.d);
                        raw_phase_error = d_axis_phase_error();
                    } else {
                        q_column.q += filter_alpha*(pair.q-q_column.q);
                        q_column.d += filter_alpha*(pair.d-q_column.d);
                        q_voltage_column.q += filter_alpha
                            *(voltage_pair.q-q_voltage_column.q);
                        q_voltage_column.d += filter_alpha
                            *(voltage_pair.d-q_voltage_column.d);
                        raw_phase_error = d_axis_phase_error();
                        float tensor_phase_error;
                        if (tensor_solution(tensor_phase_error)) {
                            raw_phase_error = wrap_saliency_error(
                                tensor_phase_error
                            );
                        }
                    }
                    output.d_response = d_column.d;
                    output.q_response = d_column.q;
                    if (d_axis_settling_count
                        < d_axis_settling_updates) {
                        if (++d_axis_settling_count
                            == d_axis_settling_updates) {
                            if (!d_axis_speed_tracking_enabled) {
                                speed = 0.0f;
                                filtered_speed = 0.0f;
                            }
                            output.ready = true;
                        }
                    } else {
                        phase_error = LIMIT_RANGE(
                            raw_phase_error, -0.30f, 0.30f
                        );
                        pll_update(phase_error, response_period);
                    }
                }
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
                        float raw_phase_error;
                        if (tensor_solution(raw_phase_error)) {
                            const float response_period = sample_period*4.0f
                                                        *half_period_samples;
                            process_tensor_result(
                                raw_phase_error, response_period, phase_error
                            );
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

        if (pll_running()) {
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
        /* A leaky type-II PLL retains a finite constant-speed phase error.
         * Its raw integrator output is lower than rotor speed by
         * ki/(ki+kp*leak).  Report the corresponding DC-gain compensation
         * while keeping the leak in the internal state for standstill drift
         * rejection. */
        output.speed = filtered_speed*speed_output_gain();
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
    typedef enum
    {
        FOC_HFI_POLARITY_AXIS_SETTLE = 0,
        FOC_HFI_POLARITY_AXIS_AVERAGE,
        FOC_HFI_POLARITY_POSITIVE_SETTLE,
        FOC_HFI_POLARITY_POSITIVE_AVERAGE,
        FOC_HFI_POLARITY_ZERO_SETTLE,
        FOC_HFI_POLARITY_NEGATIVE_SETTLE,
        FOC_HFI_POLARITY_NEGATIVE_AVERAGE,
        FOC_HFI_POLARITY_FINAL_ZERO,
    } foc_hfi_polarity_stage_t;

    bool tensor_solution(float &raw_phase_error)
    {
        const float determinant =
            q_voltage_column.q*d_voltage_column.d
           -d_voltage_column.q*q_voltage_column.d;
        if (fabsf(determinant) <= 1.0e-4f) {
            return false;
        }

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
        const float cross = expected_saliency_sign*(b_qd+b_dq);
        const float diagonal = expected_saliency_sign*(b_dd-b_qq);

        tensor_mean_admittance = 0.5f*(b_qq+b_dd);
        tensor_antisymmetric_cross = 0.5f*(b_qd-b_dq);
        tensor_d_admittance = b_dd;
        tensor_saliency_admittance = hypotf(diagonal, cross);
        raw_phase_error = 0.5f*atan2f(cross, diagonal);
        output.d_response = (mode == FOC_HFI_MODE_POLARITY_DETECTION)
                          ? tensor_d_admittance : diagonal;
        output.q_response = 0.5f*cross;
        return true;
    }

    void process_tensor_result(float raw_phase_error,
                               float response_period,
                               float &phase_error)
    {
        if (mode == FOC_HFI_MODE_ENCODER_CALIBRATION) {
            if (bias_calibration_count >= bias_settling_updates) {
                bias_calibration_sum += raw_phase_error;
            }
            ++bias_calibration_count;
            if (bias_calibration_count >= bias_settling_updates
                                       +bias_average_updates) {
                phase_error_bias = bias_calibration_sum
                    /(float)bias_average_updates;
                mode = FOC_HFI_MODE_TENSOR_TRACK;
                speed = 0.0f;
                angle = wrap_angle(
                    calibration_reference_angle+calibration_initial_offset
                   -output_alignment
                );
                reset_tensor_state();
                output.ready = true;
            }
            return;
        }

        if (mode == FOC_HFI_MODE_TENSOR_TRACK) {
            phase_error = wrap_saliency_error(
                raw_phase_error-phase_error_bias
            );
            pll_update(phase_error, response_period);
            return;
        }

        if (mode == FOC_HFI_MODE_AXIS_ACQUIRE) {
            if (polarity_stage == FOC_HFI_POLARITY_AXIS_SETTLE) {
                if (++stage_update_count >= axis_settling_updates) {
                    polarity_stage = FOC_HFI_POLARITY_AXIS_AVERAGE;
                    stage_update_count = 0U;
                }
            } else {
                axis_sine_sum += arm_sin_f32(2.0f*raw_phase_error);
                axis_cosine_sum += arm_cos_f32(2.0f*raw_phase_error);
                if (++stage_update_count >= axis_average_updates) {
                    const float correction = 0.5f*atan2f(
                        axis_sine_sum, axis_cosine_sum
                    );
                    angle = wrap_angle(angle+correction);
                    speed = 0.0f;
                    filtered_speed = 0.0f;
                    output.axis_valid = true;
                    output.d_current_reference = polarity_current_a;
                    mode = FOC_HFI_MODE_POLARITY_DETECTION;
                    polarity_stage = FOC_HFI_POLARITY_POSITIVE_SETTLE;
                    stage_update_count = 0U;
                    reset_tensor_state();
                }
            }
            return;
        }

        if (mode != FOC_HFI_MODE_POLARITY_DETECTION) {
            return;
        }

        phase_error = wrap_saliency_error(raw_phase_error);
        pll_update(phase_error, response_period);
        switch (polarity_stage) {
            case FOC_HFI_POLARITY_POSITIVE_SETTLE:
                if (++stage_update_count >= polarity_settling_updates) {
                    polarity_stage = FOC_HFI_POLARITY_POSITIVE_AVERAGE;
                    stage_update_count = 0U;
                    positive_response_sum = 0.0f;
                }
                break;
            case FOC_HFI_POLARITY_POSITIVE_AVERAGE:
                positive_response_sum += tensor_d_admittance;
                if (++stage_update_count >= polarity_average_updates) {
                    output.d_current_reference = 0.0f;
                    polarity_stage = FOC_HFI_POLARITY_ZERO_SETTLE;
                    stage_update_count = 0U;
                }
                break;
            case FOC_HFI_POLARITY_ZERO_SETTLE:
                if (++stage_update_count >= polarity_zero_updates) {
                    output.d_current_reference = -polarity_current_a;
                    polarity_stage = FOC_HFI_POLARITY_NEGATIVE_SETTLE;
                    stage_update_count = 0U;
                }
                break;
            case FOC_HFI_POLARITY_NEGATIVE_SETTLE:
                if (++stage_update_count >= polarity_settling_updates) {
                    polarity_stage = FOC_HFI_POLARITY_NEGATIVE_AVERAGE;
                    stage_update_count = 0U;
                    negative_response_sum = 0.0f;
                }
                break;
            case FOC_HFI_POLARITY_NEGATIVE_AVERAGE:
                negative_response_sum += tensor_d_admittance;
                if (++stage_update_count >= polarity_average_updates) {
                    const float positive_response = positive_response_sum
                        /(float)polarity_average_updates;
                    const float negative_response = negative_response_sum
                        /(float)polarity_average_updates;
                    output.polarity_contrast =
                        (positive_response-negative_response)
                        /fabsf(negative_response);
                    /* A correctly oriented axis has a clear positive-current
                     * saturation increment. The opposite pole only loses
                     * that increment on this motor, so use the established
                     * 15% single-pulse ambiguity threshold instead of
                     * requiring the contrast sign to reverse. */
                    if (output.polarity_contrast < 0.15f) {
                        angle = wrap_angle(angle+PI);
                    }
                    output.d_current_reference = 0.0f;
                    output.polarity_valid = true;
                    polarity_stage = FOC_HFI_POLARITY_FINAL_ZERO;
                    stage_update_count = 0U;
                }
                break;
            case FOC_HFI_POLARITY_FINAL_ZERO:
                if (++stage_update_count >= polarity_zero_updates) {
                    begin_d_axis_tracking(false);
                }
                break;
            case FOC_HFI_POLARITY_AXIS_SETTLE:
            case FOC_HFI_POLARITY_AXIS_AVERAGE:
            default:
                break;
        }
    }

    float d_axis_phase_error(void) const
    {
        const float response_q = d_column.q
            -tensor_antisymmetric_cross*d_voltage_column.d;
        const float residual_q = response_q
            -tensor_mean_admittance*d_voltage_column.q;
        const float sine_double_error = LIMIT_RANGE(
            2.0f*expected_saliency_sign*residual_q
            /(tensor_saliency_admittance*d_voltage_column.d),
            -1.0f, 1.0f
        );
        return 0.5f*asinf(sine_double_error);
    }

    void begin_d_axis_tracking(bool enable_speed_tracking,
                               float initial_speed = 0.0f)
    {
        mode = FOC_HFI_MODE_D_AXIS_TRACK;
        speed = initial_speed/speed_output_gain();
        filtered_speed = speed;
        d_axis_settling_count = 0U;
        d_axis_speed_tracking_enabled = enable_speed_tracking;
        reset_tensor_state();
        output.ready = false;
    }

    void pll_update(float phase_error, float response_period)
    {
        const float kp = (mode != FOC_HFI_MODE_D_AXIS_TRACK)
                       ? pll_kp
                       : (d_axis_speed_tracking_enabled
                          ? d_axis_track_pll_kp
                          : d_axis_acquire_pll_kp);
        if((mode == FOC_HFI_MODE_D_AXIS_TRACK)
        && !d_axis_speed_tracking_enabled){
            speed = 0.0f;
            angle = wrap_angle(angle+kp*phase_error*response_period);
            return;
        }
        const float ki = (mode == FOC_HFI_MODE_D_AXIS_TRACK)
                       ? d_axis_track_pll_ki : pll_ki;
        speed *= expf(-pll_speed_leak_rad_s*response_period);
        speed += ki*phase_error*response_period;
        /* Command direction may reset the velocity state at a reversal, but
         * it must not clip the observer correction.  A PLL whose estimate is
         * ahead of the rotor has to move backwards even during a positive
         * speed command; preventing that correction makes the phase error
         * rail and loses the tracked axis under torque current. */
        angle = wrap_angle(
            angle+kp*phase_error*response_period
        );
    }

    bool pll_running(void) const
    {
        return (mode == FOC_HFI_MODE_TENSOR_TRACK)
            || (mode == FOC_HFI_MODE_POLARITY_DETECTION)
            || (mode == FOC_HFI_MODE_D_AXIS_TRACK);
    }

    float speed_output_gain(void) const
    {
        if (pll_speed_leak_rad_s == 0.0f) {
            return 1.0f;
        }
        const bool d_axis_tracking =
            (mode == FOC_HFI_MODE_D_AXIS_TRACK);
        const float kp = d_axis_tracking ? d_axis_track_pll_kp : pll_kp;
        const float ki = d_axis_tracking ? d_axis_track_pll_ki : pll_ki;
        return 1.0f+kp*pll_speed_leak_rad_s/ki;
    }

    uint32_t state_count(void) const
    {
        if (mode == FOC_HFI_MODE_D_AXIS_SCAN) {
            return 2U;
        }
        if (mode == FOC_HFI_MODE_D_AXIS_TRACK) {
            return d_axis_refresh_state_count;
        }
        return 4U;
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
        const bool q_axis = (mode == FOC_HFI_MODE_D_AXIS_TRACK)
                          ? (state >= d_axis_q_refresh_state)
                          : ((mode != FOC_HFI_MODE_D_AXIS_SCAN)
                             && (state >= 2U));
        if (q_axis) {
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
    static const uint32_t axis_settling_updates = 128U;
    static const uint32_t axis_average_updates = 256U;
    static const uint32_t polarity_settling_updates = 128U;
    static const uint32_t polarity_average_updates = 256U;
    static const uint32_t polarity_zero_updates = 128U;
    static const uint32_t d_axis_settling_updates = 128U;
    float expected_saliency_sign;
    float injection_amplitude;
    uint32_t half_period_samples;
    float demod_filter_rad_s;
    float demod_polarity;
    float output_alignment;
    float polarity_current_a;
    float pll_kp;
    float pll_ki;
    float d_axis_acquire_pll_kp;
    float d_axis_track_pll_kp;
    float d_axis_track_pll_ki;
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
    float tensor_mean_admittance;
    float tensor_antisymmetric_cross;
    float tensor_d_admittance;
    float tensor_saliency_admittance;
    uint32_t d_axis_settling_count;
    bool d_axis_speed_tracking_enabled;
    float speed_tracking_direction;

    enum
    {
        d_axis_refresh_d_pairs = 3U,
        d_axis_q_refresh_state = 2U*d_axis_refresh_d_pairs,
        d_axis_refresh_state_count = d_axis_q_refresh_state+2U,
    };
    float fixed_d_response;
    float fixed_q_response;
    qd_t fixed_voltage_response;
    float angle;
    float speed;
    float axis_sine_sum;
    float axis_cosine_sum;
    float positive_response_sum;
    float negative_response_sum;
    foc_hfi_polarity_stage_t polarity_stage;
    uint32_t stage_update_count;
    uint32_t next_state;
    uint32_t applied_state;
    uint32_t carrier_count;
    float filter_sample_period;
    float filter_alpha;
    foc_hfi_output_t output;
};

#endif
