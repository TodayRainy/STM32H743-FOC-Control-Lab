#ifndef _FOC_HFI_FLUX_HANDOFF_HPP
#define _FOC_HFI_FLUX_HANDOFF_HPP

#include "foc_math.hpp"

#include <math.h>

typedef enum {
    FOC_HFI_FLUX_HFI_ACTIVE = 0,
    FOC_HFI_FLUX_TO_FLUX,
    FOC_HFI_FLUX_FLUX_ACTIVE,
    FOC_HFI_FLUX_HFI_QUALIFY,
    FOC_HFI_FLUX_TO_HFI,
    FOC_HFI_FLUX_FAILED,
} foc_hfi_flux_state_t;

typedef struct {
    float control_angle;
    float control_speed;
    float observer_angle_error;
    float observer_speed_error;
    foc_hfi_flux_state_t state;
    bool start_hfi;
    bool stop_hfi;
    bool failed;
} foc_hfi_flux_output_t;

/*
 * Full-speed sensorless source selector. HFI owns zero/low speed; Active Flux
 * owns medium/high speed. Thresholds and speed gates use electrical rad/s.
 */
class foc_hfi_flux_handoff
{
public:
    foc_hfi_flux_handoff(void)
    {
        configure(550.0f*_2PI/60.0f*10.0f,
                  650.0f*_2PI/60.0f*10.0f,
                  525.0f*_2PI/60.0f*10.0f,
                  700.0f*_2PI/60.0f*10.0f,
                  450.0f*_2PI/60.0f*10.0f,
                  30.0f*PI/180.0f,
                  100.0f*_2PI/60.0f*10.0f,
                  0.020f, 0.020f);
        clear();
    }

    void configure(float hfi_to_flux_speed,
                   float hfi_prepare_speed,
                   float flux_to_hfi_speed,
                   float hfi_speed_ceiling,
                   float flux_speed_floor,
                   float angle_gate,
                   float speed_gate,
                   float qualification_time,
                   float blend_time)
    {
        this->hfi_to_flux_speed = hfi_to_flux_speed;
        this->hfi_prepare_speed = hfi_prepare_speed;
        this->flux_to_hfi_speed = flux_to_hfi_speed;
        this->hfi_speed_ceiling = hfi_speed_ceiling;
        this->flux_speed_floor = flux_speed_floor;
        this->angle_gate = angle_gate;
        this->speed_gate = speed_gate;
        this->qualification_time_required = qualification_time;
        this->blend_time = blend_time;
    }

    void clear(void)
    {
        qualification_time = 0.0f;
        transition_time = 0.0f;
        output.control_angle = 0.0f;
        output.control_speed = 0.0f;
        output.observer_angle_error = 0.0f;
        output.observer_speed_error = 0.0f;
        output.state = FOC_HFI_FLUX_HFI_ACTIVE;
        output.start_hfi = false;
        output.stop_hfi = false;
        output.failed = false;
    }

    void start(float hfi_angle, float hfi_speed)
    {
        clear();
        output.control_angle = wrap_positive(hfi_angle);
        output.control_speed = hfi_speed;
    }

    foc_hfi_flux_output_t update(float hfi_angle, float hfi_speed,
                                 bool hfi_ready,
                                 float flux_angle, float flux_speed,
                                 bool flux_valid,
                                 float target_speed,
                                 float sample_period)
    {
        output.start_hfi = false;
        output.stop_hfi = false;
        output.observer_angle_error = wrap_signed(flux_angle-hfi_angle);
        output.observer_speed_error = flux_speed-hfi_speed;

        const bool observers_agree = hfi_ready && flux_valid
            && (fabsf(output.observer_angle_error) <= angle_gate)
            && (fabsf(output.observer_speed_error) <= speed_gate);

        switch (output.state) {
            case FOC_HFI_FLUX_HFI_ACTIVE:
                select_hfi(hfi_angle, hfi_speed);
                if ((fabsf(hfi_speed) >= hfi_to_flux_speed)
                && qualify(observers_agree, sample_period)) {
                    output.state = FOC_HFI_FLUX_TO_FLUX;
                    transition_time = 0.0f;
                } else if (fabsf(hfi_speed) >= hfi_speed_ceiling) {
                    fail();
                }
                break;

            case FOC_HFI_FLUX_TO_FLUX:
                blend(hfi_angle, hfi_speed, flux_angle, flux_speed,
                      sample_period, FOC_HFI_FLUX_FLUX_ACTIVE);
                if (output.state == FOC_HFI_FLUX_FLUX_ACTIVE) {
                    output.stop_hfi = true;
                }
                break;

            case FOC_HFI_FLUX_FLUX_ACTIVE:
                select_flux(flux_angle, flux_speed);
                if ((fabsf(target_speed) <= flux_to_hfi_speed)
                && (fabsf(flux_speed) <= hfi_prepare_speed)) {
                    qualification_time = 0.0f;
                    output.state = FOC_HFI_FLUX_HFI_QUALIFY;
                    output.start_hfi = true;
                }
                break;

            case FOC_HFI_FLUX_HFI_QUALIFY:
                select_flux(flux_angle, flux_speed);
                if ((fabsf(target_speed) > flux_to_hfi_speed)
                || (fabsf(flux_speed) > hfi_prepare_speed)) {
                    qualification_time = 0.0f;
                    output.state = FOC_HFI_FLUX_FLUX_ACTIVE;
                    output.stop_hfi = true;
                } else if ((fabsf(flux_speed) <= flux_to_hfi_speed)
                       && qualify(observers_agree, sample_period)) {
                    output.state = FOC_HFI_FLUX_TO_HFI;
                    transition_time = 0.0f;
                } else if (fabsf(flux_speed) <= flux_speed_floor) {
                    fail();
                }
                break;

            case FOC_HFI_FLUX_TO_HFI:
                blend(flux_angle, flux_speed, hfi_angle, hfi_speed,
                      sample_period, FOC_HFI_FLUX_HFI_ACTIVE);
                break;

            case FOC_HFI_FLUX_FAILED:
            default:
                output.failed = true;
                break;
        }
        return output;
    }

    foc_hfi_flux_output_t get_output(void) const
    {
        return output;
    }

private:
    bool qualify(bool condition, float sample_period)
    {
        if (condition) {
            qualification_time += sample_period;
        } else {
            qualification_time = 0.0f;
        }
        if (qualification_time >= qualification_time_required) {
            qualification_time = 0.0f;
            return true;
        }
        return false;
    }

    void blend(float source_angle, float source_speed,
               float destination_angle, float destination_speed,
               float sample_period, foc_hfi_flux_state_t destination_state)
    {
        transition_time += sample_period;
        float progress = transition_time/blend_time;
        if (progress > 1.0f) {
            progress = 1.0f;
        }
        output.control_angle = wrap_positive(
            source_angle+progress*wrap_signed(destination_angle-source_angle)
        );
        output.control_speed = source_speed
                             +progress*(destination_speed-source_speed);
        if (progress >= 1.0f) {
            output.state = destination_state;
        }
    }

    void select_hfi(float angle, float speed)
    {
        output.control_angle = wrap_positive(angle);
        output.control_speed = speed;
    }

    void select_flux(float angle, float speed)
    {
        output.control_angle = wrap_positive(angle);
        output.control_speed = speed;
    }

    void fail(void)
    {
        output.state = FOC_HFI_FLUX_FAILED;
        output.failed = true;
    }

    static float wrap_positive(float angle)
    {
        while (angle >= _2PI) {
            angle -= _2PI;
        }
        while (angle < 0.0f) {
            angle += _2PI;
        }
        return angle;
    }

    static float wrap_signed(float angle)
    {
        while (angle >= PI) {
            angle -= _2PI;
        }
        while (angle < -PI) {
            angle += _2PI;
        }
        return angle;
    }

    float hfi_to_flux_speed;
    float hfi_prepare_speed;
    float flux_to_hfi_speed;
    float hfi_speed_ceiling;
    float flux_speed_floor;
    float angle_gate;
    float speed_gate;
    float qualification_time_required;
    float blend_time;
    float qualification_time;
    float transition_time;
    foc_hfi_flux_output_t output;
};

#endif
