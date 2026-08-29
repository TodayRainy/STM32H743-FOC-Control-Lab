#ifndef _FOC_HFI_SPEED_STARTUP_HPP
#define _FOC_HFI_SPEED_STARTUP_HPP

#include "foc_math.hpp"

#include <math.h>

typedef enum {
    FOC_HFI_SPEED_WAITING = 0,
    FOC_HFI_SPEED_ACCELERATING,
    FOC_HFI_SPEED_CLOSED_LOOP,
} foc_hfi_speed_startup_state_t;

typedef struct {
    float speed_reference;
    float current_reference;
    foc_hfi_speed_startup_state_t state;
    bool start_speed_tracking;
    bool enter_speed_loop;
} foc_hfi_speed_startup_output_t;

/*
 * HFI-angle current-assisted startup.  The rotor angle always comes from HFI;
 * this state only ramps a speed reference and applies bounded proportional
 * q-axis current until the normal low-speed PI has enough motion feedback.
 * Speeds and acceleration use electrical rad/s and rad/s^2.
 */
class foc_hfi_speed_startup
{
public:
    foc_hfi_speed_startup(void)
    {
        configure(0.50f, 250.0f*_2PI/60.0f*10.0f, 0.0016f,
                  350.0f*_2PI/60.0f*10.0f,
                  250.0f*_2PI/60.0f*10.0f, 0.020f);
        clear();
    }

    void configure(float current_limit, float acceleration,
                   float proportional_gain, float acceleration_target_speed,
                   float closed_loop_entry_speed, float qualification_time)
    {
        this->current_limit = current_limit;
        this->acceleration = acceleration;
        this->proportional_gain = proportional_gain;
        this->acceleration_target_speed = acceleration_target_speed;
        this->closed_loop_entry_speed = closed_loop_entry_speed;
        this->qualification_time_required = qualification_time;
    }

    void clear(void)
    {
        direction = 0.0f;
        qualification_time = 0.0f;
        output.speed_reference = 0.0f;
        output.current_reference = 0.0f;
        output.state = FOC_HFI_SPEED_WAITING;
        output.start_speed_tracking = false;
        output.enter_speed_loop = false;
    }

    foc_hfi_speed_startup_output_t update(float target_speed,
                                          float measured_speed,
                                          bool observer_ready,
                                          float sample_period)
    {
        output.start_speed_tracking = false;
        output.enter_speed_loop = false;

        if (output.state == FOC_HFI_SPEED_CLOSED_LOOP) {
            return output;
        }

        if (output.state == FOC_HFI_SPEED_WAITING) {
            output.current_reference = 0.0f;
            if (observer_ready && (target_speed != 0.0f)) {
                direction = (target_speed > 0.0f) ? 1.0f : -1.0f;
                output.speed_reference = measured_speed;
                output.state = FOC_HFI_SPEED_ACCELERATING;
                output.start_speed_tracking = true;
            }
            return output;
        }

        const float target_direction = (target_speed > 0.0f) ? 1.0f
                                     : ((target_speed < 0.0f) ? -1.0f
                                                             : direction);
        if ((target_speed != 0.0f) && (target_direction != direction)) {
            direction = target_direction;
            qualification_time = 0.0f;
        }

        const bool entry_requested = fabsf(target_speed)
                                  >= closed_loop_entry_speed;
        float startup_target = target_speed;
        if (entry_requested) {
            startup_target = direction*acceleration_target_speed;
        }
        const float maximum_step = acceleration*sample_period;
        const float reference_error = startup_target-output.speed_reference;
        output.speed_reference += LIMIT_RANGE(
            reference_error, -maximum_step, maximum_step
        );
        output.current_reference = LIMIT_RANGE(
            proportional_gain*(output.speed_reference-measured_speed),
            -current_limit, current_limit
        );

        const bool entry_reached = direction*measured_speed
                                >= closed_loop_entry_speed;
        if (entry_requested && entry_reached) {
            qualification_time += sample_period;
            if (qualification_time >= qualification_time_required) {
                output.state = FOC_HFI_SPEED_CLOSED_LOOP;
                output.enter_speed_loop = true;
            }
        } else {
            qualification_time = 0.0f;
        }
        return output;
    }

    foc_hfi_speed_startup_output_t get_output(void) const
    {
        return output;
    }

private:
    float current_limit;
    float acceleration;
    float proportional_gain;
    float acceleration_target_speed;
    float closed_loop_entry_speed;
    float qualification_time_required;
    float direction;
    float qualification_time;
    foc_hfi_speed_startup_output_t output;
};

#endif
