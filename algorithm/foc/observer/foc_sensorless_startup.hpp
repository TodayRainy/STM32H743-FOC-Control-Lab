#ifndef _FOC_SENSORLESS_STARTUP_HPP
#define _FOC_SENSORLESS_STARTUP_HPP

#include "foc_math.hpp"
#include "foc_type.h"

typedef enum {
    FOC_SENSORLESS_IDLE = 0,
    FOC_SENSORLESS_ALIGN,
    FOC_SENSORLESS_OPEN_LOOP,
    FOC_SENSORLESS_BLEND,
    FOC_SENSORLESS_CLOSED_LOOP,
    FOC_SENSORLESS_FAILED
} foc_sensorless_startup_state_t;

typedef struct {
    float control_angle;
    float control_speed;
    float virtual_angle;
    float virtual_speed;
    float phase_error;
    qd_t current_reference;
    foc_sensorless_startup_state_t state;
} foc_sensorless_startup_output_t;

/*
 * Current-controlled I/f start followed by a phase-continuous observer handoff.
 * All speeds and accelerations are electrical rad/s and rad/s^2.
 */
class foc_sensorless_startup
{
public:
    foc_sensorless_startup(void)
    {
        configure(0.8f, 1.0f, 0.15f, 5.0f, 0.2f, 3000.0f,
                  1000.0f, 20.0f*PI/180.0f,
                  100.0f, 0.02f, 0.02f, 0.5f);
        clear();
    }

    void configure(float align_current_a, float open_loop_current_a,
                   float handoff_current_a, float current_taper_rate_a_s,
                   float align_time_s, float acceleration_rad_s2,
                   float handoff_speed_rad_s, float phase_gate_rad,
                   float speed_gate_rad_s, float qualification_time_s,
                   float blend_time_s, float qualification_timeout_s)
    {
        this->align_current_a = align_current_a;
        this->open_loop_current_a = open_loop_current_a;
        this->handoff_current_a = handoff_current_a;
        this->current_taper_rate_a_s = current_taper_rate_a_s;
        this->align_time_s = align_time_s;
        this->acceleration_rad_s2 = acceleration_rad_s2;
        this->handoff_speed_rad_s = handoff_speed_rad_s;
        this->phase_gate_rad = phase_gate_rad;
        this->speed_gate_rad_s = speed_gate_rad_s;
        this->qualification_time_s = qualification_time_s;
        this->blend_time_s = blend_time_s;
        this->qualification_timeout_s = qualification_timeout_s;
    }

    void clear(void)
    {
        direction = 1.0f;
        state_time = 0.0f;
        qualification_time = 0.0f;
        handoff_wait_time = 0.0f;
        virtual_angle = 0.0f;
        virtual_speed = 0.0f;
        blend_offset = 0.0f;
        open_loop_current_command = open_loop_current_a;
        output.control_angle = 0.0f;
        output.control_speed = 0.0f;
        output.virtual_angle = 0.0f;
        output.virtual_speed = 0.0f;
        output.phase_error = 0.0f;
        output.current_reference.q = 0.0f;
        output.current_reference.d = 0.0f;
        output.state = FOC_SENSORLESS_IDLE;
    }

    void start(float requested_direction)
    {
        clear();
        direction = (requested_direction < 0.0f) ? -1.0f : 1.0f;
        output.state = FOC_SENSORLESS_ALIGN;
    }

    foc_sensorless_startup_output_t update(float observer_angle,
                                           float observer_speed,
                                           float sample_period)
    {
        output.phase_error = wrap_angle(observer_angle-virtual_angle);
        output.current_reference.q = 0.0f;
        output.current_reference.d = 0.0f;

        switch(output.state){
            case FOC_SENSORLESS_ALIGN:
                state_time += sample_period;
                output.current_reference.d = align_current_a;
                output.control_angle = virtual_angle;
                output.control_speed = 0.0f;
                if(state_time >= align_time_s){
                    state_time = 0.0f;
                    output.state = FOC_SENSORLESS_OPEN_LOOP;
                }
                break;

            case FOC_SENSORLESS_OPEN_LOOP:
            {
                const float target_speed = direction*handoff_speed_rad_s;
                const float speed_step = acceleration_rad_s2*sample_period;
                if(direction*virtual_speed < handoff_speed_rad_s){
                    virtual_speed += direction*speed_step;
                    if(direction*virtual_speed > handoff_speed_rad_s){
                        virtual_speed = target_speed;
                    }
                }
                virtual_angle = wrap_angle(virtual_angle
                                           +virtual_speed*sample_period);
                output.phase_error = wrap_angle(observer_angle-virtual_angle);
                if((direction*virtual_speed >= handoff_speed_rad_s)
                && (open_loop_current_command > handoff_current_a)){
                    open_loop_current_command -= current_taper_rate_a_s
                                               *sample_period;
                    if(open_loop_current_command < handoff_current_a){
                        open_loop_current_command = handoff_current_a;
                    }
                }
                output.current_reference.q = direction
                                           *open_loop_current_command;
                output.control_angle = virtual_angle;
                output.control_speed = virtual_speed;

                if((direction*virtual_speed >= handoff_speed_rad_s)
                && (open_loop_current_command <= handoff_current_a)){
                    handoff_wait_time += sample_period;
                    if((direction*observer_speed > 0.0f)
                    && (fabsf(observer_speed-virtual_speed)
                        <= speed_gate_rad_s)
                    && (fabsf(output.phase_error) <= phase_gate_rad)){
                        qualification_time += sample_period;
                    }else{
                        qualification_time = 0.0f;
                    }

                    if(qualification_time >= qualification_time_s){
                        blend_offset = wrap_angle(virtual_angle-observer_angle);
                        state_time = 0.0f;
                        output.state = FOC_SENSORLESS_BLEND;
                    }else if(handoff_wait_time >= qualification_timeout_s){
                        output.current_reference.q = 0.0f;
                        output.state = FOC_SENSORLESS_FAILED;
                    }
                }
                break;
            }

            case FOC_SENSORLESS_BLEND:
            {
                state_time += sample_period;
                float progress = state_time/blend_time_s;
                if(progress > 1.0f){
                    progress = 1.0f;
                }
                virtual_angle = wrap_angle(virtual_angle
                                           +virtual_speed*sample_period);
                output.phase_error = wrap_angle(observer_angle-virtual_angle);
                output.control_angle = wrap_angle(observer_angle
                                     +(1.0f-progress)*blend_offset);
                output.control_speed = virtual_speed
                                     +progress*(observer_speed-virtual_speed);
                /* Preserve the stationary current vector while the control
                 * frame rotates from I/f angle to observer angle. */
                const float frame_shift = wrap_angle(output.control_angle
                                                     -virtual_angle);
                const float if_current = direction
                                       *open_loop_current_command;
                output.current_reference.q = if_current
                                           *arm_cos_f32(frame_shift);
                output.current_reference.d = if_current
                                           *arm_sin_f32(frame_shift);
                if(progress >= 1.0f){
                    output.state = FOC_SENSORLESS_CLOSED_LOOP;
                }
                break;
            }

            case FOC_SENSORLESS_CLOSED_LOOP:
                output.control_angle = observer_angle;
                output.control_speed = observer_speed;
                break;

            case FOC_SENSORLESS_FAILED:
            case FOC_SENSORLESS_IDLE:
            default:
                output.control_angle = observer_angle;
                output.control_speed = 0.0f;
                break;
        }

        output.virtual_angle = virtual_angle;
        output.virtual_speed = virtual_speed;
        return output;
    }

    foc_sensorless_startup_output_t get_output(void) const
    {
        return output;
    }

    bool closed_loop(void) const
    {
        return output.state == FOC_SENSORLESS_CLOSED_LOOP;
    }

    bool failed(void) const
    {
        return output.state == FOC_SENSORLESS_FAILED;
    }

private:
    static float wrap_angle(float angle)
    {
        while(angle >= PI){
            angle -= _2PI;
        }
        while(angle < -PI){
            angle += _2PI;
        }
        return angle;
    }

    float align_current_a;
    float open_loop_current_a;
    float handoff_current_a;
    float current_taper_rate_a_s;
    float align_time_s;
    float acceleration_rad_s2;
    float handoff_speed_rad_s;
    float phase_gate_rad;
    float speed_gate_rad_s;
    float qualification_time_s;
    float blend_time_s;
    float qualification_timeout_s;

    float direction;
    float state_time;
    float qualification_time;
    float handoff_wait_time;
    float virtual_angle;
    float virtual_speed;
    float blend_offset;
    float open_loop_current_command;
    foc_sensorless_startup_output_t output;
};

#endif
