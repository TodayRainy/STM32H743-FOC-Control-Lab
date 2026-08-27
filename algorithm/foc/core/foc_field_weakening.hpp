#ifndef _FOC_FIELD_WEAKENING_HPP
#define _FOC_FIELD_WEAKENING_HPP

#include <math.h>
#include <stdint.h>

typedef enum
{
    FIELD_WEAKENING_MODE_OFF = 0U,
    FIELD_WEAKENING_MODE_FEEDBACK = 1U,
    FIELD_WEAKENING_MODE_HYBRID = 2U,
} FIELD_WEAKENING_MODE_t;

typedef struct
{
    float id_ref;
    float iq_limit;
    float voltage_request;
    float voltage_target;
    float model_id;
    float correction_id;
} foc_field_weakening_output_t;

static inline float FOC_FieldWeakeningIqLimit(float current_limit, float id_ref)
{
    const float remaining = current_limit*current_limit - id_ref*id_ref;
    return (remaining > 0.0f) ? sqrtf(remaining) : 0.0f;
}

/* Steady-state PMSM voltage ellipse. The returned root is the feasible
 * negative d-axis current closest to zero. omega_e is signed electrical rad/s. */
static inline float FOC_FieldWeakeningModelId(
    float omega_e, float iq, float voltage_target,
    float rs, float ld, float lq, float flux)
{
    const float uq_zero = rs*iq + omega_e*flux;
    const float ud_zero = -omega_e*lq*iq;
    const float a = rs*rs + omega_e*omega_e*ld*ld;
    const float b = 2.0f*(rs*ud_zero + omega_e*ld*uq_zero);
    const float c = ud_zero*ud_zero + uq_zero*uq_zero
                  - voltage_target*voltage_target;

    if(c <= 0.0f){
        return 0.0f;
    }

    const float discriminant = b*b - 4.0f*a*c;
    if(discriminant <= 0.0f){
        const float vertex = -b/(2.0f*a);
        return (vertex < 0.0f) ? vertex : 0.0f;
    }

    const float root_span = sqrtf(discriminant)/(2.0f*a);
    const float root_center = -b/(2.0f*a);
    const float root_high = root_center + root_span;
    const float root_low = root_center - root_span;
    if(root_high <= 0.0f){
        return root_high;
    }
    return (root_low <= 0.0f) ? root_low : 0.0f;
}

class foc_field_weakening
{
private:
    float rs;
    float ld;
    float lq;
    float flux;
    float kp;
    float ki;
    float current_limit;
    float target_utilization;
    float voltage_filter_alpha;
    float id_rate_limit;
    float correction_integral;
    float filtered_voltage;
    float previous_id_ref;
    bool filter_initialized;
    volatile uint32_t mode;
    foc_field_weakening_output_t output;

public:
    foc_field_weakening(void)
        : rs(0.0f), ld(0.0f), lq(0.0f), flux(0.0f), kp(0.0f), ki(0.0f),
          current_limit(0.0f), target_utilization(1.0f),
          voltage_filter_alpha(1.0f), id_rate_limit(0.0f),
          correction_integral(0.0f), filtered_voltage(0.0f),
          previous_id_ref(0.0f), filter_initialized(false),
          mode((uint32_t)FIELD_WEAKENING_MODE_OFF)
    {
        clear();
    }

    void configure(float motor_rs, float motor_ld, float motor_lq,
                   float motor_flux, float controller_kp, float controller_ki,
                   float total_current_limit, float utilization,
                   float filter_alpha, float maximum_id_rate)
    {
        rs = motor_rs;
        ld = motor_ld;
        lq = motor_lq;
        flux = motor_flux;
        kp = controller_kp;
        ki = controller_ki;
        current_limit = total_current_limit;
        target_utilization = utilization;
        voltage_filter_alpha = filter_alpha;
        id_rate_limit = maximum_id_rate;
        clear();
    }

    void set_mode(FIELD_WEAKENING_MODE_t new_mode)
    {
        mode = (uint32_t)new_mode;
        clear();
    }

    FIELD_WEAKENING_MODE_t mode_get(void) const
    {
        return (FIELD_WEAKENING_MODE_t)mode;
    }

    void clear(void)
    {
        correction_integral = 0.0f;
        filtered_voltage = 0.0f;
        previous_id_ref = 0.0f;
        filter_initialized = false;
        output.id_ref = 0.0f;
        output.iq_limit = current_limit;
        output.voltage_request = 0.0f;
        output.voltage_target = 0.0f;
        output.model_id = 0.0f;
        output.correction_id = 0.0f;
    }

    foc_field_weakening_output_t run(float voltage_request,
                                     float available_voltage,
                                     float omega_e, float measured_iq,
                                     float Ts)
    {
        if(!filter_initialized){
            filtered_voltage = voltage_request;
            filter_initialized = true;
        }else{
            filtered_voltage += voltage_filter_alpha
                              *(voltage_request-filtered_voltage);
        }

        output.voltage_request = filtered_voltage;
        output.voltage_target = target_utilization*available_voltage;
        output.model_id = 0.0f;
        output.correction_id = 0.0f;

        const FIELD_WEAKENING_MODE_t active_mode = mode_get();
        if(active_mode == FIELD_WEAKENING_MODE_OFF){
            correction_integral = 0.0f;
            output.id_ref = 0.0f;
            output.iq_limit = current_limit;
            return output;
        }

        if(active_mode == FIELD_WEAKENING_MODE_HYBRID){
            output.model_id = FOC_FieldWeakeningModelId(
                omega_e, measured_iq, output.voltage_target,
                rs, ld, lq, flux
            );
            const float model_d_limit = FOC_FieldWeakeningIqLimit(
                current_limit, measured_iq
            );
            if(output.model_id < -model_d_limit){
                output.model_id = -model_d_limit;
            }
        }

        const float voltage_error = filtered_voltage-output.voltage_target;
        correction_integral -= ki*voltage_error*Ts;
        output.correction_id = -kp*voltage_error + correction_integral;

        const float requested_id = output.model_id + output.correction_id;
        if(requested_id > 0.0f){
            output.id_ref = 0.0f;
        }else if(requested_id < -current_limit){
            output.id_ref = -current_limit;
        }else{
            output.id_ref = requested_id;
        }

        const float maximum_id_step = id_rate_limit*Ts;
        if(output.id_ref > previous_id_ref+maximum_id_step){
            output.id_ref = previous_id_ref+maximum_id_step;
        }else if(output.id_ref < previous_id_ref-maximum_id_step){
            output.id_ref = previous_id_ref-maximum_id_step;
        }
        previous_id_ref = output.id_ref;

        /* Unity tracking keeps the PI state consistent with the d-axis rate and
         * current-circle constraints, including q-axis reduction at 5 A. */
        correction_integral += output.id_ref-requested_id;
        output.correction_id = output.id_ref-output.model_id;
        output.iq_limit = FOC_FieldWeakeningIqLimit(current_limit, output.id_ref);
        return output;
    }
};

#endif
