#ifndef FOC_CURRENT_TUNING_HPP
#define FOC_CURRENT_TUNING_HPP

struct foc_current_pi_gains_t
{
    float kp;
    float ki;
};

/* Parallel PI for R-L plant: Kp = wc*L, Ki = wc*R.
 * kp is V/A and ki is V/(A*s). */
static inline bool foc_current_pi_design(float resistance_ohm,
                                         float inductance_h,
                                         float bandwidth_hz,
                                         foc_current_pi_gains_t *gains)
{
    if((gains == nullptr) || (resistance_ohm <= 0.0f)
       || (inductance_h <= 0.0f) || (bandwidth_hz <= 0.0f)){
        return false;
    }

    const float omega_c = 6.283185307179586f * bandwidth_hz;
    gains->kp = omega_c * inductance_h;
    gains->ki = omega_c * resistance_ohm;
    return true;
}

#endif
