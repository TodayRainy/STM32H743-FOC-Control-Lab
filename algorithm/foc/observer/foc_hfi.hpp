#ifndef _FOC_HFI_HPP
#define _FOC_HFI_HPP

#include "foc_hfi_observer.hpp"
#include "foc_speed.hpp"

class foc_hfi
{
public:
    foc_hfi(float ld, float lq, float injection_voltage,
            uint8_t half_period_samples, float demod_filter_hz,
            float pll_bandwidth_hz, float pll_speed_leak_hz,
            float speed_output_filter_hz,
            float demod_polarity,
            float output_alignment,
            uint32_t pole_pairs,
            uint32_t gear_ratio, float offset);
    ~foc_hfi();

    void clear(void);
    void start_tensor_tracking(float initial_angle, float injection_voltage);
    void start_encoder_calibrated_shadow(float reference_angle,
                                         float initial_offset,
                                         float injection_voltage);
    void start_d_axis_scan(float initial_angle, float injection_voltage);
    void start_tensor_scan(float initial_angle, float injection_voltage);
    void stop(void);
    bool running(void) const;
    bool tracking(void) const;
    foc_hfi_mode_t mode_get(void) const;

    float ElecAngleCalc(alphabeta_t current,
                        alphabeta_t applied_voltage,
                        float sample_period);
    float ElecSpeedCalc(void);
    alphabeta_t InjectionVoltage(void) const;
    foc_hfi_output_t output(void) const;
    const speed_t &speed_output(void) const;

private:
    foc_hfi_observer observer;
    foc_speed speed_estimator;
};

#endif
