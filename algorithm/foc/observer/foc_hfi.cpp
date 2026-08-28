#include "foc_hfi.hpp"

#pragma arm section code = "RAMCODE"

foc_hfi::foc_hfi(float ld, float lq, float injection_voltage,
                 uint8_t half_period_samples, float demod_filter_hz,
                 float pll_bandwidth_hz, float pll_speed_leak_hz,
                 float speed_output_filter_hz,
                 float demod_polarity,
                 float output_alignment,
                 uint32_t pole_pairs,
                 uint32_t gear_ratio, float offset)
    : observer(ld, lq, injection_voltage, half_period_samples,
               demod_filter_hz, pll_bandwidth_hz, pll_speed_leak_hz,
               speed_output_filter_hz,
               demod_polarity,
               output_alignment),
      speed_estimator(pole_pairs, gear_ratio, offset)
{
}

foc_hfi::~foc_hfi()
{
}

void foc_hfi::clear(void)
{
    observer.clear();
    speed_estimator.clear();
}

void foc_hfi::start_tensor_tracking(float initial_angle,
                                    float injection_voltage)
{
    observer.set_injection_voltage(injection_voltage);
    observer.start_tensor_tracking(initial_angle);
    speed_estimator.clear();
    speed_estimator.AngleRefresh(initial_angle);
}

void foc_hfi::start_encoder_calibrated_shadow(float reference_angle,
                                              float initial_offset,
                                              float injection_voltage)
{
    observer.set_injection_voltage(injection_voltage);
    observer.start_encoder_calibrated_tensor_tracking(reference_angle,
                                                      initial_offset);
    speed_estimator.clear();
    speed_estimator.AngleRefresh(reference_angle);
}

void foc_hfi::start_d_axis_scan(float initial_angle,
                                float injection_voltage)
{
    observer.set_injection_voltage(injection_voltage);
    observer.start_d_axis_scan_carrier(initial_angle);
    speed_estimator.clear();
    speed_estimator.AngleRefresh(initial_angle);
}

void foc_hfi::start_tensor_scan(float initial_angle,
                                float injection_voltage)
{
    observer.set_injection_voltage(injection_voltage);
    observer.start_tensor_scan_carrier(initial_angle);
    speed_estimator.clear();
    speed_estimator.AngleRefresh(initial_angle);
}

void foc_hfi::stop(void)
{
    observer.stop();
    speed_estimator.SpeedRefresh(0.0f);
}

bool foc_hfi::running(void) const
{
    return observer.running();
}

bool foc_hfi::tracking(void) const
{
    return observer.tracking();
}

foc_hfi_mode_t foc_hfi::mode_get(void) const
{
    return observer.mode_get();
}

float foc_hfi::ElecAngleCalc(alphabeta_t current,
                            alphabeta_t applied_voltage,
                            float sample_period)
{
    const foc_hfi_output_t estimate = observer.update(
        current, applied_voltage, sample_period
    );
    speed_estimator.AngleRefresh(estimate.angle);
    speed_estimator.SpeedRefresh(estimate.speed);
    return estimate.angle;
}

float foc_hfi::ElecSpeedCalc(void)
{
    return speed_estimator.speed_data.elec_speed;
}

alphabeta_t foc_hfi::InjectionVoltage(void) const
{
    return observer.get_output().injection_voltage;
}

foc_hfi_output_t foc_hfi::output(void) const
{
    return observer.get_output();
}

const speed_t &foc_hfi::speed_output(void) const
{
    return speed_estimator.speed_data;
}

#pragma arm section
