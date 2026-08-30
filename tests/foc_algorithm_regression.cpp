#include "foc_math.hpp"
#include "foc_pid.hpp"
#include "foc_pwm.hpp"
#include "foc_ctrl.hpp"
#include "foc_iden_math.hpp"
#include "foc_current_tuning.hpp"
#include "foc_encoder_math.hpp"
#include "foc_current_feedforward.hpp"
#include "foc_field_weakening.hpp"
#include "foc_speed.hpp"
#include "foc_sensorless_observer.hpp"
#include "foc_sensorless_startup.hpp"
#include "foc_hfi_observer.hpp"
#include "foc_hfi_speed_startup.hpp"
#include "foc_hfi_flux_handoff.hpp"
#include "foc_fault.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

int failures = 0;

void check(bool condition, const char *message)
{
    if (!condition) {
        std::printf("FAIL: %s\n", message);
        ++failures;
    }
}

void test_transforms(void)
{
    float max_clarke_error = 0.0f;
    float max_park_error = 0.0f;

    for (int sample = -20; sample <= 20; ++sample) {
        const abc_t abc = {
            sample * 0.25f,
            (sample - 7) * -0.13f,
            0.0f,
        };
        const abc_t zero_sum = {abc.a, abc.b, -abc.a - abc.b};
        const alphabeta_t alphabeta = FOC_Clarke(zero_sum);
        const abc_t restored = FOC_Rev_Clarke(alphabeta);

        max_clarke_error = std::max(max_clarke_error, std::fabs(restored.a - zero_sum.a));
        max_clarke_error = std::max(max_clarke_error, std::fabs(restored.b - zero_sum.b));
        max_clarke_error = std::max(max_clarke_error, std::fabs(restored.c - zero_sum.c));

        for (int angle_index = 0; angle_index < 24; ++angle_index) {
            const float angle = angle_index * _2PI / 24.0f;
            const qd_t qd = Foc_Park(alphabeta, angle);
            const alphabeta_t restored_alphabeta = FOC_Rev_Park(qd, angle);
            max_park_error = std::max(max_park_error,
                                      std::fabs(restored_alphabeta.alpha - alphabeta.alpha));
            max_park_error = std::max(max_park_error,
                                      std::fabs(restored_alphabeta.beta - alphabeta.beta));
        }
    }

    check(max_clarke_error < 2.0e-6f, "Clarke round trip");
    check(max_park_error < 2.0e-6f, "Park round trip");
    std::printf("transform max error: Clarke=%g Park=%g\n",
                max_clarke_error, max_park_error);
}

void test_svpwm_duty_range(void)
{
    constexpr float bus_voltage = 24.0f;
    constexpr uint32_t pwm_period_ticks = 6000;
    constexpr float compare_max = pwm_period_ticks / 2.0f;

    foc_pwm pwm(pwm_period_ticks);
    float duty_min = compare_max;
    float duty_max = 0.0f;
    float max_linear_voltage_error = 0.0f;
    bool duty_range_ok = true;

    for (int q_index = -3; q_index <= 3; ++q_index) {
        for (int d_index = -3; d_index <= 3; ++d_index) {
            const qd_t voltage = {q_index * 5.0f, d_index * 5.0f};

            for (int angle_index = 0; angle_index < 360; ++angle_index) {
                const float angle = angle_index * _2PI / 360.0f;
                pwm.run(voltage, angle, bus_voltage, SVPWM_MODE);

                const float duties[] = {
                    pwm.Debug_Tabc.a,
                    pwm.Debug_Tabc.b,
                    pwm.Debug_Tabc.c,
                };
                for (float duty : duties) {
                    duty_min = std::min(duty_min, duty);
                    duty_max = std::max(duty_max, duty);
                    duty_range_ok = duty_range_ok
                                    && duty >= -1.0e-3f
                                    && duty <= compare_max + 1.0e-3f;
                }

                if (std::hypot(voltage.q, voltage.d) <= 10.0f) {
                    const float common_mode = (pwm.Debug_Uabc.a
                                               + pwm.Debug_Uabc.b
                                               + pwm.Debug_Uabc.c) / 3.0f;
                    const abc_t phase_voltage = {
                        pwm.Debug_Uabc.a - common_mode,
                        pwm.Debug_Uabc.b - common_mode,
                        pwm.Debug_Uabc.c - common_mode,
                    };
                    const alphabeta_t actual = FOC_Clarke(phase_voltage);
                    const alphabeta_t expected = FOC_Rev_Park(voltage, angle);

                    max_linear_voltage_error = std::max(
                        max_linear_voltage_error,
                        std::fabs(actual.alpha - expected.alpha));
                    max_linear_voltage_error = std::max(
                        max_linear_voltage_error,
                        std::fabs(actual.beta - expected.beta));
                }
            }
        }
    }

    check(duty_range_ok, "SVPWM compare value outside timer range");
    check(max_linear_voltage_error < 1.0e-4f,
          "SVPWM changed voltage magnitude or direction in the linear region");
    std::printf("SVPWM compare range: [%g, %g], expected [0, %g]\n",
                duty_min, duty_max, compare_max);
    std::printf("SVPWM linear-region max voltage error: %g V\n",
                max_linear_voltage_error);
}

void test_pi_integrator_limit(void)
{
    foc_pid pi(4.0f, 1.0f, 0.0f, 10.0f, 0.25f, 1.0f);

    (void)pi.pi(1.0f, 0.0f, 0.001f);
    const float recovery_output = pi.pi(0.0f, 0.0f, 0.001f);

    check(std::fabs(recovery_output) <= 0.25f + 1.0e-6f,
          "PI anti-windup bypassed the configured integrator limit");
    std::printf("PI output after saturation and zero error: %g\n", recovery_output);
}

void test_runtime_gains_and_speed_ramp(void)
{
    foc_pid pi(1.0f, 0.0f, 0.0f, 10.0f, 10.0f, 10.0f);
    check(std::fabs(pi.pi(1.0f, 0.0f, 0.001f) - 1.0f) < 1.0e-6f,
          "initial runtime PI gain");
    pi.set_gains(2.0f, 0.0f, 0.0f);
    check(std::fabs(pi.pi(1.0f, 0.0f, 0.001f) - 2.0f) < 1.0e-6f,
          "runtime PI gain update");
    pi.set_output_limit(0.5f);
    check(std::fabs(pi.pi(1.0f, 0.0f, 0.001f) - 0.5f) < 1.0e-6f,
          "runtime PI output limit update");

    foc_track track;
    float value = 0.0f;
    for (int step = 0; step < 50; ++step) {
        value = track.run(100.0f, 0.0f, 200.0f, 0.01f, TRACK_MODE_RAMP);
    }
    check(std::fabs(value - 100.0f) < 1.0e-5f,
          "positive speed ramp rate");
    check(std::fabs(track.rate() - 200.0f) < 1.0e-5f,
          "trajectory ramp rate");

    value = track.run(-100.0f, value, 200.0f, 0.01f, TRACK_MODE_RAMP);
    check(std::fabs(value - 98.0f) < 1.0e-5f,
          "speed ramp direction change");
    value = track.run(25.0f, value, 0.0f, 0.01f, TRACK_MODE_NONE);
    check(std::fabs(value - 25.0f) < 1.0e-6f,
          "speed step mode");

    track.clear();
    value = track.run(10.0f, 5.0f, 10.0f, 0.1f, TRACK_MODE_RAMP);
    check(std::fabs(value - 6.0f) < 1.0e-6f,
          "speed ramp initializes from feedback");
}

void test_position_ramp(void)
{
    foc_pos_ctrl position(1.0f, 0.0f, 0.0f, 100.0f, 10.0f, 100.0f,
                          TC_POS_CONTINUOUS, TC_ROTATE_SHORTEST_PATH);
    position.set_track(TRACK_MODE_RAMP, 2.0f);

    const float first_output = position.run(10.0f, 1.0f, 1.0f,
                                            TC_ROTATE_SHORTEST_PATH, 0.1f);
    check(std::fabs(position.reference() - 1.2f) < 1.0e-6f,
          "position ramp does not start from feedback");
    check(std::fabs(first_output - 0.2f) < 1.0e-6f,
          "position loop output for ramped reference");

    position.set_velocity_feedforward(10.0f);
    const float feedforward_output = position.run(
        10.0f, 1.0f, 1.0f, TC_ROTATE_SHORTEST_PATH, 0.1f
    );
    check(std::fabs(feedforward_output - 20.4f) < 1.0e-5f,
          "position trajectory velocity feedforward");

    position.set_track(TRACK_MODE_NONE, 0.0f);
    (void)position.run(-2.0f, 1.0f, 1.0f, TC_ROTATE_SHORTEST_PATH, 0.1f);
    check(std::fabs(position.reference() + 2.0f) < 1.0e-6f,
          "position step mode");

    position.set_output_limit(1.0f);
    check(std::fabs(position.pid() + 1.0f) < 1.0e-6f,
          "position speed output limit");
}

void test_position_trapezoid(void)
{
    constexpr float sample_period = 0.001f;
    constexpr float velocity_limit = 2.0f;
    constexpr float acceleration_limit = 10.0f;
    foc_track track;
    float value = 0.0f;
    float previous_value = value;
    float previous_rate = 0.0f;
    float peak_rate = 0.0f;
    float peak_rate_step = 0.0f;

    for (int step = 0; step < 8000; ++step) {
        value = track.run(10.0f, value, velocity_limit, sample_period,
                          TRACK_MODE_TRAPEZOID, acceleration_limit);
        const float rate = track.rate();
        check(value >= previous_value - 1.0e-6f,
              "position trapezoid is monotonic");
        check(std::fabs(rate) <= velocity_limit + 1.0e-5f,
              "position trapezoid velocity limit");
        peak_rate_step = std::max(peak_rate_step,
                                  std::fabs(rate - previous_rate));
        peak_rate = std::max(peak_rate, std::fabs(rate));
        previous_value = value;
        previous_rate = rate;
        if ((std::fabs(value - 10.0f) < 1.0e-6f)
        && (std::fabs(rate) < 1.0e-6f)) {
            break;
        }
    }
    check(std::fabs(value - 10.0f) < 1.0e-6f,
          "position trapezoid reaches target");
    check(peak_rate > velocity_limit - 1.0e-4f,
          "long position move contains constant-speed section");
    check(peak_rate_step
              <= acceleration_limit * sample_period + 1.0e-5f,
          "position trapezoid acceleration limit");
    std::printf("position trapezoid: peak rate=%g, peak rate step=%g\n",
                peak_rate, peak_rate_step);

    track.clear();
    value = 0.0f;
    peak_rate = 0.0f;
    for (int step = 0; step < 2000; ++step) {
        value = track.run(0.05f, value, velocity_limit, sample_period,
                          TRACK_MODE_TRAPEZOID, acceleration_limit);
        peak_rate = std::max(peak_rate, std::fabs(track.rate()));
        if ((std::fabs(value - 0.05f) < 1.0e-6f)
        && (std::fabs(track.rate()) < 1.0e-6f)) {
            break;
        }
    }
    check(std::fabs(value - 0.05f) < 1.0e-6f,
          "short triangular position move reaches target");
    check(peak_rate < velocity_limit - 0.1f,
          "short move naturally uses triangular velocity profile");

    foc_pos_ctrl position(1.0f, 0.0f, 0.0f, 100.0f, 10.0f, 100.0f,
                          TC_POS_CONTINUOUS, TC_ROTATE_SHORTEST_PATH);
    position.set_track(TRACK_MODE_TRAPEZOID, 2.0f, 2.0f);
    position.set_velocity_feedforward(10.0f);
    const float output = position.run(10.0f, 0.0f, 0.0f,
                                      TC_ROTATE_SHORTEST_PATH, 0.1f);
    check(std::fabs(position.reference() - 0.01f) < 1.0e-6f,
          "position trapezoid integrates acceleration");
    check(std::fabs(output - 2.01f) < 1.0e-5f,
          "position trapezoid exposes velocity feedforward");

    track.clear();
    value = 0.0f;
    for (int step = 0; step < 500; ++step) {
        value = track.run(10.0f, value, velocity_limit, sample_period,
                          TRACK_MODE_TRAPEZOID, acceleration_limit);
    }
    previous_rate = track.rate();
    peak_rate_step = 0.0f;
    for (int step = 0; step < 5000; ++step) {
        value = track.run(-1.0f, value, velocity_limit, sample_period,
                          TRACK_MODE_TRAPEZOID, acceleration_limit);
        peak_rate_step = std::max(
            peak_rate_step, std::fabs(track.rate() - previous_rate)
        );
        previous_rate = track.rate();
        if ((std::fabs(value + 1.0f) < 1.0e-6f)
        && (std::fabs(track.rate()) < 1.0e-6f)) {
            break;
        }
    }
    check(std::fabs(value + 1.0f) < 1.0e-6f,
          "position trapezoid replans a target reversal");
    check(peak_rate_step
              <= acceleration_limit * sample_period + 1.0e-5f,
          "position trapezoid target reversal remains acceleration limited");
}

void test_position_scurve(void)
{
    constexpr float sample_period = 1.0f/400.0f;
    constexpr float velocity_limit = 180.0f*PI/180.0f;
    constexpr float acceleration_limit = 900.0f*PI/180.0f;
    constexpr float jerk_limit = 22500.0f*PI/180.0f;
    const float distances_deg[3] = {1.0f, 5.0f, 90.0f};
    float measured_time[3] = {};

    for(uint8_t profile = 0U; profile < 3U; ++profile){
        foc_track track;
        const float target = distances_deg[profile]*PI/180.0f;
        float position = 0.0f;
        float previous_position = 0.0f;
        float previous_acceleration = 0.0f;
        float peak_rate = 0.0f;
        float peak_acceleration = 0.0f;
        float peak_jerk = 0.0f;
        uint32_t samples = 0U;
        for(; samples < 2000U; ++samples){
            position = track.run(target, 0.0f, velocity_limit,
                                 sample_period, TRACK_MODE_SCURVE,
                                 acceleration_limit, jerk_limit);
            peak_rate = std::max(peak_rate, std::fabs(track.rate()));
            peak_acceleration = std::max(
                peak_acceleration, std::fabs(track.acceleration())
            );
            peak_jerk = std::max(
                peak_jerk,
                std::fabs(track.acceleration()-previous_acceleration)
                    /sample_period
            );
            check(position+1.0e-6f >= previous_position,
                  "position S-curve is monotonic");
            previous_position = position;
            previous_acceleration = track.acceleration();
            if((std::fabs(position-target) < 1.0e-7f)
            && (std::fabs(track.rate()) < 1.0e-7f)
            && (std::fabs(track.acceleration()) < 1.0e-7f)){
                ++samples;
                break;
            }
        }
        measured_time[profile] = samples*sample_period;
        check(samples < 2000U, "position S-curve reaches target");
        check(std::fabs(position-target) < 1.0e-6f,
              "position S-curve exact endpoint");
        check(peak_rate <= velocity_limit+1.0e-4f,
              "position S-curve velocity limit");
        check(peak_acceleration <= acceleration_limit+1.0e-4f,
              "position S-curve acceleration limit");
        check(peak_jerk <= jerk_limit+1.0f,
              "position S-curve jerk limit");
    }

    const float jerk_time = acceleration_limit/jerk_limit;
    const float short_time = 4.0f*cbrtf(
        (distances_deg[0]*PI/180.0f)/(2.0f*jerk_limit)
    );
    const float medium_accel_time = 0.5f*(
        sqrtf(jerk_time*jerk_time
              + 4.0f*(distances_deg[1]*PI/180.0f)/acceleration_limit)
        - 3.0f*jerk_time
    );
    const float medium_time = 4.0f*jerk_time+2.0f*medium_accel_time;
    const float velocity_accel_time = velocity_limit/acceleration_limit
                                    - jerk_time;
    const float velocity_distance = velocity_limit
        *(2.0f*jerk_time+velocity_accel_time);
    const float long_time = 4.0f*jerk_time+2.0f*velocity_accel_time
        + ((distances_deg[2]*PI/180.0f)-velocity_distance)/velocity_limit;
    check(std::fabs(measured_time[0]-short_time) <= sample_period,
          "four-segment S-curve duration");
    check(std::fabs(measured_time[1]-medium_time) <= sample_period,
          "six-segment S-curve duration");
    check(std::fabs(measured_time[2]-long_time) <= sample_period,
          "seven-segment S-curve duration");

    foc_track positive;
    foc_track negative;
    for(uint32_t sample = 0U; sample < 400U; ++sample){
        const float plus = positive.run(PI/2.0f, 0.0f, velocity_limit,
                                        sample_period, TRACK_MODE_SCURVE,
                                        acceleration_limit, jerk_limit);
        const float minus = negative.run(-PI/2.0f, 0.0f, velocity_limit,
                                         sample_period, TRACK_MODE_SCURVE,
                                         acceleration_limit, jerk_limit);
        check(std::fabs(plus+minus) < 2.0e-6f,
              "position S-curve direction symmetry");
        check(std::fabs(positive.rate()+negative.rate()) < 2.0e-5f,
              "position S-curve rate symmetry");
        check(std::fabs(positive.acceleration()+negative.acceleration())
              < 2.0e-4f,
              "position S-curve acceleration symmetry");
    }

    foc_track queued;
    float queued_position = 0.0f;
    float maximum_position = 0.0f;
    for(uint32_t sample = 0U; sample < 1200U; ++sample){
        const float target = (sample < 40U) ? PI/2.0f : -PI/6.0f;
        queued_position = queued.run(
            target, 0.0f, velocity_limit, sample_period,
            TRACK_MODE_SCURVE, acceleration_limit, jerk_limit
        );
        maximum_position = std::max(maximum_position, queued_position);
    }
    check(std::fabs(maximum_position-PI/2.0f) < 1.0e-5f,
          "position S-curve completes active target before queued retarget");
    check(std::fabs(queued_position+PI/6.0f) < 1.0e-5f,
          "position S-curve reaches queued target");

    std::printf("position S-curve durations: 1deg=%g 5deg=%g 90deg=%g s\n",
                measured_time[0], measured_time[1], measured_time[2]);
}

void test_voltage_vector_limit_and_tracking(void)
{
    constexpr float bus_voltage = 24.0f;
    constexpr float svpwm_limit = bus_voltage * _1_DIV_SQRT_3;
    constexpr float spwm_limit = bus_voltage * 0.5f;

    foc_pwm pwm(6000);

    const qd_t linear_request = {5.0f, -2.0f};
    const qd_t linear_applied = pwm.run(linear_request, 0.3f, bus_voltage, SVPWM_MODE);
    const float linear_scale = pwm.voltage_scale;
    check(std::fabs(linear_applied.q - linear_request.q) < 1.0e-6f
              && std::fabs(linear_applied.d - linear_request.d) < 1.0e-6f,
          "SVPWM changed a voltage vector inside the linear region");
    check(std::fabs(linear_scale - 1.0f) < 1.0e-6f,
          "linear-region voltage scale");

    pwm.set_voltage_limit_ratio(0.25f);
    check(std::fabs(pwm.get_voltage_limit(bus_voltage, SVPWM_MODE)
                    - svpwm_limit*0.25f) < 1.0e-6f,
          "reported SVPWM voltage limit");
    const qd_t test_limit_request = {4.0f, 0.0f};
    const qd_t test_limit_applied = pwm.run(
        test_limit_request, 0.3f, bus_voltage, SVPWM_MODE
    );
    const float test_limit = svpwm_limit*0.25f;
    check(std::fabs(std::hypot(test_limit_applied.q, test_limit_applied.d)
                    - test_limit) < 2.0e-6f,
          "test voltage ceiling magnitude");
    check(std::fabs(pwm.voltage_scale-test_limit/4.0f) < 1.0e-6f,
          "test voltage ceiling scale telemetry");
    pwm.set_voltage_limit_ratio(1.0f);

    const qd_t high_request = {15.0f, 15.0f};
    const qd_t svpwm_applied = pwm.run(high_request, 0.3f, bus_voltage, SVPWM_MODE);
    const float svpwm_scale = pwm.voltage_scale;
    check(std::fabs(std::hypot(svpwm_applied.q, svpwm_applied.d) - svpwm_limit) < 2.0e-6f,
          "SVPWM voltage vector limit");
    check(std::fabs(svpwm_applied.q * high_request.d
                    - svpwm_applied.d * high_request.q) < 1.0e-5f,
          "SVPWM voltage vector limit changed direction");

    const qd_t spwm_applied = pwm.run(high_request, 0.3f, bus_voltage, SPWM_MODE);
    const float spwm_scale = pwm.voltage_scale;
    check(std::fabs(std::hypot(spwm_applied.q, spwm_applied.d) - spwm_limit) < 2.0e-6f,
          "SPWM voltage vector limit");
    check(std::fabs(svpwm_scale - svpwm_limit / std::hypot(high_request.q, high_request.d))
              < 1.0e-6f,
          "SVPWM voltage scale telemetry");
    check(std::fabs(spwm_scale - spwm_limit / std::hypot(high_request.q, high_request.d))
              < 1.0e-6f,
          "SPWM voltage scale telemetry");

    foc_pid current_pi(0.0f, 100.0f, 0.0f, 10.0f, 10.0f, 10.0f);
    const float requested_output = current_pi.pi(1.0f, 0.0f, 0.01f);
    current_pi.track_output(0.25f);
    const float tracked_output = current_pi.pi(0.0f, 0.0f, 0.01f);

    check(std::fabs(requested_output - 1.0f) < 1.0e-6f,
          "PI tracking test setup");
    check(std::fabs(tracked_output - 0.25f) < 1.0e-6f,
          "PI did not track the voltage actually applied by the modulator");

    std::printf("voltage limits: SVPWM=%g V SPWM=%g V, tracked PI=%g V\n",
                std::hypot(svpwm_applied.q, svpwm_applied.d),
                std::hypot(spwm_applied.q, spwm_applied.d),
                tracked_output);
}

void test_current_feedforward_modes(void)
{
    constexpr float omega_e = 3000.0f;
    constexpr float ld = 26.043582e-6f;
    constexpr float lq = 40.343765e-6f;
    constexpr float flux = 0.90795034e-3f;
    const qd_t measured = {1.25f, -0.75f};
    const float cross_q = omega_e*ld*measured.d;
    const float cross_d = -omega_e*lq*measured.q;
    const float bemf_q = omega_e*flux;

    const qd_t off = FOC_CurrentFeedforward(
        CURRENT_FF_MODE_OFF, omega_e, ld, lq, flux, measured
    );
    const qd_t both = FOC_CurrentFeedforward(
        CURRENT_FF_MODE_BOTH, omega_e, ld, lq, flux, measured
    );
    const qd_t cross = FOC_CurrentFeedforward(
        CURRENT_FF_MODE_CROSS, omega_e, ld, lq, flux, measured
    );
    const qd_t bemf = FOC_CurrentFeedforward(
        CURRENT_FF_MODE_BEMF, omega_e, ld, lq, flux, measured
    );
    const qd_t reverse = FOC_CurrentFeedforward(
        CURRENT_FF_MODE_BOTH, -omega_e, ld, lq, flux, measured
    );

    check(off.q == 0.0f && off.d == 0.0f, "feedforward OFF mode");
    check(std::fabs(cross.q-cross_q) < 1.0e-7f
              && std::fabs(cross.d-cross_d) < 1.0e-7f,
          "feedforward CROSS mode");
    check(std::fabs(bemf.q-bemf_q) < 1.0e-7f && bemf.d == 0.0f,
          "feedforward BEMF mode");
    check(std::fabs(both.q-(cross_q+bemf_q)) < 1.0e-7f
              && std::fabs(both.d-cross_d) < 1.0e-7f,
          "feedforward BOTH mode");
    check(std::fabs(reverse.q+both.q) < 1.0e-7f
              && std::fabs(reverse.d+both.d) < 1.0e-7f,
          "feedforward speed sign");
    check(FOC_CurrentFeedforward(99U, omega_e, ld, lq, flux, measured).q == 0.0f,
          "invalid feedforward mode is off");

    const uint32_t modes[] = {
        CURRENT_FF_MODE_OFF,
        CURRENT_FF_MODE_BOTH,
        CURRENT_FF_MODE_CROSS,
        CURRENT_FF_MODE_BEMF,
    };
    for(uint32_t mode : modes){
        foc_pwm pwm(6000U);
        foc_pid pi_q(0.0f, 1200.0f, 0.0f, 10.0f, 30.0f, 30.0f);
        foc_pid pi_d(0.0f, 1200.0f, 0.0f, 10.0f, 30.0f, 30.0f);
        const float pi_q_request = pi_q.pi(1.0f, 0.0f, 0.01f);
        const float pi_d_request = pi_d.pi(-0.75f, 0.0f, 0.01f);
        const qd_t ff = FOC_CurrentFeedforward(
            mode, omega_e, ld, lq, flux, measured
        );
        const qd_t request = {pi_q_request+ff.q, pi_d_request+ff.d};
        const qd_t applied = pwm.run(request, 0.3f, 24.0f, SVPWM_MODE);

        check(pwm.voltage_scale < 1.0f,
              "feedforward tracking test reached voltage limit");
        pi_q.track_output(applied.q-ff.q);
        pi_d.track_output(applied.d-ff.d);
        const float tracked_q = pi_q.pi(0.0f, 0.0f, 0.01f);
        const float tracked_d = pi_d.pi(0.0f, 0.0f, 0.01f);
        check(std::fabs(tracked_q+ff.q-applied.q) < 1.0e-6f
                  && std::fabs(tracked_d+ff.d-applied.d) < 1.0e-6f,
              "feedforward mode PI tracking after voltage limit");
    }

    std::printf("feedforward at 3000 rad/s: cross=(%g,%g) bemf=%g V\n",
                cross_q, cross_d, bemf_q);
}

void test_field_weakening(void)
{
    constexpr float rs = 0.144818365f;
    constexpr float ld = 26.043582e-6f;
    constexpr float lq = 40.343765e-6f;
    constexpr float flux = 0.90795034e-3f;
    constexpr float current_limit = 5.0f;
    constexpr float sample_period = 1.0f/4000.0f;
    constexpr float omega_5000 = 5000.0f*2.0f*PI/60.0f*10.0f;
    constexpr float target = 24.0f*_1_DIV_SQRT_3*0.35f*0.95f;

    check(std::fabs(FOC_FieldWeakeningIqLimit(current_limit, -3.0f)-4.0f)
              < 1.0e-6f,
          "field weakening dq current circle");
    check(FOC_FieldWeakeningIqLimit(current_limit, -5.0f) == 0.0f,
          "field weakening d-axis current priority");

    const float forward_id = FOC_FieldWeakeningModelId(
        omega_5000, 1.0f, target, rs, ld, lq, flux
    );
    const float reverse_id = FOC_FieldWeakeningModelId(
        -omega_5000, -1.0f, target, rs, ld, lq, flux
    );
    const float forward_ud = rs*forward_id-omega_5000*lq;
    const float forward_uq = rs+omega_5000*(ld*forward_id+flux);
    check(std::fabs(forward_id+2.38585f) < 2.0e-4f,
          "identified voltage ellipse id solution at 5000 rpm");
    check(std::fabs(reverse_id-forward_id) < 1.0e-5f,
          "field weakening voltage ellipse reverse direction symmetry");
    check(std::fabs(std::hypot(forward_uq, forward_ud)-target) < 2.0e-5f,
          "field weakening voltage ellipse target magnitude");

    foc_field_weakening controller;
    controller.configure(
        rs, ld, lq, flux, 0.4f, 2400.0f,
        current_limit, 0.95f, 0.2695973f, 500.0f
    );
    controller.set_mode(FIELD_WEAKENING_MODE_FEEDBACK);

    float actual_id = 0.0f;
    foc_field_weakening_output_t result = {};
    const float plant_sensitivity = 0.12f;
    const float desired_id = -2.0f;
    for(int sample = 0; sample < 400; ++sample){
        const float request = target
                            + plant_sensitivity*(actual_id-desired_id);
        result = controller.run(
            request, target/0.95f, omega_5000, 1.0f, sample_period
        );
        actual_id += 0.75f*(result.id_ref-actual_id);
    }
    check(std::fabs(result.id_ref-desired_id) < 0.03f,
          "voltage-feedback field weakening PI convergence");
    check(std::hypot(result.id_ref, result.iq_limit)
              <= current_limit+1.0e-5f,
          "field weakening controller respects dq current circle");

    controller.set_mode(FIELD_WEAKENING_MODE_HYBRID);
    result = controller.run(
        target+0.1f, target/0.95f, omega_5000, 1.0f, sample_period
    );
    check(result.model_id < -2.0f
              && std::fabs(result.id_ref+0.125f) < 1.0e-6f,
          "hybrid field weakening rate-limits model entry");

    controller.set_mode(FIELD_WEAKENING_MODE_HYBRID);
    result = controller.run(
        target+0.1f, target/0.95f, omega_5000, 3.0f, sample_period
    );
    check(result.model_id >= -4.00001f,
          "hybrid model term respects remaining d-axis current");
    check(std::fabs(result.id_ref+0.125f) < 1.0e-6f,
          "hybrid field weakening rate-limits model exit");

    controller.set_mode(FIELD_WEAKENING_MODE_OFF);
    result = controller.run(
        target+1.0f, target/0.95f, omega_5000, 1.0f, sample_period
    );
    check(result.id_ref == 0.0f && result.iq_limit == current_limit,
          "field weakening OFF mode");

    std::printf("field weakening: model id=%g A, feedback id=%g A\n",
                forward_id, actual_id);
}

void test_identification_fits(void)
{
    constexpr float resistance = 0.105f;
    constexpr float voltage_offset = 0.032f;
    const float current[] = {-1.5f, 1.5f, -1.0f, 1.0f, -0.5f, 0.5f};
    float voltage[6] = {};
    for(size_t n = 0; n < 6; n++){
        const float sign_i = current[n] > 0.0f ? 1.0f : -1.0f;
        voltage[n] = resistance * current[n] + voltage_offset * sign_i;
    }

    foc_rs_fit_t rs_fit = {};
    check(foc_iden_fit_rs(current, voltage, 6U, &rs_fit),
          "Rs and inverter offset fit rejected valid data");
    check(std::fabs(rs_fit.resistance - resistance) < 1.0e-6f,
          "Rs fit accuracy");
    check(std::fabs(rs_fit.voltage_offset - voltage_offset) < 1.0e-6f,
          "inverter voltage offset fit accuracy");

    const float holdout_current[] = {-2.5f, 2.5f, -0.75f, 0.75f};
    float holdout_voltage[4] = {};
    for(size_t n = 0; n < 4; n++){
        const float sign_i = holdout_current[n] > 0.0f ? 1.0f : -1.0f;
        holdout_voltage[n] = resistance * holdout_current[n]
                           + voltage_offset * sign_i;
    }
    check(foc_iden_rs_prediction_rmse(holdout_current, holdout_voltage, 4U,
                                      rs_fit.resistance,
                                      rs_fit.voltage_offset) < 1.0e-6f,
          "Rs holdout prediction accuracy");

    constexpr float frequency = 500.0f;
    constexpr float omega = 2.0f * 3.14159265358979323846f * frequency;
    constexpr float inductance = 42.0e-6f;
    constexpr float current_amplitude = 1.2f;
    const float impedance_magnitude = std::hypot(resistance, omega * inductance);
    const float phase = std::atan2(omega * inductance, resistance);
    const float voltage_amplitude = current_amplitude * impedance_magnitude;
    constexpr uint32_t samples = 800U;
    const float voltage_cos = 0.0f;
    const float voltage_sin = voltage_amplitude * samples * 0.5f;
    const float current_cos = -current_amplitude * std::sin(phase) * samples * 0.5f;
    const float current_sin = current_amplitude * std::cos(phase) * samples * 0.5f;

    foc_impedance_fit_t impedance_fit = {};
    check(foc_iden_fit_impedance(voltage_cos, voltage_sin,
                                 current_cos, current_sin,
                                 samples, omega, &impedance_fit),
          "impedance fit rejected valid data");
    check(std::fabs(impedance_fit.resistance - resistance) < 1.0e-6f,
          "impedance resistance fit accuracy");
    check(std::fabs(impedance_fit.inductance - inductance) < 1.0e-9f,
          "inductance fit accuracy");

    constexpr float flux = 0.00091f;
    constexpr float intercept = 0.018f;
    const float speed[] = {-3000.0f, -1000.0f, 1000.0f, 3000.0f};
    float corrected_voltage[4] = {};
    for(size_t n = 0; n < 4; n++){
        corrected_voltage[n] = flux * speed[n] + intercept;
    }

    foc_flux_fit_t flux_fit = {};
    check(foc_iden_fit_flux(speed, corrected_voltage, 4U, &flux_fit),
          "flux fit rejected valid data");
    check(std::fabs(flux_fit.flux - flux) < 1.0e-8f,
          "flux fit accuracy");
    check(std::fabs(flux_fit.voltage_intercept - intercept) < 1.0e-6f,
          "flux voltage intercept fit accuracy");

    const float holdout_speed[] = {-2000.0f, 2000.0f};
    const float holdout_flux_voltage[] = {
        flux * holdout_speed[0] + intercept,
        flux * holdout_speed[1] + intercept
    };
    float holdout_max_error = -1.0f;
    check(foc_iden_flux_prediction_rmse(
              holdout_speed, holdout_flux_voltage, 2U,
              flux_fit.flux, flux_fit.voltage_intercept,
              &holdout_max_error) < 1.0e-6f,
          "flux holdout prediction accuracy");
    check(holdout_max_error < 1.0e-6f,
          "flux holdout maximum error");

    std::printf("identification fits: Rs=%g ohm Voffset=%g V L=%g uH flux=%g mWb\n",
                rs_fit.resistance, rs_fit.voltage_offset,
                impedance_fit.inductance * 1.0e6f, flux_fit.flux * 1.0e3f);
}

void test_current_pi_design(void)
{
    constexpr float rs = 0.144818365f;
    constexpr float ld = 26.043582e-6f;
    constexpr float lq = 40.343765e-6f;
    constexpr float bandwidth_hz = 1500.0f;
    constexpr float omega_c = 2.0f * 3.14159265358979323846f * bandwidth_hz;

    foc_current_pi_gains_t d_axis = {};
    foc_current_pi_gains_t q_axis = {};
    check(foc_current_pi_design(rs, ld, bandwidth_hz, &d_axis),
          "d-axis current PI design rejected valid parameters");
    check(foc_current_pi_design(rs, lq, bandwidth_hz, &q_axis),
          "q-axis current PI design rejected valid parameters");
    check(std::fabs(d_axis.kp - omega_c * ld) < 1.0e-6f,
          "d-axis current PI proportional gain");
    check(std::fabs(q_axis.kp - omega_c * lq) < 1.0e-6f,
          "q-axis current PI proportional gain");
    check(std::fabs(d_axis.ki - omega_c * rs) < 1.0e-3f,
          "current PI integral gain");
    check(q_axis.kp > d_axis.kp,
          "salient motor current PI uses distinct d/q proportional gains");

    std::printf("1500 Hz current PI: d=(%g, %g) q=(%g, %g)\n",
                d_axis.kp, d_axis.ki, q_axis.kp, q_axis.ki);
}

void test_encoder_count_angle_mapping(void)
{
    constexpr uint32_t timer_period = 4095U;
    constexpr uint32_t counts_per_revolution = timer_period + 1U;
    constexpr float two_pi = 6.28318530717958647692f;
    const float expected_step = two_pi / static_cast<float>(counts_per_revolution);

    const float angle_zero = foc_encoder_count_to_mech_angle(0U, timer_period);
    const float angle_one = foc_encoder_count_to_mech_angle(1U, timer_period);
    const float angle_last = foc_encoder_count_to_mech_angle(timer_period, timer_period);
    const float wrap_step = angle_zero + two_pi - angle_last;

    check(std::fabs(angle_zero) < 1.0e-7f,
          "encoder count zero angle");
    check(std::fabs(angle_one - expected_step) < 1.0e-7f,
          "encoder first count angle step");
    check(angle_last < two_pi,
          "encoder last count must remain below 2pi");
    check(std::fabs(angle_last - (two_pi - expected_step)) < 1.0e-6f,
          "encoder last count angle");
    check(std::fabs(wrap_step - expected_step) < 1.0e-6f,
          "encoder wrap angle step");

    float max_step_error = 0.0f;
    float previous = angle_zero;
    for (uint32_t count = 1U; count < counts_per_revolution; ++count) {
        const float angle = foc_encoder_count_to_mech_angle(count, timer_period);
        max_step_error = std::max(max_step_error,
                                  std::fabs((angle - previous) - expected_step));
        previous = angle;
    }
    max_step_error = std::max(max_step_error,
                              std::fabs((angle_zero + two_pi - previous) - expected_step));
    check(max_step_error < 1.0e-6f,
          "encoder angle steps are not uniform across wrap");

    std::printf("encoder angle: step=%g rad, max step error=%g rad\n",
                expected_step, max_step_error);
}

void test_encoder_speed_estimator_modes(void)
{
    constexpr float sample_period = 1.0f / 40000.0f;
    constexpr float electrical_speed = 200.0f;
    constexpr float electrical_count_step = _2PI * 10.0f / 4096.0f;
    foc_speed speed(10U, 1U, 0.0f);
    speed.SetAnglePre(0.0f, 0.0f, 0.0f);

    for (uint32_t index = 0U; index < 40000U; ++index) {
        const float true_angle = electrical_speed * index * sample_period;
        const float quantized = std::floor(true_angle / electrical_count_step)
                                * electrical_count_step;
        speed.AngleRefresh(Limit_Angle(quantized));
        speed.SpeedCalc(sample_period);
    }

    const float base = speed.EstimatorSpeedGet(FOC_SPEED_ESTIMATOR_BASE);
    const float win = speed.EstimatorSpeedGet(FOC_SPEED_ESTIMATOR_WIN_IIR);
    const float pll = speed.EstimatorSpeedGet(FOC_SPEED_ESTIMATOR_PLL);
    check(std::fabs(base - electrical_speed) < 1.0f,
          "BASE encoder speed estimator constant-speed error");
    check(std::fabs(win - electrical_speed) < 1.0f,
          "WIN-IIR encoder speed estimator constant-speed error");
    check(std::fabs(pll - electrical_speed) < 1.0f,
          "PLL encoder speed estimator constant-speed error");

    speed.EstimatorModeSet(FOC_SPEED_ESTIMATOR_WIN_IIR);
    speed.SpeedCalc(sample_period);
    check(speed.EstimatorModeGet() == FOC_SPEED_ESTIMATOR_WIN_IIR,
          "encoder speed estimator runtime mode");
    check(std::fabs(speed.speed_data.elec_speed
                    - speed.EstimatorSpeedGet(FOC_SPEED_ESTIMATOR_WIN_IIR)) < 1.0e-6f,
          "selected encoder speed estimator output");

    speed.AngleRefresh(1.5f);
    speed.SpeedCalc(sample_period);
    speed.clear();
    speed.AngleRefresh(1.5f);
    speed.SpeedCalc(sample_period);
    check(speed.EstimatorSpeedGet(FOC_SPEED_ESTIMATOR_BASE) == 0.0f,
          "BASE encoder speed estimator clear");
    check(speed.EstimatorSpeedGet(FOC_SPEED_ESTIMATOR_WIN_IIR) == 0.0f,
          "WIN-IIR encoder speed estimator clear");
    check(speed.EstimatorSpeedGet(FOC_SPEED_ESTIMATOR_PLL) == 0.0f,
          "PLL encoder speed estimator clear");
    check(speed.EstimatorSpeedGet(FOC_SPEED_ESTIMATOR_FUSED) == 0.0f,
          "FUSED encoder speed estimator clear");
    check(speed.speed_data.elec_speed == 0.0f,
          "encoder speed output after angle sync and clear");

    std::printf("encoder estimators at 200 rad/s: BASE=%g WIN=%g PLL=%g\n",
                base, win, pll);
}

void test_encoder_low_speed_fusion(void)
{
    constexpr float sample_period = 1.0f/40000.0f;
    constexpr uint32_t counts_per_revolution = 4096U;
    constexpr uint32_t pole_pairs = 10U;
    foc_speed speed(pole_pairs, 1U, 0.0f);
    speed.SetAnglePre(0.0f, 0.0f, 0.0f);
    speed.EstimatorModeSet(FOC_SPEED_ESTIMATOR_FUSED);

    float mechanical_count_position = 0.0f;
    int32_t previous_count = 0;
    float win_error_energy = 0.0f;
    float fused_error_energy = 0.0f;
    uint32_t measured_samples = 0U;

    for(uint32_t index = 0U; index < 40000U; ++index){
        constexpr float mechanical_rpm = 50.0f;
        const float count_rate = mechanical_rpm/60.0f
                               *(float)counts_per_revolution;
        mechanical_count_position += count_rate*sample_period;
        const int32_t count = (int32_t)floorf(mechanical_count_position)
                            %(int32_t)counts_per_revolution;
        int32_t count_delta = count-previous_count;
        if(count_delta > (int32_t)counts_per_revolution/2){
            count_delta -= (int32_t)counts_per_revolution;
        }
        previous_count = count;

        const float mech_angle = (float)count*_2PI
                               /(float)counts_per_revolution;
        speed.AngleRefresh(Limit_Angle(mech_angle*(float)pole_pairs));
        speed.SpeedCalc(sample_period, count_delta, counts_per_revolution);

        if(index >= 20000U){
            const float reference = mechanical_rpm*_2PI/60.0f
                                  *(float)pole_pairs;
            const float win_error = speed.EstimatorSpeedGet(
                FOC_SPEED_ESTIMATOR_WIN_IIR)-reference;
            const float fused_error = speed.EstimatorSpeedGet(
                FOC_SPEED_ESTIMATOR_FUSED)-reference;
            win_error_energy += win_error*win_error;
            fused_error_energy += fused_error*fused_error;
            ++measured_samples;
        }
    }

    const float win_rms = sqrtf(win_error_energy/(float)measured_samples);
    const float fused_rms = sqrtf(fused_error_energy/(float)measured_samples);
    check(fused_rms < 0.80f*win_rms,
          "FUSED estimator reduces 50 rpm quantization ripple");
    check(std::fabs(speed.speed_data.elec_speed
                    - speed.EstimatorSpeedGet(FOC_SPEED_ESTIMATOR_FUSED))
          < 1.0e-6f,
          "FUSED estimator runtime selection");

    for(uint32_t index = 0U; index < 800U; ++index){
        speed.AngleRefresh(speed.speed_data.elec_angle);
        speed.SpeedCalc(sample_period, 0, counts_per_revolution);
    }
    check(std::fabs(speed.EstimatorSpeedGet(FOC_SPEED_ESTIMATOR_FUSED)) < 0.05f,
          "FUSED estimator zero-speed timeout");

    for(uint32_t index = 0U; index < 8000U; ++index){
        constexpr float mechanical_rpm = -50.0f;
        const float count_rate = mechanical_rpm/60.0f
                               *(float)counts_per_revolution;
        mechanical_count_position += count_rate*sample_period;
        int32_t count = (int32_t)floorf(mechanical_count_position)
                      %(int32_t)counts_per_revolution;
        if(count < 0){
            count += (int32_t)counts_per_revolution;
        }
        int32_t count_delta = count-previous_count;
        if(count_delta > (int32_t)counts_per_revolution/2){
            count_delta -= (int32_t)counts_per_revolution;
        }else if(count_delta < -(int32_t)counts_per_revolution/2){
            count_delta += (int32_t)counts_per_revolution;
        }
        previous_count = count;
        const float mech_angle = (float)count*_2PI
                               /(float)counts_per_revolution;
        speed.AngleRefresh(Limit_Angle(mech_angle*(float)pole_pairs));
        speed.SpeedCalc(sample_period, count_delta, counts_per_revolution);
    }
    check(speed.EstimatorSpeedGet(FOC_SPEED_ESTIMATOR_FUSED) < 0.0f,
          "FUSED estimator reversal sign");

    std::printf("encoder low-speed fusion: WIN RMS=%g, FUSED RMS=%g rad/s\n",
                win_rms, fused_rms);
}

float circular_error(float estimate, float reference)
{
    return std::atan2(std::sin(estimate-reference),
                      std::cos(estimate-reference));
}

void test_sensorless_shadow_observers(void)
{
    constexpr float sample_period = 1.0f/40000.0f;
    constexpr float rs = 0.144818365f;
    constexpr float ld = 26.043582e-6f;
    constexpr float lq = 40.343765e-6f;
    constexpr float pm_flux = 0.90795034e-3f;
    constexpr float id = -0.5f;
    constexpr float iq = 0.75f;
    constexpr int sample_count = 40000;
    constexpr int evaluation_start = 24000;

    const float mechanical_rpm_values[] = {
        1500.0f, -1500.0f, 6500.0f, -6500.0f,
    };
    for (int speed_index = 0; speed_index < 4; ++speed_index) {
        const float mechanical_rpm = mechanical_rpm_values[speed_index];
        const float electrical_speed = mechanical_rpm*_2PI/60.0f*10.0f;
        foc_esmo_observer esmo(rs, lq, 12.0f, 0.80f, 300.0f, 60.0f);
        foc_active_flux_observer active_flux(
            rs, ld, lq, pm_flux, 100.0f, 200.0f
        );

        float true_angle = 1.2f;
        float esmo_error_sq = 0.0f;
        float flux_error_sq = 0.0f;
        float esmo_speed_error_sq = 0.0f;
        float flux_speed_error_sq = 0.0f;
        int evaluated = 0;

        for (int sample = 0; sample < sample_count; ++sample) {
            const qd_t current_qd = {iq, id};
            const qd_t voltage_qd = {
                rs*iq+electrical_speed*(pm_flux+ld*id),
                rs*id-electrical_speed*lq*iq,
            };
            const float previous_angle = Limit_Angle(
                true_angle-electrical_speed*sample_period
            );
            const alphabeta_t current = FOC_Rev_Park(current_qd, true_angle);
            const alphabeta_t previous_voltage = FOC_Rev_Park(
                voltage_qd, previous_angle
            );
            const foc_sensorless_output_t esmo_output = esmo.update(
                previous_voltage, current, sample_period
            );
            const foc_sensorless_output_t flux_output = active_flux.update(
                previous_voltage, current, sample_period
            );

            if (sample >= evaluation_start) {
                const float esmo_error = circular_error(esmo_output.angle,
                                                        true_angle);
                const float flux_error = circular_error(flux_output.angle,
                                                        true_angle);
                esmo_error_sq += esmo_error*esmo_error;
                flux_error_sq += flux_error*flux_error;
                esmo_speed_error_sq += (esmo_output.speed-electrical_speed)
                                      *(esmo_output.speed-electrical_speed);
                flux_speed_error_sq += (flux_output.speed-electrical_speed)
                                      *(flux_output.speed-electrical_speed);
                ++evaluated;
            }

            true_angle = Limit_Angle(
                true_angle+electrical_speed*sample_period
            );
        }

        const float esmo_angle_rms = std::sqrt(esmo_error_sq/evaluated);
        const float flux_angle_rms = std::sqrt(flux_error_sq/evaluated);
        const float esmo_speed_rms = std::sqrt(esmo_speed_error_sq/evaluated);
        const float flux_speed_rms = std::sqrt(flux_speed_error_sq/evaluated);

        check(esmo_angle_rms < 8.0f*PI/180.0f,
              "eSMO constant-speed angle error");
        check(flux_angle_rms < 5.0f*PI/180.0f,
              "active-flux constant-speed angle error");
        check(esmo_speed_rms < 0.03f*std::fabs(electrical_speed),
              "eSMO constant-speed speed error");
        check(flux_speed_rms < 0.03f*std::fabs(electrical_speed),
              "active-flux constant-speed speed error");

        std::printf(
            "sensorless %+.0f rpm: eSMO angle=%g deg speed=%g rad/s, "
            "active-flux angle=%g deg speed=%g rad/s\n",
            mechanical_rpm, esmo_angle_rms*180.0f/PI, esmo_speed_rms,
            flux_angle_rms*180.0f/PI, flux_speed_rms
        );
    }
}

void test_sensorless_if_handoff(void)
{
    constexpr float sample_period = 1.0f/40000.0f;
    constexpr float handoff_speed = 1000.0f*_2PI/60.0f*10.0f;
    constexpr float acceleration = 3000.0f*_2PI/60.0f*10.0f;

    for (int direction_index = 0; direction_index < 2; ++direction_index) {
        const float direction = (direction_index == 0) ? 1.0f : -1.0f;
        foc_sensorless_startup startup;
        startup.configure(0.8f, 1.0f, 0.15f, 5.0f, 0.2f, acceleration,
                          handoff_speed, 20.0f*PI/180.0f,
                          100.0f*_2PI/60.0f*10.0f,
                          0.02f, 0.02f, 0.5f);
        startup.start(direction);

        foc_sensorless_startup_output_t output = startup.get_output();
        float previous_control_angle = 0.0f;
        float maximum_control_step = 0.0f;
        bool saw_alignment_current = false;
        bool saw_open_loop_current = false;

        for (int sample = 0; sample < 40000; ++sample) {
            const float observer_angle = Limit_Angle(
                output.virtual_angle+direction*0.10f
            );
            const float observer_speed = output.virtual_speed;
            output = startup.update(observer_angle, observer_speed,
                                    sample_period);

            const float control_step = std::fabs(circular_error(
                output.control_angle, previous_control_angle
            ));
            maximum_control_step = std::max(maximum_control_step,
                                            control_step);
            previous_control_angle = output.control_angle;
            saw_alignment_current |=
                (output.state == FOC_SENSORLESS_ALIGN)
                && (output.current_reference.d > 0.79f);
            saw_open_loop_current |=
                (output.state == FOC_SENSORLESS_OPEN_LOOP)
                && (direction*output.current_reference.q > 0.99f);

            if(output.state == FOC_SENSORLESS_CLOSED_LOOP){
                break;
            }
        }

        check(output.state == FOC_SENSORLESS_CLOSED_LOOP,
              "sensorless I/f reaches closed loop");
        check(saw_alignment_current,
              "sensorless I/f applies d-axis alignment current");
        check(saw_open_loop_current,
              "sensorless I/f applies signed q-axis current");
        check(maximum_control_step < 0.04f,
              "sensorless observer handoff angle continuity");
        check(std::fabs(output.virtual_speed-direction*handoff_speed) < 1.0f,
              "sensorless I/f handoff speed");

        std::printf("sensorless I/f %s: state=%u max angle step=%g deg\n",
                    direction > 0.0f ? "forward" : "reverse",
                    static_cast<unsigned>(output.state),
                    maximum_control_step*180.0f/PI);
    }
}

void test_hfi_saliency_observer(void)
{
    constexpr float sample_period = 1.0f/40000.0f;
    constexpr float rs = 0.144818365f;
    constexpr float ld = 26.043582e-6f;
    constexpr float lq = 40.343765e-6f;
    constexpr float true_angle = 1.1f;

    foc_hfi_observer mode_observer(
        ld, lq, 0.5f, 2U, 1000.0f, 100.0f
    );
    check(mode_observer.mode_get() == FOC_HFI_MODE_OFF,
          "HFI reset mode is OFF");
    mode_observer.start_d_axis_scan_carrier(true_angle);
    check(mode_observer.mode_get() == FOC_HFI_MODE_D_AXIS_SCAN
          && !mode_observer.tracking(),
          "HFI d-axis scan is diagnostic only");
    mode_observer.start_tensor_scan_carrier(true_angle);
    check(mode_observer.mode_get() == FOC_HFI_MODE_TENSOR_SCAN
          && !mode_observer.tracking(),
          "HFI tensor scan is diagnostic only");
    mode_observer.start_encoder_calibrated_tensor_tracking(true_angle, 0.0f);
    check(mode_observer.mode_get() == FOC_HFI_MODE_ENCODER_CALIBRATION
          && !mode_observer.tracking(),
          "HFI encoder calibration is not yet a control source");
    mode_observer.stop();
    check(!mode_observer.running(), "HFI stop returns to OFF");

    for (int direction = -1; direction <= 1; direction += 2) {
        foc_hfi_observer observer(
            ld, lq, 0.5f, 2U, 1000.0f, 100.0f
        );
        observer.start_tensor_tracking(
            true_angle+direction*45.0f*PI/180.0f
        );
        check(observer.mode_get() == FOC_HFI_MODE_TENSOR_TRACK
              && observer.tracking(),
              "HFI tensor tracking mode owns the angle estimate");

        alphabeta_t current = {0.0f, 0.0f};
        alphabeta_t applied_voltage = {0.0f, 0.0f};
        foc_hfi_output_t estimate = observer.get_output();
        for (int sample = 0; sample < 12000; ++sample) {
            const qd_t current_qd = Foc_Park(current, true_angle);
            const qd_t voltage_qd = Foc_Park(applied_voltage, true_angle);
            const qd_t next_current_qd = {
                current_qd.q+sample_period
                    *(voltage_qd.q-rs*current_qd.q)/lq,
                current_qd.d+sample_period
                    *(voltage_qd.d-rs*current_qd.d)/ld,
            };
            current = FOC_Rev_Park(next_current_qd, true_angle);
            estimate = observer.update(
                current, applied_voltage, sample_period
            );
            applied_voltage = estimate.injection_voltage;
        }

        const float error = circular_error(estimate.angle, true_angle);
        check(std::fabs(error) < 2.0f*PI/180.0f,
              "HFI converges from a 45 degree axis error");
        check(std::fabs(estimate.speed) < 5.0f,
              "HFI standstill speed converges to zero");
        check(estimate.d_response > 0.05f,
              "HFI detects a d-axis carrier response");
        std::printf(
            "HFI seed %+.0f deg: angle error=%g deg speed=%g rad/s "
            "response=(%g,%g) A/V/sample\n",
            direction*45.0f, error*180.0f/PI, estimate.speed,
            estimate.d_response, estimate.q_response
        );

        check(observer.start_d_axis_tracking(),
              "HFI transitions from tensor calibration to d-axis tracking");
        for (int sample = 0; sample < 12000; ++sample) {
            const qd_t current_qd = Foc_Park(current, true_angle);
            const qd_t voltage_qd = Foc_Park(applied_voltage, true_angle);
            const qd_t next_current_qd = {
                current_qd.q+sample_period
                    *(voltage_qd.q-rs*current_qd.q)/lq,
                current_qd.d+sample_period
                    *(voltage_qd.d-rs*current_qd.d)/ld,
            };
            current = FOC_Rev_Park(next_current_qd, true_angle);
            estimate = observer.update(
                current, applied_voltage, sample_period
            );
            applied_voltage = estimate.injection_voltage;
        }
        const float d_axis_error = circular_error(
            estimate.angle, true_angle
        );
        check(observer.mode_get() == FOC_HFI_MODE_D_AXIS_TRACK,
              "HFI d-axis carrier owns the tracked angle");
        check(std::fabs(d_axis_error) < 2.0f*PI/180.0f,
              "HFI d-axis carrier preserves the tensor angle estimate");
        std::printf(
            "HFI d-axis track: angle error=%g deg speed=%g rad/s\n",
            d_axis_error*180.0f/PI, estimate.speed
        );

        float maximum_drift_error = 0.0f;
        int q_axis_carrier_samples = 0;
        for (int sample = 0; sample < 24000; ++sample) {
            const qd_t current_qd = Foc_Park(current, true_angle);
            const qd_t voltage_qd = Foc_Park(applied_voltage, true_angle);
            const float sampling_cross = 0.155f
                +0.035f*std::sin(_2PI*sample/8000.0f);
            const qd_t next_current_qd = {
                current_qd.q+sample_period
                    *(voltage_qd.q-rs*current_qd.q)/lq
                    +sampling_cross*voltage_qd.d,
                current_qd.d+sample_period
                    *(voltage_qd.d-rs*current_qd.d)/ld
                    -sampling_cross*voltage_qd.q,
            };
            current = FOC_Rev_Park(next_current_qd, true_angle);
            estimate = observer.update(
                current, applied_voltage, sample_period
            );
            applied_voltage = estimate.injection_voltage;
            const qd_t carrier_qd = Foc_Park(
                applied_voltage, estimate.angle
            );
            q_axis_carrier_samples +=
                std::fabs(carrier_qd.q) > std::fabs(carrier_qd.d);
            if (sample >= 4000) {
                maximum_drift_error = std::max(
                    maximum_drift_error,
                    std::fabs(circular_error(estimate.angle, true_angle))
                );
            }
        }
        check(maximum_drift_error < 8.0f*PI/180.0f,
              "HFI sparse q-axis refresh rejects measured sampling drift");
        const float q_axis_carrier_fraction =
            q_axis_carrier_samples/24000.0f;
        check(std::fabs(q_axis_carrier_fraction-0.25f) < 0.001f,
              "HFI tracking carrier remains 75% d-axis dominant");
        std::printf(
            "HFI sampling drift: max angle error=%g deg q fraction=%g\n",
            maximum_drift_error*180.0f/PI, q_axis_carrier_fraction
        );

        observer.set_speed_tracking_direction(static_cast<float>(direction));
        const float constrained_start = estimate.angle;
        const float shifted_true_angle = true_angle
                                       +(-direction)*20.0f*PI/180.0f;
        for (int sample = 0; sample < 4000; ++sample) {
            const qd_t current_qd = Foc_Park(current, shifted_true_angle);
            const qd_t voltage_qd = Foc_Park(
                applied_voltage, shifted_true_angle
            );
            const qd_t next_current_qd = {
                current_qd.q+sample_period
                    *(voltage_qd.q-rs*current_qd.q)/lq,
                current_qd.d+sample_period
                    *(voltage_qd.d-rs*current_qd.d)/ld,
            };
            current = FOC_Rev_Park(next_current_qd, shifted_true_angle);
            estimate = observer.update(
                current, applied_voltage, sample_period
            );
            applied_voltage = estimate.injection_voltage;
        }
        const float recovery_error = circular_error(
            estimate.angle, shifted_true_angle
        );
        const float recovery_motion = circular_error(
            estimate.angle, constrained_start
        );
        check(std::fabs(recovery_error) < 3.0f*PI/180.0f
              && direction*recovery_motion < -10.0f*PI/180.0f,
              "HFI phase correction can recover opposite to command direction");
    }
}

void test_hfi_ipd_polarity(void)
{
    constexpr float sample_period = 1.0f/40000.0f;
    constexpr float rs = 0.144818365f;
    constexpr float ld = 26.043582e-6f;
    constexpr float lq = 40.343765e-6f;
    constexpr float kp_d = 0.155978918f;
    constexpr float ki_d = 1086.465606f;
    constexpr float kp_q = 0.251296400f;
    constexpr float ki_q = 1151.639999f;
    const float true_angles[] = {0.35f, 1.70f, 3.40f, 5.20f};

    for (float true_angle : true_angles) {
        foc_hfi_observer observer(
            ld, lq, 0.5f, 2U, 1000.0f, 100.0f,
            1.0f, 50.0f, 1.0f, 0.0f, 0.5f
        );
        observer.start_sensorless(0.0f);

        alphabeta_t current = {0.0f, 0.0f};
        alphabeta_t applied_voltage = {0.0f, 0.0f};
        foc_hfi_output_t estimate = observer.get_output();
        float integral_d = 0.0f;
        float integral_q = 0.0f;
        float peak_current = 0.0f;
        for (int sample = 0; sample < 22000; ++sample) {
            estimate = observer.update(
                current, applied_voltage, sample_period
            );
            const qd_t estimated_current = Foc_Park(
                current, estimate.angle
            );
            const float error_d = estimate.d_current_reference
                                -estimated_current.d;
            const float error_q = -estimated_current.q;
            integral_d += ki_d*error_d*sample_period;
            integral_q += ki_q*error_q*sample_period;
            const qd_t control_voltage = {
                kp_q*error_q+integral_q,
                kp_d*error_d+integral_d,
            };
            applied_voltage = FOC_Rev_Park(
                control_voltage, estimate.angle
            );
            applied_voltage.alpha += estimate.injection_voltage.alpha;
            applied_voltage.beta += estimate.injection_voltage.beta;

            const qd_t true_current = Foc_Park(current, true_angle);
            const qd_t true_voltage = Foc_Park(
                applied_voltage, true_angle
            );
            const float saturation = 1.0f
                +0.40f*tanhf(true_current.d/0.35f);
            const float effective_ld = ld/saturation;
            const qd_t next_current = {
                true_current.q+sample_period
                    *(true_voltage.q-rs*true_current.q)/lq,
                true_current.d+sample_period
                    *(true_voltage.d-rs*true_current.d)/effective_ld,
            };
            current = FOC_Rev_Park(next_current, true_angle);
            peak_current = std::max(
                peak_current, std::hypot(next_current.q, next_current.d)
            );
        }

        const float error = circular_error(estimate.angle, true_angle);
        check(estimate.axis_valid && estimate.polarity_valid
              && estimate.ready,
              "HFI IPD reaches polarity-valid d-axis tracking");
        check(observer.mode_get() == FOC_HFI_MODE_D_AXIS_TRACK,
              "HFI IPD ends in d-axis carrier mode");
        check(std::fabs(error) < 5.0f*PI/180.0f,
              "HFI IPD selects the magnetic d-axis polarity");
        check(std::fabs(estimate.polarity_contrast) > 0.05f,
              "HFI IPD measures a decisive saturation contrast");
        check(std::fabs(estimate.speed) < 0.1f,
              "HFI IPD holds speed at zero before control authorization");
        check(peak_current < 3.5f,
              "HFI IPD current remains inside the measured test envelope");
        std::printf(
            "HFI IPD true=%g deg error=%g deg contrast=%g peak=%g A\n",
            true_angle*180.0f/PI, error*180.0f/PI,
            estimate.polarity_contrast, peak_current
        );
    }
}

void test_hfi_flux_handoff(void)
{
    constexpr float sample_period = 1.0f/40000.0f;
    constexpr float mechanical_to_electrical = _2PI/60.0f*10.0f;

    for (int direction_index = 0; direction_index < 2; ++direction_index) {
        const float direction = (direction_index == 0) ? 1.0f : -1.0f;
        foc_hfi_flux_handoff handoff;
        float rotor_angle = 0.7f;
        float rotor_speed = 0.0f;
        float previous_control_angle = rotor_angle;
        float maximum_control_step = 0.0f;
        bool saw_hfi_stop = false;
        bool saw_hfi_start = false;
        float hfi_restart_rpm = 0.0f;

        handoff.start(rotor_angle, rotor_speed);
        foc_hfi_flux_output_t output = handoff.get_output();
        for (int sample = 0; sample < 80000; ++sample) {
            const float time = sample*sample_period;
            float mechanical_rpm;
            if (time < 0.30f) {
                mechanical_rpm = direction*600.0f*time/0.30f;
            } else if (time < 0.55f) {
                mechanical_rpm = direction*600.0f;
            } else {
                mechanical_rpm = direction*600.0f
                               *(1.0f-(time-0.55f)/0.35f);
                if (direction*mechanical_rpm < 350.0f) {
                    mechanical_rpm = direction*350.0f;
                }
            }
            rotor_speed = mechanical_rpm*mechanical_to_electrical;
            rotor_angle = Limit_Angle(
                rotor_angle+rotor_speed*sample_period
            );
            const float hfi_angle = Limit_Angle(
                rotor_angle-direction*5.0f*PI/180.0f
            );
            const float flux_angle = Limit_Angle(
                rotor_angle+direction*4.0f*PI/180.0f
            );
            const float hfi_speed = rotor_speed-direction*15.0f
                                                 *mechanical_to_electrical;
            const float flux_speed = rotor_speed+direction*10.0f
                                                  *mechanical_to_electrical;
            output = handoff.update(
                hfi_angle, hfi_speed, true,
                flux_angle, flux_speed, true, rotor_speed, sample_period
            );
            saw_hfi_stop |= output.stop_hfi;
            if (output.start_hfi) {
                saw_hfi_start = true;
                hfi_restart_rpm = std::fabs(mechanical_rpm);
            }
            if (sample > 0) {
                maximum_control_step = std::max(
                    maximum_control_step,
                    std::fabs(circular_error(output.control_angle,
                                             previous_control_angle))
                );
            }
            previous_control_angle = output.control_angle;
        }

        check(output.state == FOC_HFI_FLUX_HFI_ACTIVE,
              "HFI/Active Flux handoff returns to HFI below overlap");
        check(saw_hfi_stop && saw_hfi_start,
              "HFI/Active Flux handoff gates injection in both directions");
        check((hfi_restart_rpm >= 520.0f) && (hfi_restart_rpm <= 530.0f),
              "HFI restart uses the frozen 525 rpm command threshold");
        check(!output.failed,
              "HFI/Active Flux handoff remains inside observer validity");
        check(maximum_control_step < 0.04f,
              "HFI/Active Flux handoff angle remains phase continuous");
        std::printf(
            "HFI/Active Flux %s: state=%u max angle step=%g deg\n",
            direction > 0.0f ? "forward" : "reverse",
            static_cast<unsigned>(output.state),
            maximum_control_step*180.0f/PI
        );
    }
}

void test_hfi_current_assisted_startup(void)
{
    constexpr float sample_period = 1.0f/4000.0f;
    constexpr float mechanical_to_electrical = _2PI/60.0f*10.0f;
    constexpr float target_speed = 350.0f*mechanical_to_electrical;

    for (int direction_index = 0; direction_index < 2; ++direction_index) {
        const float direction = (direction_index == 0) ? 1.0f : -1.0f;
        foc_hfi_speed_startup startup;
        startup.configure(
            0.35f, 250.0f*mechanical_to_electrical, 0.0016f,
            300.0f*mechanical_to_electrical,
            250.0f*mechanical_to_electrical, 0.020f
        );
        startup.clear();

        float measured_speed = 0.0f;
        float peak_current = 0.0f;
        bool started_tracking = false;
        foc_hfi_speed_startup_output_t output = startup.get_output();
        for (int sample = 0; sample < 12000; ++sample) {
            const bool observer_ready = sample >= 400;
            output = startup.update(
                direction*target_speed, measured_speed,
                observer_ready, sample_period
            );
            started_tracking |= output.start_speed_tracking;
            peak_current = std::max(
                peak_current, std::fabs(output.current_reference)
            );

            /* A low-inertia no-load plant is sufficient to verify startup
             * direction, bounded current and the PI-entry state transition. */
            measured_speed += sample_period*(
                8000.0f*output.current_reference-0.2f*measured_speed
            );
            if (output.enter_speed_loop) {
                break;
            }
        }

        check(started_tracking,
              "HFI startup enables speed tracking after angle qualification");
        check(output.state == FOC_HFI_SPEED_CLOSED_LOOP,
              "HFI current-assisted startup reaches the low-speed PI");
        check(direction*measured_speed
              >= 245.0f*mechanical_to_electrical,
              "HFI startup reaches the measured-speed entry region");
        check(peak_current <= 0.3501f,
              "HFI startup respects the q-axis current limit");
        check(direction*output.current_reference >= 0.0f,
              "HFI startup applies torque in the requested direction");
    }

    foc_hfi_speed_startup low_speed;
    low_speed.clear();
    foc_hfi_speed_startup_output_t low_output = low_speed.get_output();
    for (int sample = 0; sample < 8000; ++sample) {
        low_output = low_speed.update(
            100.0f*mechanical_to_electrical, 0.0f,
            true, sample_period
        );
    }
    check(low_output.state == FOC_HFI_SPEED_ACCELERATING,
          "sub-entry HFI command remains on bounded current control");
    check(std::fabs(low_output.current_reference) <= 0.3501f,
          "sub-entry HFI current remains bounded");
}

void test_fault_manager_and_qualification(void)
{
    foc_fault_manager faults;
    foc_fault_snapshot_t first = {};
    first.timestamp_ms = 1234U;
    first.iq = 10.7f;
    first.vbus = 23.8f;

    check(faults.set_condition(
              FOC_FAULT_SOFTWARE_OVERCURRENT, true, true, first),
          "first active trip reports a rising edge");
    check(faults.active_trip(), "trip fault is active");
    check(!faults.acknowledge(),
          "active trip cannot be acknowledged");

    foc_fault_snapshot_t later = first;
    later.timestamp_ms = 2000U;
    later.vbus = 29.0f;
    check(faults.set_condition(
              FOC_FAULT_BUS_OVERVOLTAGE, true, true, later),
          "second active trip reports its rising edge");
    foc_fault_status_t status = faults.get();
    check(status.first_fault == FOC_FAULT_SOFTWARE_OVERCURRENT
          && status.first_snapshot.timestamp_ms == 1234U
          && std::fabs(status.first_snapshot.iq-10.7f) < 1.0e-6f,
          "later faults do not overwrite the first-trip snapshot");

    (void)faults.set_condition(
        FOC_FAULT_SOFTWARE_OVERCURRENT, false, true, later
    );
    (void)faults.set_condition(
        FOC_FAULT_BUS_OVERVOLTAGE, false, true, later
    );
    check(!faults.active_trip(), "cleared conditions leave a latched fault");
    check(faults.acknowledge(), "inactive trips can be acknowledged");
    status = faults.get();
    check(status.latched_bits == 0U
          && status.first_fault == FOC_FAULT_NONE,
          "acknowledge clears trip history and first snapshot");

    (void)faults.set_condition(
        FOC_FAULT_SPEED_TRACKING, true, false, first
    );
    check(!faults.active_trip() && faults.acknowledge(),
          "an active warning does not block fault acknowledgement");
    status = faults.get();
    check(status.active_bits == FOC_FAULT_SPEED_TRACKING
          && status.latched_bits == FOC_FAULT_SPEED_TRACKING,
          "active warning remains visible after acknowledgement");

    foc_fault_monitor monitor;
    foc_fault_monitor_config_t config = {};
    config.bus_undervoltage = 18.0f;
    config.bus_undervoltage_clear = 19.0f;
    config.bus_overvoltage = 28.0f;
    config.bus_overvoltage_clear = 27.0f;
    config.bus_set_time = 0.003f;
    config.bus_clear_time = 0.002f;
    config.overspeed = 110.0f;
    config.overspeed_clear = 100.0f;
    config.overspeed_set_time = 0.003f;
    config.overspeed_clear_time = 0.002f;
    config.deadline = 25.0e-6f;
    config.deadline_clear = 23.0e-6f;
    config.deadline_set_time = 0.003f;
    config.deadline_clear_time = 0.002f;
    config.speed_tracking_error = 20.0f;
    config.speed_tracking_clear = 10.0f;
    config.speed_tracking_set_time = 0.003f;
    config.speed_tracking_clear_time = 0.002f;
    config.position_tracking_error = 0.20f;
    config.position_tracking_clear = 0.10f;
    config.position_tracking_set_time = 0.003f;
    config.position_tracking_clear_time = 0.002f;
    monitor.configure(config);

    foc_fault_monitor_input_t input = {};
    input.power_control_active = true;
    input.vbus = 17.5f;
    uint32_t monitored = 0U;
    for(int sample = 0; sample < 2; ++sample){
        monitored = monitor.update(input, 0.001f);
    }
    check((monitored & FOC_FAULT_BUS_UNDERVOLTAGE) == 0U,
          "bus fault waits for its qualification time");
    monitored = monitor.update(input, 0.001f);
    check((monitored & FOC_FAULT_BUS_UNDERVOLTAGE) != 0U,
          "qualified undervoltage becomes active");

    input.vbus = 18.5f;
    for(int sample = 0; sample < 4; ++sample){
        monitored = monitor.update(input, 0.001f);
    }
    check((monitored & FOC_FAULT_BUS_UNDERVOLTAGE) != 0U,
          "undervoltage hysteresis rejects an in-band recovery");
    input.vbus = 19.2f;
    monitored = monitor.update(input, 0.001f);
    check((monitored & FOC_FAULT_BUS_UNDERVOLTAGE) != 0U,
          "bus recovery is time-qualified");
    monitored = monitor.update(input, 0.001f);
    check((monitored & FOC_FAULT_BUS_UNDERVOLTAGE) == 0U,
          "qualified bus recovery clears the active condition");

    input.vbus = 24.0f;
    input.speed_tracking_enabled = true;
    input.speed_error = 25.0f;
    for(int sample = 0; sample < 3; ++sample){
        monitored = monitor.update(input, 0.001f);
    }
    check((monitored & FOC_FAULT_SPEED_TRACKING) != 0U,
          "tracking error qualifies as a warning");
    input.speed_tracking_enabled = false;
    for(int sample = 0; sample < 2; ++sample){
        monitored = monitor.update(input, 0.001f);
    }
    check((monitored & FOC_FAULT_SPEED_TRACKING) == 0U,
          "leaving speed control clears its warning after qualification");

    foc_fault_monitor combined_monitor;
    config.bus_set_time = 0.001f;
    config.bus_clear_time = 0.001f;
    config.overspeed_set_time = 0.001f;
    config.overspeed_clear_time = 0.001f;
    config.deadline_set_time = 0.001f;
    config.deadline_clear_time = 0.001f;
    config.position_tracking_set_time = 0.001f;
    config.position_tracking_clear_time = 0.001f;
    combined_monitor.configure(config);
    input = {};
    input.power_control_active = true;
    input.vbus = 29.0f;
    input.speed = -120.0f;
    input.execution_time = 26.0e-6f;
    input.position_tracking_enabled = true;
    input.position_error = -0.25f;
    monitored = combined_monitor.update(input, 0.001f);
    check((monitored & (FOC_FAULT_BUS_OVERVOLTAGE
                        | FOC_FAULT_OVERSPEED
                        | FOC_FAULT_CONTROL_DEADLINE
                        | FOC_FAULT_POSITION_TRACKING))
              == (FOC_FAULT_BUS_OVERVOLTAGE
                  | FOC_FAULT_OVERSPEED
                  | FOC_FAULT_CONTROL_DEADLINE
                  | FOC_FAULT_POSITION_TRACKING),
          "overvoltage, signed overspeed, deadline and position warning qualify");
    input.power_control_active = false;
    input.vbus = 24.0f;
    input.speed = 0.0f;
    input.execution_time = 0.0f;
    input.position_tracking_enabled = false;
    monitored = combined_monitor.update(input, 0.001f);
    check(monitored == 0U,
          "all monitored conditions clear at their recovery boundaries");
}

} // namespace

int main(void)
{
    test_transforms();
    test_svpwm_duty_range();
    test_pi_integrator_limit();
    test_runtime_gains_and_speed_ramp();
    test_position_ramp();
    test_position_trapezoid();
    test_position_scurve();
    test_voltage_vector_limit_and_tracking();
    test_current_feedforward_modes();
    test_field_weakening();
    test_identification_fits();
    test_current_pi_design();
    test_encoder_count_angle_mapping();
    test_encoder_speed_estimator_modes();
    test_encoder_low_speed_fusion();
    test_sensorless_shadow_observers();
    test_sensorless_if_handoff();
    test_hfi_saliency_observer();
    test_hfi_ipd_polarity();
    test_hfi_current_assisted_startup();
    test_hfi_flux_handoff();
    test_fault_manager_and_qualification();

    if (failures != 0) {
        std::printf("algorithm regression: %d failure(s)\n", failures);
        return 1;
    }

    std::puts("algorithm regression: PASS");
    return 0;
}
