#include "foc_math.hpp"
#include "foc_pid.hpp"
#include "foc_pwm.hpp"
#include "foc_ctrl.hpp"
#include "foc_iden_math.hpp"
#include "foc_current_tuning.hpp"
#include "foc_encoder_math.hpp"
#include "foc_current_feedforward.hpp"
#include "foc_speed.hpp"

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
    check(speed.speed_data.elec_speed == 0.0f,
          "encoder speed output after angle sync and clear");

    std::printf("encoder estimators at 200 rad/s: BASE=%g WIN=%g PLL=%g\n",
                base, win, pll);
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
    test_voltage_vector_limit_and_tracking();
    test_current_feedforward_modes();
    test_identification_fits();
    test_current_pi_design();
    test_encoder_count_angle_mapping();
    test_encoder_speed_estimator_modes();

    if (failures != 0) {
        std::printf("algorithm regression: %d failure(s)\n", failures);
        return 1;
    }

    std::puts("algorithm regression: PASS");
    return 0;
}
