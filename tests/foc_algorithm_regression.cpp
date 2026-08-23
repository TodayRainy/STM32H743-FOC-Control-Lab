#include "foc_math.hpp"
#include "foc_pid.hpp"
#include "foc_pwm.hpp"
#include "foc_ctrl.hpp"

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

    foc_track track;
    float value = 0.0f;
    for (int step = 0; step < 50; ++step) {
        value = track.run(100.0f, 0.0f, 200.0f, 0.01f, TRACK_MODE_RAMP);
    }
    check(std::fabs(value - 100.0f) < 1.0e-5f,
          "positive speed ramp rate");

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

    position.set_track(TRACK_MODE_NONE, 0.0f);
    (void)position.run(-2.0f, 1.0f, 1.0f, TC_ROTATE_SHORTEST_PATH, 0.1f);
    check(std::fabs(position.reference() + 2.0f) < 1.0e-6f,
          "position step mode");
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

} // namespace

int main(void)
{
    test_transforms();
    test_svpwm_duty_range();
    test_pi_integrator_limit();
    test_runtime_gains_and_speed_ramp();
    test_position_ramp();
    test_voltage_vector_limit_and_tracking();

    if (failures != 0) {
        std::printf("algorithm regression: %d failure(s)\n", failures);
        return 1;
    }

    std::puts("algorithm regression: PASS");
    return 0;
}
