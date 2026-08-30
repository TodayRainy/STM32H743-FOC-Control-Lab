#ifndef FOC_FAULT_HPP
#define FOC_FAULT_HPP

#include <stdint.h>

/* Fault bits are part of the host protocol. Append new bits; do not renumber
 * existing entries. Trip and warning policy stays explicit in the motor. */
typedef enum
{
    FOC_FAULT_NONE                 = 0U,
    FOC_FAULT_CURRENT_OFFSET       = 1U << 0,
    FOC_FAULT_SOFTWARE_OVERCURRENT = 1U << 1,
    FOC_FAULT_BUS_UNDERVOLTAGE     = 1U << 2,
    FOC_FAULT_BUS_OVERVOLTAGE      = 1U << 3,
    FOC_FAULT_OVERSPEED            = 1U << 4,
    FOC_FAULT_SENSORLESS_STARTUP   = 1U << 5,
    FOC_FAULT_ROTOR_SOURCE_LOST    = 1U << 6,
    FOC_FAULT_CONTROL_DEADLINE     = 1U << 7,
    FOC_FAULT_SPEED_TRACKING       = 1U << 8,
    FOC_FAULT_POSITION_TRACKING    = 1U << 9,
} FOC_FAULT_t;

static const uint32_t FOC_FAULT_TRIP_MASK =
    FOC_FAULT_CURRENT_OFFSET
    | FOC_FAULT_SOFTWARE_OVERCURRENT
    | FOC_FAULT_BUS_UNDERVOLTAGE
    | FOC_FAULT_BUS_OVERVOLTAGE
    | FOC_FAULT_OVERSPEED
    | FOC_FAULT_SENSORLESS_STARTUP
    | FOC_FAULT_ROTOR_SOURCE_LOST
    | FOC_FAULT_CONTROL_DEADLINE;

static const uint32_t FOC_FAULT_WARNING_MASK =
    FOC_FAULT_SPEED_TRACKING | FOC_FAULT_POSITION_TRACKING;

typedef struct
{
    uint32_t timestamp_ms;
    uint32_t lifecycle;
    uint32_t operation;
    uint32_t rotor_source;
    float iq;
    float id;
    float speed;
    float target_speed;
    float vbus;
    float voltage_scale;
} foc_fault_snapshot_t;

typedef struct
{
    uint32_t active_bits;
    uint32_t latched_bits;
    uint32_t first_fault;
    foc_fault_snapshot_t first_snapshot;
} foc_fault_status_t;

/* Stores one immutable first-trip snapshot and two bitmaps. The 40 kHz motor
 * path owns writes; the foreground reads a copy for STATUS/FAULT responses. */
class foc_fault_manager
{
public:
    foc_fault_manager()
    {
        reset();
    }

    void reset()
    {
        status.active_bits = 0U;
        status.latched_bits = 0U;
        status.first_fault = FOC_FAULT_NONE;
        foc_fault_snapshot_t empty = {};
        status.first_snapshot = empty;
    }

    bool set_condition(uint32_t fault, bool active, bool trip,
                       const foc_fault_snapshot_t &snapshot)
    {
        if(!active){
            status.active_bits &= ~fault;
            return false;
        }

        const bool rising = (status.active_bits & fault) == 0U;
        status.active_bits |= fault;
        status.latched_bits |= fault;
        if(trip && (status.first_fault == FOC_FAULT_NONE)){
            status.first_snapshot = snapshot;
            status.first_fault = fault;
        }
        return rising && trip;
    }

    bool active_trip(void) const
    {
        return (status.active_bits & FOC_FAULT_TRIP_MASK) != 0U;
    }

    bool acknowledge()
    {
        if(active_trip()){
            return false;
        }
        status.latched_bits = status.active_bits;
        status.first_fault = FOC_FAULT_NONE;
        foc_fault_snapshot_t empty = {};
        status.first_snapshot = empty;
        return true;
    }

    foc_fault_status_t get() const
    {
        return status;
    }

private:
    foc_fault_status_t status;
};

/* Time qualification and hysteresis for one measured condition. */
class foc_qualified_fault
{
public:
    foc_qualified_fault() : active(false), timer(0.0f) {}

    bool update(bool set_condition, bool clear_condition,
                float set_time, float clear_time, float sample_period)
    {
        if(active){
            if(clear_condition){
                timer += sample_period;
                if(timer >= clear_time){
                    active = false;
                    timer = 0.0f;
                }
            }else{
                timer = 0.0f;
            }
        }else{
            if(set_condition){
                timer += sample_period;
                if(timer >= set_time){
                    active = true;
                    timer = 0.0f;
                }
            }else{
                timer = 0.0f;
            }
        }
        return active;
    }

private:
    bool active;
    float timer;
};

typedef struct
{
    float bus_undervoltage;
    float bus_undervoltage_clear;
    float bus_overvoltage;
    float bus_overvoltage_clear;
    float bus_set_time;
    float bus_clear_time;
    float overspeed;
    float overspeed_clear;
    float overspeed_set_time;
    float overspeed_clear_time;
    float deadline;
    float deadline_clear;
    float deadline_set_time;
    float deadline_clear_time;
    float speed_tracking_error;
    float speed_tracking_clear;
    float speed_reference_settled;
    float speed_tracking_set_time;
    float speed_tracking_clear_time;
    float position_tracking_error;
    float position_tracking_clear;
    float position_reference_settled;
    float position_tracking_set_time;
    float position_tracking_clear_time;
} foc_fault_monitor_config_t;

typedef struct
{
    float vbus;
    float speed;
    float execution_time;
    float speed_error;
    float position_error;
    bool power_control_active;
    bool speed_tracking_enabled;
    bool position_tracking_enabled;
} foc_fault_monitor_input_t;

/* Pure signal qualification. It does not decide lifecycle or switch PWM. */
class foc_fault_monitor
{
public:
    foc_fault_monitor() : configured(false) {}

    void configure(const foc_fault_monitor_config_t &new_config)
    {
        config = new_config;
        configured = true;
    }

    uint32_t update(const foc_fault_monitor_input_t &input,
                    float sample_period)
    {
        if(!configured){
            return 0U;
        }

        uint32_t active = 0U;
        const bool power = input.power_control_active;
        if(undervoltage.update(
               power && (input.vbus <= config.bus_undervoltage),
               input.vbus >= config.bus_undervoltage_clear,
               config.bus_set_time, config.bus_clear_time, sample_period)){
            active |= FOC_FAULT_BUS_UNDERVOLTAGE;
        }
        if(overvoltage.update(
               power && (input.vbus >= config.bus_overvoltage),
               input.vbus <= config.bus_overvoltage_clear,
               config.bus_set_time, config.bus_clear_time, sample_period)){
            active |= FOC_FAULT_BUS_OVERVOLTAGE;
        }
        const float absolute_speed = input.speed < 0.0f
                                   ? -input.speed : input.speed;
        if(overspeed.update(
               power && (absolute_speed >= config.overspeed),
               absolute_speed <= config.overspeed_clear,
               config.overspeed_set_time, config.overspeed_clear_time,
               sample_period)){
            active |= FOC_FAULT_OVERSPEED;
        }
        if(deadline.update(
               power && (input.execution_time >= config.deadline),
               !power || (input.execution_time <= config.deadline_clear),
               config.deadline_set_time, config.deadline_clear_time,
               sample_period)){
            active |= FOC_FAULT_CONTROL_DEADLINE;
        }

        const float speed_error = input.speed_error < 0.0f
                                ? -input.speed_error : input.speed_error;
        if(speed_tracking.update(
               input.speed_tracking_enabled
                   && (speed_error >= config.speed_tracking_error),
               !input.speed_tracking_enabled
                   || (speed_error <= config.speed_tracking_clear),
               config.speed_tracking_set_time,
               config.speed_tracking_clear_time, sample_period)){
            active |= FOC_FAULT_SPEED_TRACKING;
        }
        const float position_error = input.position_error < 0.0f
                                   ? -input.position_error
                                   : input.position_error;
        if(position_tracking.update(
               input.position_tracking_enabled
                   && (position_error >= config.position_tracking_error),
               !input.position_tracking_enabled
                   || (position_error <= config.position_tracking_clear),
               config.position_tracking_set_time,
               config.position_tracking_clear_time, sample_period)){
            active |= FOC_FAULT_POSITION_TRACKING;
        }
        return active;
    }

private:
    bool configured;
    foc_fault_monitor_config_t config;
    foc_qualified_fault undervoltage;
    foc_qualified_fault overvoltage;
    foc_qualified_fault overspeed;
    foc_qualified_fault deadline;
    foc_qualified_fault speed_tracking;
    foc_qualified_fault position_tracking;
};

#endif
