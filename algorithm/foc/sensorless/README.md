# Sensorless module boundary

This directory owns rotor sources and transitions that do not require a rotor
sensor:

- `foc_vf.*`: forced-angle voltage/frequency source.
- `foc_smo.*`: sliding-mode observer retained for comparison.
- `foc_flux.*`: Active Flux medium/high-speed observer.
- `foc_sensorless_startup.hpp`: current-controlled I/f startup and handoff.
- `foc_hfi_observer.hpp`: HFI carrier, demodulation, IPD, angle tracking, and
  speed PLL.
- `foc_hfi.hpp/.cpp`: motor-facing HFI facade and speed telemetry adapter.
- `foc_hfi_speed_startup.hpp`: HFI-angle low-speed current assist.
- `foc_hfi_flux_handoff.hpp`: HFI/Active Flux qualification and blending.

The motor supervisor remains in `algorithm/foc/foc_motor.*`. It owns power and
control authority, applies requests at the 40 kHz boundary, selects the active
rotor source, and changes speed-controller gains/current limits. Application
motor constants remain in `app/foc_app.cpp`.

Angles are electrical radians and speeds are electrical rad/s unless a class
comment states otherwise.
