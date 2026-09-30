# IMU-only horizontal drift brake

`horizontal_drift.c` implements a deliberately bounded, short-horizon
horizontal motion estimate. Its purpose is to oppose an uncommanded transient
after the pilot centers roll, pitch, and yaw. It is not an absolute position
estimator and it does not replace optical flow, GNSS, UWB, or another external
velocity/position reference.

## Enable and runtime conditions

Set `HORIZONTAL_DRIFT_COMPENSATION_ENABLED` in
`include/horizontal_drift.h` to `0U` to compile the feature disabled. It can
also be disabled during a CCS session with:

```c
flight_control_data.horizontal_drift.config.enabled = false;
```

The brake becomes active only when all of these conditions are true:

- Throttle is at least `0.35`.
- Roll, pitch, and yaw commands are inside the existing stick deadband.
- Magnetic heading hold is active, keeping the local axes approximately fixed.
- The flight controller trusts the acceleration magnitude.

Any failed condition resets velocity, displacement, filters, and correction.
Manual commands therefore retain direct authority.

## Estimation and correction

The startup level calibration provides the stationary acceleration baseline.
Current roll and pitch remove the predicted horizontal gravity projection.
The residual is filtered at 2 Hz, passed through a 0.04 m/s² deadband, and
integrated with trapezoidal integration. Velocity and displacement use leak
time constants of 6 s and 4 s and are bounded to ±1.5 m/s and ±0.75 m.

The local axis convention is:

- X positive: motion toward the nose.
- Y positive: motion toward the left, as validated by the recorded table test.

The angular corrections are:

```text
pitch_trim = 1.5 * displacement_x + 2.5 * velocity_x
roll_trim  = 1.5 * displacement_y + 2.5 * velocity_y
```

Each correction is limited to ±2 degrees. The flight controller adds these
terms to the normal stick angle commands before the existing angle and rate
loops. Forward motion therefore requests a positive pitch correction, which
commands the validated nose-up braking response.

Motion is latched above 0.12 m/s² acceleration or 0.05 m/s speed. Once both
acceleration and speed remain below 0.08 m/s² and 0.04 m/s for 0.35 seconds,
the local origin is cleared and `reset_count` increments.

## CCS observations

Use these expressions during propeller-free validation:

```text
flight_control_data.horizontal_drift.last_status
flight_control_data.horizontal_drift.output.active
flight_control_data.horizontal_drift.output.motion_detected
flight_control_data.horizontal_drift.output.acceleration_mps2
flight_control_data.horizontal_drift.output.velocity_mps
flight_control_data.horizontal_drift.output.displacement_m
flight_control_data.horizontal_drift.output.roll_correction_deg
flight_control_data.horizontal_drift.output.pitch_correction_deg
flight_control_data.horizontal_drift.output.reset_count
flight_control_data.output.setpoint.roll_angle_deg
flight_control_data.output.setpoint.pitch_angle_deg
```

With the motors unpowered, provide valid receiver data and raise throttle above
0.35. A forward slide should produce positive X velocity/displacement and a
positive pitch correction. A right slide should produce negative Y and a
negative roll correction. Moving a stick should immediately clear the
correction. After the estimated transient settles, `reset_count` should
increase and the local estimates should return to zero.

## Observability limit

An accelerometer measures force, not velocity or position. After an impulse,
constant horizontal velocity can have nearly zero measured acceleration. Bias,
vibration, attitude error, and aerodynamic acceleration also integrate into
false velocity. The leak, bounds, gating, and automatic reset keep the feature
useful as a small transient brake, but they cannot guarantee a fixed point in
space. Reliable position hold requires an external horizontal reference.
