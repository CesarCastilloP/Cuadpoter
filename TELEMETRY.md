# USB flight telemetry protocol

UART0 uses the LaunchPad debug USB virtual COM connection on PA0/PA1. It
transmits a fixed binary frame at 100 Hz containing the sensor and controller
signals needed for flight analysis.

## Serial configuration

- Baud rate: `460800`
- Data bits: `8`
- Parity: none
- Stop bits: `1`
- Flow control: none
- Byte order: little-endian
- Floating-point format: IEEE-754 binary32

Transmission is incremental and non-blocking. If the UART cannot finish a
frame before the next deadline, the new frame is discarded and
`telemetry_data.dropped_frame_count` increases. Flight control never waits for
USB transmission.

## Schema 7 fixed frame

The complete frame is always 232 bytes. It has no CRC and contains no validity
flags. A four-byte sync word allows the receiver to recover the frame boundary.

| Offset | Type | Field | Value or unit |
|---:|---|---|---|
| 0...3 | `U32` | `sync` | `0xA55A3CC3`, bytes `C3 3C 5A A5` |
| 4...7 | `U32` | `schema_version` | `7` |
| 8...11 | `U32` | `sequence` | Frame count |
| 12...19 | `U64` | `timestamp_us` | Microseconds |
| 20...231 | `53 × float32` | Flight signals | Listed below |

The first eight bytes are:

```text
C3 3C 5A A5 07 00 00 00
```

## Flight signals

| Index | Field | Unit |
|---:|---|---|
| 0 | `imu_dt_s` | s |
| 1 | `accel_x` | m/s² |
| 2 | `accel_y` | m/s² |
| 3 | `accel_z` | m/s² |
| 4 | `imu_gyro_x` | rad/s |
| 5 | `imu_gyro_y` | rad/s |
| 6 | `imu_gyro_z` | rad/s |
| 7 | `mag_x` | µT |
| 8 | `mag_y` | µT |
| 9 | `mag_z` | µT |
| 10 | `heading` | deg, wrapped to -180...180 |
| 11 | `heading_setpoint` | deg, wrapped to -180...180 |
| 12 | `heading_error` | deg, shortest signed error |
| 13 | `horizontal_accel_x` | m/s², body/local X forward |
| 14 | `horizontal_accel_y` | m/s², local Y positive left |
| 15 | `horizontal_velocity_x` | m/s, bounded short-horizon estimate |
| 16 | `horizontal_velocity_y` | m/s, bounded short-horizon estimate |
| 17 | `horizontal_displacement_x` | m, leaky local estimate |
| 18 | `horizontal_displacement_y` | m, leaky local estimate |
| 19 | `drift_roll_correction` | deg |
| 20 | `drift_pitch_correction` | deg |
| 21 | `roll_rate_measured` | deg/s |
| 22 | `pitch_rate_measured` | deg/s |
| 23 | `yaw_rate_measured` | deg/s |
| 24 | `roll_angle` | deg |
| 25 | `pitch_angle` | deg |
| 26 | `throttle_setpoint` | 0...1 |
| 27 | `flight_roll_trim_deg` | deg, configured flight trim |
| 28 | `flight_pitch_trim_deg` | deg, configured flight trim |
| 29 | `roll_angle_setpoint` | deg |
| 30 | `pitch_angle_setpoint` | deg |
| 31 | `roll_rate_setpoint` | deg/s |
| 32 | `pitch_rate_setpoint` | deg/s |
| 33 | `yaw_rate_setpoint` | deg/s |
| 34 | `roll_error` | deg/s |
| 35 | `roll_proportional` | normalized |
| 36 | `roll_integral` | normalized |
| 37 | `roll_derivative` | normalized |
| 38 | `roll_output` | normalized |
| 39 | `pitch_error` | deg/s |
| 40 | `pitch_proportional` | normalized |
| 41 | `pitch_integral` | normalized |
| 42 | `pitch_derivative` | normalized |
| 43 | `pitch_output` | normalized |
| 44 | `yaw_error` | deg/s |
| 45 | `yaw_proportional` | normalized |
| 46 | `yaw_integral` | normalized |
| 47 | `yaw_derivative` | normalized |
| 48 | `yaw_output` | normalized |
| 49 | `motor_front_left` | µs |
| 50 | `motor_front_right` | µs |
| 51 | `motor_rear_right` | µs |
| 52 | `motor_rear_left` | µs |

The accelerometer and gyroscope fields are the latest physical IMU sample
without the controller low-pass. The gyro already has its startup bias removed.
The controller rate fields remain separate because they are filtered at 30 Hz.

The two flight-trim fields are the configured roll and pitch setpoint offsets.
They do not alter the reported attitude or IMU calibration. The transmitted
`roll_angle_setpoint` and `pitch_angle_setpoint` already contain stick, flight
trim, and horizontal-drift correction contributions.

The three magnetometer fields are hard/soft-iron corrected and expressed in
the aircraft FRD frame: X forward, Y right, Z down. The installation-specific
calibration removes the sensor-frame bias, applies the fitted 3x3 matrix, and
includes the validated orientation `[-X, -Y, +Z]`. The sensor produces 50
samples/s, so each value normally appears in two consecutive 100 Hz telemetry
frames. The heading estimator tilt-compensates these values with the current
roll and pitch, rejects field magnitudes outside the calibrated range, and
applies a circular 3 Hz low-pass filter.

When the yaw stick is centered, `heading_setpoint` is the captured magnetic
heading and `heading_error` drives the outer heading P loop. That loop produces
`yaw_rate_setpoint`; the existing gyroscope yaw-rate PID remains the inner,
fast loop. Moving the yaw stick restores direct yaw-rate command and captures a
new heading when the stick returns to center. Invalid or stale magnetic data
automatically disables heading hold while retaining yaw-rate control.

The eight horizontal-drift fields expose the bounded IMU-only brake used when
all sticks are centered. They are local, short-horizon estimates rather than
absolute navigation coordinates. Positive X is forward. Positive Y is the
left-motion convention validated with the recorded table-slide experiment.
The correction is reset when the estimator considers the transient settled,
when the pilot moves a stick, when throttle is low, or when acceleration or
heading cannot be trusted. An IMU cannot observe an already-established
constant horizontal velocity, so these signals must not be interpreted as a
position hold measurement.

The sensor snapshots use the latest available data. Offline analysis must use
consecutive `timestamp_us` differences as the recorded-row interval. The four
motor fields are the final ESC pulses after MotorOutput headroom management.

## Python/Spyder receiver

`tools/telemetry_dashboard.py` decodes the complete frame with:

```python
FRAME_STRUCT = struct.Struct("<IIIQ53f")
```

`<` selects little-endian representation, `III` represents the three U32
header fields, `Q` the U64 timestamp, and `53f` the 53 float32 signals. The
decoder:

1. Appends serial reads to a persistent byte buffer.
2. Finds `C3 3C 5A A5` and discards earlier bytes.
3. Waits for 232 bytes.
4. Requires `schema_version == 7`.
5. Rejects non-finite payload values and resumes sync search.
6. Delivers decoded frames to the Tk GUI through a bounded queue.

The dashboard provides a flight overview, nine sensor axes, controller plots,
all current values, serial diagnostics, and CSV recording. CSV columns follow
the exact order above.
