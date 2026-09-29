# USB flight telemetry protocol

UART0 uses the LaunchPad debug USB virtual COM connection on PA0/PA1. It
transmits a fixed binary frame at 100 Hz containing only the signals needed to
evaluate the cascaded PID controller.

## Serial configuration

- Baud rate: `460800`
- Data bits: `8`
- Parity: none
- Stop bits: `1`
- Flow control: none
- Byte order: little-endian
- Floating-point format: IEEE-754 binary32 (`float` in Python)

Transmission is incremental and non-blocking. If the UART cannot finish a
frame before the next deadline, the new frame is discarded and
`telemetry_data.dropped_frame_count` increases. Flight control execution is
never delayed waiting for USB serial transmission.

## Fixed frame

The complete frame is always 168 bytes. It has no CRC and contains no validity
flags. A four-byte sync word gives the receiver a reliable way to recover the
frame boundary.

| Order | Type | Field | Value or unit |
|---:|---|---|---|
| 0 | `U32` | `sync` | `0xA55A3CC3`, bytes `C3 3C 5A A5` |
| 1 | `U32` | `schema_version` | `2` |
| 2 | `U32` | `sequence` | Frame count |
| 3 | `U64` | `timestamp_us` | Microseconds |
| 4...40 | `float32` | Flight signals | Listed below |

The first eight bytes are always:

```text
C3 3C 5A A5 02 00 00 00
```

## Flight signals

All 37 signals are consecutive IEEE-754 binary32 values. Their order is the
schema reproduced by the Python decoder.

| Signal index | Cluster field | Unit |
|---:|---|---|
| 0 | `imu_dt_s` | s |
| 1 | `accel_x` | m/s² |
| 2 | `accel_y` | m/s² |
| 3 | `accel_z` | m/s² |
| 4 | `imu_gyro_x` | rad/s |
| 5 | `imu_gyro_y` | rad/s |
| 6 | `imu_gyro_z` | rad/s |
| 7 | `roll_rate_measured` | deg/s |
| 8 | `pitch_rate_measured` | deg/s |
| 9 | `yaw_rate_measured` | deg/s |
| 10 | `roll_angle` | deg |
| 11 | `pitch_angle` | deg |
| 12 | `throttle_setpoint` | 0...1 |
| 13 | `roll_angle_setpoint` | deg |
| 14 | `pitch_angle_setpoint` | deg |
| 15 | `roll_rate_setpoint` | deg/s |
| 16 | `pitch_rate_setpoint` | deg/s |
| 17 | `yaw_rate_setpoint` | deg/s |
| 18 | `roll_error` | deg/s |
| 19 | `roll_proportional` | normalized |
| 20 | `roll_integral` | normalized |
| 21 | `roll_derivative` | normalized |
| 22 | `roll_output` | normalized |
| 23 | `pitch_error` | deg/s |
| 24 | `pitch_proportional` | normalized |
| 25 | `pitch_integral` | normalized |
| 26 | `pitch_derivative` | normalized |
| 27 | `pitch_output` | normalized |
| 28 | `yaw_error` | deg/s |
| 29 | `yaw_proportional` | normalized |
| 30 | `yaw_integral` | normalized |
| 31 | `yaw_derivative` | normalized |
| 32 | `yaw_output` | normalized |
| 33 | `motor_front_left` | us |
| 34 | `motor_front_right` | us |
| 35 | `motor_rear_right` | us |
| 36 | `motor_rear_left` | us |

The six IMU fields at indices 1...6 are the latest unfiltered physical sample.
Acceleration is axis-mapped and scaled to m/s². Angular rate is axis-mapped,
scaled to rad/s, and has the startup gyroscope bias removed; no controller
low-pass has been applied. These fields are suitable for offline inertial
analysis while avoiding sensor-scale conversion in Python. The three
`*_rate_measured` fields remain separate: they are the 30 Hz low-pass filtered
rates actually consumed by the PID. The attitude estimator consumes
`flight_control_data.filtered_accel_mps2`, which has a 5 Hz low-pass.
`roll_angle` and `pitch_angle` have the stationary startup level reference
removed. Below throttle `0.35`, all three integral terms are deliberately held
at zero to prevent ground windup; this state is observable through
`flight_control_data.output.integrator_enabled` in CCS but is not added to the
compact serial frame.

The sensor runs at 416 Hz but telemetry snapshots the latest complete sample
at 100 Hz. Offline integration must therefore use consecutive frame
`timestamp_us` differences, not `imu_dt_s`, as the interval between recorded
rows. Acceleration includes gravity and must be rotated into a navigation
frame, gravity-compensated, bias-corrected, and filtered before integration.

The four motor fields are the final ESC pulses. While active, MotorOutput
moves all four pulses upward together if one request would fall below
1180 us. This preserves the requested differential torque. Only when that
collective shift would exceed 2000 us is the four-motor spread scaled. The
diagnostics are `motor_output_data.collective_shift_us` and
`motor_output_data.active_range_scale`.

## Python/Spyder receiver

`tools/telemetry_dashboard.py` provides the complete PC-side implementation.
The fixed packet is decoded by one standard-library structure:

```python
FRAME_STRUCT = struct.Struct("<IIIQ37f")
```

The `<` selects little-endian representation, `III` describes the three U32
header fields, `Q` the U64 timestamp, and `37f` the 37 float32 signals. The
incremental decoder performs the following operations:

1. Append every serial read to a persistent byte buffer.
2. Find `C3 3C 5A A5` and discard earlier bytes.
3. Wait until at least 168 bytes are available.
4. Unpack the complete frame and require `schema_version == 2`.
5. Reject non-finite float values and resume the sync search after corruption.
6. Publish the decoded frame to the Tk GUI through a bounded thread-safe queue.

The dashboard shows the header and every signal, provides live plots and
indicators, calculates receive diagnostics, and records a conventional CSV.
`sequence` reveals missing frames that were constructed by the MCU, while
`timestamp_us` is used as the graph time axis and also reveals firmware-side
snapshot gaps. See `PYTHON_TELEMETRY.md` for installation and operation.
