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

## Schema 5 fixed frame

The complete frame is always 192 bytes. It has no CRC and contains no validity
flags. A four-byte sync word allows the receiver to recover the frame boundary.

| Offset | Type | Field | Value or unit |
|---:|---|---|---|
| 0...3 | `U32` | `sync` | `0xA55A3CC3`, bytes `C3 3C 5A A5` |
| 4...7 | `U32` | `schema_version` | `5` |
| 8...11 | `U32` | `sequence` | Frame count |
| 12...19 | `U64` | `timestamp_us` | Microseconds |
| 20...191 | `43 × float32` | Flight signals | Listed below |

The first eight bytes are:

```text
C3 3C 5A A5 05 00 00 00
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
| 13 | `roll_rate_measured` | deg/s |
| 14 | `pitch_rate_measured` | deg/s |
| 15 | `yaw_rate_measured` | deg/s |
| 16 | `roll_angle` | deg |
| 17 | `pitch_angle` | deg |
| 18 | `throttle_setpoint` | 0...1 |
| 19 | `roll_angle_setpoint` | deg |
| 20 | `pitch_angle_setpoint` | deg |
| 21 | `roll_rate_setpoint` | deg/s |
| 22 | `pitch_rate_setpoint` | deg/s |
| 23 | `yaw_rate_setpoint` | deg/s |
| 24 | `roll_error` | deg/s |
| 25 | `roll_proportional` | normalized |
| 26 | `roll_integral` | normalized |
| 27 | `roll_derivative` | normalized |
| 28 | `roll_output` | normalized |
| 29 | `pitch_error` | deg/s |
| 30 | `pitch_proportional` | normalized |
| 31 | `pitch_integral` | normalized |
| 32 | `pitch_derivative` | normalized |
| 33 | `pitch_output` | normalized |
| 34 | `yaw_error` | deg/s |
| 35 | `yaw_proportional` | normalized |
| 36 | `yaw_integral` | normalized |
| 37 | `yaw_derivative` | normalized |
| 38 | `yaw_output` | normalized |
| 39 | `motor_front_left` | µs |
| 40 | `motor_front_right` | µs |
| 41 | `motor_rear_right` | µs |
| 42 | `motor_rear_left` | µs |

The accelerometer and gyroscope fields are the latest physical IMU sample
without the controller low-pass. The gyro already has its startup bias removed.
The controller rate fields remain separate because they are filtered at 30 Hz.

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

The sensor snapshots use the latest available data. Offline analysis must use
consecutive `timestamp_us` differences as the recorded-row interval. The four
motor fields are the final ESC pulses after MotorOutput headroom management.

## Python/Spyder receiver

`tools/telemetry_dashboard.py` decodes the complete frame with:

```python
FRAME_STRUCT = struct.Struct("<IIIQ43f")
```

`<` selects little-endian representation, `III` represents the three U32
header fields, `Q` the U64 timestamp, and `43f` the 43 float32 signals. The
decoder:

1. Appends serial reads to a persistent byte buffer.
2. Finds `C3 3C 5A A5` and discards earlier bytes.
3. Waits for 192 bytes.
4. Requires `schema_version == 5`.
5. Rejects non-finite payload values and resumes sync search.
6. Delivers decoded frames to the Tk GUI through a bounded queue.

The dashboard provides a flight overview, nine sensor axes, controller plots,
all current values, serial diagnostics, and CSV recording. CSV columns follow
the exact order above.
