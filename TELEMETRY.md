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

## Schema 4 fixed frame

The complete frame is always 180 bytes. It has no CRC and contains no validity
flags. A four-byte sync word allows the receiver to recover the frame boundary.

| Offset | Type | Field | Value or unit |
|---:|---|---|---|
| 0...3 | `U32` | `sync` | `0xA55A3CC3`, bytes `C3 3C 5A A5` |
| 4...7 | `U32` | `schema_version` | `4` |
| 8...11 | `U32` | `sequence` | Frame count |
| 12...19 | `U64` | `timestamp_us` | Microseconds |
| 20...179 | `40 × float32` | Flight signals | Listed below |

The first eight bytes are:

```text
C3 3C 5A A5 04 00 00 00
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
| 10 | `roll_rate_measured` | deg/s |
| 11 | `pitch_rate_measured` | deg/s |
| 12 | `yaw_rate_measured` | deg/s |
| 13 | `roll_angle` | deg |
| 14 | `pitch_angle` | deg |
| 15 | `throttle_setpoint` | 0...1 |
| 16 | `roll_angle_setpoint` | deg |
| 17 | `pitch_angle_setpoint` | deg |
| 18 | `roll_rate_setpoint` | deg/s |
| 19 | `pitch_rate_setpoint` | deg/s |
| 20 | `yaw_rate_setpoint` | deg/s |
| 21 | `roll_error` | deg/s |
| 22 | `roll_proportional` | normalized |
| 23 | `roll_integral` | normalized |
| 24 | `roll_derivative` | normalized |
| 25 | `roll_output` | normalized |
| 26 | `pitch_error` | deg/s |
| 27 | `pitch_proportional` | normalized |
| 28 | `pitch_integral` | normalized |
| 29 | `pitch_derivative` | normalized |
| 30 | `pitch_output` | normalized |
| 31 | `yaw_error` | deg/s |
| 32 | `yaw_proportional` | normalized |
| 33 | `yaw_integral` | normalized |
| 34 | `yaw_derivative` | normalized |
| 35 | `yaw_output` | normalized |
| 36 | `motor_front_left` | µs |
| 37 | `motor_front_right` | µs |
| 38 | `motor_rear_right` | µs |
| 39 | `motor_rear_left` | µs |

The accelerometer and gyroscope fields are the latest physical IMU sample
without the controller low-pass. The gyro already has its startup bias removed.
The controller rate fields remain separate because they are filtered at 30 Hz.

The three magnetometer fields are hard/soft-iron corrected and expressed in
the aircraft FRD frame: X forward, Y right, Z down. The installation-specific
calibration removes the sensor-frame bias, applies the fitted 3x3 matrix, and
includes the validated orientation `[-X, -Y, +Z]`. The sensor produces 50
samples/s, so each value normally appears in two consecutive 100 Hz telemetry
frames. These fields remain observational and do not affect yaw control.

The sensor snapshots use the latest available data. Offline analysis must use
consecutive `timestamp_us` differences as the recorded-row interval. The four
motor fields are the final ESC pulses after MotorOutput headroom management.

## Python/Spyder receiver

`tools/telemetry_dashboard.py` decodes the complete frame with:

```python
FRAME_STRUCT = struct.Struct("<IIIQ40f")
```

`<` selects little-endian representation, `III` represents the three U32
header fields, `Q` the U64 timestamp, and `40f` the 40 float32 signals. The
decoder:

1. Appends serial reads to a persistent byte buffer.
2. Finds `C3 3C 5A A5` and discards earlier bytes.
3. Waits for 180 bytes.
4. Requires `schema_version == 4`.
5. Rejects non-finite payload values and resumes sync search.
6. Delivers decoded frames to the Tk GUI through a bounded queue.

The dashboard provides a flight overview, nine sensor axes, controller plots,
all current values, serial diagnostics, and CSV recording. CSV columns follow
the exact order above.
