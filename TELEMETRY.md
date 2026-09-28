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

The complete frame is always 156 bytes. It has no CRC and contains no validity
flags. A four-byte sync word gives the receiver a reliable way to recover the
frame boundary.

| Order | Type | Field | Value or unit |
|---:|---|---|---|
| 0 | `U32` | `sync` | `0xA55A3CC3`, bytes `C3 3C 5A A5` |
| 1 | `U32` | `schema_version` | `1` |
| 2 | `U32` | `sequence` | Frame count |
| 3 | `U64` | `timestamp_us` | Microseconds |
| 4...37 | `float32` | Flight signals | Listed below |

The first eight bytes are always:

```text
C3 3C 5A A5 01 00 00 00
```

## Flight signals

All 34 signals are consecutive IEEE-754 binary32 values. Their order is the
schema reproduced by the Python decoder.

| Signal index | Cluster field | Unit |
|---:|---|---|
| 0 | `imu_dt_s` | s |
| 1 | `accel_x` | m/s² |
| 2 | `accel_y` | m/s² |
| 3 | `accel_z` | m/s² |
| 4 | `roll_rate_measured` | deg/s |
| 5 | `pitch_rate_measured` | deg/s |
| 6 | `yaw_rate_measured` | deg/s |
| 7 | `roll_angle` | deg |
| 8 | `pitch_angle` | deg |
| 9 | `throttle_setpoint` | 0...1 |
| 10 | `roll_angle_setpoint` | deg |
| 11 | `pitch_angle_setpoint` | deg |
| 12 | `roll_rate_setpoint` | deg/s |
| 13 | `pitch_rate_setpoint` | deg/s |
| 14 | `yaw_rate_setpoint` | deg/s |
| 15 | `roll_error` | deg/s |
| 16 | `roll_proportional` | normalized |
| 17 | `roll_integral` | normalized |
| 18 | `roll_derivative` | normalized |
| 19 | `roll_output` | normalized |
| 20 | `pitch_error` | deg/s |
| 21 | `pitch_proportional` | normalized |
| 22 | `pitch_integral` | normalized |
| 23 | `pitch_derivative` | normalized |
| 24 | `pitch_output` | normalized |
| 25 | `yaw_error` | deg/s |
| 26 | `yaw_proportional` | normalized |
| 27 | `yaw_integral` | normalized |
| 28 | `yaw_derivative` | normalized |
| 29 | `yaw_output` | normalized |
| 30 | `motor_front_left` | us |
| 31 | `motor_front_right` | us |
| 32 | `motor_rear_right` | us |
| 33 | `motor_rear_left` | us |

## Python/Spyder receiver

`tools/telemetry_dashboard.py` provides the complete PC-side implementation.
The fixed packet is decoded by one standard-library structure:

```python
FRAME_STRUCT = struct.Struct("<IIIQ34f")
```

The `<` selects little-endian representation, `III` describes the three U32
header fields, `Q` the U64 timestamp, and `34f` the 34 float32 signals. The
incremental decoder performs the following operations:

1. Append every serial read to a persistent byte buffer.
2. Find `C3 3C 5A A5` and discard earlier bytes.
3. Wait until at least 156 bytes are available.
4. Unpack the complete frame and require `schema_version == 1`.
5. Reject non-finite float values and resume the sync search after corruption.
6. Publish the decoded frame to the Tk GUI through a bounded thread-safe queue.

The dashboard shows the header and every signal, provides live plots and
indicators, calculates receive diagnostics, and records a conventional CSV.
`sequence` reveals missing frames that were constructed by the MCU, while
`timestamp_us` is used as the graph time axis and also reveals firmware-side
snapshot gaps. See `PYTHON_TELEMETRY.md` for installation and operation.
