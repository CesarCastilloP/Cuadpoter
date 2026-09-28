# LSM6DSR IMU

The IMU driver has one responsibility: each instance detects, configures,
calibrates, and publishes coherent sensor samples for the flight code.

## Public interface

```c
LSM6DS_Data_t imu;

status = LSM6DS_Init(&imu, &axis_map);
status = LSM6DS_Update(&imu);
```

`LSM6DS_Init()` must run once after I2C and the time base are initialized.
Keep the aircraft still while it collects 512 calibration samples.
That stationary window produces both the gyroscope bias and the mean gravity
vector used by `FlightControl_Init()` as the level attitude reference. Power
the aircraft on while its frame is level; the controller rejects a reference
tilted more than 15 degrees.

Call `LSM6DS_Update()` from the main scheduler. A return value of
`LSM6DS_STATUS_NO_NEW_DATA` is normal because the code polls at 1 kHz while
the sensor produces data at 416 Hz.

Use a sample only when all of these conditions are true:

```c
status == LSM6DS_STATUS_OK
imu.sample.fresh
imu.sample.valid
```

## Flight variables

| Field | Unit | Meaning |
| --- | --- | --- |
| `sample.gyro_rad_s.x/y/z` | rad/s | Calibrated body rates P, Q, and R |
| `sample.accel_mps2.x/y/z` | m/s^2 | Body specific force |
| `sample.dt_s` | s | Time since the previous sensor sample |
| `sample.timestamp_us` | us | Monotonic acquisition timestamp |
| `sample.temperature_c` | deg C | IMU temperature |
| `sample.sequence` | count | Increments once per fresh sample |
| `calibration.level_accel_mps2` | m/s^2 | Stationary gravity vector used to zero roll and pitch |

Raw values remain available in `sample.raw_gyro` and `sample.raw_accel` for
bench diagnostics.

Flight control applies a first-order 5 Hz low-pass to `sample.accel_mps2`
before calculating gravity magnitude or accelerometer attitude. This filter
prevents motor vibration from repeatedly removing the absolute roll/pitch
reference. The unfiltered SI acceleration remains available here and in USB
telemetry so the mechanical vibration can still be measured. In CCS,
`flight_control_data.filtered_accel_mps2` is the value consumed by the
estimator and `flight_control_data.accel_trusted` indicates whether its norm
is inside 0.90 g...1.10 g.

## Axis mapping

The driver maps sensor axes into a right-handed aircraft frame before it
publishes data or computes gyro bias. The map is defined once in `main.c`:

```c
static const LSM6DS_AxisMap_t g_imu_axis_map = {
    { LSM6DS_AXIS_X, LSM6DS_AXIS_Y, LSM6DS_AXIS_Z },
    { 1, 1, 1 }
};
```

The current identity map preserves the sensor coordinate system. Before
flight, change the three source axes and signs to match the physical mounting.
The driver rejects repeated axes, invalid signs, and left-handed mappings.

For an FRD aircraft frame, X points forward, Y right, and Z down. A stationary
level accelerometer then measures approximately `[0, 0, -9.81] m/s^2`.

## Configuration

The LSM6DSR is detected at either `0x6A` or `0x6B` and must return `0x6B` from
`WHO_AM_I`. The selected configuration is:

- 416 Hz accelerometer and gyroscope output rate
- accelerometer range: +/-8 g
- gyroscope range: +/-2000 deg/s
- block data update and register auto-increment enabled
- I3C disabled for reliable I2C operation

The conversion factors come from the
[LSM6DSR datasheet](https://www.st.com/resource/en/datasheet/lsm6dsr.pdf).

## Debug validation

Add `lsm6ds_data` to the CCS Expressions view and enable continuous refresh.
The most useful fields are:

- `initialized`: must be `1`
- `device_id`: must be `0x6B`
- `calibration.gyro_valid`: must be `1`
- `calibration.level_valid`: must be `1`
- `calibration.level_accel_mps2`: should remain close to one g
- `sample.sequence`: must increase continuously
- `sample.valid`: must be `1` on accepted samples
- `last_status`: normally alternates between `OK` and `NO_NEW_DATA`
- `communication_error_count`, `timing_fault_count`, and `saturation_count`:
  should remain zero during a stationary bench test

`NO_NEW_DATA` does not invalidate the last complete sample. It only means that
the next 416 Hz sensor frame was not ready during that 1 kHz poll.
