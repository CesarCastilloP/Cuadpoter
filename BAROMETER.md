# BMP390 Barometer

The barometer follows the same instance-based pattern as the IMU:

```c
BMP390L_Data_t barometer;

status = BMP390L_Init(&barometer);
status = BMP390L_Update(&barometer);
```

`BMP390L_Init()` detects addresses `0x76` and `0x77`, verifies the `0x60`
chip ID, enters sleep before configuration, checks every configuration write,
and loads the 21 factory calibration bytes in one transaction.

`BMP390L_Update()` first checks that pressure and temperature are both ready.
It then reads their six registers in one burst so both raw values belong to
the same measurement. Compensation follows the Bosch floating-point reference
formula.

## Published values

Use a measurement only when all three conditions are true:

```c
status == BMP390L_STATUS_OK
barometer.sample.fresh
barometer.sample.valid
```

| Field | Unit | Meaning |
| --- | --- | --- |
| `sample.pressure_pa` | Pa | Compensated absolute pressure |
| `sample.pressure_hpa` | hPa | Compensated pressure for telemetry |
| `sample.temperature_c` | deg C | Compensated sensor temperature |
| `sample.timestamp_us` | us | Monotonic acquisition timestamp |
| `sample.sequence` | count | Increments once per fresh sample |

Raw pressure and temperature remain available in `sample.raw_pressure` and
`sample.raw_temperature` for diagnostics.

## Configuration

- Output data rate: 12.5 Hz
- Pressure oversampling: x32
- Temperature oversampling: x2
- IIR filter coefficient: 1
- Pressure and temperature enabled in normal mode

The scheduler calls `BMP390L_Update()` every 80 ms. A
`BMP390L_STATUS_NO_NEW_DATA` result is harmless and leaves the previous sample
unchanged with `fresh == false`.

The configuration and compensation constants come from the
[Bosch BMP390 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf).

## CCS validation

Add `barometer_data` to the Expressions view. The main checks are:

- `initialized == 1`
- `device_id == 0x60`
- `sample.sequence` increases about 12 or 13 times per second
- `sample.valid == 1` on accepted measurements
- `sample.pressure_hpa` is physically reasonable for local altitude/weather
- `sample.temperature_c` is physically reasonable
- `communication_error_count == 0`

Altitude is intentionally not calculated in the sensor driver because it
requires a reference sea-level pressure or a takeoff-pressure reference from
the flight estimator.
