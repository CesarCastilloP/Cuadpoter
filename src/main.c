/**
 * @file main.c
 * @author Alberto Vazquez
 *
 * @brief System initialization and cooperative flight-sensor scheduler.
 *
 * @version 1.5.0
 * @date 2026-09-27
 */

#include "functions.h"
#include "driverlib/sysctl.h"
#include "systick.h"
#include "rp4tdm.h"
#include "bmp390l.h"
#include "flight_control.h"
#include "lsm6ds.h"
#include "i2c0_drone.h"
#include "motor_output.h"
#include "status_led.h"
#include "telemetry.h"
#include "uart.h"

/* The IMU is polled faster than its 416 Hz ODR to minimize ready-detection jitter. */
#define IMU_POLL_RATE_HZ                1000U
#define RECEIVER_SERVICE_RATE_HZ        100U
#define BAROMETER_SERVICE_PERIOD_MS     80U
#define ESC_PREPARATION_TIME_MS         3000U
#define IMU_CALIBRATION_RETRY_DELAY_MS  250U

static uint32_t config(void);
static bool schedule_is_due(uint64_t now_cycles,
                            uint64_t period_cycles,
                            uint64_t *next_deadline,
                            volatile uint32_t *missed_period_count);
static void wait_for_esc_preparation(uint32_t start_ms);
static LSM6DS_Status_t initialize_imu(void);
static void halt_initialization(void);

/**
 * @brief Radio receiver data shared with the interrupt-driven decoder.
 */
RP4TDM_Data_t rp4tdm_data = {0};

/**
 * @brief Barometer calibration and latest compensated measurement.
 */
BMP390L_Data_t barometer_data;

/** Flight controller state and normalized motor commands. */
FlightControl_Data_t flight_control_data;

/** Motor pulse conversion and synchronized PWM hardware state. */
MotorOutput_Data_t motor_output_data;

/** LaunchPad initialization-state indicator. */
StatusLed_Data_t status_led_data;

/** Binary flight telemetry sent through the LaunchPad USB virtual COM port. */
Telemetry_Data_t telemetry_data;

/**
 * @brief IMU state and flight-ready outputs.
 *
 * The identity map keeps the sensor coordinate system. Change only this map
 * after defining how the board is mounted in the aircraft.
 */
LSM6DS_Data_t lsm6ds_data;
static const LSM6DS_AxisMap_t g_imu_axis_map = {
    { (uint8_t)LSM6DS_AXIS_X,
      (uint8_t)LSM6DS_AXIS_Y,
      (uint8_t)LSM6DS_AXIS_Z },
    { 1, 1, 1 }
};

/* Persistent runtime diagnostics that are easy to inspect in the debugger. */
volatile LSM6DS_Status_t g_imu_runtime_status =
    LSM6DS_STATUS_NOT_INITIALIZED;
volatile BMP390L_Status_t g_barometer_runtime_status =
    BMP390L_STATUS_NOT_INITIALIZED;
volatile FlightControl_Status_t g_flight_control_runtime_status =
    FLIGHT_CONTROL_STATUS_NOT_INITIALIZED;
volatile MotorOutput_Status_t g_motor_output_runtime_status =
    MOTOR_OUTPUT_STATUS_NOT_INITIALIZED;
volatile StatusLed_Status_t g_status_led_runtime_status =
    STATUS_LED_STATUS_NOT_INITIALIZED;
volatile Telemetry_Status_t g_telemetry_runtime_status =
    TELEMETRY_STATUS_NOT_INITIALIZED;
volatile uint32_t g_reset_cause = 0U;
volatile bool g_system_ready = false;
volatile uint32_t g_imu_initialization_retry_count = 0U;
volatile uint32_t g_imu_poll_missed_periods = 0U;
volatile uint32_t g_receiver_missed_periods = 0U;
volatile uint32_t g_barometer_missed_periods = 0U;

/**
 * @brief Run each device at its own deadline without blocking flight sampling.
 */
int main(void)
{
    uint32_t system_clock_hz;
    uint64_t imu_period_cycles;
    uint64_t receiver_period_cycles;
    uint64_t barometer_period_cycles;
    uint64_t next_imu_poll;
    uint64_t next_receiver_service;
    uint64_t next_barometer_service;
    uint64_t now_cycles;

    system_clock_hz = config();
    imu_period_cycles =
        (uint64_t)system_clock_hz / (uint64_t)IMU_POLL_RATE_HZ;
    receiver_period_cycles =
        (uint64_t)system_clock_hz / (uint64_t)RECEIVER_SERVICE_RATE_HZ;
    barometer_period_cycles =
        ((uint64_t)system_clock_hz *
         (uint64_t)BAROMETER_SERVICE_PERIOD_MS) / 1000ULL;

    now_cycles = Timebase_GetCycles();
    next_imu_poll = now_cycles;
    next_receiver_service = now_cycles;
    next_barometer_service = now_cycles + barometer_period_cycles;

    while(true)
    {
        /* Consume completed receiver buffers before a newer frame replaces them. */
        RP4TDM_Process(&rp4tdm_data);

        now_cycles = Timebase_GetCycles();
        if(schedule_is_due(now_cycles, imu_period_cycles,
                           &next_imu_poll,
                           &g_imu_poll_missed_periods))
        {
            g_imu_runtime_status = LSM6DS_Update(&lsm6ds_data);
            g_flight_control_runtime_status = FlightControl_Update(
                &flight_control_data,
                g_imu_runtime_status,
                &lsm6ds_data.sample,
                &rp4tdm_data.controls);
            g_motor_output_runtime_status = MotorOutput_Update(
                &motor_output_data,
                &flight_control_data.output);
        }

        now_cycles = Timebase_GetCycles();
        if(schedule_is_due(now_cycles, receiver_period_cycles,
                           &next_receiver_service,
                           &g_receiver_missed_periods))
        {
            /*
             * Frame parsing runs every pass above. Only link age advances at
             * 100 Hz, preserving the intended timeout duration.
             */
            RP4TDM_TimeoutTick(&rp4tdm_data);
        }

        now_cycles = Timebase_GetCycles();
        if(schedule_is_due(now_cycles, barometer_period_cycles,
                           &next_barometer_service,
                           &g_barometer_missed_periods))
        {
            g_barometer_runtime_status =
                BMP390L_Update(&barometer_data);
        }

        g_telemetry_runtime_status = Telemetry_Update(
            &telemetry_data,
            &lsm6ds_data,
            &flight_control_data,
            &motor_output_data);
    }
}

/**
 * @brief Advance an absolute deadline and report skipped service periods.
 */
static bool schedule_is_due(uint64_t now_cycles,
                            uint64_t period_cycles,
                            uint64_t *next_deadline,
                            volatile uint32_t *missed_period_count)
{
    uint64_t elapsed_periods;
    uint64_t missed_periods;
    uint32_t available_count;

    if((period_cycles == 0ULL) || (next_deadline == NULL) ||
       (missed_period_count == NULL) || (now_cycles < *next_deadline))
    {
        return false;
    }

    elapsed_periods =
        ((now_cycles - *next_deadline) / period_cycles) + 1ULL;
    *next_deadline += elapsed_periods * period_cycles;

    if(elapsed_periods > 1ULL)
    {
        missed_periods = elapsed_periods - 1ULL;
        available_count = 0xFFFFFFFFU - *missed_period_count;
        if(missed_periods > (uint64_t)available_count)
        {
            *missed_period_count = 0xFFFFFFFFU;
        }
        else
        {
            *missed_period_count += (uint32_t)missed_periods;
        }
    }

    return true;
}

/**
 * @brief Configure clocks, communications, and all sensors.
 */
static uint32_t config(void)
{
    uint32_t system_clock_hz;
    uint32_t esc_start_ms;
    RP4TDM_Status_t receiver_status;
    BMP390L_Status_t barometer_status;
    FlightControl_Status_t flight_control_status;
    MotorOutput_Status_t motor_output_status;
    StatusLed_Status_t status_led_status;
    Telemetry_Status_t telemetry_status;
    LSM6DS_Status_t imu_status;
    I2C0_Status_t i2c_status;

    g_reset_cause = SysCtlResetCauseGet();
    SysCtlResetCauseClear(g_reset_cause);

    system_clock_hz = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ |
                                        SYSCTL_OSC_MAIN |
                                        SYSCTL_USE_PLL |
                                        SYSCTL_CFG_VCO_480,
                                        120000000U);
    if(system_clock_hz == 0U)
    {
        halt_initialization();
    }

    status_led_status = StatusLed_Init(&status_led_data);
    g_status_led_runtime_status = status_led_status;
    if(status_led_status != STATUS_LED_STATUS_OK)
    {
        halt_initialization();
    }
    g_status_led_runtime_status = StatusLed_Set(
        &status_led_data, STATUS_LED_STAGE_ESC_PREPARATION);

    SysTick_Init(system_clock_hz);
    if(!Timebase_Init(system_clock_hz))
    {
        halt_initialization();
    }

    /* Start a valid low-throttle PWM signal before any blocking sensor setup. */
    motor_output_status = MotorOutput_Init(&motor_output_data,
                                           system_clock_hz,
                                           NULL);
    g_motor_output_runtime_status = motor_output_status;
    if(motor_output_status != MOTOR_OUTPUT_STATUS_SAFE)
    {
        halt_initialization();
    }
    esc_start_ms = SysTick_Millis();

    receiver_status = RP4TDM_Init(&rp4tdm_data, 420000U, NULL);
    if((receiver_status != RP4TDM_STATUS_OK) &&
       (receiver_status != RP4TDM_STATUS_WAITING_FOR_DATA))
    {
        halt_initialization();
    }

    /* Let the ESC startup tones finish before measuring gyro bias. */
    wait_for_esc_preparation(esc_start_ms);
    g_status_led_runtime_status = StatusLed_Set(
        &status_led_data, STATUS_LED_STAGE_SENSOR_INITIALIZATION);

    /* Both onboard sensors support 400 kHz fast-mode I2C. */
    i2c_status = I2C0_Init(system_clock_hz, I2C_400);
    if(i2c_status != I2C_OK)
    {
        halt_initialization();
    }

    barometer_status = BMP390L_Init(&barometer_data);
    g_barometer_runtime_status = barometer_status;

    imu_status = initialize_imu();
    if(imu_status != LSM6DS_STATUS_OK)
    {
        halt_initialization();
    }

    flight_control_status = FlightControl_Init(&flight_control_data, NULL);
    g_flight_control_runtime_status = flight_control_status;
    if(flight_control_status != FLIGHT_CONTROL_STATUS_OK)
    {
        halt_initialization();
    }

    telemetry_status = Telemetry_Init(
        &telemetry_data,
        TELEMETRY_DEFAULT_BAUD_RATE,
        TELEMETRY_DEFAULT_OUTPUT_RATE_HZ);
    g_telemetry_runtime_status = telemetry_status;
    if(telemetry_status != TELEMETRY_STATUS_OK)
    {
        halt_initialization();
    }

    uart6_init(115200U, NULL);
    g_status_led_runtime_status = StatusLed_Set(
        &status_led_data, STATUS_LED_STAGE_READY);
    g_system_ready = true;
    return system_clock_hz;
}

/**
 * @brief Keep a valid low-throttle command long enough for every ESC to arm.
 */
static void wait_for_esc_preparation(uint32_t start_ms)
{
    uint32_t elapsed_ms = (uint32_t)(SysTick_Millis() - start_ms);

    if(elapsed_ms < ESC_PREPARATION_TIME_MS)
    {
        Delay_ms(ESC_PREPARATION_TIME_MS - elapsed_ms);
    }
}

/**
 * @brief Calibrate the IMU after ESC startup vibration has stopped.
 *
 * Motion during calibration is recoverable. Keeping the PWM outputs at their
 * safe pulse and retrying avoids requiring a microcontroller reset.
 */
static LSM6DS_Status_t initialize_imu(void)
{
    LSM6DS_Status_t status;

    do
    {
        status = LSM6DS_Init(&lsm6ds_data, &g_imu_axis_map);
        g_imu_runtime_status = status;

        if((status == LSM6DS_STATUS_CALIBRATION_MOTION) ||
           (status == LSM6DS_STATUS_CALIBRATION_TIMEOUT))
        {
            if(g_imu_initialization_retry_count < 0xFFFFFFFFU)
            {
                g_imu_initialization_retry_count++;
            }
            Delay_ms(IMU_CALIBRATION_RETRY_DELAY_MS);
        }
    }
    while((status == LSM6DS_STATUS_CALIBRATION_MOTION) ||
          (status == LSM6DS_STATUS_CALIBRATION_TIMEOUT));

    return status;
}

/**
 * @brief Stop initialization with safe PWM and a visible error indication.
 */
static void halt_initialization(void)
{
    if(status_led_data.initialized)
    {
        g_status_led_runtime_status = StatusLed_Set(
            &status_led_data, STATUS_LED_STAGE_ERROR);
    }

    while(true)
    {
    }
}
