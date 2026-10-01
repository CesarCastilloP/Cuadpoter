/**
 * @file main.c
 * @author Alberto Vazquez
 *
 * @brief System initialization and cooperative flight-sensor scheduler.
 *
 * @details Beginner's reading guide:
 * - This file is the program entry point. Execution starts in main().
 * - config() initializes the clock, communications, sensors, controller, LEDs,
 *   and motor-output hardware in the only safe dependency order.
 * - The infinite while loop is a cooperative scheduler: it repeatedly checks
 *   time deadlines and calls each module when that module is due.
 * - Inputs are hardware samples and CRSF receiver commands. Outputs are four
 *   normalized motor demands, four ESC pulse widths in microseconds, telemetry,
 *   and status LEDs. This file does not implement sensor mathematics or PID
 *   equations; it connects the modules that own those responsibilities.
 * - A variable ending in _hz is a frequency in hertz, _cycles is a count of
 *   120 MHz system-clock ticks, _us is microseconds, and _ms is milliseconds.
 * - "static" makes a name private to this source file; "volatile" marks data
 *   that may change outside the current statement, for example in an ISR.
 *
 * @version 1.7.0
 * @date 2026-09-29
 */

#include "functions.h"       /* Common integer, Boolean, and DriverLib definitions. */
#include "driverlib/sysctl.h" /* TM4C clock and reset-cause hardware functions. */
#include "systick.h"         /* Millisecond delays and 64-bit execution timebase. */
#include "rp4tdm.h"          /* CRSF radio-receiver decoder and normalized sticks. */
#include "bmp390l.h"         /* Pressure and temperature sensor driver. */
#include "flight_control.h"  /* Attitude estimation, PID control, and motor mixer. */
#include "lsm6ds.h"          /* Six-axis accelerometer and gyroscope driver. */
#include "lis2mdl.h"         /* Three-axis magnetic-field sensor driver. */
#include "i2c0_drone.h"      /* Shared I2C0 transport used by all three sensors. */
#include "motor_output.h"    /* Safety policy and normalized-to-microsecond mapping. */
#include "status_led.h"      /* LaunchPad RGB LED initialization-state display. */
#include "telemetry.h"       /* Binary UART0 diagnostic frame generator. */
#include "uart.h"            /* UART hardware driver; UART6 is reserved here. */

/*
 * Cooperative-scheduler rates.
 *
 * These constants specify how often main() asks a module to do work.  A poll
 * rate is not necessarily the physical sensor output rate.  For example, the
 * IMU produces a new sample at 416 Hz, while main() checks its DATA_READY bits
 * at 1000 Hz.  The extra checks reduce the delay between the instant a sample
 * becomes ready and the instant the controller consumes it.  LSM6DS_Update()
 * returns NO_NEW_DATA on the checks that occur between physical samples.
 */
#define IMU_POLL_RATE_HZ                1000U /* One IMU readiness check every 1 ms. */
#define RECEIVER_SERVICE_RATE_HZ        100U  /* One receiver link-age tick every 10 ms. */
#define MAGNETOMETER_POLL_RATE_HZ       100U  /* One readiness check every 10 ms for 50 Hz data. */
#define BAROMETER_SERVICE_PERIOD_MS     80U   /* One pressure service every 80 ms = 12.5 Hz. */
#define ESC_PREPARATION_TIME_MS         3000U /* Hold 1000 us pulses for at least 3 s at boot. */
#define IMU_CALIBRATION_RETRY_DELAY_MS  250U  /* Pause 250 ms after rejected moving calibration. */

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

/** Latest raw, calibrated, and airframe-mapped magnetic measurement. */
LIS2MDL_Data_t lis2mdl_data;

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
    /* source_axis: body X<-sensor X, body Y<-sensor Y, body Z<-sensor Z. */
    { (uint8_t)LSM6DS_AXIS_X,
      (uint8_t)LSM6DS_AXIS_Y,
      (uint8_t)LSM6DS_AXIS_Z },
    { 1U, 1U, 1U } /* sign: no sensor-axis inversion for current mounting. */
};

/* Persistent runtime diagnostics that are easy to inspect in the debugger. */
/* Latest IMU update/initialization result; LSM6DS_Status_t. */
volatile LSM6DS_Status_t g_imu_runtime_status =
    LSM6DS_STATUS_NOT_INITIALIZED;
/* Latest barometer result; BMP390L_Status_t. */
volatile BMP390L_Status_t g_barometer_runtime_status =
    BMP390L_STATUS_NOT_INITIALIZED;
/* Latest magnetometer result; LIS2MDL_Status_t. */
volatile LIS2MDL_Status_t g_magnetometer_runtime_status =
    LIS2MDL_STATUS_NOT_INITIALIZED;
/* Latest control-law result; FlightControl_Status_t. */
volatile FlightControl_Status_t g_flight_control_runtime_status =
    FLIGHT_CONTROL_STATUS_NOT_INITIALIZED;
/* Latest normalized-to-PWM result; MotorOutput_Status_t. */
volatile MotorOutput_Status_t g_motor_output_runtime_status =
    MOTOR_OUTPUT_STATUS_NOT_INITIALIZED;
/* Latest LaunchPad LED driver result; StatusLed_Status_t. */
volatile StatusLed_Status_t g_status_led_runtime_status =
    STATUS_LED_STATUS_NOT_INITIALIZED;
/* Latest USB telemetry service result; Telemetry_Status_t. */
volatile Telemetry_Status_t g_telemetry_runtime_status =
    TELEMETRY_STATUS_NOT_INITIALIZED;
volatile uint32_t g_reset_cause = 0U; /**< Raw SysCtl reset-cause bit mask. */
volatile bool g_system_ready = false; /**< True after every mandatory module starts. */
volatile uint32_t g_imu_initialization_retry_count = 0U; /**< Motion retries. */
volatile uint32_t g_imu_poll_missed_periods = 0U; /**< Skipped 1 kHz polls. */
volatile uint32_t g_receiver_missed_periods = 0U; /**< Skipped 100 Hz link ticks. */
volatile uint32_t g_barometer_missed_periods = 0U; /**< Skipped 80 ms services. */
volatile uint32_t g_magnetometer_missed_periods = 0U; /**< Skipped 100 Hz polls. */

/**
 * @brief Run each device at its own deadline without blocking flight sampling.
 * @return This embedded entry point never returns; the value exists only to
 *         satisfy the C runtime signature.
 * @note The loop order is receiver parse, IMU/control/motors, receiver age,
 *       barometer, magnetometer, and telemetry. Fresh IMU data is the control clock.
 */
int main(void)
{
    /*
     * system_clock_hz is the real processor frequency reported by DriverLib.
     * The requested and normally returned value is 120,000,000 Hz.  It is used
     * instead of a repeated literal so all timing remains correct if the clock
     * configuration changes later.
     */
    uint32_t system_clock_hz;

    /*
     * A period expressed in clock cycles is the interval between two service
     * opportunities.  At 120 MHz the current nominal values are:
     *
     *   IMU poll:          120,000 cycles = 1 ms       = 1000 Hz
     *   Receiver service: 1,200,000 cycles = 10 ms     = 100 Hz
     *   Barometer:        9,600,000 cycles = 80 ms     = 12.5 Hz
     *   Magnetometer:     1,200,000 cycles = 10 ms     = 100 Hz polling
     *
     * uint64_t is used because the free-running timebase and all deadlines are
     * 64-bit values; this prevents overflow during normal flight duration.
     */
    uint64_t imu_period_cycles;
    uint64_t receiver_period_cycles;
    uint64_t barometer_period_cycles;
    uint64_t magnetometer_period_cycles;

    /*
     * Every next_* variable is an absolute TIMER7 timestamp, not a delay.
     * schedule_is_due() advances it by exact multiples of its period.  This
     * prevents the execution time of one iteration from slowly shifting the
     * intended frequency.
     */
    uint64_t next_imu_poll;
    uint64_t next_receiver_service;
    uint64_t next_barometer_service;
    uint64_t next_magnetometer_poll;

    /* Latest 64-bit snapshot of the free-running 120 MHz TIMER7 timebase. */
    uint64_t now_cycles;

    /*
     * Initialize every mandatory peripheral before starting periodic work.
     * config() returns only after the clock, safe PWM, receiver, I2C, sensors,
     * controller, telemetry, and status LED have reached their required state.
     */
    system_clock_hz = config();

    /*
     * Convert each requested frequency to cycles with:
     *
     *     period_cycles = system_clock_hz / frequency_hz
     *
     * The cast to uint64_t makes the division and later deadline arithmetic
     * use 64 bits.  These are integer periods because hardware time is counted
     * in whole processor-clock cycles.
     */
    imu_period_cycles =
        (uint64_t)system_clock_hz / (uint64_t)IMU_POLL_RATE_HZ;
    receiver_period_cycles =
        (uint64_t)system_clock_hz / (uint64_t)RECEIVER_SERVICE_RATE_HZ;

    /*
     * The barometer interval is configured in milliseconds, so multiply by
     * 80 ms and divide by 1000 ms/s to obtain the matching cycle count.
     */
    barometer_period_cycles =
        ((uint64_t)system_clock_hz *
         (uint64_t)BAROMETER_SERVICE_PERIOD_MS) / 1000ULL;
    magnetometer_period_cycles =
        (uint64_t)system_clock_hz /
        (uint64_t)MAGNETOMETER_POLL_RATE_HZ;

    /* Read one common time origin after every period has been calculated. */
    now_cycles = Timebase_GetCycles();

    /*
     * IMU, receiver, and magnetometer are due immediately on the first loop.
     * The barometer waits one full 80 ms period because its first compensated
     * reading is not needed to start attitude control.
     */
    next_imu_poll = now_cycles;
    next_receiver_service = now_cycles;
    next_barometer_service = now_cycles + barometer_period_cycles;
    next_magnetometer_poll = now_cycles;

    /*
     * There is no operating system in this firmware.  This endless loop is the
     * scheduler.  Each pass performs short, non-blocking work and checks which
     * timed service has reached its absolute deadline.
     */
    while(true)
    {
        /*
         * Receiver bytes arrive asynchronously in the UART4 interrupt.  This
         * call is therefore made on every loop pass, with no periodic gate.  It
         * validates any completed CRSF frame and publishes normalized throttle,
         * roll, pitch, and yaw before a newer frame can replace the buffer.
         */
        RP4TDM_Process(&rp4tdm_data);

        /*
         * Capture time immediately before testing the IMU deadline.  TIMER7 is
         * free-running, so this read does not stop or reset the counter.
         */
        now_cycles = Timebase_GetCycles();

        /*
         * Test the 1 ms IMU polling deadline.  Arguments mean:
         *
         *   now_cycles                 current absolute time [clock cycles]
         *   imu_period_cycles          1 ms desired interval [clock cycles]
         *   &next_imu_poll             deadline updated for the following poll
         *   &g_imu_poll_missed_periods diagnostic count of skipped 1 ms slots
         *
         * The body runs only when the deadline has arrived.  This is the code
         * shown in the referenced CCS image: it establishes the timing of IMU
         * readiness checks and therefore the opportunity to run flight control.
         */
        if(schedule_is_due(now_cycles, imu_period_cycles,
                           &next_imu_poll,
                           &g_imu_poll_missed_periods))
        {
            /*
             * Ask the LSM6DSR whether both gyro and accelerometer data are new.
             * When ready, the driver reads all six axes, converts their units,
             * computes dt, updates lsm6ds_data.sample, and returns STATUS_OK.
             * Between 416 Hz samples it returns STATUS_NO_NEW_DATA normally.
             */
            g_imu_runtime_status = LSM6DS_Update(&lsm6ds_data);

            /*
             * Pass the IMU result together with the newest IMU, magnetometer,
             * and receiver values to the controller.  FlightControl_Update()
             * recomputes attitude, PID terms, and the X mixer only for a valid
             * fresh IMU sample.  Its normalized [0,1] motor requests are stored
             * in flight_control_data.output.
             */
            g_flight_control_runtime_status = FlightControl_Update(
                &flight_control_data,
                g_imu_runtime_status,
                &lsm6ds_data.sample,
                &lis2mdl_data,
                &rp4tdm_data.controls);

            /*
             * Apply receiver/sensor/controller safety policy and convert the
             * four normalized mixer results into 1000...2000 us ESC commands.
             * EscPwm writes all four PWM compare values synchronously.  Calling
             * this after the controller preserves the chain:
             * IMU -> attitude/PID -> mixer -> safety -> PWM.
             */
            g_motor_output_runtime_status = MotorOutput_Update(
                &motor_output_data,
                &flight_control_data.output);
        }

        /* Take a new timestamp because the IMU/control/PWM block consumed time. */
        now_cycles = Timebase_GetCycles();

        /*
         * Every 10 ms, advance the receiver link-age counters.  Frame parsing
         * itself occurred at the top of every loop; this timed call exists only
         * so one timeout tick always represents exactly 10 ms.  When too many
         * ticks pass without a valid CRSF frame, controls.valid becomes false
         * and the motor-output safety logic commands the minimum pulse.
         */
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

        /* Refresh time after receiver service before checking the next device. */
        now_cycles = Timebase_GetCycles();

        /*
         * Service the BMP390L every 80 ms (12.5 Hz).  The barometer is currently
         * measured and exposed for diagnostics; it does not participate in the
         * roll, pitch, yaw, or motor-control calculations.
         */
        if(schedule_is_due(now_cycles, barometer_period_cycles,
                           &next_barometer_service,
                           &g_barometer_missed_periods))
        {
            /* Read raw pressure/temperature and publish compensated SI values. */
            g_barometer_runtime_status =
                BMP390L_Update(&barometer_data);
        }

        /* Refresh time again because an I2C barometer transaction consumed time. */
        now_cycles = Timebase_GetCycles();

        /*
         * Check the LIS2MDL every 10 ms.  Its configured output rate is 50 Hz,
         * so approximately every other check reports NO_NEW_DATA.  A fresh
         * sample is calibrated and saved for magnetic heading on a later IMU
         * control iteration; this block never drives a motor directly.
         */
        if(schedule_is_due(now_cycles, magnetometer_period_cycles,
                           &next_magnetometer_poll,
                           &g_magnetometer_missed_periods))
        {
            /* Read X/Y/Z magnetic field when the LIS2MDL data-ready bit is set. */
            g_magnetometer_runtime_status =
                LIS2MDL_Update(&lis2mdl_data);
        }

        /*
         * Give telemetry an opportunity to send on every loop pass.  The
         * telemetry module owns its 100 Hz deadline and UART0 capacity check,
         * so this call returns quickly when no frame is due or the transmitter
         * cannot accept a complete frame.  It never delays IMU scheduling.
         */
        g_telemetry_runtime_status = Telemetry_Update(
            &telemetry_data,
            &lsm6ds_data,
            &lis2mdl_data,
            &flight_control_data,
            &motor_output_data);
    }
}

/**
 * @brief Advance an absolute deadline and report skipped service periods.
 * @param now_cycles Current monotonic TIMER7 time [system-clock cycles].
 * @param period_cycles Desired interval between services [system-clock cycles].
 * @param next_deadline Writable absolute deadline [system-clock cycles].
 * @param missed_period_count Saturating diagnostic counter for skipped periods.
 * @return true when the service is due now; false before its deadline or on bad input.
 */
static bool schedule_is_due(uint64_t now_cycles,
                            uint64_t period_cycles,
                            uint64_t *next_deadline,
                            volatile uint32_t *missed_period_count)
{
    uint64_t elapsed_periods; /* Deadlines elapsed since the last service. */
    uint64_t missed_periods;  /* Elapsed periods beyond the one serviced now. */
    uint32_t available_count; /* Remaining space before diagnostic saturation. */

    /*
     * Reject a zero period to avoid division by zero.  Reject NULL output
     * pointers because this function must update both the next deadline and
     * the diagnostic counter.  The final comparison is the normal early exit:
     * current time is still before the scheduled absolute deadline.
     */
    if((period_cycles == 0ULL) || (next_deadline == NULL) ||
       (missed_period_count == NULL) || (now_cycles < *next_deadline))
    {
        return false;
    }

    /*
     * Count how many complete service slots have elapsed, including the slot
     * being serviced now.  Example: if now is 2.4 periods late, integer
     * division gives 2 and +1 advances the deadline by 3 whole periods.
     */
    elapsed_periods =
        ((now_cycles - *next_deadline) / period_cycles) + 1ULL;

    /* Move from the previous deadline, rather than from now, to prevent drift. */
    *next_deadline += elapsed_periods * period_cycles;

    /* More than one elapsed slot means at least one invocation was skipped. */
    if(elapsed_periods > 1ULL)
    {
        /* The current invocation is executed, so only the remaining slots are missed. */
        missed_periods = elapsed_periods - 1ULL;

        /* Compute how much room remains in the 32-bit diagnostic counter. */
        available_count = 0xFFFFFFFFU - *missed_period_count;

        /* Saturate at UINT32_MAX instead of wrapping the counter back to zero. */
        if(missed_periods > (uint64_t)available_count)
        {
            *missed_period_count = 0xFFFFFFFFU;
        }
        else
        {
            *missed_period_count += (uint32_t)missed_periods;
        }
    }

    /* Tell the caller to execute this service exactly once on the current pass. */
    return true;
}

/**
 * @brief Configure clocks, communications, and all sensors.
 * @return Actual system clock [Hz] after every mandatory subsystem is ready.
 * @note Optional barometer/magnetometer errors remain observable but do not halt
 *       attitude control. Clock, timebase, PWM, IMU, control, and telemetry fail hard.
 */
static uint32_t config(void)
{
    uint32_t system_clock_hz; /* Actual system clock returned by DriverLib, Hz. */
    uint32_t esc_start_ms;    /* SysTick time when safe PWM began, milliseconds. */
    RP4TDM_Status_t receiver_status; /* Receiver initialization result. */
    BMP390L_Status_t barometer_status; /* Optional barometer initialization result. */
    LIS2MDL_Status_t magnetometer_status; /* Optional compass result. */
    FlightControl_Status_t flight_control_status; /* Control initialization result. */
    MotorOutput_Status_t motor_output_status; /* PWM path initialization result. */
    StatusLed_Status_t status_led_status; /* LaunchPad indicator result. */
    Telemetry_Status_t telemetry_status; /* USB telemetry initialization result. */
    LSM6DS_Status_t imu_status; /* Mandatory IMU initialization result. */
    I2C0_Status_t i2c_status;   /* Shared I2C0 bus initialization result. */

    /* Capture reset origin before clearing the sticky hardware flags. */
    g_reset_cause = SysCtlResetCauseGet();
    SysCtlResetCauseClear(g_reset_cause);

    /* 25 MHz crystal + 480 MHz VCO produce a 120 MHz processor clock. */
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

    /* Barometer failure is observable but does not block attitude flight control. */
    barometer_status = BMP390L_Init(&barometer_data);
    g_barometer_runtime_status = barometer_status;

    /* Magnetic heading is optional: invalid data falls back to yaw-rate mode. */
    magnetometer_status = LIS2MDL_Init(&lis2mdl_data);
    g_magnetometer_runtime_status = magnetometer_status;

    imu_status = initialize_imu();
    if(imu_status != LSM6DS_STATUS_OK)
    {
        halt_initialization();
    }

    flight_control_status = FlightControl_Init(
        &flight_control_data,
        NULL,
        &lsm6ds_data.calibration);
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

    /* UART6 is reserved at 115200 bit/s for future auxiliary integration. */
    uart6_init(115200U, NULL);
    g_status_led_runtime_status = StatusLed_Set(
        &status_led_data, STATUS_LED_STAGE_READY);
    g_system_ready = true;
    return system_clock_hz;
}

/**
 * @brief Keep a valid low-throttle command long enough for every ESC to arm.
 * @param start_ms SysTick value [milliseconds] captured when safe PWM began.
 * @return Nothing; the function blocks only for the remaining portion of 3 seconds.
 */
static void wait_for_esc_preparation(uint32_t start_ms)
{
    /* Unsigned subtraction remains correct across the millisecond-counter wrap. */
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
 * @return Final IMU initialization/calibration status after all recoverable retries.
 */
static LSM6DS_Status_t initialize_imu(void)
{
    /* Current sensor initialization/calibration result. */
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
 * @return Nothing; this function intentionally remains in an infinite loop.
 * @warning PWM already initialized before most failure paths remains at its last
 *          safe command. A clock failure occurs too early for PWM initialization.
 */
static void halt_initialization(void)
{
    /* Keep existing safe PWM running and make the failed boot visible on LEDs. */
    if(status_led_data.initialized)
    {
        g_status_led_runtime_status = StatusLed_Set(
            &status_led_data, STATUS_LED_STAGE_ERROR);
    }

    while(true)
    {
    }
}
