/**
 * @file functions.h
 * @author Alberto Vazquez
 *
 * @brief Common scalar types and constants shared by firmware modules.
 *
 * @details This header contains no executable control logic. It gives every
 * project module the same fixed-width integer types, Boolean type, NULL value,
 * and float32_t alias. Fixed widths matter because sensor registers, telemetry
 * fields, counters, and peripheral registers require known byte sizes.
 *
 * @version 1.0.0
 * @date 2026-08-12
 */

/**
 * @addtogroup main
 * @{
 */

#ifndef INCLUDE_FUNCTIONS_H_
#define INCLUDE_FUNCTIONS_H_

#include <stdint.h>
#include <stdbool.h>

#ifndef NULL
/** Null pointer constant used by the TI C environment when not provided. */
#define NULL    0
#endif

#ifndef M_PI
/** Single-precision ratio of circumference to diameter. Unit: radians/half-turn. */
#define M_PI 3.14159265358979323846f
#endif

/** Project alias for IEEE-754 single-precision values. Unit depends on field. */
#define float32_t float

/** Project alias for IEEE-754 double-precision values. Unit depends on field. */
#define double64_t double

/** Byte/float overlay retained for modules that need explicit float packing. */
typedef union
{
    /** Four bytes in target little-endian memory order. */
    uint8_t q[4];
    /** The same 32 bits interpreted as one IEEE-754 binary32 value. */
    float32_t f_value;
}ieee_format;

#endif /* INCLUDE_FUNCTIONS_H_ */
