/**
 * @file functions.h
 * @author Alberto Vazquez
 *
 * @brief header file, h code for functions of the project
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
#define NULL    0
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define float32_t float

#define double64_t double


typedef union
{
    uint8_t q[4];
    float32_t f_value;
}ieee_format;

#endif /* INCLUDE_FUNCTIONS_H_ */
