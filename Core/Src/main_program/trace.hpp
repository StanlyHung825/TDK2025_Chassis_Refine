#ifndef TRACE_HPP
#define TRACE_HPP

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f446xx.h"

/*-----------Front------------*/
/*          0   Right         */
/*          1     |           */
/*          2     |           */
/*          3     V           */
/*          4   Left          */
/*----------------------------*/
/*-----------Middle-----------*/
/*          5   Right         */
/*          6   Left          */
/*----------------------------*/

// TODO: trace_check_point

#define w_kp 0.21
#define w_kd 0

extern ADC_HandleTypeDef hadc1;
extern uint16_t adcRead[7];


void   trace_init();            // Initialize infrared sensor
double trace_straight_line();   // Output: the rotation speed. Compute the offset from straight line
void   trace_check_point();     // Check if the car reach the check point and refresh the current true location

#ifdef __cplusplus
}
#endif

#endif // TRACE_HPP