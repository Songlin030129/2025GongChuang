#ifndef __COMMON_INC_H__
#define __COMMON_INC_H__

#ifdef __cplusplus

extern "C"
{
#endif

    /*---------------------------- C Scope ---------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "task.h"
#include "string.h"

#include "can.h"
#include "main.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"
#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

    void Main();

#ifdef __cplusplus
}

#endif
#endif // __COMMON_INC_H__