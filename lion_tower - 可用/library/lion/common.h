#ifndef _COMMON_H_
#define _COMMON_H_

#include "stm32f4xx.h"
#include <math.h>

// 取值限制
#define value_limit(val, min, max)\
if(val <= min)\
{\
    val = min;\
}\
else if(val >= max)\
{\
    val = max;\
}\

#define PI              3.142f
#define DEG_TO_RAD      0.017f     // = 2 * PI / 360.f
#define RAD_TO_DEG      57.29f     // = 360.f / (2 * PI)
#define PERIMETER_COEF  6.284f     // = 360.f * RAD_TO_DEG
#define SEC_PER_MIN     60.f


// 用于byte数组和float之间的转换
typedef __packed union
{
    float f;
    uint8_t a[sizeof(float)];
}A2F_u;


// 快速求取平方根
float invSqrt(float x);


// 求圆内最小缺角
float angle_error_calc(float set, float get);

// 将角度限制在[0, 360)
float angle_limit(float current);

float abs_to_angle(float absolute_angle);

#endif
