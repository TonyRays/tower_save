#include "common.h"

// 快速求取平方根
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float angle_error_calc(float set, float get)
{
    // 误差角
    float error = set - get;
		return angle_limit(error);
}

// 限制角度在(-180, 180]范围内
float angle_limit(float current)
{
    if (current >= 180.f)
        return (current - 360.f);
    else if (current < -180.f)
        return (current + 360.f);
    else
        return current;
}

float abs_to_angle(float absolute_angle){
	if(absolute_angle > 0)
		return (((int32_t)absolute_angle % 360) >  180) ? (((int32_t)absolute_angle % 360) - 360) : ((int32_t)absolute_angle % 360);
	else
		return (((int32_t)absolute_angle % 360) < -180) ? (((int32_t)absolute_angle % 360) + 360) : ((int32_t)absolute_angle % 360);
}
