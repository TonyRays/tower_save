/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file pid.h
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief pid parameter initialization, position and delta pid calculate
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
    
#ifndef __PID_H__
#define __PID_H__

#include "stm32f4xx.h"

enum
{
    LLAST = 0,
    LAST,
    NOW,
    POSITION_PID,
    DELTA_PID,
    COMMON_ERR,
    ANGLE_ERR,
};

typedef struct pid_t
{
    float p;
    float i;
    float d;

    float set;
    float get;
    float err[3];

    float pout;
    float iout;
    float dout;
    float out;

    float input_max_err;    //input max err;
    float output_deadband;  //output deadband; 

    uint32_t err_mode;
    uint32_t pid_mode;
    uint32_t max_out;
    uint32_t integral_limit;

    void (*f_param_init)(struct pid_t *pid, 
                       uint32_t      pid_mode,
                       uint32_t      err_mode,
                       uint32_t      max_output,
                       uint32_t      inte_limit,
                       float         p,
                       float         i,
                       float         d);
    void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);
 
} pid_t;


#define PID_PARAM_DEFAULT \
{\
  0,\
  0,\
  0,\
  0,\
  0,\
  {0,0,0},\
  0,\
  0,\
  0,\
  0,\
  0,\
  0,\
}\

typedef struct
{
  float p;
  float i;
  float d;

  float set;
  float get;
  float err[3]; //error

  float pout; 
  float iout; 
  float dout; 
  float out;

  float input_max_err;    //input max err;
  float output_deadband;  //output deadband; 

  float p_far;
  float p_near;
  float grade_range;
  
  uint32_t pid_mode;
  uint32_t max_out;
  uint32_t integral_limit;

  void (*f_param_init)(struct pid_t *pid, 
                       uint32_t      pid_mode,
                       uint32_t      max_output,
                       uint32_t      inte_limit,
                       float         p,
                       float         i,
                       float         d);
  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);
 
} grade_pid_t;

void PID_struct_init(
    pid_t*   pid,
    uint32_t pid_mode,
    uint32_t err_mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd);

float pid_calc(pid_t *pid, float fdb, float ref);
    
extern float angle_error_calc(float set, float get);
    
extern pid_t pid_yaw_position;
extern pid_t pid_pitch_position;
extern pid_t pid_yaw_speed;
extern pid_t pid_pitch_speed;
extern pid_t pid_wheel_speed[4];
extern pid_t pid_chassis_angle;
extern pid_t pid_feed_position;
extern pid_t pid_feed_speed;
extern pid_t pid_friction_speed[2];

#endif
