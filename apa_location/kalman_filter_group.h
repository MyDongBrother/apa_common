#ifndef KALMAN_FILTER_GROUP_H
#define KALMAN_FILTER_GROUP_H
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "codegenall_types.h"

extern float kf1d_s(float z, float t, float q, float r); //

/*
    1D Kalman Filter for roll
    param_in:
    z:measurement of roll
    t:loop time
    q:process noise covariance
    r:measurement nosie covariance
    return:
    state output
*/
extern float kf1d_roll(float z, float t, float q, float r); //
/*
    1D Kalman Filter for pitch
    param_in:
    z:measurement of pitch
    t:loop time
    q:process noise covariance
    r:measurement nosie covariance
    return:
    state output
*/
extern float kf1d_pitch(float z, float t, float q, float r); //
/*
    3D Kalman Filter for distance, speed and acceleration
    param_in:
    z[2]:measurement of distance and speed
    t:loop time
    q:process noise covariance
    r[2]:measurement nosie covariance(distance, speed)
    param_out:
    y[3]:state output(distance, speed and acceleration)
    return:
    state output
*/

#endif