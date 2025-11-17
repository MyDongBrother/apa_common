/**
 * @file LC_fcn.c
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-09-02
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-09-02 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#pragma section all "CPU1.Private"
#include "PK_LocationImpl.h"
#include "PK_Calibration.h"
#include "kalman_filter_group.h"
#include "mass_estimate.h"

/********************************************************************
NAME: DeltaWheelCounter
FUNC: wheel counter difference to last count
PARA_IN:
    count_last
    count_cur
PARA_OUT:
    void
RETURN:
    delta count
********************************************************************/
int DeltaWheelCounter(int count_cur, int count_last)
{
    static const int MAX_COUNTER_RANGE =
        1022;                           // zhouzj  4095 -> 1022 ;//counter range 1-4095
    static const int UPPER_LIMIT = 200; // dcounter range 0-200
    int dcount                   = 0;
    dcount = DROUND_COUNTER(count_cur, count_last, MAX_COUNTER_RANGE);
    if (abs(dcount) > UPPER_LIMIT)
    {
        dcount = 0;
    }
    return dcount;
}

uint8 hold_EKF_LC_reset(uint8 error_state) // hold for 5 times
{
    static uint8 counter         = 0;
    static uint8 state           = 0; // 0 no error, 1 error
    static uint8 error_state_pre = 0;

    if (error_state > 0)
    {
        error_state_pre = error_state;
        counter         = 1;
    }

    if (counter >= 1)
    {
        state = error_state_pre;
        counter++;
    }
    else
    {
        state = error_state;
    }

    if (counter > 5)
    {
        counter = 0;
    }

    return state;
}

/*cout continuous error. output error only if counter exceeds limits */
void count_error_signal(Signal_diagnose_info *error_state_in)
{

    if (1 == error_state_in->cur_state)
    {
        error_state_in->state_counter++;
    }
    else
    {
        error_state_in->state_counter = 0;
    }

    if (error_state_in->state_counter >= error_state_in->window_size) // 100
    {
        error_state_in->state_counter = error_state_in->window_size;
        error_state_in->state         = 1;
    }
    else
    {
        error_state_in->state = 0;
    }
}

// if gear change done
uint8 Is_gear_done(uint8 gear_state)
{
    static uint8 counter        = 0;
    uint8 state                 = 0;
    static uint8 gear_state_pre = 0;
    const int couter_limit      = (int)(0.5 / LocatingPeriod_dt); // 500ms

    if (gear_state_pre == gear_state)
    {
        counter++;
    }
    else
    {
        counter = 0;
    }

    if (counter >= couter_limit) // 1000ms
    {
        counter = couter_limit;
        state   = 1;
    }

    gear_state_pre = gear_state;

    return state;
}

// Identify if car is moving or not by Motorspd
int car_moving_stateByMotor(int Motorspd)
{
    const uint8 Max_counter_delay  = 10;
    static int directed_car_moving = 0;
    static int Motorspd_buffer[10];
    static unsigned char idx           = 0;
    static unsigned char counter_delay = 0;
    static int sum_motorspd, mean_motorspd;
    sum_motorspd         = sum_motorspd - Motorspd_buffer[idx] + Motorspd;
    Motorspd_buffer[idx] = Motorspd;
    mean_motorspd        = sum_motorspd / 10.0f;

    if (mean_motorspd > 10)
    {
        directed_car_moving = 1; // forward
    }
    else if (mean_motorspd < -10)
    {
        directed_car_moving = -1; // backward
    }
    else
    {
        directed_car_moving = 0; // stop
        counter_delay       = 0;
    }

    idx++;

    if (idx == 10)
    {
        idx = 0;
    }

    if (counter_delay < Max_counter_delay)
    {
        counter_delay++;
        directed_car_moving = 0;
    }

    return directed_car_moving;
}

// Identify if car is moving or not
unsigned char car_moving_stateByDcounter(int dCounter_RL, int dCounter_RR)
{
    //
    static unsigned char CarStopCounter = 0;
    static unsigned char car_moving     = 1;
    static int max_count                = 0;
    max_count                           = (int)(1.0 / LocatingPeriod_dt);

    if ((dCounter_RL + dCounter_RR) == 0)
    {
        if (CarStopCounter <
            max_count) // 5 times is 100ms, CarStopCounter==5mean car stops
        {
            CarStopCounter++;
        }
    }
    else
    {
        CarStopCounter = 0;
    }

    if (CarStopCounter == max_count)
    {
        car_moving = 0;
    }
    else
    {
        car_moving = 1;
    }

    return car_moving;
}

float cal_curvature(const float AngRate_Z, const float speed, const uint8 stateFlg)
{
    static float AngRate_Z_Smooth = 0.0f;
    float curvature               = 0.0f;
    const float MAX_curvature     = 0.4f;
    AngRate_Z_Smooth              = AngRate_Z_Smooth * 0.9f + AngRate_Z * 0.1f;

    if (fabsf(speed) > 0.01f && 0 == stateFlg)
    {
        curvature = AngRate_Z_Smooth / speed;
    }
    else
    {
        curvature = 0.0f;
    }

    if (curvature > MAX_curvature)
    {
        curvature = MAX_curvature;
    }

    if (curvature < (-MAX_curvature))
    {
        curvature = (-MAX_curvature);
    }

    return curvature;
}

uint8 mass_level_function(float vel_speed_motor, float Accel_X, float Vehicle_Pitch,
                          float MotorTorq, float mass_estimate_res[2])
{
    static unsigned char mass_level = 1;
    static float Mass0              = 1600.0f;
    static float MotorTorq_pre      = 0.0f;
    static uint8 mass_estimate_reset;
    const float MASS_SHRESHOLD = 1800.0F;
    // static float mass_estimate_res[2] = {1600.0f,10.0f};

    if ((fabsf(vel_speed_motor) < 5.0f) && (fabsf(vel_speed_motor) > 1.0f) &&
        (MotorTorq > 40.0F) && (fabsf(Vehicle_Pitch) < 1.5f))
    {
        // 0.32f*1550.0f*9.8f*0.014f
        // kf3d_res_cpp=((7.9f/640.0f*MotorTorq - Accel_X)/(0.32f/640.0f*9.8f*0.014f));
        // Mass0 = (int)((7.9f/640.0f*MotorTorq - Accel_X)/(0.32f/640.0f*9.8f*0.014f));
        // Mass0 = (7.9f*(MotorTorq-8.0F))/0.32f/Accel_X;
        // kf3d_res_cpp = (7.9f*(MotorTorq-8.0F))/0.32f/Accel_X;

        mass_estimate_reset = 1;
        if (MotorTorq - MotorTorq_pre > 0.0f)
        {
            mass_estimate_reset = 0;
        }

        mass_estimate(7.9f, 0.32f, Accel_X, Vehicle_Pitch * 0.01745329251f * 1.0f,
                      MotorTorq, mass_estimate_reset, mass_estimate_res);
        // kf3d_res_cpp = mass_estimate_res[0];//mass
        // Mass0 = ((7.9f * (MotorTorq - 8.0F)) / 0.32f + 1.0f * Mass0 * 9.8f *
        // sinf(Vehicle_Pitch * 0.01745329251f)) / Accel_X;
        Mass0 = mass_estimate_res[0];
        // Mass0 = kf1d_s(Mass0, 0.01f, 20, 100);
    }
    // get mass level from 1 to 3, means low weight, medium weight, high weight
    if (Mass0 < MASS_SHRESHOLD) // low weight
    {
        mass_level = 1;
    }
    else if (Mass0 >= MASS_SHRESHOLD) // medium weight
    {
        mass_level = 2;
    }

    MotorTorq_pre = MotorTorq;

    return mass_level;
}
