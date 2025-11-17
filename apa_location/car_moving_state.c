#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: car_moving_state.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 02-Jun-2018 14:00:30
 */

/* Include Files */

#include "car_moving_state.h"

/* Variable Definitions */
static unsigned char CarStopCounter;

/* Function Definitions */

/*
 * function car_moving = car_moving_state(dS)
 * Arguments    : float dS
 * Return Type  : unsigned char
 */
unsigned char car_moving_state(float dS, float CurSpd_Motor)
{
    unsigned char car_moving;

    /* 'car_moving_state:2' coder.inline('never') */
    /* 'car_moving_state:6' if isempty(CarStopCounter) */
    /* 'car_moving_state:10' if (abs(dS) < 1e-7) */
    if (fabsf(dS) < 1.0E-7F)
    {
        /* 'car_moving_state:11' if(CarStopCounter<5) */
        if (CarStopCounter < 20)
        {
            /* 5 times is 100ms, CarStopCounter==5mean car stops */
            /* 'car_moving_state:12' CarStopCounter = CarStopCounter+uint8(1); */
            CarStopCounter++;
        }
    }
    else
    {
        /* 'car_moving_state:14' else */
        /* 'car_moving_state:15' CarStopCounter = uint8(0); */
        CarStopCounter = 0;
    }

    /* 'car_moving_state:18' if(CarStopCounter==uint8(5)) */
    if (CarStopCounter == 20)
    {
        /* 'car_moving_state:19' car_moving = uint8(0); */
        car_moving = 0;
    }
    else
    {
        /* 'car_moving_state:20' else */
        /* 'car_moving_state:21' car_moving = uint8(1); */
        car_moving = 1;
    }
    if (fabsf(CurSpd_Motor) < 0.02 && car_moving == 0) // 0.02m/s
    {
        car_moving = 0;
    }
    else
    {
        car_moving = 1;
    }

    return car_moving;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void car_moving_state_init(void)
{
    /* 'car_moving_state:7' CarStopCounter = uint8(0); */
    CarStopCounter = 0;
}

/*
 * File trailer for car_moving_state.c
 *
 * [EOF]
 */
