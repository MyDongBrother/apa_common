#pragma section all "CPU1.Private"

/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: power.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-May-2018 09:49:16
 */

/* Include Files */

#include "powerf.h"

/* Function Definitions */

/*
 * Arguments    : const float a[2]
 *                float y[2]
 * Return Type  : void
 */
void power(const float a[2], float y[2])
{
    int k;
    for (k = 0; k < 2; k++)
    {
        y[k] = a[k] * a[k];
    }
}

/*
 * File trailer for power.c
 *
 * [EOF]
 */
