#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sum.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

/* Include Files */

#include "sum.h"

/* Function Definitions */

/*
 * Arguments    : const float x[4]
 * Return Type  : float
 */
float sum(const float x[4])
{
    float y;
    int k;
    y = x[0];
    for (k = 0; k < 3; k++)
    {
        y += x[k + 1];
    }

    return y;
}

/*
 * File trailer for sum.c
 *
 * [EOF]
 */
