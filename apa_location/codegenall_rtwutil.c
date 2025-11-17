#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: codegenall_rtwutil.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 11-Oct-2018 23:25:44
 */

/* Include Files */

#include "codegenall_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
float rt_hypotf(float u0, float u1)
{
    float y;
    float a;
    float b;
    a = fabsf(u0);
    b = fabsf(u1);
    if (a < b)
    {
        a /= b;
        y = b * sqrtf(a * a + 1.0F);
    }
    else if (a > b)
    {
        b /= a;
        y = a * sqrtf(b * b + 1.0F);
    }
    else
    {
        y = a * 1.41421354F;
    }

    return y;
}

/*
 * File trailer for codegenall_rtwutil.c
 *
 * [EOF]
 */
