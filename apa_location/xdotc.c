#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xdotc.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

/* Include Files */

#include "xdotc.h"

/* Function Definitions */

/*
 * Arguments    : const float x[4]
 *                const float y[4]
 * Return Type  : float
 */
float xdotc(const float x[4], const float y[4])
{
    float d;
    int ix;
    int iy;
    int k;
    d  = 0.0F;
    ix = 0;
    iy = 2;
    for (k = 0; k < 2; k++)
    {
        d += x[ix] * y[iy];
        ix++;
        iy++;
    }

    return d;
}

/*
 * File trailer for xdotc.c
 *
 * [EOF]
 */
