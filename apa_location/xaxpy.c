#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xaxpy.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

/* Include Files */

#include "xaxpy.h"

/* Function Definitions */

/*
 * Arguments    : float a
 *                float y[4]
 * Return Type  : void
 */
void xaxpy(float a, float y[4])
{
    int ix;
    int iy;
    int k;
    if (!(a == 0.0F))
    {
        ix = 0;
        iy = 2;
        for (k = 0; k < 2; k++)
        {
            y[iy] += a * y[ix];
            ix++;
            iy++;
        }
    }
}

/*
 * File trailer for xaxpy.c
 *
 * [EOF]
 */
