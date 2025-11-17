#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mean.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 07-Nov-2018 20:40:29
 */

/* Include Files */

#include "mean.h"

/* Function Definitions */

/*
 * Arguments    : const float x[30]
 *                float y[3]
 * Return Type  : void
 */
void mean(const float x[30], float y[3])
{
    int xj;
    int k;
    int xoffset;
    for (xj = 0; xj < 3; xj++)
    {
        y[xj] = x[xj];
    }

    for (k = 0; k < 9; k++)
    {
        xoffset = (k + 1) * 3;
        for (xj = 0; xj < 3; xj++)
        {
            y[xj] += x[xoffset + xj];
        }
    }

    for (xj = 0; xj < 3; xj++)
    {
        y[xj] /= 10.0F;
    }
}

/*
 * File trailer for mean.c
 *
 * [EOF]
 */
