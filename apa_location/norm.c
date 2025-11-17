#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: norm.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

/* Include Files */

#include "norm.h"

/* Function Definitions */

/*
 * Arguments    : const float x[3]
 * Return Type  : float
 */
float b_norm(const float x[3])
{
    float y;
    float scale;
    int k;
    float absxk;
    float t;
    y     = 0.0F;
    scale = 1.29246971E-26F;
    for (k = 0; k < 3; k++)
    {
        absxk = fabsf(x[k]);
        if (absxk > scale)
        {
            t     = scale / absxk;
            y     = 1.0F + y * t * t;
            scale = absxk;
        }
        else
        {
            t = absxk / scale;
            y += t * t;
        }
    }

    return scale * sqrtf(y);
}

/*
 * Arguments    : const float x[2]
 * Return Type  : float
 */
float norm(const float x[2])
{
    float y;
    float scale;
    int k;
    float absxk;
    float t;
    y     = 0.0F;
    scale = 1.29246971E-26F;
    for (k = 0; k < 2; k++)
    {
        absxk = fabsf(x[k]);
        if (absxk > scale)
        {
            t     = scale / absxk;
            y     = 1.0F + y * t * t;
            scale = absxk;
        }
        else
        {
            t = absxk / scale;
            y += t * t;
        }
    }

    return scale * sqrtf(y);
}

/*
 * File trailer for norm.c
 *
 * [EOF]
 */
