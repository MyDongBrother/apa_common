#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: abs.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 11-Oct-2018 23:25:44
 */

/* Include Files */

#include "abs.h"

/* Function Definitions */

/*
 * Arguments    : const float x[16]
 *                float y[16]
 * Return Type  : void
 */
void b_abs(const float x[16], float y[16])
{
    int k;
    for (k = 0; k < 16; k++)
    {
        y[k] = fabsf(x[k]);
    }
}

/*
 * File trailer for abs.c
 *
 * [EOF]
 */
