#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: diag.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 11-Oct-2018 23:25:44
 */

/* Include Files */

#include "diag.h"

/* Function Definitions */

/*
 * Arguments    : const float v[2]
 *                float d[4]
 * Return Type  : void
 */
void diag(const float v[2], float d[4])
{
    int j;
    for (j = 0; j < 4; j++)
    {
        d[j] = 0.0F;
    }

    for (j = 0; j < 2; j++)
    {
        d[j + (j << 1)] = v[j];
    }
}

/*
 * File trailer for diag.c
 *
 * [EOF]
 */
