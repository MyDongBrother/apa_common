#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xger.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 11-Oct-2018 23:25:44
 */

/* Include Files */

#include "xger.h"

/* Function Definitions */

/*
 * Arguments    : int m
 *                int n
 *                float alpha1
 *                int ix0
 *                const float y[2]
 *                float A[20]
 *                int ia0
 * Return Type  : void
 */
void b_xger(int m, int n, float alpha1, int ix0, const float y[2], float A[20], int ia0)
{
    int jA;
    int jy;
    int j;
    float temp;
    int ix;
    int i12;
    int ijA;
    if (!(alpha1 == 0.0F))
    {
        jA = ia0 - 1;
        jy = 0;
        for (j = 1; j <= n; j++)
        {
            if (y[jy] != 0.0F)
            {
                temp = y[jy] * alpha1;
                ix   = ix0;
                i12  = m + jA;
                for (ijA = jA; ijA + 1 <= i12; ijA++)
                {
                    A[ijA] += A[ix - 1] * temp;
                    ix++;
                }
            }

            jy++;
            jA += 10;
        }
    }
}

/*
 * Arguments    : int m
 *                int n
 *                float alpha1
 *                int ix0
 *                const float y[4]
 *                float A[48]
 *                int ia0
 * Return Type  : void
 */
void xger(int m, int n, float alpha1, int ix0, const float y[4], float A[48], int ia0)
{
    int jA;
    int jy;
    int j;
    float temp;
    int ix;
    int i9;
    int ijA;
    if (!(alpha1 == 0.0F))
    {
        jA = ia0 - 1;
        jy = 0;
        for (j = 1; j <= n; j++)
        {
            if (y[jy] != 0.0F)
            {
                temp = y[jy] * alpha1;
                ix   = ix0;
                i9   = m + jA;
                for (ijA = jA; ijA + 1 <= i9; ijA++)
                {
                    A[ijA] += A[ix - 1] * temp;
                    ix++;
                }
            }

            jy++;
            jA += 12;
        }
    }
}

/*
 * File trailer for xger.c
 *
 * [EOF]
 */
