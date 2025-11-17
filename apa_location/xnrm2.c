#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xnrm2.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

/* Include Files */

#include "xnrm2.h"

/* Function Definitions */

/*
 * Arguments    : int n
 *                const float x[20]
 *                int ix0
 * Return Type  : float
 */
float b_xnrm2(int n, const float x[20], int ix0)
{
    float y;
    float scale;
    int kend;
    int k;
    float absxk;
    float t;
    y = 0.0F;
    if (!(n < 1))
    {
        if (n == 1)
        {
            y = fabsf(x[ix0 - 1]);
        }
        else
        {
            scale = 1.29246971E-26F;
            kend  = (ix0 + n) - 1;
            for (k = ix0; k <= kend; k++)
            {
                absxk = fabsf(x[k - 1]);
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

            y = scale * sqrtf(y);
        }
    }

    return y;
}

/*
 * Arguments    : int n
 *                const float x[16]
 *                int ix0
 * Return Type  : float
 */
float c_xnrm2(int n, const float x[16], int ix0)
{
    float y;
    float scale;
    int kend;
    int k;
    float absxk;
    float t;
    y = 0.0F;
    if (!(n < 1))
    {
        if (n == 1)
        {
            y = fabsf(x[ix0 - 1]);
        }
        else
        {
            scale = 1.29246971E-26F;
            kend  = (ix0 + n) - 1;
            for (k = ix0; k <= kend; k++)
            {
                absxk = fabsf(x[k - 1]);
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

            y = scale * sqrtf(y);
        }
    }

    return y;
}

/*
 * Arguments    : int n
 *                const float x[3]
 * Return Type  : float
 */
float d_xnrm2(int n, const float x[3])
{
    float y;
    float scale;
    int k;
    float absxk;
    float t;
    y = 0.0F;
    if (!(n < 1))
    {
        if (n == 1)
        {
            y = fabsf(x[1]);
        }
        else
        {
            scale = 1.29246971E-26F;
            for (k = 2; k <= n + 1; k++)
            {
                absxk = fabsf(x[k - 1]);
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

            y = scale * sqrtf(y);
        }
    }

    return y;
}

/*
 * Arguments    : const float x[4]
 * Return Type  : float
 */
float e_xnrm2(const float x[4])
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
 * Arguments    : int n
 *                const float x[48]
 *                int ix0
 * Return Type  : float
 */
float xnrm2(int n, const float x[48], int ix0)
{
    float y;
    float scale;
    int kend;
    int k;
    float absxk;
    float t;
    y = 0.0F;
    if (!(n < 1))
    {
        if (n == 1)
        {
            y = fabsf(x[ix0 - 1]);
        }
        else
        {
            scale = 1.29246971E-26F;
            kend  = (ix0 + n) - 1;
            for (k = ix0; k <= kend; k++)
            {
                absxk = fabsf(x[k - 1]);
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

            y = scale * sqrtf(y);
        }
    }

    return y;
}

/*
 * File trailer for xnrm2.c
 *
 * [EOF]
 */
