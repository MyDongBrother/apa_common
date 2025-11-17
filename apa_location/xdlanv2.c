#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xdlanv2.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 11-Oct-2018 23:25:44
 */

/* Include Files */

#include "xdlanv2.h"
#include "SUKF.h"
#include "codegenall_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : float *a
 *                float *b
 *                float *c
 *                float *d
 *                float *rt1r
 *                float *rt1i
 *                float *rt2r
 *                float *rt2i
 *                float *cs
 *                float *sn
 * Return Type  : void
 */
void xdlanv2(float *a, float *b, float *c, float *d, float *rt1r, float *rt1i,
             float *rt2r, float *rt2i, float *cs, float *sn)
{
    float temp;
    float p;
    float bcmax;
    int b_b;
    int b_c;
    float bcmis;
    float scale;
    float z;
    float tau;
    float b_p;
    int b_bcmax;
    if (*c == 0.0F)
    {
        *cs = 1.0F;
        *sn = 0.0F;
    }
    else if (*b == 0.0F)
    {
        *cs  = 0.0F;
        *sn  = 1.0F;
        temp = *d;
        *d   = *a;
        *a   = temp;
        *b   = -*c;
        *c   = 0.0F;
    }
    else if ((*a - *d == 0.0F) && ((*b < 0.0F) != (*c < 0.0F)))
    {
        *cs = 1.0F;
        *sn = 0.0F;
    }
    else
    {
        temp  = *a - *d;
        p     = 0.5F * temp;
        bcmax = fmaxf(fabsf(*b), fabsf(*c));
        if (!(*b < 0.0F))
        {
            b_b = 1;
        }
        else
        {
            b_b = -1;
        }

        if (!(*c < 0.0F))
        {
            b_c = 1;
        }
        else
        {
            b_c = -1;
        }

        bcmis = fminf(fabsf(*b), fabsf(*c)) * (float)b_b * (float)b_c;
        scale = fmaxf(fabsf(p), bcmax);
        z     = p / scale * p + bcmax / scale * bcmis;
        if (z >= 8.8817842E-16F)
        {
            *a = sqrtf(scale) * sqrtf(z);
            if (!(p < 0.0F))
            {
                b_p = *a;
            }
            else
            {
                b_p = -*a;
            }

            z  = p + b_p;
            *a = *d + z;
            *d -= bcmax / z * bcmis;
            tau = rt_hypotf(*c, z);
            *cs = z / tau;
            *sn = *c / tau;
            *b -= *c;
            *c = 0.0F;
        }
        else
        {
            bcmax = *b + *c;
            tau   = rt_hypotf(bcmax, temp);
            *cs   = sqrtf(0.5F * (1.0F + fabsf(bcmax) / tau));
            if (!(bcmax < 0.0F))
            {
                b_bcmax = 1;
            }
            else
            {
                b_bcmax = -1;
            }

            *sn   = -(p / (tau * *cs)) * (float)b_bcmax;
            z     = *a * *cs + *b * *sn;
            bcmis = -*a * *sn + *b * *cs;
            scale = *c * *cs + *d * *sn;
            bcmax = -*c * *sn + *d * *cs;
            *b    = bcmis * *cs + bcmax * *sn;
            *c    = -z * *sn + scale * *cs;
            temp  = 0.5F * ((z * *cs + scale * *sn) + (-bcmis * *sn + bcmax * *cs));
            *a    = temp;
            *d    = temp;
            if (*c != 0.0F)
            {
                if (*b != 0.0F)
                {
                    if ((*b < 0.0F) == (*c < 0.0F))
                    {
                        bcmax = sqrtf(fabsf(*b));
                        scale = sqrtf(fabsf(*c));
                        *a    = bcmax * scale;
                        if (!(*c < 0.0F))
                        {
                            p = *a;
                        }
                        else
                        {
                            p = -*a;
                        }

                        tau = 1.0F / sqrtf(fabsf(*b + *c));
                        *a  = temp + p;
                        *d  = temp - p;
                        *b -= *c;
                        *c    = 0.0F;
                        bcmis = bcmax * tau;
                        bcmax = scale * tau;
                        temp  = *cs * bcmis - *sn * bcmax;
                        *sn   = *cs * bcmax + *sn * bcmis;
                        *cs   = temp;
                    }
                }
                else
                {
                    *b   = -*c;
                    *c   = 0.0F;
                    temp = *cs;
                    *cs  = -*sn;
                    *sn  = temp;
                }
            }
        }
    }

    *rt1r = *a;
    *rt2r = *d;
    if (*c == 0.0F)
    {
        *rt1i = 0.0F;
        *rt2i = 0.0F;
    }
    else
    {
        *rt1i = sqrtf(fabsf(*b)) * sqrtf(fabsf(*c));
        *rt2i = -*rt1i;
    }
}

/*
 * File trailer for xdlanv2.c
 *
 * [EOF]
 */
