#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: svd.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

/* Include Files */

#include "svd.h"
#include "xrotg.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"

/* Function Definitions */

/*
 * Arguments    : const float A[4]
 *                float U[2]
 * Return Type  : void
 */
void svd(const float A[4], float U[2])
{
    int kase;
    boolean_T apply_transform;
    float b_A[4];
    float nrm;
    float s[2];
    int m;
    float ztest;
    float e[2];
    int q;
    int iter;
    float snorm;
    float rt;
    boolean_T exitg1;
    int qs;
    float f;
    float varargin_1[5];
    float mtmp;
    float sqds;
    for (kase = 0; kase < 4; kase++)
    {
        b_A[kase] = A[kase];
    }

    apply_transform = false;
    nrm             = e_xnrm2(A);
    if (nrm > 0.0F)
    {
        apply_transform = true;
        if (A[0] < 0.0F)
        {
            nrm = -nrm;
        }

        if (fabsf(nrm) >= 9.86076132E-32F)
        {
            ztest = 1.0F / nrm;
            for (kase = 0; kase < 2; kase++)
            {
                b_A[kase] *= ztest;
            }
        }
        else
        {
            for (kase = 0; kase < 2; kase++)
            {
                b_A[kase] /= nrm;
            }
        }

        b_A[0]++;
        s[0] = -nrm;
    }
    else
    {
        s[0] = 0.0F;
    }

    if (apply_transform)
    {
        xaxpy(-(xdotc(b_A, b_A) / b_A[0]), b_A);
    }

    m     = 0;
    s[1]  = b_A[3];
    e[0]  = b_A[2];
    e[1]  = 0.0F;
    ztest = b_A[2];
    for (q = 0; q < 2; q++)
    {
        if (s[q] != 0.0F)
        {
            rt   = fabsf(s[q]);
            nrm  = s[q] / rt;
            s[q] = rt;
            if (q + 1 < 2)
            {
                ztest /= nrm;
            }
        }

        if ((q + 1 < 2) && (ztest != 0.0F))
        {
            rt    = fabsf(ztest);
            nrm   = ztest;
            ztest = rt;
            s[1] *= rt / nrm;
        }

        e[0] = ztest;
    }

    iter  = 0;
    snorm = 0.0F;
    for (kase = 0; kase < 2; kase++)
    {
        snorm = fmaxf(snorm, fmaxf(fabsf(s[kase]), fabsf(e[kase])));
    }

    while ((m + 2 > 0) && (!(iter >= 75)))
    {
        q      = m + 1;
        exitg1 = false;
        while (!(exitg1 || (q == 0)))
        {
            nrm = fabsf(e[0]);
            if ((nrm <= 1.1920929E-7F * (fabsf(s[0]) + fabsf(s[1]))) ||
                (nrm <= 9.86076132E-32F) ||
                ((iter > 20) && (nrm <= 1.1920929E-7F * snorm)))
            {
                e[0]   = 0.0F;
                exitg1 = true;
            }
            else
            {
                q = 0;
            }
        }

        if (q == m + 1)
        {
            kase = 4;
        }
        else
        {
            qs     = m + 2;
            kase   = m + 2;
            exitg1 = false;
            while ((!exitg1) && (kase >= q))
            {
                qs = kase;
                if (kase == q)
                {
                    exitg1 = true;
                }
                else
                {
                    nrm = 0.0F;
                    if (kase < m + 2)
                    {
                        nrm = fabsf(e[kase - 1]);
                    }

                    if (kase > q + 1)
                    {
                        nrm += fabsf(e[0]);
                    }

                    ztest = fabsf(s[kase - 1]);
                    if ((ztest <= 1.1920929E-7F * nrm) || (ztest <= 9.86076132E-32F))
                    {
                        s[kase - 1] = 0.0F;
                        exitg1      = true;
                    }
                    else
                    {
                        kase--;
                    }
                }
            }

            if (qs == q)
            {
                kase = 3;
            }
            else if (qs == m + 2)
            {
                kase = 1;
            }
            else
            {
                kase = 2;
                q    = qs;
            }
        }

        switch (kase)
        {
            case 1:
                f    = e[m];
                e[m] = 0.0F;
                kase = m + 1;
                while (kase >= q + 1)
                {
                    xrotg(&s[0], &f, &nrm, &ztest);
                    kase = 0;
                }
                break;

            case 2:
                f        = e[q - 1];
                e[q - 1] = 0.0F;
                while (q + 1 <= m + 2)
                {
                    xrotg(&s[q], &f, &nrm, &ztest);
                    f = -ztest * e[q];
                    e[q] *= nrm;
                    q++;
                }
                break;

            case 3:
                varargin_1[0] = fabsf(s[m + 1]);
                varargin_1[1] = fabsf(s[m]);
                varargin_1[2] = fabsf(e[m]);
                varargin_1[3] = fabsf(s[q]);
                varargin_1[4] = fabsf(e[q]);
                mtmp          = varargin_1[0];
                for (kase = 0; kase < 4; kase++)
                {
                    if (varargin_1[kase + 1] > mtmp)
                    {
                        mtmp = varargin_1[kase + 1];
                    }
                }

                f     = s[m + 1] / mtmp;
                nrm   = s[m] / mtmp;
                ztest = e[m] / mtmp;
                sqds  = s[q] / mtmp;
                rt    = ((nrm + f) * (nrm - f) + ztest * ztest) / 2.0F;
                nrm   = f * ztest;
                nrm *= nrm;
                if ((rt != 0.0F) || (nrm != 0.0F))
                {
                    ztest = sqrtf(rt * rt + nrm);
                    if (rt < 0.0F)
                    {
                        ztest = -ztest;
                    }

                    ztest = nrm / (rt + ztest);
                }
                else
                {
                    ztest = 0.0F;
                }

                f  = (sqds + f) * (sqds - f) + ztest;
                rt = sqds * (e[q] / mtmp);
                while (q + 1 <= m + 1)
                {
                    xrotg(&f, &rt, &nrm, &ztest);
                    f    = nrm * s[0] + ztest * e[0];
                    e[0] = nrm * e[0] - ztest * s[0];
                    rt   = ztest * s[1];
                    s[1] *= nrm;
                    s[0] = f;
                    xrotg(&s[0], &rt, &nrm, &ztest);
                    f    = nrm * e[0] + ztest * s[1];
                    s[1] = -ztest * e[0] + nrm * s[1];
                    rt   = ztest * e[1];
                    e[1] *= nrm;
                    q = 1;
                }

                e[m] = f;
                iter++;
                break;

            default:
                if (s[q] < 0.0F)
                {
                    s[q] = -s[q];
                }

                kase = q + 1;
                while ((q + 1 < 2) && (s[q] < s[kase]))
                {
                    rt      = s[q];
                    s[q]    = s[kase];
                    s[kase] = rt;
                    q       = kase;
                    kase++;
                }

                iter = 0;
                m--;
                break;
        }
    }

    for (kase = 0; kase < 2; kase++)
    {
        U[kase] = s[kase];
    }
}

/*
 * File trailer for svd.c
 *
 * [EOF]
 */
