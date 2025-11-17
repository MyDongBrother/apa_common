#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xdhseqr.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 11-Oct-2018 23:25:44
 */

/* Include Files */

#include "xdhseqr.h"
#include "xdlanv2.h"
#include "xzlarfg.h"
#include "sqrt.h"

/* Function Definitions */

/*
 * Arguments    : float h[16]
 * Return Type  : int
 */
int eml_dlahqr(float h[16])
{
    int info;
    int i;
    float v[3];
    boolean_T exitg1;
    int L;
    boolean_T goto150;
    int its;
    boolean_T exitg2;
    int k;
    boolean_T exitg3;
    float tst;
    float htmp1;
    float ab;
    float ba;
    float aa;
    float rt2r;
    float rt1r;
    float s;
    float sn;
    int nr;
    int hoffset;
    int m;
    int b_k;
    int j;
    info = 0;
    for (i = 0; i < 3; i++)
    {
        v[i] = 0.0F;
    }

    h[2]   = 0.0F;
    h[3]   = 0.0F;
    h[7]   = 0.0F;
    i      = 3;
    exitg1 = false;
    while ((!exitg1) && (i + 1 >= 1))
    {
        L       = 1;
        goto150 = false;
        its     = 0;
        exitg2  = false;
        while ((!exitg2) && (its < 301))
        {
            k      = i;
            exitg3 = false;
            while ((!exitg3) &&
                   ((k + 1 > L) && (!(fabsf(h[k + ((k - 1) << 2)]) <= 3.94430453E-31F))))
            {
                tst = fabsf(h[(k + ((k - 1) << 2)) - 1]) + fabsf(h[k + (k << 2)]);
                if (tst == 0.0F)
                {
                    if (k - 1 >= 1)
                    {
                        tst = fabsf(h[(k + ((k - 2) << 2)) - 1]);
                    }

                    if (k + 2 <= 4)
                    {
                        tst += fabsf(h[(k + (k << 2)) + 1]);
                    }
                }

                if (fabsf(h[k + ((k - 1) << 2)]) <= 1.1920929E-7F * tst)
                {
                    htmp1 = fabsf(h[k + ((k - 1) << 2)]);
                    tst   = fabsf(h[(k + (k << 2)) - 1]);
                    if (htmp1 > tst)
                    {
                        ab = htmp1;
                        ba = tst;
                    }
                    else
                    {
                        ab = tst;
                        ba = htmp1;
                    }

                    htmp1 = fabsf(h[k + (k << 2)]);
                    tst   = fabsf(h[(k + ((k - 1) << 2)) - 1] - h[k + (k << 2)]);
                    if (htmp1 > tst)
                    {
                        aa    = htmp1;
                        htmp1 = tst;
                    }
                    else
                    {
                        aa = tst;
                    }

                    s = aa + ab;
                    if (ba * (ab / s) <=
                        fmaxf(3.94430453E-31F, 1.1920929E-7F * (htmp1 * (aa / s))))
                    {
                        exitg3 = true;
                    }
                    else
                    {
                        k--;
                    }
                }
                else
                {
                    k--;
                }
            }

            L = k + 1;
            if (k + 1 > 1)
            {
                h[k + ((k - 1) << 2)] = 0.0F;
            }

            if (k + 1 >= i)
            {
                goto150 = true;
                exitg2  = true;
            }
            else
            {
                if (its == 10)
                {
                    s = fabsf(h[(k + (k << 2)) + 1]) + fabsf(h[(k + ((k + 1) << 2)) + 2]);
                    tst   = 0.75F * s + h[k + (k << 2)];
                    ab    = -0.4375F * s;
                    htmp1 = s;
                    aa    = tst;
                }
                else if (its == 20)
                {
                    s = fabsf(h[i + ((i - 1) << 2)]) + fabsf(h[(i + ((i - 2) << 2)) - 1]);
                    tst   = 0.75F * s + h[i + (i << 2)];
                    ab    = -0.4375F * s;
                    htmp1 = s;
                    aa    = tst;
                }
                else
                {
                    tst   = h[(i + ((i - 1) << 2)) - 1];
                    htmp1 = h[i + ((i - 1) << 2)];
                    ab    = h[(i + (i << 2)) - 1];
                    aa    = h[i + (i << 2)];
                }

                s = ((fabsf(tst) + fabsf(ab)) + fabsf(htmp1)) + fabsf(aa);
                if (s == 0.0F)
                {
                    rt1r = 0.0F;
                    ba   = 0.0F;
                    rt2r = 0.0F;
                    aa   = 0.0F;
                }
                else
                {
                    tst /= s;
                    htmp1 /= s;
                    ab /= s;
                    aa /= s;
                    ba    = (tst + aa) / 2.0F;
                    tst   = (tst - ba) * (aa - ba) - ab * htmp1;
                    htmp1 = fabsf(tst);
                    b_sqrt(&htmp1);
                    if (tst >= 0.0F)
                    {
                        rt1r = ba * s;
                        rt2r = rt1r;
                        ba   = htmp1 * s;
                        aa   = -ba;
                    }
                    else
                    {
                        rt1r = ba + htmp1;
                        rt2r = ba - htmp1;
                        if (fabsf(rt1r - aa) <= fabsf(rt2r - aa))
                        {
                            rt1r *= s;
                            rt2r = rt1r;
                        }
                        else
                        {
                            rt2r *= s;
                            rt1r = rt2r;
                        }

                        ba = 0.0F;
                        aa = 0.0F;
                    }
                }

                m      = i - 2;
                exitg3 = false;
                while ((!exitg3) && (m + 1 >= k + 1))
                {
                    s = (fabsf(h[m + (m << 2)] - rt2r) + fabsf(aa)) +
                        fabsf(h[(m + (m << 2)) + 1]);
                    tst  = h[(m + (m << 2)) + 1] / s;
                    v[0] = (tst * h[m + ((m + 1) << 2)] +
                            (h[m + (m << 2)] - rt1r) * ((h[m + (m << 2)] - rt2r) / s)) -
                           ba * (aa / s);
                    v[1] =
                        tst *
                        (((h[m + (m << 2)] + h[(m + ((m + 1) << 2)) + 1]) - rt1r) - rt2r);
                    v[2] = tst * h[(m + ((m + 1) << 2)) + 2];
                    s    = (fabsf(v[0]) + fabsf(v[1])) + fabsf(v[2]);
                    tst  = v[0] / s;
                    v[0] /= s;
                    htmp1 = v[1] / s;
                    v[1] /= s;
                    ab = v[2] / s;
                    v[2] /= s;
                    if ((m + 1 == k + 1) ||
                        (fabsf(h[m + ((m - 1) << 2)]) * (fabsf(htmp1) + fabsf(ab)) <=
                         1.1920929E-7F * fabsf(tst) *
                             ((fabsf(h[(m + ((m - 1) << 2)) - 1]) +
                               fabsf(h[m + (m << 2)])) +
                              fabsf(h[(m + ((m + 1) << 2)) + 1]))))
                    {
                        exitg3 = true;
                    }
                    else
                    {
                        m--;
                    }
                }

                for (b_k = m; b_k + 1 <= i; b_k++)
                {
                    nr = (i - b_k) + 1;
                    if (3 < nr)
                    {
                        nr = 3;
                    }

                    if (b_k + 1 > m + 1)
                    {
                        hoffset = b_k + ((b_k - 1) << 2);
                        for (j = 1; j <= nr; j++)
                        {
                            v[j - 1] = h[(j + hoffset) - 1];
                        }
                    }

                    tst  = v[0];
                    rt2r = xzlarfg(nr, &tst, v);
                    v[0] = tst;
                    if (b_k + 1 > m + 1)
                    {
                        h[b_k + ((b_k - 1) << 2)]       = tst;
                        h[(b_k + ((b_k - 1) << 2)) + 1] = 0.0F;
                        if (b_k + 1 < i)
                        {
                            h[(b_k + ((b_k - 1) << 2)) + 2] = 0.0F;
                        }
                    }
                    else
                    {
                        if (m + 1 > k + 1)
                        {
                            h[b_k + ((b_k - 1) << 2)] *= 1.0F - rt2r;
                        }
                    }

                    tst   = v[1];
                    htmp1 = rt2r * v[1];
                    if (nr == 3)
                    {
                        ba = v[2];
                        aa = rt2r * v[2];
                        for (j = b_k; j + 1 < 5; j++)
                        {
                            ab = (h[b_k + (j << 2)] + tst * h[(b_k + (j << 2)) + 1]) +
                                 ba * h[(b_k + (j << 2)) + 2];
                            h[b_k + (j << 2)] -= ab * rt2r;
                            h[(b_k + (j << 2)) + 1] -= ab * htmp1;
                            h[(b_k + (j << 2)) + 2] -= ab * aa;
                        }

                        if (b_k + 4 < i + 1)
                        {
                            nr = b_k;
                        }
                        else
                        {
                            nr = i - 3;
                        }

                        for (j = 0; j + 1 <= nr + 4; j++)
                        {
                            ab = (h[j + (b_k << 2)] + tst * h[j + ((b_k + 1) << 2)]) +
                                 ba * h[j + ((b_k + 2) << 2)];
                            h[j + (b_k << 2)] -= ab * rt2r;
                            h[j + ((b_k + 1) << 2)] -= ab * htmp1;
                            h[j + ((b_k + 2) << 2)] -= ab * aa;
                        }
                    }
                    else
                    {
                        if (nr == 2)
                        {
                            for (j = b_k; j + 1 < 5; j++)
                            {
                                ab = h[b_k + (j << 2)] + tst * h[(b_k + (j << 2)) + 1];
                                h[b_k + (j << 2)] -= ab * rt2r;
                                h[(b_k + (j << 2)) + 1] -= ab * htmp1;
                            }

                            for (j = 0; j + 1 <= i + 1; j++)
                            {
                                ab = h[j + (b_k << 2)] + tst * h[j + ((b_k + 1) << 2)];
                                h[j + (b_k << 2)] -= ab * rt2r;
                                h[j + ((b_k + 1) << 2)] -= ab * htmp1;
                            }
                        }
                    }
                }

                its++;
            }
        }

        if (!goto150)
        {
            info   = i + 1;
            exitg1 = true;
        }
        else
        {
            if ((L != i + 1) && (L == i))
            {
                tst   = h[(i + (i << 2)) - 1];
                htmp1 = h[i + ((i - 1) << 2)];
                ab    = h[i + (i << 2)];
                xdlanv2(&h[(i + ((i - 1) << 2)) - 1], &tst, &htmp1, &ab, &ba, &aa, &rt2r,
                        &rt1r, &s, &sn);
                h[(i + (i << 2)) - 1] = tst;
                h[i + ((i - 1) << 2)] = htmp1;
                h[i + (i << 2)]       = ab;
                if (4 > i + 1)
                {
                    nr      = (i + ((i + 1) << 2)) - 1;
                    hoffset = i + ((i + 1) << 2);
                    for (k = 1; k <= 3 - i; k++)
                    {
                        tst        = s * h[nr] + sn * h[hoffset];
                        h[hoffset] = s * h[hoffset] - sn * h[nr];
                        h[nr]      = tst;
                        hoffset += 4;
                        nr += 4;
                    }
                }

                if (!(i - 1 < 1))
                {
                    nr      = (i - 1) << 2;
                    hoffset = i << 2;
                    for (k = 1; k < i; k++)
                    {
                        tst        = s * h[nr] + sn * h[hoffset];
                        h[hoffset] = s * h[hoffset] - sn * h[nr];
                        h[nr]      = tst;
                        hoffset++;
                        nr++;
                    }
                }
            }

            i = L - 2;
        }
    }

    return info;
}

/*
 * File trailer for xdhseqr.c
 *
 * [EOF]
 */
