#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzhgeqz.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 11-Oct-2018 23:25:44
 */

/* Include Files */

#include "xzhgeqz.h"
#include "xzlartg.h"
#include "SUKF.h"
#include "xzlanhs.h"
#include "codegenall_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : const creal32_T A[16]
 *                int ilo
 *                int ihi
 *                int *info
 *                creal32_T alpha1[4]
 *                creal32_T beta1[4]
 * Return Type  : void
 */
void xzhgeqz(const creal32_T A[16], int ilo, int ihi, int *info, creal32_T alpha1[4],
             creal32_T beta1[4])
{
    creal32_T b_A[16];
    int i;
    float eshift_re;
    float eshift_im;
    creal32_T ctemp;
    float anorm;
    float yi;
    float b_atol;
    float ascale;
    boolean_T failed;
    int j;
    boolean_T guard1 = false;
    boolean_T guard2 = false;
    int ifirst;
    int istart;
    int ilast;
    int ilastm1;
    int ifrstm;
    int ilastm;
    int iiter;
    boolean_T goto60;
    boolean_T goto70;
    boolean_T goto90;
    int jiter;
    int exitg1;
    boolean_T exitg2;
    boolean_T ilazro;
    creal32_T b_ascale;
    creal32_T shift;
    float ascale_re;
    float ascale_im;
    float ad22_re;
    float ad22_im;
    float t1_re;
    int x;
    float t1_im;
    float absxr;
    memcpy(&b_A[0], &A[0], sizeof(creal32_T) << 4);
    *info = 0;
    for (i = 0; i < 4; i++)
    {
        alpha1[i].re = 0.0F;
        alpha1[i].im = 0.0F;
        beta1[i].re  = 1.0F;
        beta1[i].im  = 0.0F;
    }

    eshift_re = 0.0F;
    eshift_im = 0.0F;
    ctemp.re  = 0.0F;
    ctemp.im  = 0.0F;
    anorm     = xzlanhs(A, ilo, ihi);
    yi        = 1.1920929E-7F * anorm;
    b_atol    = 1.17549435E-38F;
    if (yi > 1.17549435E-38F)
    {
        b_atol = yi;
    }

    yi = 1.17549435E-38F;
    if (anorm > 1.17549435E-38F)
    {
        yi = anorm;
    }

    ascale = 1.0F / yi;
    failed = true;
    for (j = ihi; j + 1 < 5; j++)
    {
        alpha1[j] = A[j + (j << 2)];
    }

    guard1 = false;
    guard2 = false;
    if (ihi >= ilo)
    {
        ifirst  = ilo;
        istart  = ilo;
        ilast   = ihi - 1;
        ilastm1 = ihi - 2;
        ifrstm  = ilo;
        ilastm  = ihi;
        iiter   = 0;
        goto60  = false;
        goto70  = false;
        goto90  = false;
        jiter   = 1;
        do
        {
            exitg1 = 0;
            if (jiter <= 30 * ((ihi - ilo) + 1))
            {
                if (ilast + 1 == ilo)
                {
                    goto60 = true;
                }
                else if (fabsf(b_A[ilast + (ilastm1 << 2)].re) +
                             fabsf(b_A[ilast + (ilastm1 << 2)].im) <=
                         b_atol)
                {
                    b_A[ilast + (ilastm1 << 2)].re = 0.0F;
                    b_A[ilast + (ilastm1 << 2)].im = 0.0F;
                    goto60                         = true;
                }
                else
                {
                    j      = ilastm1;
                    exitg2 = false;
                    while ((!exitg2) && (j + 1 >= ilo))
                    {
                        if (j + 1 == ilo)
                        {
                            ilazro = true;
                        }
                        else if (fabsf(b_A[j + ((j - 1) << 2)].re) +
                                     fabsf(b_A[j + ((j - 1) << 2)].im) <=
                                 b_atol)
                        {
                            b_A[j + ((j - 1) << 2)].re = 0.0F;
                            b_A[j + ((j - 1) << 2)].im = 0.0F;
                            ilazro                     = true;
                        }
                        else
                        {
                            ilazro = false;
                        }

                        if (ilazro)
                        {
                            ifirst = j + 1;
                            goto70 = true;
                            exitg2 = true;
                        }
                        else
                        {
                            j--;
                        }
                    }
                }

                if (goto60 || goto70)
                {
                    ilazro = true;
                }
                else
                {
                    ilazro = false;
                }

                if (!ilazro)
                {
                    for (i = 0; i < 4; i++)
                    {
                        alpha1[i].re = 0.0F;
                        alpha1[i].im = 0.0F;
                        beta1[i].re  = 0.0F;
                        beta1[i].im  = 0.0F;
                    }

                    *info  = 1;
                    exitg1 = 1;
                }
                else if (goto60)
                {
                    goto60        = false;
                    alpha1[ilast] = b_A[ilast + (ilast << 2)];
                    ilast         = ilastm1;
                    ilastm1--;
                    if (ilast + 1 < ilo)
                    {
                        failed = false;
                        guard2 = true;
                        exitg1 = 1;
                    }
                    else
                    {
                        iiter     = 0;
                        eshift_re = 0.0F;
                        eshift_im = 0.0F;
                        ilastm    = ilast + 1;
                        if (ifrstm > ilast + 1)
                        {
                            ifrstm = ilo;
                        }

                        jiter++;
                    }
                }
                else
                {
                    if (goto70)
                    {
                        goto70 = false;
                        iiter++;
                        ifrstm = ifirst;
                        if (iiter - iiter / 10 * 10 != 0)
                        {
                            anorm = ascale * b_A[ilastm1 + (ilastm1 << 2)].re;
                            yi    = ascale * b_A[ilastm1 + (ilastm1 << 2)].im;
                            if (yi == 0.0F)
                            {
                                shift.re = anorm / 0.5F;
                                shift.im = 0.0F;
                            }
                            else if (anorm == 0.0F)
                            {
                                shift.re = 0.0F;
                                shift.im = yi / 0.5F;
                            }
                            else
                            {
                                shift.re = anorm / 0.5F;
                                shift.im = yi / 0.5F;
                            }

                            anorm = ascale * b_A[ilast + (ilast << 2)].re;
                            yi    = ascale * b_A[ilast + (ilast << 2)].im;
                            if (yi == 0.0F)
                            {
                                ad22_re = anorm / 0.5F;
                                ad22_im = 0.0F;
                            }
                            else if (anorm == 0.0F)
                            {
                                ad22_re = 0.0F;
                                ad22_im = yi / 0.5F;
                            }
                            else
                            {
                                ad22_re = anorm / 0.5F;
                                ad22_im = yi / 0.5F;
                            }

                            t1_re = 0.5F * (shift.re + ad22_re);
                            t1_im = 0.5F * (shift.im + ad22_im);
                            anorm = ascale * b_A[ilastm1 + (ilast << 2)].re;
                            yi    = ascale * b_A[ilastm1 + (ilast << 2)].im;
                            if (yi == 0.0F)
                            {
                                ascale_re = anorm / 0.5F;
                                ascale_im = 0.0F;
                            }
                            else if (anorm == 0.0F)
                            {
                                ascale_re = 0.0F;
                                ascale_im = yi / 0.5F;
                            }
                            else
                            {
                                ascale_re = anorm / 0.5F;
                                ascale_im = yi / 0.5F;
                            }

                            anorm = ascale * b_A[ilast + (ilastm1 << 2)].re;
                            yi    = ascale * b_A[ilast + (ilastm1 << 2)].im;
                            if (yi == 0.0F)
                            {
                                absxr = anorm / 0.5F;
                                anorm = 0.0F;
                            }
                            else if (anorm == 0.0F)
                            {
                                absxr = 0.0F;
                                anorm = yi / 0.5F;
                            }
                            else
                            {
                                absxr = anorm / 0.5F;
                                anorm = yi / 0.5F;
                            }

                            yi       = shift.re * ad22_im + shift.im * ad22_re;
                            shift.re = ((t1_re * t1_re - t1_im * t1_im) +
                                        (ascale_re * absxr - ascale_im * anorm)) -
                                       (shift.re * ad22_re - shift.im * ad22_im);
                            shift.im = ((t1_re * t1_im + t1_im * t1_re) +
                                        (ascale_re * anorm + ascale_im * absxr)) -
                                       yi;
                            if (shift.im == 0.0F)
                            {
                                if (shift.re < 0.0F)
                                {
                                    anorm = 0.0F;
                                    yi    = sqrtf(-shift.re);
                                }
                                else
                                {
                                    anorm = sqrtf(shift.re);
                                    yi    = 0.0F;
                                }
                            }
                            else if (shift.re == 0.0F)
                            {
                                if (shift.im < 0.0F)
                                {
                                    anorm = sqrtf(-shift.im / 2.0F);
                                    yi    = -anorm;
                                }
                                else
                                {
                                    anorm = sqrtf(shift.im / 2.0F);
                                    yi    = anorm;
                                }
                            }
                            else
                            {
                                absxr = fabsf(shift.re);
                                anorm = fabsf(shift.im);
                                if ((absxr > 8.50705867E+37F) ||
                                    (anorm > 8.50705867E+37F))
                                {
                                    absxr *= 0.5F;
                                    anorm *= 0.5F;
                                    anorm = rt_hypotf(absxr, anorm);
                                    if (anorm > absxr)
                                    {
                                        anorm =
                                            sqrtf(anorm) * sqrtf(1.0F + absxr / anorm);
                                    }
                                    else
                                    {
                                        anorm = sqrtf(anorm) * 1.41421354F;
                                    }
                                }
                                else
                                {
                                    anorm =
                                        sqrtf((rt_hypotf(absxr, anorm) + absxr) * 0.5F);
                                }

                                if (shift.re > 0.0F)
                                {
                                    yi = 0.5F * (shift.im / anorm);
                                }
                                else
                                {
                                    if (shift.im < 0.0F)
                                    {
                                        yi = -anorm;
                                    }
                                    else
                                    {
                                        yi = anorm;
                                    }

                                    anorm = 0.5F * (shift.im / yi);
                                }
                            }

                            if ((t1_re - ad22_re) * anorm + (t1_im - ad22_im) * yi <=
                                0.0F)
                            {
                                shift.re = t1_re + anorm;
                                shift.im = t1_im + yi;
                            }
                            else
                            {
                                shift.re = t1_re - anorm;
                                shift.im = t1_im - yi;
                            }
                        }
                        else
                        {
                            anorm = ascale * b_A[ilast + (ilastm1 << 2)].re;
                            yi    = ascale * b_A[ilast + (ilastm1 << 2)].im;
                            if (yi == 0.0F)
                            {
                                ascale_re = anorm / 0.5F;
                                ascale_im = 0.0F;
                            }
                            else if (anorm == 0.0F)
                            {
                                ascale_re = 0.0F;
                                ascale_im = yi / 0.5F;
                            }
                            else
                            {
                                ascale_re = anorm / 0.5F;
                                ascale_im = yi / 0.5F;
                            }

                            eshift_re += ascale_re;
                            eshift_im += ascale_im;
                            shift.re = eshift_re;
                            shift.im = eshift_im;
                        }

                        j      = ilastm1;
                        i      = ilastm1 + 1;
                        exitg2 = false;
                        while ((!exitg2) && (j + 1 > ifirst))
                        {
                            istart   = j + 1;
                            ctemp.re = ascale * b_A[j + (j << 2)].re - shift.re * 0.5F;
                            ctemp.im = ascale * b_A[j + (j << 2)].im - shift.im * 0.5F;
                            yi       = fabsf(ctemp.re) + fabsf(ctemp.im);
                            anorm    = ascale * (fabsf(b_A[i + (j << 2)].re) +
                                              fabsf(b_A[i + (j << 2)].im));
                            absxr    = yi;
                            if (anorm > yi)
                            {
                                absxr = anorm;
                            }

                            if ((absxr < 1.0F) && (absxr != 0.0F))
                            {
                                yi /= absxr;
                                anorm /= absxr;
                            }

                            if ((fabsf(b_A[j + ((j - 1) << 2)].re) +
                                 fabsf(b_A[j + ((j - 1) << 2)].im)) *
                                    anorm <=
                                yi * b_atol)
                            {
                                goto90 = true;
                                exitg2 = true;
                            }
                            else
                            {
                                i = j;
                                j--;
                            }
                        }

                        if (!goto90)
                        {
                            istart = ifirst;
                            ctemp.re =
                                ascale * b_A[(ifirst + ((ifirst - 1) << 2)) - 1].re -
                                shift.re * 0.5F;
                            ctemp.im =
                                ascale * b_A[(ifirst + ((ifirst - 1) << 2)) - 1].im -
                                shift.im * 0.5F;
                            goto90 = true;
                        }
                    }

                    if (goto90)
                    {
                        goto90      = false;
                        b_ascale.re = ascale * b_A[istart + ((istart - 1) << 2)].re;
                        b_ascale.im = ascale * b_A[istart + ((istart - 1) << 2)].im;
                        b_xzlartg(ctemp, b_ascale, &anorm, &shift);
                        j = istart;
                        i = istart - 2;
                        while (j < ilast + 1)
                        {
                            if (j > istart)
                            {
                                xzlartg(b_A[(j + (i << 2)) - 1], b_A[j + (i << 2)],
                                        &anorm, &shift, &b_A[(j + (i << 2)) - 1]);
                                b_A[j + (i << 2)].re = 0.0F;
                                b_A[j + (i << 2)].im = 0.0F;
                            }

                            for (i = j - 1; i + 1 <= ilastm; i++)
                            {
                                ad22_re = anorm * b_A[(j + (i << 2)) - 1].re +
                                          (shift.re * b_A[j + (i << 2)].re -
                                           shift.im * b_A[j + (i << 2)].im);
                                ad22_im = anorm * b_A[(j + (i << 2)) - 1].im +
                                          (shift.re * b_A[j + (i << 2)].im +
                                           shift.im * b_A[j + (i << 2)].re);
                                yi = b_A[(j + (i << 2)) - 1].re;
                                b_A[j + (i << 2)].re =
                                    anorm * b_A[j + (i << 2)].re -
                                    (shift.re * b_A[(j + (i << 2)) - 1].re +
                                     shift.im * b_A[(j + (i << 2)) - 1].im);
                                b_A[j + (i << 2)].im =
                                    anorm * b_A[j + (i << 2)].im -
                                    (shift.re * b_A[(j + (i << 2)) - 1].im -
                                     shift.im * yi);
                                b_A[(j + (i << 2)) - 1].re = ad22_re;
                                b_A[(j + (i << 2)) - 1].im = ad22_im;
                            }

                            shift.re = -shift.re;
                            shift.im = -shift.im;
                            x        = j;
                            if (ilast + 1 < j + 2)
                            {
                                x = ilast - 1;
                            }

                            for (i = ifrstm - 1; i + 1 <= x + 2; i++)
                            {
                                ad22_re = anorm * b_A[i + (j << 2)].re +
                                          (shift.re * b_A[i + ((j - 1) << 2)].re -
                                           shift.im * b_A[i + ((j - 1) << 2)].im);
                                ad22_im = anorm * b_A[i + (j << 2)].im +
                                          (shift.re * b_A[i + ((j - 1) << 2)].im +
                                           shift.im * b_A[i + ((j - 1) << 2)].re);
                                yi = b_A[i + (j << 2)].re;
                                b_A[i + ((j - 1) << 2)].re =
                                    anorm * b_A[i + ((j - 1) << 2)].re -
                                    (shift.re * b_A[i + (j << 2)].re +
                                     shift.im * b_A[i + (j << 2)].im);
                                b_A[i + ((j - 1) << 2)].im =
                                    anorm * b_A[i + ((j - 1) << 2)].im -
                                    (shift.re * b_A[i + (j << 2)].im - shift.im * yi);
                                b_A[i + (j << 2)].re = ad22_re;
                                b_A[i + (j << 2)].im = ad22_im;
                            }

                            i = j - 1;
                            j++;
                        }
                    }

                    jiter++;
                }
            }
            else
            {
                guard2 = true;
                exitg1 = 1;
            }
        } while (exitg1 == 0);
    }
    else
    {
        guard1 = true;
    }

    if (guard2)
    {
        if (failed)
        {
            *info = ilast + 1;
            for (i = 0; i + 1 <= ilast + 1; i++)
            {
                alpha1[i].re = 0.0F;
                alpha1[i].im = 0.0F;
                beta1[i].re  = 0.0F;
                beta1[i].im  = 0.0F;
            }
        }
        else
        {
            guard1 = true;
        }
    }

    if (guard1)
    {
        for (j = 0; j + 1 < ilo; j++)
        {
            alpha1[j] = b_A[j + (j << 2)];
        }
    }
}

/*
 * File trailer for xzhgeqz.c
 *
 * [EOF]
 */
