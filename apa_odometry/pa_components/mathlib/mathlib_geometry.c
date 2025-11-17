/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: ap_psd.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/
/*! enables usage of external visible declarations from other MATHLIB modules */
#define COMPONENT_MTL
// the following line specifies a documentation section for MATHLIB
/*!\addtogroup COMPONENT_MATHLIB MATHLIB documentation */
#include <pa_components/base_type/global_include.h>
/** The geometry library shall only be visible if a component using the
  geometry library is included in the target configuration */
#if (GS_MTL_GEOLIB != SW_OFF)

#include "mathlib_api.h"
#include "mathlib_geometry_api.h"

#ifdef QAC_MSG_OFF
/*BEGIN_QAC_COMMENT*/
/*
  Suppress QA-C Warning 0506 -> Dereferencing pointer value that is possibly
  NULL. All pointers used in function parameter lists shall be used in a
  call-by-reference fashion. Every function call shall only contain statically
  initialized pointers. Suppress QA-C Warning 3103 -> Integer division or
  remainder operation with an operand whose underlying type is signed or
  negative. This pops up for every division. Suppress QA-C Warning 2217 -> Line
  Length exceeds 130 characters. This pops up even for lines within function
  comments.
*/
/*END_QAC_COMMENT*/
#pragma PRQA_MESSAGES_OFF 506, 3103, 2217
#endif

/*--------------------------------------------------------------------------*/
/*- Check if Compiler Switches (utilised in this file) are defined         -*/
/*--------------------------------------------------------------------------*/

// from here on all documentation is only for internals
//!\cond INTERNAL
/*--------------------------------------------------------------------------*/
/*!\addtogroup MATHLIB_L01                local symbolic constants
 * \ingroup COMPONENT_MATHLIB
 @{ *//*--------------------------------------------------------------------*/

/*!@} */ /*------------------------------------------------------------------*/
/*!\addtogroup MATHLIB_L02                local macros
 * \ingroup COMPONENT_MATHLIB
 @{ *//*--------------------------------------------------------------------*/

#define gd_MTL_2EXP14_si16 ((SInt16)16384)

/*!@} */ /*------------------------------------------------------------------*/
/*!\addtogroup MATHLIB_L03                local data types
 * \ingroup COMPONENT_MATHLIB
 @{ *//*--------------------------------------------------------------------*/

/*!@} */ /*------------------------------------------------------------------*/
/*!\addtogroup MATHLIB_L04                local data
 * \ingroup COMPONENT_MATHLIB
 @{ *//*--------------------------------------------------------------------*/

/*!@}*/ /*-------------------------------------------------------------------*/
/*-                      prototypes of local functions                     -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*!\addtogroup MATHLIB_G01                global data objects
 * \ingroup COMPONENT_MATHLIB
 @{ *//*--------------------------------------------------------------------*/

gType_mtlAngle_st const g_mtlAngle_PI_2_st = {(SInt16)gd_MTL_PI_2_ui16,
                                              gd_MTL_2EXP14_si16, 0}; // sin = 1, cos = 0
gType_mtlAngle_st const g_mtlAngle_PI_st   = {(SInt16)gd_MTL_PI_ui16, 0,
                                              -gd_MTL_2EXP14_si16}; // sin = 0, cos = -1

/*!@}*/ /*-------------------------------------------------------------------*/
/*!\addtogroup MATHLIB_L05                local functions
 * \ingroup COMPONENT_MATHLIB
 @{ *//*--------------------------------------------------------------------*/

/*!@}*/ /*-------------------------------------------------------------------*/
/*!\addtogroup MATHLIB_G02                public functions
 * \ingroup COMPONENT_MATHLIB
 @{ *//*--------------------------------------------------------------------*/

/************************************************************************/
/* Mathlib core function candidates                                     */
/************************************************************************/

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_SolveQuadEquationWithSqrt_ui8                      *//**
\brief         Calculate real roots (solutions) of quadratic polynomial equations.
\details       Solve a*x^2 + 2*beta*x + c = 0 (0 to 2 solutions for x)
  with beta = b/2

The number of real roots (either zero, one or two) is returned,
and their locations are stored in x0 and x1.
If one real root is found (i.e. if a=0) then it is stored in x0.
When two real roots are found they are stored in x0 and x1 in ascending order.
In case of coincident roots (if they are detected with the given numerical precision)
the number of returned roots is 1.

If a = 0 and b = 0 and c |= 0, no solutions exist and 0 is returned.
If a, b and c are 0, infinite solutions exist. For compatibility, numberOfSolutions = 1 is returned.

We use beta = b/2 as parameter to avoid large integer numbers as much as possible.

Formula:
x_{1,2} = (-beta +- \sqrt{beta^2 - a c} ) / a

The number of roots found depends on the sign of the discriminant beta^2 - a c.
This will be subject to rounding and cancellation errors,
and will also be subject to errors if the coefficients of the polynomial are inexact.
These errors may cause a discrete change in the number of roots.

The only difference to g_mtl_SolveQuadEquation_ui8 is that this function uses sqrt(),
while the other function uses an iterative method based on law of Vieta.
The sqrt-based implementation may have SLIGHTLY higher precision (in range of 0.1%)
in some situations.

Unit test: m_Test_SolveQuadEquation_ui8

*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_a_si16     a in a*x^2 + 2*beta*x + c = 0
\param[in]     f_beta_si16  b in a*x^2 + 2*beta*x + c = 0
\param[in]     f_c_si16     c in a*x^2 + 2*beta*x + c = 0
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    f_Solution1F16_si32  smaller solution, x_1 = (-b + sqrt(beta^2 - a c)) / a
\param[out]    f_Solution2F16_si32  larger solution, x_2 = (-b - sqrt(beta^2 - a c)) / a
\return        number of solutions [0..2]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        Arwed Starke (str2lr)
*//*************************************************************************/
UInt8 g_mtl_SolveQuadEquationWithSqrt_ui8(SInt32 *f_Solution1F16_si32,
                                          SInt32 *f_Solution2F16_si32, SInt16 f_a_si16,
                                          SInt16 f_beta_si16, SInt16 f_c_si16)
{
    UInt8 l_NumberOfSolutions_ui8; // Die anzahl der l�sungen der Gleichung
    UInt32 l_radix_ui32;
    SInt32 l_TestValue1_si32;
    SInt32 l_TestValue2_si32;

    if (f_a_si16 == 0)
    {
        // Wenn A = 0 dann haben wir eine lineare Gleichung
        // getrenntes Verfahren n�tig ...
        // 2 * b * x + c = 0
        // x = - c / (2 * b)
        if (f_beta_si16 != 0)
        {
            // Sonderfall f�r eine quadratische Gleichung die keine ist...
            // c * 2^16 / (2 * b) = c * 2^15 / b
            *f_Solution1F16_si32    = (-(SInt32)f_c_si16 * 0x8000) / (SInt32)f_beta_si16;
            *f_Solution2F16_si32    = *f_Solution1F16_si32;
            l_NumberOfSolutions_ui8 = 1; // Eine L�sung
        }
        else
        {
            // A = 0 und B = 0 sind nur im Fall C= 0 l�sbar: unendlich viele L�sungen
            if (f_c_si16 == 0)
            {
                // Sonderfall: 0 = 0, x ist beliebig
                *f_Solution1F16_si32    = 0;
                *f_Solution2F16_si32    = 0;
                l_NumberOfSolutions_ui8 = 1; // eigentlich unendlich viele L�sungen
            }
            else
            {
                l_NumberOfSolutions_ui8 = 0; // c = 0. Hier gibt es keine L�sung
            }
        }
    }
    else
    {
        if (f_c_si16 == 0)
        {
            // entartete Gleichung: a*x^2 + 2*beta*x = 0
            // L�sungen: x1 = 0, x2 = -2*beta/a
            *f_Solution1F16_si32 = 0;
            *f_Solution2F16_si32 =
                g_mtl_s32_Div_s32_si32(-f_beta_si16 * (1 << 16),
                                       f_a_si16); // -b/a; return value shall be F16

            l_NumberOfSolutions_ui8 = 1;

            if (g_mtl_Abs_mac(*f_Solution2F16_si32) > (GD_MAX_SI32 / 2))
            {
                *f_Solution2F16_si32 =
                    GD_MAX_SI32; // 2nd solution not representable in result type
            }
            else
            {
                *f_Solution2F16_si32 *= 2;     // -2b/a
                if (*f_Solution2F16_si32 != 0) // case if b != 0
                {
                    l_NumberOfSolutions_ui8 = 2;
                }
            }
        }
        else
        {
            // Pr�fung ob (beta^2 - ac) >= 0, durch Vergleich B^2 >= AC zur Kontrolle
            // der L�sbarkeit
            l_TestValue1_si32 = (SInt32)f_beta_si16 * f_beta_si16;
            l_TestValue2_si32 = (SInt32)f_a_si16 * f_c_si16; // input scaling squared

            if (l_TestValue1_si32 == l_TestValue2_si32)
            {
                // Triviale L�sung: Die Wurzel wird 0. x = -beta/a.
                *f_Solution1F16_si32 =
                    (-(SInt32)f_beta_si16 * 0x10000) / (SInt32)f_a_si16;
                *f_Solution2F16_si32    = *f_Solution1F16_si32;
                l_NumberOfSolutions_ui8 = 1;
            }
            else if (l_TestValue1_si32 > l_TestValue2_si32)
            {
                l_NumberOfSolutions_ui8 = 2;

                if (f_beta_si16 == 0)
                {
                    // simplified solution: x = +- sqrt(-c/a). Note that c/a < 0
                    *f_Solution1F16_si32 = -g_mtl_s32_Div_s32_si32(
                        (SInt32)f_c_si16 * (1 << 16), f_a_si16); // F16
                    *f_Solution2F16_si32 =
                        (SInt32)((UInt32)g_mtl_Sqrt_u32_ui16((UInt32)*f_Solution1F16_si32)
                                 << 8); // F8 * 2^8 = F16
                    *f_Solution1F16_si32 = -*f_Solution2F16_si32;
                }
                else
                {
                    // normal solution. The generally accepted algorithm that computes the
                    // larger result first, avoiding numeric errors due to catastrophic
                    // cancellation
                    // The return of the macro g_mtl_Signum_mac is always equal to +1 or
                    // -1. Conversion from si16 to si8 has no side effects
                    SInt8 l_signBeta_si8 = (SInt8)g_mtl_Signum_mac(f_beta_si16);
                    SInt32 l_temp_si32;
                    // sqrt(b^2 - ac) - kein Overflow m�glich, da 1 Bit gewonnen und
                    // TestValue1 immer positiv ist
                    l_radix_ui32 = (UInt32)(l_TestValue1_si32 - l_TestValue2_si32);
                    l_radix_ui32 =
                        g_mtl_Sqrt_u32_ui16(l_radix_ui32); // input scaling restored

                    if (l_radix_ui32 == 0) // whoops! bad precision
                    {
                        l_NumberOfSolutions_ui8 = 1;
                    }

                    // x1 = -(beta + sign(beta) * sqrt(b^2 - ac)) / a = temp/a,     x_2 =
                    // c / (a * x_1) = c/temp

                    // Overflow consideration: l_radix < 2^16; l_temp_si32 < 2^17; in very
                    // rare cases it may be above 2^16
                    l_temp_si32 = -(((SInt32)(f_beta_si16)) +
                                    (l_signBeta_si8 * (SInt32)l_radix_ui32));
                    if (g_mtl_Abs_mac(l_temp_si32) > GD_MAX_SI16)
                    {
                        *f_Solution1F16_si32 = g_mtl_s32_Div_s32_si32(
                            l_temp_si32 * (1 << 15), f_a_si16 / 2); // shift up: F16
                    }
                    else
                    {
                        *f_Solution1F16_si32 = g_mtl_s32_Div_s32_si32(
                            l_temp_si32 * (1 << 16), f_a_si16); // F16
                    }

                    *f_Solution2F16_si32 = g_mtl_s32_Div_s32_si32(
                        (SInt32)f_c_si16 * (1 << 16), (SInt32)l_temp_si32);

                    if (*f_Solution1F16_si32 > *f_Solution2F16_si32)
                    {
                        l_TestValue1_si32    = *f_Solution1F16_si32;
                        *f_Solution1F16_si32 = *f_Solution2F16_si32;
                        *f_Solution2F16_si32 = l_TestValue1_si32;
                    }
                }
            }
            else
            {
                l_NumberOfSolutions_ui8 = (UInt8)0;
            }
        }
    }
    return l_NumberOfSolutions_ui8;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_SolveQuadEquation_ui8                              *//**
\brief         Calculate the real roots of a quadratic polynomial equation
               without using the square root function.
\details
Solve a*x^2 + 2*beta*x + c = 0 (0 to 2 solutions for x)
  with beta = b/2

The number of real roots (either zero, one or two) is returned,
and their locations are stored in x0 and x1.
If one real root is found (i.e. if a=0) then it is stored in x0.
When two real roots are found they are stored in x0 and x1 in ascending order.
In case of coincident roots (if they are detected with the given numerical precision)
the number of returned roots is 1.

If a = 0 and b = 0 and c |= 0, no solutions exist and 0 is returned.
If a, b and c are 0, infinite solutions exist. For compatibility, numberOfSolutions = 1 is returned.

We use beta = b/2 as parameter to avoid large integer numbers as much as possible.

Formula:
x_{1,2} = (-beta +- \sqrt{beta^2 - a c} ) / a

The number of roots found depends on the sign of the discriminant beta^2 - a c.
This will be subject to rounding and cancellation errors,
and will also be subject to errors if the coefficients of the polynomial are inexact.
These errors may cause a discrete change in the number of roots.

Vieta's formula is used to find the best approximate solution by
repeated comparison of x_1 * x_2 = c/a, starting from vertex point x_v = -beta/a

Unit test: m_Test_SolveQuadEquation_ui8

*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_KoeffOfXQuad_si16     a in a*x^2 + b*x + c = 0
\param[in]     f_KoeffOfX_si16         b in a*x^2 + b*x + c = 0
\param[in]     f_KoeffWithoutX_si16     c in a*x^2 + b*x + c = 0
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    f_Solution1F16_si32  smaller solution (x_1)
\param[out]    f_Solution2F16_si32  larger  solution (x_2)
\return        number of solutions [0..2]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        Thomas Scheuerle (sce2lr), Arwed Starke (str2lr)
*//*************************************************************************/
UInt8 g_mtl_SolveQuadEquation_ui8(SInt32 *f_Solution1F16_si32,
                                  SInt32 *f_Solution2F16_si32, SInt16 f_a_si16,
                                  SInt16 f_beta_si16, SInt16 f_c_si16)
{
    UInt8 l_NumberOfSolutions_ui8;  // Die anzahl der l�sungen der Gleichung
    SInt32 l_ScheitelX_F16_si32;    // Der X Wert zum Scheitelpunkt der Parabel
    SInt32 l_ScheitelXQuadF16_si32; // Der X Wert zum Scheitelpunkt der Parabel
                                    // im Quadrat
    UInt32 l_Bitmask_ui32;          // Eine rotierende Bitmaske
    UInt32 l_radix_ui32;            // Das Ergebnisoffset zum Scheitel
    SInt32 l_sum_si32;              // Aktuelle bewiesene Bits + Testbit + Scheitel
    SInt32 l_TestValue1_si32;       // A*X+B
    SInt32 l_TestValue2_si32;       // (-C)/X

    if (f_a_si16 == 0)
    {
        // Wenn A = 0 dann haben wir eine lineare Gleichung
        // getrenntes Verfahren n�tig ...
        if (f_beta_si16 != 0)
        {
            // Sonderfall f�r eine quadratische Gleichung die keine ist: x = -c/b =
            // -c/(2*beta)
            *f_Solution1F16_si32 = (-(SInt32)f_c_si16 * (SInt32)0x8000) /
                                   f_beta_si16; // 2^16/ 2 = 2^15 = 0x8000
            *f_Solution2F16_si32    = *f_Solution1F16_si32;
            l_NumberOfSolutions_ui8 = 1; // Eine L�sung
        }
        else
        {
            // A = 0 und B = 0 sind nur im Fall C= 0 l�sbar: unendlich viele L�sungen
            if (f_c_si16 == 0)
            {
                // Sonderfall: 0 = 0, x ist beliebig
                *f_Solution1F16_si32    = 0;
                *f_Solution2F16_si32    = 0;
                l_NumberOfSolutions_ui8 = 1; // Eine L�sung, eigentlich unendlich viele...
            }
            else
            {
                l_NumberOfSolutions_ui8 = 0; // c = 0. Hier gibt es keine L�sung
            }
        }
    }
    else
    {
        // Wir Klappen das Vorzeichen der Gleichung um falls n�tig
        // wenn A<0 dann A,B,C *-1
        if (f_a_si16 < 0)
        {
            f_a_si16    = -f_a_si16;
            f_beta_si16 = -f_beta_si16;
            f_c_si16    = -f_c_si16;
        }

        // Pr�fung ob (beta^2 - ac) >= 0, durch Vergleich B^2 >= AC zur Kontrolle
        // der L�sbarkeit
        l_TestValue1_si32 = (SInt32)f_beta_si16 * f_beta_si16;
        l_TestValue2_si32 = (SInt32)f_a_si16 * f_c_si16; // input scaling squared

        // Zun�chst bestimmen wir das X im Scheitelpunkt der Parabel Scheitelpunkt =
        // -beta/a
        l_ScheitelX_F16_si32 =
            g_mtl_s32_Div_s32_si32(-(SInt32)f_beta_si16 * 0x10000, f_a_si16); // F16

        if (l_TestValue1_si32 == l_TestValue2_si32)
        {
            // Triviale L�sung f�r bessere Genauigkeit: Die Wurzel wird 0. x =
            // -beta/a.
            *f_Solution1F16_si32    = l_ScheitelX_F16_si32;
            *f_Solution2F16_si32    = l_ScheitelX_F16_si32;
            l_NumberOfSolutions_ui8 = 1;
        }
        else if (l_TestValue1_si32 > l_TestValue2_si32)
        {
            if (f_c_si16 == 0)
            {
                // Abk�rzung f�r bessere Genauigkeit: L�sung f�r die Gleichung: a*x^2 +
                // 2*beta*x = 0 L�sungen: x1 = 0, x2 = -2beta/a
                *f_Solution1F16_si32 = 0;

                if (g_mtl_Abs_mac(l_ScheitelX_F16_si32) > (GD_MAX_SI32 / 2))
                {
                    l_NumberOfSolutions_ui8 =
                        1; // 2nd solution not representable in result type
                    *f_Solution2F16_si32 = GD_MAX_SI32;
                }
                else
                {
                    *f_Solution2F16_si32    = l_ScheitelX_F16_si32 * 2; // -2b/a
                    l_NumberOfSolutions_ui8 = 2;
                }
            }
            else
            {
                l_ScheitelXQuadF16_si32 =
                    g_mtl_ReducePrecision_si32(l_ScheitelX_F16_si32, 8); // F8

                // Wir m�ssen pr�fen ob wir genug Bits zum quadrieren haben.
                // Wenn nicht, deutet dies darauf hin dass beta >> a, was auch bedeutet
                // dass der Wurzelterm (beta^2 - a*c) ~ beta^2, was wiederum schlecht
                // f�r die Genauigkeit der L�sung ist. Wikipedia: Wenn b/a >> c/a, dann
                // ist Vietas Approximation genauer.
                if (g_mtl_Abs_mac(l_ScheitelXQuadF16_si32) < (SInt32)0x7fff)
                {
                    // Vorberechnung Scheitelwert im Quadrat
                    l_ScheitelXQuadF16_si32 =
                        g_mtl_Square_mac(l_ScheitelXQuadF16_si32); // F8 * F8 = F16

                    // Nun versuchen wir mit einer wandernden Bitmaske
                    // Bits in einem L�sungsoffset L so zu setzen dass Gilt:
                    // (Scheitelpunkt +/- L) = X bei AX^2 +BX +C = 0
                    // Zum Test der Hypothese nehmen wir den Satz des Vieta:
                    // x_1 * x_2 = c/a

                    l_Bitmask_ui32 =
                        0x8000U; // Bei A!=0 sind 16 Bit L�sungsmenge erwartet
                    l_radix_ui32 = 0U;

                    // Vorberechnung von c/a (q)
                    l_TestValue2_si32 = (SInt32)f_c_si16 * (SInt32)0x10000;
                    l_TestValue2_si32 /= (SInt32)f_a_si16;

                    // Wir laufen maximal 16mal durch die Bitschieberschleife
                    // Hier wird der Vorkommateil der ergebnisse berechnet
                    while (l_Bitmask_ui32 > (UInt32)0)
                    {
                        // Wir berechnen eine neue Hypothese vorl�ufig in l_sum_si32
                        l_sum_si32 = (SInt32)(l_radix_ui32 + l_Bitmask_ui32);

                        // Pr�fe ob x_1 * x_2 = c/a
                        // Ersetzen x_1 * x_2 durch (Scheitel + Radix) * (Scheitel -
                        // Radix) Binom: Scheitel^2 - Radix^2 = c/a

                        l_TestValue1_si32 =
                            l_ScheitelXQuadF16_si32 - (l_sum_si32 * l_sum_si32);
                        // Immer wenn l_TestValue1_si32 >= l_TestValue2_si32
                        // wollen wir das als Hypothese in l_sum_si32 vorl�ufig
                        // gesetzte Bit endg�ltig in l_radix_ui16 �bernehmen

                        if (l_TestValue1_si32 >= l_TestValue2_si32)
                        {
                            l_radix_ui32 += l_Bitmask_ui32;

                            if (l_TestValue1_si32 == l_TestValue2_si32)
                            {
                                // exakte L�sung - wir sind fertig
                                break;
                                // Ein �berfl�ssiges weiterlaufen riskiert einen �berlauf
                                // im Hypothesentest da nun unser Parabelast steil
                                // ausscheeren wird
                                // ...
                            }
                        }
                        l_Bitmask_ui32 >>= 1;
                    }

                    // Berechnung der L�sungen
                    *f_Solution1F16_si32 =
                        l_ScheitelX_F16_si32 - ((SInt32)l_radix_ui32 * (SInt32)0x100);
                    *f_Solution2F16_si32 =
                        l_ScheitelX_F16_si32 + ((SInt32)l_radix_ui32 * (SInt32)0x100);

                    if (l_radix_ui32 != 0)
                    {
                        l_NumberOfSolutions_ui8 = (UInt8)2;
                    }
                    else
                    {
                        l_NumberOfSolutions_ui8 = (UInt8)1;
                    }
                }
                else
                {
                    // Hier haben wir zu wenig Bits: b^2 >> a*c, sqrt(b^2 - a*c) ~ b, und
                    // damit ist x_2 << x_1. daher: x_1 + x_2 ~= x_1, und folgich ist x_1
                    // ~= -b/a, x_2 ~= -c/b, bzw: x_1 ~= -(2*beta)/a,  x_2 ~= -c/(2*beta).
                    *f_Solution1F16_si32 = l_ScheitelX_F16_si32 * 2;
                    *f_Solution2F16_si32 =
                        g_mtl_s32_Div_s32_si32(-(SInt32)f_c_si16 * 0x8000, f_beta_si16);
                    l_NumberOfSolutions_ui8 = (UInt8)2;
                }

                if (*f_Solution1F16_si32 > *f_Solution2F16_si32)
                {
                    l_TestValue1_si32    = *f_Solution1F16_si32;
                    *f_Solution1F16_si32 = *f_Solution2F16_si32;
                    *f_Solution2F16_si32 = l_TestValue1_si32;
                }
            }
        }
        else
        {
            // negative Wurzel: keine reelle L�sung
            l_NumberOfSolutions_ui8 = (UInt8)0;
        }
    }
    return l_NumberOfSolutions_ui8;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_ShiftScaledValueToTargetResolution_vd               *//**
\brief         Shift a value with variable scaling to target resolution.
               This function usually comes last in a pseudo-float calculation.
\details       use the "pseudo floating point" functions if you want to
               preserve as much information as possible for following calculations
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_value_ui32     the value
\param[in]     f_scaling_ui8    the scaling of the value (e.g. 27 means
               that the normalized value is [value * 2^-27]
\param[in]     f_targetResolution_ui8    the target resolution, e.g. 10 if
               f_value_pui32 should be a F10 variable.
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\return        the value in target resolution. scaling is now f_targetResolution_ui8.
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                      *//**
\author        str2lr
*//*************************************************************************/
UInt32 g_mtl_ShiftScaledValueToTargetResolution_ui32(UInt32 f_Value_ui32,
                                                     UInt8 f_Scaling_ui8,
                                                     UInt8 f_TargetResolution_ui8)
{
    if (f_Scaling_ui8 >= f_TargetResolution_ui8)
    {
        f_Value_ui32 = g_mtl_ReducePrecision_ui32(
            f_Value_ui32, f_Scaling_ui8 - f_TargetResolution_ui8); // F10
    }
    else
    {
        f_Value_ui32 <<= (f_TargetResolution_ui8 - f_Scaling_ui8); // F10
    }
    return f_Value_ui32;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_ShiftScaledValueToTargetResolution_si32               *//**
\brief         Shift a value with variable scaling to target resolution.
               This function usually comes last in a pseudo-float calculation.
\details       use the "pseudo floating point" functions if you want to
               preserve as much information as possible for following calculations.
      This function should belong to mtl__math!!
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_Value_si32     the value
\param[in]     f_Scaling_si8    the scaling of the value (e.g. 27 means
               that the normalized value is [value * 2^-27]
\param[in]     f_TargetResolution_ui8    the target resolution, e.g. 10 if
               f_Value_si32 should be a F10 variable.
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\return        the value in target resolution. scaling is now f_targetResolution_ui8.
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                      *//**
\author        str2lr
*//*************************************************************************/
SInt32 g_mtl_ShiftScaledValueToTargetResolution_si32(SInt32 f_Value_si32,
                                                     SInt8 f_Scaling_si8,
                                                     UInt8 f_TargetResolution_ui8)
{
    if (f_Scaling_si8 >= f_TargetResolution_ui8)
    {
        f_Value_si32 = g_mtl_ReducePrecision_si32(
            f_Value_si32, f_Scaling_si8 - f_TargetResolution_ui8); // F10
    }
    else
    {
        f_Value_si32 *= (1 << (f_TargetResolution_ui8 - f_Scaling_si8)); // F10
    }

    return f_Value_si32;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_ScaledSqrt_ui32                                     *//**
\brief         root operation on values with variable scaling
\details       use the "pseudo floating point" functions if you want to
               preserve as much information as possible for following calculations
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_Value_ui32     the value
\param[in]     f_Scaling_pui8    the scaling of the value (e.g. 27 means
                                 that the normalized value is [value * 2^-27]
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\return        the result of the sqrt with scale [2^-Scaling]
\param[out]    f_Scaling_pui8    updated scaling
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                      *//**
\author        str2lr
*//*************************************************************************/
UInt32 g_mtl_ScaledSqrt_ui32(UInt32 f_Value_ui32, UInt8 *f_Scaling_pui8)
{
    while ((f_Value_ui32 < (UInt32)0x7fffffff) && (f_Value_ui32 != 0))
    {
        f_Value_ui32 <<= 1;
        *f_Scaling_pui8 += (UInt8)1;
    }
    /* scaling must be even in cause of square root operation */
    if ((*f_Scaling_pui8 % (UInt8)2) == 1)
    {
        f_Value_ui32 >>= 1;
        *f_Scaling_pui8 -= (UInt8)1;
    }
    f_Value_ui32 = (UInt32)g_mtl_Sqrt_u32_ui16(f_Value_ui32); // Scaling/2

    *f_Scaling_pui8 >>= 1;

    return f_Value_ui32;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_ScaleUp_ui32                                        *//**
\brief         Scale up an unsigned value (e.g. for division)
\details       use the "pseudo floating point" functions if you want to
preserve as much information as possible for following calculations.

This loop is guaranteed to run less than 32 iterations.

*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_Value_ui32     the value
\param[in]     f_Scaling_pui8   the scaling of the value (e.g. 27 means
                                that the normalized value is [value * 2^-27]
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\return         value is scaled up so that the MSB is 1
\param[out]    f_Scaling_pui8    updated scaling
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                      *//**
\author        str2lr
*//*************************************************************************/
UInt32 g_mtl_ScaleUp_ui32(UInt32 f_Value_ui32, UInt8 *f_Scaling_pui8)
{
    while ((f_Value_ui32 < (UInt32)0x7fffffff) && (f_Value_ui32 != 0))
    {
        f_Value_ui32 <<= 1;
        *f_Scaling_pui8 += (UInt8)1;
    }

    return f_Value_ui32;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_ScaleUp_si32                                        *//**
\brief         Scale up a signed value (e.g. for division)
\details       use the "pseudo floating point" functions if you want to
preserve as much information as possible for following calculations.

This loop is guaranteed to run less than 31 iterations.

*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_Value_si32     the value
\param[in]     f_Scaling_pui8   the scaling of the value (e.g. 27 means
                                that the normalized value is [value * 2^-27]
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\return        value is scaled up so that MSB-1 is 1. The sign bit is preserved.
\param[out]    f_Scaling_pui8    updated scaling
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                      *//**
\author        str2lr
*//*************************************************************************/
SInt32 g_mtl_ScaleUp_si32(SInt32 f_Value_si32, SInt8 *f_Scaling_psi8)
{
    while ((f_Value_si32 < (SInt32)0x3fffffff) && (f_Value_si32 > -(SInt32)0x3fffffff) &&
           (f_Value_si32 != 0))
    {
        f_Value_si32 *= 2;
        *f_Scaling_psi8 += (SInt8)1;
    }

    return f_Value_si32;
}

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
------------------------------------------------------------------------------
Function name: g_mtl_ScaleDown_si32_to_si16_vd *//*!
\brief       Change the scale of X so that the X value would fit into a SInt16.
\details     using the Pseudo-floating point operations. Every value
             comes with a scaling value
*//*--------------------------------------------------------------------------
S I D E  E F F E C T S                                                   *//*!
\pre  X has the previous value
\post the value and the Scale for X are changed
*//*--------------------------------------------------------------------------
I N P U T                                                                *//*!
\param[in]  *X_si32      The X value.
            *X_scale_si8 The corresponding scale to X.
*//*--------------------------------------------------------------------------
O U T P U T                                                             *//*!
\param[out] *X_si32      The X value, scaled so that it fits into SInt16
			      *X_scale_si8 The corresponding scale to the changed X.
            1 is subtracted For each division by 2. E.g. value has been
            divided by 8 to fit into SInt16, then this is (Scale_in - 3)
*//*--------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                       *//*!
\author   $RuA1Lr
*//**************************************************************************/
void g_mtl_ScaleDown_si32_toFit_si16_vd(SInt32 *f_X_si32, SInt8 *f_X_scale_si8)
{
    const UInt8 l_largestAllowedValueBit_ui8 =
        (UInt8)14; // Bit 15 and above are critical (SInt16: Bit15 is sign bit)
    UInt8 l_requiredShift_ui8 = l_largestAllowedValueBit_ui8;
    const UInt32 l_absX_ui32 =
        (UInt32)g_mtl_Abs_mac(*f_X_si32); // do not test bits on 2-complement numbers
    UInt8 l_i_ui8;
    // search for MSB (starting at largest allowed value bit)
    for (l_i_ui8 = l_largestAllowedValueBit_ui8; l_i_ui8 < 32; l_i_ui8++)
    {
        if ((l_absX_ui32 & ((UInt32)1 << l_i_ui8)) != FALSE)
        {
            l_requiredShift_ui8 = l_i_ui8;
        }
    }
    l_requiredShift_ui8 -= l_largestAllowedValueBit_ui8; // if bit 15 is set, shift 1 bit
                                                         // left, and so on

    *f_X_si32 = g_mtl_ReducePrecision_si32(*f_X_si32, l_requiredShift_ui8);
    *f_X_scale_si8 -= (SInt8)l_requiredShift_ui8;
}

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
------------------------------------------------------------------------------
Function name: g_mtl_ScaleDown2_si32_toFit_si16_vd                        *//*!
\brief       Change the scale of 2 values by the same amount, so that both fit into SInt16.
\details     using the Pseudo-floating point operations. Every value
             comes with a scaling value
*//*--------------------------------------------------------------------------
S I D E  E F F E C T S                                                   *//*!
\pre  X has the previous value
\post the value and the Scale for X are changed
*//*--------------------------------------------------------------------------
I N P U T                                                                *//*!
\param[in]  f_X_si32      pointer to the X value
\param[in]  f_Y_si        pointer to the Y value
\param[in]  f_scale_si8   The corresponding scale to the X and Y values
*//*--------------------------------------------------------------------------
O U T P U T                                                             *//*!
\param[out] f_X_si32      The X value, scaled so that X and Y fit into SInt16
\param[out] f_Y_si32      The X value, scaled so that X and Y fit into SInt16
\param[our] f_scale_si8   The precision reduction is subtracted from the scale value
*//*--------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                       *//*!
\author    Arwed Starke (str2lr)
*//**************************************************************************/
void g_mtl_ScaleDown2_si32_vd(UInt8 f_largestAllowedValueBit_ui8, SInt32 *f_X_si32,
                              SInt32 *f_Y_si32, SInt8 *f_scale_si8)
{
    UInt8 l_requiredShift_ui8 = f_largestAllowedValueBit_ui8;
    // do not test bits on 2-complement numbers, so take largest absolute value
    // first
    const UInt32 l_absXYmax_ui32 =
        (UInt32)g_mtl_Max_mac(g_mtl_Abs_mac(*f_X_si32), g_mtl_Abs_mac(*f_Y_si32));
    UInt8 l_i_ui8;
    // search for MSB (starting at largest allowed value bit)
    for (l_i_ui8 = f_largestAllowedValueBit_ui8; l_i_ui8 < 32; l_i_ui8++)
    {
        if ((l_absXYmax_ui32 & ((UInt32)1 << l_i_ui8)) != FALSE)
        {
            l_requiredShift_ui8 = l_i_ui8;
        }
    }
    l_requiredShift_ui8 -= f_largestAllowedValueBit_ui8; // if bit 15 is set, shift 1 bit
                                                         // left, and so on

    *f_X_si32 = g_mtl_ReducePrecision_si32(*f_X_si32, l_requiredShift_ui8);
    *f_Y_si32 = g_mtl_ReducePrecision_si32(*f_Y_si32, l_requiredShift_ui8);
    *f_scale_si8 -= (SInt8)l_requiredShift_ui8;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_ScaledAdd_si32                                      *//**
\brief         Adds 2 scaled numbers with overflow protection.
\details       Both numbers are required to have the same scaling before
               addition. If one of the numbers is 2^30 or higher, both numbers
               are divided by 2 to make sure no sign overflow occurs.
               The scaling is updated accordingly.
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_summand1_si32   summand 1
\param[in]     f_summand2_si32   summand 2
\param[in]     f_Scaling_psi8    the scaling of the value (e.g. 27 means
                                 that the normalized value is [value * 2^-27]
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\return        summand1 + summand2 (with scaling f_Scaling_psi8)
\param[out]    f_Scaling_psi8    updated scaling
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                      *//**
\author        str2lr
*//*************************************************************************/
SInt32 g_mtl_ScaledAdd_si32(SInt32 f_Summand1_si32, SInt32 f_Summand2_si32,
                            SInt8 *f_Scaling_psi8)
{
    if ((f_Summand1_si32 > 0x1fffffff) || (f_Summand1_si32 < -0x1fffffff) ||
        (f_Summand2_si32 > 0x1fffffff) ||
        (f_Summand2_si32 < -0x1fffffff)) // overflow Protection
    {
        f_Summand1_si32 /= 2;
        f_Summand2_si32 /= 2;

        *f_Scaling_psi8 = *f_Scaling_psi8 - 1;
    }

    return (f_Summand1_si32 + f_Summand2_si32);
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_AdjustScaling_vd                                   *//**
\brief         Adjust 2 pseudo float numbers so that their scaling is equal.
\details       Signs are preserved. Please make sure that the scaling values
               are correct before invoking this function.
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[out]    f_Value1_psi32     value 1
\param[out]    f_Scaling1_psi8    scaling of value 1 (e.g. 27 means
                                  that the normalized value is [value * 2^-27]
\param[out]    f_Value2_psi32     value 2
\param[out]    f_Scaling2_psi8    scaling of value 2
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    f_Value1_psi32     updated value. Max. value bits used: 31
                                  (max. scaled value of 2^31-1)
\param[out]    f_Scaling1_psi8    updated scaling. Scaling1 == Scaling2
\param[out]    f_Value2_psi32     updated value. Max. value bits used: 31
                                  (max. scaled value of 2^31-1)
\param[out]    f_Scaling2_psi8    updated scaling. Scaling1 == Scaling2
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                      *//**
\author        str2lr
*//*************************************************************************/
void g_mtl_AdjustScaling_vd(SInt32 *f_Value1_psi32, SInt8 *f_Scaling1_psi8,
                            SInt32 *f_Value2_psi32, SInt8 *f_Scaling2_psi8)
{
    // if value2 has the larger scaling, scale up value1 and/or scalue down value2
    while (*f_Scaling2_psi8 > *f_Scaling1_psi8)
    {
        if ((*f_Value1_psi32 < (SInt32)0x1fffffff) &&
            (*f_Value1_psi32 > -(SInt32)0x1fffffff))
        {
            *f_Value1_psi32 *= (SInt32)2;
            *f_Scaling1_psi8 += (SInt8)1;
        }
        else
        {
            *f_Value2_psi32 = g_mtl_s32_Div_s32_si32(*f_Value2_psi32, (SInt32)2);
            *f_Scaling2_psi8 -= (SInt8)1;
        }
    }
    // if value1 has the larger scaling initially, scale up value 2 and/or scale
    // down value 1 until scaling is equal
    while (*f_Scaling1_psi8 > *f_Scaling2_psi8)
    {
        if ((*f_Value2_psi32 < (SInt32)0x1fffffff) &&
            (*f_Value2_psi32 > -(SInt32)0x1fffffff))
        {
            *f_Value2_psi32 *= (SInt32)2;
            *f_Scaling2_psi8 += (SInt8)1;
        }
        else
        {
            *f_Value1_psi32 = g_mtl_s32_Div_s32_si32(*f_Value1_psi32, (SInt32)2);
            *f_Scaling1_psi8 -= (SInt8)1;
        }
    }
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_AngleAdd_ui32                                        *//**
\brief         safe addition of F22 angles
\details       -
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_Angle1_ui32   angle in interval [0�,360�) in 2^-22 [rad]
\param[in]     f_Angle2_ui32   angle in interval [0�,360�) in 2^-22 [rad]
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\return        (Angle1 + Angle2) in interval [0�,360�) in 2^-22 [rad]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                      *//**
\author        sce2lr
*//*************************************************************************/
UInt32 g_mtl_AngleAdd_ui32(UInt32 f_Angle1_ui32, UInt32 f_Angle2_ui32)
{
    UInt32 l_Returnangle_ui32;

    l_Returnangle_ui32 = f_Angle1_ui32 + f_Angle2_ui32;

    if (l_Returnangle_ui32 >= gd_MTL_2_PI_ui32)
    {
        l_Returnangle_ui32 -= gd_MTL_2_PI_ui32;
    }

    if (l_Returnangle_ui32 == gd_MTL_2_PI_ui32)
    {
        l_Returnangle_ui32 = 0;
    }

    return l_Returnangle_ui32;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_AngleSub_ui32                                      *//**
\brief         safe subtraction of F22 angles
\details       -
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_Angle1_ui32   angle in interval [0�,360�) in 2^-22 [rad]
\param[in]     f_Angle2_ui32   angle in interval [0�,360�) in 2^-22 [rad]
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\return        (Angle1 - Angle2) in interval [0�,360�) in 2^-22 [rad]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                      *//**
\author        sce2lr
*//*************************************************************************/
UInt32 g_mtl_AngleSub_ui32(UInt32 f_Angle1_ui32, UInt32 f_Angle2_ui32)
{
    UInt32 l_Returnangle_ui32;

    if (f_Angle1_ui32 >= f_Angle2_ui32)
    {
        l_Returnangle_ui32 = f_Angle1_ui32 - f_Angle2_ui32;
    }
    else
    {
        l_Returnangle_ui32 = gd_MTL_2_PI_ui32 - (f_Angle2_ui32 - f_Angle1_ui32);
    }

    if (l_Returnangle_ui32 == gd_MTL_2_PI_ui32)
    {
        l_Returnangle_ui32 = 0;
    }

    return l_Returnangle_ui32;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_AngleSub_si32                                      *//**
\brief         convert angle
\details       [detailed description, formula]
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_Angle1_ui32   angle [2^-22 rad] in range [0 .. 2*pi]
\param[in]     f_Angle2_ui32   angle [2^-22 rad] in range [0 .. 2*pi]
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        angle  [2^-22 rad] in range [-pi .. pi]
\throw         [error values]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        gra1lr
*//*************************************************************************/
SInt32 g_mtl_AngleSub_si32(UInt32 f_Angle1_ui32, UInt32 f_Angle2_ui32)
{
    SInt32 l_Returnangle_si32;

    if (f_Angle1_ui32 >= f_Angle2_ui32)
    {
        l_Returnangle_si32 = (SInt32)(f_Angle1_ui32 - f_Angle2_ui32);
    }
    else
    {
        l_Returnangle_si32 = (SInt32)(gd_MTL_2_PI_ui32 - (f_Angle2_ui32 - f_Angle1_ui32));
    }

    if (l_Returnangle_si32 > (SInt32)gd_MTL_PI_ui32)
    {
        l_Returnangle_si32 = l_Returnangle_si32 - (SInt32)gd_MTL_2_PI_ui32;
    }

    return l_Returnangle_si32;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_AngleAdd_si16                                      *//**
\brief         difference of two angles
\details       [detailed description, formula]
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_Angle1_si16   angle [2^-12 rad] in range [-pi .. pi]
\param[in]     f_Angle2_si16   angle [2^-12 rad] in range [-pi .. pi]
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        angle [2^-12 rad] in range [-pi .. pi]
\throw         [error values]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        gra1lr
*//*************************************************************************/
SInt16 g_mtl_AngleAdd_si16(SInt16 f_Angle1_si16, SInt16 f_Angle2_si16)
{
    UInt32 f_Angle1_ui32, f_Angle2_ui32;

    f_Angle1_ui32 = g_mtl_AngleF12ToF22_ui32(f_Angle1_si16);
    f_Angle2_ui32 = g_mtl_AngleF12ToF22_ui32(f_Angle2_si16);

    return g_mtl_AngleF22ToF12_si16(g_mtl_AngleAdd_ui32(f_Angle1_ui32, f_Angle2_ui32));
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_AngleSub_si16                                      *//**
\brief         difference of two angles
\details       [detailed description, formula]
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_Angle1_si16   angle [2^-12 rad] in range [-pi .. pi]
\param[in]     f_Angle2_si16   angle [2^-12 rad] in range [-pi .. pi]
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        angle [2^-12 rad] in range [-pi .. pi]
\throw         [error values]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        gra1lr
*//*************************************************************************/
SInt16 g_mtl_AngleSub_si16(SInt16 f_Angle1_si16, SInt16 f_Angle2_si16)
{
    UInt32 f_Angle1_ui32, f_Angle2_ui32;

    f_Angle1_ui32 = g_mtl_AngleF12ToF22_ui32(f_Angle1_si16);
    f_Angle2_ui32 = g_mtl_AngleF12ToF22_ui32(f_Angle2_si16);

    return g_mtl_AngleF22ToF12_si16(g_mtl_AngleSub_ui32(f_Angle1_ui32, f_Angle2_ui32));
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_AngleF12ToF22_ui32                                 *//**
\brief         convert angle
\details       [detailed description, formula]
*//*-------------------------------------------------------------------------
I N P U T                                                              *//**
\param[in]     f_Angle_si16    angle [2^-12 rad]
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\return        angle [2^-22 rad] in range [0 .. 2*pi]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        sce2lr
*//*************************************************************************/
UInt32 g_mtl_AngleF12ToF22_ui32(SInt16 f_Angle_si16)
{
    UInt32 l_ReturnAngle_ui32;

    if (f_Angle_si16 >= 0)
    {
        l_ReturnAngle_ui32 = (UInt32)f_Angle_si16 << 10;
    }
    else
    {
        l_ReturnAngle_ui32 = ((UInt32)(-f_Angle_si16)) << 10;
        l_ReturnAngle_ui32 = g_mtl_AngleSub_ui32(gd_MTL_2_PI_ui32, l_ReturnAngle_ui32);
    }

    return l_ReturnAngle_ui32;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_AngleF22ToF12_si16                                 *//**
\brief         convert angle
\details       [detailed description, formula]
*//*-------------------------------------------------------------------------
I N P U T                                                              *//**
\param[in]     f_Angle_ui32   angle [2^-22 rad] in range [0 .. 2*pi]
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        angle  [2^-12 rad] in range [-pi .. pi]
\throw         [error values]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        sce2lr
*//*************************************************************************/
SInt16 g_mtl_AngleF22ToF12_si16(UInt32 f_Angle_ui32)
{
    SInt16 l_ReturnAngle_si16;

    if (f_Angle_ui32 < gd_MTL_PI_ui32)
    {
        l_ReturnAngle_si16 = (SInt16)(f_Angle_ui32 >> 10);
    }
    else
    {
        const UInt32 l_AngleTemp_ui32 =
            g_mtl_AngleSub_ui32(gd_MTL_2_PI_ui32, f_Angle_ui32);
        l_ReturnAngle_si16 = -(SInt16)(l_AngleTemp_ui32 >> 10);
    }

    return l_ReturnAngle_si16;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_NormalizeAngle_si16                                *//**
\brief         normalize angle [F12] to value range [-pi,+pi)
\details       -
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_Angle_si16   angle in interval [-2.5*pi,2.5*pi) in 2^-12 [rad]
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\return        angle in interval [-pi,+pi) in 2^-12 [rad]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                      *//**
\author        str2lr
*//*************************************************************************/
SInt16 g_mtl_NormalizeAngle_si16(SInt16 f_Angle_si16)
{
    if (f_Angle_si16 >= (SInt16)gd_MTL_PI_ui16)
    {
        f_Angle_si16 -= (SInt16)gd_MTL_2_PI_ui16;
    }
    if (f_Angle_si16 < -(SInt16)gd_MTL_PI_ui16)
    {
        f_Angle_si16 += (SInt16)gd_MTL_2_PI_ui16;
    }

    return f_Angle_si16;
}

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
------------------------------------------------------------------------------
Function name g_mtl_SinCosMul_si16                                      *//*!
\brief      multiplication with sin/cos results
*//*--------------------------------------------------------------------------
I N P U T                                                                *//*!
\param[in]  f_Multiplier_si16  Value to multiplicate
\param[in]  f_SinCos_si16     Sin or Cos value (F14)
*//*--------------------------------------------------------------------------
O U T P U T                                                              *//*!
\return     multiplicated value (same scale as f_Value_si16)
*//*--------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                       *//*!
\author   sce2lr
*//**************************************************************************/
SInt16 g_mtl_SinCosMul_si16(const SInt16 f_Multiplier_si16, const SInt16 f_SinCos_si16)
{
    SInt32 l_retval_si32;

    l_retval_si32 = ((SInt32)f_Multiplier_si16) * ((SInt32)f_SinCos_si16); // Fxx * F14
    l_retval_si32 = g_mtl_ReducePrecision_si32(l_retval_si32, (UInt8)14);  // Fxx

    return (SInt16)l_retval_si32;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_AngleMul_si16                                      *//**
\brief         Multiply a length with an angle, e.g. to calculate an arc length
               S = phi * R.
\details
*//*-------------------------------------------------------------------------
I N P U T                                                              *//**
\param[in]     f_Angle_si16    angle [2^-12 rad]
\param[in]     f_length_si16   length (arbitrary scale)
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\return        arc length in scale of f_length_si16
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        str2lr
*//*************************************************************************/
SInt32 g_mtl_AngleMul_si32(SInt16 f_Angle_si16, SInt16 f_Length_si16)
{
    SInt32 l_Result_si32;

    // Overflow possible, even for range [-pi;pi): 12867 * (2^15-1) / 2^12 =
    // 102932 > 65535.
    l_Result_si32 = f_Angle_si16 * f_Length_si16;                  // InScale * F12
    l_Result_si32 = g_mtl_ReducePrecision_si32(l_Result_si32, 12); // InScale

    return l_Result_si32;
}

/************************************************************************/
/* Angle functions                                                      */
/************************************************************************/

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
------------------------------------------------------------------------------
Function name     g_mtlAngle_Add_vd                                      *//*!
\brief      Angle addition. result = a + b
\details
Sine/Cosine of result is calculated by addition theorem.
\verbatim
sin(a + b) = sin a * cos b + cos a * sin b
cos(a + b) = cos a * cos b - sin a * sin b
\endverbatim
The function is overflow-safe.
The function can be used like this: g_mtlAngle_Add_vd(a, a, b);
*//*--------------------------------------------------------------------------
I N P U T                                                                *//*!
\param[in]  a_pst
\param[in]  b_pst
*//*--------------------------------------------------------------------------
O U T P U T                                                              *//*!
\param[out]  (a + b). Angle, Sine and cosine of Result are set. The angle is normalized.
*//*--------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                       *//*!
\author   str2lr
*//**************************************************************************/
void g_mtlAngle_Add_vd( // result = a + b
    gType_mtlAngle_st *f_Result_pst, gType_mtlAngle_st const *f_A_pst,
    gType_mtlAngle_st const *f_B_pst)
{
    SInt32 l_temp_si32;
    SInt16 l_SinResult_si16;

    // F14 * F14 + F14 * F14 = F28
    // Overflow-Betrachtung: mathematisch gesehen sollten sin/cos immer <= 1
    // bleiben, ansonsten war schon vorher was falsch
    l_temp_si32 =
        (f_A_pst->Sin_si16 * f_B_pst->Cos_si16) + (f_A_pst->Cos_si16 * f_B_pst->Sin_si16);
    l_SinResult_si16 = (SInt16)g_mtl_ReducePrecision_si32(l_temp_si32, 14); // F14

    l_temp_si32 =
        (f_A_pst->Cos_si16 * f_B_pst->Cos_si16) - (f_A_pst->Sin_si16 * f_B_pst->Sin_si16);
    f_Result_pst->Cos_si16 = (SInt16)g_mtl_ReducePrecision_si32(l_temp_si32, 14); // F14
    f_Result_pst->Sin_si16 = l_SinResult_si16; // zuweisung erst hier, damit a =
                                               // a + b auch funktioniert

    f_Result_pst->Angle_si16 =
        g_mtl_NormalizeAngle_si16(f_A_pst->Angle_si16 + f_B_pst->Angle_si16);
}

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
------------------------------------------------------------------------------
Function name     g_mtlAngle_Sub_vd                                      *//*!
\brief      Angle subtraction. result = a - b
\details
Sine/Cosine of result is calculated by addition theorem.
\verbatim
sin(a - b) = sin a * cos b - cos a * sin b
cos(a - b) = cos a * cos b + sin a * sin b
\endverbatim
The function is overflow-safe.
The function can be used like this: g_mtlAngle_Sub_vd(a, a, b);
*//*--------------------------------------------------------------------------
I N P U T                                                                *//*!
\param[in]  f_A_pst
\param[in]  f_B_pst
*//*--------------------------------------------------------------------------
O U T P U T                                                              *//*!
\param[out]  (a - b). Angle, Sine and cosine of Result are set. The angle is normalized.
*//*--------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                       *//*!
\author   str2lr
*//**************************************************************************/
void g_mtlAngle_Sub_vd( // result = a - b
    gType_mtlAngle_st *f_Result_pst, gType_mtlAngle_st const *f_A_pst,
    gType_mtlAngle_st const *f_B_pst)
{
    SInt32 l_temp_si32;
    SInt16 l_SinResult_si16;

    // F14 * F14 + F14 * F14 = F28
    // Overflow-Betrachtung: mathematisch gesehen sollten sin/cos immer <= 1
    // bleiben, ansonsten war schon vorher was falsch
    l_temp_si32 = ((SInt32)f_A_pst->Sin_si16 * f_B_pst->Cos_si16) -
                  ((SInt32)f_A_pst->Cos_si16 * f_B_pst->Sin_si16);
    l_SinResult_si16 = (SInt16)g_mtl_ReducePrecision_si32(l_temp_si32, 14); // F14

    l_temp_si32 = ((SInt32)f_A_pst->Cos_si16 * f_B_pst->Cos_si16) +
                  ((SInt32)f_A_pst->Sin_si16 * f_B_pst->Sin_si16);
    f_Result_pst->Cos_si16 = (SInt16)g_mtl_ReducePrecision_si32(l_temp_si32, 14); // F14
    f_Result_pst->Sin_si16 = l_SinResult_si16; // zuweisung erst hier, damit a =
                                               // a + b auch funktioniert

    f_Result_pst->Angle_si16 =
        g_mtl_NormalizeAngle_si16(f_A_pst->Angle_si16 - f_B_pst->Angle_si16);
}

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
------------------------------------------------------------------------------
Function name     g_mtlAngle_Neg_vd                                      *//*!
\brief      Angle negation.  angle = -angle
\details
The function is overflow-safe.
*//*--------------------------------------------------------------------------
I N P U T                                                                *//*!
\param[in]   f_angle_pst
*//*--------------------------------------------------------------------------
O U T P U T                                                              *//*!
\param[out]  f_angle_pst. Angle, Sine and cosine of Result are set.
*//*--------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                       *//*!
\author   str2lr
*//**************************************************************************/
void g_mtlAngle_Neg_vd(gType_mtlAngle_st *f_Angle_pst)
{
    f_Angle_pst->Angle_si16 = -f_Angle_pst->Angle_si16;
    f_Angle_pst->Sin_si16   = -f_Angle_pst->Sin_si16;
}

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
------------------------------------------------------------------------------
Function name     g_mtlAngle_Set_vd                                      *//*!
\brief     Angle initialization. angle := alpha
\details   -
*//*--------------------------------------------------------------------------
I N P U T                                                                *//*!
\param[in]  f_alpha_si16   angle [F12 rad]
*//*--------------------------------------------------------------------------
O U T P U T                                                              *//*!
\param[out]  f_angle_pst. Angle, Sine and cosine of Result are set. The angle is normalized.
*//*--------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                       *//*!
\author   str2lr
*//**************************************************************************/
void g_mtlAngle_Set_vd(gType_mtlAngle_st *f_Angle_pst, SInt16 f_Alpha_si16)
{
    f_Angle_pst->Angle_si16 = g_mtl_NormalizeAngle_si16(f_Alpha_si16);
    f_Angle_pst->Cos_si16   = g_mtl_Cos_si16(f_Angle_pst->Angle_si16);
    f_Angle_pst->Sin_si16   = g_mtl_Sin_si16(f_Angle_pst->Angle_si16);
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_CalcCathetus_ui16                                  *//**
\brief         This function calculates b = sqrt(c^2 - a^2), which
               can be used to calculate the 2nd cathethus of a right triangle.
\details

Since the calculation of sqrt(c^2 - a^2) is not overflow-safe, this function
calculates sqrt((c + a) * (c - a)) and is overflow-safe.

\verbatim
                     ____________
 Pythagoras: b :=  \/ c^2 - a^2

\endverbatim

Unit Test:
\see m_Test_CalcCathethus_ui8()
*//*-------------------------------------------------------------------------
I N P U T                                                              *//**
\param[in]    f_Hypothenuse_ui16    Side c. Precondition: hypothenuse > kat
\param[in]    f_Cathetus_ui16       Side a
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\return       cathetus 2 (b) or 0 if the precondition was violated. [Output Scale = Input Scale]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        str2lr            Arwed Starke          AE-BE/EPM3
*//*************************************************************************/
UInt16 g_mtl_CalcCathetus_ui16(UInt16 f_Hypothenuse_ui16, UInt16 f_Cathetus_ui16)
{
    UInt32 l_kat2_ui32 = 0;

    // we assume that hyp > kat, so hyp^2 - kat^2 must be > 0!
    if (f_Cathetus_ui16 < f_Hypothenuse_ui16)
    {
        // sqrt(c^2 - a^2) = sqrt((c - a)*(c + a))
        l_kat2_ui32 = (UInt32)(f_Hypothenuse_ui16 + f_Cathetus_ui16);
        l_kat2_ui32 *= (UInt32)(f_Hypothenuse_ui16 - f_Cathetus_ui16); // InScale^2
    }

    return g_mtl_Sqrt_u32_ui16(l_kat2_ui32); // InScale
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_ExteriorProduct_si32                                  *//**
\brief         calculate the exterior product of 2-Element vectors
               (area of parallelogram spanned by the 2 vectors)
\details
The exterior product is defined as:

\f[
               v_1 \wedge v_2 = (x_1 * y_2) - (x_2 * y_1)
\f]

So it's the "cross product" of vectors in R2. The return value is the
area of the parallelogram described by the two vectors.
The sign of the returned value is the sign of the angle theta between the vectors.

               http://de.wikipedia.org/wiki/Kreuzprodukt

\ingroup
*//*-------------------------------------------------------------------------
I N P U T                                                              *//**
\param[in]     f_Vector1_pst
\param[in]     f_Vector2_pst
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        area of parallelogram between the two vectors [InputScale^2]
               |v1| * |v2| * sin(theta)
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        gra1lr
*//*************************************************************************/
SInt32 g_mtl_ExteriorProduct_si32(gType_mtlPoint_st const *f_Vector1_pst,
                                  gType_mtlPoint_st const *f_Vector2_pst)
{
    SInt32 l_retval_si32;

    l_retval_si32 = ((SInt32)f_Vector1_pst->X_si16 * (SInt32)f_Vector2_pst->Y_si16) -
                    ((SInt32)f_Vector2_pst->X_si16 * (SInt32)f_Vector1_pst->Y_si16);

    return l_retval_si32;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_ExteriorProduct_si32                                  *//**
\brief         calculate the exterior product of 2-Element vectors
               (area of parallelogram spanned by the 2 vectors)
\details
The exterior product is defined as:

\f[
               v_1 \wedge v_2 = (x_1 * y_2) - (x_2 * y_1)
\f]

So it's the "cross product" of vectors in R2. The return value is the
area of the parallelogram described by the two vectors.
The sign of the returned value is the sign of the angle theta between the vectors.

               http://de.wikipedia.org/wiki/Kreuzprodukt

\ingroup
*//*-------------------------------------------------------------------------
I N P U T                                                              *//**
\param[in]     f_Vector1_pst
\param[in]     f_Vector2_pst
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        area of parallelogram between the two vectors [InputScale^2]
               |v1| * |v2| * sin(theta)
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        gra1lr
*//*************************************************************************/
SInt32 g_mtl_ExteriorProductExtend_si32(gType_mtlPointPos_st const *f_Vector1_pst,
                                        gType_mtlPointPos_st const *f_Vector2_pst)
{
    SInt32 l_retval_si32;

    l_retval_si32 = ((SInt32)f_Vector1_pst->X_si32 * (SInt32)f_Vector2_pst->Y_si32) -
                    ((SInt32)f_Vector2_pst->X_si32 * (SInt32)f_Vector1_pst->Y_si32);

    return l_retval_si32;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_PointCCW_si32                                      *//**
\brief         Determine turn direction between two vectors given by three points.
               CCW stands for counter-clockwise.
\details       Determine CCW value from vectors \f$\vec{AB}\f$ and \f$\vec{AP}\f$
               given three points $A$, $B$, and $P$ in the following order:

\verbatim
                         o C
                        /
                       /
                      /
                     /
                    /
                A  o----------o B
\endverbatim

               The vectors are determined using the formula
\f{eqnarray*}{
               dx_{AB} & = & (x_B - x_A) \\
               dy_{AB} & = & (y_B - y_A) \\
               dx_{AC} & = & (x_C - x_A) \\
               dy_{AC} & = & (y_C - y_A)
\f}
               and then the exterior product is calculated:
               retVal = AB x AC

NOTE: Slightly different formula, but same result as g_od_mtl_PointLeftToVector_bl
*//*-------------------------------------------------------------------------
I N P U T                                                              *//**
\param[in]     f_A_pst     start point of vertex (arbitrary scaling)
\param[in]     f_B_pst     end point of vertex (arbitrary scaling)
\param[in]     f_C_pst     point C (arbitrary scaling)
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        exterior product (AB x AC)
               > 0 if vector C is left of vector AB (vector AC is counter-clockwise to AB)
               = 0 C is collinear
               < 0 if C is right of vector AB
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        gra1lr
*//*************************************************************************/
SInt32 g_mtl_PointCCW_si32(gType_mtlPoint_st const *f_A_pst,
                           gType_mtlPoint_st const *f_B_pst,
                           gType_mtlPoint_st const *f_C_pst)
{
    gType_mtlPoint_st l_AB_st, l_AC_st;
    SInt32 l_retval_si32;

    l_AB_st.X_si16 = f_B_pst->X_si16 - f_A_pst->X_si16;
    l_AB_st.Y_si16 = f_B_pst->Y_si16 - f_A_pst->Y_si16;
    l_AC_st.X_si16 = f_C_pst->X_si16 - f_A_pst->X_si16;
    l_AC_st.Y_si16 = f_C_pst->Y_si16 - f_A_pst->Y_si16;

    l_retval_si32 = g_mtl_ExteriorProduct_si32(&l_AB_st, &l_AC_st);

    return l_retval_si32;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_PointCCW_si32                                      *//**
\brief         Determine turn direction between two vectors given by three points.
               CCW stands for counter-clockwise.
\details       Determine CCW value from vectors \f$\vec{AB}\f$ and \f$\vec{AP}\f$
               given three points $A$, $B$, and $P$ in the following order:

\verbatim
                         o C
                        /
                       /
                      /
                     /
                    /
                A  o----------o B
\endverbatim

               The vectors are determined using the formula
\f{eqnarray*}{
               dx_{AB} & = & (x_B - x_A) \\
               dy_{AB} & = & (y_B - y_A) \\
               dx_{AC} & = & (x_C - x_A) \\
               dy_{AC} & = & (y_C - y_A)
\f}
               and then the exterior product is calculated:
               retVal = AB x AC

NOTE: Slightly different formula, but same result as g_od_mtl_PointLeftToVector_bl
*//*-------------------------------------------------------------------------
I N P U T                                                              *//**
\param[in]     f_A_pst     start point of vertex (arbitrary scaling)
\param[in]     f_B_pst     end point of vertex (arbitrary scaling)
\param[in]     f_C_pst     point C (arbitrary scaling)
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        exterior product (AB x AC)
               > 0 if vector C is left of vector AB (vector AC is counter-clockwise to AB)
               = 0 C is collinear
               < 0 if C is right of vector AB
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        gra1lr
*//*************************************************************************/
SInt32 g_mtl_PointCCWExtend_si32(gType_mtlPoint_st const *f_A_pst,
                                 gType_mtlPoint_st const *f_B_pst,
                                 gType_mtlPoint_st const *f_C_pst)
{
    gType_mtlPointPos_st l_AB_st, l_AC_st;
    SInt32 l_retval_si32;

    l_AB_st.X_si32 = (SInt32)f_B_pst->X_si16 - (SInt32)f_A_pst->X_si16;
    l_AB_st.Y_si32 = (SInt32)f_B_pst->Y_si16 - (SInt32)f_A_pst->Y_si16;
    l_AC_st.X_si32 = (SInt32)f_C_pst->X_si16 - (SInt32)f_A_pst->X_si16;
    l_AC_st.Y_si32 = (SInt32)f_C_pst->Y_si16 - (SInt32)f_A_pst->Y_si16;

    l_retval_si32 = g_mtl_ExteriorProductExtend_si32(&l_AB_st, &l_AC_st);

    return l_retval_si32;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_PointInsideConvexPolygon_bl                        *//**
\brief         Determine whether a point is inside a convex polygon.
\details
The function checks whether the point is left of (or right of) all edges of the polygon.
points on the contour (or points that are vertices of the polygon) are counted as inside.

Computational cost: O(n), 5 subtractions, 2 multiplications per edge

\see also: g_appasPointInsideDisplayArea_bl()
*//*-------------------------------------------------------------------------
I N P U T                                                              *//**
\param[in]     f_P_pst            point to be tested (arbitrary scaling)
\param[in]     f_Vertices_pst     The vertices of the polygon are
      passed as an array of point structs. The vertices must be ordered either
      counter-clockwise or clockwise. The pointer must point to an array of
      at least f_numVertices_ui8 elements. Vertices must not be repeated.
      Points can have arbitrary scaling.
\param[in]     f_numVertices_ui8  number of vertices of the polygon (range 3 ... 255).
               3 for a triangle. 4 for a rectangle, etc.
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        TRUE if point is inside the polygon, FALSE if it is outside.
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        str2lr
*//*************************************************************************/
Boolean g_mtl_PointInsideConvexPolygon_bl(gType_mtlPoint_st const *f_P_pst,
                                          gType_mtlPoint_st const *f_Vertices_pst,
                                          UInt8 f_numVertices_ui8)
{
    UInt8 l_vertex_ui8;
    Boolean l_allLeft_bl  = TRUE;
    Boolean l_allRight_bl = TRUE;
    SInt32 l_ExteriorProduct_si32;

    for (l_vertex_ui8 = 0; l_vertex_ui8 < f_numVertices_ui8; l_vertex_ui8++)
    {
        l_ExteriorProduct_si32 = g_mtl_PointCCW_si32(
            &f_Vertices_pst[l_vertex_ui8],
            &f_Vertices_pst[(l_vertex_ui8 + 1) % f_numVertices_ui8], f_P_pst);

        // if vertices are ordered counter-clockwise, P is left of all vectors,
        // otherwise right of all vectors the equal sign means that points on the
        // contour (or points that are vertices of the polygon) are counted as
        // inside
        l_allLeft_bl &= (l_ExteriorProduct_si32 >= 0);
        l_allRight_bl &= (l_ExteriorProduct_si32 <= 0);
    }

    return (Boolean)(l_allLeft_bl || l_allRight_bl);
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_PointInsideConvexPolygon_bl                        *//**
\brief         Determine whether a point is inside a convex polygon.
\details
The function checks whether the point is left of (or right of) all edges of the polygon.
points on the contour (or points that are vertices of the polygon) are counted as inside.

Computational cost: O(n), 5 subtractions, 2 multiplications per edge

\see also: g_appasPointInsideDisplayArea_bl()
*//*-------------------------------------------------------------------------
I N P U T                                                              *//**
\param[in]     f_P_pst            point to be tested (arbitrary scaling)
\param[in]     f_Vertices_pst     The vertices of the polygon are
      passed as an array of point structs. The vertices must be ordered either
      counter-clockwise or clockwise. The pointer must point to an array of
      at least f_numVertices_ui8 elements. Vertices must not be repeated.
      Points can have arbitrary scaling.
\param[in]     f_numVertices_ui8  number of vertices of the polygon (range 3 ... 255).
               3 for a triangle. 4 for a rectangle, etc.
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        TRUE if point is inside the polygon, FALSE if it is outside.
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        str2lr
*//*************************************************************************/
Boolean g_mtl_PointInsideConvexPolygonExtend_bl(gType_mtlPoint_st const *f_P_pst,
                                                gType_mtlPoint_st const *f_Vertices_pst,
                                                UInt8 f_numVertices_ui8)
{
    UInt8 l_vertex_ui8;
    Boolean l_allLeft_bl  = TRUE;
    Boolean l_allRight_bl = TRUE;
    SInt32 l_ExteriorProduct_si32;

    for (l_vertex_ui8 = 0; l_vertex_ui8 < f_numVertices_ui8; l_vertex_ui8++)
    {
        l_ExteriorProduct_si32 = g_mtl_PointCCWExtend_si32(
            &f_Vertices_pst[l_vertex_ui8],
            &f_Vertices_pst[(l_vertex_ui8 + 1) % f_numVertices_ui8], f_P_pst);

        // if vertices are ordered counter-clockwise, P is left of all vectors,
        // otherwise right of all vectors the equal sign means that points on the
        // contour (or points that are vertices of the polygon) are counted as
        // inside
        l_allLeft_bl &= (l_ExteriorProduct_si32 >= 0);
        l_allRight_bl &= (l_ExteriorProduct_si32 <= 0);
    }

    return (Boolean)(l_allLeft_bl || l_allRight_bl);
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_GetAngularFromThreePoints_ui32                      *//**
\brief         Return the smaller angle which is made by three points
               (BITSHIFT is necessary before usage).
\details

\verbatim
                         o P3
                        /
                       /
                      /
                     /
                    / phi
                P2 o----------o P1

\endverbatim

First, the vector A = P2 - P1 and  B = P2 - P3 are built (could also
be P1 - P2, doesn't matter). Then, the angle between the vectors can
be calculated by the arctan function. First, the vectors are rotated such
that P12 lies on the X-Axis. This is achieved by using the rotation matrix

\verbatim
|  A_x  A_y|
| -A_y  A_x|
\endverbatim

After vector B is rotated, only arctan2(B_y, B_x) has to be calculated.
Vector A can be ignored since the arcus tangens of a vector lying on the X-axi
is zero. The Rotation can be calculated as:

\verbatim
B_x = A * B (inner product)
B_y = A x B (cross product)
\endverbatim

Please verify this yourself. The idea for this calculation is from:

http://stackoverflow.com/questions/3486172/angle-between-3-points

This function can be used with arbitrary input value scaling (cm, mm, F10 m)

Unit Test:
\see UT_GetAngleBetweenThreePoints.m

*//*-------------------------------------------------------------------------
I N P U T	                                                            *//**
\param[in]     f_p1_pst      first point   (cm, mm, F10 m)
\param[in]     f_p2_pst      center point  (cm, mm, F10 m)
\param[in]     f_p3_pst      last point    (cm, mm, F10 m)
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        Angle [F12 rad]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        Arwed Starke (str2lr)
*//*************************************************************************/
SInt16 g_mtl_GetAngleBetweenThreePoints_si16(gType_mtlPoint_st const *f_p1_pst,
                                             gType_mtlPoint_st const *f_p2_pst,
                                             gType_mtlPoint_st const *f_p3_pst)
{
    // return the angle of P1_P2 <-> P2_P3
    SInt16 l_phi_si16;
    SInt32 l_dy_si32;
    SInt32 l_dx_si32;
    SInt8 l_scale_si8; // unused
    gType_mtlPoint_st l_v1_st, l_v2_st;

    // overflow protection: Division by 2 may be necessary (does not influence
    // result)
    l_dx_si32      = (SInt32)f_p2_pst->X_si16 - (SInt32)f_p1_pst->X_si16;
    l_dy_si32      = (SInt32)f_p2_pst->Y_si16 - (SInt32)f_p1_pst->Y_si16;
    l_v1_st.X_si16 = (SInt16)l_dx_si32;
    l_v1_st.Y_si16 = (SInt16)l_dy_si32;
    if (((SInt32)l_v1_st.X_si16 != l_dx_si32) || ((SInt32)l_v1_st.Y_si16 != l_dy_si32))
    {
        l_v1_st.X_si16 = (SInt16)(l_dx_si32 / 2);
        l_v1_st.Y_si16 = (SInt16)(l_dy_si32 / 2);
    }

    l_dx_si32      = (SInt32)f_p2_pst->X_si16 - (SInt32)f_p3_pst->X_si16;
    l_dy_si32      = (SInt32)f_p2_pst->Y_si16 - (SInt32)f_p3_pst->Y_si16;
    l_v2_st.X_si16 = (SInt16)l_dx_si32;
    l_v2_st.Y_si16 = (SInt16)l_dy_si32;
    if (((SInt32)l_v2_st.X_si16 != l_dx_si32) || ((SInt32)l_v2_st.Y_si16 != l_dy_si32))
    {
        l_v2_st.X_si16 = (SInt16)(l_dx_si32 / 2);
        l_v2_st.Y_si16 = (SInt16)(l_dy_si32 / 2);
    }

    // rotate v2 by the amount needed to rotate v1 to the x-axis (in an efficient
    // way)
    l_dx_si32 = g_mtl_InnerProduct_si32(&l_v1_st, &l_v2_st);
    l_dy_si32 = g_mtl_ExteriorProduct_si32(&l_v1_st, &l_v2_st);
    // overflow protection
    g_mtl_ScaleDown2_si32_toFit_si16_vd(&l_dx_si32, &l_dy_si32, &l_scale_si8);

    l_phi_si16 = g_mtl_ArcTan_Approx_si16((SInt16)l_dy_si32, (SInt16)l_dx_si32, 3); // F12
    return l_phi_si16;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_LineIntersection_bl                                *//**
\brief         This function calculates the intersection point of two straight
               lines using Cramer's in order to avoid numerical problems.
\details       Additionally the lengths between to given points to the
               intersection will be calculated.

\verbatim
                       /
                      /
                     /
                    /
         o---------o------------
        P_1       /  P (X, Y)
                 /
                /
               /
              o P_2
\endverbatim

 For further information read DOC\Geolib_Design.docx

 For information about Cramer's rule see:
 http://en.wikipedia.org/wiki/Cramer's_rule

Unit Test:
\see m_Test_LineIntersection_ui8() and UT_LineIntersection.m

*//*-------------------------------------------------------------------------
I N P U T                                                              *//**
\param[in]     f_Line1_pst   First line.
               Scaling of X, Y values can be anything from cm up to F10 [2^-10 m].
         SinPsi and CosPsi have to be set!
\param[in]     f_Line2_pst  Second line.
               Scaling of X, Y values can be anything from cm up to F10 [2^-10 m].
         SinPsi and CosPsi have to be set!
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    f_Length1_si16 Distance between given point of the first line
               and point of intersection. The length is negative if the intersection lies in opposite
         direction of Pos1. Psi on the line given by Pos1 and positive
         otherwise. The scaling equals the input scaling.
\param[out]    f_Length2_si16   length from 2nd position to intersection.
               The length is negative if the intersection lies in opposite
         direction of Pos2. Psi on the line given by Pos1 and positive
         otherwise. The scaling equals the input scaling.
         NOTE: Length1 and Length2 are not guaranteed to be exact (error up to 6 mm was observed).
\param[out]    f_PIntersect_pst  intersection point. The scaling equals the input scaling.
\return        True if lines intersect and FALSE if lines are (almost) parallel.
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        sce2lr, str2lr, gra1lr
*//*************************************************************************/
Boolean g_mtl_LineIntersection_bl(gType_mtlLineSinCos_st const *f_Line1_pst,
                                  gType_mtlLineSinCos_st const *f_Line2_pst,
                                  SInt16 *f_Length1_si16, SInt16 *f_Length2_si16,
                                  gType_mtlPoint_st *f_P_pst)
{
    SInt32 l_DeterminanteF12_si32;
    SInt32 l_Nebendeterminante1_si32;
    SInt32 l_Nebendeterminante2_si32;
    SInt32 l_TempX1_si32;
    SInt32 l_TempX2_si32;
    SInt32 l_TempY1_si32;
    SInt32 l_TempY2_si32;
    SInt32 l_Temp_si32;
    Boolean l_Return_bl;

    l_DeterminanteF12_si32 =
        ((SInt32)f_Line1_pst->Phi_st.Sin_si16 * (SInt32)f_Line2_pst->Phi_st.Cos_si16) -
        ((SInt32)f_Line1_pst->Phi_st.Cos_si16 *
         (SInt32)f_Line2_pst->Phi_st.Sin_si16); // F14*F14 = F28

    l_Nebendeterminante1_si32 =
        (SInt32)g_mtl_SinCosMul_si16(f_Line1_pst->P_st.Y_si16 - f_Line2_pst->P_st.Y_si16,
                                     f_Line2_pst->Phi_st.Cos_si16) -
        (SInt32)g_mtl_SinCosMul_si16(f_Line1_pst->P_st.X_si16 - f_Line2_pst->P_st.X_si16,
                                     f_Line2_pst->Phi_st.Sin_si16);

    l_Nebendeterminante2_si32 =
        (SInt32)g_mtl_SinCosMul_si16(f_Line1_pst->P_st.Y_si16 - f_Line2_pst->P_st.Y_si16,
                                     f_Line1_pst->Phi_st.Cos_si16) -
        (SInt32)g_mtl_SinCosMul_si16(f_Line1_pst->P_st.X_si16 - f_Line2_pst->P_st.X_si16,
                                     f_Line1_pst->Phi_st.Sin_si16);

    l_DeterminanteF12_si32 /= 16384;

    if (l_DeterminanteF12_si32 != 0)
    {
        l_Return_bl = TRUE;

        // compute length of first segment  // InScale * F14 / F14 = InScale
        l_Temp_si32 = -g_mtl_s32_Div_s32_si32((l_Nebendeterminante1_si32 * (SInt32)16384),
                                              l_DeterminanteF12_si32);

        // �berlauf �berpr�fen
        if (g_mtl_Abs_mac(l_Temp_si32) > (SInt32)GD_MAX_SI16)
        {
            // Wir setzen daas maximum als beste N�hrung
            l_Temp_si32 = (SInt32)GD_MAX_SI16 * g_mtl_Signum_mac(l_Temp_si32);
            l_Return_bl = FALSE;
        }
        *f_Length1_si16 = (SInt16)l_Temp_si32; // InScale

        // InScale * F14 / F14 = InScale
        l_Temp_si32 = -g_mtl_s32_Div_s32_si32((l_Nebendeterminante2_si32 * (SInt32)16384),
                                              l_DeterminanteF12_si32);

        // �berlauf �berpr�fen
        if (g_mtl_Abs_mac(l_Temp_si32) > (SInt32)GD_MAX_SI16)
        {
            // Wir setzen daas maximum als beste N�hrung
            l_Temp_si32 = (SInt32)GD_MAX_SI16 * g_mtl_Signum_mac(l_Temp_si32);
            l_Return_bl = FALSE;
        }

        *f_Length2_si16 = (SInt16)l_Temp_si32; // InScale

        // Wir berechnen den Schnittpunkt
        // Zur Fehlerminimierung �ber 2 Wege mit anschliessendem Durchschnitt.
        l_TempX1_si32 =
            (SInt32)f_Line1_pst->P_st.X_si16 +
            (SInt32)g_mtl_SinCosMul_si16(*f_Length1_si16, f_Line1_pst->Phi_st.Cos_si16);
        l_TempY1_si32 =
            (SInt32)f_Line1_pst->P_st.Y_si16 +
            (SInt32)g_mtl_SinCosMul_si16(*f_Length1_si16, f_Line1_pst->Phi_st.Sin_si16);
        l_TempX2_si32 =
            (SInt32)f_Line2_pst->P_st.X_si16 +
            (SInt32)g_mtl_SinCosMul_si16(*f_Length2_si16, f_Line2_pst->Phi_st.Cos_si16);
        l_TempY2_si32 =
            (SInt32)f_Line2_pst->P_st.Y_si16 +
            (SInt32)g_mtl_SinCosMul_si16(*f_Length2_si16, f_Line2_pst->Phi_st.Sin_si16);

        l_Temp_si32 = ((l_TempX1_si32 + l_TempX2_si32) / 2);

        // �berlauf �berpr�fen
        if (g_mtl_Abs_mac(l_Temp_si32) > (SInt32)GD_MAX_SI16)
        {
            // Wir setzen daas maximum als beste N�hrung
            l_Temp_si32 = (SInt32)GD_MAX_SI16 * g_mtl_Signum_mac(l_Temp_si32);
            l_Return_bl = FALSE;
        }
        f_P_pst->X_si16 = (SInt16)l_Temp_si32;

        l_Temp_si32 = ((l_TempY1_si32 + l_TempY2_si32) / 2);

        // �berlauf �berpr�fen
        if (g_mtl_Abs_mac(l_Temp_si32) > (SInt32)GD_MAX_SI16)
        {
            // Wir setzen das Maximum als beste N�hrung
            l_Temp_si32 = (SInt32)GD_MAX_SI16 * g_mtl_Signum_mac(l_Temp_si32);
            l_Return_bl = FALSE;
        }

        f_P_pst->Y_si16 = (SInt16)l_Temp_si32;
    }
    else
    {
        // Geraden sind Parallel dieser Fall wird nicht unterst�tzt !
        // Fuer failsave cases wird der mittelpunkt zwischen beiden testpunkten
        // als Schnittpunkt angenommen
        // Beste n�hrung auch f�r den fall eines nicht schneidens.
        l_TempX1_si32 = (SInt32)(f_Line1_pst->P_st.X_si16 - f_Line2_pst->P_st.X_si16);
        l_TempX1_si32 *= l_TempX1_si32; // InScale^2
        l_TempY1_si32 = (SInt32)(f_Line1_pst->P_st.Y_si16 - f_Line2_pst->P_st.Y_si16);
        l_TempY1_si32 *= l_TempY1_si32; // InScale^2

        *f_Length1_si16 =
            (SInt16)(g_mtl_Sqrt_u32_ui16((UInt32)(l_TempX1_si32 + l_TempY1_si32)) /
                     2); // InScale
        *f_Length2_si16 = *f_Length1_si16;
        f_P_pst->X_si16 =
            (SInt16)((f_Line1_pst->P_st.X_si16 + f_Line2_pst->P_st.X_si16) / 2);
        f_P_pst->Y_si16 =
            (SInt16)((f_Line1_pst->P_st.Y_si16 + f_Line2_pst->P_st.Y_si16) / 2);

        l_Return_bl = FALSE;
    }

    return l_Return_bl;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_Vector_Set_vd                                       *//**
\brief         Set the vector based on two points. Start:P1; End:P2.
\details       [detailed description, formula]
*//*-------------------------------------------------------------------------
I N P U T	                                                            *//**
\param[in]     f_P1_pst
\param[in]     f_P2_pst
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    f_out_pst
\return        -
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        oas2tm
*//*************************************************************************/
void g_mtl_Vector_Set_vd(const gType_mtlPoint_st *f_P1_pst,
                         const gType_mtlPoint_st *f_P2_pst, gType_mtlPoint_st *f_out_pst)
{
    f_out_pst->X_si16 = f_P2_pst->X_si16 - f_P1_pst->X_si16;
    f_out_pst->Y_si16 = f_P2_pst->Y_si16 - f_P1_pst->Y_si16;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_VectorNormSqr_ui32                                 *//**
\brief         Compute norm (length) of the vector.
\details       [detailed description, formula]
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_Vector_pst
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        Squared norm (length) of the vector. [InputScale^2]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        oas2tm
*//*************************************************************************/
UInt32 g_mtl_VectorNormSqr_ui32(const gType_mtlPoint_st *f_Vector_pst)
{
    UInt32 l_result_ui32;
    UInt32 l_xSQ_ui32, l_ySQ_ui32;

    l_xSQ_ui32 = (UInt32)g_mtl_Square_mac((SInt32)f_Vector_pst->X_si16);
    l_ySQ_ui32 = (UInt32)g_mtl_Square_mac((SInt32)f_Vector_pst->Y_si16);

    l_result_ui32 = l_xSQ_ui32 + l_ySQ_ui32;

    return l_result_ui32;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_VectorNorm_ui16                                    *//**
\brief         Compute norm (length) of the vector.
\details       [detailed description, formula]
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_Vector_pst
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        Norm (length) of the vector. [Output scale = input scale]
\throw         [error values]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        oas2tm, gra1lr
*//*************************************************************************/
UInt16 g_mtl_VectorNorm_ui16(const gType_mtlPoint_st *f_Vector_pst)
{
    UInt32 l_normsqr_ui32;
    UInt16 l_result_ui16;

    l_normsqr_ui32 = g_mtl_VectorNormSqr_ui32(f_Vector_pst);

    l_result_ui16 = g_mtl_Sqrt_u32_ui16(l_normsqr_ui32);

    return l_result_ui16;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_NormalizeVector_ui16                               *//**
\brief         Normalize vector to have length f_Norm_ui16.
\details       The length must be less than \ref GD_MAX_SI16
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_Vector_pst
\param[in]     f_Norm_ui16
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[in,out] f_Vector_pst
\return        -
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        gra1lr
*//*************************************************************************/
void g_mtl_NormalizeVector_ui16(gType_mtlPoint_st *f_Vector_pst, const UInt16 f_Norm_ui16)
{
    UInt16 l_Norm_ui16, l_VectorNorm_ui16;

    l_VectorNorm_ui16 = g_mtl_VectorNorm_ui16(f_Vector_pst);
    l_Norm_ui16       = g_mtl_Min_mac(f_Norm_ui16, (UInt16)GD_MAX_SI16);

    f_Vector_pst->X_si16 = (SInt16)g_mtl_s32_Div_s32_si32(
        (SInt32)l_Norm_ui16 * (SInt32)f_Vector_pst->X_si16, (SInt32)l_VectorNorm_ui16);

    f_Vector_pst->Y_si16 = (SInt16)g_mtl_s32_Div_s32_si32(
        (SInt32)l_Norm_ui16 * (SInt32)f_Vector_pst->Y_si16, (SInt32)l_VectorNorm_ui16);
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_InnerProduct_si16
\brief         Return the inner product (or, scalar product) of two vectors.
\details       [detailed description, formula]
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_Vector1_pst first vector
\param[in]     f_Vector2_pst second vector
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        Inner product of the given vectors. [InputScale^2]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        oas2tm, gra1lr
*//*************************************************************************/
SInt32 g_mtl_InnerProduct_si32(gType_mtlPoint_st const *f_Vector1_pst,
                               gType_mtlPoint_st const *f_Vector2_pst)
{
    SInt32 l_result_si32;
    SInt32 l_x1x2_si32, l_y1y2_si32;

    l_x1x2_si32 = (SInt32)f_Vector1_pst->X_si16 * (SInt32)f_Vector2_pst->X_si16;
    l_y1y2_si32 = (SInt32)f_Vector1_pst->Y_si16 * (SInt32)f_Vector2_pst->Y_si16;

    l_result_si32 = l_x1x2_si32 + l_y1y2_si32;

    return l_result_si32;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_VectorProjection_vd
\brief         Return the projection of a vector \f$\mathbf{r}\f$ onto another
               vector \f$\mathbf{v}\f$ .
\details

\verbatim
                       Point
                        o
                   ->  /.
         Vector1 = r  / .
                     /  .
                    /   .                     ->
                   o----o-------->  Vector2 = v
                  proj_v(r)
\endverbatim

\f[
               proj_\mathbf{v} \mathbf{r} = \frac{\mathbf{r} \cdot
\mathbf{v}}{\left|\mathbf{v}\right|} \f]
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_Vector1_pst Vector to be projected to another.
\param[in]     f_Vector2_pst  Vector to project on.
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        Signed vector projection (length of the segment). [Output Scale = InputScale]
\throw         [error values]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        oas2tm, gra1lr
*//*************************************************************************/
SInt16 g_mtl_VectorProjection_si16(gType_mtlPoint_st const *f_Vector1_pst,
                                   gType_mtlPoint_st const *f_Vector2_pst)
{
    SInt16 l_retval_si16;
    SInt32 l_innerproduct_si32, l_vnorm_si32;

    l_innerproduct_si32 = g_mtl_InnerProduct_si32(f_Vector1_pst, f_Vector2_pst);
    l_vnorm_si32        = (SInt32)g_mtl_VectorNorm_ui16(f_Vector2_pst);

    l_retval_si16 = (SInt16)g_mtl_s32_Div_s32_si32(l_innerproduct_si32, l_vnorm_si32);

    return l_retval_si16;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_DistPointVectorSegment_si16                        *//**
\brief         Determine the perpendicular distance between a point and a line.
\details       Determine the distance between a point and a line given
by two points on that line using vector projection method as shown in the figure.

\verbatim
                       Point
                        o
                   ->  /|
                   r  / | ->
                     /  | v
                    /   |
                 --o-----------o--
               Point1        Point2
\endverbatim
according to the formula

\f{eqnarray*}{
               d & = & proj_\mathbf{v} \mathbf{r} \\
                 & = & \frac{\left|\mathbf{r} \cdot \mathbf{v} \right|}{\left|\mathbf{v}\right|}
\f}

where \f$r\f$ is the vector pointing from a point on the line
segment to the point in question and \f$v\f$ is the vector between
the two points on the line. For further details please refer to

http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html

The function is not overflow-safe.

NOTE: @see g_mtl_DistPointLineSegment_si16, use the right function for your task!

Unit Test:
\see m_Test_DistPointVectorSegment_si16()
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_P_pst              point with arbitrary scaling
\param[in]     f_LineSegment_pst    a line given by two points on the line
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        Distance \f$d\f$. [Output Scale = InputScale]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        weurban
*//*************************************************************************/
SInt16 g_mtl_DistPointVectorSegment_si16(const gType_mtlPoint_st *f_P_pst,
                                         const gType_mtlLineSegment_st *f_LineSegment_pst)
{
    gType_mtlPoint_st l_v_st, l_r_st;
    SInt16 l_retval_si16;

    l_v_st.X_si16 = -(f_LineSegment_pst->P2_st.Y_si16 - f_LineSegment_pst->P1_st.Y_si16);
    l_v_st.Y_si16 = (f_LineSegment_pst->P2_st.X_si16 - f_LineSegment_pst->P1_st.X_si16);

    l_r_st.X_si16 = f_P_pst->X_si16 - f_LineSegment_pst->P1_st.X_si16;
    l_r_st.Y_si16 = f_P_pst->Y_si16 - f_LineSegment_pst->P1_st.Y_si16;

    l_retval_si16 = g_mtl_VectorProjection_si16(&l_r_st, &l_v_st);

    return l_retval_si16;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_DistPointVectorAngle_si32                          *//**
\brief         This function calculates the distance between a point and a line
               given as a vector (support point and direction)
\details

The result overflows in some very pathetic cases (e.g. the line is close to
(-min_si16, -min_si16), the point close to (max_si16, max_si16)).

*//*-------------------------------------------------------------------------
I N P U T                                                              *//**
\param[in]    f_Point_pst   the point (arbitraty scaling in range cm...F10)
\param[in]    f_line_pst    support point and orientation of the line.
                            (arbitraty scaling in range cm...F10)
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\return        distance (negative if point is right of vector,
               positive if the point is left of the vector).
               This function is guaranteed to not overflow.
               [Output Scale = InputScale]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        str2lr            Arwed Starke          AE-BE/ESU3
*//*************************************************************************/
SInt32 g_mtl_DistPointVectorAngle_si32(gType_mtlPoint_st const *f_Point_pst,
                                       gType_mtlLineSinCos_st const *f_Line_pst)
{
    SInt32 l_lot_si32;

    // distance between circle center and support point of the line
    // use SInt32 to be overflow safe
    const SInt32 l_dx_si32 = f_Point_pst->X_si16 - f_Line_pst->P_st.X_si16;
    const SInt32 l_dy_si32 = f_Point_pst->Y_si16 - f_Line_pst->P_st.Y_si16;

    // Berechne Lotlaenge von Kreismittelpunkt auf Gerade
    l_lot_si32 = g_mtl_SinCosMul_si32(l_dy_si32, f_Line_pst->Phi_st.Cos_si16);
    l_lot_si32 -= g_mtl_SinCosMul_si32(l_dx_si32, f_Line_pst->Phi_st.Sin_si16);

    return l_lot_si32;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_DistPointLineSegment_ui32
\brief         Return the distance of point and line segment.
\details
calculates the distance of the closest point on the line segment from LineSeg.P1
by vector projection of [P - P] * [P2 - P1]. The distance is limited to the
interval [0, length(LineSegment)], i.e. if the closest point to P is one of the
end points of the line segment, the distance to the end point is returned.
Otherwise the distance from P to the foot point on the line is returned.

Even if the function returns UInt32 to be completely overflow safe,
the input would have to be very degenerate. It is usually safe to cast
the output to UInt16.
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_P_pst       point in euclidean coordinates (arbitrary scale)
\param[in]     f_LineSeg_pst point in euclidean coordinates (arbitrary scale)
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\return        distance [Output Scale = Input Scale]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        str2lr
*//*************************************************************************/
UInt32 g_mtl_DistPointLineSegment_ui32(gType_mtlPoint_st const *f_P_pst,
                                       gType_mtlLineSegment_st const *f_LineSeg_pst)
{
    gType_mtlPoint_st l_v1_st, l_v2_st;
    SInt32 l_t_si32; // foot point location on line segment, by distance
    SInt16 l_dx_si16 = 0,
           l_dy_si16 = 0; // vector from LineSeg.P1 to foot point for P
    UInt16 l_lineSegLength_ui16;

    // vector 1 goes from point P to start of Line segment. Not completely
    // overflow-safe!
    l_v1_st.X_si16 = f_P_pst->X_si16 - f_LineSeg_pst->P1_st.X_si16;
    l_v1_st.Y_si16 = f_P_pst->Y_si16 - f_LineSeg_pst->P1_st.Y_si16;

    // vector 2 goes from line segment end to start point. Not completely
    // overflow-safe!
    l_v2_st.X_si16       = f_LineSeg_pst->P2_st.X_si16 - f_LineSeg_pst->P1_st.X_si16;
    l_v2_st.Y_si16       = f_LineSeg_pst->P2_st.Y_si16 - f_LineSeg_pst->P1_st.Y_si16;
    l_lineSegLength_ui16 = g_mtl_VectorNorm_ui16(&l_v2_st); // InScale

    if (l_lineSegLength_ui16 > 0)
    {
        l_t_si32 = g_mtl_VectorProjection_si16(&l_v1_st, &l_v2_st); // InScale
        // norm t to interval [0, SegmentLength]
        l_t_si32 = g_mtl_Min_s32_Max_s32_si32(l_t_si32, 0, (SInt16)l_lineSegLength_ui16);

        // get foot point on line segment. Overflow safe: 2^15-1 * 2^16-1 < 2^31-1
        l_dx_si16 =
            (SInt16)((l_t_si32 * l_v2_st.X_si16) /
                     (SInt32)
                         l_lineSegLength_ui16); // InScale * InScale / InScale = InScale
        l_dy_si16 = (SInt16)((l_t_si32 * l_v2_st.Y_si16) / (SInt32)l_lineSegLength_ui16);
    }

    // store the foot point (closest point on the line segment) in v1
    l_v1_st.X_si16 = f_LineSeg_pst->P1_st.X_si16 + l_dx_si16; // InScale
    l_v1_st.Y_si16 = f_LineSeg_pst->P1_st.Y_si16 + l_dy_si16;

    // return distance between foot point and point
    return g_mtl_DistPointPoint_ui32(f_P_pst, &l_v1_st); // InScale
}

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
------------------------------------------------------------------------------
Function name: g_mtl_DistPointPoint_ui32                                 *//*!
\brief       Calculates euclidean distance between two points in R2.
\details
The parameters can be any fixed point scale, as long as they are
all of the same scale. The result will have the same scale.

Formula: \f$ \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2} \f$

This function is overflow-safe. An overflow in UInt16 could occur if the points are close to
(-max_si16, -max_si16), (+max_si16, +max_si16). This is why UInt32 is returned.
This is a very rare case. Deal with results > GD_MAX_UI16 yourself.

Unit Test:
\see m_Test_DistPointPoint_ui8()
*//*--------------------------------------------------------------------------
I N P U T                                                                *//*!
\param[in]  f_P1_pst   first point (arbitraty scaling in range cm...F10)
\param[in]  f_P2_pst   second point (arbitraty scaling in range cm...F10)
*//*--------------------------------------------------------------------------
O U T P U T                                                              *//*!
\return     distance [Output Scale = Input Scale]
*//*--------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                       *//*!
\author   str2lr
*//**************************************************************************/
UInt32 g_mtl_DistPointPoint_ui32(gType_mtlPoint_st const *f_P1_pst,
                                 gType_mtlPoint_st const *f_P2_pst)
{
    // use UInt32 for dx/dy to avoid overflow when calculating squared distance
    // max. range of dx/dy: [-2^16 ... 2^16-2] => [0 .. +2^16]
    UInt32 l_dxSq_ui32 =
        (UInt32)g_mtl_Abs_mac((SInt32)f_P2_pst->X_si16 - (SInt32)f_P1_pst->X_si16);
    UInt32 l_dySq_ui32 =
        (UInt32)g_mtl_Abs_mac((SInt32)f_P2_pst->Y_si16 - (SInt32)f_P1_pst->Y_si16);

    /* squared distances */
    // max. range of dx/dy: [0 .. 2^32] => no overflow can happen by
    // multiplication
    l_dxSq_ui32 = g_mtl_Square_mac(l_dxSq_ui32);
    l_dySq_ui32 = g_mtl_Square_mac(l_dySq_ui32);

    if ((GD_MAX_UI32 - l_dySq_ui32) < l_dxSq_ui32)
    {
        // prevent overflow in addition (could go up to 2^33-2)
        l_dxSq_ui32 >>= 2; // divide by 4
        l_dySq_ui32 >>= 2;
        l_dxSq_ui32 = l_dxSq_ui32 + l_dySq_ui32;
        l_dxSq_ui32 = g_mtl_Sqrt_u32_ui16(l_dxSq_ui32);
        l_dxSq_ui32 <<= 1; // multiply by 2 -> InScale is back
    }
    else
    {
        l_dxSq_ui32 = l_dxSq_ui32 + l_dySq_ui32;
        l_dxSq_ui32 = g_mtl_Sqrt_u32_ui16(l_dxSq_ui32);
    }

    return l_dxSq_ui32;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_Rotate_vd                                           *//**
\brief         Rotation of a 2D point around origin in cartesian coordinates
\details       Rotation: [xr yr]' = R * [x y]'

\verbatim
X_rot = x * cos(psi) - y * sin(psi)
Y_rot = x * sin(psi) + y * cos(psi)
\endverbatim

This function is overflow-safe. You can call the function with P pointing to the
same data as P_rot, like this:
rotate(p, phi, p);

NOTE: The numerical error increases linearly over the norm of P. It may become
      > 5 cm for the largest possible P's.

Unit Test:
\see m_Test_Rotate_vd()

*//*-------------------------------------------------------------------------
I N P U T      (parameter range and description)                        *//**
\param[in]     P_pst      Original Point. Scale [F10], mm or cm
\param[in]     f_Angle_pst  rotation angle.
*//*-------------------------------------------------------------------------
O U T P U T    (result parameter range and description)                 *//**
\param[out]    P_rot_pst  pointer to result. Scale: input scale
\return        -
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                      *//**
\author        str2lr
*//*************************************************************************/
void g_mtl_Rotate_vd(gType_mtlPoint_st const *f_P_pst,
                     gType_mtlAngle_st const *f_Angle_pst, gType_mtlPoint_st *f_P_rot_pst)
{
    SInt32 l_CosTemp_si32; // temporary variable a*cos(rotation angle)
    SInt32 l_SinTemp_si32; // temporary variable a*sin(rotation angle)
    SInt32 l_temp_si32;    // helper variable to support P_rot == P

    l_CosTemp_si32 = f_P_pst->X_si16 * f_Angle_pst->Cos_si16; // InScale * F14 (e.g. F24)
    l_SinTemp_si32 = f_P_pst->Y_si16 * f_Angle_pst->Sin_si16;
    l_temp_si32    = g_mtl_ReducePrecision_si32(l_CosTemp_si32 - l_SinTemp_si32,
                                                (UInt8)14); // Input scale

    l_SinTemp_si32 = f_P_pst->X_si16 * f_Angle_pst->Sin_si16;
    l_CosTemp_si32 = f_P_pst->Y_si16 * f_Angle_pst->Cos_si16; // InScale * F14 (e.g. F24)

    f_P_rot_pst->X_si16 = (SInt16)l_temp_si32;
    l_temp_si32         = g_mtl_ReducePrecision_si32(l_SinTemp_si32 + l_CosTemp_si32,
                                                     (UInt8)14); // Input scale
    f_P_rot_pst->Y_si16 = (SInt16)l_temp_si32;

    return;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_RotateAroundPoint_vd                               *//**
\brief         Rotation of a 2D point around another 2D point in cartesian coordinates
\details

\verbatim
R = [cos(phi) -sin(phi); sin(phi) cos(phi)];
P_rot = M + R * (P - M);
\endverbatim

This function is not overflow-safe (you can rotate a point to a position outside the SInt16 range).

You can call the function with P pointing to the same data as P_rot, like this:

\verbatim
g_mtl_RotateAroundPoint_vd(P, PCenter, rotAngle, P);
\endverbatim

NOTE: The numerical error increases linearly over the norm of (P - PCenter).
      It may become > 5 cm for very large distances.

Unit test:
\see m_Test_RotateAroundPoint_vd()

*//*-------------------------------------------------------------------------
I N P U T      (parameter range and description)                        *//**
\param[in]     P_pst      Original Point. Scale [F10], mm or cm
\param[in]     f_Angle_pst  rotation angle.
*//*-------------------------------------------------------------------------
O U T P U T    (result parameter range and description)                 *//**
\param[out]    P_rot_pst  pointer to result. Scale: input scale
\return        -
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                      *//**
\author        str2lr
*//*************************************************************************/
extern void g_mtl_RotateAroundPoint_vd(gType_mtlPoint_st const *f_P_pst,
                                       gType_mtlPoint_st const *f_RotationCenter_pst,
                                       gType_mtlAngle_st const *f_Angle_pst,
                                       gType_mtlPoint_st *f_P_rot_pst)
{
    // these 2 lines are not overflow-safe
    f_P_rot_pst->X_si16 = f_P_pst->X_si16 - f_RotationCenter_pst->X_si16;
    f_P_rot_pst->Y_si16 = f_P_pst->Y_si16 - f_RotationCenter_pst->Y_si16;

    g_mtl_Rotate_vd(f_P_rot_pst, f_Angle_pst, f_P_rot_pst);

    f_P_rot_pst->X_si16 += f_RotationCenter_pst->X_si16;
    f_P_rot_pst->Y_si16 += f_RotationCenter_pst->Y_si16;

    return;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_TransformToLocal_vd                               *//**
\brief         Compute the coordinates of a point (in global coordinates)
               in a local coordinate system.
\details
This is the inverse operation of g_mtl_TransformToGlobal_vd.

Example: The local coordinate system is the vehicle position within the parking
space coordinate system. To find out whether a point in the parking space coordinate
system lies within the vehicle's path (straight line), do:

\verbatim
g_mtl_TransformToLocal_vd(P, VehPos, P_trans);
\endverbatim

P_trans.X will now contain the distance to the point along the vehicle course,
P_trans.Y will contain the lateral/transversal distance to the vehicle course
(if P_trans.Y < -Vehicle.HalfWidth, it lies to the right of the driving corridor.
 If P_trans.Y > Vehicle.HalfWidth, the point lies left of the driving corridor.)

\verbatim
1. Translation: P' = (P - P0)
2. Rotation:    P'' = P * R(-phi)
\endverbatim

This function is overflow-safe. You can call the function with P pointing to the
same data as P_trans, like this:
\verbatim
TransformToLocal(P, CoordSys, P);
\endverbatim

Unit Test:
\see m_Test_TransformToLocal_Global_vd()

*//*-------------------------------------------------------------------------
I N P U T      (parameter range and description)                        *//**
\param[in]    f_P_pst              point in global coordinate sytem (arbitrary scale)
\param[in]    f_LocalCoordSys_pst  coordinates of local coordinate system within global coordinate system.
              The angle is the direction of the +x Axis [F12 rad]
*//*-------------------------------------------------------------------------
O U T P U T    (result parameter range and description)                 *//**
\param[out]    f_P_trans_pst      point in local coordinate system (x/y relative to LocalCoordSys)
\return        -
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y (use network account)                      *//**
\author        Arwed Starke - str2lr
*//*************************************************************************/
void g_mtl_TransformToLocal_vd(gType_mtlPoint_st const *f_P_pst,
                               gType_mtlLineSinCos_st const *f_LocalCoordSys_pst,
                               gType_mtlPoint_st *f_P_trans_pst)
{
    gType_mtlAngle_st l_rotAngle_st;
    l_rotAngle_st = f_LocalCoordSys_pst->Phi_st;
    g_mtlAngle_Neg_vd(&l_rotAngle_st);

    f_P_trans_pst->X_si16 = f_P_pst->X_si16 - f_LocalCoordSys_pst->P_st.X_si16;
    f_P_trans_pst->Y_si16 = f_P_pst->Y_si16 - f_LocalCoordSys_pst->P_st.Y_si16;

    g_mtl_Rotate_vd(f_P_trans_pst, &l_rotAngle_st, f_P_trans_pst);
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name: g_mtl_TransformToGlobal_vd                               *//**
\brief       Transform a position given in local coordinates to global coordinates.
\details
The position and orientation of the local coordinate system within the global
coordinate system is given in LocalCoordSys. The global coordinate system is
defined as X = 0, Y = 0, with Phi = 0, like this:
\verbatim

y ^
  |
  +---> x

\endverbatim

Example: The local coordinate system is the vehicle position within the parking
space coordinate system. Find out the position of the right front corner in
the parking space coordinate system by:

\verbatim
P = [+Vehicle.LengthFront; -Vehicle.HalfWidth]
g_mtl_TransformToGlobal_vd(P, VehPos, P_trans);
\endverbatim

Calculation:

\verbatim
1. Rotation:    P' = R(phi) * P    with R = [cos(phi) -sin(phi); sin(phi) cos(phi)]
2. Translation: P'' = (P' + P0)
\endverbatim

This function is overflow-safe. You can call the function with P pointing to the
same data as P_rot, like this:
\verbatim
TransformToGlobal(P, Coord, P);
\endverbatim

Unit Test:
\see m_Test_TransformToLocal_Global_vd()
*//*-------------------------------------------------------------------------
I N P U T      (parameter range and description)                        *//**
\param[in]   f_P_pst              point in local coordinate sytem (arbitrary scale)
\param[in]   f_LocalCoordSys_pst  coordinates of local coordinate system within global coordinate system.
              The angle is the direction of the +x Axis [F12 rad]
*//*-------------------------------------------------------------------------
O U T P U T    (result parameter range and description)                 *//**
\param[out]  f_PTrans_pst         point in global coordinates
*//*-------------------------------------------------------------------------
A U T H O R    Arwed Starke - str2lr
*//*************************************************************************/
extern void g_mtl_TransformToGlobal_vd(gType_mtlPoint_st const *f_P_pst,
                                       gType_mtlLineSinCos_st const *f_LocalCoordSys_pst,
                                       gType_mtlPoint_st *f_PTrans_pst)
{
    g_mtl_Rotate_vd(f_P_pst, &f_LocalCoordSys_pst->Phi_st, f_PTrans_pst);
    f_PTrans_pst->X_si16 += f_LocalCoordSys_pst->P_st.X_si16;
    f_PTrans_pst->Y_si16 += f_LocalCoordSys_pst->P_st.Y_si16;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name: g_mtl_GetPointOnCircle_bl                               *//**
\brief       calculate a point on a circle given circle and angle of point
\details     It's more or less a polar to cartesian coordinate transformation.
Return the (X/Y) coordinate for a point on a circle. The point is determined
by the circle center, radius and the angle from the circle center to the point.
Function is overflow-aware.
*//*-------------------------------------------------------------------------
I N P U T      (parameter range and description)                        *//**
\param[in]   f_Circle_pst   the Circle (center + radius), arbitrary scaling
\param[in]   f_Phi_pst      the angle from the center to the point on the circle [F12 rad]
*//*-------------------------------------------------------------------------
O U T P U T    (result parameter range and description)                 *//**
\param[out]  f_PointOnCircle_pst    point on circle in cartesian coordinates (X,Y).
                                    Only use result if return value is TRUE
\return      FALSE on overflow, TRUE if calculation was ok
*//*-------------------------------------------------------------------------
A U T H O R    Arwed Starke - str2lr
*//*************************************************************************/
extern Boolean g_mtl_GetPointOnCircle_bl(gType_mtlCircleU16_st const *f_Circle_pst,
                                         gType_mtlAngle_st const *f_Phi_st,
                                         gType_mtlPoint_st *f_PointOnCircle_pst)
{
    SInt32 l_X_si32, l_Y_si32;
    Boolean l_CalculationOk_bl = FALSE;
    l_X_si32 = g_mtl_SinCosMul_si32(f_Circle_pst->R_ui16, f_Phi_st->Cos_si16) +
               f_Circle_pst->P_st.X_si16;
    l_Y_si32 = g_mtl_SinCosMul_si32(f_Circle_pst->R_ui16, f_Phi_st->Sin_si16) +
               f_Circle_pst->P_st.Y_si16;

    if ((g_mtl_Abs_mac(l_X_si32) <= GD_MAX_SI16) &&
        (g_mtl_Abs_mac(l_Y_si32) <= GD_MAX_SI16))
    {
        l_CalculationOk_bl          = TRUE;
        f_PointOnCircle_pst->X_si16 = (SInt16)l_X_si32;
        f_PointOnCircle_pst->Y_si16 = (SInt16)l_Y_si32;
    }

    return l_CalculationOk_bl;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_FootPointLineSegment_si16                          *//**
\brief         Determine the perpendicular of a point with a line segment.
\details       Outputs the point even if it is located outside of the segment
               given by the points A and B. (The function's correct name
               would be FootPointVectorSegment...)

\verbatim
                      P                          P
                      o                          o
                     /|                          |\
                    / |                          | \
                   /  |                          |  \
                  /   |                          |   \
                 o----o------o  Line (Segment)   o....o------------o
                 A    ^      B                   ^    A            B
                      |                          |
                      C ... Foot Point           C ... Foot Point
\endverbatim

               The geometric relation of the vectors \f$\overrightarrow{AB}\f$,
               \f$\overrightarrow{AP}\f$, and \f$\overrightarrow{AC}\f$ is used
               to find the length of the section \f$AC\f$. With
\f[
               \cos\left(\angle BAP\right) = \frac{\overrightarrow{AB} \cdot \overrightarrow{AP}}{\left\|\overrightarrow{AB}\right\| \cdot \left\|\overrightarrow{AP}\right\|}
\f]
and
\f[
               \cos\left(\angle CAP\right) = \frac{\left\|\overrightarrow{AC}\right\|}{\left\|\overrightarrow{AP}\right\|}
\f]
we find
\f[
               \left\|\overrightarrow{AC}\right\| = \frac{\overrightarrow{AB} \cdot \overrightarrow{AP}}{\left\|\overrightarrow{AB}\right\|}
\f]


Unit Test:
\see m_Test_FootPointLineSegment_ui8() and UT_FootPointLineSegment.m

*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_Line_pst
\param[in]     f_P_pst
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    f_FootPoint_pst
\return        -
\throw         [error values]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        weurban, gra1lr
*//*************************************************************************/
void g_mtl_FootPointLineSegment_vd(const gType_mtlLineSegment_st *f_Line_pst,
                                   const gType_mtlPoint_st *f_P_pst,
                                   gType_mtlPoint_st *f_FootPoint_pst)
{
    gType_mtlPoint_st l_AB_st, l_AP_st;
    SInt16 l_dx_AC_si16, l_dy_AC_si16;
    SInt32 l_innerproduct_si32;
    UInt32 l_normsqr_AB_ui32, l_normsqr_AP_ui32, l_scale_ui32;
    UInt16 l_norm_AB_ui16, l_norm_AP_ui16;

    //
    // Vector AB defined by the two points A and B of the line segment
    //
    l_AB_st.X_si16 = (f_Line_pst->P2_st.X_si16 - f_Line_pst->P1_st.X_si16);
    l_AB_st.Y_si16 = (f_Line_pst->P2_st.Y_si16 - f_Line_pst->P1_st.Y_si16);

    //
    // Vector AP defined by A and P
    //
    l_AP_st.X_si16 = (f_P_pst->X_si16 - f_Line_pst->P1_st.X_si16);
    l_AP_st.Y_si16 = (f_P_pst->Y_si16 - f_Line_pst->P1_st.Y_si16);

    //
    // special case handling: the line on which AB is located can be defined as
    // A+t(B-A) if t<0, P's projection is located "in front of" A, thus order of A
    // and B are reversed to be sure that cos(ABP) is non negative
    //
    // t = innerproduct(AB,AP)/|BA|^2 -> |AB|*|AP|*cos(PAB)/|BA|^2 ->
    // |AP|*cos(PAB)/|BA|, i.e if t is non negative projection of P is not located
    // "in front of" A innerproduct(AB,AP) is checked only as |BA|^2 is
    // non-negative
    //

    l_innerproduct_si32 = g_mtl_InnerProduct_si32(&l_AB_st, &l_AP_st);

    if (l_innerproduct_si32 < 0)
    {
        // reverse AB
        l_AB_st.X_si16 = (f_Line_pst->P1_st.X_si16 - f_Line_pst->P2_st.X_si16);
        l_AB_st.Y_si16 = (f_Line_pst->P1_st.Y_si16 - f_Line_pst->P2_st.Y_si16);
    }

    //
    // square norm of vector AB
    //
    l_normsqr_AB_ui32 = g_mtl_VectorNormSqr_ui32(&l_AB_st);
    l_normsqr_AP_ui32 = g_mtl_VectorNormSqr_ui32(&l_AP_st);

    //
    // normalize length of vector AB to make sure that |AB| >= |AP|
    //
    if (l_normsqr_AB_ui32 < l_normsqr_AP_ui32)
    {
        l_norm_AB_ui16 = g_mtl_Sqrt_u32_ui16(l_normsqr_AB_ui32);
        l_norm_AP_ui16 = g_mtl_Sqrt_u32_ui16(l_normsqr_AP_ui32);

        l_AB_st.X_si16 = (SInt16)g_mtl_s32_Div_s32_si32(
            ((SInt32)l_norm_AP_ui16 * (SInt32)l_AB_st.X_si16), (SInt32)l_norm_AB_ui16);
        l_AB_st.Y_si16 = (SInt16)g_mtl_s32_Div_s32_si32(
            ((SInt32)l_norm_AP_ui16 * (SInt32)l_AB_st.Y_si16), (SInt32)l_norm_AB_ui16);

        l_normsqr_AB_ui32 = l_normsqr_AP_ui32;
    }

    //
    // inner product of vector AB and AP
    //
    l_innerproduct_si32 = g_mtl_InnerProduct_si32(&l_AB_st, &l_AP_st);

    //
    // downscale to enable for below operations
    //
    l_scale_ui32 = (UInt32)1;
    while (l_innerproduct_si32 > (SInt32)GD_MAX_SI16)
    {
        l_innerproduct_si32 = g_mtl_ReducePrecision_si32(l_innerproduct_si32, 1);
        l_scale_ui32        = l_scale_ui32 << 1;
    }

    //
    // scaling possible without restrictions as |AB|^2 >= AB * AP, "if" used to
    // avoid QAC...
    //
    if (l_scale_ui32 != 0)
    {
        l_normsqr_AB_ui32 = l_normsqr_AB_ui32 / l_scale_ui32;
    }

    //
    // length between first point of line segment and point in question via
    // intercept theorem using equations innerproduct(AB,AP) = |AB|*|AP|*cos(PAB)
    // and AB.X = |AB|cos(ABB_projected_to_x_axis)

    l_dx_AC_si16 = (SInt16)g_mtl_s32_Div_s32_si32(
        (l_innerproduct_si32 * (SInt32)l_AB_st.X_si16), (SInt32)l_normsqr_AB_ui32);
    l_dy_AC_si16 = (SInt16)g_mtl_s32_Div_s32_si32(
        (l_innerproduct_si32 * (SInt32)l_AB_st.Y_si16), (SInt32)l_normsqr_AB_ui32);

    //
    // compute actual foot point
    //
    f_FootPoint_pst->X_si16 = f_Line_pst->P1_st.X_si16 + l_dx_AC_si16;
    f_FootPoint_pst->Y_si16 = f_Line_pst->P1_st.Y_si16 + l_dy_AC_si16;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_IntersectionCircleCircle_si8                       *//**
\brief         Determine the intersection points of two circles
\details       see below

NOTE: Only overflow safe for R < 2^15-1
      Not overflow safe for all circle center positions

Unit Test:
\see m_Test_IntersectionCircleCircle_ui8()

*//*-------------------------------------------------------------------------
I N P U T                                                                *//**
\param[in]     f_Circle1_pst      Circle 1. Arbitrary value scaling.
\param[in]     f_Circle2_pst      Circle 2. Same scaling as Circle 1.
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]     f_P1_pst           First intersection point. [Output Scale = Input Scale]
\param[out]     f_P2_pst           Second intersection point. [Output Scale = Input Scale]
\return        number of intersection points
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        weurban, str2lr
*//*************************************************************************/
UInt8 g_mtl_IntersectionCircleCircle_ui8(gType_mtlPoint_st *f_P1_pst,
                                         gType_mtlPoint_st *f_P2_pst,
                                         gType_mtlCircleU16_st const *f_Circle1_pst,
                                         gType_mtlCircleU16_st const *f_Circle2_pst)
{
    SInt32 l_DistY_si32;
    UInt16 l_DistX_ui16;
    UInt32 l_SumRadii_ui32;
    UInt16 l_RadiiDiff_ui16;
    SInt16 l_SPdx_si16, l_SPdy_si16;

    SInt16 l_temp1_si16, l_temp2_si16, l_temp3_si16, l_temp4_si16;
    UInt32 l_distMP1MP2_ui32 =
        g_mtl_DistPointPoint_ui32(&f_Circle1_pst->P_st, &f_Circle2_pst->P_st); // InScale
    UInt8 l_numSolutions_ui8 = 2;

    // no overflow possible, since R < 2^16
    l_SumRadii_ui32  = f_Circle1_pst->R_ui16 + f_Circle2_pst->R_ui16; // InScale
    l_RadiiDiff_ui16 = (UInt16)g_mtl_Abs_mac((SInt32)f_Circle1_pst->R_ui16 -
                                             (SInt32)f_Circle2_pst->R_ui16); // InScale

    /* no circle intersections */
    if ((l_SumRadii_ui32 < l_distMP1MP2_ui32) || (l_RadiiDiff_ui16 > l_distMP1MP2_ui32) ||
        (l_distMP1MP2_ui32 == 0))
    {
        /* intersection point position is infinite */
        f_P1_pst->X_si16   = GD_MAX_SI16;
        f_P1_pst->Y_si16   = GD_MAX_SI16;
        f_P2_pst->X_si16   = GD_MAX_SI16;
        f_P2_pst->Y_si16   = GD_MAX_SI16;
        l_numSolutions_ui8 = 0;
    }
    else
    {
        // if radii are almost equal, tell caller that only 1 intersection point
        // exists
        if (g_mtl_Abs_mac((SInt32)l_SumRadii_ui32 - (SInt32)l_distMP1MP2_ui32) < 2)
        {
            l_numSolutions_ui8 = 1;
        }

        /* distance of intersection point and position of MP1 in y-direction: */
        //           1                 R1^2 - R2^2
        // DistY  := - * [SensDist + ---------------]
        //           2                  SensDist

        // (R1^2 - R2^2): If R1 = 2^16-1 and R2 close to 0, overflow possible:
        // (2^16-1)^2 > (2^31-1) for R1, R2 < 2^15, no overflow possible
        l_DistY_si32 = (SInt32)g_mtl_Square_mac(f_Circle1_pst->R_ui16) -
                       (SInt32)g_mtl_Square_mac(f_Circle2_pst->R_ui16); // InScale^2

        // lateral position
        l_DistY_si32 = g_mtl_s32_Div_s32_si32(l_DistY_si32,
                                              (SInt32)l_distMP1MP2_ui32); // InScale
        l_DistY_si32 += (SInt32)l_distMP1MP2_ui32;
        l_DistY_si32 = g_mtl_s32_Div_s32_si32(l_DistY_si32, 2);

        /* distance of intersection point and position sensor A in x-direction: */
        /*                          ____________      */
        /*                      \  /   2       2      */
        /* Pythagoras: DistX :=  \/  R1 - DistY       */
        /*                                            */
        if (f_Circle1_pst->R_ui16 > (UInt32)g_mtl_Abs_mac(l_DistY_si32))
        {
            l_DistX_ui16 = g_mtl_CalcCathetus_ui16(f_Circle1_pst->R_ui16,
                                                   (UInt16)g_mtl_Abs_mac(l_DistY_si32));
        }
        else
        {
            l_DistX_ui16 =
                0; // only one intersection point if one circle is inside the other
            l_numSolutions_ui8 = 1;
        }

        /* intersection points: */
        /* ===================_ */
        /* PosS1_Y := SensAPos_Y  + (DistY/SensDist)*SPdy + (DistX/SensDist)*SPdx */
        /* PosS1_X := SensAPos_X  + (DistY/SensDist)*SPdx - (DistX/SensDist)*SPdy */
        /*                                                                         */
        /* PosS2_Y := SensAPos_Y  + (DistY/SensDist)*SPdy - (DistX/SensDist)*SPdx */
        /* PosS2_X := SensAPos_X  + (DistY/SensDist)*SPdx + (DistX/SensDist)*SPdy */

        // TODO: Overflow possible
        l_SPdx_si16 = f_Circle2_pst->P_st.X_si16 - f_Circle1_pst->P_st.X_si16;
        l_SPdy_si16 = f_Circle2_pst->P_st.Y_si16 - f_Circle1_pst->P_st.Y_si16;

        // Scaling:  // InScale * InScale / InScale = Inscale
        l_temp1_si16 = (SInt16)g_mtl_s32_Div_s32_si32(
            ((SInt32)(l_DistY_si32)*l_SPdy_si16), (SInt32)l_distMP1MP2_ui32);
        l_temp2_si16 = (SInt16)g_mtl_s32_Div_s32_si32(
            ((SInt32)(l_DistX_ui16)*l_SPdx_si16), (SInt32)l_distMP1MP2_ui32);
        l_temp3_si16 = (SInt16)g_mtl_s32_Div_s32_si32(
            ((SInt32)(l_DistY_si32)*l_SPdx_si16), (SInt32)l_distMP1MP2_ui32);
        l_temp4_si16 = (SInt16)g_mtl_s32_Div_s32_si32(
            ((SInt32)(l_DistX_ui16)*l_SPdy_si16), (SInt32)l_distMP1MP2_ui32);

        f_P1_pst->Y_si16 = f_Circle1_pst->P_st.Y_si16 + (l_temp1_si16 + l_temp2_si16);
        f_P1_pst->X_si16 = f_Circle1_pst->P_st.X_si16 + (l_temp3_si16 - l_temp4_si16);
        f_P2_pst->Y_si16 = f_Circle1_pst->P_st.Y_si16 + (l_temp1_si16 - l_temp2_si16);
        f_P2_pst->X_si16 = f_Circle1_pst->P_st.X_si16 + (l_temp3_si16 + l_temp4_si16);
    }

    return l_numSolutions_ui8;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_ArcTan_Approx_si16                                   *//**
\brief         Approximation of arctan using lagrange interpolation.
\details       This method is suggested in the IEEE article
               "Efficient approximations for the arctangent function" which
			   can be found under following the link.

               http://dx.doi.org/10.1109/MSP.2006.1628884

			   The first order approximation

			   \f[
			   arctan\left(x\right) \approx \frac{\pi}{4} x
			   \f],

			   the second order approximation

			   \f[
			   arctan\left(x\right) \approx \frac{\pi}{4} x + 0.285 \cdot x \cdot \left(1 - \left|x\right|\right)
			   \f],

			   and the third order approximation

			   \f[
			   arctan\left(x\right) \approx \frac{\pi}{4} x - x \cdot \left(\left|x\right| - 1\right) \cdot \left(0.0663*\left|x\right| + 0.2447\right)
			   \f]

			   are available. The desired order may be chosen using the third
			   parameter.

NOTE: Error can become roughly 0.1�

Unit Test:
\see UT_ArcTanApprox.m
*//*-------------------------------------------------------------------------
I N P U T	                                                            *//**
\param[in]     f_dY_si16              y value of vector (arbitrary scale)
\param[in]     f_dX_si16              x value of vector (arbitrary scale)
\param[in]     f_Order_ui8            approximation of which order (use 3 for best precision)
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        arctan2(dy,dx)  Angle in [F12 rad]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        gra1lr
*//*************************************************************************/
SInt16 g_mtl_ArcTan_Approx_si16(SInt16 f_dY_si16, SInt16 f_dX_si16, UInt8 f_Order_ui8)
{
    Boolean l_dygtdx_bl;
    SInt16 l_angle_si16;
    SInt32 l_x_si32, l_term1_si32, l_term2_si32, l_term3_si32;

    if (g_mtl_Abs_mac(f_dX_si16) < g_mtl_Abs_mac(f_dY_si16))
    {
        l_dygtdx_bl = TRUE;
        l_x_si32 = g_mtl_s32_Div_s32_si32((SInt32)4096 * (SInt32)g_mtl_Abs_mac(f_dX_si16),
                                          (SInt32)g_mtl_Abs_mac(f_dY_si16));
    }
    else
    {
        l_dygtdx_bl = FALSE;
        l_x_si32 = g_mtl_s32_Div_s32_si32((SInt32)4096 * (SInt32)g_mtl_Abs_mac(f_dY_si16),
                                          (SInt32)g_mtl_Abs_mac(f_dX_si16));
    }

    //
    // APPROXIMATE ANGLE IN 0� ~ 45�
    //
    l_term1_si32 =
        g_mtl_ReducePrecision_si32((SInt32)(gd_MTL_PI_2_ui16 / 2) * l_x_si32, 12);
    if (f_Order_ui8 == 3)
    {
        l_term2_si32 = g_mtl_ReducePrecision_si32(l_x_si32 * (l_x_si32 - 4096), 12);
        l_term3_si32 = 1002 + ((663 * l_x_si32) / 10000);

        l_angle_si16 = (SInt16)(l_term1_si32 - g_mtl_ReducePrecision_si32(
                                                   l_term2_si32 * l_term3_si32, 12));
    }
    else if (f_Order_ui8 == 2)
    {
        l_term2_si32 =
            g_mtl_ReducePrecision_si32((285 * (l_x_si32 * (4096 - l_x_si32))) / 1000, 12);

        l_angle_si16 = (SInt16)(l_term1_si32 + l_term2_si32);
    }
    else
    {
        l_angle_si16 = (SInt16)l_term1_si32;
    }

    //
    // MIRROR FOR SUPPORT OF ANGLES BETWEEN 45� ~ 90�
    //

    //
    // ADD OFFSET FOR SUPPORT OF ANGELS BETWEEN 90� ~ 180�
    //
    if (f_dX_si16 < 0)
    {
        if (l_dygtdx_bl != FALSE)
        {
            l_angle_si16 = (SInt16)gd_MTL_PI_2_ui16 + l_angle_si16;
        }
        else
        {
            l_angle_si16 = (SInt16)gd_MTL_PI_ui16 - l_angle_si16;
        }
    }
    else
    {
        if (l_dygtdx_bl != FALSE)
        {
            l_angle_si16 = (SInt16)gd_MTL_PI_2_ui16 - l_angle_si16;
        }
    }

    //
    // MIRROT FOR SUPPORT OF ANGELS < 0�
    //
    if (f_dY_si16 < 0)
    {
        l_angle_si16 = -l_angle_si16;
    }

    return l_angle_si16;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_IsValueInRangeS16_bl                                *//**
\brief         Return TRUE if the value is in the given range (SInt16).
\details       [detailed description, formula]
*//*-------------------------------------------------------------------------
I N P U T	                                                            *//**
\param[in]     f_value_si16
\param[in]     f_min_si16
\param[in]     f_max_si16
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        -
\throw         [error values]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        oas2tm, wxa1lr
*//*************************************************************************/
Boolean g_mtl_IsValueInRangeS16_bl(const SInt16 f_value_si16, const SInt16 f_min_si16,
                                   const SInt16 f_max_si16)
{
    Boolean l_result_bl;
    if ((f_min_si16 <= f_value_si16) && (f_value_si16 <= f_max_si16))
    {
        l_result_bl = TRUE;
    }
    else
    {
        l_result_bl = FALSE;
    }
    return (l_result_bl);
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_IsValueInOIRangeS16_bl                             *//**
\brief         Return TRUE if the value is in the given open interval range (SInt16).
\details       [detailed description, formula]
*//*-------------------------------------------------------------------------
I N P U T                                                               *//**
\param[in]     f_value_si16
\param[in]     f_min_si16
\param[in]     f_max_si16
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        -
\throw         [error values]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        oas2tm, wxa1lr
*//*************************************************************************/
Boolean g_mtl_IsValueInOIRangeS16_bl(const SInt16 f_value_si16, const SInt16 f_min_si16,
                                     const SInt16 f_max_si16)
{
    Boolean l_result_bl;
    if ((f_min_si16 < f_value_si16) && (f_value_si16 < f_max_si16))
    {
        l_result_bl = TRUE;
    }
    else
    {
        l_result_bl = FALSE;
    }
    return (l_result_bl);
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_IsValueInRangeU16_bl                                *//**
\brief         Return TRUE if the value is in the given range(UInt16).
\details       [detailed description, formula]
*//*-------------------------------------------------------------------------
S I D E  E F F E C T S                                                  *//**
\pre           [pre-conditions]
\post          [post-conditions]
*//*-------------------------------------------------------------------------
I N P U T	                                                            *//**
\param[in]     f_value_ui16
\param[in]     f_min_ui16
\param[in]     f_max_ui16
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        -
\throw         [error values]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        oas2tm, wxa1lr
*//*************************************************************************/
Boolean g_mtl_IsValueInRangeU16_bl(const UInt16 f_value_ui16, const UInt16 f_min_ui16,
                                   const UInt16 f_max_ui16)
{
    Boolean l_result_bl;
    if ((f_min_ui16 <= f_value_ui16) && (f_value_ui16 <= f_max_ui16))
    {
        l_result_bl = TRUE;
    }
    else
    {
        l_result_bl = FALSE;
    }
    return (l_result_bl);
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_GetRange2D_vd                                     *//**
\brief         Get the 2D-Range of a line segment with two points.
\details       [detailed description, formula]
*//*-------------------------------------------------------------------------
I N P U T	                                                            *//**
\param[in]     f_P1_pst
\param[in]     f_P2_pst
\param[in]
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    f_Range2D_pst
\return        -
\throw         [error values]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        gra1lr, wxa1lr
*//*************************************************************************/
void g_mtl_Range2D_vd(const gType_mtlPoint_st *f_P1_pst,
                      const gType_mtlPoint_st *f_P2_pst,
                      gType_mtlRange2D_st *f_Range2D_pst)
{
    f_Range2D_pst->XMin_si16 = g_mtl_Min_mac(f_P1_pst->X_si16, f_P2_pst->X_si16);
    f_Range2D_pst->XMax_si16 = g_mtl_Max_mac(f_P1_pst->X_si16, f_P2_pst->X_si16);
    f_Range2D_pst->YMin_si16 = g_mtl_Min_mac(f_P1_pst->Y_si16, f_P2_pst->Y_si16);
    f_Range2D_pst->YMax_si16 = g_mtl_Max_mac(f_P1_pst->Y_si16, f_P2_pst->Y_si16);

    return;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_SideLengthOfRightTriangle_ui16                     *//**
\brief         Return length of a cathethus of a triangle
\TODO:  This function is a duplicate to g_xpgCalcCathetus_ui16 and
        is to be replaced by it because this function is not overflow-safe!!!
*//*-------------------------------------------------------------------------
I N P U T	                                                            *//**
\param[in]     f_hypotenuse_ui16
\param[in]     f_otherside_ui16
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        other cathetus
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        oas2tm
*//*************************************************************************/
UInt16 g_mtl_SideLengthOfRightTriangle_ui16(const UInt16 f_hypotenuse_ui16,
                                            const UInt16 f_otherside_ui16)
{
    return g_mtl_CalcCathetus_ui16(f_hypotenuse_ui16, f_otherside_ui16);
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_IntersectionCircleLine_si8                         *//**
\brief         This function calculates the intersection points of a circle
               and a line.
\details       Details: Refer to APG library function documentation

Unit Test
\see m_Test_IntersectionCircleLine_ui8()
*//*-------------------------------------------------------------------------
I N P U T                                                              *//**
\param[in]    f_Circle_pst  circle
\param[in]    f_line_pst    support point and orientation of the line.
                  The orientation angle has to be in range [0..2pi). cos and
                  sin values have to be provided. X/Y has to be in F10 format
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    f_P1_st    intersection point 1 (if return value > 0, undefined otherwise)
\param[out]    f_P2_st    intersection point 2 (if return value == 2)
                If an overflow occurred calculating one intersection point,
                the remaining good solution will be returned in P1, the
                number of solutions will be set to 1 and P2 will contain
                (GD_MAX_SI16/GD_MAX_SI16). If only one solution exists, P2 will
                be set to (0/0)
\return        number of intersection points (0, 1 or 2).
               If an overflow occurred in both calculations, -1 will be returned.

*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        str2lr            Arwed Starke          AE-BE/ESU3
*//*************************************************************************/
SInt8 g_mtl_IntersectionCircleLine_si8(gType_mtlCircleU16_st const *f_Circle_pst,
                                       gType_mtlLineSinCos_st const *f_line_pst,
                                       gType_mtlPoint_st *f_P1_pst,
                                       gType_mtlPoint_st *f_P2_pst)
{
    Boolean l_overflowSolution1_bl = FALSE;
    SInt8 l_numSolutions_si8       = 0;
    SInt32 l_temp_si32; // F10
    // Distanz vom Stuetzpunkt zum Lotpunkt des Kreismittelpunkts auf der Geraden
    SInt32 l_distOnLine_si32; // F10 m
    // Distanz von den Kreisschnittpunkten zum Lotpunkt des Kreismittelpunkts auf
    // der Geraden
    SInt32 l_distToLot_si32; // F10

    // distance between circle center and support point of the line
    const SInt16 l_dx_si16 = f_Circle_pst->P_st.X_si16 - f_line_pst->P_st.X_si16;
    const SInt16 l_dy_si16 = f_Circle_pst->P_st.Y_si16 - f_line_pst->P_st.Y_si16;

    // Berechne Lotlaenge von Kreismittelpunkt auf Gerade
    l_temp_si32 = (SInt32)g_mtl_SinCosMul_si16(
        l_dy_si16, f_line_pst->Phi_st.Cos_si16); // Input Scale
    l_temp_si32 -= (SInt32)g_mtl_SinCosMul_si16(
        l_dx_si16, f_line_pst->Phi_st.Sin_si16); // Input Scale

    // calculate l^2 = R^2 - lot^2
    if ((SInt32)f_Circle_pst->R_ui16 >= l_temp_si32)
    {
        // otherwise square root would be negative -> distance between circle center
        // and line is larger than radius, no solutions.

        l_distToLot_si32 = ((SInt32)f_Circle_pst->R_ui16 + l_temp_si32);
        l_distToLot_si32 *= ((SInt32)f_Circle_pst->R_ui16 - l_temp_si32); // F20

        if (l_distToLot_si32 < 0) // numerical overflow
        {
            l_numSolutions_si8 = -1;
        }
        else
        {
            l_numSolutions_si8 = 1; // at least 1 solution
            f_P2_pst->X_si16   = 0;
            f_P2_pst->Y_si16   = 0;

            l_distToLot_si32 =
                (SInt32)g_mtl_Sqrt_u32_ui16((UInt32)l_distToLot_si32); // F10

            l_distOnLine_si32 =
                g_mtl_SinCosMul_si16(l_dx_si16, f_line_pst->Phi_st.Cos_si16);
            l_distOnLine_si32 +=
                g_mtl_SinCosMul_si16(l_dy_si16, f_line_pst->Phi_st.Sin_si16);

            // x_1 = x_0 + cos(phi) * (d + l)
            l_temp_si32 = l_distOnLine_si32 + l_distToLot_si32;        // F10
            l_temp_si32 *= f_line_pst->Phi_st.Cos_si16;                // F10 * F14 = F24
            l_temp_si32 = g_mtl_ReducePrecision_si32(l_temp_si32, 14); // F10
            l_temp_si32 = l_temp_si32 + f_line_pst->P_st.X_si16;
            if ((l_temp_si32 > (SInt32)GD_MAX_SI16) ||
                (l_temp_si32 < (SInt32)GD_MIN_SI16))
            {
                l_overflowSolution1_bl = TRUE;
            }
            else
            {
                f_P1_pst->X_si16 = (SInt16)l_temp_si32;
            }

            // y_1 = y_0 + sin(phi) * (d + l)
            l_temp_si32 = l_distOnLine_si32 + l_distToLot_si32;        // F10
            l_temp_si32 *= f_line_pst->Phi_st.Sin_si16;                // F10 * F14 = F24
            l_temp_si32 = g_mtl_ReducePrecision_si32(l_temp_si32, 14); // F10
            l_temp_si32 = l_temp_si32 + f_line_pst->P_st.Y_si16;
            if ((l_temp_si32 > (SInt32)GD_MAX_SI16) ||
                (l_temp_si32 < (SInt32)GD_MIN_SI16))
            {
                l_overflowSolution1_bl = TRUE;
            }
            else
            {
                f_P1_pst->Y_si16 = (SInt16)l_temp_si32;
            }

            if (l_distToLot_si32 != 0) // 2nd solution exists
            {
                l_numSolutions_si8 = 2;

                // x_2 = x_0 + sin(phi) * (d - l)
                l_temp_si32 = l_distOnLine_si32 - l_distToLot_si32; // F10
                l_temp_si32 *= f_line_pst->Phi_st.Cos_si16;         // F10 * F14 = F24
                l_temp_si32 = g_mtl_ReducePrecision_si32(l_temp_si32, 14); // F10
                l_temp_si32 = l_temp_si32 + f_line_pst->P_st.X_si16;
                if ((l_temp_si32 > (SInt32)GD_MAX_SI16) ||
                    (l_temp_si32 < (SInt32)GD_MIN_SI16))
                {
                    l_numSolutions_si8 = 1; // 2nd solution not calculable, overflow
                }
                else
                {
                    f_P2_pst->X_si16 = (SInt16)l_temp_si32;
                }

                // y_2 = y_0 + sin(phi) * (d - l)
                l_temp_si32 = l_distOnLine_si32 - l_distToLot_si32; // F10
                l_temp_si32 *= f_line_pst->Phi_st.Sin_si16;         // F10 * F14 = F24
                l_temp_si32 = g_mtl_ReducePrecision_si32(l_temp_si32, 14); // F10
                l_temp_si32 = l_temp_si32 + f_line_pst->P_st.Y_si16;
                if ((l_temp_si32 > (SInt32)GD_MAX_SI16) ||
                    (l_temp_si32 < (SInt32)GD_MIN_SI16))
                {
                    l_numSolutions_si8 = 1; // 2nd solution not calculable, overflow
                }
                else
                {
                    f_P2_pst->Y_si16 = (SInt16)l_temp_si32;
                }

                if (l_numSolutions_si8 == 1) // special overflow treatment
                {
                    f_P2_pst->X_si16 = GD_MAX_SI16;
                    f_P2_pst->Y_si16 = GD_MAX_SI16;
                }
            }

            // pick good solution if solution 1 could not be computed
            if (l_overflowSolution1_bl != FALSE)
            {
                l_numSolutions_si8 -= 1;    // solution 1 does not count
                if (l_numSolutions_si8 > 0) // at least solution 2 is ok
                {
                    f_P1_pst->X_si16 = f_P2_pst->X_si16;
                    f_P1_pst->Y_si16 = f_P2_pst->Y_si16;
                }
                else
                {
                    l_numSolutions_si8 = -1; // overflow error everywhere
                }
            }
        }
    }
    return l_numSolutions_si8;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_CalcRadiusFromPointToCorner_si16                     *//**
\brief         Return the Y-coordinate (x of center of circle is 0) of a circle
               that fulfills the following conditions:
               * Point1 lies on the circle
               * CornerCircle lies within the circle
               * CornerCircle and the circle are tangential (have one common point)
              The function can be used to determine which radius is necessary to pass
              an obstacle point.
\details
\verbatim
             y ^    # # # # #
               |         (.)# # #
             --|------>    CornerCircle
               |      x           # #
               |                    # #. Point1
               |                       #
               | ,- Circle Center       #
               X                        #
\endverbatim

The function is tested within OOC unit test.

For further information read DOC\Geolib_Design.docx
*//*-------------------------------------------------------------------------
I N P U T	                                                              *//**
\param[in]     f_point_pst      coordinate of obstacle point [cm]
\param[in]     f_circle_pst     circle describing a vehicle corner [cm]
\param[in]     f_halfWidth_ui16  special handling of "almost-straight-drive"
              situations or situations where a turn in the other direction
              would be possible. Pass g_parGetParaAPG_XPG_Vehicle_HalfWidth_ui16
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\return       Y-coordinate of circle center (with X = 0) [cm]
              Special return values:
    GD_MAX_SI16-1
    GD_MIN_SI16+2 (R = GD_MIN_SI16+1 is for Straight driving)
    -> equivalent to "R_max, R_min"
    meaning: Straight driving results in collision, but the radius to pass is larger
    than GD_MAX_SI16-1
    GD_MAX_SI16, GD_MIN_SI16+1 -> straight driving ok to pass point
    0 -> not possible to pass

    Overflow safe for point in range (-3276, 3276)cm. Max. error ~18cm
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        wxa1lr, str2lr
*//*************************************************************************/
SInt16 g_mtl_CalcRadiusFromPointToCorner_si16(gType_mtlPoint_st const *f_point_pst,
                                              gType_mtlCircleU16_st const *f_circle_pst,
                                              UInt16 f_halfWidth_ui16)
{
    SInt16 l_result_si16;
    SInt32 l_temp1_si32, l_temp2_si32, l_SqrtTerm_si32;
    SInt32 l_a_si32, l_c_si32, l_beta_si32;
    SInt32 l_YCenter1_si32, l_YCenter2_si32; // solutions
    SInt32 l_resultTemp_si32;
    SInt32 const l_CoefSq_si32 = 64;
    SInt8 l_Scale1_si8         = 0;
    UInt8 l_NegExpTemp1_ui8;
    Boolean l_overflow_bl = FALSE;

    l_temp1_si32 = g_mtl_Square_mac((SInt32)f_point_pst->X_si16);
    l_temp1_si32 -= g_mtl_Square_mac((SInt32)f_circle_pst->P_st.X_si16);
    l_temp1_si32 += g_mtl_Square_mac((SInt32)f_point_pst->Y_si16);
    l_temp1_si32 -= g_mtl_Square_mac((SInt32)f_circle_pst->P_st.Y_si16);
    l_temp1_si32 -= g_mtl_Square_mac((SInt32)f_circle_pst->R_ui16); // Scaling: S*S = S^2

    // temp1 = (x2^2 - x1^2 + y2^2 - y1^2 - R1^2) / (2 * R1)
    l_temp1_si32 = g_mtl_s32_Div_s32_si32(
        l_temp1_si32,
        (SInt32)(f_circle_pst->R_ui16 << 1)); // Scaling: S^2 / S =  S

    // temp2 = (y2 - y1) / R1
    l_temp2_si32 = (SInt32)(f_point_pst->Y_si16 - f_circle_pst->P_st.Y_si16) *
                   l_CoefSq_si32; // Scaling: S * c^2
    l_temp2_si32 = g_mtl_s32_Div_s32_si32(
        l_temp2_si32, f_circle_pst->R_ui16); // Scaling: S * c^2 / S = c^2

    if (g_mtl_Abs_mac(l_temp2_si32) > GD_MAX_SI16)
    {
        l_overflow_bl = TRUE;
    }

    // l_temp2_SQ_si32 = 64 * l_temp2_SQ_si32
    // a = temp2^2 - 1 = (temp2 - 1) * (temp2 + 1)  // Scaling: c^2 * c^2 = c^4
    l_a_si32 =
        (l_temp2_si32 - l_CoefSq_si32) * (l_temp2_si32 + l_CoefSq_si32); // Scaling: c^4
    l_a_si32 = g_mtl_s32_Div_s32_si32(l_a_si32, l_CoefSq_si32);          // Scaling: c^2

    // b = 2 * (y1 - temp1 * temp2) -> beta = b/2
    g_mtl_ScaleDown_si32_toFit_si16_vd(&l_temp1_si32, &l_Scale1_si8);
    l_NegExpTemp1_ui8 = (UInt8)g_mtl_Abs_mac(l_Scale1_si8);
    // Scaling: S * 2^-Scale1 * c^2 / (c^2 * 2^-Scale1) = S
    l_beta_si32 = g_mtl_s32_Div_s32_si32(
        l_temp1_si32 * l_temp2_si32,
        g_mtl_ReducePrecision_si32(l_CoefSq_si32, l_NegExpTemp1_ui8));
    l_beta_si32 = f_circle_pst->P_st.Y_si16 - l_beta_si32; // Scaling: S

    // max l_Scale1_si8 = 4. Scaling of l_c_si32: (S * 2^(-Scale1))^2 = S^2 *
    // 2^(-2*Scale1)
    l_c_si32 = g_mtl_Square_mac(l_temp1_si32);
    l_c_si32 -= g_mtl_ReducePrecision_si32(
        g_mtl_Square_mac((SInt32)f_circle_pst->P_st.X_si16), 2 * l_NegExpTemp1_ui8);
    l_c_si32 -= g_mtl_ReducePrecision_si32(
        g_mtl_Square_mac((SInt32)f_circle_pst->P_st.Y_si16), 2 * l_NegExpTemp1_ui8);

    /*************************************************************************/
    /* Run midnight formula to solve quadratic equation (special version)    */
    /*                                                                       */
    /* This is done in a special way to deal with the different scalings and */
    /* the different orders of magnitude in the inputs (usually a ~ 10^2,    */
    /* beta ~ 10^3, c ~ 10^9)                                                */
    /*************************************************************************/
    if (l_a_si32 == 0)
    {
        if (l_beta_si32 == 0)
        {
            // l_a_si32==0 && l_b_si32==0 means the equation a*R^2+b*R+C=0
            // C !=0 => No R can satisify the equation =>
            // No possible R to pass the given point
            l_result_si16 = gd_InvalidRadius_si16;
        }
        else if (g_mtl_Abs_mac(f_point_pst->Y_si16) <
                 g_mtl_Abs_mac(f_circle_pst->P_st.Y_si16))
        {
            // solution = -c / b = (-c/2) / (b/2) = (-c/2) / beta = -c / (2 * beta)
            // Scaling: [S^2 * 2^(-2*Scale1)] * 2^(2*Scale1) / S = S
            l_result_si16 =
                (SInt16)((-l_c_si32 / l_beta_si32) * (1 << l_NegExpTemp1_ui8));
        }
        else
        {
            if (g_mtl_Abs_mac(f_point_pst->Y_si16) < (SInt16)f_halfWidth_ui16)
            {
                // Kappa To left
                if (f_circle_pst->P_st.Y_si16 < 0)
                {
                    l_result_si16 = gd_MaxRadius_si16;
                }
                // Kappa To right
                else
                {
                    l_result_si16 =
                        -gd_MaxRadius_si16; //= -32766 //R=-32767 is straight driving
                }
            }
            else
            {
                // straight driving
                l_result_si16 = gd_RadiusStraight_si16;
            }
        }
    }
    else
    {
        // Overflow handling
        SInt8 l_ScaleB_si8 = 0;

        // calculate square root term = beta^2 - a * c
        // Scaling of beta: S         Scaling of a: Coef^2         Scaling of c:
        // [S^2 * 2^(-2*Scale1)] Putting it together: beta^2 * S^2 - a * c * S^2 *
        // 2^(-2*Scale1) * Coef^2 / (2^(-2*Scale1) * Coef^2) = [beta^2 - a * c] *
        // S^2 l_beta_si32^2 - l_a_si32 * l_c_si32 * (2^(2*Scale1) / Coef^2) =
        // [beta^2 - a * c] * S^2

        // and now with Beta scaling!
        g_mtl_ScaleDown_si32_toFit_si16_vd(&l_beta_si32, &l_ScaleB_si8);

        // Scaling of beta: S * 2^(-ScaleB)        Scaling of a: Coef^2 Scaling of
        // c: [S^2 * 2^(-2*Scale1)] Putting it together: beta^2 * S^2 *
        // 2^(-2*ScaleB) - 2^(-2*ScaleB) * a * c * S^2 * 2^(-2*Scale1) * Coef^2 /
        // (2^(-2*Scale1) * Coef^2) = [beta^2 - a * c] * S^2 * 2^(-2*ScaleB)
        // l_beta_si32^2 - l_a_si32 * l_c_si32 * 2^(2*Scale1) / (2^(2*ScaleB) *
        // Coef^2) = [beta^2 - a * c] * S^2

        l_SqrtTerm_si32 =
            l_c_si32 /
            (l_CoefSq_si32 * (SInt32)(1 << (UInt16)g_mtl_Abs_mac(2 * l_ScaleB_si8)));
        l_SqrtTerm_si32 *= (l_a_si32 * (SInt32)(1 << (2 * l_NegExpTemp1_ui8)));
        l_SqrtTerm_si32 = g_mtl_Square_mac(l_beta_si32) -
                          l_SqrtTerm_si32; // Scaling: S^2 * 2^(-2*ScaleB)

        if (l_SqrtTerm_si32 < 0)
        {
            // not possible for tangential passing
            l_result_si16 = gd_InvalidRadius_si16;
        }
        else
        {
            l_SqrtTerm_si32 = (SInt32)g_mtl_Sqrt_u32_ui16((UInt32)l_SqrtTerm_si32);
            // calculate the two possible circle center positions: R = (- beta +-
            // SqrtTerm) / a Scaling of beta: S * 2^-ScaleB        Scaling of
            // SqrtTerm: S * 2^-ScaleB
            l_YCenter1_si32 = -l_beta_si32 + l_SqrtTerm_si32;
            l_YCenter2_si32 = -l_beta_si32 - l_SqrtTerm_si32; // Scaling: S * 2^-ScaleB

            // choose optimal order of operations for maximum precision without
            // overflow
            if (g_mtl_Abs_mac(l_YCenter1_si32) < (GD_MAX_SI32 >> 5))
            {
                // Scaling of a: Coef^2
                // Putting it together:       add this ----v
                // R1 * S * 2^-ScaleB / (a * Coef^2)   * (Coef^2 * 2^ScaleB) = R1/a * S
                l_YCenter1_si32 =
                    g_mtl_s32_Div_s32_si32(l_CoefSq_si32 * l_YCenter1_si32, l_a_si32) *
                    (1 << (UInt8)g_mtl_Abs_mac(l_ScaleB_si8));
            }
            else
            {
                l_YCenter1_si32 = g_mtl_s32_Div_s32_si32(l_YCenter1_si32, l_a_si32) *
                                  (1 << (UInt8)g_mtl_Abs_mac(l_ScaleB_si8));
                l_YCenter1_si32 *= l_CoefSq_si32;
            }

            if (g_mtl_Abs_mac(l_YCenter2_si32) < (GD_MAX_SI32 >> 5))
            {
                l_YCenter2_si32 =
                    g_mtl_s32_Div_s32_si32(l_CoefSq_si32 * l_YCenter2_si32, l_a_si32) *
                    (1 << (UInt8)g_mtl_Abs_mac(l_ScaleB_si8));
            }
            else
            {
                l_YCenter2_si32 = g_mtl_s32_Div_s32_si32(l_YCenter2_si32, l_a_si32) *
                                  (1 << (UInt8)g_mtl_Abs_mac(l_ScaleB_si8));
                l_YCenter2_si32 *= l_CoefSq_si32;
            }

            // Scaling of radius1, radius2 is now original scaling S. Great, let's
            // never do that again...

            {
                // choose the suitable radius by using the original equation:
                // sqrt(x2^2 + (y2 - y0)^2) = sqrt(x1^2 + (y1 - y0)^2) + R1

                // so this must hold for a valid YCenter (y0):
                // x2^2 + (y2 - y0)^2  >  x1^2 + (y1 - y0)^2
                // (x1^2 + (y1 - y0)^2 - x2^2 - (y2 - y0)^2 ) < 0   must hold
                SInt32 l_foo_si32, l_bar_si32;

                // scale the critical values down (YCenter1 can be larger than SInt16
                // range)
                l_Scale1_si8 = 0;
                l_foo_si32   = (SInt32)f_circle_pst->P_st.Y_si16 - l_YCenter1_si32;
                l_bar_si32   = (SInt32)f_point_pst->Y_si16 - l_YCenter1_si32;
                g_mtl_ScaleDown2_si32_vd(13, &l_foo_si32, &l_bar_si32, &l_Scale1_si8);
                l_Scale1_si8 = -l_Scale1_si8; // make positive

                l_a_si32 = g_mtl_Square_mac(g_mtl_ReducePrecision_si16(
                    f_circle_pst->P_st.X_si16, (UInt8)l_Scale1_si8));
                l_a_si32 -= g_mtl_Square_mac(
                    g_mtl_ReducePrecision_si16(f_point_pst->X_si16, (UInt8)l_Scale1_si8));
                l_a_si32 += g_mtl_Square_mac(l_foo_si32);
                l_a_si32 -= g_mtl_Square_mac(l_bar_si32);

                // scale down the check value for solution 2 as well. The scaling of
                // solution 1 and 2 don't have to match since we only compare check
                // value a and c against zero later
                l_Scale1_si8 = 0;
                l_foo_si32   = (SInt32)f_circle_pst->P_st.Y_si16 - l_YCenter2_si32;
                l_bar_si32   = (SInt32)f_point_pst->Y_si16 - l_YCenter2_si32;
                g_mtl_ScaleDown2_si32_vd(13, &l_foo_si32, &l_bar_si32, &l_Scale1_si8);
                l_Scale1_si8 = -l_Scale1_si8; // make positive

                l_c_si32 = g_mtl_Square_mac(g_mtl_ReducePrecision_si16(
                    f_circle_pst->P_st.X_si16, (UInt8)l_Scale1_si8));
                l_c_si32 -= g_mtl_Square_mac(
                    g_mtl_ReducePrecision_si16(f_point_pst->X_si16, (UInt8)l_Scale1_si8));
                l_c_si32 += g_mtl_Square_mac(l_foo_si32);
                l_c_si32 -= g_mtl_Square_mac(l_bar_si32);
            }

            if ((l_a_si32 < 0) && (l_c_si32 > 0))
            {
                l_resultTemp_si32 = l_YCenter1_si32; // only solution 1 fulfills condition
            }
            else if ((l_a_si32 > 0) && (l_c_si32 < 0))
            {
                l_resultTemp_si32 = l_YCenter2_si32; // only solution 2 fulfills condition
            }
            else if ((l_a_si32 < 0) && (l_c_si32 < 0))
            {
                // both solutions fulfill condition, check based on implicit knowledge
                // which one to use Right front corner for kappa to left
                if (f_circle_pst->P_st.Y_si16 < 0)
                {
                    if (l_YCenter1_si32 > 0)
                    {
                        l_resultTemp_si32 = l_YCenter1_si32;
                    }
                    else if (l_YCenter2_si32 > 0)
                    {
                        l_resultTemp_si32 = l_YCenter2_si32;
                    }
                    else
                    {
                        // impossible to pass
                        l_resultTemp_si32 = gd_InvalidRadius_si16;
                    }
                }
                // left front corner for Kappa to right
                else
                {
                    if (l_YCenter1_si32 < 0)
                    {
                        l_resultTemp_si32 = l_YCenter1_si32;
                    }
                    else if (l_YCenter2_si32 < 0)
                    {
                        l_resultTemp_si32 = l_YCenter2_si32;
                    }
                    else
                    {
                        l_resultTemp_si32 = (SInt32)0;
                    }
                }
            }
            else // both solutions invalid
            {
                if (l_YCenter2_si32 == l_YCenter1_si32)
                {
                    l_resultTemp_si32 = l_YCenter1_si32;
                }
                else
                {
                    // not possible for tangential passing
                    l_resultTemp_si32 = gd_InvalidRadius_si16;
                }
            }

            // Special case 1: object on the left side to left corner
            if ((f_circle_pst->P_st.Y_si16 > 0) &&
                (f_point_pst->Y_si16 > (SInt16)f_halfWidth_ui16))
            {
                if ((l_YCenter1_si32 > 0) && (l_YCenter2_si32 > 0))
                {
                    l_resultTemp_si32 = g_mtl_Max_mac(l_YCenter1_si32, l_YCenter2_si32);
                }
            }
            // Special case 2: object on the right side to right corner
            if ((f_circle_pst->P_st.Y_si16 < 0) &&
                (f_point_pst->Y_si16 < -(SInt16)f_halfWidth_ui16))
            {
                if ((l_YCenter1_si32 < 0) && (l_YCenter2_si32 < 0))
                {
                    l_resultTemp_si32 = g_mtl_Min_mac(l_YCenter1_si32, l_YCenter2_si32);
                }
            }

            // special case 3: not possible for tangential passing
            if ((f_circle_pst->P_st.Y_si16 > 0) &&
                (f_point_pst->Y_si16 <= f_halfWidth_ui16) && (l_resultTemp_si32 > 0))
            {
                l_resultTemp_si32 = gd_InvalidRadius_si16;
            }
            if ((f_circle_pst->P_st.Y_si16 < 0) &&
                (f_point_pst->Y_si16 > -(SInt16)f_halfWidth_ui16) &&
                (l_resultTemp_si32 < 0))
            {
                l_resultTemp_si32 = gd_InvalidRadius_si16;
            }

            // special case 4: can pass with straight driving
            if ((l_resultTemp_si32 >= GD_MAX_SI16) || (l_resultTemp_si32 <= -GD_MAX_SI16))
            {
                if (g_mtl_Abs_mac(f_point_pst->Y_si16) < (SInt16)f_halfWidth_ui16)
                {
                    // Kappa To left
                    if (f_circle_pst->P_st.Y_si16 < 0)
                    {
                        l_result_si16 = gd_MaxRadius_si16;
                    }
                    // Kappa To right
                    else
                    {
                        l_result_si16 = -gd_MaxRadius_si16;
                    }
                }
                else
                {
                    // straight driving
                    l_result_si16 = gd_RadiusStraight_si16;
                }
            }
            else
            {
                l_result_si16 = (SInt16)l_resultTemp_si32;
            }
        }
    }

    if (l_overflow_bl != FALSE)
    {
        l_result_si16 = gd_InvalidRadius_si16;
    }

    return l_result_si16;
}

/****************************************************************************
F U N C T I O N    D E S C R I P T I O N
-----------------------------------------------------------------------------
Function name  g_mtl_GetRadiusFromTwoPoints_si32                         *//**
\brief         Return the vehicle radius (x of center of circle is 0)
               which crosses given two points.
\details       [detailed description, formula]
*//*-------------------------------------------------------------------------
S I D E  E F F E C T S                                                  *//**
\pre           [pre-conditions]
\post          [post-conditions]
*//*-------------------------------------------------------------------------
I N P U T                                                              *//**
\param[in]     f_point1_pst
\param[in]     f_point2_pst
*//*-------------------------------------------------------------------------
O U T P U T                                                             *//**
\param[out]    -
\return        Radius through both points
\throw         [error values]
*//*-------------------------------------------------------------------------
A U T H O R  I D E N T I T Y                                            *//**
\author        oas2tm, wxa1lr
*//*************************************************************************/
SInt32 g_mtl_GetRadiusFromTwoPoints_si32(const gType_mtlPoint_st *f_point1_pst,
                                         const gType_mtlPoint_st *f_point2_pst)
{
    SInt32 l_result_si32;
    SInt32 l_numerator_si32, l_demonimator_si32;

    SInt32 l_x1SQ_si32, l_x2SQ_si32, l_y1SQ_si32, l_y2SQ_si32;

    l_x1SQ_si32 = g_mtl_Square_mac((SInt32)f_point1_pst->X_si16);
    l_y1SQ_si32 = g_mtl_Square_mac((SInt32)f_point1_pst->Y_si16);
    l_x2SQ_si32 = g_mtl_Square_mac((SInt32)f_point2_pst->X_si16);
    l_y2SQ_si32 = g_mtl_Square_mac((SInt32)f_point2_pst->Y_si16);

    l_numerator_si32 = (l_x1SQ_si32 + l_y1SQ_si32) - (l_x2SQ_si32 + l_y2SQ_si32);
    l_demonimator_si32 =
        2 * ((SInt32)f_point1_pst->Y_si16 - (SInt32)f_point2_pst->Y_si16);

    l_result_si32 = g_mtl_s32_Div_s32_si32(l_numerator_si32, l_demonimator_si32);

    return (l_result_si32);
}
/*!\endcond */

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 506, 3103, 2217
#endif

#endif /*(GS_MTL_GEOLIB != SW_OFF)*/

/*!@}*/ /*================== EoF (mathlib_geometry.c) ===========*/
