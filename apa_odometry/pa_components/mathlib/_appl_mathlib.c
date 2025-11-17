/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: ap_psd.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/
#define COMPONENT_MTL

#define APPL_MATHLIB_C_TEMPL_NUMBER 0x090921

#include <pa_components/base_type/global_include.h>

#include "pa_components/mathlib/mathlib_api.h" // optional, may not be needed here

#if (LS_MTL_TESTHARNESS != SW_OFF)
#include <mathlib_testharness.h>
#endif

/*--------------------------------------------------------------------------*/
/*- Check if Compiler Switches (utilised in this file) are defined         -*/
/*--------------------------------------------------------------------------*/
#ifndef SW_ON
#error ('global switch SW_ON is not defined')
#endif
#ifndef SW_OFF
#error ('global switch SW_OFF is not defined')
#endif
#ifndef LS_MTL_TESTHARNESS
#error ('switch LS_MTL_TESTHARNESS is not defined')
#endif

#ifndef GS_MTL_USE_ASM
#error ('switch GS_MTL_USE_ASM is not defined')
#endif

#ifndef GS_MTL_USE_ERROR_HOOK
#error ('switch GS_MTL_USE_ERROR_HOOK is not defined')
#endif

#if (APPL_MATHLIB_H_TEMPL_NUMBER != APPL_MATHLIB_H_REF)
#error ('Wrong version of H TEMPLATE FILE - check whether the template has changed')
#endif
#if (APPL_MATHLIB_C_TEMPL_NUMBER != APPL_MATHLIB_C_REF)
#error ('Wrong version of C TEMPLATE FILE - check whether the template has changed')
#endif
/*--------------------------------------------------------------------------*/
/*- local symbolic constants                                               -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- local macros                                                           -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- local data types                                                       -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- local data                                                             -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- prototypes of local functions                                          -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- global data objects                                                    -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- definition of local functions                                          -*/
/*--------------------------------------------------------------------------*/

#if (GS_MTL_USE_ERROR_HOOK != SW_OFF)
/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_ErrorHook_vd
|     Purpose: error hook if mathlib detects a malfunction
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: none
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: none
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
void g_mtl_ErrorHook_vd(UInt8 *f_Msg_pui8)
{
    Boolean l_Test_bl = FALSE;

    // endless loop
    do
    {
        l_Test_bl = TRUE;
    } while (l_Test_bl != FALSE);

} // mtl_ErrorHook_vd
#endif // #if( GS_MTL_USE_ERROR_HOOK != SW_OFF )

/*========================= EoF (_appl_mathlib.c) ========================*/
