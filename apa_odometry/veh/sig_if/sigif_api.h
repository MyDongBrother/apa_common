/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: sigif_api.h
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/
#ifndef _SIG_IF_API_H
#define _SIG_IF_API_H
/*--------------------------------------------------------------------------*/
/*- MACRO DECLARE                                                          -*/
/*--------------------------------------------------------------------------*/
#ifndef ODO_EXTERNAL_CFG
#define ODO_EXTERNAL_CFG SW_OFF
#endif

/*--------------------------------------------------------------------------*/
/*- include files                                                           -*/
/*--------------------------------------------------------------------------*/
#include <pa_components/base_type/global_include.h>

#include <veh/vhs/vhs_api.h>
#include "signal_input.h"
#include "signal_output.h"
#if (ODO_EXTERNAL_CFG == SW_ON)
#include "odo_external_adi.h"
#else
#include <veh/vho/vho_api.h>
#endif
/*--------------------------------------------------------------------------*/
/*- TYPE DECLARE                                                           -*/
/*--------------------------------------------------------------------------*/

// adi

typedef gType_vhoUpdateState_en (*gType_VHOGetUpdateState_pf)(void);
typedef Boolean (*gType_adiGetVehPosOdo_pf)(gType_VehPosOdo_st *f_VehPosOdo_pst);

typedef void (*gType_SigIfProcessing_pf)(void);

typedef struct gTag_adiSigIf_st
{
    // adi
    gType_VHOGetUpdateState_pf VHOGetUpdateState_pf;
    gType_adiGetVehPosOdo_pf adiGetVehPosOdo_pf;

} gType_adiSigIf_st, *gType_adiSigIf_pst;
/*--------------------------------------------------------------------------*/
/*- EXTERN API                                                             -*/
/*--------------------------------------------------------------------------*/
// extern const gType_SigIfModule_st* g_SigIfModule_pst(void);
extern const gType_adiSigIf_st *g_adiSigIfGet_pst(void);
extern void g_SigIfInit_vd(void);
extern void g_SigIfInputProcessing_vd(void);
extern void g_SigIfOutputProcessing_vd(void);
extern void g_SigIf_odoProcessing_vd(void);

#endif