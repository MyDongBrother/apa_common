/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: sigif.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/
#ifndef MODULE_LOG_NAME
#define MODULE_LOG_NAME "sig_if"
#endif

/*--------------------------------------------------------------------------*/
/*                                                                          */
/*include files                                                              */
/*                                                                          */
/*--------------------------------------------------------------------------*/

// #include <uss/psd/psd_api.h>
#ifdef md_RUN_ON_POSIX
#include <pa_components/debugprintf/debug_log.h>
#include <sys/time.h>
#endif // md_RUN_ON_POSIX

// #include <uss/pdc/ap_pdc_api.h>
// #include <uss/psd/psm/psm_ussps_api.h>

#include "appl_canSignal.h"
#include "sigif_api.h"
/*--------------------------------------------------------------------------*/
/*                                                                          */
/* FUNCTION DECLARE PUBLIC                                                  */
/*                                                                          */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*                                                                          */
/* FUNCTION DECLARE PRIVATE                                                 */
/*                                                                          */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*                                                                          */
/* VARIABLE DEFINE                                                          */
/*                                                                          */
/*--------------------------------------------------------------------------*/

// adi
/**
 * @brief
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
static gType_adiSigIf_st g_SigIfAdi_st = {

#if (ODO_EXTERNAL_CFG == SW_ON)
    .VHOGetUpdateState_pf = m_VHOGetUpdateState_external_en,
    .adiGetVehPosOdo_pf   = m_adiGetVehPosOdo_external_bl,
#else
    .VHOGetUpdateState_pf = g_VHOGetUpdateState_en,
    .adiGetVehPosOdo_pf   = g_adiGetVehPosOdo_bl,
#endif

};

/*--------------------------------------------------------------------------*/
/*                                                                          */
/* MACRO DECLARE                                                            */
/*                                                                          */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*                                                                          */
/* FUNCTION - PUBLIC                                                        */
/*                                                                          */
/*--------------------------------------------------------------------------*/

// const gType_SigIfModule_st* g_SigIfModule_pst(void) {
//   return &g_SigIfModule_st;
// }
/**
 * @brief
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
const gType_adiSigIf_st *g_adiSigIfGet_pst(void) { return &g_SigIfAdi_st; }

/******************************* Init  *****************************/
/**
 * @brief
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_SigIfInit_vd(void)
{
    g_vhsInit_vd();

#if (ODO_EXTERNAL_CFG == SW_OFF)
    g_vhoInit_vd();
#endif
}

/************************ odo process  *****************************/
/**
 * @brief
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_SigIf_odoProcessing_vd(void)
{
#if (ODO_EXTERNAL_CFG == SW_OFF)
    g_vhoProcessing_vd();
#else
    m_odoExternal_processing_vd();
#endif
}
