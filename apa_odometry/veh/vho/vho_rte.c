/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: vho_rte.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#ifdef VHO_COMPILE_GEN4
#define COMPONENT_VHO

#include "sig_api.h"
#include "vho.h"     // SMS Include Structure: ok
#include "vho_api.h" // SMS Include Structure: ok
#include <string.h>  // SMS Include Structure: ok

#ifndef VISIBILITY
#error ('compiler switch VISIBILITY is not defined')
#endif

typedef struct
{
    gType_VHOVehicleState_st VehicleState_st;
    gType_VHOCanSig_st CanSig_st;
} mType_rteVHO_st;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
VISIBILITY mType_rteVHO_st m_rteVHO_st;
#endif

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
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
gType_VHOVehicleState_st g_rteVehicleState_st(void)
{
    return m_rteVHO_st.VehicleState_st;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief
 * @details
 *
 * @param         f_SAS_pst
 * @return        Boolean
 *
 * @note
 * @see
 * @warning
 */
Boolean g_rteVHOSteeringWheelAngle_bl(gType_SAS_st *f_SAS_pst)
{
    f_SAS_pst->SAS_Signal_si16 = (SInt16)g_VHOCanSig_st.SWA_st.ValCor_si32;
    f_SAS_pst->SAS_Tick_ui16   = g_VHOCanSig_st.SWA_Tick_ui16;
    return (Boolean)g_VHOCanSig_st.SWA_ValidFlag_bl;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
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
void g_rteVhoCopy_vd(void)
{
    (void)memcpy(&m_rteVHO_st.VehicleState_st, &g_VHOVehicleState_st,
                 sizeof(gType_VHOVehicleState_st));

    (void)memcpy(&m_rteVHO_st.CanSig_st, &g_VHOCanSig_st, sizeof(gType_VHOCanSig_st));
}

#endif // #ifdef VHO_COMPILE_GEN4
