/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: odo_external_adi.c
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
#include <pa_components/base_type/global_include.h>

#include "odo_external_adi.h"

/*--------------------------------------------------------------------------*/
/*                                                                          */
/* FUNCTION DECLARE PUBLIC                                                  */
/*                                                                          */

gType_VehPosOdo_st m_VehPosOdo_external_st;
gType_vhoUpdateState_en m_vhoUpdateState_external_en;

gType_vhoUpdateState_en m_VHMGetUpdateState_external_en(void)
{

#if 0

  return m_vhoUpdateState_external_en;

#else

    return g_vhoStateUpdated_enm;

#endif
}
/**
 * @brief  get odo data from external module, not from alg vho module.
 * @details
 *
 * @param  f_VehPosOdo_pst ,  data struct to save odo.
 * @return
 *
 * @note
 * @see
 * @warning
 */
Boolean m_adiGetVehPosOdo_external_bl(gType_VehPosOdo_st *f_VehPosOdo_pst)
{

    if (f_VehPosOdo_pst == NULL)
    {
        return FALSE;
    }
    else
    {
        f_VehPosOdo_pst->SPosMM_si32 = m_VehPosOdo_external_st.SPosMM_si32; // 1LSB = 1mm
        f_VehPosOdo_pst->XPosMM_si32 = m_VehPosOdo_external_st.XPosMM_si32; // 1LSB = 1mm
        f_VehPosOdo_pst->YPosMM_si32 = m_VehPosOdo_external_st.YPosMM_si32; // 1LSB = 1mm
        f_VehPosOdo_pst->YawAngle_ui32 = m_VehPosOdo_external_st.YawAngle_ui32; // F22

        f_VehPosOdo_pst->WheelAngleFront_si16 =
            m_VehPosOdo_external_st.WheelAngleFront_si16; // 1LSB = 1.0 rad / 2^12

#if (GS_VHM_RUN_MODE == GS_VHM_CONSIDER_USS_DATA)

        f_VehPosOdo_pst->YawAngleYRSens_ui32 =
            m_VehPosOdo_external_st.YawAngleYRSens_ui32; // 1LSB = 1.0 rad / 2^22

#endif

        return TRUE;
    }
}

void m_odoExternal_processing_vd(void) {}