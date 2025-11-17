/**
 * @file dataOutput_adi.c
 * @brief  //TODO
 * @author xin.shi (xin.shi@bm-intelligent.com)
 * @version 1.0
 * @date 2022-08-30
 *
 * @copyright Copyright (c) 2022  bm-intelligent
 *
 * @par 修改日志:
 *
 * Date        Version  Author   Description
 * 2022-08-30 1.0     shixin      First version
 *
 */

#ifndef MODULE_LOG_NAME
#define MODULE_LOG_NAME "dataOutput_adi"
#endif

#define COMPONENT_PSD
/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/
#include <pa_components/parameter/par_api.h>

#ifdef md_RUN_ON_POSIX
#include <pa_components/debugprintf/debug_log.h>
#include <sys/time.h>
#endif // md_RUN_ON_POSIX

#include "dataInput_adi.h"
#include "dataOutput_adi.h"
#include <veh/sig_if/sigif_api.h>
/*--------------------------------------------------------------------------*/
/*- FUNCTION DECLARE PUBLIC                                                -*/
/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
/*- FUNCTION DECLARE PRIVAT                                                -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- module implementations                                                 -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- FUNCTION  PUBLIC                                                       -*/
/*--------------------------------------------------------------------------*/
/*************************************************************
 * @brief  获取ODO数据接口
 * @author rain (rain.zhang@bm-intelligent.com)
 * @param[in] f_VehOdoFrame_pst
 ************************************************************/
void g_adiGetMsgOdo_vd(gType_VehOdoFrame_st *f_VehOdoFrame_pst)
{
    if (f_VehOdoFrame_pst != NULL)
    {
        gType_VehPosOdo_st l_VehiclePos_st = {0};
        g_adiSigIfGet_pst()->adiGetVehPosOdo_pf(&l_VehiclePos_st);
        f_VehOdoFrame_pst->VehPosOdoSMm_si32      = l_VehiclePos_st.SPosMM_si32;
        f_VehOdoFrame_pst->VehPosOdoXMm_si32      = l_VehiclePos_st.XPosMM_si32;
        f_VehOdoFrame_pst->VehPosOdoYMm_si32      = l_VehiclePos_st.YPosMM_si32;
        f_VehOdoFrame_pst->VehPosOdoYawAngle_ui32 = l_VehiclePos_st.YawAngle_ui32;
        f_VehOdoFrame_pst->ODOTimeStamp_si64      = g_GetTimeMs_ui16();
        f_VehOdoFrame_pst->StructLen_ui16         = sizeof(gType_VehOdoFrame_st);
    }
}

/**
 * @brief Get the structure size of a parameter by calling getParameterStructSize.
 *
 * @see getParameterStructSize
 */
UInt16 g_adiGetParameterStructSize_vd(UInt8 f_parameterID_ui8)
{
    UInt16 l_ret_cluster_size_ui16 = 0;
    l_ret_cluster_size_ui16        = g_getParameterStructSize_ui16(f_parameterID_ui8);
    return l_ret_cluster_size_ui16;
}

/**
 * @brief Get the number of variants for a parameter by calling getParameterVariantCount.
 *
 * @see getParameterVariantCount
 */
UInt8 g_adiGetParameterVariantCount_ui8(UInt8 f_parameterID_ui8)
{
    UInt8 l_ret_number_subID_ui8 = 0;
    l_ret_number_subID_ui8       = g_getParameterVariantCount_ui8(f_parameterID_ui8);
    return l_ret_number_subID_ui8;
}
/**
 * @brief Gets the number of parameter clusters.
 *
 * This function returns the total number of parameter clusters defined by the constant
 * `gd_PAR_NUM_CLUSTERS_uix`.
 *
 * @return UInt16 Returns the number of parameter clusters.
 *
 * @see gd_PAR_NUM_CLUSTERS_uix
 */
UInt16 g_adiGetParameterNumberOfClusters_ui16(void) { return gd_PAR_NUM_CLUSTERS_uix; }
