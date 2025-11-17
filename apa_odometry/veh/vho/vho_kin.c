/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: vho_kin.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#define COMPONENT_VHO

#include "vho_api.h"
#include <pa_components/base_type/global_include.h>
#include <pa_components/mathlib/mathlib_api.h>
#include <veh/vhs/vhs_api.h>

// whole module just used in case of RUN_MODE == CONSIDER_USS_DATA
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#include "vho.h"     // SMS Include Structure: ok
#include "vho_kin.h" // SMS Include Structure: ok

// #include <vho_loghdl.h>   // SMS Include Structure: ok
#include <string.h> // SMS Include Structure: ok

#ifndef VISIBILITY
#error ('compiler switch VISIBILITY is not defined')
#endif

#ifndef GS_4WS_VEHICLE
#error ('compiler switch GS_4WS_VEHICLE is not defined')
#endif

#if ((LS_VHO_DIST_FROM_REAR_WIC != LS_VHO_DIST_FROM_REAR_WIC_AVG) && \
     (LS_VHO_DIST_FROM_REAR_WIC != LS_VHO_DIST_FROM_REAR_WIC_ACKERMANN_STEER))
#error ('LS_VHO_DIST_FROM_REAR_WIC is not or wrong defined')
#endif
/*----------------------------------------------------------------*/
/*- local symbolic constants                                     -*/
/*----------------------------------------------------------------*/
#if (0)
#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL))
#define md_WINDOW_SIZE_VELO_EMA 2
#define md_RM_SIZE_VELO_EMA     (md_WINDOW_SIZE_VELO_EMA - 1)
#endif
#endif

#if (LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON)
#define md_WINDOW_SIZE_VELO_EMA 2
#define md_RM_SIZE_VELO_EMA     (md_WINDOW_SIZE_VELO_EMA - 1)
#endif

#if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS)
#define md_WINDOW_SIZE_VELO_EMA 5
#define md_RM_SIZE_VELO_EMA     (md_WINDOW_SIZE_VELO_EMA - 1)
#endif

#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) || \
     (LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON))

// p = 0.5 => md_FiltConstExpMovAvg_ui8 = 50 resolution = [0.01]
#define md_FiltConstExpMovAvg_ui8 ((UInt8)50)

#define md_RM_SIZE_VELO_SMOOTH 5

#define md_RM_SIZE_ACC_WIC 5

#define md_RM_SIZE_Ax_GRAD_ui8 (UInt8)3

/* resolution [0.01] */
#define md_MinGradFastResponse_ui8 ((UInt8)50)

#define md_MaxNumHighGrad_ui8 ((UInt8)2)

/* resolution [0.01] */
#define md_FiltConstLowGrad_ui8 ((UInt8)80)
/* resolution [0.01] */
#define md_FiltConstHighGrad_ui8 ((UInt8)30)

/*----------------------------------------------------------------*/
/*- local macros                                                 -*/
/*----------------------------------------------------------------*/

/*----------------------------------------------------------------*/
/*- local data types                                             -*/
/*----------------------------------------------------------------*/
static SInt16 m_AccRaw_si16;

/*----------------------------------------------------------------*/
/*- local objects                                                -*/
/*----------------------------------------------------------------*/
static UInt16 m_RMVeloEMA_pui16[md_RM_SIZE_VELO_EMA];
static UInt8 m_RMVeloEMANextElem_ui8;

static UInt16 m_RMVeloSmooth_pui16[md_RM_SIZE_VELO_SMOOTH];
static UInt8 m_RMVeloSmoothNextElem_ui8;
static UInt8 m_RMVeloNumStoredElem_ui8;

static SInt16 m_RMAccWIC_psi16[md_RM_SIZE_ACC_WIC];
static SInt16 m_RMAccWICNextElem_si16;
static SInt16 m_RMAccWICNumStoredElem_si16;

static SInt16 m_RMAxGrad_psi16[md_RM_SIZE_Ax_GRAD_ui8];
static SInt16 m_RMAxGradNextElem_si16;
static SInt16 m_RMAxGradNumStoredElem_si16;

static UInt16 m_TimeSinceLastVirtCanMsgMS_ui16;
static UInt16 m_VHOKinRawVelocityMperSF8_ui16;
static UInt32 m_DrivenDistanceSinceLastVirtCanMsgMMF8_ui32;

static SInt32 m_AccSum_si32;
static UInt8 m_AccCnt_ui8;

static UInt16 m_VeloExpMA_MperSF8_ui16;

static UInt8 l_HighGradient_ui8;

typedef struct
{
    SInt16 SumX_si16;
    SInt16 SumXX_si16;
    SInt32 SumY_si32;
    SInt32 SumXY_si32;
    SInt16 OldestElement_si16;
    SInt16 CramerDenominator_si16;
    Boolean Init_bl;
} gType_VHOLstSqFit_st;

/*----------------------------------------------------------------*/
/*- prototypes of local functions                                -*/
/*----------------------------------------------------------------*/
VISIBILITY void m_VHOKinRMAddAx_vd(SInt16 f_Ax_si16);

VISIBILITY UInt8 m_VHOKinNormVirtLSCan_ui8(SInt32 f_DrivenDistanceMMF8_si32,
                                           UInt16 *l_RawVelocityMperSF8_pui16);

VISIBILITY Boolean m_VHOKinRMBufCmplAcc_bl(void);

VISIBILITY Boolean m_VHOKinRMBufCmplVel_bl(void);

VISIBILITY UInt8 m_VHOKinGetFiltCoeff_ui8(void);

VISIBILITY SInt16 m_VHOKinRMGetVel_si16(UInt8 f_I_ui8);

VISIBILITY SInt16 m_VHOKinRMGetAccWIC_si16(UInt8 f_I_ui8);

VISIBILITY SInt16 m_VHOKinGetAccLstSqFit_si16(void);

VISIBILITY SInt16 m_VHOKinGetAccWICLstSqFit_si16(void);

VISIBILITY void m_VHOKinRMAddAccWic_vd(SInt16 f_AccWIC_si16);

VISIBILITY void m_VHOKinRMAddAxGrad_vd(SInt16 f_AxGrad_si16);

VISIBILITY UInt16 m_VHOKinSmoothVel_ui16(UInt16 f_Velo_ui16);

VISIBILITY SInt16 m_VHOKinExpMovAvgSigned_si16(SInt16 f_New_si16, SInt16 f_Old_si16,
                                               UInt8 f_p_ui8);

VISIBILITY UInt16 m_VHOKinExpMovAvg_ui16(UInt16 f_New_ui16, UInt16 f_Old_ui16,
                                         UInt8 f_p_ui8);

VISIBILITY Boolean m_VHOKinRMBufCmplAcc_bl(void);

VISIBILITY Boolean m_VHOKinRMBufCmplVel_bl(void);

VISIBILITY Boolean m_VHOKinRMBufCmplAccWIC_bl(void);

// VISIBILITY Boolean m_VHOKinRMBufCmplAxGrad_bl(void);

VISIBILITY SInt16 m_VHOKinLstSqFit_si16(gType_VHOLstSqFit_st *f_LstSqFit_st,
                                        UInt8 f_RM_Size_ui8,
                                        SInt16 (*f_GetRMElement)(UInt8 f_I_ui8));

#endif // #if( (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) ||
       //(LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON) )

VISIBILITY UInt32 m_VHOGetDrivenDistFrontAxleCenter_ui32(UInt16 f_WICDeltaLeft_ui16,
                                                         UInt16 f_WICDeltaRight_ui16,
                                                         SInt16 f_SA_rad_F12_si16);

#if ((GS_4WS_VEHICLE == SW_ON) && \
     (LS_VHO_DIST_FROM_REAR_WIC == LS_VHO_DIST_FROM_REAR_WIC_ACKERMANN_STEER))
VISIBILITY UInt32 m_VHOGetDrivenDistRearAxleCenter_ui32(UInt16 f_WICDeltaLeft_ui16,
                                                        UInt16 f_WICDeltaRight_ui16,
                                                        SInt16 f_SAF_rad_F12_si16,
                                                        SInt16 f_SAR_rad_F12_si16);
#endif // #if ( (GS_4WS_VEHICLE == SW_ON) && (LS_VHO_DIST_FROM_REAR_WIC ==
       //  LS_VHO_DIST_FROM_REAR_WIC_ACKERMANN_STEER) )

/*----------------------------------------------------------------*/
/*- global data objects                                          -*/
/*----------------------------------------------------------------*/

/*----------------------------------------------------------------*/
/*- definition of exported functions                             -*/
/*----------------------------------------------------------------*/

#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) || \
     (LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON))

void g_VHOKinInit_vd(void)
{
    (void)memset(&m_RMVeloEMA_pui16, 0, md_RM_SIZE_VELO_EMA * sizeof(UInt16));
    m_RMVeloEMANextElem_ui8 = 0;

    (void)memset(&m_RMVeloSmooth_pui16, 0, md_RM_SIZE_VELO_SMOOTH * sizeof(UInt16));
    m_RMVeloSmoothNextElem_ui8 = 0;
    m_RMVeloNumStoredElem_ui8  = 0;

    (void)memset(&m_RMAccWIC_psi16, 0, md_RM_SIZE_ACC_WIC * sizeof(SInt16));
    m_RMAccWICNumStoredElem_si16 = 0;
    m_RMAccWICNextElem_si16      = 0;

    (void)memset(&m_RMAxGrad_psi16, 0, md_RM_SIZE_Ax_GRAD_ui8 * sizeof(SInt16));
    m_RMAxGradNumStoredElem_si16 = 0;
    m_RMAxGradNextElem_si16      = 0;

    g_VHOVehicleState_st.VehicleAcceleration_si16 = 0;

    m_TimeSinceLastVirtCanMsgMS_ui16             = 0;
    m_DrivenDistanceSinceLastVirtCanMsgMMF8_ui32 = 0;

    m_AccSum_si32 = 0;
    m_AccCnt_ui8  = 0;

    m_VeloExpMA_MperSF8_ui16 = 0;

    l_HighGradient_ui8 = 0;
}

#endif // #if( (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) ||
       // (LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON) )

/**
 * @brief main function of kinematic module; purpose is to
 *        determine s, v and a over time
 * @details
 *
 * @param
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
UInt8 g_VHOKin_ui8(SInt32 f_DrivenDistanceMMF8_si32)
{
    UInt8 l_VirtCanMsgRcv_ui8;

/*
  supress QAC warning messages: 3197, 3198.
  The initialization of l_VirtCanMsgRcv_ui8 is important and not redundant.
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3197, 3198, 3204
#endif

    l_VirtCanMsgRcv_ui8 = ((UInt8)(0));

/*
  reactivate supressed QAC warning messages: 3197, 3198.
  The initialization of l_VirtCanMsgRcv_ui8 is important and not redundant.
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3197, 3198, 3204
#endif

#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) || \
     (LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON))

    l_VirtCanMsgRcv_ui8 = m_VHOKinNormVirtLSCan_ui8(f_DrivenDistanceMMF8_si32,
                                                    &m_VHOKinRawVelocityMperSF8_ui16);

    if (l_VirtCanMsgRcv_ui8 == ((UInt8)(1)))
    {
        UInt16 l_VeloSmoothedMperSF8_ui16;

        m_VeloExpMA_MperSF8_ui16 =
            m_VHOKinExpMovAvg_ui16(m_VHOKinRawVelocityMperSF8_ui16,
                                   m_VeloExpMA_MperSF8_ui16, md_FiltConstExpMovAvg_ui8);

        l_VeloSmoothedMperSF8_ui16 = m_VHOKinSmoothVel_ui16(m_VeloExpMA_MperSF8_ui16);

        g_VHOVehicleState_st.VehicleVelocity_ui16 = l_VeloSmoothedMperSF8_ui16;

        l_VeloSmoothedMperSF8_ui16 = (UInt16)g_mtl_s32_Div_s32_si32(
            ((SInt32)(l_VeloSmoothedMperSF8_ui16)) * ((SInt32)(100)), ((SInt32)(256)));

        // write preprocessed velocity to ring memory which serves as
        // the input for calculation of the acceleration
        m_VHOKinRMAddVel_vd(l_VeloSmoothedMperSF8_ui16);

        m_AccRaw_si16 =
            (SInt16)(((SInt32)(m_VHOKinGetAccLstSqFit_si16()) * ((SInt32)(130))) /
                     ((SInt32)(100)));

        {
            const UInt8 l_tmp_ui8 = m_VHOKinGetFiltCoeff_ui8();

            g_VHOVehicleState_st.VehicleAcceleration_si16 = m_VHOKinExpMovAvgSigned_si16(
                m_AccRaw_si16, g_VHOVehicleState_st.VehicleAcceleration_si16, l_tmp_ui8);
        }

        m_VHOKinRMAddAccWic_vd(g_VHOVehicleState_st.VehicleAcceleration_si16);

        g_VHORollRecog_st.GradAccWIC_si16 = m_VHOKinGetAccWICLstSqFit_si16();
    }
    else
    { /* do nothing */
    }

#endif // #if( (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) ||
       // (LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON) )

// limiting of standstill counter
#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) || \
     (LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON))
    // if CAN velocity available too late this might cause problems in the rolling
    // direction algorithm STOP state might be kept too long => driven distance
    // might be lost
    if ((g_VHOVehicleState_st.VehicleVelocity_ui16 == ((UInt16)(0))) &&
        (f_DrivenDistanceMMF8_si32 == ((SInt32)(0))))
#else
    if ((g_VHOVehicleState_st.VehicleVelCanMPerS_ui16 == ((UInt16)(0))) &&
        (f_DrivenDistanceMMF8_si32 == ((SInt32)(0))))
#endif
    {
        if (g_VHOVehicleState_st.StandstillCycleCnt_ui16 < ((UInt16)(0xFFFF)))
        {
            g_VHOVehicleState_st.StandstillCycleCnt_ui16 += ((UInt16)(1));
        }
    }
    else
    {
        g_VHOVehicleState_st.StandstillCycleCnt_ui16 = ((UInt16)(0));
    }

    return l_VirtCanMsgRcv_ui8;
}

#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) || \
     (LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON))

VISIBILITY UInt8 m_VHOKinNormVirtLSCan_ui8(SInt32 f_DrivenDistanceMMF8_si32,
                                           UInt16 *l_RawVelocityMperSF8_pui16)
{
    UInt8 l_RetVal_ui8 = ((UInt8)(0));

    // emulate virtual low speed can by reducing the sample time of the input
    // signal
    m_TimeSinceLastVirtCanMsgMS_ui16 += g_VHOOdoBuf_st.WICCanTimeDeltaMS_ui16;
    m_DrivenDistanceSinceLastVirtCanMsgMMF8_ui32 +=
        (UInt32)g_mtl_Abs_mac(f_DrivenDistanceMMF8_si32);
    m_AccSum_si32 += (SInt32)g_VHOCanSig_st.Acc_si16;
    m_AccCnt_ui8++;

    if (m_TimeSinceLastVirtCanMsgMS_ui16 >
        (UInt16)(g_parGetParaVHO_Odo_VIRTUAL_LSCAN_TCYC_MS_ui16 -
                 g_parGetParaVHO_Odo_VIRTUAL_LSCAN_TTOL_MS_ui16))
    {
        // e.g. m_TimeSinceLastVirtCanMsgMS_ui16 = 100,
        // m_DrivenDistanceSinceLastVirtCanMsgMMF8_ui32 = 1024000 raw velocity = s/t
        // = (1024000/100) / 2^8 m/s= 40m/s
        (*l_RawVelocityMperSF8_pui16) =
            (UInt16)(m_DrivenDistanceSinceLastVirtCanMsgMMF8_ui32 /
                     (UInt32)m_TimeSinceLastVirtCanMsgMS_ui16);

        // m_CycleTimeLastVirtCanMsgMS_ui16 = m_TimeSinceLastVirtCanMsgMS_ui16;
        m_TimeSinceLastVirtCanMsgMS_ui16             = ((UInt16)(0));
        m_DrivenDistanceSinceLastVirtCanMsgMMF8_ui32 = ((UInt32)(0));

        // write acceleration signal to ring memory
        m_VHOKinRMAddAx_vd(
            (SInt16)g_mtl_s32_Div_s32_si32(m_AccSum_si32, (SInt32)m_AccCnt_ui8));

        m_AccSum_si32 = ((SInt32)(0));
        m_AccCnt_ui8  = ((UInt8)(0));
        l_RetVal_ui8  = ((UInt8)(1));
    }
    return l_RetVal_ui8;
}

VISIBILITY Boolean m_VHOKinRMBufCmplAcc_bl(void)
{
    Boolean l_RetVal_bl;

    if (g_VHORollRecog_st.RMAxNumStoredElem_ui8 < gd_RM_SIZE_Ax_ui8)
    {
        l_RetVal_bl = FALSE;
    }
    else
    {
        l_RetVal_bl = TRUE;
    }
    return l_RetVal_bl;
}

/*
VISIBILITY Boolean m_VHOKinRMBufCmplAxGrad_bl(void)
{
  Boolean l_RetVal_bl;

  if (m_RMAxGradNumStoredElem_si16 < md_RM_SIZE_Ax_GRAD_ui8)
  {
    l_RetVal_bl = FALSE;
  }
  else
  {
    l_RetVal_bl = TRUE;
  }
  return l_RetVal_bl;
}
*/

VISIBILITY Boolean m_VHOKinRMBufCmplVel_bl(void)
{
    Boolean l_RetVal_bl;

    if (m_RMVeloNumStoredElem_ui8 < md_RM_SIZE_VELO_SMOOTH)
    {
        l_RetVal_bl = FALSE;
    }
    else
    {
        l_RetVal_bl = TRUE;
    }
    return l_RetVal_bl;
}

VISIBILITY Boolean m_VHOKinRMBufCmplAccWIC_bl(void)
{
    Boolean l_RetVal_bl;

    if (m_RMAccWICNumStoredElem_si16 < md_RM_SIZE_ACC_WIC)
    {
        l_RetVal_bl = FALSE;
    }
    else
    {
        l_RetVal_bl = TRUE;
    }
    return l_RetVal_bl;
}

VISIBILITY void m_VHOKinRMAddAx_vd(SInt16 f_Ax_si16)
{
    if (g_VHORollRecog_st.RMAxNumStoredElem_ui8 < gd_RM_SIZE_Ax_ui8)
    {
        g_VHORollRecog_st.RMAxNumStoredElem_ui8++;
    }

    g_VHORollRecog_st.RMAx_psi16[g_VHORollRecog_st.RMAxNextElem_ui8] = f_Ax_si16;

    g_VHORollRecog_st.RMAxNextElem_ui8 =
        (((UInt8)(g_VHORollRecog_st.RMAxNextElem_ui8 + (UInt8)1)) % gd_RM_SIZE_Ax_ui8);
}

VISIBILITY void m_VHOKinRMAddAxGrad_vd(SInt16 f_AxGrad_si16)
{
    if (m_RMAxGradNumStoredElem_si16 < (SInt16)md_RM_SIZE_Ax_GRAD_ui8)
    {
        m_RMAxGradNumStoredElem_si16++;
    }

    m_RMAxGrad_psi16[m_RMAxGradNextElem_si16] = f_AxGrad_si16;

    m_RMAxGradNextElem_si16 =
        (SInt16)(((UInt8)(m_RMAxGradNextElem_si16 + (UInt8)1)) % md_RM_SIZE_Ax_GRAD_ui8);
}

VISIBILITY void m_VHOKinRMAddAccWic_vd(SInt16 f_AccWIC_si16)
{
    if (m_RMAccWICNumStoredElem_si16 < md_RM_SIZE_ACC_WIC)
    {
        m_RMAccWICNumStoredElem_si16++;
    }

    m_RMAccWIC_psi16[m_RMAccWICNextElem_si16] = f_AccWIC_si16;

    m_RMAccWICNextElem_si16 = (SInt16)(((UInt8)(m_RMAccWICNextElem_si16 + (UInt8)1)) %
                                       (UInt8)md_RM_SIZE_ACC_WIC);
}

SInt16 g_VHOKinRMGetAcc_si16(UInt8 f_I_ui8)
{
    SInt8 l_I_si8;
    f_I_ui8 = g_mtl_Min_mac(f_I_ui8, gd_RM_SIZE_Ax_ui8);

    l_I_si8 = (SInt8)g_VHORollRecog_st.RMAxNextElem_ui8 - (SInt8)f_I_ui8;
    if (l_I_si8 < 0)
    {
        l_I_si8 += (SInt8)gd_RM_SIZE_Ax_ui8;
    }
    return g_VHORollRecog_st.RMAx_psi16[l_I_si8];
}

SInt16 g_VHOKinRMGetAxGrad_si16(UInt8 f_I_ui8)
{
    SInt8 l_I_si8;
    f_I_ui8 = g_mtl_Min_mac(f_I_ui8, md_RM_SIZE_Ax_GRAD_ui8);

    l_I_si8 = (SInt8)m_RMAxGradNextElem_si16 - (SInt8)f_I_ui8;
    if (l_I_si8 < 0)
    {
        l_I_si8 += (SInt8)md_RM_SIZE_Ax_GRAD_ui8;
    }
    return m_RMAxGrad_psi16[l_I_si8];
}

VISIBILITY UInt8 m_VHOKinGetFiltCoeff_ui8(void)
{
    static gType_VHOLstSqFit_st VHOLstSqFit_st;
    // static UInt8 l_HighGradient_ui8 = 0;
    SInt16 l_RetVal_si16;
    UInt8 l_RetVal_ui8;

    if (m_VHOKinRMBufCmplAcc_bl() != FALSE)
    {
        l_RetVal_si16 = m_VHOKinLstSqFit_si16(&VHOLstSqFit_st, gd_RM_SIZE_Ax_ui8,
                                              &g_VHOKinRMGetAcc_si16); //[0.01]
    }
    else
    {
        /* reinit structure */
        VHOLstSqFit_st.Init_bl            = FALSE;
        VHOLstSqFit_st.SumX_si16          = 0;
        VHOLstSqFit_st.SumXX_si16         = 0;
        VHOLstSqFit_st.SumXY_si32         = 0;
        VHOLstSqFit_st.SumY_si32          = 0;
        VHOLstSqFit_st.OldestElement_si16 = 0;
        l_RetVal_si16                     = 0;
    }

    m_VHOKinRMAddAxGrad_vd(l_RetVal_si16);
    g_VHORollRecog_st.GradAx_si16 = l_RetVal_si16;

    // m_AccInputLsSq_si16 = g_VHOKinRMGetAcc_si16(1);

    if ((UInt16)g_mtl_Abs_mac(l_RetVal_si16) >= (UInt16)md_MinGradFastResponse_ui8)
    {
        if (l_HighGradient_ui8 < (UInt8)100)
        {
            l_HighGradient_ui8++;
        }
    }
    else
    {
        l_HighGradient_ui8 = 0;
    }

    if (l_HighGradient_ui8 >= md_MaxNumHighGrad_ui8)
    {
        l_RetVal_ui8 = md_FiltConstHighGrad_ui8;
    }
    else
    {
        l_RetVal_ui8 = md_FiltConstLowGrad_ui8;
    }
    return l_RetVal_ui8;
}

VISIBILITY SInt16 m_VHOKinRMGetVel_si16(UInt8 f_I_ui8)
{
    SInt8 l_I_si8;

    f_I_ui8 = g_mtl_Min_mac(f_I_ui8, (UInt8)md_RM_SIZE_VELO_SMOOTH);
    l_I_si8 = (SInt8)m_RMVeloSmoothNextElem_ui8 - (SInt8)f_I_ui8;
    if (l_I_si8 < 0)
    {
        l_I_si8 += (SInt8)md_RM_SIZE_VELO_SMOOTH;
    }

    return (SInt16)m_RMVeloSmooth_pui16[l_I_si8];
}

Boolean g_VHOKinVehicleStandstill_bl(void)
{
    Boolean l_RetVal_bl;

    if (g_VHOVehicleState_st.StandstillCycleCnt_ui16 == 0)
    {
        l_RetVal_bl = FALSE;
    }
    else
    {
        l_RetVal_bl = TRUE;
    }
    return l_RetVal_bl;
}

VISIBILITY SInt16 m_VHOKinRMGetAccWIC_si16(UInt8 f_I_ui8)
{
    SInt8 l_I_si8;

    f_I_ui8 = g_mtl_Min_mac(f_I_ui8, (UInt8)md_RM_SIZE_ACC_WIC);
    l_I_si8 = (SInt8)m_RMAccWICNextElem_si16 - (SInt8)f_I_ui8;
    if (l_I_si8 < 0)
    {
        l_I_si8 += (SInt8)md_RM_SIZE_ACC_WIC;
    }

    return m_RMAccWIC_psi16[l_I_si8];
}

VISIBILITY SInt16 m_VHOKinGetAccLstSqFit_si16(void)
{
    static gType_VHOLstSqFit_st VHOLstSqFit_st;
    SInt16 l_RetVal_si16;

    if (m_VHOKinRMBufCmplVel_bl() != FALSE)
    {
        l_RetVal_si16 = m_VHOKinLstSqFit_si16(&VHOLstSqFit_st, md_RM_SIZE_VELO_SMOOTH,
                                              &m_VHOKinRMGetVel_si16);
    }
    else
    {
        /* reinit structure */
        VHOLstSqFit_st.Init_bl            = FALSE;
        VHOLstSqFit_st.SumX_si16          = 0;
        VHOLstSqFit_st.SumXX_si16         = 0;
        VHOLstSqFit_st.SumXY_si32         = 0;
        VHOLstSqFit_st.SumY_si32          = 0;
        VHOLstSqFit_st.OldestElement_si16 = 0;
        l_RetVal_si16                     = 0;
    }
    return l_RetVal_si16;
}

VISIBILITY SInt16 m_VHOKinGetAccWICLstSqFit_si16(void)
{
    static gType_VHOLstSqFit_st VHOLstSqFit_st;
    SInt16 l_RetVal_si16;

    if (m_VHOKinRMBufCmplAccWIC_bl() != FALSE)
    {
        l_RetVal_si16 = m_VHOKinLstSqFit_si16(&VHOLstSqFit_st, md_RM_SIZE_ACC_WIC,
                                              &m_VHOKinRMGetAccWIC_si16);
    }
    else
    {
        /* reinit structure */
        VHOLstSqFit_st.Init_bl            = FALSE;
        VHOLstSqFit_st.SumX_si16          = 0;
        VHOLstSqFit_st.SumXX_si16         = 0;
        VHOLstSqFit_st.SumXY_si32         = 0;
        VHOLstSqFit_st.SumY_si32          = 0;
        VHOLstSqFit_st.OldestElement_si16 = 0;
        l_RetVal_si16                     = 0;
    }
    return l_RetVal_si16;
}

void m_VHOKinRMAddVel_vd(UInt16 l_VeloMPerS_ui16)
{
    if (m_RMVeloNumStoredElem_ui8 < md_RM_SIZE_VELO_SMOOTH)
    {
        m_RMVeloNumStoredElem_ui8++;
    }

    m_RMVeloSmooth_pui16[m_RMVeloSmoothNextElem_ui8] = l_VeloMPerS_ui16;
    m_RMVeloSmoothNextElem_ui8 =
        (m_RMVeloSmoothNextElem_ui8 + 1) % md_RM_SIZE_VELO_SMOOTH;
}

SInt16 g_VHOKinRMGetAxSyncAvg_si16(UInt8 l_WindowSize_ui8)
{
    UInt8 l_I_ui8;
    SInt32 l_Sum_si32 = 0;
    SInt16 l_RetVal_si16;
    const UInt8 lc_StartPoint_ui8 = 4;
    const UInt8 lc_NewestElem_ui8 = 1;

    if (l_WindowSize_ui8 > 0)
    {
        for (l_I_ui8 = lc_StartPoint_ui8;
             l_I_ui8 < (lc_StartPoint_ui8 + l_WindowSize_ui8); l_I_ui8++)
        {
            l_Sum_si32 += g_VHOKinRMGetAcc_si16(l_I_ui8);
        }
        l_RetVal_si16 = (SInt16)(l_Sum_si32 / (SInt32)l_WindowSize_ui8);
    }
    else
    {
        l_RetVal_si16 = g_VHOKinRMGetAcc_si16(lc_NewestElem_ui8);
    }
    return l_RetVal_si16;
}

VISIBILITY UInt16 m_VHOKinSmoothVel_ui16(UInt16 f_Velo_ui16)
{
    UInt32 l_X_ui32 = (UInt32)f_Velo_ui16;
    UInt8 l_I_ui8;

    for (l_I_ui8 = 0; l_I_ui8 < md_RM_SIZE_VELO_EMA; l_I_ui8++)
    {
        l_X_ui32 += (UInt32)m_RMVeloEMA_pui16[l_I_ui8];
    }
    m_RMVeloEMA_pui16[m_RMVeloEMANextElem_ui8] = f_Velo_ui16;
    m_RMVeloEMANextElem_ui8 = (m_RMVeloEMANextElem_ui8 + 1) % md_RM_SIZE_VELO_EMA;
    l_X_ui32 /= ((UInt32)md_WINDOW_SIZE_VELO_EMA);
    return (UInt16)l_X_ui32;
}

VISIBILITY SInt16 m_VHOKinExpMovAvgSigned_si16(SInt16 f_New_si16, SInt16 f_Old_si16,
                                               UInt8 f_p_ui8)
{
    SInt32 l_RetVal_si32;

    l_RetVal_si32 = ((SInt32)f_p_ui8 * (SInt32)f_Old_si16);
    l_RetVal_si32 += (((SInt32)100 - (SInt32)f_p_ui8) * (SInt32)f_New_si16);
    l_RetVal_si32 = (l_RetVal_si32 / ((SInt32)100));
    return (SInt16)l_RetVal_si32;
}

VISIBILITY UInt16 m_VHOKinExpMovAvg_ui16(UInt16 f_New_ui16, UInt16 f_Old_ui16,
                                         UInt8 f_p_ui8)
{
    UInt16 l_RetVal_ui16;

    l_RetVal_ui16 = (UInt16)((((UInt32)f_p_ui8 * (UInt32)f_Old_ui16) +
                              ((UInt32)((UInt8)100 - f_p_ui8) * (UInt32)f_New_ui16)) /
                             (UInt32)100);

    return l_RetVal_ui16;
}

VISIBILITY SInt16 m_VHOKinLstSqFit_si16(gType_VHOLstSqFit_st *f_LstSqFit_st,
                                        UInt8 f_RM_Size_ui8,
                                        SInt16 (*f_GetRMElement)(UInt8 f_I_ui8))
{
    UInt8 l_I_ui8;
    SInt32 l_temp1_si32;
    const UInt8 lc_NewestElem_ui8 = 1;
    // SInt16 l_temp2_si16;

    /************************************************************************/
    /* system of equations:                                                 */
    /*                                                                      */
    /*       n * a0   +    sum(xi) * a1   =  sum(yi)                        */
    /* sum(xi) * a0   +  sum(xi^2) * a1   =  sum(xi*yi)                     */
    /*                                                                      */
    /* best fit straight line => y = a0 + a1 * x                            */
    /*                                                                      */
    /* Cramer'sche rule:                                                    */
    /*             n * sum(xiyi) - sum(xi) * sum(yi)                        */
    /* slope a1 = -----------------------------------                       */
    /*             n * sum(xi^2) - sum(xi) * sum(xi)                        */
    /************************************************************************/

    if (f_LstSqFit_st->Init_bl == FALSE)
    {
        /* *****************************************************************************
         */
        /* velo: max(f_func (l_I_ui8)) = 2813 [0.01] --> 101.27 km/h  -> global max
         */
        /* acc : max(f_func (l_I_ui8)) = 1600 [0.01]      --> 16 m/s^2 */
        /*                max(RM_size) = 15; */
        /*                min(RM_size) =  2; */
        /*                  max(SumXY) = 1 * 2813 + 2 * 2813 + ... = 337560 */
        /*                  min(SumXY) = 1 * 0 + 2 * 0 = 0 */
        /*                   max(SumX) = 1 + 2 + 3 .... RM_Size = 120 */
        /*                   min(SumX) = 1 + 2 = 3 */
        /*                   max(SumY) = 2813 * 15 = 42195 */
        /*                  max(SumXX) = 1*1 + 2*2 + 3*3 + ... = 1240 */
        /* *****************************************************************************
         */
        for (l_I_ui8 = 1; l_I_ui8 <= (f_RM_Size_ui8 - (UInt8)1); l_I_ui8++)
        {
            f_LstSqFit_st->SumX_si16 += (SInt16)l_I_ui8;
            f_LstSqFit_st->SumY_si32 +=
                ((SInt32)f_GetRMElement((f_RM_Size_ui8 - l_I_ui8) + (UInt8)1)); //[0.01]
            f_LstSqFit_st->SumXX_si16 += (SInt16)l_I_ui8 * (SInt16)l_I_ui8;
        }
        f_LstSqFit_st->SumX_si16 += (SInt16)f_RM_Size_ui8;
        f_LstSqFit_st->SumXX_si16 += (SInt16)f_RM_Size_ui8 * (SInt16)f_RM_Size_ui8;

        /* calculate denominator of cramer'sche rule */
        /* max(RM_Size*SumXX) < 2^15             */
        /* min(SumX*SumX) = 4 < 2^4              */
        /* max(RM_Size*SumXX - SumX*SumX) < 2^15 */
        f_LstSqFit_st->CramerDenominator_si16 =
            ((SInt16)f_RM_Size_ui8 * f_LstSqFit_st->SumXX_si16) -
            (f_LstSqFit_st->SumX_si16 * f_LstSqFit_st->SumX_si16);

        f_LstSqFit_st->Init_bl = TRUE;
    }

    /* delete oldest element */
    f_LstSqFit_st->SumY_si32 -= (SInt32)f_LstSqFit_st->OldestElement_si16;
    /* add newest element */
    f_LstSqFit_st->SumY_si32 += (SInt32)f_GetRMElement(lc_NewestElem_ui8);

    /* reset */
    f_LstSqFit_st->SumXY_si32 = (SInt32)0;
    /* calculate 1*y1 + 2*y2 +... */
    for (l_I_ui8 = 1; l_I_ui8 <= f_RM_Size_ui8; l_I_ui8++)
    {
        f_LstSqFit_st->SumXY_si32 +=
            (((SInt32)((UInt8)(f_RM_Size_ui8 - l_I_ui8) + (UInt8)1)) *
             ((SInt32)f_GetRMElement(l_I_ui8)));
    }

    /* store current oldest element */
    f_LstSqFit_st->OldestElement_si16 = f_GetRMElement(f_RM_Size_ui8);

    /* max(RM_Size*SumXY) < 2^23             */
    /* min(SumX*SumY) = 0                    */
    /* max(RM_Size*SumXY - SumX*SumY) < 2^23 */
    l_temp1_si32 = ((SInt32)f_RM_Size_ui8 * f_LstSqFit_st->SumXY_si32) -
                   ((SInt32)f_LstSqFit_st->SumX_si16 * f_LstSqFit_st->SumY_si32);

    return (SInt16)g_mtl_s32_Div_s32_si32(
        l_temp1_si32 * (SInt32)1000,
        (SInt32)f_LstSqFit_st->CramerDenominator_si16 *
            (SInt32)g_parGetParaVHO_Odo_VIRTUAL_LSCAN_TCYC_MS_ui16); // 0.01
}

#endif // #if( (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) ||
       // (LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON) )

/**
 * @brief
 * @details calculate driven distance from wic increments of the current
 *          cycle
 *
 * @param
 * @param
 * @return driven distance in the center of the rear axle in 1/256mm (F8)
 *
 * @note
 * @see
 * @warning
 */
SInt32 g_VHOPosCalcGetDrivenDistance_si32(void)
{
    SInt32 l_DrivenDistanceF8_si32;
    UInt32 l_DrivenDistanceRearF8_ui32  = 0;
    UInt32 l_DrivenDistanceFrontF8_ui32 = 0;

#if (GS_4WS_VEHICLE == SW_ON)
    UInt32 l_temp_ui32;
    UInt32 l_CosRearAngle_ui32;
#endif

    /**************************************************************************/
    /*                                                                        */
    /* Im module vho_odo_core.c werden die WIC Quellen abhaengig von der      */
    /* Antriebsart festgelegt. Die Auswahl ist etwas eingeschraenckt.         */
    /*                                                                        */
    /* Vorderachsantrieb (case VHODrivenAxleFront_enm:)                       */
    /*           WicSource_en = VHOWicSourceTwoWICRear_enm                    */
    /*                                                                        */
    /* Hinterachsantrieb (case VHODrivenAxleRear_enm:)                        */
    /* Allradantrieb (case VHODrivenAxle4Wheel_enm:)                          */
    /*           WicSource_en = VHOWicSourceFourWIC_enm                       */
    /*                                                                        */
    /*           WicSource_en = VHOWicSourceTwoWICFront_enm wir nicht verg. ? */
    /*                                                                        */
    /**************************************************************************/

    /* =======================================================================*/
    /* Harald Michi, 10.12.2012 Der Mittelwert der WICs ist aktuell ungenutzt */
    /* =======================================================================*/
    if (g_VHOOdoState_st.WicSource_en == VHOWicSourceAvgFront_enm)
    {
        // take virtual wic of an imaginary wheel located in the middle
        // of the front axle as input
        l_DrivenDistanceFrontF8_ui32 =
            (UInt32)g_VHOOdoBuf_st.WICDelta_pui16[AvgLeftRight_enm] *
            (UInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[AvgLeftRight_enm];
    }
    /* =======================================================================*/
    /* Harald Michi, 10.12.2012 Der Mittelwert der WICs ist aktuell ungenutzt */
    /* =======================================================================*/
    else if (g_VHOOdoState_st.WicSource_en == VHOWicSourceAvgRear_enm)
    {
        // take virtual wic of an imaginary wheel located in the middle
        // of the rear axle as input
        l_DrivenDistanceRearF8_ui32 =
            (UInt32)g_VHOOdoBuf_st.WICDelta_pui16[AvgLeftRight_enm] *
            (UInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[AvgLeftRight_enm];
    }
    else
    {
        if ((g_VHOOdoState_st.WicSource_en == VHOWicSourceTwoWICRear_enm) ||
            (g_VHOOdoState_st.WicSource_en == VHOWicSourceFourWIC_enm))
        {
#if (LS_VHO_DIST_FROM_REAR_WIC == LS_VHO_DIST_FROM_REAR_WIC_AVG)
            // take wic values of the two rear wics as input;
            // distance of rear axle center is the average of
            // the single wheels distances
            l_DrivenDistanceRearF8_ui32 =
                ((UInt32)g_VHOOdoBuf_st.WICDelta_pui16[LeftRear_enm] *
                 (UInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[LeftRear_enm]) +
                ((UInt32)g_VHOOdoBuf_st.WICDelta_pui16[RightRear_enm] *
                 (UInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[RightRear_enm]);

            l_DrivenDistanceRearF8_ui32 =
                g_mtl_ReducePrecision_ui32(l_DrivenDistanceRearF8_ui32, (UInt8)1);
#endif // #if (LS_VHO_DIST_FROM_REAR_WIC == LS_VHO_DIST_FROM_REAR_WIC_AVG)

#if ((GS_4WS_VEHICLE == SW_ON) && \
     (LS_VHO_DIST_FROM_REAR_WIC == LS_VHO_DIST_FROM_REAR_WIC_ACKERMANN_STEER))
            // ====================================================================
            // if vehicle is equiped with 4WS use Ackermann to calculate the driven
            // distance from WICs of the rear axle in higher precision, with wheel
            // base virtual, steeering angle front and steering angle rear
            // ====================================================================
            // take wic values of two rear wics as input;
            // distance of rear axle center is calculated by
            // considering single wheel distances, current front steering angle
            // and current rear steering angle
            l_DrivenDistanceRearF8_ui32 = m_VHOGetDrivenDistRearAxleCenter_ui32(
                g_VHOOdoBuf_st.WICDelta_pui16[LeftRear_enm],
                g_VHOOdoBuf_st.WICDelta_pui16[RightRear_enm],
                g_VHOVehicleState_st.SAngRadF12_si16,
                g_VHOVehicleState_st.SAngRearRadF12_si16);
#endif // #if ( (GS_4WS_VEHICLE == SW_ON) && (LS_VHO_DIST_FROM_REAR_WIC ==
       //  LS_VHO_DIST_FROM_REAR_WIC_ACKERMANN_STEER) )
        }

        /* =====================================================================*/
        /* Harald Michi, 10.12.2012 VHOWicSourceTwoWICFront_enm ist ungenutzt   */
        /* =====================================================================*/
        if ((g_VHOOdoState_st.WicSource_en == VHOWicSourceTwoWICFront_enm) ||
            (g_VHOOdoState_st.WicSource_en == VHOWicSourceFourWIC_enm))
        {
            // ======================================================================
            // to calculate the driven distance from WICs of front axle use Ackermann
            // for higher precision with wheel base virtual and steering angle front
            // ======================================================================
            // take wic values of two front wics as input;
            // distance of front axle center is calculated by
            // considering single wheel distances and current curvature
            l_DrivenDistanceFrontF8_ui32 = m_VHOGetDrivenDistFrontAxleCenter_ui32(
                g_VHOOdoBuf_st.WICDelta_pui16[LeftFront_enm],
                g_VHOOdoBuf_st.WICDelta_pui16[RightFront_enm],
                g_VHOVehicleState_st.SAngRadF12_si16);
        }
    }

    if ((g_VHOOdoState_st.WicSource_en == VHOWicSourceTwoWICFront_enm) ||
        (g_VHOOdoState_st.WicSource_en == VHOWicSourceFourWIC_enm) ||
        (g_VHOOdoState_st.WicSource_en == VHOWicSourceAvgFront_enm))
    {
        // map driven distance from center of front axle to instantaneous center
        // for 2WS vehicles the instantaneous center is the center of the rear axle
        l_DrivenDistanceFrontF8_ui32 =
            g_mtl_ReducePrecision_ui32(l_DrivenDistanceFrontF8_ui32, 2);

        l_DrivenDistanceFrontF8_ui32 =
            l_DrivenDistanceFrontF8_ui32 *
            (UInt32)g_mtl_Cos_si16(g_VHOVehicleState_st.SAngRadF12_si16);

        l_DrivenDistanceFrontF8_ui32 =
            g_mtl_ReducePrecision_ui32(l_DrivenDistanceFrontF8_ui32, 12); // F20 -> F8

#if (GS_4WS_VEHICLE == SW_ON)
        // map driven distance from instantaneous center to the center of the rear
        // axle
        l_CosRearAngle_ui32 =
            (UInt32)g_mtl_Cos_si16(g_VHOVehicleState_st.SAngRearRadF12_si16); // F14

        l_temp_ui32 = l_DrivenDistanceFrontF8_ui32 * (UInt32)(16384); //* 2^14
        l_temp_ui32 = l_temp_ui32 / l_CosRearAngle_ui32; // F8 + F14 - F14 = F8
        l_DrivenDistanceFrontF8_ui32 = l_temp_ui32;
#endif
    }

    if ((g_VHOOdoState_st.WicSource_en == VHOWicSourceTwoWICFront_enm) ||
        (g_VHOOdoState_st.WicSource_en == VHOWicSourceAvgFront_enm))
    {
        // take driven distance derived from front axle wics as
        // the final distance value
        l_DrivenDistanceF8_si32 = (SInt32)l_DrivenDistanceFrontF8_ui32;
    }
    else if ((g_VHOOdoState_st.WicSource_en == VHOWicSourceTwoWICRear_enm) ||
             (g_VHOOdoState_st.WicSource_en == VHOWicSourceAvgRear_enm))
    {
        // take driven distance derived from rear axle wics as
        // the final distance value
        l_DrivenDistanceF8_si32 = (SInt32)l_DrivenDistanceRearF8_ui32;
    }
    else // (g_VHOOdoState_st.WicSource_en == VHOWicSourceFourWIC_enm)
    {
        // take driven distances derived from rear axle wics as
        // well as front axle wics as the final distance value by
        // calculating the average
        l_DrivenDistanceF8_si32 =
            g_mtl_s32_Div_s32_si32(((SInt32)g_VHOOdoState_st.WeightDistanceFront_bf3 *
                                    (SInt32)l_DrivenDistanceFrontF8_ui32) +
                                       ((SInt32)g_VHOOdoState_st.WeightDistanceRear_bf3 *
                                        (SInt32)l_DrivenDistanceRearF8_ui32),
                                   ((SInt32)g_VHOOdoState_st.WeightDistanceFront_bf3 +
                                    (SInt32)g_VHOOdoState_st.WeightDistanceRear_bf3));
    }

    if ((UInt32)l_DrivenDistanceF8_si32 > (UInt32)8388607)
    {
        // absolute value of driven distance > 32.768m
        g_vhoLogHdl_vd(VHOLog_OdoCore_enm, g_vhoLogStateOn_enm);
        l_DrivenDistanceF8_si32 = ((SInt32)8388607);
    }

    return (l_DrivenDistanceF8_si32);
}

/*
   suppress QAC warning message 3206
   The parameter 'f_SA_rad_F12_si16' is not used in this function
   (To be able to keep the interface identical for all calculation modes)
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3206
#endif

/**
 * @brief given the driven distance of the two front wheels and the steering
 *        angle, the function calculates the driven distance of the virtual
 *        wheel (located at the center of the front axle)
 * @details
 *
 * @param
 * @param
 * @return driven distance of virtual front wheel [1/256mm]
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY UInt32 m_VHOGetDrivenDistFrontAxleCenter_ui32(UInt16 f_WICDeltaLeft_ui16,
                                                         UInt16 f_WICDeltaRight_ui16,
                                                         SInt16 f_SA_rad_F12_si16)
{
#if (LS_VHO_DIST_FROM_FRONT_WIC == LS_VHO_DIST_FROM_FRONT_WIC_AVG)

    /**********************************************************************
     * calculate driven distance of the virtual front wheel from the single
     * wheel distances of the front axle by taking the average
     **********************************************************************/
    UInt32 l_DrivenDistanceFrontF8_ui32;

    l_DrivenDistanceFrontF8_ui32 =
        ((UInt32)f_WICDeltaLeft_ui16 *
         (UInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[LeftFront_enm]) +
        ((UInt32)f_WICDeltaRight_ui16 *
         (UInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[RightFront_enm]);

    return g_mtl_ReducePrecision_ui32(l_DrivenDistanceFrontF8_ui32, (UInt8)1);

#elif (LS_VHO_DIST_FROM_FRONT_WIC == LS_VHO_DIST_FROM_FRONT_WIC_ACKERMANN_STEER)

    /**********************************************************************
     * calculate driven distance of the virtual front wheel from the single
     * wheel distances of the front axle for an ideal ackermann steering
     **********************************************************************/
    UInt32 l_DrivenDistFLMM_F8_ui32;
    UInt32 l_DrivenDistFRMM_F8_ui32;
    SInt32 l_DrivenDistFMMM_F8_si32;

    SInt32 l_B_si32;
    UInt32 l_C_ui32;
    SInt32 l_L_si32;
    SInt32 l_R_si32;

    // max(g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16) = 8192 = 2^13
    // => max(result) = 2^13 * 2^13 * 2^2 = 2^28 < 2^32
    const UInt32 lc_A_ui32 =
        /*
        (UInt32) 4 *
        (UInt32) g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 *
        (UInt32) g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 ;
        */
        ((UInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 *
         (UInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16)
        << (UInt32)(2);

    // calculate driven distance of left front wheel in 1/256mm
    l_DrivenDistFLMM_F8_ui32 =
        (UInt32)f_WICDeltaLeft_ui16 *
        (UInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[LeftFront_enm];

    // calculate driven distance of right front wheel in 1/256mm
    l_DrivenDistFRMM_F8_ui32 =
        (UInt32)f_WICDeltaRight_ui16 *
        (UInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[RightFront_enm];

    if ((l_DrivenDistFRMM_F8_ui32 > (UInt32)512000) ||
        (l_DrivenDistFLMM_F8_ui32 > (UInt32)512000))
    {
        // driven distance of one front wheel exceeded tolerated maximum of 2 meters
        // value will be replaced by the average of the two front wheel distances
        // error handler will be called
        l_DrivenDistFMMM_F8_si32 = (SInt32)g_mtl_ReducePrecision_ui32(
            l_DrivenDistFRMM_F8_ui32 + l_DrivenDistFLMM_F8_ui32, (UInt8)1);
        g_vhoLogHdl_vd(VHOLog_OdoCore_enm, g_vhoLogStateOn_enm);
    }
    else // driven distance in tolerated range
    {
        l_B_si32 = (SInt32)g_mtl_Sin_si16(f_SA_rad_F12_si16); // F14

        // max(l_B_si32)^2 = 2^14 * 2^14 = 2^28 < 2^32
        // => max(result) = 2^(28-7) = 2^21 < 2^32
        l_C_ui32 = g_mtl_ReducePrecision_ui32((UInt32)(l_B_si32 * l_B_si32),
                                              (UInt8)7); // F28 => F21
        // max(g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16) = 2048 = 2^11
        // max(l_C_ui32) * max(g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16) =
        // 2^21 * 2^11 = 2^32
        // => max(result) = 2^(32-11) = 2^21
        l_C_ui32 = g_mtl_ReducePrecision_ui32(
            l_C_ui32 * g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16,
            (UInt8)11); // F21 => F10
        // max(g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16) = 2048 = 2^11
        // max(l_C_ui32) * max(g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16) =
        // 2^21 * 2^11 = 2^32
        // => max(result) = 2^(32-10) = 2^22
        l_C_ui32 = g_mtl_ReducePrecision_ui32(
            l_C_ui32 * g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16,
            (UInt8)10); // F10 => F0

        l_B_si32 = (SInt32)g_mtl_Sin_si16((SInt16)2 * f_SA_rad_F12_si16); // F14
        // max(l_B_si32) = 2^14, max(g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16)
        // = 8192 = 2^13 max(l_B_si32) *
        // max(g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16) = 2^14 * 2^13 = 2^27
        // < 2^31
        // => max(result) = 2^(27-6) = 2^21
        l_B_si32 = g_mtl_ReducePrecision_si32(
            l_B_si32 * (SInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16,
            (UInt8)6); // F8

        // max(g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16) = 2048 = 2^11
        // max(g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16) *  max(l_B_si32) =
        // 2^11 * 2^21 = 2^32
        // => max(result) = 2^(32-7) = 2^25
        // (final multiplication with 2 => bit shift of 7 instead of 8)
        l_B_si32 = g_mtl_ReducePrecision_si32(
            l_B_si32 * (SInt32)g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16,
            (UInt8)7); // F0

        // max(lc_A_ui32) + max(l_B_ui32) + max(l_C_ui32) = 2^28 +  2^21 +  2^22 <
        // 2^31
        l_L_si32 = (SInt32)lc_A_ui32 + l_B_si32 + (SInt32)l_C_ui32; // F0
        l_L_si32 = (SInt32)g_mtl_Sqrt_u32_ui16((UInt32)l_L_si32);   // F0
        l_L_si32 = g_mtl_s32_Div_s32_si32(                          // F20
            (SInt32)l_DrivenDistFLMM_F8_ui32 * (SInt32)4096, l_L_si32);

        // max(l_R_si32) <= max(l_L_si32) < 2^31
        l_R_si32 = ((SInt32)lc_A_ui32 - l_B_si32) + (SInt32)l_C_ui32; // F0
        l_R_si32 = (SInt32)g_mtl_Sqrt_u32_ui16((UInt32)l_R_si32);     // F0
        l_R_si32 = g_mtl_s32_Div_s32_si32(                            // F20
            (SInt32)l_DrivenDistFRMM_F8_ui32 * (SInt32)4096, l_R_si32);

        l_L_si32 += l_R_si32;

        if (l_L_si32 < (SInt32)524288)
        {
            l_DrivenDistFMMM_F8_si32 =
                (SInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 * (l_L_si32);
            l_DrivenDistFMMM_F8_si32 =
                g_mtl_ReducePrecision_si32(l_DrivenDistFMMM_F8_si32, (UInt8)12);
        }
        else
        {
            // to avoid overflow, driven distance of virtual front wheel will
            // be replaced by the average of the two front wheel distances
            l_DrivenDistFMMM_F8_si32 = (SInt32)g_mtl_ReducePrecision_ui32(
                l_DrivenDistFRMM_F8_ui32 + l_DrivenDistFLMM_F8_ui32, (UInt8)1);
            g_vhoLogHdl_vd(VHOLog_OdoCore_enm, g_vhoLogStateOn_enm);
        }
    }

    return (UInt32)l_DrivenDistFMMM_F8_si32;

#elif (LS_VHO_DIST_FROM_FRONT_WIC == LS_VHO_DIST_FROM_RECT_VEH_FRONT_WIC)

    UInt32 l_RadiusRearMiddleMM_ui32;
    UInt32 l_RadiusFrontLeftMM_ui32;
    UInt32 l_RadiusFrontRightMM_ui32;
    UInt32 l_RadiusFrontMiddleMM_ui32;
    UInt32 l_DrivenDistFrontLeftMMF8_ui32;
    UInt32 l_DrivenDistFrontRightMMF8_ui32;

    /* calculate driven distances front */
    l_DrivenDistFrontLeftMMF8_ui32 =
        (UInt32)f_WICDeltaLeft_ui16 *
        (UInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[LeftFront_enm];

    l_DrivenDistFrontRightMMF8_ui32 =
        (UInt32)f_WICDeltaRight_ui16 *
        (UInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[RightFront_enm];

    /* avoid overflow in radius calculation below */
    if ((SInt16)g_mtl_Abs_mac(f_SA_rad_F12_si16) < (SInt16)64) //~0.9�
    {
        return (UInt32)g_mtl_ReducePrecision_ui32(
            l_DrivenDistFrontLeftMMF8_ui32 + l_DrivenDistFrontRightMMF8_ui32, (UInt8)1);
    }
    elseif(f_SA_rad_F12_si16 > md_VHOMaxSteerAng_si16)
    {
        f_SA_rad_F12_si16 = md_VHOMaxSteerAng_si16;
    }

    /* if global parameter GS_4WS_VEHICLE is set to SW_OFF than     */
    /*    calculate the radius of the center point of the rear axle */
    /*    r = L / tan(SteeringAngle)                                */
    /* if global parameter GS_4WS_VEHICLE is set to SW_ON than      */
    /*    calculate the radius of the instantaneous center          */
    /*    r = Lvirtual / tan(SteeringAngle)                         */
    /* -> max=(8192*8192)/tan(0.9�) = 2^32 */
    /* -> min=(1500*8192)/tan(61�)  = 2^23 */
    l_RadiusRearMiddleMM_ui32 = (UInt32)g_mtl_s32_Div_s32_si32(
        (SInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 * (SInt32)8192,
        (SInt32)g_mtl_Tan_si16(g_mtl_Abs_mac(f_SA_rad_F12_si16))); /* mm */

    /* calculate radius front right and front left            */
    /* R_fr = sqrt( (R_rm - 0.5 * AxleBase)^2 + WheelBase^2 ) */
    /* R_fl = sqrt( (R_rm + 0.5 * AxleBase)^2 + WheelBase^2 ) */
    l_RadiusFrontRightMM_ui32 = (UInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 *
                                (UInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16;
    l_RadiusFrontLeftMM_ui32 = l_RadiusFrontRightMM_ui32;
    l_RadiusFrontMiddleMM_ui32 =
        l_RadiusRearMiddleMM_ui32 -
        g_mtl_ReducePrecision_ui32(
            (UInt32)g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16, (UInt8)1);
    l_RadiusFrontRightMM_ui32 += l_RadiusFrontMiddleMM_ui32 * l_RadiusFrontMiddleMM_ui32;
    l_RadiusFrontMiddleMM_ui32 =
        l_RadiusRearMiddleMM_ui32 +
        g_mtl_ReducePrecision_ui32(
            (UInt32)g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16, (UInt8)1);
    l_RadiusFrontLeftMM_ui32 += l_RadiusFrontMiddleMM_ui32 * l_RadiusFrontMiddleMM_ui32;
    l_RadiusFrontRightMM_ui32 = (UInt32)g_mtl_Sqrt_u32_ui16(l_RadiusFrontRightMM_ui32);
    l_RadiusFrontLeftMM_ui32  = (UInt32)g_mtl_Sqrt_u32_ui16(l_RadiusFrontLeftMM_ui32);

    /* calculate radius of the center point of the front axle */
    /* R_fm = sqrt( R_rm^2 + WheelBase^2 ) */
    l_RadiusFrontMiddleMM_ui32 = (UInt32)g_mtl_Sqrt_u32_ui16(
        (UInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 *
            (UInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 +
        l_RadiusRearMiddleMM_ui32 * l_RadiusRearMiddleMM_ui32);

    /* geometrical calculation of the driven distance at the center point of the
     * front axle */
    /* condition: axle base front is equal to the axle base rear */
    /*                                                                                      */
    /*   driven_dist_front_left     driven_dist_front_right
     * driven_dist_front_middle      */
    /*   ----------------------  =  ----------------------- =
     * -------------------------     */
    /*     radius_font_left            radius_font_right radius_font_middle */
    /*                                                                                      */
    /*                   ( s_fl   s_fr ) */
    /* s_fm = 1/2 * r_fm ( ---- + ---- ) */
    /*                   ( r_fl   r_fr ) */
    l_DrivenDistFrontLeftMMF8_ui32 =
        (UInt32)g_mtl_s32_Div_s32_si32((SInt32)l_DrivenDistFrontLeftMMF8_ui32,
                                       (SInt32)l_RadiusFrontLeftMM_ui32); // MMF8
    l_DrivenDistFrontLeftMMF8_ui32 +=
        (UInt32)g_mtl_s32_Div_s32_si32((SInt32)l_DrivenDistFrontRightMMF8_ui32,
                                       (SInt32)l_RadiusFrontRightMM_ui32); // MMF8
    l_DrivenDistFrontLeftMMF8_ui32 *= l_RadiusFrontMiddleMM_ui32;
    l_DrivenDistFrontLeftMMF8_ui32 =
        g_mtl_ReducePrecision_ui32(l_DrivenDistFrontLeftMMF8_ui32, (UInt8)1);

    return l_DrivenDistFrontLeftMMF8_ui32;

#else
#error ('Setting for LS_VHO_DIST_FROM_FRONT_WIC not valid!')
#endif
}

/*
   reactivate suppressed QAC warning message 3206
   The parameter 'f_SA_rad_F12_si16' is not used in this function
   (To be able to keep the interface identical for all calculation modes)
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3206
#endif

#if ((GS_4WS_VEHICLE == SW_ON) && \
     (LS_VHO_DIST_FROM_REAR_WIC == LS_VHO_DIST_FROM_REAR_WIC_ACKERMANN_STEER))
VISIBILITY UInt32 m_VHOGetDrivenDistRearAxleCenter_ui32(UInt16 f_WICDeltaLeft_ui16,
                                                        UInt16 f_WICDeltaRight_ui16,
                                                        SInt16 f_SAF_rad_F12_si16,
                                                        SInt16 f_SAR_rad_F12_si16)

{
    /**********************************************************************
     * calculate driven distance of the virtual rear wheel from the single
     * wheel distances of the rear axle for an ideal ackermann steering
     **********************************************************************/
    UInt32 l_DrivenDistRLMM_F8_ui32;
    UInt32 l_DrivenDistRRMM_F8_ui32;
    SInt32 l_DrivenDistRMMM_F8_si32;

    UInt32 l_temp1_ui32;
    UInt32 l_temp2_ui32;
    UInt32 l_temp3_ui32;
    SInt32 l_temp1_si32;
    UInt32 l_Cos2SAF_ui32;
    UInt32 l_Cos2SAR_ui32;
    UInt32 l_B_ui32;
    UInt32 l_C_ui32;
    SInt32 l_L_si32;
    SInt32 l_R_si32;

    /* only absolute value of front steering angle is needed */
    const SInt16 l_Abs_SAF_rad_F12_si16 = (SInt16)g_mtl_Abs_mac(f_SAF_rad_F12_si16);
    /* only absolute value of rear steering angle is needed */
    const SInt16 l_Abs_SAR_rad_F12_si16 = (SInt16)g_mtl_Abs_mac(f_SAR_rad_F12_si16);

    // max(g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16) = 8192 = 2^13
    // => max(result) = 2^13 * 2^13 * 2^2 = 2^28 < 2^32      Quant: mm^2
    const UInt32 lc_A_ui32 =
        /*
        (UInt32) 4 *
        (UInt32) g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 *
        (UInt32) g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 ;
        */
        ((UInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 *
         (UInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16)
        << (UInt32)(2);

    // calculate driven distance of left rear wheel in 1/256mm
    l_DrivenDistRLMM_F8_ui32 =
        (UInt32)f_WICDeltaLeft_ui16 *
        (UInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[LeftRear_enm];

    // calculate driven distance of right rear wheel in 1/256mm
    l_DrivenDistRRMM_F8_ui32 =
        (UInt32)f_WICDeltaRight_ui16 *
        (UInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[RightRear_enm];

    if ((l_DrivenDistRLMM_F8_ui32 > (UInt32)512000) ||
        (l_DrivenDistRRMM_F8_ui32 > (UInt32)512000))
    {
        // driven distance of one rear wheel exceeded tolerated maximum of 2 meters
        // value will be replaced by the average of the two rear wheel distances
        // error handler will be called
        l_DrivenDistRMMM_F8_si32 = (SInt32)g_mtl_ReducePrecision_ui32(
            l_DrivenDistRLMM_F8_ui32 + l_DrivenDistRRMM_F8_ui32, (UInt8)1);
        g_vhoLogHdl_vd(VHOLog_OdoCore_enm, g_vhoLogStateOn_enm);
    }
    else // driven distance in tolerated range
    {
        /* temp1_si32 = 2*Abs(SAF) */
        l_temp1_si32   = (SInt32)(2) * (SInt32)(l_Abs_SAF_rad_F12_si16); // F12
        l_Cos2SAF_ui32 = (UInt32)g_mtl_Cos_si16((SInt16)(l_temp1_si32)); // F14

        /* temp1_si32 = 2*Abs(SAR) */
        l_temp1_si32   = (SInt32)(2) * (SInt32)(l_Abs_SAR_rad_F12_si16); // F12
        l_Cos2SAR_ui32 = (UInt32)g_mtl_Cos_si16((SInt16)(l_temp1_si32)); // F14

        /* temp1_ui32 = 1 - cos(2*saf) */
        l_temp1_ui32 = (UInt32)(16384) - l_Cos2SAF_ui32; // F14

        /* temp2_ui32 = 1 + cos(2*saf) */
        l_temp2_ui32 = (UInt32)(16384) + l_Cos2SAF_ui32; // F14

        /* temp3_ui32 = 1 + cos(2*sar) */
        l_temp3_ui32 = (UInt32)(16384) + l_Cos2SAR_ui32; // F14

        /* (1-cos2saf)*(1+cos2sar)/1+cos2saf) */
        /* (2^14 * 2^14) / 2^14 = 2^14 < 2^32 */
        l_B_ui32 = (UInt32)g_mtl_s32_Div_s32_si32((SInt32)(l_temp1_ui32) *
                                                      (SInt32)(l_temp3_ui32),
                                                  (SInt32)(l_temp2_ui32)); // F14
        /* l_B / 2 */
        /* Die Quantisierung bleibt unver�ndert: F14 */
        l_B_ui32 = g_mtl_ReducePrecision_ui32(l_B_ui32, (UInt32)(1)); // F14

        // max(g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16) = 2048 = 2^11
        // max(l_B_ui32) * max(g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16) =
        // 2^14 * 2^11 = 2^25 < 2^32
        // => max(result) = 2^(25-4) = 2^21
        l_B_ui32 = g_mtl_ReducePrecision_ui32(
            (l_B_ui32 * g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16),
            (UInt32)(4)); // F21 => F10
        // max(g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16) = 2048 = 2^11
        // max(l_C_ui32) * max(g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16) =
        // 2^21 * 2^11 = 2^32
        // => max(result) = 2^(32-10) = 2^22
        // Quant: mm^2
        l_B_ui32 = g_mtl_ReducePrecision_ui32(
            (l_B_ui32 * g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16),
            (UInt32)(10)); // F21 => F10

        /* temp1_ui32 = sin(Abs(saf)) */
        l_temp1_ui32 = (UInt32)g_mtl_Sin_si16(l_Abs_SAF_rad_F12_si16); // F14

        /* temp2_ui32 = cos(Abs(saf)) */
        l_temp2_ui32 = (UInt32)g_mtl_Cos_si16(l_Abs_SAF_rad_F12_si16); // F14

        /* sin(saf)*(1+cos2sar)/cossaf) */
        /* (2^14 * 2^14) / 2^14 = 2^14 < 2^32 */
        l_C_ui32 = (UInt32)g_mtl_s32_Div_s32_si32((SInt32)(l_temp1_ui32) *
                                                      (SInt32)(l_temp3_ui32),
                                                  (SInt32)(l_temp2_ui32)); // F14

        // max(l_B_si32) = 2^14, max(g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16)
        // = 8192 = 2^13 max(l_B_si32) *
        // max(g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16) = 2^14 * 2^13 = 2^27
        // < 2^31
        // => max(result) = 2^(27-6) = 2^21
        l_C_ui32 = g_mtl_ReducePrecision_ui32(
            l_C_ui32 * (UInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16,
            (UInt8)6); // F8

        // max(g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16) = 2048 = 2^11
        // max(g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16) *  max(l_C_ui32) =
        // 2^11 * 2^21 = 2^32
        // => max(result) = 2^(32-7) = 2^25
        // (final multiplication with 2 => bit shift of 7 instead of 8)
        l_C_ui32 = g_mtl_ReducePrecision_ui32(
            l_C_ui32 * (UInt32)g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16,
            (UInt8)7); // F0

        // max(lc_A_ui32) + max(l_B_ui32) + max(l_C_ui32) = 2^28 +  2^21 +  2^22 <
        // 2^31
        l_L_si32 = (SInt32)(lc_A_ui32) + (SInt32)(l_B_ui32) + (SInt32)(l_C_ui32); // F0
        l_L_si32 = (SInt32)g_mtl_Sqrt_u32_ui16((UInt32)(l_L_si32));               // F0
        l_L_si32 = g_mtl_s32_Div_s32_si32((SInt32)(l_DrivenDistRLMM_F8_ui32) *
                                              (SInt32)(4096), // F20
                                          (l_L_si32));

        // max(l_R_si32) <= max(l_L_si32) < 2^31
        l_R_si32 = ((SInt32)(lc_A_ui32) + (SInt32)(l_B_ui32)) - (SInt32)(l_C_ui32); // F0
        l_R_si32 = (SInt32)g_mtl_Sqrt_u32_ui16((UInt32)(l_R_si32));                 // F0
        l_R_si32 = g_mtl_s32_Div_s32_si32((SInt32)(l_DrivenDistRRMM_F8_ui32) *
                                              (SInt32)(4096), // F20
                                          (l_R_si32));

        l_L_si32 += l_R_si32;

        if (l_L_si32 < (SInt32)524288)
        {
            l_DrivenDistRMMM_F8_si32 =
                (SInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 * (l_L_si32);
            l_DrivenDistRMMM_F8_si32 =
                g_mtl_ReducePrecision_si32(l_DrivenDistRMMM_F8_si32, (UInt8)12);
        }
        else
        {
            // to avoid overflow, driven distance of virtual front wheel will
            // be replaced by the average of the two front wheel distances
            l_DrivenDistRMMM_F8_si32 = (SInt32)g_mtl_ReducePrecision_ui32(
                l_DrivenDistRRMM_F8_ui32 + l_DrivenDistRLMM_F8_ui32, (UInt8)1);
            g_vhoLogHdl_vd(VHOLog_OdoCore_enm, g_vhoLogStateOn_enm);
        }
    }

    return (UInt32)l_DrivenDistRMMM_F8_si32;
}
#endif // #if ( (GS_4WS_VEHICLE == SW_ON) && (LS_VHO_DIST_FROM_REAR_WIC ==
       //  LS_VHO_DIST_FROM_REAR_WIC_ACKERMANN_STEER) )
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
