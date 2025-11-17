/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: vho_adi.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#define COMPONENT_VHO
#ifndef MODULE_LOG_NAME
#define MODULE_LOG_NAME "vho_adi"
#endif

/*----------------------------------------------------------------*/
/*- include files -*/
/*----------------------------------------------------------------*/
#include <pa_components/base_type/global_include.h>
#ifdef md_RUN_ON_POSIX
#include <pa_components/debugprintf/debug_log.h>
#include <sys/time.h>
#endif // md_RUN_ON_POSIX

#include <pa_components/mathlib/mathlib_api.h>
#include <veh/vhs/vhs_api.h>

#include "vho_api.h"

// #include <mathlib_api.h>
#include "vho.h"
// #include <vhm_record.h>
#include <string.h>

#ifndef VISIBILITY
#error ('compiler switch VISIBILITY is not defined')
#endif

#ifndef GS_4WS_VEHICLE
#error ('compiler switch GS_4WS_VEHICLE is not defined')
#endif

#ifndef GS_VHS_VHO_PRESENT
#error ('Switch GS_VHS_VHO_PRESENT not defined!');
#endif

#if (GS_VHS_VHO_PRESENT == SW_OFF)
#error ('GS_VHS_VHO_PRESENT has to be turned to SW_ON if VHO component is compiled in the project!');
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#ifndef LS_VHO_WIC_FILTER_ELEPHANT
#error ('Switch LS_VHO_WIC_FILTER_ELEPHANT not defined!');
#endif
#endif

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 2006
#endif
/**
 * @brief
 * @details
 *
 * @param         f_VehPosOdo_pst
 * @return        Boolean
 *
 * @note
 * @see
 * @warning
 */
void g_adiSetVehPosOdo_bl(gType_FVInputVehOdoFrame_st *f_VehPosOdo_pst)
{
    g_VHOVehicleODO_st.DrivenDistance_si32 = f_VehPosOdo_pst->VehPosOdoSMm_si32;
    g_VHOVehicleODO_st.XPosition_si32      = f_VehPosOdo_pst->VehPosOdoXMm_si32;
    g_VHOVehicleODO_st.YPosition_si32      = f_VehPosOdo_pst->VehPosOdoYMm_si32;
    g_VHOVehicleODO_st.YawAngle_ui32       = f_VehPosOdo_pst->VehPosOdoYawAngle_ui32;
}
/**
 * @brief
 * @details
 *
 * @param         f_VehPosOdo_pst
 * @return        Boolean
 *
 * @note
 * @see
 * @warning
 */
Boolean g_adiGetVehPosOdo_bl(gType_VehPosOdo_st *f_VehPosOdo_pst)
{
    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (f_VehPosOdo_pst == NULL)
    {
        return FALSE;
    }
    else
    {
#if 1
        f_VehPosOdo_pst->SPosMM_si32 =
            g_VHOVehicleState_st.DrivenDistance_si32;                        // 1LSB = 1mm
        f_VehPosOdo_pst->XPosMM_si32 = g_VHOVehicleState_st.XPosition_si32;  // 1LSB = 1mm
        f_VehPosOdo_pst->YPosMM_si32 = g_VHOVehicleState_st.YPosition_si32;  // 1LSB = 1mm
        f_VehPosOdo_pst->YawAngle_ui32 = g_VHOVehicleState_st.YawAngle_ui32; // F22

        f_VehPosOdo_pst->WheelAngleFront_si16 =
            g_VHOVehicleState_st.SAngRadF12_si16; // 1LSB = 1.0 rad / 2^12

        // DBG("[ODOdata] ODO-X: %d, ODO-Y: %d, ODO-S: %d, ODO-Ang: %d\n",
        //     g_VHOVehicleState_st.XPosition_si32, g_VHOVehicleState_st.YPosition_si32,
        //     g_VHOVehicleState_st.DrivenDistance_si32,
        //     g_VHOVehicleState_st.YawAngle_ui32);

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

        f_VehPosOdo_pst->YawAngleYRSens_ui32 =
            g_VHOVehicleState_st.YawAngleYRSens_ui32; // 1LSB = 1.0 rad / 2^22

#if (GS_4WS_VEHICLE == SW_ON)
        {
            SInt16 l_YawAngleF12_si16;
            SInt16 l_CosYawAngle_si16;
            SInt16 l_SinYawAngle_si16;
            SInt32 l_temp1_si32;
            SInt32 l_temp2_si32;
            SInt32 l_XPosDelta_si32;
            SInt32 l_YPosDelta_si32;
            SInt32 l_XPosRearAxle_si32;
            SInt32 l_YPosRearAxle_si32;

            //=====================================================================
            // map driven distance from center of rear axle to instantaneous center
            //=====================================================================
            l_temp1_si32 = g_VHOVehicleState_st.DrivenDistance_si32;    // 1LSB = 1mm
            l_temp1_si32 = g_mtl_ReducePrecision_si32(l_temp1_si32, 2); // -F2
            // + F14
            l_temp1_si32 = l_temp1_si32 * (SInt32)g_mtl_Cos_si16(
                                              g_VHOVehicleState_st.SAngRearRadF12_si16);
            l_temp1_si32 = g_mtl_ReducePrecision_si32(l_temp1_si32, 12); // -F12
            f_VehPosOdo_pst->SPosInstantCenterMM_si32 = l_temp1_si32;    // 1LSB = 1mm

            //========================================================================
            // map XPosition center of the rear axle to XPosition instantaneous center
            // map YPosition center of the rear axle to YPosition instantaneous center
            //========================================================================
            // F12 = F22 - F10
            l_YawAngleF12_si16 = (SInt16)g_mtl_ReducePrecision_si32(
                (SInt32)g_VHOVehicleState_st.YawAngle_ui32, 10);
            l_CosYawAngle_si16 = g_mtl_Cos_si16(l_YawAngleF12_si16); // F14
            l_SinYawAngle_si16 = g_mtl_Sin_si16(l_YawAngleF12_si16); // F14

            l_temp1_si32 =
                (SInt32)g_VHOWheelBaseVirtual_st.DeltaWheelBaseVirtual_si16; // 1LSB = 1mm
            l_temp1_si32 = g_mtl_ReducePrecision_si32(l_temp1_si32, 2);      // -F2

            l_temp2_si32 = l_temp1_si32 * (SInt32)l_CosYawAngle_si16; // +F14
            l_XPosDelta_si32 =
                g_mtl_ReducePrecision_si32(l_temp2_si32, 12); // -F12  1LSB = 1mm

            l_temp2_si32 = l_temp1_si32 * (SInt32)l_SinYawAngle_si16; // +F14
            l_YPosDelta_si32 =
                g_mtl_ReducePrecision_si32(l_temp2_si32, 12); // -F12  1LSB = 1mm

            l_XPosRearAxle_si32 = g_VHOVehicleState_st.XPosition_si32; // 1LSB = 1mm
            l_YPosRearAxle_si32 = g_VHOVehicleState_st.YPosition_si32; // 1LSB = 1mm

            f_VehPosOdo_pst->XPosInstantCenterMM_si32 =
                l_XPosRearAxle_si32 - l_XPosDelta_si32; // 1LSB = 1mm
            f_VehPosOdo_pst->YPosInstantCenterMM_si32 =
                l_YPosRearAxle_si32 - l_YPosDelta_si32; // 1LSB = 1mm
        }
        f_VehPosOdo_pst->WheelBaseVirtualMM_si32 =
            (SInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16; // 1LSB = 1mm
        f_VehPosOdo_pst->DeltaWheelBaseVirtualMM_si32 =
            (SInt32)g_VHOWheelBaseVirtual_st.DeltaWheelBaseVirtual_si16; // 1LSB = 1mm
        f_VehPosOdo_pst->SlipAngle_si32 =
            ((SInt32)(g_VHOVehicleState_st.SAngRearRadF12_si16) *
             (SInt32)(1024)); // F12 --> F22
#endif
#endif
#else
        f_VehPosOdo_pst->SPosMM_si32   = g_VHOVehicleODO_st.DrivenDistance_si32;
        f_VehPosOdo_pst->XPosMM_si32   = g_VHOVehicleODO_st.XPosition_si32;
        f_VehPosOdo_pst->YPosMM_si32   = g_VHOVehicleODO_st.YPosition_si32;
        f_VehPosOdo_pst->YawAngle_ui32 = g_VHOVehicleODO_st.YawAngle_ui32;

#endif
        return TRUE;
    }
}
/**
 * @brief
 * @details
 *
 * @param         f_Kappa_psi16
 * @return        Boolean
 *
 * @note
 * @see
 * @warning
 */
Boolean g_adiGetVehKappaOdo_bl(SInt16 *f_Kappa_psi16)
{
    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (f_Kappa_psi16 != NULL)
    {
        (*f_Kappa_psi16) = g_VHOVehicleState_st.KappaF14_si16;
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/*
Boolean g_adiGetVehSpeed_bl (UInt16* f_VehSpeed_pui16)
{
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

   UInt32 l_temp1_ui32;
   Boolean l_RetVal_bl;

  //----------------
  // nprod00133486 |
  // ---------------
  // in case of invalid wheel impulse counter the
  // can speed must be forwarded to the adi interface
  if ((g_VHOCanSig_st.WIC_Invalid_ui8 != (UInt8) 0) ||
(g_VHOAutocalib_st.WPTimeCounterStart2_ui8 != 0))
  {
    // g_VHOCanSig_st.Vel_st.ValRaw_si32 = 0.1km/h
    g_VHOVehicleState_st.adiVehicleVelocity_ui16 = (UInt16)
g_VHOCanSig_st.Vel_ui16; //0.1km/h l_RetVal_bl =
g_VHOCanSig_st.Vel_ValidFlag_bl;
  }
  else
  {
    if (g_VHOParameters_st.VHO_Odo_pst->ThresholdPassThroughExternalSpeed_ui16
== 0)
    {
      // in this case the external velocity signal is passed trough
      // without any restrictions
      g_VHOVehicleState_st.adiVehicleVelocity_ui16 = (UInt16)
g_VHOCanSig_st.Vel_ui16; //0.1km/h l_RetVal_bl =
g_VHOCanSig_st.Vel_ValidFlag_bl;
    }
    else if ((UInt16) g_VHOCanSig_st.Vel_ui16 <
g_VHOParameters_st.VHO_Odo_pst->ThresholdPassThroughExternalSpeed_ui16)
    {
      // if the external velocity signal is lower as the threshold then the VHO
internal velocity
      // will pass through the ADI output

      //1/256 m/s --> 0.1km/h
      l_temp1_ui32 = ((UInt32) g_VHOVehicleState_st.VehicleVelocity_ui16 *
(UInt32)36);

      g_VHOVehicleState_st.adiVehicleVelocity_ui16 = (UInt16)  (l_temp1_ui32 /
(UInt32) 256); //0.1km/h l_RetVal_bl = TRUE;
    }
    else
    {
      // g_VHOCanSig_st.Vel_st.ValCor_si32 >
g_VHOParameters_st.VHO_Odo_pst->ThresholdPassThroughExternalSpeed_ui16

      //if the external velocity signal exceeds the threshold the external
signal is passed through the ADI output
      g_VHOVehicleState_st.adiVehicleVelocity_ui16 = (UInt16)
g_VHOCanSig_st.Vel_ui16; //0.1km/h l_RetVal_bl =
g_VHOCanSig_st.Vel_ValidFlag_bl;
    }
  }
#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)

  Boolean l_RetVal_bl;

  g_VHOVehicleState_st.adiVehicleVelocity_ui16 = (UInt16)
g_VHOCanSig_st.Vel_ui16; //0.1km/h l_RetVal_bl = TRUE;


#endif
  (*f_VehSpeed_pui16) = g_VHOVehicleState_st.adiVehicleVelocity_ui16;
  return l_RetVal_bl;
}
*/

/* shi2lr: Funktion wird nicht mehr vom VHO bereit gestellt, sondern nur vom VHS
s. Beschluss im SUP-SE-Team vom 12.07.2011

Boolean g_adiGetSWA_bl (gType_SWAandTime_st* f_SWA_pst)
{

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

  f_SWA_pst->SWA_Signal_si16 = (SInt16) g_VHOCanSig_st.SWA_si16;
  f_SWA_pst->SWA_Tick_ui16 = g_VHOCanSig_st.SWA_Tick_ui16;
  return (Boolean) g_VHOCanSig_st.SWA_ValidFlag_bl;

#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)

  f_SWA_pst->SWA_Signal_si16 = (SInt16) g_VHOCanSig_st.SWA_si16;
  f_SWA_pst->SWA_Tick_ui16   = (UInt16) 0;
  return TRUE;

#endif

}

Boolean g_adiGetLongAcc_bl (SInt16* f_LongAcc_psi16)
{
  #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    (*f_LongAcc_psi16) = g_VHOVehicleState_st.VehicleAcceleration_si16;
  #else
    (*f_LongAcc_psi16) = 0;
  #endif
  return TRUE;
}

Boolean g_adiGetYawRate_bl (SInt16* f_YawRate_psi16)
{
  #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    (*f_YawRate_psi16) = g_VHOVehicleState_st.YawRate_si16;
  #else
    (*f_YawRate_psi16) = 0;
  #endif
  return TRUE;
}
*/
#if 0
/**
 * @brief
 * @details
 *
 * @param         f_CarMoveDir_pen
 * @return        Boolean
 *
 * @note
 * @see
 * @warning
 */
Boolean g_adiGetCarMoveDir_bl(gType_CarMoveDir_en *f_CarMoveDir_pen) {
  /**************************************/
  /* Avoid possible use of zero pointer */
  /**************************************/
  if (f_CarMoveDir_pen == NULL) {
    return FALSE;
  } else {
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if (((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)) &&    \
     (LS_VHO_WIC_FILTER_ELEPHANT == SW_ON))
    // i.e., if elephant test is available
    // in case of LS_VHO_DIRDETECT_ALL and parameter DirDetectMode is set to
    // gear, this variable is always FALSE and does not lead to a wrong car move
    // direction
    if (g_VHORollRecog_st.WIC_MovingIndication_bl != FALSE) {
      (*f_CarMoveDir_pen) = MoveDirStop_enm;
    } else {
#endif
      switch (g_VHOVehicleState_st.RollingDirection_en) {
      case VHO_MOV_FORW_enm:
      case VHO_MOV_TURNED_FORW_enm:
        (*f_CarMoveDir_pen) = MoveDirForward_enm;
        break;

      case VHO_MOV_BACKW_enm:
      case VHO_MOV_TURNED_BACKW_enm:
        (*f_CarMoveDir_pen) = MoveDirBackward_enm;
        break;

      case VHO_STOP_enm:
        (*f_CarMoveDir_pen) = MoveDirStop_enm;
        break;

      case VHO_MOV_UNKNOWN_enm:
      case VHO_MOV_UNDER_INVESTIGATION_enm:
        (*f_CarMoveDir_pen) = MoveDirUnknown_enm;
        break;

      default:
        (*f_CarMoveDir_pen) = MoveDirUnknown_enm;
        break;
      }
#if (((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)) &&    \
     (LS_VHO_WIC_FILTER_ELEPHANT == SW_ON))
    }
#endif // (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) || ...
#else  // i.e. if (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)
    // Workaround bzgl. Switch-Klammerung, da im Mini-VHO der Switch
    // LS_VHO_WIC_FILTER_ELEPHANT nicht definiert ist.

    switch (g_VHOVehicleState_st.RollingDirection_en) {
    case VHO_MOV_FORW_enm:
    case VHO_MOV_TURNED_FORW_enm:
      (*f_CarMoveDir_pen) = MoveDirForward_enm;
      break;

    case VHO_MOV_BACKW_enm:
    case VHO_MOV_TURNED_BACKW_enm:
      (*f_CarMoveDir_pen) = MoveDirBackward_enm;
      break;

    case VHO_STOP_enm:
      (*f_CarMoveDir_pen) = MoveDirStop_enm;
      break;

    case VHO_MOV_UNKNOWN_enm:
    case VHO_MOV_UNDER_INVESTIGATION_enm:
      (*f_CarMoveDir_pen) = MoveDirUnknown_enm;
      break;

    default:
      (*f_CarMoveDir_pen) = MoveDirUnknown_enm;
      break;
    }
#endif // (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

    return TRUE;
  }
}
#endif
/**
 * @brief vho drive distance from start to now.
 * @details
 *
 * @param         f_adiVHODrivenDistanceAbsolute_pui32, can msg
 * @return        ok ~ TRUE, error ~ FALSE.
 *
 * @note
 * @see
 * @warning
 */
Boolean g_adiVHODrivenDistanceAbsolute_bl(UInt32 *f_adiVHODrivenDistanceAbsolute_pui32)
{
    // The ADI function "g_adiGetVehPosOdo_bl" providing a driven distance which
    // increment or decrement depending on the detected rolling direction. Special
    // customer requirements claim a system reaction depending on a detected
    // driven distance independent of a detected driving direction.
    // --> Pull-Out-Control, Back-Roll-Avoidance

    // Tracked Item: nprod00134646

    // g_VHOOdoBuf_st.DrivenDistanceMMF8_si32 --> buffered driven distance of
    // detected wic deltas in current cycle
    // g_VHOOdoBuf_st.DrivenDistance_bufF8_si16 --> variable contain the
    // corresponding high resolution part for values smaller than 1mm (range: 0mm
    // - 255/256mm)

    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (f_adiVHODrivenDistanceAbsolute_pui32 != NULL)
    {
        (*f_adiVHODrivenDistanceAbsolute_pui32) =
            g_VHOVehicleState_st.DrivenDistanceAbsolute_ui32;
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/**
 * @brief get vehicle acceleration from can msg
 * @details
 *
 * @param         f_VehAcc_psi16
 * @return        Boolean
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
Boolean g_adiGetVehicleAcceleration_bl(SInt16 *f_VehAcc_psi16)
{
#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) || \
     (LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON))
    {
        /**************************************/
        /* Avoid possible use of zero pointer */
        /**************************************/
        if (f_VehAcc_psi16 != NULL)
        {
            (*f_VehAcc_psi16) = g_VHOVehicleState_st.VehicleAcceleration_si16;
            return TRUE;
        }
        else
        {
            return FALSE;
        }
    }
#else
    {
        return FALSE;
    }
#endif
}
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (LS_VHO_PROVIDE_DEVELOPMENT_DATA == SW_ON)
Boolean g_adiGetVhoDevelopmentData_bl(
    gType_VHODevelopmentData_st *f_VhoDevelopmentData_pst)
{
    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (f_VhoDevelopmentData_pst != NULL)
    {
        f_VhoDevelopmentData_pst->SWAState_enm = g_VHOAutocalibCtrl_st.SWAState_enm;
        f_VhoDevelopmentData_pst->YRState_enm  = g_VHOAutocalibCtrl_st.YRState_enm;
        f_VhoDevelopmentData_pst->SWAOff_si16  = g_VHOAutocalibCtrl_st.SWOffDegRes_si16;
        f_VhoDevelopmentData_pst->YROff_si16   = g_VHOAutocalibCtrl_st.YROff_si16;
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
#endif
#endif

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 2006
#endif
