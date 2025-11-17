/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: vhs_adi.h
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#ifndef VHS_API_H
#define VHS_API_H

/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/

// component headers include strategy:
// ===================================
//       <vhs_bas.h>                          (optional header)            |
//            I--> <appl_vhs.h>               (optional header)            V
//                           I--> <vhs_api.h> (header include order)

#include "appl_vhs.h"

/*--------------------------------------------------------------------------*/
/*- check if compiler switches (utilized in this file) are defined         -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*!\addtogroup VHS_M02           exported symbolic constants
 * \ingroup COMPONENT_VHS
 @{ *//*--------------------------------------------------------------------*/
#ifdef COMPONENT_VHS // Only defined within this component! /*!\cond INTERNAL */
// number (current date "JJMMDD") for component compatibility check between
// appl_... and core files
#define APPL_VHS_H_REF        0x110723
#define APPL_VHS_C_REF        0x110723
#define APPL_VHS_OUTPUT_C_REF 0x110421

#endif // COMPONENT_VHS                                     /*!\endcond       */

/*!@} */ /*------------------------------------------------------------------*/
/*!\addtogroup VHS_M03           exported macros
 * \ingroup COMPONENT_VHS
 @{ *//*--------------------------------------------------------------------*/
#ifdef COMPONENT_VHS // Only defined within this component! /*!\cond INTERNAL */
#endif // COMPONENT_VHS                                     /*!\endcond       */

/*!@} */ /*------------------------------------------------------------------*/
/*!\addtogroup VHS_M04           exported data types
 * \ingroup COMPONENT_VHS
 @{ *//*--------------------------------------------------------------------*/
#ifdef COMPONENT_VHS // Only defined within this component! /*!\cond INTERNAL */
#endif // COMPONENT_VHS                                     /*!\endcond       */

/*!@} */ /*------------------------------------------------------------------*/
/*-                           external object declarations                 -*/
/*--------------------------------------------------------------------------*/

extern gType_VhsDegModes_en g_VhsDegMode_en;

extern UInt16 g_adiVehSpeed_ui16;
#if (GS_VHS_GEAR != SW_OFF)
extern gType_Gear_en g_adiGear_en;
#endif
extern SInt8 g_adiOutsideTemp_si8;
extern Boolean g_adiTrailerHitchPresent_bl;
#if (GS_VHS_CAR_MOVE_DIR != SW_OFF)
extern gType_CarMoveDir_en g_adiCarMoveDir_en;
#endif
#if (GS_VHS_SWA != SW_OFF)
extern gType_SWAandTime_st g_adiSWA_st;
#endif
#if (GS_VHS_INDICATOR != SW_OFF)
extern gType_Indicator_en g_adiIndicator_en;
#endif
#if (GS_VHS_WICRR != SW_OFF)
extern gType_WICandTime_st g_adiWICRR_st;
#endif
#if (GS_VHS_WICRL != SW_OFF)
extern gType_WICandTime_st g_adiWICRL_st;
#endif
#if (GS_VHS_WICFR != SW_OFF)
extern gType_WICandTime_st g_adiWICFR_st;
#endif
#if (GS_VHS_WICFL != SW_OFF)
extern gType_WICandTime_st g_adiWICFL_st;
#endif
#if (GS_VHS_WHEEL_DIR_RR != SW_OFF)
extern gType_WheelDirRR_en g_adiWheelDirRR_en;
#endif
#if (GS_VHS_WHEEL_DIR_RL != SW_OFF)
extern gType_WheelDirRL_en g_adiWheelDirRL_en;
#endif
#if (GS_VHS_WHEEL_DIR_FR != SW_OFF)
extern gType_WheelDirFR_en g_adiWheelDirFR_en;
#endif
#if (GS_VHS_WHEEL_DIR_FL != SW_OFF)
extern gType_WheelDirFL_en g_adiWheelDirFL_en;
#endif
extern UInt16 g_adiWheelRotationRR_ui16;
extern UInt16 g_adiWheelRotationRL_ui16;
extern UInt16 g_adiWheelRotationFR_ui16;
extern UInt16 g_adiWheelRotationFL_ui16;

extern SInt16 g_adiLongAcc_si16;
extern SInt16 g_adiYawRate_si16;
#if (GS_VHS_EPS_STATUS != SW_OFF)
extern gType_EpsStatus_en g_adiEpsStatus_en;
#endif
#if (GS_VHS_ENGINE_START != SW_OFF)
extern gType_EngineStart_en g_adiEngineStart_en;
#endif

extern gType_EscStatus_en g_adiEscStatus_en;

extern gType_PowerMode_en g_adiPowerMode_en;

extern gType_PSMode_en g_adiPSMode_en;

extern void g_vhsInit_vd(
    void); // executed once after start-up initialization is completed
extern void g_vhsInputProcessing_vd(
    void); // executed every 10ms in cyclic task (for input signal conversion)
extern void g_vhsOutputProcessing_vd(
    void); // executed every 10ms in cyclic task (for output signal conversion)

extern void g_vhsSetDegMode_vd(gType_VhsDegModes_en f_VhsDegMode_en);

/*=====================================================*/
/*          Parking aid (PA) INPUT SIGNALS             */
/*  ADI Interface to all other PA internal components  */
/*=====================================================*/

extern Boolean g_adiGetVehSpeed_bl(UInt16 *f_VehSpeed_pui16);

#if (GS_VHS_CAR_MOVE_DIR != SW_OFF)
extern Boolean g_adiGetCarMoveDir_bl(gType_CarMoveDir_en *f_CarMoveDir_pen);
#endif

#if (GS_VHS_GEAR != SW_OFF)
extern Boolean g_adiGetGear_bl(gType_Gear_en *f_Gear_pen);
#endif
extern Boolean g_adiGetOutsideTemp_bl(SInt8 *f_OutsideTemp_psi8);
extern Boolean g_adiGetTrailerHitchPresent_bl(Boolean *f_TrailerHitchPresent_pbl);
extern Boolean g_adiGetTrailerAttached_bl(Boolean *f_TrailerAttached_pbl);
#if (GS_VHS_SWA != SW_OFF)
extern Boolean g_adiGetSWA_bl(gType_SWAandTime_st *f_SWA_pst);
#endif
#if (GS_VHS_INDICATOR != SW_OFF)
extern Boolean g_adiGetIndicator_bl(gType_Indicator_en *f_Indicator_pen);
// "g_adiGetIndicatorInfo_bl" is a temporary solution until all components have
// changed to g_adiGetIndicator_bl() g_adiGetIndicator_bl() is generated by
// GenSig
#define g_adiGetIndicatorInfo_bl(x) g_adiGetIndicator_bl(x)
#endif

#if (GS_VHS_WICRR != SW_OFF)
extern Boolean g_adiGetWICRR_bl(gType_WICandTime_st *f_WICRR_pst);
#endif
#if (GS_VHS_WICRL != SW_OFF)
extern Boolean g_adiGetWICRL_bl(gType_WICandTime_st *f_WICRL_pst);
#endif
#if (GS_VHS_WICFR != SW_OFF)
extern Boolean g_adiGetWICFR_bl(gType_WICandTime_st *f_WICFR_pst);
#endif
#if (GS_VHS_WICFL != SW_OFF)
extern Boolean g_adiGetWICFL_bl(gType_WICandTime_st *f_WICFL_pst);
#endif

#if (GS_VHS_WHEEL_DIR_RR != SW_OFF)
extern Boolean g_adiGetWheelDirRR_bl(gType_WheelDir_en *f_WheelDirRR_pen);
#endif
#if (GS_VHS_WHEEL_DIR_RL != SW_OFF)
extern Boolean g_adiGetWheelDirRL_bl(gType_WheelDir_en *f_WheelDirRL_pen);
#endif
#if (GS_VHS_WHEEL_DIR_FR != SW_OFF)
extern Boolean g_adiGetWheelDirFR_bl(gType_WheelDir_en *f_WheelDirFR_pen);
#endif
#if (GS_VHS_WHEEL_DIR_FL != SW_OFF)
extern Boolean g_adiGetWheelDirFL_bl(gType_WheelDir_en *f_WheelDirFL_pen);
#endif

extern Boolean g_adiGetWheelRotationRR_bl(UInt16 *f_WheelRotationRR_pui16);
extern Boolean g_adiGetWheelRotationRL_bl(UInt16 *f_WheelRotationRL_pui16);
extern Boolean g_adiGetWheelRotationFR_bl(UInt16 *f_WheelRotationFR_pui16);
extern Boolean g_adiGetWheelRotationFL_bl(UInt16 *f_WheelRotationFL_pui16);
extern Boolean g_adiGetLongAcc_bl(SInt16 *f_LongAcc_psi16);
extern Boolean g_adiGetYawRate_bl(SInt16 *f_YawRate_psi16);

extern Boolean g_adiGetWickTick_bl(UInt16 *f_wicTick_pui16);

#if (GS_VHS_EPS_STATUS != SW_OFF)
extern Boolean g_adiGetEpsStatus_bl(gType_EpsStatus_en *f_EpsStatus_pen);
#endif

Boolean g_adiGetEscStatus_bl(gType_EscStatus_en *f_EscStatus_pen);

#if (GS_VHS_ENGINE_START != SW_OFF)
extern Boolean g_adiGetEngineStart_bl(gType_EngineStart_en *f_EngineStart_pen);
#endif

#endif
