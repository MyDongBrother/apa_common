/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: vhs_adi.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/
#define COMPONENT_VHS

#include <pa_components/base_type/global_include.h>

// within COMPONENT_VHS)
#include "vhs_api.h"
// #include <par_api.h>

#ifndef SW_ON
#error ('global switch SW_ON is not defined')
#endif

#ifndef SW_OFF
#error ('global switch SW_OFF is not defined')
#endif

#if (GS_VHS_VEH_SPEED != SW_OFF)

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
------------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         Interface function for accessing VehSpeed signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_VehSpeed_pui16
 * @return        Boolean             TRUE  Signal was correctly converted;
 *                                    FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note          PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetVehSpeed_bl(UInt16 *f_VehSpeed_pui16)
{

    *f_VehSpeed_pui16 = g_adiVehSpeed_ui16;

    return (TRUE);
}
#endif

#if (GS_VHS_CAR_MOVE_DIR != SW_OFF)
/**
 * @brief  get car move direction
 * @details
 *
 * @param      f_CarMoveDir_pen
 * @return
 *
 * @note
 * @see
 * @warning
 */
Boolean g_adiGetCarMoveDir_bl(gType_CarMoveDir_en *f_CarMoveDir_pen)
{

    *f_CarMoveDir_pen = g_adiCarMoveDir_en;

    return (TRUE);
}

#endif

#if (GS_VHS_GEAR != SW_OFF)
/**
 * @brief  get gear value
 * @details
 *
 * @param      f_Gear_pen
 * @return
 *
 * @note
 * @see
 * @warning
 */
Boolean g_adiGetGear_bl(gType_Gear_en *f_Gear_pen)
{

    *f_Gear_pen = g_adiGear_en;

    return (TRUE);
}
#endif

#if (GS_VHS_OUTSIDE_TEMP != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
------------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         Interface function for accessing OutsideTemp signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_OutsideTemp_psi8
 * @return        Boolean              TRUE  Signal was correctly converted;
 *                                     FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note          PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetOutsideTemp_bl(SInt8 *f_OutsideTemp_psi8)
{

    *f_OutsideTemp_psi8 = g_adiOutsideTemp_si8;

    return (TRUE);
}
#endif

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
------------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         Interface function for accessing TrailerHitchPresent signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_TrailerHitchPresent_pbl
 * @return        Boolean             TRUE  Signal was correctly converted;
 *                                    FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note          PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetTrailerHitchPresent_bl(Boolean *f_TrailerHitchPresent_pbl)
{

    *f_TrailerHitchPresent_pbl = g_adiTrailerHitchPresent_bl;

    return (TRUE);
}

#if (GS_VHS_SWA != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Interface function for accessing SWA signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_SWA_pst
 * @return        Boolean             TRUE  Signal was correctly converted;
 *                                    FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note          PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetSWA_bl(gType_SWAandTime_st *f_SWA_pst)
{

    f_SWA_pst->SWA_Signal_si16 = g_adiSWA_st.SWA_Signal_si16;
    f_SWA_pst->SWA_Tick_ui16   = g_adiSWA_st.SWA_Tick_ui16;

    return (TRUE);
}
#endif

#if (GS_VHS_INDICATOR != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
------------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         Interface function for accessing Indicator signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_Indicator_pen
 * @return        Boolean           TRUE  Signal was correctly converted;
 *                                  FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note          PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetIndicator_bl(gType_Indicator_en *f_Indicator_pen)
{

    *f_Indicator_pen = g_adiIndicator_en;

    return (TRUE);
}
#endif

#if (GS_VHS_WICRR != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Interface function for accessing WICRR signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_WICRR_pst
 * @return        Boolean           TRUE  Signal was correctly converted;
 *                                  FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note          PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetWICRR_bl(gType_WICandTime_st *f_WICRR_pst)
{

    f_WICRR_pst->WIC_Signal_ui16 = g_adiWICRR_st.WIC_Signal_ui16;
    f_WICRR_pst->WIC_Tick_ui16   = g_adiWICRR_st.WIC_Tick_ui16;

    return (TRUE);
}
#endif
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
------------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         Interface function for accessing WICRL signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_WICRL_pst
 * @return        Boolean              TRUE  Signal was correctly converted;
 *                                     FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note          PA adi signal updated
 * @see
 * @warning
 */
#if (GS_VHS_WICRL != SW_OFF)

Boolean g_adiGetWICRL_bl(gType_WICandTime_st *f_WICRL_pst)
{

    f_WICRL_pst->WIC_Signal_ui16 = g_adiWICRL_st.WIC_Signal_ui16;
    f_WICRL_pst->WIC_Tick_ui16   = g_adiWICRL_st.WIC_Tick_ui16;

    return (TRUE);
}
#endif
/**
 * @brief get wheel tick of a wheel.
 * @details
 *
 * @param      f_wicTick_pui16, argument to save wheel tick value.
 * @return
 *
 * @note
 * @see
 * @warning
 */
Boolean g_adiGetWickTick_bl(UInt16 *f_wicTick_pui16)
{

    *f_wicTick_pui16 = g_adiWICRL_st.WIC_Tick_ui16;

    return (TRUE);
}

#if (GS_VHS_WICFR != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Interface function for accessing WICFR signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_WICFR_pst
 * @return        Boolean             TRUE  Signal was correctly converted;
 *                                    FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note          PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetWICFR_bl(gType_WICandTime_st *f_WICFR_pst)
{

    f_WICFR_pst->WIC_Signal_ui16 = g_adiWICFR_st.WIC_Signal_ui16;
    f_WICFR_pst->WIC_Tick_ui16   = g_adiWICFR_st.WIC_Tick_ui16;

    return (TRUE);
}
#endif

#if (GS_VHS_WICFL != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Interface function for accessing WICFL signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_WICFL_pst
 * @return        Boolean           TRUE  Signal was correctly converted;
 *                                  FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note       later,   PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetWICFL_bl(gType_WICandTime_st *f_WICFL_pst)
{

    f_WICFL_pst->WIC_Signal_ui16 = g_adiWICFL_st.WIC_Signal_ui16;
    f_WICFL_pst->WIC_Tick_ui16   = g_adiWICFL_st.WIC_Tick_ui16;

    return (TRUE);
}
#endif

#if (GS_VHS_WHEEL_DIR_RR != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Interface function for accessing WheelDirRR signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_WheelDirRR_pen
 * @return        Boolean           TRUE  Signal was correctly converted;
 *                                  FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note        later,  PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetWheelDirRR_bl(gType_WheelDir_en *f_WheelDirRR_pen)
{

    *f_WheelDirRR_pen = g_adiWheelDirRR_en;

    return (TRUE);
}
#endif

#if (GS_VHS_WHEEL_DIR_RL != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Interface function for accessing WheelDirRL signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_WheelDirRL_pen
 * @return        Boolean             TRUE  Signal was correctly converted;
 *                                    FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note      later,   PA adi signal updated
 * @see
 * @warning
 */

Boolean g_adiGetWheelDirRL_bl(gType_WheelDir_en *f_WheelDirRL_pen)
{

    *f_WheelDirRL_pen = g_adiWheelDirRL_en;

    return (TRUE);
}
#endif

#if (GS_VHS_WHEEL_DIR_FR != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Interface function for accessing WheelDirFR signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_WheelDirFR_pen
 * @return        Boolean             TRUE  Signal was correctly converted;
 *                                    FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note      later,    PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetWheelDirFR_bl(gType_WheelDir_en *f_WheelDirFR_pen)
{

    *f_WheelDirFR_pen = g_adiWheelDirFR_en;

    return (TRUE);
}
#endif

#if (GS_VHS_WHEEL_DIR_FL != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Interface function for accessing WheelDirFL signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_WheelDirFL_pen
 * @return        Boolean             TRUE  Signal was correctly converted;
 *                                    FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note      later,    PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetWheelDirFL_bl(gType_WheelDir_en *f_WheelDirFL_pen)
{

    *f_WheelDirFL_pen = g_adiWheelDirFL_en;

    return (TRUE);
}
#endif

#if (GS_VHS_WHEEL_ROTATION_RR != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Interface function for accessing WheelRotationRR signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_WheelRotationRR_pui16
 * @return        Boolean             TRUE  Signal was correctly converte
 *                                    FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note      later    PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetWheelRotationRR_bl(UInt16 *f_WheelRotationRR_pui16)
{

    *f_WheelRotationRR_pui16 = g_adiWheelRotationRR_ui16;

    return (TRUE);
}
#endif

#if (GS_VHS_WHEEL_ROTATION_RL != SW_OFF)

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Interface function for accessing WheelRotationRL signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_WheelRotationRL_pui16
 * @return        Boolean            TRUE  Signal was correctly converted;
 *                                   FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note        later,  PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetWheelRotationRL_bl(UInt16 *f_WheelRotationRL_pui16)
{

    *f_WheelRotationRL_pui16 = g_adiWheelRotationRL_ui16;

    return (TRUE);
}
#endif

#if (GS_VHS_WHEEL_ROTATION_FR != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Interface function for accessing WheelRotationFR signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_WheelRotationFR_pui16
 * @return        Boolean            TRUE  Signal was correctly converted;
 *                                   FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note        later,  PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetWheelRotationFR_bl(UInt16 *f_WheelRotationFR_pui16)
{

    *f_WheelRotationFR_pui16 = g_adiWheelRotationFR_ui16;

    return (TRUE);
}
#endif

#if (GS_VHS_WHEEL_ROTATION_FL != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Interface function for accessing WheelRotationFL signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_WheelRotationFR_pui16
 * @return        Boolean            TRUE  Signal was correctly converted;
 *                                   FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note        later,  PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetWheelRotationFL_bl(UInt16 *f_WheelRotationFL_pui16)
{

    *f_WheelRotationFL_pui16 = g_adiWheelRotationFL_ui16;

    return (TRUE);
}
#endif

#if (GS_VHS_LONG_ACC != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Interface function for accessing LongAcc signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_LongAcc_psi16
 * @return        Boolean              TRUE  Signal was correctly converted;
 *                                     FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note      later,    PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetLongAcc_bl(SInt16 *f_LongAcc_psi16)
{

    *f_LongAcc_psi16 = g_adiLongAcc_si16;

    return (TRUE);
}
#endif

#if (GS_VHS_YAW_RATE != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
------------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         Interface function for accessing YawRate signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_YawRate_psi16
 * @return        Boolean              TRUE  Signal was correctly converted;
 *                                     FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note        later,  PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetYawRate_bl(SInt16 *f_YawRate_psi16)
{

    *f_YawRate_psi16 = g_adiYawRate_si16;

    return (TRUE);
}
#endif

#if (GS_VHS_EPS_STATUS != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Interface function for accessing EpsStatus signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_EpsStatus_pen
 * @return        Boolean             TRUE  Signal was correctly converted;
 *                                    FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note       later,   PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetEpsStatus_bl(gType_EpsStatus_en *f_EpsStatus_pen)
{

    *f_EpsStatus_pen = g_adiEpsStatus_en;

    return (TRUE);
}
#endif

#if (GS_VHS_ENGINE_START != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Interface function for accessing EngineStart signal.
 * @details       Signal provided by VHS for other components
 *
 * @param         f_EngineStart_pen
 * @return        Boolean             TRUE  Signal was correctly converted;
 *                                    FALSE Signal was substituted (by default
 * value or last known value)
 *
 * @note        later,  PA adi signal updated
 * @see
 * @warning
 */
Boolean g_adiGetEngineStart_bl(gType_EngineStart_en *f_EngineStart_pen)
{

    *f_EngineStart_pen = g_adiEngineStart_en;

    return (TRUE);
}
#endif
/**
 * @brief get ECU power status
 * @details
 *
 * @param   f_PowerMode_pen
 * @return
 *
 * @note
 * @see
 * @warning
 */
Boolean g_adiGetPowerMode_bl(gType_PowerMode_en *f_PowerMode_pen)
{

    *f_PowerMode_pen = g_adiPowerMode_en;

    return (TRUE);
}
/**
 * @brief     PSVeloSwitch
 * @details
 *
 * @param   f_PSMode_pen
 * @return
 *
 * @note
 * @see
 * @warning
 */
/*Boolean g_adiGetPSMode_bl(gType_PSMode_en *f_PSMode_pen) {

  *f_PSMode_pen = g_adiPSMode_en;

  return (TRUE);
}*/
/**
 * @brief  get esc status,
 * @details
 *
 * @param   f_EscStatus_pen
 * @return
 *
 * @note
 * @see
 * @warning
 */
Boolean g_adiGetEscStatus_bl(gType_EscStatus_en *f_EscStatus_pen)
{

    *f_EscStatus_pen = g_adiEscStatus_en;

    return (TRUE);
}
