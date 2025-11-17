/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: vhs.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#define COMPONENT_VHS

#include <pa_components/base_type/global_include.h>
#include <pa_components/mathlib/mathlib_api.h>
#include <pa_components/parameter/par_api.h>
// #include <veh/vhm/vhm_bas.h>
// #include <sp_fsv_api.h> // must be defined before vhs_api.h (only required
//  within COMPONENT_VHS)
#include "vhs.h"
#include "vhs_api.h"

#if (LS_VHS_TESTHARNESS != SW_OFF)
#include <vhs_testharness.h>
#endif
// #include <mathlib_api.h>
#include <string.h>

#ifndef SW_ON
#error ('global switch SW_ON is not defined')
#endif

#ifndef SW_OFF
#error ('global switch SW_OFF is not defined')
#endif

#ifndef LS_VHS_TESTHARNESS
#error ('switch LS_VHS_TESTHARNESS is not defined')
#endif
#ifndef LS_VHS_FLTSYMPT_TRIG
#error ('switch LS_VHS_FLTSYMPT_TRIG is not defined')
#endif

#ifndef GS_VHS_VEH_SPEED
#error ('compiler switch GS_VHS_VEH_SPEED is not defined')
#endif
#ifndef GS_VHS_WICRR
#error ('compiler switch GS_VHS_WICRR is not defined')
#endif
#ifndef GS_VHS_WICRL
#error ('compiler switch GS_VHS_WICRL is not defined')
#endif

#ifndef LS_VHS_CAN_SPEED_SUBST
#error ('switch LS_VHS_CAN_SPEED_SUBST is not defined')
#endif

#if ((LS_VHS_CAN_SPEED_SUBST != SW_OFF) &&                  \
     ((GS_VHS_WICRR != SW_ON) || (GS_VHS_WICRL != SW_ON) || \
      (GS_VHS_VEH_SPEED != SW_ON)))
#error ('When LS_VHS_CAN_SPEED_SUBST is set to SW_ON, the follwing switches must als be set to SW_ON: WICRR, WICRL, VEH_SPEED')
#endif

#if ((GS_4WS_VEHICLE != SW_OFF) && (GS_VHS_REAR_AXLE_ACTUAL_SA != SW_ON))
#error ('GS_VHS_REAR_AXLE_ACTUAL_SA can be SW_OFF only if GS_4WS_VEHICLE is SW_OFF')
#endif

#if ((GS_4WS_VEHICLE != SW_OFF) && (GS_VHS_OUTPUT_REAR_AXLE_DESIRED_SA != SW_ON))
#error ('GS_VHS_OUTPUT_REAR_AXLE_DESIRED_SA can be SW_OFF only if GS_4WS_VEHICLE is SW_OFF')
#endif

gType_VhsDegModes_en g_VhsDegMode_en;

#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
// static UInt16 m_prevCANspeed_ui16;
// static UInt16 m_prevWICspeed_ui16;
// static UInt16 m_prevWheelspeed_ui16;
static UInt16 m_prevVehSpeed_ui16;
static gType_vhsWICbuf_st g_vhsWICRRbuf_st;
static gType_vhsWICbuf_st g_vhsWICRLbuf_st;
static gType_vhsWICbuf_st g_vhsWICFRbuf_st;
static gType_vhsWICbuf_st g_vhsWICFLbuf_st;
#endif // #if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         This init function is called once before the 10ms cyclic PA
 * call starts.
 * @details       This functions initializes all input signals with a default
 * value.
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_vhsInit_vd(void)
{
    // initialize project specific signals and degradation mode
    g_vhsApplInit_vd();

#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
    // m_prevCANspeed_ui16 = gd_VeloInvalid_ui16;
    // m_prevWICspeed_ui16 = gd_VeloInvalid_ui16;
    // m_prevWheelspeed_ui16 = gd_VeloInvalid_ui16;
    m_prevVehSpeed_ui16 = g_adiVehSpeed_ui16;

    g_InitVehicleSpeedCalcWICbuf_vd(&g_vhsWICRRbuf_st);
    g_InitVehicleSpeedCalcWICbuf_vd(&g_vhsWICRLbuf_st);
    g_InitVehicleSpeedCalcWICbuf_vd(&g_vhsWICFRbuf_st);
    g_InitVehicleSpeedCalcWICbuf_vd(&g_vhsWICFLbuf_st);
    g_InitVehicleSpeed_vd();
#endif

    return;
}

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Processing function for input signals, called once every 10ms
 * if PA is active.
 * @details       Is called every 10ms in the cyclic PA call.
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_vhsInputProcessing_vd(void)
{
#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
    UInt16 l_TEMPv_ui16;
    UInt8 l_NumberOfValidWICSpeeds_ui8;
    UInt16 l_vWICspeed_ui16;
    UInt8 l_NumberOfValidWheelSpeeds_ui8;
    UInt16 l_vWheelspeed_ui16;
    UInt16 l_vCANspeed_ui16;
    l_vWICspeed_ui16               = 0;
    l_NumberOfValidWICSpeeds_ui8   = 0;
    l_vWheelspeed_ui16             = 0;
    l_NumberOfValidWheelSpeeds_ui8 = 0;
#endif

    // Preconversion:
    // e.g. read signals that are used in other conversion functions for
    // consistant signal conversion
    g_VhsApplInputPreConversion_vd();

#if (GS_VHS_VEH_SPEED != SW_OFF)
    g_VehSpeedConversion_vd();
#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
    l_vCANspeed_ui16 = g_adiVehSpeed_ui16;
#endif
#endif
#if (GS_VHS_GEAR != SW_OFF)
    g_GearConversion_vd();
#endif
#if (GS_VHS_OUTSIDE_TEMP != SW_OFF)
    g_OutsideTempConversion_vd();
#endif
#if (GS_VHS_TRAILER_HITCH_PRESENT != SW_OFF)
    g_TrailerHitchPresentConversion_vd();
#endif
#if (GS_VHS_WHEEL_DIR_RR != SW_OFF)
    g_WheelDirRRConversion_vd();
#endif
#if (GS_VHS_WHEEL_DIR_RL != SW_OFF)
    g_WheelDirRLConversion_vd();
#endif
#if (GS_VHS_WHEEL_DIR_FR != SW_OFF)
    g_WheelDirFRConversion_vd();
#endif
#if (GS_VHS_WHEEL_DIR_FL != SW_OFF)
    g_WheelDirFLConversion_vd();
#endif
#if (GS_VHS_CAR_MOVE_DIR != SW_OFF)
    g_CarMoveDirConversion_vd();
#endif
#if (GS_VHS_SWA != SW_OFF)
    g_SWAConversion_vd();
#endif
#if (GS_VHS_INDICATOR != SW_OFF)
    g_IndicatorConversion_vd();
#endif
#if (GS_VHS_WICRR != SW_OFF)
    g_WICRRConversion_vd();
#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
    // calculate WIC speed for lowspeed (not supported on CAN signals)
    if (g_parGetParaVHO_Veh_DrivenAxle_ui8 != VHSAxleRear_enm)
    {
        l_TEMPv_ui16 =
            g_calculateWICspeed_ui16(g_adiWICRR_st.WIC_Signal_ui16,
                                     g_adiWICRR_st.WIC_Tick_ui16, &g_vhsWICRRbuf_st);
        if (l_TEMPv_ui16 != gd_VeloInvalid_ui16)
        {
            l_vWICspeed_ui16 += l_TEMPv_ui16;
            l_NumberOfValidWICSpeeds_ui8++;
        }
    }
#endif // #if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
#endif
#if (GS_VHS_WICRL != SW_OFF)
    g_WICRLConversion_vd();
#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
    // calculate WIC speed for lowspeed (not supported on CAN signals)
    if (g_parGetParaVHO_Veh_DrivenAxle_ui8 != VHSAxleRear_enm)
    {
        l_TEMPv_ui16 =
            g_calculateWICspeed_ui16(g_adiWICRL_st.WIC_Signal_ui16,
                                     g_adiWICRL_st.WIC_Tick_ui16, &g_vhsWICRLbuf_st);
        if (l_TEMPv_ui16 != gd_VeloInvalid_ui16)
        {
            l_vWICspeed_ui16 += l_TEMPv_ui16;
            l_NumberOfValidWICSpeeds_ui8++;
        }
    }
#endif // #if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
#endif
#if (GS_VHS_WICFR != SW_OFF)
    g_WICFRConversion_vd();
#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
    // calculate WIC speed for lowspeed (not supported on CAN signals)
    if (g_parGetParaVHO_Veh_DrivenAxle_ui8 != VHSAxleFront_enm)
    {
        l_TEMPv_ui16 =
            g_calculateWICspeed_ui16(g_adiWICFR_st.WIC_Signal_ui16,
                                     g_adiWICFR_st.WIC_Tick_ui16, &g_vhsWICFRbuf_st);
        if (l_TEMPv_ui16 != gd_VeloInvalid_ui16)
        {
            l_vWICspeed_ui16 += l_TEMPv_ui16;
            l_NumberOfValidWICSpeeds_ui8++;
        }
    }
#endif // #if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
#endif
#if (GS_VHS_WICFL != SW_OFF)
    g_WICFLConversion_vd();
#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
    // calculate WIC speed for lowspeed (not supported on CAN signals)
    if (g_parGetParaVHO_Veh_DrivenAxle_ui8 != VHSAxleFront_enm)
    {
        l_TEMPv_ui16 =
            g_calculateWICspeed_ui16(g_adiWICFL_st.WIC_Signal_ui16,
                                     g_adiWICRL_st.WIC_Tick_ui16, &g_vhsWICRLbuf_st);
        if (l_TEMPv_ui16 != gd_VeloInvalid_ui16)
        {
            l_vWICspeed_ui16 += l_TEMPv_ui16;
            l_NumberOfValidWICSpeeds_ui8++;
        }
    }
#endif // #if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
#endif
#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
    if (l_NumberOfValidWICSpeeds_ui8 > 0)
    {
        l_vWICspeed_ui16 /= l_NumberOfValidWICSpeeds_ui8;
    }
    else
    {
        l_vWICspeed_ui16 = gd_VeloInvalid_ui16;
    }
#endif // #if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)

#if (GS_VHS_WHEEL_ROTATION_RR != SW_OFF)
    g_WheelRotationRRConversion_vd();
#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
    // calculate Wheelspeed for lowspeed (not supported on CAN signals)
    if (g_parGetParaVHO_Veh_DrivenAxle_ui8 != VHSAxleRear_enm)
    {
        l_TEMPv_ui16 = (g_adiWheelRotationRR_ui16);
        if ((l_TEMPv_ui16 != gd_VeloInvalid_ui16) && (l_TEMPv_ui16 != 0))
        {
            l_vWheelspeed_ui16 += l_TEMPv_ui16;
            l_NumberOfValidWheelSpeeds_ui8++;
        }
    }
#endif
#endif
#if (GS_VHS_WHEEL_ROTATION_RL != SW_OFF)
    g_WheelRotationRLConversion_vd();
#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
    // calculate Wheelspeed for lowspeed (not supported on CAN signals)
    if (g_parGetParaVHO_Veh_DrivenAxle_ui8 != VHSAxleRear_enm)
    {
        l_TEMPv_ui16 = (g_adiWheelRotationRL_ui16);
        if ((l_TEMPv_ui16 != gd_VeloInvalid_ui16) && (l_TEMPv_ui16 != 0))
        {
            l_vWheelspeed_ui16 += l_TEMPv_ui16;
            l_NumberOfValidWheelSpeeds_ui8++;
        }
    }
#endif
#endif
#if (GS_VHS_WHEEL_ROTATION_FR != SW_OFF)
    g_WheelRotationFRConversion_vd();
#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
    // calculate Wheelspeed for lowspeed (not supported on CAN signals)
    if (g_parGetParaVHO_Veh_DrivenAxle_ui8 != VHSAxleFront_enm)
    {
        l_TEMPv_ui16 = (g_adiWheelRotationFR_ui16);
        if ((l_TEMPv_ui16 != gd_VeloInvalid_ui16) && (l_TEMPv_ui16 != 0))
        {
            l_vWheelspeed_ui16 += l_TEMPv_ui16;
            l_NumberOfValidWheelSpeeds_ui8++;
        }
    }
#endif
#endif
#if (GS_VHS_WHEEL_ROTATION_FL != SW_OFF)
    g_WheelRotationFLConversion_vd();
#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
    // calculate Wheelspeed for lowspeed (not supported on CAN signals)
    if (g_parGetParaVHO_Veh_DrivenAxle_ui8 != VHSAxleFront_enm)
    {
        l_TEMPv_ui16 = (g_adiWheelRotationFL_ui16);
        if ((l_TEMPv_ui16 != gd_VeloInvalid_ui16) && (l_TEMPv_ui16 != 0))
        {
            l_vWheelspeed_ui16 += l_TEMPv_ui16;
            l_NumberOfValidWheelSpeeds_ui8++;
        }
    }
#endif
#endif

#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
    if (l_NumberOfValidWheelSpeeds_ui8 > 0)
    {
        l_vWheelspeed_ui16 /= l_NumberOfValidWheelSpeeds_ui8;
    }
    else
    {
        l_vWheelspeed_ui16 = gd_VeloInvalid_ui16;
    }
#endif
#if (GS_VHS_LONG_ACC != SW_OFF)
    g_LongAccConversion_vd();
#endif
#if (GS_VHS_YAW_RATE != SW_OFF)
    g_YawRateConversion_vd();
#endif
#if (GS_VHS_EPS_STATUS != SW_OFF)
    g_EpsStatusConversion_vd();
#endif
    g_EscStatusConversion_vd();
#if (GS_VHS_ENGINE_START != SW_OFF)
    g_EngineStartConversion_vd();
#endif

#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
    g_adiVehSpeed_ui16 =
        g_CalcVehicleSpeed_ui16(l_vWICspeed_ui16,   // m_prevWICspeed_ui16,
                                l_vWheelspeed_ui16, // m_prevWhelspeed_ui16,
                                l_vCANspeed_ui16,   // m_prevCANspeed_ui16
                                m_prevVehSpeed_ui16);
    // store previous VehicleSpeed for further calculations

    // m_prevCANspeed_ui16 = l_vCANspeed_ui16;
    // m_prevWICspeed_ui16 =  l_vWICspeed_ui16;
    // m_prevWhelspeed_ui16 = l_vWheelspeed_ui16;
    m_prevVehSpeed_ui16 = g_adiVehSpeed_ui16;
#endif

    g_VhsApplInputPostConversion_vd();

    return;
}

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Processing function for output signals, called once every 10ms
 * if PA is active.
 * @details       Is called every 10ms in the cyclic PA call.
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_vhsOutputProcessing_vd(void) { return; }

/**
 * @brief          set vhs degrade mode.
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_vhsSetDegMode_vd(gType_VhsDegModes_en f_VhsDegMode_en)
{
    g_VhsDegMode_en = f_VhsDegMode_en;
    return;
}

/*========================= EoF (vhs.c) ========================*/
