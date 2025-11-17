/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: _appl_vho.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#define COMPONENT_VHO
#define COMPONENT_VHOTCE
#define COMPONENT_VHOSRC

/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/
// #include <swcpa_include.h>
// #include <swcpa_api.h>

#include <pa_components/base_type/global_include.h>

#include "vho.h"
#include "vho_api.h"
#include <pa_components/mathlib/mathlib_api.h>
#include <veh/vhs/vhs_api.h>

#include <string.h>

// #include <mai_api.h>

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if ((GS_VHOTCE_EXTERNAL_TIRE_CRCUMFERENCE_ESTIMATION == SW_ON) || \
     (GS_VHOTCE_EXTERNAL_TIRE_TOLERANCE_CALIBRATION == SW_ON))
#include <appl_vho_plugin_tce.h>
#endif

#if (LS_VHO_EXT_COORDINATE_RESET == SW_ON)
// #include "uss/psd/psd_api.h"
#endif

#if (GS_VHOSRC_SLIP_RECOGNITION == SW_ON)
#include <appl_vho_plugin_src.h>
#endif

#endif // (GS_VHM_RUN_MODE == GS_VHM_CONSIDER_USS_DATA)

#ifndef VISIBILITY
#error ('compiler switch VISIBILITY is not defined')
#endif

#ifndef GS_VHS_VEH_SPEED
#error ('compiler switch GS_VHS_VEH_SPEED is not defined')
#endif
#ifndef GS_VHS_GEAR
#error ('compiler switch GS_VHS_GEAR is not defined')
#endif
#ifndef GS_VHS_SWA
#error ('compiler switch GS_VHS_SWA is not defined')
#endif
#ifndef GS_VHS_WICRR
#error ('compiler switch GS_VHS_WICRR is not defined')
#endif
#ifndef GS_VHS_WICRL
#error ('compiler switch GS_VHS_WICRL is not defined')
#endif
#ifndef GS_VHS_WICFR
#error ('compiler switch GS_VHS_WICFR is not defined')
#endif
#ifndef GS_VHS_WICFL
#error ('compiler switch GS_VHS_WICFL is not defined')
#endif
#ifndef GS_VHS_WHEEL_DIR_RR
#error ('compiler switch GS_VHS_WHEEL_DIR_RR is not defined')
#endif
#ifndef GS_VHS_WHEEL_DIR_RL
#error ('compiler switch GS_VHS_WHEEL_DIR_RL is not defined')
#endif
#ifndef GS_VHS_WHEEL_DIR_FR
#error ('compiler switch GS_VHS_WHEEL_DIR_FR is not defined')
#endif
#ifndef GS_VHS_WHEEL_DIR_FL
#error ('compiler switch GS_VHS_WHEEL_DIR_FL is not defined')
#endif
#ifndef GS_VHS_WHEEL_ROTATION_RR
#error ('compiler switch GS_VHS_WHEEL_ROTATION_RR is not defined')
#endif
#ifndef GS_VHS_WHEEL_ROTATION_RL
#error ('compiler switch GS_VHS_WHEEL_ROTATION_RL is not defined')
#endif
#ifndef GS_VHS_WHEEL_ROTATION_FR
#error ('compiler switch GS_VHS_WHEEL_ROTATION_FR is not defined')
#endif
#ifndef GS_VHS_WHEEL_ROTATION_FL
#error ('compiler switch GS_VHS_WHEEL_ROTATION_FL is not defined')
#endif
#ifndef GS_VHS_LONG_ACC
#error ('compiler switch GS_VHS_LONG_ACC is not defined')
#endif
#ifndef GS_VHS_YAW_RATE
#error ('compiler switch GS_VHS_YAW_RATE is not defined')
#endif

#define APPL_VHO_C_TEMPL_NUMBER 0x140325
/*----------------------------------------------------------------*/

#ifndef APPL_VHO_H_REF
#error "Compiler switch APPL_VHO_H_REF not defined!"
#endif
#ifndef APPL_VHO_C_REF
#error "Compiler switch APPL_VHO_C_REF not defined!"
#endif

#if (APPL_VHO_H_TEMPL_NUMBER != APPL_VHO_H_REF)
#error ('Wrong version of H TEMPLATE FILE - check whether the template has changed')
#endif
#if (APPL_VHO_C_TEMPL_NUMBER != APPL_VHO_C_REF)
#error ('Wrong version of C TEMPLATE FILE - check whether the template has changed')
#endif

// #define VHO_WHEEL_VELOCITIES_REAR_AVAILABLE
// #define VHO_WHEEL_VELOCITIES_FRONT_AVAILABLE

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3892
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if (LS_VHO_EXT_COORDINATE_RESET == SW_ON)
typedef struct
{
    UInt8 StartReset_ui8;  // initiate reset by writing a value != 0
    SInt32 XPosition_si32; // x-position [mm]
    SInt32 YPosition_si32; // y-position [mm]
    UInt32 YawAngle_ui32;  // orientation [1/2^22 rad]
} mType_VHOExtCoordinateState_st;
#endif

#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if (LS_VHO_EXT_COORDINATE_RESET == SW_ON)
static mType_VHOExtCoordinateState_st m_VHOExtCoordinateState_st;
#endif

VISIBILITY gType_EEPROMState_en m_EEPROMState_en;
VISIBILITY gType_EEPROMOperation_en m_lastEEPROMRequest_en;
VISIBILITY gType_EE_RequestMode_en m_EE_RequestMode_en;
VISIBILITY gType_NvM_RequestResultType_en m_EepRequestResult_en;

#if (GS_VHOSRC_SLIP_RECOGNITION == SW_ON)
static gType_VhoMonitoringMode_en m_vhoMonitoringMode_en;
#endif

#endif

gType_VHOParaAutocalib_st AutocalibStateEEPROM_st;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if (LS_VHO_EXT_COORDINATE_RESET == SW_ON)
VISIBILITY void m_VHOExtResetProcessing_vd(void);
#endif

#if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC == SW_ON) || \
     (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON) ||                                    \
     (GS_VHOTCE_EXTERNAL_TIRE_CRCUMFERENCE_ESTIMATION == SW_ON) ||                \
     (GS_VHOTCE_EXTERNAL_TIRE_TOLERANCE_CALIBRATION == SW_ON))
VISIBILITY void m_VHOProcessPendingEepromRequest_vd(void);
static void m_vhoEEPROMRequestAppl_vd(gType_EEPROMOperation_en f_EEPROMOperation_en);
#endif

#endif

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N     D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         project specific pre init steps that are executed when
 *                g_vhoPreInit_vd is called
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_vhoPreInitAppl_vd(void) {}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N     D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         project specific init steps that are executed when
 *                g_vhoReinit_vd is called
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_vhoReInitAppl_vd(void) {}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N     D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief          project specific init steps that are executed when
 *                 g_vhoInit_vd is called
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_vhoInitAppl_vd(void)
{
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

    // m_EEPROMState_en = EEPROMIdle_enm;
    // m_lastEEPROMRequest_en = EEPROMRead_enm;
    // m_EepRequestResult_en = NVM_REQ_OK_enm;
    // read could only be performed after init; once per ignition cycle!!!!!
    // m_EE_RequestMode_en = m_vho_EE_RequestMode_ReadAutocalibData_enm;

    /* The following Lines are commented-out (not required for Release 5.12):
             ======================================================================
    // Special Configuration for BMW 3er (Adaption to Release 5.8 BMW Targets)
    #if(GS_VEHICLE_TYPE == VEHICLE_BMW_3)
      g_VHOOdoState_st.ExternalCtrl = TRUE;
            g_VHOOdoState_st.WicSource_en = VHOWicSourceTwoWICFront_enm;
            if(g_VHOAutocalibCtrl_st.TTCState_enm == VHOCalibState_STABLE_enm)
            {
                    g_VHOOdoState_st.YawAngCalc_en =
    VHOYAngCalcSingleXDualTrFront_enm;
            }
            else
            {
                    g_VHOOdoState_st.YawAngCalc_en = VHOYAngCalcSingleTrack_enm;
            }
    #endif  //#if(GS_VEHICLE_TYPE == VEHICLE_BMW_3)
    */

#if ((GS_VHOTCE_EXTERNAL_TIRE_CRCUMFERENCE_ESTIMATION == SW_ON) || \
     (GS_VHOTCE_EXTERNAL_TIRE_TOLERANCE_CALIBRATION == SW_ON))
    g_vho_plugin_tce_Init_vd();
#endif

#if (GS_VHOSRC_SLIP_RECOGNITION == SW_ON)
    g_vho_plugin_sliprecog_Init_vd();
#endif

#endif
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N     D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         returns TRUE if PSX system is not running => VHO
 * autocalibration will execute
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
Boolean g_VHPSMxIdle_bl(void) { return TRUE; }

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N     D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         function that enables to perform project specific actions
 * before the execution of one call cycle
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_vhoProcessingPreAppl_vd(void)
{
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if (LS_VHO_EXT_COORDINATE_RESET == SW_ON)
    // handle external coordinate reset
    m_VHOExtResetProcessing_vd();
#endif

#if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC == SW_ON) || \
     (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON) ||                                    \
     (GS_VHOTCE_EXTERNAL_TIRE_CRCUMFERENCE_ESTIMATION == SW_ON) ||                \
     (GS_VHOTCE_EXTERNAL_TIRE_TOLERANCE_CALIBRATION == SW_ON))
    m_VHOProcessPendingEepromRequest_vd();
#endif

#endif
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N     D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         Initialization of the VHO parameters
 * @details
 *
 * @param         f_VHOParameters_pst      Pointer to the VHO parameter struct
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

void g_vhoInitParamsAppl_vd(gType_VHOParameters_st *f_VHOParameters_pst)
{

#if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS)
    f_VHOParameters_pst->RollRecogMode_en = VHO_Direction_Accsens_enm;
#elif (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_GEAR)
    f_VHOParameters_pst->RollRecogMode_en = VHO_Direction_Gear_enm;
#elif (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC)
    f_VHOParameters_pst->RollRecogMode_en = VHO_Direction_Wic_enm;
#elif (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC)
    f_VHOParameters_pst->RollRecogMode_en = VHO_Direction_Yaw_Wic_enm;
#elif (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)
    if ((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
        VHO_Direction_Gear_enm)
    {
        f_VHOParameters_pst->RollRecogMode_en = VHO_Direction_Gear_enm;
    }
    else if ((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
             VHO_Direction_Wic_enm)
    {
        f_VHOParameters_pst->RollRecogMode_en = VHO_Direction_Wic_enm;
    }
    else if ((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
             VHO_Direction_Yaw_Wic_enm)
    {
        f_VHOParameters_pst->RollRecogMode_en = VHO_Direction_Yaw_Wic_enm;
    }
    else
    {
        // default:
        f_VHOParameters_pst->RollRecogMode_en = VHO_Direction_Gear_enm;
    }
#else
#error ('Setting for LS_VHO_DIRDETECT_MODE not valid!')
#endif

#if (LS_VHO_SPP_WIC == SW_ON)
    f_VHOParameters_pst->WICPreprocActive_bl = TRUE;
#else
    f_VHOParameters_pst->WICPreprocActive_bl = FALSE;
#endif

    m_vhoGetPARPointer_vd(&g_VHOParameters_st);
}

#endif

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N     D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         function that enables to perform project specific actions
 * after the execution of one call cycle
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_vhoProcessingPostAppl_vd(void)
{
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

    if (g_parIsParaUpdate_bl(0) != FALSE)
    {
        m_vhoGetPARPointer_vd(&g_VHOParameters_st);
    }

#if ((GS_VHOTCE_EXTERNAL_TIRE_CRCUMFERENCE_ESTIMATION == SW_ON) || \
     (GS_VHOTCE_EXTERNAL_TIRE_TOLERANCE_CALIBRATION == SW_ON))
    g_vho_plugin_tce_vd();
#endif

#endif
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N     D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief
 * @details       read input signals from rte
 *
 * @param         f_VHOCanSig_pst
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_VHOGetSignalsAppl_vd(gType_VHOCanSig_st *f_VHOCanSig_pst)
{
// Time Stamp Correction for RECAM:
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if (LS_VHO_EXT_COORDINATE_RESET == SW_ON)
    gType_WICandTime_st l_WIC_RL_st;

    g_adiGetWICRL_bl(&l_WIC_RL_st);

    if (l_WIC_RL_st.WIC_Tick_ui16 == f_VHOCanSig_pst->WIC_CanTime_ui16)
    {
        // Correct WIC Time Stamp (-3 ms) according to Signal Delay analyzed
        // during RECAM Task Force Days (see Report of J. Wild):
        f_VHOCanSig_pst->WIC_CanTime_ui16 -= (UInt16)3;
    }
#endif

// only executed if VHO_RUN_MODE != MINI_VHO
#if (GS_VHS_WHEEL_ROTATION_RR == SW_ON)
    {
        UInt16 l_WhlVelo_ui16;
        // read wheel velocities
        (void)g_adiGetWheelRotationRR_bl(&l_WhlVelo_ui16);
        f_VHOCanSig_pst->WheelRev_pui16[RightRear_enm] = l_WhlVelo_ui16;
    }
#endif

#if (GS_VHS_WHEEL_ROTATION_RL == SW_ON)
    {
        UInt16 l_WhlVelo_ui16;
        (void)g_adiGetWheelRotationRL_bl(&l_WhlVelo_ui16);
        f_VHOCanSig_pst->WheelRev_pui16[LeftRear_enm] = l_WhlVelo_ui16;
    }
#endif

#if (GS_VHS_WHEEL_ROTATION_FR == SW_ON)
    {
        UInt16 l_WhlVelo_ui16;

        // read wheel velocities
        (void)g_adiGetWheelRotationFR_bl(&l_WhlVelo_ui16);
        f_VHOCanSig_pst->WheelRev_pui16[RightFront_enm] = l_WhlVelo_ui16;
    }
#endif

#if (GS_VHS_WHEEL_ROTATION_FL == SW_ON)
    {
        UInt16 l_WhlVelo_ui16;
        (void)g_adiGetWheelRotationFL_bl(&l_WhlVelo_ui16);
        f_VHOCanSig_pst->WheelRev_pui16[LeftFront_enm] = l_WhlVelo_ui16;
    }
#endif

#if (GS_VHS_LONG_ACC == SW_ON)
    {
        SInt16 l_A_si16;

        f_VHOCanSig_pst->Acc_ValidFlag_bl = g_adiGetLongAcc_bl(&l_A_si16);
        f_VHOCanSig_pst->Acc_si16         = l_A_si16;
    }
#endif

#if (GS_VHOSRC_SLIP_RECOGNITION == SW_ON)
    g_vho_plugin_sliprecog_vd(f_VHOCanSig_pst);
#endif

#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

    return;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N     D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief
 * @details       External Coordinate Reset-Processing
 *                extends application by a function to reset the
 *                VHO and PSD on command to a given position by an external tool
 *                (e.g. CANape or debugger)
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if (LS_VHO_EXT_COORDINATE_RESET == SW_ON)

VISIBILITY void m_VHOExtResetProcessing_vd(void)
{
    // check external reset
    if (m_VHOExtCoordinateState_st.StartReset_ui8 > (UInt8)0)
    {
        // init vho
        g_vhoReInit_vd(m_VHOExtCoordinateState_st.XPosition_si32,
                       m_VHOExtCoordinateState_st.YPosition_si32,
                       m_VHOExtCoordinateState_st.YawAngle_ui32,
                       1); // reset driven distance

        // init psd
        {
// Set PSD off and reset to active Mode to reset the PSD asynchronously:
// In Release 5.6, PSD2 shall be used together with POC:
#if (GS_PP_FUNC_POC == SW_ON)
            // PSD2 Operating Modes for Parallel Parking Spaces:
            g_psdSetOperatingMode_vd(OpMode_PSD_MODE_OFF_enm);
            g_psdSetOperatingMode_vd(OpMode_PSD_MODE_PARALLEL_enm);
#else
#if (GS_PP_FUNC_CPSC == SW_OFF)
            // PSD1 Operating Modes for Parallel Parking Spaces:
            g_psdSetOperatingMode_vd(PSD_MODE_OFF_enm);
            g_psdSetOperatingMode_vd(PSD_MODE_BOTH_enm);
#else
            // PSD2 Operating Modes for Cross Parking Spaces:
            // g_psdSetOperatingMode_vd(OpMode_PSD_MODE_OFF_enm);
            // g_psdSetOperatingMode_vd(OpMode_PSD_MODE_CROSS_enm);
#endif
#endif
        }

        // reset external request
        m_VHOExtCoordinateState_st.StartReset_ui8 = (UInt8)0;
    }
}

#endif // LS_VHO_EXT_COORDINATE_RESET

#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         handler of EEPROM action
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC == SW_ON) || \
     (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON) ||                                    \
     (GS_VHOTCE_EXTERNAL_TIRE_CRCUMFERENCE_ESTIMATION == SW_ON) ||                \
     (GS_VHOTCE_EXTERNAL_TIRE_TOLERANCE_CALIBRATION == SW_ON))

VISIBILITY void m_VHOProcessPendingEepromRequest_vd(void)
{
    if (m_EE_RequestMode_en == m_vho_EE_RequestMode_ReadAutocalibData_enm)
    {
        // call EEPROM
        m_vhoEEPROMRequestAppl_vd(EEPROMRead_enm);

        if (m_EEPROMState_en == EEPROMIdle_enm)
        {
            if (m_EepRequestResult_en == NVM_REQ_OK_enm)
            {
                m_EE_RequestMode_en = m_vho_EE_RequestMode_None_enm;
                g_InitAutocalibFromEEPROM_ui8(&g_VHOParameters_st.AutocalibData_st);
            }
            else
            {
                m_EE_RequestMode_en = m_vho_EE_RequestMode_None_enm;
                // set autocalibration to default, because EEPROM is busy or whatever
                g_VHOSignalPreprocInit_vd();
            }
        }
        else
        {
            // EEPROM busy, ...
            // set autocalibration to default, because EEPROM is busy or whatever
            g_VHOSignalPreprocInit_vd();
        }
    }
    else if (m_EE_RequestMode_en == m_vho_EE_RequestMode_WriteAutocalibData_enm)
    {
        // call EEPROM
        m_vhoEEPROMRequestAppl_vd(EEPROMWrite_enm);

        if (m_EEPROMState_en == EEPROMIdle_enm)
        {
            m_EE_RequestMode_en = m_vho_EE_RequestMode_None_enm;
        }
        else
        {
            // nothing to do --> EEPROM busy, ...
        }
    }
    else
    {
        // requested mode is none --> nothing to do
    }
    return;
}

#endif /*    #if ( (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) ||                 \
                   (LS_VHO_AUTOCALIB_TTC == SW_ON) ||                        \
                         (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON) ||           \
                         (GS_VHOTCE_EXTERNAL_TIRE_CRCUMFERENCE_ESTIMATION == \
          SW_ON) ||                                                          \
                         (GS_VHOTCE_EXTERNAL_TIRE_TOLERANCE_CALIBRATION ==   \
          SW_ON) ) */

#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         set EEPROM request mode --> called from vho algorithm
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC == SW_ON) || \
     (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON) ||                                    \
     (GS_VHOTCE_EXTERNAL_TIRE_CRCUMFERENCE_ESTIMATION == SW_ON) ||                \
     (GS_VHOTCE_EXTERNAL_TIRE_TOLERANCE_CALIBRATION == SW_ON))

void m_VHOSetEepromRequestModeToWrite_vd(void)
{
    m_EE_RequestMode_en = m_vho_EE_RequestMode_WriteAutocalibData_enm;
}

#endif /*    #if ( (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) ||                 \
                   (LS_VHO_AUTOCALIB_TTC == SW_ON) ||                        \
                         (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON) ||           \
                         (GS_VHOTCE_EXTERNAL_TIRE_CRCUMFERENCE_ESTIMATION == \
          SW_ON) ||                                                          \
                         (GS_VHOTCE_EXTERNAL_TIRE_TOLERANCE_CALIBRATION ==   \
          SW_ON) )  */

/*
        reactivate suppressed QAC warning message 3892
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3892
#endif

#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief
 * @details
 *
 * @param         f_EEPROMOperation_en
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC == SW_ON) || \
     (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON) ||                                    \
     (GS_VHOTCE_EXTERNAL_TIRE_CRCUMFERENCE_ESTIMATION == SW_ON) ||                \
     (GS_VHOTCE_EXTERNAL_TIRE_TOLERANCE_CALIBRATION == SW_ON))

/* MISRA-C 2004 RULE 11.4; REFERENCE - ISO:C90-6.3.4 Cast Operators */
/* Details of the deviation:                    Return values E_OK and E_NOT_OK
 * are type casted to void */
/* Circumstances for the need of the deviation: The standard AUTOSAR return
 * values E_OK and E_NOT_OK are not being used */
/* Potential consequences:                      Must rely on error status values
 */
/* Justification for the deviation:             NvM error status is used instead
 * of E_OK and E_NOT_OK return values */
/* Demonstration how safety is assured:         All read and write requests are
   followed by a call to g_vhoGetErrorStatusVhoCalibData_vd to obtain EEPROM
   status; Casted return values are neither assigned nor used.
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 0310
#endif
static void m_vhoEEPROMRequestAppl_vd(gType_EEPROMOperation_en f_EEPROMOperation_en)
{
    // If EEPROM is Idle
    if (EEPROMIdle_enm == m_EEPROMState_en)
    {
        // Determine requested EEPROM operation
        switch (f_EEPROMOperation_en)
        {
                // Requested operation is EEPROMRead
            case EEPROMRead_enm:
                // Read EEPROM block for VHO calibration data
                // (void)g_vhoReadBlockVhoCalibData_en(&g_VHOParameters_st.AutocalibData_st);
                break;

            // Requested operation is EEPROMWrite
            case EEPROMWrite_enm:
                // Write EEPROM block for VHO calibration data
                // (void)g_vhoWriteBlockVhoCalibData_en(&g_VHOParameters_st.AutocalibData_st);
                break;

            // Unknown request
            default:
                // Do nothing
                break;
        }

        // Set EEPROM State
        m_EEPROMState_en = EEPROMBusy_enm;
        // Store requested operation
        m_lastEEPROMRequest_en = f_EEPROMOperation_en;
        // Determine EEPROM status for immediately preceeding operation

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3417, 3426
#endif
        // g_vhoGetErrorStatusVhoCalibData_vd(&m_EepRequestResult_en);
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3417, 3426
#endif
    }
    // If EEPROM is Idle
    else
    // if (EEPROMBusy_enm == m_EEPROMState_en)
    {
        // Determine EEPROM status
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3417, 3426
#endif
        // g_vhoGetErrorStatusVhoCalibData_vd(&m_EepRequestResult_en);
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3417, 3426
#endif
    } // if (EEPROMBusy_enm == m_EEPROMState_en)

    // Determine EEPROM status
    switch (m_EepRequestResult_en)
    {
            // EEPROM status is 'Ok'
        case NVM_REQ_OK_enm:
            // Previous request successfully completed, change EEPROM state to Idle
            m_EEPROMState_en = EEPROMIdle_enm;
            break;

        // EEPROM status is 'Pending'
        case NVM_REQ_PENDING_enm:
            // Previous request is being processed, change EEPROM state to Busy
            m_EEPROMState_en = EEPROMBusy_enm;
            break;

        // Any other EEPROM status is likely to be an error
        default:
            // Determine which operation was requested - Read OR Write
            switch (m_lastEEPROMRequest_en)
            {
                // If requested operation was EEPROMRead
                case EEPROMRead_enm:
                    // Log EEPROM read error
                    g_vhoLogHdl_vd(VHOLog_EEPROMRead_enm, g_vhoLogStateOn_enm);
                    break;

                // If requested operation was EEPROMWrite
                case EEPROMWrite_enm:
                    // Log EEPROM write error
                    g_vhoLogHdl_vd(VHOLog_EEPROMWrite_enm, g_vhoLogStateOn_enm);
                    break;

                // Unknown request
                default:
                    // Do nothing
                    break;
            }

            // Set EEPROM status to Idle to enable requests
            m_EEPROMState_en = EEPROMIdle_enm;
            break;
    }

    return;
}

/*
        reactivate supressed QAC warning message 0310
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 0310
#endif

#endif /*    #if ( (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) ||                 \
                   (LS_VHO_AUTOCALIB_TTC == SW_ON) ||                        \
                         (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON) ||           \
                         (GS_VHOTCE_EXTERNAL_TIRE_CRCUMFERENCE_ESTIMATION == \
          SW_ON) ||                                                          \
                         (GS_VHOTCE_EXTERNAL_TIRE_TOLERANCE_CALIBRATION ==   \
          SW_ON) )  */

#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Initialize pointers to parameter clusters
 * @details
 *
 * @param         f_VHOParameters_pst
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

void m_vhoGetPARPointer_vd(gType_VHOParameters_st *f_VHOParameters_pst)
{
    f_VHOParameters_pst->VHO_Veh_pst = g_parGetParaReadAccess_pst(VHO_Veh);
    f_VHOParameters_pst->VHO_Odo_pst = g_parGetParaReadAccess_pst(VHO_Odo);
}

#endif

/****************************************************************************
                                Interface for VHS to get PSX time stamp
                                (synchronous to g_VHOVehicleState_st,
                                refer to
g_VHOUSSProcessingGetEstimatedPosition_vd)
****************************************************************************/
#if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) || \
     ((GS_VHO_RUN_MODE == GS_VHO_MINI_VHO) &&         \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC)))

void g_adiVhoGetPsxTimestamp_vd(UInt16 *f_WicTime_pui16)
{
    *f_WicTime_pui16 = g_VHOCanSig_st.WIC_CanTime_ui16;
    return;
}

#endif

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Allows override of internal TCC and WICLength estimtion
 * @details
 *
 * @param         f_TyreCircumferenceMMF8_pui32
 * @return        Boolean
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

Boolean m_VHOPluginSetExternalCircumference_bl(UInt32 *f_TyreCircumferenceMMF8_pui32)
{
    // f_TyreCircumferenceMMF8_pui32[LeftRear_enm] =
    // f_TyreCircumferenceMMF8_pui32[RightRear_enm] =
    // f_TyreCircumferenceMMF8_pui32[LeftFront_enm] =
    // f_TyreCircumferenceMMF8_pui32[RightFront_enm] =
#if ((GS_VHOTCE_EXTERNAL_TIRE_CRCUMFERENCE_ESTIMATION == SW_ON) || \
     (GS_VHOTCE_EXTERNAL_TIRE_TOLERANCE_CALIBRATION == SW_ON))

    UInt16 l_WICLengthMMF8_pui16[4];
    Boolean l_Return_bl;
    gType_VHOAutocalibCtrl_st l_AutocalibCtrl_st;
    UInt16 l_AvgWicLengthRearAxle_ui16;

#if (GS_VHOTCE_EXTERNAL_TIRE_CRCUMFERENCE_ESTIMATION == SW_ON)
    {
        g_vho_plugin_getWIClength_bl(&l_AutocalibCtrl_st);

        if (l_AutocalibCtrl_st.TyreRadState_enm == VHOCalibState_STABLE_enm)
        {
            l_AvgWicLengthRearAxle_ui16 = l_AutocalibCtrl_st.AvgWICLengthRearAxle_ui16;
        }
        else
        {
            l_AvgWicLengthRearAxle_ui16 = g_VHOParameters_st.VHO_Veh_pst->WICLength_ui16;
        }
    }
#else
    l_AvgWicLengthRearAxle_ui16 = g_VHOParameters_st.VHO_Veh_pst->WICLength_ui16;
#endif

#if (GS_VHOTCE_EXTERNAL_TIRE_TOLERANCE_CALIBRATION == SW_ON)
    g_vho_plugin_getRTAoutputs_bl(&l_AutocalibCtrl_st);
#else
    l_AutocalibCtrl_st = g_VHOAutocalibCtrl_st;
#endif

    if (l_AutocalibCtrl_st.TTCState_enm == VHOCalibState_STABLE_enm)
    {
        l_WICLengthMMF8_pui16[LeftRear_enm] =
            (UInt16)((
                         // wic length is assumed to be smaller than 6cm =>
                         // l_AvgWicLengthRearAxle_ui16 < 15360 < 41943 = 2^32/102400
                         (UInt32)102400 * (UInt32)l_AvgWicLengthRearAxle_ui16) /
                     ((UInt32)51200 + (UInt32)l_AutocalibCtrl_st.ScalFacRR_ui16));

        // get wic length of other three tyres by scaling the wic length
        // of the reference wheel with the factor generated by ttc;
        // the scaling facor is itself scaled with 51200, so the result value
        // is calculated with
        //
        //         wl_lr * scal_fac_x
        //  wl_x = ------------------
        //         51200

        l_WICLengthMMF8_pui16[RightRear_enm] =
            (UInt16)(((UInt32)l_WICLengthMMF8_pui16[LeftRear_enm] *
                      (UInt32)l_AutocalibCtrl_st.ScalFacRR_ui16) /
                     (UInt32)51200);

        l_WICLengthMMF8_pui16[LeftFront_enm] =
            (UInt16)(((UInt32)l_WICLengthMMF8_pui16[LeftRear_enm] *
                      (UInt32)l_AutocalibCtrl_st.ScalFacLF_ui16) /
                     (UInt32)51200);

        l_WICLengthMMF8_pui16[RightFront_enm] =
            (UInt16)(((UInt32)l_WICLengthMMF8_pui16[LeftRear_enm] *
                      (UInt32)l_AutocalibCtrl_st.ScalFacRF_ui16) /
                     (UInt32)51200);

        f_TyreCircumferenceMMF8_pui32[LeftRear_enm] =
            (UInt32)l_WICLengthMMF8_pui16[LeftRear_enm] *
            (UInt32)g_parGetParaVHO_Veh_WICTeethCount_ui16;
        f_TyreCircumferenceMMF8_pui32[RightRear_enm] =
            (UInt32)l_WICLengthMMF8_pui16[RightRear_enm] *
            (UInt32)g_parGetParaVHO_Veh_WICTeethCount_ui16;
        f_TyreCircumferenceMMF8_pui32[LeftFront_enm] =
            (UInt32)l_WICLengthMMF8_pui16[LeftFront_enm] *
            (UInt32)g_parGetParaVHO_Veh_WICTeethCount_ui16;
        f_TyreCircumferenceMMF8_pui32[RightFront_enm] =
            (UInt32)l_WICLengthMMF8_pui16[RightFront_enm] *
            (UInt32)g_parGetParaVHO_Veh_WICTeethCount_ui16;
        l_Return_bl = TRUE;
    }
    else
    {
        l_Return_bl = FALSE;
    }

    return l_Return_bl;

    /*  #if ( (GS_VHOTCE_EXTERNAL_TIRE_CRCUMFERENCE_ESTIMATION == SW_ON) || \
              (GS_VHOTCE_EXTERNAL_TIRE_TOLERANCE_CALIBRATION == SW_ON) ) */
#else
    return FALSE;
#endif
}

#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         Sets nad gets the vho monitoring mode
 * @details
 *
 * @param         f_VhoMonitoringMode_en
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (GS_VHOSRC_SLIP_RECOGNITION == SW_ON)

void g_vhoSetVHOMonitoringMode_vd(gType_VhoMonitoringMode_en f_VhoMonitoringMode_en)
{
    m_vhoMonitoringMode_en = f_VhoMonitoringMode_en;
}

gType_VhoMonitoringMode_en g_vhoGetVHOMonitoringMode_vd(void)
{
    return m_vhoMonitoringMode_en;
}

#endif // #if (GS_VHOSRC_SLIP_RECOGNITION == SW_ON)
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
