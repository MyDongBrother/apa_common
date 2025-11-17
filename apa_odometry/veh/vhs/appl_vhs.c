/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: appl_vhs.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#define COMPONENT_VHS

#define APPL_VHS_C_TEMPL_NUMBER 0x110723

#include <pa_components/base_type/global_include.h>

// #include <sp_fsv_api.h> // must be defined befor vhs_api.h (only required
//  within COMPONENT_VHS)
#ifdef md_RUN_ON_POSIX
#include <pa_components/debugprintf/debug_log.h>
#include <sys/time.h>
#endif // md_RUN_ON_POSIX

#include <veh/sig_if/appl_canSignal.h>

#include "vhs_api.h"
#include "data_io/dataInput_adi.h"

#if (GS_PP_FUNC_PS != SW_OFF)
//  #include <ap_ps_api.h>
#endif

// #include "global.h"
#if (APPL_VHS_H_TEMPL_NUMBER != APPL_VHS_H_REF)
#error ('Wrong version of H TEMPLATE FILE - check whether the template has changed')
#endif

#if (APPL_VHS_C_TEMPL_NUMBER != APPL_VHS_C_REF)
#error ('Wrong version of C TEMPLATE FILE - check whether the template has changed')
#endif
#ifndef LS_VHS_SIGNAL_SIMULATION
#error ('switch LS_VHS_SIGNAL_SIMULATION is not defined')
#endif
#ifndef LS_VHS_RAWSIGNAL_DEBUG
#error ('switch LS_VHS_RAWSIGNAL_DEBUG is not defined')
#endif

#if ((GS_VHS_WICFL != SW_OFF) || (GS_VHS_WICFR != SW_OFF) || (GS_VHS_WICRL != SW_OFF) || \
     (GS_VHS_WICRR != SW_OFF) || (GS_VHS_SWA != SW_OFF))
typedef struct mT_TickTmp_st
{
    UInt16 WIC_FL_Tick_ui16; // Temporary Timestamp signal generated during interrupt
    // | Wheel Impulse Counter tick Signal  |      signal range      |
    // | Resolution (1Bit)                  | Min value | Max value  |
    // | 1 system tick (derived from OS)    | 0         | 65535      |

    UInt16 WIC_FR_Tick_ui16; // Temporary Timestamp signal generated during interrupt
    // | Wheel Impulse Counter tick Signal  |      signal range      |
    // | Resolution (1Bit)                  | Min value | Max value  |
    // | 1 system tick (derived from OS)    | 0         | 65535      |

    UInt16 WIC_RL_Tick_ui16; // Temporary Timestamp signal generated during interrupt
    // | Wheel Impulse Counter tick Signal  |      signal range      |
    // | Resolution (1Bit)                  | Min value | Max value  |
    // | 1 system tick (derived from OS)    | 0         | 65535      |

    UInt16 WIC_RR_Tick_ui16; // Temporary Timestamp signal generated during interrupt
    // | Wheel Impulse Counter tick Signal  |      signal range      |
    // | Resolution (1Bit)                  | Min value | Max value  |
    // | 1 system tick (derived from OS)    | 0         | 65535      |

    UInt16 EPSSAS_Tick_ui16; // Temporary Timestamp signal generated during interrupt
    // | EPS SAS tick Signal                |      signal range      |
    // | Resolution (1Bit)                  | Min value | Max value  |
    // | 1 system tick (derived from OS)    | 0         | 65535      |

    UInt16 SAS_Tick_ui16; // Final Timestamp copied after normalization to Final
                          // value
    // | Steering Angle Sensor tick Signal  |      signal range      |
    // | Resolution (1Bit)                  | Min value | Max value  |
    // | 1 system tick (derived from OS)    | 0         | 65535      |
} mType_TickTmp_st;
#endif

#if ((GS_VHS_WICFL != SW_OFF) || (GS_VHS_WICFR != SW_OFF) || (GS_VHS_WICRL != SW_OFF) || \
     (GS_VHS_WICRR != SW_OFF) || (GS_VHS_SWA != SW_OFF))
static mType_TickTmp_st m_TickTmp_st;
#endif

/*!@}*/ /*-------------------------------------------------------------------*/
/*-                       definition of local functions                     */
/*!\cond INTERNAL*/ /*-------------------------------------------------------*/

#if (GS_VHS_WICFL == SW_ON)
extern void g_SetFLWICTick_vd(void);
#endif
#if (GS_VHS_WICFR == SW_ON)
extern void g_SetFRWICTick_vd(void);
#endif
#if (GS_VHS_WICRL == SW_ON)
extern void g_SetRLWICTick_vd(void);
#endif
#if (GS_VHS_WICRR == SW_ON)
extern void g_SetRRWICTick_vd(void);
#endif

static gType_Gear_en m_VhsApplFilterGearTransition_en(gType_Gear_en f_NewGear_en);
static gType_WheelDir_en m_ConvertMoveDirCOM2RTE_en(UInt8 f_MoveDirCOM_ui8);

#if (GS_VHS_VEH_SPEED != SW_OFF)

UInt16 g_adiVehSpeed_ui16;

static const UInt16 gc_DefaultVehSpeed_ui16 = 0; // Default AND initialization value
#define ld_VehSpeedMax_ui16 ((UInt16)2500)

void g_VehSpeedConversion_vd(void)
{
    UInt16 l_RawVehSpeed_ui16; // local copy of raw input signal
    UInt8 l_VehSpeedvalid_ui8;

    l_RawVehSpeed_ui16  = g_ApplCanSig_st.RXSignals_st.VehicleSpeed_ui16;
    l_VehSpeedvalid_ui8 = g_ApplCanSig_st.RXSignals_st.VehicleSpeed_valid;

    if (l_VehSpeedvalid_ui8 == 0)
    {
        g_adiVehSpeed_ui16 = l_RawVehSpeed_ui16;

        // Check for MAX Limit
        if (g_adiVehSpeed_ui16 > ld_VehSpeedMax_ui16)
        {
            g_adiVehSpeed_ui16 = ld_VehSpeedMax_ui16;
        }
    }
    else
    {
        // A substituted signal shall be used:
        g_adiVehSpeed_ui16 =
            g_adiVehSpeed_ui16; // use last valid "VehSpeed" value incase of SNV
    }

    return;
}

#endif // (GS_VHS_VEH_SPEED != SW_OFF)

#if (GS_VHS_GEAR != SW_OFF)
/******************** ADI interface ************************
 * *********************************************************/
gType_Gear_en g_adiGear_en;
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "Gear" into park pilot
 * internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note       post,   PA adi signal updated
 * @see
 * @warning
 */
static gType_Gear_en g_adiGearUnfiltered_en; // Resolution: Enumeration,   see Enumeration
static const gType_Gear_en gc_DefaultGear_en =
    Park_enm; // Default AND initialization value

/**************************************************************************************/

void g_GearConversion_vd(void)
{
    UInt8 l_RawGear_ui8; // local copy of raw input signal

    l_RawGear_ui8 = g_ApplCanSig_st.RXSignals_st.Gear_ui8;

    switch (l_RawGear_ui8)
    {
        case (UInt8)0: // Position_P
            g_adiGear_en           = m_VhsApplFilterGearTransition_en(Park_enm);
            g_adiGearUnfiltered_en = Park_enm;
            break;
        case (UInt8)1: // Position_R
            g_adiGear_en           = m_VhsApplFilterGearTransition_en(Reverse_enm);
            g_adiGearUnfiltered_en = Reverse_enm;
            break;
        case (UInt8)2: // Position_N
            g_adiGear_en           = m_VhsApplFilterGearTransition_en(Neutral_enm);
            g_adiGearUnfiltered_en = Neutral_enm;
            break;
        case (UInt8)3: // Position_D
            g_adiGear_en           = m_VhsApplFilterGearTransition_en(Drive_enm);
            g_adiGearUnfiltered_en = Drive_enm;
            break;
        default:
            // keep previous value
            g_adiGear_en           = g_adiGear_en;
            g_adiGearUnfiltered_en = g_adiGearUnfiltered_en;
            break;
    }

    return;
}

#endif // (GS_VHS_GEAR != SW_OFF)

#if (GS_VHS_OUTSIDE_TEMP != SW_OFF)

/************* ADI interface
 * *********************************************************/
SInt8 g_adiOutsideTemp_si8;

static const SInt8 gc_DefaultOutsideTemp_si8 = 0; // Default AND initialization value
#define ld_OutsideTempMin_si8 ((SInt8) - 40)
#define ld_OutsideTempMax_si8 ((SInt8)80)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
****************************************************************************/
/**
 * @brief         Function to convert the incomming "OutsideTemp" into park
 * pilot internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note          PA adi signal updated
 * @see
 * @warning
 */
void g_OutsideTempConversion_vd(void)
{
    SInt8 l_RawOutsideTemp_si8; // local copy of raw input signal
    UInt8 l_OutsideTempvalid_ui8;

    l_RawOutsideTemp_si8   = g_ApplCanSig_st.RXSignals_st.Temperature_si8;
    l_OutsideTempvalid_ui8 = g_ApplCanSig_st.RXSignals_st.TemperatureInvalid;

    if (l_OutsideTempvalid_ui8 == 0)
    {
        g_adiOutsideTemp_si8 = l_RawOutsideTemp_si8; // "OutsideTemp" conversion

        // Check for MIN Limit
        if (g_adiOutsideTemp_si8 <= ld_OutsideTempMin_si8)
        {
            g_adiOutsideTemp_si8 = ld_OutsideTempMin_si8;
        }

        // Check for MAX Limit
        if (g_adiOutsideTemp_si8 > ld_OutsideTempMax_si8)
        {
            g_adiOutsideTemp_si8 = ld_OutsideTempMax_si8;
        }
    }
    else
    {
        g_adiOutsideTemp_si8 = 25;
    }

    return;
}

#endif // (GS_VHS_OUTSIDE_TEMP != SW_OFF)

#if (GS_VHS_TRAILER_HITCH_PRESENT != SW_OFF)
/************* ADI interface
 * *********************************************************/
Boolean g_adiTrailerHitchPresent_bl;

static const Boolean gc_DefaultTrailerHitchPresent_bl =
    FALSE; // Default AND initialization value
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
 ****************************************************************************/
/**
 * @brief         Function to convert the incomming "TrailerHitchPresent" into
 * park pilot internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note          PA adi signal updated
 * @see
 * @warning
 */
void g_TrailerHitchPresentConversion_vd(void)
{
    UInt8 l_RawTrailerHitchPresent_ui8; // local copy of raw input signal

    l_RawTrailerHitchPresent_ui8 =
        (UInt8)g_ApplCanSig_st.RXSignals_st.TrailerHitchPresent_ui8;

    // This is the place to implement a diagnostic signal replacement. (e.g. if
    // diag is active use default signal)
    if (l_RawTrailerHitchPresent_ui8 == 0x00)
    {
        g_adiTrailerHitchPresent_bl = FALSE;
    }
    else if (l_RawTrailerHitchPresent_ui8 == 0x01)
    {
        g_adiTrailerHitchPresent_bl = TRUE;
    }
    else
    {
    }

    return;
}

#endif // (GS_VHS_TRAILER_HITCH_PRESENT != SW_OFF)

#if (GS_VHS_TRAILER_ATTACHED != SW_OFF)
Boolean g_adiTrailerAttached_bl;

static const Boolean gc_DefaultTrailerAttached_bl =
    TRUE; // Default AND initialization value

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "TrailerAttached" into park
 * pilot internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note          PA adi signal updated
 * @see
 * @warning
 */
void g_TrailerAttachedConversion_vd(void)
{
    UInt8 l_RawTrailerAttached_ui8; // local copy of raw input signal

    // get "TrailerAttached" value and information from RTE
    l_RawTrailerAttached_ui8 = g_ApplCanSig_st.RXSignals_st.TrailerAttached_ui8;

    // This is the place to implement a diagnostic signal replacement. (e.g. if
    // diag is active use default signal)
    if (l_RawTrailerAttached_ui8 == 0x00)
    {
        g_adiTrailerAttached_bl = FALSE;
    }
    else if (l_RawTrailerAttached_ui8 == 0x01)
    {
        g_adiTrailerAttached_bl = TRUE;
    }
    else
    {
    }

    return;
}

#endif // (GS_VHS_TRAILER_ATTACHED != SW_OFF)

#if (GS_VHS_CAR_MOVE_DIR != SW_OFF)
/************* ADI interface
 * *********************************************************/
gType_CarMoveDir_en g_adiCarMoveDir_en;
/*
@@ SYMBOL      = g_adiCarMoveDir_en
@@ A2L_TYPE    = MEASURE VHS_CarMoveDir
@@ DATA_TYPE   = SBYTE
@@ GROUP       = VHS | VHS_Kat1
@@ EVENT XCP   = VARIABLE $DAQ_10ms_INPUT$
@@ ALIAS       = g_adiCarMoveDir_en
@@ CONVERSION  = TABLE 0 "Forward" 1 "Backward" 2 "Unknown" 3 "Stop"
@@ DESCRIPTION = "for PP only, for PSX use VHM_VehicleState_RollDir (Only
avalaible if signal switch is set to SW_ON)"
@@ END
*/
static const gType_CarMoveDir_en gc_DefaultCarMoveDir_en =
    MoveDirUnknown_enm; // Default AND initialization value

/**************************************************************************************/
/**
 * @brief         Function to convert the incomming "CarMoveDir" into park pilot
 * internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note          PA adi signal updated
 * @see
 * @warning
 */
void g_CarMoveDirConversion_vd(void)
{
    // Car moving Dir is virtual signal(derived from other signals), hence no RTE
    // call

    // Take Gear Info as Source for Moving Direction:

    switch (g_adiGear_en)
    {
        case Neutral_enm:
        case Park_enm:
            g_adiCarMoveDir_en = MoveDirUnknown_enm;
            break;
        case Drive_enm:
            g_adiCarMoveDir_en = MoveDirForward_enm;
            break;
        case Reverse_enm:
            g_adiCarMoveDir_en = MoveDirBackward_enm;
            break;
        default:
            // keep previous value
            g_adiCarMoveDir_en = g_adiCarMoveDir_en;
            break;
    }

    return;
}

#endif // (GS_VHS_CAR_MOVE_DIR != SW_OFF)

#if (GS_VHS_SWA != SW_OFF)
gType_SWAandTime_st g_adiSWA_st;
/*
@@ SYMBOL      = g_adiSWA_st.SWA_Signal_si16
@@ A2L_TYPE    = MEASURE VHS_SWA
@@ DATA_TYPE   = SWORD
@@ GROUP       = VHS | VHS_Kat1
@@ EVENT XCP   = VARIABLE $DAQ_10ms_INPUT$
@@ ALIAS       = g_adiSWA_st.SWA_Signal_si16
@@ CONVERSION  = LINEAR 0.04 0 "?
@@ DESCRIPTION = "PP only, for PSX use g_VHMCanSig_st.SWA_si16 (Only avalaible
if signal switch is set to SW_ON)"
@@ END

@@ SYMBOL      = g_adiSWA_st.SWA_Tick_ui16
@@ A2L_TYPE    = MEASURE VHS_SWA_timestamp
@@ DATA_TYPE   = UWORD
@@ GROUP       = VHS | VHS_Kat2
@@ EVENT XCP   = VARIABLE $DAQ_10ms_INPUT$
@@ ALIAS       = g_adiSWA_st.SWA_Tick_ui16
@@ CONVERSION  = LINEAR 1 0 "ms"
@@ DESCRIPTION = "Steering Wheel Angle Time (last update) (for PP-Systems only,
for PSX use g_VHMCanSig_st.SWA_Tick_ui16)"
@@ END
*/
static const gType_SWAandTime_st gc_DefaultSWA_st = {
    0, 0}; // Default AND initialization value
#define ld_SWAMin_si16 ((SInt16) - 25600)
#define ld_SWAMax_si16 ((SInt16)25600)

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "SWA" into park pilot
 * internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note          PA adi signal updated
 * @see
 * @warning
 */
void g_SWAConversion_vd(void)
{
    SInt32 l_RawSWA_si32; // local copy of raw input signal

    l_RawSWA_si32 = g_ApplCanSig_st.RXSignals_st.SteerWheelAngle_si32;

    l_RawSWA_si32 = l_RawSWA_si32 / (SInt32)4;

    // Check for MIN Limit
    if (l_RawSWA_si32 < ld_SWAMin_si16)
    {
        l_RawSWA_si32 = (SInt16)ld_SWAMin_si16;
    }

    // Check for MAX Limit
    if (l_RawSWA_si32 > ld_SWAMax_si16)
    {
        l_RawSWA_si32 = (SInt16)ld_SWAMax_si16;
    }

    g_adiSWA_st.SWA_Signal_si16 = (SInt16)l_RawSWA_si32; // "SWA" conversion

    return;
}

#endif // (GS_VHS_SWA != SW_OFF)

#if (GS_VHS_INDICATOR != SW_OFF)

gType_Indicator_en g_adiIndicator_en;
static const gType_Indicator_en gc_DefaultIndicator_en =
    IndicateIdle_enm; // Default AND initialization value
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "Indicator" into park pilot
 * internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note          PA adi signal updated
 * @see
 * @warning
 */
void g_IndicatorConversion_vd(void)
{
    UInt8 l_RawIndicator_ui8; // local copy of raw input signal

    l_RawIndicator_ui8 = g_ApplCanSig_st.RXSignals_st.TurnIndicator;

    switch (l_RawIndicator_ui8)
    {
        case ((UInt8)2): // Right
        {
            g_adiIndicator_en = IndicateRight_enm;
            break;
        }
        case ((UInt8)1): // Left
        {
            g_adiIndicator_en = IndicateLeft_enm;
            break;
        }
        case ((UInt8)0): // Idle
        {
            g_adiIndicator_en = IndicateIdle_enm;
            break;
        }
        case ((UInt8)3): // warningLights
        {
            g_adiIndicator_en = IndicateWarningLights_enm;
            break;
        }
        default: {
            g_adiIndicator_en = g_adiIndicator_en;
            break;
        }
    } // l_RawIndicator_ui8

    return;
}

#endif // (GS_VHS_INDICATOR != SW_OFF)

#if (GS_VHS_WICRR != SW_OFF)
gType_WICandTime_st g_adiWICRR_st;

static const gType_WICandTime_st gc_DefaultWICRR_st = {
    0, 0}; // Default AND initialization value
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "WICRR" into park pilot
 * internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note          PA adi signal updated
 * @see
 * @warning
 */
void g_WICRRConversion_vd(void)
{
    UInt16 l_RawWICRR_ui16; // local copy of raw input signal
    UInt16 l_WICRRTick_ui16;
    UInt8 l_WICRRvalid_ui8;

    l_RawWICRR_ui16  = g_ApplCanSig_st.RXSignals_st.ESC_RRWheelSpeedRC;
    l_WICRRTick_ui16 = g_ApplCanSig_st.RXSignals_st.RR_WicTick_ui16;
    l_WICRRvalid_ui8 = g_ApplCanSig_st.RXSignals_st.ESC_RRWheelSpeedRC_valid;

    if (l_WICRRvalid_ui8 == 0)
    {
        g_adiWICRR_st.WIC_Signal_ui16 = l_RawWICRR_ui16;
        g_adiWICRR_st.WIC_Tick_ui16   = l_WICRRTick_ui16;
    }
    else
    {
        g_adiWICRR_st.WIC_Signal_ui16 = g_adiWICRR_st.WIC_Signal_ui16;
    }

    return;
}

#endif // (GS_VHS_WICRR != SW_OFF)

#if (GS_VHS_WICRL != SW_OFF)
gType_WICandTime_st g_adiWICRL_st;
static const gType_WICandTime_st gc_DefaultWICRL_st = {
    0, 0}; // Default AND initialization value

/**
 * @brief         Function to convert the incomming "WICRL" into park pilot
 *internal resolution.
 * @details
 *					|                  Definition for signal
 *WICRL
 *					|-----------------------------------------------------------------------------
 *					|  TX NODE  |  MESSAGE     |  DBC SIGNAL
 *| ADI
 *					|-----------------------------------------------------------------------------
 *					|  CLAUS    |  msgESP_WIC  |
 *sigOdoWIC_RL | g_adiGetWICRL_bl
 *
 * @param
 * @return
 *
 * @note     post     PA adi signal updated
 * @see
 * @warning
 */
void g_WICRLConversion_vd(void)
{
    UInt16 l_RawWICRL_ui16; // local copy of raw input signal
    UInt16 l_WICRLTick_ui16;
    UInt8 l_WICRLvalid_ui8;

    l_RawWICRL_ui16  = g_ApplCanSig_st.RXSignals_st.ESC_RLWheelSpeedRC;
    l_WICRLTick_ui16 = g_ApplCanSig_st.RXSignals_st.RL_WicTick_ui16;
    l_WICRLvalid_ui8 = g_ApplCanSig_st.RXSignals_st.ESC_RLWheelSpeedRC_valid;

    if (l_WICRLvalid_ui8 == 0)
    {
        g_adiWICRL_st.WIC_Signal_ui16 = l_RawWICRL_ui16;
        g_adiWICRL_st.WIC_Tick_ui16   = l_WICRLTick_ui16;
    }
    else
    {
        g_adiWICRL_st.WIC_Signal_ui16 = g_adiWICRL_st.WIC_Signal_ui16;
    }

    return;
}

#endif // (GS_VHS_WICRL != SW_OFF)

#if (GS_VHS_WICFR != SW_OFF)
gType_WICandTime_st g_adiWICFR_st;

static const gType_WICandTime_st gc_DefaultWICFR_st = {
    0, 0}; // Default AND initialization value
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "WICFR" into park pilot
 * internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note      post     PA adi signal updated
 * @see
 * @warning
 */
void g_WICFRConversion_vd(void)
{
    UInt16 l_RawWICFR_ui16; // local copy of raw input signal
    UInt16 l_WICFRTick_ui16;
    UInt8 l_WICFRvalid_ui8;

    l_RawWICFR_ui16  = g_ApplCanSig_st.RXSignals_st.ESC_FRWheelSpeedRC;
    l_WICFRTick_ui16 = g_ApplCanSig_st.RXSignals_st.FR_WicTick_ui16;
    l_WICFRvalid_ui8 = g_ApplCanSig_st.RXSignals_st.ESC_FRWheelSpeedRC_valid;

    if (l_WICFRvalid_ui8 == 0)
    {
        g_adiWICFR_st.WIC_Signal_ui16 = l_RawWICFR_ui16;
        g_adiWICFR_st.WIC_Tick_ui16   = l_WICFRTick_ui16;
    }
    else
    {
        g_adiWICFR_st.WIC_Signal_ui16 = g_adiWICFR_st.WIC_Signal_ui16;
    }

    return;
}

#endif // (GS_VHS_WICFR != SW_OFF)

#if (GS_VHS_WICFL != SW_OFF)
gType_WICandTime_st g_adiWICFL_st;
static const gType_WICandTime_st gc_DefaultWICFL_st = {
    0, 0}; // Default AND initialization value
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "WICFL" into park pilot
 * internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note    post       PA adi signal updated
 * @see
 * @warning
 */
void g_WICFLConversion_vd(void)
{
    UInt16 l_RawWICFL_ui16; // local copy of raw input signal
    UInt16 l_WICFLTick_ui16;
    UInt8 l_WICFLvalid_ui8;

    l_RawWICFL_ui16  = g_ApplCanSig_st.RXSignals_st.ESC_FLWheelSpeedRC;
    l_WICFLTick_ui16 = g_ApplCanSig_st.RXSignals_st.FL_WicTick_ui16;
    l_WICFLvalid_ui8 = g_ApplCanSig_st.RXSignals_st.ESC_FLWheelSpeedRC_valid;

    if (l_WICFLvalid_ui8 == 0)
    {
        g_adiWICFL_st.WIC_Signal_ui16 = l_RawWICFL_ui16;
        g_adiWICFL_st.WIC_Tick_ui16   = l_WICFLTick_ui16;
    }
    else
    {
        g_adiWICFL_st.WIC_Signal_ui16 = g_adiWICFL_st.WIC_Signal_ui16;
    }

    return;
}

#endif // (GS_VHS_WICFL != SW_OFF)

#if (GS_VHS_WHEEL_DIR_RR != SW_OFF)
gType_WheelDirRR_en g_adiWheelDirRR_en;

static const gType_WheelDirRR_en gc_DefaultWheelDirRR_en =
    WheelDirUnknown_enm; // Default AND initialization value

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "WheelDirRR" into park pilot
 * internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note        post   PA adi signal updated
 * @see
 * @warning
 */
void g_WheelDirRRConversion_vd(void)
{
    UInt8 l_RawWheelDirRR_ui8; // local copy of raw input signal

    l_RawWheelDirRR_ui8 = g_ApplCanSig_st.RXSignals_st.ESC_RRWheelDirection;

    // Update Wheel direction
    g_adiWheelDirRR_en = m_ConvertMoveDirCOM2RTE_en(l_RawWheelDirRR_ui8);

    return;
}

#endif // (GS_VHS_WHEEL_DIR_RR != SW_OFF)

#if (GS_VHS_WHEEL_DIR_RL != SW_OFF)
gType_WheelDirRL_en g_adiWheelDirRL_en;

static const gType_WheelDirRL_en gc_DefaultWheelDirRL_en =
    WheelDirUnknown_enm; // Default AND initialization value
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "WheelDirRL" into park pilot
 * internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note      post     PA adi signal updated
 * @see
 * @warning
 */
void g_WheelDirRLConversion_vd(void)
{
    UInt8 l_RawWheelDirRL_ui8; // local copy of raw input signal

    l_RawWheelDirRL_ui8 = g_ApplCanSig_st.RXSignals_st.ESC_RLWheelDirection;

    // Update Wheel direction
    g_adiWheelDirRL_en = m_ConvertMoveDirCOM2RTE_en(l_RawWheelDirRL_ui8);

    return;
}

#endif // (GS_VHS_WHEEL_DIR_RL != SW_OFF)

#if (GS_VHS_WHEEL_DIR_FR != SW_OFF)

gType_WheelDirFR_en g_adiWheelDirFR_en;

static const gType_WheelDirFR_en gc_DefaultWheelDirFR_en =
    WheelDirUnknown_enm; // Default AND initialization value
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "WheelDirFR" into park pilot
 * internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note       post    PA adi signal updated
 * @see
 * @warning
 */
void g_WheelDirFRConversion_vd(void)
{
    UInt8 l_RawWheelDirFR_ui8; // local copy of raw input signal

    l_RawWheelDirFR_ui8 = g_ApplCanSig_st.RXSignals_st.ESC_FRWheelDirection;

    g_adiWheelDirFR_en = m_ConvertMoveDirCOM2RTE_en(l_RawWheelDirFR_ui8);

    return;
}

#endif // (GS_VHS_WHEEL_DIR_FR != SW_OFF)

#if (GS_VHS_WHEEL_DIR_FL != SW_OFF)
gType_WheelDirFL_en g_adiWheelDirFL_en;

static const gType_WheelDirFL_en gc_DefaultWheelDirFL_en =
    WheelDirUnknown_enm; // Default AND initialization value
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "WheelDirFL" into park
 * pilot internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note    post       PA adi signal updated
 * @see
 * @warning
 */
void g_WheelDirFLConversion_vd(void)
{
    UInt8 l_RawWheelDirFL_ui8; // local copy of raw input signal

    l_RawWheelDirFL_ui8 = g_ApplCanSig_st.RXSignals_st.ESC_FLWheelDirection;

    g_adiWheelDirFL_en =
        m_ConvertMoveDirCOM2RTE_en(l_RawWheelDirFL_ui8); // Update Wheel direction
}

#endif // (GS_VHS_WHEEL_DIR_FL != SW_OFF)

#if (GS_VHS_WHEEL_ROTATION_RR != SW_OFF)
UInt16 g_adiWheelRotationRR_ui16;

static const UInt16 gc_DefaultWheelRotationRR_ui16 =
    0; // Default AND initialization value
#define ld_WheelRotationRRMax_ui16 ((UInt16)16800)

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "WheelRotationRR" into park
 * pilot internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note      post     PA adi signal updated
 * @see
 * @warning
 */
void g_WheelRotationRRConversion_vd(void)
{
    UInt16 l_RawWheelRotationRR_ui16; // local copy of raw input signal
    UInt8 l_WheelRotationRRvalid_ui8;

    l_RawWheelRotationRR_ui16  = g_ApplCanSig_st.RXSignals_st.ESC_RRWheelSpeedKPH_ui16;
    l_WheelRotationRRvalid_ui8 = g_ApplCanSig_st.RXSignals_st.ESC_RRWheelSpeed_valid;

    if (l_WheelRotationRRvalid_ui8 == 0)
    {
        g_adiWheelRotationRR_ui16 =
            (UInt16)((UInt32)l_RawWheelRotationRR_ui16 /
                     (UInt32)10); // "wheel Rotation RR" conversion 0.1kmh
    }
    else
    {
        g_adiWheelRotationRR_ui16 = g_adiWheelRotationRR_ui16;
    }
    // Check for MAX Limit
    if (g_adiWheelRotationRR_ui16 > ld_WheelRotationRRMax_ui16)
    {
        g_adiWheelRotationRR_ui16 = ld_WheelRotationRRMax_ui16;
    }

    return;
}

#endif // (GS_VHS_WHEEL_ROTATION_RR != SW_OFF)

#if (GS_VHS_WHEEL_ROTATION_RL != SW_OFF)

UInt16 g_adiWheelRotationRL_ui16;

static const UInt16 gc_DefaultWheelRotationRL_ui16 =
    0; // Default AND initialization value
#define ld_WheelRotationRLMax_ui16 ((UInt16)16800)

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "WheelRotationRL" into park
 * pilot internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note    post       PA adi signal updated
 * @see
 * @warning
 */
void g_WheelRotationRLConversion_vd(void)
{
    UInt16 l_RawWheelRotationRL_ui16; // local copy of raw input signal
    UInt8 l_WheelRotationRLvalid_ui8;

    l_RawWheelRotationRL_ui16  = g_ApplCanSig_st.RXSignals_st.ESC_RLWheelSpeedKPH_ui16;
    l_WheelRotationRLvalid_ui8 = g_ApplCanSig_st.RXSignals_st.ESC_RLWheelSpeed_valid;

    if (l_WheelRotationRLvalid_ui8 == 0)
    {
        g_adiWheelRotationRL_ui16 =
            (UInt16)((UInt32)l_RawWheelRotationRL_ui16 /
                     (UInt32)10); // "wheel Rotation RL" conversion 0.1kmh
    }
    else
    {
        g_adiWheelRotationRL_ui16 = g_adiWheelRotationRL_ui16;
    }
    // Check for MAX Limit
    if (g_adiWheelRotationRL_ui16 > ld_WheelRotationRLMax_ui16)
    {
        g_adiWheelRotationRL_ui16 = ld_WheelRotationRLMax_ui16;
    }

    return;
}

#endif // (GS_VHS_WHEEL_ROTATION_RL != SW_OFF)

#if (GS_VHS_WHEEL_ROTATION_FR != SW_OFF)
UInt16 g_adiWheelRotationFR_ui16;

static const UInt16 gc_DefaultWheelRotationFR_ui16 =
    0; // Default AND initialization value
#define ld_WheelRotationFRMax_ui16 ((UInt16)16800)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "WheelRotationFR" into park
 * pilot internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note      post    PA adi signal updated
 * @see
 * @warning
 */
void g_WheelRotationFRConversion_vd(void)
{
    UInt16 l_RawWheelRotationFR_ui16; // local copy of raw input signal
    UInt8 l_WheelRotationFRvalid_ui8;

    l_RawWheelRotationFR_ui16  = g_ApplCanSig_st.RXSignals_st.ESC_FRWheelSpeedKPH_ui16;
    l_WheelRotationFRvalid_ui8 = g_ApplCanSig_st.RXSignals_st.ESC_FRWheelSpeed_valid;

    if (l_WheelRotationFRvalid_ui8 == 0)
    {
        g_adiWheelRotationFR_ui16 =
            (UInt16)((UInt32)l_RawWheelRotationFR_ui16 /
                     (UInt32)10); // "wheel Rotation FR" conversion 0.1kmh
    }
    else
    {
        g_adiWheelRotationFR_ui16 = g_adiWheelRotationFR_ui16;
    }
    // Check for MAX Limit
    if (g_adiWheelRotationFR_ui16 > ld_WheelRotationFRMax_ui16)
    {
        g_adiWheelRotationFR_ui16 = ld_WheelRotationFRMax_ui16;
    }

    return;
}

#endif // (GS_VHS_WHEEL_ROTATION_FR != SW_OFF)

#if (GS_VHS_WHEEL_ROTATION_FL != SW_OFF)
UInt16 g_adiWheelRotationFL_ui16;

static const UInt16 gc_DefaultWheelRotationFL_ui16 =
    0; // Default AND initialization value
#define ld_WheelRotationFLMax_ui16 ((UInt16)16800)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "WheelRotationFL" into park
 * pilot internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note       post   PA adi signal updated
 * @see
 * @warning
 */
void g_WheelRotationFLConversion_vd(void)
{
    UInt16 l_RawWheelRotationFL_ui16; // local copy of raw input signal
    UInt8 l_WheelRotationFLvalid_ui8;

    l_RawWheelRotationFL_ui16  = g_ApplCanSig_st.RXSignals_st.ESC_FLWheelSpeedKPH_ui16;
    l_WheelRotationFLvalid_ui8 = g_ApplCanSig_st.RXSignals_st.ESC_FLWheelSpeed_valid;

    if (l_WheelRotationFLvalid_ui8 == 0)
    {
        g_adiWheelRotationFL_ui16 =
            (UInt16)((UInt32)l_RawWheelRotationFL_ui16 /
                     (UInt32)10); // "wheel Rotation FL" conversion 0.1kmh
    }
    else
    {
        g_adiWheelRotationFL_ui16 = g_adiWheelRotationFL_ui16;
    }

    // Check for MAX Limit
    if (g_adiWheelRotationFL_ui16 > ld_WheelRotationFLMax_ui16)
    {
        g_adiWheelRotationFL_ui16 = ld_WheelRotationFLMax_ui16;
    }

    return;
}

#endif // (GS_VHS_WHEEL_ROTATION_FL != SW_OFF)

#if (GS_VHS_LONG_ACC != SW_OFF)
SInt16 g_adiLongAcc_si16;

static const SInt16 gc_DefaultLongAcc_si16 = 0; // Default AND initialization value
#define ld_LongAccMin_si16 ((SInt16) - 2000)
#define ld_LongAccMax_si16 ((SInt16)2000)

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "LongAcc" into park pilot
 * internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note      post    PA adi signal updated
 * @see
 * @warning
 */
void g_LongAccConversion_vd(void)
{
    SInt16 l_RawLongAcc_si16; // local copy of raw input signal
    UInt8 l_LongitAcce_valid_ui8;

    l_RawLongAcc_si16      = g_ApplCanSig_st.RXSignals_st.LongitAcce_si16;
    l_LongitAcce_valid_ui8 = g_ApplCanSig_st.RXSignals_st.LongitAcce_valid;

    // This is the place to implement a diagnostic signal replacement. (e.g. if
    // diag is active use default signal)
    if (l_LongitAcce_valid_ui8 == 0)
    {
        g_adiLongAcc_si16 = (SInt16)(l_RawLongAcc_si16); // 0.01 is pf resolution

        // Check for MIN Limit
        if (g_adiLongAcc_si16 < ld_LongAccMin_si16)
        {
            g_adiLongAcc_si16 = ld_LongAccMin_si16;
        }

        // Check for MAX Limit
        if (g_adiLongAcc_si16 > ld_LongAccMax_si16)
        {
            g_adiLongAcc_si16 = ld_LongAccMax_si16;
        }
    }
    else
    {
        g_adiLongAcc_si16 = g_adiLongAcc_si16;
    }

    return;
}

#endif // (GS_VHS_LONG_ACC != SW_OFF)

#if (GS_VHS_YAW_RATE != SW_OFF)

SInt16 g_adiYawRate_si16;

static const SInt16 gc_DefaultYawRate_si16 = 0; // Default AND initialization value
#define ld_YawRateMin_si16 ((SInt16) - 10000)
#define ld_YawRateMax_si16 ((SInt16)10000)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "YawRate" into park pilot
 * internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note       post,   PA adi signal updated
 * @see
 * @warning
 */
void g_YawRateConversion_vd(void)
{
    SInt16 l_RawYawRate_si16; // local copy of raw input signal

    l_RawYawRate_si16 = g_ApplCanSig_st.RXSignals_st.YawRate_si16;

    // This is the place to implement a diagnostic signal replacement. (e.g. if
    // diag is active use default signal) "wheel Rotation FL" conversion

    g_adiYawRate_si16 = (SInt16)(l_RawYawRate_si16);

    // Check for MIN Limit
    if (g_adiYawRate_si16 < ld_YawRateMin_si16)
    {
        g_adiYawRate_si16 = ld_YawRateMin_si16;
    }

    // Check for MAX Limit
    if (g_adiYawRate_si16 > ld_YawRateMax_si16)
    {
        g_adiYawRate_si16 = ld_YawRateMax_si16;
    }

    return;
}

#endif // (GS_VHS_YAW_RATE != SW_OFF)

#if (GS_VHS_EPS_STATUS != SW_OFF)
gType_EpsStatus_en g_adiEpsStatus_en;

static const gType_EpsStatus_en gc_DefaultEpsStatus_en =
    EpsInactive_enm; // Default AND initialization value
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "EpsStatus" into park pilot
 * internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note      post,    PA adi signal updated
 * @see
 * @warning
 */
void g_EpsStatusConversion_vd(void)
{
    UInt8 l_RawEpsStatus_ui8; // local copy of raw input signal
    UInt8 l_RawEpsStatusValid_ui8;

    l_RawEpsStatus_ui8      = g_ApplCanSig_st.RXSignals_st.EPS_status;
    l_RawEpsStatusValid_ui8 = 0;

    if ((l_RawEpsStatusValid_ui8 == 2) || (l_RawEpsStatusValid_ui8 == 3) ||
        (l_RawEpsStatusValid_ui8 == 4))
    {
        g_adiEpsStatus_en = gc_DefaultEpsStatus_en; // use default "EPSStatus" value
    }
    else
    {
        g_adiEpsStatus_en = (gType_EpsStatus_en)
            l_RawEpsStatus_ui8; // use last valid "EPSStatus" value incase of SNV
    }

    return;
}

#endif // (GS_VHS_EPS_STATUS != SW_OFF)

gType_EscStatus_en g_adiEscStatus_en;

static const gType_EscStatus_en gc_DefaultEscStatus_en =
    EscOff_enm; // Default AND initialization value

/**
 * @brief  esc status coversion
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_EscStatusConversion_vd(void)
{
    UInt8 l_RawEscStatus_ui8; // local copy of raw input signal

    l_RawEscStatus_ui8 = g_ApplCanSig_st.RXSignals_st.ESC_Status;

    switch (l_RawEscStatus_ui8)
    {
        case 0:
            g_adiEscStatus_en = EscStandby_enm;
            break;
        case 1:
            g_adiEscStatus_en = EscActive_enm;
            break;
        case 2:
            g_adiEscStatus_en = EscOff_enm;
            break;
        case 3:
            g_adiEscStatus_en = EscOff_enm;
            break;
        case 4:
            g_adiEscStatus_en = EscError_enm;
            break;
        default:
            g_adiEscStatus_en = EscOff_enm;
            break;
    }

    return;
}

#if (GS_VHS_ENGINE_START != SW_OFF)
gType_EngineStart_en g_adiEngineStart_en;

static const gType_EngineStart_en gc_DefaultEngineStart_en =
    EngineNoStart_enm; // Default AND initialization value

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
------------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "EngineStart" into park
 * pilot internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note       post,   PA adi signal updated
 * @see
 * @warning
 */
void g_EngineStartConversion_vd(void)
{
    UInt8 l_RawEngineStart_ui8; // local copy of raw input signal

    l_RawEngineStart_ui8 = g_ApplCanSig_st.RXSignals_st.EngineStatus;

    if (l_RawEngineStart_ui8 == 0x00)
    {
        g_adiEngineStart_en = EngineNoStart_enm;
    }
    else if (l_RawEngineStart_ui8 == 0x01)
    {
        g_adiEngineStart_en = EngineFirstStart_enm;
    }
    else
    {
        g_adiEngineStart_en = EngineNoStart_enm;
    }

    return;
}

#endif // (GS_VHS_ENGINE_START != SW_OFF)

#if (GS_VHS_POWERMODE != SW_OFF)

gType_PowerMode_en g_adiPowerMode_en; // Resolution: Enumeration
static gType_PowerMode_en m_PrePowerMode_en;
static const gType_PowerMode_en gc_DefaultPowerMode_en =
    KL_15OFF_enm; // Default AND initialization value

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "PowerMode" into park pilot
 * internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note   post,       PA adi signal updated
 * @see
 * @warning
 */
static void m_PowerModeConversion_vd(void)
{
    UInt8 l_RawPowerMode_ui8; // local copy of raw input signal

    l_RawPowerMode_ui8 = g_ApplCanSig_st.RXSignals_st.PowerMode;

    if (l_RawPowerMode_ui8 == 0)
    {
        g_adiPowerMode_en = KL_15OFF_enm;
    }
    else if (l_RawPowerMode_ui8 == 1)
    {
        g_adiPowerMode_en = KL_15_enm;
    }
    else
    {
    }

    m_PrePowerMode_en = g_adiPowerMode_en;

    return;
}

#endif // #if (GS_VHS_POWERMODE != SW_OFF)

gType_PSMode_en g_adiPSMode_en; // Resolution:

static const gType_PSMode_en gc_DefaultPSMode_en =
    PSNone_enm; // Default AND initialization value
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Function to convert the incomming "PSMode" into park pilot
 * internal resolution.
 * @details
 *
 * @param
 * @return
 *
 * @note      post,    PA adi signal updated
 * @see
 * @warning
 */
void g_PSModeConversion_vd(void)
{
    UInt8 l_RawPSMode_ui8; // local copy of raw input signal

    l_RawPSMode_ui8 = 0;

    switch (l_RawPSMode_ui8)
    {
        case (UInt8)0:
            g_adiPSMode_en = PSNone_enm;
            break;
        case (UInt8)1:
            g_adiPSMode_en = PSEmergencyStop_enm;
            break;
        case (UInt8)2:
            g_adiPSMode_en = PSComfort_enm;
            break;
        default:
            // do nothing
            break;
    } // l_RawPSMode_ui8

    return;
}
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Initialize project specific signals at startup.
 * @details
 *
 * @param
 * @return
 *
 * @note          project specific signals initialised at startup
 * @see
 * @warning
 */
void g_vhsApplInit_vd(void)
{
    // initialise variables in appl_vhs_output.c
    //  g_vhsApplOutputInit_vd();

// initial degradation mode set (degradation mode will be set by EFH)
#if 1
    g_VhsDegMode_en = DegMode_VHS_NoDegradation_enm; // DegMode_VHS_allSigSubstitute_enm;
#else
    g_VhsDegMode_en = DegMode_VHS_NoDegradation_enm;
#endif // sinkwang

    m_PrePowerMode_en = gc_DefaultPowerMode_en;

#if (GS_VHS_VEH_SPEED != SW_OFF)
    g_SetDefaultValues_mac(g_adiVehSpeed_ui16, gc_DefaultVehSpeed_ui16);
#endif
#if (GS_VHS_GEAR != SW_OFF)
    g_SetDefaultValues_mac(g_adiGear_en, gc_DefaultGear_en);
#endif
#if (GS_VHS_OUTSIDE_TEMP != SW_OFF)
    g_SetDefaultValues_mac(g_adiOutsideTemp_si8, gc_DefaultOutsideTemp_si8);
#endif
#if (GS_VHS_TRAILER_HITCH_PRESENT != SW_OFF)
    g_SetDefaultValues_mac(g_adiTrailerHitchPresent_bl, gc_DefaultTrailerHitchPresent_bl);
#endif
#if (GS_VHS_CAR_MOVE_DIR != SW_OFF)
    g_SetDefaultValues_mac(g_adiCarMoveDir_en, gc_DefaultCarMoveDir_en);
#endif
#if (GS_VHS_SWA != SW_OFF)
    g_SetDefaultValues_mac(g_adiSWA_st, gc_DefaultSWA_st);
#endif
#if (GS_VHS_INDICATOR != SW_OFF)
    g_SetDefaultValues_mac(g_adiIndicator_en, gc_DefaultIndicator_en);
#endif
#if (GS_VHS_WICRR != SW_OFF)
    g_SetDefaultValues_mac(g_adiWICRR_st, gc_DefaultWICRR_st);
#endif
#if (GS_VHS_WICRL != SW_OFF)
    g_SetDefaultValues_mac(g_adiWICRL_st, gc_DefaultWICRL_st);
#endif
#if (GS_VHS_WICFR != SW_OFF)
    g_SetDefaultValues_mac(g_adiWICFR_st, gc_DefaultWICFR_st);
#endif
#if (GS_VHS_WICFL != SW_OFF)
    g_SetDefaultValues_mac(g_adiWICFL_st, gc_DefaultWICFL_st);
#endif
#if (GS_VHS_WHEEL_DIR_RR != SW_OFF)
    g_SetDefaultValues_mac(g_adiWheelDirRR_en, gc_DefaultWheelDirRR_en);
#endif
#if (GS_VHS_WHEEL_DIR_RL != SW_OFF)
    g_SetDefaultValues_mac(g_adiWheelDirRL_en, gc_DefaultWheelDirRL_en);
#endif
#if (GS_VHS_WHEEL_DIR_FR != SW_OFF)
    g_SetDefaultValues_mac(g_adiWheelDirFR_en, gc_DefaultWheelDirFR_en);
#endif
#if (GS_VHS_WHEEL_DIR_FL != SW_OFF)
    g_SetDefaultValues_mac(g_adiWheelDirFL_en, gc_DefaultWheelDirFL_en);
#endif
#if (GS_VHS_WHEEL_ROTATION_RR != SW_OFF)
    g_SetDefaultValues_mac(g_adiWheelRotationRR_ui16, gc_DefaultWheelRotationRR_ui16);
#endif
#if (GS_VHS_WHEEL_ROTATION_RL != SW_OFF)
    g_SetDefaultValues_mac(g_adiWheelRotationRL_ui16, gc_DefaultWheelRotationRL_ui16);
#endif
#if (GS_VHS_WHEEL_ROTATION_FR != SW_OFF)
    g_SetDefaultValues_mac(g_adiWheelRotationFR_ui16, gc_DefaultWheelRotationFR_ui16);
#endif
#if (GS_VHS_WHEEL_ROTATION_FL != SW_OFF)
    g_SetDefaultValues_mac(g_adiWheelRotationFL_ui16, gc_DefaultWheelRotationFL_ui16);
#endif
#if (GS_VHS_LONG_ACC != SW_OFF)
    g_SetDefaultValues_mac(g_adiLongAcc_si16, gc_DefaultLongAcc_si16);
#endif
#if (GS_VHS_YAW_RATE != SW_OFF)
    g_SetDefaultValues_mac(g_adiYawRate_si16, gc_DefaultYawRate_si16);
#endif

#if (GS_VHS_EPS_STATUS != SW_OFF)
    g_SetDefaultValues_mac(g_adiEpsStatus_en, gc_DefaultEpsStatus_en);
#endif
#if (GS_VHS_ENGINE_START != SW_OFF)
    g_SetDefaultValues_mac(g_adiEngineStart_en, gc_DefaultEngineStart_en);
#endif

#if (GS_VHS_TRAILER_HITCH_PRESENT != SW_OFF)
    g_SetDefaultValues_mac(g_adiTrailerHitchPresent_bl, gc_DefaultTrailerHitchPresent_bl);
#endif
#if (GS_VHS_TRAILER_ATTACHED != SW_OFF)
    g_SetDefaultValues_mac(g_adiTrailerAttached_bl, gc_DefaultTrailerAttached_bl);
#endif

    /***********************************************************************************/
    /* Posibility to initialize additional project specific input signals: */
    /***********************************************************************************/

    g_SetDefaultValues_mac(g_adiPowerMode_en, gc_DefaultPowerMode_en);

    g_SetDefaultValues_mac(g_adiPSMode_en, gc_DefaultPSMode_en);

    g_SetDefaultValues_mac(g_adiEscStatus_en, gc_DefaultEscStatus_en);

    return;
}
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Project specific function.
 * @details       This function is called cyclic within the function
 * g_vhsProcessing_vd, before all other conversion functions for input signals
 * are called.
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_VhsApplInputPreConversion_vd(void)
{
    // Read signals from RTE that are used in more than one conversion function
    // to be able to create consistant data.

    return;
}

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Project specific conversion function.
 * @details       This function is called cyclic within the function
 * g_vhsProcessing_vd, after all other conversion functions for input signals
 * are called.
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_VhsApplInputPostConversion_vd(void)
{
    // Implement project specific conversion functions
    // ----------------------------------------------------

#if (GS_VHS_POWERMODE != SW_OFF)
    m_PowerModeConversion_vd();
#endif

    g_PSModeConversion_vd();

    return;
}
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         routine to filter gear transition when shifting fromP <-> D.
 * @details       problem: order of gear positions in vehicle P <-> R <-> N <->
 * D transitions, may cause unwanted system states, transitions from R or N to
 * any other position will be accepted immediately, transitions from P or D to R
 * or N will be accecped after LS_APPLSIG_GEARBOX_FILTER_TIME ms
 *
 * @param         f_NewGear_en   new, normalized, value from CAN,
 *                               possible return values: PARK, REVERSE, NEUTRAL,
 * DRIVE
 * @return                       iltered gear position,
 *                               possible return values: PARK, REVERSE, NEUTRAL,
 * DRIVE
 *
 * @note
 * @see
 * @warning
 */

static gType_Gear_en m_VhsApplFilterGearTransition_en(gType_Gear_en f_NewGear_en)
{
    // return value - init with current signal state
    gType_Gear_en l_retVal_en;
    // Filter counter
    static UInt8 ls_FilterCounter_ui8;

    // debounce if shift from park or drive to neutral or reverse
    if ((g_adiGear_en == Park_enm) || (g_adiGear_en == Drive_enm))
    {
        if (((f_NewGear_en == Reverse_enm) || (f_NewGear_en == Neutral_enm)) &&
            (ls_FilterCounter_ui8 < gd_VHS_GearboxFilterTime_ui8))
        {
            ls_FilterCounter_ui8++;
            // keep old value
            l_retVal_en = g_adiGear_en;
        }
        else
        {
            // Reset filter counter
            ls_FilterCounter_ui8 = (UInt8)0;
            l_retVal_en          = f_NewGear_en;
        }
    }
    else
    {
        // Reset filter counter
        ls_FilterCounter_ui8 = (UInt8)0;
        l_retVal_en          = f_NewGear_en;
    }

    return l_retVal_en;
}

#if (GS_VHS_WICFL != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Derive Tick-Signal from OS and store Tick value for WIC Signal
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_SetFLWICTick_vd(void)
{
    // local variable
    UInt16 l_CurWICTick_ui16 = 0;

    // Generate WIC Tick
    // (void)g_BswOs1msTimeCounter_en (&l_CurWICTick_ui16);
    l_CurWICTick_ui16 = g_GetTimeMs_ui16();

    // Store Tick Signal in temporary data structure
    m_TickTmp_st.WIC_FL_Tick_ui16 = l_CurWICTick_ui16;
}
#endif

#if (GS_VHS_WICFR != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Derive Tick-Signal from OS and store Tick value for WIC Signal
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_SetFRWICTick_vd(void)
{
    // local variable
    UInt16 l_CurWICTick_ui16 = 0;

    // Generate WIC Tick
    // (void)g_BswOs1msTimeCounter_en (&l_CurWICTick_ui16);
    l_CurWICTick_ui16 = g_GetTimeMs_ui16();

    // Store Tick Signal in temporary data structure
    m_TickTmp_st.WIC_FR_Tick_ui16 = l_CurWICTick_ui16;
}
#endif

#if (GS_VHS_WICRL != SW_OFF)
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Derive Tick-Signal from OS and store Tick value for WIC Signal
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_SetRLWICTick_vd(void)
{
    // local variable
    UInt16 l_CurWICTick_ui16 = 0;

    // Generate WIC Tick
    // (void)g_BswOs1msTimeCounter_en (&l_CurWICTick_ui16);
    l_CurWICTick_ui16 = g_GetTimeMs_ui16();

    // Store Tick Signal in temporary data structure
    m_TickTmp_st.WIC_RL_Tick_ui16 = l_CurWICTick_ui16;
}
#endif

#if (GS_VHS_WICRR != SW_OFF)

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Derive Tick-Signal from OS and store Tick value for WIC Signal
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_SetRRWICTick_vd(void)
{
    // local variable
    UInt16 l_CurWICTick_ui16 = 0;

    // Generate WIC Tick
    // (void)g_BswOs1msTimeCounter_en (&l_CurWICTick_ui16);
    l_CurWICTick_ui16 = g_GetTimeMs_ui16();

    // Store Tick Signal in temporary data structure
    m_TickTmp_st.WIC_RR_Tick_ui16 = l_CurWICTick_ui16;
}
#endif

#if (GS_VHS_SWA != SW_OFF)

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Derive Tick-Signal from OS and store Tick value for SAS Signal
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_SetSASTick_vd(void)
{
    // local variable
    UInt16 l_CurSASTick_ui16 = 0;

    // Generate SAS Tick
    // (void)g_BswOs1msTimeCounter_en (&l_CurSASTick_ui16);
    l_CurSASTick_ui16 = g_GetTimeMs_ui16();

    // Store Tick Signal in temporary data structure
    m_TickTmp_st.SAS_Tick_ui16 = l_CurSASTick_ui16;
}
#endif

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         Converts Moving Direction as received from COM Stack into ADI
 * Resolution.
 * @details
 *
 * @param         f_MoveDirCOM_ui8 Moving Direction as received from COM Stack ,
 *                                 {0 = StillStand; 1 = Forward; 2 = Backward; 3
 * = Error/Unknown}
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if ((GS_VHS_WHEEL_DIR_RR != SW_OFF) || (GS_VHS_WHEEL_DIR_RL != SW_OFF) || \
     (GS_VHS_WHEEL_DIR_FR != SW_OFF) || (GS_VHS_WHEEL_DIR_FL != SW_OFF))
static gType_WheelDir_en m_ConvertMoveDirCOM2RTE_en(UInt8 f_MoveDirCOM_ui8)
{
    gType_WheelDir_en l_Result_en;

    switch (f_MoveDirCOM_ui8)
    {
        case (UInt8)0:
            l_Result_en = WheelDirStop_enm;
            break;
        case (UInt8)1:
            l_Result_en = WheelDirForward_enm;
            break;
        case (UInt8)2:
            l_Result_en = WheelDirBackward_enm;
            break;
        default:
            l_Result_en = WheelDirUnknown_enm;
            break;
    }
    // endswitch

    return l_Result_en;
}
#endif // ((GS_VHS_WHEEL_DIR_RR != SW_OFF) || (GS_VHS_WHEEL_DIR_RL != SW_OFF) ||
       //  (GS_VHS_WHEEL_DIR_FR != SW_OFF) || (GS_VHS_WHEEL_DIR_FL != SW_OFF))