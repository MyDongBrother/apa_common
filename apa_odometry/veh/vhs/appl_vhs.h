/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: appl_vhs.h
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#ifndef APPL_VHS_H
#define APPL_VHS_H

//! number (current date "JJMMDD") for component compatibility check between
//! appl_... and core files
#define APPL_VHS_H_TEMPL_NUMBER 0x110723

#include "vhs_bas.h"
// #include "mp_api.h"
// #include "Protocol_Api.h"

#ifdef COMPONENT_VHS // Only defined within this component! /*!\cond INTERNAL */
#endif // COMPONENT_VHS                                     /*!\endcond       */
#if (GS_PP_FUNC_VARIANTHANDLER != SW_OFF)
#define M_VEHICLEVAR_FILTERTIME_COUNT \
    (UInt8)(10) // x300ms(Cycle time of CAN msg) = 3000ms
#endif

#if (GS_PP_FUNC_PS != SW_OFF)
#define GS_VHS_ESP_PS_STATE SW_ON
#define g_adiGetEspPsState_bl \
    g_adiGetESPPSState_bl // Tobe removed once PS component change the ADI
#define gType_EspPSState_en \
    gType_EspPsState_en // Tobe removed once PS component change the typedef
#else
#define GS_VHS_ESP_PS_STATE SW_OFF
#endif

#ifdef COMPONENT_VHS // Only defined within this component! /*!\cond INTERNAL */
#endif

// Degradation modes provided by VHS (DegModes are set by EFH)
typedef enum gT_VhsDegModes_en
{
    DegMode_VHS_NoDegradation_enm,   // manditory DegMode
    DegMode_VHS_allSigSubstitute_enm // manditory DegMode
    // possibility to add additional project specific degradation modes
} gType_VhsDegModes_en;

/**************************************/
// Signal structures and enumerations
/**************************************/

#if (GS_VHS_GEAR != SW_OFF)
typedef enum gT_Gear_en
{
    Park_enm = 0,
    Reverse_enm,
    Neutral_enm,
    Drive_enm,
    NoSignal_enm
    // possibility to add additional project specific gear modes
} gType_Gear_en;
#endif

#if (GS_VHS_CAR_MOVE_DIR != SW_OFF)
typedef enum gT_CarMoveDir_en
{
    MoveDirForward_enm,
    MoveDirBackward_enm,
    MoveDirUnknown_enm,
    MoveDirStop_enm
    // possibility to add additional project specific car moving directions
} gType_CarMoveDir_en;
#endif

#if (GS_VHS_INDICATOR != SW_OFF)
typedef enum gT_Indicator_en
{
    IndicateIdle_enm,
    IndicateRight_enm,
    IndicateLeft_enm,
    IndicateWarningLights_enm
    // possibility to add additional project specific indicator information
} gType_Indicator_en;
#endif

#if ((GS_VHS_WHEEL_DIR_RR != SW_OFF) || (GS_VHS_WHEEL_DIR_RL != SW_OFF) || \
     (GS_VHS_WHEEL_DIR_FR != SW_OFF) || (GS_VHS_WHEEL_DIR_FL != SW_OFF))
typedef enum gT_WheelDir_en
{
    WheelDirForward_enm,
    WheelDirBackward_enm,
    WheelDirUnknown_enm,
    WheelDirStop_enm
    // possibility to add additional project specific wheel direction information
} gType_WheelDir_en;
// the following defines are required to be able to create the wheel directions
// conversion with the genSig tooling
typedef gType_WheelDir_en gType_WheelDirRR_en;
typedef gType_WheelDir_en gType_WheelDirRL_en;
typedef gType_WheelDir_en gType_WheelDirFR_en;
typedef gType_WheelDir_en gType_WheelDirFL_en;

#endif

typedef enum
{
    VHSAxleRear_enm   = 1,
    VHSAxleFront_enm  = 2,
    VHSAxle4Wheel_enm = 3
} gType_VHSAxle_en;

#if (GS_VHS_EPS_STATUS != SW_OFF)
typedef enum gT_EpsStatus_en
{
    EpsReady_enm,
    EpsActive_enm,
    EpsAborted_enm,
    EpsInactive_enm
    // possibility to add additional project specific EPS status
} gType_EpsStatus_en;
#endif

typedef enum gT_EscStatus_en
{
    EscOff_enm     = 0x00,
    EscStandby_enm = 0x01,
    EscActive_enm  = 0x02,
    EscError_enm   = 0x03
} gType_EscStatus_en;

typedef enum gT_EscError_en
{
    EscNoError_enm                  = 0x00,
    EscVehBlockedError_enm          = 0x01,
    EscUnexpectedGearError_enm      = 0x02,
    EscUnexpectedEPBError_enm       = 0x03,
    EscUnexpectedBrakeError_enm     = 0x04,
    EscUnexpectedGearInterError_enm = 0x05,
    EscLimitSlopeError_enm          = 0x06,
    EscOtherError_enm               = 0x07
} gType_EscError_en;

#if (GS_VHS_ENGINE_START != SW_OFF)
typedef enum gT_EngineStart_en
{
    EngineNoStart_enm,    // No engine start is planned (engine may already be
                          // running or is turned off)
    EngineFirstStart_enm, // First engine start planned (engine will be started by
                          // the driver)
    EngineRestart_enm     // Engine restart is planned  (engine will be automatically
                          // restarted (start-stop-))
    // possibility to add additional project specific Engine Start status
} gType_EngineStart_en;
#endif

/*sigSystemActivationDir values */
typedef enum gT_sigSystemActivationDir_en
{
    NoActivationDir_enm = 0,
    Backwards_enm       = 1,
    Forwards_enm        = 2
} gType_sigSystemActivationDir_en;

typedef enum gT_SystemParkDir_en
{
    NoParkDir_enm = 0,
    PullIn_enm    = 1,
    PullOut_enm   = 2
} gType_SystemParkDir_en;

#if (GS_VHS_SWA != SW_OFF)
// SWA (steering wheel angle)
typedef struct gT_SWAandTime_st
{
    SInt16 SWA_Signal_si16; // SWA Signal
    // | Steering Wheel Angle Sensor Signal |      signal range      |
    // | Resolution (1Bit = 0,04?          | Min value | Max value  |
    // |                                    | -1024?   | +1024?    |

    UInt16 SWA_Tick_ui16; // Final Timestamp copied after normalization to Final
                          // value
    // | Steering Wheel Angle Sensor tick   |      signal range      |
    // | Resolution (1Bit = 1ms)            | Min value | Max value  |
    // | (Time derived from OS)             | 0 ms      | 65535 ms   |
} gType_SWAandTime_st;
#endif

#if ((GS_VHS_WICRR != SW_OFF) || (GS_VHS_WICRL != SW_OFF) || (GS_VHS_WICFR != SW_OFF) || \
     (GS_VHS_WICFL != SW_OFF))
// WIC structure
typedef struct gT_WICandTime_st
{
    UInt16 WIC_Signal_ui16; // WIC Signal
    // | Wheel Impulse Counter Signal       |      signal range      |
    // | Resolution (1Bit = 1 pulse)        | Min value | Max value  |
    // |                                    | 0 pulse   | 65535 pulse|

    UInt16 WIC_Tick_ui16; // Final Timestamp, copied after normalization to Final
                          // value
    // | Wheel Impulse Counter tick Signal  |      signal range      |
    // | Resolution (1Bit = 1ms)            | Min value | Max value  |
    // | (ms derived from OS)               | 0 ms      | 65535 ms   |

} gType_WICandTime_st;
#endif

/*******************************************************************************/
/* Possibility to add additional project specific structures and enumerations:
 */
/*******************************************************************************/

/* Powermodes (ignition key positions) */
typedef enum gT_PowerMode_en
{
    KL_15OFF_enm  = 0, // No ignition key present                (car completely OFF)
    KL_15C_enm    = 1, // Ignition key present                   (accessory)
    KL_15R_enm    = 2, // Ignition key in Radio ON position      (radio on)
    KL_15_enm     = 4, // Ignition key in Ignition ON position   (ignition on)
    KL_15X_enm    = 8, // Ignition key in crank request position (crank/engine start)
    KL_15Erun_enm = 16 // Ignition key ON / Engine running       (no crank request active)
} gType_PowerMode_en;

typedef gType_PowerMode_en gType_Powermode_en; // required for the component AP_CTL

typedef enum gT_Lowbeam_en
{
    LowBeamOFF_enm = 0, //[ LowBeamOFF ]
    LowBeamON_enm  = 1  //[ LowBeamON ]
} gType_Lowbeam_en;

typedef enum gT_Mainbeam_en
{
    MainBeamOFF_enm = 0, //[ MainBeamOFF ]
    MainBeamON_enm  = 1  //[ MainBeamON ]
} gType_Mainbeam_en;

typedef enum gT_EpbMode_en
{
    EPB_Unkonw    = 0,
    EPB_Closed    = 1,
    EPB_Released  = 2,
    EPB_Inprocess = 3,
    EPB_Reserved  = 4

} gType_EpbMode_en;

typedef enum gT_DoorStatus_en
{
    Door_Closed = 0,
    Door_Open   = 1
} gType_DoorStatus_en;

typedef enum gT_BrakeStatus_en
{
    Brake_notActive = 0,
    Brake_Active    = 1,
} gType_BrakeStatus_en;

typedef enum gT_AccPedalStatus_en
{
    AccPedal_notActive = 0,
    AccPedal_Active    = 1,
} gType_AccPedalStatus_en;

typedef enum gT_WinperStatus_en
{
    Winper_Off       = 0,
    Winper_LowSpeed  = 1,
    Winper_HighSpeed = 2
} gType_WinperStatus_en;

typedef enum gT_SeatBeltStatus_en
{
    SeatBelt_tied     = 0,
    SeatBelt_not_tied = 1
} gType_SeatBeltStatus_en;

#if (GS_VHS_ESP_PS_STATE != SW_OFF)
typedef enum gT_EspPSState_en
{
#if (GS_PP_FUNC_PS != SW_OFF)
    PapInit_enm         = 0,
    PapStandBy_enm      = 15,
    PapActivatable_enm  = 51,
    PapSSM_enm          = 60,
    PapControl_enm      = 85,
    PapHMI_enm          = 90,
    PapError_enm        = 102,
    PapControlMBAc_enm  = 105,
    PapControlMBAc_Stby = 150
#endif

    // Possibility to add additional Project specific EPS Status:
    // ...
} gType_EspPsState_en;
#endif

/*PSVeloSwitch values */
typedef enum gT_PSMode_en
{
    PSNone_enm          = 0, //[ None ]
    PSEmergencyStop_enm = 1, //[ EmergencyStop]
    PSComfort_enm       = 2, //[ Comfort ]
    PSOff_enm           = 3
} gType_PSMode_en;

// Parkpilot switch logical states
typedef enum gT_Switch_en
{
    SWITCH_NEUTRAL_enm        = 0, // Switch in idle position
    SWITCH_ON_enm             = 1, // Switch activated: ON position
    SWITCH_PSC_ON_enm         = 2, // Switch activated: PSC Position
    SWITCH_cPSC_ON_enm        = 3, // Switch activated: cPSC Position
    SWITCH_PSC_POC_ON_enm     = 4, // Switch activated: POC Position
    SWITCH_SVA_ON_enm         = 5, // Switch activated: SVA Position
    SWITCH_BA_ON_enm          = 6, // Switch activated: BA Position
    SWITCH_OFF_enm            = 7, // Switch activated: OFF position
    SWITCH_ABORT_GUIDANCE_enm = 8, // Switch activated: Abording on CPsc/PSC/POC guidance
    SWITCH_INTO_GUIDANCE_enm =
        9, // Switch activated: if PsPosPos and car is standing still
    SWITCH_cPSC_POC_ON_enm = 10
} gType_SystemActivation_en;

/* definition of user specific signals */
typedef enum
{
    gd_applVhmEPSActivatable_enm = 4,
    gd_applVhmEPSActive_enm      = 6,
    gd_applVhmEPSReady_enm       = 8,
    gd_applVhmEPSRestart_enm     = 9,
    gd_applVhmEPSAblort_enm      = 10,
} gType_applVhmEPSStatus_en;

/*System Activation switch values */
typedef enum gT_SysActivationSwitch_en
{
    SysOFF_enm  = 0,
    SysPP_enm   = 1,
    SysPSI_enm  = 2,
    SysPSC_enm  = 3,
    SysCPSI_enm = 4,
    SysCPSC_enm = 5,
    SysPOC_enm  = 6,
    SysSDW_enm  = 7
} gType_SysActivationSwitch_en;

typedef enum
{
    PauseReq_None_enm       = 0,
    PauseReq_Activation_enm = 1,
} gType_ParkingPauseReq_en;

typedef enum
{
    SlotType_None_enm     = 0,
    SlotType_Parallel_enm = 1,
    SlotType_Cross_enm    = 2,
    SlotType_Diagonal_enm = 3,
} gType_ParkSlotType_en;

typedef enum
{
    SelPS_None_enm   = 0,
    SelPS_Left1_enm  = 1,
    SelPS_Left2_enm  = 2,
    SelPS_Left3_enm  = 3,
    SelPS_Left4_enm  = 4,
    SelPS_Left5_enm  = 5,
    SelPS_Left6_enm  = 6,
    SelPS_Left7_enm  = 7,
    SelPS_Left8_enm  = 8,
    SelPS_Right1_enm = 9,
    SelPS_Right2_enm = 10,
    SelPS_Right3_enm = 11,
    SelPS_Right4_enm = 12,
    SelPS_Right5_enm = 13,
    SelPS_Right6_enm = 14,
    SelPS_Right7_enm = 15,
    SelPS_Right8_enm = 16
} gType_SelectParkSlot_en;

typedef enum
{
    ParkingMode_None_enm     = 0,
    ParkingMode_APA_enm      = 1,
    ParkingMode_RPA_enm      = 2,
    ParkingMode_AVP_enm      = 3,
    ParkingMode_Forword_enm  = 4,
    ParkingMode_Tracking_enm = 5,
} gType_ParkingModeSrc_en;

typedef enum
{
    SelPoc_None_enm       = 0,
    SelPoc_Front_enm      = 1,
    SelPoc_Rear_enm       = 2,
    SelPoc_Left_enm       = 3,
    SelPoc_Right_enm      = 4,
    SelPoc_FrontLeft_enm  = 5,
    SelPoc_FrontRight_enm = 6,
    SelPoc_RearLeft_enm   = 7,
    SelPoc_RearRight_enm  = 8,

} gType_SelectPocDirect_en;

typedef enum
{
    AppSts_DisConnect_enm = 0,
    AppSts_Connect_enm    = 1,
    AppSts_Unstable_enm   = 2,
} gType_AppAliveStatus_en;

typedef enum gT_MEBWorkSts_en
{
    MEB_OFF_enm = 0,
    MEB_Standby,
    MEB_Active,
    MEB_Failed,
} gType_MEBWorkSts_en;
typedef enum gT_MEB_EnableSts_en
{
    MEB_Disable_enm = 0,
    MEB_Enable_enm,
} gType_MEB_EnableSts_en;

#endif
/*========================= EoF (appl_vhs.h) ===================*/
