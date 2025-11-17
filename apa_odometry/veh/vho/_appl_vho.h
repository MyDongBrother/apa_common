/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.锛孡td
 * All rights reserved.
 *
 * @file      _appl_vho.h
 * @brief
 * @details
 *
 *****************************************************************************/
#ifndef APPL_VHO_H
#define APPL_VHO_H

#define APPL_VHO_H_TEMPL_NUMBER 0x140325
/*----------------------------------------------------------------*/

#include "vho_bas.h" // SMS Include Structure: new

#define ld_TestFlag_bl (TRUE)

// Enumerated states for EEPROM - Idle or Busy
typedef enum
{
    EEPROMIdle_enm = 0,
    EEPROMBusy_enm = 1
} gType_EEPROMState_en;

// EEPROM request types - Read or Write
typedef enum
{
    EEPROMRead_enm    = 0,
    EEPROMWrite_enm   = 1,
    EEPROMUnknown_enm = 2
} gType_EEPROMOperation_en;

typedef enum
{
    m_vho_EE_RequestMode_ReadAutocalibData_enm = 0, // read could only be performed after
                                                    // init; once per ignition cycle!!!!!
    m_vho_EE_RequestMode_WriteAutocalibData_enm = 1,
    m_vho_EE_RequestMode_None_enm               = 2
} gType_EE_RequestMode_en;

typedef enum
{
    MONITORING_OFF_enm,
    MONITORING_SEARCH_enm,
    MONITORING_GUIDANCE_enm
} gType_VhoMonitoringMode_en;

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 0781
#endif
/*Because of historical reason 'AutocalibStateEEPROM_st' is defined
as structure and with the same name it is a member of other structure */
// extern gType_VHOParaAutocalib_st AutocalibStateEEPROM_st;

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 0781
#endif

extern void g_vhoPreInitAppl_vd(void);

extern void g_vhoInitAppl_vd(void);

#ifdef COMPONENT_VHO
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
extern void g_vhoInitParamsAppl_vd(gType_VHOParameters_st *f_VHOParameters_pst);
#endif
#endif

extern void g_vhoReInitAppl_vd(void);

extern void g_vhoProcessingPreAppl_vd(void);

extern void g_vhoProcessingPostAppl_vd(void);

#ifdef COMPONENT_VHO
extern void g_VHOGetSignalsAppl_vd(gType_VHOCanSig_st *f_VHOCanSig_pst);
#endif

#ifdef COMPONENT_VHO
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC == SW_ON) || \
     (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON) ||                                    \
     (GS_VHOTCE_EXTERNAL_TIRE_CRCUMFERENCE_ESTIMATION == SW_ON) ||                \
     (GS_VHOTCE_EXTERNAL_TIRE_TOLERANCE_CALIBRATION == SW_ON))
extern void m_VHOSetEepromRequestModeToWrite_vd(void);
#endif
#endif
#endif

extern Boolean g_VHPSMxIdle_bl(void);

#ifdef COMPONENT_VHO
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
void m_vhoGetPARPointer_vd(gType_VHOParameters_st *f_VHOParameters_pst);
#endif
#endif

extern void g_adiVhoGetPsxTimestamp_vd(UInt16 *f_WicTime_pui16);

#ifdef COMPONENT_VHO
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
Boolean m_VHOPluginSetExternalCircumference_bl(UInt32 *f_TyreCircumferenceMMF8_pui32);
#endif
#endif

#ifdef COMPONENT_VHOSRC
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (GS_VHOSRC_SLIP_RECOGNITION == SW_ON)
/* API function for mode change */
extern void g_vhoSetVHOMonitoringMode_vd(
    gType_VhoMonitoringMode_en f_VhoMonitoringMode_en);
/* API function to retrieve mode change */
extern gType_VhoMonitoringMode_en g_vhoGetVHOMonitoringMode_vd(void);
#endif // #if (GS_VHOSRC_SLIP_RECOGNITION == SW_ON)
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#endif // #ifdef COMPONENT_VHOSRC

#define g_vhoReadBlockVhoCalibData_en(data) \
    {                                       \
    }
#define g_vhoWriteBlockVhoCalibData_en(data) \
    {                                        \
    }
#define g_vhoGetErrorStatusVhoCalibData_vd(data) \
    {                                            \
    }

#endif // #ifndef APPL_VHO_H
