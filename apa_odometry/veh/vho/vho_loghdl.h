/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.锛孡td
 * All rights reserved.
 *
 * @file      vho_loghdl.h
 * @brief
 * @details
 *
 *****************************************************************************/

#ifndef VHO_LOGHDL_H
#define VHO_LOGHDL_H

#ifdef COMPONENT_VHO

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

// fault byte bit indices
#define md_VHOLog_FaultWordIdx_InitFault_ui8       ((UInt8)0)
#define md_VHOLog_FaultWordIdx_InSigFailureWIC_ui8 ((UInt8)1)
// #define md_VHOLog_FaultWordIdx_InSigFailureSWA_ui8              ((UInt8)2)
#define md_VHOLog_FaultWordIdx_UnknownDistDiscarded_ui8 ((UInt8)2)
#define md_VHOLog_FaultWordIdx_UnknownDistExceeded_ui8  ((UInt8)3)
#define md_VHOLog_FaultWordIdx_ExtraPolPosFault_ui8     ((UInt8)4)
// fault index used to indicate a disabled bit in the fault byte
#define md_VHOLog_FaultWordIdx_Unknown_ui8 ((UInt8)255)

// fault array
// each bit represents the state (active/inactive) of one fault
typedef union {
    // bit wise access - used for debugging
    struct
    {
        UInt8 InitFault_ui8 : 1;
        UInt8 InSigFailureWIC_ui8 : 1;
        //    UInt8 InSigFailureSWA_ui8                 : 1;
        UInt8 EEPROMWrite_ui8 : 1;
        UInt8 EEPROMRead_ui8 : 1;
        UInt8 Autocalib_ui8 : 1;
        UInt8 DirRecog_ui8 : 1;
        UInt8 VHOMain_ui8 : 1;
        UInt8 USSProc_ui8 : 1;
        UInt8 OdoCore_ui8 : 1;
        UInt8 VHOLog_ExtraPolPosFault_ui8 : 1;
        UInt8 UnspecErr_ui8 : 1;
        UInt8 MaxUnknownDistExceeded_enm : 1;
        UInt8 MaxUnknownDistDiscarded_enm : 1;
    } bit_access_st;

    // byte wise access - access to memory; set/reset bits
    UInt8 byte_access_pui8[(VHOLog_TotalNumber_enm / 8) + 1];

} mType_vhoLogFaultArray_un;

// fault byte
// extract of the fault array (one bit represents up to n faults of the fault array)
typedef union {
    // bit access - used for debugging
    struct
    {
        UInt8 InitFault_ui8 : 1;
        UInt8 InSigFailureWIC_ui8 : 1;
        //    UInt8 InSigFailureSWA_ui8                     : 1;
        UInt8 DirectionErrMaxDistDiscarded_ui8 : 1;
        UInt8 DirectionErrMaxDistExceeded_ui8 : 1;
        UInt8 ExtraPolPosFault_ui8 : 1;
        UInt8 Unused_ui8 : 6;
    } bit_access_st;

    // byte access - access to memory; set/reset bits
    UInt16 word_access_ui16;

} mType_vhoLogFaultWord_un;

typedef enum
{
    VHO_SYMPTOM_IGNORE_enm,
    VHO_SYMPTOM_OFF_enm,
    VHO_SYMPTOM_ON_enm
} gType_VhoSymptomState_en;

typedef enum
{
    VhoWicFailure_enm,
    VhoUnknownDirection_enm,
    VhoExtraPolPosFault_enm,
    NUM_OF_VHO_SYMPTOMS_enm /* number of all symptoms - do not delete */
} gType_VhoSymptoms_en;

extern void g_vhoLogHdlInit_vd(void);

extern void g_vhoLogHdlReportToFSV_vd(void);

extern void g_VhoReportSymTestResult_vd(gType_VhoSymptoms_en f_SymptomId_en,
                                        gType_VhoSymptomState_en f_state_en);

#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#endif // COMPONENT_VHO
#endif
