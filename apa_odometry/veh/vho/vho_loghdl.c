/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: vho_loghdl.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/
#define COMPONENT_VHO

#include "vho_api.h"

#include <pa_components/base_type/global_include.h>
#include <pa_components/mathlib/mathlib_api.h>

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#include "vho.h"
#include "vho_loghdl.h"

#ifndef LS_VHO_EXTRAPOL_ENGINE_RESTART
#error ('compiler switch LS_VHO_EXTRAPOL_ENGINE_RESTART is not defined')
#endif

// fault byte configuration
// structure which contains the configuration of one bit of the fault byte
typedef struct
{
    UInt8 FaultWordIdx_ui8;        // bit index in the fault word
    Boolean SpFsvReportingEnabled; // report fault to FSM (TRUE -> yes / FALSE -> no)
    UInt16 SpFsvSymptomID_en;      // FSV symptom ID from appl_fsm_prj.h
} mType_vhoLogFaultWordConfig_st;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

// fault array
VISIBILITY mType_vhoLogFaultArray_un m_vhoLogFaultArray_un;

// fault byte
VISIBILITY mType_vhoLogFaultWord_un m_vhoLogFaultWord_un;

// fault byte config for "init fault"
// directly route the init fault from the fault array to the fault byte - no
// filtering  set/reset immediately
VISIBILITY const mType_vhoLogFaultWordConfig_st mc_FaultWordConfigInitFault_st = {
    md_VHOLog_FaultWordIdx_InitFault_ui8, // bit index in the fault byte

#if (LS_VHO_ENABLEFAULTINIT == SW_OFF)
    FALSE,                  // report fault to FSV (TRUE -> yes / FALSE -> no)
    NUM_OF_VHO_SYMPTOMS_enm // FSV symptom ID
#else
    TRUE,            // report fault to SpFsv (TRUE -> yes / FALSE -> no)
    FsvEpVhoInit_enm // SpFsv symptom ID from appl_SpFsv_prj.h
#endif
};

// fault byte config for "InSigFailureWIC: debounce with count filter
VISIBILITY const mType_vhoLogFaultWordConfig_st mc_FaultWordConfigInSigFailureWIC_st = {
    md_VHOLog_FaultWordIdx_InSigFailureWIC_ui8, // bit index in the fault
                                                // byte
    TRUE, // report fault to SpFsv (TRUE -> yes / FALSE -> no)
    0     // SpFsv symptom ID from appl_SpFsv_prj.h
};

/*
// fault byte config for "InSigFailureSWA"
// directly route the init fault from the fault array to the fault byte - no
filtering  set/reset immediately VISIBILITY const mType_vhoLogFaultWordConfig_st
mc_FaultWordConfigInSigFailureSWA_st =
{
    md_VHOLog_FaultWordIdx_InSigFailureSWA_ui8,       // bit index in the fault
byte TRUE,                                             // report fault to SpFsv
(TRUE -> yes / FALSE -> no) FsvEpVhoSwaFailure_enm                            //
SpFsv symptom ID from appl_SpFsv_prj.h
};
*/

// fault byte config for "UnknownDirection"
// directly route the init fault from the fault array to the fault byte - no
// filtering  set/reset immediately
VISIBILITY const mType_vhoLogFaultWordConfig_st mc_FaultWordConfigMaxUnkDistExeeded_st = {
    md_VHOLog_FaultWordIdx_UnknownDistExceeded_ui8, // bit index in the
                                                    // fault byte
    TRUE, // report fault to SpFsv (TRUE -> yes / FALSE -> no)
    0     // SpFsv symptom ID from appl_SpFsv_prj.h
};

// fault byte config for "UnknownDirection"
// directly route the init fault from the fault array to the fault byte - no
// filtering  set/reset immediately
VISIBILITY const mType_vhoLogFaultWordConfig_st
    mc_FaultWordConfigMaxUnkDistDiscarded_st = {
        md_VHOLog_FaultWordIdx_UnknownDistDiscarded_ui8, // bit index in the
                                                         // fault byte
        TRUE, // report fault to SpFsv (TRUE -> yes / FALSE -> no)
        0     // SpFsv symptom ID from appl_SpFsv_prj.h
};

// fault byte config for "ExtraPolPosFault"
VISIBILITY const mType_vhoLogFaultWordConfig_st mc_FaultWordConfigExtraPolPosFault_st = {
    // nprod00135766 --> Control Inclusion of Code Related to Engine Restart
    // Failure Handling
    md_VHOLog_FaultWordIdx_ExtraPolPosFault_ui8, // bit index in the fault
                                                 // byte
#if (GS_SP_FSV_ENG_RESTART_FAULT_HDL == SW_OFF)
    FALSE,                  // report fault to FSV (TRUE -> yes / FALSE -> no)
    NUM_OF_VHO_SYMPTOMS_enm // FSV symptom ID
#else
    TRUE, // report fault to SpFsv (TRUE -> yes / FALSE -> no)
    0     // SpFsv symptom ID from appl_SpFsv_prj.h
#endif
};

// fault byte config for "unused fault"
// everything disabled used as a dummy for unused entries
VISIBILITY const mType_vhoLogFaultWordConfig_st mc_FaultWordConfigUnused_st = {
    md_VHOLog_FaultWordIdx_Unknown_ui8, // bit index in the fault byte
    FALSE,                  // report fault to SpFsv (TRUE -> yes / FALSE -> no)
    NUM_OF_VHO_SYMPTOMS_enm // SpFsv symptom ID from appl_SpFsv_prj.h
};

// fault array configuration
// contains pointers to the fault byte configurations per fault ID
VISIBILITY const mType_vhoLogFaultWordConfig_st
    *const mc_vhoLogFaultArrayConfig_ppst[VHOLog_TotalNumber_enm] = {
        &mc_FaultWordConfigInitFault_st,       // VHOLog_InitFault_enm = 0,
        &mc_FaultWordConfigInSigFailureWIC_st, // VHOLog_InSigFailureWIC_enm
                                               // = 1,
        //   &mc_FaultWordConfigInSigFailureSWA_st,          //
        //   VHOLog_InSigFailureSWA_enm          = 2,
        &mc_FaultWordConfigUnused_st,             // VHOLog_EEPROMWrite_enm = 2,
        &mc_FaultWordConfigUnused_st,             // VHOLog_EEPROMRead_enm = 3,
        &mc_FaultWordConfigUnused_st,             // VHOLog_Autocalib_enm = 4,
        &mc_FaultWordConfigUnused_st,             // VHOLog_DirRecog_enm = 5,
        &mc_FaultWordConfigUnused_st,             // VHOLog_VHOMain_enm = 6,
        &mc_FaultWordConfigUnused_st,             // VHOLog_USSProc_enm = 7,
        &mc_FaultWordConfigUnused_st,             // VHOLog_OdoCore_enm = 8,
        &mc_FaultWordConfigExtraPolPosFault_st,   // VHOLog_ExtraPolPosFault_enm
                                                  // = 9,
        &mc_FaultWordConfigUnused_st,             // VHOLog_UnspecErr_enm = 10,
        &mc_FaultWordConfigMaxUnkDistExeeded_st,  // VHOLog_MaxUnknownDistExceeded_enm
                                                  // = 11,
        &mc_FaultWordConfigMaxUnkDistDiscarded_st // VHOLog_MaxUnknownDistDiscarded_enm
                                                  // = 12
};

#endif

/**
 * @brief Initialize error handle for vho component
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
void g_vhoLogHdlInit_vd(void)
{
    UInt32 l_i_ui32;

    // reset all fault information
    for (l_i_ui32 = 0; l_i_ui32 < (SInt32)VHOLog_TotalNumber_enm; l_i_ui32++)
    {
/*
  suppress QAC warning messages 1482
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 1482
#endif

        g_vhoLogHdl_vd((gType_vhoLogFaultID_en)l_i_ui32, g_vhoLogStateReset_enm);

/*
  reactivate suppressed QAC warning messages 1482
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 1482
#endif
    }
}

/**
 * @brief
 * @details Error reporting to FSV based on internal error state and
 *          degredation mode
 *
 * @param
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_vhoLogHdlReportToFSV_vd(void)
{
#if (LS_VHO_EXTRAPOL_ENGINE_RESTART == SW_ON)
    if ((g_VHOCtrlStat_st.DegMode_en == DegMode_VHO_Extrapolation_enm) ||
        (m_vhoLogFaultArray_un.bit_access_st.VHOLog_ExtraPolPosFault_ui8 != 0))
    {
        // set/reset error VhoExtraPolPosFault_enm independent from degredation
        // mode do not set any other error even if it is present
        if (m_vhoLogFaultArray_un.bit_access_st.VHOLog_ExtraPolPosFault_ui8 != 0)
        {
            //      g_VhoReportSymTestResult_vd(VhoExtraPolPosFault_enm,
            //      VHO_SYMPTOM_ON_enm);
        }
        else
        {
            //      g_VhoReportSymTestResult_vd(VhoExtraPolPosFault_enm,
            //      VHO_SYMPTOM_OFF_enm);
        }
    }
    else
    {
#endif // (LS_VHO_EXTRAPOL_ENGINE_RESTART == SW_ON)

        // no degredation: set/reset all errors depending on internal state
        // remark: this includes VhoExtraPolPosFault_enm (reset is only done in
        // mode NoDeg) report error to SpFsv if reporting is enabled for this error

        UInt8 l_FaultID_ui8;
        UInt8 l_byte_idx_ui8;
        UInt8 l_bit_idx_ui8;
        gType_VhoSymptomState_en l_sp_fsv_sym_state_en = VHO_SYMPTOM_OFF_enm;
        Boolean l_DirError_bl                          = FALSE;

        for (l_FaultID_ui8 = 0; l_FaultID_ui8 < (UInt8)VHOLog_TotalNumber_enm;
             l_FaultID_ui8++)
        {
            // iterate through all internal errors
            if (mc_vhoLogFaultArrayConfig_ppst[l_FaultID_ui8]->SpFsvReportingEnabled !=
                FALSE)
            {
                // reporting SHALL be done
                l_byte_idx_ui8 = l_FaultID_ui8 / (UInt8)8;
                l_bit_idx_ui8  = l_FaultID_ui8 % (UInt8)8;

                if (g_mtl_TstBit_u8_bl(
                        m_vhoLogFaultArray_un.byte_access_pui8[l_byte_idx_ui8],
                        l_bit_idx_ui8) != FALSE)
                {
                    l_sp_fsv_sym_state_en = VHO_SYMPTOM_ON_enm;
                }
                else
                {
                    l_sp_fsv_sym_state_en = VHO_SYMPTOM_OFF_enm;
                }
            }

            if ((l_FaultID_ui8 == (UInt8)VHOLog_MaxUnknownDistExceeded_enm) ||
                (l_FaultID_ui8 == (UInt8)VHOLog_MaxUnknownDistDiscarded_enm))
            {
                // handling of direction errors that are cumulated to one single FSV
                // error
                if (l_sp_fsv_sym_state_en == VHO_SYMPTOM_ON_enm)
                {
                    l_DirError_bl = TRUE;
                }
            }
            else
            {
                // report error state to FSV
                //  g_VhoReportSymTestResult_vd(mc_vhoLogFaultArrayConfig_ppst[l_FaultID_ui8]->SpFsvSymptomID_en,
                //  l_sp_fsv_sym_state_en);
            }
        }
        // handling of direction error
        if (l_DirError_bl != FALSE)
        {
            // g_VhoReportSymTestResult_vd(mc_vhoLogFaultArrayConfig_ppst[VHOLog_MaxUnknownDistExceeded_enm]->SpFsvSymptomID_en,
            //                VHO_SYMPTOM_ON_enm);
        }
        else
        {
            // g_VhoReportSymTestResult_vd(mc_vhoLogFaultArrayConfig_ppst[VHOLog_MaxUnknownDistExceeded_enm]->SpFsvSymptomID_en,
            //                VHO_SYMPTOM_OFF_enm);
        }
#if (LS_VHO_EXTRAPOL_ENGINE_RESTART == SW_ON)
    } // NoDegMode
#endif // (LS_VHO_EXTRAPOL_ENGINE_RESTART == SW_ON)
}

/**
 * @brief Error handle function for VHO component
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
void g_vhoLogHdl_vd(gType_vhoLogFaultID_en f_fault_id_enm,
                    gType_vhoLogState_en f_state_enm)
{
    // check fault range
    if (f_fault_id_enm < VHOLog_TotalNumber_enm)
    {
        // fault array processing: set flag in m_vhoLogFaultArray_un that is
        // corresponding to f_fault_id_enm
        {
            const UInt8 l_byte_idx_ui8 = (UInt8)f_fault_id_enm / (UInt8)8;
            const UInt8 l_bit_idx_ui8  = (UInt8)f_fault_id_enm % (UInt8)8;

            // set/reset fault in fault array
            if (f_state_enm == g_vhoLogStateOn_enm)
            {
                g_mtl_SetBit_u8_mac(
                    m_vhoLogFaultArray_un.byte_access_pui8[l_byte_idx_ui8],
                    l_bit_idx_ui8);
            }
            else
            {

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 0290, 4130, 0564
#endif
                // off or reset
                // remark: with the log handler init function all the errors are reset
                // in initialization!
                g_mtl_ClrBit_u8_mac(
                    m_vhoLogFaultArray_un.byte_access_pui8[l_byte_idx_ui8],
                    l_bit_idx_ui8);

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 0290, 4130, 0564
#endif
            }
        }

        // set bit in fault byte if it is configured for the error
        if (mc_vhoLogFaultArrayConfig_ppst[f_fault_id_enm]->FaultWordIdx_ui8 !=
            md_VHOLog_FaultWordIdx_Unknown_ui8)
        {
            if (f_state_enm == g_vhoLogStateOn_enm)
            {
                // set corresponding bit in fault byte
                g_mtl_SetBit_u16_mac(
                    m_vhoLogFaultWord_un.word_access_ui16,
                    mc_vhoLogFaultArrayConfig_ppst[f_fault_id_enm]->FaultWordIdx_ui8);
            }
            else
            {
                // reset corresponding bit in fault byte
                g_mtl_ClrBit_u16_mac(
                    m_vhoLogFaultWord_un.word_access_ui16,
                    mc_vhoLogFaultArrayConfig_ppst[f_fault_id_enm]->FaultWordIdx_ui8);
            }
        }

        /*
        // report error to SpFsv if reporting is enabled for this error
        if ( mc_vhoLogFaultArrayConfig_ppst[f_fault_id_enm]->SpFsvReportingEnabled
        != FALSE )
        {
          gType_VhoSymptomState_en l_sp_fsv_sym_state_en;

          // always directly route reset events
          if ( f_state_enm == g_vhoLogStateReset_enm )
          {
            l_sp_fsv_sym_state_en = SP_FSV_SYMPTOM_IGNORE_enm;
            g_SpFsvResetSymFilter_vd(mc_vhoLogFaultArrayConfig_ppst[f_fault_id_enm]->SpFsvSymptomID_en);
          }
          else
          {
            // report on only if error is present
            if ( f_state_enm == g_vhoLogStateOn_enm )
            {
              l_sp_fsv_sym_state_en = VHO_SYMPTOM_ON_enm;
            }
            else
            {
              // f_state_enm == g_vhoLogStateOff_enm
              l_sp_fsv_sym_state_en = VHO_SYMPTOM_OFF_enm;
            }

            // report to SpFsv
            g_VhoReportSymTestResult_vd(mc_vhoLogFaultArrayConfig_ppst[f_fault_id_enm]->SpFsvSymptomID_en,
        l_sp_fsv_sym_state_en);
          }
        } // reporting enabled
        */
    } // error is within the valid range
}

void g_VhoReportSymTestResult_vd(gType_VhoSymptoms_en f_SymptomId_en,
                                 gType_VhoSymptomState_en f_state_en)
{

    return;
}

#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
