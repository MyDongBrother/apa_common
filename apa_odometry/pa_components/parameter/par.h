/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.锛孡td
 * All rights reserved.
 *
 * @file      parm.h
 * @brief
 * @details
 *
 *****************************************************************************/

#ifndef PARM_H
#define PARM_H

#ifdef COMPONENT_PAR // Only defined within this component!
#endif               // COMPONENT_PAR

#ifdef COMPONENT_PAR
#endif // COMPONENT_PAR

#ifdef COMPONENT_PAR

// component processing state
typedef enum Type_parProcessingState_en
{
    PAR_PROCESSING_STATE_CHECK_enm,     // check parameter integrity
    PAR_PROCESSING_STATE_PARA_INIT_enm, // parameter initialization
    PAR_PROCESSING_STATE_NORMAL_enm     // normal operating mode
} gType_parProcessingState_en;

typedef struct Type_parState_st
{
    gType_parProcessingState_en
        ProcessingState_en; // current state of internal state machine
} gType_parState_st;

#endif // COMPONENT_PAR

#ifdef COMPONENT_PAR

extern gType_parState_st g_parState_st;

extern UInt8 g_parMajorVariant_ui8;

extern UInt8 g_parUpdateFlags_ui8;
extern UInt8 g_parUpdateFlags_Shadow_ui8;

#if (GS_PAR_CFG_APPLICATION_MODE == SW_ON)

extern UInt8 g_parApplicationModeFlag_ui8;
extern UInt8 g_parApplicationModeFlag_Prev_ui8;

#endif

extern const UInt8 gc_parBitPatternLookUp_pui8[8];

// extern gType_VehParaType_en g_Curparatype_en;

#endif // COMPONENT_PAR

#ifdef COMPONENT_PAR

extern UInt8 g_parGetDataOffset_ui8(UInt8 f_cl_idx_ui8, UInt8 f_minor_var_idx_ui8,
                                    UInt8 sub_var_idx_ui8);

#endif // COMPONENT_PAR

#endif /* PARM_H */
/*========================= EoF (parm.h) ====================*/
