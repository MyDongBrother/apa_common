/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.锛孡td
 * All rights reserved.
 *
 * @file      appl_parm.h
 * @brief
 * @details
 *
 *****************************************************************************/

#ifndef APPL_PARM_H
#define APPL_PARM_H

#define APPL_PAR_H_TEMPL_NUMBER 0x090326

#ifdef COMPONENT_PAR
#endif // COMPONENT_PAR

#ifdef COMPONENT_PAR
#endif // COMPONENT_PAR

#ifdef COMPONENT_PAR

// definition of the lower calibration header
typedef struct
{
    // implement additional information here
    // Be carefull when using combinations of UInt8, UInt16 or UInt32 constants
    // -> compiler may add padding bytes inbetween
    UInt32 dummy_ui32;
} gType_parCalHeaderLow_st;

// definition of the lower calibration header
typedef struct
{
    // implement additional information here
    // Be carefull when using combinations of UInt8, UInt16 or UInt32 constants
    // -> compiler may add padding bytes inbetween
    UInt32 dummy_ui32;
} gType_parCalHeaderHigh_st;

#endif // COMPONENT_PAR

#ifdef COMPONENT_PAR
#endif // COMPONENT_PAR

#ifdef COMPONENT_PAR
#endif // COMPONENT_PAR

extern Boolean g_parApplCheckParaIntegrity_bl(void);
extern Boolean g_parApplInitPara_bl(void);
extern UInt8 g_parApplGetMajorVariant_ui8(void);
extern void g_parApplInit_vd(void);
extern void g_parApplProcessing_vd(void);

#endif /* APPL_PARM_H */
/*========================= EoF (appl_parm.h) ===================*/
