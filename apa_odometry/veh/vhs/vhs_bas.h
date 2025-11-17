/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: vhs_bas.h
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#ifndef VHS_BAS_H
#define VHS_BAS_H

#ifdef COMPONENT_VHS // Only defined within this component! /*!\cond INTERNAL */
#endif // COMPONENT_VHS                                     /*!\endcond       */

#define LS_VHS_TESTHARNESS SW_OFF
/* Madatory Setting:           SW_OFF for all projects */

#ifdef COMPONENT_VHS // Only defined within this component! /*!\cond INTERNAL */
#define GS_VHS_OUTPUT_SWADESIRED LS_VHS_OUTPUT_SWA_DESIRED
#endif // COMPONENT_VHS                                     /*!\endcond       */

#ifdef COMPONENT_VHS // Only defined within this component! /*!\cond INTERNAL */

// The following macro only fills in the default signals during initialization
// of the component VHS.

#define g_SetDefaultValues_mac(f_adiSignal, f_DefaultSignal) \
    do                                                       \
    {                                                        \
        (f_adiSignal) = (f_DefaultSignal);                   \
    } while (0)

#endif // COMPONENT_VHS                                     /*!\endcond       */

#ifdef COMPONENT_VHS // Only defined within this component! /*!\cond INTERNAL */
#endif // COMPONENT_VHS                                     /*!\endcond       */

/*!@} */ /*------------------------------------------------------------------*/
/*-                           external object declarations                 -*/
/*--------------------------------------------------------------------------*/
#ifdef COMPONENT_VHS // Only defined within this component! /*!\cond INTERNAL */
#endif // COMPONENT_VHS                                     /*!\endcond       */

/*--------------------------------------------------------------------------*/
/*-                         prototypes of exported functions               -*/
/*--------------------------------------------------------------------------*/
#ifdef COMPONENT_VHS // Only defined within this component! /*!\cond INTERNAL */
extern void g_vhsApplInit_vd(void);

extern void g_VhsApplInputPreConversion_vd(void);

extern void g_VehSpeedConversion_vd(void);
extern void g_GearConversion_vd(void);
extern void g_OutsideTempConversion_vd(void);
extern void g_TrailerHitchPresentConversion_vd(void);
extern void g_TrailerAttachedConversion_vd(void);
extern void g_CarMoveDirConversion_vd(void);
extern void g_SWAConversion_vd(void);
extern void g_IndicatorConversion_vd(void);
extern void g_WICRRConversion_vd(void);
extern void g_WICRLConversion_vd(void);
extern void g_WICFRConversion_vd(void);
extern void g_WICFLConversion_vd(void);
extern void g_WheelDirRRConversion_vd(void);
extern void g_WheelDirRLConversion_vd(void);
extern void g_WheelDirFRConversion_vd(void);
extern void g_WheelDirFLConversion_vd(void);
extern void g_WheelRotationRRConversion_vd(void);
extern void g_WheelRotationRLConversion_vd(void);
extern void g_WheelRotationFRConversion_vd(void);
extern void g_WheelRotationFLConversion_vd(void);
extern void g_LongAccConversion_vd(void);
extern void g_YawRateConversion_vd(void);

extern void g_EpsStatusConversion_vd(void);
extern void g_EngineStartConversion_vd(void);
extern void g_EscStatusConversion_vd(void);

extern void g_VhsApplInputPostConversion_vd(void);

#endif /* VHS_BAS_H */

#endif
/*========================= EoF (vhs_bas.h) ====================*/
