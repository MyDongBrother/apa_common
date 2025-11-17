/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: vhs.h
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#ifndef VHS_H
#define VHS_H

/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/

#ifdef COMPONENT_VHS // Only defined within this component! /*!\cond INTERNAL */

/*--------------------------------------------------------------------------*/
/*- check if compiler switches (utilized in this file) are defined         -*/
/*--------------------------------------------------------------------------*/
#ifndef LS_VHS_CAN_SPEED_SUBST
#error ('switch LS_VHS_CAN_SPEED_SUBST is not defined')
#endif

/*--------------------------------------------------------------------------*/
/*!\addtogroup VHS_M02           exported symbolic constants
 * \ingroup COMPONENT_VHS
 @{ *//*--------------------------------------------------------------------*/
#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
#define gd_VeloInvalid_ui16 ((UInt16)0xffff)
#define gd_MaxWICelements_ui8 \
    ((UInt8)60) // 400ms Abdeckung -> bei 10ms zwischen 2 CAN Botschaften = 40
                // Elemente

#define gd_WICFiltertime_ui16 ((UInt16)600)
#define gd_WICcycleTime_ui8   ((UInt8)10)
#define gd_WICindexMax_ui8    (UInt8)(gd_WICFiltertime_ui16 / gd_WICcycleTime_ui8)

#define gd_WICnoSpeedUpdate_ui16 \
    ((UInt16)65535) // 65535 = 6553,5 km/h  -> this speed should never be reached
#endif

#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
typedef struct
{
    UInt16 CanTime_ui16;
    UInt16 WICBuf_ui16;
    UInt16 WICVeloLast_ui16;
    UInt16 WICdelta_pui16[gd_MaxWICelements_ui8];
    UInt16 CanDelta_pui16[gd_MaxWICelements_ui8];
    UInt8 WICdeltaIndex_ui8;
    Boolean InitPhase_bl;
} gType_vhsWICbuf_st;
#endif

/*--------------------------------------------------------------------------*/
/*-                         prototypes of exported functions               -*/
/*--------------------------------------------------------------------------*/
#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)
UInt16 g_CalcVehicleSpeed_ui16(UInt16 f_vWICspeed_ui16,
                               // UInt16 f_prevWICspeed_ui16,
                               UInt16 f_vWheelspeed_ui16,
                               // UInt16 f_prevWhelspeed_ui16,
                               UInt16 f_vCANspeed_ui16,
                               // UInt16 f_prevCANspeed_ui16
                               UInt16 f_prevVehSpeed_ui16);

void g_InitVehicleSpeed_vd(void);
void g_InitVehicleSpeedCalcWICbuf_vd(gType_vhsWICbuf_st *f_vhsWICbuf_pst);
UInt16 g_calculateWICspeed_ui16(UInt16 f_WICvalue_ui16, UInt16 f_WIC_CanTime_ui16,
                                gType_vhsWICbuf_st *g_vhsWICbuf_pst);
UInt16 g_calculateWheelSpeed_ui16(UInt16 f_WheelrotationValue_ui16);
#endif

#endif // COMPONENT_VHS                                     /*!\endcond       */

#endif /* VHS_H */

/*========================= EoF (vhs.h) ========================*/
