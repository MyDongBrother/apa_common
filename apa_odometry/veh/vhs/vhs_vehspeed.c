/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: vhs_vehspeed.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/
#define COMPONENT_VHS

#include <pa_components/base_type/global_include.h>
#include <pa_components/mathlib/mathlib_api.h>
#include <pa_components/parameter/par_api.h>

// #include <sp_fsv_api.h> // must be defined before vhs_api.h (only required
//  within COMPONENT_VHS)
#include "vhs.h"
#include "vhs_api.h"

// #include <par_api.h>
#include <string.h>

#ifndef LS_VHS_CAN_SPEED_SUBST
#error ('switch LS_VHS_CAN_SPEED_SUBST is not defined')
#endif

#if (LS_VHS_CAN_SPEED_SUBST != SW_OFF)

#define ld_VeloSubstFilterCyclesPerOneTenthKmh_ui16 ((UInt16)10)

typedef enum
{
    VHSVeloSubstCAN_enm,
    VHSVeloSubstWIC_enm,
    VHSVeloSubstWheel_enm
} gType_vhsVelocitySubstSelection_en;

static gType_vhsVelocitySubstSelection_en ls_LastVeloSubstMode_en;
static Boolean ls_VeloSubstFilteringActive_bl;
static SInt16 ls_VeloSubstFilterLastDelta_si16;
static UInt16 ls_VeloSubstFilterCycleCounter_ui16;

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief  caculate vehicle speed.
 * @details
 *
 * @param         f_vWICspeed_ui16
 * @param         f_vWheelspeed_ui16
 * @param         f_vCANspeed_ui16
 * @param         f_prevVehSpeed_ui16
 * @return        UInt16
 *
 * @note
 * @see
 * @warning
 */
UInt16 g_CalcVehicleSpeed_ui16(UInt16 f_vWICspeed_ui16, UInt16 f_vWheelspeed_ui16,
                               UInt16 f_vCANspeed_ui16, UInt16 f_prevVehSpeed_ui16)
{
    UInt16 l_vVeh_ui16;
    UInt16 l_VeloUnfiltered_ui16;
    gType_vhsVelocitySubstSelection_en l_VeloSubstMode_en;

    l_VeloUnfiltered_ui16 = gd_VeloInvalid_ui16;
#if 1
    // Highestpriority for Wheel velocity, as most accurate solution.
    if ((f_vWheelspeed_ui16 != gd_VeloInvalid_ui16) && (f_vWheelspeed_ui16 > 0))
    {
        l_VeloUnfiltered_ui16 = f_vWheelspeed_ui16;
        l_VeloSubstMode_en    = VHSVeloSubstWheel_enm;
    }
    else
    {
        // Second priority for CAN velocity, as inaccurate but smooth solution.
        if (f_vCANspeed_ui16 > 0)
        {
            l_VeloUnfiltered_ui16 = f_vCANspeed_ui16;
            l_VeloSubstMode_en    = VHSVeloSubstCAN_enm;
        }
        else
        {
            // Last Solution WIC Speed
            l_VeloUnfiltered_ui16 = f_vWICspeed_ui16;
            l_VeloSubstMode_en    = VHSVeloSubstWIC_enm;
        }
    }
#else
    // Due to the large error between the current wheel speed and the actual
    // speed,
    // three speed priorities are adjusted.Wic_speed had the highest priority,
    // can_speed followed, and wheel_speed had the lowest.
    if ((f_vWheelspeed_ui16 != gd_VeloInvalid_ui16) && (f_vWICspeed_ui16 > 0)) // sinkwang
    {
        l_VeloUnfiltered_ui16 = f_vWICspeed_ui16;
        l_VeloSubstMode_en    = VHSVeloSubstWIC_enm;
    }
    else
    {
        if (f_vCANspeed_ui16 > 0)
        {
            l_VeloUnfiltered_ui16 = f_vCANspeed_ui16;
            l_VeloSubstMode_en    = VHSVeloSubstCAN_enm;
        }
        else
        {
            l_VeloUnfiltered_ui16 = f_vWheelspeed_ui16;
            l_VeloSubstMode_en    = VHSVeloSubstCAN_enm;
        }
    }
#endif

    // Fallback to CAN-Speed if substitution fails
    if (l_VeloUnfiltered_ui16 == gd_VeloInvalid_ui16)
    {
        l_VeloUnfiltered_ui16 = f_vCANspeed_ui16;
        l_VeloSubstMode_en    = VHSVeloSubstCAN_enm;
    }

    if ((ls_LastVeloSubstMode_en != l_VeloSubstMode_en) && (f_prevVehSpeed_ui16 != 0))
    {
        // Signalquelle wurde ge�ndert.
        // Filterung muss aktiviert werden um Spr�nge zu verhindern.
        ls_VeloSubstFilteringActive_bl      = TRUE;
        ls_VeloSubstFilterCycleCounter_ui16 = ld_VeloSubstFilterCyclesPerOneTenthKmh_ui16;
        ls_VeloSubstFilterLastDelta_si16 =
            (SInt16)l_VeloUnfiltered_ui16 - (SInt16)f_prevVehSpeed_ui16;
    }

    ls_LastVeloSubstMode_en = l_VeloSubstMode_en;

    // Use filtered or Raw value
    if (ls_VeloSubstFilteringActive_bl != FALSE)
    {
        if (((SInt16)l_VeloUnfiltered_ui16 - ls_VeloSubstFilterLastDelta_si16) > 0)
        {
            l_vVeh_ui16 = (UInt16)((SInt16)l_VeloUnfiltered_ui16 -
                                   ls_VeloSubstFilterLastDelta_si16);
        }
        else
        {
            l_vVeh_ui16                      = 0;
            ls_VeloSubstFilterLastDelta_si16 = (SInt16)l_VeloUnfiltered_ui16;
        }
    }
    else
    {
        l_vVeh_ui16 = l_VeloUnfiltered_ui16;
    }

    // Decrement Filteroffset
    if (ls_VeloSubstFilterLastDelta_si16 != 0)
    {
        if (ls_VeloSubstFilterCycleCounter_ui16 > 0)
        {
            ls_VeloSubstFilterCycleCounter_ui16--;
        }
        else
        {
            ls_VeloSubstFilterCycleCounter_ui16 =
                ld_VeloSubstFilterCyclesPerOneTenthKmh_ui16;
            ls_VeloSubstFilterLastDelta_si16 -=
                g_mtl_Signum_mac(ls_VeloSubstFilterLastDelta_si16);
        }
    }
    else
    {
        // Filtering is done Filter can be deactivated.
        ls_VeloSubstFilteringActive_bl = FALSE;
    }
    // Filterung wird im Stillstand schnell abgesenkt.
    if (l_VeloUnfiltered_ui16 == 0)
    {
        ls_VeloSubstFilterLastDelta_si16 /= 2;
    }

    return l_vVeh_ui16;
}
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief  init Wic buffer about vehicle speed.
 * @details
 *
 * @param         f_vhsWICbuf_pst, wic buffer data.
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_InitVehicleSpeedCalcWICbuf_vd(gType_vhsWICbuf_st *f_vhsWICbuf_pst)
{
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 0506
    // Msg(2:0506) Dereferencing pointer value that is possibly NULL.
#endif

    (void)memset(f_vhsWICbuf_pst, 0x00, sizeof(gType_vhsWICbuf_st));
    f_vhsWICbuf_pst->WICdeltaIndex_ui8  = (UInt8)(gd_WICindexMax_ui8 - 1);
    f_vhsWICbuf_pst->InitPhase_bl       = TRUE;
    f_vhsWICbuf_pst->WICVeloLast_ui16   = gd_VeloInvalid_ui16;
    ls_LastVeloSubstMode_en             = VHSVeloSubstCAN_enm;
    ls_VeloSubstFilteringActive_bl      = FALSE;
    ls_VeloSubstFilterLastDelta_si16    = 0;
    ls_VeloSubstFilterCycleCounter_ui16 = 0;

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 0506
#endif
    return;
}
/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/

/**
 * @brief  Init vehicle speed variable.
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_InitVehicleSpeed_vd(void)
{
    ls_LastVeloSubstMode_en             = VHSVeloSubstCAN_enm;
    ls_VeloSubstFilteringActive_bl      = FALSE;
    ls_VeloSubstFilterLastDelta_si16    = 0;
    ls_VeloSubstFilterCycleCounter_ui16 = 0;
    return;
}

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief calculate wic speed,
 * @details
 *
 * @param         f_WICvalue_ui16 , wic value
 * @param         f_WIC_CanTime_ui16, wic can time
 * @param         g_vhsWICbuf_pst  wic buffer struct.
 * @return        wic speed
 *
 * @note
 * @see
 * @warning
 */
UInt16 g_calculateWICspeed_ui16(UInt16 f_WICvalue_ui16, UInt16 f_WIC_CanTime_ui16,
                                gType_vhsWICbuf_st *g_vhsWICbuf_pst)
{
    UInt16 l_WICspeed_ui16        = gd_VeloInvalid_ui16;
    UInt32 l_WICdeltaSUM_ui32     = 0;
    UInt32 l_CANtimedeltaSUM_ui32 = 0;
    UInt8 l_TempCount_ui8;
    UInt8 l_Counter_ui8;

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 0506
    // Msg(2:0506) Dereferencing pointer value that is possibly NULL.
#endif

    if (f_WIC_CanTime_ui16 == g_vhsWICbuf_pst->CanTime_ui16)
    {
        // WIC timestamp did not change, nothing to be calculated
        l_WICspeed_ui16 = g_vhsWICbuf_pst->WICVeloLast_ui16;
    }
    else // WIC timestamp is NOT identical
    {
        // check for initialization phase (speed can only be calculated, if 2 valid
        // values are present)
        if (g_vhsWICbuf_pst->InitPhase_bl != FALSE)
        {
            g_vhsWICbuf_pst->WICBuf_ui16  = f_WICvalue_ui16;
            g_vhsWICbuf_pst->CanTime_ui16 = f_WIC_CanTime_ui16;
            g_vhsWICbuf_pst->InitPhase_bl = FALSE;
        }
        else
        {
            const UInt16 lc_MaxCanDeltaMS_ui16 = (UInt16)2000;

            // check if index reaches array bounds
            if (g_vhsWICbuf_pst->WICdeltaIndex_ui8 < (UInt8)(gd_WICindexMax_ui8 - 1))
            {
                // increment index
                g_vhsWICbuf_pst->WICdeltaIndex_ui8++;
            }
            else
            {
                // array bound reached -> reset index
                g_vhsWICbuf_pst->WICdeltaIndex_ui8 = 0;
            }

            g_vhsWICbuf_pst->CanDelta_pui16[g_vhsWICbuf_pst->WICdeltaIndex_ui8] =
                f_WIC_CanTime_ui16 - g_vhsWICbuf_pst->CanTime_ui16;

            // store the current CAN timestamp for next cycle
            g_vhsWICbuf_pst->CanTime_ui16 = f_WIC_CanTime_ui16;

            if (g_vhsWICbuf_pst->CanDelta_pui16[g_vhsWICbuf_pst->WICdeltaIndex_ui8] >
                lc_MaxCanDeltaMS_ui16)
            {
                // implausible change of can time; reinitialize the buffers
                g_InitVehicleSpeedCalcWICbuf_vd(g_vhsWICbuf_pst);
            }
            else
            {
                // CALCULATE WIC INCREMENTS OF THE CURRENT CYCLE
                if (f_WICvalue_ui16 >= g_vhsWICbuf_pst->WICBuf_ui16)
                {
                    // wic has been incremented without wrap around
                    g_vhsWICbuf_pst->WICdelta_pui16[g_vhsWICbuf_pst->WICdeltaIndex_ui8] =
                        f_WICvalue_ui16 - g_vhsWICbuf_pst->WICBuf_ui16;
                }
                else //(f_WICvalue_ui16 < g_vhsWICbuf_pst->WICBuf_ui16)
                {
                    // wic has been incremented, wrap around occured
                    g_vhsWICbuf_pst->WICdelta_pui16[g_vhsWICbuf_pst->WICdeltaIndex_ui8] =
                        (UInt16)((1 + f_WICvalue_ui16 +
                                  g_parGetParaVHO_Veh_MaxWICValue_ui16) -
                                 g_vhsWICbuf_pst->WICBuf_ui16);
                }
                //        g_vhsWICbuf_pst->WICTemp1_pui16[g_vhsWICbuf_pst->WICdeltaIndex_ui8]
                //        = g_vhsWICbuf_pst->WICBuf_ui16;
                //        g_vhsWICbuf_pst->WICTemp2_pui16[g_vhsWICbuf_pst->WICdeltaIndex_ui8]
                //        = f_WICvalue_ui16;

                // store current WIC value for next cycle
                g_vhsWICbuf_pst->WICBuf_ui16 = f_WICvalue_ui16;
                // calculate WIC speed in km/h
                for (l_Counter_ui8 = 0; l_Counter_ui8 < gd_WICindexMax_ui8;
                     l_Counter_ui8++)
                {
                    // check for index overflow
                    if (g_vhsWICbuf_pst->WICdeltaIndex_ui8 >= l_Counter_ui8)
                    {
                        l_TempCount_ui8 =
                            g_vhsWICbuf_pst->WICdeltaIndex_ui8 - l_Counter_ui8;
                    }
                    else
                    {
                        l_TempCount_ui8 = (UInt8)((g_vhsWICbuf_pst->WICdeltaIndex_ui8 +
                                                   gd_WICindexMax_ui8) -
                                                  l_Counter_ui8);
                    }

                    l_WICdeltaSUM_ui32 = l_WICdeltaSUM_ui32 +
                                         g_vhsWICbuf_pst->WICdelta_pui16[l_TempCount_ui8];
                    l_CANtimedeltaSUM_ui32 =
                        l_CANtimedeltaSUM_ui32 +
                        g_vhsWICbuf_pst->CanDelta_pui16[l_TempCount_ui8];
                }
                // check for zero time (should not occure)
                if (l_CANtimedeltaSUM_ui32 == 0)
                {
                    l_CANtimedeltaSUM_ui32 = 1;
                }

                // calculated WIC speed
                l_WICspeed_ui16                   = (UInt16)((l_WICdeltaSUM_ui32 *
                                            g_parGetParaVHO_Veh_WICLength_ui16 * 36) /
                                           (l_CANtimedeltaSUM_ui32 * 256));
                g_vhsWICbuf_pst->WICVeloLast_ui16 = l_WICspeed_ui16;
            }
        }
    }
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 0506
#endif
    return l_WICspeed_ui16;
}

/*****************************************************************************
F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief  cacluate the wheel speed,
 * @details
 *
 * @param         f_WheelrotationValue_ui16, wheel rotation value
 * @return        wheel speed
 *
 * @note
 * @see
 * @warning
 */
UInt16 g_calculateWheelSpeed_ui16(UInt16 f_WheelrotationValue_ui16)
{
    const UInt32 lc_WheelCircumferenceMM_ui32 =
        ((((UInt32)g_parGetParaVHO_Veh_WICLength_ui16) *
          ((UInt32)g_parGetParaVHO_Veh_WICTeethCount_ui16)) >>
         8);

    UInt32 l_VehCalc_ui32;
    //   lc_WheelCircumferenceMM_ui32 * 3.6 * 100   lc_WheelCircumferenceMM_ui32 *
    //   3
    //   --------------------------------------- =
    //   --------------------------------------- 2 * 10000 * 60 10000

    l_VehCalc_ui32 =
        (((UInt32)f_WheelrotationValue_ui16 * lc_WheelCircumferenceMM_ui32 * (UInt32)3) /
         (UInt32)10000);

    return (UInt16)l_VehCalc_ui32;
}
#endif // (LS_VHS_CAN_SPEED_SUBST == SW_ON)

/*!\endcond */
/*!@}*/ /*================== EoF (vhs_speedcalc.c) ===========*/
