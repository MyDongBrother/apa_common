/*****************************************************************************
 | C O P Y R I G H T
 |-----------------------------------------------------------------------------
 | Copyright (c) 2023 by Forvision-tech.                All rights reserved.
 |
 | This file is property of Forvision-tech. Any unauthorised copy, use or
 | distribution is an offensive act against international law and may me
 | prosecuted under federal law. Its content is company confidential.
 |-----------------------------------------------------------------------------
 | D E S C R I P T I O N
 |-----------------------------------------------------------------------------
 | $ProjectName:
 d:/MKS_Data/Projects/PPJ/PP_GEN5/PP_SIP5_Fx4/SW/SRC/PA/DFS_PRJ/SRC/TOOLS/PARGen/pc_pp_pargen.pj
 $ |      $Source: Program.cs $ |    $Revision: 1.220 $ |        $Date: 2012/11/12
 16:37:54CST $ |       $State: in_work $
 |
 |      Purpose: generated component configuration file
 |-----------------------------------------------------------------------------
 | I N F O
 |-----------------------------------------------------------------------------
 | File:                  gen_par_main.c
 | Input File:            ..\appl_sip_param.xml
 | Version:               5.8
 | Date:                  2023.5.11 - 15:12
 |
 | Generation Date (UTC): 2023.05.11 - 07:13
 *****************************************************************************/
/* clang-format off */
/*--------------------------------------------------------------------------*/
/*- magic number                                                           -*/
/*--------------------------------------------------------------------------*/

#define GEN_PAR_MAIN_MAGIC_NUMBER  0x05110713

/*--------------------------------------------------------------------------*/
/*- disabled QAC warnings in this module                                   -*/
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*- component definition                                                   -*/
/*--------------------------------------------------------------------------*/

#define COMPONENT_PAR

/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/

#include <pa_components/base_type/global_include.h>


#include <pa_components/parameter/par_api.h>

/*--------------------------------------------------------------------------*/
/*- check if compiler switches (utilised in this component) are defined    -*/
/*--------------------------------------------------------------------------*/

#if ( GEN_PAR_MAIN_MAGIC_NUMBER != GEN_PAR_SWITCHES_MAGIC_NUMBER )
  #error ("Wrong version of 'gen_par_switches.h' - check whether the generated files match!")
#endif

#if ( GEN_PAR_MAIN_MAGIC_NUMBER != GEN_PAR_HEADER_MAGIC_NUMBER )
  #error ("Wrong version of 'gen_par_header.h' - check whether the generated files match!")
#endif

/*--------------------------------------------------------------------------*/
/*- local symbolic constants                                               -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- local macros                                                           -*/
/*--------------------------------------------------------------------------*/


// macro to convert 'define' to a quoted string
// g_QUOTE_CAL2_mac is used to encapsulate #CAL_IDstring with brackets (#CAL_IDstring)
#define g_QUOTE_CAL2_mac(CAL_IDstring)  #CAL_IDstring
#define g_QUOTE_CAL_mac(CAL_IDstring)   g_QUOTE_CAL2_mac(CAL_IDstring)


/*--------------------------------------------------------------------------*/
/*- local data types                                                       -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- local data                                                             -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- prototypes of local functions                                          -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- global data objects                                                    -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- RAM                                                                    -*/
/*--------------------------------------------------------------------------*/

//
// global data RAM
//

#if ( (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_STAND_ALONE) || (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_APPL_ONLY) )
  gType_parRAMData_st g_parRAMData_st;
#elif (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_CAL_ONLY)
  // not included in this configuration
#else
  #error 'invalid configuration!'
#endif


//
// application memory
//

#if ( GS_PAR_CFG_APPLICATION_MODE == SW_ON )

  #if ( (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_STAND_ALONE) || (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_APPL_ONLY) )
    gType_parApplicationMemory_st g_parApplicationMemory_st;
  #elif (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_CAL_ONLY)
    // not included in this configuration
  #else
    #error 'invalid configuration!'
  #endif

#endif

/*--------------------------------------------------------------------------*/
/*- ROM                                                                    -*/
/*--------------------------------------------------------------------------*/

//
// Block Status
//

#if ( (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_STAND_ALONE) || (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_CAL_ONLY) )

//#pragma ghs section rodata = ".CAL_BlockStatus"

const UInt8 gc_parBlockStatus_pui8[256] =
{
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

//#pragma ghs section rodata = ".pa_const"

#endif


//
// CAL Header Low
//

#if ( (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_STAND_ALONE) || (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_CAL_ONLY) )

//#pragma ghs section rodata = ".CAL_HdrLow"

//#pragma ghs section rodata = ".pa_const"

#endif


//
// CAL Header High
//

#if ( (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_STAND_ALONE) || (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_CAL_ONLY) )

//#pragma ghs section rodata = ".CAL_HdrHigh"

//#pragma ghs section rodata = ".pa_const"

#endif


//
// global data ROM
//

#if ( (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_STAND_ALONE) || (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_CAL_ONLY) )


//#define PAR_START_SEC_REF
//#include "par_mem_map.h"
//#pragma ghs section rodata = ".CAL_Const"

gType_parROMData_st g_parMemoryData_st;
const gType_parROMData_st gc_parROMData_st =
{
  //
  // version information
  //

  {
    {'C', 'A', 'L', '-', 'S', 'e', 'c', 't', 'i', 'o', 'n', 'S', 't', 'a', 'r', 't'},
    g_QUOTE_CAL_mac(gd_PAR_VersionInfo_pc),
    gd_PAR_CAL_StructureIntegrityHash_pui16,
    0x2023,
    0x05,
    0x11,
    0x15,
    0x12,
    {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    }
  },

  //
  // parameter memory ROM
  //

  {
    // cluster: VHOSRC_PluginSRC
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        -150, // accelMaxWhileBreaking_si16
        150, // accelMinWhileAcceleration_si16
        2, // normalWeightFront_ui8
        1, // normalWeightRear_ui8
        1, // accelWeightFront_ui8
        0, // accelWeightRear_ui8
        0, // decelWeightFront_ui8
        1, // decelWeightRear_ui8
        500, // dynStateHoldTime_ui16
        1200, // distanceMaxABSintervention_ui16
        1000, // distanceMaxASRintervention_ui16
        500 // distanceMaxESPintervention_ui16
      }
    },
    // cluster: VHOTCE_PluginTTC
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        1000, // VeloMax_ui16
        1000, // DrivingDist_ui32
        45740, // MaxAngle_si32
        1250, // SWAMax_ui16
        25 // NumberOfSamples_ui8
      }
    },
    // cluster: VHOTCE_PluginTCE
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        500, // TargetForQualityCummulation_ui16
        50, // VeloMin_ui16
        400, // VeloMax_ui16
        180, // LongACCMax_ui16
        1250, // SWAMin_ui16
        15000, // SWAMax_ui16
        731840, // YawangleFinish_ui32
        731840, // YawangleMax_ui32
        4574, // YawangleWICError_ui32
        150, // ToleratedVelocityDiff_ui16
        4500, // ToleratedSWADiff_ui16
        500, // YawrateMin_ui16
        6000 // YawrateMax_ui16
      }
    },
    // cluster: VHOTCE_PluginTCE_clustering
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        25, // ProcentualCenteringOfCluster_ui8
        100, // ThresholdForTirechange_ui16
        3, // NumberOfClusters_ui8
        // WICLengthOfCluster_pui16[10]
        {
          5070, 5235, 5321, 5120, 5120, 5120, 5120, 5120, 5120, 5120
        },
        // WheelBaseOfCluster_pui16[10]
        {
          15450, 15330, 15700, 19950, 19950, 19950, 19950, 19950, 19950, 19950
        }
      }
    },
    // cluster: AP_PS_Config
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        0, // SpeedLimitBrakingPSX_bl
        0, // ParkEmergencyBraking_bl
        0, // ManeuverEmergencyBraking_bl
        0, // ShortenedDrivingTube_bl
        1, // CloseObjectDetectedBraking_bl
        1 // BrakeForHighObjectsOnly_bl
      }
    },
    // cluster: AP_PS_Speed
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        0, // DeactivationLow_ui8
        110, // DeactivationHigh_ui8
        100, // FirstClothoidShift_ui8
        50, // LimitStraight_ui8
        40, // LimitSteering1_ui8
        30, // LimitSteering2_ui8
        30, // LimitNMove_ui8
        30, // LimitCPSCBackStraight_ui8
        10, // LimitCPSCBack_ui8
        20, // LimitCPSCForw_ui8
        20, // LimitFwdCPSC_ui8
        10, // LimitHysteresis_ui8
        40, // MbaC_Speedlimit1_ui8
        30, // MbaC_Speedlimit2_ui8
        20, // MbaC_Speedlimit3_ui8
        10, // MbaC_Speedlimit4_ui8
        50 // MbaC_ActiveNoObject_ui8
      }
    },
    // cluster: AP_PS_Brake
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        5, // HoldTime_ui8
        6552, // TorqueSpeedLimiting_ui16
        8190, // TorqueCollision_ui16
        40, // AccelerationSpeedLimiting_ui8
        120, // AccelerationCollision_ui8
        10, // DeadTime_ui8
        14, // MaxJerk_ui8
        1, // Mult_ui8
        0, // Add_ui8
        0, // TimeToCollision1_ui8
        0, // TimeToCollision2_ui8
        0, // ComfortableDecelMaximum_ui16
        0, // CloseObjectDecelMaximum_ui16
        0, // MaximumEngineJerk_si16
        0, // MaximumDecelerationJerk_si16
        -700, // MaxEngJerkCloseObject_si16
        -700, // MaxDecelJerkCloseObject_si16
        20, // ESP_MinControlDist_ui8
        0, // OocSafeDistPCB_ui8
        10, // CreepModusMinDist_ui8
        20, // SafetyDistVehicle_ui8
        50, // NextMoveRequestDistancePar_ui8
        220, // NextMoveRequestDistancePerp_ui16
        50, // ComfStopBrakeFactor_ui8
        1, // StopDist_1_ui8
        14, // StopDist_2_ui8
        28, // StopDist_3_ui8
        43, // StopDist_4_ui8
        59, // StopDist_5_ui8
        76, // StopDist_6_ui8
        94, // StopDist_7_ui8
        113, // StopDist_8_ui8
        133, // StopDist_9_ui8
        155, // StopDist_10_ui8
        177, // StopDist_11_ui8
        1, // StopDistMEB_1_ui8
        14, // StopDistMEB_2_ui8
        28, // StopDistMEB_3_ui8
        43, // StopDistMEB_4_ui8
        59, // StopDistMEB_5_ui8
        76, // StopDistMEB_6_ui8
        94, // StopDistMEB_7_ui8
        113, // StopDistMEB_8_ui8
        133, // StopDistMEB_9_ui8
        155, // StopDistMEB_10_ui8
        177 // StopDistMEB_11_ui8
      }
    },
    // cluster: AP_PS_Objects
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        50, // MinExistProbability_ui8
        200, // MinExistProbabilityDECEonly_ui8
        10, // MinimumDistance_ui8
        1700, // MbaC_ObjDist1_ui16
        1300, // MbaC_ObjDist2_ui16
        800, // MbaC_ObjDist3_ui16
        500, // MbaC_ObjDist4_ui16
        400 // MAA_Active_ObjDist_ui16
      }
    },
    // cluster: BDA_Parameter
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        1, // SetAllSensorsToNotBlind_bl
        8127, // RelevantSensorsBitMask_ui16
        1, // TriggerBlindnessTestAtInit_bl
        8640000, // Risk_TimeOfNoObject_ui32
        60000, // Risk_TimeOfBlindSensor_ui32
        8097, // vDependingSetBlindMask1_ui16
        25, // vDependingThreshold1_ui8
        4257, // vDependingSetBlindMask2_ui16
        250, // vDependingThreshold2_ui8
        5, // MinEchoDistance_ui16
        450, // MaxEchoDistance_ui16
        80, // MinGroundEchoDistance_ui16
        200, // MaxGroundEchoDistance_ui16
        4100, // MaxEchoWidth_ui16
        100, // DurationOfBlindnessTest_ui16
        5, // MaxTempForBlindnessTest_si8
        8640000 // UnknownStatusMaxWaitTime_ui32
      }
    },
    // cluster: PSD_Veh
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        302, // FrontSensorXDist_ui16
        87, // FrontSensorYDist_ui16
        80, // FrontSensorAngle_ui8
        -40, // RearSensorXDist_si16
        86, // RearSensorYDist_ui16
        90, // RearSensorAngle_ui8
        0, // AdditionalPSLengthFOV_si8
        0, // DisplacementOfPS_si8
        525, // PSLengthMin_ui16
        800, // PSLengthMax_ui16
        190 // PSDepthDefault_ui16
      }
    },
    // cluster: PSD_Proj
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        143, // AOIMinDist_ui16
        350, // AOIMaxDist_ui16
        143, // LatDist_Min_ui16
        200, // PSDistMax_ui16
        150, // MergeObst_DistMax_ui16
        30 // MergeObst_SafetyDist_si16
      }
    },
    // cluster: PSD_Proj_OCP
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        30, // MinCurbDepth_ui16
        143, // MaxCurbDepth_ui16
        300, // MinCurbLength_ui16
        50, // OCP2SafetyDistToCurb_ui16
        30 // OCP4SafetyDistToCurb_ui16
      }
    },
    // cluster: PSD_Proj_Cross
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        180, // PSLengthMin_ui16
        650, // PSLengthMax_ui16
        150, // LatDiffMax_ui16
        320, // PassingDistMax_ui16
        450, // PSLengthVCType1_ui16
        450, // PSLengthVCType2_ui16
        330 // PSLengthVCType1Shortened_ui16
      }
    },
    // cluster: PSD_RuleSet1
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        50, // Normal_OffsetToPSDat_Max_si16
        50, // OffsetOnStreetLat_Max_si16
        25, // Normal_SafeDistToObst_Lat_si16
        15, // Normal_SafeDistToCurb_si16
        40, // OCP2_Safe_AOI_Obj2_Max_si16
        10, // OCP2_Safe_AOI_Obj2_Min_si16
        50, // OCP2_SafeDistToCurb_Lat_si16
        30, // OCP4_SafeDistToCurb_Lat_si16
        30, // Normal_SafeDistToObst_Long_si16
        250 // MinLengthForPlausObject_ui16
      }
    },
    // cluster: PSD_Proj_VC
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        300, // MinCurbLength_ui16
        800, // MinPSLength_ui16
        1200, // MaxPSDength_ui16
        150, // MinPSDepth_ui16
        250, // MaxPSDepth_ui16
        600 // MinPSLengthShortened_ui16
      }
    },
    // cluster: PSD_Proj_SPR
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        500, // OperatingUSDist_ui16
        100 // MinWayForUpdate_ui16
      }
    },
    // cluster: MP_USS_FISelection
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FI_Std
      {
        0, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      }
    },
    // cluster: MP_USS_FIBlock1
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FI_Std
      {
        // DataBlock1_pui8[10]
        {
          132, 37, 75, 226, 78, 82, 16, 128, 105, 192
        }
      }
    },
    // cluster: MP_USS_FIBlock2
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FI_Std
      {
        // DataBlock2_pui8[5]
        {
          118, 118, 118, 53, 174
        }
      }
    },
    // cluster: MP_USS_RISelection
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RI_Std
      {
        0, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      }
    },
    // cluster: MP_USS_RIBlock1
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RI_Std
      {
        // DataBlock1_pui8[10]
        {
          132, 37, 75, 226, 78, 82, 16, 128, 105, 192
        }
      }
    },
    // cluster: MP_USS_RIBlock2
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RI_Std
      {
        // DataBlock2_pui8[5]
        {
          118, 118, 118, 53, 174
        }
      }
    },
    // cluster: MP_USS_FOSelection
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_TEMP0
      {
        0, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_TEMP1
      {
        1, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_TEMP2
      {
        2, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_TEMP3
      {
        3, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_TEMP4
      {
        4, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_TEMP5
      {
        5, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_TEMP6
      {
        6, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_TEMP7
      {
        7, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_TEMP8
      {
        8, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_TEMP9
      {
        9, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA1
      {
        10, // CcSetBl1Id_ui8
        1 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA2
      {
        11, // CcSetBl1Id_ui8
        1 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA3
      {
        12, // CcSetBl1Id_ui8
        1 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA4
      {
        13, // CcSetBl1Id_ui8
        1 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA5
      {
        14, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA6
      {
        15, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA7
      {
        16, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA8
      {
        17, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA9
      {
        18, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA10
      {
        19, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA11
      {
        20, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA12
      {
        21, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA13
      {
        22, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA14
      {
        23, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA15
      {
        24, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA16
      {
        25, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA17
      {
        26, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA18
      {
        27, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVA19
      {
        28, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: FO_SVABS
      {
        29, // CcSetBl1Id_ui8
        3 // CcSetBl2Id_ui8
      }
    },
    // cluster: MP_USS_FOBlock1
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 0_FO_TEMP0
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 1_FO_TEMP1
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 2_FO_TEMP2
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 3_FO_TEMP3
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 4_FO_TEMP4
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 5_FO_TEMP5
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 6_FO_TEMP6
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 7_FO_TEMP7
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 8_FO_TEMP8
      {
        // DataBlock1_pui8[10]
        {
          132, 47, 173, 246, 244, 130, 144, 64, 107, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 9_FO_TEMP9
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 10_FO_SVA1
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 11_FO_SVA2
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 12_FO_SVA3
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 13_FO_SVA4
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 14_FO_SVA5
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 15_FO_SVA6
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 16_FO_SVA7
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 17_FO_SVA8
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 18_FO_SVA9
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 19_FO_SVA10
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 20_FO_SVA11
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 21_FO_SVA12
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 22_FO_SVA13
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 23_FO_SVA14
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 24_FO_SVA15
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 25_FO_SVA16
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 26_FO_SVA17
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 27_FO_SVA18
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 28_FO_SVA19
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 29_FO_SVABS
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      }
    },
    // cluster: MP_USS_FOBlock2
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 0_FO_0
      {
        // DataBlock2_pui8[5]
        {
          118, 118, 118, 53, 174
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 1_FO_SVA1
      {
        // DataBlock2_pui8[5]
        {
          118, 118, 118, 53, 174
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 2_FO_SVA2
      {
        // DataBlock2_pui8[5]
        {
          118, 118, 118, 53, 174
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 3_FO_SVA3
      {
        // DataBlock2_pui8[5]
        {
          118, 118, 118, 53, 174
        }
      }
    },
    // cluster: MP_USS_ROSelection
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_TEMP0
      {
        0, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_TEMP1
      {
        1, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_TEMP2
      {
        2, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_TEMP3
      {
        3, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_TEMP4
      {
        4, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_TEMP5
      {
        5, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_TEMP6
      {
        6, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_TEMP7
      {
        7, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_TEMP8
      {
        8, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_TEMP9
      {
        9, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA1
      {
        1, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA2
      {
        2, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA3
      {
        3, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA4
      {
        4, // CcSetBl1Id_ui8
        0 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA5
      {
        5, // CcSetBl1Id_ui8
        1 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA6
      {
        6, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA7
      {
        7, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA8
      {
        8, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA9
      {
        9, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA10
      {
        10, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA11
      {
        11, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA12
      {
        12, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA13
      {
        13, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA14
      {
        14, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA15
      {
        15, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA16
      {
        16, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA17
      {
        17, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA18
      {
        18, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVA19
      {
        19, // CcSetBl1Id_ui8
        2 // CcSetBl2Id_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: RO_SVABS
      {
        20, // CcSetBl1Id_ui8
        3 // CcSetBl2Id_ui8
      }
    },
    // cluster: MP_USS_ROBlock1
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 0_RO_TEMP0
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 1_RO_TEMP1
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 2_RO_TEMP2
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 3_RO_TEMP3
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 4_RO_TEMP4
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 5_RO_TEMP5
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 6_RO_TEMP6
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 7_RO_TEMP7
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 8_RO_TEMP8
      {
        // DataBlock1_pui8[10]
        {
          132, 47, 173, 246, 244, 130, 144, 64, 107, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 9_RO_TEMP9
      {
        // DataBlock1_pui8[10]
        {
          132, 49, 190, 114, 243, 114, 142, 64, 104, 128
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 10_RO_SVA1
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 11_RO_SVA2
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 12_RO_SVA3
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 13_RO_SVA4
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 14_RO_SVA5
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 15_RO_SVA6
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 16_RO_SVA7
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 17_RO_SVA8
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 18_RO_SVA9
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 19_RO_SVA10
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 20_RO_SVA11
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 21_RO_SVA12
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 22_RO_SVA13
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 23_RO_SVA14
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 24_RO_SVA15
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 25_RO_SVA16
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 26_RO_SVA17
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 27_RO_SVA18
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 28_RO_SVA19
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 29_RO_SVABS
      {
        // DataBlock1_pui8[10]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      }
    },
    // cluster: MP_USS_ROBlock2
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 0_RO_0
      {
        // DataBlock2_pui8[5]
        {
          118, 118, 118, 53, 174
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 1_RO_SVA1
      {
        // DataBlock2_pui8[5]
        {
          118, 118, 118, 53, 174
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 2_RO_SVA2
      {
        // DataBlock2_pui8[5]
        {
          118, 118, 118, 53, 174
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: 3_RO_SVA3
      {
        // DataBlock2_pui8[5]
        {
          118, 118, 118, 53, 174
        }
      }
    },
    // cluster: MP_USS_CcSetInfo
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        10, // NumCcSets_ui8
        6, // DefaultIdx_ui8
        // TempRange_psi8[10]
        {
          -7, -2, 2, 6, 10, 17, 24, 31, 40, 85
        },
        2 // TempHyst_si8
      }
    },
    // cluster: APCTL_CFG
    {
      // global var(s): DAI_BR245 | min var: PSC_BSD | sub var: not labeled
      {
        18, // VCutOffPAS_accelerate_ui8
        2, // VCutOffPAS_hysteresis_ui8
        35, // VCutOffPSX_accelerate_ui8
        1, // VCutOffPSX_hysteresis_ui8
        200, // BSDCutoffSpeed_ui8
        5, // BSDCutoffSpeedHys_ui8
        18, // BSDMinVelocityAc_ui8
        16, // BSDMinVelocityDe_ui8
        10, // GuidanceVmaxExceeded_ui8
        5, // VStandStillCutOffPSX_ui8
        -2250, // SWAMinOMA_si16
        2250 // SWAMaxOMA_si16
      },
      // global var(s): DAI_BR245 | min var: BSD | sub var: not labeled
      {
        18, // VCutOffPAS_accelerate_ui8
        2, // VCutOffPAS_hysteresis_ui8
        41, // VCutOffPSX_accelerate_ui8
        4, // VCutOffPSX_hysteresis_ui8
        200, // BSDCutoffSpeed_ui8
        5, // BSDCutoffSpeedHys_ui8
        2, // BSDMinVelocityAc_ui8
        0, // BSDMinVelocityDe_ui8
        10, // GuidanceVmaxExceeded_ui8
        5, // VStandStillCutOffPSX_ui8
        -2250, // SWAMinOMA_si16
        2250 // SWAMaxOMA_si16
      }
    },
    // cluster: DISPLAY_Veh
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        // DurationTone_pui8[2]
        {
          200, 200
        },
        // DurationNoTone_pui8[2]
        {
          200, 200
        },
        // ToneSetup_pui8[2]
        {
          10, 2
        },
        // RangeZoneFront_pui8[10]
        {
          25, 45, 60, 75, 90, 105, 120, 255, 255, 255
        },
        // RangeZoneRear_pui8[10]
        {
          25, 45, 60, 75, 90, 105, 120, 255, 255, 255
        },
        // DisplayConfigFront_pui8[10]
        {
          249, 232, 7, 6, 5, 4, 3, 0, 0, 0
        },
        // DisplayConfigRear_pui8[10]
        {
          249, 232, 7, 6, 5, 4, 3, 0, 0, 0
        }
      }
    },
    // cluster: PSM_DiscardCriteria
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        1500, // MaxSPos_si16
        -1500, // MinSPos_si16
        2500, // MaxSPosRearRef_si16
        0, // MinSPosRearRef_si16
        2000, // MaxAbsoluteDist_ui16
        2000, // MaxYPos_si16
        -2000, // MinYPos_si16
        1608, // MaxPhi_ui16
        4825, // MinPhi_ui16
        1430, // MaxPhiB4ThrshldX_ui16
        5004, // MinPhiB4ThrshldX_ui16
        200 // ThresholdPhiX_si16
      }
    },
    // cluster: PSM_DiscardCriteriaCPSC
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        1000, // MaxSPos_si16
        -250, // MinSPos_si16
        1500, // MaxSPosRearRef_si16
        0, // MinSPosRearRef_si16
        1000, // MaxAbsoluteDist_ui16
        2000, // MaxYPos_si16
        -2000, // MinYPos_si16
        1608, // MaxPhi_ui16
        4825, // MinPhi_ui16
        1608, // MaxPhiB4ThrshldX_ui16
        4825, // MinPhiB4ThrshldX_ui16
        100, // ThresholdPhiX_si16
        1000 // MaxAbsoluteDistOMA_ui16
      }
    },
    // cluster: PSM_ParkableCriteria
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        2000, // MaxYPos_si16
        -2000, // MinYPos_si16
        1500, // MaxXPos_si16
        1500, // MaxSPos_si16
        1500, // ResXPos_si16
        -1500, // MinXPos_si16
        1608, // MaxPhi_ui16
        4825, // MinPhi_ui16
        8750 // MaxSWA_si16
      }
    },
    // cluster: PSM_ParkableCriteriaCPSC
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        2000, // MaxYPos_si16
        -2000, // MinYPos_si16
        1000, // MaxXPos_si16
        1000, // MaxSPos_si16
        400, // ResXPos_si16
        -250, // MinXPos_si16
        1608, // MaxPhi_ui16
        4825, // MinPhi_ui16
        20017 // MaxSWA_si16
      }
    },
    // cluster: PSM_FilterCriteria
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        1 // MinParkable_ui8
      }
    },
    // cluster: PSM_ParametersDPSC
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        55, // DiagonalPSAngle_si8
        70, // DiagonalXOffset_ui8
        260 // DiagonalMinLen_ui16
      }
    },
    // cluster: MP_SensorTimes
    {
      // global var(s): DAI_BR245 | min var: PDC_Only | sub var: F_16
      {
        14860, // mpMaxMeasTime_ui16
        14859 // mpStopTimeVerifyShot_ui16
      },
      // global var(s): DAI_BR245 | min var: PDC_Only | sub var: F_25
      {
        14860, // mpMaxMeasTime_ui16
        14859 // mpStopTimeVerifyShot_ui16
      },
      // global var(s): DAI_BR245 | min var: PDC_Only | sub var: F_34
      {
        14860, // mpMaxMeasTime_ui16
        14859 // mpStopTimeVerifyShot_ui16
      },
      // global var(s): DAI_BR245 | min var: PDC_Only | sub var: R_16
      {
        14860, // mpMaxMeasTime_ui16
        14859 // mpStopTimeVerifyShot_ui16
      },
      // global var(s): DAI_BR245 | min var: PDC_Only | sub var: R_25
      {
        14860, // mpMaxMeasTime_ui16
        14859 // mpStopTimeVerifyShot_ui16
      },
      // global var(s): DAI_BR245 | min var: PDC_Only | sub var: R_34
      {
        14860, // mpMaxMeasTime_ui16
        14859 // mpStopTimeVerifyShot_ui16
      },
      // global var(s): DAI_BR245 | min var: PDC_PSX | sub var: F_16
      {
        26530, // mpMaxMeasTime_ui16
        7869 // mpStopTimeVerifyShot_ui16
      },
      // global var(s): DAI_BR245 | min var: PDC_PSX | sub var: F_25
      {
        10200, // mpMaxMeasTime_ui16
        9029 // mpStopTimeVerifyShot_ui16
      },
      // global var(s): DAI_BR245 | min var: PDC_PSX | sub var: F_34
      {
        10200, // mpMaxMeasTime_ui16
        10199 // mpStopTimeVerifyShot_ui16
      },
      // global var(s): DAI_BR245 | min var: PDC_PSX | sub var: R_16
      {
        14860, // mpMaxMeasTime_ui16
        14859 // mpStopTimeVerifyShot_ui16
      },
      // global var(s): DAI_BR245 | min var: PDC_PSX | sub var: R_25
      {
        14860, // mpMaxMeasTime_ui16
        14859 // mpStopTimeVerifyShot_ui16
      },
      // global var(s): DAI_BR245 | min var: PDC_PSX | sub var: R_34
      {
        14860, // mpMaxMeasTime_ui16
        14859 // mpStopTimeVerifyShot_ui16
      },
      // global var(s): DAI_BR245 | min var: PSX_BSD | sub var: F_16
      {
        29440, // mpMaxMeasTime_ui16
        7869 // mpStopTimeVerifyShot_ui16
      },
      // global var(s): DAI_BR245 | min var: PSX_BSD | sub var: F_25
      {
        14860, // mpMaxMeasTime_ui16
        14859 // mpStopTimeVerifyShot_ui16
      },
      // global var(s): DAI_BR245 | min var: PSX_BSD | sub var: F_34
      {
        14860, // mpMaxMeasTime_ui16
        14859 // mpStopTimeVerifyShot_ui16
      },
      // global var(s): DAI_BR245 | min var: PSX_BSD | sub var: R_16
      {
        29440, // mpMaxMeasTime_ui16
        7869 // mpStopTimeVerifyShot_ui16
      },
      // global var(s): DAI_BR245 | min var: PSX_BSD | sub var: R_25
      {
        14860, // mpMaxMeasTime_ui16
        14859 // mpStopTimeVerifyShot_ui16
      },
      // global var(s): DAI_BR245 | min var: PSX_BSD | sub var: R_34
      {
        14860, // mpMaxMeasTime_ui16
        14859 // mpStopTimeVerifyShot_ui16
      }
    },
    // cluster: MP_SensorStatic
    {
      // global var(s): DAI_BR245 | min var: PAS_Normal | sub var: F_16
      {
        0, // DeIdleTime_ui16
        1759, // CeInnerIdleTime_ui16
        0 // CeOuterIdleTime_ui16
      },
      // global var(s): DAI_BR245 | min var: PAS_Normal | sub var: F_25
      {
        0, // DeIdleTime_ui16
        2455, // CeInnerIdleTime_ui16
        1759 // CeOuterIdleTime_ui16
      },
      // global var(s): DAI_BR245 | min var: PAS_Normal | sub var: F_34
      {
        0, // DeIdleTime_ui16
        3093, // CeInnerIdleTime_ui16
        2455 // CeOuterIdleTime_ui16
      },
      // global var(s): DAI_BR245 | min var: PAS_Normal | sub var: R_16
      {
        0, // DeIdleTime_ui16
        1759, // CeInnerIdleTime_ui16
        0 // CeOuterIdleTime_ui16
      },
      // global var(s): DAI_BR245 | min var: PAS_Normal | sub var: R_25
      {
        0, // DeIdleTime_ui16
        2455, // CeInnerIdleTime_ui16
        1759 // CeOuterIdleTime_ui16
      },
      // global var(s): DAI_BR245 | min var: PAS_Normal | sub var: R_34
      {
        0, // DeIdleTime_ui16
        2513, // CeInnerIdleTime_ui16
        2455 // CeOuterIdleTime_ui16
      },
      // global var(s): DAI_BR245 | min var: PAS_AHK | sub var: F_16
      {
        0, // DeIdleTime_ui16
        1759, // CeInnerIdleTime_ui16
        0 // CeOuterIdleTime_ui16
      },
      // global var(s): DAI_BR245 | min var: PAS_AHK | sub var: F_25
      {
        0, // DeIdleTime_ui16
        2455, // CeInnerIdleTime_ui16
        1759 // CeOuterIdleTime_ui16
      },
      // global var(s): DAI_BR245 | min var: PAS_AHK | sub var: F_34
      {
        0, // DeIdleTime_ui16
        3093, // CeInnerIdleTime_ui16
        2455 // CeOuterIdleTime_ui16
      },
      // global var(s): DAI_BR245 | min var: PAS_AHK | sub var: R_16
      {
        0, // DeIdleTime_ui16
        1759, // CeInnerIdleTime_ui16
        0 // CeOuterIdleTime_ui16
      },
      // global var(s): DAI_BR245 | min var: PAS_AHK | sub var: R_25
      {
        0, // DeIdleTime_ui16
        2455, // CeInnerIdleTime_ui16
        1759 // CeOuterIdleTime_ui16
      },
      // global var(s): DAI_BR245 | min var: PAS_AHK | sub var: R_34
      {
        2571, // DeIdleTime_ui16
        2913, // CeInnerIdleTime_ui16
        2455 // CeOuterIdleTime_ui16
      },
      // global var(s): DAI_BR245 | min var: SVA | sub var: F_16
      {
        0, // DeIdleTime_ui16
        0, // CeInnerIdleTime_ui16
        0 // CeOuterIdleTime_ui16
      },
      // global var(s): DAI_BR245 | min var: SVA | sub var: F_25
      {
        0, // DeIdleTime_ui16
        0, // CeInnerIdleTime_ui16
        0 // CeOuterIdleTime_ui16
      },
      // global var(s): DAI_BR245 | min var: SVA | sub var: F_34
      {
        0, // DeIdleTime_ui16
        0, // CeInnerIdleTime_ui16
        0 // CeOuterIdleTime_ui16
      },
      // global var(s): DAI_BR245 | min var: SVA | sub var: R_16
      {
        0, // DeIdleTime_ui16
        0, // CeInnerIdleTime_ui16
        0 // CeOuterIdleTime_ui16
      },
      // global var(s): DAI_BR245 | min var: SVA | sub var: R_25
      {
        0, // DeIdleTime_ui16
        0, // CeInnerIdleTime_ui16
        0 // CeOuterIdleTime_ui16
      },
      // global var(s): DAI_BR245 | min var: SVA | sub var: R_34
      {
        0, // DeIdleTime_ui16
        0, // CeInnerIdleTime_ui16
        0 // CeOuterIdleTime_ui16
      }
    },
    // cluster: MP_PdcMeasGroupParId
    {
      // global var(s): DAI_BR245 | min var: F6_R6 | sub var: not labeled
      {
        6, // mpFrontGroupId_ui8
        16 // mpRearGroupId_ui8
      },
      // global var(s): DAI_BR245 | min var: F6_R4 | sub var: not labeled
      {
        6, // mpFrontGroupId_ui8
        14 // mpRearGroupId_ui8
      },
      // global var(s): DAI_BR245 | min var: F4_R4 | sub var: not labeled
      {
        4, // mpFrontGroupId_ui8
        14 // mpRearGroupId_ui8
      },
      // global var(s): DAI_BR245 | min var: F0_R4 | sub var: not labeled
      {
        255, // mpFrontGroupId_ui8
        14 // mpRearGroupId_ui8
      },
      // global var(s): DAI_BR245 | min var: F0_R3 | sub var: not labeled
      {
        255, // mpFrontGroupId_ui8
        13 // mpRearGroupId_ui8
      }
    },
    // cluster: MP_Timings
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: PAS
      {
        50, // mpSensStatusCheckTime_si16
        100, // mpSensPwrCheckTime_si16
        100, // mpSensTempReadTime_si16
        200, // mpSensOSCFreqCheckTime_si16
        25 // mpDisturbWaitTime_ui8
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: PSX
      {
        100, // mpSensStatusCheckTime_si16
        100, // mpSensPwrCheckTime_si16
        100, // mpSensTempReadTime_si16
        200, // mpSensOSCFreqCheckTime_si16
        25 // mpDisturbWaitTime_ui8
      }
    },
    // cluster: MP_SensorType
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        // mpSensorPortType_pui8[14]
        {
          1, 2, 2, 2, 2, 1, 0, 1, 2, 2, 2, 2, 1, 0
        }
      }
    },
    // cluster: MP_SensorOrientation
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        // mpSensorPortOrien_pui8[14]
        {
          2, 0, 0, 0, 0, 2, 5, 2, 1, 1, 1, 1, 2, 5
        }
      }
    },
    // cluster: MP_RingTimeInfor
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        4, // NumCcSets_ui8
        3, // DefaultIdx_ui8
        // TempRange_psi8[4]
        {
          -10, 0, 10, 90
        },
        2 // TempHyst_si8
      }
    },
    // cluster: MP_RingTimeRange
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: TEMP0
      {
        // DataRange_pui16[4]
        {
          500, 2800, 500, 2500
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: TEMP1
      {
        // DataRange_pui16[4]
        {
          500, 2800, 500, 2500
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: TEMP2
      {
        // DataRange_pui16[4]
        {
          500, 2800, 500, 2500
        }
      },
      // global var(s): DAI_BR245 | min var: not labeled | sub var: TEMP3
      {
        // DataRange_pui16[4]
        {
          500, 2800, 500, 2500
        }
      }
    },
    // cluster: MP_SensorPort
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        1, // mpSensorPortID01_ui8
        2, // mpSensorPortID02_ui8
        3, // mpSensorPortID03_ui8
        4, // mpSensorPortID04_ui8
        5, // mpSensorPortID05_ui8
        6, // mpSensorPortID06_ui8
        0, // mpSensorPortID07_ui8
        9, // mpSensorPortID08_ui8
        10, // mpSensorPortID09_ui8
        11, // mpSensorPortID10_ui8
        12, // mpSensorPortID11_ui8
        13, // mpSensorPortID12_ui8
        14, // mpSensorPortID13_ui8
        0 // mpSensorPortID14_ui8
      }
    },
    // cluster: OD_General
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        1, // NumCyclesStartup_ui8
        5 // LowSpeed_ui8
      }
    },
    // cluster: OD_MaskActVelo
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        20, // odMaskActVeloPdcExclusive_decel_ui16
        30, // odMaskActVeloPdcExtended_accel_ui16
        60, // odMaskActVeloPdcExtended_decel_ui16
        80, // odMaskActVeloPdcCornerFront_accel_ui16
        150, // odMaskActVeloPdcCornerFront_decel_ui16
        180, // odMaskActVeloPsxBsd_accel_ui16
        250, // odMaskActVeloPsxBsd_decel_ui16
        300 // odMaskActVeloBsdExclusive_accel_ui16
      }
    },
    // cluster: OD_IdleDist
    {
      // global var(s): DAI_BR245 | min var: PDC_Normal | sub var: not labeled
      {
        // IdleDistDeFront_pui16[6]
        {
          250, 200, 200, 200, 200, 250
        },
        // IdleDistCeFront_pui16[5]
        {
          210, 170, 320, 170, 210
        },
        // IdleDistDeRear_pui16[6]
        {
          250, 200, 200, 200, 200, 250
        },
        // IdleDistCeRear_pui16[5]
        {
          130, 230, 320, 230, 130
        }
      },
      // global var(s): DAI_BR245 | min var: PDC_AHK | sub var: not labeled
      {
        // IdleDistDeFront_pui16[6]
        {
          200, 200, 200, 200, 200, 200
        },
        // IdleDistCeFront_pui16[5]
        {
          260, 380, 490, 380, 260
        },
        // IdleDistDeRear_pui16[6]
        {
          200, 200, 200, 200, 200, 200
        },
        // IdleDistCeRear_pui16[5]
        {
          260, 380, 400, 380, 260
        }
      }
    },
    // cluster: OD_Sens
    {
      // global var(s): DAI_BR245 | min var: PDC_Normal | sub var: not labeled
      {
        // SensDistFront_pui16[5]
        {
          406, 339, 642, 339, 406
        },
        // SensDistRear_pui16[5]
        {
          265, 454, 638, 454, 265
        },
        // SensPosXFront_psi16[6]
        {
          3021, 3360, 3461, 3461, 3360, 3021
        },
        // SensPosYFront_psi16[6]
        {
          868, 644, 321, -321, -644, -866
        },
        // SensPosZFront_psi16[6]
        {
          546, 475, 942, 942, 475, 546
        },
        // SensPosXRear_psi16[6]
        {
          -401, -638, -795, -795, -638, -401
        },
        // SensPosYRear_psi16[6]
        {
          -863, -746, -319, 319, 746, 863
        },
        // SensPosZRear_psi16[6]
        {
          552, 550, 544, 544, 550, 552
        },
        // SensBeamDirFront_psi16[3]
        {
          5719, 2145, 572
        },
        // SensBeamWidthFront_psi16[3]
        {
          4289, 8579, 8579
        },
        // SensBeamWidth4DeplausWall_psi16[3]
        {
          3217, 6434, 7149
        },
        // SensBeamWidth4DeplausPnt_psi16[3]
        {
          3932, 7149, 7149
        },
        4000, // SensBeamDist4DeplausMajorObj_ui16
        4500, // SensBeamDist4DeplausMinorObj_ui16
        // SensBeamDistMin4CurbstoneFront_pui16[3]
        {
          850, 800, 700
        },
        // SensBeamDistMin4CurbstoneRear_pui16[3]
        {
          900, 850, 750
        },
        // SensBeamDistFront_pui16[3]
        {
          5100, 5100, 5100
        },
        // SensBeamDirRear_psi16[3]
        {
          6434, 2860, 500
        },
        // SensBeamWidthRear_psi16[3]
        {
          4289, 12510, 12510
        },
        // SensBeamDistRear_pui16[3]
        {
          4500, 2500, 2500
        }
      },
      // global var(s): DAI_BR245 | min var: PDC_AHK | sub var: not labeled
      {
        // SensDistFront_pui16[5]
        {
          156, 438, 640, 438, 156
        },
        // SensDistRear_pui16[5]
        {
          158, 447, 460, 447, 158
        },
        // SensPosXFront_psi16[6]
        {
          3290, 3421, 3559, 3559, 3421, 3290
        },
        // SensPosYFront_psi16[6]
        {
          818, 735, 320, -320, -735, -818
        },
        // SensPosZFront_psi16[6]
        {
          444, 437, 419, 419, 437, 444
        },
        // SensPosXRear_psi16[6]
        {
          -481, -570, -641, -641, -570, -481
        },
        // SensPosYRear_psi16[6]
        {
          -801, -671, -230, 230, 671, 801
        },
        // SensPosZRear_psi16[6]
        {
          435, 434, 434, 434, 434, 435
        },
        // SensBeamDirFront_psi16[3]
        {
          5362, 2717, 929
        },
        // SensBeamWidthFront_psi16[3]
        {
          6434, 12510, 12510
        },
        // SensBeamWidth4DeplausWall_psi16[3]
        {
          3217, 6434, 7149
        },
        // SensBeamWidth4DeplausPnt_psi16[3]
        {
          3932, 8579, 8579
        },
        2000, // SensBeamDist4DeplausMajorObj_ui16
        2500, // SensBeamDist4DeplausMinorObj_ui16
        // SensBeamDistMin4CurbstoneFront_pui16[3]
        {
          610, 480, 500
        },
        // SensBeamDistMin4CurbstoneRear_pui16[3]
        {
          590, 610, 600
        },
        // SensBeamDistFront_pui16[3]
        {
          4500, 4500, 4500
        },
        // SensBeamDirRear_psi16[3]
        {
          2860, 1430, 286
        },
        // SensBeamWidthRear_psi16[3]
        {
          8579, 12510, 12510
        },
        // SensBeamDistRear_pui16[3]
        {
          4500, 4500, 4500
        }
      }
    },
    // cluster: OD_Veh
    {
      // global var(s): DAI_BR245 | min var: PDC_Normal | sub var: not labeled
      {
        900, // VehWidthHalf_ui16
        6, // NumberContourPntsFront_ui8
        6, // NumberContourPntsRear_ui8
        5, // NumberContourPntsSide_ui8
        // VehContourFrontX_psi16[6]
        {
          3073, 3257, 3371, 3449, 3476, 3489
        },
        // VehContourFrontY_psi16[6]
        {
          -855, -778, -652, -487, -275, 0
        },
        // VehContourRearX_psi16[6]
        {
          -302, -492, -677, -778, -822, -829
        },
        // VehContourRearY_psi16[6]
        {
          -894, -842, -722, -542, -232, 0
        },
        // VehContourSideX_psi16[5]
        {
          3073, 2231, 1364, 492, -302
        },
        // VehContourSideY_psi16[5]
        {
          -855, -864, -875, -885, -894
        }
      },
      // global var(s): DAI_BR245 | min var: PDC_AHK | sub var: not labeled
      {
        885, // VehWidthHalf_ui16
        4, // NumberContourPntsFront_ui8
        4, // NumberContourPntsRear_ui8
        5, // NumberContourPntsSide_ui8
        // VehContourFrontX_psi16[6]
        {
          3191, 3401, 3511, 3600, 0, 0
        },
        // VehContourFrontY_psi16[6]
        {
          -880, -789, -543, -85, 0, 0
        },
        // VehContourRearX_psi16[6]
        {
          -343, -500, -701, -955, 0, 0
        },
        // VehContourRearY_psi16[6]
        {
          -872, -806, -331, -158, 0, 0
        },
        // VehContourSideX_psi16[5]
        {
          3191, 2310, 2121, 1962, -343
        },
        // VehContourSideY_psi16[5]
        {
          900, 900, 900, 900, 900
        }
      }
    },
    // cluster: OD_Echo_ApFlt
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        300, // ApFltEchoDelta1_ui16
        600, // ApFltEchoDelta2_ui16
        900, // ApFltEchoDelta3_ui16
        200, // ApFltEchoDeltaCe_ui16
        100, // ApFltEchoDelta1KMH_ui16
        800, // EchoDeltaOfMaxSpeed_ui16
        300, // ApFltEchoZone1Dist_ui16
        500, // ApFltEchoZone2Dist_ui16
        700, // ApFltEchoZone3Dist_ui16
        1200, // ApFltEchoZone4Dist_ui16
        3, // ApFltEchoDelta1Penalty_ui8
        3, // ApFltEchoDelta2Penalty_ui8
        3, // ApFltEchoDelta3Penalty_ui8
        1, // ApFltEchoZone1Penalty_ui8
        0, // ApFltEchoZone2Penalty_ui8
        0, // ApFltEchoZone3Penalty_ui8
        0, // ApFltEchoZone4Penalty_ui8
        0, // ApFltEchoZone5Penalty_ui8
        1 // ApFltEchoZone6Penalty_ui8
      }
    },
    // cluster: OD_Echo_WRF
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        1, // WRFEnable_bl
        2145, // WRFWheelAngLimit_ui16
        350, // WRFWheelOutwardDeLowerLimit_ui16
        450, // WRFWheelOutwardDeUpperLimit_ui16
        200, // WRFWheelInwardDeLowerLimit_ui16
        200 // WRFWheelInwardDeUpperLimit_ui16
      }
    },
    // cluster: OD_Echo_SRF
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        // SRFEnable_pbl[2]
        {
          0, 0
        },
        // SRFDeLimit_pui16[2]
        {
          0, 800
        },
        // SRFCeLimit_pui16[2]
        {
          0, 500
        }
      }
    },
    // cluster: OD_Echo_HLF
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        // HLFEnable_pbl[2]
        {
          1, 0
        },
        // HLFCe23RangeMin_pui16[2]
        {
          190, 190
        },
        // HLFCe23RangeMax_pui16[2]
        {
          460, 560
        },
        // HLFDe2RangeMin_pui16[2]
        {
          560, 390
        },
        // HLFDe2RangeMax_pui16[2]
        {
          580, 410
        },
        // HLFCe23Limit_pui16[2]
        {
          410, 190
        },
        // HLFDe2Limit_pui16[2]
        {
          340, 300
        },
        // HLFCe12Limit_pui16[2]
        {
          450, 480
        },
        // HLFDe3LimitForCe23_pui16[2]
        {
          280, 490
        },
        // HLFDe3LimitForDe2_pui16[2]
        {
          490, 400
        },
        10 // HLFSpeedLim_ui8
      }
    },
    // cluster: OD_Echo_MoveDir
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        20, // RelMovingDeTol_ui16
        20, // RelMovingCeTol_ui16
        0 // MaxCyclMovingHold_ui8
      }
    },
    // cluster: OD_Echo_Clip_Std
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        // FrontDeLimit_pui16[6]
        {
          1200, 1200, 2000, 2000, 1200, 1200
        },
        // FrontCeLimit_pui16[5]
        {
          2500, 2500, 4500, 2500, 2500
        },
        // RearDeLimit_pui16[6]
        {
          5100, 5100, 5100, 5100, 5100, 5100
        },
        // RearCeLimit_pui16[5]
        {
          5100, 5100, 5100, 5100, 5100
        },
        40 // ClippingHyst_ui16
      }
    },
    // cluster: OD_Obj_Meas
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        250, // CriticalDist_ui16
        100, // CriticalDistHyst_ui16
        150, // ObjTypeDeltaDist_ui16
        150, // OblFiltHystNear_ui16
        150 // OblFiltHystFar_ui16
      }
    },
    // cluster: OD_Obj_Trck
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        0, // EnableTrackEndless_bl
        500, // TrackCycleMax_ui16
        500, // TrackCycleMaxWall_ui16
        200, // TrackCycleMin_ui16
        24, // TrackCycleWait4Deplaus_ui16
        400, // TrackDistMax_ui16
        5000, // TrackVehDistFront_ui16
        5000, // TrackVehDistSide_ui16
        10000, // RelevanceRange4HighLevelObjFront_ui16
        10000, // RelevanceRange4HighLevelObjRear_ui16
        7000, // RelevanceRange4HighLevelObjSide_ui16
        150, // TrackVehDistMin_ui16
        300, // MatchWindowInit_ui16
        500, // MatchWindowMax_ui16
        10, // MatchWindowTrackCycleFactor_ui16
        358, // MatchWindowTrackDistFactor_ui16
        100, // MatchWallTolX_ui16
        120, // MatchWallTolXCeObj_ui16
        963, // CosAngleToleranceWall_ui16
        22, // ExistProbMin_ui8
        51, // ExistProbLimFrontObj4MaxTrck_ui8
        20, // ExistProbLimSideObj4MaxTrck_ui8
        35, // ExistProbMatchFactor_ui8
        35, // ExistProbMatchSlowFactor_ui8
        10, // ExistProbDecFactor_ui8
        7, // ExistProbDecFastFactor_ui8
        5, // ExistProbDecFast2Factor_ui8
        128, // ObjRelMax4ExistProbMin_ui8
        2860, // ObjMoveDeltaAngleMax_ui16
        512, // ObjMoveDeltaDistFak_ui16
        250, // SensDistMin4WallType_ui16
        6500, // WallLengthMax_ui16
        3000, // WallLengthMax4AngleCorr_ui16
        10000, // WallLengthMax4PlausDec_ui16
        120, // DoubleEchoTol4HighObj_ui16
        80, // HeightClassificationLow_ui8
        169 // HeightClassificationHigh_ui8
      }
    },
    // cluster: SP_PWR_CFG
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        200, // UbattMax_ui8
        6, // UbattMaxHysteresis_ui8
        60, // UbattMin_ui8
        4 // UbattMinHysteresis_ui8
      }
    },
    // cluster: VHS_PL_TORQUE_Controller_ZFLS
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        30, // PosPID_Kp_si32
        7, // PosPID_KpScale_si8
        100, // PosPID_Ki_si32
        10, // PosPID_KiScale_si8
        0, // PosPID_Kd_si32
        13, // PosPID_KdScale_si8
        20, // PosPID_MaxIntegral_si32
        0, // PosPID_MaxIntegralScale_si8
        60, // SpeedPID_Kp_si32
        15, // SpeedPID_KpScale_si8
        70, // SpeedPID_Ki_si32
        11, // SpeedPID_KiScale_si8
        10, // SpeedPID_Kd_si32
        19, // SpeedPID_KdScale_si8
        70, // SpeedPID_MaxIntegral_si32
        0, // SpeedPID_MaxIntegralScale_si8
        12800, // PosPIDscale_factor_si16
        550, // maxSWASpeed_si32
        10000, // maxSWAAcc_si32
        14, // maxTorqueReq_si32
        1, // maxTorqueReq_scale_si8
        200, // maxTorqueSlewRate_si32
        3, // outputFIR_size_ui8
        8169, // outputFIR_coefficient0_si32
        16427, // outputFIR_coefficient1_si32
        8169, // outputFIR_coefficient2_si32
        0, // outputFIR_coefficient3_si32
        0, // outputFIR_coefficient4_si32
        0, // outputFIR_coefficient5_si32
        0, // outputFIR_coefficient6_si32
        0, // outputFIR_coefficient7_si32
        0, // outputFIR_coefficient8_si32
        0, // outputFIR_coefficient9_si32
        15, // outputFIR_coefficient_scaling_si8
        3, // speed_reqFIR_size_ui8
        8169, // speed_reqFIR_coefficient0_si32
        16427, // speed_reqFIR_coefficient1_si32
        8169, // speed_reqFIR_coefficient2_si32
        0, // speed_reqFIR_coefficient3_si32
        0, // speed_reqFIR_coefficient4_si32
        0, // speed_reqFIR_coefficient5_si32
        0, // speed_reqFIR_coefficient6_si32
        0, // speed_reqFIR_coefficient7_si32
        0, // speed_reqFIR_coefficient8_si32
        0, // speed_reqFIR_coefficient9_si32
        15, // speed_reqFIR_coefficient_scaling_si8
        3, // speed_curFIR_size_ui8
        8169, // speed_curFIR_coefficient0_si32
        16427, // speed_curFIR_coefficient1_si32
        8169, // speed_curFIR_coefficient2_si32
        0, // speed_curFIR_coefficient3_si32
        0, // speed_curFIR_coefficient4_si32
        0, // speed_curFIR_coefficient5_si32
        0, // speed_curFIR_coefficient6_si32
        0, // speed_curFIR_coefficient7_si32
        0, // speed_curFIR_coefficient8_si32
        0, // speed_curFIR_coefficient9_si32
        15 // speed_curFIR_coefficient_scaling_si8
      }
    },
    // cluster: VHS_PL_TORQUE_Controller_CR
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        1, // B_si32
        1, // B_scale_si8
        1, // Observer_a11_si32
        0, // Observer_a11_scale_si8
        9, // Observer_a12_si32
        10, // Observer_a12_scale_si8
        -151, // Observer_a13_si32
        16, // Observer_a13_scale_si8
        25, // Observer_a22_si32
        5, // Observer_a22_scale_si8
        -7, // Observer_a23_si32
        4, // Observer_a23_scale_si8
        1, // Observer_a33_si32
        0, // Observer_a33_scale_si8
        151, // Observer_b1_si32
        16, // Observer_b1_scale_si8
        7, // Observer_b2_si32
        4, // Observer_b2_scale_si8
        87, // Observer_L1_si32
        7, // Observer_L1_scale_si8
        96, // Observer_L2_si32
        3, // Observer_L2_scale_si8
        -783, // Observer_L3_si32
        7, // Observer_L3_scale_si8
        2, // K1_si32
        0, // K1_scale_si8
        869, // K2_si32
        13, // K2_scale_si8
        187, // ratio_collumn_motor_si32
        12 // ratio_collumn_motor_scale_si8
      }
    },
    // cluster: AP_BSD_Ap
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        100, // SteWheAngMax_ui16
        1000, // MinWarnTime_ui16
        1000, // OvertakenCarSuppressionTime_ui16
        2000, // OvertakenCarSuppressionTime2_ui16
        1500, // OvertakenCarSuppressionTime2Mid_ui16
        80, // OvertaCarSpeedSwitch_ui8
        1100, // AdditionalWarnTime_ui16
        9000, // SensorCoverageDistance_ui16
        6000, // SuppressStartup_ui16
        1500, // OvertakenCarSuppressionTimeMid_ui16
        240, // MidSensorWarnDistLowSpeed_ui16
        260 // MidSensorWarnDistHighSpeed_ui16
      }
    },
    // cluster: EP_BSD_Ep
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        3, // CharTableOffsetFrontLow_ui8
        3, // CharTableOffsetRearLow_ui8
        3, // CharTableOffsetFrontMid_ui8
        3, // CharTableOffsetRearMid_ui8
        2, // CharTableOffsetFrontHigh_ui8
        2, // CharTableOffsetRearHigh_ui8
        19, // CharTableMaxFrontAndRear_ui8
        0, // CharTempThresholdLow_si8
        27, // CharTempThresholdHigh_si8
        0, // BlockedSensorTestAtStartUp_bl
        1, // BlockedSensorTestAtMeasure_bl
        180000, // BlockedSensorTime_ui32
        2000, // BlockedSensorTestDuration_ui16
        50, // StayInTestVelocityMin_ui8
        5, // BlockedSensorTestTempLimit_si8
        100, // HighSpeedTrackingAc_ui8
        95, // HighSpeedTrackingDe_ui8
        0, // CcIDOffsetFrontHST_si8
        0, // CcIDOffsetRearHST_si8
        120, // VelocityMaxAc_ui8
        115, // VelocityMaxDe_ui8
        0, // HighNoiseHysteresisFront_ui8
        0 // HighNoiseHysteresisRear_ui8
      }
    },
    // cluster: EP_BSD_SDI
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        0, // MinObjStayTime_ui16
        1000, // WarnLvlHoldTime_ui16
        7, // NumOfWarnLevels_ui8
        // WarnDistancesLeft_pui8[7]
        {
          50, 62, 74, 85, 97, 109, 120
        },
        // WarnDistancesRight_pui8[7]
        {
          50, 62, 74, 85, 97, 109, 120
        }
      }
    },
    // cluster: AP_PDC_General
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        2, // NumCyclesStartupSC_ui8
        60, // ApPdcTimeOutFront_ui8
        60 // ApPdcTimeOutRear_ui8
      }
    },
    // cluster: AP_PDC_Disp_Global
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        250, // FullWarnDist_ui16
        200, // DispAreaHoldTime_ui16
        0, // DispAreaDelayTime_ui16
        40, // DispAreaApproachHyst_ui16
        100, // DispAreaDepartHyst_ui16
        130, // DispAreaCloseDepartHyst_ui16
        200, // DispDist4CloseDepartHyst_ui16
        10, // DispAreaDistRes_ui16
        0, // SafetyDist4OOC_si16
        100, // Dist2CourseHystOOC_si16
        150 // DispDistMinimum_ui16
      }
    },
    // cluster: AP_PDC_Disp_Lat
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        2000, // ObjCaptureTime_ui16
        // DispAreaFrontMin_pui8[5]
        {
          0, 5, 1, 1, 1
        },
        // DispAreaFrontMax_pui8[5]
        {
          5, 10, 0, 0, 0
        },
        // DispAreaFrontHystMin_pui8[5]
        {
          0, 5, 1, 1, 1
        },
        // DispAreaFrontHystMax_pui8[5]
        {
          5, 10, 0, 0, 0
        },
        // DispAreaRearMin_pui8[5]
        {
          2, 5, 1, 1, 1
        },
        // DispAreaRearMax_pui8[5]
        {
          5, 8, 0, 0, 0
        },
        // DispAreaRearHystMin_pui8[5]
        {
          2, 5, 1, 1, 1
        },
        // DispAreaRearHystMax_pui8[5]
        {
          5, 8, 0, 0, 0
        }
      }
    },
    // cluster: AP_PDC_View_Lat
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        // ClipFrontZone_pui16[23]
        {
          600, 600, 750, 900, 900, 900, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
          1000, 900, 900, 900, 750, 600, 600
        },
        // ClipRearZone_pui16[23]
        {
          0, 0, 0, 0, 600, 600, 800, 1000, 1100, 1100, 1100, 1100, 1100, 1100, 1100, 1000,
          800, 600, 600, 0, 0, 0, 0
        },
        40 // ClippingHyst_ui16
      }
    },
    // cluster: AP_PDC_View_Prm
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        51, // ExistProbMin4Disp_ui8
        51, // ExistProbMinMinorObj4Disp_ui8
        // DispAreaFrontA1X_psi16[6]
        {
          3073, 3339, 3466, 3489, 3466, 3339
        },
        // DispAreaFrontA1Y_psi16[6]
        {
          855, 687, 348, 0, -348, -687
        },
        // DispAreaFrontA2X_psi16[6]
        {
          3339, 3466, 3489, 3466, 3339, 3073
        },
        // DispAreaFrontA2Y_psi16[6]
        {
          687, 348, 0, -348, -687, -855
        },
        // DispAreaFrontA3X_psi16[6]
        {
          6072, 8020, 8489, 8020, 6072, 3095
        },
        // DispAreaFrontA3Y_psi16[6]
        {
          4874, 2413, 0, -2413, -4874, -5854
        },
        // DispAreaFrontA4X_psi16[6]
        {
          3095, 6072, 8020, 8489, 8020, 6072
        },
        // DispAreaFrontA4Y_psi16[6]
        {
          5854, 4874, 2413, 0, -2413, -4874
        },
        // DispAreaFrontHysA1X_psi16[6]
        {
          3023, 3305, 3459, 3486, 3472, 3372
        },
        // DispAreaFrontHysA1Y_psi16[6]
        {
          854, 724, 398, 50, -299, -649
        },
        // DispAreaFrontHysA2X_psi16[6]
        {
          3372, 3472, 3486, 3459, 3305, 3023
        },
        // DispAreaFrontHysA2Y_psi16[6]
        {
          649, 298, -50, -398, -724, -854
        },
        // DispAreaFrontHysA3X_psi16[6]
        {
          6135, 8041, 8486, 8003, 5998, 3004
        },
        // DispAreaFrontHysA3Y_psi16[6]
        {
          4815, 2330, -50, -2484, -4937, -5854
        },
        // DispAreaFrontHysA4X_psi16[6]
        {
          3004, 5998, 8003, 8486, 8041, 6135
        },
        // DispAreaFrontHysA4Y_psi16[6]
        {
          5854, 4937, 2484, 50, -2330, -4815
        },
        // DispAreaRearA1X_psi16[6]
        {
          -302, -641, -809, -829, -809, -641
        },
        // DispAreaRearA1Y_psi16[6]
        {
          -894, -745, -323, 0, 323, 745
        },
        // DispAreaRearA2X_psi16[6]
        {
          -641, -809, -829, -809, -641, -302
        },
        // DispAreaRearA2Y_psi16[6]
        {
          -745, -323, 0, 323, 745, 894
        },
        // DispAreaRearA3X_psi16[6]
        {
          -3335, -5284, -5829, -5284, -3335, -294
        },
        // DispAreaRearA3Y_psi16[6]
        {
          -4957, -2552, 0, 2552, 4957, 5894
        },
        // DispAreaRearA4X_psi16[6]
        {
          -294, -3335, -5284, -5829, -5284, -3335
        },
        // DispAreaRearA4Y_psi16[6]
        {
          -5894, -4957, -2552, 0, 2552, 4957
        },
        // DispAreaRearHysA1X_psi16[6]
        {
          -252, -599, -801, -827, -815, -681
        },
        // DispAreaRearHysA1Y_psi16[6]
        {
          -894, -772, -372, -50, 273, 714
        },
        // DispAreaRearHysA2X_psi16[6]
        {
          -681, -815, -827, -801, -599, -251
        },
        // DispAreaRearHysA2Y_psi16[6]
        {
          -714, -273, 50, 372, 772, 894
        },
        // DispAreaRearHysA3X_psi16[6]
        {
          -3433, -5315, -5827, -5247, -3225, -233
        },
        // DispAreaRearHysA3Y_psi16[6]
        {
          -4889, -2454, 73, 2660, 5027, 5894
        },
        // DispAreaRearHysA4X_psi16[6]
        {
          -233, -3225, -5247, -5827, -5315, -3433
        },
        // DispAreaRearHysA4Y_psi16[6]
        {
          -5894, -5027, -2660, -73, 2453, 4889
        },
        // DispAreaLeftA1X_psi16[5]
        {
          -301, 499, 1364, 2231, 2620
        },
        // DispAreaLeftA1Y_psi16[5]
        {
          894, 884, 874, 864, 859
        },
        // DispAreaLeftA2X_psi16[5]
        {
          499, 1364, 2231, 2620, 3073
        },
        // DispAreaLeftA2Y_psi16[5]
        {
          884, 874, 864, 859, 854
        },
        // DispAreaLeftA3X_psi16[5]
        {
          489, 1376, 2272, 2639, 3094
        },
        // DispAreaLeftA3Y_psi16[5]
        {
          5884, 5874, 5864, 5859, 5854
        },
        // DispAreaLeftA4X_psi16[5]
        {
          -294, 489, 1376, 2272, 2639
        },
        // DispAreaLeftA4Y_psi16[5]
        {
          5894, 5884, 5874, 5864, 5859
        },
        // DispAreaLeftHysA1X_psi16[5]
        {
          -301, 499, 1364, 2231, 2620
        },
        // DispAreaLeftHysA1Y_psi16[5]
        {
          894, 884, 874, 864, 859
        },
        // DispAreaLeftHysA2X_psi16[5]
        {
          499, 1364, 2231, 2620, 3073
        },
        // DispAreaLeftHysA2Y_psi16[5]
        {
          884, 874, 864, 859, 854
        },
        // DispAreaLeftHysA3X_psi16[5]
        {
          489, 1376, 2272, 2639, 3094
        },
        // DispAreaLeftHysA3Y_psi16[5]
        {
          5884, 5874, 5864, 5859, 5854
        },
        // DispAreaLeftHysA4X_psi16[5]
        {
          -294, 489, 1376, 2272, 2639
        },
        // DispAreaLeftHysA4Y_psi16[5]
        {
          5894, 5884, 5874, 5864, 5859
        },
        // DispAreaRightA1X_psi16[5]
        {
          3073, 2620, 2231, 1364, 492
        },
        // DispAreaRightA1Y_psi16[5]
        {
          -854, -859, -864, -874, -885
        },
        // DispAreaRightA2X_psi16[5]
        {
          2620, 2231, 1364, 499, -302
        },
        // DispAreaRightA2Y_psi16[5]
        {
          -859, -864, -874, -885, -894
        },
        // DispAreaRightA3X_psi16[5]
        {
          2639, 2274, 1376, 481, -294
        },
        // DispAreaRightA3Y_psi16[5]
        {
          -5859, -5864, -5874, -5884, -5894
        },
        // DispAreaRightA4X_psi16[5]
        {
          3094, 2639, 2272, 1376, 489
        },
        // DispAreaRightA4Y_psi16[5]
        {
          -5854, -5859, -5864, -5874, -5884
        },
        // DispAreaRightHysA1X_psi16[5]
        {
          3073, 2620, 2231, 1364, 492
        },
        // DispAreaRightHysA1Y_psi16[5]
        {
          -854, -859, -864, -874, -885
        },
        // DispAreaRightHysA2X_psi16[5]
        {
          2620, 2231, 1364, 499, -302
        },
        // DispAreaRightHysA2Y_psi16[5]
        {
          -859, -864, -874, -885, -894
        },
        // DispAreaRightHysA3X_psi16[5]
        {
          2639, 2274, 1376, 481, -294
        },
        // DispAreaRightHysA3Y_psi16[5]
        {
          -5859, -5864, -5874, -5884, -5894
        },
        // DispAreaRightHysA4X_psi16[5]
        {
          3094, 2639, 2272, 1376, 489
        },
        // DispAreaRightHysA4Y_psi16[5]
        {
          -5854, -5859, -5864, -5874, -5884
        }
      }
    },
    // cluster: AP_PDC_Offset_Std
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        // OffsetFront_pui16[5]
        {
          30, 30, 50, 30, 30
        },
        // OffsetRear_pui16[5]
        {
          0, 20, 20, 20, 0
        },
        // TrailerOffset_pui16[2]
        {
          0, 300
        }
      }
    },
    // cluster: AP_PDC_Offset_Lat
    {
      // global var(s): DAI_BR245 | min var: PDC_Normal | sub var: not labeled
      {
        // OffsetFrontDir_pui16[11]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        // OffsetFrontZone_pui16[23]
        {
          20, 20, 20, 20, 30, 30, 30, 30, 30, 50, 50, 50, 50, 50, 30, 30,
          30, 30, 30, 20, 20, 20, 20
        },
        // OffsetRearDir_pui16[11]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        // OffsetRearZone_pui16[23]
        {
          0, 0, 0, 0, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20,
          20, 20, 20, 0, 0, 0, 0
        }
      },
      // global var(s): DAI_BR245 | min var: PDC_AHK | sub var: not labeled
      {
        // OffsetFrontDir_pui16[11]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        // OffsetFrontZone_pui16[23]
        {
          20, 20, 20, 20, 30, 30, 30, 30, 30, 50, 50, 50, 50, 50, 30, 30,
          30, 30, 30, 20, 20, 20, 20
        },
        // OffsetRearDir_pui16[11]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        // OffsetRearZone_pui16[23]
        {
          0, 0, 0, 0, 20, 20, 20, 20, 20, 20, 400, 400, 400, 20, 20, 20,
          20, 20, 20, 0, 0, 0, 0
        }
      }
    },
    // cluster: AP_PDC_SC_DT_Std
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        1, // DTFrontEnabled_bl
        1, // DTRearEnabled_bl
        520, // DTFrontDeAMin_ui16
        610, // DTFrontDeAMax_ui16
        360, // DTFrontDe25Lock_ui16
        570, // DTFrontDeB_ui16
        650, // DTFrontCeLR_ui16
        530, // DTFrontCeMin_ui16
        760, // DTFrontCeMax_ui16
        330, // DTRearDeAMin_ui16
        410, // DTRearDeAMax_ui16
        300, // DTRearDe25Lock_ui16
        450, // DTRearDeB_ui16
        470, // DTRearCeLR_ui16
        380, // DTRearCeMin_ui16
        490 // DTRearCeMax_ui16
      }
    },
    // cluster: AP_PDC_SC_DT_Lat
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        4, // DTFrontEnabled_ui8
        14, // DTRearEnabled_ui8
        // DTFrontActivateDistNarrow_pui16[5]
        {
          400, 400, 380, 400, 400
        },
        // DTFrontActivateDistCEObj_pui16[5]
        {
          400, 400, 530, 400, 400
        },
        // DTFrontActivateDistDEObj_pui16[5]
        {
          200, 200, 520, 200, 200
        },
        // DTRearActivateDistNarrow_pui16[5]
        {
          400, 380, 380, 380, 400
        },
        // DTRearActivateDistCEObj_pui16[5]
        {
          400, 400, 400, 400, 400
        },
        // DTRearActivateDistDEObj_pui16[5]
        {
          200, 330, 330, 330, 200
        },
        // DTFrontDeactivateDistNarrow_pui16[5]
        {
          470, 470, 490, 470, 470
        },
        // DTFrontDeactivateDistCEObj_pui16[5]
        {
          470, 470, 760, 470, 470
        },
        // DTFrontDeactivateDistDEObj_pui16[5]
        {
          470, 470, 610, 470, 470
        },
        // DTRearDeactivateDistNarrow_pui16[5]
        {
          470, 500, 500, 500, 470
        },
        // DTRearDeactivateDistCEObj_pui16[5]
        {
          470, 510, 510, 510, 470
        },
        // DTRearDeactivateDistDEObj_pui16[5]
        {
          470, 410, 410, 410, 470
        }
      }
    },
    // cluster: AP_PDC_SC_DU_Lat
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        0, // DUFrontEnabled_ui8
        0, // DURearEnabled_ui8
        // DUFrontLatzoneMin_pui8[5]
        {
          1, 1, 1, 1, 1
        },
        // DURearLatzoneMin_pui8[5]
        {
          1, 1, 1, 1, 1
        },
        // DUFrontLatzoneMax_pui8[5]
        {
          7, 7, 7, 7, 7
        },
        // DURearLatzoneMax_pui8[5]
        {
          7, 7, 7, 7, 7
        },
        // DUFrontDistTeachMax_pui16[5]
        {
          800, 800, 800, 800, 800
        },
        // DURearDistTeachMax_pui16[5]
        {
          800, 800, 800, 800, 800
        },
        100, // DUFrontDistDeltaMinMeasured_ui16
        100, // DURearDistDeltaMinMeasured_ui16
        // DUFrontDistCriticalMax_pui16[5]
        {
          500, 500, 500, 500, 500
        },
        // DURearDistCriticalMax_pui16[5]
        {
          500, 500, 500, 500, 500
        },
        400, // DUFrontDistDeltaObjLost_ui16
        400, // DURearDistDeltaObjLost_ui16
        1000, // DUFrontActiveTime_ui16
        1000 // DURearActiveTime_ui16
      }
    },
    // cluster: AP_PDC_SC_DU_Prm
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        1, // DUFrontEnabled_bl
        1, // DURearEnabled_bl
        800, // DUFrontDistTeachMax_ui16
        800, // DURearDistTeachMax_ui16
        100, // DUFrontDistDeltaMinApproach_ui16
        100, // DURearDistDeltaMinApproach_ui16
        500, // DUFrontDistCriticalMax_ui16
        500, // DURearDistCriticalMax_ui16
        1000, // DUFrontActiveTime_ui16
        1000 // DURearActiveTime_ui16
      }
    },
    // cluster: AP_PDC_SC_OE_Std
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        0, // OEFrontEnabled_bl
        1, // OERearEnabled_bl
        0, // OEFrontEnabledInner_bl
        1, // OERearEnabledInner_bl
        0, // OEFrontDeA_ui16
        0, // OEFrontDeB_ui16
        0, // OEFrontCeA_ui16
        0, // OEFrontCeB_ui16
        0, // OEFrontOffset_ui16
        0, // OEFrontDeAInner_ui16
        0, // OEFrontCeAInner_ui16
        0, // OEFrontCeBInner_ui16
        0, // OEFrontOffsetInner_ui16
        440, // OERearDeA_ui16
        600, // OERearDeB_ui16
        440, // OERearCeA_ui16
        830, // OERearCeB_ui16
        50, // OERearOffset_ui16
        350, // OERearDeAInner_ui16
        490, // OERearCeAInner_ui16
        880, // OERearCeBInner_ui16
        0 // OERearOffsetInner_ui16
      }
    },
    // cluster: AP_PDC_SC_OE_Lat
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        0, // OEFrontEnabled_bl
        1, // OERearEnabled_bl
        0, // OEFrontGroupLeft_ui8
        4, // OEFrontGroupRight_ui8
        1, // OERearGroupLeft_ui8
        3, // OERearGroupRight_ui8
        500, // OEFrontMaxDist_ui16
        330, // OEFrontMinDist_ui16
        500, // OERearMaxDist_ui16
        330 // OERearMinDist_ui16
      }
    },
    // cluster: AP_PDC_SC_SWD_Std
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        0, // SWDFrontEnabled_bl
        0, // SWDRearEnabled_bl
        0, // SWDFrontGroupLeft_ui8
        4, // SWDFrontGroupRight_ui8
        1, // SWDRearGroupLeft_ui8
        3, // SWDRearGroupRight_ui8
        400, // SWDFrontDistCutOff_ui16
        400, // SWDRearDistCutOff_ui16
        100, // SWDDistHyst_ui16
        5, // SWDFrontSpeedCutOff_ui8
        5, // SWDRearSpeedCutOff_ui8
        31, // SWDDelayTime_ui8
        100, // SWDFrontDistApproach_ui16
        100 // SWDRearDistApproach_ui16
      }
    },
    // cluster: AP_PDC_SC_SWD_Lat
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        0, // SWDFrontEnabled_bl
        0, // SWDRearEnabled_bl
        0, // SWDFrontGroupLeft_ui8
        4, // SWDFrontGroupRight_ui8
        1, // SWDRearGroupLeft_ui8
        3, // SWDRearGroupRight_ui8
        400, // SWDDistCutOff_ui16
        100, // SWDDistCutOffHyst_ui16
        5, // SWDSpeedCutOff_ui8
        2, // SWDSpeedCutOffHyst_ui8
        1500, // SWDDriveDist_ui16
        400, // SWDMaxAppDepSpeed_ui16
        100 // SWDMaxAppDepSpeedHyst_ui16
      }
    },
    // cluster: AP_PDC_SC_SWD_Prm
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        0, // SWDFrontEnabled_bl
        0, // SWDRearEnabled_bl
        400, // SWDDistCutOff_ui16
        100, // SWDDistCutOffHyst_ui16
        5, // SWDSpeedCutOff_ui8
        2, // SWDSpeedCutOffHyst_ui8
        1000, // SWDDriveDist_ui16
        100, // SWDDistDeltaMax_ui16
        30 // SWDDistDeltaMaxHyst_ui16
      }
    },
    // cluster: AP_PDC_SC_WBL_Prm
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        950, // WBLWallLengthMax_ui16
        // WBLGroupCenterDistStart_pui16[2]
        {
          530, 460
        },
        // WBLGroupCenterDistStop_pui16[2]
        {
          330, 260
        },
        // WBLGroupCenterDeDistStart_pui16[2]
        {
          530, 460
        },
        // WBLGroupCenterDeDistStop_pui16[2]
        {
          330, 260
        },
        // WBLGroupCenterCeDistStart_pui16[2]
        {
          650, 580
        },
        // WBLGroupCenterCeDistStop_pui16[2]
        {
          450, 380
        },
        // WBLGroupSideDistStart_pui16[2]
        {
          450, 450
        },
        // WBLGroupSideDistStop_pui16[2]
        {
          250, 250
        },
        // WBLGroupSideDeDistStart_pui16[2]
        {
          450, 450
        },
        // WBLGroupSideDeDistStop_pui16[2]
        {
          250, 250
        },
        // WBLGroupSideCeDistStart_pui16[2]
        {
          570, 570
        },
        // WBLGroupSideCeDistStop_pui16[2]
        {
          370, 370
        },
        // WBLEdgeDistStart_pui16[2]
        {
          250, 200
        },
        // WBLEdgeDistStop_pui16[2]
        {
          250, 200
        },
        // WBLEdgeDeDistStart_pui16[2]
        {
          250, 200
        },
        // WBLEdgeDeDistStop_pui16[2]
        {
          250, 200
        },
        // WBLEdgeCeDistStart_pui16[2]
        {
          250, 250
        },
        // WBLEdgeCeDistStop_pui16[2]
        {
          250, 250
        },
        // WBLEdgeWidth_pui16[2]
        {
          100, 100
        }
      }
    },
    // cluster: AP_PDC_AO
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        20, // AOMaxSamples_ui8
        5, // AOMinNumDetectSamples_ui8
        2, // AOMinNumRegularEchos_ui8
        1200 // AODistMax_ui16
      }
    },
    // cluster: AP_PDC_Danger
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        1000, // DangerDelayMS_ui16
        3000, // DangerDecelMMS2_ui16
        300, // DangerStopDist_ui16
        500, // DangerActMinDist_ui16
        1200, // DangerDeactDist_ui16
        550, // DangerActMinSpeed_ui16
        280 // DangerDeactSpeed_ui16
      }
    },
    // cluster: PSU_CPSC
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        100, // LengthShortenMax_si16
        100, // LengthExtendMax_si16
        300, // DepthMin_si16
        700, // DepthMax_si16
        50, // DepthShortenMax_si16
        50 // DepthExtendMax_si16
      }
    },
    // cluster: PSU_PSC
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        100, // LengthShortenMax_si16
        100, // LengthExtendMax_si16
        150, // DepthMin_si16
        250 // DepthMax_si16
      }
    },
    // cluster: PSU_POC
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        230, // FoVRangeFront_ui16
        230 // FoVRangeRear_ui16
      }
    },
    // cluster: PSU_CPX
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        357, // BackIn_MinYaw_PLOutput_ui16
        400, // BackIn_MaxLengthPS_ui16
        357, // FrontIn_MinYaw_PLOutput_ui16
        400 // FrontIn_MaxLengthPS_ui16
      }
    },
    // cluster: PSU_Object
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        50, // ExistProbMin_ui8
        200, // DECEExistProbMin_ui8
        100 // ErrorMin_ui16
      }
    },
    // cluster: PSU_Reference
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        25, // WeightMin_ui16
        1025 // WeightMax_ui16
      }
    },
    // cluster: APG_XPG_Vehicle
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        90, // HalfWidth_ui16
        262, // Wheelbase_ui16
        347, // LengthFront_ui16
        85, // LengthRear_ui16
        56, // CornerRadiusFront_ui16
        37, // CornerRadiusRear_ui16
        17479, // MaxSWA_si16
        14298, // SWAVeloMax_si16
        26 // RWheel_ui16
      }
    },
    // cluster: APG_XPG_SafetyDistParallel
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        5, // MinLowLatRef_si16
        15, // TargetLowLatRef_si16
        20, // MinHighLatRef_si16
        30, // TargetHighLatRef_si16
        -10, // MinVirtLatRef_si16
        0, // TargetVirtLatRef_si16
        25, // Front_si16
        25, // Rear_si16
        25, // Corner_si16
        20, // Side_si16
        30, // OppositeSide_si16
        50, // ParkingSide_si16
        6 // EarlyPosOkLatDistExt_si16
      }
    },
    // cluster: APG_XPG_SafetyDistCross
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        40, // MinTargetSideDist_si16
        30, // Side1stMove_si16
        25, // SideNMove_si16
        40, // Object1_ui8
        40, // OppositeSide_si16
        15, // Corner1_si16
        45 // Side_FI_si16
      }
    },
    // cluster: APG_XPG_Ctrl
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        71, // TargetPosTolPsi_ui16
        10, // TargetTolX_CPSC_ui8
        86, // TargetTolPsi_CPSC_ui16
        20, // VeloStandstill_ui16
        2000, // VeloAbort_ui16
        700, // VeloSlowDown_ui16
        100, // VeloSlowDownHyst_ui16
        11, // MaxNumOfMovesPSC_ui8
        7, // MaxNumOfMovesCPSC_ui8
        10, // MaxNumOfMovesCPSC_FI_ui8
        130, // MinLatDistForNMove_si16
        20, // StopVeloHyst_ui16
        300, // ReactionTime_ui16
        500, // Deceleration_ui16
        1, // TPR_Prediction_si8
        600, // MaxUncontrolledTravelDist_si16
        50, // MaxPosDev_ui16
        30, // MaxDrivenDistOnPASDegraded_ui16
        30000, // DeactTimePathBlockedByObstacle_ui16
        7320441, // MinYawAngleForEarlyStopOnInsufficientSpace_ui32
        50 // BestGuessPSShiftInBlockadeSituation_ui8
      }
    },
    // cluster: APG_XPG_NMove
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        -16299, // SWAForward_si16
        16442, // SWABackwardMin_si16
        16442, // SWABackwardMax_si16
        180, // StraightenClothoidVelo_si16
        10, // MinLengthOfLongitudinalAlignmentMove_si16
        0, // MaxLengthOfLongitudinalAlignmentMove_ui8
        78, // AlignmentDistFrontMax_si16
        0, // MaxDist2RearRef_ui8
        0 // MaxDist2FrontRef_ui8
      }
    },
    // cluster: APG_XPG_Parkable
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        2502, // EndPsiMaxF12_si16
        300, // VeloHyst_si16
        10, // DistHyst_ui16
        300, // ReactionTime_ui16
        500 // Deceleration_ui16
      }
    },
    // cluster: APG_APG_Pathplanning_Parallel
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        16442, // SWA3Min_si16
        16442, // SWA3Max_si16
        16442, // SWA6_si16
        500, // Velo0_si16
        500, // Velo2_si16
        900, // Velo2Max_si16
        500, // Velo4_si16
        800, // Velo4Max_si16
        400, // Velo5_si16
        300, // Velo7_si16
        80, // AddSafetyDistFrontMax_si16
        614 // SafetyDistFrontIncreaseRatioF10_si16
      }
    },
    // cluster: APG_APG_Steer_Ctrl
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        50, // DevPosP_si16
        33, // DevPosI_si16
        50, // DevPosT_ui16
        150, // DevPsiP_si16
        100, // DevPsiI_si16
        50, // DevPsiT_ui16
        20, // DevPosMax_si16
        0, // VeloMax_si16
        0, // SWAFilterT_ui16
        1000, // SWAFilterTStandstill_ui16
        200 // TargetPosFilterT_ui16
      }
    },
    // cluster: APG_APG_Ctrl
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        100, // DeadTime_ui8
        100, // EPSHoldTimeBeforeRelease_ui16
        3500, // EPSTimeoutStraightening_ui16
        16442, // SWASpecLimit_si16
        14298, // SWAVeloStandStillMax_ui16
        14298, // SWAVeloSpecLimit_ui16
        35744, // SWAAccMaxStandstill_ui16
        89088, // LimiterFilter_SWAAccMax_si32
        8936, // MaxSWADev_ui16
        1072, // SWATolStandstill_ui16
        357 // TargetSWAStraightening_ui16
      }
    },
    // cluster: APG_APG_Pathplanning_Cross
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        300, // VeloForwMove_si16
        200, // VeloStraighten_si16
        300, // VeloBackMove_si16
        30, // XLeftTolPer_ui8
        30, // XLeftTolPerNMove_ui8
        307, // XLeftTolMaxF10_ui16
        10, // PSDepthOffset_si16
        50, // CornerLatDiffMax_ui8
        16085, // SWAMax_si16
        3217, // SWA2ndMin_si16
        20, // ToleranceDistForEarlyGearSwitch_ui8
        -10, // XLimitFirstBackwardMove_si16
        1, // AlwaysUseSwitchPoint_bl
        -500, // MaxDepthForSwitchPoint_si16
        1, // Move2Optimization_bl
        950, // VirtualOppositeSide_si16
        400, // ThresholdOneSideAlignment_ui16
        70, // TargetDistSideAlignment_si16
        30, // TargetDist2LatRef_si16
        50 // MaxDist2LatRefFullWarn_si16
      }
    },
    // cluster: APG_APG_PathplanCPSC_FI
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        400, // VeloForwMoveFast_si16
        32, // FinalPosXTolPer_ui8
        307, // FinalPosXTolMaxF10_ui16
        75, // TargetDistSideAlignment_si16
        35, // SafetyDistCornerAndSide_si16
        35, // SafetyDistParkingSideBackwMove_si16
        30, // SafetyDistParkingSideForwMove_si16
        8192, // VirtualOppositeSideMove3F10_si16
        9728, // VirtualOppositeSideMove1F10_si16
        7168, // MaxLengthStraightBackwMoveF10_ui16
        -1787, // MaxStartYawForStraightBackwMove_si16
        -2355, // MinDepthForVehicleInsidePS_si16
        512, // MaxYStraightBackwMove_si16
        -3288, // ConductorMoveTargetYaw_si16
        -5719, // ConductorMovePreferredYaw_si16
        1, // SteerLeftInBackwMoveInPS_bl
        1, // OptimizeBackwMoveEndYaw_bl
        0, // ForceForwardMove_bl
        -8192 // XLimit1stBackwMove_si16
      }
    },
    // cluster: APG_APG_POC
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        25, // ClothoidLength_si16
        16442, // MaxSWA_si16
        2502, // SWAOvershoot_si16
        15, // MaxNumOfMoves_ui8
        750, // MaxMoveLength_si16
        90, // SteerThresStraightRevMove_ui8
        40, // SteerThresStraightRevMoveMin_ui8
        5 // LatRefDistInitial_ui16
      }
    },
    // cluster: APG_CPSC
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        3072, // BO_StraightMoveLength_ui16
        3072 // FO_StraightMoveLength_ui16
      }
    },
    // cluster: APG_PL_POC_Handover
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        30, // SafetyMargin_ui8
        200, // DeactThreshold_MaxExtensionFrontCorner_ui16
        6222375, // DeactThreshold_MaxYawAngle_ui32
        2196132 // YawAngleThreshold_ui32
      }
    },
    // cluster: PAGLOBAL_SysFct
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        255, // PpParallelFunc_bf8 : 8
        0 // RearAxleSteering_bl
      }
    },
    // cluster: VHO_Veh
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        // DifFrontWheelRadiiTable_pui16[33]
        {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0
        },
        2, // DrivenAxle_ui8
        2720, // WheelBase_ui16
        1875, // AxleBaseRear_ui16
        96, // WICTeethCount_ui16
        5475, // WICLength_ui16
        1023, // MaxWICValue_ui16
        115, // MultMonomA_ForwSWA2SA_si16
        -26, // MultMonomB_ForwSWA2SA_si16
        42781, // CoeffC_ForwSWA2SA_si32
        6138, // CoeffD_ForwSWA2SA_si32
        102, // MultMonomA_BackSWA2SA_si16
        99, // MultMonomB_BackSWA2SA_si16
        44533, // CoeffC_BackSWA2SA_si32
        0, // CoeffD_BackSWA2SA_si32
        3, // MultMonomA_ForwSA2SWAL_si16
        442, // MultMonomB_ForwSA2SWAL_si16
        67585, // CoeffC_ForwSA2SWAL_si32
        2288263, // CoeffD_ForwSA2SWAL_si32
        -20, // MultMonomA_BackSA2SWAL_si16
        31, // MultMonomB_BackSA2SWAL_si16
        59690, // CoeffC_BackSA2SWAL_si32
        282590, // CoeffD_BackSA2SWAL_si32
        -137, // MultMonomA_ForwSA2SWAC_si16
        -199, // MultMonomB_ForwSA2SWAC_si16
        62121, // CoeffC_ForwSA2SWAC_si32
        -3168, // CoeffD_ForwSA2SWAC_si32
        -31, // MultMonomA_BackSA2SWAC_si16
        -147, // MultMonomB_BackSA2SWAC_si16
        59073, // CoeffC_BackSA2SWAC_si32
        -610, // CoeffD_BackSA2SWAC_si32
        9, // MultMonomA_ForwSA2SWAR_si16
        -475, // MultMonomB_ForwSA2SWAR_si16
        67336, // CoeffC_ForwSA2SWAR_si32
        -2165034, // CoeffD_ForwSA2SWAR_si32
        -16, // MultMonomA_BackSA2SWAR_si16
        -70, // MultMonomB_BackSA2SWAR_si16
        59775, // CoeffC_BackSA2SWAR_si32
        -317679, // CoeffD_BackSA2SWAR_si32
        -1118, // ClippingAngle1_si16
        1126, // ClippingAngle2_si16
        2, // ScalingMonomA_SA2SWAL_bf3 : 3
        2, // ScalingMonomB_SA2SWAL_bf3 : 3
        0, // ScalingMonomCD_SA2SWAL_bf3 : 3
        2, // ScalingMonomA_SA2SWAC_bf3 : 3
        3, // ScalingMonomB_SA2SWAC_bf3 : 3
        0, // ScalingMonomCD_SA2SWAC_bf3 : 3
        2, // ScalingMonomA_SA2SWAR_bf3 : 3
        2, // ScalingMonomB_SA2SWAR_bf3 : 3
        0, // ScalingMonomCD_SA2SWAR_bf3 : 3
        1, // ScalingMonomA_SWA2SA_bf3 : 3
        3, // ScalingMonomB_SWA2SA_bf3 : 3
        0, // ScalingMonomCD_SWA2SA_bf3 : 3
        20 // WIC_CycleTime_MS_ui8
      }
    },
    // cluster: VHO_Odo
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        1, // WeightSingleTrack_ui8
        1, // WeightDualTrack_ui8
        2, // WeightDualTrackRear_ui8
        1, // WeightDualTrackFront_ui8
        102400, // MaxDistDiscarded_ui32
        128000, // MaxDistDirUndef_ui32
        8, // VCALPhaseTol_ui16
        4, // VCALVeloDependentPhaseTol_ui16
        1, // VCALMinDeltaZeroAllowed_bl
        1, // WeightDistanceFront_ui8
        1, // WeightDistanceRear_ui8
        100, // VIRTUAL_LSCAN_TCYC_MS_ui16
        7, // VIRTUAL_LSCAN_TTOL_MS_ui16
        30, // YawAngSmoothKoeffMin_ui8
        90, // YawAngSmoothKoeffMax_ui8
        30, // EngineRestartExtrapolationDistance_ui8
        720, // EngineRestartExtrapolationTimeout_ui16
        11, // SWADelayUnitCM_ui8
        2500, // SWAMaxDeviationDeg_si16
        2, // DirDetectMode_ui8
        1, // MinWICDirForMovDir_ui8
        0, // MinDistAftStdstlNoWICDir_ui8
        32942 // MaxYawAngCorrAtStandstill_ui32
      }
    },
    // cluster: MP_BDA_Block1
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        // DataBlock1_pui8[9]
        {
          16, 66, 8, 33, 132, 16, 66, 152, 6
        }
      }
    },
    // cluster: MP_BDA_Block2
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        // DataBlock2_pui8[7]
        {
          255, 255, 255, 255, 255, 64, 38
        }
      }
    },
    // cluster: MP_BDA_Block3
    {
      // global var(s): DAI_BR245 | min var: not labeled | sub var: not labeled
      {
        // DataBlock3_pui8[5]
        {
          176, 229, 172, 103, 11
        }
      }
    }
  }
};


//#pragma ghs section rodata = ".pa_const"
//#define PAR_STOP_SEC_REF
//#include "par_mem_map.h"


#elif (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_APPL_ONLY)
  // not included in this configuration
#else
  #error 'invalid configuration!'
#endif


//
// global meta data ROM
//

#if ( (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_STAND_ALONE) || (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_APPL_ONLY) )


//#define PAR_START_SEC_REF
//#include "par_mem_map.h"
//#pragma ghs section rodata = ".CAL_Const"
const gType_parROMMetaData_st gc_parROMMetaData_st =
{
  //
  // indirection table for parameter memory
  //


  {
    &(g_parMemoryData_st.ParameterMemoryROM_st.VHOSRC_PluginSRC_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.VHOTCE_PluginTTC_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.VHOTCE_PluginTCE_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.VHOTCE_PluginTCE_clustering_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PS_Config_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PS_Speed_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PS_Brake_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PS_Objects_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.BDA_Parameter_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSD_Veh_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSD_Proj_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSD_Proj_OCP_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSD_Proj_Cross_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSD_RuleSet1_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSD_Proj_VC_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSD_Proj_SPR_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_USS_FISelection_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_USS_FIBlock1_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_USS_FIBlock2_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_USS_RISelection_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_USS_RIBlock1_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_USS_RIBlock2_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_USS_FOSelection_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_USS_FOBlock1_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_USS_FOBlock2_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_USS_ROSelection_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_USS_ROBlock1_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_USS_ROBlock2_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_USS_CcSetInfo_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.APCTL_CFG_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.DISPLAY_Veh_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSM_DiscardCriteria_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSM_DiscardCriteriaCPSC_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSM_ParkableCriteria_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSM_ParkableCriteriaCPSC_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSM_FilterCriteria_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSM_ParametersDPSC_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_SensorTimes_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_SensorStatic_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_PdcMeasGroupParId_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_Timings_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_SensorType_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_SensorOrientation_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_RingTimeInfor_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_RingTimeRange_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_SensorPort_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.OD_General_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.OD_MaskActVelo_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.OD_IdleDist_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.OD_Sens_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.OD_Veh_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.OD_Echo_ApFlt_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.OD_Echo_WRF_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.OD_Echo_SRF_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.OD_Echo_HLF_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.OD_Echo_MoveDir_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.OD_Echo_Clip_Std_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.OD_Obj_Meas_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.OD_Obj_Trck_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.SP_PWR_CFG_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.VHS_PL_TORQUE_Controller_ZFLS_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.VHS_PL_TORQUE_Controller_CR_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_BSD_Ap_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.EP_BSD_Ep_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.EP_BSD_SDI_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_General_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_Disp_Global_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_Disp_Lat_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_View_Lat_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_View_Prm_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_Offset_Std_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_Offset_Lat_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_SC_DT_Std_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_SC_DT_Lat_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_SC_DU_Lat_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_SC_DU_Prm_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_SC_OE_Std_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_SC_OE_Lat_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_SC_SWD_Std_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_SC_SWD_Lat_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_SC_SWD_Prm_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_SC_WBL_Prm_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_AO_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.AP_PDC_Danger_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSU_CPSC_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSU_PSC_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSU_POC_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSU_CPX_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSU_Object_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PSU_Reference_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.APG_XPG_Vehicle_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.APG_XPG_SafetyDistParallel_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.APG_XPG_SafetyDistCross_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.APG_XPG_Ctrl_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.APG_XPG_NMove_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.APG_XPG_Parkable_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.APG_APG_Pathplanning_Parallel_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.APG_APG_Steer_Ctrl_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.APG_APG_Ctrl_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.APG_APG_Pathplanning_Cross_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.APG_APG_PathplanCPSC_FI_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.APG_APG_POC_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.APG_CPSC_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.APG_PL_POC_Handover_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.PAGLOBAL_SysFct_pst[0]),
    &(g_parRAMData_st.ParameterMemoryRAM_st.SIP_VHOParaAutocalib_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.VHO_Veh_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.VHO_Odo_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_BDA_Block1_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_BDA_Block2_pst[0]),
    &(g_parMemoryData_st.ParameterMemoryROM_st.MP_BDA_Block3_pst[0])
  },



  //
  // RAM default copy table
  //

  #if ( GS_PAR_CFG_INIT_NVM_MIRROR_MEM == SW_ON )



  {
    {
      NULL,
      NULL,
      0,
      {0},
      {0}
    }
  },



  #endif


  //
  // application memory cluster pointer
  //

  #if ( GS_PAR_CFG_APPLICATION_MODE == SW_ON )

  {
    &(g_parApplicationMemory_st.VHOSRC_PluginSRC_st),
    &(g_parApplicationMemory_st.VHOTCE_PluginTTC_st),
    &(g_parApplicationMemory_st.VHOTCE_PluginTCE_st),
    &(g_parApplicationMemory_st.VHOTCE_PluginTCE_clustering_st),
    &(g_parApplicationMemory_st.AP_PS_Config_st),
    &(g_parApplicationMemory_st.AP_PS_Speed_st),
    &(g_parApplicationMemory_st.AP_PS_Brake_st),
    &(g_parApplicationMemory_st.AP_PS_Objects_st),
    &(g_parApplicationMemory_st.BDA_Parameter_st),
    &(g_parApplicationMemory_st.PSD_Veh_st),
    &(g_parApplicationMemory_st.PSD_Proj_st),
    &(g_parApplicationMemory_st.PSD_Proj_OCP_st),
    &(g_parApplicationMemory_st.PSD_Proj_Cross_st),
    &(g_parApplicationMemory_st.PSD_RuleSet1_st),
    &(g_parApplicationMemory_st.PSD_Proj_VC_st),
    &(g_parApplicationMemory_st.PSD_Proj_SPR_st),
    &(g_parApplicationMemory_st.MP_USS_FISelection_st),
    &(g_parApplicationMemory_st.MP_USS_FIBlock1_st),
    &(g_parApplicationMemory_st.MP_USS_FIBlock2_st),
    &(g_parApplicationMemory_st.MP_USS_RISelection_st),
    &(g_parApplicationMemory_st.MP_USS_RIBlock1_st),
    &(g_parApplicationMemory_st.MP_USS_RIBlock2_st),
    &(g_parApplicationMemory_st.MP_USS_FOSelection_st),
    &(g_parApplicationMemory_st.MP_USS_FOBlock1_st),
    &(g_parApplicationMemory_st.MP_USS_FOBlock2_st),
    &(g_parApplicationMemory_st.MP_USS_ROSelection_st),
    &(g_parApplicationMemory_st.MP_USS_ROBlock1_st),
    &(g_parApplicationMemory_st.MP_USS_ROBlock2_st),
    &(g_parApplicationMemory_st.MP_USS_CcSetInfo_st),
    &(g_parApplicationMemory_st.APCTL_CFG_st),
    &(g_parApplicationMemory_st.DISPLAY_Veh_st),
    &(g_parApplicationMemory_st.PSM_DiscardCriteria_st),
    &(g_parApplicationMemory_st.PSM_DiscardCriteriaCPSC_st),
    &(g_parApplicationMemory_st.PSM_ParkableCriteria_st),
    &(g_parApplicationMemory_st.PSM_ParkableCriteriaCPSC_st),
    &(g_parApplicationMemory_st.PSM_FilterCriteria_st),
    &(g_parApplicationMemory_st.PSM_ParametersDPSC_st),
    &(g_parApplicationMemory_st.MP_SensorTimes_pst[0]),
    &(g_parApplicationMemory_st.MP_SensorStatic_pst[0]),
    &(g_parApplicationMemory_st.MP_PdcMeasGroupParId_st),
    &(g_parApplicationMemory_st.MP_Timings_st),
    &(g_parApplicationMemory_st.MP_SensorType_st),
    &(g_parApplicationMemory_st.MP_SensorOrientation_st),
    &(g_parApplicationMemory_st.MP_RingTimeInfor_st),
    &(g_parApplicationMemory_st.MP_RingTimeRange_st),
    &(g_parApplicationMemory_st.MP_SensorPort_st),
    &(g_parApplicationMemory_st.OD_General_st),
    &(g_parApplicationMemory_st.OD_MaskActVelo_st),
    &(g_parApplicationMemory_st.OD_IdleDist_st),
    &(g_parApplicationMemory_st.OD_Sens_st),
    &(g_parApplicationMemory_st.OD_Veh_st),
    &(g_parApplicationMemory_st.OD_Echo_ApFlt_st),
    &(g_parApplicationMemory_st.OD_Echo_WRF_st),
    &(g_parApplicationMemory_st.OD_Echo_SRF_st),
    &(g_parApplicationMemory_st.OD_Echo_HLF_st),
    &(g_parApplicationMemory_st.OD_Echo_MoveDir_st),
    &(g_parApplicationMemory_st.OD_Echo_Clip_Std_st),
    &(g_parApplicationMemory_st.OD_Obj_Meas_st),
    &(g_parApplicationMemory_st.OD_Obj_Trck_st),
    &(g_parApplicationMemory_st.SP_PWR_CFG_st),
    &(g_parApplicationMemory_st.VHS_PL_TORQUE_Controller_ZFLS_st),
    &(g_parApplicationMemory_st.VHS_PL_TORQUE_Controller_CR_st),
    &(g_parApplicationMemory_st.AP_BSD_Ap_st),
    &(g_parApplicationMemory_st.EP_BSD_Ep_st),
    &(g_parApplicationMemory_st.EP_BSD_SDI_st),
    &(g_parApplicationMemory_st.AP_PDC_General_st),
    &(g_parApplicationMemory_st.AP_PDC_Disp_Global_st),
    &(g_parApplicationMemory_st.AP_PDC_Disp_Lat_st),
    &(g_parApplicationMemory_st.AP_PDC_View_Lat_st),
    &(g_parApplicationMemory_st.AP_PDC_View_Prm_st),
    &(g_parApplicationMemory_st.AP_PDC_Offset_Std_st),
    &(g_parApplicationMemory_st.AP_PDC_Offset_Lat_st),
    &(g_parApplicationMemory_st.AP_PDC_SC_DT_Std_st),
    &(g_parApplicationMemory_st.AP_PDC_SC_DT_Lat_st),
    &(g_parApplicationMemory_st.AP_PDC_SC_DU_Lat_st),
    &(g_parApplicationMemory_st.AP_PDC_SC_DU_Prm_st),
    &(g_parApplicationMemory_st.AP_PDC_SC_OE_Std_st),
    &(g_parApplicationMemory_st.AP_PDC_SC_OE_Lat_st),
    &(g_parApplicationMemory_st.AP_PDC_SC_SWD_Std_st),
    &(g_parApplicationMemory_st.AP_PDC_SC_SWD_Lat_st),
    &(g_parApplicationMemory_st.AP_PDC_SC_SWD_Prm_st),
    &(g_parApplicationMemory_st.AP_PDC_SC_WBL_Prm_st),
    &(g_parApplicationMemory_st.AP_PDC_AO_st),
    &(g_parApplicationMemory_st.AP_PDC_Danger_st),
    &(g_parApplicationMemory_st.PSU_CPSC_st),
    &(g_parApplicationMemory_st.PSU_PSC_st),
    &(g_parApplicationMemory_st.PSU_POC_st),
    &(g_parApplicationMemory_st.PSU_CPX_st),
    &(g_parApplicationMemory_st.PSU_Object_st),
    &(g_parApplicationMemory_st.PSU_Reference_st),
    &(g_parApplicationMemory_st.APG_XPG_Vehicle_st),
    &(g_parApplicationMemory_st.APG_XPG_SafetyDistParallel_st),
    &(g_parApplicationMemory_st.APG_XPG_SafetyDistCross_st),
    &(g_parApplicationMemory_st.APG_XPG_Ctrl_st),
    &(g_parApplicationMemory_st.APG_XPG_NMove_st),
    &(g_parApplicationMemory_st.APG_XPG_Parkable_st),
    &(g_parApplicationMemory_st.APG_APG_Pathplanning_Parallel_st),
    &(g_parApplicationMemory_st.APG_APG_Steer_Ctrl_st),
    &(g_parApplicationMemory_st.APG_APG_Ctrl_st),
    &(g_parApplicationMemory_st.APG_APG_Pathplanning_Cross_st),
    &(g_parApplicationMemory_st.APG_APG_PathplanCPSC_FI_st),
    &(g_parApplicationMemory_st.APG_APG_POC_st),
    &(g_parApplicationMemory_st.APG_CPSC_st),
    &(g_parApplicationMemory_st.APG_PL_POC_Handover_st),
    &(g_parApplicationMemory_st.PAGLOBAL_SysFct_st),
    &(g_parApplicationMemory_st.SIP_VHOParaAutocalib_st),
    &(g_parApplicationMemory_st.VHO_Veh_st),
    &(g_parApplicationMemory_st.VHO_Odo_st),
    &(g_parApplicationMemory_st.MP_BDA_Block1_st),
    &(g_parApplicationMemory_st.MP_BDA_Block2_st),
    &(g_parApplicationMemory_st.MP_BDA_Block3_st)
  },

  #endif


  //
  // cluster size
  //

  {
    sizeof (gType_parVHOSRC_PluginSRC_st),
    sizeof (gType_parVHOTCE_PluginTTC_st),
    sizeof (gType_parVHOTCE_PluginTCE_st),
    sizeof (gType_parVHOTCE_PluginTCE_clustering_st),
    sizeof (gType_parAP_PS_Config_st),
    sizeof (gType_parAP_PS_Speed_st),
    sizeof (gType_parAP_PS_Brake_st),
    sizeof (gType_parAP_PS_Objects_st),
    sizeof (gType_parBDA_Parameter_st),
    sizeof (gType_parPSD_Veh_st),
    sizeof (gType_parPSD_Proj_st),
    sizeof (gType_parPSD_Proj_OCP_st),
    sizeof (gType_parPSD_Proj_Cross_st),
    sizeof (gType_parPSD_RuleSet1_st),
    sizeof (gType_parPSD_Proj_VC_st),
    sizeof (gType_parPSD_Proj_SPR_st),
    sizeof (gType_parMP_USS_FISelection_st),
    sizeof (gType_parMP_USS_FIBlock1_st),
    sizeof (gType_parMP_USS_FIBlock2_st),
    sizeof (gType_parMP_USS_RISelection_st),
    sizeof (gType_parMP_USS_RIBlock1_st),
    sizeof (gType_parMP_USS_RIBlock2_st),
    sizeof (gType_parMP_USS_FOSelection_st),
    sizeof (gType_parMP_USS_FOBlock1_st),
    sizeof (gType_parMP_USS_FOBlock2_st),
    sizeof (gType_parMP_USS_ROSelection_st),
    sizeof (gType_parMP_USS_ROBlock1_st),
    sizeof (gType_parMP_USS_ROBlock2_st),
    sizeof (gType_parMP_USS_CcSetInfo_st),
    sizeof (gType_parAPCTL_CFG_st),
    sizeof (gType_parDISPLAY_Veh_st),
    sizeof (gType_parPSM_DiscardCriteria_st),
    sizeof (gType_parPSM_DiscardCriteriaCPSC_st),
    sizeof (gType_parPSM_ParkableCriteria_st),
    sizeof (gType_parPSM_ParkableCriteriaCPSC_st),
    sizeof (gType_parPSM_FilterCriteria_st),
    sizeof (gType_parPSM_ParametersDPSC_st),
    sizeof (gType_parMP_SensorTimes_st),
    sizeof (gType_parMP_SensorStatic_st),
    sizeof (gType_parMP_PdcMeasGroupParId_st),
    sizeof (gType_parMP_Timings_st),
    sizeof (gType_parMP_SensorType_st),
    sizeof (gType_parMP_SensorOrientation_st),
    sizeof (gType_parMP_RingTimeInfor_st),
    sizeof (gType_parMP_RingTimeRange_st),
    sizeof (gType_parMP_SensorPort_st),
    sizeof (gType_parOD_General_st),
    sizeof (gType_parOD_MaskActVelo_st),
    sizeof (gType_parOD_IdleDist_st),
    sizeof (gType_parOD_Sens_st),
    sizeof (gType_parOD_Veh_st),
    sizeof (gType_parOD_Echo_ApFlt_st),
    sizeof (gType_parOD_Echo_WRF_st),
    sizeof (gType_parOD_Echo_SRF_st),
    sizeof (gType_parOD_Echo_HLF_st),
    sizeof (gType_parOD_Echo_MoveDir_st),
    sizeof (gType_parOD_Echo_Clip_Std_st),
    sizeof (gType_parOD_Obj_Meas_st),
    sizeof (gType_parOD_Obj_Trck_st),
    sizeof (gType_parSP_PWR_CFG_st),
    sizeof (gType_parVHS_PL_TORQUE_Controller_ZFLS_st),
    sizeof (gType_parVHS_PL_TORQUE_Controller_CR_st),
    sizeof (gType_parAP_BSD_Ap_st),
    sizeof (gType_parEP_BSD_Ep_st),
    sizeof (gType_parEP_BSD_SDI_st),
    sizeof (gType_parAP_PDC_General_st),
    sizeof (gType_parAP_PDC_Disp_Global_st),
    sizeof (gType_parAP_PDC_Disp_Lat_st),
    sizeof (gType_parAP_PDC_View_Lat_st),
    sizeof (gType_parAP_PDC_View_Prm_st),
    sizeof (gType_parAP_PDC_Offset_Std_st),
    sizeof (gType_parAP_PDC_Offset_Lat_st),
    sizeof (gType_parAP_PDC_SC_DT_Std_st),
    sizeof (gType_parAP_PDC_SC_DT_Lat_st),
    sizeof (gType_parAP_PDC_SC_DU_Lat_st),
    sizeof (gType_parAP_PDC_SC_DU_Prm_st),
    sizeof (gType_parAP_PDC_SC_OE_Std_st),
    sizeof (gType_parAP_PDC_SC_OE_Lat_st),
    sizeof (gType_parAP_PDC_SC_SWD_Std_st),
    sizeof (gType_parAP_PDC_SC_SWD_Lat_st),
    sizeof (gType_parAP_PDC_SC_SWD_Prm_st),
    sizeof (gType_parAP_PDC_SC_WBL_Prm_st),
    sizeof (gType_parAP_PDC_AO_st),
    sizeof (gType_parAP_PDC_Danger_st),
    sizeof (gType_parPSU_CPSC_st),
    sizeof (gType_parPSU_PSC_st),
    sizeof (gType_parPSU_POC_st),
    sizeof (gType_parPSU_CPX_st),
    sizeof (gType_parPSU_Object_st),
    sizeof (gType_parPSU_Reference_st),
    sizeof (gType_parAPG_XPG_Vehicle_st),
    sizeof (gType_parAPG_XPG_SafetyDistParallel_st),
    sizeof (gType_parAPG_XPG_SafetyDistCross_st),
    sizeof (gType_parAPG_XPG_Ctrl_st),
    sizeof (gType_parAPG_XPG_NMove_st),
    sizeof (gType_parAPG_XPG_Parkable_st),
    sizeof (gType_parAPG_APG_Pathplanning_Parallel_st),
    sizeof (gType_parAPG_APG_Steer_Ctrl_st),
    sizeof (gType_parAPG_APG_Ctrl_st),
    sizeof (gType_parAPG_APG_Pathplanning_Cross_st),
    sizeof (gType_parAPG_APG_PathplanCPSC_FI_st),
    sizeof (gType_parAPG_APG_POC_st),
    sizeof (gType_parAPG_CPSC_st),
    sizeof (gType_parAPG_PL_POC_Handover_st),
    sizeof (gType_parPAGLOBAL_SysFct_st),
    sizeof (gType_parSIP_VHOParaAutocalib_st),
    sizeof (gType_parVHO_Veh_st),
    sizeof (gType_parVHO_Odo_st),
    sizeof (gType_parMP_BDA_Block1_st),
    sizeof (gType_parMP_BDA_Block2_st),
    sizeof (gType_parMP_BDA_Block3_st)
  },


  //
  // sub variant application mode
  //

  #if ( GS_PAR_CFG_APPLICATION_MODE == SW_ON )

  {
    0,  // VHOSRC_PluginSRC
    0,  // VHOTCE_PluginTTC
    0,  // VHOTCE_PluginTCE
    0,  // VHOTCE_PluginTCE_clustering
    0,  // AP_PS_Config
    0,  // AP_PS_Speed
    0,  // AP_PS_Brake
    0,  // AP_PS_Objects
    0,  // BDA_Parameter
    0,  // PSD_Veh
    0,  // PSD_Proj
    0,  // PSD_Proj_OCP
    0,  // PSD_Proj_Cross
    0,  // PSD_RuleSet1
    0,  // PSD_Proj_VC
    0,  // PSD_Proj_SPR
    0,  // MP_USS_FISelection
    0,  // MP_USS_FIBlock1
    0,  // MP_USS_FIBlock2
    0,  // MP_USS_RISelection
    0,  // MP_USS_RIBlock1
    0,  // MP_USS_RIBlock2
    0,  // MP_USS_FOSelection
    0,  // MP_USS_FOBlock1
    0,  // MP_USS_FOBlock2
    0,  // MP_USS_ROSelection
    0,  // MP_USS_ROBlock1
    0,  // MP_USS_ROBlock2
    0,  // MP_USS_CcSetInfo
    0,  // APCTL_CFG
    0,  // DISPLAY_Veh
    0,  // PSM_DiscardCriteria
    0,  // PSM_DiscardCriteriaCPSC
    0,  // PSM_ParkableCriteria
    0,  // PSM_ParkableCriteriaCPSC
    0,  // PSM_FilterCriteria
    0,  // PSM_ParametersDPSC
    1,  // MP_SensorTimes
    1,  // MP_SensorStatic
    0,  // MP_PdcMeasGroupParId
    0,  // MP_Timings
    0,  // MP_SensorType
    0,  // MP_SensorOrientation
    0,  // MP_RingTimeInfor
    0,  // MP_RingTimeRange
    0,  // MP_SensorPort
    0,  // OD_General
    0,  // OD_MaskActVelo
    0,  // OD_IdleDist
    0,  // OD_Sens
    0,  // OD_Veh
    0,  // OD_Echo_ApFlt
    0,  // OD_Echo_WRF
    0,  // OD_Echo_SRF
    0,  // OD_Echo_HLF
    0,  // OD_Echo_MoveDir
    0,  // OD_Echo_Clip_Std
    0,  // OD_Obj_Meas
    0,  // OD_Obj_Trck
    0,  // SP_PWR_CFG
    0,  // VHS_PL_TORQUE_Controller_ZFLS
    0,  // VHS_PL_TORQUE_Controller_CR
    0,  // AP_BSD_Ap
    0,  // EP_BSD_Ep
    0,  // EP_BSD_SDI
    0,  // AP_PDC_General
    0,  // AP_PDC_Disp_Global
    0,  // AP_PDC_Disp_Lat
    0,  // AP_PDC_View_Lat
    0,  // AP_PDC_View_Prm
    0,  // AP_PDC_Offset_Std
    0,  // AP_PDC_Offset_Lat
    0,  // AP_PDC_SC_DT_Std
    0,  // AP_PDC_SC_DT_Lat
    0,  // AP_PDC_SC_DU_Lat
    0,  // AP_PDC_SC_DU_Prm
    0,  // AP_PDC_SC_OE_Std
    0,  // AP_PDC_SC_OE_Lat
    0,  // AP_PDC_SC_SWD_Std
    0,  // AP_PDC_SC_SWD_Lat
    0,  // AP_PDC_SC_SWD_Prm
    0,  // AP_PDC_SC_WBL_Prm
    0,  // AP_PDC_AO
    0,  // AP_PDC_Danger
    0,  // PSU_CPSC
    0,  // PSU_PSC
    0,  // PSU_POC
    0,  // PSU_CPX
    0,  // PSU_Object
    0,  // PSU_Reference
    0,  // APG_XPG_Vehicle
    0,  // APG_XPG_SafetyDistParallel
    0,  // APG_XPG_SafetyDistCross
    0,  // APG_XPG_Ctrl
    0,  // APG_XPG_NMove
    0,  // APG_XPG_Parkable
    0,  // APG_APG_Pathplanning_Parallel
    0,  // APG_APG_Steer_Ctrl
    0,  // APG_APG_Ctrl
    0,  // APG_APG_Pathplanning_Cross
    0,  // APG_APG_PathplanCPSC_FI
    0,  // APG_APG_POC
    0,  // APG_CPSC
    0,  // APG_PL_POC_Handover
    0,  // PAGLOBAL_SysFct
    0,  // SIP_VHOParaAutocalib
    0,  // VHO_Veh
    0,  // VHO_Odo
    0,  // MP_BDA_Block1
    0,  // MP_BDA_Block2
    0   // MP_BDA_Block3
  },

  #endif


  //
  // max sub variants
  //
  // 0 = SINGLE | 1 = MINOR_ALL | 2 = MAJOR_ALL
  //

  #if ( GS_PAR_CFG_APPLICATION_MODE == SW_ON )

  {
    1,  // VHOSRC_PluginSRC
    1,  // VHOTCE_PluginTTC
    1,  // VHOTCE_PluginTCE
    1,  // VHOTCE_PluginTCE_clustering
    1,  // AP_PS_Config
    1,  // AP_PS_Speed
    1,  // AP_PS_Brake
    1,  // AP_PS_Objects
    1,  // BDA_Parameter
    1,  // PSD_Veh
    1,  // PSD_Proj
    1,  // PSD_Proj_OCP
    1,  // PSD_Proj_Cross
    1,  // PSD_RuleSet1
    1,  // PSD_Proj_VC
    1,  // PSD_Proj_SPR
    1,  // MP_USS_FISelection
    1,  // MP_USS_FIBlock1
    1,  // MP_USS_FIBlock2
    1,  // MP_USS_RISelection
    1,  // MP_USS_RIBlock1
    1,  // MP_USS_RIBlock2
    1,  // MP_USS_FOSelection
    1,  // MP_USS_FOBlock1
    1,  // MP_USS_FOBlock2
    1,  // MP_USS_ROSelection
    1,  // MP_USS_ROBlock1
    1,  // MP_USS_ROBlock2
    1,  // MP_USS_CcSetInfo
    1,  // APCTL_CFG
    1,  // DISPLAY_Veh
    1,  // PSM_DiscardCriteria
    1,  // PSM_DiscardCriteriaCPSC
    1,  // PSM_ParkableCriteria
    1,  // PSM_ParkableCriteriaCPSC
    1,  // PSM_FilterCriteria
    1,  // PSM_ParametersDPSC
    6,  // MP_SensorTimes
    6,  // MP_SensorStatic
    1,  // MP_PdcMeasGroupParId
    1,  // MP_Timings
    1,  // MP_SensorType
    1,  // MP_SensorOrientation
    1,  // MP_RingTimeInfor
    1,  // MP_RingTimeRange
    1,  // MP_SensorPort
    1,  // OD_General
    1,  // OD_MaskActVelo
    1,  // OD_IdleDist
    1,  // OD_Sens
    1,  // OD_Veh
    1,  // OD_Echo_ApFlt
    1,  // OD_Echo_WRF
    1,  // OD_Echo_SRF
    1,  // OD_Echo_HLF
    1,  // OD_Echo_MoveDir
    1,  // OD_Echo_Clip_Std
    1,  // OD_Obj_Meas
    1,  // OD_Obj_Trck
    1,  // SP_PWR_CFG
    1,  // VHS_PL_TORQUE_Controller_ZFLS
    1,  // VHS_PL_TORQUE_Controller_CR
    1,  // AP_BSD_Ap
    1,  // EP_BSD_Ep
    1,  // EP_BSD_SDI
    1,  // AP_PDC_General
    1,  // AP_PDC_Disp_Global
    1,  // AP_PDC_Disp_Lat
    1,  // AP_PDC_View_Lat
    1,  // AP_PDC_View_Prm
    1,  // AP_PDC_Offset_Std
    1,  // AP_PDC_Offset_Lat
    1,  // AP_PDC_SC_DT_Std
    1,  // AP_PDC_SC_DT_Lat
    1,  // AP_PDC_SC_DU_Lat
    1,  // AP_PDC_SC_DU_Prm
    1,  // AP_PDC_SC_OE_Std
    1,  // AP_PDC_SC_OE_Lat
    1,  // AP_PDC_SC_SWD_Std
    1,  // AP_PDC_SC_SWD_Lat
    1,  // AP_PDC_SC_SWD_Prm
    1,  // AP_PDC_SC_WBL_Prm
    1,  // AP_PDC_AO
    1,  // AP_PDC_Danger
    1,  // PSU_CPSC
    1,  // PSU_PSC
    1,  // PSU_POC
    1,  // PSU_CPX
    1,  // PSU_Object
    1,  // PSU_Reference
    1,  // APG_XPG_Vehicle
    1,  // APG_XPG_SafetyDistParallel
    1,  // APG_XPG_SafetyDistCross
    1,  // APG_XPG_Ctrl
    1,  // APG_XPG_NMove
    1,  // APG_XPG_Parkable
    1,  // APG_APG_Pathplanning_Parallel
    1,  // APG_APG_Steer_Ctrl
    1,  // APG_APG_Ctrl
    1,  // APG_APG_Pathplanning_Cross
    1,  // APG_APG_PathplanCPSC_FI
    1,  // APG_APG_POC
    1,  // APG_CPSC
    1,  // APG_PL_POC_Handover
    1,  // PAGLOBAL_SysFct
    1,  // SIP_VHOParaAutocalib
    1,  // VHO_Veh
    1,  // VHO_Odo
    1,  // MP_BDA_Block1
    1,  // MP_BDA_Block2
    1   // MP_BDA_Block3
  },

  #endif


  //
  // major variant meta data
  //

  {
    { {   0 } },  // PluginSRC
    { {   1 } },  // PluginTTC
    { {   2 } },  // PluginTCE
    { {   3 } },  // PluginTCE_clustering
    { {   4 } },  // Config
    { {   5 } },  // Speed
    { {   6 } },  // Brake
    { {   7 } },  // Objects
    { {   8 } },  // Parameter
    { {   9 } },  // Veh
    { {  10 } },  // Proj
    { {  11 } },  // Proj_OCP
    { {  12 } },  // Proj_Cross
    { {  13 } },  // RuleSet1
    { {  14 } },  // Proj_VC
    { {  15 } },  // Proj_SPR
    { {  16 } },  // FISelection
    { {  17 } },  // FIBlock1
    { {  18 } },  // FIBlock2
    { {  19 } },  // RISelection
    { {  20 } },  // RIBlock1
    { {  21 } },  // RIBlock2
    { {  22 } },  // FOSelection
    { {  23 } },  // FOBlock1
    { {  24 } },  // FOBlock2
    { {  25 } },  // ROSelection
    { {  26 } },  // ROBlock1
    { {  27 } },  // ROBlock2
    { {  28 } },  // CcSetInfo
    { {  29 } },  // CFG
    { {  30 } },  // Veh
    { {  31 } },  // DiscardCriteria
    { {  32 } },  // DiscardCriteriaCPSC
    { {  33 } },  // ParkableCriteria
    { {  34 } },  // ParkableCriteriaCPSC
    { {  35 } },  // FilterCriteria
    { {  36 } },  // ParametersDPSC
    { {  37 } },  // SensorTimes
    { {  38 } },  // SensorStatic
    { {  39 } },  // PdcMeasGroupParId
    { {  40 } },  // Timings
    { {  41 } },  // SensorType
    { {  42 } },  // SensorOrientation
    { {  43 } },  // RingTimeInfor
    { {  44 } },  // RingTimeRange
    { {  45 } },  // SensorPort
    { {  46 } },  // General
    { {  47 } },  // MaskActVelo
    { {  48 } },  // IdleDist
    { {  49 } },  // Sens
    { {  50 } },  // Veh
    { {  51 } },  // Echo_ApFlt
    { {  52 } },  // Echo_WRF
    { {  53 } },  // Echo_SRF
    { {  54 } },  // Echo_HLF
    { {  55 } },  // Echo_MoveDir
    { {  56 } },  // Echo_Clip_Std
    { {  57 } },  // Obj_Meas
    { {  58 } },  // Obj_Trck
    { {  59 } },  // CFG
    { {  60 } },  // Controller_ZFLS
    { {  61 } },  // Controller_CR
    { {  62 } },  // Ap
    { {  63 } },  // Ep
    { {  64 } },  // SDI
    { {  65 } },  // General
    { {  66 } },  // Disp_Global
    { {  67 } },  // Disp_Lat
    { {  68 } },  // View_Lat
    { {  69 } },  // View_Prm
    { {  70 } },  // Offset_Std
    { {  71 } },  // Offset_Lat
    { {  72 } },  // SC_DT_Std
    { {  73 } },  // SC_DT_Lat
    { {  74 } },  // SC_DU_Lat
    { {  75 } },  // SC_DU_Prm
    { {  76 } },  // SC_OE_Std
    { {  77 } },  // SC_OE_Lat
    { {  78 } },  // SC_SWD_Std
    { {  79 } },  // SC_SWD_Lat
    { {  80 } },  // SC_SWD_Prm
    { {  81 } },  // SC_WBL_Prm
    { {  82 } },  // AO
    { {  83 } },  // Danger
    { {  84 } },  // CPSC
    { {  85 } },  // PSC
    { {  86 } },  // POC
    { {  87 } },  // CPX
    { {  88 } },  // Object
    { {  89 } },  // Reference
    { {  90 } },  // XPG_Vehicle
    { {  91 } },  // XPG_SafetyDistParallel
    { {  92 } },  // XPG_SafetyDistCross
    { {  93 } },  // XPG_Ctrl
    { {  94 } },  // XPG_NMove
    { {  95 } },  // XPG_Parkable
    { {  96 } },  // APG_Pathplanning_Parallel
    { {  97 } },  // APG_Steer_Ctrl
    { {  98 } },  // APG_Ctrl
    { {  99 } },  // APG_Pathplanning_Cross
    { { 100 } },  // APG_PathplanCPSC_FI
    { { 101 } },  // APG_POC
    { { 102 } },  // CPSC
    { { 103 } },  // Handover
    { { 104 } },  // SysFct
    { { 105 } },  // VHOParaAutocalib
    { { 106 } },  // Veh
    { { 107 } },  // Odo
    { { 108 } },  // Block1
    { { 109 } },  // Block2
    { { 110 } }   // Block3
  },


  //
  // minor variant meta data
  //

  {
    {   1,   0 },  // PluginSRC
    {   1,   1 },  // PluginTTC
    {   1,   2 },  // PluginTCE
    {   1,   3 },  // PluginTCE_clustering
    {   1,   4 },  // Config
    {   1,   5 },  // Speed
    {   1,   6 },  // Brake
    {   1,   7 },  // Objects
    {   1,   8 },  // Parameter
    {   1,   9 },  // Veh
    {   1,  10 },  // Proj
    {   1,  11 },  // Proj_OCP
    {   1,  12 },  // Proj_Cross
    {   1,  13 },  // RuleSet1
    {   1,  14 },  // Proj_VC
    {   1,  15 },  // Proj_SPR
    {   1,  16 },  // FISelection
    {   1,  17 },  // FIBlock1
    {   1,  18 },  // FIBlock2
    {   1,  19 },  // RISelection
    {   1,  20 },  // RIBlock1
    {   1,  21 },  // RIBlock2
    {   1,  22 },  // FOSelection
    {   1,  23 },  // FOBlock1
    {   1,  24 },  // FOBlock2
    {   1,  25 },  // ROSelection
    {   1,  26 },  // ROBlock1
    {   1,  27 },  // ROBlock2
    {   1,  28 },  // CcSetInfo
    {   2,  29 },  // CFG
    {   1,  31 },  // Veh
    {   1,  32 },  // DiscardCriteria
    {   1,  33 },  // DiscardCriteriaCPSC
    {   1,  34 },  // ParkableCriteria
    {   1,  35 },  // ParkableCriteriaCPSC
    {   1,  36 },  // FilterCriteria
    {   1,  37 },  // ParametersDPSC
    {   3,  38 },  // SensorTimes
    {   3,  41 },  // SensorStatic
    {   5,  44 },  // PdcMeasGroupParId
    {   1,  49 },  // Timings
    {   1,  50 },  // SensorType
    {   1,  51 },  // SensorOrientation
    {   1,  52 },  // RingTimeInfor
    {   1,  53 },  // RingTimeRange
    {   1,  54 },  // SensorPort
    {   1,  55 },  // General
    {   1,  56 },  // MaskActVelo
    {   2,  57 },  // IdleDist
    {   2,  59 },  // Sens
    {   2,  61 },  // Veh
    {   1,  63 },  // Echo_ApFlt
    {   1,  64 },  // Echo_WRF
    {   1,  65 },  // Echo_SRF
    {   1,  66 },  // Echo_HLF
    {   1,  67 },  // Echo_MoveDir
    {   1,  68 },  // Echo_Clip_Std
    {   1,  69 },  // Obj_Meas
    {   1,  70 },  // Obj_Trck
    {   1,  71 },  // CFG
    {   1,  72 },  // Controller_ZFLS
    {   1,  73 },  // Controller_CR
    {   1,  74 },  // Ap
    {   1,  75 },  // Ep
    {   1,  76 },  // SDI
    {   1,  77 },  // General
    {   1,  78 },  // Disp_Global
    {   1,  79 },  // Disp_Lat
    {   1,  80 },  // View_Lat
    {   1,  81 },  // View_Prm
    {   1,  82 },  // Offset_Std
    {   2,  83 },  // Offset_Lat
    {   1,  85 },  // SC_DT_Std
    {   1,  86 },  // SC_DT_Lat
    {   1,  87 },  // SC_DU_Lat
    {   1,  88 },  // SC_DU_Prm
    {   1,  89 },  // SC_OE_Std
    {   1,  90 },  // SC_OE_Lat
    {   1,  91 },  // SC_SWD_Std
    {   1,  92 },  // SC_SWD_Lat
    {   1,  93 },  // SC_SWD_Prm
    {   1,  94 },  // SC_WBL_Prm
    {   1,  95 },  // AO
    {   1,  96 },  // Danger
    {   1,  97 },  // CPSC
    {   1,  98 },  // PSC
    {   1,  99 },  // POC
    {   1, 100 },  // CPX
    {   1, 101 },  // Object
    {   1, 102 },  // Reference
    {   1, 103 },  // XPG_Vehicle
    {   1, 104 },  // XPG_SafetyDistParallel
    {   1, 105 },  // XPG_SafetyDistCross
    {   1, 106 },  // XPG_Ctrl
    {   1, 107 },  // XPG_NMove
    {   1, 108 },  // XPG_Parkable
    {   1, 109 },  // APG_Pathplanning_Parallel
    {   1, 110 },  // APG_Steer_Ctrl
    {   1, 111 },  // APG_Ctrl
    {   1, 112 },  // APG_Pathplanning_Cross
    {   1, 113 },  // APG_PathplanCPSC_FI
    {   1, 114 },  // APG_POC
    {   1, 115 },  // CPSC
    {   1, 116 },  // Handover
    {   1, 117 },  // SysFct
    {   1, 118 },  // VHOParaAutocalib
    {   1, 119 },  // Veh
    {   1, 120 },  // Odo
    {   1, 121 },  // Block1
    {   1, 122 },  // Block2
    {   1, 123 }   // Block3
  },


  //
  // sub variant meta data
  //

  {
    {   1,   0 },  // PluginSRC
    {   1,   0 },  // PluginTTC
    {   1,   0 },  // PluginTCE
    {   1,   0 },  // PluginTCE_clustering
    {   1,   0 },  // Config
    {   1,   0 },  // Speed
    {   1,   0 },  // Brake
    {   1,   0 },  // Objects
    {   1,   0 },  // Parameter
    {   1,   0 },  // Veh
    {   1,   0 },  // Proj
    {   1,   0 },  // Proj_OCP
    {   1,   0 },  // Proj_Cross
    {   1,   0 },  // RuleSet1
    {   1,   0 },  // Proj_VC
    {   1,   0 },  // Proj_SPR
    {   1,   0 },  // FISelection
    {   1,   0 },  // FIBlock1
    {   1,   0 },  // FIBlock2
    {   1,   0 },  // RISelection
    {   1,   0 },  // RIBlock1
    {   1,   0 },  // RIBlock2
    {  30,   0 },  // FOSelection
    {  30,   0 },  // FOBlock1
    {   4,   0 },  // FOBlock2
    {  30,   0 },  // ROSelection
    {  30,   0 },  // ROBlock1
    {   4,   0 },  // ROBlock2
    {   1,   0 },  // CcSetInfo
    {   1,   0 },  // CFG
    {   1,   1 },  // CFG
    {   1,   0 },  // Veh
    {   1,   0 },  // DiscardCriteria
    {   1,   0 },  // DiscardCriteriaCPSC
    {   1,   0 },  // ParkableCriteria
    {   1,   0 },  // ParkableCriteriaCPSC
    {   1,   0 },  // FilterCriteria
    {   1,   0 },  // ParametersDPSC
    {   6,   0 },  // SensorTimes
    {   6,   6 },  // SensorTimes
    {   6,  12 },  // SensorTimes
    {   6,   0 },  // SensorStatic
    {   6,   6 },  // SensorStatic
    {   6,  12 },  // SensorStatic
    {   1,   0 },  // PdcMeasGroupParId
    {   1,   1 },  // PdcMeasGroupParId
    {   1,   2 },  // PdcMeasGroupParId
    {   1,   3 },  // PdcMeasGroupParId
    {   1,   4 },  // PdcMeasGroupParId
    {   2,   0 },  // Timings
    {   1,   0 },  // SensorType
    {   1,   0 },  // SensorOrientation
    {   1,   0 },  // RingTimeInfor
    {   4,   0 },  // RingTimeRange
    {   1,   0 },  // SensorPort
    {   1,   0 },  // General
    {   1,   0 },  // MaskActVelo
    {   1,   0 },  // IdleDist
    {   1,   1 },  // IdleDist
    {   1,   0 },  // Sens
    {   1,   1 },  // Sens
    {   1,   0 },  // Veh
    {   1,   1 },  // Veh
    {   1,   0 },  // Echo_ApFlt
    {   1,   0 },  // Echo_WRF
    {   1,   0 },  // Echo_SRF
    {   1,   0 },  // Echo_HLF
    {   1,   0 },  // Echo_MoveDir
    {   1,   0 },  // Echo_Clip_Std
    {   1,   0 },  // Obj_Meas
    {   1,   0 },  // Obj_Trck
    {   1,   0 },  // CFG
    {   1,   0 },  // Controller_ZFLS
    {   1,   0 },  // Controller_CR
    {   1,   0 },  // Ap
    {   1,   0 },  // Ep
    {   1,   0 },  // SDI
    {   1,   0 },  // General
    {   1,   0 },  // Disp_Global
    {   1,   0 },  // Disp_Lat
    {   1,   0 },  // View_Lat
    {   1,   0 },  // View_Prm
    {   1,   0 },  // Offset_Std
    {   1,   0 },  // Offset_Lat
    {   1,   1 },  // Offset_Lat
    {   1,   0 },  // SC_DT_Std
    {   1,   0 },  // SC_DT_Lat
    {   1,   0 },  // SC_DU_Lat
    {   1,   0 },  // SC_DU_Prm
    {   1,   0 },  // SC_OE_Std
    {   1,   0 },  // SC_OE_Lat
    {   1,   0 },  // SC_SWD_Std
    {   1,   0 },  // SC_SWD_Lat
    {   1,   0 },  // SC_SWD_Prm
    {   1,   0 },  // SC_WBL_Prm
    {   1,   0 },  // AO
    {   1,   0 },  // Danger
    {   1,   0 },  // CPSC
    {   1,   0 },  // PSC
    {   1,   0 },  // POC
    {   1,   0 },  // CPX
    {   1,   0 },  // Object
    {   1,   0 },  // Reference
    {   1,   0 },  // XPG_Vehicle
    {   1,   0 },  // XPG_SafetyDistParallel
    {   1,   0 },  // XPG_SafetyDistCross
    {   1,   0 },  // XPG_Ctrl
    {   1,   0 },  // XPG_NMove
    {   1,   0 },  // XPG_Parkable
    {   1,   0 },  // APG_Pathplanning_Parallel
    {   1,   0 },  // APG_Steer_Ctrl
    {   1,   0 },  // APG_Ctrl
    {   1,   0 },  // APG_Pathplanning_Cross
    {   1,   0 },  // APG_PathplanCPSC_FI
    {   1,   0 },  // APG_POC
    {   1,   0 },  // CPSC
    {   1,   0 },  // Handover
    {   1,   0 },  // SysFct
    {   1,   0 },  // VHOParaAutocalib
    {   1,   0 },  // Veh
    {   1,   0 },  // Odo
    {   1,   0 },  // Block1
    {   1,   0 },  // Block2
    {   1,   0 }   // Block3
  }
};


//#pragma ghs section rodata = ".pa_const"
//#define PAR_STOP_SEC_REF
//#include "par_mem_map.h"

#elif (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_CAL_ONLY)
  // not included in this configuration
#else
  #error 'invalid configuration!'
#endif
