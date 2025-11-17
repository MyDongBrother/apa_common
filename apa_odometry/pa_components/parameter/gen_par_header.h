/*****************************************************************************
 | C O P Y R I G H T
 |-----------------------------------------------------------------------------
 | Copyright (c) 2023 by Robert Bosch GmbH.                All rights reserved.
 |
 | This file is property of Robert Bosch GmbH. Any unauthorised copy, use or
 | distribution is an offensive act against international law and may me
 | prosecuted under federal law. Its content is company confidential.
 |-----------------------------------------------------------------------------
 | D E S C R I P T I O N
 |-----------------------------------------------------------------------------
 | $ProjectName: d:/MKS_Data/Projects/PPJ/PP_GEN5/PP_SIP5_Fx4/SW/SRC/PA/DFS_PRJ/SRC/TOOLS/PARGen/pc_pp_pargen.pj $
 |      $Source: Program.cs $
 |    $Revision: 1.220 $
 |        $Date: 2012/11/12 16:37:54CST $
 |       $State: in_work $
 |
 |      Purpose: generated component configuration file
 |-----------------------------------------------------------------------------
 | I N F O
 |-----------------------------------------------------------------------------
 | File:                  gen_par_header.h
 | Input File:            ..\appl_sip_param.xml
 | Version:               5.8
 | Date:                  2023.5.11 - 15:12
 |
 | Generation Date (UTC): 2023.05.11 - 07:13
 *****************************************************************************/

#ifndef GEN_PAR_H
#define GEN_PAR_H
/* clang-format off */
/*--------------------------------------------------------------------------*/
/*- magic number                                                           -*/
/*--------------------------------------------------------------------------*/

#define GEN_PAR_HEADER_MAGIC_NUMBER  0x05110713

/*--------------------------------------------------------------------------*/
/*- component switches                                                     -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/

#include <stdlib.h>

/*--------------------------------------------------------------------------*/
/*- check if compiler switches (utilised in this component) are defined    -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- exported symbolic constants                                            -*/
/*--------------------------------------------------------------------------*/

#ifdef COMPONENT_PAR
#endif // COMPONENT_PAR

// number of clusters
#define gd_PAR_NUM_CLUSTERS_uix                                                         (111)

// number of major variants
#define gd_PAR_NUM_MAJOR_VARIANTS_uix                                                   (1)

// number of minor var meta data entries
#define gd_PAR_NUM_MINOR_VAR_META_DATA_uix                                              (111)

// number of sub var meta data entries
#define gd_PAR_NUM_SUB_VAR_META_DATA_uix                                                (124)


// major variant index
#define gd_PAR_MJV_DAI_BR245_uix                                                        (0)

// minor variant index
#define gd_PAR_MIV_DEFAULT_uix                                                          (0)
#define gd_PAR_MIV_APCTL_CFG_PSC_BSD_uix                                                (0)
#define gd_PAR_MIV_APCTL_CFG_BSD_uix                                                    (1)
#define gd_PAR_MIV_MP_SensorTimes_PDC_Only_uix                                          (0)
#define gd_PAR_MIV_MP_SensorTimes_PDC_PSX_uix                                           (1)
#define gd_PAR_MIV_MP_SensorTimes_PSX_BSD_uix                                           (2)
#define gd_PAR_MIV_MP_SensorStatic_PAS_Normal_uix                                       (0)
#define gd_PAR_MIV_MP_SensorStatic_PAS_AHK_uix                                          (1)
#define gd_PAR_MIV_MP_SensorStatic_SVA_uix                                              (2)
#define gd_PAR_MIV_MP_PdcMeasGroupParId_F6_R6_uix                                       (0)
#define gd_PAR_MIV_MP_PdcMeasGroupParId_F6_R4_uix                                       (1)
#define gd_PAR_MIV_MP_PdcMeasGroupParId_F4_R4_uix                                       (2)
#define gd_PAR_MIV_MP_PdcMeasGroupParId_F0_R4_uix                                       (3)
#define gd_PAR_MIV_MP_PdcMeasGroupParId_F0_R3_uix                                       (4)
#define gd_PAR_MIV_OD_IdleDist_PDC_Normal_uix                                           (0)
#define gd_PAR_MIV_OD_IdleDist_PDC_AHK_uix                                              (1)
#define gd_PAR_MIV_OD_Sens_PDC_Normal_uix                                               (0)
#define gd_PAR_MIV_OD_Sens_PDC_AHK_uix                                                  (1)
#define gd_PAR_MIV_OD_Veh_PDC_Normal_uix                                                (0)
#define gd_PAR_MIV_OD_Veh_PDC_AHK_uix                                                   (1)
#define gd_PAR_MIV_AP_PDC_Offset_Lat_PDC_Normal_uix                                     (0)
#define gd_PAR_MIV_AP_PDC_Offset_Lat_PDC_AHK_uix                                        (1)

// sub variant index
#define gd_PAR_SUV_DEFAULT_uix                                                          (0)
#define gd_PAR_SUV_MP_USS_FISelection_FI_Std_uix                                        (0)
#define gd_PAR_SUV_MP_USS_FIBlock1_FI_Std_uix                                           (0)
#define gd_PAR_SUV_MP_USS_FIBlock2_FI_Std_uix                                           (0)
#define gd_PAR_SUV_MP_USS_RISelection_RI_Std_uix                                        (0)
#define gd_PAR_SUV_MP_USS_RIBlock1_RI_Std_uix                                           (0)
#define gd_PAR_SUV_MP_USS_RIBlock2_RI_Std_uix                                           (0)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_TEMP0_uix                                      (0)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_TEMP1_uix                                      (1)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_TEMP2_uix                                      (2)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_TEMP3_uix                                      (3)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_TEMP4_uix                                      (4)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_TEMP5_uix                                      (5)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_TEMP6_uix                                      (6)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_TEMP7_uix                                      (7)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_TEMP8_uix                                      (8)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_TEMP9_uix                                      (9)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA1_uix                                       (10)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA2_uix                                       (11)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA3_uix                                       (12)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA4_uix                                       (13)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA5_uix                                       (14)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA6_uix                                       (15)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA7_uix                                       (16)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA8_uix                                       (17)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA9_uix                                       (18)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA10_uix                                      (19)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA11_uix                                      (20)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA12_uix                                      (21)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA13_uix                                      (22)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA14_uix                                      (23)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA15_uix                                      (24)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA16_uix                                      (25)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA17_uix                                      (26)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA18_uix                                      (27)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVA19_uix                                      (28)
#define gd_PAR_SUV_MP_USS_FOSelection_FO_SVABS_uix                                      (29)
#define gd_PAR_SUV_MP_USS_FOBlock1_0_FO_TEMP0_uix                                       (0)
#define gd_PAR_SUV_MP_USS_FOBlock1_1_FO_TEMP1_uix                                       (1)
#define gd_PAR_SUV_MP_USS_FOBlock1_2_FO_TEMP2_uix                                       (2)
#define gd_PAR_SUV_MP_USS_FOBlock1_3_FO_TEMP3_uix                                       (3)
#define gd_PAR_SUV_MP_USS_FOBlock1_4_FO_TEMP4_uix                                       (4)
#define gd_PAR_SUV_MP_USS_FOBlock1_5_FO_TEMP5_uix                                       (5)
#define gd_PAR_SUV_MP_USS_FOBlock1_6_FO_TEMP6_uix                                       (6)
#define gd_PAR_SUV_MP_USS_FOBlock1_7_FO_TEMP7_uix                                       (7)
#define gd_PAR_SUV_MP_USS_FOBlock1_8_FO_TEMP8_uix                                       (8)
#define gd_PAR_SUV_MP_USS_FOBlock1_9_FO_TEMP9_uix                                       (9)
#define gd_PAR_SUV_MP_USS_FOBlock1_10_FO_SVA1_uix                                       (10)
#define gd_PAR_SUV_MP_USS_FOBlock1_11_FO_SVA2_uix                                       (11)
#define gd_PAR_SUV_MP_USS_FOBlock1_12_FO_SVA3_uix                                       (12)
#define gd_PAR_SUV_MP_USS_FOBlock1_13_FO_SVA4_uix                                       (13)
#define gd_PAR_SUV_MP_USS_FOBlock1_14_FO_SVA5_uix                                       (14)
#define gd_PAR_SUV_MP_USS_FOBlock1_15_FO_SVA6_uix                                       (15)
#define gd_PAR_SUV_MP_USS_FOBlock1_16_FO_SVA7_uix                                       (16)
#define gd_PAR_SUV_MP_USS_FOBlock1_17_FO_SVA8_uix                                       (17)
#define gd_PAR_SUV_MP_USS_FOBlock1_18_FO_SVA9_uix                                       (18)
#define gd_PAR_SUV_MP_USS_FOBlock1_19_FO_SVA10_uix                                      (19)
#define gd_PAR_SUV_MP_USS_FOBlock1_20_FO_SVA11_uix                                      (20)
#define gd_PAR_SUV_MP_USS_FOBlock1_21_FO_SVA12_uix                                      (21)
#define gd_PAR_SUV_MP_USS_FOBlock1_22_FO_SVA13_uix                                      (22)
#define gd_PAR_SUV_MP_USS_FOBlock1_23_FO_SVA14_uix                                      (23)
#define gd_PAR_SUV_MP_USS_FOBlock1_24_FO_SVA15_uix                                      (24)
#define gd_PAR_SUV_MP_USS_FOBlock1_25_FO_SVA16_uix                                      (25)
#define gd_PAR_SUV_MP_USS_FOBlock1_26_FO_SVA17_uix                                      (26)
#define gd_PAR_SUV_MP_USS_FOBlock1_27_FO_SVA18_uix                                      (27)
#define gd_PAR_SUV_MP_USS_FOBlock1_28_FO_SVA19_uix                                      (28)
#define gd_PAR_SUV_MP_USS_FOBlock1_29_FO_SVABS_uix                                      (29)
#define gd_PAR_SUV_MP_USS_FOBlock2_0_FO_0_uix                                           (0)
#define gd_PAR_SUV_MP_USS_FOBlock2_1_FO_SVA1_uix                                        (1)
#define gd_PAR_SUV_MP_USS_FOBlock2_2_FO_SVA2_uix                                        (2)
#define gd_PAR_SUV_MP_USS_FOBlock2_3_FO_SVA3_uix                                        (3)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_TEMP0_uix                                      (0)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_TEMP1_uix                                      (1)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_TEMP2_uix                                      (2)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_TEMP3_uix                                      (3)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_TEMP4_uix                                      (4)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_TEMP5_uix                                      (5)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_TEMP6_uix                                      (6)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_TEMP7_uix                                      (7)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_TEMP8_uix                                      (8)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_TEMP9_uix                                      (9)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA1_uix                                       (10)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA2_uix                                       (11)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA3_uix                                       (12)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA4_uix                                       (13)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA5_uix                                       (14)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA6_uix                                       (15)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA7_uix                                       (16)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA8_uix                                       (17)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA9_uix                                       (18)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA10_uix                                      (19)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA11_uix                                      (20)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA12_uix                                      (21)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA13_uix                                      (22)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA14_uix                                      (23)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA15_uix                                      (24)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA16_uix                                      (25)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA17_uix                                      (26)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA18_uix                                      (27)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVA19_uix                                      (28)
#define gd_PAR_SUV_MP_USS_ROSelection_RO_SVABS_uix                                      (29)
#define gd_PAR_SUV_MP_USS_ROBlock1_0_RO_TEMP0_uix                                       (0)
#define gd_PAR_SUV_MP_USS_ROBlock1_1_RO_TEMP1_uix                                       (1)
#define gd_PAR_SUV_MP_USS_ROBlock1_2_RO_TEMP2_uix                                       (2)
#define gd_PAR_SUV_MP_USS_ROBlock1_3_RO_TEMP3_uix                                       (3)
#define gd_PAR_SUV_MP_USS_ROBlock1_4_RO_TEMP4_uix                                       (4)
#define gd_PAR_SUV_MP_USS_ROBlock1_5_RO_TEMP5_uix                                       (5)
#define gd_PAR_SUV_MP_USS_ROBlock1_6_RO_TEMP6_uix                                       (6)
#define gd_PAR_SUV_MP_USS_ROBlock1_7_RO_TEMP7_uix                                       (7)
#define gd_PAR_SUV_MP_USS_ROBlock1_8_RO_TEMP8_uix                                       (8)
#define gd_PAR_SUV_MP_USS_ROBlock1_9_RO_TEMP9_uix                                       (9)
#define gd_PAR_SUV_MP_USS_ROBlock1_10_RO_SVA1_uix                                       (10)
#define gd_PAR_SUV_MP_USS_ROBlock1_11_RO_SVA2_uix                                       (11)
#define gd_PAR_SUV_MP_USS_ROBlock1_12_RO_SVA3_uix                                       (12)
#define gd_PAR_SUV_MP_USS_ROBlock1_13_RO_SVA4_uix                                       (13)
#define gd_PAR_SUV_MP_USS_ROBlock1_14_RO_SVA5_uix                                       (14)
#define gd_PAR_SUV_MP_USS_ROBlock1_15_RO_SVA6_uix                                       (15)
#define gd_PAR_SUV_MP_USS_ROBlock1_16_RO_SVA7_uix                                       (16)
#define gd_PAR_SUV_MP_USS_ROBlock1_17_RO_SVA8_uix                                       (17)
#define gd_PAR_SUV_MP_USS_ROBlock1_18_RO_SVA9_uix                                       (18)
#define gd_PAR_SUV_MP_USS_ROBlock1_19_RO_SVA10_uix                                      (19)
#define gd_PAR_SUV_MP_USS_ROBlock1_20_RO_SVA11_uix                                      (20)
#define gd_PAR_SUV_MP_USS_ROBlock1_21_RO_SVA12_uix                                      (21)
#define gd_PAR_SUV_MP_USS_ROBlock1_22_RO_SVA13_uix                                      (22)
#define gd_PAR_SUV_MP_USS_ROBlock1_23_RO_SVA14_uix                                      (23)
#define gd_PAR_SUV_MP_USS_ROBlock1_24_RO_SVA15_uix                                      (24)
#define gd_PAR_SUV_MP_USS_ROBlock1_25_RO_SVA16_uix                                      (25)
#define gd_PAR_SUV_MP_USS_ROBlock1_26_RO_SVA17_uix                                      (26)
#define gd_PAR_SUV_MP_USS_ROBlock1_27_RO_SVA18_uix                                      (27)
#define gd_PAR_SUV_MP_USS_ROBlock1_28_RO_SVA19_uix                                      (28)
#define gd_PAR_SUV_MP_USS_ROBlock1_29_RO_SVABS_uix                                      (29)
#define gd_PAR_SUV_MP_USS_ROBlock2_0_RO_0_uix                                           (0)
#define gd_PAR_SUV_MP_USS_ROBlock2_1_RO_SVA1_uix                                        (1)
#define gd_PAR_SUV_MP_USS_ROBlock2_2_RO_SVA2_uix                                        (2)
#define gd_PAR_SUV_MP_USS_ROBlock2_3_RO_SVA3_uix                                        (3)
#define gd_PAR_SUV_MP_SensorTimes_F_16_uix                                              (0)
#define gd_PAR_SUV_MP_SensorTimes_F_25_uix                                              (1)
#define gd_PAR_SUV_MP_SensorTimes_F_34_uix                                              (2)
#define gd_PAR_SUV_MP_SensorTimes_R_16_uix                                              (3)
#define gd_PAR_SUV_MP_SensorTimes_R_25_uix                                              (4)
#define gd_PAR_SUV_MP_SensorTimes_R_34_uix                                              (5)
#define gd_PAR_SUV_MP_SensorStatic_F_16_uix                                             (0)
#define gd_PAR_SUV_MP_SensorStatic_F_25_uix                                             (1)
#define gd_PAR_SUV_MP_SensorStatic_F_34_uix                                             (2)
#define gd_PAR_SUV_MP_SensorStatic_R_16_uix                                             (3)
#define gd_PAR_SUV_MP_SensorStatic_R_25_uix                                             (4)
#define gd_PAR_SUV_MP_SensorStatic_R_34_uix                                             (5)
#define gd_PAR_SUV_MP_Timings_PAS_uix                                                   (0)
#define gd_PAR_SUV_MP_Timings_PSX_uix                                                   (1)
#define gd_PAR_SUV_MP_RingTimeRange_TEMP0_uix                                           (0)
#define gd_PAR_SUV_MP_RingTimeRange_TEMP1_uix                                           (1)
#define gd_PAR_SUV_MP_RingTimeRange_TEMP2_uix                                           (2)
#define gd_PAR_SUV_MP_RingTimeRange_TEMP3_uix                                           (3)

// general cluster index
#define gd_PAR_IDX_VHOSRC_PluginSRC_uix                                                 (0)
#define gd_PAR_IDX_VHOTCE_PluginTTC_uix                                                 (1)
#define gd_PAR_IDX_VHOTCE_PluginTCE_uix                                                 (2)
#define gd_PAR_IDX_VHOTCE_PluginTCE_clustering_uix                                      (3)
#define gd_PAR_IDX_AP_PS_Config_uix                                                     (4)
#define gd_PAR_IDX_AP_PS_Speed_uix                                                      (5)
#define gd_PAR_IDX_AP_PS_Brake_uix                                                      (6)
#define gd_PAR_IDX_AP_PS_Objects_uix                                                    (7)
#define gd_PAR_IDX_BDA_Parameter_uix                                                    (8)
#define gd_PAR_IDX_PSD_Veh_uix                                                          (9)
#define gd_PAR_IDX_PSD_Proj_uix                                                         (10)
#define gd_PAR_IDX_PSD_Proj_OCP_uix                                                     (11)
#define gd_PAR_IDX_PSD_Proj_Cross_uix                                                   (12)
#define gd_PAR_IDX_PSD_RuleSet1_uix                                                     (13)
#define gd_PAR_IDX_PSD_Proj_VC_uix                                                      (14)
#define gd_PAR_IDX_PSD_Proj_SPR_uix                                                     (15)
#define gd_PAR_IDX_MP_USS_FISelection_uix                                               (16)
#define gd_PAR_IDX_MP_USS_FIBlock1_uix                                                  (17)
#define gd_PAR_IDX_MP_USS_FIBlock2_uix                                                  (18)
#define gd_PAR_IDX_MP_USS_RISelection_uix                                               (19)
#define gd_PAR_IDX_MP_USS_RIBlock1_uix                                                  (20)
#define gd_PAR_IDX_MP_USS_RIBlock2_uix                                                  (21)
#define gd_PAR_IDX_MP_USS_FOSelection_uix                                               (22)
#define gd_PAR_IDX_MP_USS_FOBlock1_uix                                                  (23)
#define gd_PAR_IDX_MP_USS_FOBlock2_uix                                                  (24)
#define gd_PAR_IDX_MP_USS_ROSelection_uix                                               (25)
#define gd_PAR_IDX_MP_USS_ROBlock1_uix                                                  (26)
#define gd_PAR_IDX_MP_USS_ROBlock2_uix                                                  (27)
#define gd_PAR_IDX_MP_USS_CcSetInfo_uix                                                 (28)
#define gd_PAR_IDX_APCTL_CFG_uix                                                        (29)
#define gd_PAR_IDX_DISPLAY_Veh_uix                                                      (30)
#define gd_PAR_IDX_PSM_DiscardCriteria_uix                                              (31)
#define gd_PAR_IDX_PSM_DiscardCriteriaCPSC_uix                                          (32)
#define gd_PAR_IDX_PSM_ParkableCriteria_uix                                             (33)
#define gd_PAR_IDX_PSM_ParkableCriteriaCPSC_uix                                         (34)
#define gd_PAR_IDX_PSM_FilterCriteria_uix                                               (35)
#define gd_PAR_IDX_PSM_ParametersDPSC_uix                                               (36)
#define gd_PAR_IDX_MP_SensorTimes_uix                                                   (37)
#define gd_PAR_IDX_MP_SensorStatic_uix                                                  (38)
#define gd_PAR_IDX_MP_PdcMeasGroupParId_uix                                             (39)
#define gd_PAR_IDX_MP_Timings_uix                                                       (40)
#define gd_PAR_IDX_MP_SensorType_uix                                                    (41)
#define gd_PAR_IDX_MP_SensorOrientation_uix                                             (42)
#define gd_PAR_IDX_MP_RingTimeInfor_uix                                                 (43)
#define gd_PAR_IDX_MP_RingTimeRange_uix                                                 (44)
#define gd_PAR_IDX_MP_SensorPort_uix                                                    (45)
#define gd_PAR_IDX_OD_General_uix                                                       (46)
#define gd_PAR_IDX_OD_MaskActVelo_uix                                                   (47)
#define gd_PAR_IDX_OD_IdleDist_uix                                                      (48)
#define gd_PAR_IDX_OD_Sens_uix                                                          (49)
#define gd_PAR_IDX_OD_Veh_uix                                                           (50)
#define gd_PAR_IDX_OD_Echo_ApFlt_uix                                                    (51)
#define gd_PAR_IDX_OD_Echo_WRF_uix                                                      (52)
#define gd_PAR_IDX_OD_Echo_SRF_uix                                                      (53)
#define gd_PAR_IDX_OD_Echo_HLF_uix                                                      (54)
#define gd_PAR_IDX_OD_Echo_MoveDir_uix                                                  (55)
#define gd_PAR_IDX_OD_Echo_Clip_Std_uix                                                 (56)
#define gd_PAR_IDX_OD_Obj_Meas_uix                                                      (57)
#define gd_PAR_IDX_OD_Obj_Trck_uix                                                      (58)
#define gd_PAR_IDX_SP_PWR_CFG_uix                                                       (59)
#define gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix                                    (60)
#define gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix                                      (61)
#define gd_PAR_IDX_AP_BSD_Ap_uix                                                        (62)
#define gd_PAR_IDX_EP_BSD_Ep_uix                                                        (63)
#define gd_PAR_IDX_EP_BSD_SDI_uix                                                       (64)
#define gd_PAR_IDX_AP_PDC_General_uix                                                   (65)
#define gd_PAR_IDX_AP_PDC_Disp_Global_uix                                               (66)
#define gd_PAR_IDX_AP_PDC_Disp_Lat_uix                                                  (67)
#define gd_PAR_IDX_AP_PDC_View_Lat_uix                                                  (68)
#define gd_PAR_IDX_AP_PDC_View_Prm_uix                                                  (69)
#define gd_PAR_IDX_AP_PDC_Offset_Std_uix                                                (70)
#define gd_PAR_IDX_AP_PDC_Offset_Lat_uix                                                (71)
#define gd_PAR_IDX_AP_PDC_SC_DT_Std_uix                                                 (72)
#define gd_PAR_IDX_AP_PDC_SC_DT_Lat_uix                                                 (73)
#define gd_PAR_IDX_AP_PDC_SC_DU_Lat_uix                                                 (74)
#define gd_PAR_IDX_AP_PDC_SC_DU_Prm_uix                                                 (75)
#define gd_PAR_IDX_AP_PDC_SC_OE_Std_uix                                                 (76)
#define gd_PAR_IDX_AP_PDC_SC_OE_Lat_uix                                                 (77)
#define gd_PAR_IDX_AP_PDC_SC_SWD_Std_uix                                                (78)
#define gd_PAR_IDX_AP_PDC_SC_SWD_Lat_uix                                                (79)
#define gd_PAR_IDX_AP_PDC_SC_SWD_Prm_uix                                                (80)
#define gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix                                                (81)
#define gd_PAR_IDX_AP_PDC_AO_uix                                                        (82)
#define gd_PAR_IDX_AP_PDC_Danger_uix                                                    (83)
#define gd_PAR_IDX_PSU_CPSC_uix                                                         (84)
#define gd_PAR_IDX_PSU_PSC_uix                                                          (85)
#define gd_PAR_IDX_PSU_POC_uix                                                          (86)
#define gd_PAR_IDX_PSU_CPX_uix                                                          (87)
#define gd_PAR_IDX_PSU_Object_uix                                                       (88)
#define gd_PAR_IDX_PSU_Reference_uix                                                    (89)
#define gd_PAR_IDX_APG_XPG_Vehicle_uix                                                  (90)
#define gd_PAR_IDX_APG_XPG_SafetyDistParallel_uix                                       (91)
#define gd_PAR_IDX_APG_XPG_SafetyDistCross_uix                                          (92)
#define gd_PAR_IDX_APG_XPG_Ctrl_uix                                                     (93)
#define gd_PAR_IDX_APG_XPG_NMove_uix                                                    (94)
#define gd_PAR_IDX_APG_XPG_Parkable_uix                                                 (95)
#define gd_PAR_IDX_APG_APG_Pathplanning_Parallel_uix                                    (96)
#define gd_PAR_IDX_APG_APG_Steer_Ctrl_uix                                               (97)
#define gd_PAR_IDX_APG_APG_Ctrl_uix                                                     (98)
#define gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix                                       (99)
#define gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix                                          (100)
#define gd_PAR_IDX_APG_APG_POC_uix                                                      (101)
#define gd_PAR_IDX_APG_CPSC_uix                                                         (102)
#define gd_PAR_IDX_APG_PL_POC_Handover_uix                                              (103)
#define gd_PAR_IDX_PAGLOBAL_SysFct_uix                                                  (104)
#define gd_PAR_IDX_SIP_VHOParaAutocalib_uix                                             (105)
#define gd_PAR_IDX_VHO_Veh_uix                                                          (106)
#define gd_PAR_IDX_VHO_Odo_uix                                                          (107)
#define gd_PAR_IDX_MP_BDA_Block1_uix                                                    (108)
#define gd_PAR_IDX_MP_BDA_Block2_uix                                                    (109)
#define gd_PAR_IDX_MP_BDA_Block3_uix                                                    (110)

// write access cluster index
#define gd_PAR_IDX_WRITE_SIP_VHOParaAutocalib_uix                                       (105)

// cluster flag buffer size
#define gd_PAR_CLUSTER_FLAG_BUFFER_SIZE_uix                                             ((gd_PAR_NUM_CLUSTERS_uix + 7) / 8)

/*--------------------------------------------------------------------------*/
/*- component versions                                                     -*/
/*--------------------------------------------------------------------------*/

// VHOSRC - version 5.8
#define gd_PAR_COMP_VERSION_MAJOR_VHOSRC_uix (5)
#define gd_PAR_COMP_VERSION_MINOR_VHOSRC_uix (8)
#define gd_PAR_COMP_VERSION_SUB_VHOSRC_uix   (0)
// VHOTCE - version 5.8
#define gd_PAR_COMP_VERSION_MAJOR_VHOTCE_uix (5)
#define gd_PAR_COMP_VERSION_MINOR_VHOTCE_uix (8)
#define gd_PAR_COMP_VERSION_SUB_VHOTCE_uix   (0)
// MP_SNOWMUD - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_MP_SNOWMUD_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_MP_SNOWMUD_uix (0)
#define gd_PAR_COMP_VERSION_SUB_MP_SNOWMUD_uix   (0)
// EOL - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_EOL_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_EOL_uix (0)
#define gd_PAR_COMP_VERSION_SUB_EOL_uix   (0)
// AP_PS - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_AP_PS_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_AP_PS_uix (0)
#define gd_PAR_COMP_VERSION_SUB_AP_PS_uix   (0)
// BDA - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_BDA_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_BDA_uix (0)
#define gd_PAR_COMP_VERSION_SUB_BDA_uix   (0)
// FUS - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_FUS_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_FUS_uix (0)
#define gd_PAR_COMP_VERSION_SUB_FUS_uix   (0)
// OOC - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_OOC_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_OOC_uix (0)
#define gd_PAR_COMP_VERSION_SUB_OOC_uix   (0)
// MP_DIAG_TEST - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_MP_DIAG_TEST_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_MP_DIAG_TEST_uix (0)
#define gd_PAR_COMP_VERSION_SUB_MP_DIAG_TEST_uix   (0)
// PSD - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_PSD_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_PSD_uix (0)
#define gd_PAR_COMP_VERSION_SUB_PSD_uix   (0)
// MP_USS - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_MP_USS_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_MP_USS_uix (0)
#define gd_PAR_COMP_VERSION_SUB_MP_USS_uix   (0)
// MP_INTERFERENCE - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_MP_INTERFERENCE_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_MP_INTERFERENCE_uix (0)
#define gd_PAR_COMP_VERSION_SUB_MP_INTERFERENCE_uix   (0)
// APCTL - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_APCTL_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_APCTL_uix (0)
#define gd_PAR_COMP_VERSION_SUB_APCTL_uix   (0)
// DISPLAY - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_DISPLAY_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_DISPLAY_uix (0)
#define gd_PAR_COMP_VERSION_SUB_DISPLAY_uix   (0)
// ACS - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_ACS_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_ACS_uix (0)
#define gd_PAR_COMP_VERSION_SUB_ACS_uix   (0)
// PSM - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_PSM_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_PSM_uix (0)
#define gd_PAR_COMP_VERSION_SUB_PSM_uix   (0)
// MP - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_MP_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_MP_uix (0)
#define gd_PAR_COMP_VERSION_SUB_MP_uix   (0)
// MP_BUFF - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_MP_BUFF_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_MP_BUFF_uix (0)
#define gd_PAR_COMP_VERSION_SUB_MP_BUFF_uix   (0)
// MP_PL - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_MP_PL_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_MP_PL_uix (0)
#define gd_PAR_COMP_VERSION_SUB_MP_PL_uix   (0)
// MTL - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_MTL_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_MTL_uix (0)
#define gd_PAR_COMP_VERSION_SUB_MTL_uix   (0)
// OD - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_OD_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_OD_uix (0)
#define gd_PAR_COMP_VERSION_SUB_OD_uix   (0)
// PAR - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_PAR_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_PAR_uix (0)
#define gd_PAR_COMP_VERSION_SUB_PAR_uix   (0)
// SP_FSV - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_SP_FSV_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_SP_FSV_uix (0)
#define gd_PAR_COMP_VERSION_SUB_SP_FSV_uix   (0)
// SP_PWR - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_SP_PWR_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_SP_PWR_uix (0)
#define gd_PAR_COMP_VERSION_SUB_SP_PWR_uix   (0)
// SMS - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_SMS_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_SMS_uix (0)
#define gd_PAR_COMP_VERSION_SUB_SMS_uix   (0)
// VHS_PL_TORQUE - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_VHS_PL_TORQUE_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_VHS_PL_TORQUE_uix (0)
#define gd_PAR_COMP_VERSION_SUB_VHS_PL_TORQUE_uix   (0)
// VHS - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_VHS_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_VHS_uix (0)
#define gd_PAR_COMP_VERSION_SUB_VHS_uix   (0)
// AP_BSD - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_AP_BSD_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_AP_BSD_uix (0)
#define gd_PAR_COMP_VERSION_SUB_AP_BSD_uix   (0)
// EP_BSD - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_EP_BSD_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_EP_BSD_uix (0)
#define gd_PAR_COMP_VERSION_SUB_EP_BSD_uix   (0)
// AP_PDC - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_AP_PDC_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_AP_PDC_uix (0)
#define gd_PAR_COMP_VERSION_SUB_AP_PDC_uix   (0)
// PSU - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_PSU_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_PSU_uix (0)
#define gd_PAR_COMP_VERSION_SUB_PSU_uix   (0)
// APG - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_APG_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_APG_uix (0)
#define gd_PAR_COMP_VERSION_SUB_APG_uix   (0)
// APG_PL_POC - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_APG_PL_POC_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_APG_PL_POC_uix (0)
#define gd_PAR_COMP_VERSION_SUB_APG_PL_POC_uix   (0)
// PAGLOBAL - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_PAGLOBAL_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_PAGLOBAL_uix (0)
#define gd_PAR_COMP_VERSION_SUB_PAGLOBAL_uix   (0)
// DSW - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_DSW_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_DSW_uix (0)
#define gd_PAR_COMP_VERSION_SUB_DSW_uix   (0)
// VARIANT - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_VARIANT_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_VARIANT_uix (0)
#define gd_PAR_COMP_VERSION_SUB_VARIANT_uix   (0)
// SIP - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_SIP_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_SIP_uix (0)
#define gd_PAR_COMP_VERSION_SUB_SIP_uix   (0)
// VHO - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_VHO_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_VHO_uix (0)
#define gd_PAR_COMP_VERSION_SUB_VHO_uix   (0)
// MP_USER_MODE - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_MP_USER_MODE_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_MP_USER_MODE_uix (0)
#define gd_PAR_COMP_VERSION_SUB_MP_USER_MODE_uix   (0)
// MP_BDA - version 1.0
#define gd_PAR_COMP_VERSION_MAJOR_MP_BDA_uix (1)
#define gd_PAR_COMP_VERSION_MINOR_MP_BDA_uix (0)
#define gd_PAR_COMP_VERSION_SUB_MP_BDA_uix   (0)

/*--------------------------------------------------------------------------*/
/*- exported macros                                                        -*/
/*--------------------------------------------------------------------------*/

#ifdef COMPONENT_PAR
#endif // COMPONENT_PAR

/*--------------------------------------------------------------------------*/
/*- parameter access macros                                                -*/
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*- VHOSRC                                                                 -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaVHOSRC_PluginSRC_accelMaxWhileBreaking_si16 (((gType_parVHOSRC_PluginSRC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOSRC_PluginSRC_uix]))->accelMaxWhileBreaking_si16)
#define g_parGetParaVHOSRC_PluginSRC_accelMinWhileAcceleration_si16 (((gType_parVHOSRC_PluginSRC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOSRC_PluginSRC_uix]))->accelMinWhileAcceleration_si16)
#define g_parGetParaVHOSRC_PluginSRC_normalWeightFront_ui8 (((gType_parVHOSRC_PluginSRC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOSRC_PluginSRC_uix]))->normalWeightFront_ui8)
#define g_parGetParaVHOSRC_PluginSRC_normalWeightRear_ui8 (((gType_parVHOSRC_PluginSRC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOSRC_PluginSRC_uix]))->normalWeightRear_ui8)
#define g_parGetParaVHOSRC_PluginSRC_accelWeightFront_ui8 (((gType_parVHOSRC_PluginSRC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOSRC_PluginSRC_uix]))->accelWeightFront_ui8)
#define g_parGetParaVHOSRC_PluginSRC_accelWeightRear_ui8 (((gType_parVHOSRC_PluginSRC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOSRC_PluginSRC_uix]))->accelWeightRear_ui8)
#define g_parGetParaVHOSRC_PluginSRC_decelWeightFront_ui8 (((gType_parVHOSRC_PluginSRC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOSRC_PluginSRC_uix]))->decelWeightFront_ui8)
#define g_parGetParaVHOSRC_PluginSRC_decelWeightRear_ui8 (((gType_parVHOSRC_PluginSRC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOSRC_PluginSRC_uix]))->decelWeightRear_ui8)
#define g_parGetParaVHOSRC_PluginSRC_dynStateHoldTime_ui16 (((gType_parVHOSRC_PluginSRC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOSRC_PluginSRC_uix]))->dynStateHoldTime_ui16)
#define g_parGetParaVHOSRC_PluginSRC_distanceMaxABSintervention_ui16 (((gType_parVHOSRC_PluginSRC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOSRC_PluginSRC_uix]))->distanceMaxABSintervention_ui16)
#define g_parGetParaVHOSRC_PluginSRC_distanceMaxASRintervention_ui16 (((gType_parVHOSRC_PluginSRC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOSRC_PluginSRC_uix]))->distanceMaxASRintervention_ui16)
#define g_parGetParaVHOSRC_PluginSRC_distanceMaxESPintervention_ui16 (((gType_parVHOSRC_PluginSRC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOSRC_PluginSRC_uix]))->distanceMaxESPintervention_ui16)

/*--------------------------------------------------------------------------*/
/*- VHOTCE                                                                 -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaVHOTCE_SIZE_LONGACC_HISTORYBUFFER_ui8 gd_VHOTCE_SIZE_LONGACC_HISTORYBUFFER_ui8
#define g_parGetParaVHOTCE_SIZE_DUALTRACKSMOOTHBUFFER_ui8 gd_VHOTCE_SIZE_DUALTRACKSMOOTHBUFFER_ui8
#define g_parGetParaVHOTCE_SIZE_YAWRATEDELAYBUFFER_ui8 gd_VHOTCE_SIZE_YAWRATEDELAYBUFFER_ui8
#define g_parGetParaVHOTCE_SIZE_SYNCHRONFILTER_ui8 gd_VHOTCE_SIZE_SYNCHRONFILTER_ui8
#define g_parGetParaVHOTCE_PluginTTC_VeloMax_ui16 (((gType_parVHOTCE_PluginTTC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTTC_uix]))->VeloMax_ui16)
#define g_parGetParaVHOTCE_PluginTTC_DrivingDist_ui32 (((gType_parVHOTCE_PluginTTC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTTC_uix]))->DrivingDist_ui32)
#define g_parGetParaVHOTCE_PluginTTC_MaxAngle_si32 (((gType_parVHOTCE_PluginTTC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTTC_uix]))->MaxAngle_si32)
#define g_parGetParaVHOTCE_PluginTTC_SWAMax_ui16 (((gType_parVHOTCE_PluginTTC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTTC_uix]))->SWAMax_ui16)
#define g_parGetParaVHOTCE_PluginTTC_NumberOfSamples_ui8 (((gType_parVHOTCE_PluginTTC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTTC_uix]))->NumberOfSamples_ui8)
#define g_parGetParaVHOTCE_PluginTCE_TargetForQualityCummulation_ui16 (((gType_parVHOTCE_PluginTCE_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_uix]))->TargetForQualityCummulation_ui16)
#define g_parGetParaVHOTCE_PluginTCE_VeloMin_ui16 (((gType_parVHOTCE_PluginTCE_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_uix]))->VeloMin_ui16)
#define g_parGetParaVHOTCE_PluginTCE_VeloMax_ui16 (((gType_parVHOTCE_PluginTCE_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_uix]))->VeloMax_ui16)
#define g_parGetParaVHOTCE_PluginTCE_LongACCMax_ui16 (((gType_parVHOTCE_PluginTCE_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_uix]))->LongACCMax_ui16)
#define g_parGetParaVHOTCE_PluginTCE_SWAMin_ui16 (((gType_parVHOTCE_PluginTCE_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_uix]))->SWAMin_ui16)
#define g_parGetParaVHOTCE_PluginTCE_SWAMax_ui16 (((gType_parVHOTCE_PluginTCE_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_uix]))->SWAMax_ui16)
#define g_parGetParaVHOTCE_PluginTCE_YawangleFinish_ui32 (((gType_parVHOTCE_PluginTCE_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_uix]))->YawangleFinish_ui32)
#define g_parGetParaVHOTCE_PluginTCE_YawangleMax_ui32 (((gType_parVHOTCE_PluginTCE_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_uix]))->YawangleMax_ui32)
#define g_parGetParaVHOTCE_PluginTCE_YawangleWICError_ui32 (((gType_parVHOTCE_PluginTCE_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_uix]))->YawangleWICError_ui32)
#define g_parGetParaVHOTCE_PluginTCE_ToleratedVelocityDiff_ui16 (((gType_parVHOTCE_PluginTCE_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_uix]))->ToleratedVelocityDiff_ui16)
#define g_parGetParaVHOTCE_PluginTCE_ToleratedSWADiff_ui16 (((gType_parVHOTCE_PluginTCE_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_uix]))->ToleratedSWADiff_ui16)
#define g_parGetParaVHOTCE_PluginTCE_YawrateMin_ui16 (((gType_parVHOTCE_PluginTCE_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_uix]))->YawrateMin_ui16)
#define g_parGetParaVHOTCE_PluginTCE_YawrateMax_ui16 (((gType_parVHOTCE_PluginTCE_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_uix]))->YawrateMax_ui16)
#define g_parGetParaVHOTCE_PluginTCE_clustering_ProcentualCenteringOfCluster_ui8 (((gType_parVHOTCE_PluginTCE_clustering_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_clustering_uix]))->ProcentualCenteringOfCluster_ui8)
#define g_parGetParaVHOTCE_PluginTCE_clustering_ThresholdForTirechange_ui16 (((gType_parVHOTCE_PluginTCE_clustering_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_clustering_uix]))->ThresholdForTirechange_ui16)
#define g_parGetParaVHOTCE_PluginTCE_clustering_NumberOfClusters_ui8 (((gType_parVHOTCE_PluginTCE_clustering_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_clustering_uix]))->NumberOfClusters_ui8)
#define g_parGetParaVHOTCE_PluginTCE_clustering_WICLengthOfCluster_pui16 (((gType_parVHOTCE_PluginTCE_clustering_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_clustering_uix]))->WICLengthOfCluster_pui16)
#define g_parGetParaVHOTCE_PluginTCE_clustering_WheelBaseOfCluster_pui16 (((gType_parVHOTCE_PluginTCE_clustering_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHOTCE_PluginTCE_clustering_uix]))->WheelBaseOfCluster_pui16)

/*--------------------------------------------------------------------------*/
/*- MP_SNOWMUD                                                             -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaMP_SNOWMUD_MAX_DIST_NEAR_RANGE_ECHO_SIDE_SENSOR_ui16 gd_MP_SNOWMUD_MAX_DIST_NEAR_RANGE_ECHO_SIDE_SENSOR_ui16
#define g_parGetParaMP_SNOWMUD_MAX_DIST_NEAR_RANGE_ECHO_NORMAL_SENSOR_ui16 gd_MP_SNOWMUD_MAX_DIST_NEAR_RANGE_ECHO_NORMAL_SENSOR_ui16

/*--------------------------------------------------------------------------*/
/*- EOL                                                                    -*/
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*- AP_PS                                                                  -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaAP_PS_Config_SpeedLimitBrakingPSX_bl (((gType_parAP_PS_Config_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Config_uix]))->SpeedLimitBrakingPSX_bl)
#define g_parGetParaAP_PS_Config_ParkEmergencyBraking_bl (((gType_parAP_PS_Config_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Config_uix]))->ParkEmergencyBraking_bl)
#define g_parGetParaAP_PS_Config_ManeuverEmergencyBraking_bl (((gType_parAP_PS_Config_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Config_uix]))->ManeuverEmergencyBraking_bl)
#define g_parGetParaAP_PS_Config_ShortenedDrivingTube_bl (((gType_parAP_PS_Config_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Config_uix]))->ShortenedDrivingTube_bl)
#define g_parGetParaAP_PS_Config_CloseObjectDetectedBraking_bl (((gType_parAP_PS_Config_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Config_uix]))->CloseObjectDetectedBraking_bl)
#define g_parGetParaAP_PS_Config_BrakeForHighObjectsOnly_bl (((gType_parAP_PS_Config_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Config_uix]))->BrakeForHighObjectsOnly_bl)
#define g_parGetParaAP_PS_Speed_DeactivationLow_ui8 (((gType_parAP_PS_Speed_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Speed_uix]))->DeactivationLow_ui8)
#define g_parGetParaAP_PS_Speed_DeactivationHigh_ui8 (((gType_parAP_PS_Speed_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Speed_uix]))->DeactivationHigh_ui8)
#define g_parGetParaAP_PS_Speed_FirstClothoidShift_ui8 (((gType_parAP_PS_Speed_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Speed_uix]))->FirstClothoidShift_ui8)
#define g_parGetParaAP_PS_Speed_LimitStraight_ui8 (((gType_parAP_PS_Speed_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Speed_uix]))->LimitStraight_ui8)
#define g_parGetParaAP_PS_Speed_LimitSteering1_ui8 (((gType_parAP_PS_Speed_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Speed_uix]))->LimitSteering1_ui8)
#define g_parGetParaAP_PS_Speed_LimitSteering2_ui8 (((gType_parAP_PS_Speed_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Speed_uix]))->LimitSteering2_ui8)
#define g_parGetParaAP_PS_Speed_LimitNMove_ui8 (((gType_parAP_PS_Speed_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Speed_uix]))->LimitNMove_ui8)
#define g_parGetParaAP_PS_Speed_LimitCPSCBackStraight_ui8 (((gType_parAP_PS_Speed_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Speed_uix]))->LimitCPSCBackStraight_ui8)
#define g_parGetParaAP_PS_Speed_LimitCPSCBack_ui8 (((gType_parAP_PS_Speed_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Speed_uix]))->LimitCPSCBack_ui8)
#define g_parGetParaAP_PS_Speed_LimitCPSCForw_ui8 (((gType_parAP_PS_Speed_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Speed_uix]))->LimitCPSCForw_ui8)
#define g_parGetParaAP_PS_Speed_LimitFwdCPSC_ui8 (((gType_parAP_PS_Speed_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Speed_uix]))->LimitFwdCPSC_ui8)
#define g_parGetParaAP_PS_Speed_LimitHysteresis_ui8 (((gType_parAP_PS_Speed_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Speed_uix]))->LimitHysteresis_ui8)
#define g_parGetParaAP_PS_Speed_MbaC_Speedlimit1_ui8 (((gType_parAP_PS_Speed_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Speed_uix]))->MbaC_Speedlimit1_ui8)
#define g_parGetParaAP_PS_Speed_MbaC_Speedlimit2_ui8 (((gType_parAP_PS_Speed_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Speed_uix]))->MbaC_Speedlimit2_ui8)
#define g_parGetParaAP_PS_Speed_MbaC_Speedlimit3_ui8 (((gType_parAP_PS_Speed_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Speed_uix]))->MbaC_Speedlimit3_ui8)
#define g_parGetParaAP_PS_Speed_MbaC_Speedlimit4_ui8 (((gType_parAP_PS_Speed_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Speed_uix]))->MbaC_Speedlimit4_ui8)
#define g_parGetParaAP_PS_Speed_MbaC_ActiveNoObject_ui8 (((gType_parAP_PS_Speed_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Speed_uix]))->MbaC_ActiveNoObject_ui8)
#define g_parGetParaAP_PS_Brake_HoldTime_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->HoldTime_ui8)
#define g_parGetParaAP_PS_Brake_TorqueSpeedLimiting_ui16 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->TorqueSpeedLimiting_ui16)
#define g_parGetParaAP_PS_Brake_TorqueCollision_ui16 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->TorqueCollision_ui16)
#define g_parGetParaAP_PS_Brake_AccelerationSpeedLimiting_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->AccelerationSpeedLimiting_ui8)
#define g_parGetParaAP_PS_Brake_AccelerationCollision_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->AccelerationCollision_ui8)
#define g_parGetParaAP_PS_Brake_DeadTime_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->DeadTime_ui8)
#define g_parGetParaAP_PS_Brake_MaxJerk_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->MaxJerk_ui8)
#define g_parGetParaAP_PS_Brake_Mult_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->Mult_ui8)
#define g_parGetParaAP_PS_Brake_Add_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->Add_ui8)
#define g_parGetParaAP_PS_Brake_TimeToCollision1_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->TimeToCollision1_ui8)
#define g_parGetParaAP_PS_Brake_TimeToCollision2_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->TimeToCollision2_ui8)
#define g_parGetParaAP_PS_Brake_ComfortableDecelMaximum_ui16 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->ComfortableDecelMaximum_ui16)
#define g_parGetParaAP_PS_Brake_CloseObjectDecelMaximum_ui16 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->CloseObjectDecelMaximum_ui16)
#define g_parGetParaAP_PS_Brake_MaximumEngineJerk_si16 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->MaximumEngineJerk_si16)
#define g_parGetParaAP_PS_Brake_MaximumDecelerationJerk_si16 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->MaximumDecelerationJerk_si16)
#define g_parGetParaAP_PS_Brake_MaxEngJerkCloseObject_si16 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->MaxEngJerkCloseObject_si16)
#define g_parGetParaAP_PS_Brake_MaxDecelJerkCloseObject_si16 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->MaxDecelJerkCloseObject_si16)
#define g_parGetParaAP_PS_Brake_ESP_MinControlDist_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->ESP_MinControlDist_ui8)
#define g_parGetParaAP_PS_Brake_OocSafeDistPCB_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->OocSafeDistPCB_ui8)
#define g_parGetParaAP_PS_Brake_CreepModusMinDist_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->CreepModusMinDist_ui8)
#define g_parGetParaAP_PS_Brake_SafetyDistVehicle_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->SafetyDistVehicle_ui8)
#define g_parGetParaAP_PS_Brake_NextMoveRequestDistancePar_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->NextMoveRequestDistancePar_ui8)
#define g_parGetParaAP_PS_Brake_NextMoveRequestDistancePerp_ui16 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->NextMoveRequestDistancePerp_ui16)
#define g_parGetParaAP_PS_Brake_ComfStopBrakeFactor_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->ComfStopBrakeFactor_ui8)
#define g_parGetParaAP_PS_Brake_StopDist_1_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDist_1_ui8)
#define g_parGetParaAP_PS_Brake_StopDist_2_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDist_2_ui8)
#define g_parGetParaAP_PS_Brake_StopDist_3_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDist_3_ui8)
#define g_parGetParaAP_PS_Brake_StopDist_4_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDist_4_ui8)
#define g_parGetParaAP_PS_Brake_StopDist_5_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDist_5_ui8)
#define g_parGetParaAP_PS_Brake_StopDist_6_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDist_6_ui8)
#define g_parGetParaAP_PS_Brake_StopDist_7_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDist_7_ui8)
#define g_parGetParaAP_PS_Brake_StopDist_8_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDist_8_ui8)
#define g_parGetParaAP_PS_Brake_StopDist_9_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDist_9_ui8)
#define g_parGetParaAP_PS_Brake_StopDist_10_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDist_10_ui8)
#define g_parGetParaAP_PS_Brake_StopDist_11_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDist_11_ui8)
#define g_parGetParaAP_PS_Brake_StopDistMEB_1_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDistMEB_1_ui8)
#define g_parGetParaAP_PS_Brake_StopDistMEB_2_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDistMEB_2_ui8)
#define g_parGetParaAP_PS_Brake_StopDistMEB_3_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDistMEB_3_ui8)
#define g_parGetParaAP_PS_Brake_StopDistMEB_4_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDistMEB_4_ui8)
#define g_parGetParaAP_PS_Brake_StopDistMEB_5_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDistMEB_5_ui8)
#define g_parGetParaAP_PS_Brake_StopDistMEB_6_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDistMEB_6_ui8)
#define g_parGetParaAP_PS_Brake_StopDistMEB_7_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDistMEB_7_ui8)
#define g_parGetParaAP_PS_Brake_StopDistMEB_8_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDistMEB_8_ui8)
#define g_parGetParaAP_PS_Brake_StopDistMEB_9_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDistMEB_9_ui8)
#define g_parGetParaAP_PS_Brake_StopDistMEB_10_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDistMEB_10_ui8)
#define g_parGetParaAP_PS_Brake_StopDistMEB_11_ui8 (((gType_parAP_PS_Brake_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Brake_uix]))->StopDistMEB_11_ui8)
#define g_parGetParaAP_PS_Objects_MinExistProbability_ui8 (((gType_parAP_PS_Objects_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Objects_uix]))->MinExistProbability_ui8)
#define g_parGetParaAP_PS_Objects_MinExistProbabilityDECEonly_ui8 (((gType_parAP_PS_Objects_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Objects_uix]))->MinExistProbabilityDECEonly_ui8)
#define g_parGetParaAP_PS_Objects_MinimumDistance_ui8 (((gType_parAP_PS_Objects_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Objects_uix]))->MinimumDistance_ui8)
#define g_parGetParaAP_PS_Objects_MbaC_ObjDist1_ui16 (((gType_parAP_PS_Objects_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Objects_uix]))->MbaC_ObjDist1_ui16)
#define g_parGetParaAP_PS_Objects_MbaC_ObjDist2_ui16 (((gType_parAP_PS_Objects_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Objects_uix]))->MbaC_ObjDist2_ui16)
#define g_parGetParaAP_PS_Objects_MbaC_ObjDist3_ui16 (((gType_parAP_PS_Objects_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Objects_uix]))->MbaC_ObjDist3_ui16)
#define g_parGetParaAP_PS_Objects_MbaC_ObjDist4_ui16 (((gType_parAP_PS_Objects_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Objects_uix]))->MbaC_ObjDist4_ui16)
#define g_parGetParaAP_PS_Objects_MAA_Active_ObjDist_ui16 (((gType_parAP_PS_Objects_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PS_Objects_uix]))->MAA_Active_ObjDist_ui16)

/*--------------------------------------------------------------------------*/
/*- BDA                                                                    -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaBDA_EchoHistoMinNumOfShots_ui8 gd_BDA_EchoHistoMinNumOfShots_ui8
#define g_parGetParaBDA_EchoHistoThreshold_ui8 gd_BDA_EchoHistoThreshold_ui8
#define g_parGetParaBDA_EchoHistoMaxCount_ui8 gd_BDA_EchoHistoMaxCount_ui8
#define g_parGetParaBDA_Parameter_SetAllSensorsToNotBlind_bl (((gType_parBDA_Parameter_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_BDA_Parameter_uix]))->SetAllSensorsToNotBlind_bl)
#define g_parGetParaBDA_Parameter_RelevantSensorsBitMask_ui16 (((gType_parBDA_Parameter_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_BDA_Parameter_uix]))->RelevantSensorsBitMask_ui16)
#define g_parGetParaBDA_Parameter_TriggerBlindnessTestAtInit_bl (((gType_parBDA_Parameter_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_BDA_Parameter_uix]))->TriggerBlindnessTestAtInit_bl)
#define g_parGetParaBDA_Parameter_Risk_TimeOfNoObject_ui32 (((gType_parBDA_Parameter_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_BDA_Parameter_uix]))->Risk_TimeOfNoObject_ui32)
#define g_parGetParaBDA_Parameter_Risk_TimeOfBlindSensor_ui32 (((gType_parBDA_Parameter_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_BDA_Parameter_uix]))->Risk_TimeOfBlindSensor_ui32)
#define g_parGetParaBDA_Parameter_vDependingSetBlindMask1_ui16 (((gType_parBDA_Parameter_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_BDA_Parameter_uix]))->vDependingSetBlindMask1_ui16)
#define g_parGetParaBDA_Parameter_vDependingThreshold1_ui8 (((gType_parBDA_Parameter_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_BDA_Parameter_uix]))->vDependingThreshold1_ui8)
#define g_parGetParaBDA_Parameter_vDependingSetBlindMask2_ui16 (((gType_parBDA_Parameter_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_BDA_Parameter_uix]))->vDependingSetBlindMask2_ui16)
#define g_parGetParaBDA_Parameter_vDependingThreshold2_ui8 (((gType_parBDA_Parameter_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_BDA_Parameter_uix]))->vDependingThreshold2_ui8)
#define g_parGetParaBDA_Parameter_MinEchoDistance_ui16 (((gType_parBDA_Parameter_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_BDA_Parameter_uix]))->MinEchoDistance_ui16)
#define g_parGetParaBDA_Parameter_MaxEchoDistance_ui16 (((gType_parBDA_Parameter_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_BDA_Parameter_uix]))->MaxEchoDistance_ui16)
#define g_parGetParaBDA_Parameter_MinGroundEchoDistance_ui16 (((gType_parBDA_Parameter_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_BDA_Parameter_uix]))->MinGroundEchoDistance_ui16)
#define g_parGetParaBDA_Parameter_MaxGroundEchoDistance_ui16 (((gType_parBDA_Parameter_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_BDA_Parameter_uix]))->MaxGroundEchoDistance_ui16)
#define g_parGetParaBDA_Parameter_MaxEchoWidth_ui16 (((gType_parBDA_Parameter_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_BDA_Parameter_uix]))->MaxEchoWidth_ui16)
#define g_parGetParaBDA_Parameter_DurationOfBlindnessTest_ui16 (((gType_parBDA_Parameter_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_BDA_Parameter_uix]))->DurationOfBlindnessTest_ui16)
#define g_parGetParaBDA_Parameter_MaxTempForBlindnessTest_si8 (((gType_parBDA_Parameter_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_BDA_Parameter_uix]))->MaxTempForBlindnessTest_si8)
#define g_parGetParaBDA_Parameter_UnknownStatusMaxWaitTime_ui32 (((gType_parBDA_Parameter_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_BDA_Parameter_uix]))->UnknownStatusMaxWaitTime_ui32)

/*--------------------------------------------------------------------------*/
/*- FUS                                                                    -*/
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*- OOC                                                                    -*/
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*- MP_DIAG_TEST                                                           -*/
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*- PSD                                                                    -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaPSD_NO_ECHOES_ui8 gd_PSD_NO_ECHOES_ui8
#define g_parGetParaPSD_Veh_FrontSensorXDist_ui16 (((gType_parPSD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Veh_uix]))->FrontSensorXDist_ui16)
#define g_parGetParaPSD_Veh_FrontSensorYDist_ui16 (((gType_parPSD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Veh_uix]))->FrontSensorYDist_ui16)
#define g_parGetParaPSD_Veh_FrontSensorAngle_ui8 (((gType_parPSD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Veh_uix]))->FrontSensorAngle_ui8)
#define g_parGetParaPSD_Veh_RearSensorXDist_si16 (((gType_parPSD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Veh_uix]))->RearSensorXDist_si16)
#define g_parGetParaPSD_Veh_RearSensorYDist_ui16 (((gType_parPSD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Veh_uix]))->RearSensorYDist_ui16)
#define g_parGetParaPSD_Veh_RearSensorAngle_ui8 (((gType_parPSD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Veh_uix]))->RearSensorAngle_ui8)
#define g_parGetParaPSD_Veh_AdditionalPSLengthFOV_si8 (((gType_parPSD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Veh_uix]))->AdditionalPSLengthFOV_si8)
#define g_parGetParaPSD_Veh_DisplacementOfPS_si8 (((gType_parPSD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Veh_uix]))->DisplacementOfPS_si8)
#define g_parGetParaPSD_Veh_PSLengthMin_ui16 (((gType_parPSD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Veh_uix]))->PSLengthMin_ui16)
#define g_parGetParaPSD_Veh_PSLengthMax_ui16 (((gType_parPSD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Veh_uix]))->PSLengthMax_ui16)
#define g_parGetParaPSD_Veh_PSDepthDefault_ui16 (((gType_parPSD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Veh_uix]))->PSDepthDefault_ui16)
#define g_parGetParaPSD_Proj_AOIMinDist_ui16 (((gType_parPSD_Proj_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_uix]))->AOIMinDist_ui16)
#define g_parGetParaPSD_Proj_AOIMaxDist_ui16 (((gType_parPSD_Proj_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_uix]))->AOIMaxDist_ui16)
#define g_parGetParaPSD_Proj_LatDist_Min_ui16 (((gType_parPSD_Proj_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_uix]))->LatDist_Min_ui16)
#define g_parGetParaPSD_Proj_PSDistMax_ui16 (((gType_parPSD_Proj_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_uix]))->PSDistMax_ui16)
#define g_parGetParaPSD_Proj_MergeObst_DistMax_ui16 (((gType_parPSD_Proj_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_uix]))->MergeObst_DistMax_ui16)
#define g_parGetParaPSD_Proj_MergeObst_SafetyDist_si16 (((gType_parPSD_Proj_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_uix]))->MergeObst_SafetyDist_si16)
#define g_parGetParaPSD_Proj_OCP_MinCurbDepth_ui16 (((gType_parPSD_Proj_OCP_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_OCP_uix]))->MinCurbDepth_ui16)
#define g_parGetParaPSD_Proj_OCP_MaxCurbDepth_ui16 (((gType_parPSD_Proj_OCP_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_OCP_uix]))->MaxCurbDepth_ui16)
#define g_parGetParaPSD_Proj_OCP_MinCurbLength_ui16 (((gType_parPSD_Proj_OCP_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_OCP_uix]))->MinCurbLength_ui16)
#define g_parGetParaPSD_Proj_OCP_OCP2SafetyDistToCurb_ui16 (((gType_parPSD_Proj_OCP_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_OCP_uix]))->OCP2SafetyDistToCurb_ui16)
#define g_parGetParaPSD_Proj_OCP_OCP4SafetyDistToCurb_ui16 (((gType_parPSD_Proj_OCP_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_OCP_uix]))->OCP4SafetyDistToCurb_ui16)
#define g_parGetParaPSD_Proj_Cross_PSLengthMin_ui16 (((gType_parPSD_Proj_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_Cross_uix]))->PSLengthMin_ui16)
#define g_parGetParaPSD_Proj_Cross_PSLengthMax_ui16 (((gType_parPSD_Proj_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_Cross_uix]))->PSLengthMax_ui16)
#define g_parGetParaPSD_Proj_Cross_LatDiffMax_ui16 (((gType_parPSD_Proj_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_Cross_uix]))->LatDiffMax_ui16)
#define g_parGetParaPSD_Proj_Cross_PassingDistMax_ui16 (((gType_parPSD_Proj_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_Cross_uix]))->PassingDistMax_ui16)
#define g_parGetParaPSD_Proj_Cross_PSLengthVCType1_ui16 (((gType_parPSD_Proj_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_Cross_uix]))->PSLengthVCType1_ui16)
#define g_parGetParaPSD_Proj_Cross_PSLengthVCType2_ui16 (((gType_parPSD_Proj_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_Cross_uix]))->PSLengthVCType2_ui16)
#define g_parGetParaPSD_Proj_Cross_PSLengthVCType1Shortened_ui16 (((gType_parPSD_Proj_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_Cross_uix]))->PSLengthVCType1Shortened_ui16)
#define g_parGetParaPSD_RuleSet1_Normal_OffsetToPSDat_Max_si16 (((gType_parPSD_RuleSet1_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_RuleSet1_uix]))->Normal_OffsetToPSDat_Max_si16)
#define g_parGetParaPSD_RuleSet1_OffsetOnStreetLat_Max_si16 (((gType_parPSD_RuleSet1_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_RuleSet1_uix]))->OffsetOnStreetLat_Max_si16)
#define g_parGetParaPSD_RuleSet1_Normal_SafeDistToObst_Lat_si16 (((gType_parPSD_RuleSet1_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_RuleSet1_uix]))->Normal_SafeDistToObst_Lat_si16)
#define g_parGetParaPSD_RuleSet1_Normal_SafeDistToCurb_si16 (((gType_parPSD_RuleSet1_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_RuleSet1_uix]))->Normal_SafeDistToCurb_si16)
#define g_parGetParaPSD_RuleSet1_OCP2_Safe_AOI_Obj2_Max_si16 (((gType_parPSD_RuleSet1_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_RuleSet1_uix]))->OCP2_Safe_AOI_Obj2_Max_si16)
#define g_parGetParaPSD_RuleSet1_OCP2_Safe_AOI_Obj2_Min_si16 (((gType_parPSD_RuleSet1_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_RuleSet1_uix]))->OCP2_Safe_AOI_Obj2_Min_si16)
#define g_parGetParaPSD_RuleSet1_OCP2_SafeDistToCurb_Lat_si16 (((gType_parPSD_RuleSet1_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_RuleSet1_uix]))->OCP2_SafeDistToCurb_Lat_si16)
#define g_parGetParaPSD_RuleSet1_OCP4_SafeDistToCurb_Lat_si16 (((gType_parPSD_RuleSet1_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_RuleSet1_uix]))->OCP4_SafeDistToCurb_Lat_si16)
#define g_parGetParaPSD_RuleSet1_Normal_SafeDistToObst_Long_si16 (((gType_parPSD_RuleSet1_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_RuleSet1_uix]))->Normal_SafeDistToObst_Long_si16)
#define g_parGetParaPSD_RuleSet1_MinLengthForPlausObject_ui16 (((gType_parPSD_RuleSet1_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_RuleSet1_uix]))->MinLengthForPlausObject_ui16)
#define g_parGetParaPSD_Proj_VC_MinCurbLength_ui16 (((gType_parPSD_Proj_VC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_VC_uix]))->MinCurbLength_ui16)
#define g_parGetParaPSD_Proj_VC_MinPSLength_ui16 (((gType_parPSD_Proj_VC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_VC_uix]))->MinPSLength_ui16)
#define g_parGetParaPSD_Proj_VC_MaxPSDength_ui16 (((gType_parPSD_Proj_VC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_VC_uix]))->MaxPSDength_ui16)
#define g_parGetParaPSD_Proj_VC_MinPSDepth_ui16 (((gType_parPSD_Proj_VC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_VC_uix]))->MinPSDepth_ui16)
#define g_parGetParaPSD_Proj_VC_MaxPSDepth_ui16 (((gType_parPSD_Proj_VC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_VC_uix]))->MaxPSDepth_ui16)
#define g_parGetParaPSD_Proj_VC_MinPSLengthShortened_ui16 (((gType_parPSD_Proj_VC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_VC_uix]))->MinPSLengthShortened_ui16)
#define g_parGetParaPSD_Proj_SPR_OperatingUSDist_ui16 (((gType_parPSD_Proj_SPR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_SPR_uix]))->OperatingUSDist_ui16)
#define g_parGetParaPSD_Proj_SPR_MinWayForUpdate_ui16 (((gType_parPSD_Proj_SPR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSD_Proj_SPR_uix]))->MinWayForUpdate_ui16)

/*--------------------------------------------------------------------------*/
/*- MP_USS                                                                 -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaMP_USS_INNER_SENSORS_FRONT_ui16 gd_MP_USS_INNER_SENSORS_FRONT_ui16
#define g_parGetParaMP_USS_OUTER_SENSORS_FRONT_ui16 gd_MP_USS_OUTER_SENSORS_FRONT_ui16
#define g_parGetParaMP_USS_INNER_SENSORS_REAR_ui16 gd_MP_USS_INNER_SENSORS_REAR_ui16
#define g_parGetParaMP_USS_OUTER_SENSORS_REAR_ui16 gd_MP_USS_OUTER_SENSORS_REAR_ui16
#define g_parGetParaMP_USS_SVA_SENSORS_FRONT_ui16 gd_MP_USS_SVA_SENSORS_FRONT_ui16
#define g_parGetParaMP_USS_SVA_SENSORS_REAR_ui16 gd_MP_USS_SVA_SENSORS_REAR_ui16
#define g_parGetParaMP_USS_FISelection_CcSetBl1Id_ui8 (((gType_parMP_USS_FISelection_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_FISelection_uix]))->CcSetBl1Id_ui8)
#define g_parGetParaMP_USS_FISelection_CcSetBl2Id_ui8 (((gType_parMP_USS_FISelection_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_FISelection_uix]))->CcSetBl2Id_ui8)
#define g_parGetParaMP_USS_FIBlock1_DataBlock1_pui8 (((gType_parMP_USS_FIBlock1_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_FIBlock1_uix]))->DataBlock1_pui8)
#define g_parGetParaMP_USS_FIBlock2_DataBlock2_pui8 (((gType_parMP_USS_FIBlock2_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_FIBlock2_uix]))->DataBlock2_pui8)
#define g_parGetParaMP_USS_RISelection_CcSetBl1Id_ui8 (((gType_parMP_USS_RISelection_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_RISelection_uix]))->CcSetBl1Id_ui8)
#define g_parGetParaMP_USS_RISelection_CcSetBl2Id_ui8 (((gType_parMP_USS_RISelection_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_RISelection_uix]))->CcSetBl2Id_ui8)
#define g_parGetParaMP_USS_RIBlock1_DataBlock1_pui8 (((gType_parMP_USS_RIBlock1_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_RIBlock1_uix]))->DataBlock1_pui8)
#define g_parGetParaMP_USS_RIBlock2_DataBlock2_pui8 (((gType_parMP_USS_RIBlock2_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_RIBlock2_uix]))->DataBlock2_pui8)
#define g_parGetParaMP_USS_FOSelection_CcSetBl1Id_ui8 (((gType_parMP_USS_FOSelection_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_FOSelection_uix]))->CcSetBl1Id_ui8)
#define g_parGetParaMP_USS_FOSelection_CcSetBl2Id_ui8 (((gType_parMP_USS_FOSelection_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_FOSelection_uix]))->CcSetBl2Id_ui8)
#define g_parGetParaMP_USS_FOBlock1_DataBlock1_pui8 (((gType_parMP_USS_FOBlock1_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_FOBlock1_uix]))->DataBlock1_pui8)
#define g_parGetParaMP_USS_FOBlock2_DataBlock2_pui8 (((gType_parMP_USS_FOBlock2_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_FOBlock2_uix]))->DataBlock2_pui8)
#define g_parGetParaMP_USS_ROSelection_CcSetBl1Id_ui8 (((gType_parMP_USS_ROSelection_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_ROSelection_uix]))->CcSetBl1Id_ui8)
#define g_parGetParaMP_USS_ROSelection_CcSetBl2Id_ui8 (((gType_parMP_USS_ROSelection_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_ROSelection_uix]))->CcSetBl2Id_ui8)
#define g_parGetParaMP_USS_ROBlock1_DataBlock1_pui8 (((gType_parMP_USS_ROBlock1_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_ROBlock1_uix]))->DataBlock1_pui8)
#define g_parGetParaMP_USS_ROBlock2_DataBlock2_pui8 (((gType_parMP_USS_ROBlock2_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_ROBlock2_uix]))->DataBlock2_pui8)
#define g_parGetParaMP_USS_CcSetInfo_NumCcSets_ui8 (((gType_parMP_USS_CcSetInfo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_CcSetInfo_uix]))->NumCcSets_ui8)
#define g_parGetParaMP_USS_CcSetInfo_DefaultIdx_ui8 (((gType_parMP_USS_CcSetInfo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_CcSetInfo_uix]))->DefaultIdx_ui8)
#define g_parGetParaMP_USS_CcSetInfo_TempRange_psi8 (((gType_parMP_USS_CcSetInfo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_CcSetInfo_uix]))->TempRange_psi8)
#define g_parGetParaMP_USS_CcSetInfo_TempHyst_si8 (((gType_parMP_USS_CcSetInfo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_USS_CcSetInfo_uix]))->TempHyst_si8)

/*--------------------------------------------------------------------------*/
/*- MP_INTERFERENCE                                                        -*/
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*- APCTL                                                                  -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaAPCTL_CFG_VCutOffPAS_accelerate_ui8 (((gType_parAPCTL_CFG_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APCTL_CFG_uix]))->VCutOffPAS_accelerate_ui8)
#define g_parGetParaAPCTL_CFG_VCutOffPAS_hysteresis_ui8 (((gType_parAPCTL_CFG_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APCTL_CFG_uix]))->VCutOffPAS_hysteresis_ui8)
#define g_parGetParaAPCTL_CFG_VCutOffPSX_accelerate_ui8 (((gType_parAPCTL_CFG_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APCTL_CFG_uix]))->VCutOffPSX_accelerate_ui8)
#define g_parGetParaAPCTL_CFG_VCutOffPSX_hysteresis_ui8 (((gType_parAPCTL_CFG_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APCTL_CFG_uix]))->VCutOffPSX_hysteresis_ui8)
#define g_parGetParaAPCTL_CFG_BSDCutoffSpeed_ui8 (((gType_parAPCTL_CFG_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APCTL_CFG_uix]))->BSDCutoffSpeed_ui8)
#define g_parGetParaAPCTL_CFG_BSDCutoffSpeedHys_ui8 (((gType_parAPCTL_CFG_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APCTL_CFG_uix]))->BSDCutoffSpeedHys_ui8)
#define g_parGetParaAPCTL_CFG_BSDMinVelocityAc_ui8 (((gType_parAPCTL_CFG_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APCTL_CFG_uix]))->BSDMinVelocityAc_ui8)
#define g_parGetParaAPCTL_CFG_BSDMinVelocityDe_ui8 (((gType_parAPCTL_CFG_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APCTL_CFG_uix]))->BSDMinVelocityDe_ui8)
#define g_parGetParaAPCTL_CFG_GuidanceVmaxExceeded_ui8 (((gType_parAPCTL_CFG_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APCTL_CFG_uix]))->GuidanceVmaxExceeded_ui8)
#define g_parGetParaAPCTL_CFG_VStandStillCutOffPSX_ui8 (((gType_parAPCTL_CFG_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APCTL_CFG_uix]))->VStandStillCutOffPSX_ui8)
#define g_parGetParaAPCTL_CFG_SWAMinOMA_si16 (((gType_parAPCTL_CFG_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APCTL_CFG_uix]))->SWAMinOMA_si16)
#define g_parGetParaAPCTL_CFG_SWAMaxOMA_si16 (((gType_parAPCTL_CFG_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APCTL_CFG_uix]))->SWAMaxOMA_si16)

/*--------------------------------------------------------------------------*/
/*- DISPLAY                                                                -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaDISPLAY_Veh_DurationTone_pui8 (((gType_parDISPLAY_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_DISPLAY_Veh_uix]))->DurationTone_pui8)
#define g_parGetParaDISPLAY_Veh_DurationNoTone_pui8 (((gType_parDISPLAY_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_DISPLAY_Veh_uix]))->DurationNoTone_pui8)
#define g_parGetParaDISPLAY_Veh_ToneSetup_pui8 (((gType_parDISPLAY_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_DISPLAY_Veh_uix]))->ToneSetup_pui8)
#define g_parGetParaDISPLAY_Veh_RangeZoneFront_pui8 (((gType_parDISPLAY_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_DISPLAY_Veh_uix]))->RangeZoneFront_pui8)
#define g_parGetParaDISPLAY_Veh_RangeZoneRear_pui8 (((gType_parDISPLAY_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_DISPLAY_Veh_uix]))->RangeZoneRear_pui8)
#define g_parGetParaDISPLAY_Veh_DisplayConfigFront_pui8 (((gType_parDISPLAY_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_DISPLAY_Veh_uix]))->DisplayConfigFront_pui8)
#define g_parGetParaDISPLAY_Veh_DisplayConfigRear_pui8 (((gType_parDISPLAY_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_DISPLAY_Veh_uix]))->DisplayConfigRear_pui8)

/*--------------------------------------------------------------------------*/
/*- ACS                                                                    -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaACS_CLOCK_PRESCALER_ui8 gd_ACS_CLOCK_PRESCALER_ui8
#define g_parGetParaACS_CHARGE_PUMP_CLOCK_ui8 gd_ACS_CHARGE_PUMP_CLOCK_ui8
#define g_parGetParaACS_RESET_THRESHOLD_ui8 gd_ACS_RESET_THRESHOLD_ui8
#define g_parGetParaACS_SPI_LEVEL_ui8 gd_ACS_SPI_LEVEL_ui8

/*--------------------------------------------------------------------------*/
/*- PSM                                                                    -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaPSM_SPRMaxNumOfPoints_ui8 gd_PSM_SPRMaxNumOfPoints_ui8
#define g_parGetParaPSM_NumOfParkingSpaces_ui8 gd_PSM_NumOfParkingSpaces_ui8
#define g_parGetParaPSM_MaxNumOfObjects_ui8 gd_PSM_MaxNumOfObjects_ui8
#define g_parGetParaPSM_DiscardCriteria_MaxSPos_si16 (((gType_parPSM_DiscardCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteria_uix]))->MaxSPos_si16)
#define g_parGetParaPSM_DiscardCriteria_MinSPos_si16 (((gType_parPSM_DiscardCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteria_uix]))->MinSPos_si16)
#define g_parGetParaPSM_DiscardCriteria_MaxSPosRearRef_si16 (((gType_parPSM_DiscardCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteria_uix]))->MaxSPosRearRef_si16)
#define g_parGetParaPSM_DiscardCriteria_MinSPosRearRef_si16 (((gType_parPSM_DiscardCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteria_uix]))->MinSPosRearRef_si16)
#define g_parGetParaPSM_DiscardCriteria_MaxAbsoluteDist_ui16 (((gType_parPSM_DiscardCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteria_uix]))->MaxAbsoluteDist_ui16)
#define g_parGetParaPSM_DiscardCriteria_MaxYPos_si16 (((gType_parPSM_DiscardCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteria_uix]))->MaxYPos_si16)
#define g_parGetParaPSM_DiscardCriteria_MinYPos_si16 (((gType_parPSM_DiscardCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteria_uix]))->MinYPos_si16)
#define g_parGetParaPSM_DiscardCriteria_MaxPhi_ui16 (((gType_parPSM_DiscardCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteria_uix]))->MaxPhi_ui16)
#define g_parGetParaPSM_DiscardCriteria_MinPhi_ui16 (((gType_parPSM_DiscardCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteria_uix]))->MinPhi_ui16)
#define g_parGetParaPSM_DiscardCriteria_MaxPhiB4ThrshldX_ui16 (((gType_parPSM_DiscardCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteria_uix]))->MaxPhiB4ThrshldX_ui16)
#define g_parGetParaPSM_DiscardCriteria_MinPhiB4ThrshldX_ui16 (((gType_parPSM_DiscardCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteria_uix]))->MinPhiB4ThrshldX_ui16)
#define g_parGetParaPSM_DiscardCriteria_ThresholdPhiX_si16 (((gType_parPSM_DiscardCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteria_uix]))->ThresholdPhiX_si16)
#define g_parGetParaPSM_DiscardCriteriaCPSC_MaxSPos_si16 (((gType_parPSM_DiscardCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteriaCPSC_uix]))->MaxSPos_si16)
#define g_parGetParaPSM_DiscardCriteriaCPSC_MinSPos_si16 (((gType_parPSM_DiscardCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteriaCPSC_uix]))->MinSPos_si16)
#define g_parGetParaPSM_DiscardCriteriaCPSC_MaxSPosRearRef_si16 (((gType_parPSM_DiscardCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteriaCPSC_uix]))->MaxSPosRearRef_si16)
#define g_parGetParaPSM_DiscardCriteriaCPSC_MinSPosRearRef_si16 (((gType_parPSM_DiscardCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteriaCPSC_uix]))->MinSPosRearRef_si16)
#define g_parGetParaPSM_DiscardCriteriaCPSC_MaxAbsoluteDist_ui16 (((gType_parPSM_DiscardCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteriaCPSC_uix]))->MaxAbsoluteDist_ui16)
#define g_parGetParaPSM_DiscardCriteriaCPSC_MaxYPos_si16 (((gType_parPSM_DiscardCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteriaCPSC_uix]))->MaxYPos_si16)
#define g_parGetParaPSM_DiscardCriteriaCPSC_MinYPos_si16 (((gType_parPSM_DiscardCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteriaCPSC_uix]))->MinYPos_si16)
#define g_parGetParaPSM_DiscardCriteriaCPSC_MaxPhi_ui16 (((gType_parPSM_DiscardCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteriaCPSC_uix]))->MaxPhi_ui16)
#define g_parGetParaPSM_DiscardCriteriaCPSC_MinPhi_ui16 (((gType_parPSM_DiscardCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteriaCPSC_uix]))->MinPhi_ui16)
#define g_parGetParaPSM_DiscardCriteriaCPSC_MaxPhiB4ThrshldX_ui16 (((gType_parPSM_DiscardCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteriaCPSC_uix]))->MaxPhiB4ThrshldX_ui16)
#define g_parGetParaPSM_DiscardCriteriaCPSC_MinPhiB4ThrshldX_ui16 (((gType_parPSM_DiscardCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteriaCPSC_uix]))->MinPhiB4ThrshldX_ui16)
#define g_parGetParaPSM_DiscardCriteriaCPSC_ThresholdPhiX_si16 (((gType_parPSM_DiscardCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteriaCPSC_uix]))->ThresholdPhiX_si16)
#define g_parGetParaPSM_DiscardCriteriaCPSC_MaxAbsoluteDistOMA_ui16 (((gType_parPSM_DiscardCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_DiscardCriteriaCPSC_uix]))->MaxAbsoluteDistOMA_ui16)
#define g_parGetParaPSM_ParkableCriteria_MaxYPos_si16 (((gType_parPSM_ParkableCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteria_uix]))->MaxYPos_si16)
#define g_parGetParaPSM_ParkableCriteria_MinYPos_si16 (((gType_parPSM_ParkableCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteria_uix]))->MinYPos_si16)
#define g_parGetParaPSM_ParkableCriteria_MaxXPos_si16 (((gType_parPSM_ParkableCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteria_uix]))->MaxXPos_si16)
#define g_parGetParaPSM_ParkableCriteria_MaxSPos_si16 (((gType_parPSM_ParkableCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteria_uix]))->MaxSPos_si16)
#define g_parGetParaPSM_ParkableCriteria_ResXPos_si16 (((gType_parPSM_ParkableCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteria_uix]))->ResXPos_si16)
#define g_parGetParaPSM_ParkableCriteria_MinXPos_si16 (((gType_parPSM_ParkableCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteria_uix]))->MinXPos_si16)
#define g_parGetParaPSM_ParkableCriteria_MaxPhi_ui16 (((gType_parPSM_ParkableCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteria_uix]))->MaxPhi_ui16)
#define g_parGetParaPSM_ParkableCriteria_MinPhi_ui16 (((gType_parPSM_ParkableCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteria_uix]))->MinPhi_ui16)
#define g_parGetParaPSM_ParkableCriteria_MaxSWA_si16 (((gType_parPSM_ParkableCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteria_uix]))->MaxSWA_si16)
#define g_parGetParaPSM_ParkableCriteriaCPSC_MaxYPos_si16 (((gType_parPSM_ParkableCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteriaCPSC_uix]))->MaxYPos_si16)
#define g_parGetParaPSM_ParkableCriteriaCPSC_MinYPos_si16 (((gType_parPSM_ParkableCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteriaCPSC_uix]))->MinYPos_si16)
#define g_parGetParaPSM_ParkableCriteriaCPSC_MaxXPos_si16 (((gType_parPSM_ParkableCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteriaCPSC_uix]))->MaxXPos_si16)
#define g_parGetParaPSM_ParkableCriteriaCPSC_MaxSPos_si16 (((gType_parPSM_ParkableCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteriaCPSC_uix]))->MaxSPos_si16)
#define g_parGetParaPSM_ParkableCriteriaCPSC_ResXPos_si16 (((gType_parPSM_ParkableCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteriaCPSC_uix]))->ResXPos_si16)
#define g_parGetParaPSM_ParkableCriteriaCPSC_MinXPos_si16 (((gType_parPSM_ParkableCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteriaCPSC_uix]))->MinXPos_si16)
#define g_parGetParaPSM_ParkableCriteriaCPSC_MaxPhi_ui16 (((gType_parPSM_ParkableCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteriaCPSC_uix]))->MaxPhi_ui16)
#define g_parGetParaPSM_ParkableCriteriaCPSC_MinPhi_ui16 (((gType_parPSM_ParkableCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteriaCPSC_uix]))->MinPhi_ui16)
#define g_parGetParaPSM_ParkableCriteriaCPSC_MaxSWA_si16 (((gType_parPSM_ParkableCriteriaCPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParkableCriteriaCPSC_uix]))->MaxSWA_si16)
#define g_parGetParaPSM_FilterCriteria_MinParkable_ui8 (((gType_parPSM_FilterCriteria_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_FilterCriteria_uix]))->MinParkable_ui8)
#define g_parGetParaPSM_ParametersDPSC_DiagonalPSAngle_si8 (((gType_parPSM_ParametersDPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParametersDPSC_uix]))->DiagonalPSAngle_si8)
#define g_parGetParaPSM_ParametersDPSC_DiagonalXOffset_ui8 (((gType_parPSM_ParametersDPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParametersDPSC_uix]))->DiagonalXOffset_ui8)
#define g_parGetParaPSM_ParametersDPSC_DiagonalMinLen_ui16 (((gType_parPSM_ParametersDPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSM_ParametersDPSC_uix]))->DiagonalMinLen_ui16)

/*--------------------------------------------------------------------------*/
/*- MP                                                                     -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaMP_MAX_USED_SENSORS_ui8 gd_MP_MAX_USED_SENSORS_ui8
#define g_parGetParaMP_NUMBER_OF_PAS_ECHOES_ui8 gd_MP_NUMBER_OF_PAS_ECHOES_ui8
#define g_parGetParaMP_NUMBER_OF_PSX_ECHOES_ui8 gd_MP_NUMBER_OF_PSX_ECHOES_ui8
#define g_parGetParaMP_NUMBER_OF_SVA_ECHOES_ui8 gd_MP_NUMBER_OF_SVA_ECHOES_ui8
#define g_parGetParaMP_NUMBER_OF_POC_ECHOES_ui8 gd_MP_NUMBER_OF_POC_ECHOES_ui8
#define g_parGetParaMP_USS_IF_UBATT_THRESHOLD_ui8 gd_MP_USS_IF_UBATT_THRESHOLD_ui8
#define g_parGetParaMP_RINGING_DROP_BORDER_ui16 gd_MP_RINGING_DROP_BORDER_ui16
#define g_parGetParaMP_MAX_RINGING_DROP_WIDTH_si16 gd_MP_MAX_RINGING_DROP_WIDTH_si16
#define g_parGetParaMP_RINGING_DROP_BORDER_TESTMODE_ui16 gd_MP_RINGING_DROP_BORDER_TESTMODE_ui16
#define g_parGetParaMP_MAX_RINGING_DROP_WIDTH_TESTMODE_si16 gd_MP_MAX_RINGING_DROP_WIDTH_TESTMODE_si16
#define g_parGetParaMP_CLOSE_OBJ_DIS_SPEED_SIDE_SENSOR_ui16 gd_MP_CLOSE_OBJ_DIS_SPEED_SIDE_SENSOR_ui16
#define g_parGetParaMP_CLOSE_OBJ_DIS_SPEED_ui16 gd_MP_CLOSE_OBJ_DIS_SPEED_ui16
#define g_parGetParaMP_OUTSIDE_TEMP_DEFAULT_si8 gd_MP_OUTSIDE_TEMP_DEFAULT_si8
#define g_parGetParaMP_DDW_DURATION_ui8 gd_MP_DDW_DURATION_ui8
#define g_parGetParaMP_MAX_DISTURBANCE_DURATION_ui16 gd_MP_MAX_DISTURBANCE_DURATION_ui16
#define g_parGetParaMP_RUNTIME_TEST_PHASE_1_ui16 gd_MP_RUNTIME_TEST_PHASE_1_ui16
#define g_parGetParaMP_RUNTIME_TEST_PHASE_2_ui16 gd_MP_RUNTIME_TEST_PHASE_2_ui16
#define g_parGetParaMP_USS_b1_ui16 gd_MP_USS_b1_ui16
#define g_parGetParaMP_USS_b2_ui16 gd_MP_USS_b2_ui16
#define g_parGetParaMP_USS_b3_ui16 gd_MP_USS_b3_ui16
#define g_parGetParaMP_USS_b4_ui16 gd_MP_USS_b4_ui16
#define g_parGetParaMP_USS_a1_ui16 gd_MP_USS_a1_ui16
#define g_parGetParaMP_USS_a2_ui16 gd_MP_USS_a2_ui16
#define g_parGetParaMP_USS_a3_ui16 gd_MP_USS_a3_ui16
#define g_parGetParaMP_USS_a4_ui16 gd_MP_USS_a4_ui16
#define g_parGetParaMP_USS_ar_ui16 gd_MP_USS_ar_ui16
#define g_parGetParaMP_USS_IF_RX_FILTER_ui16 gd_MP_USS_IF_RX_FILTER_ui16
#define g_parGetParaMP_USS_TS_ui16 gd_MP_USS_TS_ui16
#define g_parGetParaMP_USS_TMF_ui16 gd_MP_USS_TMF_ui16
#define g_parGetParaMP_USS_IF_TSG_POS_ui16 gd_MP_USS_IF_TSG_POS_ui16
#define g_parGetParaMP_USS_IF_TSG_NEG_ui16 gd_MP_USS_IF_TSG_NEG_ui16
#define g_parGetParaMP_USS_IF_TSEN_POS_ui16 gd_MP_USS_IF_TSEN_POS_ui16
#define g_parGetParaMP_USS_IF_TSEN_NEG_ui16 gd_MP_USS_IF_TSEN_NEG_ui16
#define g_parGetParaMP_SPEED_BORDER_LOW_MID_ui16 gd_MP_SPEED_BORDER_LOW_MID_ui16
#define g_parGetParaMP_SPEED_BORDER_MID_HIGH_ui16 gd_MP_SPEED_BORDER_MID_HIGH_ui16
#define g_parGetParaMP_LOW_SPEED_MIN_TIME_ui16 gd_MP_LOW_SPEED_MIN_TIME_ui16
#define g_parGetParaMP_MID_SPEED_MIN_TIME_ui16 gd_MP_MID_SPEED_MIN_TIME_ui16
#define g_parGetParaMP_HIGH_SPEED_MIN_TIME_ui16 gd_MP_HIGH_SPEED_MIN_TIME_ui16
#define g_parGetParaMP_MEAS_OBJ_TIME_OUT_ui16 gd_MP_MEAS_OBJ_TIME_OUT_ui16
#define g_parGetParaMP_MEAS_RINGING_END_TIME_ui16 gd_MP_MEAS_RINGING_END_TIME_ui16
#define g_parGetParaMP_MEAS_RINGING_START_TIME_ui16 gd_MP_MEAS_RINGING_START_TIME_ui16
#define g_parGetParaMP_MAX_NUMBER_OF_SENSORS_ui8 gd_MP_MAX_NUMBER_OF_SENSORS_ui8
#define g_parGetParaMP_MAX_NUMBER_OF_SENSORS_MASK_ui16 gd_MP_MAX_NUMBER_OF_SENSORS_MASK_ui16
#define g_parGetParaMP_MAX_NUMBER_OF_HW_CANNELS_ui8 gd_MP_MAX_NUMBER_OF_HW_CANNELS_ui8
#define g_parGetParaMP_ECHO_DIST_MIN_ui16 gd_MP_ECHO_DIST_MIN_ui16
#define g_parGetParaMP_PULS_WIDTH_DIST_MIN_ui16 gd_MP_PULS_WIDTH_DIST_MIN_ui16
#define g_parGetParaMP_SENSOR_STATUS_FAST_CHECK_RATE_si16 gd_MP_SENSOR_STATUS_FAST_CHECK_RATE_si16
#define g_parGetParaMP_SensorTimes_mpMaxMeasTime_ui16 (((gType_parMP_SensorTimes_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorTimes_uix]))->mpMaxMeasTime_ui16)
#define g_parGetParaMP_SensorTimes_mpStopTimeVerifyShot_ui16 (((gType_parMP_SensorTimes_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorTimes_uix]))->mpStopTimeVerifyShot_ui16)
#define g_parGetParaMP_SensorStatic_DeIdleTime_ui16 (((gType_parMP_SensorStatic_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorStatic_uix]))->DeIdleTime_ui16)
#define g_parGetParaMP_SensorStatic_CeInnerIdleTime_ui16 (((gType_parMP_SensorStatic_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorStatic_uix]))->CeInnerIdleTime_ui16)
#define g_parGetParaMP_SensorStatic_CeOuterIdleTime_ui16 (((gType_parMP_SensorStatic_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorStatic_uix]))->CeOuterIdleTime_ui16)
#define g_parGetParaMP_PdcMeasGroupParId_mpFrontGroupId_ui8 (((gType_parMP_PdcMeasGroupParId_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_PdcMeasGroupParId_uix]))->mpFrontGroupId_ui8)
#define g_parGetParaMP_PdcMeasGroupParId_mpRearGroupId_ui8 (((gType_parMP_PdcMeasGroupParId_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_PdcMeasGroupParId_uix]))->mpRearGroupId_ui8)
#define g_parGetParaMP_Timings_mpSensStatusCheckTime_si16 (((gType_parMP_Timings_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_Timings_uix]))->mpSensStatusCheckTime_si16)
#define g_parGetParaMP_Timings_mpSensPwrCheckTime_si16 (((gType_parMP_Timings_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_Timings_uix]))->mpSensPwrCheckTime_si16)
#define g_parGetParaMP_Timings_mpSensTempReadTime_si16 (((gType_parMP_Timings_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_Timings_uix]))->mpSensTempReadTime_si16)
#define g_parGetParaMP_Timings_mpSensOSCFreqCheckTime_si16 (((gType_parMP_Timings_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_Timings_uix]))->mpSensOSCFreqCheckTime_si16)
#define g_parGetParaMP_Timings_mpDisturbWaitTime_ui8 (((gType_parMP_Timings_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_Timings_uix]))->mpDisturbWaitTime_ui8)
#define g_parGetParaMP_SensorType_mpSensorPortType_pui8 (((gType_parMP_SensorType_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorType_uix]))->mpSensorPortType_pui8)
#define g_parGetParaMP_SensorOrientation_mpSensorPortOrien_pui8 (((gType_parMP_SensorOrientation_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorOrientation_uix]))->mpSensorPortOrien_pui8)
#define g_parGetParaMP_RingTimeInfor_NumCcSets_ui8 (((gType_parMP_RingTimeInfor_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_RingTimeInfor_uix]))->NumCcSets_ui8)
#define g_parGetParaMP_RingTimeInfor_DefaultIdx_ui8 (((gType_parMP_RingTimeInfor_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_RingTimeInfor_uix]))->DefaultIdx_ui8)
#define g_parGetParaMP_RingTimeInfor_TempRange_psi8 (((gType_parMP_RingTimeInfor_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_RingTimeInfor_uix]))->TempRange_psi8)
#define g_parGetParaMP_RingTimeInfor_TempHyst_si8 (((gType_parMP_RingTimeInfor_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_RingTimeInfor_uix]))->TempHyst_si8)
#define g_parGetParaMP_RingTimeRange_DataRange_pui16 (((gType_parMP_RingTimeRange_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_RingTimeRange_uix]))->DataRange_pui16)
#define g_parGetParaMP_SensorPort_mpSensorPortID01_ui8 (((gType_parMP_SensorPort_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorPort_uix]))->mpSensorPortID01_ui8)
#define g_parGetParaMP_SensorPort_mpSensorPortID02_ui8 (((gType_parMP_SensorPort_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorPort_uix]))->mpSensorPortID02_ui8)
#define g_parGetParaMP_SensorPort_mpSensorPortID03_ui8 (((gType_parMP_SensorPort_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorPort_uix]))->mpSensorPortID03_ui8)
#define g_parGetParaMP_SensorPort_mpSensorPortID04_ui8 (((gType_parMP_SensorPort_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorPort_uix]))->mpSensorPortID04_ui8)
#define g_parGetParaMP_SensorPort_mpSensorPortID05_ui8 (((gType_parMP_SensorPort_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorPort_uix]))->mpSensorPortID05_ui8)
#define g_parGetParaMP_SensorPort_mpSensorPortID06_ui8 (((gType_parMP_SensorPort_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorPort_uix]))->mpSensorPortID06_ui8)
#define g_parGetParaMP_SensorPort_mpSensorPortID07_ui8 (((gType_parMP_SensorPort_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorPort_uix]))->mpSensorPortID07_ui8)
#define g_parGetParaMP_SensorPort_mpSensorPortID08_ui8 (((gType_parMP_SensorPort_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorPort_uix]))->mpSensorPortID08_ui8)
#define g_parGetParaMP_SensorPort_mpSensorPortID09_ui8 (((gType_parMP_SensorPort_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorPort_uix]))->mpSensorPortID09_ui8)
#define g_parGetParaMP_SensorPort_mpSensorPortID10_ui8 (((gType_parMP_SensorPort_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorPort_uix]))->mpSensorPortID10_ui8)
#define g_parGetParaMP_SensorPort_mpSensorPortID11_ui8 (((gType_parMP_SensorPort_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorPort_uix]))->mpSensorPortID11_ui8)
#define g_parGetParaMP_SensorPort_mpSensorPortID12_ui8 (((gType_parMP_SensorPort_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorPort_uix]))->mpSensorPortID12_ui8)
#define g_parGetParaMP_SensorPort_mpSensorPortID13_ui8 (((gType_parMP_SensorPort_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorPort_uix]))->mpSensorPortID13_ui8)
#define g_parGetParaMP_SensorPort_mpSensorPortID14_ui8 (((gType_parMP_SensorPort_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_SensorPort_uix]))->mpSensorPortID14_ui8)

/*--------------------------------------------------------------------------*/
/*- MP_BUFF                                                                -*/
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*- MP_PL                                                                  -*/
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*- MTL                                                                    -*/
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*- OD                                                                     -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaOD_OUTERMOST_FRONT_LEFT_SENSOR_ui8 gd_OD_OUTERMOST_FRONT_LEFT_SENSOR_ui8
#define g_parGetParaOD_OUTERMOST_FRONT_RIGHT_SENSOR_ui8 gd_OD_OUTERMOST_FRONT_RIGHT_SENSOR_ui8
#define g_parGetParaOD_OUTERMOST_REAR_LEFT_SENSOR_ui8 gd_OD_OUTERMOST_REAR_LEFT_SENSOR_ui8
#define g_parGetParaOD_OUTERMOST_REAR_RIGHT_SENSOR_ui8 gd_OD_OUTERMOST_REAR_RIGHT_SENSOR_ui8
#define g_parGetParaOD_General_NumCyclesStartup_ui8 (((gType_parOD_General_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_General_uix]))->NumCyclesStartup_ui8)
#define g_parGetParaOD_General_LowSpeed_ui8 (((gType_parOD_General_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_General_uix]))->LowSpeed_ui8)
#define g_parGetParaOD_MaskActVelo_odMaskActVeloPdcExclusive_decel_ui16 (((gType_parOD_MaskActVelo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_MaskActVelo_uix]))->odMaskActVeloPdcExclusive_decel_ui16)
#define g_parGetParaOD_MaskActVelo_odMaskActVeloPdcExtended_accel_ui16 (((gType_parOD_MaskActVelo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_MaskActVelo_uix]))->odMaskActVeloPdcExtended_accel_ui16)
#define g_parGetParaOD_MaskActVelo_odMaskActVeloPdcExtended_decel_ui16 (((gType_parOD_MaskActVelo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_MaskActVelo_uix]))->odMaskActVeloPdcExtended_decel_ui16)
#define g_parGetParaOD_MaskActVelo_odMaskActVeloPdcCornerFront_accel_ui16 (((gType_parOD_MaskActVelo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_MaskActVelo_uix]))->odMaskActVeloPdcCornerFront_accel_ui16)
#define g_parGetParaOD_MaskActVelo_odMaskActVeloPdcCornerFront_decel_ui16 (((gType_parOD_MaskActVelo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_MaskActVelo_uix]))->odMaskActVeloPdcCornerFront_decel_ui16)
#define g_parGetParaOD_MaskActVelo_odMaskActVeloPsxBsd_accel_ui16 (((gType_parOD_MaskActVelo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_MaskActVelo_uix]))->odMaskActVeloPsxBsd_accel_ui16)
#define g_parGetParaOD_MaskActVelo_odMaskActVeloPsxBsd_decel_ui16 (((gType_parOD_MaskActVelo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_MaskActVelo_uix]))->odMaskActVeloPsxBsd_decel_ui16)
#define g_parGetParaOD_MaskActVelo_odMaskActVeloBsdExclusive_accel_ui16 (((gType_parOD_MaskActVelo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_MaskActVelo_uix]))->odMaskActVeloBsdExclusive_accel_ui16)
#define g_parGetParaOD_IdleDist_IdleDistDeFront_pui16 (((gType_parOD_IdleDist_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_IdleDist_uix]))->IdleDistDeFront_pui16)
#define g_parGetParaOD_IdleDist_IdleDistCeFront_pui16 (((gType_parOD_IdleDist_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_IdleDist_uix]))->IdleDistCeFront_pui16)
#define g_parGetParaOD_IdleDist_IdleDistDeRear_pui16 (((gType_parOD_IdleDist_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_IdleDist_uix]))->IdleDistDeRear_pui16)
#define g_parGetParaOD_IdleDist_IdleDistCeRear_pui16 (((gType_parOD_IdleDist_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_IdleDist_uix]))->IdleDistCeRear_pui16)
#define g_parGetParaOD_Sens_SensDistFront_pui16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensDistFront_pui16)
#define g_parGetParaOD_Sens_SensDistRear_pui16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensDistRear_pui16)
#define g_parGetParaOD_Sens_SensPosXFront_psi16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensPosXFront_psi16)
#define g_parGetParaOD_Sens_SensPosYFront_psi16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensPosYFront_psi16)
#define g_parGetParaOD_Sens_SensPosZFront_psi16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensPosZFront_psi16)
#define g_parGetParaOD_Sens_SensPosXRear_psi16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensPosXRear_psi16)
#define g_parGetParaOD_Sens_SensPosYRear_psi16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensPosYRear_psi16)
#define g_parGetParaOD_Sens_SensPosZRear_psi16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensPosZRear_psi16)
#define g_parGetParaOD_Sens_SensBeamDirFront_psi16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensBeamDirFront_psi16)
#define g_parGetParaOD_Sens_SensBeamWidthFront_psi16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensBeamWidthFront_psi16)
#define g_parGetParaOD_Sens_SensBeamWidth4DeplausWall_psi16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensBeamWidth4DeplausWall_psi16)
#define g_parGetParaOD_Sens_SensBeamWidth4DeplausPnt_psi16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensBeamWidth4DeplausPnt_psi16)
#define g_parGetParaOD_Sens_SensBeamDist4DeplausMajorObj_ui16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensBeamDist4DeplausMajorObj_ui16)
#define g_parGetParaOD_Sens_SensBeamDist4DeplausMinorObj_ui16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensBeamDist4DeplausMinorObj_ui16)
#define g_parGetParaOD_Sens_SensBeamDistMin4CurbstoneFront_pui16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensBeamDistMin4CurbstoneFront_pui16)
#define g_parGetParaOD_Sens_SensBeamDistMin4CurbstoneRear_pui16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensBeamDistMin4CurbstoneRear_pui16)
#define g_parGetParaOD_Sens_SensBeamDistFront_pui16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensBeamDistFront_pui16)
#define g_parGetParaOD_Sens_SensBeamDirRear_psi16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensBeamDirRear_psi16)
#define g_parGetParaOD_Sens_SensBeamWidthRear_psi16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensBeamWidthRear_psi16)
#define g_parGetParaOD_Sens_SensBeamDistRear_pui16 (((gType_parOD_Sens_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Sens_uix]))->SensBeamDistRear_pui16)
#define g_parGetParaOD_Veh_VehWidthHalf_ui16 (((gType_parOD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Veh_uix]))->VehWidthHalf_ui16)
#define g_parGetParaOD_Veh_NumberContourPntsFront_ui8 (((gType_parOD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Veh_uix]))->NumberContourPntsFront_ui8)
#define g_parGetParaOD_Veh_NumberContourPntsRear_ui8 (((gType_parOD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Veh_uix]))->NumberContourPntsRear_ui8)
#define g_parGetParaOD_Veh_NumberContourPntsSide_ui8 (((gType_parOD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Veh_uix]))->NumberContourPntsSide_ui8)
#define g_parGetParaOD_Veh_VehContourFrontX_psi16 (((gType_parOD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Veh_uix]))->VehContourFrontX_psi16)
#define g_parGetParaOD_Veh_VehContourFrontY_psi16 (((gType_parOD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Veh_uix]))->VehContourFrontY_psi16)
#define g_parGetParaOD_Veh_VehContourRearX_psi16 (((gType_parOD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Veh_uix]))->VehContourRearX_psi16)
#define g_parGetParaOD_Veh_VehContourRearY_psi16 (((gType_parOD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Veh_uix]))->VehContourRearY_psi16)
#define g_parGetParaOD_Veh_VehContourSideX_psi16 (((gType_parOD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Veh_uix]))->VehContourSideX_psi16)
#define g_parGetParaOD_Veh_VehContourSideY_psi16 (((gType_parOD_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Veh_uix]))->VehContourSideY_psi16)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoDelta1_ui16 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoDelta1_ui16)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoDelta2_ui16 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoDelta2_ui16)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoDelta3_ui16 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoDelta3_ui16)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoDeltaCe_ui16 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoDeltaCe_ui16)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoDelta1KMH_ui16 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoDelta1KMH_ui16)
#define g_parGetParaOD_Echo_ApFlt_EchoDeltaOfMaxSpeed_ui16 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->EchoDeltaOfMaxSpeed_ui16)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoZone1Dist_ui16 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoZone1Dist_ui16)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoZone2Dist_ui16 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoZone2Dist_ui16)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoZone3Dist_ui16 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoZone3Dist_ui16)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoZone4Dist_ui16 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoZone4Dist_ui16)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoDelta1Penalty_ui8 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoDelta1Penalty_ui8)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoDelta2Penalty_ui8 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoDelta2Penalty_ui8)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoDelta3Penalty_ui8 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoDelta3Penalty_ui8)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoZone1Penalty_ui8 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoZone1Penalty_ui8)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoZone2Penalty_ui8 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoZone2Penalty_ui8)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoZone3Penalty_ui8 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoZone3Penalty_ui8)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoZone4Penalty_ui8 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoZone4Penalty_ui8)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoZone5Penalty_ui8 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoZone5Penalty_ui8)
#define g_parGetParaOD_Echo_ApFlt_ApFltEchoZone6Penalty_ui8 (((gType_parOD_Echo_ApFlt_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_ApFlt_uix]))->ApFltEchoZone6Penalty_ui8)
#define g_parGetParaOD_Echo_WRF_WRFEnable_bl (((gType_parOD_Echo_WRF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_WRF_uix]))->WRFEnable_bl)
#define g_parGetParaOD_Echo_WRF_WRFWheelAngLimit_ui16 (((gType_parOD_Echo_WRF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_WRF_uix]))->WRFWheelAngLimit_ui16)
#define g_parGetParaOD_Echo_WRF_WRFWheelOutwardDeLowerLimit_ui16 (((gType_parOD_Echo_WRF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_WRF_uix]))->WRFWheelOutwardDeLowerLimit_ui16)
#define g_parGetParaOD_Echo_WRF_WRFWheelOutwardDeUpperLimit_ui16 (((gType_parOD_Echo_WRF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_WRF_uix]))->WRFWheelOutwardDeUpperLimit_ui16)
#define g_parGetParaOD_Echo_WRF_WRFWheelInwardDeLowerLimit_ui16 (((gType_parOD_Echo_WRF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_WRF_uix]))->WRFWheelInwardDeLowerLimit_ui16)
#define g_parGetParaOD_Echo_WRF_WRFWheelInwardDeUpperLimit_ui16 (((gType_parOD_Echo_WRF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_WRF_uix]))->WRFWheelInwardDeUpperLimit_ui16)
#define g_parGetParaOD_Echo_SRF_SRFEnable_pbl (((gType_parOD_Echo_SRF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_SRF_uix]))->SRFEnable_pbl)
#define g_parGetParaOD_Echo_SRF_SRFDeLimit_pui16 (((gType_parOD_Echo_SRF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_SRF_uix]))->SRFDeLimit_pui16)
#define g_parGetParaOD_Echo_SRF_SRFCeLimit_pui16 (((gType_parOD_Echo_SRF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_SRF_uix]))->SRFCeLimit_pui16)
#define g_parGetParaOD_Echo_HLF_HLFEnable_pbl (((gType_parOD_Echo_HLF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_HLF_uix]))->HLFEnable_pbl)
#define g_parGetParaOD_Echo_HLF_HLFCe23RangeMin_pui16 (((gType_parOD_Echo_HLF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_HLF_uix]))->HLFCe23RangeMin_pui16)
#define g_parGetParaOD_Echo_HLF_HLFCe23RangeMax_pui16 (((gType_parOD_Echo_HLF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_HLF_uix]))->HLFCe23RangeMax_pui16)
#define g_parGetParaOD_Echo_HLF_HLFDe2RangeMin_pui16 (((gType_parOD_Echo_HLF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_HLF_uix]))->HLFDe2RangeMin_pui16)
#define g_parGetParaOD_Echo_HLF_HLFDe2RangeMax_pui16 (((gType_parOD_Echo_HLF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_HLF_uix]))->HLFDe2RangeMax_pui16)
#define g_parGetParaOD_Echo_HLF_HLFCe23Limit_pui16 (((gType_parOD_Echo_HLF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_HLF_uix]))->HLFCe23Limit_pui16)
#define g_parGetParaOD_Echo_HLF_HLFDe2Limit_pui16 (((gType_parOD_Echo_HLF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_HLF_uix]))->HLFDe2Limit_pui16)
#define g_parGetParaOD_Echo_HLF_HLFCe12Limit_pui16 (((gType_parOD_Echo_HLF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_HLF_uix]))->HLFCe12Limit_pui16)
#define g_parGetParaOD_Echo_HLF_HLFDe3LimitForCe23_pui16 (((gType_parOD_Echo_HLF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_HLF_uix]))->HLFDe3LimitForCe23_pui16)
#define g_parGetParaOD_Echo_HLF_HLFDe3LimitForDe2_pui16 (((gType_parOD_Echo_HLF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_HLF_uix]))->HLFDe3LimitForDe2_pui16)
#define g_parGetParaOD_Echo_HLF_HLFSpeedLim_ui8 (((gType_parOD_Echo_HLF_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_HLF_uix]))->HLFSpeedLim_ui8)
#define g_parGetParaOD_Echo_MoveDir_RelMovingDeTol_ui16 (((gType_parOD_Echo_MoveDir_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_MoveDir_uix]))->RelMovingDeTol_ui16)
#define g_parGetParaOD_Echo_MoveDir_RelMovingCeTol_ui16 (((gType_parOD_Echo_MoveDir_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_MoveDir_uix]))->RelMovingCeTol_ui16)
#define g_parGetParaOD_Echo_MoveDir_MaxCyclMovingHold_ui8 (((gType_parOD_Echo_MoveDir_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_MoveDir_uix]))->MaxCyclMovingHold_ui8)
#define g_parGetParaOD_Echo_Clip_Std_FrontDeLimit_pui16 (((gType_parOD_Echo_Clip_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_Clip_Std_uix]))->FrontDeLimit_pui16)
#define g_parGetParaOD_Echo_Clip_Std_FrontCeLimit_pui16 (((gType_parOD_Echo_Clip_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_Clip_Std_uix]))->FrontCeLimit_pui16)
#define g_parGetParaOD_Echo_Clip_Std_RearDeLimit_pui16 (((gType_parOD_Echo_Clip_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_Clip_Std_uix]))->RearDeLimit_pui16)
#define g_parGetParaOD_Echo_Clip_Std_RearCeLimit_pui16 (((gType_parOD_Echo_Clip_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_Clip_Std_uix]))->RearCeLimit_pui16)
#define g_parGetParaOD_Echo_Clip_Std_ClippingHyst_ui16 (((gType_parOD_Echo_Clip_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Echo_Clip_Std_uix]))->ClippingHyst_ui16)
#define g_parGetParaOD_Obj_Meas_CriticalDist_ui16 (((gType_parOD_Obj_Meas_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Meas_uix]))->CriticalDist_ui16)
#define g_parGetParaOD_Obj_Meas_CriticalDistHyst_ui16 (((gType_parOD_Obj_Meas_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Meas_uix]))->CriticalDistHyst_ui16)
#define g_parGetParaOD_Obj_Meas_ObjTypeDeltaDist_ui16 (((gType_parOD_Obj_Meas_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Meas_uix]))->ObjTypeDeltaDist_ui16)
#define g_parGetParaOD_Obj_Meas_OblFiltHystNear_ui16 (((gType_parOD_Obj_Meas_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Meas_uix]))->OblFiltHystNear_ui16)
#define g_parGetParaOD_Obj_Meas_OblFiltHystFar_ui16 (((gType_parOD_Obj_Meas_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Meas_uix]))->OblFiltHystFar_ui16)
#define g_parGetParaOD_Obj_Trck_EnableTrackEndless_bl (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->EnableTrackEndless_bl)
#define g_parGetParaOD_Obj_Trck_TrackCycleMax_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->TrackCycleMax_ui16)
#define g_parGetParaOD_Obj_Trck_TrackCycleMaxWall_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->TrackCycleMaxWall_ui16)
#define g_parGetParaOD_Obj_Trck_TrackCycleMin_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->TrackCycleMin_ui16)
#define g_parGetParaOD_Obj_Trck_TrackCycleWait4Deplaus_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->TrackCycleWait4Deplaus_ui16)
#define g_parGetParaOD_Obj_Trck_TrackDistMax_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->TrackDistMax_ui16)
#define g_parGetParaOD_Obj_Trck_TrackVehDistFront_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->TrackVehDistFront_ui16)
#define g_parGetParaOD_Obj_Trck_TrackVehDistSide_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->TrackVehDistSide_ui16)
#define g_parGetParaOD_Obj_Trck_RelevanceRange4HighLevelObjFront_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->RelevanceRange4HighLevelObjFront_ui16)
#define g_parGetParaOD_Obj_Trck_RelevanceRange4HighLevelObjRear_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->RelevanceRange4HighLevelObjRear_ui16)
#define g_parGetParaOD_Obj_Trck_RelevanceRange4HighLevelObjSide_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->RelevanceRange4HighLevelObjSide_ui16)
#define g_parGetParaOD_Obj_Trck_TrackVehDistMin_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->TrackVehDistMin_ui16)
#define g_parGetParaOD_Obj_Trck_MatchWindowInit_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->MatchWindowInit_ui16)
#define g_parGetParaOD_Obj_Trck_MatchWindowMax_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->MatchWindowMax_ui16)
#define g_parGetParaOD_Obj_Trck_MatchWindowTrackCycleFactor_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->MatchWindowTrackCycleFactor_ui16)
#define g_parGetParaOD_Obj_Trck_MatchWindowTrackDistFactor_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->MatchWindowTrackDistFactor_ui16)
#define g_parGetParaOD_Obj_Trck_MatchWallTolX_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->MatchWallTolX_ui16)
#define g_parGetParaOD_Obj_Trck_MatchWallTolXCeObj_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->MatchWallTolXCeObj_ui16)
#define g_parGetParaOD_Obj_Trck_CosAngleToleranceWall_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->CosAngleToleranceWall_ui16)
#define g_parGetParaOD_Obj_Trck_ExistProbMin_ui8 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->ExistProbMin_ui8)
#define g_parGetParaOD_Obj_Trck_ExistProbLimFrontObj4MaxTrck_ui8 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->ExistProbLimFrontObj4MaxTrck_ui8)
#define g_parGetParaOD_Obj_Trck_ExistProbLimSideObj4MaxTrck_ui8 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->ExistProbLimSideObj4MaxTrck_ui8)
#define g_parGetParaOD_Obj_Trck_ExistProbMatchFactor_ui8 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->ExistProbMatchFactor_ui8)
#define g_parGetParaOD_Obj_Trck_ExistProbMatchSlowFactor_ui8 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->ExistProbMatchSlowFactor_ui8)
#define g_parGetParaOD_Obj_Trck_ExistProbDecFactor_ui8 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->ExistProbDecFactor_ui8)
#define g_parGetParaOD_Obj_Trck_ExistProbDecFastFactor_ui8 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->ExistProbDecFastFactor_ui8)
#define g_parGetParaOD_Obj_Trck_ExistProbDecFast2Factor_ui8 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->ExistProbDecFast2Factor_ui8)
#define g_parGetParaOD_Obj_Trck_ObjRelMax4ExistProbMin_ui8 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->ObjRelMax4ExistProbMin_ui8)
#define g_parGetParaOD_Obj_Trck_ObjMoveDeltaAngleMax_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->ObjMoveDeltaAngleMax_ui16)
#define g_parGetParaOD_Obj_Trck_ObjMoveDeltaDistFak_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->ObjMoveDeltaDistFak_ui16)
#define g_parGetParaOD_Obj_Trck_SensDistMin4WallType_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->SensDistMin4WallType_ui16)
#define g_parGetParaOD_Obj_Trck_WallLengthMax_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->WallLengthMax_ui16)
#define g_parGetParaOD_Obj_Trck_WallLengthMax4AngleCorr_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->WallLengthMax4AngleCorr_ui16)
#define g_parGetParaOD_Obj_Trck_WallLengthMax4PlausDec_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->WallLengthMax4PlausDec_ui16)
#define g_parGetParaOD_Obj_Trck_DoubleEchoTol4HighObj_ui16 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->DoubleEchoTol4HighObj_ui16)
#define g_parGetParaOD_Obj_Trck_HeightClassificationLow_ui8 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->HeightClassificationLow_ui8)
#define g_parGetParaOD_Obj_Trck_HeightClassificationHigh_ui8 (((gType_parOD_Obj_Trck_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_OD_Obj_Trck_uix]))->HeightClassificationHigh_ui8)

/*--------------------------------------------------------------------------*/
/*- PAR                                                                    -*/
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*- SP_FSV                                                                 -*/
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*- SP_PWR                                                                 -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaSP_PWR_CFG_UbattMax_ui8 (((gType_parSP_PWR_CFG_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_SP_PWR_CFG_uix]))->UbattMax_ui8)
#define g_parGetParaSP_PWR_CFG_UbattMaxHysteresis_ui8 (((gType_parSP_PWR_CFG_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_SP_PWR_CFG_uix]))->UbattMaxHysteresis_ui8)
#define g_parGetParaSP_PWR_CFG_UbattMin_ui8 (((gType_parSP_PWR_CFG_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_SP_PWR_CFG_uix]))->UbattMin_ui8)
#define g_parGetParaSP_PWR_CFG_UbattMinHysteresis_ui8 (((gType_parSP_PWR_CFG_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_SP_PWR_CFG_uix]))->UbattMinHysteresis_ui8)

/*--------------------------------------------------------------------------*/
/*- SMS                                                                    -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaSMS_InitTOutCycles_ui8 gd_SMS_InitTOutCycles_ui8

/*--------------------------------------------------------------------------*/
/*- VHS_PL_TORQUE                                                          -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_PosPID_Kp_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->PosPID_Kp_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_PosPID_KpScale_si8 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->PosPID_KpScale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_PosPID_Ki_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->PosPID_Ki_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_PosPID_KiScale_si8 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->PosPID_KiScale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_PosPID_Kd_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->PosPID_Kd_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_PosPID_KdScale_si8 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->PosPID_KdScale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_PosPID_MaxIntegral_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->PosPID_MaxIntegral_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_PosPID_MaxIntegralScale_si8 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->PosPID_MaxIntegralScale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_SpeedPID_Kp_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->SpeedPID_Kp_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_SpeedPID_KpScale_si8 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->SpeedPID_KpScale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_SpeedPID_Ki_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->SpeedPID_Ki_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_SpeedPID_KiScale_si8 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->SpeedPID_KiScale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_SpeedPID_Kd_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->SpeedPID_Kd_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_SpeedPID_KdScale_si8 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->SpeedPID_KdScale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_SpeedPID_MaxIntegral_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->SpeedPID_MaxIntegral_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_SpeedPID_MaxIntegralScale_si8 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->SpeedPID_MaxIntegralScale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_PosPIDscale_factor_si16 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->PosPIDscale_factor_si16)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_maxSWASpeed_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->maxSWASpeed_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_maxSWAAcc_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->maxSWAAcc_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_maxTorqueReq_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->maxTorqueReq_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_maxTorqueReq_scale_si8 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->maxTorqueReq_scale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_maxTorqueSlewRate_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->maxTorqueSlewRate_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_outputFIR_size_ui8 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->outputFIR_size_ui8)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_outputFIR_coefficient0_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->outputFIR_coefficient0_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_outputFIR_coefficient1_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->outputFIR_coefficient1_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_outputFIR_coefficient2_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->outputFIR_coefficient2_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_outputFIR_coefficient3_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->outputFIR_coefficient3_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_outputFIR_coefficient4_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->outputFIR_coefficient4_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_outputFIR_coefficient5_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->outputFIR_coefficient5_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_outputFIR_coefficient6_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->outputFIR_coefficient6_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_outputFIR_coefficient7_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->outputFIR_coefficient7_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_outputFIR_coefficient8_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->outputFIR_coefficient8_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_outputFIR_coefficient9_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->outputFIR_coefficient9_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_outputFIR_coefficient_scaling_si8 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->outputFIR_coefficient_scaling_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_reqFIR_size_ui8 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_reqFIR_size_ui8)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_reqFIR_coefficient0_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_reqFIR_coefficient0_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_reqFIR_coefficient1_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_reqFIR_coefficient1_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_reqFIR_coefficient2_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_reqFIR_coefficient2_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_reqFIR_coefficient3_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_reqFIR_coefficient3_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_reqFIR_coefficient4_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_reqFIR_coefficient4_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_reqFIR_coefficient5_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_reqFIR_coefficient5_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_reqFIR_coefficient6_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_reqFIR_coefficient6_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_reqFIR_coefficient7_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_reqFIR_coefficient7_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_reqFIR_coefficient8_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_reqFIR_coefficient8_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_reqFIR_coefficient9_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_reqFIR_coefficient9_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_reqFIR_coefficient_scaling_si8 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_reqFIR_coefficient_scaling_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_curFIR_size_ui8 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_curFIR_size_ui8)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_curFIR_coefficient0_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_curFIR_coefficient0_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_curFIR_coefficient1_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_curFIR_coefficient1_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_curFIR_coefficient2_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_curFIR_coefficient2_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_curFIR_coefficient3_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_curFIR_coefficient3_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_curFIR_coefficient4_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_curFIR_coefficient4_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_curFIR_coefficient5_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_curFIR_coefficient5_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_curFIR_coefficient6_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_curFIR_coefficient6_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_curFIR_coefficient7_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_curFIR_coefficient7_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_curFIR_coefficient8_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_curFIR_coefficient8_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_curFIR_coefficient9_si32 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_curFIR_coefficient9_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_ZFLS_speed_curFIR_coefficient_scaling_si8 (((gType_parVHS_PL_TORQUE_Controller_ZFLS_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_ZFLS_uix]))->speed_curFIR_coefficient_scaling_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_B_si32 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->B_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_B_scale_si8 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->B_scale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_a11_si32 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_a11_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_a11_scale_si8 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_a11_scale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_a12_si32 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_a12_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_a12_scale_si8 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_a12_scale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_a13_si32 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_a13_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_a13_scale_si8 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_a13_scale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_a22_si32 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_a22_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_a22_scale_si8 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_a22_scale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_a23_si32 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_a23_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_a23_scale_si8 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_a23_scale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_a33_si32 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_a33_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_a33_scale_si8 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_a33_scale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_b1_si32 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_b1_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_b1_scale_si8 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_b1_scale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_b2_si32 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_b2_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_b2_scale_si8 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_b2_scale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_L1_si32 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_L1_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_L1_scale_si8 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_L1_scale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_L2_si32 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_L2_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_L2_scale_si8 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_L2_scale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_L3_si32 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_L3_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_Observer_L3_scale_si8 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->Observer_L3_scale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_K1_si32 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->K1_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_K1_scale_si8 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->K1_scale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_K2_si32 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->K2_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_K2_scale_si8 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->K2_scale_si8)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_ratio_collumn_motor_si32 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->ratio_collumn_motor_si32)
#define g_parGetParaVHS_PL_TORQUE_Controller_CR_ratio_collumn_motor_scale_si8 (((gType_parVHS_PL_TORQUE_Controller_CR_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHS_PL_TORQUE_Controller_CR_uix]))->ratio_collumn_motor_scale_si8)

/*--------------------------------------------------------------------------*/
/*- VHS                                                                    -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaVHS_GearboxFilterTime_ui8 gd_VHS_GearboxFilterTime_ui8
#define g_parGetParaVHS_TaskCycleInMS_ui8 gd_VHS_TaskCycleInMS_ui8

/*--------------------------------------------------------------------------*/
/*- AP_BSD                                                                 -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaAP_BSD_Ap_SteWheAngMax_ui16 (((gType_parAP_BSD_Ap_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_BSD_Ap_uix]))->SteWheAngMax_ui16)
#define g_parGetParaAP_BSD_Ap_MinWarnTime_ui16 (((gType_parAP_BSD_Ap_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_BSD_Ap_uix]))->MinWarnTime_ui16)
#define g_parGetParaAP_BSD_Ap_OvertakenCarSuppressionTime_ui16 (((gType_parAP_BSD_Ap_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_BSD_Ap_uix]))->OvertakenCarSuppressionTime_ui16)
#define g_parGetParaAP_BSD_Ap_OvertakenCarSuppressionTime2_ui16 (((gType_parAP_BSD_Ap_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_BSD_Ap_uix]))->OvertakenCarSuppressionTime2_ui16)
#define g_parGetParaAP_BSD_Ap_OvertakenCarSuppressionTime2Mid_ui16 (((gType_parAP_BSD_Ap_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_BSD_Ap_uix]))->OvertakenCarSuppressionTime2Mid_ui16)
#define g_parGetParaAP_BSD_Ap_OvertaCarSpeedSwitch_ui8 (((gType_parAP_BSD_Ap_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_BSD_Ap_uix]))->OvertaCarSpeedSwitch_ui8)
#define g_parGetParaAP_BSD_Ap_AdditionalWarnTime_ui16 (((gType_parAP_BSD_Ap_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_BSD_Ap_uix]))->AdditionalWarnTime_ui16)
#define g_parGetParaAP_BSD_Ap_SensorCoverageDistance_ui16 (((gType_parAP_BSD_Ap_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_BSD_Ap_uix]))->SensorCoverageDistance_ui16)
#define g_parGetParaAP_BSD_Ap_SuppressStartup_ui16 (((gType_parAP_BSD_Ap_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_BSD_Ap_uix]))->SuppressStartup_ui16)
#define g_parGetParaAP_BSD_Ap_OvertakenCarSuppressionTimeMid_ui16 (((gType_parAP_BSD_Ap_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_BSD_Ap_uix]))->OvertakenCarSuppressionTimeMid_ui16)
#define g_parGetParaAP_BSD_Ap_MidSensorWarnDistLowSpeed_ui16 (((gType_parAP_BSD_Ap_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_BSD_Ap_uix]))->MidSensorWarnDistLowSpeed_ui16)
#define g_parGetParaAP_BSD_Ap_MidSensorWarnDistHighSpeed_ui16 (((gType_parAP_BSD_Ap_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_BSD_Ap_uix]))->MidSensorWarnDistHighSpeed_ui16)

/*--------------------------------------------------------------------------*/
/*- EP_BSD                                                                 -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaEP_BSD_MAP_USS_TO_BSD_FL_ui8 gd_EP_BSD_MAP_USS_TO_BSD_FL_ui8
#define g_parGetParaEP_BSD_MAP_USS_TO_BSD_FR_ui8 gd_EP_BSD_MAP_USS_TO_BSD_FR_ui8
#define g_parGetParaEP_BSD_MAP_USS_TO_BSD_ML_ui8 gd_EP_BSD_MAP_USS_TO_BSD_ML_ui8
#define g_parGetParaEP_BSD_MAP_USS_TO_BSD_MR_ui8 gd_EP_BSD_MAP_USS_TO_BSD_MR_ui8
#define g_parGetParaEP_BSD_MAP_USS_TO_BSD_RR_ui8 gd_EP_BSD_MAP_USS_TO_BSD_RR_ui8
#define g_parGetParaEP_BSD_MAP_USS_TO_BSD_RL_ui8 gd_EP_BSD_MAP_USS_TO_BSD_RL_ui8
#define g_parGetParaEP_BSD_SDI_SENSORS_ui8 gd_EP_BSD_SDI_SENSORS_ui8
#define g_parGetParaEP_BSD_Ep_CharTableOffsetFrontLow_ui8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->CharTableOffsetFrontLow_ui8)
#define g_parGetParaEP_BSD_Ep_CharTableOffsetRearLow_ui8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->CharTableOffsetRearLow_ui8)
#define g_parGetParaEP_BSD_Ep_CharTableOffsetFrontMid_ui8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->CharTableOffsetFrontMid_ui8)
#define g_parGetParaEP_BSD_Ep_CharTableOffsetRearMid_ui8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->CharTableOffsetRearMid_ui8)
#define g_parGetParaEP_BSD_Ep_CharTableOffsetFrontHigh_ui8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->CharTableOffsetFrontHigh_ui8)
#define g_parGetParaEP_BSD_Ep_CharTableOffsetRearHigh_ui8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->CharTableOffsetRearHigh_ui8)
#define g_parGetParaEP_BSD_Ep_CharTableMaxFrontAndRear_ui8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->CharTableMaxFrontAndRear_ui8)
#define g_parGetParaEP_BSD_Ep_CharTempThresholdLow_si8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->CharTempThresholdLow_si8)
#define g_parGetParaEP_BSD_Ep_CharTempThresholdHigh_si8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->CharTempThresholdHigh_si8)
#define g_parGetParaEP_BSD_Ep_BlockedSensorTestAtStartUp_bl (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->BlockedSensorTestAtStartUp_bl)
#define g_parGetParaEP_BSD_Ep_BlockedSensorTestAtMeasure_bl (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->BlockedSensorTestAtMeasure_bl)
#define g_parGetParaEP_BSD_Ep_BlockedSensorTime_ui32 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->BlockedSensorTime_ui32)
#define g_parGetParaEP_BSD_Ep_BlockedSensorTestDuration_ui16 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->BlockedSensorTestDuration_ui16)
#define g_parGetParaEP_BSD_Ep_StayInTestVelocityMin_ui8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->StayInTestVelocityMin_ui8)
#define g_parGetParaEP_BSD_Ep_BlockedSensorTestTempLimit_si8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->BlockedSensorTestTempLimit_si8)
#define g_parGetParaEP_BSD_Ep_HighSpeedTrackingAc_ui8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->HighSpeedTrackingAc_ui8)
#define g_parGetParaEP_BSD_Ep_HighSpeedTrackingDe_ui8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->HighSpeedTrackingDe_ui8)
#define g_parGetParaEP_BSD_Ep_CcIDOffsetFrontHST_si8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->CcIDOffsetFrontHST_si8)
#define g_parGetParaEP_BSD_Ep_CcIDOffsetRearHST_si8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->CcIDOffsetRearHST_si8)
#define g_parGetParaEP_BSD_Ep_VelocityMaxAc_ui8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->VelocityMaxAc_ui8)
#define g_parGetParaEP_BSD_Ep_VelocityMaxDe_ui8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->VelocityMaxDe_ui8)
#define g_parGetParaEP_BSD_Ep_HighNoiseHysteresisFront_ui8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->HighNoiseHysteresisFront_ui8)
#define g_parGetParaEP_BSD_Ep_HighNoiseHysteresisRear_ui8 (((gType_parEP_BSD_Ep_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_Ep_uix]))->HighNoiseHysteresisRear_ui8)
#define g_parGetParaEP_BSD_SDI_MinObjStayTime_ui16 (((gType_parEP_BSD_SDI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_SDI_uix]))->MinObjStayTime_ui16)
#define g_parGetParaEP_BSD_SDI_WarnLvlHoldTime_ui16 (((gType_parEP_BSD_SDI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_SDI_uix]))->WarnLvlHoldTime_ui16)
#define g_parGetParaEP_BSD_SDI_NumOfWarnLevels_ui8 (((gType_parEP_BSD_SDI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_SDI_uix]))->NumOfWarnLevels_ui8)
#define g_parGetParaEP_BSD_SDI_WarnDistancesLeft_pui8 (((gType_parEP_BSD_SDI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_SDI_uix]))->WarnDistancesLeft_pui8)
#define g_parGetParaEP_BSD_SDI_WarnDistancesRight_pui8 (((gType_parEP_BSD_SDI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_EP_BSD_SDI_uix]))->WarnDistancesRight_pui8)

/*--------------------------------------------------------------------------*/
/*- AP_PDC                                                                 -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaAP_PDC_SWD_FRONT_GROUP_LEFT_ui8 gd_AP_PDC_SWD_FRONT_GROUP_LEFT_ui8
#define g_parGetParaAP_PDC_SWD_FRONT_GROUP_RIGHT_ui8 gd_AP_PDC_SWD_FRONT_GROUP_RIGHT_ui8
#define g_parGetParaAP_PDC_SWD_REAR_GROUP_LEFT_ui8 gd_AP_PDC_SWD_REAR_GROUP_LEFT_ui8
#define g_parGetParaAP_PDC_SWD_REAR_GROUP_RIGHT_ui8 gd_AP_PDC_SWD_REAR_GROUP_RIGHT_ui8
#define g_parGetParaAP_PDC_SWD_FRONT_OUTER_SENSOR_LEFT_ui8 gd_AP_PDC_SWD_FRONT_OUTER_SENSOR_LEFT_ui8
#define g_parGetParaAP_PDC_SWD_FRONT_CENTRAL_SENSOR_LEFT_ui8 gd_AP_PDC_SWD_FRONT_CENTRAL_SENSOR_LEFT_ui8
#define g_parGetParaAP_PDC_SWD_FRONT_CENTRAL_SENSOR_RIGHT_ui8 gd_AP_PDC_SWD_FRONT_CENTRAL_SENSOR_RIGHT_ui8
#define g_parGetParaAP_PDC_SWD_FRONT_OUTER_SENSOR_RIGHT_ui8 gd_AP_PDC_SWD_FRONT_OUTER_SENSOR_RIGHT_ui8
#define g_parGetParaAP_PDC_SWD_REAR_OUTER_SENSOR_LEFT_ui8 gd_AP_PDC_SWD_REAR_OUTER_SENSOR_LEFT_ui8
#define g_parGetParaAP_PDC_SWD_REAR_CENTRAL_SENSOR_LEFT_ui8 gd_AP_PDC_SWD_REAR_CENTRAL_SENSOR_LEFT_ui8
#define g_parGetParaAP_PDC_SWD_REAR_CENTRAL_SENSOR_RIGHT_ui8 gd_AP_PDC_SWD_REAR_CENTRAL_SENSOR_RIGHT_ui8
#define g_parGetParaAP_PDC_SWD_REAR_OUTER_SENSOR_RIGHT_ui8 gd_AP_PDC_SWD_REAR_OUTER_SENSOR_RIGHT_ui8
#define g_parGetParaAP_PDC_General_NumCyclesStartupSC_ui8 (((gType_parAP_PDC_General_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_General_uix]))->NumCyclesStartupSC_ui8)
#define g_parGetParaAP_PDC_General_ApPdcTimeOutFront_ui8 (((gType_parAP_PDC_General_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_General_uix]))->ApPdcTimeOutFront_ui8)
#define g_parGetParaAP_PDC_General_ApPdcTimeOutRear_ui8 (((gType_parAP_PDC_General_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_General_uix]))->ApPdcTimeOutRear_ui8)
#define g_parGetParaAP_PDC_Disp_Global_FullWarnDist_ui16 (((gType_parAP_PDC_Disp_Global_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Global_uix]))->FullWarnDist_ui16)
#define g_parGetParaAP_PDC_Disp_Global_DispAreaHoldTime_ui16 (((gType_parAP_PDC_Disp_Global_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Global_uix]))->DispAreaHoldTime_ui16)
#define g_parGetParaAP_PDC_Disp_Global_DispAreaDelayTime_ui16 (((gType_parAP_PDC_Disp_Global_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Global_uix]))->DispAreaDelayTime_ui16)
#define g_parGetParaAP_PDC_Disp_Global_DispAreaApproachHyst_ui16 (((gType_parAP_PDC_Disp_Global_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Global_uix]))->DispAreaApproachHyst_ui16)
#define g_parGetParaAP_PDC_Disp_Global_DispAreaDepartHyst_ui16 (((gType_parAP_PDC_Disp_Global_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Global_uix]))->DispAreaDepartHyst_ui16)
#define g_parGetParaAP_PDC_Disp_Global_DispAreaCloseDepartHyst_ui16 (((gType_parAP_PDC_Disp_Global_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Global_uix]))->DispAreaCloseDepartHyst_ui16)
#define g_parGetParaAP_PDC_Disp_Global_DispDist4CloseDepartHyst_ui16 (((gType_parAP_PDC_Disp_Global_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Global_uix]))->DispDist4CloseDepartHyst_ui16)
#define g_parGetParaAP_PDC_Disp_Global_DispAreaDistRes_ui16 (((gType_parAP_PDC_Disp_Global_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Global_uix]))->DispAreaDistRes_ui16)
#define g_parGetParaAP_PDC_Disp_Global_SafetyDist4OOC_si16 (((gType_parAP_PDC_Disp_Global_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Global_uix]))->SafetyDist4OOC_si16)
#define g_parGetParaAP_PDC_Disp_Global_Dist2CourseHystOOC_si16 (((gType_parAP_PDC_Disp_Global_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Global_uix]))->Dist2CourseHystOOC_si16)
#define g_parGetParaAP_PDC_Disp_Global_DispDistMinimum_ui16 (((gType_parAP_PDC_Disp_Global_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Global_uix]))->DispDistMinimum_ui16)
#define g_parGetParaAP_PDC_Disp_Lat_ObjCaptureTime_ui16 (((gType_parAP_PDC_Disp_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Lat_uix]))->ObjCaptureTime_ui16)
#define g_parGetParaAP_PDC_Disp_Lat_DispAreaFrontMin_pui8 (((gType_parAP_PDC_Disp_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Lat_uix]))->DispAreaFrontMin_pui8)
#define g_parGetParaAP_PDC_Disp_Lat_DispAreaFrontMax_pui8 (((gType_parAP_PDC_Disp_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Lat_uix]))->DispAreaFrontMax_pui8)
#define g_parGetParaAP_PDC_Disp_Lat_DispAreaFrontHystMin_pui8 (((gType_parAP_PDC_Disp_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Lat_uix]))->DispAreaFrontHystMin_pui8)
#define g_parGetParaAP_PDC_Disp_Lat_DispAreaFrontHystMax_pui8 (((gType_parAP_PDC_Disp_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Lat_uix]))->DispAreaFrontHystMax_pui8)
#define g_parGetParaAP_PDC_Disp_Lat_DispAreaRearMin_pui8 (((gType_parAP_PDC_Disp_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Lat_uix]))->DispAreaRearMin_pui8)
#define g_parGetParaAP_PDC_Disp_Lat_DispAreaRearMax_pui8 (((gType_parAP_PDC_Disp_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Lat_uix]))->DispAreaRearMax_pui8)
#define g_parGetParaAP_PDC_Disp_Lat_DispAreaRearHystMin_pui8 (((gType_parAP_PDC_Disp_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Lat_uix]))->DispAreaRearHystMin_pui8)
#define g_parGetParaAP_PDC_Disp_Lat_DispAreaRearHystMax_pui8 (((gType_parAP_PDC_Disp_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Disp_Lat_uix]))->DispAreaRearHystMax_pui8)
#define g_parGetParaAP_PDC_View_Lat_ClipFrontZone_pui16 (((gType_parAP_PDC_View_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Lat_uix]))->ClipFrontZone_pui16)
#define g_parGetParaAP_PDC_View_Lat_ClipRearZone_pui16 (((gType_parAP_PDC_View_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Lat_uix]))->ClipRearZone_pui16)
#define g_parGetParaAP_PDC_View_Lat_ClippingHyst_ui16 (((gType_parAP_PDC_View_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Lat_uix]))->ClippingHyst_ui16)
#define g_parGetParaAP_PDC_View_Prm_ExistProbMin4Disp_ui8 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->ExistProbMin4Disp_ui8)
#define g_parGetParaAP_PDC_View_Prm_ExistProbMinMinorObj4Disp_ui8 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->ExistProbMinMinorObj4Disp_ui8)
#define g_parGetParaAP_PDC_View_Prm_DispAreaFrontA1X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaFrontA1X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaFrontA1Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaFrontA1Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaFrontA2X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaFrontA2X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaFrontA2Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaFrontA2Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaFrontA3X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaFrontA3X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaFrontA3Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaFrontA3Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaFrontA4X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaFrontA4X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaFrontA4Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaFrontA4Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaFrontHysA1X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaFrontHysA1X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaFrontHysA1Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaFrontHysA1Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaFrontHysA2X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaFrontHysA2X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaFrontHysA2Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaFrontHysA2Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaFrontHysA3X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaFrontHysA3X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaFrontHysA3Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaFrontHysA3Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaFrontHysA4X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaFrontHysA4X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaFrontHysA4Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaFrontHysA4Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRearA1X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRearA1X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRearA1Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRearA1Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRearA2X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRearA2X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRearA2Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRearA2Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRearA3X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRearA3X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRearA3Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRearA3Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRearA4X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRearA4X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRearA4Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRearA4Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRearHysA1X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRearHysA1X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRearHysA1Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRearHysA1Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRearHysA2X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRearHysA2X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRearHysA2Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRearHysA2Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRearHysA3X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRearHysA3X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRearHysA3Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRearHysA3Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRearHysA4X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRearHysA4X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRearHysA4Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRearHysA4Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaLeftA1X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaLeftA1X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaLeftA1Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaLeftA1Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaLeftA2X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaLeftA2X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaLeftA2Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaLeftA2Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaLeftA3X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaLeftA3X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaLeftA3Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaLeftA3Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaLeftA4X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaLeftA4X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaLeftA4Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaLeftA4Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaLeftHysA1X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaLeftHysA1X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaLeftHysA1Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaLeftHysA1Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaLeftHysA2X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaLeftHysA2X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaLeftHysA2Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaLeftHysA2Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaLeftHysA3X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaLeftHysA3X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaLeftHysA3Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaLeftHysA3Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaLeftHysA4X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaLeftHysA4X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaLeftHysA4Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaLeftHysA4Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRightA1X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRightA1X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRightA1Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRightA1Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRightA2X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRightA2X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRightA2Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRightA2Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRightA3X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRightA3X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRightA3Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRightA3Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRightA4X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRightA4X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRightA4Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRightA4Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRightHysA1X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRightHysA1X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRightHysA1Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRightHysA1Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRightHysA2X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRightHysA2X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRightHysA2Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRightHysA2Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRightHysA3X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRightHysA3X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRightHysA3Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRightHysA3Y_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRightHysA4X_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRightHysA4X_psi16)
#define g_parGetParaAP_PDC_View_Prm_DispAreaRightHysA4Y_psi16 (((gType_parAP_PDC_View_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_View_Prm_uix]))->DispAreaRightHysA4Y_psi16)
#define g_parGetParaAP_PDC_Offset_Std_OffsetFront_pui16 (((gType_parAP_PDC_Offset_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Offset_Std_uix]))->OffsetFront_pui16)
#define g_parGetParaAP_PDC_Offset_Std_OffsetRear_pui16 (((gType_parAP_PDC_Offset_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Offset_Std_uix]))->OffsetRear_pui16)
#define g_parGetParaAP_PDC_Offset_Std_TrailerOffset_pui16 (((gType_parAP_PDC_Offset_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Offset_Std_uix]))->TrailerOffset_pui16)
#define g_parGetParaAP_PDC_Offset_Lat_OffsetFrontDir_pui16 (((gType_parAP_PDC_Offset_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Offset_Lat_uix]))->OffsetFrontDir_pui16)
#define g_parGetParaAP_PDC_Offset_Lat_OffsetFrontZone_pui16 (((gType_parAP_PDC_Offset_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Offset_Lat_uix]))->OffsetFrontZone_pui16)
#define g_parGetParaAP_PDC_Offset_Lat_OffsetRearDir_pui16 (((gType_parAP_PDC_Offset_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Offset_Lat_uix]))->OffsetRearDir_pui16)
#define g_parGetParaAP_PDC_Offset_Lat_OffsetRearZone_pui16 (((gType_parAP_PDC_Offset_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Offset_Lat_uix]))->OffsetRearZone_pui16)
#define g_parGetParaAP_PDC_SC_DT_Std_DTFrontEnabled_bl (((gType_parAP_PDC_SC_DT_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Std_uix]))->DTFrontEnabled_bl)
#define g_parGetParaAP_PDC_SC_DT_Std_DTRearEnabled_bl (((gType_parAP_PDC_SC_DT_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Std_uix]))->DTRearEnabled_bl)
#define g_parGetParaAP_PDC_SC_DT_Std_DTFrontDeAMin_ui16 (((gType_parAP_PDC_SC_DT_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Std_uix]))->DTFrontDeAMin_ui16)
#define g_parGetParaAP_PDC_SC_DT_Std_DTFrontDeAMax_ui16 (((gType_parAP_PDC_SC_DT_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Std_uix]))->DTFrontDeAMax_ui16)
#define g_parGetParaAP_PDC_SC_DT_Std_DTFrontDe25Lock_ui16 (((gType_parAP_PDC_SC_DT_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Std_uix]))->DTFrontDe25Lock_ui16)
#define g_parGetParaAP_PDC_SC_DT_Std_DTFrontDeB_ui16 (((gType_parAP_PDC_SC_DT_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Std_uix]))->DTFrontDeB_ui16)
#define g_parGetParaAP_PDC_SC_DT_Std_DTFrontCeLR_ui16 (((gType_parAP_PDC_SC_DT_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Std_uix]))->DTFrontCeLR_ui16)
#define g_parGetParaAP_PDC_SC_DT_Std_DTFrontCeMin_ui16 (((gType_parAP_PDC_SC_DT_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Std_uix]))->DTFrontCeMin_ui16)
#define g_parGetParaAP_PDC_SC_DT_Std_DTFrontCeMax_ui16 (((gType_parAP_PDC_SC_DT_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Std_uix]))->DTFrontCeMax_ui16)
#define g_parGetParaAP_PDC_SC_DT_Std_DTRearDeAMin_ui16 (((gType_parAP_PDC_SC_DT_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Std_uix]))->DTRearDeAMin_ui16)
#define g_parGetParaAP_PDC_SC_DT_Std_DTRearDeAMax_ui16 (((gType_parAP_PDC_SC_DT_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Std_uix]))->DTRearDeAMax_ui16)
#define g_parGetParaAP_PDC_SC_DT_Std_DTRearDe25Lock_ui16 (((gType_parAP_PDC_SC_DT_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Std_uix]))->DTRearDe25Lock_ui16)
#define g_parGetParaAP_PDC_SC_DT_Std_DTRearDeB_ui16 (((gType_parAP_PDC_SC_DT_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Std_uix]))->DTRearDeB_ui16)
#define g_parGetParaAP_PDC_SC_DT_Std_DTRearCeLR_ui16 (((gType_parAP_PDC_SC_DT_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Std_uix]))->DTRearCeLR_ui16)
#define g_parGetParaAP_PDC_SC_DT_Std_DTRearCeMin_ui16 (((gType_parAP_PDC_SC_DT_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Std_uix]))->DTRearCeMin_ui16)
#define g_parGetParaAP_PDC_SC_DT_Std_DTRearCeMax_ui16 (((gType_parAP_PDC_SC_DT_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Std_uix]))->DTRearCeMax_ui16)
#define g_parGetParaAP_PDC_SC_DT_Lat_DTFrontEnabled_ui8 (((gType_parAP_PDC_SC_DT_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Lat_uix]))->DTFrontEnabled_ui8)
#define g_parGetParaAP_PDC_SC_DT_Lat_DTRearEnabled_ui8 (((gType_parAP_PDC_SC_DT_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Lat_uix]))->DTRearEnabled_ui8)
#define g_parGetParaAP_PDC_SC_DT_Lat_DTFrontActivateDistNarrow_pui16 (((gType_parAP_PDC_SC_DT_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Lat_uix]))->DTFrontActivateDistNarrow_pui16)
#define g_parGetParaAP_PDC_SC_DT_Lat_DTFrontActivateDistCEObj_pui16 (((gType_parAP_PDC_SC_DT_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Lat_uix]))->DTFrontActivateDistCEObj_pui16)
#define g_parGetParaAP_PDC_SC_DT_Lat_DTFrontActivateDistDEObj_pui16 (((gType_parAP_PDC_SC_DT_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Lat_uix]))->DTFrontActivateDistDEObj_pui16)
#define g_parGetParaAP_PDC_SC_DT_Lat_DTRearActivateDistNarrow_pui16 (((gType_parAP_PDC_SC_DT_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Lat_uix]))->DTRearActivateDistNarrow_pui16)
#define g_parGetParaAP_PDC_SC_DT_Lat_DTRearActivateDistCEObj_pui16 (((gType_parAP_PDC_SC_DT_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Lat_uix]))->DTRearActivateDistCEObj_pui16)
#define g_parGetParaAP_PDC_SC_DT_Lat_DTRearActivateDistDEObj_pui16 (((gType_parAP_PDC_SC_DT_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Lat_uix]))->DTRearActivateDistDEObj_pui16)
#define g_parGetParaAP_PDC_SC_DT_Lat_DTFrontDeactivateDistNarrow_pui16 (((gType_parAP_PDC_SC_DT_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Lat_uix]))->DTFrontDeactivateDistNarrow_pui16)
#define g_parGetParaAP_PDC_SC_DT_Lat_DTFrontDeactivateDistCEObj_pui16 (((gType_parAP_PDC_SC_DT_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Lat_uix]))->DTFrontDeactivateDistCEObj_pui16)
#define g_parGetParaAP_PDC_SC_DT_Lat_DTFrontDeactivateDistDEObj_pui16 (((gType_parAP_PDC_SC_DT_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Lat_uix]))->DTFrontDeactivateDistDEObj_pui16)
#define g_parGetParaAP_PDC_SC_DT_Lat_DTRearDeactivateDistNarrow_pui16 (((gType_parAP_PDC_SC_DT_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Lat_uix]))->DTRearDeactivateDistNarrow_pui16)
#define g_parGetParaAP_PDC_SC_DT_Lat_DTRearDeactivateDistCEObj_pui16 (((gType_parAP_PDC_SC_DT_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Lat_uix]))->DTRearDeactivateDistCEObj_pui16)
#define g_parGetParaAP_PDC_SC_DT_Lat_DTRearDeactivateDistDEObj_pui16 (((gType_parAP_PDC_SC_DT_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DT_Lat_uix]))->DTRearDeactivateDistDEObj_pui16)
#define g_parGetParaAP_PDC_SC_DU_Lat_DUFrontEnabled_ui8 (((gType_parAP_PDC_SC_DU_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Lat_uix]))->DUFrontEnabled_ui8)
#define g_parGetParaAP_PDC_SC_DU_Lat_DURearEnabled_ui8 (((gType_parAP_PDC_SC_DU_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Lat_uix]))->DURearEnabled_ui8)
#define g_parGetParaAP_PDC_SC_DU_Lat_DUFrontLatzoneMin_pui8 (((gType_parAP_PDC_SC_DU_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Lat_uix]))->DUFrontLatzoneMin_pui8)
#define g_parGetParaAP_PDC_SC_DU_Lat_DURearLatzoneMin_pui8 (((gType_parAP_PDC_SC_DU_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Lat_uix]))->DURearLatzoneMin_pui8)
#define g_parGetParaAP_PDC_SC_DU_Lat_DUFrontLatzoneMax_pui8 (((gType_parAP_PDC_SC_DU_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Lat_uix]))->DUFrontLatzoneMax_pui8)
#define g_parGetParaAP_PDC_SC_DU_Lat_DURearLatzoneMax_pui8 (((gType_parAP_PDC_SC_DU_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Lat_uix]))->DURearLatzoneMax_pui8)
#define g_parGetParaAP_PDC_SC_DU_Lat_DUFrontDistTeachMax_pui16 (((gType_parAP_PDC_SC_DU_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Lat_uix]))->DUFrontDistTeachMax_pui16)
#define g_parGetParaAP_PDC_SC_DU_Lat_DURearDistTeachMax_pui16 (((gType_parAP_PDC_SC_DU_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Lat_uix]))->DURearDistTeachMax_pui16)
#define g_parGetParaAP_PDC_SC_DU_Lat_DUFrontDistDeltaMinMeasured_ui16 (((gType_parAP_PDC_SC_DU_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Lat_uix]))->DUFrontDistDeltaMinMeasured_ui16)
#define g_parGetParaAP_PDC_SC_DU_Lat_DURearDistDeltaMinMeasured_ui16 (((gType_parAP_PDC_SC_DU_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Lat_uix]))->DURearDistDeltaMinMeasured_ui16)
#define g_parGetParaAP_PDC_SC_DU_Lat_DUFrontDistCriticalMax_pui16 (((gType_parAP_PDC_SC_DU_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Lat_uix]))->DUFrontDistCriticalMax_pui16)
#define g_parGetParaAP_PDC_SC_DU_Lat_DURearDistCriticalMax_pui16 (((gType_parAP_PDC_SC_DU_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Lat_uix]))->DURearDistCriticalMax_pui16)
#define g_parGetParaAP_PDC_SC_DU_Lat_DUFrontDistDeltaObjLost_ui16 (((gType_parAP_PDC_SC_DU_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Lat_uix]))->DUFrontDistDeltaObjLost_ui16)
#define g_parGetParaAP_PDC_SC_DU_Lat_DURearDistDeltaObjLost_ui16 (((gType_parAP_PDC_SC_DU_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Lat_uix]))->DURearDistDeltaObjLost_ui16)
#define g_parGetParaAP_PDC_SC_DU_Lat_DUFrontActiveTime_ui16 (((gType_parAP_PDC_SC_DU_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Lat_uix]))->DUFrontActiveTime_ui16)
#define g_parGetParaAP_PDC_SC_DU_Lat_DURearActiveTime_ui16 (((gType_parAP_PDC_SC_DU_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Lat_uix]))->DURearActiveTime_ui16)
#define g_parGetParaAP_PDC_SC_DU_Prm_DUFrontEnabled_bl (((gType_parAP_PDC_SC_DU_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Prm_uix]))->DUFrontEnabled_bl)
#define g_parGetParaAP_PDC_SC_DU_Prm_DURearEnabled_bl (((gType_parAP_PDC_SC_DU_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Prm_uix]))->DURearEnabled_bl)
#define g_parGetParaAP_PDC_SC_DU_Prm_DUFrontDistTeachMax_ui16 (((gType_parAP_PDC_SC_DU_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Prm_uix]))->DUFrontDistTeachMax_ui16)
#define g_parGetParaAP_PDC_SC_DU_Prm_DURearDistTeachMax_ui16 (((gType_parAP_PDC_SC_DU_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Prm_uix]))->DURearDistTeachMax_ui16)
#define g_parGetParaAP_PDC_SC_DU_Prm_DUFrontDistDeltaMinApproach_ui16 (((gType_parAP_PDC_SC_DU_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Prm_uix]))->DUFrontDistDeltaMinApproach_ui16)
#define g_parGetParaAP_PDC_SC_DU_Prm_DURearDistDeltaMinApproach_ui16 (((gType_parAP_PDC_SC_DU_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Prm_uix]))->DURearDistDeltaMinApproach_ui16)
#define g_parGetParaAP_PDC_SC_DU_Prm_DUFrontDistCriticalMax_ui16 (((gType_parAP_PDC_SC_DU_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Prm_uix]))->DUFrontDistCriticalMax_ui16)
#define g_parGetParaAP_PDC_SC_DU_Prm_DURearDistCriticalMax_ui16 (((gType_parAP_PDC_SC_DU_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Prm_uix]))->DURearDistCriticalMax_ui16)
#define g_parGetParaAP_PDC_SC_DU_Prm_DUFrontActiveTime_ui16 (((gType_parAP_PDC_SC_DU_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Prm_uix]))->DUFrontActiveTime_ui16)
#define g_parGetParaAP_PDC_SC_DU_Prm_DURearActiveTime_ui16 (((gType_parAP_PDC_SC_DU_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_DU_Prm_uix]))->DURearActiveTime_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OEFrontEnabled_bl (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OEFrontEnabled_bl)
#define g_parGetParaAP_PDC_SC_OE_Std_OERearEnabled_bl (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OERearEnabled_bl)
#define g_parGetParaAP_PDC_SC_OE_Std_OEFrontEnabledInner_bl (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OEFrontEnabledInner_bl)
#define g_parGetParaAP_PDC_SC_OE_Std_OERearEnabledInner_bl (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OERearEnabledInner_bl)
#define g_parGetParaAP_PDC_SC_OE_Std_OEFrontDeA_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OEFrontDeA_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OEFrontDeB_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OEFrontDeB_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OEFrontCeA_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OEFrontCeA_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OEFrontCeB_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OEFrontCeB_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OEFrontOffset_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OEFrontOffset_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OEFrontDeAInner_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OEFrontDeAInner_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OEFrontCeAInner_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OEFrontCeAInner_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OEFrontCeBInner_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OEFrontCeBInner_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OEFrontOffsetInner_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OEFrontOffsetInner_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OERearDeA_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OERearDeA_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OERearDeB_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OERearDeB_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OERearCeA_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OERearCeA_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OERearCeB_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OERearCeB_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OERearOffset_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OERearOffset_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OERearDeAInner_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OERearDeAInner_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OERearCeAInner_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OERearCeAInner_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OERearCeBInner_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OERearCeBInner_ui16)
#define g_parGetParaAP_PDC_SC_OE_Std_OERearOffsetInner_ui16 (((gType_parAP_PDC_SC_OE_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Std_uix]))->OERearOffsetInner_ui16)
#define g_parGetParaAP_PDC_SC_OE_Lat_OEFrontEnabled_bl (((gType_parAP_PDC_SC_OE_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Lat_uix]))->OEFrontEnabled_bl)
#define g_parGetParaAP_PDC_SC_OE_Lat_OERearEnabled_bl (((gType_parAP_PDC_SC_OE_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Lat_uix]))->OERearEnabled_bl)
#define g_parGetParaAP_PDC_SC_OE_Lat_OEFrontGroupLeft_ui8 (((gType_parAP_PDC_SC_OE_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Lat_uix]))->OEFrontGroupLeft_ui8)
#define g_parGetParaAP_PDC_SC_OE_Lat_OEFrontGroupRight_ui8 (((gType_parAP_PDC_SC_OE_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Lat_uix]))->OEFrontGroupRight_ui8)
#define g_parGetParaAP_PDC_SC_OE_Lat_OERearGroupLeft_ui8 (((gType_parAP_PDC_SC_OE_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Lat_uix]))->OERearGroupLeft_ui8)
#define g_parGetParaAP_PDC_SC_OE_Lat_OERearGroupRight_ui8 (((gType_parAP_PDC_SC_OE_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Lat_uix]))->OERearGroupRight_ui8)
#define g_parGetParaAP_PDC_SC_OE_Lat_OEFrontMaxDist_ui16 (((gType_parAP_PDC_SC_OE_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Lat_uix]))->OEFrontMaxDist_ui16)
#define g_parGetParaAP_PDC_SC_OE_Lat_OEFrontMinDist_ui16 (((gType_parAP_PDC_SC_OE_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Lat_uix]))->OEFrontMinDist_ui16)
#define g_parGetParaAP_PDC_SC_OE_Lat_OERearMaxDist_ui16 (((gType_parAP_PDC_SC_OE_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Lat_uix]))->OERearMaxDist_ui16)
#define g_parGetParaAP_PDC_SC_OE_Lat_OERearMinDist_ui16 (((gType_parAP_PDC_SC_OE_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_OE_Lat_uix]))->OERearMinDist_ui16)
#define g_parGetParaAP_PDC_SC_SWD_Std_SWDFrontEnabled_bl (((gType_parAP_PDC_SC_SWD_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Std_uix]))->SWDFrontEnabled_bl)
#define g_parGetParaAP_PDC_SC_SWD_Std_SWDRearEnabled_bl (((gType_parAP_PDC_SC_SWD_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Std_uix]))->SWDRearEnabled_bl)
#define g_parGetParaAP_PDC_SC_SWD_Std_SWDFrontGroupLeft_ui8 (((gType_parAP_PDC_SC_SWD_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Std_uix]))->SWDFrontGroupLeft_ui8)
#define g_parGetParaAP_PDC_SC_SWD_Std_SWDFrontGroupRight_ui8 (((gType_parAP_PDC_SC_SWD_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Std_uix]))->SWDFrontGroupRight_ui8)
#define g_parGetParaAP_PDC_SC_SWD_Std_SWDRearGroupLeft_ui8 (((gType_parAP_PDC_SC_SWD_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Std_uix]))->SWDRearGroupLeft_ui8)
#define g_parGetParaAP_PDC_SC_SWD_Std_SWDRearGroupRight_ui8 (((gType_parAP_PDC_SC_SWD_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Std_uix]))->SWDRearGroupRight_ui8)
#define g_parGetParaAP_PDC_SC_SWD_Std_SWDFrontDistCutOff_ui16 (((gType_parAP_PDC_SC_SWD_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Std_uix]))->SWDFrontDistCutOff_ui16)
#define g_parGetParaAP_PDC_SC_SWD_Std_SWDRearDistCutOff_ui16 (((gType_parAP_PDC_SC_SWD_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Std_uix]))->SWDRearDistCutOff_ui16)
#define g_parGetParaAP_PDC_SC_SWD_Std_SWDDistHyst_ui16 (((gType_parAP_PDC_SC_SWD_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Std_uix]))->SWDDistHyst_ui16)
#define g_parGetParaAP_PDC_SC_SWD_Std_SWDFrontSpeedCutOff_ui8 (((gType_parAP_PDC_SC_SWD_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Std_uix]))->SWDFrontSpeedCutOff_ui8)
#define g_parGetParaAP_PDC_SC_SWD_Std_SWDRearSpeedCutOff_ui8 (((gType_parAP_PDC_SC_SWD_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Std_uix]))->SWDRearSpeedCutOff_ui8)
#define g_parGetParaAP_PDC_SC_SWD_Std_SWDDelayTime_ui8 (((gType_parAP_PDC_SC_SWD_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Std_uix]))->SWDDelayTime_ui8)
#define g_parGetParaAP_PDC_SC_SWD_Std_SWDFrontDistApproach_ui16 (((gType_parAP_PDC_SC_SWD_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Std_uix]))->SWDFrontDistApproach_ui16)
#define g_parGetParaAP_PDC_SC_SWD_Std_SWDRearDistApproach_ui16 (((gType_parAP_PDC_SC_SWD_Std_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Std_uix]))->SWDRearDistApproach_ui16)
#define g_parGetParaAP_PDC_SC_SWD_Lat_SWDFrontEnabled_bl (((gType_parAP_PDC_SC_SWD_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Lat_uix]))->SWDFrontEnabled_bl)
#define g_parGetParaAP_PDC_SC_SWD_Lat_SWDRearEnabled_bl (((gType_parAP_PDC_SC_SWD_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Lat_uix]))->SWDRearEnabled_bl)
#define g_parGetParaAP_PDC_SC_SWD_Lat_SWDFrontGroupLeft_ui8 (((gType_parAP_PDC_SC_SWD_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Lat_uix]))->SWDFrontGroupLeft_ui8)
#define g_parGetParaAP_PDC_SC_SWD_Lat_SWDFrontGroupRight_ui8 (((gType_parAP_PDC_SC_SWD_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Lat_uix]))->SWDFrontGroupRight_ui8)
#define g_parGetParaAP_PDC_SC_SWD_Lat_SWDRearGroupLeft_ui8 (((gType_parAP_PDC_SC_SWD_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Lat_uix]))->SWDRearGroupLeft_ui8)
#define g_parGetParaAP_PDC_SC_SWD_Lat_SWDRearGroupRight_ui8 (((gType_parAP_PDC_SC_SWD_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Lat_uix]))->SWDRearGroupRight_ui8)
#define g_parGetParaAP_PDC_SC_SWD_Lat_SWDDistCutOff_ui16 (((gType_parAP_PDC_SC_SWD_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Lat_uix]))->SWDDistCutOff_ui16)
#define g_parGetParaAP_PDC_SC_SWD_Lat_SWDDistCutOffHyst_ui16 (((gType_parAP_PDC_SC_SWD_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Lat_uix]))->SWDDistCutOffHyst_ui16)
#define g_parGetParaAP_PDC_SC_SWD_Lat_SWDSpeedCutOff_ui8 (((gType_parAP_PDC_SC_SWD_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Lat_uix]))->SWDSpeedCutOff_ui8)
#define g_parGetParaAP_PDC_SC_SWD_Lat_SWDSpeedCutOffHyst_ui8 (((gType_parAP_PDC_SC_SWD_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Lat_uix]))->SWDSpeedCutOffHyst_ui8)
#define g_parGetParaAP_PDC_SC_SWD_Lat_SWDDriveDist_ui16 (((gType_parAP_PDC_SC_SWD_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Lat_uix]))->SWDDriveDist_ui16)
#define g_parGetParaAP_PDC_SC_SWD_Lat_SWDMaxAppDepSpeed_ui16 (((gType_parAP_PDC_SC_SWD_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Lat_uix]))->SWDMaxAppDepSpeed_ui16)
#define g_parGetParaAP_PDC_SC_SWD_Lat_SWDMaxAppDepSpeedHyst_ui16 (((gType_parAP_PDC_SC_SWD_Lat_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Lat_uix]))->SWDMaxAppDepSpeedHyst_ui16)
#define g_parGetParaAP_PDC_SC_SWD_Prm_SWDFrontEnabled_bl (((gType_parAP_PDC_SC_SWD_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Prm_uix]))->SWDFrontEnabled_bl)
#define g_parGetParaAP_PDC_SC_SWD_Prm_SWDRearEnabled_bl (((gType_parAP_PDC_SC_SWD_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Prm_uix]))->SWDRearEnabled_bl)
#define g_parGetParaAP_PDC_SC_SWD_Prm_SWDDistCutOff_ui16 (((gType_parAP_PDC_SC_SWD_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Prm_uix]))->SWDDistCutOff_ui16)
#define g_parGetParaAP_PDC_SC_SWD_Prm_SWDDistCutOffHyst_ui16 (((gType_parAP_PDC_SC_SWD_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Prm_uix]))->SWDDistCutOffHyst_ui16)
#define g_parGetParaAP_PDC_SC_SWD_Prm_SWDSpeedCutOff_ui8 (((gType_parAP_PDC_SC_SWD_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Prm_uix]))->SWDSpeedCutOff_ui8)
#define g_parGetParaAP_PDC_SC_SWD_Prm_SWDSpeedCutOffHyst_ui8 (((gType_parAP_PDC_SC_SWD_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Prm_uix]))->SWDSpeedCutOffHyst_ui8)
#define g_parGetParaAP_PDC_SC_SWD_Prm_SWDDriveDist_ui16 (((gType_parAP_PDC_SC_SWD_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Prm_uix]))->SWDDriveDist_ui16)
#define g_parGetParaAP_PDC_SC_SWD_Prm_SWDDistDeltaMax_ui16 (((gType_parAP_PDC_SC_SWD_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Prm_uix]))->SWDDistDeltaMax_ui16)
#define g_parGetParaAP_PDC_SC_SWD_Prm_SWDDistDeltaMaxHyst_ui16 (((gType_parAP_PDC_SC_SWD_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_SWD_Prm_uix]))->SWDDistDeltaMaxHyst_ui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLWallLengthMax_ui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLWallLengthMax_ui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLGroupCenterDistStart_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLGroupCenterDistStart_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLGroupCenterDistStop_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLGroupCenterDistStop_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLGroupCenterDeDistStart_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLGroupCenterDeDistStart_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLGroupCenterDeDistStop_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLGroupCenterDeDistStop_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLGroupCenterCeDistStart_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLGroupCenterCeDistStart_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLGroupCenterCeDistStop_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLGroupCenterCeDistStop_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLGroupSideDistStart_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLGroupSideDistStart_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLGroupSideDistStop_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLGroupSideDistStop_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLGroupSideDeDistStart_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLGroupSideDeDistStart_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLGroupSideDeDistStop_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLGroupSideDeDistStop_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLGroupSideCeDistStart_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLGroupSideCeDistStart_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLGroupSideCeDistStop_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLGroupSideCeDistStop_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLEdgeDistStart_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLEdgeDistStart_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLEdgeDistStop_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLEdgeDistStop_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLEdgeDeDistStart_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLEdgeDeDistStart_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLEdgeDeDistStop_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLEdgeDeDistStop_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLEdgeCeDistStart_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLEdgeCeDistStart_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLEdgeCeDistStop_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLEdgeCeDistStop_pui16)
#define g_parGetParaAP_PDC_SC_WBL_Prm_WBLEdgeWidth_pui16 (((gType_parAP_PDC_SC_WBL_Prm_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_SC_WBL_Prm_uix]))->WBLEdgeWidth_pui16)
#define g_parGetParaAP_PDC_AO_AOMaxSamples_ui8 (((gType_parAP_PDC_AO_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_AO_uix]))->AOMaxSamples_ui8)
#define g_parGetParaAP_PDC_AO_AOMinNumDetectSamples_ui8 (((gType_parAP_PDC_AO_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_AO_uix]))->AOMinNumDetectSamples_ui8)
#define g_parGetParaAP_PDC_AO_AOMinNumRegularEchos_ui8 (((gType_parAP_PDC_AO_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_AO_uix]))->AOMinNumRegularEchos_ui8)
#define g_parGetParaAP_PDC_AO_AODistMax_ui16 (((gType_parAP_PDC_AO_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_AO_uix]))->AODistMax_ui16)
#define g_parGetParaAP_PDC_Danger_DangerDelayMS_ui16 (((gType_parAP_PDC_Danger_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Danger_uix]))->DangerDelayMS_ui16)
#define g_parGetParaAP_PDC_Danger_DangerDecelMMS2_ui16 (((gType_parAP_PDC_Danger_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Danger_uix]))->DangerDecelMMS2_ui16)
#define g_parGetParaAP_PDC_Danger_DangerStopDist_ui16 (((gType_parAP_PDC_Danger_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Danger_uix]))->DangerStopDist_ui16)
#define g_parGetParaAP_PDC_Danger_DangerActMinDist_ui16 (((gType_parAP_PDC_Danger_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Danger_uix]))->DangerActMinDist_ui16)
#define g_parGetParaAP_PDC_Danger_DangerDeactDist_ui16 (((gType_parAP_PDC_Danger_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Danger_uix]))->DangerDeactDist_ui16)
#define g_parGetParaAP_PDC_Danger_DangerActMinSpeed_ui16 (((gType_parAP_PDC_Danger_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Danger_uix]))->DangerActMinSpeed_ui16)
#define g_parGetParaAP_PDC_Danger_DangerDeactSpeed_ui16 (((gType_parAP_PDC_Danger_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_AP_PDC_Danger_uix]))->DangerDeactSpeed_ui16)

/*--------------------------------------------------------------------------*/
/*- PSU                                                                    -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaPSU_CPSC_LengthShortenMax_si16 (((gType_parPSU_CPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_CPSC_uix]))->LengthShortenMax_si16)
#define g_parGetParaPSU_CPSC_LengthExtendMax_si16 (((gType_parPSU_CPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_CPSC_uix]))->LengthExtendMax_si16)
#define g_parGetParaPSU_CPSC_DepthMin_si16 (((gType_parPSU_CPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_CPSC_uix]))->DepthMin_si16)
#define g_parGetParaPSU_CPSC_DepthMax_si16 (((gType_parPSU_CPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_CPSC_uix]))->DepthMax_si16)
#define g_parGetParaPSU_CPSC_DepthShortenMax_si16 (((gType_parPSU_CPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_CPSC_uix]))->DepthShortenMax_si16)
#define g_parGetParaPSU_CPSC_DepthExtendMax_si16 (((gType_parPSU_CPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_CPSC_uix]))->DepthExtendMax_si16)
#define g_parGetParaPSU_PSC_LengthShortenMax_si16 (((gType_parPSU_PSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_PSC_uix]))->LengthShortenMax_si16)
#define g_parGetParaPSU_PSC_LengthExtendMax_si16 (((gType_parPSU_PSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_PSC_uix]))->LengthExtendMax_si16)
#define g_parGetParaPSU_PSC_DepthMin_si16 (((gType_parPSU_PSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_PSC_uix]))->DepthMin_si16)
#define g_parGetParaPSU_PSC_DepthMax_si16 (((gType_parPSU_PSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_PSC_uix]))->DepthMax_si16)
#define g_parGetParaPSU_POC_FoVRangeFront_ui16 (((gType_parPSU_POC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_POC_uix]))->FoVRangeFront_ui16)
#define g_parGetParaPSU_POC_FoVRangeRear_ui16 (((gType_parPSU_POC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_POC_uix]))->FoVRangeRear_ui16)
#define g_parGetParaPSU_CPX_BackIn_MinYaw_PLOutput_ui16 (((gType_parPSU_CPX_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_CPX_uix]))->BackIn_MinYaw_PLOutput_ui16)
#define g_parGetParaPSU_CPX_BackIn_MaxLengthPS_ui16 (((gType_parPSU_CPX_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_CPX_uix]))->BackIn_MaxLengthPS_ui16)
#define g_parGetParaPSU_CPX_FrontIn_MinYaw_PLOutput_ui16 (((gType_parPSU_CPX_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_CPX_uix]))->FrontIn_MinYaw_PLOutput_ui16)
#define g_parGetParaPSU_CPX_FrontIn_MaxLengthPS_ui16 (((gType_parPSU_CPX_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_CPX_uix]))->FrontIn_MaxLengthPS_ui16)
#define g_parGetParaPSU_Object_ExistProbMin_ui8 (((gType_parPSU_Object_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_Object_uix]))->ExistProbMin_ui8)
#define g_parGetParaPSU_Object_DECEExistProbMin_ui8 (((gType_parPSU_Object_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_Object_uix]))->DECEExistProbMin_ui8)
#define g_parGetParaPSU_Object_ErrorMin_ui16 (((gType_parPSU_Object_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_Object_uix]))->ErrorMin_ui16)
#define g_parGetParaPSU_Reference_WeightMin_ui16 (((gType_parPSU_Reference_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_Reference_uix]))->WeightMin_ui16)
#define g_parGetParaPSU_Reference_WeightMax_ui16 (((gType_parPSU_Reference_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PSU_Reference_uix]))->WeightMax_ui16)

/*--------------------------------------------------------------------------*/
/*- APG                                                                    -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaAPG_XPG_Vehicle_HalfWidth_ui16 (((gType_parAPG_XPG_Vehicle_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Vehicle_uix]))->HalfWidth_ui16)
#define g_parGetParaAPG_XPG_Vehicle_Wheelbase_ui16 (((gType_parAPG_XPG_Vehicle_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Vehicle_uix]))->Wheelbase_ui16)
#define g_parGetParaAPG_XPG_Vehicle_LengthFront_ui16 (((gType_parAPG_XPG_Vehicle_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Vehicle_uix]))->LengthFront_ui16)
#define g_parGetParaAPG_XPG_Vehicle_LengthRear_ui16 (((gType_parAPG_XPG_Vehicle_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Vehicle_uix]))->LengthRear_ui16)
#define g_parGetParaAPG_XPG_Vehicle_CornerRadiusFront_ui16 (((gType_parAPG_XPG_Vehicle_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Vehicle_uix]))->CornerRadiusFront_ui16)
#define g_parGetParaAPG_XPG_Vehicle_CornerRadiusRear_ui16 (((gType_parAPG_XPG_Vehicle_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Vehicle_uix]))->CornerRadiusRear_ui16)
#define g_parGetParaAPG_XPG_Vehicle_MaxSWA_si16 (((gType_parAPG_XPG_Vehicle_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Vehicle_uix]))->MaxSWA_si16)
#define g_parGetParaAPG_XPG_Vehicle_SWAVeloMax_si16 (((gType_parAPG_XPG_Vehicle_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Vehicle_uix]))->SWAVeloMax_si16)
#define g_parGetParaAPG_XPG_Vehicle_RWheel_ui16 (((gType_parAPG_XPG_Vehicle_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Vehicle_uix]))->RWheel_ui16)
#define g_parGetParaAPG_XPG_SafetyDistParallel_MinLowLatRef_si16 (((gType_parAPG_XPG_SafetyDistParallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistParallel_uix]))->MinLowLatRef_si16)
#define g_parGetParaAPG_XPG_SafetyDistParallel_TargetLowLatRef_si16 (((gType_parAPG_XPG_SafetyDistParallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistParallel_uix]))->TargetLowLatRef_si16)
#define g_parGetParaAPG_XPG_SafetyDistParallel_MinHighLatRef_si16 (((gType_parAPG_XPG_SafetyDistParallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistParallel_uix]))->MinHighLatRef_si16)
#define g_parGetParaAPG_XPG_SafetyDistParallel_TargetHighLatRef_si16 (((gType_parAPG_XPG_SafetyDistParallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistParallel_uix]))->TargetHighLatRef_si16)
#define g_parGetParaAPG_XPG_SafetyDistParallel_MinVirtLatRef_si16 (((gType_parAPG_XPG_SafetyDistParallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistParallel_uix]))->MinVirtLatRef_si16)
#define g_parGetParaAPG_XPG_SafetyDistParallel_TargetVirtLatRef_si16 (((gType_parAPG_XPG_SafetyDistParallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistParallel_uix]))->TargetVirtLatRef_si16)
#define g_parGetParaAPG_XPG_SafetyDistParallel_Front_si16 (((gType_parAPG_XPG_SafetyDistParallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistParallel_uix]))->Front_si16)
#define g_parGetParaAPG_XPG_SafetyDistParallel_Rear_si16 (((gType_parAPG_XPG_SafetyDistParallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistParallel_uix]))->Rear_si16)
#define g_parGetParaAPG_XPG_SafetyDistParallel_Corner_si16 (((gType_parAPG_XPG_SafetyDistParallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistParallel_uix]))->Corner_si16)
#define g_parGetParaAPG_XPG_SafetyDistParallel_Side_si16 (((gType_parAPG_XPG_SafetyDistParallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistParallel_uix]))->Side_si16)
#define g_parGetParaAPG_XPG_SafetyDistParallel_OppositeSide_si16 (((gType_parAPG_XPG_SafetyDistParallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistParallel_uix]))->OppositeSide_si16)
#define g_parGetParaAPG_XPG_SafetyDistParallel_ParkingSide_si16 (((gType_parAPG_XPG_SafetyDistParallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistParallel_uix]))->ParkingSide_si16)
#define g_parGetParaAPG_XPG_SafetyDistParallel_EarlyPosOkLatDistExt_si16 (((gType_parAPG_XPG_SafetyDistParallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistParallel_uix]))->EarlyPosOkLatDistExt_si16)
#define g_parGetParaAPG_XPG_SafetyDistCross_MinTargetSideDist_si16 (((gType_parAPG_XPG_SafetyDistCross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistCross_uix]))->MinTargetSideDist_si16)
#define g_parGetParaAPG_XPG_SafetyDistCross_Side1stMove_si16 (((gType_parAPG_XPG_SafetyDistCross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistCross_uix]))->Side1stMove_si16)
#define g_parGetParaAPG_XPG_SafetyDistCross_SideNMove_si16 (((gType_parAPG_XPG_SafetyDistCross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistCross_uix]))->SideNMove_si16)
#define g_parGetParaAPG_XPG_SafetyDistCross_Object1_ui8 (((gType_parAPG_XPG_SafetyDistCross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistCross_uix]))->Object1_ui8)
#define g_parGetParaAPG_XPG_SafetyDistCross_OppositeSide_si16 (((gType_parAPG_XPG_SafetyDistCross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistCross_uix]))->OppositeSide_si16)
#define g_parGetParaAPG_XPG_SafetyDistCross_Corner1_si16 (((gType_parAPG_XPG_SafetyDistCross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistCross_uix]))->Corner1_si16)
#define g_parGetParaAPG_XPG_SafetyDistCross_Side_FI_si16 (((gType_parAPG_XPG_SafetyDistCross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_SafetyDistCross_uix]))->Side_FI_si16)
#define g_parGetParaAPG_XPG_Ctrl_TargetPosTolPsi_ui16 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->TargetPosTolPsi_ui16)
#define g_parGetParaAPG_XPG_Ctrl_TargetTolX_CPSC_ui8 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->TargetTolX_CPSC_ui8)
#define g_parGetParaAPG_XPG_Ctrl_TargetTolPsi_CPSC_ui16 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->TargetTolPsi_CPSC_ui16)
#define g_parGetParaAPG_XPG_Ctrl_VeloStandstill_ui16 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->VeloStandstill_ui16)
#define g_parGetParaAPG_XPG_Ctrl_VeloAbort_ui16 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->VeloAbort_ui16)
#define g_parGetParaAPG_XPG_Ctrl_VeloSlowDown_ui16 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->VeloSlowDown_ui16)
#define g_parGetParaAPG_XPG_Ctrl_VeloSlowDownHyst_ui16 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->VeloSlowDownHyst_ui16)
#define g_parGetParaAPG_XPG_Ctrl_MaxNumOfMovesPSC_ui8 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->MaxNumOfMovesPSC_ui8)
#define g_parGetParaAPG_XPG_Ctrl_MaxNumOfMovesCPSC_ui8 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->MaxNumOfMovesCPSC_ui8)
#define g_parGetParaAPG_XPG_Ctrl_MaxNumOfMovesCPSC_FI_ui8 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->MaxNumOfMovesCPSC_FI_ui8)
#define g_parGetParaAPG_XPG_Ctrl_MinLatDistForNMove_si16 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->MinLatDistForNMove_si16)
#define g_parGetParaAPG_XPG_Ctrl_StopVeloHyst_ui16 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->StopVeloHyst_ui16)
#define g_parGetParaAPG_XPG_Ctrl_ReactionTime_ui16 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->ReactionTime_ui16)
#define g_parGetParaAPG_XPG_Ctrl_Deceleration_ui16 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->Deceleration_ui16)
#define g_parGetParaAPG_XPG_Ctrl_TPR_Prediction_si8 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->TPR_Prediction_si8)
#define g_parGetParaAPG_XPG_Ctrl_MaxUncontrolledTravelDist_si16 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->MaxUncontrolledTravelDist_si16)
#define g_parGetParaAPG_XPG_Ctrl_MaxPosDev_ui16 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->MaxPosDev_ui16)
#define g_parGetParaAPG_XPG_Ctrl_MaxDrivenDistOnPASDegraded_ui16 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->MaxDrivenDistOnPASDegraded_ui16)
#define g_parGetParaAPG_XPG_Ctrl_DeactTimePathBlockedByObstacle_ui16 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->DeactTimePathBlockedByObstacle_ui16)
#define g_parGetParaAPG_XPG_Ctrl_MinYawAngleForEarlyStopOnInsufficientSpace_ui32 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->MinYawAngleForEarlyStopOnInsufficientSpace_ui32)
#define g_parGetParaAPG_XPG_Ctrl_BestGuessPSShiftInBlockadeSituation_ui8 (((gType_parAPG_XPG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Ctrl_uix]))->BestGuessPSShiftInBlockadeSituation_ui8)
#define g_parGetParaAPG_XPG_NMove_SWAForward_si16 (((gType_parAPG_XPG_NMove_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_NMove_uix]))->SWAForward_si16)
#define g_parGetParaAPG_XPG_NMove_SWABackwardMin_si16 (((gType_parAPG_XPG_NMove_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_NMove_uix]))->SWABackwardMin_si16)
#define g_parGetParaAPG_XPG_NMove_SWABackwardMax_si16 (((gType_parAPG_XPG_NMove_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_NMove_uix]))->SWABackwardMax_si16)
#define g_parGetParaAPG_XPG_NMove_StraightenClothoidVelo_si16 (((gType_parAPG_XPG_NMove_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_NMove_uix]))->StraightenClothoidVelo_si16)
#define g_parGetParaAPG_XPG_NMove_MinLengthOfLongitudinalAlignmentMove_si16 (((gType_parAPG_XPG_NMove_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_NMove_uix]))->MinLengthOfLongitudinalAlignmentMove_si16)
#define g_parGetParaAPG_XPG_NMove_MaxLengthOfLongitudinalAlignmentMove_ui8 (((gType_parAPG_XPG_NMove_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_NMove_uix]))->MaxLengthOfLongitudinalAlignmentMove_ui8)
#define g_parGetParaAPG_XPG_NMove_AlignmentDistFrontMax_si16 (((gType_parAPG_XPG_NMove_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_NMove_uix]))->AlignmentDistFrontMax_si16)
#define g_parGetParaAPG_XPG_NMove_MaxDist2RearRef_ui8 (((gType_parAPG_XPG_NMove_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_NMove_uix]))->MaxDist2RearRef_ui8)
#define g_parGetParaAPG_XPG_NMove_MaxDist2FrontRef_ui8 (((gType_parAPG_XPG_NMove_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_NMove_uix]))->MaxDist2FrontRef_ui8)
#define g_parGetParaAPG_XPG_Parkable_EndPsiMaxF12_si16 (((gType_parAPG_XPG_Parkable_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Parkable_uix]))->EndPsiMaxF12_si16)
#define g_parGetParaAPG_XPG_Parkable_VeloHyst_si16 (((gType_parAPG_XPG_Parkable_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Parkable_uix]))->VeloHyst_si16)
#define g_parGetParaAPG_XPG_Parkable_DistHyst_ui16 (((gType_parAPG_XPG_Parkable_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Parkable_uix]))->DistHyst_ui16)
#define g_parGetParaAPG_XPG_Parkable_ReactionTime_ui16 (((gType_parAPG_XPG_Parkable_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Parkable_uix]))->ReactionTime_ui16)
#define g_parGetParaAPG_XPG_Parkable_Deceleration_ui16 (((gType_parAPG_XPG_Parkable_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_XPG_Parkable_uix]))->Deceleration_ui16)
#define g_parGetParaAPG_APG_Pathplanning_Parallel_SWA3Min_si16 (((gType_parAPG_APG_Pathplanning_Parallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Parallel_uix]))->SWA3Min_si16)
#define g_parGetParaAPG_APG_Pathplanning_Parallel_SWA3Max_si16 (((gType_parAPG_APG_Pathplanning_Parallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Parallel_uix]))->SWA3Max_si16)
#define g_parGetParaAPG_APG_Pathplanning_Parallel_SWA6_si16 (((gType_parAPG_APG_Pathplanning_Parallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Parallel_uix]))->SWA6_si16)
#define g_parGetParaAPG_APG_Pathplanning_Parallel_Velo0_si16 (((gType_parAPG_APG_Pathplanning_Parallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Parallel_uix]))->Velo0_si16)
#define g_parGetParaAPG_APG_Pathplanning_Parallel_Velo2_si16 (((gType_parAPG_APG_Pathplanning_Parallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Parallel_uix]))->Velo2_si16)
#define g_parGetParaAPG_APG_Pathplanning_Parallel_Velo2Max_si16 (((gType_parAPG_APG_Pathplanning_Parallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Parallel_uix]))->Velo2Max_si16)
#define g_parGetParaAPG_APG_Pathplanning_Parallel_Velo4_si16 (((gType_parAPG_APG_Pathplanning_Parallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Parallel_uix]))->Velo4_si16)
#define g_parGetParaAPG_APG_Pathplanning_Parallel_Velo4Max_si16 (((gType_parAPG_APG_Pathplanning_Parallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Parallel_uix]))->Velo4Max_si16)
#define g_parGetParaAPG_APG_Pathplanning_Parallel_Velo5_si16 (((gType_parAPG_APG_Pathplanning_Parallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Parallel_uix]))->Velo5_si16)
#define g_parGetParaAPG_APG_Pathplanning_Parallel_Velo7_si16 (((gType_parAPG_APG_Pathplanning_Parallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Parallel_uix]))->Velo7_si16)
#define g_parGetParaAPG_APG_Pathplanning_Parallel_AddSafetyDistFrontMax_si16 (((gType_parAPG_APG_Pathplanning_Parallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Parallel_uix]))->AddSafetyDistFrontMax_si16)
#define g_parGetParaAPG_APG_Pathplanning_Parallel_SafetyDistFrontIncreaseRatioF10_si16 (((gType_parAPG_APG_Pathplanning_Parallel_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Parallel_uix]))->SafetyDistFrontIncreaseRatioF10_si16)
#define g_parGetParaAPG_APG_Steer_Ctrl_DevPosP_si16 (((gType_parAPG_APG_Steer_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Steer_Ctrl_uix]))->DevPosP_si16)
#define g_parGetParaAPG_APG_Steer_Ctrl_DevPosI_si16 (((gType_parAPG_APG_Steer_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Steer_Ctrl_uix]))->DevPosI_si16)
#define g_parGetParaAPG_APG_Steer_Ctrl_DevPosT_ui16 (((gType_parAPG_APG_Steer_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Steer_Ctrl_uix]))->DevPosT_ui16)
#define g_parGetParaAPG_APG_Steer_Ctrl_DevPsiP_si16 (((gType_parAPG_APG_Steer_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Steer_Ctrl_uix]))->DevPsiP_si16)
#define g_parGetParaAPG_APG_Steer_Ctrl_DevPsiI_si16 (((gType_parAPG_APG_Steer_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Steer_Ctrl_uix]))->DevPsiI_si16)
#define g_parGetParaAPG_APG_Steer_Ctrl_DevPsiT_ui16 (((gType_parAPG_APG_Steer_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Steer_Ctrl_uix]))->DevPsiT_ui16)
#define g_parGetParaAPG_APG_Steer_Ctrl_DevPosMax_si16 (((gType_parAPG_APG_Steer_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Steer_Ctrl_uix]))->DevPosMax_si16)
#define g_parGetParaAPG_APG_Steer_Ctrl_VeloMax_si16 (((gType_parAPG_APG_Steer_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Steer_Ctrl_uix]))->VeloMax_si16)
#define g_parGetParaAPG_APG_Steer_Ctrl_SWAFilterT_ui16 (((gType_parAPG_APG_Steer_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Steer_Ctrl_uix]))->SWAFilterT_ui16)
#define g_parGetParaAPG_APG_Steer_Ctrl_SWAFilterTStandstill_ui16 (((gType_parAPG_APG_Steer_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Steer_Ctrl_uix]))->SWAFilterTStandstill_ui16)
#define g_parGetParaAPG_APG_Steer_Ctrl_TargetPosFilterT_ui16 (((gType_parAPG_APG_Steer_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Steer_Ctrl_uix]))->TargetPosFilterT_ui16)
#define g_parGetParaAPG_APG_Ctrl_DeadTime_ui8 (((gType_parAPG_APG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Ctrl_uix]))->DeadTime_ui8)
#define g_parGetParaAPG_APG_Ctrl_EPSHoldTimeBeforeRelease_ui16 (((gType_parAPG_APG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Ctrl_uix]))->EPSHoldTimeBeforeRelease_ui16)
#define g_parGetParaAPG_APG_Ctrl_EPSTimeoutStraightening_ui16 (((gType_parAPG_APG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Ctrl_uix]))->EPSTimeoutStraightening_ui16)
#define g_parGetParaAPG_APG_Ctrl_SWASpecLimit_si16 (((gType_parAPG_APG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Ctrl_uix]))->SWASpecLimit_si16)
#define g_parGetParaAPG_APG_Ctrl_SWAVeloStandStillMax_ui16 (((gType_parAPG_APG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Ctrl_uix]))->SWAVeloStandStillMax_ui16)
#define g_parGetParaAPG_APG_Ctrl_SWAVeloSpecLimit_ui16 (((gType_parAPG_APG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Ctrl_uix]))->SWAVeloSpecLimit_ui16)
#define g_parGetParaAPG_APG_Ctrl_SWAAccMaxStandstill_ui16 (((gType_parAPG_APG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Ctrl_uix]))->SWAAccMaxStandstill_ui16)
#define g_parGetParaAPG_APG_Ctrl_LimiterFilter_SWAAccMax_si32 (((gType_parAPG_APG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Ctrl_uix]))->LimiterFilter_SWAAccMax_si32)
#define g_parGetParaAPG_APG_Ctrl_MaxSWADev_ui16 (((gType_parAPG_APG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Ctrl_uix]))->MaxSWADev_ui16)
#define g_parGetParaAPG_APG_Ctrl_SWATolStandstill_ui16 (((gType_parAPG_APG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Ctrl_uix]))->SWATolStandstill_ui16)
#define g_parGetParaAPG_APG_Ctrl_TargetSWAStraightening_ui16 (((gType_parAPG_APG_Ctrl_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Ctrl_uix]))->TargetSWAStraightening_ui16)
#define g_parGetParaAPG_APG_Pathplanning_Cross_VeloForwMove_si16 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->VeloForwMove_si16)
#define g_parGetParaAPG_APG_Pathplanning_Cross_VeloStraighten_si16 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->VeloStraighten_si16)
#define g_parGetParaAPG_APG_Pathplanning_Cross_VeloBackMove_si16 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->VeloBackMove_si16)
#define g_parGetParaAPG_APG_Pathplanning_Cross_XLeftTolPer_ui8 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->XLeftTolPer_ui8)
#define g_parGetParaAPG_APG_Pathplanning_Cross_XLeftTolPerNMove_ui8 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->XLeftTolPerNMove_ui8)
#define g_parGetParaAPG_APG_Pathplanning_Cross_XLeftTolMaxF10_ui16 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->XLeftTolMaxF10_ui16)
#define g_parGetParaAPG_APG_Pathplanning_Cross_PSDepthOffset_si16 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->PSDepthOffset_si16)
#define g_parGetParaAPG_APG_Pathplanning_Cross_CornerLatDiffMax_ui8 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->CornerLatDiffMax_ui8)
#define g_parGetParaAPG_APG_Pathplanning_Cross_SWAMax_si16 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->SWAMax_si16)
#define g_parGetParaAPG_APG_Pathplanning_Cross_SWA2ndMin_si16 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->SWA2ndMin_si16)
#define g_parGetParaAPG_APG_Pathplanning_Cross_ToleranceDistForEarlyGearSwitch_ui8 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->ToleranceDistForEarlyGearSwitch_ui8)
#define g_parGetParaAPG_APG_Pathplanning_Cross_XLimitFirstBackwardMove_si16 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->XLimitFirstBackwardMove_si16)
#define g_parGetParaAPG_APG_Pathplanning_Cross_AlwaysUseSwitchPoint_bl (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->AlwaysUseSwitchPoint_bl)
#define g_parGetParaAPG_APG_Pathplanning_Cross_MaxDepthForSwitchPoint_si16 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->MaxDepthForSwitchPoint_si16)
#define g_parGetParaAPG_APG_Pathplanning_Cross_Move2Optimization_bl (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->Move2Optimization_bl)
#define g_parGetParaAPG_APG_Pathplanning_Cross_VirtualOppositeSide_si16 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->VirtualOppositeSide_si16)
#define g_parGetParaAPG_APG_Pathplanning_Cross_ThresholdOneSideAlignment_ui16 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->ThresholdOneSideAlignment_ui16)
#define g_parGetParaAPG_APG_Pathplanning_Cross_TargetDistSideAlignment_si16 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->TargetDistSideAlignment_si16)
#define g_parGetParaAPG_APG_Pathplanning_Cross_TargetDist2LatRef_si16 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->TargetDist2LatRef_si16)
#define g_parGetParaAPG_APG_Pathplanning_Cross_MaxDist2LatRefFullWarn_si16 (((gType_parAPG_APG_Pathplanning_Cross_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_Pathplanning_Cross_uix]))->MaxDist2LatRefFullWarn_si16)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_VeloForwMoveFast_si16 (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->VeloForwMoveFast_si16)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_FinalPosXTolPer_ui8 (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->FinalPosXTolPer_ui8)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_FinalPosXTolMaxF10_ui16 (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->FinalPosXTolMaxF10_ui16)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_TargetDistSideAlignment_si16 (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->TargetDistSideAlignment_si16)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_SafetyDistCornerAndSide_si16 (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->SafetyDistCornerAndSide_si16)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_SafetyDistParkingSideBackwMove_si16 (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->SafetyDistParkingSideBackwMove_si16)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_SafetyDistParkingSideForwMove_si16 (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->SafetyDistParkingSideForwMove_si16)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_VirtualOppositeSideMove3F10_si16 (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->VirtualOppositeSideMove3F10_si16)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_VirtualOppositeSideMove1F10_si16 (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->VirtualOppositeSideMove1F10_si16)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_MaxLengthStraightBackwMoveF10_ui16 (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->MaxLengthStraightBackwMoveF10_ui16)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_MaxStartYawForStraightBackwMove_si16 (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->MaxStartYawForStraightBackwMove_si16)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_MinDepthForVehicleInsidePS_si16 (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->MinDepthForVehicleInsidePS_si16)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_MaxYStraightBackwMove_si16 (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->MaxYStraightBackwMove_si16)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_ConductorMoveTargetYaw_si16 (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->ConductorMoveTargetYaw_si16)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_ConductorMovePreferredYaw_si16 (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->ConductorMovePreferredYaw_si16)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_SteerLeftInBackwMoveInPS_bl (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->SteerLeftInBackwMoveInPS_bl)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_OptimizeBackwMoveEndYaw_bl (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->OptimizeBackwMoveEndYaw_bl)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_ForceForwardMove_bl (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->ForceForwardMove_bl)
#define g_parGetParaAPG_APG_PathplanCPSC_FI_XLimit1stBackwMove_si16 (((gType_parAPG_APG_PathplanCPSC_FI_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_PathplanCPSC_FI_uix]))->XLimit1stBackwMove_si16)
#define g_parGetParaAPG_APG_POC_ClothoidLength_si16 (((gType_parAPG_APG_POC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_POC_uix]))->ClothoidLength_si16)
#define g_parGetParaAPG_APG_POC_MaxSWA_si16 (((gType_parAPG_APG_POC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_POC_uix]))->MaxSWA_si16)
#define g_parGetParaAPG_APG_POC_SWAOvershoot_si16 (((gType_parAPG_APG_POC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_POC_uix]))->SWAOvershoot_si16)
#define g_parGetParaAPG_APG_POC_MaxNumOfMoves_ui8 (((gType_parAPG_APG_POC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_POC_uix]))->MaxNumOfMoves_ui8)
#define g_parGetParaAPG_APG_POC_MaxMoveLength_si16 (((gType_parAPG_APG_POC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_POC_uix]))->MaxMoveLength_si16)
#define g_parGetParaAPG_APG_POC_SteerThresStraightRevMove_ui8 (((gType_parAPG_APG_POC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_POC_uix]))->SteerThresStraightRevMove_ui8)
#define g_parGetParaAPG_APG_POC_SteerThresStraightRevMoveMin_ui8 (((gType_parAPG_APG_POC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_POC_uix]))->SteerThresStraightRevMoveMin_ui8)
#define g_parGetParaAPG_APG_POC_LatRefDistInitial_ui16 (((gType_parAPG_APG_POC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_APG_POC_uix]))->LatRefDistInitial_ui16)
#define g_parGetParaAPG_CPSC_BO_StraightMoveLength_ui16 (((gType_parAPG_CPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_CPSC_uix]))->BO_StraightMoveLength_ui16)
#define g_parGetParaAPG_CPSC_FO_StraightMoveLength_ui16 (((gType_parAPG_CPSC_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_CPSC_uix]))->FO_StraightMoveLength_ui16)

/*--------------------------------------------------------------------------*/
/*- APG_PL_POC                                                             -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaAPG_PL_POC_Handover_SafetyMargin_ui8 (((gType_parAPG_PL_POC_Handover_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_PL_POC_Handover_uix]))->SafetyMargin_ui8)
#define g_parGetParaAPG_PL_POC_Handover_DeactThreshold_MaxExtensionFrontCorner_ui16 (((gType_parAPG_PL_POC_Handover_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_PL_POC_Handover_uix]))->DeactThreshold_MaxExtensionFrontCorner_ui16)
#define g_parGetParaAPG_PL_POC_Handover_DeactThreshold_MaxYawAngle_ui32 (((gType_parAPG_PL_POC_Handover_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_PL_POC_Handover_uix]))->DeactThreshold_MaxYawAngle_ui32)
#define g_parGetParaAPG_PL_POC_Handover_YawAngleThreshold_ui32 (((gType_parAPG_PL_POC_Handover_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_APG_PL_POC_Handover_uix]))->YawAngleThreshold_ui32)

/*--------------------------------------------------------------------------*/
/*- PAGLOBAL                                                               -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaPAGLOBAL_SYSFCT_PDC_ui8 gd_SYSFCT_PDC_ui8
#define g_parGetParaPAGLOBAL_SYSFCT_PSX_ui8 gd_SYSFCT_PSX_ui8
#define g_parGetParaPAGLOBAL_SYSFCT_BSD_ui8 gd_SYSFCT_BSD_ui8
#define g_parGetParaPAGLOBAL_SYSFCT_POC_ui8 gd_SYSFCT_POC_ui8
#define g_parGetParaPAGLOBAL_SYSFCT_PS_ui8 gd_SYSFCT_PS_ui8
#define g_parGetParaPAGLOBAL_SYSFCT_SDW_ui8 gd_SYSFCT_SDW_ui8
#define g_parGetParaPAGLOBAL_SysFct_PpParallelFunc_bf8 (((gType_parPAGLOBAL_SysFct_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PAGLOBAL_SysFct_uix]))->PpParallelFunc_bf8)
#define g_parGetParaPAGLOBAL_SysFct_RearAxleSteering_bl (((gType_parPAGLOBAL_SysFct_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_PAGLOBAL_SysFct_uix]))->RearAxleSteering_bl)

/*--------------------------------------------------------------------------*/
/*- DSW                                                                    -*/
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*- VARIANT                                                                -*/
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*- SIP                                                                    -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaSIP_VHOParaAutocalib_NumStableTTCCycles_ui8 (((gType_parSIP_VHOParaAutocalib_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_SIP_VHOParaAutocalib_uix]))->NumStableTTCCycles_ui8)
#define g_parGetParaSIP_VHOParaAutocalib_NumStableSWACycles_ui8 (((gType_parSIP_VHOParaAutocalib_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_SIP_VHOParaAutocalib_uix]))->NumStableSWACycles_ui8)
#define g_parGetParaSIP_VHOParaAutocalib_NumStableTyreRadCycles_ui8 (((gType_parSIP_VHOParaAutocalib_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_SIP_VHOParaAutocalib_uix]))->NumStableTyreRadCycles_ui8)
#define g_parGetParaSIP_VHOParaAutocalib_NumStableAxlebaseCycles_ui8 (((gType_parSIP_VHOParaAutocalib_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_SIP_VHOParaAutocalib_uix]))->NumStableAxlebaseCycles_ui8)
#define g_parGetParaSIP_VHOParaAutocalib_IndexOfTyreRadCluster_ui8 (((gType_parSIP_VHOParaAutocalib_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_SIP_VHOParaAutocalib_uix]))->IndexOfTyreRadCluster_ui8)
#define g_parGetParaSIP_VHOParaAutocalib_SWOffDegRes_si16 (((gType_parSIP_VHOParaAutocalib_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_SIP_VHOParaAutocalib_uix]))->SWOffDegRes_si16)
#define g_parGetParaSIP_VHOParaAutocalib_AvgWICLengthRearAxle_ui16 (((gType_parSIP_VHOParaAutocalib_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_SIP_VHOParaAutocalib_uix]))->AvgWICLengthRearAxle_ui16)
#define g_parGetParaSIP_VHOParaAutocalib_AxlebaseRear_ui16 (((gType_parSIP_VHOParaAutocalib_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_SIP_VHOParaAutocalib_uix]))->AxlebaseRear_ui16)
#define g_parGetParaSIP_VHOParaAutocalib_ScalFacRR_ui16 (((gType_parSIP_VHOParaAutocalib_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_SIP_VHOParaAutocalib_uix]))->ScalFacRR_ui16)
#define g_parGetParaSIP_VHOParaAutocalib_ScalFacLF_ui16 (((gType_parSIP_VHOParaAutocalib_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_SIP_VHOParaAutocalib_uix]))->ScalFacLF_ui16)
#define g_parGetParaSIP_VHOParaAutocalib_ScalFacRF_ui16 (((gType_parSIP_VHOParaAutocalib_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_SIP_VHOParaAutocalib_uix]))->ScalFacRF_ui16)

/*--------------------------------------------------------------------------*/
/*- VHO                                                                    -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaVHO_NO_ECHOES_ui8 gd_VHO_NO_ECHOES_ui8
#define g_parGetParaVHO_Veh_DifFrontWheelRadiiTable_pui16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->DifFrontWheelRadiiTable_pui16)
#define g_parGetParaVHO_Veh_DrivenAxle_ui8 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->DrivenAxle_ui8)
#define g_parGetParaVHO_Veh_WheelBase_ui16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->WheelBase_ui16)
#define g_parGetParaVHO_Veh_AxleBaseRear_ui16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->AxleBaseRear_ui16)
#define g_parGetParaVHO_Veh_WICTeethCount_ui16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->WICTeethCount_ui16)
#define g_parGetParaVHO_Veh_WICLength_ui16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->WICLength_ui16)
#define g_parGetParaVHO_Veh_MaxWICValue_ui16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->MaxWICValue_ui16)
#define g_parGetParaVHO_Veh_MultMonomA_ForwSWA2SA_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->MultMonomA_ForwSWA2SA_si16)
#define g_parGetParaVHO_Veh_MultMonomB_ForwSWA2SA_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->MultMonomB_ForwSWA2SA_si16)
#define g_parGetParaVHO_Veh_CoeffC_ForwSWA2SA_si32 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->CoeffC_ForwSWA2SA_si32)
#define g_parGetParaVHO_Veh_CoeffD_ForwSWA2SA_si32 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->CoeffD_ForwSWA2SA_si32)
#define g_parGetParaVHO_Veh_MultMonomA_BackSWA2SA_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->MultMonomA_BackSWA2SA_si16)
#define g_parGetParaVHO_Veh_MultMonomB_BackSWA2SA_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->MultMonomB_BackSWA2SA_si16)
#define g_parGetParaVHO_Veh_CoeffC_BackSWA2SA_si32 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->CoeffC_BackSWA2SA_si32)
#define g_parGetParaVHO_Veh_CoeffD_BackSWA2SA_si32 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->CoeffD_BackSWA2SA_si32)
#define g_parGetParaVHO_Veh_MultMonomA_ForwSA2SWAL_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->MultMonomA_ForwSA2SWAL_si16)
#define g_parGetParaVHO_Veh_MultMonomB_ForwSA2SWAL_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->MultMonomB_ForwSA2SWAL_si16)
#define g_parGetParaVHO_Veh_CoeffC_ForwSA2SWAL_si32 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->CoeffC_ForwSA2SWAL_si32)
#define g_parGetParaVHO_Veh_CoeffD_ForwSA2SWAL_si32 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->CoeffD_ForwSA2SWAL_si32)
#define g_parGetParaVHO_Veh_MultMonomA_BackSA2SWAL_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->MultMonomA_BackSA2SWAL_si16)
#define g_parGetParaVHO_Veh_MultMonomB_BackSA2SWAL_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->MultMonomB_BackSA2SWAL_si16)
#define g_parGetParaVHO_Veh_CoeffC_BackSA2SWAL_si32 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->CoeffC_BackSA2SWAL_si32)
#define g_parGetParaVHO_Veh_CoeffD_BackSA2SWAL_si32 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->CoeffD_BackSA2SWAL_si32)
#define g_parGetParaVHO_Veh_MultMonomA_ForwSA2SWAC_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->MultMonomA_ForwSA2SWAC_si16)
#define g_parGetParaVHO_Veh_MultMonomB_ForwSA2SWAC_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->MultMonomB_ForwSA2SWAC_si16)
#define g_parGetParaVHO_Veh_CoeffC_ForwSA2SWAC_si32 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->CoeffC_ForwSA2SWAC_si32)
#define g_parGetParaVHO_Veh_CoeffD_ForwSA2SWAC_si32 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->CoeffD_ForwSA2SWAC_si32)
#define g_parGetParaVHO_Veh_MultMonomA_BackSA2SWAC_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->MultMonomA_BackSA2SWAC_si16)
#define g_parGetParaVHO_Veh_MultMonomB_BackSA2SWAC_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->MultMonomB_BackSA2SWAC_si16)
#define g_parGetParaVHO_Veh_CoeffC_BackSA2SWAC_si32 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->CoeffC_BackSA2SWAC_si32)
#define g_parGetParaVHO_Veh_CoeffD_BackSA2SWAC_si32 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->CoeffD_BackSA2SWAC_si32)
#define g_parGetParaVHO_Veh_MultMonomA_ForwSA2SWAR_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->MultMonomA_ForwSA2SWAR_si16)
#define g_parGetParaVHO_Veh_MultMonomB_ForwSA2SWAR_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->MultMonomB_ForwSA2SWAR_si16)
#define g_parGetParaVHO_Veh_CoeffC_ForwSA2SWAR_si32 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->CoeffC_ForwSA2SWAR_si32)
#define g_parGetParaVHO_Veh_CoeffD_ForwSA2SWAR_si32 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->CoeffD_ForwSA2SWAR_si32)
#define g_parGetParaVHO_Veh_MultMonomA_BackSA2SWAR_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->MultMonomA_BackSA2SWAR_si16)
#define g_parGetParaVHO_Veh_MultMonomB_BackSA2SWAR_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->MultMonomB_BackSA2SWAR_si16)
#define g_parGetParaVHO_Veh_CoeffC_BackSA2SWAR_si32 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->CoeffC_BackSA2SWAR_si32)
#define g_parGetParaVHO_Veh_CoeffD_BackSA2SWAR_si32 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->CoeffD_BackSA2SWAR_si32)
#define g_parGetParaVHO_Veh_ClippingAngle1_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->ClippingAngle1_si16)
#define g_parGetParaVHO_Veh_ClippingAngle2_si16 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->ClippingAngle2_si16)
#define g_parGetParaVHO_Veh_ScalingMonomA_SA2SWAL_bf3 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->ScalingMonomA_SA2SWAL_bf3)
#define g_parGetParaVHO_Veh_ScalingMonomB_SA2SWAL_bf3 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->ScalingMonomB_SA2SWAL_bf3)
#define g_parGetParaVHO_Veh_ScalingMonomCD_SA2SWAL_bf3 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->ScalingMonomCD_SA2SWAL_bf3)
#define g_parGetParaVHO_Veh_ScalingMonomA_SA2SWAC_bf3 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->ScalingMonomA_SA2SWAC_bf3)
#define g_parGetParaVHO_Veh_ScalingMonomB_SA2SWAC_bf3 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->ScalingMonomB_SA2SWAC_bf3)
#define g_parGetParaVHO_Veh_ScalingMonomCD_SA2SWAC_bf3 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->ScalingMonomCD_SA2SWAC_bf3)
#define g_parGetParaVHO_Veh_ScalingMonomA_SA2SWAR_bf3 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->ScalingMonomA_SA2SWAR_bf3)
#define g_parGetParaVHO_Veh_ScalingMonomB_SA2SWAR_bf3 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->ScalingMonomB_SA2SWAR_bf3)
#define g_parGetParaVHO_Veh_ScalingMonomCD_SA2SWAR_bf3 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->ScalingMonomCD_SA2SWAR_bf3)
#define g_parGetParaVHO_Veh_ScalingMonomA_SWA2SA_bf3 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->ScalingMonomA_SWA2SA_bf3)
#define g_parGetParaVHO_Veh_ScalingMonomB_SWA2SA_bf3 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->ScalingMonomB_SWA2SA_bf3)
#define g_parGetParaVHO_Veh_ScalingMonomCD_SWA2SA_bf3 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->ScalingMonomCD_SWA2SA_bf3)
#define g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8 (((gType_parVHO_Veh_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Veh_uix]))->WIC_CycleTime_MS_ui8)
#define g_parGetParaVHO_Odo_WeightSingleTrack_ui8 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->WeightSingleTrack_ui8)
#define g_parGetParaVHO_Odo_WeightDualTrack_ui8 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->WeightDualTrack_ui8)
#define g_parGetParaVHO_Odo_WeightDualTrackRear_ui8 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->WeightDualTrackRear_ui8)
#define g_parGetParaVHO_Odo_WeightDualTrackFront_ui8 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->WeightDualTrackFront_ui8)
#define g_parGetParaVHO_Odo_MaxDistDiscarded_ui32 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->MaxDistDiscarded_ui32)
#define g_parGetParaVHO_Odo_MaxDistDirUndef_ui32 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->MaxDistDirUndef_ui32)
#define g_parGetParaVHO_Odo_VCALPhaseTol_ui16 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->VCALPhaseTol_ui16)
#define g_parGetParaVHO_Odo_VCALVeloDependentPhaseTol_ui16 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->VCALVeloDependentPhaseTol_ui16)
#define g_parGetParaVHO_Odo_VCALMinDeltaZeroAllowed_bl (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->VCALMinDeltaZeroAllowed_bl)
#define g_parGetParaVHO_Odo_WeightDistanceFront_ui8 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->WeightDistanceFront_ui8)
#define g_parGetParaVHO_Odo_WeightDistanceRear_ui8 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->WeightDistanceRear_ui8)
#define g_parGetParaVHO_Odo_VIRTUAL_LSCAN_TCYC_MS_ui16 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->VIRTUAL_LSCAN_TCYC_MS_ui16)
#define g_parGetParaVHO_Odo_VIRTUAL_LSCAN_TTOL_MS_ui16 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->VIRTUAL_LSCAN_TTOL_MS_ui16)
#define g_parGetParaVHO_Odo_YawAngSmoothKoeffMin_ui8 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->YawAngSmoothKoeffMin_ui8)
#define g_parGetParaVHO_Odo_YawAngSmoothKoeffMax_ui8 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->YawAngSmoothKoeffMax_ui8)
#define g_parGetParaVHO_Odo_EngineRestartExtrapolationDistance_ui8 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->EngineRestartExtrapolationDistance_ui8)
#define g_parGetParaVHO_Odo_EngineRestartExtrapolationTimeout_ui16 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->EngineRestartExtrapolationTimeout_ui16)
#define g_parGetParaVHO_Odo_SWADelayUnitCM_ui8 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->SWADelayUnitCM_ui8)
#define g_parGetParaVHO_Odo_SWAMaxDeviationDeg_si16 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->SWAMaxDeviationDeg_si16)
#define g_parGetParaVHO_Odo_DirDetectMode_ui8 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->DirDetectMode_ui8)
#define g_parGetParaVHO_Odo_MinWICDirForMovDir_ui8 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->MinWICDirForMovDir_ui8)
#define g_parGetParaVHO_Odo_MinDistAftStdstlNoWICDir_ui8 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->MinDistAftStdstlNoWICDir_ui8)
#define g_parGetParaVHO_Odo_MaxYawAngCorrAtStandstill_ui32 (((gType_parVHO_Odo_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_VHO_Odo_uix]))->MaxYawAngCorrAtStandstill_ui32)

/*--------------------------------------------------------------------------*/
/*- MP_USER_MODE                                                           -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaMP_USER_MODE_SVA_INIT_ui8 gd_MP_USER_MODE_SVA_INIT_ui8
#define g_parGetParaMP_USER_MODE_PSX_INIT_ui8 gd_MP_USER_MODE_PSX_INIT_ui8
#define g_parGetParaMP_USER_MODE_PP_INIT_ui8 gd_MP_USER_MODE_PP_INIT_ui8
#define g_parGetParaMP_USER_MODE_PAS_INIT_ui8 gd_MP_USER_MODE_PAS_INIT_ui8

/*--------------------------------------------------------------------------*/
/*- MP_BDA                                                                 -*/
/*--------------------------------------------------------------------------*/

#define g_parGetParaMP_BDA_TEST_MAX_MEAS_TIME_ui16 gd_MP_BDA_TEST_MAX_MEAS_TIME_ui16
#define g_parGetParaMP_BDA_Block1_DataBlock1_pui8 (((gType_parMP_BDA_Block1_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_BDA_Block1_uix]))->DataBlock1_pui8)
#define g_parGetParaMP_BDA_Block2_DataBlock2_pui8 (((gType_parMP_BDA_Block2_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_BDA_Block2_uix]))->DataBlock2_pui8)
#define g_parGetParaMP_BDA_Block3_DataBlock3_pui8 (((gType_parMP_BDA_Block3_st *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_MP_BDA_Block3_uix]))->DataBlock3_pui8)


/*--------------------------------------------------------------------------*/
/*- exported data types                                                    -*/
/*--------------------------------------------------------------------------*/

#ifdef COMPONENT_PAR
#endif // COMPONENT_PAR

//
// info structure
//

//#pragma ghs struct_min_alignment(4)

typedef struct Type_parInfoData_st
{
  // ID-String to find the start of the CAL-section
  signed char Header_psc[16];

  // version (hex code decimal)
  signed char Version_psc[64];

  // structure integrity hash - 160 bit SHA1 hash
  UInt8 StructureIntegrityHash_pui8[20];

  // date (hex code decimal)
  UInt16 Year_ui16;
  UInt8 Month_ui8;
  UInt8 Day_ui8;

  // time (hex code decimal)
  UInt8 Hours_ui8;
  UInt8 Minutes_ui8;

  UInt8 reserved_pui8[22];  // fill-up to reach 128 bytes
}
gType_parInfoData_st;

//#pragma ghs struct_min_alignment()


//
// RAM default copy table
//

#if ( GS_PAR_CFG_INIT_NVM_MIRROR_MEM == SW_ON )
  typedef struct Type_parDefaultCopyData_st
  {
    void * Source_pvd;
    void * Target_pvd;
    size_t Size_six;
    UInt8 SubVarOffset_pui8[gd_PAR_NUM_MAJOR_VARIANTS_uix];
    UInt8 NumSubVars_pui8[gd_PAR_NUM_MAJOR_VARIANTS_uix];
  }
  gType_parDefaultCopyData_st;
#endif

//
// parameter cluster
//

//#pragma ghs struct_min_alignment(4)

/*--------------------------------------------------------------------------*/
/*- VHOSRC                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup VHOSRC_M04           exported data types
 * \ingroup COMPONENT_VHOSRC
 @{ */

//! parameter cluster VHOSRC_PluginSRC: Section headline only visual structuring
typedef struct Type_parVHOSRC_PluginSRC_st
{
  SInt16 accelMaxWhileBreaking_si16; //!< Accleration threshold while braking Below this braking slip is detected
  SInt16 accelMinWhileAcceleration_si16; //!< Accleration threshold while acceleration Above this traction slip is detected
  UInt8 normalWeightFront_ui8; //!< normal weight of front axle for odometry
  UInt8 normalWeightRear_ui8; //!< normal weght of rear axle for odometry
  UInt8 accelWeightFront_ui8; //!< weight of front axle while traction control for odometry
  UInt8 accelWeightRear_ui8; //!< weight of rear axle while traction control for odometry
  UInt8 decelWeightFront_ui8; //!< weight of front axle while anti block control for odometry
  UInt8 decelWeightRear_ui8; //!< weight of rear axle while anti block control for odometry
  UInt16 dynStateHoldTime_ui16; //!< timer to prevent toggle of dyn state
  UInt16 distanceMaxABSintervention_ui16; //!< maximum allowed driven distance in mm while anti block control before guidance shut down is advised
  UInt16 distanceMaxASRintervention_ui16; //!< maximum allowed driven distance in mm while traction control before guidance shut down is advised
  UInt16 distanceMaxESPintervention_ui16; //!< maximum allowed driven distance in mm while vehicle dynamic control before gui dance shut down is ad
}
gType_parVHOSRC_PluginSRC_st; //!< copydoc Type_parVHOSRC_PluginSRC_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- VHOTCE                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup VHOTCE_M04           exported data types
 * \ingroup COMPONENT_VHOTCE
 @{ */

//! parameter cluster VHOTCE_PluginTTC: Section headline only visual structuring
typedef struct Type_parVHOTCE_PluginTTC_st
{
  UInt16 VeloMax_ui16; //!< Maximum velocity used for data sampling
  UInt32 DrivingDist_ui32; //!< Distancetarget for simple samplie
  SInt32 MaxAngle_si32; //!< Maximum allowed yawangle difference between trajectory start and end
  UInt16 SWAMax_ui16; //!< Maximum alowed SWA while datasampling
  UInt8 NumberOfSamples_ui8; //!< Number of datasamples which are considered in result calculation
}
gType_parVHOTCE_PluginTTC_st; //!< copydoc Type_parVHOTCE_PluginTTC_st


//! parameter cluster VHOTCE_PluginTCE: Section headline only visual structuring
typedef struct Type_parVHOTCE_PluginTCE_st
{
  UInt16 TargetForQualityCummulation_ui16; //!< Higher values will get higher estimation qality but also higher amount of time to get first calibrat
  UInt16 VeloMin_ui16; //!< Lower velocity limit for algorithm operation
  UInt16 VeloMax_ui16; //!< Uper velocity limit for algorithm operation
  UInt16 LongACCMax_ui16; //!< Maximum longitutinal acceleration for algorithm operation
  UInt16 SWAMin_ui16; //!< Minimum steering wheel angle for data usage
  UInt16 SWAMax_ui16; //!< Maximum steeringwheelangle for data usage
  UInt32 YawangleFinish_ui32; //!< Target value for yawangle integration
  UInt32 YawangleMax_ui32; //!< If integration for example on left side has not reached finish value intgration may proceed onright
  UInt32 YawangleWICError_ui32; //!< Minimum yawangleuncertanity caused by WIC quantisation
  UInt16 ToleratedVelocityDiff_ui16; //!< Maximum mean velocity deviation between right and left integration
  UInt16 ToleratedSWADiff_ui16; //!< Maximum mean SWA deviation between right and left integration
  UInt16 YawrateMin_ui16; //!< Minimum yawrate
  UInt16 YawrateMax_ui16; //!< Maximum yawrate
}
gType_parVHOTCE_PluginTCE_st; //!< copydoc Type_parVHOTCE_PluginTCE_st


//! parameter cluster VHOTCE_PluginTCE_clustering: Section headline only visual structuring
typedef struct Type_parVHOTCE_PluginTCE_clustering_st
{
  UInt8 ProcentualCenteringOfCluster_ui8; //!< If set to 50% both clusterborders are in same distance to WICLength of cluster If value is smaller t
  UInt16 ThresholdForTirechange_ui16; //!< If this value is greater than 0 a cluster change is only allowed if after KL15 a change in WIC Lengt
  UInt8 NumberOfClusters_ui8; //!< Number of used entries in table below
  UInt16 WICLengthOfCluster_pui16[10]; //!< WICLength of tire cluster Value that algorithm will measure with VHO common wheel base parameter
  UInt16 WheelBaseOfCluster_pui16[10]; //!< effective wheel base estimated for tire cluster
}
gType_parVHOTCE_PluginTCE_clustering_st; //!< copydoc Type_parVHOTCE_PluginTCE_clustering_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- MP_SNOWMUD                                                             -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_SNOWMUD_M04           exported data types
 * \ingroup COMPONENT_MP_SNOWMUD
 @{ */

/*!@} */


/*--------------------------------------------------------------------------*/
/*- EOL                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup EOL_M04           exported data types
 * \ingroup COMPONENT_EOL
 @{ */

/*!@} */


/*--------------------------------------------------------------------------*/
/*- AP_PS                                                                  -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup AP_PS_M04           exported data types
 * \ingroup COMPONENT_AP_PS
 @{ */

//! parameter cluster AP_PS_Config: Section headline only visual structuring
typedef struct Type_parAP_PS_Config_st
{
  Boolean SpeedLimitBrakingPSX_bl; //!< activate/deactivate Braking for Speed Limiting during park guidance by Parameter only takes effect i
  Boolean ParkEmergencyBraking_bl; //!< activate/deactivate Emergency Braking for Obstacles during park guidance by Parameter only takes eff
  Boolean ManeuverEmergencyBraking_bl; //!< activate/deactivate Emergency Braking for Obstacles during low speed maneuvers by Parameter only tak
  Boolean ShortenedDrivingTube_bl; //!< activate/deactivate the shortened driving tube Shortened driving tube means only obstacles immediatl
  Boolean CloseObjectDetectedBraking_bl; //!< activate/deactivate braking for close objects detected during PCB maneuver Only takes effect if swit
  Boolean BrakeForHighObjectsOnly_bl; //!< activate/deactivate braking for InchhighInch/InchunknownInch objects OLO height classification only
}
gType_parAP_PS_Config_st; //!< copydoc Type_parAP_PS_Config_st


//! parameter cluster AP_PS_Speed: Section headline only visual structuring
typedef struct Type_parAP_PS_Speed_st
{
  UInt8 DeactivationLow_ui8; //!< At speeds lower than this parameter the AP_PS component will not apply any torque / acceleration for
  UInt8 DeactivationHigh_ui8; //!< At speeds higher than this parameter the AP_PS component will not apply any torque / acceleration fo
  UInt8 FirstClothoidShift_ui8; //!< To assure that the vehicle speed is already at InchLimitSteeringInch value the change to the lower v
  UInt8 LimitStraight_ui8; //!< Speed limit for the straight start of the parking maneuver Only relevant for parallel parking maneuv
  UInt8 LimitSteering1_ui8; //!< Speed limit for the 1st clothoid of the parking maneuver Only relevant for parallel parking maneuver
  UInt8 LimitSteering2_ui8; //!< Speed limit for the inflection point of the trajectory 2nd clothoid Only relevant for parallel parki
  UInt8 LimitNMove_ui8; //!< This parameter is applied for parallel parking manouvers with more than one move Sets the speed limi
  UInt8 LimitCPSCBackStraight_ui8; //!< Parameter is used as speed limit for first half of CPSC 1st backward move If there is no no straight
  UInt8 LimitCPSCBack_ui8; //!< Parameter is used as speed limit for second half of CPSC 1st backward move and all following CPSC ba
  UInt8 LimitCPSCForw_ui8; //!< Parameter is used as speed limit for all CPSC forward moves
  UInt8 LimitFwdCPSC_ui8; //!< Parameter is used as speed limit for all Forward CPSC moves
  UInt8 LimitHysteresis_ui8; //!< The hysteresis is used for all speed limiting parameters If braking is applied due to speed limiting
  UInt8 MbaC_Speedlimit1_ui8; //!< Parameter is used to define speed limit for obejcts closer than parameter MbaC_ObjDist1
  UInt8 MbaC_Speedlimit2_ui8; //!< Parameter is used to define speed limit for obejcts closer than parameter MbaC_ObjDist2
  UInt8 MbaC_Speedlimit3_ui8; //!< Parameter is used to define speed limit for obejcts closer than parameter MbaC_ObjDist3
  UInt8 MbaC_Speedlimit4_ui8; //!< Parameter is used to define speed limit for obejcts closer than parameter MbaC_ObjDist4
  UInt8 MbaC_ActiveNoObject_ui8; //!< Parameter is used to define speed limit if no object is closer than parameter MbaC_ObjDist1
}
gType_parAP_PS_Speed_st; //!< copydoc Type_parAP_PS_Speed_st


//! parameter cluster AP_PS_Brake: Section headline only visual structuring
typedef struct Type_parAP_PS_Brake_st
{
  UInt8 HoldTime_ui8; //!< After reaching stand still brake will will applied until given time period is reached
  UInt16 TorqueSpeedLimiting_ui16; //!< Brake torque used for speed limiting
  UInt16 TorqueCollision_ui16; //!< Brake torque used for collision avoidance/mitigation
  UInt8 AccelerationSpeedLimiting_ui8; //!< Brake acceleration used for speed limiting
  UInt8 AccelerationCollision_ui8; //!< Brake acceleration used for collision avoidance/mitigation
  UInt8 DeadTime_ui8; //!< Dead time of the brake
  UInt8 MaxJerk_ui8; //!< Maximum brake jerk i e for hi mue and medium load
  UInt8 Mult_ui8; //!< factor to multiply calculated stop distance for application purposes
  UInt8 Add_ui8; //!< additional distance which will be added to calculated stop distance for application purposes
  UInt8 TimeToCollision1_ui8; //!< used to decide if close object braking is neccessary
  UInt8 TimeToCollision2_ui8; //!< used to decide if emergency braking is neccessary
  UInt16 ComfortableDecelMaximum_ui16; //!< maximum decel value which can be used during usual comfortable braking
  UInt16 CloseObjectDecelMaximum_ui16; //!< maximum decel value which can be used during close object detected braking
  SInt16 MaximumEngineJerk_si16; //!< Maximum brake jerk which is used during acceleration phase to brake the vehicle in order to reach a
  SInt16 MaximumDecelerationJerk_si16; //!< Maximum brake jerk which is used during deceleration phase to brake the vehicle to lower velocity or
  SInt16 MaxEngJerkCloseObject_si16; //!<  only used for close object detected braking Maximum brake jerk which is used during acceleration ph
  SInt16 MaxDecelJerkCloseObject_si16; //!<  only used for close object detected braking Maximum brake jerk which is used during deceleration ph
  UInt8 ESP_MinControlDist_ui8; //!< The parameter specifies the minimum distance the ESP brake controller is able to move from standstil
  UInt8 OocSafeDistPCB_ui8; //!< Safety distance which should be between vehicle and object in standstill if braking because of an OO
  UInt8 CreepModusMinDist_ui8; //!< Minimum distance to obejct at which creep modus should end
  UInt8 SafetyDistVehicle_ui8; //!< Safety distance which should be used for calculation of OOC object on course
  UInt8 NextMoveRequestDistancePar_ui8; //!< If vehicle reaches standstill due to an object on course within this distance to end point calculate
  UInt16 NextMoveRequestDistancePerp_ui16; //!< If vehicle reaches standstill due to an object on course within this distance to end point calculate
  UInt8 ComfStopBrakeFactor_ui8; //!< Factor to scale StopDistance in order to generate smooth and fast braking for standstill Parameter i
  UInt8 StopDist_1_ui8; //!< distance to object at which brake is engaged at 1kmh Parameter for PEB Default value is for VW Golf
  UInt8 StopDist_2_ui8; //!< distance to object at which brake is engaged at 2kmh Parameter for PEB Default value is for VW Golf
  UInt8 StopDist_3_ui8; //!< distance to object at which brake is engaged at 3kmh Parameter for PEB Default value is for VW Golf
  UInt8 StopDist_4_ui8; //!< distance to object at which brake is engaged at 4kmh Parameter for PEB Default value is for VW Golf
  UInt8 StopDist_5_ui8; //!< distance to object at which brake is engaged at 5kmh Parameter for PEB Default value is for VW Golf
  UInt8 StopDist_6_ui8; //!< distance to object at which brake is engaged at 6kmh Parameter for PEB Default value is for VW Golf
  UInt8 StopDist_7_ui8; //!< distance to object at which brake is engaged at 7kmh Parameter for PEB Default value is for VW Golf
  UInt8 StopDist_8_ui8; //!< distance to object at which brake is engaged at 8kmh Parameter for PEB Default value is for VW Golf
  UInt8 StopDist_9_ui8; //!< distance to object at which brake is engaged at 9kmh Parameter for PEB Default value is for VW Golf
  UInt8 StopDist_10_ui8; //!< distance to object at which brake is engaged at 10kmh Parameter for PEB Default value is for VW Golf
  UInt8 StopDist_11_ui8; //!< distance to object at which brake is engaged at 11kmh Parameter for PEB Default value is for VW Golf
  UInt8 StopDistMEB_1_ui8; //!< distance to object at which brake is engaged at 1kmh Parameter for MEB Default value is for VW Golf
  UInt8 StopDistMEB_2_ui8; //!< distance to object at which brake is engaged at 2kmh Parameter for MEB Default value is for VW Golf
  UInt8 StopDistMEB_3_ui8; //!< distance to object at which brake is engaged at 3kmh Parameter for MEB Default value is for VW Golf
  UInt8 StopDistMEB_4_ui8; //!< distance to object at which brake is engaged at 4kmh Parameter for MEB Default value is for VW Golf
  UInt8 StopDistMEB_5_ui8; //!< distance to object at which brake is engaged at 5kmh Parameter for MEB Default value is for VW Golf
  UInt8 StopDistMEB_6_ui8; //!< distance to object at which brake is engaged at 6kmh Parameter for MEB Default value is for VW Golf
  UInt8 StopDistMEB_7_ui8; //!< distance to object at which brake is engaged at 7kmh Parameter for MEB Default value is for VW Golf
  UInt8 StopDistMEB_8_ui8; //!< distance to object at which brake is engaged at 8kmh Parameter for MEB Default value is for VW Golf
  UInt8 StopDistMEB_9_ui8; //!< distance to object at which brake is engaged at 9kmh Parameter for MEB Default value is for VW Golf
  UInt8 StopDistMEB_10_ui8; //!< distance to object at which brake is engaged at 10kmh Parameter for MEB Default value is for VW Golf
  UInt8 StopDistMEB_11_ui8; //!< distance to object at which brake is engaged at 11kmh Parameter for MEB Default value is for VW Golf
}
gType_parAP_PS_Brake_st; //!< copydoc Type_parAP_PS_Brake_st


//! parameter cluster AP_PS_Objects: Section headline only visual structuring
typedef struct Type_parAP_PS_Objects_st
{
  UInt8 MinExistProbability_ui8; //!< Only objects with a higher existance probability than this parameter are used for braking
  UInt8 MinExistProbabilityDECEonly_ui8; //!< Only objects with a higher existance probability than this parameter are used for braking Parameter
  UInt8 MinimumDistance_ui8; //!< Standstill shall be reached if the object has a distance equal or smaller than this parameter
  UInt16 MbaC_ObjDist1_ui16; //!< Parameter is used to define at which distance to objects the speed limit defined in parameter MbaC_S
  UInt16 MbaC_ObjDist2_ui16; //!< Parameter is used to define at which distance to objects the speed limit defined in parameter MbaC_S
  UInt16 MbaC_ObjDist3_ui16; //!< Parameter is used to define at which distance to objects the speed limit defined in parameter MbaC_S
  UInt16 MbaC_ObjDist4_ui16; //!< Parameter is used to define at which distance to objects the speed limit defined in parameter MbaC_S
  UInt16 MAA_Active_ObjDist_ui16; //!< Parameter is used to define at which distance to objects MAA should reduce engine torque
}
gType_parAP_PS_Objects_st; //!< copydoc Type_parAP_PS_Objects_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- BDA                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup BDA_M04           exported data types
 * \ingroup COMPONENT_BDA
 @{ */

//! parameter cluster BDA_Parameter: Section headline only visual structuring
typedef struct Type_parBDA_Parameter_st
{
  Boolean SetAllSensorsToNotBlind_bl; //!< 0 false output will be used for system reaction 1 true BDA is running but all outputs will be set to
  UInt16 RelevantSensorsBitMask_ui16; //!< Relevant sensors which shall be tested and BDA output is set bit mask Examples A Test result only re
  Boolean TriggerBlindnessTestAtInit_bl; //!< 0 false do not request for a blindness test at every initialisation 1 true request for a blindness t
  UInt32 Risk_TimeOfNoObject_ui32; //!< Time since a sensor has been active but no object has been detected set risk to InchhighInch and ret
  UInt32 Risk_TimeOfBlindSensor_ui32; //!< Time since a sensor has been not active and this sensor was already determined as blind at the last
  UInt16 vDependingSetBlindMask1_ui16; //!< Bitmask for sensors for which InchBlindnessInch might be set above the velocity threshold InchvDepen
  UInt8 vDependingThreshold1_ui8; //!< If velocity is above this threshold the BitMask InchvDependingSetBlindMask1Inch is used
  UInt16 vDependingSetBlindMask2_ui16; //!< Bitmask for sensors for which InchBlindnessInch might be set above the velocity threshold InchvDepen
  UInt8 vDependingThreshold2_ui8; //!< If velocity is above this threshold the BitMask InchvDependingSetBlindMask2Inch is used
  UInt16 MinEchoDistance_ui16; //!< Minimum valid distance of sensor data in cm
  UInt16 MaxEchoDistance_ui16; //!< Maximum valid distance of sensor data in cm
  UInt16 MinGroundEchoDistance_ui16; //!< Minimum valid distance for ground echo cm
  UInt16 MaxGroundEchoDistance_ui16; //!< Maximum valid distance for ground echo cm
  UInt16 MaxEchoWidth_ui16; //!< Maximum valid echo width of sensor data in microsec Focus filter disturbances / noise forced by a ri
  UInt16 DurationOfBlindnessTest_ui16; //!< Time while a really high sensitivity is used by SW for all sensors Special test mode active in MP
  SInt8 MaxTempForBlindnessTest_si8; //!< Temperature threshold BlindnessTest is only triggered by BDA if CAN value is equal or below this thr
  UInt32 UnknownStatusMaxWaitTime_ui32; //!< Time threshold in sec Maximum wait time since a BlindnessTest is triggered by BDA but was not procee
}
gType_parBDA_Parameter_st; //!< copydoc Type_parBDA_Parameter_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- FUS                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup FUS_M04           exported data types
 * \ingroup COMPONENT_FUS
 @{ */

/*!@} */


/*--------------------------------------------------------------------------*/
/*- OOC                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup OOC_M04           exported data types
 * \ingroup COMPONENT_OOC
 @{ */

/*!@} */


/*--------------------------------------------------------------------------*/
/*- MP_DIAG_TEST                                                           -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_DIAG_TEST_M04           exported data types
 * \ingroup COMPONENT_MP_DIAG_TEST
 @{ */

/*!@} */


/*--------------------------------------------------------------------------*/
/*- PSD                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup PSD_M04           exported data types
 * \ingroup COMPONENT_PSD
 @{ */

//! parameter cluster PSD_Veh: Section headline only visual structuring
typedef struct Type_parPSD_Veh_st
{
  UInt16 FrontSensorXDist_ui16; //!< position of the sensor distance between rear axle of the car and sensor in x direction left and righ
  UInt16 FrontSensorYDist_ui16; //!< position of the sensor distance between middle line of the car and sensor in y direction left and ri
  UInt8 FrontSensorAngle_ui8; //!< Out of order at this moment angle of the PSD sensor angle between middle line of the car and sensor
  SInt16 RearSensorXDist_si16; //!< position of the rear sensor distance between rear axle of the car and sensor in x direction left and
  UInt16 RearSensorYDist_ui16; //!< position of the rear sensor distance between middle line of the car and sensor in y direction left a
  UInt8 RearSensorAngle_ui8; //!< Out of order at this moment angle of the rear PSD sensor angle between middle line of the car and se
  SInt8 AdditionalPSLengthFOV_si8; //!< Out of order at this moment correction of the statistical length of all parking spaces depending on
  SInt8 DisplacementOfPS_si8; //!< Displacement of parking space By this parameter it s possible to displace the whole parking space in
  UInt16 PSLengthMin_ui16; //!< Minimum length to declare the parking space as valid proposal e g car length This parameter is used
  UInt16 PSLengthMax_ui16; //!< Maximum length to declare the parking space as valid Remark if this value is higher as MinPSDength i
  UInt16 PSDepthDefault_ui16; //!< Default depth of the parking space this will be used for the depth if no valid lateral object curbst
}
gType_parPSD_Veh_st; //!< copydoc Type_parPSD_Veh_st


//! parameter cluster PSD_Proj: no valid cluster description available
typedef struct Type_parPSD_Proj_st
{
  UInt16 AOIMinDist_ui16; //!<  Area of interest Minimum distance between the car edge and the lateral object for using the as vali
  UInt16 AOIMaxDist_ui16; //!<  Area of interest Maximum distance between the car edge and the lateral object for using the as vali
  UInt16 LatDist_Min_ui16; //!< Minimum lateral distance between 1st parking space corner or 2nd parking space corner and the latera
  UInt16 PSDistMax_ui16; //!< Maximum drive by distance to declare the parking space as valid if the driver is passing both longit
  UInt16 MergeObst_DistMax_ui16; //!< Depends on InchMERGE_OBSTACLE_WITH_CARInch Distance between obstacle start and object start
  SInt16 MergeObst_SafetyDist_si16; //!< Depends on InchMERGE_OBSTACLE_WITH_CARInch Safety distance to obstacle parking space length will be
}
gType_parPSD_Proj_st; //!< copydoc Type_parPSD_Proj_st


//! parameter cluster PSD_Proj_OCP: Section headline only visual structuring
typedef struct Type_parPSD_Proj_OCP_st
{
  UInt16 MinCurbDepth_ui16; //!< Minimum lateral distance between 1st parking space corner or 2nd parking space corner and the curbst
  UInt16 MaxCurbDepth_ui16; //!< Maximum lateral distance between 1st parking space corner or 2nd parking space corner and the curbst
  UInt16 MinCurbLength_ui16; //!< Minimum length of the curbstone lateral object to declare the parking space on this curbstone as val
  UInt16 OCP2SafetyDistToCurb_ui16; //!< gd_PSDPara_ProjOCP_OCP2SafetyDistToCurb_ui16 in case of parking on curbstone with 2 wheels Safety di
  UInt16 OCP4SafetyDistToCurb_ui16; //!< gd_PSDPara_ProjOCP_OCP4SafetyDistToCurb_ui16 in case of parking on curbstone with 4 wheels Safety di
}
gType_parPSD_Proj_OCP_st; //!< copydoc Type_parPSD_Proj_OCP_st


//! parameter cluster PSD_Proj_Cross: Section headline only visual structuring
typedef struct Type_parPSD_Proj_Cross_st
{
  UInt16 PSLengthMin_ui16; //!< Minimal length of a valid cross parking space
  UInt16 PSLengthMax_ui16; //!< Maximum length of a valid cross parking space
  UInt16 LatDiffMax_ui16; //!< Maximum lateral distance between two longitudinal objects for a valid parking space
  UInt16 PassingDistMax_ui16; //!< Maximum lateral passing distance to both longitudinal objects for a valid parking space
  UInt16 PSLengthVCType1_ui16; //!< Length of the virtual crossparking space of type 1 Scene nothing object Length of the virtual crossp
  UInt16 PSLengthVCType2_ui16; //!< Length of the virtual crossparking space of type 2 Scene object nothing Length of the virtual crossp
  UInt16 PSLengthVCType1Shortened_ui16; //!< Length of virtual crosspakring space of type 1 to which the parking space is shortened maximally if
}
gType_parPSD_Proj_Cross_st; //!< copydoc Type_parPSD_Proj_Cross_st


//! parameter cluster PSD_RuleSet1: Section headline only visual structuring
typedef struct Type_parPSD_RuleSet1_st
{
  SInt16 Normal_OffsetToPSDat_Max_si16; //!< Threshold to decide between parking in front of the curbstone or alignment to 2nd corner This parame
  SInt16 OffsetOnStreetLat_Max_si16; //!< Theshold to determine a valid parking space normal Maximal lateral displacement in end position This
  SInt16 Normal_SafeDistToObst_Lat_si16; //!< Safety distance to obstacle in case of parking before the obstacle Determines whether the parking sp
  SInt16 Normal_SafeDistToCurb_si16; //!< Safety distance to curbstone in case of parking in front of a curbstone normal parking slot Determin
  SInt16 OCP2_Safe_AOI_Obj2_Max_si16; //!< Theshold to determine parking space type OCP2 or OCP4 If depth of 2nd object higher than this value
  SInt16 OCP2_Safe_AOI_Obj2_Min_si16; //!< Theshold to determine parking space type OCP2 or OCP4 Minimal depth of 2nd object for parking on cur
  SInt16 OCP2_SafeDistToCurb_Lat_si16; //!< Safety distance to the curbstone in case of parking on the curbstone OCP2
  SInt16 OCP4_SafeDistToCurb_Lat_si16; //!< Safety distance to the curbstone in case of parking on the curbstone OCP4
  SInt16 Normal_SafeDistToObst_Long_si16; //!< Longitudinal safety distance to not plausible objects or obstacles in the parking space This is beca
  UInt16 MinLengthForPlausObject_ui16; //!< Minimum length of an object to set this object to the status plausible All objects smaller than that
}
gType_parPSD_RuleSet1_st; //!< copydoc Type_parPSD_RuleSet1_st


//! parameter cluster PSD_Proj_VC: Section headline only visual structuring
typedef struct Type_parPSD_Proj_VC_st
{
  UInt16 MinCurbLength_ui16; //!< Parameter for the scene Car parking space nothing Minimum length of the curbstone lateral object to
  UInt16 MinPSLength_ui16; //!< Parameter is used for all virtual scenes Scene Car parking space nothing Minimal length of the virtu
  UInt16 MaxPSDength_ui16; //!< Parameter for the scene Car parking space nothing Maximal length of the virtual parking space Depend
  UInt16 MinPSDepth_ui16; //!< Scene Car parking space nothing Minimum lateral distance between 1st parking space corner and the cu
  UInt16 MaxPSDepth_ui16; //!< Scene Car parking space nothing Maximum lateral distance between 1st parking space corner and the cu
  UInt16 MinPSLengthShortened_ui16; //!< to which length the length of virtual parking space will be shortened in case that driven distance s
}
gType_parPSD_Proj_VC_st; //!< copydoc Type_parPSD_Proj_VC_st


//! parameter cluster PSD_Proj_SPR: Section headline only visual structuring
typedef struct Type_parPSD_Proj_SPR_st
{
  UInt16 OperatingUSDist_ui16; //!< Maximal range of the side protection functionality If no object has been detected in this range this
  UInt16 MinWayForUpdate_ui16; //!< Driven distance between two actualizations of the SPR output structure default value platform value
}
gType_parPSD_Proj_SPR_st; //!< copydoc Type_parPSD_Proj_SPR_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- MP_USS                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_USS_M04           exported data types
 * \ingroup COMPONENT_MP_USS
 @{ */

//! parameter cluster MP_USS_FISelection: structure to hold the selected Cc set block variants
typedef struct Type_parMP_USS_FISelection_st
{
  UInt8 CcSetBl1Id_ui8; //!< Id of the block 1 of the Cc set for front inner/middle sensors
  UInt8 CcSetBl2Id_ui8; //!< Id of the block 2 of the Cc set for front inner/middle sensors
}
gType_parMP_USS_FISelection_st; //!< copydoc Type_parMP_USS_FISelection_st


//! parameter cluster MP_USS_FIBlock1: no valid cluster description available
typedef struct Type_parMP_USS_FIBlock1_st
{
  UInt8 DataBlock1_pui8[10]; //!< USS characteristic curve set block 1 contains all echo sensitivity threshold limits 4 inner front US
}
gType_parMP_USS_FIBlock1_st; //!< copydoc Type_parMP_USS_FIBlock1_st


//! parameter cluster MP_USS_FIBlock2: no valid cluster description available
typedef struct Type_parMP_USS_FIBlock2_st
{
  UInt8 DataBlock2_pui8[5]; //!< USS characteristic curve set block 2 contains all measurement setting 4 inner front USS channels
}
gType_parMP_USS_FIBlock2_st; //!< copydoc Type_parMP_USS_FIBlock2_st


//! parameter cluster MP_USS_RISelection: structure to hold the selected Cc set block variants
typedef struct Type_parMP_USS_RISelection_st
{
  UInt8 CcSetBl1Id_ui8; //!< Id of the block 1 of the Cc set for rear inner/middle sensors
  UInt8 CcSetBl2Id_ui8; //!< Id of the block 2 of the Cc set for rear inner/middle sensors
}
gType_parMP_USS_RISelection_st; //!< copydoc Type_parMP_USS_RISelection_st


//! parameter cluster MP_USS_RIBlock1: no valid cluster description available
typedef struct Type_parMP_USS_RIBlock1_st
{
  UInt8 DataBlock1_pui8[10]; //!< USS characteristic curve set block 1 contains all echo sensitivity threshold limits 4 inner rear USS
}
gType_parMP_USS_RIBlock1_st; //!< copydoc Type_parMP_USS_RIBlock1_st


//! parameter cluster MP_USS_RIBlock2: no valid cluster description available
typedef struct Type_parMP_USS_RIBlock2_st
{
  UInt8 DataBlock2_pui8[5]; //!< USS characteristic curve set block 2 contains all measurement setting 4 inner rear USS channels
}
gType_parMP_USS_RIBlock2_st; //!< copydoc Type_parMP_USS_RIBlock2_st


//! parameter cluster MP_USS_FOSelection: structure to hold the selected Cc set block variants
typedef struct Type_parMP_USS_FOSelection_st
{
  UInt8 CcSetBl1Id_ui8; //!< Id of the block 1 of the Cc set for front outer Uss 1 and 6
  UInt8 CcSetBl2Id_ui8; //!< Id of the block 2 of the Cc set for front outer Uss 1 and 6
}
gType_parMP_USS_FOSelection_st; //!< copydoc Type_parMP_USS_FOSelection_st


//! parameter cluster MP_USS_FOBlock1: no valid cluster description available
typedef struct Type_parMP_USS_FOBlock1_st
{
  UInt8 DataBlock1_pui8[10]; //!< USS characteristic curve set block 1 contains all echo sensitivity threshold limits 2 outer front US
}
gType_parMP_USS_FOBlock1_st; //!< copydoc Type_parMP_USS_FOBlock1_st


//! parameter cluster MP_USS_FOBlock2: no valid cluster description available
typedef struct Type_parMP_USS_FOBlock2_st
{
  UInt8 DataBlock2_pui8[5]; //!< USS characteristic curve set block 2 contains all measurement setting 2 outer front USS channels 1 a
}
gType_parMP_USS_FOBlock2_st; //!< copydoc Type_parMP_USS_FOBlock2_st


//! parameter cluster MP_USS_ROSelection: structure to hold the selected Cc set block variants
typedef struct Type_parMP_USS_ROSelection_st
{
  UInt8 CcSetBl1Id_ui8; //!< Id of the block 1 of the Cc set for rear outer Uss 8 and 13
  UInt8 CcSetBl2Id_ui8; //!< Id of the block 2 of the Cc set for rear outer Uss 8 and 13
}
gType_parMP_USS_ROSelection_st; //!< copydoc Type_parMP_USS_ROSelection_st


//! parameter cluster MP_USS_ROBlock1: no valid cluster description available
typedef struct Type_parMP_USS_ROBlock1_st
{
  UInt8 DataBlock1_pui8[10]; //!< USS characteristic curve set block 1 contains all echo sensitivity threshold limits 2 outer front US
}
gType_parMP_USS_ROBlock1_st; //!< copydoc Type_parMP_USS_ROBlock1_st


//! parameter cluster MP_USS_ROBlock2: no valid cluster description available
typedef struct Type_parMP_USS_ROBlock2_st
{
  UInt8 DataBlock2_pui8[5]; //!< USS characteristic curve set block 2 contains all measurement setting 2 outer rear USS channels 1 an
}
gType_parMP_USS_ROBlock2_st; //!< copydoc Type_parMP_USS_ROBlock2_st


//! parameter cluster MP_USS_CcSetInfo: Parameter for the PSx temperature hysteresis
typedef struct Type_parMP_USS_CcSetInfo_st
{
  UInt8 NumCcSets_ui8; //!< number of different Characteristic Curve Sets
  UInt8 DefaultIdx_ui8; //!< ID of default Characteristic Curve Set
  SInt8 TempRange_psi8[10]; //!< Upper limit of temperature range for the different Characteristic Curve Sets
  SInt8 TempHyst_si8; //!< Parameter for the PSx temperature hysteresis
}
gType_parMP_USS_CcSetInfo_st; //!< copydoc Type_parMP_USS_CcSetInfo_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- MP_INTERFERENCE                                                        -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_INTERFERENCE_M04           exported data types
 * \ingroup COMPONENT_MP_INTERFERENCE
 @{ */

/*!@} */


/*--------------------------------------------------------------------------*/
/*- APCTL                                                                  -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup APCTL_M04           exported data types
 * \ingroup COMPONENT_APCTL
 @{ */

//! parameter cluster APCTL_CFG: SP_PWR configuration parameters
typedef struct Type_parAPCTL_CFG_st
{
  UInt8 VCutOffPAS_accelerate_ui8; //!< cut off speed pas for accelerating car
  UInt8 VCutOffPAS_hysteresis_ui8; //!< hysteresis for pas cut off speed
  UInt8 VCutOffPSX_accelerate_ui8; //!< cut off speed psx for accelerating car
  UInt8 VCutOffPSX_hysteresis_ui8; //!< hysteresis for psx cut off speed
  UInt8 BSDCutoffSpeed_ui8; //!< CutOff Speed for BSD
  UInt8 BSDCutoffSpeedHys_ui8; //!< Hysterisis for CutOff Speed for BSD
  UInt8 BSDMinVelocityAc_ui8; //!< Minimmum velocity for BSD Activation
  UInt8 BSDMinVelocityDe_ui8; //!< Minimmum velocity for BSD Deactivation
  UInt8 GuidanceVmaxExceeded_ui8; //!< Maximum velocity during Guidance
  UInt8 VStandStillCutOffPSX_ui8; //!< Cut off threshold for stand still
  SInt16 SWAMinOMA_si16; //!< SWA parameter to set OMA op mode and OMA side for CPFI The parameter should have 90 degree
  SInt16 SWAMaxOMA_si16; //!< SWA parameter to set OMA op mode and OMA side for CPFI The parameter should have 90 degree
}
gType_parAPCTL_CFG_st; //!< copydoc Type_parAPCTL_CFG_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- DISPLAY                                                                -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup DISPLAY_M04           exported data types
 * \ingroup COMPONENT_DISPLAY
 @{ */

//! parameter cluster DISPLAY_Veh: Section headline only visual structuring
typedef struct Type_parDISPLAY_Veh_st
{
  UInt8 DurationTone_pui8[2]; //!< DurationTone
  UInt8 DurationNoTone_pui8[2]; //!< DurationNoTone
  UInt8 ToneSetup_pui8[2]; //!< tone configuration tone setup
  UInt8 RangeZoneFront_pui8[10]; //!<  range zones front system
  UInt8 RangeZoneRear_pui8[10]; //!<  range zones rear system
  UInt8 DisplayConfigFront_pui8[10]; //!< display setup display and tone front system
  UInt8 DisplayConfigRear_pui8[10]; //!< display setup display and tone rear system
}
gType_parDISPLAY_Veh_st; //!< copydoc Type_parDISPLAY_Veh_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- ACS                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup ACS_M04           exported data types
 * \ingroup COMPONENT_ACS
 @{ */

/*!@} */


/*--------------------------------------------------------------------------*/
/*- PSM                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup PSM_M04           exported data types
 * \ingroup COMPONENT_PSM
 @{ */

//! parameter cluster PSM_DiscardCriteria: Discarding Parameters
typedef struct Type_parPSM_DiscardCriteria_st
{
  SInt16 MaxSPos_si16; //!< Maximum distance between parking space measured from x 0 and the permitted distance to drive on in r
  SInt16 MinSPos_si16; //!< Minimum distance between parking space measured from x 0 and the permitted distance to drive back in
  SInt16 MaxSPosRearRef_si16; //!< Maximum distance between the parking space measured from first corner and the permitted distance to
  SInt16 MinSPosRearRef_si16; //!< Minimum distance between the parking space measured from first corner and the permitted distance to
  UInt16 MaxAbsoluteDist_ui16; //!< Maximum absolute distance to be travelled after which which PS is removed from PSM Buffer Note Absol
  SInt16 MaxYPos_si16; //!< Maximum permitted vehicle position y in relation to the parking spaceafter which PS is removed from
  SInt16 MinYPos_si16; //!< Minimum permitted vehicle position y in relation to the parking spaceafter which PS is removed from
  UInt16 MaxPhi_ui16; //!< Maximum possible Vehicle angle Applicable for x Veh larger than ThresholdPhiX after which PS is remo
  UInt16 MinPhi_ui16; //!< Minimum possible Vehicle angle Applicable for x Veh less than ThresholdPhiX after which PS is remove
  UInt16 MaxPhiB4ThrshldX_ui16; //!< Maximum possible Vehicle angle Applicable for x Veh less than ThresholdPhiX after which PS is remove
  UInt16 MinPhiB4ThrshldX_ui16; //!< Minimum possible Vehicle angle Applicable for x Veh less than ThresholdPhiX after which PS is remove
  SInt16 ThresholdPhiX_si16; //!< Beyond x Veh of ThresholdPhiX Angles MaxPhi and MinPhi are applicable and for x values less than Thr
}
gType_parPSM_DiscardCriteria_st; //!< copydoc Type_parPSM_DiscardCriteria_st


//! parameter cluster PSM_DiscardCriteriaCPSC: Discarding Parameters
typedef struct Type_parPSM_DiscardCriteriaCPSC_st
{
  SInt16 MaxSPos_si16; //!< Maximum distance between parking space measured from x 0 and the permitted distance to drive on in r
  SInt16 MinSPos_si16; //!< Minimum distance between parking space measured from x 0 and the permitted distance to drive back in
  SInt16 MaxSPosRearRef_si16; //!< Maximum distance between the parking space measured from first corner and the permitted distance to
  SInt16 MinSPosRearRef_si16; //!< Minimum distance between the parking space measured from first corner and the permitted distance to
  UInt16 MaxAbsoluteDist_ui16; //!< Maximum absolute distance to be travelled after which PS is removed from PSM Buffer Note Absolute di
  SInt16 MaxYPos_si16; //!< Maximum permitted vehicle position y in relation to the parking space after which PS is removed from
  SInt16 MinYPos_si16; //!< Minimum permitted vehicle position y in relation to the parking spaceafter which PS is removed from
  UInt16 MaxPhi_ui16; //!< Maximum possible Vehicle angle Applicable for x Veh larger than ThresholdPhiX after which PS is remo
  UInt16 MinPhi_ui16; //!< Minimum possible Vehicle angle Applicable for x Veh less than ThresholdPhiX after which PS is remove
  UInt16 MaxPhiB4ThrshldX_ui16; //!< Maximum possible Vehicle angle Applicable for x Veh less than ThresholdPhiX after which PS is remove
  UInt16 MinPhiB4ThrshldX_ui16; //!< Minimum possible Vehicle angle Applicable for x Veh less than ThresholdPhiX after which PS is remove
  SInt16 ThresholdPhiX_si16; //!< Beyond x Veh of ThresholdPhiX Angles MaxPhi and MinPhi are applicable and for x values less than Thr
  UInt16 MaxAbsoluteDistOMA_ui16; //!< Maximum absolute distance to be travelled for Parking spaces from OMA after which the PS is removed
}
gType_parPSM_DiscardCriteriaCPSC_st; //!< copydoc Type_parPSM_DiscardCriteriaCPSC_st


//! parameter cluster PSM_ParkableCriteria: Parkable Criteria Parameters
typedef struct Type_parPSM_ParkableCriteria_st
{
  SInt16 MaxYPos_si16; //!< Maximum permitted vehicle position y in relation to the parking space after which PS will be reset t
  SInt16 MinYPos_si16; //!< Minimum permitted vehicle position y in relation to the parking space after which PS will be reset t
  SInt16 MaxXPos_si16; //!< Maximum vehicle position x between parking space measured from x 0 and the permitted distance to dri
  SInt16 MaxSPos_si16; //!< Maximum distance between parking space measured from x 0 and the permitted distance to drive on in r
  SInt16 ResXPos_si16; //!< This Parameter is used as restricted XPosition When two consequent parking spaces are detected then
  SInt16 MinXPos_si16; //!< Minimum vehicle position x between parking space measured from x 0 and the permitted distance to dri
  UInt16 MaxPhi_ui16; //!< Maximum possible Vehicle angle after which PS will be reset to PSOk by PSM if one of the conditions
  UInt16 MinPhi_ui16; //!< Minimum possible Vehicle angle after which PS will be reset to PSOk by PSM if one of the conditions
  SInt16 MaxSWA_si16; //!< Maximum possible SWA after which PS will be reset to PSOk by PSM if one of the conditions applies No
}
gType_parPSM_ParkableCriteria_st; //!< copydoc Type_parPSM_ParkableCriteria_st


//! parameter cluster PSM_ParkableCriteriaCPSC: Parkable Criteria Parameters
typedef struct Type_parPSM_ParkableCriteriaCPSC_st
{
  SInt16 MaxYPos_si16; //!< Maximum permitted vehicle position y in relation to the parking space after which PS will be reset t
  SInt16 MinYPos_si16; //!< Minimum permitted vehicle position y in relation to the parking space after which PS will be reset t
  SInt16 MaxXPos_si16; //!< Maximum distance between parking space measured from x 0 and the permitted distance to drive on in r
  SInt16 MaxSPos_si16; //!< Maximum distance between parking space measured from x 0 and the permitted distance to drive on in r
  SInt16 ResXPos_si16; //!< This Parameter is used as restricted XPosition When two consequent parking spaces are detected then
  SInt16 MinXPos_si16; //!< Minimum distance between parking space measured from x 0 and the permitted distance to drive back in
  UInt16 MaxPhi_ui16; //!< Maximum possible Vehicle angle after which PS will be reset to PSOk by PSM if one of the conditions
  UInt16 MinPhi_ui16; //!< Minimum possible Vehicle angle after which PS will be reset to PSOk by PSM if one of the conditions
  SInt16 MaxSWA_si16; //!< Maximum possible SWA after which PS will be reset to PSOk by PSM if one of the conditions applies No
}
gType_parPSM_ParkableCriteriaCPSC_st; //!< copydoc Type_parPSM_ParkableCriteriaCPSC_st


//! parameter cluster PSM_FilterCriteria: Filter Criteria Parameters
typedef struct Type_parPSM_FilterCriteria_st
{
  UInt8 MinParkable_ui8; //!< Only parking spaces meeting this criteria are processed in the PSM memory others are filtered AllPS
}
gType_parPSM_FilterCriteria_st; //!< copydoc Type_parPSM_FilterCriteria_st


//! parameter cluster PSM_ParametersDPSC: Parameters Diagonal Parking
typedef struct Type_parPSM_ParametersDPSC_st
{
  SInt8 DiagonalPSAngle_si8; //!< Parking Slot Angle for Diagonal PS in degrees
  UInt8 DiagonalXOffset_ui8; //!< X Offset to be shifted after transformation This is specific to the pf assumption of dPSC Model
  UInt16 DiagonalMinLen_ui16; //!< This is Parkable criteria for dPSC In addition to Parkable criteria of cPSC Minimum Length of the tr
}
gType_parPSM_ParametersDPSC_st; //!< copydoc Type_parPSM_ParametersDPSC_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- MP                                                                     -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_M04           exported data types
 * \ingroup COMPONENT_MP
 @{ */

//! parameter cluster MP_SensorTimes: Sensor time configuration
typedef struct Type_parMP_SensorTimes_st
{
  UInt16 mpMaxMeasTime_ui16; //!< max measurement distance based time value the USS shall be active for echo reception
  UInt16 mpStopTimeVerifyShot_ui16; //!< re fire border distance based time value MP is triggering re fire shot if echo runtime measured is b
}
gType_parMP_SensorTimes_st; //!< copydoc Type_parMP_SensorTimes_st


//! parameter cluster MP_SensorStatic: USS configuration properties
typedef struct Type_parMP_SensorStatic_st
{
  UInt16 DeIdleTime_ui16; //!< Direct echo idle time MUST not be identical difference of at least 2??s required for sensors in the s
  UInt16 CeInnerIdleTime_ui16; //!< Cross echo idle time inner direction MUST not be identical difference of at least 2??s required for s
  UInt16 CeOuterIdleTime_ui16; //!< Cross echo idle time outer direction MUST not be identical difference of at least 2??s required for s
}
gType_parMP_SensorStatic_st; //!< copydoc Type_parMP_SensorStatic_st


//! parameter cluster MP_PdcMeasGroupParId: PDC group configuration
typedef struct Type_parMP_PdcMeasGroupParId_st
{
  UInt8 mpFrontGroupId_ui8; //!< PDC function only Id of the sensor group definition table that shall be used in measurement by the M
  UInt8 mpRearGroupId_ui8; //!< PDC function only Id of the sensor group definition table that shall be used in measurement by the M
}
gType_parMP_PdcMeasGroupParId_st; //!< copydoc Type_parMP_PdcMeasGroupParId_st


//! parameter cluster MP_Timings: Status and Disturbance timing configuration
typedef struct Type_parMP_Timings_st
{
  SInt16 mpSensStatusCheckTime_si16; //!< Time duration between 2 status check requests sent to USS from MP
  SInt16 mpSensPwrCheckTime_si16; //!< Time between check sensor power voltage when mp degradation is SENS_PWR_OFF_HDL
  SInt16 mpSensTempReadTime_si16; //!< Time between read sensor temperature
  SInt16 mpSensOSCFreqCheckTime_si16; //!< Time between check sensor OSC frequence
  UInt8 mpDisturbWaitTime_ui8; //!< additional wait time between measurement shots
}
gType_parMP_Timings_st; //!< copydoc Type_parMP_Timings_st


//! parameter cluster MP_SensorType: Sensor ADI number to ECU ASIC Port assignment
typedef struct Type_parMP_SensorType_st
{
  UInt8 mpSensorPortType_pui8[14]; //!< USS Gen5 type Value 0 none 1 48 khz 2 55 khz
}
gType_parMP_SensorType_st; //!< copydoc Type_parMP_SensorType_st


//! parameter cluster MP_SensorOrientation: Sensor ADI number to ECU ASIC Port assignment
typedef struct Type_parMP_SensorOrientation_st
{
  UInt8 mpSensorPortOrien_pui8[14]; //!< USS view direction vehicle mounting position dependent view direction of a USS Values 0 viewToFront
}
gType_parMP_SensorOrientation_st; //!< copydoc Type_parMP_SensorOrientation_st


//! parameter cluster MP_RingTimeInfor: Parameter for the PSx temperature hysteresis
typedef struct Type_parMP_RingTimeInfor_st
{
  UInt8 NumCcSets_ui8; //!< number of different Ring Time Range
  UInt8 DefaultIdx_ui8; //!< ID of default Ring Time Range
  SInt8 TempRange_psi8[4]; //!< See above
  SInt8 TempHyst_si8; //!< Parameter for the PSx temperature hysteresis
}
gType_parMP_RingTimeInfor_st; //!< copydoc Type_parMP_RingTimeInfor_st


//! parameter cluster MP_RingTimeRange: no valid cluster description available
typedef struct Type_parMP_RingTimeRange_st
{
  UInt16 DataRange_pui16[4]; //!<  48 khz Sensor Ring time lower limint
}
gType_parMP_RingTimeRange_st; //!< copydoc Type_parMP_RingTimeRange_st


//! parameter cluster MP_SensorPort: Sensor ADI number to ECU ASIC Port assignment
typedef struct Type_parMP_SensorPort_st
{
  UInt8 mpSensorPortID01_ui8; //!< configuration which sensor port ID belongs to which ADI sensor system ID
  UInt8 mpSensorPortID02_ui8; //!< configuration which sensor port ID belongs to which ADI sensor system ID
  UInt8 mpSensorPortID03_ui8; //!< configuration which sensor port ID belongs to which ADI sensor system ID
  UInt8 mpSensorPortID04_ui8; //!< configuration which sensor port ID belongs to which ADI sensor system ID
  UInt8 mpSensorPortID05_ui8; //!< configuration which sensor port ID belongs to which ADI sensor system ID
  UInt8 mpSensorPortID06_ui8; //!< configuration which sensor port ID belongs to which ADI sensor system ID
  UInt8 mpSensorPortID07_ui8; //!< configuration which sensor port ID belongs to which ADI sensor system ID
  UInt8 mpSensorPortID08_ui8; //!< configuration which sensor port ID belongs to which ADI sensor system ID
  UInt8 mpSensorPortID09_ui8; //!< configuration which sensor port ID belongs to which ADI sensor system ID
  UInt8 mpSensorPortID10_ui8; //!< configuration which sensor port ID belongs to which ADI sensor system ID
  UInt8 mpSensorPortID11_ui8; //!< configuration which sensor port ID belongs to which ADI sensor system ID
  UInt8 mpSensorPortID12_ui8; //!< configuration which sensor port ID belongs to which ADI sensor system ID
  UInt8 mpSensorPortID13_ui8; //!< configuration which sensor port ID belongs to which ADI sensor system ID
  UInt8 mpSensorPortID14_ui8; //!< configuration which sensor port ID belongs to which ADI sensor system ID
}
gType_parMP_SensorPort_st; //!< copydoc Type_parMP_SensorPort_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- MP_BUFF                                                                -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_BUFF_M04           exported data types
 * \ingroup COMPONENT_MP_BUFF
 @{ */

/*!@} */


/*--------------------------------------------------------------------------*/
/*- MP_PL                                                                  -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_PL_M04           exported data types
 * \ingroup COMPONENT_MP_PL
 @{ */

/*!@} */


/*--------------------------------------------------------------------------*/
/*- MTL                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MTL_M04           exported data types
 * \ingroup COMPONENT_MTL
 @{ */

/*!@} */


/*--------------------------------------------------------------------------*/
/*- OD                                                                     -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup OD_M04           exported data types
 * \ingroup COMPONENT_OD
 @{ */

//! parameter cluster OD_General: EP_OD GENERAL PARAMETERS
typedef struct Type_parOD_General_st
{
  UInt8 NumCyclesStartup_ui8; //!< number of complete sensor transceiving cycles for start up phase recommended 0 2 EP_PAS indicates af
  UInt8 LowSpeed_ui8; //!< speed which is considered as low This speed is used in the following cases 1 relevant for fast filte
}
gType_parOD_General_st; //!< copydoc Type_parOD_General_st


//! parameter cluster OD_MaskActVelo: Thresholds for changing the mask table according the vehicle speed
typedef struct Type_parOD_MaskActVelo_st
{
  UInt16 odMaskActVeloPdcExclusive_decel_ui16; //!< vehicle speed to switch in Pdc exclusive mode coming from a higher vehicle speed
  UInt16 odMaskActVeloPdcExtended_accel_ui16; //!< vehicle speed to switch in Pdc extended mode coming from a lower vehicle speed
  UInt16 odMaskActVeloPdcExtended_decel_ui16; //!< vehicle speed to switch in Pdc extended mode coming from a higher vehicle speed
  UInt16 odMaskActVeloPdcCornerFront_accel_ui16; //!< vehicle speed to switch in Pdc corner front mode coming from a lower vehicle speed
  UInt16 odMaskActVeloPdcCornerFront_decel_ui16; //!< vehicle speed to switch in Pdc corner front mode coming from a higher vehicle speed
  UInt16 odMaskActVeloPsxBsd_accel_ui16; //!< vehicle speed to switch in PsxBsd mode coming from a lower vehicle speed
  UInt16 odMaskActVeloPsxBsd_decel_ui16; //!< vehicle speed to switch in PsxBsd mode coming from a higher vehicle speed
  UInt16 odMaskActVeloBsdExclusive_accel_ui16; //!< vehicle speed to switch in Bsd exclusive mode coming from a lower vehicle speed
}
gType_parOD_MaskActVelo_st; //!< copydoc Type_parOD_MaskActVelo_st


//! parameter cluster OD_IdleDist: EP_OD ECHO IDLE DISTANCE PARAMETERS
typedef struct Type_parOD_IdleDist_st
{
  UInt16 IdleDistDeFront_pui16[6]; //!< idle distances of direct and cross echo values for front and rear vehicle side In case of entering a
  UInt16 IdleDistCeFront_pui16[5]; //!< See above
  UInt16 IdleDistDeRear_pui16[6]; //!< See above
  UInt16 IdleDistCeRear_pui16[5]; //!< See above
}
gType_parOD_IdleDist_st; //!< copydoc Type_parOD_IdleDist_st


//! parameter cluster OD_Sens: EP_OD SENSOR CONFIG PARAMETERS
typedef struct Type_parOD_Sens_st
{
  UInt16 SensDistFront_pui16[5]; //!< Distance values between two neighbouring sensors of a vehicle side The number of parameters are desi
  UInt16 SensDistRear_pui16[5]; //!< See above
  SInt16 SensPosXFront_psi16[6]; //!<  Sensor coordinate values related to the middle point of rear axle Orientation of coordinate system
  SInt16 SensPosYFront_psi16[6]; //!< See above
  SInt16 SensPosZFront_psi16[6]; //!< See above
  SInt16 SensPosXRear_psi16[6]; //!< See above
  SInt16 SensPosYRear_psi16[6]; //!< See above
  SInt16 SensPosZRear_psi16[6]; //!< See above
  SInt16 SensBeamDirFront_psi16[3]; //!<  Sensor beam values SensBeamDir angle of the sensor direction relative to the vehicle coordinate sys
  SInt16 SensBeamWidthFront_psi16[3]; //!< See above
  SInt16 SensBeamWidth4DeplausWall_psi16[3]; //!< See above
  SInt16 SensBeamWidth4DeplausPnt_psi16[3]; //!< See above
  UInt16 SensBeamDist4DeplausMajorObj_ui16; //!< See above
  UInt16 SensBeamDist4DeplausMinorObj_ui16; //!< See above
  UInt16 SensBeamDistMin4CurbstoneFront_pui16[3]; //!< Minimal Distance of the echo for which curbstones can be detected with this sensor Be sure that ther
  UInt16 SensBeamDistMin4CurbstoneRear_pui16[3]; //!< See above
  UInt16 SensBeamDistFront_pui16[3]; //!< See above
  SInt16 SensBeamDirRear_psi16[3]; //!< See above
  SInt16 SensBeamWidthRear_psi16[3]; //!< See above
  UInt16 SensBeamDistRear_pui16[3]; //!< See above
}
gType_parOD_Sens_st; //!< copydoc Type_parOD_Sens_st


//! parameter cluster OD_Veh: EP_OD VEHICLE CONFIG PARAMETERS
typedef struct Type_parOD_Veh_st
{
  UInt16 VehWidthHalf_ui16; //!< half vehicle width
  UInt8 NumberContourPntsFront_ui8; //!< number of configured contour points for front bumper
  UInt8 NumberContourPntsRear_ui8; //!< number of configured contour points for rear bumper
  UInt8 NumberContourPntsSide_ui8; //!< number of configured contour points for vehicle side
  SInt16 VehContourFrontX_psi16[6]; //!< vehicle contour X coordinates front related to middle point rear axle mm
  SInt16 VehContourFrontY_psi16[6]; //!< vehicle contour Y coordinates front related to middle point rear axle mm
  SInt16 VehContourRearX_psi16[6]; //!< vehicle contour X coordinates rear related to middle point rear axle mm
  SInt16 VehContourRearY_psi16[6]; //!< vehicle contour Y coordinates rear related to middle point rear axle mm
  SInt16 VehContourSideX_psi16[5]; //!< vehicle contour X coordinates side related to middle point rear axle mm
  SInt16 VehContourSideY_psi16[5]; //!< vehicle contour Y coordinates side related to middle point rear axle mm
}
gType_parOD_Veh_st; //!< copydoc Type_parOD_Veh_st


//! parameter cluster OD_Echo_ApFlt: EP_OD ECHO APPEARANCE FILTER PARAMETERS
typedef struct Type_parOD_Echo_ApFlt_st
{
  UInt16 ApFltEchoDelta1_ui16; //!<  definition of the jump distance of a raw echo that should be filtered a smaller jump as distance de
  UInt16 ApFltEchoDelta2_ui16; //!< See above
  UInt16 ApFltEchoDelta3_ui16; //!< See above
  UInt16 ApFltEchoDeltaCe_ui16; //!< See above
  UInt16 ApFltEchoDelta1KMH_ui16; //!< increases the delta per 1km/h step by a factor of ApFltEchoDelta1KMH mm note changes in this configu
  UInt16 EchoDeltaOfMaxSpeed_ui16; //!< above low speed the velocity depending additional delta is limited to ApFltEchoDeltaOfMaxSpeed note
  UInt16 ApFltEchoZone1Dist_ui16; //!< define distance zones for filtering The number of penalty rounds is added to the delta penalty round
  UInt16 ApFltEchoZone2Dist_ui16; //!< See above
  UInt16 ApFltEchoZone3Dist_ui16; //!< See above
  UInt16 ApFltEchoZone4Dist_ui16; //!< See above
  UInt8 ApFltEchoDelta1Penalty_ui8; //!< definition number of filter rounds Filter action 1 Number of penalty cycles if corresponding filter
  UInt8 ApFltEchoDelta2Penalty_ui8; //!< See above
  UInt8 ApFltEchoDelta3Penalty_ui8; //!< See above
  UInt8 ApFltEchoZone1Penalty_ui8; //!< See above
  UInt8 ApFltEchoZone2Penalty_ui8; //!< See above
  UInt8 ApFltEchoZone3Penalty_ui8; //!< See above
  UInt8 ApFltEchoZone4Penalty_ui8; //!< See above
  UInt8 ApFltEchoZone5Penalty_ui8; //!< See above
  UInt8 ApFltEchoZone6Penalty_ui8; //!< See above
}
gType_parOD_Echo_ApFlt_st; //!< copydoc Type_parOD_Echo_ApFlt_st


//! parameter cluster OD_Echo_WRF: EP_OD ECHO WHEEL REFLECTION FILTER PARAMETERS
typedef struct Type_parOD_Echo_WRF_st
{
  Boolean WRFEnable_bl; //!< Enable/disable wheel reflection filter functionality The wheel reflection filter functionality can b
  UInt16 WRFWheelAngLimit_ui16; //!< Minimum wheel angle for identification of echo reflections from the front wheels
  UInt16 WRFWheelOutwardDeLowerLimit_ui16; //!< lower limit of echo range of potential reflections on outwards turned front wheel tire profile
  UInt16 WRFWheelOutwardDeUpperLimit_ui16; //!< upper limit of echo range of potential reflections on outwards turned front wheel tire profile
  UInt16 WRFWheelInwardDeLowerLimit_ui16; //!< lower limit of echo range of potential reflections on inwards turned front wheel tire profile
  UInt16 WRFWheelInwardDeUpperLimit_ui16; //!< upper limit of echo range of potential reflections on inwards turned front wheel tire profile
}
gType_parOD_Echo_WRF_st; //!< copydoc Type_parOD_Echo_WRF_st


//! parameter cluster OD_Echo_SRF: EP_OD ECHO SELF REFLECTION FILTER PARAMETERS
typedef struct Type_parOD_Echo_SRF_st
{
  Boolean SRFEnable_pbl[2]; //!< Enable/disable self reflection filter functionality The self reflection filter functionality can be
  UInt16 SRFDeLimit_pui16[2]; //!< Direct echo and cross echo limits for self reflection filter DeLimit threshold for direct echo value
  UInt16 SRFCeLimit_pui16[2]; //!< See above
}
gType_parOD_Echo_SRF_st; //!< copydoc Type_parOD_Echo_SRF_st


//! parameter cluster OD_Echo_HLF: EP_OD ECHO HEAT LAYER FILTER PARAMETERS
typedef struct Type_parOD_Echo_HLF_st
{
  Boolean HLFEnable_pbl[2]; //!< Enable/disable heat layer filter functionality The heat layer filter functionality can be enabled fo
  UInt16 HLFCe23RangeMin_pui16[2]; //!<  minimum / maximum distance range for Ce23 In this range the cross echo can be a reflection of the l
  UInt16 HLFCe23RangeMax_pui16[2]; //!< See above
  UInt16 HLFDe2RangeMin_pui16[2]; //!< See above
  UInt16 HLFDe2RangeMax_pui16[2]; //!< See above
  UInt16 HLFCe23Limit_pui16[2]; //!< See above
  UInt16 HLFDe2Limit_pui16[2]; //!< See above
  UInt16 HLFCe12Limit_pui16[2]; //!< See above
  UInt16 HLFDe3LimitForCe23_pui16[2]; //!< See above
  UInt16 HLFDe3LimitForDe2_pui16[2]; //!< See above
  UInt8 HLFSpeedLim_ui8; //!< activation of heat layer filter only if speed configured value
}
gType_parOD_Echo_HLF_st; //!< copydoc Type_parOD_Echo_HLF_st


//! parameter cluster OD_Echo_MoveDir: EP_OD ECHO MOVE DIRECTION PARAMETERS
typedef struct Type_parOD_Echo_MoveDir_st
{
  UInt16 RelMovingDeTol_ui16; //!< desired minimum distance change of echo values for detection of a relative movement usually RelMovin
  UInt16 RelMovingCeTol_ui16; //!< desired minimum distance change of echo values for detection of a relative movement usually RelMovin
  UInt8 MaxCyclMovingHold_ui8; //!< Maximum number of cycles the stationary moving direction is hold
}
gType_parOD_Echo_MoveDir_st; //!< copydoc Type_parOD_Echo_MoveDir_st


//! parameter cluster OD_Echo_Clip_Std: EP_OD ECHO CLIPPING PARAMETERS for Standard Approach
typedef struct Type_parOD_Echo_Clip_Std_st
{
  UInt16 FrontDeLimit_pui16[6]; //!< clipping values for direct and cross echos limitation of the echo values for all sensors and respect
  UInt16 FrontCeLimit_pui16[5]; //!< clipping values for direct and cross echos limitation of the echo values for all sensors and respect
  UInt16 RearDeLimit_pui16[6]; //!< clipping values for direct and cross echos limitation of the echo values for all sensors and respect
  UInt16 RearCeLimit_pui16[5]; //!< clipping values for direct and cross echos limitation of the echo values for all sensors and respect
  UInt16 ClippingHyst_ui16; //!< clipping hysteresis
}
gType_parOD_Echo_Clip_Std_st; //!< copydoc Type_parOD_Echo_Clip_Std_st


//! parameter cluster OD_Obj_Meas: EP_OD MEAS OBJECT PROCESSING PARAMETERS
typedef struct Type_parOD_Obj_Meas_st
{
  UInt16 CriticalDist_ui16; //!< Project specific critical distance below this distance measured echoes can become uncertain double r
  UInt16 CriticalDistHyst_ui16; //!< Project specific critical distance below this distance measured echoes can become uncertain double r
  UInt16 ObjTypeDeltaDist_ui16; //!< Maintain previous object type if the distance change is smaller than this configuration value tracki
  UInt16 OblFiltHystNear_ui16; //!< hysteresis for leaving the object filtering distance region an object which is seen by the neighbour
  UInt16 OblFiltHystFar_ui16; //!< hysteresis for leaving the object filtering distance region an object which is seen by the neighbour
}
gType_parOD_Obj_Meas_st; //!< copydoc Type_parOD_Obj_Meas_st


//! parameter cluster OD_Obj_Trck: EP_OD TRACKED OBJECT PARAMETERS
typedef struct Type_parOD_Obj_Trck_st
{
  Boolean EnableTrackEndless_bl; //!< Enable/disable endless tracking duration If flag is enabled objects with a minimum ExistProb of Exis
  UInt16 TrackCycleMax_ui16; //!< maximum number of cycles an OD object can be tracked
  UInt16 TrackCycleMaxWall_ui16; //!< maximum number of cycles an OD object can be tracked
  UInt16 TrackCycleMin_ui16; //!< minimum number of cycles an OD object will be tracked
  UInt16 TrackCycleWait4Deplaus_ui16; //!< waiting cycles for allowed ExistProb decreasing after ExistProb increase
  UInt16 TrackDistMax_ui16; //!< maximum allowed object tracking distance
  UInt16 TrackVehDistFront_ui16; //!< object tracking area for objects in front of the vehicle OD objetct located in front of the vehicle
  UInt16 TrackVehDistSide_ui16; //!< object tracking area for objects beside the vehicle OD objetct located beside the vehicle will be tr
  UInt16 RelevanceRange4HighLevelObjFront_ui16; //!< object tracking area for high level objects in front of the vehicle High level OD objetct located in
  UInt16 RelevanceRange4HighLevelObjRear_ui16; //!< object tracking area for high level objects behind the vehicle High level OD objetct located behind
  UInt16 RelevanceRange4HighLevelObjSide_ui16; //!< object tracking area for high level objects beside the vehicle High level OD objetct located beside
  UInt16 TrackVehDistMin_ui16; //!< mimimum distance for object tracking OD objects with a lower distance than the specified value will
  UInt16 MatchWindowInit_ui16; //!< matching window initial value mm
  UInt16 MatchWindowMax_ui16; //!< matching window maximum value mm
  UInt16 MatchWindowTrackCycleFactor_ui16; //!< factor for increasing matching window depending on the tracking cycles mm / cycle
  UInt16 MatchWindowTrackDistFactor_ui16; //!< factor for increasing matching window depending on the tracked distance 1/1024
  UInt16 MatchWallTolX_ui16; //!< maximum allowed vertical distance of new measured object to a tracked wall for matching
  UInt16 MatchWallTolXCeObj_ui16; //!< maximum allowed vertical distance of Ce object to a tracked wall for matching
  UInt16 CosAngleToleranceWall_ui16; //!< maximum allowed angle between the two walls for matching
  UInt8 ExistProbMin_ui8; //!< starting value of object existance probability in case of first detection 1/256
  UInt8 ExistProbLimFrontObj4MaxTrck_ui8; //!< above this limit of ExistProb front objects will be tracked up to the configured maximum tracking du
  UInt8 ExistProbLimSideObj4MaxTrck_ui8; //!< above this limit of ExistProb side objects will be tracked up to the configured maximum tracking dur
  UInt8 ExistProbMatchFactor_ui8; //!< factor for increasing of the object existance probabilty 1/16 in case of position matching with curr
  UInt8 ExistProbMatchSlowFactor_ui8; //!< factor for increasing of the object existance probabilty 1/16 in case of position matching with curr
  UInt8 ExistProbDecFactor_ui8; //!< factor for decreasing of the object existance probabilty 1/16 in case that the object is located ins
  UInt8 ExistProbDecFastFactor_ui8; //!< factor for decreasing of the existance probabilty 1/16 for objects with obviously unreliable positio
  UInt8 ExistProbDecFast2Factor_ui8; //!< factor for decreasing of the existance probabilty 1/16 for objects with obviously implausible positi
  UInt8 ObjRelMax4ExistProbMin_ui8; //!< maximum possible object relevance in case of minimum existance probabiity 1/256
  UInt16 ObjMoveDeltaAngleMax_ui16; //!< maximum allowed difference between the current and the tracked object moving direction for detection
  UInt16 ObjMoveDeltaDistFak_ui16; //!< maximum allowed relative difference between the current and the tracked object moving distance for d
  UInt16 SensDistMin4WallType_ui16; //!< minimum sensor distance for generating a wall like measurement object a wall like object generated b
  UInt16 WallLengthMax_ui16; //!< max length of tracked wall object
  UInt16 WallLengthMax4AngleCorr_ui16; //!< max wall length for angle correction
  UInt16 WallLengthMax4PlausDec_ui16; //!< max wall length for allowed decreasing of ExistProb
  UInt16 DoubleEchoTol4HighObj_ui16; //!< allowed tolerance between expected double echo value and measured one
  UInt8 HeightClassificationLow_ui8; //!< threshold for object height probability to be classified as low object 1/256
  UInt8 HeightClassificationHigh_ui8; //!< threshold for object height probability to be classified as high object 1/256
}
gType_parOD_Obj_Trck_st; //!< copydoc Type_parOD_Obj_Trck_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- PAR                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup PAR_M04           exported data types
 * \ingroup COMPONENT_PAR
 @{ */

/*!@} */


/*--------------------------------------------------------------------------*/
/*- SP_FSV                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup SP_FSV_M04           exported data types
 * \ingroup COMPONENT_SP_FSV
 @{ */

/*!@} */


/*--------------------------------------------------------------------------*/
/*- SP_PWR                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup SP_PWR_M04           exported data types
 * \ingroup COMPONENT_SP_PWR
 @{ */

//! parameter cluster SP_PWR_CFG: SP_PWR configuration parameters
typedef struct Type_parSP_PWR_CFG_st
{
  UInt8 UbattMax_ui8; //!< Maximum permitted value for UBATT
  UInt8 UbattMaxHysteresis_ui8; //!< Amount of allowable hysteresis for maximum battery voltage
  UInt8 UbattMin_ui8; //!< Minimum permitted value for UBATT
  UInt8 UbattMinHysteresis_ui8; //!< Amount of allowable hysteresis for minimum battery voltage
}
gType_parSP_PWR_CFG_st; //!< copydoc Type_parSP_PWR_CFG_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- SMS                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup SMS_M04           exported data types
 * \ingroup COMPONENT_SMS
 @{ */

/*!@} */


/*--------------------------------------------------------------------------*/
/*- VHS_PL_TORQUE                                                          -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup VHS_PL_TORQUE_M04           exported data types
 * \ingroup COMPONENT_VHS_PL_TORQUE
 @{ */

//! parameter cluster VHS_PL_TORQUE_Controller_ZFLS: no valid cluster description available
typedef struct Type_parVHS_PL_TORQUE_Controller_ZFLS_st
{
  SInt32 PosPID_Kp_si32; //!< Proportional gain for the position PID controller Kp_physical Kp 2^ KpScale must be in the range 0 1
  SInt8 PosPID_KpScale_si8; //!< Scale factor for PosPID_Kp Kp_physical Kp 2^ KpScale
  SInt32 PosPID_Ki_si32; //!< Integral gain for the position PID controller Ki_physical Ki 2^ KiScale must be in the range 0 1 The
  SInt8 PosPID_KiScale_si8; //!< Scale factor for PosPID_Ki Ki_physical Ki 2^ KiScale
  SInt32 PosPID_Kd_si32; //!< Derivative gain for the position PID controller Kd_physical Kd 2^ KdScale must be in the range 0 1 T
  SInt8 PosPID_KdScale_si8; //!< Scale factor for PosPID_Kd Kd_physical Kd 2^ KdScale
  SInt32 PosPID_MaxIntegral_si32; //!< Upper limit for the value of the Integral part of the Position PID
  SInt8 PosPID_MaxIntegralScale_si8; //!< Scale for PosPID_MaxIntegral
  SInt32 SpeedPID_Kp_si32; //!< Proportional gain for the speed PID controller Kp_physical Kp 2^ KpScale must be in the range 0 1 Th
  SInt8 SpeedPID_KpScale_si8; //!< Scale factor for SpeedPID_Kp Kp_physical Kp 2^ KpScale
  SInt32 SpeedPID_Ki_si32; //!< Integral gain for the speed PID controller Ki_physical Ki 2^ KiScale must be in the range 0 1 The I
  SInt8 SpeedPID_KiScale_si8; //!< Scale factor for SpeedPID_Ki
  SInt32 SpeedPID_Kd_si32; //!< Derivative gain for the speed PID controller Kd_physical Kd 2^ KdScale must be in the range 0 1 The
  SInt8 SpeedPID_KdScale_si8; //!< Scale factor for SpeedPID_Kd
  SInt32 SpeedPID_MaxIntegral_si32; //!< Maximum Value for the Integral part of the speed PID
  SInt8 SpeedPID_MaxIntegralScale_si8; //!< Scale for SpeedPID_MaxIntegral
  SInt16 PosPIDscale_factor_si16; //!< A scaling factor that increases the influence of the position PID final speed request PosPIDscale_fa
  SInt32 maxSWASpeed_si32; //!< The maximum allowed SWA speed If the final speed request lies above this threshold it is reduced to
  SInt32 maxSWAAcc_si32; //!< The maximum allowed SWA acceleration If the final speed request has a gradient that lies above this
  SInt32 maxTorqueReq_si32; //!< The maximum allowed absolute torque request If the calculated torque from the controller lies above
  SInt8 maxTorqueReq_scale_si8; //!< scale factor for maxTorqueReq
  SInt32 maxTorqueSlewRate_si32; //!< The maximum allowed direvative of the torque request slew rate If the calcilated torque request from
  UInt8 outputFIR_size_ui8; //!< The number of element over which the output FIR filter filters The Software will only calculate the
  SInt32 outputFIR_coefficient0_si32; //!<  coefficient of the output FIR 0
  SInt32 outputFIR_coefficient1_si32; //!<  coefficient of the output FIR 1
  SInt32 outputFIR_coefficient2_si32; //!<  coefficient of the output FIR 2
  SInt32 outputFIR_coefficient3_si32; //!<  coefficient of the output FIR 3
  SInt32 outputFIR_coefficient4_si32; //!<  coefficient of the output FIR 4
  SInt32 outputFIR_coefficient5_si32; //!<  coefficient of the output FIR 5
  SInt32 outputFIR_coefficient6_si32; //!<  coefficient of the output FIR 6
  SInt32 outputFIR_coefficient7_si32; //!<  coefficient of the output FIR 7
  SInt32 outputFIR_coefficient8_si32; //!<  coefficient of the output FIR 8
  SInt32 outputFIR_coefficient9_si32; //!<  coefficient of the output FIR 9
  SInt8 outputFIR_coefficient_scaling_si8; //!< Scale of the output coeffitients
  UInt8 speed_reqFIR_size_ui8; //!< The number of element over which the speed_req FIR filter filters The Software will only calculate t
  SInt32 speed_reqFIR_coefficient0_si32; //!<  coefficient of the speed_req FIR 0
  SInt32 speed_reqFIR_coefficient1_si32; //!<  coefficient of the speed_req FIR 1
  SInt32 speed_reqFIR_coefficient2_si32; //!<  coefficient of the speed_req FIR 2
  SInt32 speed_reqFIR_coefficient3_si32; //!<  coefficient of the speed_req FIR 3
  SInt32 speed_reqFIR_coefficient4_si32; //!<  coefficient of the speed_req FIR 4
  SInt32 speed_reqFIR_coefficient5_si32; //!<  coefficient of the speed_req FIR 5
  SInt32 speed_reqFIR_coefficient6_si32; //!<  coefficient of the speed_req FIR 6
  SInt32 speed_reqFIR_coefficient7_si32; //!<  coefficient of the speed_req FIR 7
  SInt32 speed_reqFIR_coefficient8_si32; //!<  coefficient of the speed_req FIR 8
  SInt32 speed_reqFIR_coefficient9_si32; //!<  coefficient of the speed_req FIR 9
  SInt8 speed_reqFIR_coefficient_scaling_si8; //!< Scale of the speed_req coeffitients
  UInt8 speed_curFIR_size_ui8; //!< The number of element over which the speed_cur FIR filter filters The Software will only calculate t
  SInt32 speed_curFIR_coefficient0_si32; //!<  coefficient of the speed_cur FIR 0
  SInt32 speed_curFIR_coefficient1_si32; //!<  coefficient of the speed_cur FIR 1
  SInt32 speed_curFIR_coefficient2_si32; //!<  coefficient of the speed_cur FIR 2
  SInt32 speed_curFIR_coefficient3_si32; //!<  coefficient of the speed_cur FIR 3
  SInt32 speed_curFIR_coefficient4_si32; //!<  coefficient of the speed_cur FIR 4
  SInt32 speed_curFIR_coefficient5_si32; //!<  coefficient of the speed_cur FIR 5
  SInt32 speed_curFIR_coefficient6_si32; //!<  coefficient of the speed_cur FIR 6
  SInt32 speed_curFIR_coefficient7_si32; //!<  coefficient of the speed_cur FIR 7
  SInt32 speed_curFIR_coefficient8_si32; //!<  coefficient of the speed_cur FIR 8
  SInt32 speed_curFIR_coefficient9_si32; //!<  coefficient of the speed_cur FIR 9
  SInt8 speed_curFIR_coefficient_scaling_si8; //!< Scale of the speed_cur coeffitients
}
gType_parVHS_PL_TORQUE_Controller_ZFLS_st; //!< copydoc Type_parVHS_PL_TORQUE_Controller_ZFLS_st


//! parameter cluster VHS_PL_TORQUE_Controller_CR: no valid cluster description available
typedef struct Type_parVHS_PL_TORQUE_Controller_CR_st
{
  SInt32 B_si32; //!< Friction constant B
  SInt8 B_scale_si8; //!< Scale factor for B_si32 B_physical B 2^ BScale
  SInt32 Observer_a11_si32; //!< Element of observer Matrix A 1st row 1st collumn
  SInt8 Observer_a11_scale_si8; //!< Scale factor for OBSERVER_a11_si32 a11_physical a11 2^ a11Scale
  SInt32 Observer_a12_si32; //!< Element of observer Matrix A 1st row 2nd collumn
  SInt8 Observer_a12_scale_si8; //!< Scale factor for OBSERVER_a12_si32 a12_physical a12 2^ a12Scale
  SInt32 Observer_a13_si32; //!< Element of observer Matrix A 1st row 3rd collumn
  SInt8 Observer_a13_scale_si8; //!< Scale factor for OBSERVER_a13_si32 a13_physical a13 2^ a13Scale
  SInt32 Observer_a22_si32; //!< Element of observer staatrix A 2nd row 2nd collumn
  SInt8 Observer_a22_scale_si8; //!< Scale factor for OBSERVER_a22_si32 a22_physical a22 2^ a22Scale
  SInt32 Observer_a23_si32; //!< Element of observer Matrix A 2nd row 13rd collumn
  SInt8 Observer_a23_scale_si8; //!< Scale factor for OBSERVER_a23_si32 a23_physical a23 2^ a23Scale
  SInt32 Observer_a33_si32; //!< Element of observer Matrix A 3rd row 3rd collumn
  SInt8 Observer_a33_scale_si8; //!< Scale factor for OBSERVER_a33_si32 a33_physical a33 2^ a33Scale
  SInt32 Observer_b1_si32; //!< Element of observer Matrix B 1st row
  SInt8 Observer_b1_scale_si8; //!< Scale factor for OBSERVER_b1_si32 b1_physical b1 2^ b1Scale
  SInt32 Observer_b2_si32; //!< Element of observer Matrix B 2nd row
  SInt8 Observer_b2_scale_si8; //!< Scale factor for OBSERVER_b2_si32 b2_physical b2 2^ b2Scale
  SInt32 Observer_L1_si32; //!< 1st element of observer Gain L
  SInt8 Observer_L1_scale_si8; //!< Scale factor for VHS_pl_tq_L1_si32 L1_physical L1 2^ L1Scale
  SInt32 Observer_L2_si32; //!< 2nd element of observer Gain L
  SInt8 Observer_L2_scale_si8; //!< Scale factor for VHS_pl_tq_L2_si32 L2_physical L2 2^ L2Scale
  SInt32 Observer_L3_si32; //!< 3rd element of observer Gain L
  SInt8 Observer_L3_scale_si8; //!< Scale factor for VHS_pl_tq_L3_si32 L3_physical L3 2^ L3Scale
  SInt32 K1_si32; //!< 1st element of state feedback Gain K
  SInt8 K1_scale_si8; //!< Scale factor for VHS_pl_tq_K1_scale_si8 K1_physical K1 2^ K1Scale
  SInt32 K2_si32; //!< 2nd element of state feedback Gain K
  SInt8 K2_scale_si8; //!< Scale factor for VHS_pl_tq_K2_scale_si8 K2_physical K2 2^ K2Scale
  SInt32 ratio_collumn_motor_si32; //!< The transmission ratio from the EPS collumn to the EPS motor provided by the EPS supplier
  SInt8 ratio_collumn_motor_scale_si8; //!< Scale factor for the transmission ratio
}
gType_parVHS_PL_TORQUE_Controller_CR_st; //!< copydoc Type_parVHS_PL_TORQUE_Controller_CR_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- VHS                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup VHS_M04           exported data types
 * \ingroup COMPONENT_VHS
 @{ */

/*!@} */


/*--------------------------------------------------------------------------*/
/*- AP_BSD                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup AP_BSD_M04           exported data types
 * \ingroup COMPONENT_AP_BSD
 @{ */

//! parameter cluster AP_BSD_Ap: Section headline only visual structuring
typedef struct Type_parAP_BSD_Ap_st
{
  UInt16 SteWheAngMax_ui16; //!< Maximum steering wheel angle where warnings are issued
  UInt16 MinWarnTime_ui16; //!< Minimum warning time
  UInt16 OvertakenCarSuppressionTime_ui16; //!< Time before a warning will be issued for an overtaken car if speed is equal or lower than OvertaCarS
  UInt16 OvertakenCarSuppressionTime2_ui16; //!< Time before a warning will be issued for an overtaken car if speed is higher than OvertaCarSpeedSwit
  UInt16 OvertakenCarSuppressionTime2Mid_ui16; //!< Time before a warning will be issued for an overtaken car if speed is higher than OvertaCarSpeedSwit
  UInt8 OvertaCarSpeedSwitch_ui8; //!< If speed is greater than this define the value OvertakenCarSuppressionTime2 is used instead of Overt
  UInt16 AdditionalWarnTime_ui16; //!< After an object has left the blind spot warning is still issued for this amount of time
  UInt16 SensorCoverageDistance_ui16; //!< Coverage distance of BSD sensors
  UInt16 SuppressStartup_ui16; //!< Suppression distance after starting up
  UInt16 OvertakenCarSuppressionTimeMid_ui16; //!< Time before a warning will be issued for an overtaken car if speed is equal or lower than OvertaCarS
  UInt16 MidSensorWarnDistLowSpeed_ui16; //!< If a target vehicle gets close to the middle sensorsr than this distance a warning will be issued Di
  UInt16 MidSensorWarnDistHighSpeed_ui16; //!< If a target vehicle gets closer to the middle sensors than this distance a warning will be issued Di
}
gType_parAP_BSD_Ap_st; //!< copydoc Type_parAP_BSD_Ap_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- EP_BSD                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup EP_BSD_M04           exported data types
 * \ingroup COMPONENT_EP_BSD
 @{ */

//! parameter cluster EP_BSD_Ep: no valid cluster description available
typedef struct Type_parEP_BSD_Ep_st
{
  UInt8 CharTableOffsetFrontLow_ui8; //!< fixed offset on first char ID for front sensors for temperatures CharTempThresholdLow used after ini
  UInt8 CharTableOffsetRearLow_ui8; //!< fixed offset on first char ID for rear sensors for temperatures CharTempThresholdLow used after init
  UInt8 CharTableOffsetFrontMid_ui8; //!< fixed offset on first char ID for front sensors for temperatures CharTempThresholdLow CharTempThresh
  UInt8 CharTableOffsetRearMid_ui8; //!< fixed offset on first char ID for rear sensors for temperatures CharTempThresholdLow CharTempThresho
  UInt8 CharTableOffsetFrontHigh_ui8; //!< fixed offset on first char ID for front sensors for temperatures CharTempThresholdHigh used after in
  UInt8 CharTableOffsetRearHigh_ui8; //!< fixed offset on first char ID for rear sensors for temperatures CharTempThresholdHigh used after ini
  UInt8 CharTableMaxFrontAndRear_ui8; //!< maximum char ID for used for front and rear sensors Hint parameter is used for front rear because Cc
  SInt8 CharTempThresholdLow_si8; //!< lower temperature threshold used for offset on first char ID for the sensors
  SInt8 CharTempThresholdHigh_si8; //!< higher temperature threshold used for offset on first char ID for the sensors
  Boolean BlockedSensorTestAtStartUp_bl; //!< 0 false not activated 1 true perform blocked sensor detection when system returns from standby
  Boolean BlockedSensorTestAtMeasure_bl; //!< 0 false not activated 1 true perform blocked sensor detection when BSD is active and no object is se
  UInt32 BlockedSensorTime_ui32; //!< Blocked sensor test will be performed if no object has been detected within this time
  UInt16 BlockedSensorTestDuration_ui16; //!< Maximum duration of proceeding blocked sensor test below StayInTestVelocityMin km/h Next blocked sen
  UInt8 StayInTestVelocityMin_ui8; //!< Below this speed blocked sensor test is interrupted after BlockedSensorTestDuration ms and will star
  SInt8 BlockedSensorTestTempLimit_si8; //!< Blocked sensor test is only executed below this temperature Use a conveniently high value e g 100??C
  UInt8 HighSpeedTrackingAc_ui8; //!< Start with high speed tracking if speed is above this limit for acceleration
  UInt8 HighSpeedTrackingDe_ui8; //!< Start with high speed tracking if speed is above this limit for deceleration
  SInt8 CcIDOffsetFrontHST_si8; //!< offset on charID for front sensors in case that high speed tracking is active attention dependencies
  SInt8 CcIDOffsetRearHST_si8; //!< offset on charID for rear sensors in case that high speed tracking is active attention dependencies
  UInt8 VelocityMaxAc_ui8; //!< Disable warnings BSD output if speed is above this limit for acceleration For values 140km/h Limitat
  UInt8 VelocityMaxDe_ui8; //!< Disable warnings BSD output if speed is above this limit for deceleration For values 140km/h Limitat
  UInt8 HighNoiseHysteresisFront_ui8; //!< number of char Ids which have to be decreased by noise level estimation to reset an existing InchHig
  UInt8 HighNoiseHysteresisRear_ui8; //!< number of char Ids which have to be decreased by noise level estimation to reset an existing InchHig
}
gType_parEP_BSD_Ep_st; //!< copydoc Type_parEP_BSD_Ep_st


//! parameter cluster EP_BSD_SDI: Section headline only visual structuring
typedef struct Type_parEP_BSD_SDI_st
{
  UInt16 MinObjStayTime_ui16; //!< Minimal time before SDI warning is initiated it can be used to avoid warnings due to small objects e
  UInt16 WarnLvlHoldTime_ui16; //!< SDI warning level will be maintained for this time if no SDI warning with higher priority level is r
  UInt8 NumOfWarnLevels_ui8; //!< Number of warning levels to be used i e this number of warning distances will be considered E g NumO
  UInt8 WarnDistancesLeft_pui8[7]; //!< SDI warning level 0 is initiated on the left side if left object is closer than this distance
  UInt8 WarnDistancesRight_pui8[7]; //!< SDI warning level 0 is initiated on the right side if right object is closer than this distance
}
gType_parEP_BSD_SDI_st; //!< copydoc Type_parEP_BSD_SDI_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- AP_PDC                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup AP_PDC_M04           exported data types
 * \ingroup COMPONENT_AP_PDC
 @{ */

//! parameter cluster AP_PDC_General: AP_PDC GENERAL PARAMETERS
typedef struct Type_parAP_PDC_General_st
{
  UInt8 NumCyclesStartupSC_ui8; //!< number of complete sensor transceiving cycles for start up phase special cases recommended 0 2
  UInt8 ApPdcTimeOutFront_ui8; //!< Time until a ap_PDC update is forced on the front side even if there was no measurement on the front
  UInt8 ApPdcTimeOutRear_ui8; //!< Time until a ap_PDC update is forced on the rear side even if there was no measurement on the rear s
}
gType_parAP_PDC_General_st; //!< copydoc Type_parAP_PDC_General_st


//! parameter cluster AP_PDC_Disp_Global: AP_PDC GLOBAL DISPLAY PARAMETERS
typedef struct Type_parAP_PDC_Disp_Global_st
{
  UInt16 FullWarnDist_ui16; //!< Project specific distance to trigger a stop warning
  UInt16 DispAreaHoldTime_ui16; //!<  Display stabilization Hold time for display areas
  UInt16 DispAreaDelayTime_ui16; //!< Display stabilization Delay time for the activation of display areas
  UInt16 DispAreaApproachHyst_ui16; //!<  Display stabilization Approaching hysteresis for display areas
  UInt16 DispAreaDepartHyst_ui16; //!<  Display stabilization Departing hysteresis for display areas
  UInt16 DispAreaCloseDepartHyst_ui16; //!< Departing hysteresis close range close region hysteresis for standard and premium approach
  UInt16 DispDist4CloseDepartHyst_ui16; //!< maximum display distance for Departing hysteresis close range
  UInt16 DispAreaDistRes_ui16; //!< distance resolution mm old display value will be hold if the demanded change of the display distance
  SInt16 SafetyDist4OOC_si16; //!< Safety dist for increasing the travel path All objects within this extended travel path will be take
  SInt16 Dist2CourseHystOOC_si16; //!< Hysterese for Object on Course evaluation regarding Distance to Course If Object is outside travel p
  UInt16 DispDistMinimum_ui16; //!< Project specific minimum display distance
}
gType_parAP_PDC_Disp_Global_st; //!< copydoc Type_parAP_PDC_Disp_Global_st


//! parameter cluster AP_PDC_Disp_Lat: AP_PDC LATZONE DISPLAY PARAMETERS
typedef struct Type_parAP_PDC_Disp_Lat_st
{
  UInt16 ObjCaptureTime_ui16; //!<  Display cature time Object capture time for display Only active for 2 display areas
  UInt8 DispAreaFrontMin_pui8[5]; //!< mapping of the lateral bumper directions to display areas bumper directions 0 10 display areas 0 4 0
  UInt8 DispAreaFrontMax_pui8[5]; //!< See above
  UInt8 DispAreaFrontHystMin_pui8[5]; //!< See above
  UInt8 DispAreaFrontHystMax_pui8[5]; //!< See above
  UInt8 DispAreaRearMin_pui8[5]; //!< See above
  UInt8 DispAreaRearMax_pui8[5]; //!< See above
  UInt8 DispAreaRearHystMin_pui8[5]; //!< See above
  UInt8 DispAreaRearHystMax_pui8[5]; //!< See above
}
gType_parAP_PDC_Disp_Lat_st; //!< copydoc Type_parAP_PDC_Disp_Lat_st


//! parameter cluster AP_PDC_View_Lat: AP_PDC LATZONE FIELD OF VIEW PARAMETERS
typedef struct Type_parAP_PDC_View_Lat_st
{
  UInt16 ClipFrontZone_pui16[23]; //!< Clipping values for the bumper zones 0 22 for front side limitation of the object distances for late
  UInt16 ClipRearZone_pui16[23]; //!< Clipping values for the bumper zones 0 22 for rear side limitation of the object distances for later
  UInt16 ClippingHyst_ui16; //!< clipping hysteresis value
}
gType_parAP_PDC_View_Lat_st; //!< copydoc Type_parAP_PDC_View_Lat_st


//! parameter cluster AP_PDC_View_Prm: AP_PDC PREMIUM GLOBAL DISPLAY PARAMETERS
typedef struct Type_parAP_PDC_View_Prm_st
{
  UInt8 ExistProbMin4Disp_ui8; //!<  minimum existance probability for high class objects for being displayed
  UInt8 ExistProbMinMinorObj4Disp_ui8; //!<  minimum existance probability for De or Ce only objects for being displayed
  SInt16 DispAreaFrontA1X_psi16[6]; //!<  x value of 1st point of DisplayArea A
  SInt16 DispAreaFrontA1Y_psi16[6]; //!<  y value of 1st point of DisplayArea A
  SInt16 DispAreaFrontA2X_psi16[6]; //!<  x value of 2nd point of DisplayArea A
  SInt16 DispAreaFrontA2Y_psi16[6]; //!<  y value of 2nd point of DisplayArea A
  SInt16 DispAreaFrontA3X_psi16[6]; //!<  x value of 3rd point of DisplayArea A
  SInt16 DispAreaFrontA3Y_psi16[6]; //!<  y value of 3rd point of DisplayArea A
  SInt16 DispAreaFrontA4X_psi16[6]; //!<  x value of 4th point of DisplayArea A
  SInt16 DispAreaFrontA4Y_psi16[6]; //!<  y value of 4th point of DisplayArea A
  SInt16 DispAreaFrontHysA1X_psi16[6]; //!<  x value of 1st point of Hysteresis DisplayArea A
  SInt16 DispAreaFrontHysA1Y_psi16[6]; //!<  y value of 1st point of Hysteresis DisplayArea A
  SInt16 DispAreaFrontHysA2X_psi16[6]; //!<  x value of 2nd point of Hysteresis DisplayArea A
  SInt16 DispAreaFrontHysA2Y_psi16[6]; //!<  y value of 2nd point of Hysteresis DisplayArea A
  SInt16 DispAreaFrontHysA3X_psi16[6]; //!<  x value of 3rd point of Hysteresis DisplayArea A
  SInt16 DispAreaFrontHysA3Y_psi16[6]; //!<  y value of 3rd point of Hysteresis DisplayArea A
  SInt16 DispAreaFrontHysA4X_psi16[6]; //!<  x value of 4th point of Hysteresis DisplayArea A
  SInt16 DispAreaFrontHysA4Y_psi16[6]; //!<  y value of 4th point of Hysteresis DisplayArea A
  SInt16 DispAreaRearA1X_psi16[6]; //!<  x value of 1st point of DisplayArea A
  SInt16 DispAreaRearA1Y_psi16[6]; //!<  y value of 1st point of DisplayArea A
  SInt16 DispAreaRearA2X_psi16[6]; //!<  x value of 2nd point of DisplayArea A
  SInt16 DispAreaRearA2Y_psi16[6]; //!<  y value of 2nd point of DisplayArea A
  SInt16 DispAreaRearA3X_psi16[6]; //!<  x value of 3rd point of DisplayArea A
  SInt16 DispAreaRearA3Y_psi16[6]; //!<  y value of 3rd point of DisplayArea A
  SInt16 DispAreaRearA4X_psi16[6]; //!<  x value of 4th point of DisplayArea A
  SInt16 DispAreaRearA4Y_psi16[6]; //!<  y value of 4th point of DisplayArea A
  SInt16 DispAreaRearHysA1X_psi16[6]; //!<  x value of 1st point of Hysteresis DisplayArea A
  SInt16 DispAreaRearHysA1Y_psi16[6]; //!<  y value of 1st point of Hysteresis DisplayArea A
  SInt16 DispAreaRearHysA2X_psi16[6]; //!<  x value of 2nd point of Hysteresis DisplayArea A
  SInt16 DispAreaRearHysA2Y_psi16[6]; //!<  y value of 2nd point of Hysteresis DisplayArea A
  SInt16 DispAreaRearHysA3X_psi16[6]; //!<  x value of 3rd point of Hysteresis DisplayArea A
  SInt16 DispAreaRearHysA3Y_psi16[6]; //!<  y value of 3rd point of Hysteresis DisplayArea A
  SInt16 DispAreaRearHysA4X_psi16[6]; //!<  x value of 4th point of Hysteresis DisplayArea A
  SInt16 DispAreaRearHysA4Y_psi16[6]; //!<  y value of 4th point of Hysteresis DisplayArea A
  SInt16 DispAreaLeftA1X_psi16[5]; //!<  x value of 1st point of DisplayArea A
  SInt16 DispAreaLeftA1Y_psi16[5]; //!<  y value of 1st point of DisplayArea A
  SInt16 DispAreaLeftA2X_psi16[5]; //!<  x value of 2nd point of DisplayArea A
  SInt16 DispAreaLeftA2Y_psi16[5]; //!<  y value of 2nd point of DisplayArea A
  SInt16 DispAreaLeftA3X_psi16[5]; //!<  x value of 3rd point of DisplayArea A
  SInt16 DispAreaLeftA3Y_psi16[5]; //!<  y value of 3rd point of DisplayArea A
  SInt16 DispAreaLeftA4X_psi16[5]; //!<  x value of 4th point of DisplayArea A
  SInt16 DispAreaLeftA4Y_psi16[5]; //!<  y value of 4th point of DisplayArea A
  SInt16 DispAreaLeftHysA1X_psi16[5]; //!<  x value of 1st point of Hysteresis DisplayArea A
  SInt16 DispAreaLeftHysA1Y_psi16[5]; //!<  y value of 1st point of Hysteresis DisplayArea A
  SInt16 DispAreaLeftHysA2X_psi16[5]; //!<  x value of 2nd point of Hysteresis DisplayArea A
  SInt16 DispAreaLeftHysA2Y_psi16[5]; //!<  y value of 2nd point of Hysteresis DisplayArea A
  SInt16 DispAreaLeftHysA3X_psi16[5]; //!<  x value of 3rd point of Hysteresis DisplayArea A
  SInt16 DispAreaLeftHysA3Y_psi16[5]; //!<  y value of 3rd point of Hysteresis DisplayArea A
  SInt16 DispAreaLeftHysA4X_psi16[5]; //!<  x value of 4th point of Hysteresis DisplayArea A
  SInt16 DispAreaLeftHysA4Y_psi16[5]; //!<  y value of 4th point of Hysteresis DisplayArea A
  SInt16 DispAreaRightA1X_psi16[5]; //!<  x value of 1st point of DisplayArea A
  SInt16 DispAreaRightA1Y_psi16[5]; //!<  y value of 1st point of DisplayArea A
  SInt16 DispAreaRightA2X_psi16[5]; //!<  x value of 2nd point of DisplayArea A
  SInt16 DispAreaRightA2Y_psi16[5]; //!<  y value of 2nd point of DisplayArea A
  SInt16 DispAreaRightA3X_psi16[5]; //!<  x value of 3rd point of DisplayArea A
  SInt16 DispAreaRightA3Y_psi16[5]; //!<  y value of 3rd point of DisplayArea A
  SInt16 DispAreaRightA4X_psi16[5]; //!<  x value of 4th point of DisplayArea A
  SInt16 DispAreaRightA4Y_psi16[5]; //!<  y value of 4th point of DisplayArea A
  SInt16 DispAreaRightHysA1X_psi16[5]; //!<  x value of 1st point of Hysteresis DisplayArea A
  SInt16 DispAreaRightHysA1Y_psi16[5]; //!<  y value of 1st point of Hysteresis DisplayArea A
  SInt16 DispAreaRightHysA2X_psi16[5]; //!<  x value of 2nd point of Hysteresis DisplayArea A
  SInt16 DispAreaRightHysA2Y_psi16[5]; //!<  y value of 2nd point of Hysteresis DisplayArea A
  SInt16 DispAreaRightHysA3X_psi16[5]; //!<  x value of 3rd point of Hysteresis DisplayArea A
  SInt16 DispAreaRightHysA3Y_psi16[5]; //!<  y value of 3rd point of Hysteresis DisplayArea A
  SInt16 DispAreaRightHysA4X_psi16[5]; //!<  x value of 4th point of Hysteresis DisplayArea A
  SInt16 DispAreaRightHysA4Y_psi16[5]; //!<  y value of 4th point of Hysteresis DisplayArea A
}
gType_parAP_PDC_View_Prm_st; //!< copydoc Type_parAP_PDC_View_Prm_st


//! parameter cluster AP_PDC_Offset_Std: AP_PDC STANDARD BUMPER OFFSET PARAMETERS
typedef struct Type_parAP_PDC_Offset_Std_st
{
  UInt16 OffsetFront_pui16[5]; //!< Vehicle contour offset value for sensor pair S1S2 front
  UInt16 OffsetRear_pui16[5]; //!< Vehicle contour offset value for sensor pair S1S2 rear
  UInt16 TrailerOffset_pui16[2]; //!< Vehicle trailer offset value front
}
gType_parAP_PDC_Offset_Std_st; //!< copydoc Type_parAP_PDC_Offset_Std_st


//! parameter cluster AP_PDC_Offset_Lat: AP_PDC LATZONE BUMPER OFFSET PARAMETERS
typedef struct Type_parAP_PDC_Offset_Lat_st
{
  UInt16 OffsetFrontDir_pui16[11]; //!< Vehicle contour offsets front rough offsets offset values for each front bumper direction OffsetFron
  UInt16 OffsetFrontZone_pui16[23]; //!< Description see InchOffsetFrontDirInch
  UInt16 OffsetRearDir_pui16[11]; //!< Description see InchOffsetFrontDirInch
  UInt16 OffsetRearZone_pui16[23]; //!< Description see InchOffsetFrontDirInch
}
gType_parAP_PDC_Offset_Lat_st; //!< copydoc Type_parAP_PDC_Offset_Lat_st


//! parameter cluster AP_PDC_SC_DT_Std: AP_PDC STANDARD SC DIVE THROUGH
typedef struct Type_parAP_PDC_SC_DT_Std_st
{
  Boolean DTFrontEnabled_bl; //!< Enable/disable Flags for DT functionality for standard approach The DT functionality can be enabled
  Boolean DTRearEnabled_bl; //!< Enable/disable Flags for DT functionality for standard approach The DT functionality can be enabled
  UInt16 DTFrontDeAMin_ui16; //!< Front distance threshold values for the direct echos for activation DT warning for standard approach
  UInt16 DTFrontDeAMax_ui16; //!< See above
  UInt16 DTFrontDe25Lock_ui16; //!< See above
  UInt16 DTFrontDeB_ui16; //!< See above
  UInt16 DTFrontCeLR_ui16; //!< Front distance threshold values for the cross echos for activation DT warning for standard approach
  UInt16 DTFrontCeMin_ui16; //!< See above
  UInt16 DTFrontCeMax_ui16; //!< See above
  UInt16 DTRearDeAMin_ui16; //!< Rear distance threshold values for the direct echos for activation DT warning for standard approach
  UInt16 DTRearDeAMax_ui16; //!< See above
  UInt16 DTRearDe25Lock_ui16; //!< See above
  UInt16 DTRearDeB_ui16; //!< See above
  UInt16 DTRearCeLR_ui16; //!< Rear distance threshold values for the cross echos for activation DT warning for standard approach
  UInt16 DTRearCeMin_ui16; //!< See above
  UInt16 DTRearCeMax_ui16; //!< See above
}
gType_parAP_PDC_SC_DT_Std_st; //!< copydoc Type_parAP_PDC_SC_DT_Std_st


//! parameter cluster AP_PDC_SC_DT_Lat: AP_PDC LATZONE SC DIVE THROUGH
typedef struct Type_parAP_PDC_SC_DT_Lat_st
{
  UInt8 DTFrontEnabled_ui8; //!< Enable/disable Flags for front side DT functionality for LatZone approach The DT functionality can b
  UInt8 DTRearEnabled_ui8; //!< Enable/disable Flags for rear side DT functionality for LatZone approach The DT functionality can be
  UInt16 DTFrontActivateDistNarrow_pui16[5]; //!< narrow object distance threshold for front sensor pair 12 to activate DT warning for LatZone approac
  UInt16 DTFrontActivateDistCEObj_pui16[5]; //!< Ce object distance threshold for front sensor pair 12 to activate DT warning for LatZone approach
  UInt16 DTFrontActivateDistDEObj_pui16[5]; //!< De object distance threshold for front sensor pair 12 to activate DT warning for LatZone approach
  UInt16 DTRearActivateDistNarrow_pui16[5]; //!< narrow object distance threshold for rear sensor pair 12 to activate DT warning for LatZone approach
  UInt16 DTRearActivateDistCEObj_pui16[5]; //!< Ce object distance threshold for rear sensor pair 12 to activate DT warning for LatZone approach
  UInt16 DTRearActivateDistDEObj_pui16[5]; //!< De object distance threshold for rear sensor pair 12 to activate DT warning for LatZone approach
  UInt16 DTFrontDeactivateDistNarrow_pui16[5]; //!< narrow object distance threshold for front sensor pair 12 to deactivate DT warning for LatZone appro
  UInt16 DTFrontDeactivateDistCEObj_pui16[5]; //!< Ce object distance threshold for front sensor pair 12 to deactivate DT warning for LatZone approach
  UInt16 DTFrontDeactivateDistDEObj_pui16[5]; //!< De object distance threshold for front sensor pair 12 to deactivate DT warning for LatZone approach
  UInt16 DTRearDeactivateDistNarrow_pui16[5]; //!< narrow object distance threshold for rear sensor pair 12 to deactivate DT warning for LatZone approa
  UInt16 DTRearDeactivateDistCEObj_pui16[5]; //!< Ce object distance threshold for rear sensor pair 12 to deactivate DT warning for LatZone approach
  UInt16 DTRearDeactivateDistDEObj_pui16[5]; //!< De object distance threshold for rear sensor pair 12 to deactivate DT warning for LatZone approach
}
gType_parAP_PDC_SC_DT_Lat_st; //!< copydoc Type_parAP_PDC_SC_DT_Lat_st


//! parameter cluster AP_PDC_SC_DU_Lat: AP_PDC LATZONE SC DIVE UNDER
typedef struct Type_parAP_PDC_SC_DU_Lat_st
{
  UInt8 DUFrontEnabled_ui8; //!< Enable/disable Flags for front side DU functionality for LatZone approach The DU functionality can b
  UInt8 DURearEnabled_ui8; //!< Enable/disable Flags for rear side DU functionality for LatZone approach The DU functionality can be
  UInt8 DUFrontLatzoneMin_pui8[5]; //!< left lateral zone of teach area ?? area within a potential diving under obstacle can still be detecte
  UInt8 DURearLatzoneMin_pui8[5]; //!< See above
  UInt8 DUFrontLatzoneMax_pui8[5]; //!< right lateral zone of teach area ?? area within a potential diving under obstacle can still be detect
  UInt8 DURearLatzoneMax_pui8[5]; //!< See above
  UInt16 DUFrontDistTeachMax_pui16[5]; //!< maximum distance of teach area ?? area within a potential diving under obstacle can still be detected
  UInt16 DURearDistTeachMax_pui16[5]; //!< See above
  UInt16 DUFrontDistDeltaMinMeasured_ui16; //!< required minimum decreasing of the obstacle distance within the teaching area ?? area within a potent
  UInt16 DURearDistDeltaMinMeasured_ui16; //!< See above
  UInt16 DUFrontDistCriticalMax_pui16[5]; //!< maximum distance of critical area ?? area within the detection lost of a potential diving under obsta
  UInt16 DURearDistCriticalMax_pui16[5]; //!< See above
  UInt16 DUFrontDistDeltaObjLost_ui16; //!< required minimum jump of the obstacle distance within the critical area ?? area within the detection
  UInt16 DURearDistDeltaObjLost_ui16; //!< See above
  UInt16 DUFrontActiveTime_ui16; //!< duration time of dive under warning The full warning due to a dive under situation will be hold as l
  UInt16 DURearActiveTime_ui16; //!< See above
}
gType_parAP_PDC_SC_DU_Lat_st; //!< copydoc Type_parAP_PDC_SC_DU_Lat_st


//! parameter cluster AP_PDC_SC_DU_Prm: AP_PDC LATZONE SC DIVE UNDER
typedef struct Type_parAP_PDC_SC_DU_Prm_st
{
  Boolean DUFrontEnabled_bl; //!< Enable/disable Flags for DU functionality of vehicle front side for premium approach
  Boolean DURearEnabled_bl; //!< Enable/disable Flags for DU functionality of vehicle rear side for premium approach
  UInt16 DUFrontDistTeachMax_ui16; //!< maximum distance of teach area ?? area within a potential diving under obstacle can still be detected
  UInt16 DURearDistTeachMax_ui16; //!< maximum distance of teach area ?? area within a potential diving under obstacle can still be detected
  UInt16 DUFrontDistDeltaMinApproach_ui16; //!< required minimum obstacle approaching within the teaching area ?? area within a potential diving unde
  UInt16 DURearDistDeltaMinApproach_ui16; //!< required minimum obstacle approaching within the teaching area ?? area within a potential diving unde
  UInt16 DUFrontDistCriticalMax_ui16; //!< maximum distance of critical area ?? area within the detection lost of a potential diving under obsta
  UInt16 DURearDistCriticalMax_ui16; //!< maximum distance of critical area ?? area within the detection lost of a potential diving under obsta
  UInt16 DUFrontActiveTime_ui16; //!< duration time of dive under warning for vehicel front side The full warning due to a dive under situ
  UInt16 DURearActiveTime_ui16; //!< duration time of dive under warning for vehicel rear side The full warning due to a dive under situa
}
gType_parAP_PDC_SC_DU_Prm_st; //!< copydoc Type_parAP_PDC_SC_DU_Prm_st


//! parameter cluster AP_PDC_SC_OE_Std: AP_PDC STANDARD SC OUTER EDGE
typedef struct Type_parAP_PDC_SC_OE_Std_st
{
  Boolean OEFrontEnabled_bl; //!< Enable/disable Flags for OE functionality The OE functionality can be enabled for front and rear veh
  Boolean OERearEnabled_bl; //!< Enable/disable Flags for OE functionality The OE functionality can be enabled for front and rear veh
  Boolean OEFrontEnabledInner_bl; //!< Enable/disable Flags for OE functionality The OE functionality can be enabled for front and rear veh
  Boolean OERearEnabledInner_bl; //!< Enable/disable Flags for OE functionality The OE functionality can be enabled for front and rear veh
  UInt16 OEFrontDeA_ui16; //!<  DE_A threshold for direct echo of outer sensor DE_B threshold for direct echo of sensor neighboured
  UInt16 OEFrontDeB_ui16; //!< See above
  UInt16 OEFrontCeA_ui16; //!< See above
  UInt16 OEFrontCeB_ui16; //!< See above
  UInt16 OEFrontOffset_ui16; //!< See above
  UInt16 OEFrontDeAInner_ui16; //!< See above
  UInt16 OEFrontCeAInner_ui16; //!< See above
  UInt16 OEFrontCeBInner_ui16; //!< See above
  UInt16 OEFrontOffsetInner_ui16; //!< See above
  UInt16 OERearDeA_ui16; //!< See above
  UInt16 OERearDeB_ui16; //!< See above
  UInt16 OERearCeA_ui16; //!< See above
  UInt16 OERearCeB_ui16; //!< See above
  UInt16 OERearOffset_ui16; //!< See above
  UInt16 OERearDeAInner_ui16; //!< See above
  UInt16 OERearCeAInner_ui16; //!< See above
  UInt16 OERearCeBInner_ui16; //!< See above
  UInt16 OERearOffsetInner_ui16; //!< See above
}
gType_parAP_PDC_SC_OE_Std_st; //!< copydoc Type_parAP_PDC_SC_OE_Std_st


//! parameter cluster AP_PDC_SC_OE_Lat: AP_PDC LATZONE SC OUTER EDGE
typedef struct Type_parAP_PDC_SC_OE_Lat_st
{
  Boolean OEFrontEnabled_bl; //!< Enable/disable Flags for OE functionality The OE functionality can be enabled for front and rear veh
  Boolean OERearEnabled_bl; //!< Enable/disable Flags for OE functionality The OE functionality can be enabled for front and rear veh
  UInt8 OEFrontGroupLeft_ui8; //!< front outer left sensor group relevant for outer edge warning
  UInt8 OEFrontGroupRight_ui8; //!< front outer right sensor group relevant for outer edge warning
  UInt8 OERearGroupLeft_ui8; //!< rear outer left sensor group relevant for outer edge warning
  UInt8 OERearGroupRight_ui8; //!< rear outer right sensor group relevant for outer edge warning
  UInt16 OEFrontMaxDist_ui16; //!<  Maximum distance when OE starts Distance which is mapped to gd_AP_PDC_FULL_WARN_DIST_ui16 These dis
  UInt16 OEFrontMinDist_ui16; //!< See above
  UInt16 OERearMaxDist_ui16; //!< See above
  UInt16 OERearMinDist_ui16; //!< See above
}
gType_parAP_PDC_SC_OE_Lat_st; //!< copydoc Type_parAP_PDC_SC_OE_Lat_st


//! parameter cluster AP_PDC_SC_SWD_Std: AP_PDC STANDARD SC SIDE WALL DETECTION
typedef struct Type_parAP_PDC_SC_SWD_Std_st
{
  Boolean SWDFrontEnabled_bl; //!< Enable/disable Flags for SWD functionality The SWD functionality can be enabled for front and rear v
  Boolean SWDRearEnabled_bl; //!< Enable/disable Flags for SWD functionality The SWD functionality can be enabled for front and rear v
  UInt8 SWDFrontGroupLeft_ui8; //!< front outer left sensor group relevant for side wall detection
  UInt8 SWDFrontGroupRight_ui8; //!< front outer right sensor group relevant for side wall detection
  UInt8 SWDRearGroupLeft_ui8; //!< rear outer left sensor group relevant for side wall detection
  UInt8 SWDRearGroupRight_ui8; //!< rear outer right sensor group relevant for side wall detection
  UInt16 SWDFrontDistCutOff_ui16; //!<  cut off distance for front vehicle side below this distance a SWD will not be triggered
  UInt16 SWDRearDistCutOff_ui16; //!<  cut off distance for rear vehicle side below this distance a SWD will not be triggered
  UInt16 SWDDistHyst_ui16; //!<  Hysteresis for cutoff distance in case of departing wall
  UInt8 SWDFrontSpeedCutOff_ui8; //!< cut off speed for front vehicle side above this speed a SWD will not be triggered
  UInt8 SWDRearSpeedCutOff_ui8; //!< cut off speed for rear vehicle side above this speed a SWD will not be triggered
  UInt8 SWDDelayTime_ui8; //!< delay time for activation of side wall detection
  UInt16 SWDFrontDistApproach_ui16; //!< Allowed approach distance tolerance for front vehicle side
  UInt16 SWDRearDistApproach_ui16; //!< Allowed approach distance tolerance for rear vehicle side
}
gType_parAP_PDC_SC_SWD_Std_st; //!< copydoc Type_parAP_PDC_SC_SWD_Std_st


//! parameter cluster AP_PDC_SC_SWD_Lat: AP_PDC LATZONE SC SIDE WALL DETECTION
typedef struct Type_parAP_PDC_SC_SWD_Lat_st
{
  Boolean SWDFrontEnabled_bl; //!< Enable/disable Flags for SWD functionality The SWD functionality can be enabled for front and rear v
  Boolean SWDRearEnabled_bl; //!< Enable/disable Flags for SWD functionality The SWD functionality can be enabled for front and rear v
  UInt8 SWDFrontGroupLeft_ui8; //!< front outer left sensor group relevant for side wall detection
  UInt8 SWDFrontGroupRight_ui8; //!< front outer right sensor group relevant for side wall detection
  UInt8 SWDRearGroupLeft_ui8; //!< rear outer left sensor group relevant for side wall detection
  UInt8 SWDRearGroupRight_ui8; //!< rear outer right sensor group relevant for side wall detection
  UInt16 SWDDistCutOff_ui16; //!<  cut off distance below this distance a SWD will not be triggered
  UInt16 SWDDistCutOffHyst_ui16; //!<  Hysteresis for cutoff distance
  UInt8 SWDSpeedCutOff_ui8; //!<  cut off speed above this speed a SWD will not be triggered
  UInt8 SWDSpeedCutOffHyst_ui8; //!<  Hysteresis for cutoff speed
  UInt16 SWDDriveDist_ui16; //!< required drive way before activation of side wall detection
  UInt16 SWDMaxAppDepSpeed_ui16; //!<  Allowed approaching/departing speed
  UInt16 SWDMaxAppDepSpeedHyst_ui16; //!<  Hysteresis for approaching/departing speed
}
gType_parAP_PDC_SC_SWD_Lat_st; //!< copydoc Type_parAP_PDC_SC_SWD_Lat_st


//! parameter cluster AP_PDC_SC_SWD_Prm: AP_PDC PREMIUM SC SIDE WALL DETECTION
typedef struct Type_parAP_PDC_SC_SWD_Prm_st
{
  Boolean SWDFrontEnabled_bl; //!< Enable/disable Flags for SWD functionality The SWD functionality can be enabled for front and rear v
  Boolean SWDRearEnabled_bl; //!< Enable/disable Flags for SWD functionality The SWD functionality can be enabled for front and rear v
  UInt16 SWDDistCutOff_ui16; //!< cut off distance below this distance a SWD will not be triggered
  UInt16 SWDDistCutOffHyst_ui16; //!< Hysteresis for cutoff distance
  UInt8 SWDSpeedCutOff_ui8; //!< cut off speed above this speed a SWD will not be triggered
  UInt8 SWDSpeedCutOffHyst_ui8; //!< Hysteresis for cutoff speed
  UInt16 SWDDriveDist_ui16; //!< required drive way before activation of side wall detection
  UInt16 SWDDistDeltaMax_ui16; //!< allowed distance alteration to side wall
  UInt16 SWDDistDeltaMaxHyst_ui16; //!< Hysteresis for DistDeltaMax
}
gType_parAP_PDC_SC_SWD_Prm_st; //!< copydoc Type_parAP_PDC_SC_SWD_Prm_st


//! parameter cluster AP_PDC_SC_WBL_Prm: AP_PDC PREMIUM SC WARN BEFORE LOST
typedef struct Type_parAP_PDC_SC_WBL_Prm_st
{
  UInt16 WBLWallLengthMax_ui16; //!< maximum length of wall objects for being considered for WBL
  UInt16 WBLGroupCenterDistStart_pui16[2]; //!<  WBL start distance for center sensor pair of front side below this object distance a reducing of th
  UInt16 WBLGroupCenterDistStop_pui16[2]; //!<  WBL stop distance for center sensor pair of front side at this object distance the distance to be d
  UInt16 WBLGroupCenterDeDistStart_pui16[2]; //!<  WBL start distance for De only objects for center sensor pair of front side
  UInt16 WBLGroupCenterDeDistStop_pui16[2]; //!<  WBL stop distance for De only objects for center sensor pair of front side
  UInt16 WBLGroupCenterCeDistStart_pui16[2]; //!<  WBL start distance for Ce only objects for center sensor pair of front side
  UInt16 WBLGroupCenterCeDistStop_pui16[2]; //!<  WBL stop distance for Ce only objects for center sensor pair of front side
  UInt16 WBLGroupSideDistStart_pui16[2]; //!<  WBL start distance for sensor pairs 23 and 45 of front side
  UInt16 WBLGroupSideDistStop_pui16[2]; //!<  WBL stop distance for sensor pairs 23 and 45 of front side
  UInt16 WBLGroupSideDeDistStart_pui16[2]; //!<  WBL start distance for De only objects for sensor pairs 23 and 45 of front side
  UInt16 WBLGroupSideDeDistStop_pui16[2]; //!<  WBL stop distance for De only objects for sensor pairs 23 and 45 of front side
  UInt16 WBLGroupSideCeDistStart_pui16[2]; //!<  WBL start distance for Ce only objects for sensor pairs 23 and 45 of front side
  UInt16 WBLGroupSideCeDistStop_pui16[2]; //!<  WBL stop distance for Ce only objects for sensor pairs 23 and 45 of front side
  UInt16 WBLEdgeDistStart_pui16[2]; //!<  WBL start distance for edge section of front side
  UInt16 WBLEdgeDistStop_pui16[2]; //!<  WBL stop distance for edge section of front side
  UInt16 WBLEdgeDeDistStart_pui16[2]; //!<  WBL start distance for De only objects for edge section of front side
  UInt16 WBLEdgeDeDistStop_pui16[2]; //!<  WBL stop distance for De only objects for edge section of front side
  UInt16 WBLEdgeCeDistStart_pui16[2]; //!<  WBL start distance for Ce only objects for edge section of front side
  UInt16 WBLEdgeCeDistStop_pui16[2]; //!<  WBL stop distance for Ce only objects for edge section of front side
  UInt16 WBLEdgeWidth_pui16[2]; //!<  width of edge section of front side
}
gType_parAP_PDC_SC_WBL_Prm_st; //!< copydoc Type_parAP_PDC_SC_WBL_Prm_st


//! parameter cluster AP_PDC_AO: AP_PDC ATTACHED OBJECTS
typedef struct Type_parAP_PDC_AO_st
{
  UInt8 AOMaxSamples_ui8; //!< maximum number of samples measured
  UInt8 AOMinNumDetectSamples_ui8; //!< minimum number of samples with object detections
  UInt8 AOMinNumRegularEchos_ui8; //!< minimum number of regular echoes necessary to accept an attached object
  UInt16 AODistMax_ui16; //!< maximum distanced accepted for attached object
}
gType_parAP_PDC_AO_st; //!< copydoc Type_parAP_PDC_AO_st


//! parameter cluster AP_PDC_Danger: AP_PDC DANGER INDICATOR
typedef struct Type_parAP_PDC_Danger_st
{
  UInt16 DangerDelayMS_ui16; //!< delay time consisting of reaction time of driver and HMI t_delay t_react t_HMI ms
  UInt16 DangerDecelMMS2_ui16; //!< expected deceleration cm/s^2
  UInt16 DangerStopDist_ui16; //!< expected stop distance between object and vehicle mm
  UInt16 DangerActMinDist_ui16; //!< minimum distance for indicating danger mm
  UInt16 DangerDeactDist_ui16; //!< deactivation distance mm
  UInt16 DangerActMinSpeed_ui16; //!< minimum speed for indicating danger mm/s
  UInt16 DangerDeactSpeed_ui16; //!< deactivation speed mm/s
}
gType_parAP_PDC_Danger_st; //!< copydoc Type_parAP_PDC_Danger_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- PSU                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup PSU_M04           exported data types
 * \ingroup COMPONENT_PSU
 @{ */

//! parameter cluster PSU_CPSC: Section headline only visual structuring
typedef struct Type_parPSU_CPSC_st
{
  SInt16 LengthShortenMax_si16; //!< Maximum amount a parking space may be shortened at each side This value doesn t apply for a virtual
  SInt16 LengthExtendMax_si16; //!< Maximum amount a parking space may be extended at each side This value doesn t apply for a virtual r
  SInt16 DepthMin_si16; //!< Minimum depth for a parking space This value defines the max shortening of the rear reference This v
  SInt16 DepthMax_si16; //!< Maximum supported depth of a parking space for perpendicular parking and diagonal parking This value
  SInt16 DepthShortenMax_si16; //!< Maximum amount a confident lateral reference may be shifted in direction of parking space inner side
  SInt16 DepthExtendMax_si16; //!< Maximum amount a confident lateral reference may be shifted in direction outside of parking space
}
gType_parPSU_CPSC_st; //!< copydoc Type_parPSU_CPSC_st


//! parameter cluster PSU_PSC: Section headline only visual structuring
typedef struct Type_parPSU_PSC_st
{
  SInt16 LengthShortenMax_si16; //!< Maximum amount a parallel parking space may be shortened at each side This parameter is used for POC
  SInt16 LengthExtendMax_si16; //!< Maximum amount a parallel parking space may be extended at each side This parameter is used for POC
  SInt16 DepthMin_si16; //!< Minimum supported depth for adaption of parallel parking space
  SInt16 DepthMax_si16; //!< Maximum supported depth for adaption of parallel parking space
}
gType_parPSU_PSC_st; //!< copydoc Type_parPSU_PSC_st


//! parameter cluster PSU_POC: Section headline only visual structuring
typedef struct Type_parPSU_POC_st
{
  UInt16 FoVRangeFront_ui16; //!< Front Field Of View FOV Parameter for POC to set References if no objects are detected see nprod0028
  UInt16 FoVRangeRear_ui16; //!< Rear Field Of View FOV Parameter for POC to set References if no objects are detected see nprod00280
}
gType_parPSU_POC_st; //!< copydoc Type_parPSU_POC_st


//! parameter cluster PSU_CPX: Section headline only visual structuring
typedef struct Type_parPSU_CPX_st
{
  UInt16 BackIn_MinYaw_PLOutput_ui16; //!<  Min yaw angle between the vehicle and the parking space in order to put out the parking slot by PSU
  UInt16 BackIn_MaxLengthPS_ui16; //!<  Max allowed parking slot length If a parking slot has been detected with a smaller length it will b
  UInt16 FrontIn_MinYaw_PLOutput_ui16; //!< refer to BackIn
  UInt16 FrontIn_MaxLengthPS_ui16; //!< refer to BackIn
}
gType_parPSU_CPX_st; //!< copydoc Type_parPSU_CPX_st


//! parameter cluster PSU_Object: Section headline only visual structuring
typedef struct Type_parPSU_Object_st
{
  UInt8 ExistProbMin_ui8; //!< Minimum existence probability to be considered for matching with parking space references
  UInt8 DECEExistProbMin_ui8; //!< Minimum existence probability for DE/CE point objects to be considered for matching with parking spa
  UInt16 ErrorMin_ui16; //!<
}
gType_parPSU_Object_st; //!< copydoc Type_parPSU_Object_st


//! parameter cluster PSU_Reference: Section headline only visual structuring
typedef struct Type_parPSU_Reference_st
{
  UInt16 WeightMin_ui16; //!< Minimum weight of an entry in the reference array
  UInt16 WeightMax_ui16; //!< Maximum weight of an entry in the reference array
}
gType_parPSU_Reference_st; //!< copydoc Type_parPSU_Reference_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- APG                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup APG_M04           exported data types
 * \ingroup COMPONENT_APG
 @{ */

//! parameter cluster APG_XPG_Vehicle: no valid cluster description available
typedef struct Type_parAPG_XPG_Vehicle_st
{
  UInt16 HalfWidth_ui16; //!< half width of vehicle relating to distance between the outsides of the rear wheels Appl Process cust
  UInt16 Wheelbase_ui16; //!< Vehicle wheelbase distance between front and rear vehicle axles Appl Process customer spec mechanica
  UInt16 LengthFront_ui16; //!< length between rear axle and front of vehicle Appl Process customer spec mechanical drawing
  UInt16 LengthRear_ui16; //!< length between rear axle and rear of the vehicle Note A trailer hook is to be implicated when attach
  UInt16 CornerRadiusFront_ui16; //!< Radius of front corner of the vehicle shape Appl Process The specific radius could be estimated with
  UInt16 CornerRadiusRear_ui16; //!< Radius of rear corner of the vehicle shape Appl Process The specific radius could be estimated with
  SInt16 MaxSWA_si16; //!< maximum steering wheel angle that is reachable by EPS electronical power steering Note This value ca
  SInt16 SWAVeloMax_si16; //!< maximum steering wheel angle rate which could be performed by the EPS electronical power steering wh
  UInt16 RWheel_ui16; //!< distance between tyre midle to cubstone nearly wheel radius Appl Process Following formula applies h
}
gType_parAPG_XPG_Vehicle_st; //!< copydoc Type_parAPG_XPG_Vehicle_st


//! parameter cluster APG_XPG_SafetyDistParallel: no valid cluster description available
typedef struct Type_parAPG_XPG_SafetyDistParallel_st
{
  SInt16 MinLowLatRef_si16; //!< Safety distance after 1st move between low lateral reference and rear tyre center of reference conta
  SInt16 TargetLowLatRef_si16; //!< Lateral distance to the low lateral reference in end position Note This value should always be withi
  SInt16 MinHighLatRef_si16; //!< Safety distance after 1st move between high lateral reference and vehicle reference Rounded rear veh
  SInt16 TargetHighLatRef_si16; //!< Lateral distance to the high lateral reference in end position Note please refer to DistTargetLowLat
  SInt16 MinVirtLatRef_si16; //!< Safety distance after 1st move between virtual lateral reference and vehicle center of reference mid
  SInt16 TargetVirtLatRef_si16; //!< Lateral distance to the virtual lateral reference in end position Note please refer to DistTargetLow
  SInt16 Front_si16; //!< Distance between front of vehicle and the front limiting vehicle calculated using APG contour for eg
  SInt16 Rear_si16; //!< distance between the rear of the ego vehicle and the parking space limiting vehicle in the back Appl
  SInt16 Corner_si16; //!< Defines the radius of the circle around parking space corner P2 that must not be touched by the fron
  SInt16 Side_si16; //!< Defines the radius of the circle around parking space corner P2 that must not be touched by the park
  SInt16 OppositeSide_si16; //!< Minimum distance during parking maneuver between the car and objects detected on the opposite side o
  SInt16 ParkingSide_si16; //!< Minimum distance during parking maneuver between the car and objects detected on the parking side No
  SInt16 EarlyPosOkLatDistExt_si16; //!< Extension of the acceptable distance to the lateral reference after a n move maneuver due to an Earl
}
gType_parAPG_XPG_SafetyDistParallel_st; //!< copydoc Type_parAPG_XPG_SafetyDistParallel_st


//! parameter cluster APG_XPG_SafetyDistCross: no valid cluster description available
typedef struct Type_parAPG_XPG_SafetyDistCross_st
{
  SInt16 MinTargetSideDist_si16; //!< Minimal distance on the side between the ego vehicle and neighbor vehicles in final parking position
  SInt16 Side1stMove_si16; //!< Distance between the vehicle side of the car and the parking space corner parking space vehicle as r
  SInt16 SideNMove_si16; //!< Distance between the vehicle side of the car and the parking space object 2 front reference vehicle
  UInt8 Object1_ui8; //!< Safety distance between the ego vehicle and the first parking space limiting object Only relevant in
  SInt16 OppositeSide_si16; //!< Minimum distance during parking maneuver between the car and objects detected on the opposite side o
  SInt16 Corner1_si16; //!< CPSC FrontIn only Safety distance of vehicle side to parking space corner 1 Collision protection for
  SInt16 Side_FI_si16; //!< CPSC FrontIn only Safety distance of vehicle front corner to parking space side 2 This parameter is
}
gType_parAPG_XPG_SafetyDistCross_st; //!< copydoc Type_parAPG_XPG_SafetyDistCross_st


//! parameter cluster APG_XPG_Ctrl: no valid cluster description available
typedef struct Type_parAPG_XPG_Ctrl_st
{
  UInt16 TargetPosTolPsi_ui16; //!< tolerance for target yaw angle at the target position PSC Appl Process customer requirements tests
  UInt8 TargetTolX_CPSC_ui8; //!< Base tolerance for the x position at the target position CPSC based on remeasured side objects Note
  UInt16 TargetTolPsi_CPSC_ui16; //!< tolerance for yaw angle at the target position CPSC based on remeasured side objects Appl Process Se
  UInt16 VeloStandstill_ui16; //!< Speed km/h threshold which is interpreted as standstill due to noise in the velocity signal value un
  UInt16 VeloAbort_ui16; //!< maximal vehicle velocity for active guidance Note APG aborts with an error notification above this s
  UInt16 VeloSlowDown_ui16; //!< Above this vehicle speed threshold APG displays the notification ?slow down?? Note This value should
  UInt16 VeloSlowDownHyst_ui16; //!< Above a defined velocity km/h the driver will be advised to ??slow down?? The parameter defines a hyst
  UInt8 MaxNumOfMovesPSC_ui8; //!< max number of guided moves for parallel parking changes of forward/backward driving Note This parame
  UInt8 MaxNumOfMovesCPSC_ui8; //!< max number of guided moves for cross parking changes of forward/backward driving Note An odd number
  UInt8 MaxNumOfMovesCPSC_FI_ui8; //!< max number of guided moves for perpendicular parking front in changes of forward/backward driving No
  SInt16 MinLatDistForNMove_si16; //!< Minimum distance between vehicle and lat reference which has to be reached until a premature correct
  UInt16 StopVeloHyst_ui16; //!< Velocity Hysteresis for ?Stop?? notification to avoid a leaping HMI message Note In case of delays in
  UInt16 ReactionTime_ui16; //!< Assumed driver reaction time ms for the InchStopInch commands during the parking maneuver The InchSt
  UInt16 Deceleration_ui16; //!< Assumed vehicle deceleration m/s2 for the InchStopInch commands during the parking maneuver Appl Pro
  SInt8 TPR_Prediction_si8; //!< Configure when APG decides what the next command is at the end of the current move TargetPosReached
  SInt16 MaxUncontrolledTravelDist_si16; //!< Maximum distance that may be covered while a STOP command or a command to drive in the opposite dire
  UInt16 MaxPosDev_ui16; //!< Maximum deviation between actual vehicle position and the planned trajectory before the guidance is
  UInt16 MaxDrivenDistOnPASDegraded_ui16; //!< defines the maximum driven distance before reset of XPG after degredation mode PASDegraded has been
  UInt16 DeactTimePathBlockedByObstacle_ui16; //!< defines the time after the APG deactivates the function if the guidance is blocked due to an obstacl
  UInt32 MinYawAngleForEarlyStopOnInsufficientSpace_ui32; //!< If a planned CPSC backward move usually the 1st move would lead the driver into full warning or woul
  UInt8 BestGuessPSShiftInBlockadeSituation_ui8; //!< If the first move CPSC ends in a full warning because the vehicle is too close to parking space corn
}
gType_parAPG_XPG_Ctrl_st; //!< copydoc Type_parAPG_XPG_Ctrl_st


//! parameter cluster APG_XPG_NMove: no valid cluster description available
typedef struct Type_parAPG_XPG_NMove_st
{
  SInt16 SWAForward_si16; //!< Constant set steering wheel angle for further forward moves 1 Appl Process Set to a little less than
  SInt16 SWABackwardMin_si16; //!< Min steering wheel angle for further backward moves 1 Additional parameter to be considered SWABackw
  SInt16 SWABackwardMax_si16; //!< max steering wheel angle for further backward moves 1 with the premise SWABackwardMax SWABackwardMin
  SInt16 StraightenClothoidVelo_si16; //!< Max supported velocity for straighten clothoid in PSC N Moves Used only in conjunction with planned
  SInt16 MinLengthOfLongitudinalAlignmentMove_si16; //!< minimal length of a longitudinal alignment move shorter moves will not be offered
  UInt8 MaxLengthOfLongitudinalAlignmentMove_ui8; //!< maximal length of a longitudinal alignment move longer moves will not be offered Set to 0 to disable
  SInt16 AlignmentDistFrontMax_si16; //!< The alignment move will align the vehicle centered in the parking space equal distance to front and
  UInt8 MaxDist2RearRef_ui8; //!< The maximal distance of the vehicle to parking space rear reference object at the end of the move to
  UInt8 MaxDist2FrontRef_ui8; //!< The maximal distance of the vehicle to parking space front reference object at the end of the move t
}
gType_parAPG_XPG_NMove_st; //!< copydoc Type_parAPG_XPG_NMove_st


//! parameter cluster APG_XPG_Parkable: no valid cluster description available
typedef struct Type_parAPG_XPG_Parkable_st
{
  SInt16 EndPsiMaxF12_si16; //!< maximum allowable yaw angle at the end of the first move Note This parameter is linked to the parame
  SInt16 VeloHyst_si16; //!< Hysteresis for vehicle velocity to avoid flickering of Parkable status Appl Process These parameters
  UInt16 DistHyst_ui16; //!< Hysteresis for sideprotection safety distance to avoid flickering of Parkable status Appl Process Th
  UInt16 ReactionTime_ui16; //!< Assumed driver reaction time m/s for the parking space evaluation Parkable Only used if APGCFG_PREDI
  UInt16 Deceleration_ui16; //!< Assumed vehicle deceleration m/s2 Only used if APGCFG_PREDICTPOSINPARKABLE is SW_ON Appl Process see
}
gType_parAPG_XPG_Parkable_st; //!< copydoc Type_parAPG_XPG_Parkable_st


//! parameter cluster APG_APG_Pathplanning_Parallel: no valid cluster description available
typedef struct Type_parAPG_APG_Pathplanning_Parallel_st
{
  SInt16 SWA3Min_si16; //!< Defines allowable minimum of steering wheel angle for the circle which guides the vehicle into the p
  SInt16 SWA3Max_si16; //!< defines allowable maximum of steering wheel angle for the circle which guide the vehicle into the pa
  SInt16 SWA6_si16; //!< Defines steering wheel angle to be used for the last circle of the first move Appl Process Usually f
  SInt16 Velo0_si16; //!< Max supported velocity for the clothoid segment 0 start navigation segments The clothoid is calculat
  SInt16 Velo2_si16; //!< The base value for Inchsupported vehicle speedInch for the clothoid segment 2 which guides the vehic
  SInt16 Velo2Max_si16; //!< The maximum value for Inchsupported vehicle speedInch for clothoid segment 2 The available space wil
  SInt16 Velo4_si16; //!< The base value for Inchsupported vehicle speedInch for the clothoid segment 4 which guides the vehic
  SInt16 Velo4Max_si16; //!< The maximum value for Inchsupported vehicle speedInch for clothoid segment 4 The available space wil
  SInt16 Velo5_si16; //!< The base value for Inchsupported vehicle speedInch for the clothoid segment 5 which guides the vehic
  SInt16 Velo7_si16; //!< Max supported velocity for clothoid segment 7 used for a guided straightening of the steering wheel
  SInt16 AddSafetyDistFrontMax_si16; //!< Maximal extra distance between the front of the ego vehicle and the longitudinal parking space refer
  SInt16 SafetyDistFrontIncreaseRatioF10_si16; //!< The target position for large parking spaces can be adjusted with this parameter It describes the ra
}
gType_parAPG_APG_Pathplanning_Parallel_st; //!< copydoc Type_parAPG_APG_Pathplanning_Parallel_st


//! parameter cluster APG_APG_Steer_Ctrl: no valid cluster description available
typedef struct Type_parAPG_APG_Steer_Ctrl_st
{
  SInt16 DevPosP_si16; //!< steering path controller proportional coefficient on position deviation Note Only used when steering
  SInt16 DevPosI_si16; //!< steering path controller integration coefficient on position deviation Note Only used when steering
  UInt16 DevPosT_ui16; //!< Filter constant time for filter of curvature based on position deviation Note Position deviation is
  SInt16 DevPsiP_si16; //!< steering path controller proportional coefficient on yaw angle deviation Note Only used when steerin
  SInt16 DevPsiI_si16; //!< steering path controller integration coefficient on yaw angle deviation Note Only used when steering
  UInt16 DevPsiT_ui16; //!< Filter constant time for filter of curvature based on yaw angle deviation Note Yaw angle deviation i
  SInt16 DevPosMax_si16; //!< The max position deviation for steering path controller to remain active Setting this parameter valu
  SInt16 VeloMax_si16; //!< The max velocity for steering path controller to remain active Setting this parameter value to 0 dis
  UInt16 SWAFilterT_ui16; //!< Filter constant SWA over time for smoothing of the SWA output Smoothing of the SWA is done based on
  UInt16 SWAFilterTStandstill_ui16; //!< Filter constant SWA over time for smoothing of the SWA output if the vehicle is in standstill Set th
  UInt16 TargetPosFilterT_ui16; //!< Filter time constant for filtering the target position updates Target position updates will not be l
}
gType_parAPG_APG_Steer_Ctrl_st; //!< copydoc Type_parAPG_APG_Steer_Ctrl_st


//! parameter cluster APG_APG_Ctrl: no valid cluster description available
typedef struct Type_parAPG_APG_Ctrl_st
{
  UInt8 DeadTime_ui8; //!< Delay dead time of electric power steering EPS steering angle interface Note Enter the delay i e the
  UInt16 EPSHoldTimeBeforeRelease_ui16; //!< Value indicates time period in which the steering wheel should be kept in a certain position after t
  UInt16 EPSTimeoutStraightening_ui16; //!< Value indicates max time for the steering to the final target angle at the end of the guidance final
  SInt16 SWASpecLimit_si16; //!< maximum steering wheel angle request that can be requested from the EPS This is a steering wheel ang
  UInt16 SWAVeloStandStillMax_ui16; //!< Steering wheel velocity which will be used for the correction moves Turning in standstill Appl Proce
  UInt16 SWAVeloSpecLimit_ui16; //!< Max allowed steering angle velocity specified by the EPS Note The gradient of the set steering wheel
  UInt16 SWAAccMaxStandstill_ui16; //!< The max acceleration of the steering wheel angle for steering in standstill This value is usually be
  SInt32 LimiterFilter_SWAAccMax_si32; //!< Customer Projects 0 APG limiter filter is deactivated Values 0 APG Limiter filter is active For valu
  UInt16 MaxSWADev_ui16; //!< Maximum allowable difference between current and requested steering wheel angle Note Value must alwa
  UInt16 SWATolStandstill_ui16; //!< Allowed tolerance of steering wheel angle difference between the target and the real steering wheel
  UInt16 TargetSWAStraightening_ui16; //!< Target steering wheel angle that is set at the end of the PSC guidance if the velocity is low The st
}
gType_parAPG_APG_Ctrl_st; //!< copydoc Type_parAPG_APG_Ctrl_st


//! parameter cluster APG_APG_Pathplanning_Cross: no valid cluster description available
typedef struct Type_parAPG_APG_Pathplanning_Cross_st
{
  SInt16 VeloForwMove_si16; //!< Values apply to the necessarily assumed driving velocities for all forward moves CPSC and CPSC FI ex
  SInt16 VeloStraighten_si16; //!< Values apply to the necessarily assumed driving velocities for the straightening clothoid in CPSC at
  SInt16 VeloBackMove_si16; //!< Values apply to the necessarily assumed driving velocities for all forward moves The increasing of t
  UInt8 XLeftTolPer_ui8; //!< defines how much additional space at the vehicle side is allowed to use for longitudinal vehicle shi
  UInt8 XLeftTolPerNMove_ui8; //!< defines how much additional space at the vehicle side is allowed to use for longitudinal vehicle shi
  UInt16 XLeftTolMaxF10_ui16; //!< Defines the maximum allowed longitudinal X Direction shift of the vehicle target position away from
  SInt16 PSDepthOffset_si16; //!< offset value of parking space depth With the help of this parameter the lateral endposition could be
  UInt8 CornerLatDiffMax_ui8; //!< Maximal difference of lateral position of parking space object 1 and 2 for parking spaces with centr
  SInt16 SWAMax_si16; //!< Maximal allowed steering wheel angle for 1st 2nd and 3rd move of a CPSC maneuver or basically all pl
  SInt16 SWA2ndMin_si16; //!< minimal allowed steering wheel angle for 2nd move Appl Process 90?? is the smallest possible value sm
  UInt8 ToleranceDistForEarlyGearSwitch_ui8; //!< defines a distance to the stop point of the first move where a prematurelly switch to the forward ge
  SInt16 XLimitFirstBackwardMove_si16; //!< Defines the min possible x position of the rear right corner of the vehicle that is tolerated at the
  Boolean AlwaysUseSwitchPoint_bl; //!< With this setting you can alter the length of the first backward move of a 3 move maneuver A Inchswi
  SInt16 MaxDepthForSwitchPoint_si16; //!< The switch point should not be set deeper inside the parking space than this value This parameter de
  Boolean Move2Optimization_bl; //!< Selects the optimization strategy for the optimization loop of CPSC path planning of the 2nd move fo
  SInt16 VirtualOppositeSide_si16; //!< defines a max lateral distance which is allowed for the 2nd move avoid trajectory into oppsite lane
  UInt16 ThresholdOneSideAlignment_ui16; //!< if parking space length for a cross parking slot is bigger than the parameter the car will align not
  SInt16 TargetDistSideAlignment_si16; //!< if the vehicle aligns to one longitudinal reference not centered in the middle of the slot this para
  SInt16 TargetDist2LatRef_si16; //!< Distance to the lateral reference that is used to finish the parking guidance if the parking slot is
  SInt16 MaxDist2LatRefFullWarn_si16; //!< Tolerance to finish the parking guidance if the target position can not be reached due to a full war
}
gType_parAPG_APG_Pathplanning_Cross_st; //!< copydoc Type_parAPG_APG_Pathplanning_Cross_st


//! parameter cluster APG_APG_PathplanCPSC_FI: no valid cluster description available
typedef struct Type_parAPG_APG_PathplanCPSC_FI_st
{
  SInt16 VeloForwMoveFast_si16; //!< This value describes the highest driving velocity assumed during a forward move Note that this param
  UInt8 FinalPosXTolPer_ui8; //!< defines how much additional space at the vehicle side is allowed to use for longitudinal vehicle shi
  UInt16 FinalPosXTolMaxF10_ui16; //!< This is the upper limit of the tolerance in the planning of the final position See explanation above
  SInt16 TargetDistSideAlignment_si16; //!< If the vehicle aligns to one longitudinal reference not centered in the middle of the slot this para
  SInt16 SafetyDistCornerAndSide_si16; //!< Safety distance of the vehicle to the parking space corner and side The critical side is the left si
  SInt16 SafetyDistParkingSideBackwMove_si16; //!< Defines the minimal safety distance to be kept to obstacles on the parking side for a backward move
  SInt16 SafetyDistParkingSideForwMove_si16; //!< Defines the minimal safety distance to be kept to obstacles on the parking side for a forward move A
  SInt16 VirtualOppositeSideMove3F10_si16; //!< defines a max lateral distance which is allowed for the backward move 3 This parameter is especially
  SInt16 VirtualOppositeSideMove1F10_si16; //!< defines a max lateral distance which is allowed for straight backward moves The straight backward mo
  UInt16 MaxLengthStraightBackwMoveF10_ui16; //!< Limit on the length of a backward move 1 Appl Process Customer Requirements
  SInt16 MaxStartYawForStraightBackwMove_si16; //!< Limit the range of allowed yaw angles for a straight backward move 1 which is used from start positi
  SInt16 MinDepthForVehicleInsidePS_si16; //!< If the vehicle contour has crossed this virtual depth line the vehicle is logically considered to ha
  SInt16 MaxYStraightBackwMove_si16; //!< If the vehicle is considered Inchinside the parking spaceInch as by the definition of the parameter
  SInt16 ConductorMoveTargetYaw_si16; //!< This parameter modifies the path planning of CPSC FI straight backward moves in the case that no sta
  SInt16 ConductorMovePreferredYaw_si16; //!< If the target position of a conductor move is above this value the conductor move is preferred over
  Boolean SteerLeftInBackwMoveInPS_bl; //!< Steering left in a backward move starting INSIDE the parking space is usually not allowed except in
  Boolean OptimizeBackwMoveEndYaw_bl; //!< This switch influences how a left turn backward move the move following a conductor move or the firs
  Boolean ForceForwardMove_bl; //!< Force a forward move at the beginning of guidance in a CPSC FI Activation in Maneuver AiM situation
  SInt16 XLimit1stBackwMove_si16; //!< defines a maneuvering space limit in negative X direction street orientation for the first backward
}
gType_parAPG_APG_PathplanCPSC_FI_st; //!< copydoc Type_parAPG_APG_PathplanCPSC_FI_st


//! parameter cluster APG_APG_POC: no valid cluster description available
typedef struct Type_parAPG_APG_POC_st
{
  SInt16 ClothoidLength_si16; //!< Length of clothoid for steering within first move Note This parameter is used only when APGCFG_POC_F
  SInt16 MaxSWA_si16; //!< steering wheel angle for POC moves Note This value is used for steering after gear change at the beg
  SInt16 SWAOvershoot_si16; //!< If the steering is set to SWA_Max and the EPS is released the angle is turning back a bit in neutral
  UInt8 MaxNumOfMoves_ui8; //!< Max number of moves for POC
  SInt16 MaxMoveLength_si16; //!< Max length of a POC move Note POC guidance will be aborted after exceeding this length
  UInt8 SteerThresStraightRevMove_ui8; //!< if APGCFG_POC_FIRSTBACKWARDMOVESTRAIGHT is set to SW_ON and the parameter has a value 0 the steering
  UInt8 SteerThresStraightRevMoveMin_ui8; //!< If the vehicle starts with a distance to the rear reference below this threshold no final steering o
  UInt16 LatRefDistInitial_ui16; //!< inital curbstone distance
}
gType_parAPG_APG_POC_st; //!< copydoc Type_parAPG_APG_POC_st


//! parameter cluster APG_CPSC: no valid cluster description available
typedef struct Type_parAPG_CPSC_st
{
  UInt16 BO_StraightMoveLength_ui16; //!< Length of the straight backward pull out move performed when pulling backward out of a perpendicular
  UInt16 FO_StraightMoveLength_ui16; //!< Length of the straight forward pull out move performed when pulling backward out of a perpendicular
}
gType_parAPG_CPSC_st; //!< copydoc Type_parAPG_CPSC_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- APG_PL_POC                                                             -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup APG_PL_POC_M04           exported data types
 * \ingroup COMPONENT_APG_PL_POC
 @{ */

//! parameter cluster APG_PL_POC_Handover: no valid cluster description available
typedef struct Type_parAPG_PL_POC_Handover_st
{
  UInt8 SafetyMargin_ui8; //!< This parameter applies if OLO_BASED_HANDOVER is set The remeasured front reference will be increased
  UInt16 DeactThreshold_MaxExtensionFrontCorner_ui16; //!< If DEACT_ON_MAX_OLO_EXTENSION_REACHED is set this parameter specifies the max extension of the front
  UInt32 DeactThreshold_MaxYawAngle_ui32; //!< if DEACT_ON_MAX_YAW_ANGLE_REACHED is set this parameter specifies a vehicle yaw angle that must not
  UInt32 YawAngleThreshold_ui32; //!< If yaw angle based handover is turned on this parameter specifies the vehicle yaw angle that will tr
}
gType_parAPG_PL_POC_Handover_st; //!< copydoc Type_parAPG_PL_POC_Handover_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- PAGLOBAL                                                               -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup PAGLOBAL_M04           exported data types
 * \ingroup COMPONENT_PAGLOBAL
 @{ */

//! parameter cluster PAGLOBAL_SysFct: no valid cluster description available
typedef struct Type_parPAGLOBAL_SysFct_st
{
  Bitfield8 PpParallelFunc_bf8 : 8; //!< Configuration of the system functions that shall be executed as parallel mode function This paramete
  Boolean RearAxleSteering_bl; //!< If the value is zero false then two wheel steering is active 2WS Otherwise four wheel steering is ac
}
gType_parPAGLOBAL_SysFct_st; //!< copydoc Type_parPAGLOBAL_SysFct_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- DSW                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup DSW_M04           exported data types
 * \ingroup COMPONENT_DSW
 @{ */

/*!@} */


/*--------------------------------------------------------------------------*/
/*- VARIANT                                                                -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup VARIANT_M04           exported data types
 * \ingroup COMPONENT_VARIANT
 @{ */

/*!@} */


/*--------------------------------------------------------------------------*/
/*- SIP                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup SIP_M04           exported data types
 * \ingroup COMPONENT_SIP
 @{ */

//! parameter cluster SIP_VHOParaAutocalib: Section headline only visual structuring
typedef struct Type_parSIP_VHOParaAutocalib_st
{
  UInt8 NumStableTTCCycles_ui8; //!< TRB
  UInt8 NumStableSWACycles_ui8; //!< TRB
  UInt8 NumStableTyreRadCycles_ui8; //!< TRB
  UInt8 NumStableAxlebaseCycles_ui8; //!< TRB
  UInt8 IndexOfTyreRadCluster_ui8; //!< TRB
  SInt16 SWOffDegRes_si16; //!< TRB
  UInt16 AvgWICLengthRearAxle_ui16; //!< TRB
  UInt16 AxlebaseRear_ui16; //!< TRB
  UInt16 ScalFacRR_ui16; //!< TRB
  UInt16 ScalFacLF_ui16; //!< TRB
  UInt16 ScalFacRF_ui16; //!< TRB
}
gType_parSIP_VHOParaAutocalib_st; //!< copydoc Type_parSIP_VHOParaAutocalib_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- VHO                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup VHO_M04           exported data types
 * \ingroup COMPONENT_VHO
 @{ */

//! parameter cluster VHO_Veh: Section headline only visual structuring
typedef struct Type_parVHO_Veh_st
{
  UInt16 DifFrontWheelRadiiTable_pui16[33]; //!< If the vehicle is rear axle driven or 4 wheel driven the wheel pulse signals WIC from the front axle
  UInt8 DrivenAxle_ui8; //!< Driven axle of the vehicle Depending on this parameter the wheel pulse signals that are used for pos
  UInt16 WheelBase_ui16; //!< distance L between front and rear axle This parameter is set manually in the MATLAB generator VHOgen
  UInt16 AxleBaseRear_ui16; //!< Distance b between the centers of both rear axle tires This parameter is automatically generated fro
  UInt16 WICTeethCount_ui16; //!< Number of teeth of the wheel pulse counters per tire This parameter is needed to convert RPM to km/h
  UInt16 WICLength_ui16; //!< Distance that the tire is passing with one WIC impulse It is possible to generate this parameter wit
  UInt16 MaxWICValue_ui16; //!< max value of the wheel impulse signal most frequently with a wic signal of n bit the valid range is
  SInt16 MultMonomA_ForwSWA2SA_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt16 MultMonomB_ForwSWA2SA_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt32 CoeffC_ForwSWA2SA_si32; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt32 CoeffD_ForwSWA2SA_si32; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt16 MultMonomA_BackSWA2SA_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt16 MultMonomB_BackSWA2SA_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt32 CoeffC_BackSWA2SA_si32; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt32 CoeffD_BackSWA2SA_si32; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt16 MultMonomA_ForwSA2SWAL_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt16 MultMonomB_ForwSA2SWAL_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt32 CoeffC_ForwSA2SWAL_si32; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt32 CoeffD_ForwSA2SWAL_si32; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt16 MultMonomA_BackSA2SWAL_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt16 MultMonomB_BackSA2SWAL_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt32 CoeffC_BackSA2SWAL_si32; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt32 CoeffD_BackSA2SWAL_si32; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt16 MultMonomA_ForwSA2SWAC_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt16 MultMonomB_ForwSA2SWAC_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt32 CoeffC_ForwSA2SWAC_si32; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt32 CoeffD_ForwSA2SWAC_si32; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt16 MultMonomA_BackSA2SWAC_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt16 MultMonomB_BackSA2SWAC_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt32 CoeffC_BackSA2SWAC_si32; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt32 CoeffD_BackSA2SWAC_si32; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt16 MultMonomA_ForwSA2SWAR_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt16 MultMonomB_ForwSA2SWAR_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt32 CoeffC_ForwSA2SWAR_si32; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt32 CoeffD_ForwSA2SWAR_si32; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt16 MultMonomA_BackSA2SWAR_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt16 MultMonomB_BackSA2SWAR_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt32 CoeffC_BackSA2SWAR_si32; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt32 CoeffD_BackSA2SWAR_si32; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt16 ClippingAngle1_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  SInt16 ClippingAngle2_si16; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  Bitfield8 ScalingMonomA_SA2SWAL_bf3 : 3; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  Bitfield8 ScalingMonomB_SA2SWAL_bf3 : 3; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  Bitfield8 ScalingMonomCD_SA2SWAL_bf3 : 3; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  Bitfield8 ScalingMonomA_SA2SWAC_bf3 : 3; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  Bitfield8 ScalingMonomB_SA2SWAC_bf3 : 3; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  Bitfield8 ScalingMonomCD_SA2SWAC_bf3 : 3; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  Bitfield8 ScalingMonomA_SA2SWAR_bf3 : 3; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  Bitfield8 ScalingMonomB_SA2SWAR_bf3 : 3; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  Bitfield8 ScalingMonomCD_SA2SWAR_bf3 : 3; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  Bitfield8 ScalingMonomA_SWA2SA_bf3 : 3; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  Bitfield8 ScalingMonomB_SWA2SA_bf3 : 3; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  Bitfield8 ScalingMonomCD_SWA2SA_bf3 : 3; //!< parameter of steering characteristic curve application via VHOgen Matlab Tooling
  UInt8 WIC_CycleTime_MS_ui8; //!< time between two consecutive WIC messages on the network the PSx ECU is connected to Normally derive
}
gType_parVHO_Veh_st; //!< copydoc Type_parVHO_Veh_st


//! parameter cluster VHO_Odo: no valid cluster description available
typedef struct Type_parVHO_Odo_st
{
  UInt8 WeightSingleTrack_ui8; //!< weights of single track model when a combined model is used with a weight md_VHOWeightSingleTrack_ui
  UInt8 WeightDualTrack_ui8; //!< weights of dual track model when a combined model is used with a weight md_VHOWeightSingleTrack_ui8
  UInt8 WeightDualTrackRear_ui8; //!< weights of rear axle in dual track model with a weight md_VHOWeightDualTrackRear_ui8 for the rear ax
  UInt8 WeightDualTrackFront_ui8; //!< weights of front axle in dual track model with a weight md_VHOWeightDualTrackRear_ui8 for the rear a
  UInt32 MaxDistDiscarded_ui32; //!< maximum driven distance that is allowed to be discarded in a parking process a driven distance will
  UInt32 MaxDistDirUndef_ui32; //!< Maximum distance that the vehicle is allowed to drive until the rolling direction has to be availabl
  UInt16 VCALPhaseTol_ui16; //!< inside the wic signal processing the possible change of the wic increments per cycle is estimated th
  UInt16 VCALVeloDependentPhaseTol_ui16; //!< the parameter defines the velocity depending adaptive increase/decrease of wic increments or decreme
  Boolean VCALMinDeltaZeroAllowed_bl; //!< parameter defines whether a zero increment will ALWAYS be allowed 1 or not 0 this parameter should b
  UInt8 WeightDistanceFront_ui8; //!< weight of the front axle in driven distance calculation with md_VHOWeightDistanceFront_ui8 and md_VH
  UInt8 WeightDistanceRear_ui8; //!< weight of the rear axle in driven distance calculation with md_VHOWeightDistanceFront_ui8 and md_VHO
  UInt16 VIRTUAL_LSCAN_TCYC_MS_ui16; //!< Only needed for projects which have nothing but a LowSpeed CAN VHO simulates a virtual low speed can
  UInt16 VIRTUAL_LSCAN_TTOL_MS_ui16; //!< Only needed for projects which have nothing but a LowSpeed CAN md_VIRTUAL_LSCAN_TTOL_MS_ui16 is used
  UInt8 YawAngSmoothKoeffMin_ui8; //!< the yaw angle calculated by the VHO is smoothed with an exponential moving average filter the filter
  UInt8 YawAngSmoothKoeffMax_ui8; //!< the yaw angle calculated by the VHO is smoothed with an exponential moving average filter the filter
  UInt8 EngineRestartExtrapolationDistance_ui8; //!< Max allowed distance cm that the vehicle is allowed to pass in degredation mode InchextrapolationInc
  UInt16 EngineRestartExtrapolationTimeout_ui16; //!< Max allowed time that the system is allowed to remain in degredation mode InchextrapolationInch Afte
  UInt8 SWADelayUnitCM_ui8; //!< This parameter can be used to improve the distance to curbstone a lower value increases the distance
  SInt16 SWAMaxDeviationDeg_si16; //!< If SWADelayUnitCM is 0 this parameter has to be set Compare deviation of steering wheel angles in th
  UInt8 DirDetectMode_ui8; //!< Has to be set if DIRDETECT_MODE LS_VHO_DIRDETECT_ALL is chosen not to be set in customer projects Ot
  UInt8 MinWICDirForMovDir_ui8; //!< If vehicles are equiped with activeWICs the component VHO prefers vehicle direction information base
  UInt8 MinDistAftStdstlNoWICDir_ui8; //!< If vehicles are equiped with activeWICs the component VHO prefers vehicle direction information base
  UInt32 MaxYawAngCorrAtStandstill_ui32; //!< If steering wheel angle changes very much while vehicle stands still it is possible that the vehicle
}
gType_parVHO_Odo_st; //!< copydoc Type_parVHO_Odo_st


/*!@} */


/*--------------------------------------------------------------------------*/
/*- MP_USER_MODE                                                           -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_USER_MODE_M04           exported data types
 * \ingroup COMPONENT_MP_USER_MODE
 @{ */

/*!@} */


/*--------------------------------------------------------------------------*/
/*- MP_BDA                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_BDA_M04           exported data types
 * \ingroup COMPONENT_MP_BDA
 @{ */

//! parameter cluster MP_BDA_Block1: no valid cluster description available
typedef struct Type_parMP_BDA_Block1_st
{
  UInt8 DataBlock1_pui8[9]; //!< USS characteristic curve set block 1 contains all echo sensitivity threshold limits 2 outer front US
}
gType_parMP_BDA_Block1_st; //!< copydoc Type_parMP_BDA_Block1_st


//! parameter cluster MP_BDA_Block2: no valid cluster description available
typedef struct Type_parMP_BDA_Block2_st
{
  UInt8 DataBlock2_pui8[7]; //!< USS characteristic curve set block 2 contains all time based sampling poins for the echo sensitivity
}
gType_parMP_BDA_Block2_st; //!< copydoc Type_parMP_BDA_Block2_st


//! parameter cluster MP_BDA_Block3: no valid cluster description available
typedef struct Type_parMP_BDA_Block3_st
{
  UInt8 DataBlock3_pui8[5]; //!< USS characteristic curve set 3 contains parameter for amplification modifiation of CE duration of se
}
gType_parMP_BDA_Block3_st; //!< copydoc Type_parMP_BDA_Block3_st


/*!@} */


//#pragma ghs struct_min_alignment()


//
// CAL Header Low
//


//
// parameter memory ROM
//

//#pragma ghs struct_min_alignment(4)

typedef struct Type_parParameterMemoryROM_st
{
  gType_parVHOSRC_PluginSRC_st VHOSRC_PluginSRC_pst[1];
  gType_parVHOTCE_PluginTTC_st VHOTCE_PluginTTC_pst[1];
  gType_parVHOTCE_PluginTCE_st VHOTCE_PluginTCE_pst[1];
  gType_parVHOTCE_PluginTCE_clustering_st VHOTCE_PluginTCE_clustering_pst[1];
  gType_parAP_PS_Config_st AP_PS_Config_pst[1];
  gType_parAP_PS_Speed_st AP_PS_Speed_pst[1];
  gType_parAP_PS_Brake_st AP_PS_Brake_pst[1];
  gType_parAP_PS_Objects_st AP_PS_Objects_pst[1];
  gType_parBDA_Parameter_st BDA_Parameter_pst[1];
  gType_parPSD_Veh_st PSD_Veh_pst[1];
  gType_parPSD_Proj_st PSD_Proj_pst[1];
  gType_parPSD_Proj_OCP_st PSD_Proj_OCP_pst[1];
  gType_parPSD_Proj_Cross_st PSD_Proj_Cross_pst[1];
  gType_parPSD_RuleSet1_st PSD_RuleSet1_pst[1];
  gType_parPSD_Proj_VC_st PSD_Proj_VC_pst[1];
  gType_parPSD_Proj_SPR_st PSD_Proj_SPR_pst[1];
  gType_parMP_USS_FISelection_st MP_USS_FISelection_pst[1];
  gType_parMP_USS_FIBlock1_st MP_USS_FIBlock1_pst[1];
  gType_parMP_USS_FIBlock2_st MP_USS_FIBlock2_pst[1];
  gType_parMP_USS_RISelection_st MP_USS_RISelection_pst[1];
  gType_parMP_USS_RIBlock1_st MP_USS_RIBlock1_pst[1];
  gType_parMP_USS_RIBlock2_st MP_USS_RIBlock2_pst[1];
  gType_parMP_USS_FOSelection_st MP_USS_FOSelection_pst[30];
  gType_parMP_USS_FOBlock1_st MP_USS_FOBlock1_pst[30];
  gType_parMP_USS_FOBlock2_st MP_USS_FOBlock2_pst[4];
  gType_parMP_USS_ROSelection_st MP_USS_ROSelection_pst[30];
  gType_parMP_USS_ROBlock1_st MP_USS_ROBlock1_pst[30];
  gType_parMP_USS_ROBlock2_st MP_USS_ROBlock2_pst[4];
  gType_parMP_USS_CcSetInfo_st MP_USS_CcSetInfo_pst[1];
  gType_parAPCTL_CFG_st APCTL_CFG_pst[2];
  gType_parDISPLAY_Veh_st DISPLAY_Veh_pst[1];
  gType_parPSM_DiscardCriteria_st PSM_DiscardCriteria_pst[1];
  gType_parPSM_DiscardCriteriaCPSC_st PSM_DiscardCriteriaCPSC_pst[1];
  gType_parPSM_ParkableCriteria_st PSM_ParkableCriteria_pst[1];
  gType_parPSM_ParkableCriteriaCPSC_st PSM_ParkableCriteriaCPSC_pst[1];
  gType_parPSM_FilterCriteria_st PSM_FilterCriteria_pst[1];
  gType_parPSM_ParametersDPSC_st PSM_ParametersDPSC_pst[1];
  gType_parMP_SensorTimes_st MP_SensorTimes_pst[18];
  gType_parMP_SensorStatic_st MP_SensorStatic_pst[18];
  gType_parMP_PdcMeasGroupParId_st MP_PdcMeasGroupParId_pst[5];
  gType_parMP_Timings_st MP_Timings_pst[2];
  gType_parMP_SensorType_st MP_SensorType_pst[1];
  gType_parMP_SensorOrientation_st MP_SensorOrientation_pst[1];
  gType_parMP_RingTimeInfor_st MP_RingTimeInfor_pst[1];
  gType_parMP_RingTimeRange_st MP_RingTimeRange_pst[4];
  gType_parMP_SensorPort_st MP_SensorPort_pst[1];
  gType_parOD_General_st OD_General_pst[1];
  gType_parOD_MaskActVelo_st OD_MaskActVelo_pst[1];
  gType_parOD_IdleDist_st OD_IdleDist_pst[2];
  gType_parOD_Sens_st OD_Sens_pst[2];
  gType_parOD_Veh_st OD_Veh_pst[2];
  gType_parOD_Echo_ApFlt_st OD_Echo_ApFlt_pst[1];
  gType_parOD_Echo_WRF_st OD_Echo_WRF_pst[1];
  gType_parOD_Echo_SRF_st OD_Echo_SRF_pst[1];
  gType_parOD_Echo_HLF_st OD_Echo_HLF_pst[1];
  gType_parOD_Echo_MoveDir_st OD_Echo_MoveDir_pst[1];
  gType_parOD_Echo_Clip_Std_st OD_Echo_Clip_Std_pst[1];
  gType_parOD_Obj_Meas_st OD_Obj_Meas_pst[1];
  gType_parOD_Obj_Trck_st OD_Obj_Trck_pst[1];
  gType_parSP_PWR_CFG_st SP_PWR_CFG_pst[1];
  gType_parVHS_PL_TORQUE_Controller_ZFLS_st VHS_PL_TORQUE_Controller_ZFLS_pst[1];
  gType_parVHS_PL_TORQUE_Controller_CR_st VHS_PL_TORQUE_Controller_CR_pst[1];
  gType_parAP_BSD_Ap_st AP_BSD_Ap_pst[1];
  gType_parEP_BSD_Ep_st EP_BSD_Ep_pst[1];
  gType_parEP_BSD_SDI_st EP_BSD_SDI_pst[1];
  gType_parAP_PDC_General_st AP_PDC_General_pst[1];
  gType_parAP_PDC_Disp_Global_st AP_PDC_Disp_Global_pst[1];
  gType_parAP_PDC_Disp_Lat_st AP_PDC_Disp_Lat_pst[1];
  gType_parAP_PDC_View_Lat_st AP_PDC_View_Lat_pst[1];
  gType_parAP_PDC_View_Prm_st AP_PDC_View_Prm_pst[1];
  gType_parAP_PDC_Offset_Std_st AP_PDC_Offset_Std_pst[1];
  gType_parAP_PDC_Offset_Lat_st AP_PDC_Offset_Lat_pst[2];
  gType_parAP_PDC_SC_DT_Std_st AP_PDC_SC_DT_Std_pst[1];
  gType_parAP_PDC_SC_DT_Lat_st AP_PDC_SC_DT_Lat_pst[1];
  gType_parAP_PDC_SC_DU_Lat_st AP_PDC_SC_DU_Lat_pst[1];
  gType_parAP_PDC_SC_DU_Prm_st AP_PDC_SC_DU_Prm_pst[1];
  gType_parAP_PDC_SC_OE_Std_st AP_PDC_SC_OE_Std_pst[1];
  gType_parAP_PDC_SC_OE_Lat_st AP_PDC_SC_OE_Lat_pst[1];
  gType_parAP_PDC_SC_SWD_Std_st AP_PDC_SC_SWD_Std_pst[1];
  gType_parAP_PDC_SC_SWD_Lat_st AP_PDC_SC_SWD_Lat_pst[1];
  gType_parAP_PDC_SC_SWD_Prm_st AP_PDC_SC_SWD_Prm_pst[1];
  gType_parAP_PDC_SC_WBL_Prm_st AP_PDC_SC_WBL_Prm_pst[1];
  gType_parAP_PDC_AO_st AP_PDC_AO_pst[1];
  gType_parAP_PDC_Danger_st AP_PDC_Danger_pst[1];
  gType_parPSU_CPSC_st PSU_CPSC_pst[1];
  gType_parPSU_PSC_st PSU_PSC_pst[1];
  gType_parPSU_POC_st PSU_POC_pst[1];
  gType_parPSU_CPX_st PSU_CPX_pst[1];
  gType_parPSU_Object_st PSU_Object_pst[1];
  gType_parPSU_Reference_st PSU_Reference_pst[1];
  gType_parAPG_XPG_Vehicle_st APG_XPG_Vehicle_pst[1];
  gType_parAPG_XPG_SafetyDistParallel_st APG_XPG_SafetyDistParallel_pst[1];
  gType_parAPG_XPG_SafetyDistCross_st APG_XPG_SafetyDistCross_pst[1];
  gType_parAPG_XPG_Ctrl_st APG_XPG_Ctrl_pst[1];
  gType_parAPG_XPG_NMove_st APG_XPG_NMove_pst[1];
  gType_parAPG_XPG_Parkable_st APG_XPG_Parkable_pst[1];
  gType_parAPG_APG_Pathplanning_Parallel_st APG_APG_Pathplanning_Parallel_pst[1];
  gType_parAPG_APG_Steer_Ctrl_st APG_APG_Steer_Ctrl_pst[1];
  gType_parAPG_APG_Ctrl_st APG_APG_Ctrl_pst[1];
  gType_parAPG_APG_Pathplanning_Cross_st APG_APG_Pathplanning_Cross_pst[1];
  gType_parAPG_APG_PathplanCPSC_FI_st APG_APG_PathplanCPSC_FI_pst[1];
  gType_parAPG_APG_POC_st APG_APG_POC_pst[1];
  gType_parAPG_CPSC_st APG_CPSC_pst[1];
  gType_parAPG_PL_POC_Handover_st APG_PL_POC_Handover_pst[1];
  gType_parPAGLOBAL_SysFct_st PAGLOBAL_SysFct_pst[1];
  gType_parVHO_Veh_st VHO_Veh_pst[1];
  gType_parVHO_Odo_st VHO_Odo_pst[1];
  gType_parMP_BDA_Block1_st MP_BDA_Block1_pst[1];
  gType_parMP_BDA_Block2_st MP_BDA_Block2_pst[1];
  gType_parMP_BDA_Block3_st MP_BDA_Block3_pst[1];
}
gType_parParameterMemoryROM_st;

//#pragma ghs struct_min_alignment()


//
// parameter memory RAM
//

typedef struct Type_parParameterMemoryRAM_st
{
  gType_parSIP_VHOParaAutocalib_st SIP_VHOParaAutocalib_pst[1];
}
gType_parParameterMemoryRAM_st;


//
// major variant meta data
//

typedef struct Type_parMajorVarMetaData_st
{
  // index to minor variant meta data
  UInt16 MinorVarMetaDataIdx_pui16[gd_PAR_NUM_MAJOR_VARIANTS_uix];
}
gType_parMajorVarMetaData_st;


//
// minor variant meta data
//

typedef struct Type_parMinorVarMetaData_st
{
  // number of variants
  UInt8 NumVariants_ui8;

  // index of sub variant meta data
  UInt16 SubVarMetaDataIdx_ui16;
}
gType_parMinorVarMetaData_st;


//
// sub variant meta data
//

typedef struct Type_parSubVarMetaData_st
{
  // number of variants
  UInt8 NumVariants_ui8;

  // sub variant data offset
  UInt8 DataOffset_ui8;
}
gType_parSubVarMetaData_st;


//
// application memory
//

#if ( GS_PAR_CFG_APPLICATION_MODE == SW_ON )

typedef struct Type_parApplicationMemory_st
{
  gType_parVHOSRC_PluginSRC_st VHOSRC_PluginSRC_st;
  gType_parVHOTCE_PluginTTC_st VHOTCE_PluginTTC_st;
  gType_parVHOTCE_PluginTCE_st VHOTCE_PluginTCE_st;
  gType_parVHOTCE_PluginTCE_clustering_st VHOTCE_PluginTCE_clustering_st;
  gType_parAP_PS_Config_st AP_PS_Config_st;
  gType_parAP_PS_Speed_st AP_PS_Speed_st;
  gType_parAP_PS_Brake_st AP_PS_Brake_st;
  gType_parAP_PS_Objects_st AP_PS_Objects_st;
  gType_parBDA_Parameter_st BDA_Parameter_st;
  gType_parPSD_Veh_st PSD_Veh_st;
  gType_parPSD_Proj_st PSD_Proj_st;
  gType_parPSD_Proj_OCP_st PSD_Proj_OCP_st;
  gType_parPSD_Proj_Cross_st PSD_Proj_Cross_st;
  gType_parPSD_RuleSet1_st PSD_RuleSet1_st;
  gType_parPSD_Proj_VC_st PSD_Proj_VC_st;
  gType_parPSD_Proj_SPR_st PSD_Proj_SPR_st;
  gType_parMP_USS_FISelection_st MP_USS_FISelection_st;
  gType_parMP_USS_FIBlock1_st MP_USS_FIBlock1_st;
  gType_parMP_USS_FIBlock2_st MP_USS_FIBlock2_st;
  gType_parMP_USS_RISelection_st MP_USS_RISelection_st;
  gType_parMP_USS_RIBlock1_st MP_USS_RIBlock1_st;
  gType_parMP_USS_RIBlock2_st MP_USS_RIBlock2_st;
  gType_parMP_USS_FOSelection_st MP_USS_FOSelection_st;
  gType_parMP_USS_FOBlock1_st MP_USS_FOBlock1_st;
  gType_parMP_USS_FOBlock2_st MP_USS_FOBlock2_st;
  gType_parMP_USS_ROSelection_st MP_USS_ROSelection_st;
  gType_parMP_USS_ROBlock1_st MP_USS_ROBlock1_st;
  gType_parMP_USS_ROBlock2_st MP_USS_ROBlock2_st;
  gType_parMP_USS_CcSetInfo_st MP_USS_CcSetInfo_st;
  gType_parAPCTL_CFG_st APCTL_CFG_st;
  gType_parDISPLAY_Veh_st DISPLAY_Veh_st;
  gType_parPSM_DiscardCriteria_st PSM_DiscardCriteria_st;
  gType_parPSM_DiscardCriteriaCPSC_st PSM_DiscardCriteriaCPSC_st;
  gType_parPSM_ParkableCriteria_st PSM_ParkableCriteria_st;
  gType_parPSM_ParkableCriteriaCPSC_st PSM_ParkableCriteriaCPSC_st;
  gType_parPSM_FilterCriteria_st PSM_FilterCriteria_st;
  gType_parPSM_ParametersDPSC_st PSM_ParametersDPSC_st;
  gType_parMP_SensorTimes_st MP_SensorTimes_pst[6];
  gType_parMP_SensorStatic_st MP_SensorStatic_pst[6];
  gType_parMP_PdcMeasGroupParId_st MP_PdcMeasGroupParId_st;
  gType_parMP_Timings_st MP_Timings_st;
  gType_parMP_SensorType_st MP_SensorType_st;
  gType_parMP_SensorOrientation_st MP_SensorOrientation_st;
  gType_parMP_RingTimeInfor_st MP_RingTimeInfor_st;
  gType_parMP_RingTimeRange_st MP_RingTimeRange_st;
  gType_parMP_SensorPort_st MP_SensorPort_st;
  gType_parOD_General_st OD_General_st;
  gType_parOD_MaskActVelo_st OD_MaskActVelo_st;
  gType_parOD_IdleDist_st OD_IdleDist_st;
  gType_parOD_Sens_st OD_Sens_st;
  gType_parOD_Veh_st OD_Veh_st;
  gType_parOD_Echo_ApFlt_st OD_Echo_ApFlt_st;
  gType_parOD_Echo_WRF_st OD_Echo_WRF_st;
  gType_parOD_Echo_SRF_st OD_Echo_SRF_st;
  gType_parOD_Echo_HLF_st OD_Echo_HLF_st;
  gType_parOD_Echo_MoveDir_st OD_Echo_MoveDir_st;
  gType_parOD_Echo_Clip_Std_st OD_Echo_Clip_Std_st;
  gType_parOD_Obj_Meas_st OD_Obj_Meas_st;
  gType_parOD_Obj_Trck_st OD_Obj_Trck_st;
  gType_parSP_PWR_CFG_st SP_PWR_CFG_st;
  gType_parVHS_PL_TORQUE_Controller_ZFLS_st VHS_PL_TORQUE_Controller_ZFLS_st;
  gType_parVHS_PL_TORQUE_Controller_CR_st VHS_PL_TORQUE_Controller_CR_st;
  gType_parAP_BSD_Ap_st AP_BSD_Ap_st;
  gType_parEP_BSD_Ep_st EP_BSD_Ep_st;
  gType_parEP_BSD_SDI_st EP_BSD_SDI_st;
  gType_parAP_PDC_General_st AP_PDC_General_st;
  gType_parAP_PDC_Disp_Global_st AP_PDC_Disp_Global_st;
  gType_parAP_PDC_Disp_Lat_st AP_PDC_Disp_Lat_st;
  gType_parAP_PDC_View_Lat_st AP_PDC_View_Lat_st;
  gType_parAP_PDC_View_Prm_st AP_PDC_View_Prm_st;
  gType_parAP_PDC_Offset_Std_st AP_PDC_Offset_Std_st;
  gType_parAP_PDC_Offset_Lat_st AP_PDC_Offset_Lat_st;
  gType_parAP_PDC_SC_DT_Std_st AP_PDC_SC_DT_Std_st;
  gType_parAP_PDC_SC_DT_Lat_st AP_PDC_SC_DT_Lat_st;
  gType_parAP_PDC_SC_DU_Lat_st AP_PDC_SC_DU_Lat_st;
  gType_parAP_PDC_SC_DU_Prm_st AP_PDC_SC_DU_Prm_st;
  gType_parAP_PDC_SC_OE_Std_st AP_PDC_SC_OE_Std_st;
  gType_parAP_PDC_SC_OE_Lat_st AP_PDC_SC_OE_Lat_st;
  gType_parAP_PDC_SC_SWD_Std_st AP_PDC_SC_SWD_Std_st;
  gType_parAP_PDC_SC_SWD_Lat_st AP_PDC_SC_SWD_Lat_st;
  gType_parAP_PDC_SC_SWD_Prm_st AP_PDC_SC_SWD_Prm_st;
  gType_parAP_PDC_SC_WBL_Prm_st AP_PDC_SC_WBL_Prm_st;
  gType_parAP_PDC_AO_st AP_PDC_AO_st;
  gType_parAP_PDC_Danger_st AP_PDC_Danger_st;
  gType_parPSU_CPSC_st PSU_CPSC_st;
  gType_parPSU_PSC_st PSU_PSC_st;
  gType_parPSU_POC_st PSU_POC_st;
  gType_parPSU_CPX_st PSU_CPX_st;
  gType_parPSU_Object_st PSU_Object_st;
  gType_parPSU_Reference_st PSU_Reference_st;
  gType_parAPG_XPG_Vehicle_st APG_XPG_Vehicle_st;
  gType_parAPG_XPG_SafetyDistParallel_st APG_XPG_SafetyDistParallel_st;
  gType_parAPG_XPG_SafetyDistCross_st APG_XPG_SafetyDistCross_st;
  gType_parAPG_XPG_Ctrl_st APG_XPG_Ctrl_st;
  gType_parAPG_XPG_NMove_st APG_XPG_NMove_st;
  gType_parAPG_XPG_Parkable_st APG_XPG_Parkable_st;
  gType_parAPG_APG_Pathplanning_Parallel_st APG_APG_Pathplanning_Parallel_st;
  gType_parAPG_APG_Steer_Ctrl_st APG_APG_Steer_Ctrl_st;
  gType_parAPG_APG_Ctrl_st APG_APG_Ctrl_st;
  gType_parAPG_APG_Pathplanning_Cross_st APG_APG_Pathplanning_Cross_st;
  gType_parAPG_APG_PathplanCPSC_FI_st APG_APG_PathplanCPSC_FI_st;
  gType_parAPG_APG_POC_st APG_APG_POC_st;
  gType_parAPG_CPSC_st APG_CPSC_st;
  gType_parAPG_PL_POC_Handover_st APG_PL_POC_Handover_st;
  gType_parPAGLOBAL_SysFct_st PAGLOBAL_SysFct_st;
  gType_parSIP_VHOParaAutocalib_st SIP_VHOParaAutocalib_st;
  gType_parVHO_Veh_st VHO_Veh_st;
  gType_parVHO_Odo_st VHO_Odo_st;
  gType_parMP_BDA_Block1_st MP_BDA_Block1_st;
  gType_parMP_BDA_Block2_st MP_BDA_Block2_st;
  gType_parMP_BDA_Block3_st MP_BDA_Block3_st;
}
gType_parApplicationMemory_st;

#endif


//
// global data ROM
//

typedef struct Type_parROMData_st
{
  // version information
  gType_parInfoData_st InfoData_st;

  gType_parParameterMemoryROM_st ParameterMemoryROM_st;
}
gType_parROMData_st;


//
// global meta data ROM
//

typedef struct Type_parROMMetaData_st
{
  void const * IndData_ppvd[gd_PAR_NUM_CLUSTERS_uix];

  #if ( GS_PAR_CFG_INIT_NVM_MIRROR_MEM == SW_ON )
    gType_parDefaultCopyData_st DefaultCopyData_pst[1];
  #endif

  #if ( GS_PAR_CFG_APPLICATION_MODE == SW_ON )
    void * const ApplMemClusterPointer_ppvd[gd_PAR_NUM_CLUSTERS_uix];
  #endif

  UInt16 const ClusterSize_pui16[gd_PAR_NUM_CLUSTERS_uix];

  #if ( GS_PAR_CFG_APPLICATION_MODE == SW_ON )
    UInt8 const ApplSubVariantMode_pui8[gd_PAR_NUM_CLUSTERS_uix];
  #endif

  #if ( GS_PAR_CFG_APPLICATION_MODE == SW_ON )
    UInt8 const MaxApplSubVariants_pui8[gd_PAR_NUM_CLUSTERS_uix];
  #endif

  gType_parMajorVarMetaData_st MajorVarMetaData_pst[gd_PAR_NUM_CLUSTERS_uix];
  gType_parMinorVarMetaData_st MinorVarMetaData_pst[gd_PAR_NUM_MINOR_VAR_META_DATA_uix];
  gType_parSubVarMetaData_st   SubVarMetaData_pst[gd_PAR_NUM_SUB_VAR_META_DATA_uix];
}
gType_parROMMetaData_st;


//
// global data RAM
//

typedef struct Type_parRAMData_st
{
  gType_parParameterMemoryRAM_st ParameterMemoryRAM_st;
  void * ClusterPointer_ppvd[gd_PAR_NUM_CLUSTERS_uix];

  // [x][0] -> minor variant / [x][1] -> sub variant
  #if ( GS_PAR_CFG_APPLICATION_MODE == SW_ON )
    UInt8 VariantMemory_ppui8[gd_PAR_NUM_CLUSTERS_uix][2];
  #endif
}
gType_parRAMData_st;

/*--------------------------------------------------------------------------*/
/*- external object declarations                                           -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- RAM                                                                    -*/
/*--------------------------------------------------------------------------*/

#ifdef COMPONENT_PAR

//
// application memory
//

#if ( GS_PAR_CFG_APPLICATION_MODE == SW_ON )

  #if ( (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_STAND_ALONE) || (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_APPL_ONLY) )
  extern gType_parApplicationMemory_st g_parApplicationMemory_st;
  #elif (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_CAL_ONLY)
    // not included in this configuration
  #else
    #error 'invalid configuration!'
  #endif

#endif

#endif // COMPONENT_PAR

//
// global data RAM
//

#if ( (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_STAND_ALONE) || (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_APPL_ONLY) )
  extern gType_parRAMData_st g_parRAMData_st;
#elif (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_CAL_ONLY)
  // not included in this configuration
#else
  #error 'invalid configuration!'
#endif

/*--------------------------------------------------------------------------*/
/*- ROM                                                                    -*/
/*--------------------------------------------------------------------------*/

#ifdef COMPONENT_PAR
#endif

#if ( (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_STAND_ALONE) || (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_CAL_ONLY) )
  extern const UInt8 gc_parBlockStatus_pui8[256];
#endif

//
// global data ROM
//

#if ( (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_STAND_ALONE) || (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_CAL_ONLY) )
  extern const gType_parROMData_st gc_parROMData_st;
  extern gType_parROMData_st g_parMemoryData_st;
#elif (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_APPL_ONLY)


  extern UInt8 __ghsbegin_CAL_Const[];


  #define gc_parRAMData_st (*((const gType_parROMData_st *) __ghsbegin_CAL_Const))
#else
  #error 'invalid configuration!'
#endif

#if ( (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_STAND_ALONE) || (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_APPL_ONLY) )
  extern const gType_parROMMetaData_st gc_parROMMetaData_st;
#elif (GS_PAR_CFG_COMP_MODE == GS_PAR_CFG_COMP_MODE_CAL_ONLY)
  // not included in this configuration
#else
  #error 'invalid configuration!'
#endif

/*--------------------------------------------------------------------------*/
/*- prototypes of exported functions                                       -*/
/*--------------------------------------------------------------------------*/

#ifdef COMPONENT_PAR
#endif // COMPONENT_PAR
/* clang-format on */
#endif /* GEN_PAR_H */
