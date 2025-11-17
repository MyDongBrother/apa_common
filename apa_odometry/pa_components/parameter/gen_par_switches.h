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
 | $ProjectName:
 d:/MKS_Data/Projects/PPJ/PP_GEN5/PP_SIP5_Fx4/SW/SRC/PA/DFS_PRJ/SRC/TOOLS/PARGen/pc_pp_pargen.pj
 $ |      $Source: Program.cs $ |    $Revision: 1.220 $ |        $Date: 2012/11/12
 16:37:54CST $ |       $State: in_work $
 |
 |      Purpose: generated component configuration file
 |-----------------------------------------------------------------------------
 | I N F O
 |-----------------------------------------------------------------------------
 | File:                  gen_par_switch.h
 | Input File:            ..\appl_sip_param.xml
 | Version:               5.8
 | Date:                  2023.5.11 - 15:12
 |
 | Generation Date (UTC): 2023.05.11 - 07:13
 *****************************************************************************/

#ifndef GEN_PAR_SWITCHES_H
#define GEN_PAR_SWITCHES_H
/* clang-format off */
/*--------------------------------------------------------------------------*/
/*- magic number                                                           -*/
/*--------------------------------------------------------------------------*/

#define GEN_PAR_SWITCHES_MAGIC_NUMBER  0x05110713

/*--------------------------------------------------------------------------*/
/*- version information                                                    -*/
/*--------------------------------------------------------------------------*/

#ifdef COMPONENT_PAR
  #ifndef gd_PAR_VersionInfo_pc
    #define gd_PAR_VersionInfo_pc 5.8
  #endif
#endif

/*--------------------------------------------------------------------------*/
/*- CAL-section structure integrity hash                                   -*/
/*--------------------------------------------------------------------------*/

#ifdef COMPONENT_PAR



#define gd_PAR_CAL_StructureIntegrityHash_pui16 { 0x98, 0x82, 0xf0, 0xc8, 0xcc, 0x60, 0x98, 0x66, 0xd0, 0xf1, 0x44, 0x94, 0xd5, 0xd7, 0x77, 0xaf, 0x0c, 0x69, 0x2b, 0x8d }



#endif

/*--------------------------------------------------------------------------*/
/*- NVM-section structure integrity hash                                   -*/
/*--------------------------------------------------------------------------*/

#ifdef COMPONENT_PAR



#define gd_PAR_NVM_StructureIntegrityHash_pui16 { 0x15, 0x6e, 0x6c, 0xa0, 0xf6, 0x85, 0xbe, 0x99, 0x8b, 0x76, 0xe7, 0x00, 0x21, 0x20, 0xad, 0xcd, 0xe6, 0xe8, 0x20, 0xbe }



#endif

/*--------------------------------------------------------------------------*/
/*- component switches                                                     -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- compilation mode configuration                                         -*/
/*--------------------------------------------------------------------------*/

#define GS_PAR_CFG_COMP_MODE_STAND_ALONE (1)
#define GS_PAR_CFG_COMP_MODE_CAL_ONLY    (2)
#define GS_PAR_CFG_COMP_MODE_APPL_ONLY   (3)

#ifndef GS_PAR_CFG_COMP_MODE
  #define GS_PAR_CFG_COMP_MODE GS_PAR_CFG_COMP_MODE_STAND_ALONE
#endif

/*--------------------------------------------------------------------------*/
/*- PAR automatically generated                                            -*/
/*--------------------------------------------------------------------------*/

#define GS_PAR_CFG_INIT_NVM_MIRROR_MEM SW_OFF

/*--------------------------------------------------------------------------*/
/*- VHOSRC                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup VHOSRC_M02           exported symbolic constants
 * \ingroup COMPONENT_VHOSRC
 @{ */

#ifndef GS_VHOSRC_SLIP_RECOGNITION
  #define GS_VHOSRC_SLIP_RECOGNITION SW_OFF
#endif

#ifdef COMPONENT_VHOSRC

  #ifndef LS_VHOSRC_SLIP_RECOGNITION_ACC
    #define LS_VHOSRC_SLIP_RECOGNITION_ACC SW_OFF /*!< The calculation of yaw angle and driven distance is influenced by the weights of the vehicle axles I */
  #endif

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- VHOTCE                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup VHOTCE_M02           exported symbolic constants
 * \ingroup COMPONENT_VHOTCE
 @{ */

#ifndef GS_VHOTCE_EXTERNAL_TIRE_CRCUMFERENCE_ESTIMATION
  #define GS_VHOTCE_EXTERNAL_TIRE_CRCUMFERENCE_ESTIMATION SW_OFF
#endif
#ifndef GS_VHOTCE_EXTERNAL_TIRE_TOLERANCE_CALIBRATION
  #define GS_VHOTCE_EXTERNAL_TIRE_TOLERANCE_CALIBRATION SW_OFF
#endif

#ifdef COMPONENT_VHOTCE

  #define gd_VHOTCE_SIZE_LONGACC_HISTORYBUFFER_ui8 ((UInt8)(10)) /*!< Buffer for history of longitutinal acceleration For checking of valid signal quality whole buffer le */
  #define gd_VHOTCE_SIZE_DUALTRACKSMOOTHBUFFER_ui8 ((UInt8)(20)) /*!< Filterlength for smoothing of dualtrack pseudo yawrate Dont change  */
  #define gd_VHOTCE_SIZE_YAWRATEDELAYBUFFER_ui8 ((UInt8)(22)) /*!< Timedelaycompensation between WIC signal and yawrate Normally greater than 0 5 SIZE_DUALTRACKSMOOTHB */
  #define gd_VHOTCE_SIZE_SYNCHRONFILTER_ui8 ((UInt8)(60)) /*!< Filterlength for generation of reference signal in integrator synchronisation  */
  #ifndef LS_VHOTCE_CALIBRATE_AXLEBASE
    #define LS_VHOTCE_CALIBRATE_AXLEBASE SW_OFF /*!< Prototypic Dont activate yet  */
  #endif
  #ifndef LS_VHOTCE_USEGATING
    #define LS_VHOTCE_USEGATING SW_ON /*!< SW_OFF may result in quicker results for driving in resident area but will significantly reduce accu */
  #endif

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- MP_SNOWMUD                                                             -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_SNOWMUD_M02           exported symbolic constants
 * \ingroup COMPONENT_MP_SNOWMUD
 @{ */

#ifndef GS_MP_SNOWMUD_SNOWMUD
  #define GS_MP_SNOWMUD_SNOWMUD SW_OFF
#endif
#define gd_MP_SNOWMUD_MAX_DIST_NEAR_RANGE_ECHO_SIDE_SENSOR_ui16 ((UInt16)(200))
#define gd_MP_SNOWMUD_MAX_DIST_NEAR_RANGE_ECHO_NORMAL_SENSOR_ui16 ((UInt16)(200))

#ifdef COMPONENT_MP_SNOWMUD


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- EOL                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup EOL_M02           exported symbolic constants
 * \ingroup COMPONENT_EOL
 @{ */


#ifdef COMPONENT_EOL


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- AP_PS                                                                  -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup AP_PS_M02           exported symbolic constants
 * \ingroup COMPONENT_AP_PS
 @{ */


#ifdef COMPONENT_AP_PS

  #ifndef LS_AP_PS_PARKSTOP_CALCSTOPDIST
    #define LS_AP_PS_PARKSTOP_CALCSTOPDIST SW_OFF /*!< Turns on the calculation of the stop distance The lookup table is not used prototype  */
  #endif
  #ifndef LS_AP_PS_PARKSTOP_PCBCREEPMODUS
    #define LS_AP_PS_PARKSTOP_PCBCREEPMODUS SW_OFF /*!< Activate calculation of creep modus output for PCB */
  #endif
  #ifndef LS_AP_PS_PARKSTOP_PROTOTYPE
    #define LS_AP_PS_PARKSTOP_PROTOTYPE SW_OFF /*!< Turns on prototype specific code for plattform prototypes must not be set to SW_ON in customer proje */
  #endif
  #ifndef LS_AP_PS_PARKSTOP_EXTERNALMOVETRIGGER
    #define LS_AP_PS_PARKSTOP_EXTERNALMOVETRIGGER SW_OFF /*!< Allows triggering of new move by external CAN signal if switched on */
  #endif
  #ifndef LS_AP_PS_PARKSTOP_OBJECTBRAKING_PSX
    #define LS_AP_PS_PARKSTOP_OBJECTBRAKING_PSX LS_PXB_OFF /*!< Turns on the support for braking for collision relevant objects during park guidance For PCB PCB_ACC */
  #endif
  #ifndef LS_AP_PS_PARKSTOP_OBJECTBRAKING_PDC
    #define LS_AP_PS_PARKSTOP_OBJECTBRAKING_PDC LS_MXB_OFF /*!< Turns on support for braking for collision relevant objects during low speed maneuvers MCB Maneuver  */
  #endif
  #ifndef LS_AP_PS_PARKSTOP_SPEEDLIMITING_PSX
    #define LS_AP_PS_PARKSTOP_SPEEDLIMITING_PSX LS_SLPSX_TORQUE /*!< Turns on braking for uncomfortable speed limiting during park guidance TORQUE using torque interface */
  #endif

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- BDA                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup BDA_M02           exported symbolic constants
 * \ingroup COMPONENT_BDA
 @{ */

#ifndef gd_BDA_EchoHistoMinNumOfShots_ui8
  #define gd_BDA_EchoHistoMinNumOfShots_ui8 ((UInt8)(4))
#endif
#ifndef gd_BDA_EchoHistoThreshold_ui8
  #define gd_BDA_EchoHistoThreshold_ui8 ((UInt8)(5))
#endif
#ifndef gd_BDA_EchoHistoMaxCount_ui8
  #define gd_BDA_EchoHistoMaxCount_ui8 ((UInt8)(7))
#endif
#ifndef GS_BDA_CANAPE_INTERFACE
  #define GS_BDA_CANAPE_INTERFACE SW_ON
#endif

#ifdef COMPONENT_BDA


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- FUS                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup FUS_M02           exported symbolic constants
 * \ingroup COMPONENT_FUS
 @{ */

#ifndef GS_FUS_RESET_PATH
  #define GS_FUS_RESET_PATH SW_OFF
#endif
#ifndef GS_FUS_TEST_PATH
  #define GS_FUS_TEST_PATH SW_OFF
#endif
#ifndef GS_FUS_DEBUG_SWITCH
  #define GS_FUS_DEBUG_SWITCH SW_OFF
#endif

#ifdef COMPONENT_FUS


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- OOC                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup OOC_M02           exported symbolic constants
 * \ingroup COMPONENT_OOC
 @{ */


#ifdef COMPONENT_OOC


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- MP_DIAG_TEST                                                           -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_DIAG_TEST_M02           exported symbolic constants
 * \ingroup COMPONENT_MP_DIAG_TEST
 @{ */

#ifndef GS_MP_DIAG_TEST_DIAG_TEST
  #define GS_MP_DIAG_TEST_DIAG_TEST SW_OFF
#endif

#ifdef COMPONENT_MP_DIAG_TEST


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- PSD                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup PSD_M02           exported symbolic constants
 * \ingroup COMPONENT_PSD
 @{ */

#define GS_PSD_ALLOW_OP_MODE_ALL SW_ON
#define GS_PSD_ADI_DYNAMIC_OBJECTS SW_ON
#ifndef GS_PSD_USE_REAR_SENSORS
  #define GS_PSD_USE_REAR_SENSORS SW_OFF
#endif
#define GS_PSD_SPOS_INFO_DYN_OBJECTS SW_OFF
#define GS_PSD_CEM SW_OFF
#define GS_PSD_SCENE_OCP4 SW_OFF

#ifndef LS_PSD_USSMAP_REAR_SENSOR_ACTIVE
  #define LS_PSD_USSMAP_REAR_SENSOR_ACTIVE SW_ON /*!< Synchronize the vehicle state calculated by the odometry module not only with the front USS measurem */
#endif

#ifndef LS_PSD_SEARCH_METHOD_1
  #define LS_PSD_SEARCH_METHOD_1  ((0))   /*!< Use Only Sensor1 or Sensor6 to search for parking spaces forward or backword*/
#endif

#ifndef LS_PSD_SEARCH_METHOD_2
  #define LS_PSD_SEARCH_METHOD_2  ((1))    /*!< Use Sensor1 or Sensor6 to search for parking spaces forward And Use Sensor7 or Sensor12 to search for parking spaces Backward*/
#endif

#ifndef LS_PSD_SEARCH_METHOD
  #define LS_PSD_SEARCH_METHOD  LS_PSD_SEARCH_METHOD_2  /*if set LS_PSD_SEARCH_METHOD_1, then set LS_PSD_USSMAP_REAR_SENSOR_ACTIVE SW_OFF*/
                                                        /*if set LS_PSD_SEARCH_METHOD_2, then set LS_PSD_USSMAP_REAR_SENSOR_ACTIVE SW_ON*/
#endif

#ifdef COMPONENT_PSD

  #ifndef LS_PSD_PARALLELPARKING
    #define LS_PSD_PARALLELPARKING SW_ON /*!< SW_OFF parallel parking is not activated SW_ON parallel parking is activated */
  #endif
  #ifndef LS_PSD_RULE_SET
    #define LS_PSD_RULE_SET LS_PSD_RULE_SET_FREE_CONFIG /*!< Switch to activate the standard full configurable SIP target or a special alignment rule set for a s */
  #endif
  #ifndef LS_PSD_CROSS_TYPE
    #define LS_PSD_CROSS_TYPE LS_PSD_CROSS_TYPE_VIRTUAL_ALL /*!< LS_PSD_CROSS_TYPE Switch to activate several types of cross parking spaces Available are Parking bet */
  #endif
  #define LS_PSD_APPLICATION_OUTPUT SW_ON /*!< SW_OFF deactivate visibility of all PS information SW_ON activate visibility of all PS information c */
  #define LS_PSD_ALIGNMENT_RULE_PARALLEL LS_PSD_ALIGNMENT_RULE_PARALLEL_1 /*!< Switch to activate different rules for the alignment in case of parrallel parking spaces LS_PSD_ALIG */
  #define LS_PSD_NORMAL_PARKING_CURB_WALL SW_ON /*!< This switch is to activate the wall curbstone classification If the switch is set to SW_ON In case t */
  #define LS_PSD_ADVANCED_OBJECT_CLASSIFICATION SW_ON /*!< This switch is to activate a special object classification SW_ON The 2nd echo in combination with ot */
  #define LS_PSD_MERGE_OBSTACLE_WITH_CAR SW_ON /*!< Switch to activite merging of small obstacles with near cars in a defined area internal parameters P */
  #define LS_PSD_PS_ORIENTATION LS_PSD_PS_ORIENTATION_PASSING /*!< PS_ORIENTATION_PASSING orientation of parking space in situation without curbstone is based on avera */
  #ifndef LS_PSD_ADVANCED_PILLAR_DETECTION
    #define LS_PSD_ADVANCED_PILLAR_DETECTION SW_ON /*!< Switch to activate special detection of pillars Pro The chance of detecting a narrow pillar echo ref */
  #endif
  #define LS_PSD_SCENE_CROSS_OCP SW_OFF /*!< Switch to activate parking on curbstone for cross parking Scenes can be reprocessed in order to sepa */
  #define LS_PSD_MEASUREMENT_NRCLUTRANSFORM SW_ON /*!< Switch to activate additional information about nr of transformation cluster object for Canape added */
  #ifndef LS_PSD_OCP_ONCURVE
    #define LS_PSD_OCP_ONCURVE SW_OFF /*!< Switch to activate the functionality of parking on curbstone in a curve If activated also OCP4 has t */
  #endif
  #ifndef LS_PSD_SCENE_PARALLEL
    #define LS_PSD_SCENE_PARALLEL LS_PSD_SCENE_PARALLEL_3 /*!< Switch to activate different scenes for parallel parking For scene constellation see figure LS_PSD_S */
  #endif
  #define LS_PSD_SCENE_OCP2 LS_PSD_SCENE_OCP2_2 /*!< Switch to activate different scenes for parking on curbstone with 2 wheels For scene constellation s */
  #ifndef LS_PSD_SCENE_VIRTUAL
    #define LS_PSD_SCENE_VIRTUAL LS_PSD_SCENE_VIRTUAL_5 /*!< Switch to activate different scenes for virtual parking For detailed description of this parameter s */
  #endif
  #ifndef LS_PSD_MEASUREMENT
    #define LS_PSD_MEASUREMENT SW_ON /*!< Compiler switch which enables measurements with CANape additional RAM is needed if activated Off 0 O */
  #endif
  #define gd_PSD_NO_ECHOES_ui8 ((UInt8)(3)) /*!< Number of US echoes that shall be copied with each measurement from the VHM This value should be equ */

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- MP_USS                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_USS_M02           exported symbolic constants
 * \ingroup COMPONENT_MP_USS
 @{ */


#ifdef COMPONENT_MP_USS

  #define gd_MP_USS_INNER_SENSORS_FRONT_ui16 ((UInt16)(30)) /*!< Mask with bit set for a USS_IF channel on ECU ASIC used for SVA front sensors */
  #define gd_MP_USS_OUTER_SENSORS_FRONT_ui16 ((UInt16)(33)) /*!< Mask with bit set for a USS_IF channel on ECU ASIC used for SVA front sensors */
  #define gd_MP_USS_INNER_SENSORS_REAR_ui16 ((UInt16)(3840)) /*!< Mask with bit set for a USS_IF channel on ECU ASIC used for SVA front sensors */
  #define gd_MP_USS_OUTER_SENSORS_REAR_ui16 ((UInt16)(4224)) /*!< Mask with bit set for a USS_IF channel on ECU ASIC used for SVA front sensors */
  #define gd_MP_USS_SVA_SENSORS_FRONT_ui16 ((UInt16)(33)) /*!< Mask with bit set for a USS_IF channel on ECU ASIC used for SVA front sensors */
  #define gd_MP_USS_SVA_SENSORS_REAR_ui16 ((UInt16)(4224)) /*!< Mask with bit set for a USS_IF channel on ECU ASIC used for SVA rear sensors */
  #ifndef LS_MP_USS_PSX_USE_REAR_SENSORS
    #define LS_MP_USS_PSX_USE_REAR_SENSORS SW_OFF /*!< To use the rear outer sensors for PSX Switch is activating temperature dependant PSX Parameter for t */
  #endif

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- MP_INTERFERENCE                                                        -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_INTERFERENCE_M02           exported symbolic constants
 * \ingroup COMPONENT_MP_INTERFERENCE
 @{ */

#ifndef GS_MP_INTERFERENCE_PL_INTERF
  #define GS_MP_INTERFERENCE_PL_INTERF SW_ON
#endif

#ifdef COMPONENT_MP_INTERFERENCE


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- APCTL                                                                  -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup APCTL_M02           exported symbolic constants
 * \ingroup COMPONENT_APCTL
 @{ */


#ifdef COMPONENT_APCTL


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- DISPLAY                                                                -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup DISPLAY_M02           exported symbolic constants
 * \ingroup COMPONENT_DISPLAY
 @{ */


#ifdef COMPONENT_DISPLAY


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- ACS                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup ACS_M02           exported symbolic constants
 * \ingroup COMPONENT_ACS
 @{ */

#ifndef GS_ACS_ASIC_VERSION
  #define GS_ACS_ASIC_VERSION GS_ACS_ASIC_VERSION_CA171AA
#endif
#define GS_ACS_ENABLE_UNIFIED_COM_BUFFER SW_ON
#define GS_ACS_WATCHDOG_MONITORING SW_OFF

#ifdef COMPONENT_ACS

  #define LS_ACS_SWAP_TX_BUFFER_BYTE_ORDER SW_OFF /*!< Swap byte order of the tx buffer before transmission Setting depends on capabilities of the BSW SPI  */
  #define LS_ACS_SWAP_RX_BUFFER_BYTE_ORDER SW_OFF /*!< Swap byte order of the rx buffer after reception Setting depends on capabilities of the BSW SPI driv */
  #define gd_ACS_CLOCK_PRESCALER_ui8 ((UInt8)(1)) /*!< ASIC clock pre scaler Only needed if the ASIC clock is externally supplied CLK SEL pin connected to  */
  #define gd_ACS_CHARGE_PUMP_CLOCK_ui8 ((UInt8)(0)) /*!< ASIC HSS charge pump clock selection  */
  #define gd_ACS_RESET_THRESHOLD_ui8 ((UInt8)(1)) /*!< Reset threshold sensitivity of the microcontroller external reset input pin  */
  #define gd_ACS_SPI_LEVEL_ui8 ((UInt8)(1)) /*!< SPI communication level e g NEC V850 has 5V a PPC usually uses a level of 3 3V  */
  #define LS_ACS_MEMORY_LAYOUT LS_ACS_MEMORY_LAYOUT_NORMAL /*!< Selects the ASIC s memory layout Old projects should use the legacy layout  */

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- PSM                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup PSM_M02           exported symbolic constants
 * \ingroup COMPONENT_PSM
 @{ */

#ifndef GS_PSM_SIDEPROTECTION
  #define GS_PSM_SIDEPROTECTION SW_ON
#endif
#ifndef GS_PSM_SPR_OBJECTS
  #define GS_PSM_SPR_OBJECTS SW_OFF
#endif
#define gd_PSM_SPRMaxNumOfPoints_ui8 ((UInt8)(20))
#ifndef gd_PSM_NumOfParkingSpaces_ui8
  #define gd_PSM_NumOfParkingSpaces_ui8 ((UInt8)(4))
#endif
#define gd_PSM_MaxNumOfObjects_ui8 ((UInt8)(6))
#ifndef GS_PSM_DUALPARKABLE
  #define GS_PSM_DUALPARKABLE SW_ON
#endif

#ifdef COMPONENT_PSM

  #define LS_PSM_NEWPSADDMODE LS_MPS_NEWPSADDMODE_KEEPVALID /*!< When the MPS Buffer is full with parking spaces this switch determines how the parking slots in the  */
  #define LS_PSM_ONGUIDANCE LS_MPS_ONGUIDANCE_KEEPSELECTED /*!< Configuration to determine how the PSs in the buffer other than the selected PS must be handled on A */
  #define LS_PSM_VIRTUALCORNERDISCARDREF LS_MPS_VIRTUALCORNERDISCARDREF_VIRTUAL /*!< This switch is applicable only if DISCARDSPOSREF LS_MPS_DISCARDSPOSREF_FRONT and if PSL detects seco */
  #define LS_PSM_DISCARDSPOSREF LS_MPS_DISCARDSPOSREF_FRONT /*!< Sets the reference for sPos Discard Criteria either 1st corner or 2nd corner f Parking Space  */
  #define LS_PSM_DISCARDPSREARREFREACHED SW_OFF /*!< Enable or disable discarding for the parking space if vehicle reaches rear reference if not already  */
  #define LS_PSM_ALLOWNOSELECTEDSIDE SW_OFF /*!< This switch allows parking even without Indicator set applicable for Right Parking Space  */
  #define LS_PSM_PARKABLECRITERIA_PARKABLERESET SW_ON /*!< For parking spaces that meet parkable criteria the AP_MPS resets the Parkability flag from ??PSOkPosO */
  #ifndef LS_PSM_DISP_OBJECTS
    #define LS_PSM_DISP_OBJECTS SW_OFF /*!< Enable or Disable display of Objects on HMI SW_ON Provides interface to Display Objects on HMI SW_OF */
  #endif
  #define LS_PSM_CALCULATEPARKABLITY LS_MPS_CALCULATEPARKABLITY_ALL /*!< When there are multiple PSs in the MPS buffer this switch setting determines which Parking Slots mus */
  #ifndef LS_PSM_STOREUNIVERSALPSAFTERPARKABLE
    #define LS_PSM_STOREUNIVERSALPSAFTERPARKABLE SW_ON /*!< For Universal PSs Parking slots where both Cross and Parallel Parking is possible when this switch i */
  #endif
  #define LS_PSM_EXTERNAL_PS_SELECTION SW_OFF /*!< The switch is provided to have an external selection of parking slot for parking SW_ON Parking slots */

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- MP                                                                     -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_M02           exported symbolic constants
 * \ingroup COMPONENT_MP
 @{ */

#ifndef GS_MP_USE_USS_ASIC_WORKAROUNDS
  #define GS_MP_USE_USS_ASIC_WORKAROUNDS SW_OFF
#endif
#ifndef gd_MP_MAX_USED_SENSORS_ui8
  #define gd_MP_MAX_USED_SENSORS_ui8 ((UInt8)(14))
#endif
#define GS_MP_DEBUG_ERROR SW_OFF
#define GS_MP_TEST_BUILD SW_OFF
#ifndef GS_MP_NUMBER_OF_MULTI_ECHOES
  #define GS_MP_NUMBER_OF_MULTI_ECHOES 4
#endif
#ifndef gd_MP_NUMBER_OF_PAS_ECHOES_ui8
  #define gd_MP_NUMBER_OF_PAS_ECHOES_ui8 ((UInt8)(4))
#endif
#ifndef gd_MP_NUMBER_OF_PSX_ECHOES_ui8
  #define gd_MP_NUMBER_OF_PSX_ECHOES_ui8 ((UInt8)(2))
#endif
#ifndef gd_MP_NUMBER_OF_SVA_ECHOES_ui8
  #define gd_MP_NUMBER_OF_SVA_ECHOES_ui8 ((UInt8)(6))
#endif
#ifndef gd_MP_NUMBER_OF_POC_ECHOES_ui8
  #define gd_MP_NUMBER_OF_POC_ECHOES_ui8 ((UInt8)(4))
#endif
#define gd_MP_USS_IF_UBATT_THRESHOLD_ui8 ((UInt8)(106))
#define gd_MP_RINGING_DROP_BORDER_ui16 ((UInt16)(1200))
#define gd_MP_MAX_RINGING_DROP_WIDTH_si16 ((SInt16)(80))
#define gd_MP_RINGING_DROP_BORDER_TESTMODE_ui16 ((UInt16)(1600))
#define gd_MP_MAX_RINGING_DROP_WIDTH_TESTMODE_si16 ((SInt16)(150))
#define gd_MP_CLOSE_OBJ_DIS_SPEED_SIDE_SENSOR_ui16 ((UInt16)(200))
#define gd_MP_CLOSE_OBJ_DIS_SPEED_ui16 ((UInt16)(20))
#ifndef GS_MP_INCLUDE_ADAPTIVE_RINGDOWN_ALGO
  #define GS_MP_INCLUDE_ADAPTIVE_RINGDOWN_ALGO SW_OFF
#endif
#ifndef GS_MP_ADAPTIVE_RINGDOWN_USE_BASESENSOR
  #define GS_MP_ADAPTIVE_RINGDOWN_USE_BASESENSOR SW_OFF
#endif

#ifdef COMPONENT_MP

  #ifndef LS_MP_CFG_FRONT_2_SENSORS
    #define LS_MP_CFG_FRONT_2_SENSORS SW_OFF /*!< Enable/disables the support for a 2 sensor configuration valid for front system ADI Sensor 3 and 4 a */
  #endif
  #ifndef LS_MP_CFG_FRONT_2E_SENSORS
    #define LS_MP_CFG_FRONT_2E_SENSORS SW_OFF /*!< Enable/disables the support for a specific sensor configuration valid for front system For each sens */
  #endif
  #ifndef LS_MP_CFG_FRONT_3_SENSORS
    #define LS_MP_CFG_FRONT_3_SENSORS SW_OFF /*!< Enable/disables the support for a specific sensor configuration valid for front system For each sens */
  #endif
  #ifndef LS_MP_CFG_FRONT_4_SENSORS
    #define LS_MP_CFG_FRONT_4_SENSORS SW_OFF /*!< Enable/disables the support for a specific sensor configuration valid for front system For each sens */
  #endif
  #ifndef LS_MP_CFG_FRONT_5_SENSORS
    #define LS_MP_CFG_FRONT_5_SENSORS SW_OFF /*!< Enable/disables the support for a specific sensor configuration valid for front system For each sens */
  #endif
  #ifndef LS_MP_CFG_FRONT_6_SENSORS
    #define LS_MP_CFG_FRONT_6_SENSORS SW_ON /*!< Enable/disables the support for a specific sensor configuration valid for front system For each sens */
  #endif
  #ifndef LS_MP_CFG_REAR_2_SENSORS
    #define LS_MP_CFG_REAR_2_SENSORS SW_OFF /*!< Enable/disables the support for a 2 sensor configuration valid for rear system ADI Sensor 10 and 11  */
  #endif
  #ifndef LS_MP_CFG_REAR_2E_SENSORS
    #define LS_MP_CFG_REAR_2E_SENSORS SW_OFF /*!< Enable/disables the support for a specific sensor configuration valid for rear system For each senso */
  #endif
  #ifndef LS_MP_CFG_REAR_3_SENSORS
    #define LS_MP_CFG_REAR_3_SENSORS SW_OFF /*!< Enable/disables the support for a specific sensor configuration valid for rear system For each senso */
  #endif
  #ifndef LS_MP_CFG_REAR_4_SENSORS
    #define LS_MP_CFG_REAR_4_SENSORS SW_OFF /*!< Enable/disables the support for a specific sensor configuration valid for rear system For each senso */
  #endif
  #ifndef LS_MP_CFG_REAR_5_SENSORS
    #define LS_MP_CFG_REAR_5_SENSORS SW_OFF /*!< Enable/disables the support for a specific sensor configuration valid for rear system For each senso */
  #endif
  #ifndef LS_MP_CFG_REAR_6_SENSORS
    #define LS_MP_CFG_REAR_6_SENSORS SW_ON /*!< Enable/disables the support for a specific sensor configuration valid for rear system For each senso */
  #endif
  #ifndef LS_MP_TEST_MODE_SUPPORT
    #define LS_MP_TEST_MODE_SUPPORT SW_ON /*!< This switch enables the support for the USS test functionality in the MP module  */
  #endif
  #define LS_MP_NUMBER_OF_ECHO_EVENTS 6 /*!< Default value of echo events The MP will configure the RT module in ECU ASIC to this value per defau */
  #ifndef LS_MP_SUPPRES_RINGING_DROP
    #define LS_MP_SUPPRES_RINGING_DROP SW_OFF /*!< With this switch it is possible to activate a special treatment for echoes close to the end of the r */
  #endif
  #ifndef LS_MP_USS_SLOW_PROT_SPEED
    #define LS_MP_USS_SLOW_PROT_SPEED SW_OFF /*!< Which USS protocol speed shall be used HighSpeed PP5 or LowSpeed PP5C see USS documentation for deta */
  #endif
  #define gd_MP_OUTSIDE_TEMP_DEFAULT_si8 ((SInt8)(20)) /*!< Definition of the default value for outside temperature used by MP in case the value is returned fro */
  #define gd_MP_DDW_DURATION_ui8 ((UInt8)(2)) /*!< Definition of the duration of the disturbance detection window DDW Horchfenster The value should be  */
  #define gd_MP_MAX_DISTURBANCE_DURATION_ui16 ((UInt16)(500)) /*!< definition of the disturbance duration threshold at least 4 125us in ??s  */
  #define gd_MP_RUNTIME_TEST_PHASE_1_ui16 ((UInt16)(100)) /*!< Test runtime for Usens short to Ubat test in 10 ms  */
  #define gd_MP_RUNTIME_TEST_PHASE_2_ui16 ((UInt16)(600)) /*!< Test runtime for sensor error test in 10 ms  */
  #define gd_MP_USS_b1_ui16 ((UInt16)(94)) /*!< tx protocol timings between ECU ASIC and Sensor in ??s  */
  #define gd_MP_USS_b2_ui16 ((UInt16)(188)) /*!< tx protocol timings between ECU ASIC and Sensor in ??s  */
  #define gd_MP_USS_b3_ui16 ((UInt16)(281)) /*!< tx protocol timings between ECU ASIC and Sensor in ??s  */
  #define gd_MP_USS_b4_ui16 ((UInt16)(375)) /*!< tx protocol timings between ECU ASIC and Sensor in ??s  */
  #define gd_MP_USS_a1_ui16 ((UInt16)(94)) /*!< rx protocol timings between Sensor and ECU ASIC in ??s  */
  #define gd_MP_USS_a2_ui16 ((UInt16)(188)) /*!< rx protocol timings between Sensor and ECU ASIC in ??s  */
  #define gd_MP_USS_a3_ui16 ((UInt16)(281)) /*!< rx protocol timings between Sensor and ECU ASIC in ??s  */
  #define gd_MP_USS_a4_ui16 ((UInt16)(375)) /*!< rx protocol timings between Sensor and ECU ASIC in ??s  */
  #define gd_MP_USS_ar_ui16 ((UInt16)(1125)) /*!< rx protocol timings between Sensor and ECU ASIC in ??s  */
  #define gd_MP_USS_IF_RX_FILTER_ui16 ((UInt16)(20)) /*!< filter time of the USS_IF Rx channels in ??s  */
  #define gd_MP_USS_TS_ui16 ((UInt16)(42)) /*!< delay of the Tx activation of the USS in ??s  */
  #define gd_MP_USS_TMF_ui16 ((UInt16)(13)) /*!< delay of USS monoflop in ??s  */
  #define gd_MP_USS_IF_TSG_POS_ui16 ((UInt16)(20)) /*!< delay of USS_IF direction ECU USS in ??s  */
  #define gd_MP_USS_IF_TSG_NEG_ui16 ((UInt16)(5)) /*!< delay of USS_IF direction ECU USS in ??s  */
  #define gd_MP_USS_IF_TSEN_POS_ui16 ((UInt16)(14)) /*!< delay of USS_IF direction USS ECU in ??s  */
  #define gd_MP_USS_IF_TSEN_NEG_ui16 ((UInt16)(6)) /*!< delay of USS_IF direction USS ECU in ??s  */
  #define gd_MP_SPEED_BORDER_LOW_MID_ui16 ((UInt16)(20)) /*!< Border between Low speed minimal wait time and mid speed minimal wait time resolution 0 1 km/h  */
  #define gd_MP_SPEED_BORDER_MID_HIGH_ui16 ((UInt16)(100)) /*!< Border between mid speed minimal wait time and high speed minimal wait time resolution 0 1 km/h  */
  #define gd_MP_LOW_SPEED_MIN_TIME_ui16 ((UInt16)(27550)) /*!< minimal time after sending out ultra sonic waves before next sending is allowed for low speed static */
  #define gd_MP_MID_SPEED_MIN_TIME_ui16 ((UInt16)(22550)) /*!< minimal time after sending out ultra sonic waves before next sending is allowed for mid speed dynami */
  #define gd_MP_HIGH_SPEED_MIN_TIME_ui16 ((UInt16)(14500)) /*!< minimal time after sending out ultra sonic waves before next sending is allowed for high speed dynam */
  #define gd_MP_MEAS_OBJ_TIME_OUT_ui16 ((UInt16)(2000)) /*!< time out for a measurement object which is downloaded to the ASIC but never responsing Time 2 is tak */
  #define gd_MP_MEAS_RINGING_END_TIME_ui16 ((UInt16)(1700)) /*!< Definition of the ringing down time including application add on of a USS in the system without any  */
  #define gd_MP_MEAS_RINGING_START_TIME_ui16 ((UInt16)(850)) /*!< minimum of sensor ringing if a less ringing time is measured it s a Inchringing to shortInch error i */
  #ifndef LS_MP_RINGING_ERR_LONG_PULSE
    #define LS_MP_RINGING_ERR_LONG_PULSE SW_OFF /*!< In case a idle time is configured on the Sensor and a very long pulse over 4095 ??s is active no ring */
  #endif
  #define gd_MP_MAX_NUMBER_OF_SENSORS_ui8 ((UInt8)(14)) /*!< maximal number of sensor positions in the system */
  #define gd_MP_MAX_NUMBER_OF_SENSORS_MASK_ui16 ((UInt16)(0x3fff)) /*!< a mask corresponding to the max number of sensors number of sensor number of bits set  */
  #define gd_MP_MAX_NUMBER_OF_HW_CANNELS_ui8 ((UInt8)(12)) /*!< maximal number of HW Sensor channels in the system */
  #define gd_MP_ECHO_DIST_MIN_ui16 ((UInt16)(10)) /*!< Output echo distance in case of close echo detection no real object distance mm  */
  #define gd_MP_PULS_WIDTH_DIST_MIN_ui16 ((UInt16)(350)) /*!< Output echo pulse width in case of close echo detection no real object distance ??s  */
  #define gd_MP_SENSOR_STATUS_FAST_CHECK_RATE_si16 ((SInt16)(50)) /*!< a fixed faster USS statsu check rate used after Cc set tranfer to validate reception of Cc data in U */

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- MP_BUFF                                                                -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_BUFF_M02           exported symbolic constants
 * \ingroup COMPONENT_MP_BUFF
 @{ */

#ifndef GS_MP_BUFF_MEASBUFFER
  #define GS_MP_BUFF_MEASBUFFER SW_OFF
#endif

#ifdef COMPONENT_MP_BUFF


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- MP_PL                                                                  -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_PL_M02           exported symbolic constants
 * \ingroup COMPONENT_MP_PL
 @{ */

#ifndef GS_MP_PL_OP_MODE
  #define GS_MP_PL_OP_MODE SW_ON
#endif

#ifdef COMPONENT_MP_PL


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- MTL                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MTL_M02           exported symbolic constants
 * \ingroup COMPONENT_MTL
 @{ */

#define GS_MTL_USE_ERROR_HOOK SW_OFF
#ifndef GS_MTL_USE_ASM
  #define GS_MTL_USE_ASM SW_OFF
#endif
#define GS_MTL_TRIGONOMETRIC_FUNC SW_ON
#define GS_MTL_FILTER_FUNC SW_ON
#ifndef GS_MTL_GEOLIB
  #define GS_MTL_GEOLIB SW_ON
#endif

#ifdef COMPONENT_MTL

  #define LS_MTL_TESTHARNESS SW_OFF /*!< The mandatory setting for SW delivery is SW_OFF If component SP_MATHLIB shall be white box tested th */

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- OD                                                                     -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup OD_M02           exported symbolic constants
 * \ingroup COMPONENT_OD
 @{ */

#ifndef GS_OD_MODE
  #define GS_OD_MODE GS_OD_MODE_PREMIUM
#endif
#define GS_OD_USE_VEHICLE_MOVE_DIR SW_ON
#ifndef GS_OD_OBJHEIGHT_CLASSIFICATION
  #define GS_OD_OBJHEIGHT_CLASSIFICATION SW_ON
#endif
#define gd_OD_OUTERMOST_FRONT_LEFT_SENSOR_ui8 ((UInt8)(0))
#define gd_OD_OUTERMOST_FRONT_RIGHT_SENSOR_ui8 ((UInt8)(5))
#define gd_OD_OUTERMOST_REAR_LEFT_SENSOR_ui8 ((UInt8)(0))
#define gd_OD_OUTERMOST_REAR_RIGHT_SENSOR_ui8 ((UInt8)(5))

#ifdef COMPONENT_OD

  #define LS_OD_NEIGHBORHOOD_REL_CHECK SW_ON /*!< Enable/disable code for faster echo acceptance reduction of penalty cycles within the Appearance Fil */
  #define LS_OD_CHECK_ECHO_CROSSRUN SW_ON /*!< Enable/disable code of echo cross run situation detection logic  */
  #define LS_OD_CHECK_ECHO_RELIABILITY SW_ON /*!< Enable/disable code for self reflection filter license plate and heat layer filter Note this switch  */
  #define LS_OD_WHEELREFLECT_FILTER SW_ON /*!< Enable/disable code for the wheel reflection filter only relevant if sensors 1 and 6 are available a */
  #define LS_OD_SELFREFLECT_FILTER SW_OFF /*!< Enable/disable code for the self reflection filter license plate At least 4 sensors must be availabl */
  #ifndef LS_OD_HEAT_LAYER_FILTER
    #define LS_OD_HEAT_LAYER_FILTER SW_OFF /*!< Enable/disable code for the heat layer filter  */
  #endif
  #define LS_OD_EFH_REACTION SW_ON /*!< Enable/disable code for the re initialization of EP_OD after error detection by the FPS SW_ON no EP_ */
  #define LS_OD_METHOD_TRIANGLE SW_ON /*!< Determines the distance calculation method for CE objects no direct echo or cross echo is significan */
  #define LS_OD_MULTI_ECHOS SW_ON /*!< usage of multi echoes for application purposes only  */
  #ifndef LS_OD_APPLICATION
    #define LS_OD_APPLICATION SW_ON /*!< Enable/disable code for additional application possibilities Note SW_ON is only allowed for applicat */
  #endif
  #ifndef LS_OD_PSD_ECHOSTORE_PROCESSING_MODE
    #define LS_OD_PSD_ECHOSTORE_PROCESSING_MODE LS_OD_ESP_MODE_OFF /*!< Enable/Disable edge detection by using the echo store of the outer sensors from the PSD  */
  #endif
  #define LS_OD_6chSDW SW_OFF /*!< Enables 6 channel SDW In this configuration there are no inner front sensor allowed There can be onl */
  #define LS_OD_OBJ_RESET_TRIGGER_AVAILABLE SW_OFF /*!< Enable/disable code for request of external trigger for resetting OD object storage  */

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- PAR                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup PAR_M02           exported symbolic constants
 * \ingroup COMPONENT_PAR
 @{ */

#ifndef GS_PAR_CFG_APPLICATION_MODE
  #define GS_PAR_CFG_APPLICATION_MODE SW_OFF
#endif

#ifdef COMPONENT_PAR


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- SP_FSV                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup SP_FSV_M02           exported symbolic constants
 * \ingroup COMPONENT_SP_FSV
 @{ */

#ifndef GS_SP_FSV_USS_ADI_01
  #define GS_SP_FSV_USS_ADI_01 SW_ON
#endif
#ifndef GS_SP_FSV_USS_ADI_02
  #define GS_SP_FSV_USS_ADI_02 SW_ON
#endif
#ifndef GS_SP_FSV_USS_ADI_03
  #define GS_SP_FSV_USS_ADI_03 SW_ON
#endif
#ifndef GS_SP_FSV_USS_ADI_04
  #define GS_SP_FSV_USS_ADI_04 SW_ON
#endif
#ifndef GS_SP_FSV_USS_ADI_05
  #define GS_SP_FSV_USS_ADI_05 SW_ON
#endif
#ifndef GS_SP_FSV_USS_ADI_06
  #define GS_SP_FSV_USS_ADI_06 SW_ON
#endif
#ifndef GS_SP_FSV_USS_ADI_07
  #define GS_SP_FSV_USS_ADI_07 SW_OFF
#endif
#ifndef GS_SP_FSV_USS_ADI_08
  #define GS_SP_FSV_USS_ADI_08 SW_ON
#endif
#ifndef GS_SP_FSV_USS_ADI_09
  #define GS_SP_FSV_USS_ADI_09 SW_ON
#endif
#ifndef GS_SP_FSV_USS_ADI_10
  #define GS_SP_FSV_USS_ADI_10 SW_ON
#endif
#ifndef GS_SP_FSV_USS_ADI_11
  #define GS_SP_FSV_USS_ADI_11 SW_ON
#endif
#ifndef GS_SP_FSV_USS_ADI_12
  #define GS_SP_FSV_USS_ADI_12 SW_ON
#endif
#ifndef GS_SP_FSV_USS_ADI_13
  #define GS_SP_FSV_USS_ADI_13 SW_ON
#endif
#ifndef GS_SP_FSV_USS_ADI_14
  #define GS_SP_FSV_USS_ADI_14 SW_OFF
#endif
#ifndef GS_SP_FSV_ENABLE_VHO_SYMPTOMS
  #define GS_SP_FSV_ENABLE_VHO_SYMPTOMS SW_ON
#endif
#ifndef GS_SP_FSV_INCLUDE_DEBUG_MODULE
  #define GS_SP_FSV_INCLUDE_DEBUG_MODULE SW_ON
#endif
#define GS_SP_FSV_ENG_RESTART_FAULT_HDL SW_ON

#ifdef COMPONENT_SP_FSV

  #define LS_SP_FSV_TIME_FILTER SW_ON /*!< In order to include time based filtering algorithm in project build this switch has to be turned on  */
  #define LS_SP_FSV_EVENT_IN_TIME_FILTER SW_OFF /*!< In order to include event based in time window filtering algorithm in project build this switch has  */
  #define LS_SP_FSV_CNT_FILTER SW_ON /*!< In order to include counter based filtering algorithm in project build this switch has to be turned  */
  #define LS_SP_FSV_AO_FILTER SW_OFF /*!< In order to include AO Attached Object filtering algorithm in project build this switch has to be tu */
  #define LS_SP_FSV_CODABLE_SYM_CFG SW_OFF /*!< If the symptom configurations shall be codable then the value shall be SW_ON The symptom configurati */

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- SP_PWR                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup SP_PWR_M02           exported symbolic constants
 * \ingroup COMPONENT_SP_PWR
 @{ */


#ifdef COMPONENT_SP_PWR


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- SMS                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup SMS_M02           exported symbolic constants
 * \ingroup COMPONENT_SMS
 @{ */

#ifndef GS_SMS_APT_TASK
  #define GS_SMS_APT_TASK SW_ON
#endif
#define GS_SMS_MON GS_SMS_MON_ICE

#ifdef COMPONENT_SMS

  #define gd_SMS_InitTOutCycles_ui8 ((UInt8)(10)) /*!< This parameter defines the number of 10ms cycles within Initialization until a fault is reported The */

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- VHS_PL_TORQUE                                                          -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup VHS_PL_TORQUE_M02           exported symbolic constants
 * \ingroup COMPONENT_VHS_PL_TORQUE
 @{ */


#ifdef COMPONENT_VHS_PL_TORQUE

  #define LS_VHS_PL_TORQUE_CONTROLLER_TYPE_PLUG_IN LS_VHS_PL_ZFLS_CONTROLLER /*!< Switch to chose between the ZFLS controller cascaded PID and the CR controller state based controlle */

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- VHS                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup VHS_M02           exported symbolic constants
 * \ingroup COMPONENT_VHS
 @{ */

#ifndef GS_VHS_VHO_PRESENT
  #define GS_VHS_VHO_PRESENT SW_ON
#endif
#define GS_VHS_UBATT SW_ON
#define GS_VHS_VEH_SPEED SW_ON
#define GS_VHS_GEAR SW_ON
#define GS_VHS_OUTSIDE_TEMP SW_ON
#define GS_VHS_TRAILER_HITCH_PRESENT SW_ON
#define GS_VHS_TRAILER_ATTACHED SW_OFF
#define GS_VHS_CAR_MOVE_DIR SW_ON
#define GS_VHS_SWA SW_ON
#define GS_VHS_INDICATOR SW_ON
#define GS_VHS_WICRR SW_ON
#define GS_VHS_WICRL SW_ON
#define GS_VHS_WICFR SW_ON
#define GS_VHS_WICFL SW_ON
#define GS_VHS_WHEEL_DIR_RR SW_ON
#define GS_VHS_WHEEL_DIR_RL SW_ON
#define GS_VHS_WHEEL_DIR_FR SW_ON
#define GS_VHS_WHEEL_DIR_FL SW_ON
#define GS_VHS_WHEEL_ROTATION_RR SW_ON
#define GS_VHS_WHEEL_ROTATION_RL SW_ON
#define GS_VHS_WHEEL_ROTATION_FR SW_ON
#define GS_VHS_WHEEL_ROTATION_FL SW_ON
#define GS_VHS_LONG_ACC SW_ON
#define GS_VHS_YAW_RATE SW_ON
#define GS_VHS_VEH_DYN_INTERVENTION SW_OFF
#define GS_VHS_EPS_STATUS SW_ON
#define GS_VHS_ENGINE_START SW_ON
#define GS_VHS_OUTPUT_REAR_AXLE_DESIRED_SA SW_OFF
#define GS_VHS_REAR_AXLE_ACTUAL_SA SW_OFF
#define GS_VHS_POWERMODE SW_ON
#define GS_VHS_SYSTEM_ACTIVATION SW_ON
#ifndef GS_VHS_OUTPUT_OD
  #define GS_VHS_OUTPUT_OD SW_ON
#endif
#ifndef GS_VHS_OUTPUT_TORQUE
  #define GS_VHS_OUTPUT_TORQUE SW_OFF
#endif
#ifndef GS_VHS_HANDS_ON_DETECTION
  #define GS_VHS_HANDS_ON_DETECTION SW_OFF
#endif

#ifdef COMPONENT_VHS

  #define LS_VHS_FLTSYMPT_TRIG SW_OFF /*!< This switch activates the VHS Fault symptom trigger mechanism  */
  #ifndef LS_VHS_OUTPUT_SWA_DESIRED
    #define LS_VHS_OUTPUT_SWA_DESIRED SW_ON /*!< This switch activates the SWA desired output signal conversion and calls the corresponding RTE funct */
  #endif
  #define LS_VHS_SIGNAL_SIMULATION SW_OFF /*!< This switch activates the signal simulation all signals are substituted by CANape signals  */
  #ifndef LS_VHS_RAWSIGNAL_DEBUG
    #define LS_VHS_RAWSIGNAL_DEBUG SW_ON /*!< This switch activates the possiblity to measure all raw signals provided by RTE  */
  #endif
  #ifndef LS_VHS_CAN_SPEED_SUBST
    #define LS_VHS_CAN_SPEED_SUBST SW_ON /*!< This switch activates the substitution of the CAN signal for lower speeds it uses the vWICspeed of t */
  #endif
  #define gd_VHS_GearboxFilterTime_ui8 ((UInt8)(40)) /*!<  */
  #define gd_VHS_TaskCycleInMS_ui8 ((UInt8)(20)) /*!<  */

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- AP_BSD                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup AP_BSD_M02           exported symbolic constants
 * \ingroup COMPONENT_AP_BSD
 @{ */


#ifdef COMPONENT_AP_BSD

  #ifndef LS_AP_BSD_HIGHSPEEDTRACKINGFILTER
    #define LS_AP_BSD_HIGHSPEEDTRACKINGFILTER SW_OFF /*!< SW_OFF not activated SW_ON Filter of warning during High Speed Tracking activated */
  #endif

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- EP_BSD                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup EP_BSD_M02           exported symbolic constants
 * \ingroup COMPONENT_EP_BSD
 @{ */

#ifndef GS_EP_BSD_SENSOR_CONFIG
  #define GS_EP_BSD_SENSOR_CONFIG GS_BSD_SENSOR_CONFIG_2F2R
#endif
#ifndef GS_EP_BSD_SDI_ACTIVE
  #define GS_EP_BSD_SDI_ACTIVE SW_ON
#endif
#ifndef GS_EP_BSD_SDI_PERIODIC_OBJECTS
  #define GS_EP_BSD_SDI_PERIODIC_OBJECTS SW_OFF
#endif
#ifndef gd_EP_BSD_SDI_SENSORS_ui8
  #define gd_EP_BSD_SDI_SENSORS_ui8 ((UInt8)(2))
#endif
#ifndef GS_EP_BSD_CANAPE_INTERFACE
  #define GS_EP_BSD_CANAPE_INTERFACE SW_OFF
#endif
#ifndef GS_EP_BSD_INCREASED_DETECTION_RANGE
  #define GS_EP_BSD_INCREASED_DETECTION_RANGE SW_OFF
#endif

#ifdef COMPONENT_EP_BSD

  #ifndef gd_EP_BSD_MAP_USS_TO_BSD_FL_ui8
    #define gd_EP_BSD_MAP_USS_TO_BSD_FL_ui8 ((UInt8)(1)) /*!< Mapping of the sensor numbers to BSD postion front left */
  #endif
  #ifndef gd_EP_BSD_MAP_USS_TO_BSD_FR_ui8
    #define gd_EP_BSD_MAP_USS_TO_BSD_FR_ui8 ((UInt8)(6)) /*!< Mapping of the sensor numbers to BSD postion front right */
  #endif
  #ifndef gd_EP_BSD_MAP_USS_TO_BSD_ML_ui8
    #define gd_EP_BSD_MAP_USS_TO_BSD_ML_ui8 ((UInt8)(14)) /*!< Mapping of the sensor numbers to BSD postion middle left */
  #endif
  #ifndef gd_EP_BSD_MAP_USS_TO_BSD_MR_ui8
    #define gd_EP_BSD_MAP_USS_TO_BSD_MR_ui8 ((UInt8)(7)) /*!< Mapping of the sensor numbers to BSD postion middle right */
  #endif
  #ifndef gd_EP_BSD_MAP_USS_TO_BSD_RR_ui8
    #define gd_EP_BSD_MAP_USS_TO_BSD_RR_ui8 ((UInt8)(9)) /*!< Mapping of the sensor numbers to BSD postion rear right */
  #endif
  #ifndef gd_EP_BSD_MAP_USS_TO_BSD_RL_ui8
    #define gd_EP_BSD_MAP_USS_TO_BSD_RL_ui8 ((UInt8)(12)) /*!< Mapping of the sensor numbers to BSD postion rear left */
  #endif

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- AP_PDC                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup AP_PDC_M02           exported symbolic constants
 * \ingroup COMPONENT_AP_PDC
 @{ */

#ifndef GS_AP_PDC_MODE
  #define GS_AP_PDC_MODE GS_AP_PDC_MODE_PREMIUM
#endif
#define GS_AP_PDC_CTRL GS_AP_PDC_CTRL_SHIFT10MS_TO_OD
#define GS_AP_PDC_DANGER_INDICATOR SW_OFF
#ifndef GS_AP_PDC_NUM_DISP_AREA_FRONT
  #define GS_AP_PDC_NUM_DISP_AREA_FRONT 6
#endif
#ifndef GS_AP_PDC_NUM_DISP_AREA_REAR
  #define GS_AP_PDC_NUM_DISP_AREA_REAR 6
#endif
#ifndef GS_AP_PDC_NUM_DISP_AREA_SIDE
  #define GS_AP_PDC_NUM_DISP_AREA_SIDE 5
#endif
#define gd_AP_PDC_SWD_FRONT_GROUP_LEFT_ui8 ((UInt8)(0))
#define gd_AP_PDC_SWD_FRONT_GROUP_RIGHT_ui8 ((UInt8)(4))
#define gd_AP_PDC_SWD_REAR_GROUP_LEFT_ui8 ((UInt8)(1))
#define gd_AP_PDC_SWD_REAR_GROUP_RIGHT_ui8 ((UInt8)(3))
#define gd_AP_PDC_SWD_FRONT_OUTER_SENSOR_LEFT_ui8 ((UInt8)(0))
#define gd_AP_PDC_SWD_FRONT_CENTRAL_SENSOR_LEFT_ui8 ((UInt8)(1))
#define gd_AP_PDC_SWD_FRONT_CENTRAL_SENSOR_RIGHT_ui8 ((UInt8)(4))
#define gd_AP_PDC_SWD_FRONT_OUTER_SENSOR_RIGHT_ui8 ((UInt8)(5))
#define gd_AP_PDC_SWD_REAR_OUTER_SENSOR_LEFT_ui8 ((UInt8)(1))
#define gd_AP_PDC_SWD_REAR_CENTRAL_SENSOR_LEFT_ui8 ((UInt8)(2))
#define gd_AP_PDC_SWD_REAR_CENTRAL_SENSOR_RIGHT_ui8 ((UInt8)(3))
#define gd_AP_PDC_SWD_REAR_OUTER_SENSOR_RIGHT_ui8 ((UInt8)(4))

#ifdef COMPONENT_AP_PDC

  #ifndef LS_AP_PDC_ADAPTIVE_FULLWARNING
    #define LS_AP_PDC_ADAPTIVE_FULLWARNING SW_OFF /*!< Switch to enable/disable the code for adaptive full warning distance in case of PSC and POC and smal */
  #endif
  #ifndef LS_AP_PDC_COURSE_BASED_WARNINGDISTANCE
    #define LS_AP_PDC_COURSE_BASED_WARNINGDISTANCE SW_OFF /*!< Switch to enable/disable the code for course based distance to collision calculation Switch must be  */
  #endif
  #ifndef LS_AP_PDC_CFG_SC_DIVETHROUGH
    #define LS_AP_PDC_CFG_SC_DIVETHROUGH SW_ON /*!< Switch to enable/disable the code for special case Dive Through warning  */
  #endif
  #ifndef LS_AP_PDC_CFG_SC_DIVEUNDER
    #define LS_AP_PDC_CFG_SC_DIVEUNDER SW_OFF /*!< Switch to enable/disable the code for special case Dive Under warning  */
  #endif
  #ifndef LS_AP_PDC_CFG_SC_OUTEREDGE
    #define LS_AP_PDC_CFG_SC_OUTEREDGE SW_ON /*!< Switch to enable/disable the code for special case Outer Edge warning  */
  #endif
  #ifndef LS_AP_PDC_CFG_SC_OUTEREDGE_INNER
    #define LS_AP_PDC_CFG_SC_OUTEREDGE_INNER SW_ON /*!< Swith to enable/disable the code for extended special case Outer Edge Inner InchEnableInch is used i */
  #endif
  #define LS_AP_PDC_CFG_SC_SIDEWALL SW_OFF /*!< Switch to enable/disable the code for special case Side Wall detection Note For a 2 channel system t */
  #define LS_AP_PDC_SC_WARNBEFORELOST SW_OFF /*!< Switch to enable/disable the code for special case WarnBeforeLost Note Only for premium approach The */
  #define LS_AP_PDC_DISP_STABILIZER SW_ON /*!< Enable/disable the code for display stabilization Note this is a MUST for the lateral zone approach  */
  #define LS_AP_PDC_AO SW_OFF /*!< Switch to enable/disable the code for attached object functionality */
  #ifndef LS_AP_PDC_APPLICATION
    #define LS_AP_PDC_APPLICATION SW_ON /*!< Switch to put application specific variables in RAM Thus an enhanced parameterization during run tim */
  #endif

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- PSU                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup PSU_M02           exported symbolic constants
 * \ingroup COMPONENT_PSU
 @{ */


#ifdef COMPONENT_PSU

  #ifndef LS_PSU_ACTIVE
    #define LS_PSU_ACTIVE SW_ON /*!< Switch PSU functionality on/off In order to provide interfaces all interfaces are implemented as stu */
  #endif
  #ifndef LS_PSU_PSC
    #define LS_PSU_PSC SW_ON /*!< Switch for activation of parking space correction in parallel parking situations  */
  #endif
  #ifndef LS_PSU_CPSC
    #define LS_PSU_CPSC SW_ON /*!< Switch for activation of parking space correction in cross parking situations  */
  #endif
  #ifndef LS_PSU_CPSC_FI
    #define LS_PSU_CPSC_FI SW_ON /*!< Switch for activation of parking space correction in front in cross parking situations  */
  #endif
  #ifndef LS_PSU_POC
    #define LS_PSU_POC SW_ON /*!< Switch for activation of parking space measurement while pull out of parallel parking space  */
  #endif
  #ifndef LS_PSU_ORIENTATION
    #define LS_PSU_ORIENTATION SW_ON /*!< Enable orientation estimation of parking space boundaries Currently only implemented for left and ri */
  #endif
  #ifndef LS_PSU_POLES
    #define LS_PSU_POLES SW_OFF /*!< Enable support of poles within parking space boundaries Useful only for CPSC parking space left and  */
  #endif
  #ifndef LS_PSU_CPX_SCENE_APPROACHING_STRAIGHT
    #define LS_PSU_CPX_SCENE_APPROACHING_STRAIGHT SW_OFF /*!< Enables search for parking slots during the approach with an angle near to target orientation This s */
  #endif
  #ifndef LS_PSU_CPX_SCENE_APPROACHING_OBLIQUE
    #define LS_PSU_CPX_SCENE_APPROACHING_OBLIQUE SW_ON /*!< Enables search for parking slots during the approach with an angle oblique to the parking slot This  */
  #endif
  #ifndef LS_PSU_CPX_SCENE_INSLOT
    #define LS_PSU_CPX_SCENE_INSLOT SW_ON /*!< Enables search for parking slots for a situation with the vehicle partly or completely inside the pa */
  #endif

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- APG                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup APG_M02           exported symbolic constants
 * \ingroup COMPONENT_APG
 @{ */

#ifndef GS_APG_APGCFG_CPSC_USE_OOC_LAT_REF
  #define GS_APG_APGCFG_CPSC_USE_OOC_LAT_REF SW_OFF
#endif

#ifdef COMPONENT_APG

  #ifndef LS_APG_XPGCFG_TESTMODE
    #define LS_APG_XPGCFG_TESTMODE GS_XPGCFG_TESTMODE_CANAPE /*!< Switch to enable testmode GS_APGCFG_TESTMODE_OFF off GS_APGCFG_TESTMODE_CANAPE CANape Input GS_APGCF */
  #endif
  #define LS_APG_APGCFG_STARTNAVIGATION SW_ON /*!< En or diables an additional alignment section that is added at the beginning of the planned path of  */
  #define LS_APG_APGCFG_PREDICTPOSINPARKABLE SW_ON /*!< En /Disable the prediction of the standstill position in the parking space evaluation APGParkable  */
  #define LS_APG_APGCFG_LIMITSWADEVIATION SW_ON /*!< Description Enables or disables the limitation of the difference between set and actual steering whe */
  #define LS_APG_APGCFG_STRAIGHTENWHEELS GS_APG_STRAIGHTEN_PLANNED /*!< Sets the mode for straightening the wheels at the end of guidance GS_APG_STRAIGHTEN_STANDSTILL Syste */
  #define LS_APG_APGCFG_POC_FIRSTBACKWARDMOVESTRAIGHT SW_ON /*!< If this switch is set to ON the first backward move in POC is a straight move no steering If it is s */
  #define LS_APG_APGCFG_POC_FIRSTFORWARDMOVECLOTHOID SW_OFF /*!< IF this switch is set to ON the first forward move will be a clothoid The length of the Clothoid is  */
  #ifndef LS_APG_APGCFG_REACTIVATION
    #define LS_APG_APGCFG_REACTIVATION SW_ON /*!< Support of reactivation of guidance after guidance termination If this switch is set to ON the APG s */
  #endif
  #ifndef LS_APG_APGCFG_EARLYPOSOK
    #define LS_APG_APGCFG_EARLYPOSOK SW_ON /*!< Activates early InchPosition OK for parkingInch for PSC */
  #endif
  #ifndef LS_APG_APGCFG_CPSC_ADAPTION_LATPOS
    #define LS_APG_APGCFG_CPSC_ADAPTION_LATPOS GS_APG_CPSC_ADAPTION_LATPOS_AUTO /*!< Normally the lateral target position depends on the limiting objects on the right and/or the left si */
  #endif
  #ifndef LS_APG_APGCFG_STATIC_SWA_CONTROLLER
    #define LS_APG_APGCFG_STATIC_SWA_CONTROLLER SW_OFF /*!< A second controller that is used to compensate a residual difference between a requested target stee */
  #endif
  #ifndef LS_APG_APGCFG_PSC_LONGITUDINAL_ALIGNMENT
    #define LS_APG_APGCFG_PSC_LONGITUDINAL_ALIGNMENT SW_ON /*!< This switch defines wheter APG shall offer an additional alignment move or extend an N Move in PSC t */
  #endif
  #ifndef LS_APG_APGCFG_PSC_NO_TPR_IN_FULLWARNING
    #define LS_APG_APGCFG_PSC_NO_TPR_IN_FULLWARNING SW_OFF /*!< This switch defines wether APG shall suppress TargetPosReached and offer a next move if a full warni */
  #endif

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- APG_PL_POC                                                             -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup APG_PL_POC_M02           exported symbolic constants
 * \ingroup COMPONENT_APG_PL_POC
 @{ */


#ifdef COMPONENT_APG_PL_POC

  #ifndef LS_APG_PL_POC_OD_BASED_HANDOVER
    #define LS_APG_PL_POC_OD_BASED_HANDOVER OD_HANDOVER_FORWARDMOVE /*!< Defines whether or not to do a handover based on a front reference remeasured by OMA A safety distan */
  #endif
  #define LS_APG_PL_POC_DEACT_ON_MAX_OD_EXTENSION_REACHED SW_OFF /*!< Refer to the description of DeactThreshold_MaxExtensionFrontCorner for explanation */
  #define LS_APG_PL_POC_DEACT_ON_MAX_YAW_ANGLE_REACHED SW_ON /*!< Refer to the description of DeactThreshold_MaxYawAngle for explanation */
  #define LS_APG_PL_POC_YAW_ANGLE_BASED_HANDOVER SW_OFF /*!< Defines whether or not to do a handover based on a specified yaw angle that has been reached APG_POC */

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- PAGLOBAL                                                               -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup PAGLOBAL_M02           exported symbolic constants
 * \ingroup COMPONENT_PAGLOBAL
 @{ */

#define GS_PP_DEBUG SW_OFF
#ifndef GS_HWENV_TYPE
  #define GS_HWENV_TYPE GS_HWENV_TYPE_ECU
#endif
#ifndef GS_PP_FUNC_PDC
  #define GS_PP_FUNC_PDC SW_ON
#endif
#define GS_PP_FUNC_PSI SW_OFF
#ifndef GS_PP_FUNC_PSC
  #define GS_PP_FUNC_PSC SW_ON
#endif
#ifndef GS_PP_FUNC_CPSC
  #define GS_PP_FUNC_CPSC SW_ON
#endif
#ifndef GS_PP_FUNC_CPSC_FI
  #define GS_PP_FUNC_CPSC_FI SW_ON
#endif
#ifndef GS_PP_FUNC_CPSC_AIM
  #define GS_PP_FUNC_CPSC_AIM SW_OFF
#endif
#ifndef GS_PP_FUNC_DPSC
  #define GS_PP_FUNC_DPSC SW_OFF
#endif
#ifndef GS_PP_FUNC_POC
  #define GS_PP_FUNC_POC SW_OFF
#endif
#ifndef GS_PP_FUNC_CPSC_BO
  #define GS_PP_FUNC_CPSC_BO SW_OFF
#endif
#ifndef GS_PP_FUNC_CPSC_FO
  #define GS_PP_FUNC_CPSC_FO SW_OFF
#endif
#ifndef GS_PP_FUNC_BSD
  #define GS_PP_FUNC_BSD SW_OFF
#endif
#ifndef GS_PP_FUNC_PS
  #define GS_PP_FUNC_PS SW_OFF
#endif
#ifndef GS_PP_FUNC_SDW
  #define GS_PP_FUNC_SDW SW_ON
#endif
#ifndef GS_PP_FUNC_NRA
  #define GS_PP_FUNC_NRA SW_OFF
#endif
#ifndef GS_PP_FUNC_PBA
  #define GS_PP_FUNC_PBA SW_OFF
#endif
#ifndef GS_PP_PARALLEL_FUNC
  #define GS_PP_PARALLEL_FUNC GS_PDC_PSX
#endif
#ifndef GS_PP_FUNC_BDA
  #define GS_PP_FUNC_BDA SW_OFF
#endif
#ifndef GS_4WS_VEHICLE
  #define GS_4WS_VEHICLE SW_OFF
#endif
#define GS_COMPILER GS_COMPILER_GHS_MULTI
#define GS_ENDIAN GS_ENDIAN_LITTLE
#define GS_BITFLD_ORDER GS_BITFLD_ORDER_NORMAL
#define gd_SYSFCT_PDC_ui8 ((UInt8)(1))
#define gd_SYSFCT_PSX_ui8 ((UInt8)(2))
#define gd_SYSFCT_BSD_ui8 ((UInt8)(4))
#define gd_SYSFCT_POC_ui8 ((UInt8)(8))
#define gd_SYSFCT_PS_ui8 ((UInt8)(16))
#define gd_SYSFCT_SDW_ui8 ((UInt8)(32))
#ifndef GS_PP_FUNC_AUTOMODE
  #define GS_PP_FUNC_AUTOMODE SW_OFF
#endif
#ifndef GS_PP_RUNTIME_MEASUREMENT_FX4L
  #define GS_PP_RUNTIME_MEASUREMENT_FX4L SW_OFF
#endif
#define GS_PP_FUNC_VARIANTHANDLER SW_ON
#ifndef GS_PP_FUNC_OOC
  #define GS_PP_FUNC_OOC SW_OFF
#endif
#ifndef GS_PP_FUNC_PDC_8CH
  #define GS_PP_FUNC_PDC_8CH SW_OFF
#endif
#ifndef GS_PP_FUNC_PDC_10CH
  #define GS_PP_FUNC_PDC_10CH SW_OFF
#endif
#ifndef GS_TAE_TEST
  #define GS_TAE_TEST SW_OFF
#endif
#ifndef GS_MCS_SUPPORT
  #define GS_MCS_SUPPORT SW_OFF
#endif
#ifndef GS_PP_FUNC_APA
  #define GS_PP_FUNC_APA SW_OFF
#endif

#ifdef COMPONENT_PAGLOBAL


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- DSW                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup DSW_M02           exported symbolic constants
 * \ingroup COMPONENT_DSW
 @{ */


#ifdef COMPONENT_DSW


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- VARIANT                                                                -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup VARIANT_M02           exported symbolic constants
 * \ingroup COMPONENT_VARIANT
 @{ */


#ifdef COMPONENT_VARIANT


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- SIP                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup SIP_M02           exported symbolic constants
 * \ingroup COMPONENT_SIP
 @{ */


#ifdef COMPONENT_SIP


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- VHO                                                                    -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup VHO_M02           exported symbolic constants
 * \ingroup COMPONENT_VHO
 @{ */

#ifndef GS_VHO_RUN_MODE
  #define GS_VHO_RUN_MODE GS_VHO_CONSIDER_USS_DATA
#endif

#ifdef COMPONENT_VHO

  #ifndef LS_VHO_DIRDETECT_MODE
    #define LS_VHO_DIRDETECT_MODE LS_VHO_DIRDETECT_ALL /*!< this setting defines the way the rolling direction is determined _GEAR forward movement will be assu */
  #endif
  #ifndef LS_VHO_SPP_WIC
    #define LS_VHO_SPP_WIC SW_ON /*!< If the setting ON is chosen the per cycle WIC increments of all used wheels are monitored If a WIC i */
  #endif
  #define LS_VHO_AUTOCALIB_TTC SW_ON /*!< If the setting ON is chosen an autocalibration functionality will be turned on that is necessary to  */
  #define LS_VHO_AUTOCALIB_SWA_OFFSET SW_ON /*!< If the setting ON is chosen an autocalibration functionality will be turned on that is dynamically c */
  #define LS_VHO_CORRECTED_SWA_XPG SW_ON /*!< If the setting ON is chosen a calibrated steering wheel angle offset will be considered in the follo */
  #ifndef LS_VHO_EXT_COORDINATE_RESET
    #define LS_VHO_EXT_COORDINATE_RESET SW_ON /*!< This switch has to be set to ON if the PSX system shall be tested on RECAM It will enable the VHO to */
  #endif
  #define gd_VHO_NO_ECHOES_ui8 ((UInt8)(3)) /*!< Number of echoes that shall be buffered with each measurement If the PSL processes one echo per meas */
  #define LS_VHO_EXTRAPOL_ENGINE_RESTART SW_ON /*!< If the setting ON is chosen in case of a motor restart Warmstart Kaltstart the vehicle is allowed to */
  #define LS_VHO_WIC_FILTER_ELEPHANT SW_ON /*!< If the setting ON is chosen single WICs will be filtered to stabilize VHO direction output in stands */
  #ifndef LS_VHO_USSMAP_REAR_SENSOR_ACTIVE
    #define LS_VHO_USSMAP_REAR_SENSOR_ACTIVE SW_OFF /*!< Synchronize the vehicle state calculated by the odometry module not only with the front USS measurem */
  #endif
  #ifndef LS_VHO_CALC_VEHICLE_ACCELERATION
    #define LS_VHO_CALC_VEHICLE_ACCELERATION SW_OFF /*!< If switch is set to SW_ON vehicle acceleration will be calculated If switch is set to SW_OFF vehicle */
  #endif
  #ifndef LS_VHO_PROVIDE_DEVELOPMENT_DATA
    #define LS_VHO_PROVIDE_DEVELOPMENT_DATA SW_OFF /*!< If switch is set to SW_ON developement data will be provided If switch is set to SW_ON developement  */
  #endif

#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- MP_USER_MODE                                                           -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_USER_MODE_M02           exported symbolic constants
 * \ingroup COMPONENT_MP_USER_MODE
 @{ */

#define gd_MP_USER_MODE_SVA_INIT_ui8 ((UInt8)(0))
#define gd_MP_USER_MODE_PSX_INIT_ui8 ((UInt8)(0))
#define gd_MP_USER_MODE_PP_INIT_ui8 ((UInt8)(0))
#define gd_MP_USER_MODE_PAS_INIT_ui8 ((UInt8)(0))
#ifndef GS_MP_USER_MODE_PLUG_IN
  #define GS_MP_USER_MODE_PLUG_IN SW_OFF
#endif

#ifdef COMPONENT_MP_USER_MODE


#endif

/*!@} */

/*--------------------------------------------------------------------------*/
/*- MP_BDA                                                                 -*/
/*--------------------------------------------------------------------------*/

/*!\defgroup MP_BDA_M02           exported symbolic constants
 * \ingroup COMPONENT_MP_BDA
 @{ */

#ifndef gd_MP_BDA_TEST_MAX_MEAS_TIME_ui16
  #define gd_MP_BDA_TEST_MAX_MEAS_TIME_ui16 ((UInt16)(29000))
#endif
#ifndef GS_MP_BDA_BDA_ACTIVE
  #define GS_MP_BDA_BDA_ACTIVE SW_OFF
#endif

#ifdef COMPONENT_MP_BDA


#endif

/*!@} */
/* clang-format on */
#endif /* GEN_PAR_SWITCHES_H */
