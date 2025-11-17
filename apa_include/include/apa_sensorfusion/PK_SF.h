/**
 * @file PK_SF.h
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-09-02
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-09-02 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

/** Multiple inclusion protection */
#ifndef PK_SF
#define PK_SF

/*********************************************************************************************************
**Include common and project definition header
*********************************************************************************************************/
#include <string.h>
#include "PK_Utility.h"
#include "PK_SensorT2S.h"
#include "PK_B.h"
/*********************************************************************************************************
**Include headers of the component
*********************************************************************************************************/
#include "Rte_Types.h"
#include <netinet/in.h>
/*********************************************************************************************************
**Definition of exported symbolic constants
*********************************************************************************************************/

/** Version and module identification */
#define PK_SF_VENDOR_ID (0u)
#define PK_SF_MODULE_ID (0u)

/** Component Version Information */
#define PK_SF_SW_MAJOR_VERSION (3u)
#define PK_SF_SW_MINOR_VERSION (0u)
// #define PK_SF_SW_PATCH_VERSION (8u)
#define PK_SF_SW_PATCH_VERSION                                                          \
    (PK_SF_A_FS_SW_PATCH_VERSION + PK_SF_SUBFUN_PATCH_VERSION + PK_B_SW_PATCH_VERSION + \
     PK_SF_A_FS_SW_PATCH_VERSION + PK_SCENERECOG_SW_PATCH_VERSION +                     \
     PK_SF_AVM_BUFF_SW_PATCH_VERSION + 12u)

/*********************************************************************************************************
** Definition of exported function like macros
*********************************************************************************************************/
// 1. Total time series
extern uint32 TimeSeries_PK_SF;

// 2. Odometry information
// 2.1 Currently processed odometry location information (matched with median filtering)
// is different from CurPos.
extern float sf_curlocalx[4];
extern float sf_curlocaly[4];
extern float sf_curtheta[4];
extern float sf_curpath[4];
extern float sf_before_path[4];
// 2.2 Odometry information currently obtained by RTE
extern float sf_CurPos[4]; // Odometry obtained by RTE

// 3. State Management Control
extern PK_ModuleComType PK_SF_com;
extern PK_ModuleComType PK_SF_com_last;

// 4    Information of ultrosonic detection points
// 4.1  12 ultrosonic detection values:  udata_dir
// 4.2  8 indirect measurements:  udata_indir
// 4.3  4 two times echo values:  udata_2nd
extern U_RadarType sf_udata; // Ultrosonic value read at each time
// 4.4 Information of ultrosonic detection points,coordinates of detection points(x,y)
extern Point_T sf_curudata_detected[12];

// 9    parameter of the last object deleted by loop in current cycle
extern uint8 sf_loopflag_left;
extern uint8 sf_loopflag_right;
extern float sf_loopflag_left_S;
extern float sf_loopflag_right_S;

/*********************************************************************************************************
** Definition of off line testing
*********************************************************************************************************/

// definition for the offline testing
// #define _ULTRADEBUG
#ifdef _ULTRADEBUG
extern PK_ModuleComType TestPKCOMData[10000];
extern float TestUData[][12];
extern float TestOData[][4];
extern float TestUData_2nd[][4];
extern float TestUInData[][8];
extern float TestCurSpd[];
extern Vel_MoveMentStType TestMovement[];
// extern uint32 TimeSeries;  // Offline data principal time axis sequence, including
// process A and process B

// All information of reachable space A process is saved offline, and the output of FSL
// and FSR is saved at the same time.
extern float TestSFAResData_Front[][4];
// Reachable space A process holds all obstacle attributes while preserving the output of
// FSL and FSR
extern int TestSFAResData_Front_Attr[];
// Number of Obstacles in Reachable Space A Process
extern int TestSFAResDataNum_Front;

// All information of reachable space A process is saved offline, and the output of FSL
// and FSR is saved at the same time.
extern float TestSFAResData_Rear[1000][4];
// Reachable space A process holds all obstacle attributes while preserving the output of
// FSL and FSR
extern int TestSFAResData_Rear_Attr[1000];
// The total number of obstacles in reachable space A process while preserving the output
// of FSL and FSR
extern int TestSFAResDataNum_Rear;

// All information of reachable space A process is saved offline, and the output of FSL
// and FSR is saved at the same time.
extern float TestSFAResData_Front_disp[][4];
// Reachable space A process holds all obstacle attributes while preserving the output of
// FSL and FSR
extern int TestSFAResData_Front_Attr_disp[];
// Number of Obstacles in Reachable Space A Process
extern int TestSFAResDataNum_Front_disp;

extern FusionObj_T FusionObjLeft_RTE_Debug;
extern FusionObj_T FusionObjRight_RTE_Debug;
extern FusionObj_T FusionObjLeftRear_RTE_Debug;
extern FusionObj_T FusionObjRightRear_RTE_Debug;
#endif

/*********************************************************************************************************
** Definition of main functions
*********************************************************************************************************/

/**************************************************************************
Description		: main function of sensor fusion
Owner			:JSF
Modified Date	:2018.06.12
Parameter[In]	:None
Parameter[Out]	:None
Return			:None
***************************************************************************/
void PK_SF_Main_Visualize(int visualfd, struct sockaddr_in servaddr);
void PK_SF_Main(void);

#endif /* ifndef PK_SF */
/*********************************************************************************************************
**                                                                          End Of File:
*PK_SF.h
*********************************************************************************************************/