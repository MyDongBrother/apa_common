/**
 * @file PK_SF.c
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

/*********************************************************************************************************
**Include common and project definition header
*********************************************************************************************************/
#include "PK_Calibration.h"
#include "PK_Utility.h"
#include "MathFunc.h"
#include "PK_SF_Subfun.h"
#include "Rte_Types.h"
#include "Module_API.h"
#include "Rte_BSW.h"
#include "SystemPara.h"
#include "hobotlog/hobotlog.hpp"
#include "PK_StateManage.h"

/*********************************************************************************************************
**Include headers of the component
*********************************************************************************************************/
#include "PK_SF_A_FreeSpace.h"
// #include "PK_SF_A_FreeSpace_Disp.h"

#include "AVM_Buff.h"
#include "PK_SF_Subfun.h"
/*********************************************************************************************************
**Include other headers(socket)
*********************************************************************************************************/
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
/*********************************************************************************************************
**Check component version
*********************************************************************************************************/
/* Check if source file and COMMON header file are of the same software version */

#if ((PK_SF_SW_MAJOR_VERSION != (3u)) || (PK_SF_SW_MINOR_VERSION != (0u)) || \
     (PK_SF_SW_PATCH_VERSION !=                                              \
      (PK_SF_A_FS_SW_PATCH_VERSION + PK_SF_SUBFUN_PATCH_VERSION +            \
       PK_B_SW_PATCH_VERSION + PK_SF_A_FS_SW_PATCH_VERSION +                 \
       PK_SCENERECOG_SW_PATCH_VERSION + PK_SF_AVM_BUFF_SW_PATCH_VERSION + 12u)))
#error "Version numbers of PK_SF.c and PK_SF.h are inconsistent!"
#endif

/*********************************************************************************************************
**Definition of offline testing
*********************************************************************************************************/

//////////////////////////////////////////////////////////////////////////
// Offline data mode

#pragma section all "CPU1.Private"

#ifdef _ULTRADEBUG
PK_ModuleComType TestPKCOMData[10000];
float TestUData[10000][12];
float TestUInData[10000][8];
float TestUData_2nd[10000][4];
float TestOData[10000][4];
float TestCurSpd[10000];
Vel_MoveMentStType TestMovement[10000];

// All information of reachable space A process is saved offline, and the output of FSL
// and FSR is saved at the same time.
float TestSFAResData_Front[1000][4];
// Reachable space A process holds all obstacle attributes while preserving the output of
// FSL and FSR
int TestSFAResData_Front_Attr[1000];
// The total number of obstacles in reachable space A process while preserving the output
// of FSL and FSR
int TestSFAResDataNum_Front = 0;

// All information of reachable space A process is saved offline, and the output of FSL
// and FSR is saved at the same time.
float TestSFAResData_Rear[1000][4];
// Reachable space A process holds all obstacle attributes while preserving the output of
// FSL and FSR
int TestSFAResData_Rear_Attr[1000];
// The total number of obstacles in reachable space A process while preserving the output
// of FSL and FSR
int TestSFAResDataNum_Rear = 0;

// All information of reachable space A process is saved offline, and the output of FSL
// and FSR is saved at the same time.
float TestSFAResData_Front_disp[1000][4];
// Reachable space A process holds all obstacle attributes while preserving the output of
// FSL and FSR
int TestSFAResData_Front_Attr_disp[1000];
// Number of Obstacles in Reachable Space A Process
int TestSFAResDataNum_Front_disp;

FusionObj_T FusionObjLeft_RTE_Debug;
FusionObj_T FusionObjRight_RTE_Debug;
FusionObj_T FusionObjLeftRear_RTE_Debug;
FusionObj_T FusionObjRightRear_RTE_Debug;
// uint32 TimeSeries = 0;		// Offline data time series, including A and B

#endif

/*********************************************************************************************************
**Definition of exported symbolic constants
*********************************************************************************************************/
// 1. Total time series
uint32 TimeSeries_PK_SF = 0;

// 2. Odometry information
// float curlocalx = 0;
// float sf_curlocaly = 0;
// float sf_curtheta = 0;
// float curpath = 0;

// 3. State Management Control
PK_ModuleComType PK_SF_com      = MCOM_OFF; // Stage management data
PK_ModuleComType PK_SF_com_last = MCOM_OFF;
PK_ModuleStateType PK_SF_state  = MSTAT_OFF; // Closing calculation

// 4. Information of ultrasonic detection points
// 4.1 12 Ultrasound Measurements:  udata_dir
// 4.2 8 indirect measurements:  udata_indir
// 4.3 Four quadratic echo data:  udata_2nd
// U_RadarType udata;					//Ultrasound value read at each time

// 4.4 Information of ultrasonic detection points, coordinates of detection points (x, y)
// Point_T curudata_detected[12] = { 0,0 };

static void ReportSFObjInfo(const AVM_SlotPointsType &AVM_SlotPt,
                            const std::string &current_apa_status, int visualfd,
                            struct sockaddr_in servaddr)
{
    /*********************************************************************************
     * send avm and obs info by socket UDP to visualize add by xls@2022.6.23
     * ********************************************************************************/
    FusionObj_T ObsLeft = {0}, ObsRight = {0}, ObsLeft_Rear = {0}, ObsRight_Rear = {0};
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left(&ObsLeft);
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right(&ObsRight);
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Left(&ObsLeft_Rear);
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Right(&ObsRight_Rear);

    int start_left       = ObsLeft.num < SF_OBJ_NUM ? SF_OBJ_NUM - ObsLeft.num : 0;
    int start_right      = ObsRight.num < SF_OBJ_NUM ? SF_OBJ_NUM - ObsRight.num : 0;
    int start_left_rear  = ObsLeft.num < SF_OBJ_NUM ? SF_OBJ_NUM - ObsLeft_Rear.num : 0;
    int start_right_rear = ObsRight.num < SF_OBJ_NUM ? SF_OBJ_NUM - ObsRight_Rear.num : 0;

    AVM_Buff_Info Left_Buff, Right_Buff;
    RTE_PK_SF_Get_AVM_Buff_Left(&Left_Buff);
    RTE_PK_SF_Get_AVM_Buff_Right(&Right_Buff);

    float CurPos[4]    = {0}; //  x,y,theta,s
    VehPos_T VehCurPos = {0};
    RTE_PK_Location_Get_CurPos(CurPos);
    memcpy(&VehCurPos, CurPos, sizeof(VehCurPos));

    //  Coordinates of vehicle odometery
    Point_T NearFrontPt = {0}, NearRearPt = {0}, FarFrontPt = {0}, FarRearPt = {0};
    int8_t has_stopper = 0;

    // RTE_PD_Get_AVM_SlotPoints(&AVM_SlotPt);  //  Get AVM parking lot data
    memcpy(&NearFrontPt, AVM_SlotPt.NearFront, sizeof(NearFrontPt));
    memcpy(&NearRearPt, AVM_SlotPt.NearRear, sizeof(NearRearPt));
    memcpy(&FarFrontPt, AVM_SlotPt.FarFront, sizeof(FarFrontPt));
    memcpy(&FarRearPt, AVM_SlotPt.FarRear, sizeof(FarRearPt));
    memcpy(&has_stopper, &AVM_SlotPt.hasStop, sizeof(has_stopper));
    LOGI << "-----------------------AVM SLOT DATA---------------------------";
    LOGI << "NearFrontPt: " << NearFrontPt.x << "," << NearFrontPt.y;
    LOGI << "NearRearPt: " << NearRearPt.x << "," << NearRearPt.y;
    LOGI << "FarFrontPt: " << FarFrontPt.x << "," << FarFrontPt.y;
    LOGI << "FarRearPt: " << FarRearPt.x << "," << FarRearPt.y;
    LOGI << "has_stopper: " << (int)has_stopper;

    LOGI << "-----------------------Veh Cur Position---------------------------";
    LOGI << "Vehicle current positon x: " << VehCurPos.x << " y: " << VehCurPos.y
         << " theta: " << VehCurPos.theta;

    LOGI << "-----------------------OBSTACLE LEFT---------------------------";
    for (int obs_idx = start_left; obs_idx < SF_OBJ_NUM; obs_idx++)
    {
        LOGI << "obstacle pt1 x: " << ObsLeft.obj[obs_idx].pt1.x
             << " obstacle pt1 y: " << ObsLeft.obj[obs_idx].pt1.y;
        LOGI << "obstacle pt2 x: " << ObsLeft.obj[obs_idx].pt2.x
             << " obstacle pt2 y: " << ObsLeft.obj[obs_idx].pt2.y;
        LOGI << "obstacle attr:" << ObsLeft.attr[obs_idx];
    }

    LOGI << "-----------------------OBSTACLE RIGHT---------------------------";
    for (int obs_idx = start_right; obs_idx < SF_OBJ_NUM; obs_idx++)
    {
        LOGI << "obstacle pt1 x: " << ObsRight.obj[obs_idx].pt1.x
             << " obstacle pt1 y: " << ObsRight.obj[obs_idx].pt1.y;
        LOGI << "obstacle pt2 x: " << ObsRight.obj[obs_idx].pt2.x
             << " obstacle pt2 y: " << ObsRight.obj[obs_idx].pt2.y;
        LOGI << "obstacle attr:" << ObsRight.attr[obs_idx];
    }
    LOGI << "-----------------------CURRENT U DISTANCE---------------------------";
    LOGI << "cur_fsl: " << cur_fsl << " cur_fsr:" << cur_fsr;
    LOGI << "cur_rsl: " << cur_rsl << " cur_rsr:" << cur_rsr;

    std::string sendline = "AVM:" + std::to_string((int)(NearFrontPt.x * 100)) + " " +
                           std::to_string((int)(NearFrontPt.y * 100)) + ",";
    sendline += (std::to_string((int)(NearRearPt.x * 100)) + " " +
                 std::to_string((int)(NearRearPt.y * 100))) +
                ",";
    sendline += (std::to_string((int)(FarFrontPt.x * 100)) + " " +
                 std::to_string((int)(FarFrontPt.y * 100))) +
                ",";
    sendline += (std::to_string((int)(FarRearPt.x * 100)) + " " +
                 std::to_string((int)(FarRearPt.y * 100)));
    sendline += "\n";

    sendline += ("Veh_pos:" + std::to_string((int)(VehCurPos.x * 100)) + "," +
                 std::to_string((int)(VehCurPos.y * 100)) + "," +
                 std::to_string(VehCurPos.theta));
    sendline += "\n";

    for (int obs_idx = start_left; obs_idx < SF_OBJ_NUM; obs_idx++)
    {
        sendline += ("pt1:" + std::to_string((int)(ObsLeft.obj[obs_idx].pt1.x * 100)) +
                     " " + std::to_string((int)(ObsLeft.obj[obs_idx].pt1.y * 100))) +
                    ",";
        sendline += ("pt2:" + std::to_string((int)(ObsLeft.obj[obs_idx].pt2.x * 100)) +
                     " " + std::to_string((int)(ObsLeft.obj[obs_idx].pt2.y * 100)));
        sendline += ";";
    }
    sendline += "\n";

    for (int obs_idx = start_right; obs_idx < SF_OBJ_NUM; obs_idx++)
    {
        sendline += ("pt1:" + std::to_string((int)(ObsRight.obj[obs_idx].pt1.x * 100)) +
                     " " + std::to_string((int)(ObsRight.obj[obs_idx].pt1.y * 100))) +
                    ",";
        sendline += ("pt2:" + std::to_string((int)(ObsRight.obj[obs_idx].pt2.x * 100)) +
                     " " + std::to_string((int)(ObsRight.obj[obs_idx].pt2.y * 100)));
        sendline += ";";
    }
    sendline += "\n";

    for (int obs_idx = start_left_rear; obs_idx < SF_OBJ_NUM; obs_idx++)
    {
        sendline +=
            ("pt1:" + std::to_string((int)(ObsLeft_Rear.obj[obs_idx].pt1.x * 100)) + " " +
             std::to_string((int)(ObsLeft_Rear.obj[obs_idx].pt1.y * 100))) +
            ",";
        sendline +=
            ("pt2:" + std::to_string((int)(ObsLeft_Rear.obj[obs_idx].pt2.x * 100)) + " " +
             std::to_string((int)(ObsLeft_Rear.obj[obs_idx].pt2.y * 100)));
        sendline += ";";
    }
    sendline += "\n";

    for (int obs_idx = start_right_rear; obs_idx < SF_OBJ_NUM; obs_idx++)
    {
        sendline +=
            ("pt1:" + std::to_string((int)(ObsRight_Rear.obj[obs_idx].pt1.x * 100)) +
             " " + std::to_string((int)(ObsRight_Rear.obj[obs_idx].pt1.y * 100))) +
            ",";
        sendline +=
            ("pt2:" + std::to_string((int)(ObsRight_Rear.obj[obs_idx].pt2.x * 100)) +
             " " + std::to_string((int)(ObsRight_Rear.obj[obs_idx].pt2.y * 100)));
        sendline += ";";
    }
    sendline += "\n";
    U_RadarType sf_udata_A;
    RTE_PD_Get_U_Radar(&sf_udata_A);
    sendline +=
        std::to_string((int)(cur_fsl / 10)) + " " + std::to_string((int)(cur_fsr / 10)) +
        " " + std::to_string((int)(cur_rsl / 10)) + " " +
        std::to_string((int)(cur_rsr / 10)) + " " +
        std::to_string((int)(sf_udata_A.FOL / 10)) + " " +
        std::to_string((int)(sf_udata_A.FCL / 10)) + " " +
        std::to_string((int)(sf_udata_A.FCR / 10)) + " " +
        std::to_string((int)(sf_udata_A.FOR / 10)) + " " +
        std::to_string((int)(sf_udata_A.ROL / 10)) + " " +
        std::to_string((int)(sf_udata_A.RCL / 10)) + " " +
        std::to_string((int)(sf_udata_A.RCR / 10)) + " " +
        std::to_string((int)(sf_udata_A.ROR /
                             10)); // FSL,FSR,RSL,RSR,FOL,FCL,FCR,FOR;ROL,RCL,RCR,ROR;
    sendline += "\n";

    sendline += current_apa_status;
    sendline += "\n";
    static uint32_t send_count = 0;
    if (send_count % 10 == 0)
    {
        int send_num = sendto(visualfd, sendline.c_str(), sendline.length(), 0,
                              (struct sockaddr *)&servaddr, sizeof(servaddr));
        if (send_num < 0)
        {
            LOGE << "--------------------------------------------------------------------"
                    "--------send msg error!";
        }
    }
    send_count++;
}
/***************************************************************************************************************/

/*********************************************************************************************************
**Definition of main function of auto parking
*********************************************************************************************************/
void PK_SF_Main()
{
    /***************************************************************
     * 根据车辆当前状态，调整本模块状态
     * *************************************************************/
    Apa_WorkStateType APA_Status = RTE_SM_Get_ApaWorkState();
    std::string current_apa_status;

    switch (APA_Status)
    {
        case CarSta_Passive:
            RTE_PK_StateManage_Set_ModuleCom_SensorFusion(MCOM_OFF);
            current_apa_status = "CarSta_Passive";
            break;

        case CarSta_Serching:
        case CarSta_GuidanceAvpActive:
            RTE_PK_StateManage_Set_ModuleCom_SensorFusion(MCOM_ON);
            current_apa_status = "CarSta_Searching";
            break;

        case CarSta_GuidanceActive:
            RTE_PK_StateManage_Set_ModuleCom_SensorFusion(MCOM_ON_PARKING);
            current_apa_status = "CarSta_GuidanceActive";
            break;

        case CarSta_GuidanceSuspend:
            RTE_PK_StateManage_Set_ModuleCom_SensorFusion(MCOM_ON_PARKING);
            current_apa_status = "CarSta_GuidanceSuspend";
            break;

        case CarSta_GuidanceTerminated:
            RTE_PK_StateManage_Set_ModuleCom_SensorFusion(MCOM_ON_PARKING);
            current_apa_status = "CarSta_GuidanceTerminated";
            break;

        case CarSta_GuidanceCompleted:
            RTE_PK_StateManage_Set_ModuleCom_SensorFusion(MCOM_DATCLC);
            current_apa_status = "CarSta_GuidanceCompleted";
            break;

        case CarSta_Failure:
            RTE_PK_StateManage_Set_ModuleCom_SensorFusion(MCOM_DATCLC);
            current_apa_status = "CarSta_Failure";
            break;

        case CarSta_ParkAssist_Standby:
            RTE_PK_StateManage_Set_ModuleCom_SensorFusion(MCOM_ON);
            current_apa_status = "CarSta_ParkAssist_Standby";
            break;

        case CarSta_GuidanceInitParkingData:
            RTE_PK_StateManage_Set_ModuleCom_SensorFusion(MCOM_OFF);
            current_apa_status = "CarSta_GuidanceInitParkingData";
            break;

        case CarSta_Passive_SpdOver:
            RTE_PK_StateManage_Set_ModuleCom_SensorFusion(MCOM_OFF);
            current_apa_status = "CarSta_Passive_SpdOver";
            break;

        case CarSta_PowerOn:
            RTE_PK_StateManage_Set_ModuleCom_SensorFusion(MCOM_INIT);
            current_apa_status = "CarSta_PowerOn";
            break;

        default:
            break;
    }

    static std::string last_apa_status = current_apa_status;
    if (last_apa_status != current_apa_status)
    {
        LOGW << "APA Status changed from " << last_apa_status << " to "
             << current_apa_status;
        last_apa_status = current_apa_status;
    }

    PK_SF_com = RTE_PK_StateManage_Get_ModuleCom_SensorFusion();

    PK_ModuleStateType t2sState = RTE_PK_SensorT2S_Get_ModuleState_SensorT2S();
    if (PK_SF_com == MCOM_OFF || PK_SF_com == MCOM_DATCLC || PK_SF_com == MCOM_INIT ||
        TimeSeries_PK_SF == 0 || t2sState != MSTAT_NORM)
    {
        RTE_PD_Clr_AVM_SlotPoints(); // 清除AVM队列数据
        PK_SF_A_FreeSpace_Clear();
        PK_SF_B_Clear();
        PK_SF_B_End();
        Clean_All_AVM_SlotBuff();
        RTE_PK_SensorFusion_Set_ModuleState_SensorFusion(MSTAT_DATCLC);
    }

    // State Management: A Process Running
    if (PK_SF_com == MCOM_ON)
    {
        Check_IMU_Data();

        if (t2sState == MSTAT_NORM)
        {
            GetFilterData_V3();
            PK_SF_A_FreeSpace();
        }

        AVM_Slot_Buffer_Manager();
        RTE_PK_SensorFusion_Set_ModuleState_SensorFusion(MSTAT_NORM);
    }

    // State Management: B Process Initialization and Operation
    if (PK_SF_com == MCOM_ON_PARKING)
    {
        Check_IMU_Data();
        if (PK_SF_com_last != MCOM_ON_PARKING)
        {
            PK_SF_B_Init();
        }

        if (t2sState == MSTAT_NORM)
        {
            GetFilterData_V3();
            PK_SF_B();
        }

        RTE_PK_SensorFusion_Set_ModuleState_SensorFusion(MSTAT_NORM);
    }

    // ReportSFObjInfo(AVM_SlotPt, current_apa_status, visualfd, servaddr);

    // timing update
    TimeSeries_PK_SF = TimeSeries_PK_SF + 1;

    // COM for storing this time
    PK_SF_com_last = PK_SF_com;
}
