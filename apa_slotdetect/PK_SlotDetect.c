/**
 * @file PK_SlotDetect.c
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
#include "URadar_SlotDetect.h"
#include "AVM_SlotDetect.h"

/*********************************************************************************************************
**Include headers of the component
*********************************************************************************************************/
#include "PK_SlotDetect.h"
#include "Rte_Types.h"
#include "MathFunc.h"
#include "PK_StateManage.h"
#include "Record_Log.h"

/*********************************************************************************************************
**Include other headers
*********************************************************************************************************/
#include <string.h>
#include "hobotlog/hobotlog.hpp"
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/*********************************************************************************************************
**Check component version
*********************************************************************************************************/
/* Check if source file and COMMON header file are of the same software version */
#if ((PK_SlotDetect_SW_MAJOR_VERSION != (3u)) || \
     (PK_SlotDetect_SW_MINOR_VERSION != (2u)) || \
     (PK_SlotDetect_SW_PATCH_VERSION != (57u)))
#error "Version numbers of PK_SlotDetect.c and PK_SlotDetect.h are inconsistent!"
#endif

/*********************************************************************************************************
** Declaration for inner used variables
*********************************************************************************************************/
#define MAXLINE 1024

#pragma section all "CPU1.Private"

SlotInfo_T RTE_Left_Multi_Slot_Array[MULT_SLOT_SIDE_NUM_MAX]; //  all the left slot info
                                                              //  buffers to send to RTE
int RTE_left_multi_slot_num = 0;
SlotInfo_T RTE_Right_Multi_Slot_Array[MULT_SLOT_SIDE_NUM_MAX]; //  all the right slot info
                                                               //  buffers to send to RTE
int RTE_right_multi_slot_num = 0;
SlotObj_T g_SlotObj_A; //  slotobj detected in process A
static SlotInfo_T RTE_Multi_Slot_Array[MAX_PARKED_SLOTS];
static int RTE_Multi_Slot_Num    = 0;
static int FusionBst_ObjNum_Last = 0;
static float g_lastPath          = 0;
static Multi_Slot_Array_T RTE_Multi_Slots;

SlotInfo_T c_Left_U_Slot_Array[MAX_U_SLOT_SIDE_NUM]; //  the copy data of relative slots
SlotInfo_T c_Right_U_Slot_Array[MAX_U_SLOT_SIDE_NUM];
SlotInfo_T c_Left_AVM_Slot_Array[AVM_SLOT_NUM_MAX];  // Normal AVM Slot (Left)
SlotInfo_T c_Right_AVM_Slot_Array[AVM_SLOT_NUM_MAX]; // Normal AVM Slot (Right)

#ifdef PK_SlotDetect_OffLineTest
int g_IsASlotUpata = 0;

#endif

/*********************************************************************************************************
** Definition of structures and enumerated variables
*********************************************************************************************************/

/*********************************************************************************************************
** Declaration for inner used functions
*********************************************************************************************************/
void Initialize_SlotDetect(void);
void Multi_SlotDetect_A(void);
static void BuildSlotsIndex(SlotInfo_T RTE_Multi_Slot_Array[MAX_PARKED_SLOTS],
                            int slotNum);

/*  The main entrance function for slot detect, mainly do the response for the commond */
void PK_SlotDetect(void)
{
    // definition of inner variables
    PK_ModuleComType MCom_SlotDetect;              //  commond for slot detect
    PK_ModuleStateType MSt_SlotDetect = MSTAT_OFF; //  running state of slot detect
    static PK_ModuleComType MCom_SlotDetect_last = MCOM_ON;

    /***************************************************************
     * ????????,???????
     * *************************************************************/
    Apa_WorkStateType APA_Status = RTE_SM_Get_ApaWorkState();
    switch (APA_Status)
    {
        case CarSta_Passive:
            RTE_PK_StateManage_Set_ModuleCom_SlotDetect(MCOM_OFF);
            break;

        case CarSta_Serching:
        case CarSta_GuidanceAvpActive:

            RTE_PK_StateManage_Set_ModuleCom_SlotDetect(MCOM_ON);
            break;

        case CarSta_GuidanceActive:
            RTE_PK_StateManage_Set_ModuleCom_SlotDetect(MCOM_ON_PARKING);
            break;

        case CarSta_GuidanceSuspend:
            RTE_PK_StateManage_Set_ModuleCom_SlotDetect(MCOM_ON_PARKING);
            break;

        case CarSta_GuidanceTerminated:
            RTE_PK_StateManage_Set_ModuleCom_SlotDetect(MCOM_ON_PARKING);
            break;

        case CarSta_GuidanceCompleted:
            RTE_PK_StateManage_Set_ModuleCom_SlotDetect(MCOM_DATCLC);
            break;

        case CarSta_Failure:
            RTE_PK_StateManage_Set_ModuleCom_SlotDetect(MCOM_DATCLC);
            break;

        case CarSta_ParkAssist_Standby:
            RTE_PK_StateManage_Set_ModuleCom_SlotDetect(MCOM_ON);
            break;

        case CarSta_GuidanceInitParkingData:
            RTE_PK_StateManage_Set_ModuleCom_SlotDetect(MCOM_OFF);
            break;

        case CarSta_Passive_SpdOver:
            RTE_PK_StateManage_Set_ModuleCom_SlotDetect(MCOM_OFF);
            break;

        case CarSta_PowerOn:
            RTE_PK_StateManage_Set_ModuleCom_SlotDetect(MCOM_INIT);
            break;

        default:
            break;
    }

    // Get values from RTE
    MCom_SlotDetect = RTE_PK_StateManage_Get_ModuleCom_SlotDetect();

    if ((MCom_SlotDetect_last != MCOM_ON_PARKING && MCom_SlotDetect == MCOM_ON_PARKING) ||
        (MCom_SlotDetect_last == MCOM_ON_PARKING && MCom_SlotDetect != MCOM_ON_PARKING))
    {
        Initialize_SlotDetect();
        MSt_SlotDetect = MSTAT_GET_RESULT;
    }
    MCom_SlotDetect_last = MCom_SlotDetect;
    // the following will be deleted when status-manager is OK   add by xls @2022.5.11
    if (MCom_SlotDetect_last == MCOM_INIT && MCom_SlotDetect == MCOM_INIT)
    {
        LOGE << "Find MCom_SlotDetect Status always be MCOM_INIT, here CHANGE it to be "
                "MCOM_ON";
        RTE_PK_StateManage_Set_ModuleCom_SlotDetect(MCOM_ON);
    }

    if (MCom_SlotDetect == MCOM_OFF) // DO NOT run Slot Detecting Condition;
    {
        Initialize_SlotDetect(); //  uncomment by xls@2022.7.9

        MSt_SlotDetect = MSTAT_OFF;
        // Multi_SlotDetect_A();
    }
    else if (MCom_SlotDetect == MCOM_DATCLC)
    {
        Initialize_SlotDetect();
        MSt_SlotDetect = MSTAT_DATCLC;
    }
    else if (MCom_SlotDetect == MCOM_ON) //  process A
    {
        MSt_SlotDetect = MSTAT_NORM;
        Multi_SlotDetect_A();
    }
    else if (MCom_SlotDetect == MCOM_ON_PARKING) //  process B parking in
    {
        MSt_SlotDetect = MSTAT_GET_RESULT;
    }
    //  send RTE
    // RTE_PK_SlotDetect_Set_Multi_SlotInfo(RTE_Multi_Slot_Array);
    ////Due to add SlotNum to struct SlotInfo, adjust following  code.
    // RTE_PK_SlotDetect_Set_Multi_SlotNum(RTE_Multi_Slot_Num);
    // Multi_slot_num_ccp = RTE_Multi_Slot_Num;
    RTE_Multi_Slots.multiNum = RTE_Multi_Slot_Num;

    RTE_PK_SlotDetect_Set_Multi_SlotInfo(&RTE_Multi_Slots);
    // Multi_slot_num_ccp = RTE_Multi_Slots.multiNum;
    RTE_PK_SlotDetect_Set_ModuleState_SlotDetect(MSt_SlotDetect);

    /***********************************************************
     *                 ADD LOG & VISUALIZE THE SLOT INFO
     * *********************************************************/
    if (!is_save_log)
    {
        return;
    }
}

static void ReportToWeb(int visualfd, struct sockaddr_in servaddr)
{
    float curpos[4];
    RTE_PK_Location_Get_CurPos(curpos);
    std::string sendline = "SLOT_INFO:\n";

    for (int idx = 0; idx < RTE_Multi_Slot_Num; idx++)
    {
        SlotInfo_T slot_cur = RTE_Multi_Slots.multiArray[idx];
        sendline += (std::to_string(slot_cur.slot_index) + ";");
        sendline += (std::to_string(slot_cur.slotshap) + ";");
        sendline += (std::to_string(slot_cur.is_vision_slot) + ";");
        sendline += (std::to_string(slot_cur.dist_curveh) + ";");
        sendline += (std::to_string(slot_cur.targpos.x) + " " +
                     std::to_string(slot_cur.targpos.y) + " " +
                     std::to_string(slot_cur.targpos.theta) + ";");
        sendline += (std::to_string(slot_cur.avm_point.near_front.x) + " " +
                     std::to_string(slot_cur.avm_point.near_front.y) + ",");
        sendline += (std::to_string(slot_cur.avm_point.near_rear.x) + " " +
                     std::to_string(slot_cur.avm_point.near_rear.y) + ",");
        sendline += (std::to_string(slot_cur.avm_point.far_front.x) + " " +
                     std::to_string(slot_cur.avm_point.far_front.y) + ",");
        sendline += (std::to_string(slot_cur.avm_point.far_rear.x) + " " +
                     std::to_string(slot_cur.avm_point.far_rear.y) + ";");
        sendline += (std::to_string(slot_cur.slotobj.ptB.x) + " " +
                     std::to_string(slot_cur.slotobj.ptB.y) + ",");
        sendline += (std::to_string(slot_cur.slotobj.ptC.x) + " " +
                     std::to_string(slot_cur.slotobj.ptC.y) + ",");
        sendline += (std::to_string(slot_cur.slotobj.ptD.x) + " " +
                     std::to_string(slot_cur.slotobj.ptD.y) + ",");
        sendline += (std::to_string(slot_cur.slotobj.ptE.x) + " " +
                     std::to_string(slot_cur.slotobj.ptE.y) + ";");
        sendline += "\n";

        LOGD << "---------------------slot " << idx << "---------------------";
        //????
        if (slot_cur.slotshap == PK_SLOT_LEFT_PARA)
        {
            LOGD << "slotshap: PK_SLOT_LEFT_PARA";
        }
        else if (slot_cur.slotshap == PK_SLOT_RIGHT_PARA)
        {
            LOGD << "slotshap: PK_SLOT_RIGHT_PARA";
        }
        else if (slot_cur.slotshap == PK_SLOT_LEFT_VERT)
        {
            LOGD << "slotshap: PK_SLOT_LEFT_VERT";
        }
        else if (slot_cur.slotshap == PK_SLOT_RIGHT_VERT)
        {
            LOGD << "slotshap: PK_SLOT_RIGHT_VERT";
        }
        else
        {
            LOGD << "slotshap: PK_SLOT_ANG???";
        }
        //?????
        if (slot_cur.is_vision_slot)
        {
            LOGD << "sensor type: Vision";
        }
        else
        {
            LOGD << "sensor type: URadar";
        }

        LOGD << "slot_cur.dist_curveh: " << slot_cur.dist_curveh;
        LOGD << "slot_cur.slot_index: " << slot_cur.slot_index;

        LOGD << "slot_cur.targpos.x: " << slot_cur.targpos.x - curpos[0];
        LOGD << "slot_cur.targpos.y: " << slot_cur.targpos.y - curpos[1];
        LOGD << "slot_cur.targpos.theta: " << slot_cur.targpos.theta;

        LOGD << "(AVM V.S. SF) NF: (" << slot_cur.avm_point.near_front.x - curpos[0]
             << ", " << slot_cur.avm_point.near_front.y - curpos[1] << ") v.s. ("
             << slot_cur.slotobj.ptE.x - curpos[0] << ","
             << slot_cur.slotobj.ptE.y - curpos[1] << ") ";
        LOGD << "(AVM V.S. SF) NR: (" << slot_cur.avm_point.near_rear.x - curpos[0]
             << ", " << slot_cur.avm_point.near_rear.y - curpos[1] << ") v.s. ("
             << slot_cur.slotobj.ptB.x - curpos[0] << ","
             << slot_cur.slotobj.ptB.y - curpos[1] << ") ";
        LOGD << "(AVM V.S. SF) FF: (" << slot_cur.avm_point.far_front.x - curpos[0]
             << ", " << slot_cur.avm_point.far_front.y - curpos[1] << ") v.s. ("
             << slot_cur.slotobj.ptD.x - curpos[0] << ","
             << slot_cur.slotobj.ptD.y - curpos[1] << ") ";
        LOGD << "(AVM V.S. SF) FF: (" << slot_cur.avm_point.far_rear.x - curpos[0] << ", "
             << slot_cur.avm_point.far_rear.y - curpos[1] << ") v.s. ("
             << slot_cur.slotobj.ptC.x - curpos[0] << ","
             << slot_cur.slotobj.ptC.y - curpos[1] << ") ";
    }

    MultiPlanInfo current_planInfs;
    RTE_PK_PathPlan_Get_Multi_PlanInfo(&current_planInfs);
    //??????????
    sendline += "PathInfo:\n";
    LOGD << "-----------------------Multi_PlanInfo---------------------------";
    for (int i = 0; i < current_planInfs.slotNum && i < MAX_PARKED_SLOTS; i++)
    {
        PlanInfoType current_path = current_planInfs.slotPlans[i];
        float endPos[3];
        LOGD << "SLOT INDEX: " << current_path.Slot_index << " PATHS: ";
        for (int index = 0; index < current_path.Path_num && index < MAX_SINGLE_TRAJ_NUM;
             index++)
        {
            PK_Get_Path_EndPos(current_path.Act_traj[index], endPos);
            LOGD << "start_x: " << current_path.Act_traj[index][0]
                 << " start_y:" << current_path.Act_traj[index][1]
                 << " start_theta: " << current_path.Act_traj[index][2]
                 << " pathLen:" << current_path.Act_traj[index][3]
                 << " rotateRad:" << current_path.Act_traj[index][4]
                 << " end_x:" << endPos[0] << " end_y:" << endPos[1]
                 << " end_theta:" << endPos[2];
            char temp[128];
            sprintf(temp, "%d,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f;",
                    current_path.Slot_index, current_path.Act_traj[index][0],
                    current_path.Act_traj[index][1], current_path.Act_traj[index][2],
                    current_path.Act_traj[index][3], current_path.Act_traj[index][4],
                    endPos[0], endPos[1], endPos[2]);
            std::string temp_str(temp);
            sendline += temp_str;
        }
        sendline += "\n";
    }
    sendline += "ObsInfo_B:\n";
    FusionObj_T ObsInfo_Left_B, ObsInfo_Right_B;
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Left(&ObsInfo_Left_B);
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Right(&ObsInfo_Right_B);
    // //??B???,??????????,???????
    for (int i = 0; i < ObsInfo_Left_B.num && i < SF_OBJ_NUM; i++)
    {
        sendline += ("pt1:" + std::to_string((int)(ObsInfo_Left_B.obj[i].pt1.x * 100)) +
                     " " + std::to_string((int)(ObsInfo_Left_B.obj[i].pt1.y * 100))) +
                    ",";
        sendline += ("pt2:" + std::to_string((int)(ObsInfo_Left_B.obj[i].pt2.x * 100)) +
                     " " + std::to_string((int)(ObsInfo_Left_B.obj[i].pt2.y * 100)));
        sendline += ";";
    }
    sendline += "\n";

    for (int j = 0; j < ObsInfo_Right_B.num && j < SF_OBJ_NUM; j++)
    {
        sendline += ("pt1:" + std::to_string((int)(ObsInfo_Right_B.obj[j].pt1.x * 100)) +
                     " " + std::to_string((int)(ObsInfo_Right_B.obj[j].pt1.y * 100))) +
                    ",";
        sendline += ("pt2:" + std::to_string((int)(ObsInfo_Right_B.obj[j].pt2.x * 100)) +
                     " " + std::to_string((int)(ObsInfo_Right_B.obj[j].pt2.y * 100)));
        sendline += ";";
    }

    static int last_leftObj_num  = 0;
    static int last_rightObj_num = 0;
    if (ObsInfo_Left_B.num != last_leftObj_num)
    {
        LOGW << "left Obs obj B num change from " << last_leftObj_num << " to "
             << ObsInfo_Left_B.num;
        last_leftObj_num = ObsInfo_Left_B.num;
    }
    if (ObsInfo_Right_B.num != last_rightObj_num)
    {
        LOGW << "right Obs obj B num change from " << last_rightObj_num << " to "
             << ObsInfo_Right_B.num;
        last_rightObj_num = ObsInfo_Right_B.num;
    }

    PK_ModuleComType MCom_SlotDetect = RTE_PK_StateManage_Get_ModuleCom_SlotDetect();
    if (MCom_SlotDetect == MCOM_ON_PARKING)
    {
        LOGI << "--------------------------ObsInfo_Left_B--------------------------";
        for (int obs_idx = 0; obs_idx < ObsInfo_Left_B.num && obs_idx < SF_OBJ_NUM;
             obs_idx++)
        {
            LOGI << "obstacle pt1 x: " << ObsInfo_Left_B.obj[obs_idx].pt1.x
                 << " obstacle pt1 y: " << ObsInfo_Left_B.obj[obs_idx].pt1.y;
            LOGI << "obstacle pt2 x: " << ObsInfo_Left_B.obj[obs_idx].pt2.x
                 << " obstacle pt2 y: " << ObsInfo_Left_B.obj[obs_idx].pt2.y;
            LOGI << "obstacle attr:" << ObsInfo_Left_B.attr[obs_idx];
        }
        LOGI << "--------------------------ObsInfo_Right_B--------------------------";
        for (int obs_idx = 0; obs_idx < ObsInfo_Right_B.num && obs_idx < SF_OBJ_NUM;
             obs_idx++)
        {
            LOGI << "obstacle pt1 x: " << ObsInfo_Right_B.obj[obs_idx].pt1.x
                 << " obstacle pt1 y: " << ObsInfo_Right_B.obj[obs_idx].pt1.y;
            LOGI << "obstacle pt2 x: " << ObsInfo_Right_B.obj[obs_idx].pt2.x
                 << " obstacle pt2 y: " << ObsInfo_Right_B.obj[obs_idx].pt2.y;
            LOGI << "obstacle attr:" << ObsInfo_Right_B.attr[obs_idx];
        }
    }

    // sendline += "\n";
    static uint32_t send_count = 0;
    if (send_count++ % 10 == 0)
    {
        int send_num = sendto(visualfd, sendline.c_str(), sendline.length(), 0,
                              (struct sockaddr *)&servaddr, sizeof(servaddr));
        if (send_num < 0)
        {
            LOGE << "--------------------------------------------------------------------"
                    "--------send msg error!";
        }
    }
}

/*  Initialize all the variables for slot detect    */
void Initialize_SlotDetect(void)
{
    Init_URadar_SlotDetect();
    Init_AVM_SlotDetect();
    FusionBst_ObjNum_Last    = 0;
    g_lastPath               = 0;
    Is_BC_Update_Bst         = 0;
    Is_DE_Update_Bst         = 0;
    RTE_left_multi_slot_num  = 0;
    RTE_right_multi_slot_num = 0;
    RTE_Multi_Slot_Num       = 0;
    memset(&RTE_Multi_Slot_Array, 0, sizeof(RTE_Multi_Slot_Array));
    memset(&RTE_Multi_Slots, 0, sizeof(RTE_Multi_Slots));
#ifdef PK_SlotDetect_OffLineTest
    g_IsASlotUpata = 0;
#endif
    BuildSlotsIndex(RTE_Multi_Slot_Array, RTE_Multi_Slot_Num);
}

// filter all the slots by the relative pos between current vehicle and slot //
void Check_MultiSlot_Out_Of_Range(RECAL_DIR U_Cal, RECAL_DIR AVM_Cal)
{
    int i, j;
    float rx, ry;
    Point_T curVehPos;
    float curpos[4] = {0}, tempos[3] = {0}, mirrorpos[3] = {0};
    const float rx_lp_min = 0, rx_lp_max = 25, ry_lp_min = -10,
                ry_lp_max = -0.909f; //  left parallel slot
    const float rx_rp_min = 0, rx_rp_max = 25, ry_rp_min = 0.909f,
                ry_rp_max = 10; //  right parallel slot
    const float rx_lv_min = 3.585f, rx_lv_max = 10, ry_lv_min = 0,
                ry_lv_max = 25; //  left verticle slot rx_lv_min = 3.585f
    const float rx_rv_min = 3.585f, rx_rv_max = 10, ry_rv_min = -25,
                ry_rv_max = 0; //  right verticle slot rx_rv_min = 3.585f
    const float max_dist  = 18;
    memset(&c_Left_U_Slot_Array, 0, sizeof(c_Left_U_Slot_Array));
    memset(&c_Right_U_Slot_Array, 0, sizeof(c_Right_U_Slot_Array));
    memset(&c_Left_AVM_Slot_Array, 0, sizeof(c_Left_AVM_Slot_Array));
    memset(&c_Right_AVM_Slot_Array, 0, sizeof(c_Right_AVM_Slot_Array));

    //  get current pos
    RTE_PK_Location_Get_CurPos(curpos);
    // memcpy(&curVehPos,curpos,sizeof(curVehPos));
    mirrorpos[0] = REAR_VIEW_MIRROR_X;
    Convert(curpos, mirrorpos, tempos);
    memcpy(&curVehPos, tempos, sizeof(curVehPos));

    // left ultrasonic radar slot
    if (U_Cal == CAL_LEFT || U_Cal == CAL_BOTH)
    {
        for (i = 0; i < g_Left_U_Slot_Num; i++)
        {
            rx = Project_PointTo1stPos_rx(g_Left_U_Slot_Array[i].targpos, curVehPos);
            ry = Project_PointTo1stPos_ry(g_Left_U_Slot_Array[i].targpos, curVehPos);

            float rrtheta =
                Project_PosTo1st_rtheta((float *)&g_Left_U_Slot_Array[i].targpos, curpos);
            float rry =
                Project_PosTo1st_ry(curpos, (float *)&g_Left_U_Slot_Array[i].targpos);
            float distance =
                Cal_Dis_Pt2Pt((float *)&g_Left_U_Slot_Array[i].targpos, curpos);

            if (g_Left_U_Slot_Array[i].slotshap == PK_SLOT_LEFT_PARA)
            {
                if ((fabs(rry) > ry_rp_max) ||
                    (rx > rx_lp_min && fabs(rrtheta) > 0.4f * PI) ||
                    ((distance > max_dist) &&
                     (rx < rx_lp_min || rx > rx_lp_max || ry < ry_lp_min ||
                      ry > ry_lp_max))) //  out of range
                {
                    memset(&g_Left_U_Slot_Array[i], 0, sizeof(SlotInfo_T));
                }
                g_Left_U_Slot_Array[i].dist_curveh = distance;
            }
            else if (g_Left_U_Slot_Array[i].slotshap == PK_SLOT_LEFT_VERT)
            {
                if ((fabs(rry) > ry_rp_max) || rx < ry_lv_min ||
                    ((distance > max_dist) &&
                     (rx < rx_lv_min || rx > rx_lv_max || ry < ry_lv_min ||
                      ry > ry_lv_max))) //  out of range
                {
                    memset(&g_Left_U_Slot_Array[i], 0, sizeof(SlotInfo_T));
                }
                g_Left_U_Slot_Array[i].dist_curveh = distance;
            }
        }
    }

    // right ultrasonic radar slot
    if (U_Cal == CAL_RIGHT || U_Cal == CAL_BOTH)
    {
        for (i = 0; i < g_Right_U_Slot_Num; i++)
        {
            rx = Project_PointTo1stPos_rx(g_Right_U_Slot_Array[i].targpos, curVehPos);
            ry = Project_PointTo1stPos_ry(g_Right_U_Slot_Array[i].targpos, curVehPos);

            float rrtheta = Project_PosTo1st_rtheta(
                (float *)&g_Right_U_Slot_Array[i].targpos, curpos);
            float rry =
                Project_PosTo1st_ry(curpos, (float *)&g_Right_U_Slot_Array[i].targpos);
            float distance =
                Cal_Dis_Pt2Pt((float *)&g_Right_U_Slot_Array[i].targpos, curpos);

            if (g_Right_U_Slot_Array[i].slotshap == PK_SLOT_RIGHT_PARA)
            {
                if ((fabs(rry) > ry_rp_max) ||
                    (rx > rx_lp_min && fabs(rrtheta) > 0.4f * PI) ||
                    ((distance > max_dist) &&
                     (rx < rx_rp_min || rx > rx_rp_max || ry < ry_rp_min ||
                      ry > ry_rp_max))) //  out of range
                {
                    memset(&g_Right_U_Slot_Array[i], 0, sizeof(SlotInfo_T));
                }

                g_Right_U_Slot_Array[i].dist_curveh = distance;
            }
            else if (g_Right_U_Slot_Array[i].slotshap == PK_SLOT_RIGHT_VERT)
            {
                if ((fabs(rry) > ry_rp_max) || rx < ry_lv_min ||
                    ((distance > max_dist) &&
                     (rx < rx_rv_min || rx > rx_rv_max || ry < ry_rv_min ||
                      ry > ry_rv_max))) //  out of range
                {
                    memset(&g_Right_U_Slot_Array[i], 0, sizeof(SlotInfo_T));
                }
                g_Right_U_Slot_Array[i].dist_curveh = distance;
            }
        }
    }
    // left avm radar slot
    if (AVM_Cal == CAL_LEFT || AVM_Cal == CAL_BOTH)
    {
        for (i = 0; i < g_Left_AVM_Slot_Num; i++)
        {
            rx = Project_PointTo1stPos_rx(g_Left_AVM_Slot_Array[i].targpos, curVehPos);
            ry = Project_PointTo1stPos_ry(g_Left_AVM_Slot_Array[i].targpos, curVehPos);

            float rrtheta = Project_PosTo1st_rtheta(
                (float *)&g_Left_AVM_Slot_Array[i].targpos, curpos);
            float rry =
                Project_PosTo1st_ry(curpos, (float *)&g_Left_AVM_Slot_Array[i].targpos);
            float distance =
                Cal_Dis_Pt2Pt((float *)&g_Left_AVM_Slot_Array[i].targpos, curpos);

            if (g_Left_AVM_Slot_Array[i].slotshap == PK_SLOT_LEFT_PARA)
            {
                if ((fabs(rry) > ry_rp_max) ||
                    (rx > rx_lp_min && fabs(rrtheta) > 0.4f * PI) ||
                    ((distance > max_dist) &&
                     (rx < rx_lp_min || rx > rx_lp_max || ry < ry_lp_min ||
                      ry > ry_lp_max))) //  out of range
                {
                    memset(&g_Left_AVM_Slot_Array[i], 0, sizeof(SlotInfo_T));
                }
                g_Left_AVM_Slot_Array[i].dist_curveh = distance;
            }
            else if (g_Left_AVM_Slot_Array[i].slotshap == PK_SLOT_LEFT_VERT)
            {
                if ((fabs(rry) > ry_rp_max) || rx < ry_lv_min ||
                    ((distance > max_dist) &&
                     (rx < rx_lv_min || rx > rx_lv_max || ry < ry_lv_min ||
                      ry > ry_lv_max))) //  out of range
                {
                    memset(&g_Left_AVM_Slot_Array[i], 0, sizeof(SlotInfo_T));
                }
                g_Left_AVM_Slot_Array[i].dist_curveh = distance;
            }
        }
    }

    // right AVM radar slot
    if (AVM_Cal == CAL_RIGHT || AVM_Cal == CAL_BOTH)
    {
        for (i = 0; i < g_Right_AVM_Slot_Num; i++)
        {
            rx = Project_PointTo1stPos_rx(g_Right_AVM_Slot_Array[i].targpos, curVehPos);
            ry = Project_PointTo1stPos_ry(g_Right_AVM_Slot_Array[i].targpos, curVehPos);

            float rrtheta = Project_PosTo1st_rtheta(
                (float *)&g_Right_AVM_Slot_Array[i].targpos, curpos);
            float rry =
                Project_PosTo1st_ry(curpos, (float *)&g_Right_AVM_Slot_Array[i].targpos);
            float distance =
                Cal_Dis_Pt2Pt((float *)&g_Right_AVM_Slot_Array[i].targpos, curpos);

            if (g_Right_AVM_Slot_Array[i].slotshap == PK_SLOT_RIGHT_PARA) // para
            {
                if ((fabs(rry) > ry_rp_max) ||
                    (rx > rx_lp_min && fabs(rrtheta) > 0.4f * PI) ||
                    ((distance > max_dist) &&
                     (rx < rx_rp_min || rx > rx_rp_max || ry < ry_rp_min ||
                      ry > ry_rp_max))) //  out of range
                {
                    memset(&g_Right_AVM_Slot_Array[i], 0, sizeof(SlotInfo_T));
                }
                g_Right_AVM_Slot_Array[i].dist_curveh = distance;
            }
            else if (g_Right_AVM_Slot_Array[i].slotshap == PK_SLOT_RIGHT_VERT) // vert
            {
                if ((fabs(rry) > ry_rp_max) || rx < ry_lv_min ||
                    ((distance > max_dist) &&
                     (rx < rx_rv_min || rx > rx_rv_max || ry < ry_rv_min ||
                      ry > ry_rv_max))) //  out of range
                {
                    memset(&g_Right_AVM_Slot_Array[i], 0, sizeof(SlotInfo_T));
                }
                g_Right_AVM_Slot_Array[i].dist_curveh = distance;
            }
        }
    }

    // Resort the slot array
    j = 0;
    for (i = 0; i < MAX_U_SLOT_SIDE_NUM; i++)
    {
        if (g_Left_U_Slot_Array[i].slotshap != PK_SLOT_NO)
        {
            memcpy(&c_Left_U_Slot_Array[j], &g_Left_U_Slot_Array[i], sizeof(SlotInfo_T));
            j++;
        }
    }
    j = 0;
    for (i = 0; i < MAX_U_SLOT_SIDE_NUM; i++)
    {
        if (g_Right_U_Slot_Array[i].slotshap != PK_SLOT_NO)
        {
            memcpy(&c_Right_U_Slot_Array[j], &g_Right_U_Slot_Array[i],
                   sizeof(SlotInfo_T));
            j++;
        }
    }
    j = 0;
    for (i = 0; i < AVM_SLOT_NUM_MAX; i++)
    {
        if (g_Left_AVM_Slot_Array[i].slotshap != PK_SLOT_NO)
        {
            memcpy(&c_Left_AVM_Slot_Array[j], &g_Left_AVM_Slot_Array[i],
                   sizeof(SlotInfo_T));
            j++;
        }
    }
    j = 0;
    for (i = 0; i < AVM_SLOT_NUM_MAX; i++)
    {
        if (g_Right_AVM_Slot_Array[i].slotshap != PK_SLOT_NO)
        {
            memcpy(&c_Right_AVM_Slot_Array[j], &g_Right_AVM_Slot_Array[i],
                   sizeof(SlotInfo_T));
            j++;
        }
    }

    // Re-initialize the slot array
    memset(&g_Left_U_Slot_Array, 0, sizeof(g_Left_U_Slot_Array));
    memset(&g_Right_U_Slot_Array, 0, sizeof(g_Right_U_Slot_Array));
    memset(&g_Left_AVM_Slot_Array, 0, sizeof(g_Left_AVM_Slot_Array));
    memset(&g_Right_AVM_Slot_Array, 0, sizeof(g_Right_AVM_Slot_Array));

    memcpy(&g_Left_U_Slot_Array, &c_Left_U_Slot_Array, sizeof(g_Left_U_Slot_Array));
    memcpy(&g_Right_U_Slot_Array, &c_Right_U_Slot_Array, sizeof(g_Right_U_Slot_Array));
    memcpy(&g_Left_AVM_Slot_Array, &c_Left_AVM_Slot_Array, sizeof(g_Left_AVM_Slot_Array));
    memcpy(&g_Right_AVM_Slot_Array, &c_Right_AVM_Slot_Array,
           sizeof(g_Right_AVM_Slot_Array));
}

// check if an ultrasonic slot can be replaced by a visiual slot in the same place
// -------//
void Replace_U_Slot_By_AVM()
{
    int i, j;
    int newURadarSlotNum;
    int bufferSize = MAX_U_SLOT_SIDE_NUM * sizeof(SlotInfo_T);
    VehPos_T AVM_Tarpos;
    LineSeg_T U_VehObj[4];
    SlotInfo_T tempURadarSlotArray[MAX_U_SLOT_SIDE_NUM];

    // Initialize the memory
    memset(tempURadarSlotArray, 0, bufferSize);
    memcpy(tempURadarSlotArray, g_Left_U_Slot_Array,
           g_Left_U_Slot_Num * sizeof(SlotInfo_T));

    // left avm slot
    for (i = 0; i < g_Left_AVM_Slot_Num; i++)
    {
        if (g_Left_AVM_Slot_Array[i].slotshap != PK_SLOT_NO)
        {
            memcpy(&AVM_Tarpos, &g_Left_AVM_Slot_Array[i].targpos, sizeof(AVM_Tarpos));
            for (j = 0; j < g_Left_U_Slot_Num; j++)
            {
                if (g_Left_U_Slot_Array[j].slotshap != PK_SLOT_NO)
                {
                    Get_Veh_LineSeg(g_Left_U_Slot_Array[j].targpos, U_VehObj, 0);
                    if (PK_PosLineSegCross_Check(AVM_Tarpos, 4, U_VehObj, 0))
                    {
                        tempURadarSlotArray[j].slotshap = PK_SLOT_NO;
                    }
                }
            }
        }
    }

    // Update left URadar slot array
    newURadarSlotNum = 0;
    memset(g_Left_U_Slot_Array, 0, bufferSize);
    for (i = 0; i < g_Left_U_Slot_Num; ++i)
    {
        if (tempURadarSlotArray[i].slotshap != PK_SLOT_NO)
        {
            memcpy(g_Left_U_Slot_Array + newURadarSlotNum, tempURadarSlotArray + i,
                   sizeof(SlotInfo_T));
            ++newURadarSlotNum;
        }
    }
    g_Left_U_Slot_Num = newURadarSlotNum;

    // Initialize the memory
    memset(tempURadarSlotArray, 0, bufferSize);
    memcpy(tempURadarSlotArray, g_Right_U_Slot_Array,
           g_Right_U_Slot_Num * sizeof(SlotInfo_T));

    // right avm slot
    for (i = 0; i < g_Right_AVM_Slot_Num; i++)
    {
        if (g_Right_AVM_Slot_Array[i].slotshap != PK_SLOT_NO)
        {
            memcpy(&AVM_Tarpos, &g_Right_AVM_Slot_Array[i].targpos, sizeof(AVM_Tarpos));
            for (j = 0; j < g_Right_U_Slot_Num; j++)
            {
                if (g_Right_U_Slot_Array[j].slotshap != PK_SLOT_NO)
                {
                    Get_Veh_LineSeg(g_Right_U_Slot_Array[j].targpos, U_VehObj, 0);
                    if (PK_PosLineSegCross_Check(AVM_Tarpos, 4, U_VehObj, 0))
                    {
                        tempURadarSlotArray[j].slotshap = PK_SLOT_NO;
                    }
                }
            }
        }
    }

    // Update right URadar slot array
    newURadarSlotNum = 0;
    memset(g_Right_U_Slot_Array, 0, bufferSize);
    for (i = 0; i < g_Right_U_Slot_Num; ++i)
    {
        if (tempURadarSlotArray[i].slotshap != PK_SLOT_NO)
        {
            memcpy(g_Right_U_Slot_Array + newURadarSlotNum, tempURadarSlotArray + i,
                   sizeof(SlotInfo_T));
            ++newURadarSlotNum;
        }
    }
    g_Right_U_Slot_Num = newURadarSlotNum;
}

void Set_Multi_Slot_ccp(int RTE_left_multi_slot_num,
                        SlotInfo_T RTE_Left_Multi_Slot_Array[MAX_PARKED_SLOTS],
                        int RTE_right_multi_slot_num,
                        SlotInfo_T RTE_Right_Multi_Slot_Array[MAX_PARKED_SLOTS])
{
    static SlotInfo_T last_left_slot, last_right_slot;

    SlotInfo_T near_slot;
    int send_left_flag         = 0;
    static int last_left_Index = 0, last_right_Index = 0;

    int send_flag             = 0;
    const uint8 MinSendTimes  = 9;
    static uint8 sendCnt_left = 0, sendCnt_right = 0;

    if (RTE_left_multi_slot_num == 0)
    {
        last_left_Index = 0;
        sendCnt_left    = 0;
        memset(&last_left_slot, 0, sizeof(last_left_slot));
    }
    if (RTE_right_multi_slot_num == 0)
    {
        last_right_Index = 0;
        sendCnt_right    = 0;
        memset(&last_right_slot, 0, sizeof(last_right_slot));
    }

    if (RTE_left_multi_slot_num > 0)
    {
        if (last_left_Index < RTE_left_multi_slot_num ||
            (RTE_left_multi_slot_num == 3 &&
             Cal_Dis_Pt2Pt(last_left_slot.targpos, RTE_Left_Multi_Slot_Array[0].targpos) >
                 1.0f))
        {
            send_flag = 1;
            near_slot = RTE_Left_Multi_Slot_Array[0];
        }

        if (send_flag)
        { // comment out by xls@2022.5.10
            // SetSlotInfo_ccp(&near_slot.slotobj,&near_slot.targpos);
            send_left_flag = 1;
            sendCnt_left++;
        }
        if (sendCnt_left >= MinSendTimes)
        {
            sendCnt_left = 0;
            last_left_Index++;
            last_left_slot = RTE_Left_Multi_Slot_Array[0];
        }
    }

    send_flag = 0; //  init
    if (RTE_right_multi_slot_num > 0 && send_left_flag == 0)
    {
        if (last_right_Index < RTE_right_multi_slot_num ||
            (RTE_right_multi_slot_num == 3 &&
             Cal_Dis_Pt2Pt(last_right_slot.targpos,
                           RTE_Right_Multi_Slot_Array[0].targpos) > 1.0f))
        {
            near_slot = RTE_Right_Multi_Slot_Array[0];
            send_flag = 1;
        }
        if (send_flag)
        { // comment out by xls@2022.5.10
            // SetSlotInfo_ccp(&near_slot.slotobj, &near_slot.targpos);
            sendCnt_right++;
        }
        if (sendCnt_right >= MinSendTimes)
        {
            sendCnt_right = 0;
            last_right_Index++;
            last_right_slot = RTE_Right_Multi_Slot_Array[0];
        }
    }
}

static bool IsSameSlot(const VehPos_T &target1, const VehPos_T &target2)
{
    float dist  = Cal_Dis_Pt2Pt(target1, target2);
    float theta = Round_PI(target1.theta - target2.theta);
    if (dist < 1.0f && fabs(theta) < PI * 0.4f)
    {
        return true;
    }

    return false;
}

int CheckUradarSlotIsAvm(const SlotInfo_T AvmSlotArray[AVM_SLOT_NUM_MAX],
                         const SlotInfo_T &U_slot_info)
{
    for (int i = 0; i < AVM_SLOT_NUM_MAX; i++)
    {
        if (AvmSlotArray[i].slotshap == PK_SLOT_NO)
        {
            continue;
        }

        if (!IsSameSlot(AvmSlotArray[i].targpos, U_slot_info.targpos))
        {
            continue;
        }

        return 1;
    }

    return 0;
}

int CheckUradarSlotIsValid(AVM_Buff_Info *buff, SlotInfo_T U_slot_info)
{
    /* Update on 2018-10-23
    1.Change the AVM and Uradar Fusion logic, such that the Parallel and Parallel,
    Vertical and Vertical combination would output slot if they are spatially available
    and valid 2.Change from comparing CD vector to the vehicle body edge's vector
    */
    LineSeg_T Target_Segment;
    Point_T Targ_Slot_pt1, Targ_Slot_pt2;
    Point_T Slot_PtB, Slot_PtC, Slot_PtD, Slot_PtE;
    float RxPtB, RxPtC, RxPtD, RxPtE;
    VehPos_T pos;
    float curpos[4];
    int i;
    PK_SlotShapeType AVM_Slot;
    PK_SlotShapeType U_Slot;
    int flag = 1;
    Rect_T CornerPt;
    RTE_PK_Location_Get_CurPos(curpos);
    memcpy(&pos, curpos, sizeof(pos));
    memcpy(&U_Slot, &U_slot_info.slotshap, sizeof(U_Slot));
    // Loop through buffer to check if the slot is valid
    for (i = 0; i < buff->Num; i++)
    {
        // Copy the SlotShape
        memcpy(&AVM_Slot, &buff->SlotShap[i], sizeof(AVM_Slot));

        if (((U_Slot == PK_SLOT_LEFT_PARA) && (AVM_Slot == PK_SLOT_LEFT_PARA)) ||
            ((U_Slot == PK_SLOT_RIGHT_PARA) && (AVM_Slot == PK_SLOT_RIGHT_PARA)) ||
            ((U_Slot == PK_SLOT_LEFT_VERT) && (AVM_Slot == PK_SLOT_LEFT_VERT)) ||
            ((U_Slot == PK_SLOT_RIGHT_VERT) && (AVM_Slot == PK_SLOT_RIGHT_VERT)))
        {
            flag = 0;
        }
        else
        {
            flag = 1;
        }

        if (flag)
        {
            Get_Veh_CornerPt(U_slot_info.targpos, &CornerPt, 0);
            if ((U_Slot == PK_SLOT_LEFT_PARA) || (U_Slot == PK_SLOT_RIGHT_PARA))
            {
                memcpy(&Targ_Slot_pt1, &CornerPt.pt_rr, sizeof(Targ_Slot_pt1));
                memcpy(&Targ_Slot_pt2, &CornerPt.pt_fr, sizeof(Targ_Slot_pt2));
            }
            else if ((U_Slot == PK_SLOT_LEFT_VERT) || (U_Slot == PK_SLOT_RIGHT_VERT))
            {
                memcpy(&Targ_Slot_pt1, &CornerPt.pt_rl, sizeof(Targ_Slot_pt1));
                memcpy(&Targ_Slot_pt2, &CornerPt.pt_rr, sizeof(Targ_Slot_pt2));
            }

            memcpy(&Slot_PtB, &buff->NR[i], sizeof(Slot_PtB));
            memcpy(&Slot_PtC, &buff->FR[i], sizeof(Slot_PtC));
            memcpy(&Slot_PtD, &buff->FF[i], sizeof(Slot_PtD));
            memcpy(&Slot_PtE, &buff->NF[i], sizeof(Slot_PtE));

            Target_Segment.pt1 = Targ_Slot_pt1;
            Target_Segment.pt2 = Targ_Slot_pt2;

            Convert_Line_to_Pos(Target_Segment, &pos);
            RxPtB = Project_PointTo1stPos_rx(pos, Slot_PtB);
            RxPtC = Project_PointTo1stPos_rx(pos, Slot_PtC);
            RxPtD = Project_PointTo1stPos_rx(pos, Slot_PtD);
            RxPtE = Project_PointTo1stPos_rx(pos, Slot_PtE);

            if ((RxPtE >= RxPtB) && (RxPtD >= RxPtC))
            {
                if ((RxPtB >= 0) || (RxPtE > 0) || (RxPtC >= 0) || (RxPtD > 0))
                {
                    return 1;
                }
            }
        }
    }

    return 0;
}

static int getSlotIndex(const VehPos_T &targ, int slotIds[MAX_PARKED_SLOTS], int slotNum)
{
    int type     = 0;
    float targ_x = targ.x;
    float targ_y = targ.y;
    if (targ_x >= 0 && targ_y >= 0) //????
    {
        type = 1;
    }
    else if (targ_x <= 0 && targ_y >= 0) //????
    {
        type = 2;
    }
    else if (targ_x <= 0 && targ_y <= 0) //????
    {
        type = 3;
    }
    else //????
    {
        type = 4;
    }

    int slotId = type * 10000 + fabs(((int)targ_x) % 10) * 1000 +
                 fabs(((int)targ_y) % 10) * 100 +
                 ((int)sqrt(pow2(targ_x) + pow2(targ_y)) % 100);

    // ?-?·???ó, ?±μ?2?3?í?
    int i = 0;
    while (i < slotNum)
    {
        if (slotId == slotIds[i])
        {
            slotId = slotId + 1;
            i      = 0;
            continue;
        }
        i++;
    }

    return slotId;
}

static void BuildSlotsIndex(SlotInfo_T slotArray[MAX_PARKED_SLOTS], int slotNum)
{
    static VehPos_T lastSlotTarg[MAX_PARKED_SLOTS];
    static int lastSlotIndex[MAX_PARKED_SLOTS] = {0};
    static uint8_t lastSlotNum                 = 0;

    if (slotNum == 0)
    {
        memset(lastSlotTarg, 0, sizeof(lastSlotTarg));
        memset(lastSlotIndex, 0, sizeof(lastSlotIndex));
        lastSlotNum = 0;
        return;
    }

    // ?′ó?ò??°μ?ID
    int curIndex[MAX_PARKED_SLOTS];
    memset(curIndex, 0, sizeof(curIndex));
    for (int i = 0; i < slotNum; i++)
    {
        curIndex[i] = 0;
        for (int j = 0; j < lastSlotNum; j++)
        {
            if (IsSameSlot(slotArray[i].targpos, lastSlotTarg[j]))
            {
                curIndex[i] = lastSlotIndex[j];
                break;
            }
        }
    }

    // ??D?éú3é2???o?μ?ID
    for (int i = 0; i < slotNum; i++)
    {
        if (curIndex[i] == 0)
        {
            curIndex[i] = getSlotIndex(slotArray[i].targpos, curIndex, slotNum);
        }
    }

    bool sameflag = true;
    while (sameflag)
    {
        sameflag = false;
        for (int i = 0; i < slotNum; i++)
        {
            for (int j = i + 1; j < slotNum; j++)
            {
                if (curIndex[i] == curIndex[j])
                {
                    curIndex[j] = curIndex[j] + 1;
                    sameflag    = true;
                }
            }
        }
    }

    memset(lastSlotTarg, 0, sizeof(lastSlotTarg));
    memset(lastSlotIndex, 0, sizeof(curIndex));
    for (int i = 0; i < slotNum; i++)
    {
        memcpy((float *)&lastSlotTarg[i], (float *)&slotArray[i].targpos,
               sizeof(VehPos_T));
        slotArray[i].slot_index = curIndex[i];
        lastSlotIndex[i]        = curIndex[i];
    }
    lastSlotNum = slotNum;
}

static SlotInfo_T RTE_Left_Multi_Slot_Array_Out[MULT_SLOT_SIDE_NUM_MAX],
    RTE_Right_Multi_Slot_Array_Out[MULT_SLOT_SIDE_NUM_MAX];
static int RTE_left_out_num = 0, RTE_right_out_num = 0;
// pick out the nearest six slot and limit three most at one side   -------//
void Pick_Out_Near_Slot()
{
    //  Declaration
    float dist_curveh_min = 1e6;
    SlotInfo_T temp_slotInfo;
    int valid_left_slot_num = 0, valid_right_slot_num = 0;
    int i, j; /*, k;*/
    int is_AVM = -1, min_Index = 0;
    int invalid_flag;

    // 20181002: new add variables for slot delay output
    //  static VehPos_T LeftNearTargPos_last,RightNearTargPos_last;
    //  static uint8 left_JumpFlag = 0,right_JumpFlag = 0;
    //  static float left_slotJump_path = 0, right_slotJump_path = 0;
    //  int Multi_PlanRst[MAX_PARKED_SLOTS] = {0};
    //  int left_slot_update = 0,right_slot_update = 0;
    //  float curpos[4];
    //  const float updateDelay_Path = 0.2f;

    // Initilization
    RTE_left_multi_slot_num  = 0;
    RTE_right_multi_slot_num = 0;
    memset(&RTE_Multi_Slot_Array, 0, sizeof(RTE_Multi_Slot_Array));
    memset(&RTE_Left_Multi_Slot_Array, 0, sizeof(RTE_Left_Multi_Slot_Array));
    memset(&RTE_Right_Multi_Slot_Array, 0, sizeof(RTE_Right_Multi_Slot_Array));
    g_Left_U_Slot_Num    = 0;
    g_Right_U_Slot_Num   = 0;
    g_Left_AVM_Slot_Num  = 0;
    g_Right_AVM_Slot_Num = 0;

    // copy
    memcpy(c_Left_U_Slot_Array, g_Left_U_Slot_Array, sizeof(c_Left_U_Slot_Array));
    memcpy(c_Right_U_Slot_Array, g_Right_U_Slot_Array, sizeof(c_Right_U_Slot_Array));
    memcpy(c_Left_AVM_Slot_Array, g_Left_AVM_Slot_Array, sizeof(c_Left_AVM_Slot_Array));
    memcpy(c_Right_AVM_Slot_Array, g_Right_AVM_Slot_Array,
           sizeof(c_Right_AVM_Slot_Array));

    // calculate the valid slot num at one side
    for (i = 0; i < MAX_U_SLOT_SIDE_NUM; i++)
    {
        if (c_Left_U_Slot_Array[i].slotshap != PK_SLOT_NO)
        {
            valid_left_slot_num++;
            g_Left_U_Slot_Num++;
        }
    }
    for (i = 0; i < AVM_SLOT_NUM_MAX; i++)
    {
        if (c_Left_AVM_Slot_Array[i].slotshap != PK_SLOT_NO)
        {
            valid_left_slot_num++;
            g_Left_AVM_Slot_Num++;
        }
    }
    for (i = 0; i < MAX_U_SLOT_SIDE_NUM; i++)
    {
        if (c_Right_U_Slot_Array[i].slotshap != PK_SLOT_NO)
        {
            valid_right_slot_num++;
            g_Right_U_Slot_Num++;
        }
    }
    for (i = 0; i < AVM_SLOT_NUM_MAX; i++)
    {
        if (c_Right_AVM_Slot_Array[i].slotshap != PK_SLOT_NO)
        {
            valid_right_slot_num++;
            g_Right_AVM_Slot_Num++;
        }
    }

    // ????????
    for (j = 0; j < g_Left_U_Slot_Num; j++)
    {
        if (c_Left_U_Slot_Array[j].slotshap != PK_SLOT_NO)
        {
            invalid_flag = 0;
            invalid_flag +=
                CheckUradarSlotIsAvm(c_Left_AVM_Slot_Array, c_Left_U_Slot_Array[j]);
            invalid_flag +=
                CheckUradarSlotIsAvm(c_Right_AVM_Slot_Array, c_Left_U_Slot_Array[j]);
            if (invalid_flag > 0)
            {
                c_Left_U_Slot_Array[j].slotshap = PK_SLOT_NO;
            }
        }
    }

    for (j = 0; j < g_Right_U_Slot_Num; j++)
    {
        if (c_Right_U_Slot_Array[j].slotshap != PK_SLOT_NO)
        {
            invalid_flag = 0;

            invalid_flag +=
                CheckUradarSlotIsAvm(c_Left_AVM_Slot_Array, c_Right_U_Slot_Array[j]);
            invalid_flag +=
                CheckUradarSlotIsAvm(c_Right_AVM_Slot_Array, c_Right_U_Slot_Array[j]);
            if (invalid_flag > 0)
            {
                c_Right_U_Slot_Array[j].slotshap = PK_SLOT_NO;
            }
        }
    }

    // according to the valid slot num adopt different method
    if (valid_left_slot_num > 0)
    {
        for (i = 0; i < valid_left_slot_num;
             i++) // the max searching number can be optimized to 3. if the total num is
                  // less than 3 then set i as num, otherwise set i equal 3
        {
            dist_curveh_min = 1e6f;
            is_AVM          = -1; // init
            for (j = 0; j < g_Left_U_Slot_Num; j++)
            {
                // dist_curveh_min = 1e6;
                if (c_Left_U_Slot_Array[j].slotshap != PK_SLOT_NO)
                {
                    invalid_flag = 0;

                    invalid_flag += CheckUradarSlotIsAvm(c_Left_AVM_Slot_Array,
                                                         c_Left_U_Slot_Array[j]);
                    invalid_flag += CheckUradarSlotIsAvm(c_Right_AVM_Slot_Array,
                                                         c_Left_U_Slot_Array[j]);

                    if ((c_Left_U_Slot_Array[j].dist_curveh < dist_curveh_min) &&
                        (invalid_flag == 0)) //  avoid the same near slot
                    {
                        dist_curveh_min = c_Left_U_Slot_Array[j].dist_curveh;
                        memcpy(&temp_slotInfo, &c_Left_U_Slot_Array[j],
                               sizeof(temp_slotInfo));
                        is_AVM    = 0;
                        min_Index = j;
                    }
                    else if (invalid_flag > 0)
                    {
                        c_Left_U_Slot_Array[j].slotshap = PK_SLOT_NO;
                    }
                }
            }

            for (j = 0; j < g_Left_AVM_Slot_Num; j++)
            {
                if (c_Left_AVM_Slot_Array[j].slotshap != PK_SLOT_NO)
                {
                    if (c_Left_AVM_Slot_Array[j].dist_curveh <
                        dist_curveh_min) //  avoid the same near slot
                    {
                        dist_curveh_min = c_Left_AVM_Slot_Array[j].dist_curveh;
                        memcpy(&temp_slotInfo, &c_Left_AVM_Slot_Array[j],
                               sizeof(temp_slotInfo));
                        is_AVM    = 1;
                        min_Index = j;
                    }
                }
            }

            if (is_AVM > -1)
            {
                memcpy(&RTE_Left_Multi_Slot_Array[RTE_left_multi_slot_num],
                       &temp_slotInfo, sizeof(temp_slotInfo));
                RTE_left_multi_slot_num++;
                if (is_AVM)
                {
                    c_Left_AVM_Slot_Array[min_Index].slotshap =
                        PK_SLOT_NO; //  do not affect the next min dist slot
                }
                else
                {
                    c_Left_U_Slot_Array[min_Index].slotshap = PK_SLOT_NO;
                }
            }
            // else break the for loop
            if (RTE_left_multi_slot_num >= MULT_SLOT_SIDE_NUM_MAX)
            {
                break;
            }
        }
    }

    if (valid_right_slot_num > 0)
    {
        for (i = 0; i < valid_right_slot_num; i++)
        {
            dist_curveh_min = 1e6f;
            is_AVM          = -1; // init
            for (j = 0; j < g_Right_U_Slot_Num; j++)
            {
                if (c_Right_U_Slot_Array[j].slotshap != PK_SLOT_NO)
                {
                    invalid_flag = 0;

                    invalid_flag += CheckUradarSlotIsAvm(c_Left_AVM_Slot_Array,
                                                         c_Right_U_Slot_Array[j]);
                    invalid_flag += CheckUradarSlotIsAvm(c_Right_AVM_Slot_Array,
                                                         c_Right_U_Slot_Array[j]);

                    if ((c_Right_U_Slot_Array[j].dist_curveh < dist_curveh_min) &&
                        (invalid_flag == 0)) //  avoid the same near slot
                    {
                        dist_curveh_min = c_Right_U_Slot_Array[j].dist_curveh;
                        memcpy(&temp_slotInfo, &c_Right_U_Slot_Array[j],
                               sizeof(temp_slotInfo));
                        is_AVM    = 0;
                        min_Index = j;
                    }
                    else if (invalid_flag > 0)
                    {
                        c_Right_U_Slot_Array[j].slotshap = PK_SLOT_NO;
                    }
                }
            }

            for (j = 0; j < g_Right_AVM_Slot_Num; j++)
            {
                if (c_Right_AVM_Slot_Array[j].slotshap != PK_SLOT_NO)
                {
                    if (c_Right_AVM_Slot_Array[j].dist_curveh <
                        dist_curveh_min) //  avoid the same near slot
                    {
                        dist_curveh_min = c_Right_AVM_Slot_Array[j].dist_curveh;
                        memcpy(&temp_slotInfo, &c_Right_AVM_Slot_Array[j],
                               sizeof(temp_slotInfo));
                        is_AVM    = 1;
                        min_Index = j;
                    }
                }
            }
            if (is_AVM > -1)
            {
                memcpy(&RTE_Right_Multi_Slot_Array[RTE_right_multi_slot_num],
                       &temp_slotInfo, sizeof(temp_slotInfo));
                RTE_right_multi_slot_num++;
                if (is_AVM)
                {
                    c_Right_AVM_Slot_Array[min_Index].slotshap =
                        PK_SLOT_NO; //  do not affect the next min dist slot
                }
                else
                {
                    c_Right_U_Slot_Array[min_Index].slotshap = PK_SLOT_NO;
                }
            }

            if (RTE_right_multi_slot_num >= MULT_SLOT_SIDE_NUM_MAX)
            {
                break;
            }
        }
    }

    memcpy(RTE_Left_Multi_Slot_Array_Out, RTE_Left_Multi_Slot_Array,
           sizeof(RTE_Left_Multi_Slot_Array_Out));
    RTE_left_out_num = RTE_left_multi_slot_num;

    memcpy(RTE_Right_Multi_Slot_Array_Out, RTE_Right_Multi_Slot_Array,
           sizeof(RTE_Right_Multi_Slot_Array_Out));
    RTE_right_out_num = RTE_right_multi_slot_num;

    // 20240412:??????????,????????????
    //   put left and right slots together
    //  int indexXY = 0;
    //  int targpos_dist = 0;
    RTE_Multi_Slot_Num = 0;
    for (i = 0; i < RTE_left_out_num && RTE_Multi_Slot_Num < MAX_PARKED_SLOTS; i++)
    {
        // targpos_dist = (int)sqrt(pow2(RTE_Left_Multi_Slot_Array_Out[i].targpos.x) +
        // pow2(RTE_Left_Multi_Slot_Array_Out[i].targpos.y)); indexXY =
        // (int)fabs(2*RTE_Left_Multi_Slot_Array_Out[i].targpos.x +
        // RTE_Left_Multi_Slot_Array_Out[i].targpos.y);

        memcpy(&RTE_Multi_Slot_Array[RTE_Multi_Slot_Num],
               &RTE_Left_Multi_Slot_Array_Out[i], sizeof(SlotInfo_T));
        RTE_Multi_Slot_Num++;
    }

    for (i = 0; i < RTE_right_out_num && RTE_Multi_Slot_Num < MAX_PARKED_SLOTS; i++)
    {
        // targpos_dist = (int)sqrt(pow2(RTE_Right_Multi_Slot_Array_Out[i].targpos.x) +
        // pow2(RTE_Right_Multi_Slot_Array_Out[i].targpos.y)); indexXY =
        // (int)fabs(2*RTE_Right_Multi_Slot_Array_Out[i].targpos.x +
        // RTE_Right_Multi_Slot_Array_Out[i].targpos.y);

        memcpy(&RTE_Multi_Slot_Array[RTE_Multi_Slot_Num],
               &RTE_Right_Multi_Slot_Array_Out[i], sizeof(SlotInfo_T));
        RTE_Multi_Slot_Num++;
    }

    BuildSlotsIndex(RTE_Multi_Slot_Array, RTE_Multi_Slot_Num);

    Set_Multi_Slot_ccp(RTE_left_out_num, RTE_Left_Multi_Slot_Array_Out, RTE_right_out_num,
                       RTE_Right_Multi_Slot_Array_Out);

    // Combine both Multi_Slot_Array and Multi_Slot_Num into the same STRUCT to pass
    // along.
    for (i = 0; i < RTE_Multi_Slot_Num; i++)
    {
        if (RTE_Multi_Slot_Array[i].slot_index == 0)
        {
            continue;
        }
        for (int j = i + 1; j < RTE_Multi_Slot_Num; j++)
        {
            if (RTE_Multi_Slot_Array[j].slot_index == 0)
            {
                continue;
            }
            if (RTE_Multi_Slot_Array[i].slot_index == RTE_Multi_Slot_Array[j].slot_index)
            {
                log_warn("Different parking Spaces have the same number!\n"); //?????????
                RTE_Multi_Slot_Array[i].slot_index = 0;
            }
        }
    }

    Multi_Slot_Array_T Old_RTE_Multi_Slots;
    RTE_PK_SlotDetect_Get_Multi_SlotInfo(&Old_RTE_Multi_Slots);

    RTE_Multi_Slots.multiNum = 0;
    for (i = 0; i < RTE_Multi_Slot_Num; i++)
    {
        if (RTE_Multi_Slot_Array[i].slot_index == 0)
        {
            continue;
        }

        for (int j = 0; j < Old_RTE_Multi_Slots.multiNum; j++)
        {
            if (RTE_Multi_Slot_Array[i].slotshap ==
                    Old_RTE_Multi_Slots.multiArray[j].slotshap &&
                RTE_Multi_Slot_Array[i].slot_index !=
                    Old_RTE_Multi_Slots.multiArray[j].slot_index &&
                fabs(Round_PI(RTE_Multi_Slot_Array[i].targpos.theta) -
                     Round_PI(Old_RTE_Multi_Slots.multiArray[j].targpos.theta)) < 0.30f &&
                (pow2(RTE_Multi_Slot_Array[i].slotobj.ptB.x -
                      Old_RTE_Multi_Slots.multiArray[j].slotobj.ptB.x) +
                 pow2(RTE_Multi_Slot_Array[i].slotobj.ptB.y -
                      Old_RTE_Multi_Slots.multiArray[j].slotobj.ptB.y)) < 1.0f)
            {
                // log_warn("The same parking number is different !\n");//????????
                RTE_Multi_Slot_Array[i].slot_index =
                    Old_RTE_Multi_Slots.multiArray[j].slot_index;
            }
        }

        memcpy(&RTE_Multi_Slots.multiArray[RTE_Multi_Slots.multiNum++],
               &RTE_Multi_Slot_Array[i], sizeof(SlotInfo_T));
    }
}

/*  mutiple slot detect for both ultra radar and visual slots   */
void Multi_SlotDetect_A(void)
{
    RECAL_DIR U_Cal = CAL_NO, AVM_Cal = CAL_NO; // researched all slots
    // step1------   detect all possible visual and ultrasonic slots -------//
    U_Cal   = URadar_SlotDetect();
    AVM_Cal = AVM_SlotDetect();
    // step2------   filter all the slots by the relative pos between current vehicle and
    // slot   -------//
    Check_MultiSlot_Out_Of_Range(U_Cal, AVM_Cal);
    // step3------   check if an ultrasonic slot can be replaced by a visiual slot in the
    // same place -------//
    Replace_U_Slot_By_AVM();
    // step4------   pick out the nearest six slot and limit three most at one side
    // -------//
    Pick_Out_Near_Slot();
    //  send RTE
    // RTE_PK_SlotDetect_Set_Multi_SlotInfo(RTE_Multi_Slot_Array);
    // RTE_PK_SlotDetect_Set_Multi_SlotNum(RTE_Multi_Slot_Num);
    // Multi_slot_num_ccp = RTE_Multi_Slot_Num;
}

int Is_CurPos_Insert_Slot_BCDE(VehPos_T targpos, SlotObj_T slotobj,
                               PK_SlotShapeType SlotShap)
{
    float curpos[4];
    VehPos_T curVehPos;
    Rect_T CornerPt;
    Relation_T BC_inner_dir, DE_inner_dir;
    Relation_T pt_BC_dir, pt_DE_dir;
    float rx;
    if (SlotShap == PK_SLOT_LEFT_VERT)
    {
        BC_inner_dir = PK_ON_RIGHT;
        DE_inner_dir = PK_ON_RIGHT;
    }
    else
    {
        BC_inner_dir = PK_ON_LEFT;
        DE_inner_dir = PK_ON_LEFT;
    }

    RTE_PK_Location_Get_CurPos(curpos);
    memcpy(&curVehPos, curpos, sizeof(curVehPos));
    rx = Project_PosTo1stPos_rx(targpos, curVehPos);
    if (rx < 5.0f)
    {
        Get_Veh_CornerPt(curVehPos, &CornerPt, 0);

        //  check curpos_RL
        Get_Dist_Dir_Pt2PointLine(
            slotobj.ptB, slotobj.ptC, CornerPt.pt_rl, NULL,
            &pt_BC_dir); // pt_BC_dir =
                         // PtStayLine_dir(slot_BCDE[0],slot_BCDE[1],curpos_RL);
        Get_Dist_Dir_Pt2PointLine(
            slotobj.ptD, slotobj.ptE, CornerPt.pt_rl, NULL,
            &pt_DE_dir); // pt_DE_dir =
                         // PtStayLine_dir(slot_BCDE[2],slot_BCDE[3],curpos_RL);
        if (pt_BC_dir == BC_inner_dir && pt_DE_dir == DE_inner_dir)
        {
            return 1;
        }
        //  check curpos_RR
        Get_Dist_Dir_Pt2PointLine(
            slotobj.ptB, slotobj.ptC, CornerPt.pt_rr, NULL,
            &pt_BC_dir); // pt_BC_dir =
                         // PtStayLine_dir(slot_BCDE[0],slot_BCDE[1],curpos_RR);
        Get_Dist_Dir_Pt2PointLine(
            slotobj.ptD, slotobj.ptE, CornerPt.pt_rr, NULL,
            &pt_DE_dir); // pt_DE_dir =
                         // PtStayLine_dir(slot_BCDE[2],slot_BCDE[3],curpos_RR);
        if (pt_BC_dir == BC_inner_dir && pt_DE_dir == DE_inner_dir)
        {
            return 1;
        }
    }
    return 0;
}
