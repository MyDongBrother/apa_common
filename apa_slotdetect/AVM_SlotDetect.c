/**
 * @file AVM_SlotDetect.c
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
#include "URadar_SlotDetect.h"
#include "MathFunc.h"
#include "PK_SD_Common.h"
#include "Module_API.h"
#include "MathFunc.h"
#include "Rte_ComIF.h"

/*********************************************************************************************************
**Include headers of the component
*********************************************************************************************************/
#include "AVM_SlotDetect.h"

/*********************************************************************************************************
**Include other headers
*********************************************************************************************************/
#include <string.h>
#include "hobotlog/hobotlog.hpp"
/*********************************************************************************************************
** Definition of structures and enumerated variables
*********************************************************************************************************/

/*********************************************************************************************************
** Declaration for inner used variables
*********************************************************************************************************/

static AVM_Buff_Info g_Left_BuffInfo, g_Right_BuffInfo;
static int L_Buff_Num      = 0,
           R_Buff_Num      = 0; // to help decide whether to calculate all avm slot
static int L_Buff_Num_last = 0, R_Buff_Num_last = 0;

/*********************************************************************************************************
** Declaration for exported variables
*********************************************************************************************************/
SlotInfo_T g_Left_AVM_Slot_Array[AVM_SLOT_NUM_MAX];
int g_Left_AVM_Slot_Num = 0;
SlotInfo_T g_Right_AVM_Slot_Array[AVM_SLOT_NUM_MAX];
int g_Right_AVM_Slot_Num  = 0;
static float Last_VehPath = 0;
/*********************************************************************************************************
** Definition of AVM_SlotDetect functions
*********************************************************************************************************/
int Remove_Wrong_Dir_FusionObj(FusionObj_T *fusionObj, Point_T curNearFrontPt,
                               Point_T curNearRearPt);
void Cal_EF_Displacement(FusionObj_T fusionObj, Relation_T slotDir, SlotObj_T *slotObj,
                         float *EFDisp);
void Cal_CD_Search_Region(VehPos_T tarPose, Vec2_T slotVec, Point_T slotB, Point_T slotC,
                          Point_T slotD, Point_T slotE, Point_T *frontPt1,
                          Point_T *frontPt2, Point_T *rearPt1, Point_T *rearPt2);

void Init_AVM_SlotDetect(void)
{
    g_Left_AVM_Slot_Num  = 0;
    g_Right_AVM_Slot_Num = 0;
    Last_VehPath         = 0;
    L_Buff_Num_last      = 0;
    R_Buff_Num_last      = 0;
    L_Buff_Num           = 0;
    R_Buff_Num           = 0;
    memset(&g_Left_AVM_Slot_Array, 0, sizeof(g_Left_AVM_Slot_Array));
    memset(&g_Right_AVM_Slot_Array, 0, sizeof(g_Right_AVM_Slot_Array));
    memset(&g_Left_BuffInfo, 0, sizeof(g_Left_BuffInfo));
    memset(&g_Right_BuffInfo, 0, sizeof(g_Right_BuffInfo));
}

//  cal the side free dist from nearpt to targpos
// Calculate the distance from a point to the edge of the vehicle
float Get_Point_Targpos_side_free_dist(PK_SlotShapeType SlotShap, VehPos_T tarpos,
                                       Point_T nearpt, int is_near_BC)
{
    float side_free_dist = 0;
    if (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA)
    {
        if (is_near_BC == 1)
        {
            side_free_dist =
                fabsf(Project_PointTo1stPos_rx(tarpos, nearpt)) - REAR_SUSPENSION;
        }
        else
        {
            side_free_dist = fabsf(Project_PointTo1stPos_rx(tarpos, nearpt)) -
                             (VEHICLE_LEN - REAR_SUSPENSION);
        }
    }
    else
    {
        side_free_dist =
            fabsf(Project_PointTo1stPos_ry(tarpos, nearpt)) - VEHICLE_WID * 0.5f;
    }
    return side_free_dist;
}

// Calculate initial target pose and adjust it according to ultrasonic obstacles, i.e.,
// line objects. Return: 1-Success, 0-Error
int Cal_AVM_TargPos_By_FusionObjA(Point_T NearFrontPt, Point_T NearRearPt,
                                  Point_T FarFrontPt, Point_T FarRearPt,
                                  PK_SlotShapeType SlotShap, Vec2_T SlotVec,
                                  Vec2_T Vert_to_Curb_Vec, VehPos_T *TargPos,
                                  int SlotObjAttr[5])
{
    // float SlotDepth; // depth of a parallel slot or a vertical slot
    Point_T SlotB, SlotE; //  slot corners
    // line property: 1-near vehicle, 2-curb, 3-far vehicle, 4-5510

    LineSeg_T NearLineSeg;

    // Relation_T SlotDir;  //  slot direction
    float div_x, div_y; //  offset used to compute initial target pose
    float TargTheta;
    // Point_T NearCentPt_1;,NearCentPt_2;

    VehPos_T m_TargPos, NearVec;
    int update_flag = 1;
    // float slot_free_dist_min = 0;
    // float allow_targpos_bias = 0;
    float Dy_fr, Dy_ff, Average_Length_To_FarPts;

    // if (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_LEFT_VERT)
    // {
    //  //RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left(&g_FS_ObsInfo_A);
    //  SlotDir = PK_ON_LEFT;
    // }
    // else if(SlotShap == PK_SLOT_RIGHT_PARA || SlotShap == PK_SLOT_RIGHT_VERT)
    // {
    //  //RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right(&g_FS_ObsInfo_A);
    //  SlotDir = PK_ON_RIGHT;
    // }

    memcpy(&SlotB, &NearRearPt, sizeof(SlotB));
    memcpy(&SlotE, &NearFrontPt, sizeof(SlotE));
    TargTheta = atan2f(SlotE.y - SlotB.y, SlotE.x - SlotB.x); // slot orientation

    NearLineSeg.pt1 = NearRearPt;
    NearLineSeg.pt2 = NearFrontPt;
    Convert_Line_to_Pos(NearLineSeg, &NearVec);
    Dy_fr                    = fabsf(Project_PointTo1stPos_ry(NearVec, FarRearPt));
    Dy_ff                    = fabsf(Project_PointTo1stPos_ry(NearVec, FarFrontPt));
    Average_Length_To_FarPts = (Dy_fr + Dy_ff) / 2.0f;

    // Paramters used to generate the initial target pose

    // 对于有限位杆的平行车位，若视觉感知得到的限位杆坐标误差较小，可以根据限位杆调整平行车位目标位置，防止泊车时车辆因限位杆重规划（特别是前方有障碍物的平行车位，重规划成功率低）
    if (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA)
    {
        if (SlotShap == PK_SLOT_LEFT_PARA)
        {
            div_x = (Cal_Dis_Pt2Pt(NearFrontPt, NearRearPt) - VEHICLE_LEN) * 0.5f +
                    REAR_SUSPENSION + AVM_Para_TargPos_off_ldx;
        }
        else
        {
            div_x = (Cal_Dis_Pt2Pt(NearFrontPt, NearRearPt) - VEHICLE_LEN) * 0.5f +
                    REAR_SUSPENSION + AVM_Para_TargPos_off_rdx;
        }

        if (Average_Length_To_FarPts <= AVM_MinFarToNearDis)
        {
            div_y = VEHICLE_WID / 2.0f; // AVM_Para_TargPos_off_dy = 0.15
        }
        else if (Average_Length_To_FarPts >= AVM_MinFarToNearDis &&
                 Average_Length_To_FarPts <= AVM_MaxFarToNearDis)
        {
            div_y = Average_Length_To_FarPts / 2.0f;
        }
        else
        {
            div_y = AVM_MaxFarToNearDis / 2.0f;
        }

        div_y += AVM_Para_TargPos_off_dy;

        m_TargPos.x = NearRearPt.x + SlotVec.vx * div_x + Vert_to_Curb_Vec.vx * div_y;
        m_TargPos.y = NearRearPt.y + SlotVec.vy * div_x + Vert_to_Curb_Vec.vy * div_y;
    }
    else if (SlotShap == PK_SLOT_LEFT_VERT || SlotShap == PK_SLOT_RIGHT_VERT)
    {
        float AVM_Vert_TargPos_off_dy = 0;
        float AVM_Vert_TargPos_off_dx = 0;
        if (SlotShap == PK_SLOT_LEFT_VERT)
        {
            //     AVM_Vert_TargPos_off_dy = AVM_Vert_TargPos_off_ldy;
            AVM_Vert_TargPos_off_dy = searching_vert_slot_off_ldy;
        }
        else
        {
            //     AVM_Vert_TargPos_off_dy = AVM_Vert_TargPos_off_rdy;
            AVM_Vert_TargPos_off_dy = searching_vert_slot_off_rdy;
        }

        div_y = (VEHICLE_LEN - REAR_SUSPENSION) +
                AVM_Vert_TargPos_off_dy; // AVM_Vert_TargPos_off_dy = 0.05
        div_x = Cal_Dis_Pt2Pt(NearFrontPt, NearRearPt) * 0.5f + AVM_Vert_TargPos_off_dx;

        m_TargPos.x = NearRearPt.x + SlotVec.vx * div_x + Vert_to_Curb_Vec.vx * div_y;
        m_TargPos.y = NearRearPt.y + SlotVec.vy * div_x + Vert_to_Curb_Vec.vy * div_y;

        if (SlotShap == PK_SLOT_LEFT_VERT)
        {
            TargTheta -= PI / 2.0f;
            TargTheta = TargTheta + searching_vert_slot_off_ltheta;
        }
        else if (SlotShap == PK_SLOT_RIGHT_VERT)
        {
            TargTheta += PI / 2.0f;
            TargTheta = TargTheta + searching_vert_slot_off_rtheta;
        }
    }
    m_TargPos.theta = Round_PI(TargTheta);

    if (update_flag == 1)
    {
        memcpy(TargPos, &m_TargPos, sizeof(m_TargPos));
    }

    return update_flag;
}

//  sometimes the front side radar not detect obs but the rear radar detect
int Update_AVM_SlotObj_By_Rear_FusionObj_A(SlotObj_T *slotobj, VehPos_T *targpos,
                                           PK_SlotShapeType slotshap)
{
    int StartIndex, i, j, LineDir;
    LineSeg_T segment;
    Point_T slot_B, slot_C, slot_D, slot_E, curpt, near_BC_pt, near_DE_pt;
    float BC_pt_targ_dist_min = 1e6f, DE_pt_targ_dist_min = 1e6f, dist_temp;
    Relation_T inner_BC_dir, inner_DE_dir, targ_BC_dir, targ_DE_dir;
    Relation_T pt_BC_dir, pt_DE_dir, pt_targ_dir;
    float free_dist_BC, free_dist_DE;
    float slot_theta, temp;
    Vec2_T slotvec, Vec_targ, Vec_seg;
    float bias_BC = 0, bias_DE = 0;
    int update_flag = 0;
    float ry_1, ry_2, ry_min = 0;
    float AngOfTargSeg, OccupiedThreshold;

    memcpy(&slot_B, &slotobj->ptB, sizeof(slot_B));
    memcpy(&slot_C, &slotobj->ptC, sizeof(slot_C));
    memcpy(&slot_D, &slotobj->ptD, sizeof(slot_D));
    memcpy(&slot_E, &slotobj->ptE, sizeof(slot_E));
    //  for parallel slot it's no need to use the rear side radars to fix BC and DE

    if (slotshap == PK_SLOT_LEFT_VERT)
    {
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Left_Raw(&g_FS_RearObsInfo_A);
        inner_BC_dir = PK_ON_RIGHT;
        inner_DE_dir = PK_ON_RIGHT;
        targ_BC_dir  = PK_ON_RIGHT;
        targ_DE_dir  = PK_ON_LEFT;
        slot_theta   = targpos->theta + PI * 0.5f;
    }
    else if (slotshap == PK_SLOT_RIGHT_VERT)
    {
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Right_Raw(&g_FS_RearObsInfo_A);
        inner_BC_dir = PK_ON_LEFT;
        inner_DE_dir = PK_ON_LEFT;
        targ_BC_dir  = PK_ON_LEFT;
        targ_DE_dir  = PK_ON_RIGHT;
        slot_theta   = targpos->theta - PI * 0.5f;
    }
    else
    {
        return 0;
    }

    slotvec.vx = cosf(slot_theta);
    slotvec.vy = sinf(slot_theta);

    //  find the cross segment to tarpos
    StartIndex = SF_OBJ_NUM - g_FS_RearObsInfo_A.num;
    for (i = StartIndex; i < SF_OBJ_NUM; i++)
    {
        if (g_FS_RearObsInfo_A.attr[i] == 0) //  invalid line segments
        {
            continue;
        }
        LineDir = g_FS_RearObsInfo_A.attr[i] / 10000;
        if ((slotshap == PK_SLOT_LEFT_VERT && LineDir == 2) ||
            (slotshap == PK_SLOT_RIGHT_VERT &&
             LineDir == 1)) //  ignore segments not in the slot's direction
        {
            continue;
        }
        memcpy(&segment, &g_FS_RearObsInfo_A.obj[i], sizeof(segment));
        ry_1 = Project_PointTo1stPos_ry(*targpos, segment.pt1);
        ry_2 = Project_PointTo1stPos_ry(*targpos, segment.pt2);

        if (ry_1 * ry_2 > 0) //  two end points are in the same direction to targpos
        {
            for (j = 0; j < 2; j++)
            {
                if (j == 0)
                {
                    memcpy(&curpt, &segment.pt1, sizeof(curpt));
                }
                else
                {
                    memcpy(&curpt, &segment.pt2, sizeof(curpt));
                }

                Get_Dist_Dir_Pt2PointLine(slot_B, slot_C, curpt, NULL, &pt_BC_dir);
                Get_Dist_Dir_Pt2PointLine(slot_D, slot_E, curpt, NULL, &pt_DE_dir);
                dist_temp   = Project_PointTo1stPos_ry(*targpos, curpt);
                pt_targ_dir = dist_temp > 0 ? PK_ON_LEFT : PK_ON_RIGHT;

                //  check if BC need to update
                dist_temp = fabsf(dist_temp);
                if (pt_BC_dir == inner_BC_dir && pt_targ_dir == targ_BC_dir)
                {
                    if (dist_temp < BC_pt_targ_dist_min)
                    {
                        BC_pt_targ_dist_min = dist_temp;
                        memcpy(&near_BC_pt, &curpt, sizeof(curpt));
                    }
                }
                //  check if DE need to update
                if (pt_DE_dir == inner_DE_dir && pt_targ_dir == targ_DE_dir)
                {
                    if (dist_temp < DE_pt_targ_dist_min)
                    {
                        DE_pt_targ_dist_min = dist_temp;
                        memcpy(&near_DE_pt, &curpt, sizeof(curpt));
                    }
                }
            }
        }
        else
        {
            Vec_targ.vx = cosf(targpos->theta);
            Vec_targ.vy = sinf(targpos->theta);

            Vec_seg.vx = segment.pt2.x - segment.pt1.x;
            Vec_seg.vy = segment.pt2.y - segment.pt1.y;

            AngOfTargSeg = Get_CrossAngle_Vec2(Vec_targ, Vec_seg);

            ry_min = rte_min(fabsf(ry_1), fabsf(ry_2));

            OccupiedThreshold = fabsf(ry_min / sinf(AngOfTargSeg));

            if (OccupiedThreshold >= AVM_OccupyThresh_RearFusionObjCheck)
            {
                return -1;
            }
        }
    }
    //  check if BC need to update
    if (BC_pt_targ_dist_min < 1e6f)
    {
        free_dist_BC =
            Get_Point_Targpos_side_free_dist(slotshap, *targpos, near_BC_pt, 1);
        if (free_dist_BC < AVM_MinBCFreeDist_RearFusionObjCheck)
        {
            free_dist_BC = AVM_MinBCFreeDist_RearFusionObjCheck;
        }
        temp = rte_min(Get_Point_Targpos_side_free_dist(slotshap, *targpos, slot_B, 1),
                       Get_Point_Targpos_side_free_dist(slotshap, *targpos, slot_C, 1));
        if (temp > AVM_MinSlotToVehBodyDist_RearFusionObjCheck) //  else there is no need
                                                                //  to update
        {
            bias_BC = temp - free_dist_BC;
            if (bias_BC > 0)
            {
                temp = slotvec.vx * bias_BC;
                slotobj->ptB.x += temp; //  B
                slotobj->ptC.x += temp; //  C
                temp = slotvec.vy * bias_BC;
                slotobj->ptB.y += temp; //  B
                slotobj->ptC.y += temp; //  C
                update_flag = 1;
            }
        }
    }
    //  check if DE need to update
    if (DE_pt_targ_dist_min < 1e6)
    {
        free_dist_DE =
            Get_Point_Targpos_side_free_dist(slotshap, *targpos, near_DE_pt, 0);
        if (free_dist_DE < AVM_MinDEFreeDist_RearFusionObjCheck)
        {
            free_dist_DE = AVM_MinDEFreeDist_RearFusionObjCheck;
        }
        temp = rte_min(Get_Point_Targpos_side_free_dist(slotshap, *targpos, slot_D, 1),
                       Get_Point_Targpos_side_free_dist(slotshap, *targpos, slot_E, 1));
        if (temp > AVM_MinSlotToVehBodyDist_RearFusionObjCheck) //  else there is no need
                                                                //  to update
        {
            bias_DE = temp - free_dist_DE;
            if (bias_DE > 0)
            {
                temp = -slotvec.vx * bias_DE;
                slotobj->ptD.x += temp; //  D
                slotobj->ptE.x += temp; //  E
                temp = -slotvec.vy * bias_DE;
                slotobj->ptD.y += temp; //  D
                slotobj->ptE.y += temp; //  E
                update_flag = 1;
            }
        }
    }
    return update_flag;
}

// 可达空间对虚拟车位的裁剪更新
int Update_AVM_SlotObj_By_FusionObjA(PK_SlotShapeType SlotShap, VehPos_T *TargPos,
                                     Vec2_T SlotVec, Vec2_T Vert_to_Curb_Vec,
                                     SlotObj_T *SlotObj, int SlotObjAttr[5])
{
    int i, j, StartIndex, CurLinePro, temp_dir = 0, insertFlag = 0, isParaSlot = 0;
    Relation_T SlotDir; //  1-left side, 2-right side
    Point_T SlotA, SlotB, SlotC, SlotD, SlotE, SlotF, CurPt, TmpSlotPt, rearPt1, rearPt2,
        frontPt1, frontPt2, tarPosition, crossPt1, crossPt2;
    float TransDist_BC = 0,
          TransDist_DE = 0;         //  distances used to translate edge BC and edge DE
    Point_T OffCurbPtA, OffCurbPtB; //  temp points used to calculate off-curb direction
    float TempDist, TempDist_BC, TempDist_DE;
    Relation_T relat_temp_dir;
    Relation_T PointDir_Mid, PointDir_CB, PointDir_ED, PointDir_DC, pointDirRearEdge,
        pointDirFrontEdge; // -1-left side, 1-right side
    float SlotLen_After;
    float ToCurbOffDist;
    float off_dist_CD = 1e6f; //  distance used to translate edge CD
    float off_dist_AB = -10.0f,
          off_dist_EF = -10.0f; //  distances used to translate edge AB and edge EF
    Vec2_T off_curb_vec;        //  off-curb vector
    LineSeg_T cur_segment, lineCD, ShrukenLineCD;
    int trans_flag_BC          = 0;
    float cur_seg_len          = 0;
    float max_curb_len         = 0;
    float max_curb_off_dist_CD = 1e6f, temp_off_dist;
    float TmpPtRy              = 0.0f;
    float TmpPtRx              = 0.0f;
    float distToD = 0.0f, distToC = 0.0f;

    memcpy(&OffCurbPtA, TargPos, sizeof(OffCurbPtA));

    OffCurbPtB.x = OffCurbPtA.x - Vert_to_Curb_Vec.vx;
    OffCurbPtB.y = OffCurbPtA.y - Vert_to_Curb_Vec.vy;

    memcpy(&SlotA, &SlotObj->ptA, sizeof(SlotA));
    memcpy(&SlotB, &SlotObj->ptB, sizeof(SlotB));
    memcpy(&SlotE, &SlotObj->ptE, sizeof(SlotE));
    memcpy(&SlotF, &SlotObj->ptF, sizeof(SlotF));

    float SlotVecLen = Cal_Dis_Pt2Pt(SlotB, SlotE); // BE Length
    tarPosition.x    = TargPos->x;
    tarPosition.y    = TargPos->y;

    if (SlotShap == PK_SLOT_LEFT_PARA)
    {
        Get_Dist_Dir_Pt2PointLine(SlotA, SlotB, tarPosition, &ToCurbOffDist, NULL);
        ToCurbOffDist += VEHICLE_WID / 2;
        isParaSlot = 1;

        off_curb_vec.vx = sinf(TargPos->theta);
        off_curb_vec.vy = -cosf(TargPos->theta);
    }
    else if (SlotShap == PK_SLOT_RIGHT_PARA)
    {
        Get_Dist_Dir_Pt2PointLine(SlotA, SlotB, tarPosition, &ToCurbOffDist, NULL);
        ToCurbOffDist += VEHICLE_WID / 2;
        isParaSlot = 1;

        off_curb_vec.vx = -sinf(TargPos->theta);
        off_curb_vec.vy = cosf(TargPos->theta);
    }
    else
    {
        off_curb_vec.vx = cosf(TargPos->theta);
        off_curb_vec.vy = sinf(TargPos->theta);
        ToCurbOffDist   = VEHICLE_LEN;
        isParaSlot      = 0;
    }

    // get slot C by translating the slot B a distance of vehicle width with the aim of
    // only need to move slot B in one direction. get slot D in the same way
    SlotC.x = SlotB.x + Vert_to_Curb_Vec.vx * ToCurbOffDist; //  C
    SlotC.y = SlotB.y + Vert_to_Curb_Vec.vy * ToCurbOffDist; //  C
    SlotD.x = SlotE.x + Vert_to_Curb_Vec.vx * ToCurbOffDist; //  D
    SlotD.y = SlotE.y + Vert_to_Curb_Vec.vy * ToCurbOffDist; //  D

    // cal rear points and front points to form CD displacement search region
    if (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA)
    {
        Cal_CD_Search_Region(*TargPos, SlotVec, SlotB, SlotC, SlotD, SlotE, &frontPt1,
                             &frontPt2, &rearPt1, &rearPt2);
    }

    // cal line CD for later use
    memcpy(&lineCD.pt1, &SlotC, sizeof(Point_T));
    memcpy(&lineCD.pt2, &SlotD, sizeof(Point_T));
    memcpy(&ShrukenLineCD.pt1, &rearPt1, sizeof(Point_T));
    memcpy(&ShrukenLineCD.pt2, &frontPt1, sizeof(Point_T));

    if (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_LEFT_VERT)
    {
        SlotDir = PK_ON_LEFT;
        // RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left(&g_FS_ObsInfo_A);
    }
    else if (SlotShap == PK_SLOT_RIGHT_PARA || SlotShap == PK_SLOT_RIGHT_VERT)
    {
        SlotDir = PK_ON_RIGHT;
        // RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right(&g_FS_ObsInfo_A);
    }
    StartIndex = SF_OBJ_NUM - g_FS_ObsInfo_A.num;
    for (i = StartIndex; i < SF_OBJ_NUM; i++)
    {
        trans_flag_BC = 0;

        insertFlag = 0;
        memcpy(&cur_segment, &g_FS_ObsInfo_A.obj[i], sizeof(cur_segment));
        cur_seg_len = Get_Segment_Len(cur_segment);
        CurLinePro  = (g_FS_ObsInfo_A.attr[i] / 1000) % 10;
        if (CurLinePro > 0 &&
            CurLinePro <=
                3) //&&
                   // RectClipSegment(BCDE,&g_FS_ObsInfo_A.attr[i][0],&g_FS_ObsInfo_A.attr[i][2],&ClipPt[0],
                   //&ClipPt[2]) == 1
        {
            for (j = 0; j < 2; j++)
            {
                if (j == 0)
                {
                    memcpy(&CurPt, &cur_segment.pt1, sizeof(CurPt));
                }
                else
                {
                    memcpy(&CurPt, &cur_segment.pt2, sizeof(CurPt));
                }
                Get_Dist_Dir_Pt2PointLine(SlotD, SlotC, CurPt, NULL, &PointDir_DC);
                Get_Dist_Dir_Pt2PointLine(SlotC, SlotB, CurPt, &TempDist_BC,
                                          &PointDir_CB);
                Get_Dist_Dir_Pt2PointLine(SlotE, SlotD, CurPt, &TempDist_DE,
                                          &PointDir_ED);

                if (isParaSlot == 1)
                {
                    Get_Dist_Dir_Pt2PointLine(rearPt1, rearPt2, CurPt, NULL,
                                              &pointDirRearEdge); // pseudo PointDir_CB
                    Get_Dist_Dir_Pt2PointLine(frontPt2, frontPt1, CurPt, NULL,
                                              &pointDirFrontEdge); // pseudo PointDir_ED
                }
                else
                {
                    // just copy PointDir_CB and PointDir_ED
                    memcpy(&pointDirRearEdge, &PointDir_CB, sizeof(Relation_T));
                    memcpy(&pointDirFrontEdge, &PointDir_ED, sizeof(Relation_T));
                }

                if (PointDir_DC == SlotDir) //  non-curb line segment
                {
                    if (IsSegmentInsertTargPos(cur_segment, *TargPos, SlotShap) ==
                        1) //  segment cross targpos
                    {
                        return 0;
                    }
                    Get_Dist_Dir_Pt2PointLine(OffCurbPtB, OffCurbPtA, CurPt, NULL,
                                              &PointDir_Mid);
                    if ((SlotDir == PointDir_CB) &&
                        (PointDir_Mid == SlotDir)) //  edge BC displacement
                    {
                        if (TempDist_BC > TransDist_BC)
                        {
                            TransDist_BC = TempDist_BC;
                        }
                        trans_flag_BC = 1;
                        insertFlag    = 1;
                    }

                    if ((SlotDir == PointDir_ED) &&
                        (PointDir_Mid != SlotDir)) //  edge DE displacement
                    {
                        if (TempDist_DE > TransDist_DE)
                        {
                            TransDist_DE = TempDist_DE;
                        }

                        insertFlag = 1;
                    }

                    // if two points have been check and insertFlag is still 0, then we
                    // need to check whether the line cross the entire BCDE region
                    if (j == 1 && insertFlag == 0)
                    {
                        if (LenTwoSegmentsOverlap(lineCD, cur_segment) > 1e-6f)
                        {
                            // current line crosses the entire region
                            insertFlag = 1;
                            return 0;
                        }
                    }
                }
                else //  the line segment is a curb
                {
                    if ((SlotDir == pointDirRearEdge) && (SlotDir == pointDirFrontEdge))
                    {
                        Get_Dist_Dir_Pt2PointLine(SlotC, SlotD, CurPt, &TempDist, NULL);
                        if (TempDist < off_dist_CD)
                        {
                            off_dist_CD = TempDist; // the distance from a line segment to
                                                    // edge CD. Note that edge CD here is
                                                    // a body line of the vehicle
                        }
                        temp_off_dist = Get_Segment_Len(cur_segment);
                        if (temp_off_dist >= max_curb_len)
                        {
                            max_curb_len = temp_off_dist; // the longest line segment and
                                                          // its distance to edge CD
                            if (TempDist < max_curb_off_dist_CD)
                            {
                                max_curb_off_dist_CD = TempDist;
                            }
                        }

                        insertFlag = 1;
                    }

                    // if two points have been checked and insertFlag is still 0, then we
                    // need to check whether the line cross the entire region
                    if (j == 1 && insertFlag == 0)
                    {
                        if (LenTwoSegmentsOverlap(ShrukenLineCD, cur_segment) > 1e-6f)
                        {
                            // current line crosses the entire region
                            insertFlag = 1;

                            // cal the shortest distance for CD displacement
                            Get_CrossPt_PointLine(SlotB, SlotC, cur_segment.pt1,
                                                  cur_segment.pt2, &crossPt1);
                            Get_CrossPt_PointLine(SlotE, SlotD, cur_segment.pt1,
                                                  cur_segment.pt2, &crossPt2);

                            distToC = Cal_Dis_Pt2Pt(crossPt1, SlotC);
                            distToD = Cal_Dis_Pt2Pt(crossPt2, SlotD);

                            if (distToC - distToD < 1e-6f)
                            {
                                TempDist = distToC;
                            }
                            else
                            {
                                TempDist = distToD;
                            }

                            // set the displacement
                            if (TempDist < off_dist_CD)
                            {
                                off_dist_CD =
                                    TempDist; // the distance from a line segment to edge
                                              // CD. Note that edge CD here is a body line
                                              // of the vehicle
                            }
                            temp_off_dist = Get_Segment_Len(cur_segment);
                            if (temp_off_dist >= max_curb_len)
                            {
                                max_curb_len =
                                    temp_off_dist; // the longest line segment and its
                                                   // distance to edge CD
                                if (TempDist < max_curb_off_dist_CD)
                                {
                                    max_curb_off_dist_CD = TempDist;
                                }
                            }
                        }
                    }
                }
            }

            if (trans_flag_BC == 1 &&
                cur_seg_len > AVM_MinSegLen_BCTrans) //  cur_seg_len is used for filtering
                                                     //  noise line segments
            {
                for (j = 0; j < 2; j++)
                {
                    if (j == 0)
                    {
                        memcpy(&CurPt, &cur_segment.pt1, sizeof(CurPt));
                    }
                    else
                    {
                        memcpy(&CurPt, &cur_segment.pt2, sizeof(CurPt));
                    }
                    //  edge AB displacement
                    Get_Dist_Dir_Pt2PointLine(SlotA, SlotB, CurPt, &TempDist,
                                              &relat_temp_dir);
                    if (relat_temp_dir == PK_ON_LEFT)
                    {
                        temp_dir = -1;
                    }
                    else if (relat_temp_dir == PK_ON_RIGHT)
                    {
                        temp_dir = 1;
                    }
                    TempDist = TempDist * temp_dir;
                    if (SlotDir == PK_ON_RIGHT)
                    {
                        TempDist = -TempDist;
                    }
                    if (TempDist > off_dist_AB)
                    {
                        off_dist_AB = TempDist;
                    }
                }
            }

            //// EF displacement is calculated below
            // if (trans_flag_DE == 1 && cur_seg_len > AVM_MinSegLen_DETrans) //
            // cur_seg_len is used for filtering noise line segments
            //{
            //   for (j = 0; j < 2; j++)
            //   {
            //       if (j == 0)
            //       {
            //           memcpy(&CurPt,&cur_segment.pt1,sizeof(CurPt));
            //       }
            //       else
            //       {
            //           memcpy(&CurPt,&cur_segment.pt2,sizeof(CurPt));
            //       }
            //       //  edge EF displacement
            //       Get_Dist_Dir_Pt2PointLine(SlotE,SlotF,CurPt,&TempDist,&relat_temp_dir);
            //       if (relat_temp_dir == PK_ON_LEFT)
            //       {
            //           temp_dir = -1;
            //       }
            //       else if (relat_temp_dir == PK_ON_RIGHT)
            //       {
            //           temp_dir = 1;
            //       }
            //       TempDist = TempDist*temp_dir;
            //       if (SlotDir == PK_ON_RIGHT) //  右侧车位
            //       {
            //           TempDist = -TempDist;
            //       }
            //       if (TempDist > off_dist_EF)
            //       {
            //           off_dist_EF = TempDist;
            //       }
            //   }
            // }
        }
    }

    SlotLen_After = SlotVecLen - TransDist_BC - TransDist_DE;
    if (((SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA) &&
         SlotLen_After < MIN_AVM_PARK_LEN_PARALLEL) ||
        ((SlotShap == PK_SLOT_LEFT_VERT || SlotShap == PK_SLOT_RIGHT_VERT) &&
         SlotLen_After < MIN_AVM_PARK_LEN_VERTICAL)) // MIN_PARK_LEN_VERTICAL
    {
        return 0;
    }

    if (off_dist_CD < AVM_CDOffsetLimit &&
        max_curb_off_dist_CD < AVM_CDOffsetLimit) // detect curb
    {
        SlotObjAttr[2] = 1;
        if ((SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA) &&
            max_curb_len > 3.0f) //  detect a continuous long curb
        {
            off_dist_CD = max_curb_off_dist_CD;
        }
        // else do not need to move CD
    }
    else //  not detect curb
    {
        if (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA)
        {
            // off_dist_CD = AVM_TarPos_CD_sidedist + AVM_Para_TargPos_off_dy;   //  CD
            // offset for a wider space
            memcpy(&SlotC, &SlotObj->ptC, sizeof(SlotC));
            memcpy(&SlotD, &SlotObj->ptD, sizeof(SlotD));
            off_dist_CD = 0; //  CD offset for a wider space
        }
        else
        {
            off_dist_CD = AVM_TarPos_CD_sidedist;
        }
    }

    //  Move CD
    SlotC.x -= off_curb_vec.vx * off_dist_CD;
    SlotC.y -= off_curb_vec.vy * off_dist_CD;
    SlotD.x -= off_curb_vec.vx * off_dist_CD;
    SlotD.y -= off_curb_vec.vy * off_dist_CD;

    // Move BC
    if (TransDist_BC > 0) //  Move A\B\C
    {
        SlotObjAttr[1] = 1;
        SlotA.x += SlotVec.vx * TransDist_BC;
        SlotA.y += SlotVec.vy * TransDist_BC;
        SlotB.x += SlotVec.vx * TransDist_BC;
        SlotB.y += SlotVec.vy * TransDist_BC;
        SlotC.x += SlotVec.vx * TransDist_BC;
        SlotC.y += SlotVec.vy * TransDist_BC;
    }

    // Move DE
    if (TransDist_DE > 0) //  Move D\E\F
    {
        SlotObjAttr[3] = 1;
        SlotD.x -= SlotVec.vx * TransDist_DE;
        SlotD.y -= SlotVec.vy * TransDist_DE;
        SlotE.x -= SlotVec.vx * TransDist_DE;
        SlotE.y -= SlotVec.vy * TransDist_DE;
        SlotF.x -= SlotVec.vx * TransDist_DE;
        SlotF.y -= SlotVec.vy * TransDist_DE;
    }
    //  Update slotobj for EF displacement calculation
    memcpy(&SlotObj->ptA, &SlotA, sizeof(SlotA));
    memcpy(&SlotObj->ptB, &SlotB, sizeof(SlotB));
    memcpy(&SlotObj->ptC, &SlotC, sizeof(SlotC));
    memcpy(&SlotObj->ptD, &SlotD, sizeof(SlotD));
    memcpy(&SlotObj->ptE, &SlotE, sizeof(SlotE));
    memcpy(&SlotObj->ptF, &SlotF, sizeof(SlotF));

    // Calculate EF displacement
    Cal_EF_Displacement(g_FS_ObsInfo_A, SlotDir, SlotObj, &off_dist_EF);

    if (off_dist_AB > AVM_ABOffsetLimit) //  if AB displacement is within a valid range
    {
        // TmpSlotPt used as translated SlotB
        TmpSlotPt.x = SlotB.x + off_curb_vec.vx * off_dist_AB;
        TmpSlotPt.y = SlotB.y + off_curb_vec.vy * off_dist_AB;

        // if it is a parallel slot
        if (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA)
        {
            // Project the tmp point to the y axis of the target pose
            TmpPtRy = Project_PointTo1stPos_ry(*TargPos, TmpSlotPt);

            // Only when the SlotB is NOT over translated will the translation takes
            // effect
            if ((SlotDir == PK_ON_RIGHT && TmpPtRy > 1e-6) ||
                (SlotDir == PK_ON_LEFT && TmpPtRy < 1e-6))
            {
                SlotObjAttr[0] = 1;
                SlotA.x += off_curb_vec.vx * off_dist_AB;
                SlotA.y += off_curb_vec.vy * off_dist_AB;

                // Avoid repeated computation. SlotB is already computed.
                memcpy(&SlotB, &TmpSlotPt, sizeof(Point_T));
            }
        } // if it is a vertical slot
        else if (SlotShap == PK_SLOT_RIGHT_VERT || SlotShap == PK_SLOT_LEFT_VERT)
        {
            // Project the tmp point to the x axis of the target pose
            TmpPtRx = Project_PointTo1stPos_rx(*TargPos, TmpSlotPt);

            // Only when the SlotB is NOT over translated will the translation takes
            // effect
            if (TmpPtRx > 1e-6)
            {
                SlotObjAttr[0] = 1;
                SlotA.x += off_curb_vec.vx * off_dist_AB;
                SlotA.y += off_curb_vec.vy * off_dist_AB;

                // Avoid repeated computation. SlotB is already computed.
                memcpy(&SlotB, &TmpSlotPt, sizeof(Point_T));
            }
        }
    }

    if (off_dist_EF > AVM_EFOffsetLimit) // if EF displacement is within a valid range
    {
        // TmpSlotPt used as translated SlotE
        TmpSlotPt.x = SlotE.x + off_curb_vec.vx * off_dist_EF;
        TmpSlotPt.y = SlotE.y + off_curb_vec.vy * off_dist_EF;

        // if it is a parallel slot
        if (SlotShap == PK_SLOT_RIGHT_PARA || SlotShap == PK_SLOT_LEFT_PARA)
        {
            // Project the tmp point to the y axis of the target pose
            TmpPtRy = Project_PointTo1stPos_ry(*TargPos, TmpSlotPt);

            // Only when the SlotB is NOT over translated will the translation takes
            // effect
            if ((SlotDir == PK_ON_RIGHT && TmpPtRy > 1e-6) ||
                (SlotDir == PK_ON_LEFT && TmpPtRy < 1e-6))
            {
                SlotObjAttr[4] = 1;

                // Avoid repeated computation. SlotE is already computed.
                memcpy(&SlotE, &TmpSlotPt, sizeof(Point_T));

                SlotF.x += off_curb_vec.vx * off_dist_EF;
                SlotF.y += off_curb_vec.vy * off_dist_EF;
            }
        } // if it is a vertical slot
        else if (SlotShap == PK_SLOT_RIGHT_VERT || SlotShap == PK_SLOT_LEFT_VERT)
        {
            TmpPtRx = Project_PointTo1stPos_rx(*TargPos, TmpSlotPt);

            if (TmpPtRx > 1e-6)
            {
                SlotObjAttr[4] = 1;

                // Avoid repeated computation. SlotE is already computed.
                memcpy(&SlotE, &TmpSlotPt, sizeof(Point_T));

                SlotF.x += off_curb_vec.vx * off_dist_EF;
                SlotF.y += off_curb_vec.vy * off_dist_EF;
            }
        }
    }

    //  Update slotobj
    memcpy(&SlotObj->ptA, &SlotA, sizeof(SlotA));
    memcpy(&SlotObj->ptB, &SlotB, sizeof(SlotB));
    memcpy(&SlotObj->ptC, &SlotC, sizeof(SlotC));
    memcpy(&SlotObj->ptD, &SlotD, sizeof(SlotD));
    memcpy(&SlotObj->ptE, &SlotE, sizeof(SlotE));
    memcpy(&SlotObj->ptF, &SlotF, sizeof(SlotF));

    // RTE_PK_SlotDetect_Set_SlotObj_Attr(SlotObjAttr);
    if (TransDist_BC > 0) //  BC is translated
    {
        Is_BC_Update_Bst = 1;
    }
    if (TransDist_DE > 0)
    {
        Is_DE_Update_Bst = 1;
    }
    return 1; // slot is updated
}

void Cal_EF_Displacement(FusionObj_T fusionObj, Relation_T slotDir, SlotObj_T *slotObj,
                         float *EFDisp)
{
    if (slotDir != PK_ON_LEFT && slotDir != PK_ON_RIGHT)
    {
        return;
    }

    int i, j, StartIndex, CurLinePro, tmpDir;
    float curSegLen = 0.0f, tmpDist = 0.0f, off_dist_EF = -1e6f, overlappLen;
    LineSeg_T curSegment, lineEF;
    Point_T curPt, slotC, slotD, slotE, slotF, slotE2, slotF2;
    Vec2_T VertToSlotVec;
    Relation_T ptDCDir, ptEEDir, ptFFDir, relat_temp_dir;

    // init
    memcpy(&slotC, &(slotObj->ptC), sizeof(Point_T));
    memcpy(&slotD, &(slotObj->ptD), sizeof(Point_T));
    memcpy(&slotE, &(slotObj->ptE), sizeof(Point_T));
    memcpy(&slotF, &(slotObj->ptF), sizeof(Point_T));

    VertToSlotVec.vx = slotE.x - slotD.x;
    VertToSlotVec.vy = slotE.y - slotD.y;
    Normalize_Vec2(&VertToSlotVec);

    slotE2.x = slotE.x + VertToSlotVec.vx;
    slotE2.y = slotE.y + VertToSlotVec.vy;
    slotF2.x = slotF.x + VertToSlotVec.vx;
    slotF2.y = slotF.y + VertToSlotVec.vy;

    // if (slotDir == PK_ON_LEFT)
    // {
    //  DCRefDir = PK_ON_LEFT;
    //  EERefDir = PK_ON_LEFT;
    //  FFRefDir = PK_ON_RIGHT;
    // }
    // else if(slotDir == PK_ON_RIGHT)
    // {
    //  DCRefDir = PK_ON_RIGHT;
    //  EERefDir = PK_ON_RIGHT;
    //  FFRefDir = PK_ON_LEFT;
    // }

    // loop through all line segments
    StartIndex = SF_OBJ_NUM - fusionObj.num;
    for (i = StartIndex; i < SF_OBJ_NUM; i++)
    {
        memcpy(&curSegment, &fusionObj.obj[i], sizeof(curSegment));
        curSegLen = Get_Segment_Len(curSegment);

        // if length of the line segment is too short, pass
        if (curSegLen <= AVM_MinSegLen_DETrans)
        {
            continue;
        }

        CurLinePro = (g_FS_ObsInfo_A.attr[i] / 1000) % 10;
        if (CurLinePro > 0 && CurLinePro <= 3)
        {
            memcpy(&curPt, &(curSegment.pt1), sizeof(Point_T));

            // check whether pt1 is within region
            Get_Dist_Dir_Pt2PointLine(slotD, slotC, curPt, NULL, &ptDCDir);
            Get_Dist_Dir_Pt2PointLine(slotE, slotE2, curPt, NULL, &ptEEDir);
            Get_Dist_Dir_Pt2PointLine(slotF, slotF2, curPt, NULL, &ptFFDir);

            memcpy(&lineEF.pt1, &slotE, sizeof(Point_T));
            memcpy(&lineEF.pt2, &slotF, sizeof(Point_T));

            overlappLen = LenTwoSegmentsOverlap(lineEF, curSegment);

            /*if (ptDCDir == DCRefDir && ptEEDir == EERefDir && ptFFDir == FFRefDir)*/
            if (overlappLen > AVM_MinOverlapLen)
            {
                // if pt1 is inside the region, calculate dist of pt1 and pt2 to EF no
                // matter whether pt2 is inside the region cal dist of pt1 and pt2 to EF
                for (j = 0; j < 2; ++j)
                {
                    if (j == 1)
                    {
                        memcpy(&curPt, &(curSegment.pt2), sizeof(Point_T));
                    }

                    Get_Dist_Dir_Pt2PointLine(slotE, slotF, curPt, &tmpDist,
                                              &relat_temp_dir);
                    if (relat_temp_dir == PK_ON_LEFT)
                    {
                        tmpDir = -1;
                    }
                    else if (relat_temp_dir == PK_ON_RIGHT)
                    {
                        tmpDir = 1;
                    }
                    tmpDist = tmpDist * tmpDir;

                    if (slotDir == PK_ON_RIGHT) //  右侧车位
                    {
                        tmpDist = -tmpDist;
                    }
                    if (tmpDist > off_dist_EF)
                    {
                        off_dist_EF = tmpDist;
                    }
                }
            }
        }
    }

    *EFDisp = off_dist_EF;
}

//  20180126: Update AVM target pose according to the slot object
int Update_AVM_Targpos_By_SlotObj(PK_SlotShapeType SlotShap, SlotObj_T SlotObj,
                                  VehPos_T *TargPos)
{
    float side_safe_dist = 0; //  safe distance of a line object to the vehicle body
    Point_T slot_B, slot_C, slot_D, slot_E, targ_postion;
    float free_dist_BC = 0, free_dist_DE = 0, slot_theta;
    Vec2_T slot_vec;
    float slot_vec_off_dist = 0; //  target pose displacement
    int Rx_update = 0, Ry_update = 0;
    Rect_T CornerPt;
    VehPos_T Targ_pos;
    float Dist_Targ_CD = 0, Adjust_Ry_Dist = 0;
    Relation_T CD_Dir;
    Vec2_T off_curb_vec;

    memcpy(&targ_postion, TargPos, sizeof(targ_postion));
    memcpy(&slot_B, &SlotObj.ptB, sizeof(slot_B));
    memcpy(&slot_C, &SlotObj.ptC, sizeof(slot_C));
    memcpy(&slot_D, &SlotObj.ptD, sizeof(slot_D));
    memcpy(&slot_E, &SlotObj.ptE, sizeof(slot_E));
    memcpy(&Targ_pos, TargPos, sizeof(Targ_pos));

    if (IsTargPosVehInSertSlot(*TargPos, SlotObj) == 1)
    {
        return 0;
    }

    if (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA)
    {
        side_safe_dist = 0.5f;
        Get_Dist_Dir_Pt2PointLine(slot_B, slot_C, targ_postion, &free_dist_BC, NULL);
        free_dist_BC = free_dist_BC - REAR_SUSPENSION;
        Get_Dist_Dir_Pt2PointLine(slot_D, slot_E, targ_postion, &free_dist_DE, NULL);
        free_dist_DE = free_dist_DE - (VEHICLE_LEN - REAR_SUSPENSION);
        slot_vec.vx  = cosf(TargPos->theta);
        slot_vec.vy  = sinf(TargPos->theta);
    }
    else
    {
        side_safe_dist = 0.4f;
        Get_Dist_Dir_Pt2PointLine(slot_B, slot_C, targ_postion, &free_dist_BC, NULL);
        free_dist_BC = free_dist_BC - VEHICLE_WID * 0.5f;
        Get_Dist_Dir_Pt2PointLine(slot_D, slot_E, targ_postion, &free_dist_DE, NULL);
        free_dist_DE = free_dist_DE - VEHICLE_WID * 0.5f;
        if (SlotShap == PK_SLOT_LEFT_VERT)
        {
            slot_theta = TargPos->theta + PI * 0.5f;
        }
        else
        {
            slot_theta = TargPos->theta - PI * 0.5f;
        }
        slot_vec.vx = cosf(slot_theta);
        slot_vec.vy = sinf(slot_theta);
    }

    if (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA)
    {
        Get_Veh_CornerPt(Targ_pos, &CornerPt, 0);
        if (SlotShap == PK_SLOT_LEFT_PARA)
        {
            Get_Dist_Dir_Pt2PointLine(CornerPt.pt_rl, CornerPt.pt_fl, slot_C,
                                      &Dist_Targ_CD, &CD_Dir);
            if (CD_Dir == PK_ON_LEFT)
            {
                if (Dist_Targ_CD <=
                    Para_VehEdge2CD_Min_Dist) // Para_TargPos2CD_Min_Dist = 0.2
                {
                    Adjust_Ry_Dist = Para_VehEdge2CD_Min_Dist - Dist_Targ_CD;
                    Ry_update      = 1;
                }
                else
                {
                    Ry_update = 0;
                }
            }
            else if (CD_Dir == PK_ON_RIGHT)
            {
                Adjust_Ry_Dist = Para_VehEdge2CD_Min_Dist + Dist_Targ_CD;
                Ry_update      = 1;
            }
        }
        else if (SlotShap == PK_SLOT_RIGHT_PARA)
        {
            Get_Dist_Dir_Pt2PointLine(CornerPt.pt_rr, CornerPt.pt_fr, slot_C,
                                      &Dist_Targ_CD, &CD_Dir);
            if (CD_Dir == PK_ON_RIGHT)
            {
                if (Dist_Targ_CD <=
                    Para_VehEdge2CD_Min_Dist) // Para_TargPos2CD_Min_Dist = 0.2
                {
                    Adjust_Ry_Dist = Para_VehEdge2CD_Min_Dist - Dist_Targ_CD;
                    Ry_update      = 1;
                }
                else
                {
                    Ry_update = 0;
                }
            }
            else if (CD_Dir == PK_ON_LEFT)
            {
                Adjust_Ry_Dist = Para_VehEdge2CD_Min_Dist + Dist_Targ_CD;
                Ry_update      = 1;
            }
        }
    }

    //  Calculate target pose displacement
    if (free_dist_BC + free_dist_DE < 2 * side_safe_dist) //  centered
    {
        slot_vec_off_dist = (free_dist_DE - free_dist_BC) * 0.5f;
        Rx_update         = 1;
    }
    else if (free_dist_BC < side_safe_dist)
    {
        slot_vec_off_dist = side_safe_dist - free_dist_BC;
        Rx_update         = 1;
    }
    else if (free_dist_DE < side_safe_dist)
    {
        slot_vec_off_dist = free_dist_DE - side_safe_dist;
        Rx_update         = 1;
    }

    //  Update target pose
    if (Rx_update == 1)
    {
        TargPos->x += slot_vec.vx * slot_vec_off_dist;
        TargPos->y += slot_vec.vy * slot_vec_off_dist;
    }

    // Ry_update = 1;  // not use this module untill bugs got fixed.
    if (Ry_update == 1)
    {
        if (SlotShap == PK_SLOT_LEFT_PARA)
        {
            off_curb_vec.vx = sinf(TargPos->theta);
            off_curb_vec.vy = -cosf(TargPos->theta);
        }
        else if (SlotShap == PK_SLOT_RIGHT_PARA)
        {
            off_curb_vec.vx = -sinf(TargPos->theta);
            off_curb_vec.vy = cosf(TargPos->theta);
        }
        TargPos->x += off_curb_vec.vx * Adjust_Ry_Dist;
        TargPos->y += off_curb_vec.vy * Adjust_Ry_Dist;
    }
    return 1;
}

// simple check if the AVM slot in buffer is too
int Check_If_AVM_Buff_Slot_Is_Too_Far(Point_T NearFrontPt)
{
    const float max_away_dist = 35.0f;
    float curpos[4], dist;
    Point_T VehPos;
    RTE_PK_Location_Get_CurPos(curpos);
    memcpy(&VehPos, curpos, sizeof(VehPos));
    dist = Cal_Dis_Pt2Pt(NearFrontPt, VehPos);
    if (dist > max_away_dist)
    {
        return 1;
    }
    return 0;
}

//  2017/02/09：complete parallel slot information according to slot corners and direction
//  detected by AVM 20171114:compute the slot object and target pose based on AVM parking
//  slot corners and free-space line segments Return：0-Error, 1-Success
int CalAVMSlotInfo(Point_T NearFrontPt, Point_T NearRearPt, Point_T FarFrontPt,
                   Point_T FarRearPt, PK_SlotShapeType SlotShap, SlotObj_T *SlotObj,
                   VehPos_T *TargPos, int SlotObjAttr[5])
{
    Vec2_T SlotVec, Vert_to_Curb_Vec; //  slot direction
    float SlotVecLen;                 //  slot length
    // float SlotTheta; //  vector pointing from NR to NF
    //  float NearCentPt[2];    //  center point between NR and NF
    float ObsVehLen, ObsVehDepth; //  displacements used to construct slot
    const float ToCurbSafeDist =
        0.5f; // OBJ底部CD距离车辆的安全距离 safe distance from the CD to a vehicle
    VehPos_T Temp_TargPos;
    SlotObj_T Temp_SlotObj; // Do not modify the input variable before getting the final
                            // result!!!
    const float AVMCalibErr_y = 0.0f; // 0.45f compensation of AVM calibration error
    int AVM_SlotUpdate        = 0;
    int Virtual_AVM_Slot      = 0;
    float Dy_fr, Dy_ff, Average_Length_To_FarPts;
    LineSeg_T NearLineSeg;
    VehPos_T NearVec;

    SlotVec.vx = NearFrontPt.x - NearRearPt.x;
    SlotVec.vy = NearFrontPt.y - NearRearPt.y;
    SlotVecLen = Get_Norm_Vec2(SlotVec);
    Normalize_Vec2(&SlotVec); // Normalization
    // SlotTheta = atan2f(SlotVec.vy, SlotVec.vx);

    // compensation for AVM error
    NearFrontPt.x += SlotVec.vx * AVMCalibErr_y;
    NearFrontPt.y += SlotVec.vy * AVMCalibErr_y;
    NearRearPt.x += SlotVec.vx * AVMCalibErr_y;
    NearRearPt.y += SlotVec.vy * AVMCalibErr_y;

    //  check whether the slot length is within a valid range
    if (((SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA) &&
         (SlotVecLen < MIN_AVM_PARK_LEN_PARALLEL ||
          SlotVecLen > MAX_AVM_PARK_LEN_PARALLEL)) ||
        ((SlotShap == PK_SLOT_LEFT_VERT || SlotShap == PK_SLOT_RIGHT_VERT) &&
         (SlotVecLen < MIN_AVM_PARK_LEN_VERTICAL ||
          SlotVecLen > MAX_AVM_PARK_LEN_VERTICAL))) //  视觉车位识别有误差
    {
        LOGW << "DROPPED current AVM slot due to small or large distance between NF and "
                "NR(para<4.6 or para>7.1,vertical<2.1 or vertical>3.7) actually:"
             << SlotVecLen;
        return 0;
    }

    NearLineSeg.pt1 = NearRearPt;
    NearLineSeg.pt2 = NearFrontPt;
    Convert_Line_to_Pos(NearLineSeg, &NearVec);
    Dy_fr                    = fabsf(Project_PointTo1stPos_ry(NearVec, FarRearPt));
    Dy_ff                    = fabsf(Project_PointTo1stPos_ry(NearVec, FarFrontPt));
    Average_Length_To_FarPts = (Dy_fr + Dy_ff) / 2.0f;

    if (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_LEFT_VERT)
    {
        Vert_to_Curb_Vec.vx = -SlotVec.vy;
        Vert_to_Curb_Vec.vy = SlotVec.vx;
    }
    else if (SlotShap == PK_SLOT_RIGHT_PARA || SlotShap == PK_SLOT_RIGHT_VERT)
    {
        Vert_to_Curb_Vec.vx = SlotVec.vy;
        Vert_to_Curb_Vec.vy = -SlotVec.vx;
    }

    //===============Calculate target pose===============//
    Virtual_AVM_Slot = Cal_AVM_TargPos_By_FusionObjA(
        NearFrontPt, NearRearPt, FarFrontPt, FarRearPt, SlotShap, SlotVec,
        Vert_to_Curb_Vec, &Temp_TargPos, SlotObjAttr);
    if (Virtual_AVM_Slot == 0)
    {
        LOGW << "DROPPED current AVM slot due to failue of target position "
                "calculation(may be interfere with obstacles)!";
        return 0;
    }

    if (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA)
    {
        ObsVehLen = VEHICLE_LEN * 0.5f;

        if (Average_Length_To_FarPts <= AVM_MinFarToNearDis)
        {
            ObsVehDepth = VEHICLE_WID +
                          ToCurbSafeDist; // AVM_Para_TargPos_off_dy = 0.15//1.89 + 0.5
        }
        else if (Average_Length_To_FarPts > AVM_MinFarToNearDis &&
                 Average_Length_To_FarPts <= AVM_MaxFarToNearDis)
        {
            ObsVehDepth = Average_Length_To_FarPts;
        }
        else
        {
            ObsVehDepth = AVM_MaxFarToNearDis;
        }
    }
    else if (SlotShap == PK_SLOT_LEFT_VERT || SlotShap == PK_SLOT_RIGHT_VERT)
    {
        ObsVehLen   = VEHICLE_WID;
        ObsVehDepth = VEHICLE_LEN + ToCurbSafeDist; // 4.7+0.5
    }
    else
    {
        ObsVehLen   = 0;
        ObsVehDepth = 0;
    }
    //==================construct a virtual slot object==================//
    Temp_SlotObj.ptB.x = NearRearPt.x;
    Temp_SlotObj.ptB.y = NearRearPt.y;
    Temp_SlotObj.ptA.x = Temp_SlotObj.ptB.x - ObsVehLen * SlotVec.vx;            //  A
    Temp_SlotObj.ptA.y = Temp_SlotObj.ptB.y - ObsVehLen * SlotVec.vy;            //  A
    Temp_SlotObj.ptC.x = Temp_SlotObj.ptB.x + ObsVehDepth * Vert_to_Curb_Vec.vx; //  C
    Temp_SlotObj.ptC.y = Temp_SlotObj.ptB.y + ObsVehDepth * Vert_to_Curb_Vec.vy; //  C
    Temp_SlotObj.ptE.x = NearFrontPt.x;
    Temp_SlotObj.ptE.y = NearFrontPt.y;
    Temp_SlotObj.ptF.x = Temp_SlotObj.ptE.x + ObsVehLen * SlotVec.vx;            //  F
    Temp_SlotObj.ptF.y = Temp_SlotObj.ptE.y + ObsVehLen * SlotVec.vy;            //  F
    Temp_SlotObj.ptD.x = Temp_SlotObj.ptE.x + ObsVehDepth * Vert_to_Curb_Vec.vx; //  D
    Temp_SlotObj.ptD.y = Temp_SlotObj.ptE.y + ObsVehDepth * Vert_to_Curb_Vec.vy; //  D

#if 0
    //=============Update slot object according to line object and target pose=================//
    AVM_SlotUpdate = Update_AVM_SlotObj_By_FusionObjA(SlotShap, &Temp_TargPos, SlotVec, Vert_to_Curb_Vec, &Temp_SlotObj, SlotObjAttr);
    if (AVM_SlotUpdate == 0)
    {
        LOGW << "Update_AVM_SlotObj_By_FusionObjA failed,maybe interfere with obstacles";
        return 0;
    }

    //=============Use RS Ultrasonic to check if Segments are insert too much in the BCDE ==============//
    AVM_SlotUpdate = Update_AVM_SlotObj_By_Rear_FusionObj_A(&Temp_SlotObj, &Temp_TargPos, SlotShap);
    if (AVM_SlotUpdate == -1)
    {
        LOGW << "Update_AVM_SlotObj_By_Rear_FusionObj_A failed,maybe interfere with obstacles";
        return 0;
    }

    //===========Use RS Ultrasonic to check if Overall Segments insert length is greater than threshold =======//
    AVM_SlotUpdate = Rear_Side_Radar_Check_Slot(Temp_TargPos, SlotShap);

    if (AVM_SlotUpdate == 1)
    {
        LOGW << "Update_AVM_SlotObj_By_Rear_FusionObj_A failed,maybe interfere with obstacles";
        return 0;
    }
#endif
    AVM_SlotUpdate = IsTargPosVehInSertSlot(Temp_TargPos, Temp_SlotObj);
    if (AVM_SlotUpdate == 0)
    {
        memcpy(SlotObj, &Temp_SlotObj, sizeof(Temp_SlotObj));
        memcpy(TargPos, &Temp_TargPos, sizeof(Temp_TargPos));
        return 1;
    }
    else
    {
        LOGW << "Update_AVM_Targpos_By_SlotObj failed!";
    }
    return 0;
}

/********************************************
function : InterPolationOneDim 一维插值算法
Param :     float * x - X 轴坐标
                    float * y   - Y 轴坐标
                    float xx - 输入的 X 轴坐标
                    int n - 坐标数据个数
Return : float - 对应输入X坐标的Y值
********************************************/
////float InterPolationOneDim(const float *x,const float *y, float xx,int n)
////{
////    // 推荐使用
////    int i,sign_x,out_intp=0;//sign_x=1 单调递增，sign_x=-1 单调递减；
////    float /*x0,y0,x1,y1,*/yy=0;
////    int l,h;
////    if (x[1]-x[0]>=0)
////    {
////        sign_x=1;
////        if (xx>=x[n-1])
////        {
////            out_intp=1;
////            yy=y[n-1];
////        }
////        else if(xx<=x[0])
////        {
////            out_intp=1;
////            yy=y[0];
////        }
////    }
////    else
////    {
////        sign_x=-1;
////        if (xx>=x[0])
////        {
////            out_intp=1;
////            yy=y[0];
////        }
////        else if(xx<=x[n-1])
////        {
////            out_intp=1;
////            yy=y[n-1];
////        }
////    }
////
////    if (out_intp==0)
////    {
////    #if 0
////        for (i=0;i<n-1;i++)
////        {
////            x0=x[i];
////            x1=x[i+1];
////            y0=y[i];
////            y1=y[i+1];
////            if (sign_x*xx>=sign_x*x[i] && sign_x*xx<=sign_x*x[i+1])
////            {
////                yy=y0+(xx-x0)*(y1-y0)/(x1-x0);
////                break;
////            }
////        }
////    #else
////        // 二分法搜索
////        l = 0;
////        h = n-1;
////        while(l<h)
////        {
////            i = (l+h)/2;
////            if(sign_x*xx>=sign_x*x[i] && sign_x*xx<=sign_x*x[i+1])
////            {
////                yy=y[i]+(xx-x[i])*(y[i+1]-y[i])/(x[i+1]-x[i]);
////                break;
////            }
////            else if(sign_x*xx>sign_x*x[i+1])
////            {
////                l = i + 1;
////            }
////            else
////            {
////                h = i ;
////            }
////        }
////    #endif
////    }
////    return yy;
////}

// float PK_Location_EPS_to_Rou(float EPS_angle,float tar_spd)
//{
//   float rou;
//   if (tar_spd>0)
//       rou=InterPolationOneDim(Car_TurningModel_fac_EPS_C,Car_TurningModel_Pfac_ROU_C,EPS_angle,sizeof(Car_TurningModel_fac_EPS_C)/sizeof(float));
//   else
//       rou=InterPolationOneDim(Car_TurningModel_fac_EPS_C,Car_TurningModel_Nfac_ROU_C,EPS_angle,sizeof(Car_TurningModel_fac_EPS_C)/sizeof(float));
//   if(fabsf(rou)<1e-6f)
//       rou=sign(rou)*(1e-6f);
//   return rou;
// }

/*********************************************************************
Function description :Pre_locat 根据当前位姿，车辆状态，计算dt之后的车辆位姿

Calibration state    : Yes -->> Need RT full OP experiment data,基于RT惯导的全工况实验
Edition              : 2016/7/30 1.0 基于里程计信息

Input list:
                 Cur_state: 当前车辆信息
 typedef struct
 {
 float curpoint[3];  //当前车辆位姿
 float ds_sum;
 float cur_eps;
 float cur_vel;
 int cur_tra_nth;
 }Cur_state_autodr;
                 dt：时间间隔
Output list:
                 finpoint:  dt之后的车辆位姿
                 return：B_state
**********************************************************************/
// finpoint下个状态里程及,finpoint下个状态里程及 包含4个元素 x,y，theta,PATH_S
// int PK_Pre_locat( float curpoint[4],float finpoint[4],float EPS_angle,float
// cur_vel,float dt)
//{
//    int   B_state=1;
//    float x_out,y_out,theta_out,s_out,x0,y0,theta0,s0;
//    float g,ds;   //g 为横摆角，ds为当前路程
//    float rou;
//   x0        =curpoint[0];   //m
//   y0        =curpoint[1];   //m
//   theta0    =curpoint[2];   //rad
//   s0        =curpoint[3];   //m
//   rou=PK_Location_EPS_to_Rou(EPS_angle,cur_vel);
//   ds=dt*cur_vel;
//   g=ds*rou;
//   x_out=cosf(g/2+theta0)*ds+x0;
//   y_out=sinf(g/2+theta0)*ds+y0;
//   theta_out=theta0+g;
//   s_out=s0+fabsf(ds);
//
//   finpoint[0]=x_out;
//   finpoint[1]=y_out;
//   finpoint[2]=theta_out;
//   finpoint[3]=s_out;
//   //finpoint[4]=cur_vel;      // m/s
//   return B_state;
// }

static void Loop_AVM_Slot_Array(SlotInfo_T *slotArray, SlotInfo_T newSlot,
                                VehPos_T vehCurPos)
{
    float maxDist  = 0;
    int maxDistIdx = -1;
    for (int i = 0; i < AVM_SLOT_NUM_MAX; i++)
    {
        float dist = Cal_Dis_Pt2Pt((float *)&slotArray[i].targpos, (float *)&vehCurPos);
        if (dist > maxDist)
        {
            maxDist    = dist;
            maxDistIdx = i;
        }
    }

    float dist = Cal_Dis_Pt2Pt((float *)&newSlot.targpos, (float *)&vehCurPos);
    if (dist < maxDist && maxDistIdx >= 0)
    {
        memcpy(&slotArray[maxDistIdx], &newSlot, sizeof(SlotInfo_T));
    }
}

static bool TargetInsideAvm(const Point_T &point, const Point_T quad[4])
{
    int n       = 4;
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++)
    {
        const Point_T &p1 = quad[i];
        const Point_T &p2 = quad[j];
        if (((p1.y > point.y) != (p2.y > point.y)) &&
            (point.x < (p2.x - p1.x) * (point.y - p1.y) / (p2.y - p1.y) + p1.x))
        {
            inside = !inside;
        }
    }
    return inside;
}

void Cal_AVM_Slot_Batch(AVM_Buff_Info *buff, VehPos_T VehCurPos)
{
    int SlotObjAttr[5];
    Point_T CurNearFrontPt, CurNearRearPt, CurFarFrontPt, CurFarRearPt, CurStopBar;
    VehPos_T Temp_AVM_TargPos;
    SlotObj_T Temp_AVM_SlotObj; //
    SlotInfo_T newSlot;
    //  calculate all the slot info in the AVM buffer
    for (int i = 0; i < buff->Num; i++)
    {
        memcpy(&CurNearFrontPt, &buff->NF[i], sizeof(CurNearFrontPt));
        memcpy(&CurNearRearPt, &buff->NR[i], sizeof(CurNearRearPt));
        memcpy(&CurFarFrontPt, &buff->FF[i], sizeof(CurFarFrontPt));
        memcpy(&CurFarRearPt, &buff->FR[i], sizeof(CurFarRearPt));
        memcpy(&CurStopBar, &buff->BAR[i], sizeof(CurFarRearPt));

        int has_stopper                = buff->has_stopper[i];
        int has_lock                   = buff->has_lock[i];
        PK_SlotShapeType SlotShap_Temp = buff->SlotShap[i];
        if (has_lock)
        {
            LOGW << "Slot " << i << "Type " << SlotShap_Temp
                 << " has lock inside, ignore it! " << has_lock;
            if (Avm_CheckStop)
            {
                continue;
            }
        }

        int AVM_SlotIndex = buff->AVM_slot_index[i];
        float rx          = Project_PointTo1stPos_rx(VehCurPos, CurNearFrontPt);
        if (rx < AVM_MaxProjRx_SlotCal &&
            rx > AVM_MinProjRx_SlotCal) //  ensure the side front radar has scaned the
                                        //  vision slot area, updated on 20180822
        {
            // filter wrong side fusion obj
            if (SlotShap_Temp == PK_SLOT_RIGHT_VERT ||
                SlotShap_Temp == PK_SLOT_RIGHT_PARA)
            {
                RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right_Raw(&g_FS_ObsInfo_A);
                Remove_Wrong_Dir_FusionObj(&g_FS_ObsInfo_A, CurNearFrontPt,
                                           CurNearRearPt);
            }
            else if (SlotShap_Temp == PK_SLOT_LEFT_VERT ||
                     SlotShap_Temp == PK_SLOT_LEFT_PARA)
            {

                RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left_Raw(&g_FS_ObsInfo_A);
                Remove_Wrong_Dir_FusionObj(&g_FS_ObsInfo_A, CurNearFrontPt,
                                           CurNearRearPt);
            }
            else
            {
                LOGW << "------------buff->AVM_slot_index:" << buff->AVM_slot_index[i]
                     << "------------";
                LOGW << "NF_x:" << (buff->NF[i]).x << "NF_y:" << (buff->NF[i]).y
                     << " FF_x: " << (buff->FF[i]).x << "FF_x:" << (buff->FF[i]).y;
                LOGW << "NR_x:" << (buff->NR[i]).x << "NR_y:" << (buff->NR[i]).y
                     << " FR_x: " << (buff->FR[i]).x << "FR_x:" << (buff->FR[i]).y;
                LOGW << "DROPPED current AVM slot because it's not regular slot";
            }

            int SlotCal_rt = CalAVMSlotInfo(
                CurNearFrontPt, CurNearRearPt, CurFarFrontPt, CurFarRearPt, SlotShap_Temp,
                &Temp_AVM_SlotObj, &Temp_AVM_TargPos, SlotObjAttr);
            if (SlotCal_rt) // Temp_AVM_SlotObj  no obstacles are within the slot and the
                            // size of slot is appropriate
            {
                Point_T center, quad[4];
                memcpy((float *)&center, (float *)&Temp_AVM_TargPos, sizeof(Point_T));
                memcpy((float *)&quad[0], (float *)&CurNearRearPt, sizeof(Point_T));
                memcpy((float *)&quad[1], (float *)&CurFarRearPt, sizeof(Point_T));
                memcpy((float *)&quad[2], (float *)&CurFarFrontPt, sizeof(Point_T));
                memcpy((float *)&quad[3], (float *)&CurNearFrontPt, sizeof(Point_T));
                if (!TargetInsideAvm(center, quad))
                {
                    printf(
                        "avmdata error1! bcde:%04f,%04f,%04f,%04f,%04f,%04f,%04f,%04f "
                        "target:%04f,%04f\n",
                        quad[0].x, quad[0].y, quad[1].x, quad[1].y, quad[2].x, quad[2].y,
                        quad[3].x, quad[3].y, center.x, center.y);
                    continue;
                }

                center.x = (CurNearRearPt.x + CurFarRearPt.x + CurFarFrontPt.x +
                            CurNearFrontPt.x) *
                           0.25;
                center.y = (CurNearRearPt.y + CurFarRearPt.y + CurFarFrontPt.y +
                            CurNearFrontPt.y) *
                           0.25;
                float dist0 = Cal_Dis_Pt2Pt((float *)&center, (float *)&Temp_AVM_TargPos);
                if (dist0 > 3.0f)
                {
                    continue;
                }

                // 对角距离
                float dist1 = Cal_Dis_Pt2Pt(CurNearRearPt, CurFarFrontPt);
                float dist2 = Cal_Dis_Pt2Pt(CurNearFrontPt, CurFarRearPt);
                if (dist1 > 8.0f || dist2 > 8.0f || dist1 < 5.0f || dist2 < 5.0f)
                {
                    printf(
                        "avmdata error2! bcde:%04f,%04f,%04f,%04f,%04f,%04f,%04f,%04f "
                        "target:%04f,%04f\n",
                        quad[0].x, quad[0].y, quad[1].x, quad[1].y, quad[2].x, quad[2].y,
                        quad[3].x, quad[3].y, center.x, center.y);
                    continue;
                }

                memcpy(&newSlot.slotobj, &Temp_AVM_SlotObj, sizeof(Temp_AVM_SlotObj));
                memcpy(&newSlot.targpos, &Temp_AVM_TargPos, sizeof(Temp_AVM_TargPos));
                memcpy(newSlot.slotobj_attr, SlotObjAttr, sizeof(SlotObjAttr));

                newSlot.slotshap             = SlotShap_Temp;
                newSlot.is_vision_slot       = 1;
                newSlot.avm_point.slot_index = AVM_SlotIndex; //  记录原始视觉车位数据
                memcpy((float *)&newSlot.avm_point.near_front, (float *)&CurNearFrontPt,
                       sizeof(Point_T));
                memcpy((float *)&newSlot.avm_point.near_rear, (float *)&CurNearRearPt,
                       sizeof(Point_T));
                memcpy((float *)&newSlot.avm_point.far_front, (float *)&CurFarFrontPt,
                       sizeof(Point_T));
                memcpy((float *)&newSlot.avm_point.far_rear, (float *)&CurFarRearPt,
                       sizeof(Point_T));
                newSlot.avm_point.side_dist = buff->Side_Mirr_Dist[i];
                newSlot.has_stopper         = has_stopper;
                newSlot.occupied            = has_lock;

// 模拟版本没有恢复障碍物
#if defined(SIMULATE)
                has_stopper = 0;
#endif

                if (has_stopper > 0)
                {
                    float rsTarget[2], target[2];
                    float rx    = Project_PointTo1stPos_rx(newSlot.targpos, CurStopBar);
                    rsTarget[0] = rx;
                    rsTarget[1] = 0.0f;
                    Convert((float *)&newSlot.targpos, rsTarget, target);

                    if (fabs(rx) > 1.0f)
                    {
                        // printf("slot stopbar error! rx: %04f, target:%04f,%04f new:%04f
                        // %04f\n",
                        //         rx, newSlot.targpos.x, newSlot.targpos.y, target[0],
                        //         target[1]);
                    }
                    else
                    {
                        newSlot.targpos.x = target[0];
                        newSlot.targpos.y = target[1];
                    }
                }

                if (SlotShap_Temp == PK_SLOT_LEFT_PARA ||
                    SlotShap_Temp == PK_SLOT_LEFT_VERT)
                {
                    if (g_Left_AVM_Slot_Num < AVM_SLOT_NUM_MAX)
                    {
                        memcpy(&g_Left_AVM_Slot_Array[g_Left_AVM_Slot_Num], &newSlot,
                               sizeof(newSlot));
                        g_Left_AVM_Slot_Num++;
                    }
                    else
                    {
                        Loop_AVM_Slot_Array(g_Left_AVM_Slot_Array, newSlot, VehCurPos);
                    }
                }
                else if (SlotShap_Temp == PK_SLOT_RIGHT_PARA ||
                         SlotShap_Temp == PK_SLOT_RIGHT_VERT)
                {
                    if (g_Right_AVM_Slot_Num < AVM_SLOT_NUM_MAX)
                    {
                        memcpy(&g_Right_AVM_Slot_Array[g_Right_AVM_Slot_Num], &newSlot,
                               sizeof(newSlot));
                        g_Right_AVM_Slot_Num++;
                    }
                    else
                    {
                        Loop_AVM_Slot_Array(g_Right_AVM_Slot_Array, newSlot, VehCurPos);
                    }
                }
            }
        }
        else
        {
            LOGD << "------------buff->AVM_slot_index:" << buff->AVM_slot_index[i]
                 << "------------";
            LOGD << "NF_x:" << (buff->NF[i]).x << "NF_y:" << (buff->NF[i]).y
                 << " FF_x: " << (buff->FF[i]).x << "FF_x:" << (buff->FF[i]).y;
            LOGD << "NR_x:" << (buff->NR[i]).x << "NR_y:" << (buff->NR[i]).y
                 << " FR_x: " << (buff->FR[i]).x << "FR_x:" << (buff->FR[i]).y;
            LOGD << "DROPPED current AVM because vehicle is too near(<1.5) or too "
                    "far(>25) from NR!";
        }
    }
}

int Remove_Wrong_Dir_FusionObj(FusionObj_T *fusionObj, Point_T curNearFrontPt,
                               Point_T curNearRearPt)
{
    Vec2_T refVec, objVec;
    FusionObj_T tmpFusionObj;
    int i, j, startIndex;
    int tmpFusionObjNum = 0;

    // init
    memset(&tmpFusionObj, 0, sizeof(FusionObj_T));
    refVec.vx  = curNearFrontPt.x - curNearRearPt.x;
    refVec.vy  = curNearFrontPt.y - curNearRearPt.y;
    startIndex = SF_OBJ_NUM - fusionObj->num;

    for (i = startIndex; i < SF_OBJ_NUM; ++i)
    {
        // cal line segment vector
        objVec.vx = fusionObj->obj[i].pt2.x - fusionObj->obj[i].pt1.x;
        objVec.vy = fusionObj->obj[i].pt2.y - fusionObj->obj[i].pt1.y;

        if (Get_Dot_Vec2(refVec, objVec) > 1e-8)
        {
            // memory shift
            for (j = 1; j < SF_OBJ_NUM; ++j)
            {
                memcpy(&(tmpFusionObj.obj[j - 1]), &(tmpFusionObj.obj[j]),
                       sizeof(LineSeg_T));
                memcpy(&(tmpFusionObj.ang[j - 1]), &(tmpFusionObj.ang[j]), sizeof(float));
                memcpy(&(tmpFusionObj.attr[j - 1]), &(tmpFusionObj.attr[j]), sizeof(int));
            }

            memcpy(&(tmpFusionObj.obj[SF_OBJ_NUM - 1]), &(fusionObj->obj[i]),
                   sizeof(LineSeg_T));
            tmpFusionObj.ang[SF_OBJ_NUM - 1]  = fusionObj->ang[i];
            tmpFusionObj.attr[SF_OBJ_NUM - 1] = fusionObj->attr[i];

            ++tmpFusionObjNum;
        }
    }

    tmpFusionObj.num = tmpFusionObjNum;
    memcpy(fusionObj, &tmpFusionObj, sizeof(FusionObj_T));

    return 1;
}

// cal search region for CD edge offset calculation. Notice that this function is only
// used for parallel slot
void Cal_CD_Search_Region(VehPos_T tarPose, Vec2_T slotVec, Point_T slotB, Point_T slotC,
                          Point_T slotD, Point_T slotE, Point_T *frontPt1,
                          Point_T *frontPt2, Point_T *rearPt1, Point_T *rearPt2)
{
    float distToBC = 0.0f, distToDE = 0.0f, TempOffset = 0.0f;
    Point_T tarPosition;

    // init
    tarPosition.x = tarPose.x;
    tarPosition.y = tarPose.y;

    // cal distance from vehicle center to BC/DE edge
    Get_Dist_Dir_Pt2PointLine(slotB, slotC, tarPosition, &distToBC, NULL);
    Get_Dist_Dir_Pt2PointLine(slotD, slotE, tarPosition, &distToDE, NULL);

    TempOffset = distToBC - REAR_SUSPENSION - Para_CD_Search_Region_Offset;

    rearPt1->x = slotC.x + slotVec.vx * TempOffset;
    rearPt1->y = slotC.y + slotVec.vy * TempOffset;
    rearPt2->x = slotB.x + slotVec.vx * TempOffset;
    rearPt2->y = slotB.y + slotVec.vy * TempOffset;

    TempOffset =
        distToDE - (VEHICLE_LEN - REAR_SUSPENSION) - Para_CD_Search_Region_Offset;

    frontPt1->x = slotD.x - slotVec.vx * TempOffset;
    frontPt1->y = slotD.y - slotVec.vy * TempOffset;
    frontPt2->x = slotE.x - slotVec.vx * TempOffset;
    frontPt2->y = slotE.y - slotVec.vy * TempOffset;
}

RECAL_DIR AVM_SlotDetect()
{
    float CurPos[4] = {0}; //  x,y,theta,s
    VehPos_T VehCurPos;

    int VehMove_Flag  = 0;
    int L_Buff_Update = 0, R_Buff_Update = 0;
    RECAL_DIR Cal_dir = CAL_NO;
    int cal_num       = 0;

    RTE_PK_Location_Get_CurPos(CurPos);
    memcpy(&VehCurPos, CurPos, sizeof(VehCurPos));
    RTE_PK_SF_Get_AVM_Buff_Left(&g_Left_BuffInfo);
    RTE_PK_SF_Get_AVM_Buff_Right(&g_Right_BuffInfo);
    L_Buff_Num = g_Left_BuffInfo.AccNum;
    R_Buff_Num = g_Right_BuffInfo.AccNum;

    static int count = 0;
    if (fabsf(CurPos[3] - Last_VehPath) > 0.5f || (count++ % 10) == 0)
    {
        Last_VehPath = CurPos[3];
        VehMove_Flag = 1;
    }
    if (L_Buff_Num != L_Buff_Num_last)
    {
        L_Buff_Num_last = L_Buff_Num;
        L_Buff_Update   = 1;
    }
    if (R_Buff_Num != R_Buff_Num_last)
    {
        R_Buff_Num_last = R_Buff_Num;
        R_Buff_Update   = 1;
    }

    // CAL left slot
    if (L_Buff_Update == 1 ||
        (L_Buff_Num > 0 &&
         VehMove_Flag ==
             1)) //  find new visual slot or the vehicle moves a certain distance
    {
        // initialization
        g_Left_AVM_Slot_Num = 0;
        memset(&g_Left_AVM_Slot_Array, 0, sizeof(g_Left_AVM_Slot_Array));
        //  calculate all the slot info in the AVM buffer
        Cal_AVM_Slot_Batch(&g_Left_BuffInfo, VehCurPos);
        Cal_dir = CAL_LEFT;
        cal_num++;
    }

    // CAL right slot
    if (R_Buff_Update == 1 ||
        (R_Buff_Num > 0 &&
         VehMove_Flag ==
             1)) //  find new visual slot or the vehicle moves a certain distance
    {
        // initialization
        g_Right_AVM_Slot_Num = 0;
        memset(&g_Right_AVM_Slot_Array, 0, sizeof(g_Right_AVM_Slot_Array));
        //  calculate all the slot info in the AVM buffer
        Cal_AVM_Slot_Batch(&g_Right_BuffInfo, VehCurPos);
        Cal_dir = CAL_RIGHT;
        cal_num++;
    }
    if (cal_num == 2)
    {
        Cal_dir = CAL_BOTH;
    }
    return Cal_dir;
}
