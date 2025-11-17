#include "PK_PathPlanP.h"
#include "PathPlan_Update.h"
#include "PathPlan_Debug.h"
#include "PathPlan_Func.h"
#include "PathPlan_Config.h"
#include "PathPlan_Obj.h"
#include "PK_ObjAvoid.h"
#include "PK_Calibration.h"
#include "MathFunc.h"
#include "PK_Utility.h"
#include "PK_PathExecute.h"
#include "PathPlan_BasePlan.h"
#include "Record_Log.h"
#include "PK_StateManage.h"
#include "PathPlan_Tools.h"
#include "PathPlan_Config.h"

int PathPlan_RangeDataProcess(PlanDataCase PlanData_Ver[1], const SlotInfo_T &slot,
                              const int FS_ObjDir[FS_OBJ_ARR_NUM],
                              const float FS_Obj[FS_OBJ_ARR_NUM][4],
                              const int FS_ObjDir_Rear[FS_OBJ_ARR_NUM],
                              const float FS_Obj_Rear[FS_OBJ_ARR_NUM][4]);

/*
功能描述：检查超声波（空间）车位，调整车位入口位置
函数名称：Check_SlotObj_Is_Not_VisonSlot
INTPUT:     车位slot，规划起点stpoint, 车位入口移动距离move_ds,
前长距超声波障碍物类型FS_ObjDir，前长距超声波障碍物FS_Obj，
       后长距超声波障碍物类型FS_ObjDir_Rear，前长距超声波障碍物FS_Obj_Rear
OUTPUT:     车位slot，前长距超声波障碍物类型FS_ObjDir，前长距超声波障碍物FS_Obj，
       后长距超声波障碍物类型FS_ObjDir_Rear，前长距超声波障碍物FS_Obj_Rear
*/
static int Check_SlotObj_Is_Not_VisonSlot(SlotInfo_T &slot, float stpoint[3],
                                          float move_ds, int FS_ObjDir[FS_OBJ_ARR_NUM],
                                          float FS_Obj[FS_OBJ_ARR_NUM][4],
                                          int FS_ObjDir_Rear[FS_OBJ_ARR_NUM],
                                          float FS_Obj_Rear[FS_OBJ_ARR_NUM][4])
{
    float finpoint[3]; /*, line_AB[4], line_EF[4]*/
    float slot_wid_min = 2.8;
    int offset         = (PathPlan_SlotDir(slot.slotshap) == 2) ? SF_OBJ_NUM : 0;
    int rightFlag      = (PathPlan_SlotDir(slot.slotshap) == 2) ? 1 : -1;

    memcpy(finpoint, &slot.targpos, sizeof(finpoint));
    if (slot.slotshap <= PK_SLOT_RIGHT_PARA)
    {
        finpoint[2]  = Round_PI(finpoint[2] + rightFlag * PI_2);
        slot_wid_min = 6.5;
    }
    else
    {
        finpoint[2]  = Round_PI(finpoint[2]);
        slot_wid_min = 2.8;
    }

    float rspoint[3];
    RevConvert(finpoint, (float *)&slot.slotobj.ptE, rspoint);
    // rspoint[0] = sign(rspoint[0]) * (fabs(rspoint[0]) + 0.1);
    rspoint[1] = sign(rspoint[1]) * (fabs(rspoint[1]) - move_ds);
    Convert(finpoint, rspoint, (float *)&slot.slotobj.ptE);

    RevConvert(finpoint, (float *)&slot.slotobj.ptD, rspoint);
    rspoint[1] = sign(rspoint[1]) * (fabs(rspoint[1]) - move_ds);
    Convert(finpoint, rspoint, (float *)&slot.slotobj.ptD);

    RevConvert(finpoint, (float *)&slot.slotobj.ptB, rspoint);
    // rspoint[0] = sign(rspoint[0]) * (fabs(rspoint[0]) + 0.1);
    rspoint[1] = sign(rspoint[1]) * (fabs(rspoint[1]) + move_ds + 0.1);
    Convert(finpoint, rspoint, (float *)&slot.slotobj.ptB);

    RevConvert(finpoint, (float *)&slot.slotobj.ptC, rspoint);
    rspoint[1] = sign(rspoint[1]) * (fabs(rspoint[1]) + move_ds + 0.1);
    Convert(finpoint, rspoint, (float *)&slot.slotobj.ptC);

    RevConvert(stpoint, (float *)&slot.targpos, rspoint);
    rspoint[0] = rspoint[0] - move_ds;
    Convert(stpoint, rspoint, (float *)&slot.targpos);

    float slot_wid = fabs(Cal_Dis_Pt2Pt(slot.slotobj.ptB, slot.slotobj.ptE));

    if (slot_wid <= slot_wid_min)
    {
        float expand_ds = (slot_wid_min - slot_wid) / 2;

        RevConvert(finpoint, (float *)&slot.slotobj.ptE, rspoint);
        rspoint[1] = sign(rspoint[1]) * (fabs(rspoint[1]) + expand_ds);
        Convert(finpoint, rspoint, (float *)&slot.slotobj.ptE);

        RevConvert(finpoint, (float *)&slot.slotobj.ptD, rspoint);
        rspoint[1] = sign(rspoint[1]) * (fabs(rspoint[1]) + expand_ds);
        Convert(finpoint, rspoint, (float *)&slot.slotobj.ptD);

        RevConvert(finpoint, (float *)&slot.slotobj.ptB, rspoint);
        rspoint[1] = sign(rspoint[1]) * (fabs(rspoint[1]) + expand_ds);
        Convert(finpoint, rspoint, (float *)&slot.slotobj.ptB);

        RevConvert(finpoint, (float *)&slot.slotobj.ptC, rspoint);
        rspoint[1] = sign(rspoint[1]) * (fabs(rspoint[1]) + expand_ds);
        Convert(finpoint, rspoint, (float *)&slot.slotobj.ptC);
    }

    for (int j = 0; j < 2; j++)
    {
        for (int i = 0; i < SF_OBJ_NUM; i++)
        {
            int objDir = 0;
            LineSeg_T seg;

            if (j == 0)
            {
                objDir = FS_ObjDir[i + offset];
                memcpy(&seg, &FS_Obj[i + offset], sizeof(seg));
            }
            else
            {
                objDir = FS_ObjDir_Rear[i + offset];
                memcpy(&seg, &FS_Obj_Rear[i + offset], sizeof(seg));
            }

            if (objDir == 0)
            {
                break;
            }

            if (Get_Segment_Len(seg) < PathPlanConstCfg::valid_obj_len)
            {
                continue;
            } // HAVECHANGE Tangsj 2023/6/26

            int result = PATHPLAN_SUCCESS;
            result =
                Is_TwoSegment_Cross(slot.slotobj.ptB, slot.slotobj.ptC, seg.pt1, seg.pt2);
            if (!result)
            {
                continue;
            }
            if (slot.slotshap <= PK_SLOT_RIGHT_PARA)
            {
                break;
            }
            else
            {
                if (j == 0)
                {
                    if (FS_Obj[i + offset][0] > FS_Obj[i + offset][2])
                    {
                        FS_Obj[i + offset][0] -= (move_ds + 0.2);
                    }
                    else
                    {
                        FS_Obj[i + offset][2] -= (move_ds + 0.2);
                    }
                }
                else
                {
                    if (FS_Obj_Rear[i + offset][0] > FS_Obj[i + offset][2])
                    {
                        FS_Obj_Rear[i + offset][0] -= (move_ds + 0.2);
                    }
                    else
                    {
                        FS_Obj_Rear[i + offset][2] -= (move_ds + 0.2);
                    }
                }
            }
        }
    }

    return 1;
}

// 补全单边界超声波空间车位数据，使其符合规划所需数据要求
static void Check_SlotObj_Is_Single_Boundary(SlotInfo_T &slot)
{
    float distA, distB, distE, distF, distAB, distEF;
    float rspoint[3], finpoint[3];

    int rightFlag = (PathPlan_SlotDir(slot.slotshap) == 2) ? 1 : -1;

    memcpy(finpoint, &slot.targpos, sizeof(finpoint));
    if (slot.slotshap <= PK_SLOT_RIGHT_PARA)
    {
        finpoint[2] = Round_PI(finpoint[2] + rightFlag * PI_2);
    }
    else
    {
        finpoint[2] = Round_PI(finpoint[2]);
    }

    distA = Project_PosTo1st_ry(finpoint, &slot.slotobj.ptA.x);
    distB = Project_PosTo1st_ry(finpoint, &slot.slotobj.ptB.x);
    distE = Project_PosTo1st_ry(finpoint, &slot.slotobj.ptE.x);
    distF = Project_PosTo1st_ry(finpoint, &slot.slotobj.ptF.x);

    distAB = fabsf(distA - distB);
    distEF = fabsf(distF - distE);

    if (distAB < 0.5f)
    {
        RevConvert(finpoint, (float *)&slot.slotobj.ptB, rspoint);
        // rspoint[0] = sign(rspoint[0]) * (fabs(rspoint[0]) + 0.1);
        rspoint[1] = sign(rspoint[1]) * (fabs(rspoint[1]) + 1.0);
        Convert(finpoint, rspoint, (float *)&slot.slotobj.ptA);

        slot.is_vision_slot = 1;
        slot.has_stopper    = 0;
    }

    if (distEF < 0.5f)
    {
        RevConvert(finpoint, (float *)&slot.slotobj.ptE, rspoint);
        // rspoint[0] = sign(rspoint[0]) * (fabs(rspoint[0]) + 0.1);
        rspoint[1] = sign(rspoint[1]) * (fabs(rspoint[1]) + 1.0);
        Convert(finpoint, rspoint, (float *)&slot.slotobj.ptF);

        slot.is_vision_slot = 1;
        slot.has_stopper    = 0;
    }

    if (slot.is_vision_slot == 1)
    {
        slot.has_stopper = 0;
        memcpy(&slot.avm_point.near_rear, &slot.slotobj.ptB, 2 * sizeof(float));
        memcpy(&slot.avm_point.far_rear, &slot.slotobj.ptC, 2 * sizeof(float));
        memcpy(&slot.avm_point.far_front, &slot.slotobj.ptD, 2 * sizeof(float));
        memcpy(&slot.avm_point.near_front, &slot.slotobj.ptE, 2 * sizeof(float));
    }
}

static void AddDynObjForPreSlot(const Multi_Slot_Array_T &slots, SlotInfo_T &slot,
                                const float stpoint[3], int FS_ObjDir[FS_OBJ_ARR_NUM],
                                float FS_Obj[FS_OBJ_ARR_NUM][4])
{
    // 提前释放，人为添加障碍物（不得已方案，待感知完善后去除）
    if (slot.slotshap < PK_SLOT_LEFT_VERT)
    {
        return;
    }

    int offset = (PathPlan_SlotDir(slot.slotshap) == 2) ? SF_OBJ_NUM : 0;
    int index  = 0;
    while (index < SF_OBJ_NUM)
    {
        if (FS_ObjDir[index + offset] == 0)
        {
            break;
        }
        index++;
    }

    if (index >= SF_OBJ_NUM)
    {
        return;
    }

    float nextTargRs[2], curTargetRs[2];
    RevConvert(stpoint, (float *)&slot.targpos, curTargetRs);
    if (curTargetRs[0] < rte_min(FSL_DELTAX - VEHICLE_WID * 0.5 - 0.80, 1.2) &&
        slot.targpos.x > VEHICLE_WID * 0.5)
    {
        return;
    }

    for (int i = 0; i < slots.multiNum; i++)
    {
        if (slots.multiArray[i].is_vision_slot == 0)
        {
            continue;
        }

        RevConvert(stpoint, (float *)&slots.multiArray[i].targpos, nextTargRs);
        if (nextTargRs[0] < curTargetRs[0])
        {
            continue;
        }
        if (nextTargRs[1] * curTargetRs[1] < 0)
        {
            continue;
        }

        if (slots.multiArray[i].slot_index == slot.slot_index ||
            Cal_Dis_Pt2Pt((float *)&slots.multiArray[i].targpos, (float *)&slot.targpos) <
                0.20f)
        {
            continue;
        }

        if (Cal_Dis_Pt2Pt(slots.multiArray[i].avm_point.near_rear,
                          slot.avm_point.near_front) < 0.20f)
        {
            return;
        }

        float ry1 = Project_PointTo1stPos_ry(slot.targpos, slot.avm_point.near_front);
        float ry2 = Project_PointTo1stPos_ry(slot.targpos,
                                             slots.multiArray[i].avm_point.near_rear);
        if (fabs(ry1) > fabs(ry2))
        {
            return;
        }
    }

    float tempE[3];
    RevConvert((float *)&slot.targpos, (float *)&slot.slotobj.ptE, tempE);

    float rxe = Project_PointTo1stPos_rx(slot.targpos, slot.avm_point.near_front);
    rxe       = rte_max(rxe, 3.0f);
    rxe       = (rxe + VEHICLE_LEN - REAR_SUSPENSION) * 0.5;
    tempE[0]  = rte_max(rxe, tempE[0]) - OBJ_SWELL;

    auto wide = rte_max(1.45f, fabs(tempE[1]));
    wide      = rte_max(VEHICLE_WID * 0.5f + 0.6f, wide);
    tempE[1]  = wide * sign(tempE[1]);

    float obj[4];
    Convert((float *)&slot.targpos, tempE, &obj[0]);
    tempE[1] = (fabs(tempE[1]) + 0.7) * sign(tempE[1]);
    Convert((float *)&slot.targpos, tempE, &obj[2]);

    memcpy(FS_Obj[offset + index], obj, sizeof(obj));
    FS_ObjDir[offset + index] = (offset < SF_OBJ_NUM) ? 10000 : 20000;
}

inline static void Obj_Change_By_Slot(const float pO[3], float pin[4], float ds)
{
    float pout[2];
    RevConvert(pO, &pin[0], pout);
    pout[1] = sign(pout[1]) * (fabs(pout[1]) + ds);
    Convert(pO, pout, &pin[0]);

    RevConvert(pO, &pin[2], pout);
    pout[1] = sign(pout[1]) * (fabs(pout[1]) + ds);
    Convert(pO, pout, &pin[2]);
}

inline static void Point_Change_By_Slot(const float pO[3], float pin[2], float ds)
{
    float pout[2];
    RevConvert(pO, &pin[0], pout);
    pout[1] = sign(pout[1]) * (fabs(pout[1]) + ds);
    Convert(pO, pout, &pin[0]);
}

/*
功能描述：检查视觉车位，处理障碍物信息
函数名称：Check_SlotObj_Is_VisonSlot
INTPUT:     车位slot，规划起点stpoint,
前长距超声波障碍物类型FS_ObjDir，前长距超声波障碍物FS_Obj，
       后长距超声波障碍物类型FS_ObjDir_Rear，前长距超声波障碍物FS_Obj_Rear
OUTPUT:     前长距超声波障碍物类型FS_ObjDir，前长距超声波障碍物FS_Obj，
       后长距超声波障碍物类型FS_ObjDir_Rear，前长距超声波障碍物FS_Obj_Rear
*/
static void Check_SlotObj_Is_VisonSlot(SlotInfo_T &slot, const float stpoint[3],
                                       int FS_ObjDir[FS_OBJ_ARR_NUM],
                                       float FS_Obj[FS_OBJ_ARR_NUM][4],
                                       int FS_ObjDir_Rear[FS_OBJ_ARR_NUM],
                                       float FS_Obj_Rear[FS_OBJ_ARR_NUM][4])
{
    float finpoint[3];
    float move_ds = 0.0f;
    float tempdist0[6];
    Relation_T dir0[6];
    int offset    = (PathPlan_SlotDir(slot.slotshap) == 2) ? SF_OBJ_NUM : 0;
    int rightSlot = (PathPlan_SlotDir(slot.slotshap) == 2) ? 1 : -1;

    Relation_T outside_slotCB = rightSlot > 0 ? PK_ON_LEFT : PK_ON_RIGHT;
    Relation_T outside_slotDE = rightSlot > 0 ? PK_ON_RIGHT : PK_ON_LEFT;

    memcpy(finpoint, &slot.targpos, sizeof(finpoint));
    if (slot.slotshap <= PK_SLOT_RIGHT_PARA)
    {
        finpoint[2] = Round_PI(finpoint[2] + rightSlot * PI_2);
    }
    else
    {
        finpoint[2] = Round_PI(finpoint[2]);
    }

    if (slot.slotshap > PK_SLOT_RIGHT_PARA)
    {
        float dist_finpos_to_ptB =
            fabsf(Project_PosTo1st_ry(finpoint, (float *)&slot.slotobj.ptB));
        move_ds = -(dist_finpos_to_ptB - (VEHICLE_WID / 2 + 0.35f));
        Point_Change_By_Slot(finpoint, (float *)&slot.slotobj.ptA, move_ds);
        Point_Change_By_Slot(finpoint, (float *)&slot.slotobj.ptB, move_ds);
        Point_Change_By_Slot(finpoint, (float *)&slot.slotobj.ptC, move_ds);
        float dist_finpos_to_ptE =
            fabsf(Project_PosTo1st_ry(finpoint, (float *)&slot.slotobj.ptE));
        move_ds = -(dist_finpos_to_ptE - (VEHICLE_WID / 2 + 0.35f));
        Point_Change_By_Slot(finpoint, (float *)&slot.slotobj.ptE, move_ds);
        Point_Change_By_Slot(finpoint, (float *)&slot.slotobj.ptD, move_ds);
        Point_Change_By_Slot(finpoint, (float *)&slot.slotobj.ptF, move_ds);
    }

    float freeWid = (Cal_Dis_Pt2Pt(slot.avm_point.near_rear, slot.avm_point.near_front) -
                     VEHICLE_WID) *
                    0.5f; // 车与车位的空余空间
    for (int j = 0; j < 2; j++)
    {
        for (int i = 0; i < SF_OBJ_NUM; i++)
        {
            int objDir = 0;
            LineSeg_T seg;
            if (j == 0)
            {
                objDir = FS_ObjDir[i + offset];
                memcpy(&seg, &FS_Obj[i + offset], sizeof(seg));
            }
            else
            {
                objDir = FS_ObjDir_Rear[i + offset];
                memcpy(&seg, &FS_Obj_Rear[i + offset], sizeof(seg));
            }

            if (objDir == 0)
            {
                continue;
            }
            else if (Get_Segment_Len(seg) < PathPlanConstCfg::valid_obj_len)
            {
                if (j == 0)
                {
                    FS_ObjDir[i + offset] = 0;
                    FS_Obj[i + offset][0] = 0.0f;
                    FS_Obj[i + offset][1] = 0.0f;
                    FS_Obj[i + offset][2] = 0.0f;
                    FS_Obj[i + offset][3] = 0.0f;
                }
                else
                {
                    FS_ObjDir_Rear[i + offset] = 0;
                    FS_Obj_Rear[i + offset][0] = 0.0f;
                    FS_Obj_Rear[i + offset][1] = 0.0f;
                    FS_Obj_Rear[i + offset][2] = 0.0f;
                    FS_Obj_Rear[i + offset][3] = 0.0f;
                }
                continue;
            }

            Get_Dist_Dir_Pt2PointLine(slot.slotobj.ptC, slot.slotobj.ptB, seg.pt1,
                                      &tempdist0[0], &dir0[0]);
            Get_Dist_Dir_Pt2PointLine(slot.slotobj.ptC, slot.slotobj.ptB, seg.pt2,
                                      &tempdist0[1], &dir0[1]);
            Get_Dist_Dir_Pt2PointLine(slot.slotobj.ptD, slot.slotobj.ptE, seg.pt1,
                                      &tempdist0[2], &dir0[2]);
            Get_Dist_Dir_Pt2PointLine(slot.slotobj.ptD, slot.slotobj.ptE, seg.pt2,
                                      &tempdist0[3], &dir0[3]);

            int is_change_type = 0;
            if (dir0[0] != dir0[1] || dir0[2] != dir0[3])
            {
                // 和BC/DE相交
                if (sign(Project_PosTo1st_ry(stpoint, &seg.pt1.x)) ==
                    sign(Project_PosTo1st_ry(stpoint, &seg.pt2.x)))
                {
                    // 在当前位置同一侧
                    is_change_type = 1;
                }
            }
            else if (dir0[0] != outside_slotCB && dir0[3] != outside_slotDE &&
                     Project_PosTo1st_rx(finpoint, &seg.pt1.x) > 0 &&
                     Get_Segment_Len(seg) < 1.2f * PathPlanConstCfg::valid_obj_len)
            {
                // 在车位内
                if (j == 0)
                {
                    FS_ObjDir[i + offset] = 0;
                    FS_Obj[i + offset][0] = 0.0f;
                    FS_Obj[i + offset][1] = 0.0f;
                    FS_Obj[i + offset][2] = 0.0f;
                    FS_Obj[i + offset][3] = 0.0f;
                }
                else
                {
                    FS_ObjDir_Rear[i + offset] = 0;
                    FS_Obj_Rear[i + offset][0] = 0.0f;
                    FS_Obj_Rear[i + offset][1] = 0.0f;
                    FS_Obj_Rear[i + offset][2] = 0.0f;
                    FS_Obj_Rear[i + offset][3] = 0.0f;
                }
                continue;
            }

            if (dir0[0] != dir0[1] && is_change_type)
            {
                if (j == 0)
                {
                    if (dir0[0] == outside_slotCB &&
                        (fabs(tempdist0[0]) > fabs(tempdist0[1]) ||
                         Get_Segment_Len(seg) < 2 * PathPlanConstCfg::valid_obj_len))
                    {
                        // 和BC相交, 且pt1在车位外，且在车位外的部分大于在车位内的部分 ---
                        // 截取车位内部分
                        Obj_Change_By_Slot(finpoint, FS_Obj[i + offset],
                                           fabs(tempdist0[1]));
                    }
                    else if (dir0[1] == outside_slotCB &&
                             (fabs(tempdist0[1]) > fabs(tempdist0[0]) ||
                              Get_Segment_Len(seg) < 2 * PathPlanConstCfg::valid_obj_len))
                    {
                        // 和BC相交, 且pt2在车位外，且在车位外的部分大于在车位内的部分 ---
                        // 截取车位内部分
                        Obj_Change_By_Slot(finpoint, FS_Obj[i + offset],
                                           fabs(tempdist0[0]));
                    }
                }
                else
                {
                    if (dir0[0] == outside_slotCB &&
                        (fabs(tempdist0[0]) > fabs(tempdist0[1]) ||
                         Get_Segment_Len(seg) <
                             2 * PathPlanConstCfg::
                                     valid_obj_len)) // pt1在车位外（CB左边）且seg在车位外的部分(*3)大与在车位内的部分
                    {
                        Obj_Change_By_Slot(finpoint, FS_Obj_Rear[i + offset],
                                           fabs(tempdist0[1]));
                    }
                    else if (
                        dir0[1] == outside_slotCB &&
                        (fabs(tempdist0[1]) > fabs(tempdist0[0]) ||
                         Get_Segment_Len(seg) <
                             2 * PathPlanConstCfg::
                                     valid_obj_len)) // pt2在车位外（CB左边）且seg在车位外的部分(*3)大与在车位内的部分
                    {
                        Obj_Change_By_Slot(finpoint, FS_Obj_Rear[i + offset],
                                           fabs(tempdist0[0]));
                    }
                }

                if (slot.slotshap > PK_SLOT_RIGHT_PARA &&
                    freeWid > 0.20f) // 车位识别过宽且障碍物入侵的情况，平移目标点
                {
                    float fpoint[3] = {0, -rightSlot * 0.10f * freeWid, 0};
                    Convert(&slot.targpos.x, fpoint, &slot.targpos.x);
                }
            }

            if (dir0[2] != dir0[3] && is_change_type)
            {
                if (j == 0)
                {
                    if (dir0[2] == outside_slotDE &&
                        (fabs(tempdist0[2]) > fabs(tempdist0[3]) ||
                         Get_Segment_Len(seg) <
                             3 * PathPlanConstCfg::
                                     valid_obj_len)) // pt1在车位外（DE右边）且seg在车位外的部分(*3)大与在车位内的部分
                    {
                        Obj_Change_By_Slot(finpoint, FS_Obj[i + offset],
                                           fabs(tempdist0[3]));
                    }
                    else if (
                        dir0[3] == outside_slotDE &&
                        (fabs(tempdist0[3]) > fabs(tempdist0[2]) ||
                         Get_Segment_Len(seg) <
                             3 * PathPlanConstCfg::
                                     valid_obj_len)) // pt2在车位外（DE右边）且seg在车位外的部分(*3)大与在车位内的部分
                    {
                        Obj_Change_By_Slot(finpoint, FS_Obj[i + offset],
                                           fabs(tempdist0[2]));
                    }
                }
                else
                {
                    if (dir0[2] == outside_slotDE &&
                        (fabs(tempdist0[2]) > fabs(tempdist0[3]) ||
                         Get_Segment_Len(seg) <
                             3 * PathPlanConstCfg::
                                     valid_obj_len)) // pt1在车位外（DE右边）且seg在车位外的部分(*3)大与在车位内的部分
                    {
                        Obj_Change_By_Slot(finpoint, FS_Obj_Rear[i + offset],
                                           fabs(tempdist0[3]));
                    }
                    else if (
                        dir0[3] == outside_slotDE &&
                        (fabs(tempdist0[3]) > fabs(tempdist0[2]) ||
                         Get_Segment_Len(seg) <
                             3 * PathPlanConstCfg::
                                     valid_obj_len)) // pt2在车位外（DE右边）且seg在车位外的部分(*3)大与在车位内的部分
                    {
                        Obj_Change_By_Slot(finpoint, FS_Obj_Rear[i + offset],
                                           fabs(tempdist0[2]));
                    }
                }

                if (slot.slotshap > PK_SLOT_RIGHT_PARA &&
                    freeWid > 0.20f) // 车位识别过宽的情况，平移目标点
                {
                    float fpoint[3] = {0, rightSlot * 0.10f * freeWid, 0};
                    Convert(&slot.targpos.x, fpoint, &slot.targpos.x);
                }
            }
        }
    }
}

/*
add 2023/7/6 Tangsj
功能描述：设置限位杆，调整目标点
函数名称：GetStopper
INTPUT:     场景数据slot,planData,限位杆距B点距离stopper_dist
OUTPUT:     车辆前方边界float_limit
*/
static void GetStopper(SlotInfo_T &slot, PlanDataCase &planData, float stopper_dist)
{
    if (slot.is_vision_slot <= 0 || slot.has_stopper <= 0 ||
        slot.slotshap >= PK_SLOT_RIGHT_PARA)
    {
        return;
    }
    float rspoint[2] = {0}, obj[4] = {0}, finpoint[3] = {0};
    memcpy(finpoint, &slot.targpos, sizeof(finpoint));

    //    Multi_StopBar_Array_T stopbars;
    //    RTE_PK_SensorFusion_Get_Avm_MultiStopBar(&stopbars);
    planData.has_stopper = slot.has_stopper;

    if (slot.slotshap >= PK_SLOT_RIGHT_PARA)
    {
        return;
    }

    if (slot.slotshap <= PK_SLOT_RIGHT_PARA)
    {
        RevConvert(finpoint, (float *)&slot.avm_point.near_rear.x, rspoint);
    }
    else
    {
        RevConvert(finpoint, (float *)&slot.avm_point.far_front.x, rspoint);
    }

    rspoint[0] +=
        -sign(rspoint[0]) *
        (WHELL_RADIUS + 1.10f); // sign(rspoint[0]) * (fabs(rspoint[0]) - stopper_dist);
    rspoint[1] = rspoint[1];
    Convert(finpoint, rspoint, (float *)&obj[0]);

    RevConvert(finpoint, (float *)&slot.avm_point.far_rear.x, rspoint);
    rspoint[0] +=
        -sign(rspoint[0]) *
        (WHELL_RADIUS + 1.10f); // sign(rspoint[0]) * (fabs(rspoint[0]) - stopper_dist);
    rspoint[1] = rspoint[1];
    Convert(finpoint, rspoint, (float *)&obj[2]);

    memcpy(planData.obj_stopper[0], obj, sizeof(float) * 4);

    float new_finpoint[2] = {
        (planData.obj_stopper[0][0] + planData.obj_stopper[1][0]) * 0.5f,
        (planData.obj_stopper[0][1] + planData.obj_stopper[1][1]) * 0.5f};
    memcpy(&slot.targpos.x, new_finpoint, sizeof(new_finpoint));
}

/*********************************************************************************************************************************/
static bool CheckRectOverLap(const float rect1[4][2], const float rect2[4][2])
{
    auto CheckLineCross = [](const float line11[2], const float line12[2],
                             const float line21[2], const float line22[2]) {
        Vec2_T vec1 = {line12[0] - line11[0], line12[1] - line11[1]};
        Vec2_T vec2 = {line21[0] - line11[0], line21[1] - line11[1]};
        Vec2_T vec3 = {line22[0] - line11[0], line22[1] - line11[1]};

        float crossv1 = Get_Cross_Vec2(vec1, vec2);
        float crossv2 = Get_Cross_Vec2(vec1, vec3);
        return sign(crossv1) * sign(crossv2);
    };

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            int v1 = CheckLineCross(rect1[i % 4], rect1[(i + 1) % 4], rect2[j % 4],
                                    rect2[(j + 1) % 4]);
            int v2 = CheckLineCross(rect2[j % 4], rect2[(j + 1) % 4], rect1[i % 4],
                                    rect1[(i + 1) % 4]);
            if (v1 <= 0 && v2 <= 0)
            {
                return true;
            }
        }
    }
    return false;
}

/*
功能描述：根据车位与车当前位置重新分类障碍物
函数名称：CheckObjType
INTPUT:     stPos[3], finpos[3], slotshape,
            Fusion_Obj[FS_OBJ_ARR_NUM][4],Fusion_ObjDir[FS_OBJ_ARR_NUM]
OUTPUT:
            Fusion_Obj_Temp[FS_OBJ_ARR_NUM][4],
            Fusion_ObjDir_Temp[FS_OBJ_ARR_NUM]
RETURN:
*/
static void CheckObjType(const float stPos[3], const float finPos[3], const int slotshap,
                         const float Fusion_Obj[FS_OBJ_ARR_NUM][4],
                         const int Fusion_ObjDir[FS_OBJ_ARR_NUM],
                         float Fusion_Obj_Temp[FS_OBJ_ARR_NUM][4],
                         int Fusion_ObjDir_Temp[FS_OBJ_ARR_NUM], int dir)
{
    int objType       = 0;
    int rightFlag     = (PathPlan_SlotDir(slotshap) == 2) ? 1 : -1;
    int slot_dir      = PathPlan_SlotDir(slotshap);
    float finpoint[3] = {0};

    PK_CopyPos(finpoint, finPos);
    if (slotshap <= PK_SLOT_RIGHT_PARA)
    {
        finpoint[2] = Round_PI(finpoint[2] + rightFlag * PI_2);
    }
    else
    {
        finpoint[2] = Round_PI(finpoint[2]);
    }

    int leftobjnum  = 0;
    int rightobjnum = 0;
    float slotdir   = sign(Project_PosTo1st_ry(stPos, finpoint));
    for (int i = 0; i < FS_OBJ_ARR_NUM; i++)
    {
        if (Fusion_ObjDir[i] == 0)
        {
            continue;
        }

        LineSeg_T seg;
        memcpy(&seg, &Fusion_Obj[i], sizeof(seg));
        if (dir == 1 && seg.pt1.x > 0 && seg.pt2.x > 0)
        {
            continue;
        }

        if (slotdir * sign(Project_PosTo1st_ry(stPos, &Fusion_Obj[i][0])) > 0)
        {
            objType = slot_dir;
        }
        else
        {
            objType = slot_dir == 1 ? 2 : 1;
        }

        // if (objType == 1) // left
        // {
        //     if (leftobjnum >= 32)
        //     {
        //         memcpy(Fusion_Obj_Temp[leftobjnum], Fusion_Obj[i], sizeof(float) * 4);
        //         Fusion_ObjDir_Temp[leftobjnum] = 10000;
        //         leftobjnum++;
        //     }
        //     else
        //     {
        //         memcpy(Fusion_Obj_Temp[leftobjnum], Fusion_Obj[i], sizeof(float) * 4);
        //         Fusion_ObjDir_Temp[leftobjnum] = 10000;
        //         leftobjnum++;
        //     }
        // }
        // else if (objType == 2) // right
        // {
        //     if (rightobjnum >= 32)
        //     {
        //         memcpy(Fusion_Obj_Temp[rightobjnum + SF_OBJ_NUM], Fusion_Obj[i],
        //                sizeof(float) * 4);
        //         Fusion_ObjDir_Temp[rightobjnum + SF_OBJ_NUM] = 20000;
        //         rightobjnum++;
        //     }
        //     else
        //     {
        //         memcpy(Fusion_Obj_Temp[rightobjnum + SF_OBJ_NUM], Fusion_Obj[i],
        //                sizeof(float) * 4);
        //         Fusion_ObjDir_Temp[rightobjnum + SF_OBJ_NUM] = 20000;
        //         rightobjnum++;
        //     }
        // }

        if (objType == 1) // left
        {
            if (leftobjnum < SF_OBJ_NUM)
            {
                memcpy(Fusion_Obj_Temp[leftobjnum], Fusion_Obj[i], sizeof(float) * 4);
                Fusion_ObjDir_Temp[leftobjnum] = 10000;
                leftobjnum++;
            }
            else
            {
                // 可加日志，说明左侧已满
            }
        }
        else if (objType == 2) // right
        {
            if (rightobjnum < SF_OBJ_NUM)
            {
                memcpy(Fusion_Obj_Temp[rightobjnum + SF_OBJ_NUM], Fusion_Obj[i],
                       sizeof(float) * 4);
                Fusion_ObjDir_Temp[rightobjnum + SF_OBJ_NUM] = 20000;
                rightobjnum++;
            }
            else
            {
                // 可加日志，说明右侧已满
            }
        }
    }
}

/**
 * @brief 更新车位
 *
 * @param pathPlanState 当前apa状态
 * @return
 */
static void PathPlan_UpdateSlot(const PK_ModuleComType pathPlanState,
                                Multi_Slot_Array_T &multiSlots)
{
    SlotInfo_T manuSlot;
    RTE_PK_DataConv_Get_SlotInfo(&manuSlot);
    if (pathPlanState == MCOM_OFF || pathPlanState == MCOM_DATCLC ||
        pathPlanState == MCOM_INIT)
    {
        if (manuSlot.slotshap != PK_SLOT_NO)
        {
            manuSlot.slot_index = 0;
            manuSlot.slotshap   = PK_SLOT_NO;
            RTE_PK_DataConv_Set_SlotInfo(&manuSlot);
        }
        return;
    }
    else if (pathPlanState != MCOM_ON_PARKING && pathPlanState != MCOM_ON)
    {
        multiSlots.multiNum = 0;
        return;
    }

    float stpoint[4];
    RTE_PK_Location_Get_CurPos(stpoint);
    RTE_PK_SlotDetect_Get_Multi_SlotInfo(&multiSlots);
    if (pathPlanState == MCOM_ON && RTE_PK_SM_GetApaMode() == PARKOUT_MODE)
    {
        // 泊出返回指定车位
        if (manuSlot.slot_index != 0 && manuSlot.slotshap != PK_SLOT_NO)
        {
            memcpy(&multiSlots.multiArray[0], &manuSlot, sizeof(manuSlot));
            multiSlots.multiNum = 1;
            return;
        }
    }

    if (pathPlanState == MCOM_ON_PARKING)
    {
        // 泊车过程中返回当前车位
        auto slotshap = RTE_PK_SlotDetect_Get_SlotShape();
        if (slotshap != PK_SLOT_NO && slotshap <= PK_SLOT_RIGHT_VERT)
        {
            if (PathPlan_SetParkingSlot(multiSlots.multiArray[0]) != PATHPLAN_SUCCESS)
            {
                multiSlots.multiNum = 0;
            }
            else
            {
                multiSlots.multiNum = 1;
            }
            return;
        }
    }

    // 没有指定车位, 返回查找到车位
    if (manuSlot.slot_index == 0 && manuSlot.slotshap == PK_SLOT_NO)
    {
        return;
    }

    if (multiSlots.multiNum > 0)
    {
        float maxdist = 0.0;
        int index     = 0;
        float rectbase[4][2];
        memcpy(rectbase[0], &manuSlot.avm_point.near_rear, 2 * sizeof(float));
        memcpy(rectbase[1], &manuSlot.avm_point.far_rear, 2 * sizeof(float));

        memcpy(rectbase[2], &manuSlot.avm_point.far_front, 2 * sizeof(float));
        memcpy(rectbase[3], &manuSlot.avm_point.near_front, 2 * sizeof(float));

        Multi_Slot_Array_T tempSlots;
        memset(&tempSlots, 0, sizeof(tempSlots));
        tempSlots.multiNum = 0;
        for (int count = 0; count < multiSlots.multiNum; count++)
        {
            // obj_slot de
            float dist =
                Cal_Dis_Pt2Pt(manuSlot.targpos, multiSlots.multiArray[count].targpos);
            float rectref[4][2];

            if (multiSlots.multiArray[count].is_vision_slot != 0)
            {
                memcpy(rectref[0], &multiSlots.multiArray[count].avm_point.near_rear,
                       2 * sizeof(float));
                memcpy(rectref[1], &multiSlots.multiArray[count].avm_point.far_rear,
                       2 * sizeof(float));
                memcpy(rectref[2], &multiSlots.multiArray[count].avm_point.far_front,
                       2 * sizeof(float));
                memcpy(rectref[3], &multiSlots.multiArray[count].avm_point.near_front,
                       2 * sizeof(float));
            }
            else
            {
                memcpy(rectref[0], &multiSlots.multiArray[count].slotobj.ptB,
                       2 * sizeof(float));
                memcpy(rectref[1], &multiSlots.multiArray[count].slotobj.ptC,
                       2 * sizeof(float));
                memcpy(rectref[2], &multiSlots.multiArray[count].slotobj.ptD,
                       2 * sizeof(float));
                memcpy(rectref[3], &multiSlots.multiArray[count].slotobj.ptE,
                       2 * sizeof(float));
            }

            bool slotOverLap = CheckRectOverLap(rectbase, rectref);
            // printf("%d max %d index %d %d dist = %06f, overlap %d\n", count, index,
            // multiSlots.multiArray[count].slot_index, manuSlot.slot_index, dist,
            // slotOverLap);
            if (slotOverLap || dist < VEHICLE_WID)
            {
                ;
            }
            else
            {
                memcpy(&tempSlots.multiArray[tempSlots.multiNum],
                       &multiSlots.multiArray[count], sizeof(SlotInfo_T));
                dist = Cal_Dis_Pt2Pt(
                    stpoint, (float *)&tempSlots.multiArray[tempSlots.multiNum].targpos);
                if (dist > maxdist)
                {
                    maxdist = dist;
                    index   = tempSlots.multiNum;
                }
                tempSlots.multiNum++;
            }
        }

        if (tempSlots.multiNum < MAX_PARKED_SLOTS)
        {
            memcpy(&tempSlots.multiArray[tempSlots.multiNum], &manuSlot,
                   sizeof(SlotInfo_T));
            tempSlots.multiNum++;
        }
        else
        {
            memcpy(&tempSlots.multiArray[index], &manuSlot, sizeof(SlotInfo_T));
        }
        memcpy(&multiSlots, &tempSlots, sizeof(tempSlots));
    }
    else
    {
        memcpy(&multiSlots.multiArray[0], &manuSlot, sizeof(manuSlot));
        multiSlots.multiNum = 1;
    }
}

static void PathPlan_SelectDefaultSlot(const Multi_Slot_Array_T &Multi_Slot_Array,
                                       const float stpoint[4], int *multiPlanRst,
                                       MultiPlanInfo *multPlanInfs)
{
    int PlanRstIndex[2];
    float minDist[MAX_PARKED_SLOTS];
    int minDistIdx[MAX_PARKED_SLOTS / 2];

    if (multPlanInfs[0].slotNum <= 0)
    {
        return;
    }

    float TargPos[3];
    for (int dir = 0; dir < 2; dir++)
    {
        int slotDir = (dir == 0) ? 1 : 2;
        int offset  = (dir == 0) ? 0 : MAX_PARKED_SLOTS / 2;

        // select all planned slots;
        int counter = 0;
        for (int i = 0; i < Multi_Slot_Array.multiNum; i++)
        {
            if (multPlanInfs[0].slotPlans[i].Path_num > 0)
            {
                if (PathPlan_SlotDir(Multi_Slot_Array.multiArray[i].slotshap) == slotDir)
                {
                    minDistIdx[counter] = i;
                    if (Multi_Slot_Array.multiArray[i].is_vision_slot == 2)
                    {
                        minDist[counter] = 0.0;
                    }
                    else
                    {
                        minDist[counter] = Cal_Dis_Pt2Pt(
                            (float *)&Multi_Slot_Array.multiArray[i].targpos, stpoint);
                    }
                    counter++;
                }
            }
        }

        // sort by dist
        for (int i = 0; i < counter - 1 && i < MAX_PARKED_SLOTS / 2; i++)
        {
            for (int j = i + 1; j < counter; j++)
            {
                if (minDist[j] < minDist[i])
                {
                    int idx       = minDistIdx[i];
                    minDistIdx[i] = minDistIdx[j];
                    minDistIdx[j] = idx;
                    float dist    = minDist[i];
                    minDist[i]    = minDist[j];
                    minDist[j]    = dist;
                }
            }
        }

        // record selected slot idx
        PlanRstIndex[dir] = rte_min(counter, MAX_PARKED_SLOTS / 2);
        for (int i = 0; i < PlanRstIndex[dir]; i++)
        {
            multiPlanRst[offset + i] = minDistIdx[i];
        }
    }

    float dirminDist[2] = {0.0, 0.0};
    if (PlanRstIndex[0] > 0)
    {
        ////////////////////////////////////////////////////////
        // 3.rerange the result
        memcpy(
            TargPos,
            &Multi_Slot_Array.multiArray[multiPlanRst[0 * MAX_PARKED_SLOTS / 2]].targpos,
            sizeof(TargPos));
        dirminDist[0] = Cal_Dis_Pt2Pt(TargPos, stpoint);
    }

    if (PlanRstIndex[1] > 0)
    {
        memcpy(
            TargPos,
            &Multi_Slot_Array.multiArray[multiPlanRst[1 * MAX_PARKED_SLOTS / 2]].targpos,
            sizeof(TargPos));
        dirminDist[1] = Cal_Dis_Pt2Pt(TargPos, stpoint);
    }

    MultiPlanInfo tempsPlanInfs;
    int plannedNumn = 0;
    for (int k = 0; k < 2; k++)
    {
        int offset = (dirminDist[0] < dirminDist[1]) ? k : 1 - k;
        for (int i = 0; i < PlanRstIndex[offset]; i++)
        {
            int slotIdx = multiPlanRst[offset * MAX_PARKED_SLOTS / 2 + i];
            memcpy(&tempsPlanInfs.slotPlans[plannedNumn],
                   &multPlanInfs[0].slotPlans[slotIdx], sizeof(PlanInfoType));
            tempsPlanInfs.slotPlans[plannedNumn].slotshape =
                Multi_Slot_Array.multiArray[slotIdx].slotshap;
            tempsPlanInfs.slotPlans[plannedNumn].Slot_index =
                Multi_Slot_Array.multiArray[slotIdx].slot_index;
            multiPlanRst[offset * MAX_PARKED_SLOTS / 2 + i] =
                tempsPlanInfs.slotPlans[plannedNumn].Path_num;
            plannedNumn++;
        }
    }

    memcpy(multPlanInfs, &tempsPlanInfs, sizeof(tempsPlanInfs));
    multPlanInfs->slotNum = plannedNumn;
}

static void BuildFusionObsInfo(const FusionObj_T &fsInfo, const int dir,
                               float Obj[SF_OBJ_NUM][4], int ObjDir[SF_OBJ_NUM])
{
    int count = 0;
    while (ObjDir[count] != 0 && count < SF_OBJ_NUM)
    {
        count++;
    }

    for (int i = 0; i < SF_OBJ_NUM && count < SF_OBJ_NUM; i++)
    {
        if (fsInfo.attr[i] / 10000 == dir)
        {
            if (count >= SF_OBJ_NUM)
            {
                log_err("obj number invalid %d %d %d\n", dir, i, fsInfo.attr[i]);
            }
            else
            {
                ObjDir[count] = fsInfo.attr[i];
                memcpy(Obj[count], &fsInfo.obj[i], sizeof(LineSeg_T));
                count++;
            }
        }
    }
}

/**
 * @brief 检查车位数量和位置是否发生变化
 *
 * @param multiPlans 路线
 * @param Multi_Slot_Array 车位
 * @return  1 表示发生变化 0 表示没有变化
 */
static uint8_t CheckTrajTargetPosChanged(const MultiPlanInfo &multiPlans,
                                         const Multi_Slot_Array_T &Multi_Slot_Array)
{
    for (int i = 0; i < Multi_Slot_Array.multiNum; i++)
    {
        int j = 0;
        for (j = 0; j < multiPlans.slotNum; j++)
        {
            if (multiPlans.slotPlans[j].Slot_index ==
                Multi_Slot_Array.multiArray[i].slot_index)
            {
                break;
            }
        }

        if (j >= multiPlans.slotNum)
        {
            return 1;
        }
    }
    return 0;
}

static void PlanPathSuccess(const int connect, const int idx, float paths[][TRAJITEM_LEN],
                            const int pathNum, const int revFlag,
                            const Avm_Pot_T &avmPoints, const SlotObj_T &slotobj,
                            const int slotindex, const PK_ModuleComType state,
                            MultiPlanInfo &multPlanInfs)
{
    multPlanInfs.slotPlans[idx].Path_num = pathNum;
    if (revFlag == 1)
    {
        PathExec_PathInverse(pathNum, paths);
    }

    memcpy(multPlanInfs.slotPlans[idx].Act_traj, paths,
           MAX_SINGLE_TRAJ_NUM * PATH_ITEM_LEN);
    multPlanInfs.slotPlans[idx].IsDirConect = connect;
    multPlanInfs.slotPlans[idx].Slot_index  = slotindex;
    memcpy(&multPlanInfs.slotPlans[idx].slotObjs, &slotobj, sizeof(SlotObj_T));
    memcpy(&multPlanInfs.slotPlans[idx].avm_point, &avmPoints, sizeof(Avm_Pot_T));
    // PathPlan_PrintFootPrints(multPlanInfs.slotPlans[idx].debug);
    multPlanInfs.slotNum++;

    bool testflag = true;
    float minLen  = PathPlan_ShortDist(paths, pathNum, NULL);
    if (minLen < 0.5 && testflag)
    {
        printf("slot index %d newDist %f\n", slotindex, minLen);
        // PathPlan_ShortDist(paths, pathNum, NULL);
    }

    if (state != MCOM_ON)
    {
        return;
    }

    float endPos[3];
    PK_Get_Path_EndPos(paths[pathNum - 1], endPos);

    Multi_Slot_Array_T multiSlots;
    RTE_PK_SlotDetect_Get_Multi_SlotInfo(&multiSlots);
    int index = 0;
    while (index < multiSlots.multiNum)
    {
        if (multiSlots.multiArray[index].slot_index == slotindex)
        {
            float dist =
                Cal_Dis_Pt2Pt((float *)&multiSlots.multiArray[index].targpos, endPos);
            float thetaDiv =
                Round_PI(multiSlots.multiArray[index].targpos.theta - endPos[2]);
            if (dist > 0.50f || fabs(thetaDiv) > PI_RAD * 10.0f)
            {
                // 因为车位信息在实时更新，但是路径刷新会慢一些，因此不匹配不一定就是错误的
                printf("Target changed! old End:%04f,%04f,%04f new End:%04f,%04f,%04f\n",
                       endPos[0], endPos[1], endPos[2],
                       multiSlots.multiArray[index].targpos.x,
                       multiSlots.multiArray[index].targpos.y,
                       multiSlots.multiArray[index].targpos.theta);
            }
            return;
        }
        index++;
    }
}

void PK_PathPlanRun(void)
{

    const float dist_between_slot_min = 1.0f; // 最小泊车位间距（m）
    static int update_time_out        = 0;    // 当车辆停止时更新车位信息
    static MultiPlanInfo multPlanInfs;        // 多路径规划信息
    static FusionObj_T FS_Info_A;             // 融合后的障碍物信息，体积大，定义为静态
    static float Fusion_Obj[FS_OBJ_ARR_NUM][4]      = {{0.0f}};  // 前向融合障碍物数组
    static int Fusion_ObjDir[FS_OBJ_ARR_NUM]        = {0};       // 前向障碍物方向
    static float Fusion_Obj_Rear[FS_OBJ_ARR_NUM][4] = {{0.0f}};  // 后向融合障碍物数组
    static int Fusion_ObjDir_Rear[FS_OBJ_ARR_NUM]   = {0};       // 后向障碍物方向
    static PlanDataCase planData[1]                 = {{0}};     // 当前路径规划数据
    static Multi_Slot_Array_T Multi_Slot_Array;                  // 多车位信息
    static int MultiPlanRst[MAX_PARKED_SLOTS]     = {0};         // 多路径规划结果
    static Vel_MoveMentStType Vel_MoveMentSt_last = LOCAT_STILL; // 上一次车辆运动状态
    static float last_path                        = 0;           // 上一次路径长度
    static int last_MultiSlotNum                  = 0;           // 上一次多车位数量
    static VehPos_T Targpos_last                  = {0};         // 上一次目标位置
    static FusionObj_T extObj;                                   // 扩展障碍物信息

    float move_ds_min = 0.10f; // 最小移动距离（m）
    float curPlannedPath[4 * MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN] = {
        {0.0f}};              // 当前规划轨迹
    PlanInfoType curParkPath; // 当前泊车路径信息
    PK_ModuleStateType ModuleState_PathPlan = MSTAT_OFF;
    float curPoints[4]; // 车辆的当前位置
    float endPos[3];    // 路径终点

    uint8 targetUpdate = 0; ///< 目标更新标志
    int IsDirConnect   = 0; ///< 是否直接连接标志
    uint8 trajStPosSt  = 0; ///< 轨迹起点状态

    uint8 move_ds_flag          = 0;     ///< 移动距离标志
    uint8 slotArray_Change_flag = 0;     ///< 车位数组变化标志，用于保持路径规划与车位一致
    uint8 IsTrajStPos[MAX_PARKED_SLOTS]; ///< 各车位轨迹起点状态数组

    ////////////////////////////////////////////////////////////
    // 1.do the initialization first
    auto cfgData = PathPlanCfg::getInstance();

    // 2.get RTE value
    // 状态机给到的模块状态
    PK_ModuleComType ModuleCom_PathPlan = RTE_PK_StateManage_Get_ModuleCom_PathPlan();
    // 车辆移动方向
    Vel_MoveMentStType Vel_MoveMentSt = RTE_PK_Location_Get_Vel_MoveMentSt();
    // 更新车位状态
    PathPlan_UpdateSlot(ModuleCom_PathPlan, Multi_Slot_Array);
    // 更新当前车辆的全局坐标
    RTE_PK_Location_Get_CurPos(curPoints);

    // 关闭模式,清除全局变量
    if (ModuleCom_PathPlan == MCOM_OFF)
    {
        memset(MultiPlanRst, 0, sizeof(MultiPlanRst));
        multPlanInfs.slotNum = 0;
        last_path            = 0;
        last_MultiSlotNum    = 0;
        memset(&Targpos_last, 0, sizeof(Targpos_last));
        ModuleState_PathPlan = MSTAT_OFF;
        PathPlan_CleanPlannedPath();
        PathPlan_PrintPathEnd();
        memset(&extObj, 0, sizeof(extObj));
    }
    // 初始化模式
    else if (ModuleCom_PathPlan == MCOM_DATCLC || ModuleCom_PathPlan == MCOM_INIT)
    {
        last_path         = 0;
        last_MultiSlotNum = 0;
        memset(&Targpos_last, 0, sizeof(Targpos_last));
        memset(MultiPlanRst, 0, sizeof(MultiPlanRst));
        multPlanInfs.slotNum = 0;
        PathPlan_CleanPlannedPath();
        ModuleState_PathPlan = MSTAT_DATCLC;
        PathPlan_PrintPathEnd();
        memset(&extObj, 0, sizeof(extObj));

        memset(&curParkPath, 0, sizeof(curParkPath));
        RTE_PK_PathPlan_Set_Park_PlanInfo(&curParkPath);

        PathPlan_PrintPlanEnd();
        PathPlan_HybridClearResult(Multi_Slot_Array);
    }
    // 开启模式
    else if (ModuleCom_PathPlan == MCOM_ON || ModuleCom_PathPlan == MCOM_ON_PARKING)
    {
        int Multi_Slot_num = Multi_Slot_Array.multiNum;
        memset(planData, 0, sizeof(PlanDataCase));
        memset(Fusion_ObjDir, 0, sizeof(Fusion_ObjDir));
        memset(Fusion_Obj, 0, sizeof(Fusion_Obj));
        memset(Fusion_ObjDir_Rear, 0, sizeof(Fusion_ObjDir_Rear));
        memset(Fusion_Obj_Rear, 0, sizeof(Fusion_Obj_Rear));
        memset(&FS_Info_A, 0, sizeof(FusionObj_T));

        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left(&FS_Info_A);
        BuildFusionObsInfo(FS_Info_A, 1, &Fusion_Obj[0], &Fusion_ObjDir[0]);
        BuildFusionObsInfo(FS_Info_A, 2, &Fusion_Obj[SF_OBJ_NUM],
                           &Fusion_ObjDir[SF_OBJ_NUM]);

        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right(&FS_Info_A);
        BuildFusionObsInfo(FS_Info_A, 1, &Fusion_Obj[0], &Fusion_ObjDir[0]);
        BuildFusionObsInfo(FS_Info_A, 2, &Fusion_Obj[SF_OBJ_NUM],
                           &Fusion_ObjDir[SF_OBJ_NUM]);

        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Left(&FS_Info_A);
        BuildFusionObsInfo(FS_Info_A, 1, &Fusion_Obj_Rear[0], &Fusion_ObjDir_Rear[0]);
        BuildFusionObsInfo(FS_Info_A, 2, &Fusion_Obj_Rear[SF_OBJ_NUM],
                           &Fusion_ObjDir_Rear[SF_OBJ_NUM]);

        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Right(&FS_Info_A);
        BuildFusionObsInfo(FS_Info_A, 1, &Fusion_Obj_Rear[0], &Fusion_ObjDir_Rear[0]);
        BuildFusionObsInfo(FS_Info_A, 2, &Fusion_Obj_Rear[SF_OBJ_NUM],
                           &Fusion_ObjDir_Rear[SF_OBJ_NUM]);

        move_ds_min = 0.15f;
        if (ModuleCom_PathPlan == MCOM_ON_PARKING)
        {
            RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Left(&FS_Info_A);
            BuildFusionObsInfo(FS_Info_A, 1, &Fusion_Obj[0], &Fusion_ObjDir[0]);
            BuildFusionObsInfo(FS_Info_A, 2, &Fusion_Obj[SF_OBJ_NUM],
                               &Fusion_ObjDir[SF_OBJ_NUM]);

            RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Right(&FS_Info_A);
            BuildFusionObsInfo(FS_Info_A, 1, &Fusion_Obj[0], &Fusion_ObjDir[0]);
            BuildFusionObsInfo(FS_Info_A, 2, &Fusion_Obj[SF_OBJ_NUM],
                               &Fusion_ObjDir[SF_OBJ_NUM]);

            move_ds_min = 0.3f;
            if (update_time_out >= 500)
            {
                update_time_out = 0;
            }

            PathPlan_CleanPlannedPath();
            PathPlan_PrintPathStart();
            RTE_PK_PathExecute_Get_CurTraj(&curParkPath);
        }
        else
        {
            memset(&extObj, 0, sizeof(extObj));
            PathPlan_PrintPathEnd();
            if (update_time_out >= 3000)
            {
                update_time_out = 0;
            }
        }

        if (curPoints[3] - last_path > move_ds_min)
        {
            last_path       = curPoints[3];
            move_ds_flag    = 1;
            update_time_out = 0;
        }
        else if (update_time_out++ < 3000 && (update_time_out % 5) == 0)
        {
            move_ds_flag = 1;
        }

        if (last_MultiSlotNum != Multi_Slot_num)
        {
            last_MultiSlotNum     = Multi_Slot_num;
            slotArray_Change_flag = 1;
        }
        else if (Multi_Slot_num > 0 &&
                 Cal_Dis_Pt2Pt(Targpos_last, Multi_Slot_Array.multiArray[0].targpos) >
                     dist_between_slot_min)
        {
            slotArray_Change_flag = 1;
        }

        if (Multi_Slot_num > 0)
        {
            Targpos_last = Multi_Slot_Array.multiArray[0].targpos;
        }

        targetUpdate = 0;
        if (Vel_MoveMentSt == LOCAT_STILL && multPlanInfs.slotNum > 0)
        {
            trajStPosSt =
                Check_TrajStartPos_Is_CurPos(multPlanInfs, curPoints, IsTrajStPos);
            targetUpdate = CheckTrajTargetPosChanged(multPlanInfs, Multi_Slot_Array);
        }

        if (Multi_Slot_num <= 0) // no slot clear infor
        {
            memset(MultiPlanRst, 0, sizeof(MultiPlanRst));
            multPlanInfs.slotNum = 0;
        }
        else if (slotArray_Change_flag == 1 || move_ds_flag == 1 ||
                 multPlanInfs.slotNum < 1 ||
                 (Vel_MoveMentSt_last != LOCAT_STILL && Vel_MoveMentSt == LOCAT_STILL) ||
                 targetUpdate == 1 ||
                 trajStPosSt == 0) // Vel_MoveMentSt!=LOCAT_STILL if no result or car is
                                   // moving,then plan
        {
            // 去掉被丢弃的车位对应的缓存路径
            PathPlan_UpdatePlannedSlots(Multi_Slot_Array);

            // 调试用，向仿真调试软件发送信息
            if (ModuleCom_PathPlan == MCOM_ON)
            {
                PathPlan_ReportObjsInfo(Fusion_ObjDir, Fusion_Obj, Fusion_ObjDir_Rear,
                                        Fusion_Obj_Rear);
            }

            multPlanInfs.slotNum = 0;
            uint64_t startMs     = RTE_BSW_Get_CurTime();
            for (int i = 0; i < Multi_Slot_num; i++)
            {
                // get the only slot infor
                uint64_t endMs = RTE_BSW_Get_CurTime();
                if (endMs - startMs > 100 && i > 0)
                {
                    printf("path plan timeout(100ms): slot index %d runtime %ld(ms)\n",
                           Multi_Slot_Array.multiArray[i - 1].slot_index,
                           endMs - startMs);
                }

                startMs = RTE_BSW_Get_CurTime();

                multPlanInfs.slotPlans[i].Path_num = 0;
                SlotInfo_T &slot                   = Multi_Slot_Array.multiArray[i];
                if (slot.occupied == 1 || slot.occupied == 3)
                {
                    continue;
                }

                // if (16609 == slot.slot_index){  while(testflag) {
                // printf("------------");  }  };
                SlotInfo_T slot_temp;
                int revFlag = 0;

                memset(planData, 0, sizeof(PlanDataCase));
                if (ModuleCom_PathPlan == MCOM_ON_PARKING)
                {
                    if (RTE_BSW_Get_EPS_AngleSpd() > 50.0 ||
                        curParkPath.IsDirConect == PARK_NOUPDATE)
                    {
                        continue;
                    }

                    float vehSpd = RTE_BSW_Get_ESC_VehSpd() / 3.6;
                    // if (vehSpd > 0.1) {  continue;  }

                    float steeringAngle = RTE_BSW_Get_EPS_Angle();
                    PK_Pre_locat(curPoints, planData[0].stpoint, steeringAngle,
                                 vehSpd * sign(curParkPath.Act_traj[0][3]), 0.35f);
                }
                else
                {
                    if (cfgData[0].plan_front_slot == 0)
                    {
                        float rx = Project_PosTo1st_rx(curPoints, (float *)&slot.targpos);
                        if (slot.slotshap < PK_SLOT_LEFT_VERT)
                        {
                            if (rx > -VEHICLE_LEN * 0.5f)
                            {
                                continue;
                            }
                        }
                        else
                        {
                            if (rx > -VEHICLE_WID * 0.5f)
                            {
                                continue;
                            }
                        }
                    }

                    memcpy(planData[0].stpoint, curPoints, sizeof(curPoints));
                    planData[0].stpoint[2] = Round_PI(planData[0].stpoint[2]);
                }

                float Fusion_Obj_temp[FS_OBJ_ARR_NUM][4],
                    Fusion_Obj_Rear_temp[FS_OBJ_ARR_NUM][4];
                int Fusion_ObjDir_temp[FS_OBJ_ARR_NUM],
                    Fusion_ObjDir_Rear_temp[FS_OBJ_ARR_NUM];
                memset(Fusion_Obj_temp, 0, sizeof(Fusion_Obj_temp));
                memset(Fusion_ObjDir_temp, 0, sizeof(Fusion_ObjDir_temp));
                memcpy(Fusion_ObjDir_Rear_temp, Fusion_ObjDir_Rear,
                       sizeof(Fusion_ObjDir_Rear));
                memcpy(Fusion_Obj_Rear_temp, Fusion_Obj_Rear, sizeof(Fusion_Obj_Rear));

                // 根据车辆位置调整超声波障碍物方向类型（车辆左侧还是车辆右侧），目的为方便后续判断障碍物与车位的位置关系（同侧还是对向）
                CheckObjType(curPoints, (float *)&slot.targpos, slot.slotshap, Fusion_Obj,
                             Fusion_ObjDir, Fusion_Obj_temp, Fusion_ObjDir_temp, 0);
                CheckObjType(curPoints, (float *)&slot.targpos, slot.slotshap,
                             Fusion_Obj_Rear, Fusion_ObjDir_Rear, Fusion_Obj_Rear_temp,
                             Fusion_ObjDir_Rear_temp, 1);

                // 更新长度
                UpdateObjLenByRaw(slot, Fusion_ObjDir_temp, Fusion_Obj_temp,
                                  Fusion_ObjDir_Rear_temp, Fusion_Obj_Rear_temp);

                FilterObjByLen(Fusion_ObjDir_temp, Fusion_Obj_temp,
                               Fusion_ObjDir_Rear_temp, Fusion_Obj_Rear_temp);

                float rx1 = Project_PosTo1st_rx(curPoints, (float *)&slot.targpos);
                if (rx1 > 0 && (slot.is_vision_slot == 2 || slot.slot_index < 10000) &&
                    RTE_PK_SM_GetApaMode() ==
                        PARKOUT_MODE) // FSL_DELTAX)//HAVECHANGE Tangsj
                                      // 2023/3/20 OLD: rx1 > 0
                {
                    memcpy(&slot_temp, &slot, sizeof(SlotInfo_T));

                    revFlag = 1;
                    memcpy(planData[0].stpoint, &slot.targpos,
                           sizeof(planData[0].finpoint));
                    if (PathPlan_InverseSlot((PK_SlotShapeType)slot.slotshap, curPoints,
                                             slot) != PATHPLAN_SUCCESS)
                    {
                        PathPlan_ReportTrajsInfo(curPlannedPath, 0, planData[0],
                                                 slot.slot_index);
                        log_info("PathPlan_InverseSlot failed!\n");
                        continue;
                    }
                }

                GetStopper(slot, planData[0], 1.0);
                if (slot.is_vision_slot == 1) // 视觉车位
                {
                    // 裁剪筛选超声波障碍物
                    Check_SlotObj_Is_VisonSlot(
                        slot, planData[0].stpoint, Fusion_ObjDir_temp, Fusion_Obj_temp,
                        Fusion_ObjDir_Rear_temp, Fusion_Obj_Rear_temp);
                    AddDynObjForPreSlot(Multi_Slot_Array, slot, planData[0].stpoint,
                                        Fusion_ObjDir_temp, Fusion_Obj_temp);
                }
                else if (slot.is_vision_slot == 0) // 超声波车位
                {
                    // 补全超声波单边界车位的数据
                    Check_SlotObj_Is_Single_Boundary(slot);
                    if (!ultrasonic_parking_space) // 配置参数，超声波车位有效性，无效跳过超声波车位
                    {
                        continue;
                    }
                }
                else if (RTE_PK_SM_GetApaMode() != PARKOUT_MODE) // 不是泊出模式
                {
                    slot.is_vision_slot = 1;
                }

                memcpy(planData[0].finpoint, &slot.targpos, sizeof(planData[0].finpoint));
                planData[0].finpoint[2]    = Round_PI(planData[0].finpoint[2]);
                planData[0].slotshape      = slot.slotshap; //  INIT
                planData[0].is_vision_slot = slot.is_vision_slot;

                PathPlan_ReportSlotsInfo(slot, planData[0].stpoint);

                // 扩大车位边缘（可泊空间）slot.slotobj
                if (ExpandSlotObjs(slot, curPoints) != PATHPLAN_SUCCESS)
                {
                    printf("ExpandSlotObjs failed! index: %d\n", slot.slot_index);
                    continue;
                }

                // 根据障碍物更新车位边缘slot.slotobj
                int range = UpdateSlotWithObsInSlot(
                    slot, Fusion_ObjDir_temp, Fusion_Obj_temp, Fusion_ObjDir_Rear_temp,
                    Fusion_Obj_Rear_temp);
                if (range != PATHPLAN_SUCCESS)
                {
                    SlotObj_Convert_float(slot.slotobj, planData[0].obj_slot);
                    PathPlan_ReportTrajsInfo(curPlannedPath, 0, planData[0],
                                             slot.slot_index);
                    printf("UpdateSlotWithObsInSlot failed! slot id %d code 0x%x 0x%x\n",
                           slot.slot_index, ((uint32_t)range >> 16),
                           ((uint32_t)range & 0xff));
                    continue;
                }

                // 构建泊车规划数据
                range = PathPlan_RangeDataProcess(
                    planData, slot, Fusion_ObjDir_temp, Fusion_Obj_temp,
                    Fusion_ObjDir_Rear_temp, Fusion_Obj_Rear_temp);
                if (range <= 0)
                {
                    PathPlan_ReportTrajsInfo(curPlannedPath, 0, planData[0],
                                             slot.slot_index);
                    printf("PathPlan_RangeDataProcess failed! index: %d %d\n",
                           slot.slot_index, range);
                    continue;
                }

                // 延长EF，目的：提高车辆向前探索的安全性和成功率
                if (planData[0].is_vision_slot > 0 ||
                    sign(planData[0].stpoint[2]) ==
                        sign(Get_LineSegment_Theta(&planData[0].obj_slot[4][0],
                                                   &planData[0].obj_slot[4][2])))
                {
                    Extend_LineSegment(&planData[0].obj_slot[4][0],
                                       &planData[0].obj_slot[4][2], 1.50f);
                    Extend_LineSegment(&planData[0].obj_slot[0][2],
                                       &planData[0].obj_slot[0][0], 1.50f);
                }

                // 泊出，扩大可泊空间，以便于重规划
                if (rx1 > 0 && slot.is_vision_slot == 2 &&
                    RTE_PK_SM_GetApaMode() ==
                        PARKOUT_MODE) // 还原并扩大自选车位边界(泊出)
                {
                    slot_temp.is_vision_slot = 1;
                    ExpandSlotObjs(slot_temp, curPoints);
                    if (0) // slot.slotshap >= PK_SLOT_LEFT_VERT)
                    {
                        memcpy(&slot.slotobj, &slot_temp.slotobj, sizeof(SlotObj_T));
                        memcpy(&slot.avm_point, &slot_temp.avm_point, sizeof(Avm_Pot_T));
                    }
                }
                else if (
                    slot.slot_index <
                    10000) // 车位编号小于10000表示指定车位，非泊出状态时指定车位规划前数据处理与感知搜索车位一致
                {
                    planData[0].is_vision_slot = 2;
                }

                int pathNum = 0;
                memset(curPlannedPath, 0, sizeof(curPlannedPath));
                PlanedPath &lastPath = PathPlan_GetPlannedPath(slot.slot_index);

                // 尝试连接当前位置与旧路线的起点
                bool destUpdated = true;
                if (lastPath.pathNum > 0 && lastPath.slotIndex > 0)
                {
                    PK_Get_Path_EndPos(lastPath.trajs[lastPath.pathNum - 1], endPos);
                    destUpdated =
                        (PathPlan_CheckPosDiv(endPos, planData[0].finpoint) != 0);
                    float thetaDiv = Round_PI(endPos[2] - planData[0].finpoint[2]);
                    if (fabs(thetaDiv) > 2 * PathPlanConstCfg::div_err_theta_fin)
                    {
                        destUpdated = true;
                    }
                }

                if ((Vel_MoveMentSt_last != LOCAT_STILL &&
                     Vel_MoveMentSt == LOCAT_STILL) == false &&
                    lastPath.counter < 3 && lastPath.pathNum > 5 && !destUpdated)
                {
                    pathNum = PathPlan_UpdatePlanPath(lastPath, planData[0],
                                                      curPlannedPath, false);
                    if (pathNum > 0 && pathNum < MAX_SINGLE_TRAJ_NUM)
                    {
                        float minDist = PathPlan_ShortDist(curPlannedPath, pathNum, NULL);
                        if (minDist > 0.8)
                        {
                            PlanPathSuccess(0x300, i, curPlannedPath, pathNum, revFlag,
                                            slot.avm_point, slot.slotobj, slot.slot_index,
                                            ModuleCom_PathPlan, multPlanInfs);
                            PathPlan_ReportTrajsInfo(curPlannedPath, pathNum, planData[0],
                                                     slot.slot_index);
                            continue;
                        }
                    }
                }

                // 路径规划
                if (ModuleCom_PathPlan == MCOM_ON_PARKING)
                {
                    PathPlan_DyanObj(planData, extObj, curPoints);

                    int rearNum = 0;
                    for (int j = 0; j < FS_OBJ_ARR_NUM; j++)
                    {
                        if (Fusion_ObjDir_Rear[j] != 0)
                        {
                            rearNum++;
                        }
                    }

                    for (int i = 0; i < extObj.num; i++)
                    {
                        if (planData[0].obj_danger_num < SF_OBJ_NUM * 2)
                        {
                            memcpy((float *)&planData[0]
                                       .obj_danger[planData[0].obj_danger_num],
                                   (float *)&extObj.obj[i], sizeof(LineSeg_T));
                            planData[0].obj_danger_dir[planData[0].obj_danger_num] =
                                extObj.attr[i] / 10000;
                            planData[0].obj_danger_num++;
                        }
                        else
                        {
                            log_err("dynamic objects overflow! %d, %d\n", extObj.num, i);
                        }

                        if (rearNum < FS_OBJ_ARR_NUM && Fusion_ObjDir_Rear[rearNum] != 0)
                        {
                            memcpy(Fusion_Obj_Rear[rearNum], (float *)&extObj.obj[i],
                                   sizeof(LineSeg_T));
                            Fusion_ObjDir_Rear[rearNum] = extObj.attr[i];
                            rearNum++;
                        }
                    }

                    PathPlan_ReportObjsInfo(Fusion_ObjDir, Fusion_Obj, Fusion_ObjDir_Rear,
                                            Fusion_Obj_Rear);
                }

                uint64_t planStartMs = RTE_BSW_Get_CurTime();
                PathPlan_InitFootPrints();

                // 只有之前没有过路径才需要查询混合A*
                bool hybridReplan = false;
                pathNum =
                    PathPlan_GetTraj(Multi_Slot_Array, slot, planData, curPlannedPath,
                                     (Vel_MoveMentSt == LOCAT_STILL), &IsDirConnect);
                if (pathNum <= 0)
                {
                    pathNum = PathPlan_HybridReplanResult(slot.slot_index, planData,
                                                          curPlannedPath);
                    if (pathNum > 0)
                    {
                        hybridReplan = true;
                        PathPlan_AddPlanPath(lastPath, endMs - startMs, curPlannedPath,
                                             pathNum); // 一定要更新当前位置
                        pathNum = 0;
                    }
                }

                uint64_t planEndMs = RTE_BSW_Get_CurTime();
                if (planEndMs - planStartMs > 100)
                {
                    printf(
                        "plan timeout(100ms): slot index %d runtime %ld(ms), path num: "
                        "%d\n",
                        Multi_Slot_Array.multiArray[i].slot_index,
                        planEndMs - planStartMs, pathNum);
                }

                if (pathNum > 0)
                {
                    // printf("slot %d pathplan success! %d num:%d replan:0x%08x
                    // 0x%08x\n", slot.slot_index, slot.avm_point.slot_index, pathNum,
                    //     cfgData[0].pathplan_step, cfgData[0].pathplan_step0);
                    if (PathPlan_CheckPathOverFlow(&curPlannedPath[0][0]) != 0 ||
                        PathPlan_CheckPlannedPathValid(curPlannedPath, pathNum) != 0)
                    {
                        log_info(
                            "target plan path overflow ! index %d, id: %d target: %f %f "
                            "%f slotinfo: %d %d\n",
                            i, slot.slot_index, planData[0].finpoint[0],
                            planData[0].finpoint[1], planData[0].finpoint[2],
                            slot.is_vision_slot, slot.slotshap);
                        pathNum = 0;
                    }
                    else
                    {
                        float tempPath11[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
                        int temppathNum11 = pathNum;
                        memcpy(tempPath11, curPlannedPath, PATH_ITEM_LEN * temppathNum11);

                        // 路径的简单优化
                        float tempPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
                        int revCount[2];

                        int temppathNum =
                            PK_PathMerge(curPlannedPath, pathNum, slot.slot_index);
                        memcpy(tempPath, curPlannedPath, PATH_ITEM_LEN * temppathNum);
                        float newDist =
                            PathPlan_ShortDist(tempPath, temppathNum, &revCount[0]);

                        // pathNum = PK_PathOptimize(curPlannedPath, pathNum,
                        // slot.slot_index, planData[0].obj_danger_num,
                        // planData[0].obj_danger);
                        bool updated = false;
                        if (!destUpdated && lastPath.pathNum > 0 &&
                            slot.is_vision_slot != 2 &&
                            revCount[0] > 1) // 泊出车位不要重规划, 路径是反向的
                        {
                            float tempPath1[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
                            int tempPathNum1 = lastPath.pathNum;
                            memcpy(tempPath1, lastPath.trajs,
                                   lastPath.pathNum * PATH_ITEM_LEN);
                            int pathNum2 = PathPlan_UpdatePlanPath(lastPath, planData[0],
                                                                   curPlannedPath, false);
                            if (pathNum2 > 0 && pathNum2 < MAX_SINGLE_TRAJ_NUM)
                            {
                                float oldDist = PathPlan_ShortDist(curPlannedPath,
                                                                   pathNum, &revCount[1]);
                                if (revCount[1] <= revCount[0] && newDist < oldDist &&
                                    newDist < 1.0)
                                {
                                    memcpy(tempPath, curPlannedPath,
                                           pathNum2 * PATH_ITEM_LEN);
                                    temppathNum = pathNum2;
                                    updated     = true;
                                }
                            }
                            else
                            {
                                char info[128] = "update 0";
                                PathPlan_PrintPlanDebug(info, tempPath1, tempPathNum1,
                                                        planData[0]);
                                printf(
                                    "-------------------- update 0 slot %d failed "
                                    "%d------------------------\n",
                                    slot.slot_index, pathNum2);
                            }
                        }

                        if (!updated)
                        {
                            PathPlan_AddPlanPath(lastPath, endMs - startMs, tempPath,
                                                 temppathNum);
                        }

                        PlanPathSuccess(IsDirConnect, i, tempPath, temppathNum, revFlag,
                                        slot.avm_point, slot.slotobj, slot.slot_index,
                                        ModuleCom_PathPlan, multPlanInfs);
                        PathPlan_ReportTrajsInfo(tempPath, temppathNum, planData[0],
                                                 slot.slot_index);
                        continue;
                    }
                }
                else if (hybridReplan == false)
                {
                    // printf("slot %d pathplan failed! %d\n", slot.slot_index,
                    // slot.avm_point.slot_index);
                }

                // 当前车位路线规划失败，且该车位在之前已规划成功过
                if (((pathNum == 0 || hybridReplan) && ModuleCom_PathPlan == MCOM_ON) &&
                    lastPath.slotIndex != -1 && slot.slot_index == lastPath.slotIndex &&
                    lastPath.pathNum > 0 && slot.is_vision_slot != 2)
                {
                    float tempPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
                    int tempPathNum = lastPath.pathNum;
                    memcpy(tempPath, lastPath.trajs, lastPath.pathNum * PATH_ITEM_LEN);
                    pathNum =
                        PathPlan_UpdatePlanPath(lastPath, planData[0], curPlannedPath,
                                                destUpdated); // 先尝试连接旧路线起点
                    if (pathNum > 0 && pathNum < MAX_SINGLE_TRAJ_NUM)
                    {
                        PlanPathSuccess(0x301, i, curPlannedPath, pathNum, revFlag,
                                        slot.avm_point, slot.slotobj, slot.slot_index,
                                        ModuleCom_PathPlan, multPlanInfs);
                        PathPlan_ReportTrajsInfo(curPlannedPath, pathNum, planData[0],
                                                 slot.slot_index);
                        // uint64_t endMs = RTE_BSW_Get_CurTime();
                        // printf("replan failed update: slot index %d number %d runtime
                        // %ld(ms)\n", slot.slot_index, pathNum, endMs - startMs);
                        continue;
                    }
                    else
                    {
                        printf(
                            "-------------------- update 1 failed "
                            "%d------------------------\n",
                            pathNum);
                        char info[128] = "update 1";
                        PathPlan_PrintPlanDebug(info, tempPath, tempPathNum, planData[0]);
                        // PathPlan_HybridReplan(planData, lastPath.trajs,
                        // lastPath.pathNum, slot.slot_index); printf("slot %d connect old
                        // path failed! %d,pathNum:%d,%d\n", slot.slot_index,
                        // slot.avm_point.slot_index, pathNum,check);
                        //  pathNum = PathPlan_UpdatePlanPath(lastPath, planData[0],
                        //  curPlannedPath);//先尝试连接旧路线起点
                        //  PK_Check_Path(pathNum,curPlannedPath,planData[0].stpoint,planData[0].finpoint,planData[0].obj_danger_num,planData[0].obj_danger,0);
                    }
                }
                else
                {
                    if (pathNum == 0 && ModuleCom_PathPlan == MCOM_ON)
                    {
                        PathPlan_ReportTrajsInfo(curPlannedPath, 0, planData[0],
                                                 slot.slot_index);
                    }

                    // auto replan = PathPlan_HybridReplan(planData, lastPath.trajs, 0,
                    // slot.slot_index); printf("slot %d PathPlan_HybridReplan! result:
                    // %d\n", slot.slot_index, replan);
                }
            }

            ///////////////////////////////////////////////////////////
            // 2.select the best slot
            PathPlan_HybridClearResult(Multi_Slot_Array);
            PathPlan_SelectDefaultSlot(Multi_Slot_Array, planData[0].stpoint,
                                       MultiPlanRst, &multPlanInfs);
            // if (multPlanInfs.slotNum != Multi_Slot_Array.multiNum) {
            // printf("Multi_Slot_Array %d delete traject! %d\n", multPlanInfs.slotNum,
            // Multi_Slot_Array.multiNum);}

            ModuleState_PathPlan =
                (multPlanInfs.slotNum > 0) ? MSTAT_GET_RESULT : MSTAT_NORM;
        }
    }

    if (ModuleCom_PathPlan == MCOM_ON_PARKING && curParkPath.IsDirConect == PARK_UPDATE)
    {
        int check[6];
        memset(check, 0, sizeof(check));
        check[1] = 5;
        check[2] = (slotArray_Change_flag == 1);
        check[2] = (check[2] << 1) | (move_ds_flag == 1);
        check[2] = (check[2] << 1) | (multPlanInfs.slotNum < 1);
        check[2] = (check[2] << 1) |
                   (Vel_MoveMentSt_last != LOCAT_STILL && Vel_MoveMentSt == LOCAT_STILL);
        check[2] = (check[2] << 1) | (targetUpdate == 1);
        check[2] = (check[2] << 1) | (trajStPosSt == 0);
        check[3] = ModuleState_PathPlan;
        check[4] = 0;

        float endPos[3];
        for (int i = 0; i < curParkPath.Path_num; i++)
        {
            PK_Get_Path_EndPos(curParkPath.Act_traj[i], endPos);
            check[0] = PK_Check_Path(1, &curParkPath.Act_traj[i], curParkPath.Act_traj[i],
                                     endPos, planData[0].obj_danger_num,
                                     planData[0].obj_danger, 0);
            if (check[0] != 0)
            {
                check[4] = i;
                PathPlan_PrintDangerObj("park", &curParkPath.Act_traj[i], 1, planData[0]);
                break;
            }
        }
        // MSTAT_GET_RESULT  6 重规划成功  MSTAT_NORM  重规划失败  MSTAT_OFF 未规划

        memset(&curParkPath, 0, sizeof(curParkPath));
        if (ModuleState_PathPlan == MSTAT_GET_RESULT) // 重规划成功
        {
            int state = 0;
            if (check[0] != PATHPLAN_SUCCESS)
            {
                check[1] = (multPlanInfs.slotNum > 0) ? 0 : 1;
                state    = (multPlanInfs.slotNum > 0) ? PARK_DANGER_SWITCH
                                                      : PARK_DANGER_NOSWITCH;
            }
            else
            {
                check[1] = (multPlanInfs.slotNum > 0) ? 2 : 3;
                state    = (multPlanInfs.slotNum > 0) ? PARK_SWITCH : PARK_NOSWITCH;
            }

            auto dangerSt = PK_ObjAvoid_Dir();
            int testflag  = 1;
            if ((dangerSt == 2 &&
                 sign(multPlanInfs.slotPlans[0].Act_traj[0][3]) < -0.01f) ||
                (dangerSt == 1 && sign(multPlanInfs.slotPlans[0].Act_traj[0][3]) > 0.01f))
            {
                if (testflag)
                {
                    printf(" replan error \n");
                    usleep(10000);
                }
            }
            else
            {
                PathPlan_PrintPathInfo(multPlanInfs.slotPlans[0]);
                if (multPlanInfs.slotNum > 0)
                {
                    memcpy(&curParkPath, &multPlanInfs.slotPlans[0],
                           sizeof(PlanInfoType));
                    curParkPath.IsDirConect = state;
                    RTE_PK_PathPlan_Set_Park_PlanInfo(&curParkPath);
                }
            }
        }
        else if (ModuleState_PathPlan == MSTAT_NORM) // 重规划失败
        {
            curParkPath.Path_num = 0;
            curParkPath.IsDirConect =
                (check[0] == PATHPLAN_SUCCESS) ? PARK_NOSWITCH : PARK_DANGER_NOSWITCH;
            RTE_PK_PathPlan_Set_Park_PlanInfo(&curParkPath);
        }
        else // 未规划
        {
            RTE_PK_PathPlan_Get_Park_PlanInfo(&curParkPath);
            if (curParkPath.Path_num > 0 &&
                (curParkPath.IsDirConect == PARK_DANGER_SWITCH ||
                 curParkPath.IsDirConect == PARK_SWITCH)) // 检查之前的规划结果
            {
                auto checkres =
                    PK_Check_Path(curParkPath.Path_num, &curParkPath.Act_traj[0],
                                  curParkPath.Act_traj[0], endPos,
                                  planData[0].obj_danger_num, planData[0].obj_danger, 0);
                if (check[0] == PATHPLAN_SUCCESS && checkres == PATHPLAN_SUCCESS)
                {
                    curParkPath.IsDirConect = PARK_SWITCH;
                }
                else if (check[0] == PATHPLAN_SUCCESS && checkres != PATHPLAN_SUCCESS)
                {
                    curParkPath.IsDirConect = PARK_NOSWITCH;
                    curParkPath.Path_num    = 0;
                }
                else if (check[0] != PATHPLAN_SUCCESS && checkres == PATHPLAN_SUCCESS)
                {
                    curParkPath.IsDirConect = PARK_DANGER_SWITCH;
                }
                else if (check[0] != PATHPLAN_SUCCESS && checkres != PATHPLAN_SUCCESS)
                {
                    curParkPath.Path_num    = 0;
                    curParkPath.IsDirConect = PARK_DANGER_NOSWITCH;
                }
            }
            else
            {
                if (check[0] == PATHPLAN_SUCCESS)
                {
                    curParkPath.IsDirConect = PARK_NOSWITCH;
                }
                else
                {
                    curParkPath.IsDirConect = PARK_DANGER_NOSWITCH;
                }
            }
            RTE_PK_PathPlan_Set_Park_PlanInfo(&curParkPath);
        }

        PathPlan_PrintPathUpdateResult(check);
    }
    else
    {
        RTE_PK_PathPlan_Set_Multi_PlanInfo(&multPlanInfs);
        RTE_PK_PathPlan_Set_ModuleState_PathPlan(ModuleState_PathPlan);
        RTE_PK_PathPlan_Set_Multi_PlanRst(MultiPlanRst);
    }
    Vel_MoveMentSt_last = Vel_MoveMentSt;
}

// 20180621:get a rectangel area relative to slotB to filt the fusion objects from process
// A
static int Get_Plan_RangeArea(const PlanDataCase &PlanDataVer, float slotB_Pos[3],
                              float rect_PtB_Area[4])
{
    const float extend       = 0.8f;
    const float uRadar_Range = 3.0f;

    float rx_min = -10.0f, rx_max = 10.0f;
    float ry_min = -10.0f, ry_max = 10.0f;
    Point_T slotObjC, slotObjD;
    memcpy(&slotObjC, &PlanDataVer.obj_slot[2][0], sizeof(Point_T));
    memcpy(&slotObjD, &PlanDataVer.obj_slot[2][2], sizeof(Point_T));

    slotB_Pos[0] = (PlanDataVer.obj_slot[0][2] + PlanDataVer.obj_slot[4][0]) * 0.5;
    slotB_Pos[1] = (PlanDataVer.obj_slot[0][3] + PlanDataVer.obj_slot[4][1]) * 0.5;
    if (PlanDataVer.slotshape <= PK_SLOT_RIGHT_PARA)
    {
        slotB_Pos[2] = PlanDataVer.finpoint[2];
    }
    //  左边车位B点方位加90、右边减少90，指向E点
    else if (PlanDataVer.slotshape == PK_SLOT_LEFT_VERT ||
             PlanDataVer.slotshape == PK_SLOT_LEFT_ANG_FORWARD ||
             PlanDataVer.slotshape == PK_SLOT_LEFT_ANG_REVERSE)
    {
        slotB_Pos[2] = PlanDataVer.finpoint[2] + PI / 2.0f;
    }
    else if (PlanDataVer.slotshape == PK_SLOT_RIGHT_VERT ||
             PlanDataVer.slotshape == PK_SLOT_RIGHT_ANG_FORWARD ||
             PlanDataVer.slotshape == PK_SLOT_RIGHT_ANG_REVERSE)
    {
        slotB_Pos[2] = PlanDataVer.finpoint[2] - PI / 2.0f;
    }

    float ryc = Project_PosTo1st_ry(slotB_Pos, (float *)&slotObjC);
    if (ryc > 0)
    {
        ry_max = rte_max(ryc + extend, ry_max);
    }
    if (ryc < 0)
    {
        ry_max = rte_min(ryc - extend, ry_min);
    }

    float ryd = Project_PosTo1st_ry(slotB_Pos, (float *)&slotObjD);
    if (ryd > 0)
    {
        ry_max = rte_max(ryd + extend, ry_max);
    }
    if (ryd < 0)
    {
        ry_max = rte_min(ryd - extend, ry_min);
    }

    float ryv = Project_PosTo1st_ry(slotB_Pos, PlanDataVer.stpoint);
    if (ryv > 0)
    {
        ry_max = rte_max(ryv + (extend + VEHICLE_WID / 2.0f + uRadar_Range), ry_max);
    }
    if (ryv < 0)
    {
        ry_max = rte_min(ryv - (extend + VEHICLE_WID / 2.0f + uRadar_Range), ry_min);
    }

    float rxv    = Project_PosTo1st_rx(slotB_Pos, PlanDataVer.stpoint);
    float rxvmax = rxv + (VEHICLE_LEN - REAR_SUSPENSION + extend);
    float rxvmin = rxv - (VEHICLE_LEN - REAR_SUSPENSION + extend);
    rx_min       = rte_min(rxvmin, rx_min);
    rx_max       = rte_max(rxvmax, rx_max);

    // ry-min/ry-max 在车身半宽到 车身到B点距离 + 侧向雷达最大探测距离
    rect_PtB_Area[0] = rx_min;
    rect_PtB_Area[1] = ry_min;
    rect_PtB_Area[2] = rx_max;
    rect_PtB_Area[3] = ry_max;
    return 1;
}

static void UpdateObjByAvmObj(PlanDataCase PlanData_Ver[1])
{
    auto cfgData = PathPlanCfg::getInstance();
    if (cfgData[0].amvobj_check != 0)
    {
        AvmObstacle_T obstacle;
        RTE_PK_SensorFusion_Get_Avm_Obstacle(&obstacle); // 视觉障碍物数据
        // 加入视觉障碍物
        for (int i = 0; i < SF_OBJ_NUM; i++)
        {
            if (obstacle.obj_valid[i] == 1 &&
                PlanData_Ver[0].obj_danger_num < SF_OBJ_NUM * 2 - 1)
            {
                if (sqrt(
                        pow(obstacle.central_point[i].x - PlanData_Ver[0].stpoint[0], 2) +
                        pow(obstacle.central_point[i].y - PlanData_Ver[0].stpoint[1],
                            2)) > 20.0f)
                {
                    obstacle.obj_valid[i] = 0;
                    continue;
                }
                memcpy(PlanData_Ver[0].obj_danger[PlanData_Ver[0].obj_danger_num],
                       &obstacle.obj[i], 4 * sizeof(float));

                PlanData_Ver[0].obj_danger_dir[PlanData_Ver[0].obj_danger_num] =
                    Project_PosTo1st_ry(PlanData_Ver[0].stpoint,
                                        (float *)&obstacle.central_point) > 0
                        ? 1
                        : 2;

                PlanData_Ver[0].obj_danger_num++;
            }
        }
    }
}

// calculate the side dist between current vehicle's pos and slotobj
float Cal_Slot_SideDist_CurPos(int slotshap, float targpos[3], float curpos[4])
{
    float slotVecTheta = targpos[2]; //  along the direction of searching slots
    float TargVec[2], LeftVertVec[2];
    float OuterPos[3]; //  the outside pos of the targpos
    int i;
    float ry;

    if (slotshap == 3 || slotshap == 5 || slotshap == 6)
    {
        slotVecTheta += PI / 2.0f;
    }
    else if (slotshap > 2)
    {
        slotVecTheta -= PI / 2.0f;
    }

    TargVec[0]     = cosf(targpos[2]);
    TargVec[1]     = sinf(targpos[2]);
    LeftVertVec[0] = -TargVec[1];
    LeftVertVec[1] = TargVec[0];
    if (slotshap == 1)
    {
        for (i = 0; i < 2; i++)
        {
            OuterPos[i] = targpos[i] - LeftVertVec[i] * VEHICLE_WID / 2.0f;
        }
    }
    else if (slotshap == 2)
    {
        for (i = 0; i < 2; i++)
        {
            OuterPos[i] = targpos[i] + LeftVertVec[i] * VEHICLE_WID / 2.0f;
        }
    }
    else // verticle slot
    {
        for (i = 0; i < 2; i++)
        {
            OuterPos[i] = targpos[i] + TargVec[i] * (VEHICLE_LEN - REAR_SUSPENSION);
        }
    }

    OuterPos[2] = slotVecTheta;
    ry          = Project_PosTo1st_ry(OuterPos, curpos);
    return ry;
}

/*********************************************************************
Function description : PathPlan_RangeDataProcess-->> The pre-process for path plan

Calibration state    : NO -->>
Edition              : 2017/6/6  1.0
Input list:
Output list:
return:  return_sta>0: pass the range check；return_sta=0: out of range；return_sta = -1:
invalid slot can't plan;
**********************************************************************/
int PathPlan_RangeDataProcess(PlanDataCase PlanData_Ver[1], const SlotInfo_T &slot,
                              const int FS_ObjDir[FS_OBJ_ARR_NUM],
                              const float FS_Obj[FS_OBJ_ARR_NUM][4],
                              const int FS_ObjDir_Rear[FS_OBJ_ARR_NUM],
                              const float FS_Obj_Rear[FS_OBJ_ARR_NUM][4])
{
    float ry, ry_CDU_show; // rtheta_fin,finpoint project to stpos
    int return_sta         = 1;
    float rect_pr_possb[4] = {0};

    float swell_ABEF                   = 0.3f;
    const float PlanRange_add_side_CDU = 2.0f; //  m

    SlotObj_Convert_float(slot.slotobj, PlanData_Ver[0].obj_slot);
    memcpy(&PlanData_Ver[0].pointb_danger, &slot.slotobj.ptB, sizeof(Point_T));
    memcpy(&PlanData_Ver[0].pointf_danger, &slot.slotobj.ptE, sizeof(Point_T));

    PlanData_Ver[0].len   = VEHICLE_LEN;
    PlanData_Ver[0].width = VEHICLE_WID;
    PlanData_Ver[0].h     = REAR_SUSPENSION;

    float PlanRange_side_max =
        4.0f +
        PlanData_Ver[0].width / 2.0f; //     0912: fit for CDU display and more safe ;
    float PlanRange_ClearGap = 2.0f;

    float rx_fin =
        Project_PosTo1st_rx(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint); //
    float ry_fin =
        Project_PosTo1st_ry(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint); //

    // 20180702：this judgement method is not stable
    PlanData_Ver[0].left_fac = (ry_fin > 0) ? 1.0 : -1.0;

    //////////////////////////////////////////////////////////////////////////////////
    PlanData_Ver[0].pointb_danger[0] = PlanData_Ver[0].obj_slot[0][2]; //  slot point B
    PlanData_Ver[0].pointb_danger[1] = PlanData_Ver[0].obj_slot[0][3]; //  slot point B
    PlanData_Ver[0].pointf_danger[0] = PlanData_Ver[0].obj_slot[3][2]; //  slot point E
    PlanData_Ver[0].pointf_danger[1] = PlanData_Ver[0].obj_slot[3][3]; //  slot point E

    //  recheck slot shap
    if (PlanData_Ver[0].slotshape <=
        PK_SLOT_RIGHT_VERT) // not recheck the slotshap for angle slots
    {
        PlanData_Ver[0].slotshape =
            Cal_SlotShap(PlanData_Ver[0].finpoint, PlanData_Ver[0].obj_slot,
                         &PlanData_Ver[0].left_fac);
    }

    float PlanRange_line_min, PlanRange_line_max; // PlanRange_side_min,
    if (PlanData_Ver[0].slotshape == PK_SLOT_LEFT_PARA ||
        PlanData_Ver[0].slotshape == PK_SLOT_RIGHT_PARA) //  parallel slot
    {
        PlanRange_line_max =
            15.0f + PlanData_Ver[0].len;          // 18.0f -> 15 fit with CDU display area
        PlanRange_line_min = PlanData_Ver[0].len; // for parallel slot
        PlanData_Ver[0].pointf_danger[2] =
            PlanData_Ver[0].finpoint[2]; //  along the direction while searching slot
        PlanData_Ver[0].pointb_danger[2] = PlanData_Ver[0].finpoint[2];
    }
    else
    {
        PlanRange_line_max =
            15.0f; // 18.0f+ PlanData_Ver[0].width -> 15 fit with CDU display area
        PlanRange_line_min = PlanData_Ver[0].width;
        PlanData_Ver[0].pointf_danger[2] =
            PlanData_Ver[0].finpoint[2] +
            PlanData_Ver[0].left_fac * PI /
                2.0f; //  along the direction while searching slot
        PlanData_Ver[0].pointb_danger[2] = PlanData_Ver[0].pointf_danger[2];
    }

    PlanData_Ver[0].close_side_dist =
        fabsf(
            Project_PosTo1st_ry(PlanData_Ver[0].finpoint, PlanData_Ver[0].obj_slot[2])) -
        PlanData_Ver[0].width / 2.0f;

    //////////////////////////////////////////////////////////////////////////////////
    // curpos pos relative to pointf_pos
    ry = Cal_Slot_SideDist_CurPos(
        PlanData_Ver[0].slotshape, PlanData_Ver[0].finpoint,
        PlanData_Ver[0].stpoint); //  20181023: use the outer point of targpos
    ry_CDU_show =
        Project_PosTo1st_ry(PlanData_Ver[0].stpoint, PlanData_Ver[0].pointb_danger);

    // 1、获取路线规划区域
    float slotB_Pos[3];
    Get_Plan_RangeArea(PlanData_Ver[0], slotB_Pos, rect_pr_possb);

    // 2、检查车位有效性（可泊空间是否满足泊车要求）
    return_sta = PathPlan_UpdateSlotSwell(PlanData_Ver[0]);
    if (return_sta <= 0)
    {
        printf("PathPlan_RangeDataProcess0 UpdateSlotSwell failed! %d\n",
               slot.slot_index);
    }

    if (return_sta == 1)
    {
        if (fabsf(ry_CDU_show) < PlanRange_side_max + PlanRange_add_side_CDU &&
            fabsf(ry) < PlanRange_side_max &&
            ry * ry_fin <
                0) //&& rx_fin>-PlanRange_line_max && fabsf(rtheta)<PlanRange_dtheta_max
                   //&& rx_fin<-PlanRange_line_min)//&& fabsf(ry)>PlanRange_side_min
        {
            return_sta = 1;
        }
        else if (rx_fin < -(PlanRange_line_max + PlanRange_ClearGap) ||
                 rx_fin > -(PlanRange_line_min - PlanRange_ClearGap) ||
                 fabsf(ry) > PlanRange_side_max + PlanRange_ClearGap / 2.0f)
        {
            log_info("PathPlan_RangeDataProcess1 failed!\n");
        }
        else
        {
            // printf("PathPlan_UpdateSlotSwell4 failed!\n");
            // return_sta = 0;
        }
    }

    // 3、存入障碍物
    PathPlan_BuildSlotObjDanger(PlanData_Ver, slotB_Pos, rect_pr_possb, FS_ObjDir, FS_Obj,
                                FS_ObjDir_Rear, FS_Obj_Rear);

    if (PlanData_Ver[0].slotshape <= PK_SLOT_RIGHT_PARA)
    {
        float bcrx =
            Project_PosTo1st_rx(PlanData_Ver[0].finpoint, PlanData_Ver[0].obj_slot[1]);
        float derx =
            Project_PosTo1st_rx(PlanData_Ver[0].finpoint, PlanData_Ver[0].obj_slot[3]);

        LineSeg_T parkBars[2];
        int counter = PathPlan_BuildObjsByStopper(slot, parkBars);
        for (int i = 0; i < counter; i++)
        {
            if (PlanData_Ver[0].obj_danger_num >= FS_OBJ_ARR_NUM)
            {
                break;
            }

            float rx =
                Project_PosTo1st_rx(PlanData_Ver[0].finpoint, (float *)&parkBars[i].pt1);

            // 在BC/DE之间
            if ((rx * bcrx > 0.001f && fabs(rx) < fabs(bcrx)) ||
                (rx * derx > 0.001f && fabs(rx) < fabs(derx)))
            {
                memcpy(PlanData_Ver[0].obj_danger[PlanData_Ver[0].obj_danger_num],
                       &parkBars[0], sizeof(LineSeg_T));
                PlanData_Ver[0].obj_danger_dir[PlanData_Ver[0].obj_danger_num] =
                    (slot.slotshap == PK_SLOT_LEFT_PARA) ? 1 : 2;
                PlanData_Ver[0].obj_danger_num++;
            }
        }
    }

    //  0906: for obs in slot ingored by slotdetect!
    int result = PathPlan_UpdateSlotSwell(PlanData_Ver[0]);
    if (result <= 0)
    {
        return_sta = -2;
        log_info("PathPlan_RangeDataProcess2 UpdateSlotSwell failed! %d\n",
                 slot.slot_index);
    }

    // 4、根据停车位形状和距离信息，对停车位边界进行膨胀计算，更新停车位可泊空间。
    float avmDist[3];
    PathPlan_UpdateSideDist(PlanData_Ver, slot, avmDist);

    swell_ABEF = PK_PathPlan_SlotObj_To_stPos_By_Swell(PlanData_Ver, avmDist);

    // 更新视觉障碍物
    UpdateObjByAvmObj(PlanData_Ver);

    // 5、根据安全距离和车辆位置偏移延长障碍物线段
    PathPlan_ExpandObjBySwell(swell_ABEF, avmDist, PlanData_Ver);

    PlanData_Ver[0].rect_ds_max =
        PlanData_Ver
            ->rear_safe_dist;  // 0815: fix the error:  VEHICLE_WID/2.0f-REAR_SUSPEND
    PlanData_Ver[0].swell = 0; //  have already expended obstacles,so the vehicle's
                               //  expention can be set to zero

    PlanData_Ver[0].pointb_danger[0] = PlanData_Ver[0].obj_slot[0][2]; //  slot point B
    PlanData_Ver[0].pointb_danger[1] = PlanData_Ver[0].obj_slot[0][3]; //  slot point B
    PlanData_Ver[0].pointf_danger[0] = PlanData_Ver[0].obj_slot[3][2]; //  slot point E
    PlanData_Ver[0].pointf_danger[1] = PlanData_Ver[0].obj_slot[3][3]; //  slot point E
    return return_sta;
}

/**
 * @brief 规划针对断头车位的倒车路径
 *
 * 生成从起始点（`stpoint`）到断头车位目标位置的路径，考虑障碍物分布、车位边界、转向方向等因素。
 *
 * @param stpoint 当前车辆起始位置坐标，包含 3 个元素的数组：[x, y, theta]。
 * @param PlanData_Ver 包含当前规划数据的结构体数组，包括危险点、目标点、车位参数等信息。
 * @param Tem_Exp_traj 存储生成的探索轨迹的二维数组，包含轨迹点的详细信息。
 * @param explorfront 探索方向标志，默认为 1（探索前方路径）；为 0 时表示限制探索。
 *
 * @return 返回生成的轨迹点数量，如果路径不可行或未能生成有效轨迹，返回 0。
 */
static int PathPlan_Rear_Base_on_ObjSlot(float stpoint[3], PlanDataCase PlanData_Ver[1],
                                         float traj[TRAJITEM_LEN], float dir)
{
    const float MAX_TRAJ_3    = 5.0f;
    const float MIN_TRAJ_3    = 0.3f;
    const float MAX_RX_OFFSET = 1.8f;
    const float MIN_RX_OFFSET = 0.8f;

    float ry     = Project_PosTo1st_ry(stpoint, PlanData_Ver[0].finpoint);
    float rtheta = Project_PosTo1st_rtheta(stpoint, PlanData_Ver[0].finpoint);

    float Rr = Rrmin + 0.4f;
    PK_CopyPos(traj, stpoint);

    auto dist = rte_max(3.0 * fabs(rtheta) * Rr, 2.0f);
    traj[3]   = dist * dir;
    traj[4]   = -sign(ry) * Rr * dir;
    PathPlan_UpdateTrajLen(traj, PlanData_Ver[0].finpoint, PlanData_Ver[0].slotshape);

    PK_PathVerify(traj, 5, PlanData_Ver[0].obj_slot, 0);
    if (fabs(traj[3]) > MIN_TRAJ_3)
    {
        PK_PathVerify(traj, PlanData_Ver[0].obj_danger_num, PlanData_Ver[0].obj_danger,
                      0.1);
    }

    float endPos[3];
    PK_Get_Path_EndPos(traj, endPos);
    float rsx = Project_PosTo1st_rx(PlanData_Ver[0].finpoint, traj);
    float rfx = Project_PosTo1st_rx(PlanData_Ver[0].finpoint, endPos);
    if (fabs(traj[3]) < MIN_TRAJ_3 || (rsx > MAX_RX_OFFSET * VEHICLE_LEN && rfx > rsx) ||
        (rsx < MIN_RX_OFFSET * VEHICLE_LEN && rfx < rsx))
    {
        return -1;
    }

    if (fabs(traj[3]) > MAX_TRAJ_3)
    {
        traj[3] = MAX_TRAJ_3 * sign(traj[3]);
    }

    while ((fabs(traj[3]) > MIN_TRAJ_3) &&
           ((rfx < MIN_RX_OFFSET * VEHICLE_LEN && rsx > rfx) ||
            (rfx > MAX_RX_OFFSET * VEHICLE_LEN && rfx > rsx)))
    {
        traj[3] = (fabs(traj[3]) - 0.9 * MIN_TRAJ_3) * sign(traj[3]);
        PK_Get_Path_EndPos(traj, endPos);
        rfx = Project_PosTo1st_rx(PlanData_Ver[0].finpoint, endPos);
    }

    if (fabs(traj[3]) < MIN_TRAJ_3)
    {
        return -1;
    }

    PK_Get_Path_EndPos(traj, endPos);
    float rsy = Project_PosTo1st_ry(PlanData_Ver[0].finpoint, endPos);
    float rst = Project_PosTo1st_rtheta(PlanData_Ver[0].finpoint, endPos);
    if (sign(rsy) * sign(rst) >= 0)
    {
        return 1;
    }

    return 0;
}
static void *PathPlanRun(void *args)
{
    struct timeval start, end; // 用于记录路径规划开始和结束时间

    while (1)
    {
        uint32_t count = PathPlanConstCfg::pathplan_period; // 默认规划周期 300ms

        // 如果当前模块状态是泊车状态中，则使用更高频率规划（80ms）
        if (RTE_PK_StateManage_Get_ModuleCom_PathPlan() == MCOM_ON_PARKING)
        {
            count = 80; // 提高规划频率
        }

        usleep(count * 1000); // 按规划周期休眠（微秒 -> 毫秒 * 1000）

        // 记录开始时间
        gettimeofday(&start, 0);

        // 实际路径规划逻辑函数
        PK_PathPlanRun();

        // 记录结束时间
        gettimeofday(&end, 0);

        // 计算本次规划耗时（ms）
        uint32_t runMs =
            ((end.tv_sec - start.tv_sec) * 1000 * 1000 + end.tv_usec - start.tv_usec) /
            1000;

        // 如果规划耗时过长，打印警告
        if (runMs > count * 0.75) // 超过周期 75% 即认为繁忙
        {
            printf("path plan busy! run time is %d\n", runMs);
        }
    }
}

/**
 * @brief 路径规划接口函数
 *
 */
void PK_PathPlan(void)
{
    char config[64];
    PathPlanCfg *cfgData = PathPlanCfg::getInstance();
    if (PK_Calibration_GetDebugValue("cfg_use_avm_obj", config) > 0)
    {
        int value               = atoi(config);
        cfgData[0].amvobj_check = ((value & 0x02) != 0) ? 1 : 0;
    }

    if (PK_Calibration_GetDebugValue("cfg_plan_front_slot", config) > 0)
    {
        int value                  = atoi(config);
        cfgData[0].plan_front_slot = (value & 0x01);
    }

    PK_InitJsonReport();
    // ASyncRun(PathPlanRun, NULL, "apa_plan");
}