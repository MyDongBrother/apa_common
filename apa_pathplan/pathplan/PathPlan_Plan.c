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
#include "PathPlan_ParaPlan.h"
#include "PathPlan_Tools.h"
#include "PK_StateManage.h"
#include "Record_Log.h"

inline static void PathPlanType(uint8_t level, uint8_t type, int num)
{
    PathPlan_FootPrints(0, level, type, num);
}

int PathPlan_SpaceExplorer_Y(PlanDataCase &planData, float trajs[][TRAJITEM_LEN],
                             float middy)
{
    int head_to = 0;
    float mid_straight_ds_min;
    const float Narrow_AnotherDist = 2.2f;

    float ry = Project_PosTo1st_ry(planData.stpoint, planData.pointf_danger);
    float Nearest_dy =
        Get_AnotherObs_Nearest_dy_to_CurPos(planData.stpoint, planData.slotshape,
                                            planData.obj_danger, planData.obj_danger_num);

    if (planData.slotshape > 2) // vert
    {
        mid_straight_ds_min = 0.8;
        if (Nearest_dy <= Narrow_AnotherDist && fabs(ry) > 2.0)
        {
            head_to = 1;
        }
        if (fabs(ry) <= 2.0 && Nearest_dy > Narrow_AnotherDist)
        {
            head_to = -1;
        }
    }
    else
    {
        mid_straight_ds_min = 1.5 * middy;
        if (Nearest_dy <= Narrow_AnotherDist && fabs(ry) > 1.0)
        {
            head_to = 1;
        }
    }

    if (head_to == 0)
    {
        return 0;
    }

    float rtheta = asinf(middy / (3 * mid_straight_ds_min));
    float r      = middy / (3 * (1 - cosf(rtheta)));
    if (r < Rrmin)
    {
        return 0;
    }

    memcpy(trajs[0], planData.stpoint, 3 * sizeof(float));
    trajs[0][3] = fabs(rtheta * r);
    trajs[0][4] = head_to * sign(ry) * r;

    PK_Get_Path_EndPos(trajs[0], trajs[1]);
    trajs[1][3] = mid_straight_ds_min;
    trajs[1][4] = 0;

    PK_Get_Path_EndPos(trajs[1], trajs[2]);
    trajs[2][3] = trajs[0][3];
    trajs[2][4] = -trajs[0][4];

    PK_Get_Path_EndPos(trajs[2], trajs[3]);

    return 3;
}

static int PathPlan_Base_on_ObjSlot_BC(float stpoint[3], PlanDataCase PlanData_Ver[1],
                                       float Exp_traj[][TRAJITEM_LEN], int explorNum = 3)
{
    float tempointf[3], tempoint[3], tempoint1[3], temdist;
    int steering_direction = 0;

    float ry = Project_PosTo1st_ry(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint);
    float fx =
        Project_PosTo1st_rx(PlanData_Ver[0].finpoint, PlanData_Ver[0].pointf_danger);

    if (PlanData_Ver[0].left_fac == sign(ry))
    {
        PK_CopyPos(tempoint1, PlanData_Ver[0].pointf_danger);
        temdist            = PlanData_Ver[0].front_dist;
        steering_direction = 1;
    }
    else
    {
        PK_CopyPos(tempoint1, PlanData_Ver[0].pointb_danger);
        tempoint1[2] *= -1;
        temdist            = PlanData_Ver[0].rear_dist;
        steering_direction = -1;
    }

    if (temdist > 1.0 || fx < VEHICLE_LEN - REAR_SUSPENSION)
    {
        RevConvert(PlanData_Ver[0].finpoint, tempoint1, tempoint);
        tempoint[0] = (fx < VEHICLE_LEN - REAR_SUSPENSION)
                          ? sign(tempoint[0]) * (VEHICLE_LEN - REAR_SUSPENSION)
                          : tempoint[0];
        tempoint[1] =
            PlanData_Ver[0].front_dist > 1.0 ? sign(tempoint[1]) * 1.0 : tempoint[1];
        Convert(PlanData_Ver[0].finpoint, tempoint, tempointf);
        tempointf[2] = tempoint1[2];
    }
    else
    {
        memcpy(tempointf, tempoint1, sizeof(float) * 3);
    }

    int Exp_PathNum = 0, TrajNum_NS = 0;
    float endPos[3];
    float Rr      = Rrmin + 0.4;
    float dy      = fabs(Project_PosTo1st_ry(stpoint, tempointf));
    float dx      = Project_PosTo1st_rx(stpoint, tempointf);
    float rtheta  = Project_PosTo1st_rtheta(stpoint, PlanData_Ver[0].finpoint);
    float ftheta  = Project_PosTo1st_rtheta(tempointf, stpoint);
    float rrtheta = fabs(acosf((1 - fabs(dy) / Rr)));

    float obj_all[SF_OBJ_NUM * 2 + 5][4];
    int obj_all_num = 0;
    // 统一障碍物
    memcpy(obj_all, PlanData_Ver[0].obj_danger,
           sizeof(float) * PlanData_Ver[0].obj_danger_num * 4);
    obj_all_num = PlanData_Ver[0].obj_danger_num;
    memcpy(&obj_all[obj_all_num], PlanData_Ver[0].obj_slot, sizeof(float) * 5 * 4);
    obj_all_num += 5;

    memcpy(Exp_traj[1], stpoint, sizeof(float) * 3);
    Exp_traj[1][3] = -rrtheta * Rr;
    Exp_traj[1][4] = steering_direction * PlanData_Ver[0].left_fac * Rr;

    PK_Get_Path_EndPos(Exp_traj[1], endPos);

    float s_dist = dx - Project_PosTo1st_rx(stpoint, endPos);
    for (int i = 0; i < 10; i++)
    {
        float mtheta = Project_PosTo1st_rtheta(tempointf, endPos);
        float d_dist =
            (fabs(PlanData_Ver[0].width / (2 * sinf(mtheta))) + 0.2 * (i + 1)) /
            (ftheta != 0 ? cosf(fabs(ftheta)) : 1);

        memcpy(Exp_traj[0], stpoint, sizeof(float) * 3);
        Exp_traj[0][3] = s_dist - fabsf(d_dist);
        Exp_traj[0][3] =
            Exp_traj[0][3] > 0 && Exp_traj[0][3] < 0.5f ? 0.5f : Exp_traj[0][3];
        Exp_traj[0][4] = 0;
        PathPlan_UpdateTrajLen(Exp_traj[0], PlanData_Ver[0].finpoint,
                               PlanData_Ver[0].slotshape);

        PK_PathVerify(Exp_traj[0], obj_all_num, obj_all, 0);
        PK_Get_Path_EndPos(Exp_traj[0], Exp_traj[1]);

        TrajNum_NS = 1;
        if (i < 2)
        {
            Exp_PathNum = PK_PathPlan_BetterTra(
                Exp_traj[1], PlanData_Ver[0].finpoint, obj_all_num, obj_all,
                &Exp_traj[TrajNum_NS], PlanData_Ver[0].swell);
            if (Exp_PathNum > 0)
            {
                PathPlanType(3, 1, Exp_PathNum);
                Exp_PathNum += TrajNum_NS;
                break;
            }
        }

        Exp_traj[1][3] = -2.0 * fabs(rtheta) * Rr;
        Exp_traj[1][4] = steering_direction * PlanData_Ver[0].left_fac * Rr;
        PathPlan_UpdateTrajLen(Exp_traj[1], PlanData_Ver[0].finpoint,
                               PlanData_Ver[0].slotshape);

        PK_PathVerify(Exp_traj[1], obj_all_num, obj_all, 0);
        PK_Get_Path_EndPos(Exp_traj[1], Exp_traj[2]);

        TrajNum_NS     = 2;
        Exp_traj[1][3] = fabs(Exp_traj[1][3]) > fabs(rtheta) * Rr * 0.6
                             ? -fabs(rtheta) * Rr * 0.6
                             : Exp_traj[1][3];
        Exp_PathNum    = PathPlan_SpaceExplorer_Mid2Targ_Vert(
            Exp_traj[TrajNum_NS - 1], PlanData_Ver, &Exp_traj[TrajNum_NS], explorNum);
        Exp_PathNum = (Exp_PathNum > 0) ? TrajNum_NS + Exp_PathNum : 0;
        if (Exp_PathNum > 0 &&
            PK_Check_Path(Exp_PathNum, Exp_traj, stpoint, PlanData_Ver[0].finpoint,
                          PlanData_Ver[0].obj_danger_num, PlanData_Ver[0].obj_danger,
                          0) == PATHPLAN_SUCCESS)
        {
            PathPlanType(3, 2, Exp_PathNum);
            break;
        }
        else
        {
            Exp_PathNum = 0;
        }
    }

    return Exp_PathNum;
}

static int GetPath_By_Forward(float stpoint[5], PlanDataCase PlanData_Ver[1],
                              float Act_traj[][TRAJITEM_LEN], int *IsDirConnect)
{
    int line_num_dir = 0, TrajNum_NS = 0;
    float Act_traj_dir[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];

    float ry1    = Project_PosTo1st_ry(stpoint, PlanData_Ver[0].finpoint);
    float rtheta = Project_PosTo1st_rtheta(stpoint, PlanData_Ver[0].finpoint);
    // float fx1 = Project_PosTo1st_rx(stpoint, PlanData_Ver[0].pointf_danger);
    float rx1 = Project_PosTo1st_rx(stpoint, PlanData_Ver[0].finpoint);

    // 和车位基本垂直, 车辆已经过了E点
#if 0
    if (PlanData_Ver[0].slotshape >= PK_SLOT_LEFT_VERT &&
        PlanData_Ver[0].rear_dist < 1.0f && fx1 < 0 &&
        fabs(rtheta) > PI * 0.4f && fabs(rtheta) < PI * 0.6)
    {
        line_num_dir = PathPlan_Base_on_ObjSlot_BC(stpoint, PlanData_Ver, Act_traj_dir, 3);
        PathPlanType(2, 1, line_num_dir);
    }
#endif

    float oldstpoint[3];
    PK_CopyPos(oldstpoint, PlanData_Ver[0].stpoint);
    float Rr = Rrmin + 0.4f;

    float rrtheta = Project_PosTo1st_rtheta(PlanData_Ver[0].finpoint, stpoint);
    float rrx = Project_PosTo1st_rx(PlanData_Ver[0].finpoint, PlanData_Ver[0].stpoint);

    // 向车位另外一侧打方向
    for (float radius = -sign(ry1) * Rr; fabs(radius) < 30.0f && line_num_dir < 1;
         radius *= 1.2f) // 前
    {
        if (radius == 0)
        {
            radius = -PlanData_Ver[0].left_fac * Rr;
        }

        // 侧方位 或者夹角很大
        if (PlanData_Ver->slotshape < PK_SLOT_LEFT_VERT ||
            (fabs(rtheta) < PI * 0.4 || fabs(rtheta) > PI * 0.6))
        {
            break;
        }

        // 左右间距, rear_dist到BC距离 front_dist到DE距离
        if (PlanData_Ver[0].front_dist > 1.0 || PlanData_Ver[0].rear_dist > 1.0)
        {
            break;
        }

        // 夹角小于80度
        if (cosf(rrtheta) > cosf(80.0 * PI_RAD))
        {
            break;
        }

        // 远离车位
        if (rrx > VEHICLE_LEN - REAR_SUSPENSION + 0.5 * VEHICLE_WID + 1.5f)
        {
            break;
        }

        if (rx1 > 0)
        {
            break;
        }

        TrajNum_NS = 0;
        TrajNum_NS = GetPath_StToMidPos_Na2(stpoint, PlanData_Ver, Act_traj_dir, radius);
        if (TrajNum_NS > 0)
        {
            if (PlanData_Ver[0].slotshape >= PK_SLOT_LEFT_VERT)
            {
                line_num_dir = PathPlan_Base_on_ObjSlot_BC(Act_traj_dir[TrajNum_NS - 1],
                                                           PlanData_Ver,
                                                           &Act_traj_dir[TrajNum_NS - 1]);
            }

            PK_CopyPos(PlanData_Ver[0].stpoint, oldstpoint);

            line_num_dir = (line_num_dir > 0) ? (TrajNum_NS - 1 + line_num_dir) : 0;

            if (line_num_dir > 0 &&
                PK_Check_Path(line_num_dir, Act_traj_dir, stpoint,
                              PlanData_Ver[0].finpoint, PlanData_Ver[0].obj_danger_num,
                              PlanData_Ver[0].obj_danger, 0) == PATHPLAN_SUCCESS)
            {
                PathPlanType(2, 2, line_num_dir);
                break;
            }
            else
            {
                line_num_dir = 0;
            }
        }
    }

    // 没有过E点也探索
#if 0
    if (line_num_dir <= 0 && PlanData_Ver[0].slotshape >= PK_SLOT_LEFT_VERT &&
        fabs(rtheta) > PI * 0.4 && fabs(rtheta) < PI * 0.6 &&
        fx1 >= 0 && PlanData_Ver[0].rear_dist < 1.0)
    {
        line_num_dir = PathPlan_Base_on_ObjSlot_BC(stpoint, PlanData_Ver, Act_traj_dir, 3);
        PathPlanType(2, 8, line_num_dir);
    }
#endif

    if (line_num_dir < 1)
    {
        TrajNum_NS = 0;
        TrajNum_NS =
            GetPath_StToMidPos_Na2(stpoint, PlanData_Ver, Act_traj_dir, 0); // 直线向前
        if (TrajNum_NS > 0)
        {
            PK_CopyPos(PlanData_Ver[0].stpoint, Act_traj_dir[TrajNum_NS - 1]);
            if (PlanData_Ver[0].slotshape > 2)
            {
                line_num_dir = PathPlan_SpaceExplorer_Vert(
                    PlanData_Ver, &Act_traj_dir[TrajNum_NS - 1], IsDirConnect, 1);
                if (line_num_dir <= 0)
                {
                    line_num_dir = PathPlan_NarrowSpaceExplorer_Vert(
                        PlanData_Ver, &Act_traj_dir[TrajNum_NS - 1]);
                }
            }

            line_num_dir = (line_num_dir > 0) ? (TrajNum_NS - 1 + line_num_dir) : 0;
            PathPlanType(2, 4, line_num_dir);
            PK_CopyPos(PlanData_Ver[0].stpoint, oldstpoint);
            if (line_num_dir > 0 &&
                PK_Check_Path(line_num_dir, Act_traj_dir, stpoint,
                              PlanData_Ver[0].finpoint, PlanData_Ver[0].obj_danger_num,
                              PlanData_Ver[0].obj_danger, 0) != PATHPLAN_SUCCESS)
            {
                line_num_dir = 0;
            }
        }
    }

#if 0 // 向车位一侧探索
    if (line_num_dir < 1)
    {
        for (float radius = /*PlanData_Ver[0].left_fac*/sign(ry1) * Rrmin; fabs(radius) < 60.0; radius *= 1.2)//前
        {
            TrajNum_NS = 0;
            TrajNum_NS = GetPath_StToMidPos_Na2(PlanData_Ver, Act_traj_dir,radius);
            if (TrajNum_NS > 0)
            {
                PK_CopyPos(PlanData_Ver[0].stpoint, Act_traj_dir[TrajNum_NS - 1]);
                if (PlanData_Ver[0].slotshape > 2)
                {
                   line_num_dir = PathPlan_SpaceExplorer_Vert(PlanData_Ver, &Act_traj_dir[TrajNum_NS-1], IsDirConnect);
                }
                else
                {
                   line_num_dir = PK_PathPlan_SpaceExplorer_Para(PlanData_Ver, &Act_traj_dir[TrajNum_NS-1]);
                }
                line_num_dir = (line_num_dir > 0) ? (TrajNum_NS-1 + line_num_dir) : 0;
                if (line_num_dir > 0) { break; }
            }
            PK_CopyPos(PlanData_Ver[0].stpoint, oldstpoint);
        }
    }
    PK_CopyPos(PlanData_Ver[0].stpoint, oldstpoint);
#endif

    if (line_num_dir > 0)
    {
        memcpy(Act_traj[0], Act_traj_dir[0], PATH_ITEM_LEN * line_num_dir);
        line_num_dir = PathPlan_PathMerge(Act_traj, line_num_dir);
        return line_num_dir;
    }

    return 0;
}

/**
 * @brief 路径规划探索
 *
 * @param lineNum 当前路径点的数量
 * @param PlanData_Ver 存储路径规划的相关参数和数据
 * @param Act_traj 当前的路径数据，包含路径点和转向角度等信息
 * @param IsDirConnect 路径连接方向的标志，指示是否已经成功连接路径
 *
 * @return 返回优化后的路径点数量。如果优化失败，返回原始的路径点数量
 */
static int PathPlan_PathRevert1(PlanDataCase PlanData_Ver[1],
                                float Act_traj[][TRAJITEM_LEN])
{
    const float tryr[] = {Rrmin + 0.6f};
    float SlotBC_DE[2][4], SlotAB_EF[2][4];
    PK_PathPlan_Envi_obj_cut(PlanData_Ver[0].obj_slot, 1.0f, SlotBC_DE);  // get BC,DE
    memcpy(SlotAB_EF[0], PlanData_Ver[0].obj_slot[0], 4 * sizeof(float)); // get AB
    memcpy(SlotAB_EF[1], PlanData_Ver[0].obj_slot[4], 4 * sizeof(float)); // get EF

    float ry = Project_PosTo1st_ry(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint);
    float rtheta =
        Project_PosTo1st_rtheta(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint);
    float Act_traj_dir[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
        Exp_finpoint_rear[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];

    PK_CopyPos(Exp_finpoint_rear[0], PlanData_Ver[0].finpoint);

    float startds = 1.5f;
    float endds   = 4.5f;
    float step    = 0.3f;
    if (PlanData_Ver[0].is_vision_slot == 2)
    {
        startds = Parkout_off_dx - step;
        endds   = Parkout_off_dx;
    }

    for (size_t i = 0; i < PATHPLAN_ARRSIZE(tryr); i++)
    {
        for (float ds = endds; ds > startds; ds = ds - step)
        {
            Exp_finpoint_rear[0][3] = rte_min(ds, PATH_LENGTH_MAX - 0.5f);
            Exp_finpoint_rear[0][4] = 0;
            PathPlan_UpdateTrajLen(Exp_finpoint_rear[0], PlanData_Ver[0].finpoint,
                                   PlanData_Ver[0].slotshape);
            PK_PathVerify(Exp_finpoint_rear[0], 5, PlanData_Ver[0].obj_slot, 0.0);
            if (fabs(Exp_finpoint_rear[0][3]) < 0.6)
            {
                continue;
            }

            PK_Get_Path_EndPos(Exp_finpoint_rear[0], Exp_finpoint_rear[1]);
            Exp_finpoint_rear[1][3] =
                rte_min(fabs(rtheta) * tryr[i], PATH_LENGTH_MAX - 0.5f);
            Exp_finpoint_rear[1][4] = sign(ry) * tryr[i];

            PK_PathVerify(Exp_finpoint_rear[1], 5, PlanData_Ver[0].obj_slot, 0.0);
            PathPlan_UpdateTrajLen(Exp_finpoint_rear[1], PlanData_Ver[0].finpoint,
                                   PlanData_Ver[0].slotshape);
            PK_PathVerify(Exp_finpoint_rear[1], PlanData_Ver[0].obj_danger_num,
                          PlanData_Ver[0].obj_danger, 0.0);
            if (fabs(Exp_finpoint_rear[1][3]) < 0.6)
            {
                continue;
            }

            int line_num_dir = PK_PathPlan_TraExplorer_Con(
                Exp_finpoint_rear[1], PlanData_Ver[0].stpoint, 2, SlotAB_EF, Act_traj_dir,
                0, 0, PlanData_Ver[0].swell, 0.0);
            if (line_num_dir <= 0)
            {
                continue;
            }

            auto result =
                Path_Check_Connect(PlanData_Ver[0].stpoint, Exp_finpoint_rear[1],
                                   Act_traj_dir, line_num_dir);
            if (result == 0)
            {
                continue;
            }

            PathPlan_ReverseTraject(Exp_finpoint_rear[0]);
            PK_CopyTra(Act_traj_dir[line_num_dir], Exp_finpoint_rear[0]);
            line_num_dir += 1;

            result =
                PK_Check_Path(line_num_dir, Act_traj_dir, PlanData_Ver[0].stpoint,
                              PlanData_Ver[0].finpoint, PlanData_Ver[0].obj_danger_num,
                              PlanData_Ver[0].obj_danger, 0);
            if (result != PATHPLAN_SUCCESS)
            {
                continue;
            }
            memcpy(Act_traj[0], Act_traj_dir[0], PATH_ITEM_LEN * line_num_dir);
            return line_num_dir;
        }
    }
    return 0;
}

/**
 * @brief 路径规划探索
 *
 * @param lineNum 当前路径点的数量
 * @param PlanData_Ver 存储路径规划的相关参数和数据
 * @param Act_traj 当前的路径数据，包含路径点和转向角度等信息
 * @param IsDirConnect 路径连接方向的标志，指示是否已经成功连接路径
 *
 * @return 返回优化后的路径点数量。如果优化失败，返回原始的路径点数量
 */
static int PathPlan_PathTryOptimize(int lineNum, PlanDataCase PlanData_Ver[1],
                                    float Act_traj[][TRAJITEM_LEN], int *IsDirConnect)
{
    int line_num_dir = 0, line_num_add = 0;
    float Act_traj_dir[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
        Exp_stpoint_rear[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
    const float rtheta =
        Project_PosTo1st_rtheta(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint);

    float midpos[5];
    memcpy(midpos, PlanData_Ver[0].stpoint, sizeof(PlanData_Ver[0].stpoint));

    /*路径探索1**********************************************************************/
    if (PlanData_Ver[0].slotshape > PK_SLOT_RIGHT_PARA)
    {
        // 离车位比较近才可以向对向转
        float frx =
            Project_PosTo1st_rx(PlanData_Ver[0].finpoint, PlanData_Ver[0].stpoint);
        float srx =
            Project_PosTo1st_rx(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint);

        if (frx < VEHICLE_LEN - REAR_SUSPENSION + 0.5 * VEHICLE_WID + 1.8f &&
            ((srx < 0 && RTE_PK_SM_GetApaMode() != PARKOUT_MODE) ||
             RTE_PK_SM_GetApaMode() == PARKOUT_MODE))
        {
            line_num_dir = PathPlan_PathRevert1(PlanData_Ver, Act_traj_dir);
            PathPlanType(1, 1, line_num_dir);
        }
    }

    /*路径探索2**********************************************************************/
    if (line_num_dir < 1)
    {
        line_num_dir = GetPath_By_Forward(PlanData_Ver[0].stpoint, PlanData_Ver,
                                          Act_traj_dir, IsDirConnect);
        PathPlanType(1, 2, line_num_dir);
    }

    /*路径探索3**********************************************************************/
    if (line_num_dir < 1 && fabs(rtheta) < PI * 0.6 && fabs(rtheta) > PI * 0.4)
    {
        for (int count = 2; count < MAX_PARKED_SLOTS; count++)
        {
            line_num_add =
                PathPlan_SpaceExplorer_Y(PlanData_Ver[0], Act_traj_dir, count * 0.2);
            if (line_num_add <= 0)
            {
                continue;
            }

            line_num_dir = GetPath_By_Forward(Act_traj_dir[line_num_add], PlanData_Ver,
                                              &Act_traj_dir[line_num_add], IsDirConnect);
            if (line_num_dir > 0)
            {
                int /*line_num_add2 = 0,*/ i = 0;
                float mid_traj_pos[5];
                float mid_straight_min = 0.5;
                for (i = 0; i < 5; i++)
                {
                    if (Act_traj_dir[line_num_add + i][4] != 0)
                    {
                        PK_CopyPos(mid_traj_pos, Act_traj_dir[line_num_add + i]);
                        break;
                    }
                }
                mid_traj_pos[3] = mid_straight_min;
                mid_traj_pos[4] = 0;
                PathPlan_ReverseTraject(mid_traj_pos);

                if (Project_PosTo1st_rx(PlanData_Ver[0].stpoint, mid_traj_pos) < 0)
                {
                    int line_num_add2 = PK_PathTryConnect_TwoCon(
                        0, mid_traj_pos, PlanData_Ver, Exp_stpoint_rear);
                    if (line_num_add2 > 0)
                    {
                        PK_CopyTra(Exp_stpoint_rear[line_num_add2], mid_traj_pos);
                        line_num_add2 += 1;

                        memcpy(&Exp_stpoint_rear[line_num_add2],
                               &Act_traj_dir[line_num_add + i],
                               (line_num_dir - i) * PATH_ITEM_LEN);
                        line_num_dir -= i;
                        line_num_dir += line_num_add2;
                        memcpy(Act_traj_dir, Exp_stpoint_rear,
                               line_num_dir * PATH_ITEM_LEN);
                        PathPlanType(1, 3, line_num_dir);
                        break;
                    }
                }
                line_num_dir += line_num_add;
                PathPlanType(1, 4, line_num_dir);
                break;
            }
        }
    }

    if (line_num_dir > 0)
    {
        memcpy(Act_traj[0], Act_traj_dir[0], PATH_ITEM_LEN * line_num_dir);
    }

    return line_num_dir;
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

static int PathPlan_RevertSlotPlan(PlanDataCase planData[1],
                                   float traject[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                                   int *IsDirConnect)
{
    float startPos[3], temp[TRAJITEM_LEN];
    PK_CopyPos(startPos, planData[0].stpoint);

    int count  = 0;
    int failed = 0;
    float dir  = -1.0f; //(have_obj_ahead == 1) ? 1.0 : -1.0;

    int line_num_dir = 0;
    while (count < 10 && failed < 2 && line_num_dir <= 0)
    {
        int pathNum = PathPlan_Rear_Base_on_ObjSlot(startPos, planData, temp, dir);
        if (pathNum == 1)
        {
            failed = 0;

            float tempTry[TRAJITEM_LEN], optTrajs[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
            PK_CopyTra(tempTry, temp);
            float pointLen = rte_min(0.6, fabs(tempTry[3]));
            while (pointLen < fabs(temp[3]) + 0.001f)
            {
                float tryEnd[4], oldStart[4];
                tempTry[3] = pointLen * sign(tempTry[3]);
                PK_Get_Path_EndPos(tempTry, tryEnd);

                memcpy(oldStart, planData[0].stpoint, 4 * sizeof(float));
                memcpy(planData[0].stpoint, tryEnd, sizeof(tryEnd));

                auto trajNum =
                    PathPlan_PathTryOptimize(0, planData, optTrajs, IsDirConnect);
                memcpy(planData[0].stpoint, oldStart, 4 * sizeof(float));

                if (trajNum > 0)
                {
                    PK_CopyTra(traject[count++], tempTry);
                    memcpy(traject[count], optTrajs, trajNum * PATH_ITEM_LEN);
                    line_num_dir = trajNum + count;
                    PathPlanType(0, 6, line_num_dir);
                    break;
                }
                pointLen = pointLen + 0.3f;
            }

            if (line_num_dir <= 0)
            {
                PK_Get_Path_EndPos(temp, startPos);
                PK_CopyTra(traject[count++], temp);
            }
        }
        else if (pathNum == 0)
        {
            failed = 0;
            if (fabs(temp[3]) < 1.0 && count > 0)
            {
                float trajlen = fabs(traject[count - 1][3]);
                if (fabs(temp[3]) < 0.333f * trajlen && trajlen > 1.5 + 0.001f)
                {
                    trajlen               = rte_max(trajlen * 0.5f, 1.5f);
                    traject[count - 1][3] = trajlen * sign(traject[count - 1][3]);
                    PK_Get_Path_EndPos(traject[count - 1], startPos);
                    continue;
                }
            }

            PK_Get_Path_EndPos(temp, startPos);
            PK_CopyTra(traject[count++], temp);
        }
        else
        {
            failed++;
        }
        dir = -dir;
    }

    return line_num_dir;
}

// the main entrance function for a slot to plan
/**
 * @brief 获取路径规划的最终轨迹，根据车位类型和环境条件动态生成路径。
 *
 * @param PlanData_Ver 计划数据，包括起点、终点、车位类型、障碍物等信息。
 * @param Act_traj 最终生成的轨迹数组。
 * @param IsDirConnect 指示是否路径直接连通的标志位。
 *
 * @return 返回生成的轨迹线段数，如果无法生成轨迹则返回 0。
 */
int PathPlan_GetTraj(const Multi_Slot_Array_T &slots, const SlotInfo_T &curSlot,
                     PlanDataCase PlanData_Ver[1], float Act_traj[][TRAJITEM_LEN],
                     bool usedHybrid, int *IsDirConnect)
{
    float rx      = Project_PosTo1st_rx(PlanData_Ver[0].stpoint,
                                        PlanData_Ver[0].finpoint); // 终点相对起点的横向投影
    *IsDirConnect = 0x200; // 默认标志位，表示无直接连通路径

    float ry     = Project_PosTo1st_ry(PlanData_Ver[0].finpoint,
                                       PlanData_Ver[0].stpoint); // 终点相对起点的纵向投影
    float rtheta = Project_PosTo1st_rtheta(
        PlanData_Ver[0].finpoint, PlanData_Ver[0].stpoint); // 终点相对起点的角度差

    // 斜列式车位
    if (PlanData_Ver[0].slotshape > PK_SLOT_RIGHT_PARA && fabs(rtheta) < PI * 0.333)
    {
        if (rx > 0)
        {
            return 0;
        } // 斜列式车位不要过早规划
    }

    // 添加障碍物，返回是否存在障碍物的标志
    int have_obj_ahead = Add_Obj_Temp(slots, PlanData_Ver);
    *IsDirConnect      = 0;

    if (fabs(rtheta) < 0.01 && fabs(ry) < 0.04)
    {
        PK_CopyPos(Act_traj[0], PlanData_Ver[0].stpoint);
        Act_traj[0][3] = rx;
        Act_traj[0][4] = 0;
        auto result =
            PK_Check_Path(1, Act_traj, PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint,
                          5, PlanData_Ver[0].obj_slot, 0);
        if (result == PATHPLAN_SUCCESS)
        {
            PathPlanType(0, 1, 1);
            return 1;
        }
    }

    // 针对平行车位的路径规划
    if (PlanData_Ver[0].slotshape == PK_SLOT_RIGHT_PARA ||
        PlanData_Ver[0].slotshape == PK_SLOT_LEFT_PARA)
    {
        // 尝试多次生成轨迹，每次对终点进行微调
        int pathNum = 0;
        float rex   = Project_PosTo1st_rx(
            PlanData_Ver[0].finpoint,
            &PlanData_Ver[0].obj_slot[4][0]); // 终点相对起点的横向投影
        float rbx = Project_PosTo1st_rx(
            PlanData_Ver[0].finpoint,
            &PlanData_Ver[0].obj_slot[1][0]); // 终点相对起点的横向投影
        float distCD =
            PK_PointToLineDist(PlanData_Ver[0].obj_slot[2], PlanData_Ver[0].finpoint);

        if (rx > -1.60f ||
            fabs(rbx - rex) < VEHICLE_LEN + PathPlanConstCfg::short_path_len ||
            distCD < VEHICLE_WID * 0.35f)
        {
            return 0;
        }

        float ryOffset = 0.0f;
        if (distCD > VEHICLE_WID * 0.5f + 0.15f)
        {
            ryOffset = 0.0f;
        }
        else
        {
            ryOffset = VEHICLE_WID * 0.5f + 0.15f - distCD;
            ryOffset = rte_max(ryOffset, 0.05f);
            ryOffset = rte_min(ryOffset, 0.30f);
        }

        float distBC =
            PK_PointToLineDist(PlanData_Ver[0].obj_slot[1], PlanData_Ver[0].finpoint);
        float rxOffset = 0.0f;
        if (distBC > REAR_SUSPENSION + 0.05f)
        {
            rxOffset = 0.0f;
        }
        else
        {
            rxOffset = REAR_SUSPENSION + 0.05f - distBC;
            rxOffset = rte_max(rxOffset, 0.02f);
            rxOffset = rte_min(rxOffset, 0.20f);
        }

        float lineBE[4];
        float distVBE = VEHICLE_WID * 0.5f + 0.10f;
        if (PlanData_Ver[0].is_vision_slot == 1)
        {
            memcpy(&lineBE[0], (float *)&curSlot.avm_point.near_rear, sizeof(Point_T));
            memcpy(&lineBE[2], (float *)&curSlot.avm_point.near_front, sizeof(Point_T));
        }

        float oldSlotTarget[3];
        PK_CopyPos(oldSlotTarget, PlanData_Ver[0].finpoint);
        for (int i = 0; i < 8; i++)
        {
            float rspoint[3] = {
                rxOffset, -(i * 0.02f + ryOffset) * PlanData_Ver[0].left_fac,
                0.0f}; // -PlanData_Ver[0].left_fac * 0.01f * i}; // 微调量
            Convert(oldSlotTarget, rspoint, PlanData_Ver[0].finpoint); // 修改终点坐标

            if (PlanData_Ver[0].is_vision_slot == 1 && i > 0)
            {
                distVBE = PK_PointToLineDist(lineBE, PlanData_Ver[0].finpoint);
                if (distVBE < VEHICLE_WID * 0.5f + 0.05f)
                {
                    break;
                }
            }

            pathNum = PK_PathPlan_SpaceExplorer_Para(PlanData_Ver,
                                                     Act_traj); // 生成平行车位路径
            if (pathNum > 0)
            {
                break;
            } // 成功生成路径则退出
            // printf(" -------- success: %d distBE %f distCD %f distBC %f offset %f,%f
            // VEHWID %f\n",
            //       curSlot.slot_index,  distVBE, distCD, distBC, ryOffset, (i * 0.02f +
            //       ryOffset), VEHICLE_WID * 0.5f);
        }

        // 最终路径的完整性检查
        if (pathNum <= 0)
        {
            auto startNum = PathPlan_SpaceExplorer_Para(PlanData_Ver, Act_traj);
            if (startNum > 0 && rx < -VEHICLE_LEN * 0.25 && usedHybrid)
            {
                PathPlan_HybridReplan(PlanData_Ver, Act_traj, startNum,
                                      curSlot.slot_index);
            }
            return 0;
        }

        auto result =
            PK_Check_Path(pathNum, Act_traj, PlanData_Ver[0].stpoint,
                          PlanData_Ver[0].finpoint, 5, PlanData_Ver[0].obj_slot, 0);
        if (result != PATHPLAN_SUCCESS)
        {
            return 0;
        }

        *IsDirConnect = 0x100; // 平行车位标志
        PathPlanType(0, 2, pathNum);
        return pathNum;
    }

    // 自定义斜车位处理
    if (PlanData_Ver[0].is_vision_slot == 2)
    {
        // 缩短斜车位的AB、EF边界线段
        Extend_LineSegment(&PlanData_Ver[0].obj_slot[4][0],
                           &PlanData_Ver[0].obj_slot[4][2], 0.5);
        Extend_LineSegment(&PlanData_Ver[0].obj_slot[0][2],
                           &PlanData_Ver[0].obj_slot[0][0], 0.5);
    }

    // 检查生成的轨迹是否有效并且可以连通
    if (PlanData_Ver[0].slotshape > PK_SLOT_RIGHT_PARA) // 针对其他复杂车位
    {
        // 逆鱼骨斜车位，需要头部泊入，尝试相应的路径生成方法
        // if (fabs(rtheta) > PI * 0.6 && PlanData_Ver[0].is_vision_slot != 0)
        {
            //    line_num_dir = PathPlan_PathIntoHead(PlanData_Ver, Act_traj_dir,
            //    IsDirConnect);
        }
        // 其它复杂车位
        // else if (line_num_dir < 1)
        // 垂直车位的路径规划
        float minDst = -1.0;
        int revCount = 0;
        int oldNum   = 0;
        if (sign(rtheta) * sign(ry) > 0)
        {
            *IsDirConnect = 0x200; // 垂直车位标志
            int pathNum   = PathPlan_SpaceExplorer_Vert(
                PlanData_Ver, Act_traj, IsDirConnect, 0); // 调用垂直车位规划方法
            if (pathNum > 0 &&
                Path_Check_Connect(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint,
                                   Act_traj, pathNum))
            {
                minDst = PathPlan_ShortDist(Act_traj, pathNum, &revCount);
                if (minDst > 1.0 && revCount <= 2)
                {
                    PathPlanType(0, 3, pathNum);
                    return pathNum;
                }
                oldNum = pathNum;
            }
        }

        int line_num_dir = 0;                                      // 当前轨迹的线段数量
        float Act_traj_dir[MAX_SINGLE_TRAJ_NUM * 2][TRAJITEM_LEN]; // 中间结果存储轨迹线段
        if (have_obj_ahead == 1)                                   // > 0.8
        {
            float dsmin = 0;
            // 根据障碍物位置动态调整路径
            if (PlanData_Ver[0].slotshape > 2)
            {
                dsmin = fabsf(Project_PosTo1st_ry(
                    PlanData_Ver[0].finpoint,
                    &PlanData_Ver[0].obj_danger[PlanData_Ver[0].obj_danger_num - 1][2]));
            }
            else
            {
                dsmin = fabsf(Project_PosTo1st_rx(
                    PlanData_Ver[0].finpoint,
                    &PlanData_Ver[0].obj_danger[PlanData_Ver[0].obj_danger_num - 1][2]));
            }

            if (dsmin > 6.0f) // 前方障碍物与车位距离足够远
            {
                line_num_dir = PathPlan_PathTryOptimize(line_num_dir, PlanData_Ver,
                                                        Act_traj_dir, IsDirConnect);
                PathPlanType(0, 4, line_num_dir);
            }
        }
        else if (have_obj_ahead == 0)
        {
            line_num_dir = PathPlan_PathTryOptimize(line_num_dir, PlanData_Ver,
                                                    Act_traj_dir, IsDirConnect);
            PathPlanType(0, 5, line_num_dir);
        }

        // 针对断头车位的特殊路径规划
        bool hybrid = false;
        if (line_num_dir < 1 && have_obj_ahead != 0 &&
            PlanData_Ver[0].slotshape >= PK_SLOT_LEFT_VERT)
        {
            hybrid = true;
            line_num_dir =
                PathPlan_RevertSlotPlan(PlanData_Ver, Act_traj_dir, IsDirConnect);
        }

        // 再次检查路径有效性
        if (line_num_dir > 0 && line_num_dir <= MAX_SINGLE_TRAJ_NUM)
        {
            auto result =
                Path_Check_Connect(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint,
                                   Act_traj_dir, line_num_dir);
            if (result == 0)
            {
                line_num_dir = 0;
            }
        }

        if (line_num_dir > 0)
        {
            if (oldNum > 0)
            {
                int newRevCount = 0;
                (void)PathPlan_ShortDist(Act_traj_dir, line_num_dir, &newRevCount);
                if (newRevCount > revCount)
                {
                    return oldNum;
                }
            }

            memcpy(Act_traj[0], Act_traj_dir[0], PATH_ITEM_LEN * line_num_dir);
            return line_num_dir;
        }
        else
        {
            if (rx < -VEHICLE_WID * 0.25)
            {
                hybrid = true;
            }

            if (oldNum <= 0 && hybrid && usedHybrid)
            {
                PK_CopyPos(Act_traj[0], PlanData_Ver[0].finpoint);
                Act_traj[0][3] = 0.0f;
                Act_traj[0][4] = 0.0f;
                PathPlan_HybridReplan(PlanData_Ver, Act_traj, 1, curSlot.slot_index);
            }

            return oldNum;
        }
    }

    return 0;
}

void PK_PathPlan_Envi_obj_cut(const float Envi_obj[5][4], const float Cut_factor,
                              float Envi_obj_side[2][4])
{
    Envi_obj_side[0][0] = Cut_factor * Envi_obj[1][0] + (1 - Cut_factor) * Envi_obj[1][2];
    Envi_obj_side[0][1] = Cut_factor * Envi_obj[1][1] + (1 - Cut_factor) * Envi_obj[1][3];
    Envi_obj_side[0][2] = Envi_obj[1][2];
    Envi_obj_side[0][3] = Envi_obj[1][3];
    Envi_obj_side[1][2] = (1 - Cut_factor) * Envi_obj[3][0] + Cut_factor * Envi_obj[3][2];
    Envi_obj_side[1][3] = (1 - Cut_factor) * Envi_obj[3][1] + Cut_factor * Envi_obj[3][3];
    Envi_obj_side[1][0] = Envi_obj[3][0];
    Envi_obj_side[1][1] = Envi_obj[3][1];
}

// to expend slot obj in the direction close to stpos
/**
 * @brief 根据停车位形状和距离信息，对停车位边界进行膨胀计算，更新停车位可泊空间。
 *
 * @param PlanData_Ver 停车位的形状和距离数据，包括停车位类型、前后距离、边界点等信息。
 * @return 返回 AB 和 EF 边界的膨胀量，用于后续的路径规划计算。
 *
 * @details
 * 根据不同类型的车位（平行车位或垂直车位），结合前后障碍物的距离信息，
 * 动态计算停车位各边的膨胀量（swell），从而为路径规划提供安全范围。
 *
 * - 膨胀的具体值由 `rear_dist` 和 `front_dist` 决定，确保车辆与障碍物保持一定的安全距离。
 * - 特殊处理平行车位的底部边界（CD 边），以避免车辆与底部障碍物过于接近。
 * - 对车位的边界线（AB, BC, CD, DE, EF）分别计算膨胀后的边界点，并根据几何关系更新交点。
 * - 更新后的边界存储在 `PlanData_Ver->obj_slot` 中。
 */

float PK_PathPlan_SlotObj_To_stPos_By_Swell(PlanDataCase PlanData_Ver[1],
                                            const float avmDist[3])
{
    float obj[5][4];                                  // 临时存储停车位的各边界线坐标
    int slotshap = PlanData_Ver->slotshape;           // 车位类型（平行车位、垂直车位等）
    LineSeg_T seg_AB, seg_BC, seg_CD, seg_EF, seg_DE; // 分别表示停车位的边界线段
    Point_T slotB, slotC, slotD, slotE;               // 用于存储边界交点坐标
    float swell_ABEF = 0.35f, swell_CD = 0.15f; // 膨胀参数，表示边界向内缩小展的距离
    float swell_BC = 0;                         // BC边膨胀量
    float swell_DE = 0;                         // DE边膨胀量
    float g_swell;                              // 当前膨胀量
    float temp = 0;                             // 临时变量，用于存储膨胀计算结果
    Vec2_T objVec, VertObjLeftVec;              // 用于表示边界线的方向向量和其法线向量
    int i;                                      // 循环索引
    float left_dir = 1.0f;                      // 表示膨胀方向，1为左侧，-1为右侧

    // 初始化膨胀值
    swell_ABEF = 0.30f;                   // AB和EF的默认膨胀量
    temp       = PlanData_Ver->rear_dist; // 读取车位后方的安全距离

    // 根据车位类型计算BC边的膨胀量
    if (slotshap == PK_SLOT_RIGHT_PARA || slotshap == PK_SLOT_LEFT_PARA) // 平行车位
    {
        swell_BC = rte_min(0.35f, rte_max(0.30f, temp - 0.4f)); // 对窄平行车位的特殊处理
    }
    else
    {
        swell_BC =
            temp > 0.35f ? 0.30f : (temp >= 0.05f ? temp - 0.05f : 0.0f); // 其他车位类型
        swell_BC = temp >= 0.55f ? 0.40f : swell_BC;                      // 最大膨胀量
        if (PlanData_Ver[0].is_vision_slot != 0 && avmDist[0] > 0.01f)
        {
            // 到B点距离
            float objry = Project_PosTo1st_ry(PlanData_Ver[0].finpoint,
                                              &PlanData_Ver[0].obj_slot[1][0]);
            if (fabs(objry) < avmDist[0] && fabs(objry) < VEHICLE_WID * 0.5f + 0.30f)
            {
                swell_BC = rte_min(0.05f, swell_BC);
            }
        }
    }

    // 计算DE边的膨胀量
    temp = PlanData_Ver->front_dist; // 读取车位前方的安全距离
    if (slotshap == PK_SLOT_RIGHT_PARA || slotshap == PK_SLOT_LEFT_PARA) // 平行车位
    {
        swell_DE = rte_min(0.35f, rte_max(0.30f, temp - 0.4f)); // 对窄平行车位的特殊处理
    }
    else
    {
        swell_DE = temp > 0.35f ? 0.30f : (temp >= 0.05f ? temp - 0.05f : 0.0f);
        swell_DE = temp >= 0.50f ? 0.35f : swell_DE;
        if (PlanData_Ver[0].is_vision_slot != 0 && avmDist[2] > 0.01f)
        {
            float objry = Project_PosTo1st_ry(PlanData_Ver[0].finpoint,
                                              &PlanData_Ver[0].obj_slot[4][0]);
            if (fabs(objry) < avmDist[2] && fabs(objry) < VEHICLE_WID * 0.5f + 0.30f)
            {
                swell_DE = rte_min(0.05f, swell_DE);
            }
        }
    }

    // 针对平行车位的底部障碍物膨胀处理，确保车辆与底部障碍物保持0.3m的安全距离
    if (slotshap == PK_SLOT_RIGHT_PARA || slotshap == PK_SLOT_LEFT_PARA)
    {
        float min_swell  = PathPlanConstCfg::para_cd_minswell + 0.05f;
        float rspoint[3] = {0};
        float dist_CD =
            PK_PointToLineDist(PlanData_Ver[0].obj_slot[2], PlanData_Ver[0].finpoint);
        dist_CD -= VEHICLE_WID / 2.0f;

        // 如果距离不足安全距离，调整CD边的位置
        if (dist_CD < min_swell)
        {
            rspoint[1] = -PlanData_Ver[0].left_fac * rte_min(min_swell - dist_CD, 0.0f);
        }
        Convert(PlanData_Ver[0].finpoint, rspoint,
                PlanData_Ver[0].finpoint); // 更新停车目标点
    }
    swell_CD = PathPlanConstCfg::para_cd_minswell; // 设置CD边的默认膨胀量

    // 计算车辆后方的安全距离
    PlanData_Ver->rear_safe_dist = PlanData_Ver->rear_dist - swell_BC;

    // 复制停车位的边界信息
    memcpy(obj, PlanData_Ver->obj_slot, sizeof(obj));

    // 根据车位类型调整膨胀方向
    if (slotshap == 1 || slotshap == 3 || slotshap == 5 || slotshap == 6)
    {
        left_dir = -1.0f; // 调整膨胀方向为右侧
    }

    // 遍历车位的每个边界线段，计算膨胀后的坐标
    for (i = 0; i < 5; i++)
    {
        objVec.vx = PlanData_Ver->obj_slot[i][2] -
                    PlanData_Ver->obj_slot[i][0]; // 计算边界线的方向向量
        objVec.vy = PlanData_Ver->obj_slot[i][3] - PlanData_Ver->obj_slot[i][1];
        Normalize_Vec2(&objVec);        // 单位化方向向量
        VertObjLeftVec.vx = -objVec.vy; // 计算法线向量
        VertObjLeftVec.vy = objVec.vx;

        // 根据边界线的类型设置膨胀值
        if (i == 0 || i == 4) // AB和EF边
        {
            g_swell = swell_ABEF;
        }
        else if (i == 2) // CD边
        {
            g_swell = swell_CD;
        }
        else if (i == 1) // BC边
        {
            g_swell = swell_BC;
        }
        else // DE边
        {
            g_swell = swell_DE;
        }

        // 更新边界线的坐标
        obj[i][0] += left_dir * g_swell * VertObjLeftVec.vx;
        obj[i][1] += left_dir * g_swell * VertObjLeftVec.vy;
        obj[i][2] += left_dir * g_swell * VertObjLeftVec.vx;
        obj[i][3] += left_dir * g_swell * VertObjLeftVec.vy;
    }
    memcpy(&seg_AB, obj[0], sizeof(seg_AB));
    memcpy(&seg_BC, obj[1], sizeof(seg_BC));
    memcpy(&seg_CD, obj[2], sizeof(seg_CD));
    memcpy(&seg_DE, obj[3], sizeof(seg_DE));
    memcpy(&seg_EF, obj[4], sizeof(seg_EF));

    if (Get_CrossPt_PointLine(seg_AB.pt1, seg_AB.pt2, seg_BC.pt1, seg_BC.pt2, &slotB))
    {
        memcpy(&obj[0][2], &slotB, sizeof(slotB));
        memcpy(&obj[1][0], &slotB, sizeof(slotB));
    }
    if (Get_CrossPt_PointLine(seg_BC.pt1, seg_BC.pt2, seg_CD.pt1, seg_CD.pt2, &slotC))
    {
        memcpy(&obj[1][2], &slotC, sizeof(slotC));
        memcpy(&obj[2][0], &slotC, sizeof(slotC));
    }
    if (Get_CrossPt_PointLine(seg_DE.pt1, seg_DE.pt2, seg_CD.pt1, seg_CD.pt2, &slotD))
    {
        memcpy(&obj[2][2], &slotD, sizeof(slotD));
        memcpy(&obj[3][0], &slotD, sizeof(slotD));
    }
    if (Get_CrossPt_PointLine(seg_DE.pt1, seg_DE.pt2, seg_EF.pt1, seg_EF.pt2, &slotE))
    {
        memcpy(&obj[3][2], &slotE, sizeof(slotE));
        memcpy(&obj[4][0], &slotE, sizeof(slotE));
    }

    // 更新最终的停车位边界信息
    memcpy(PlanData_Ver->obj_slot, obj, sizeof(obj));

    return swell_ABEF;
}

static int PathPlan_VertMoveOutPlan(PlanDataCase PlanDataVer[1],
                                    float ForwdPath[TRAJITEM_LEN],
                                    float OutTraj[][TRAJITEM_LEN])
{
    int Out_PathNum   = 0;
    float ForwdRadium = ForwdPath[4];
    float line_ds, rx;
    const float standOutMax = 0.6f; // AB or EF stand out compared with targpos
    float TempTraj[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
    float EndPos[3];
    const float path_ds_min        = 0.1f;
    const float NoCollis_deep_dist = 0.7f; //  calibrate based on Rmin = 4.4m

    float obj_all[SF_OBJ_NUM * 2 + 5][4];
    int obj_all_num = 0;
    // 统一障碍物
    memcpy(obj_all, PlanDataVer[0].obj_danger,
           sizeof(float) * PlanDataVer[0].obj_danger_num * 4);
    obj_all_num = PlanDataVer[0].obj_danger_num;
    memcpy(&obj_all[obj_all_num], PlanDataVer[0].obj_slot, sizeof(float) * 5 * 4);
    obj_all_num += 5;

    // turn towards BC
    if (((PlanDataVer[0].slotshape == PK_SLOT_LEFT_VERT ||
          PlanDataVer[0].slotshape == PK_SLOT_LEFT_ANG_FORWARD ||
          PlanDataVer[0].slotshape == PK_SLOT_LEFT_ANG_REVERSE) &&
         ForwdRadium < 0) ||
        ((PlanDataVer[0].slotshape == PK_SLOT_RIGHT_VERT ||
          PlanDataVer[0].slotshape == PK_SLOT_RIGHT_ANG_FORWARD ||
          PlanDataVer[0].slotshape == PK_SLOT_RIGHT_ANG_REVERSE) &&
         ForwdRadium > 0))
    {
        rx = Project_PosTo1st_rx(PlanDataVer[0].finpoint, PlanDataVer[0].obj_slot[1]) -
             NoCollis_deep_dist; // slot_B
    }
    else // turn towards DE
    {
        rx = Project_PosTo1st_rx(PlanDataVer[0].finpoint, PlanDataVer[0].obj_slot[4]) -
             NoCollis_deep_dist; // slot_E
    }

    rx      = rte_min(rx, VEHICLE_LEN - REAR_SUSPENSION + standOutMax);
    line_ds = rx - Project_PosTo1st_rx(PlanDataVer[0].finpoint, ForwdPath);
    if (line_ds > path_ds_min) // basic check
    {
        memcpy(TempTraj[0], ForwdPath, 3 * sizeof(float));
        TempTraj[0][3] = line_ds;
        TempTraj[0][4] = 0;
        //        PathPlan_LandMark2(TempTraj[0], PlanDataVer[0].obj_danger_num,
        //        PlanDataVer[0].obj_danger, 0); PathPlan_LandMark2(TempTraj[0], 5,
        //        PlanDataVer[0].obj_slot, 0);
        PK_PathVerify(TempTraj[0], obj_all_num, obj_all, 0);

        if (TempTraj[0][3] > path_ds_min)
        {
            PK_Get_Path_EndPos(TempTraj[0], EndPos);
            memcpy(TempTraj[1], EndPos, sizeof(EndPos));
            TempTraj[1][3] = 2.5f; // set a big value
            TempTraj[1][4] = ForwdRadium;
            //            PathPlan_LandMark2(TempTraj[1], PlanDataVer[0].obj_danger_num,
            //            PlanDataVer[0].obj_danger, 0); PathPlan_LandMark2(TempTraj[1],
            //            5, PlanDataVer[0].obj_slot, 0);
            PK_PathVerify(TempTraj[1], obj_all_num, obj_all, 0);

            if (TempTraj[1][3] > path_ds_min) //  try to connect targpos
            {
                Out_PathNum = PK_PathPlan_TraExplorer_Con(
                    TempTraj[1], PlanDataVer[0].finpoint, 5, PlanDataVer[0].obj_slot,
                    &TempTraj[1], 0, 1, PlanDataVer[0].swell, 0.0f);
                if (Out_PathNum > 0)
                {
                    Out_PathNum = Out_PathNum + 1;
                    memcpy(OutTraj, TempTraj, Out_PathNum * PATH_ITEM_LEN);
                }
            }
        }
    }
    return Out_PathNum;
}

int PathPlan_SpaceExplorer_Mid2Targ_Vert(float path[TRAJITEM_LEN],
                                         PlanDataCase PlanDataVer[1],
                                         float Exp_traj[][TRAJITEM_LEN], int num)
{
    float midtra[TRAJITEM_LEN], ForwExpTraj[TRAJITEM_LEN], BackExpTraj[TRAJITEM_LEN];
    float tempTra[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
    int Exp_PathNum                = 0;
    int Con_PathNum                = 0;
    const uint8 ForwBackExpCnt_Max = num; // 5;
    int i;
    uint8 plan_sucess = 0;
    float ds_back_max = 0;
    float deep_rx;
    const float deep_rx_dist    = 0.3f;
    const float short_path_dist = EXPLORE_DIST_MIN;
    const float Small_Theta_Err = 0.0872f; // 5deg

    float obj_all[SF_OBJ_NUM * 2 + 5][4];
    // 统一障碍物
    memcpy(obj_all, PlanDataVer[0].obj_danger,
           sizeof(float) * PlanDataVer[0].obj_danger_num * 4);
    int obj_all_num = PlanDataVer[0].obj_danger_num;
    memcpy(&obj_all[obj_all_num], PlanDataVer[0].obj_slot, sizeof(float) * 5 * 4);
    obj_all_num += 5;

    // add the trajs from stpos to midpos
    PK_Get_Path_EndPos(path, midtra); //  get midpos
    float midPos_rth = Round_PI(PlanDataVer[0].finpoint[2] - midtra[2]);
    for (i = 0; i < ForwBackExpCnt_Max; i++)
    {
        if (i == 0) //  the first explore path must be forward
        {
            memcpy(ForwExpTraj, midtra, sizeof(ForwExpTraj));
            ForwExpTraj[3] = 3.5f; //  set a long distance
            ForwExpTraj[4] = sign(midPos_rth) * Rrmin;
            //            PathPlan_LandMark2(ForwExpTraj, PlanDataVer[0].obj_danger_num,
            //            PlanDataVer[0].obj_danger, 0); PathPlan_LandMark2(ForwExpTraj,
            //            5, PlanDataVer[0].obj_slot, 0);
            PK_PathVerify(ForwExpTraj, obj_all_num, obj_all, 0);
        }
        else if (i % 2 == 1) //  explore backward
        {
            PK_Get_Path_EndPos(ForwExpTraj, BackExpTraj);
            ds_back_max = Round_PI(PlanDataVer[0].finpoint[2] - BackExpTraj[2]) * Rrmin;
            BackExpTraj[3] = -rte_min(2.5f, fabsf(ds_back_max)); //  set a long distance
            BackExpTraj[4] =
                -ForwExpTraj[4]; // the opposite turning direction of last move
            //            PathPlan_LandMark2(BackExpTraj, 5, PlanDataVer[0].obj_slot, 0);
            //            PathPlan_LandMark2(BackExpTraj, PlanDataVer[0].obj_danger_num,
            //            PlanDataVer[0].obj_danger, 0);
            PK_PathVerify(BackExpTraj, obj_all_num, obj_all, 0);
        }
        else // explore forward
        {
            PK_Get_Path_EndPos(BackExpTraj, ForwExpTraj);
            ForwExpTraj[3] = 3.5f; //  move forward,set a long distance
            ForwExpTraj[4] =
                -BackExpTraj[4]; // the opposite turning direction of last move
            //            PathPlan_LandMark2(ForwExpTraj, PlanDataVer[0].obj_danger_num,
            //            PlanDataVer[0].obj_danger, 0); PathPlan_LandMark2(ForwExpTraj,
            //            5, PlanDataVer[0].obj_slot, 0);
            PK_PathVerify(ForwExpTraj, obj_all_num, obj_all, 0);
        }
        // 检测，如果路径节点车位姿态远大于终点车位姿态，停止探索
        if (fabs(PlanDataVer[0].stpoint[2] - PlanDataVer[0].finpoint[2]) <
            fabs(PlanDataVer[0].stpoint[2] - ForwExpTraj[2]) * 2 / 3)
        {
            plan_sucess = 0;
            break;
        }

        // do the collection or try to connect to targpos
        if (i % 2 == 1) //  explore backward,collection the paths
        {
            if (fabsf(BackExpTraj[3]) < short_path_dist)
            {
                break;
            }
            memcpy(Exp_traj[Exp_PathNum], BackExpTraj, sizeof(BackExpTraj));
            Exp_PathNum += 1;
        }
        else // explore forward, try to connect the finpos, if success stop exploring else
             // collect the paths
        {
            // first judge if the vehicle has move deep to slot
            deep_rx = (VEHICLE_LEN - REAR_SUSPENSION) -
                      Project_PosTo1st_rx(PlanDataVer[0].finpoint, ForwExpTraj);
            if (deep_rx > deep_rx_dist &&
                fabsf(Round_PI(ForwExpTraj[2] - PlanDataVer[0].finpoint[2])) <
                    Small_Theta_Err)
            {
                Con_PathNum = PathPlan_VertMoveOutPlan(PlanDataVer, ForwExpTraj, tempTra);
            }
            else
            {
                if (fabsf(ForwExpTraj[3]) < short_path_dist)
                {
                    break;
                }
                Con_PathNum = PK_PathPlan_TraExplorer_Con(
                    ForwExpTraj, PlanDataVer[0].finpoint, 5, PlanDataVer[0].obj_slot,
                    tempTra, 0, 1, PlanDataVer[0].swell, 0.0f);
            }
#if 1
            if (Con_PathNum <= 0)
            {
                PK_Get_Path_EndPos(ForwExpTraj, midtra);
                memcpy(tempTra[0], ForwExpTraj, sizeof(ForwExpTraj));

                float Exp_finpoint_rear[2][5] = {{0}};
                PK_CopyPos(Exp_finpoint_rear[0], PlanDataVer[0].finpoint);
                Exp_finpoint_rear[0][3] = PATH_LAST_STRAIGHT_MIN;
                Exp_finpoint_rear[0][4] = 0.0;
                PK_Get_Path_EndPos(Exp_finpoint_rear[0], Exp_finpoint_rear[1]);

                Con_PathNum = PK_PathPlan_B_DirectCon_Replan(
                    midtra, Exp_finpoint_rear[1], 5, PlanDataVer[0].obj_slot, &tempTra[1],
                    PlanDataVer[0].swell);
                //  add the ForwExpTraj
                if (Con_PathNum > 0 && Path_Check_Connect(midtra, Exp_finpoint_rear[1],
                                                          &tempTra[1], Con_PathNum))
                {
                    Con_PathNum = Con_PathNum + 1;

                    PathPlan_ReverseTraject(Exp_finpoint_rear[0]);
                    PK_CopyTra(tempTra[Con_PathNum], Exp_finpoint_rear[0]);
                    Con_PathNum += 1;
                }
            }
#endif
            if (Con_PathNum > 0)
            {
                if (Exp_PathNum + Con_PathNum < MAX_SINGLE_TRAJ_NUM)
                {
                    memcpy(Exp_traj[Exp_PathNum], tempTra, Con_PathNum * PATH_ITEM_LEN);
                    Exp_PathNum += Con_PathNum;
                }
                else
                {
                    Exp_PathNum += Con_PathNum;
                    break;
                }

                if (PK_Check_Path(Exp_PathNum, Exp_traj, Exp_traj[0],
                                  PlanDataVer[0].finpoint, PlanDataVer[0].obj_danger_num,
                                  PlanDataVer[0].obj_danger, 0) == PATHPLAN_SUCCESS)
                {
                    plan_sucess = 1;
                    break;
                }
                else
                {
                    Exp_PathNum -= Con_PathNum; //  restore
                    memcpy(Exp_traj[Exp_PathNum], ForwExpTraj, sizeof(ForwExpTraj));
                    Exp_PathNum += 1;
                }
            }
            else
            {
                memcpy(Exp_traj[Exp_PathNum], ForwExpTraj, sizeof(ForwExpTraj));
                Exp_PathNum += 1;
            }
        }

        if (Exp_PathNum >= MAX_SINGLE_TRAJ_NUM)
        {
            break;
        }
    }

    if (Exp_PathNum >= MAX_SINGLE_TRAJ_NUM)
    {
        printf("narrow vert planned failed!\n");
    }

    if (plan_sucess == 0)
    {
        Exp_PathNum = 0;
    }
    return Exp_PathNum;
}

float Get_AnotherObs_Nearest_dy_to_CurPos(float curpos[3], int slotshap,
                                          float obsobj[][4], int obsNum)
{
    float Nearest_dy = 1e3;
    for (int i = 0; i < obsNum; i++)
    {
        float ry1 = Project_PosTo1st_ry(curpos, &obsobj[i][0]);
        float ry2 = Project_PosTo1st_ry(curpos, &obsobj[i][2]);
        if (((slotshap == 1 || slotshap == 3 || slotshap == 5 || slotshap == 6) &&
             ry1 < 0 && ry2 < 0) //  left slot
            || ((slotshap == 2 || slotshap == 4 || slotshap == 7 || slotshap == 8) &&
                ry1 > 0 && ry2 > 0)) //  right slot
        {
            float ry_dist = rte_min(fabsf(ry1), fabsf(ry2));
            if (ry_dist < Nearest_dy)
            {
                Nearest_dy = ry_dist;
            }
        }
    }
    return Nearest_dy;
}

int PK_PathPlan_B_DirectCon_Replan(float stpoint[3], float finpoint[3], int obj_num,
                                   float obj_slot[][4], float traject[][TRAJITEM_LEN],
                                   float swell)
{
    const float NearDist    = 0.8f;
    const float BigThetaGap = 0.2f; // rad

    int pathNum = 0;
    float fx    = Project_PosTo1st_rx(finpoint, stpoint);
    float fy    = Project_PosTo1st_ry(finpoint, stpoint);
    float rx    = Project_PosTo1st_rx(stpoint, finpoint);
    float rth   = Round_PI(finpoint[2] - stpoint[2]);
    float tempTraj[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
    Point_T CirPt_1, CirPt_2;
    Vec2_T Vert_Stpos_Left, Vert_Finpos_Left;

    // printf("--------------------PathPlan_B_DirectCon_Replan, fx dist: %f
    // -------------------\r\n", fx);
    if (fabsf(fx) > NearDist) //  if too close it is not able to plan
    {
        float Is_Circle_1_StayLeft = 0;
        if (fabsf(rth) > BigThetaGap)
        {
            Is_Circle_1_StayLeft = sign(rx) * sign(rth); // close to the angle of finpos
        }
        else
        {
            Is_Circle_1_StayLeft =
                sign(fx) * sign(fy) * sign(rx); // close to the axis x of finpos
        }

        Vert_Stpos_Left.vx  = -sinf(stpoint[2]);
        Vert_Stpos_Left.vy  = cosf(stpoint[2]);
        Vert_Finpos_Left.vx = -sinf(finpoint[2]);
        Vert_Finpos_Left.vy = cosf(finpoint[2]);
        // calculate the center point of circle 1 with Rmin
        CirPt_1.x = stpoint[0] + Is_Circle_1_StayLeft * Vert_Stpos_Left.vx * Rrmin;
        CirPt_1.y = stpoint[1] + Is_Circle_1_StayLeft * Vert_Stpos_Left.vy * Rrmin;
        // calculate the center point of circle 2 with Rmin
        CirPt_2.x = finpoint[0] - Is_Circle_1_StayLeft * Vert_Finpos_Left.vx * Rrmin;
        CirPt_2.y = finpoint[1] - Is_Circle_1_StayLeft * Vert_Finpos_Left.vy * Rrmin;
        // judge the relationship between these two circles
        float CirCenPt_Dist = Cal_Dis_Pt2Pt(CirPt_1, CirPt_2);
        if (CirCenPt_Dist >= 2.0f * Rrmin) //  do a simple check for two circle connection
        {
            pathNum = PathPlan_TwoCircleCon(stpoint, finpoint, obj_num, obj_slot,
                                            tempTraj, swell);
        }

        //        if (pathNum <= 0) //无用
        //        {
        //            pathNum = PathPlan_TwoEqualCircleCon_AllowStposRyErr(stpoint,
        //            finpoint, Rrmin, obj_num, obj_slot, tempTraj, swell);
        //        }
    }

    if (pathNum > 0 && pathNum < MAX_SINGLE_TRAJ_NUM)
    {
        memcpy(traject, tempTraj, pathNum * PATH_ITEM_LEN);
    }
    return pathNum;
}
