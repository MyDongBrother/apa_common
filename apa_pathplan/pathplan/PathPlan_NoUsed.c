#include "PK_PathPlanP.h"
#include "PathPlan_Update.h"
#include "PathPlan_Debug.h"
#include "PathPlan_BasePlan.h"
#include "PK_Calibration.h"
#include "MathFunc.h"
#include "PK_PathExecute.h"

static int PathPlan_SpaceExplorer_ParaY(PlanDataCase &planData,
                                        float trajs[][TRAJITEM_LEN]);
static void Stretch_AnotherObs_ForVertSlot_Na(PlanDataCase PlanDataVer[1]);
static int PathPlan_SpaceExplorer_Vert_Head(PlanDataCase PlanData_Ver[1],
                                            float Exp_trajs[][TRAJITEM_LEN],
                                            int *IsDirConnect);
static int PathPlan_BaseSpaceExplorer_Y(PlanDataCase &planData, float midpoint[3],
                                        float trajs[][TRAJITEM_LEN]);
static float Cal_MinObsSideDist_CurPos(int obsnum, float obsobj[][4], float curpos[4]);

extern int PathPlan_RangeDataProcess(PlanDataCase PlanData_Ver[1], const SlotInfo_T &slot,
                                     const int FS_ObjDir[FS_OBJ_ARR_NUM],
                                     const float FS_Obj[FS_OBJ_ARR_NUM][4],
                                     const int FS_ObjDir_Rear[FS_OBJ_ARR_NUM],
                                     const float FS_Obj_Rear[FS_OBJ_ARR_NUM][4]);
extern int Path_ParaCal(const float path[5], path_para EXtr_path[1], const float swell);

static void FinpointChange(float trajs[3])
{
    float trajs_dir[TRAJITEM_LEN];
    float temtra[2] = {(VEHICLE_LEN - 2 * REAR_SUSPENSION) + slot_ang_reverse_off_dy, 0};
    Convert(trajs, temtra, trajs_dir);
    trajs_dir[2] = Round_PI(trajs[2] + 2 * PI_2);
    trajs[0]     = trajs_dir[0];
    trajs[1]     = trajs_dir[1];
    trajs[2]     = trajs_dir[2];
}
/*
功能描述：逆向探索——得到从目标点到走出车位的2段路径
函数名称：Finpoint_Extend
INTPUT:     路径起点stpoint, 路径终点finpoint, 障碍物数量obj_num,
障碍物obj_slot，路径Exp_finpoint, 目标点到车位两边障碍物的最远纵向距离dfin_ds,
到目标点前设置的直线距离more_ds, 额外增加的避障距离swell OUTPUT:     路径Exp_finpoint，
RETURN:    返回路径数
*/
int Finpoint_Extend(float stpoint[3], float finpoint[3], int obj_num,
                    const float obj_slot[][4], float Exp_finpoint[][5], float dfin_ds,
                    float more_ds, float swell, int type = 0)
{
    float rtheta, ry, ry2, extend_direction, Rmin;
    float explor_num = 0, extend_ds, extend_ds_fin = 0, extend_ds_min = EXPLORE_DIST_MIN;
    ry               = Project_PosTo1st_ry(stpoint, finpoint);
    rtheta           = Project_PosTo1st_rtheta(stpoint, finpoint);
    extend_direction = -sign(ry) * sign(rtheta);
    PK_CopyPos(Exp_finpoint[0], finpoint);

    dfin_ds *= extend_direction;
    Rmin               = (Rrmin + 0.4);
    Exp_finpoint[0][3] = 0;

    extend_ds = extend_ds_min * extend_direction;

    do
    {
        dfin_ds /= 2;
        Exp_finpoint[0][3] = extend_ds;
        Exp_finpoint[0][4] = 0;
        PK_Get_Path_EndPos(Exp_finpoint[0], Exp_finpoint[1]);

        Exp_finpoint[1][3] = extend_direction * fabs(rtheta) * Rmin;
        Exp_finpoint[1][4] = sign(ry) * Rmin;
        PK_Get_Path_EndPos(Exp_finpoint[1], Exp_finpoint[2]);
        if (PathPlan_PathVerify2(Exp_finpoint[1], obj_num, obj_slot, swell) !=
            PATHPLAN_SUCCESS)
        {
            extend_ds += dfin_ds;
        }
        else
        {
            extend_ds_fin = extend_ds;
            explor_num    = 1;

            if (fabs(dfin_ds) <= 0.05 || extend_ds_fin < more_ds)
            {
                break;
            }
            if (extend_ds != extend_ds_min * extend_direction)
            {
                extend_ds -= dfin_ds;
            }
            else
            {
                break;
            }
        }
    } while (fabs(dfin_ds) >= 0.01); //二分法

    Exp_finpoint[0][3] = extend_ds_fin;
    if (explor_num == 1)
    {
        PK_Get_Path_EndPos(Exp_finpoint[1], Exp_finpoint[2]);
        ry2 = Project_PosTo1st_ry(stpoint, Exp_finpoint[2]);

        if (sign(ry) == sign(ry2))
        {
            Exp_finpoint[0][3] += sign(Exp_finpoint[0][3]) * fabs(ry2);

            if (fabsf(Exp_finpoint[0][3]) < 1.6)
            {
                Exp_finpoint[0][3] += sign(Exp_finpoint[0][3]) * 0.4;
            }
        }
        else if (fabs(ry2) < 0.4)
        {
            Exp_finpoint[0][3] += sign(Exp_finpoint[0][3]) * (0.4 - fabs(ry2));
        }

        if (type)
        {
            Exp_finpoint[0][3] = fabsf(Exp_finpoint[0][3]) > more_ds
                                     ? Exp_finpoint[0][3]
                                     : sign(Exp_finpoint[0][3]) * more_ds;
        }

        PK_Get_Path_EndPos(Exp_finpoint[0], Exp_finpoint[1]);
    }

    return explor_num;
}

/*
功能描述：逆向探索——得到从目标点到走出车位的2段路径
函数名称：PathPlan_FinpointExtend
INTPUT:     路径起点stpoint, 路径终点finpoint, 障碍物数量obj_num,
障碍物obj_slot，路径Exp_finpoint, 目标点到车位两边障碍物的最远纵向距离dfin_ds,
到目标点前设置的直线距离more_ds, 额外增加的避障距离swell OUTPUT:     路径Exp_finpoint，
RETURN:    返回路径数
*/
int PathPlan_FinpointExtend(float stpoint[3], float finpoint[3], int obj_num,
                            const float obj_slot[][4], float Exp_finpoint[][5],
                            float dfin_ds, float more_ds, float swell, int type = 0)
{
    float rtheta, ry, ry2, extend_direction, Rmin;
    float explor_num = 0, extend_ds, extend_ds_fin = 0, extend_ds_min = EXPLORE_DIST_MIN;
    ry               = Project_PosTo1st_ry(stpoint, finpoint);
    rtheta           = Project_PosTo1st_rtheta(stpoint, finpoint);
    extend_direction = -sign(ry) * sign(rtheta);
    PK_CopyPos(Exp_finpoint[0], finpoint);

    dfin_ds *= extend_direction;
    Rmin               = (Rrmin + 0.4);
    Exp_finpoint[0][3] = 0;

    extend_ds = extend_ds_min * extend_direction;

    do
    {
        dfin_ds /= 2;
        Exp_finpoint[0][3] = extend_ds;
        Exp_finpoint[0][4] = 0;
        PK_Get_Path_EndPos(Exp_finpoint[0], Exp_finpoint[1]);

        Exp_finpoint[1][3] = extend_direction * fabs(rtheta) * Rmin;
        Exp_finpoint[1][4] = sign(ry) * Rmin;
        PK_Get_Path_EndPos(Exp_finpoint[1], Exp_finpoint[2]);
        if (PathPlan_PathVerify2(Exp_finpoint[1], obj_num, obj_slot, swell) !=
            PATHPLAN_SUCCESS)
        {
            extend_ds += dfin_ds;
        }
        else
        {
            extend_ds_fin = extend_ds;
            explor_num    = 1;

            if (fabs(dfin_ds) <= 0.05 || extend_ds_fin < more_ds)
            {
                break;
            }
            if (extend_ds != extend_ds_min * extend_direction)
            {
                extend_ds -= dfin_ds;
            }
            else
            {
                break;
            }
        }
    } while (fabs(dfin_ds) >= 0.01); //二分法

    Exp_finpoint[0][3] = extend_ds_fin;
    if (explor_num == 1)
    {
        PK_Get_Path_EndPos(Exp_finpoint[1], Exp_finpoint[2]);
        ry2 = Project_PosTo1st_ry(stpoint, Exp_finpoint[2]);

        if (sign(ry) == sign(ry2))
        {
            Exp_finpoint[0][3] += sign(Exp_finpoint[0][3]) * fabs(ry2);

            if (fabsf(Exp_finpoint[0][3]) < 1.6)
            {
                Exp_finpoint[0][3] += sign(Exp_finpoint[0][3]) * 0.4;
            }
        }
        else if (fabs(ry2) < 0.4)
        {
            Exp_finpoint[0][3] += sign(Exp_finpoint[0][3]) * (0.4 - fabs(ry2));
        }

        if (type)
        {
            Exp_finpoint[0][3] = fabsf(Exp_finpoint[0][3]) > more_ds
                                     ? Exp_finpoint[0][3]
                                     : sign(Exp_finpoint[0][3]) * more_ds;
        }

        PK_Get_Path_EndPos(Exp_finpoint[0], Exp_finpoint[1]);
    }

    return explor_num;
}

/*
add 2023/3/13 Tangsj
功能描述：对向空间受限场景中将车辆向目标车位移动（y轴方向）--更改自函数PathPlan_BaseSpaceExplorer_Y
函数名称：PathPlan_BaseSpaceExplorer_Y2
INTPUT:     场景数据planData，移动目标点midpoint，路径trajs
OUTPUT:     路径trajs，返回值路径节点数lineNum
*/
static int PathPlan_BaseSpaceExplorer_Y2(PlanDataCase &planData, float midpoint[3],
                                         float trajs[][TRAJITEM_LEN])
{
    int lineNum        = 0;
    const float base_r = 60.0;
    float rx           = fabs(Project_PosTo1st_rx(planData.stpoint, planData.finpoint));
    float ry           = Project_PosTo1st_ry(planData.stpoint, planData.pointf_danger);
    float cirlen       = rte_max(VEHICLE_LEN * 1.2, rx);
    float midtra[TRAJITEM_LEN];
    PK_CopyPos(midtra, planData.stpoint);

    for (float radius = rte_min(base_r / 2, 10.0); radius < base_r; radius = 1.2 * radius)
    {
        midtra[3] = cirlen; //向前
        midtra[4] = sign(ry) * radius;
        PathPlan_UpdateTrajLen(midtra, planData.finpoint, planData.slotshape);
        PathPlan_LandMark2(midtra, 5, planData.obj_slot, 0);
        PathPlan_LandMark2(midtra, planData.obj_danger_num, planData.obj_danger, 0);

        if (fabs(midtra[3]) < rte_min(cirlen * 0.5, VEHICLE_LEN * 0.5))
        {
            continue;
        }

        PK_CopyTra(trajs[0], midtra);
        lineNum = PK_PathPlan_TraExplorer_Con(midtra, midpoint, 5, planData.obj_slot,
                                              trajs, 0, 1, planData.swell, 0.0f);
        if (lineNum <= 0)
        {
            continue;
        }

        int result =
            PK_Check_Path(lineNum, trajs, planData.stpoint, midpoint,
                          planData.obj_danger_num, planData.obj_danger, planData.swell);
        if (result == PATHPLAN_SUCCESS &&
            PathPlan_CheckPosDiv(trajs[lineNum - 1], midpoint) == 0)
        {
            break;
        }

        lineNum = 0;
    }

    return lineNum;
}

static int PathPlan_SpaceExplorer_ParaY(PlanDataCase &planData,
                                        float trajs[][TRAJITEM_LEN])
{
    const float ultronic_min_dist = 0.30;

    float rx = fabs(Project_PosTo1st_rx(planData.stpoint, planData.finpoint));
    if (rx < 1.5 * VEHICLE_LEN)
    {
        return 0;
    }

    float ry   = Project_PosTo1st_ry(planData.stpoint, planData.finpoint);
    float dlty = VEHICLE_WID * 0.50 + rte_max(ultronic_min_dist, g_local_swell);
    if (fabs(ry) < dlty)
    {
        return 0;
    }

    float finpoint[3];
    dlty        = fabs(ry) - dlty;
    float middy = rte_min(dlty, 0.35);
    middy       = rte_max(dlty * 0.5, middy);

    finpoint[0] = planData.stpoint[0] - sign(ry) * sin(planData.stpoint[2]) * middy;
    finpoint[1] = planData.stpoint[1] + sign(ry) * cos(planData.stpoint[2]) * middy;
    finpoint[2] = planData.finpoint[2];

    int lineNum = PathPlan_BaseSpaceExplorer_Y(planData, finpoint, trajs);
    return lineNum;
}

static int PathPlan_BaseSpaceExplorer_Y(PlanDataCase &planData, float midpoint[3],
                                        float trajs[][TRAJITEM_LEN])
{
    // 最大的转弯半径，这个值需要根据规格测试，按照规则为距离障碍物50cm,
    // 超声波最小探测距离为30cm，冗余距离为20cm
    // 前驱车最小的转弯半径表示转弯是车头前角点距离障碍物不超过20cm,
    // 因此转弯半径和车长、车头形状有关
    const float base_r = 60.0;
    float rx           = fabs(Project_PosTo1st_rx(planData.stpoint, planData.finpoint));
    float ry           = Project_PosTo1st_ry(planData.stpoint, planData.pointf_danger);

    float cirlen = rte_max(VEHICLE_LEN * 1.2, rx);
    float midtra[TRAJITEM_LEN];
    PK_CopyPos(midtra, planData.stpoint);

    int lineNum = 0;
    for (float radius = rte_min(base_r / 2, 10.0); radius < base_r; radius = 1.2 * radius)
    {
        midtra[3] = -cirlen;
        midtra[4] = sign(ry) * radius;

        PathPlan_LandMark2(midtra, 5, planData.obj_slot, 0);
        PathPlan_LandMark2(midtra, planData.obj_danger_num, planData.obj_danger, 0);

        if (fabs(midtra[3]) < rte_min(cirlen * 0.5, VEHICLE_LEN * 0.5))
        {
            continue;
        }

        PK_CopyTra(trajs[0], midtra);
        lineNum = PK_PathPlan_TraExplorer_Con(midtra, midpoint, 5, planData.obj_slot,
                                              trajs, 0, 1, planData.swell, 0.0f);
        if (lineNum <= 0)
        {
            continue;
        }

        int result =
            PK_Check_Path(lineNum, trajs, planData.stpoint, midpoint,
                          planData.obj_danger_num, planData.obj_danger, planData.swell);
        if (result == PATHPLAN_SUCCESS &&
            PathPlan_CheckPosDiv(trajs[lineNum - 1], midpoint) == 0)
        {
            break;
        }

        lineNum = 0;
    }
    return lineNum;
}

static int PathPlan_SpaceExplorer_VertY(PlanDataCase &planData,
                                        float trajs[][TRAJITEM_LEN])
{
    const float Narrow_AnotherDist = 2.2f;

    float Nearest_dy =
        Get_AnotherObs_Nearest_dy_to_CurPos(planData.stpoint, planData.slotshape,
                                            planData.obj_danger, planData.obj_danger_num);
    if (Nearest_dy > Narrow_AnotherDist)
    {
        return 0;
    }

    float rx = fabs(Project_PosTo1st_rx(planData.stpoint, planData.pointf_danger));
    if (rx < VEHICLE_LEN)
    {
        return 0;
    }

    float ry    = Project_PosTo1st_ry(planData.stpoint, planData.pointf_danger);
    float dlty  = Narrow_AnotherDist + g_local_swell - Nearest_dy;
    float middy = rte_min(dlty, 0.40);
    middy       = rte_max(dlty * 0.5, middy);

    float finpoint[3];
    finpoint[0] = planData.stpoint[0] - sign(ry) * sin(planData.stpoint[2]) * middy;
    finpoint[1] = planData.stpoint[1] + sign(ry) * cos(planData.stpoint[2]) * middy;
    finpoint[2] = Round_PI(planData.finpoint[2] + sign(ry) * PI / 2);

    int lineNum = PathPlan_BaseSpaceExplorer_Y(planData, finpoint, trajs);
    return lineNum;
}

// 20181214: if the obstacle on another side of slot is on the right head of targpos it
// need to strech to EF to avoid collision for path connect to targpos
static void Stretch_AnotherObs_ForVertSlot_Na(PlanDataCase PlanDataVer[1])
{
    float ry_obs, rx_obs;
    int i;
    int AnotherObsAttr               = 0;
    float obs_pow_len                = 0;
    const float valid_obs_powlen_min = 0.09f; // 0.3m
    const float ry_zero_err          = 0.1f;
    const float stretch_len          = 1.0f;
    float pos_EF[3] = {PlanDataVer[0].obj_slot[4][0], PlanDataVer[0].obj_slot[4][1], 0};

    pos_EF[2] = atan2f(PlanDataVer[0].obj_slot[4][3] - PlanDataVer[0].obj_slot[4][1],
                       PlanDataVer[0].obj_slot[4][2] - PlanDataVer[0].obj_slot[4][0]);
    if (PlanDataVer[0].slotshape == 3 || PlanDataVer[0].slotshape == 5 ||
        PlanDataVer[0].slotshape == 6)
    {
        AnotherObsAttr = 2;
    }
    else
    {
        AnotherObsAttr = 1;
    }

    for (i = 0; i < PlanDataVer[0].obj_danger_num; i++)
    {
        if (PlanDataVer[0].obj_danger_dir[i] != AnotherObsAttr)
        {
            continue;
        }

        obs_pow_len =
            pow2(PlanDataVer[0].obj_danger[i][0] - PlanDataVer[0].obj_danger[i][2]) +
            pow2(PlanDataVer[0].obj_danger[i][1] - PlanDataVer[0].obj_danger[i][3]);

        if (obs_pow_len <= valid_obs_powlen_min)
        {
            continue;
        }

        ry_obs = Project_PosTo1st_ry(PlanDataVer[0].finpoint,
                                     &PlanDataVer[0].obj_danger[i][2]);
        if ((AnotherObsAttr == 2 && ry_obs > -ry_zero_err) ||
            (AnotherObsAttr == 1 && ry_obs < ry_zero_err))
        {
            rx_obs = Project_PosTo1st_rx(pos_EF, &PlanDataVer[0].obj_danger[i][2]);
            if (rx_obs < stretch_len)
            {
                PlanDataVer[0].obj_danger[i][2] +=
                    (stretch_len - rx_obs) * cosf(pos_EF[2]);
                PlanDataVer[0].obj_danger[i][3] +=
                    (stretch_len - rx_obs) * sinf(pos_EF[2]);
            }
        }
    }
}

//逆鱼骨斜车位（车头泊入）
static int PathPlan_PathIntoHead(PlanDataCase PlanData_Ver[1],
                                 float Act_traj[][TRAJITEM_LEN], int *IsDirConnect)
{
    float Act_traj_dir[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
    int line_num_dir = 0;

    /*路径探索4**********************************************************************/
    if (PlanData_Ver[0].slotshape > 2) //车头泊入
    {
        Extend_LineSegment(&PlanData_Ver[0].obj_slot[4][0],
                           &PlanData_Ver[0].obj_slot[4][2], 1.0);

        int TrajNum_RS =
            PathPlan_SpaceExplorer_Vert_Head(PlanData_Ver, Act_traj_dir, IsDirConnect);
        if (TrajNum_RS > 0 &&
            fabs(Project_PosTo1st_rtheta(PlanData_Ver[0].stpoint, Act_traj_dir[2]) -
                     fabs(Project_PosTo1st_rtheta(PlanData_Ver[0].stpoint,
                                                  PlanData_Ver[0].finpoint)) <
                 PI / 3))
        {
            line_num_dir = TrajNum_RS;
        }
        if (line_num_dir > 0 &&
            PK_Check_Path(line_num_dir, Act_traj_dir, PlanData_Ver[0].stpoint,
                          PlanData_Ver[0].finpoint, PlanData_Ver[0].obj_danger_num,
                          PlanData_Ver[0].obj_danger, 0) != PATHPLAN_SUCCESS)
        {
            line_num_dir = 0;
        }
    }

    if (line_num_dir > 0 && line_num_dir < 20)
    {
        memcpy(Act_traj[0], Act_traj_dir[0], PATH_ITEM_LEN * line_num_dir);

        return line_num_dir;
    }

    return 0;
}

static int PathPlan_SpaceExplorer_Vert_Head(
    PlanDataCase PlanData_Ver[1], float Exp_trajs[][TRAJITEM_LEN],
    int *IsDirConnect) // HAVECHANGE Tangsj 2023/6/27
{
    const int Explore_times = 5; // 2023
    int line_num            = 0;
    float rtheta, rx, dfin_ds, extend_ds, explor_ds,
        explor_ds_max; // front_ry_org;  //ds_1st_min,ds_1st_max,
    float Exp_stpoint_rear[2][5], Exp_stpoint_front[2][5],
        Exp_finpoint[MAX_SINGLE_TRAJ_NUM][5];
    float min_connect_dis = 0.3f;
    float Envi_obj_side[2][4], SlotAB_EF[2][4];
    int explor_num_first = 0, explor_num_last = 0; //  the explore path num
    float Exp_traj[MAX_SINGLE_TRAJ_NUM][5];
    float oldfinpoint[3] = {0};
    float obj_all[SF_OBJ_NUM * 2 + 5][4];
    int obj_all_num = 0;
    //统一障碍物
    memcpy(obj_all, PlanData_Ver[0].obj_danger,
           sizeof(float) * PlanData_Ver[0].obj_danger_num * 4);
    obj_all_num = PlanData_Ver[0].obj_danger_num;
    memcpy(&obj_all[obj_all_num], PlanData_Ver[0].obj_slot, sizeof(float) * 5 * 4);
    obj_all_num += 5;

    memcpy(oldfinpoint, PlanData_Ver[0].finpoint, sizeof(oldfinpoint));
    FinpointChange(PlanData_Ver[0].finpoint); //改变目标点

    rtheta = Project_PosTo1st_rtheta(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint);
    rx     = Project_PosTo1st_rx(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint);
    // float ry = Project_PosTo1st_ry(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint);

    PK_PathPlan_Envi_obj_cut(PlanData_Ver[0].obj_slot, 1.0f, Envi_obj_side); // get BC,DE
    memcpy(SlotAB_EF[0], PlanData_Ver[0].obj_slot[0], 4 * sizeof(float));    // get AB
    memcpy(SlotAB_EF[1], PlanData_Ver[0].obj_slot[4], 4 * sizeof(float));    // get EF

    extend_ds        = rx - 15;
    explor_num_first = Stpoint_Extend(PlanData_Ver, Exp_stpoint_rear, extend_ds);
    extend_ds        = 15 + rx;
    explor_num_first = Stpoint_Extend(PlanData_Ver, Exp_stpoint_front, extend_ds);

    memcpy(Exp_traj[0], Exp_stpoint_rear[0], PATH_ITEM_LEN * explor_num_first);

    line_num = PK_PathPlan_BetterTra(Exp_stpoint_rear[1], PlanData_Ver[0].finpoint, 2,
                                     Envi_obj_side, &Exp_traj[explor_num_first],
                                     1.5f * PlanData_Ver[0].swell); // for a big slot
    if (line_num > 0 &&
        PK_Check_Path(line_num + explor_num_first, Exp_traj, PlanData_Ver[0].stpoint,
                      PlanData_Ver[0].finpoint, obj_all_num, obj_all,
                      0) != PATHPLAN_SUCCESS) // 1.5f*PlanData_Ver[0].swellAny small
                                              // movement or Path damage
    {
        line_num = 0;
    }
    else if (line_num > 0 && Exp_traj[explor_num_first][4] < 1 &&
             sign(Exp_stpoint_rear[0][3]) != sign(Exp_traj[explor_num_first][3]))
    {
        *IsDirConnect = 1;
        memcpy(Exp_trajs, Exp_traj, sizeof(float) * 5 * (line_num + explor_num_first));
        return line_num + explor_num_first;
    }

    dfin_ds =
        (PlanData_Ver[0].len - PlanData_Ver[0].h) -
        rte_min(
            Project_PosTo1st_rx(PlanData_Ver[0].finpoint, PlanData_Ver[0].pointf_danger),
            Project_PosTo1st_rx(PlanData_Ver[0].finpoint, PlanData_Ver[0].pointb_danger));

    for (int i = 0; i < Explore_times; i++)
    {
        if (i == 0)
        {
            if (PathPlan_FinpointExtend(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint,
                                        2, Envi_obj_side, Exp_finpoint, dfin_ds, 0,
                                        PlanData_Ver[0].swell))
            {
                explor_num_last = 1;
                explor_ds_max   = PathPlan_LandMark2(Exp_finpoint[explor_num_last],
                                                     PlanData_Ver[0].obj_danger_num,
                                                     PlanData_Ver[0].obj_danger, 0);

                line_num = PK_PathPlan_TraExplorer_Con(
                    Exp_finpoint[explor_num_last], PlanData_Ver[0].stpoint, obj_all_num,
                    obj_all, Exp_traj, 0, 0, PlanData_Ver[0].swell, min_connect_dis);
                if (line_num < 1)
                {
                    line_num = PK_PathPlan_TraExplorer_Con(
                        Exp_finpoint[explor_num_last], Exp_stpoint_front[1], 2, SlotAB_EF,
                        &Exp_traj[explor_num_first], 0, 0, PlanData_Ver[0].swell,
                        min_connect_dis);
                    if (line_num > 0 && fabs(Exp_traj[explor_num_first][4]) < 1 &&
                        sign(Exp_stpoint_front[0][3]) !=
                            sign(Exp_traj[explor_num_first][3]))
                    {
                        memcpy(Exp_traj[0], Exp_stpoint_front[0], 5 * sizeof(float));
                        line_num += explor_num_first;
                    }
                    else if (line_num > 0)
                    {
                        line_num = 0;
                    }
                }
            }
            else
            {
                break;
            }
        }

        if (i % 2 == 1) // explore forward -- try to connect stpoint/Exp_stpoint[0]
        {
            rtheta = Project_PosTo1st_rtheta(PlanData_Ver[0].stpoint,
                                             Exp_finpoint[explor_num_last]);
            explor_ds =
                -sign(Exp_finpoint[explor_num_last - 1][3]) * fabs(rtheta) * Rrmin;
            Exp_finpoint[explor_num_last][3] = explor_ds;
            Exp_finpoint[explor_num_last][4] = -PlanData_Ver[0].left_fac * Rrmin;

            if (!PK_PathVerify(Exp_finpoint[explor_num_last], 2, SlotAB_EF, 0) &&
                !PK_PathVerify(Exp_finpoint[explor_num_last],
                               PlanData_Ver[0].obj_danger_num, PlanData_Ver[0].obj_danger,
                               0))
            {
                PK_Get_Path_EndPos(Exp_finpoint[explor_num_last],
                                   Exp_finpoint[explor_num_last + 1]);
                explor_num_last++;
                Exp_finpoint[explor_num_last][3] = 0.8;
                Exp_finpoint[explor_num_last][4] = 0;
                PK_Get_Path_EndPos(Exp_finpoint[explor_num_last],
                                   Exp_finpoint[explor_num_last + 1]);
                explor_num_last++;

                line_num = PK_PathTryConnect_TwoCon(0, Exp_finpoint[explor_num_last],
                                                    PlanData_Ver, Exp_traj);
            }
            if (line_num < 1)
            {
                explor_ds_max = Exp_finpoint[explor_num_last][3];

                line_num = PK_PathPlan_TraExplorer_Con(
                    Exp_finpoint[explor_num_last], PlanData_Ver[0].stpoint, obj_all_num,
                    obj_all, Exp_traj, 0, 0, PlanData_Ver[0].swell, min_connect_dis);
            }

            //            explor_ds_max =
            //            PathPlan_LandMark2(Exp_finpoint[explor_num_last], 2, SlotAB_EF,
            //            0); explor_ds_max =
            //            PathPlan_LandMark2(Exp_finpoint[explor_num_last],
            //            PlanData_Ver[0].obj_danger_num, PlanData_Ver[0].obj_danger, 0);
            if (line_num < 1)
            {
                line_num = PK_PathPlan_TraExplorer_Con(
                    Exp_finpoint[explor_num_last], Exp_stpoint_rear[1],
                    PlanData_Ver[0].obj_danger_num, PlanData_Ver[0].obj_danger,
                    &Exp_traj[explor_num_first], 0, 0, PlanData_Ver[0].swell,
                    min_connect_dis);
                if (line_num > 0 && fabs(Exp_traj[explor_num_first][4]) < 1 &&
                    sign(Exp_stpoint_rear[0][3]) != sign(Exp_traj[explor_num_first][3]))
                {
                    memcpy(Exp_traj[0], Exp_stpoint_rear[0], 5 * sizeof(float));
                    line_num += explor_num_first;
                }
                else if (line_num > 0)
                {
                    line_num = 0;
                }
            }
        }

        else if (i > 1) // explore backward
        {
            rtheta = Project_PosTo1st_rtheta(PlanData_Ver[0].stpoint,
                                             Exp_finpoint[explor_num_last]);
            Exp_finpoint[explor_num_last][3] =
                -sign(Exp_finpoint[explor_num_last - 1][3]) * fabs(rtheta) * Rrmin;
            Exp_finpoint[explor_num_last][4] = PlanData_Ver[0].left_fac * Rrmin;
            explor_ds_max = PathPlan_LandMark2(Exp_finpoint[explor_num_last],
                                               PlanData_Ver[0].obj_danger_num,
                                               PlanData_Ver[0].obj_danger, 0);

            line_num = PK_PathPlan_TraExplorer_Con(
                Exp_finpoint[explor_num_last], PlanData_Ver[0].stpoint, obj_all_num,
                obj_all, Exp_traj, 0, 0, PlanData_Ver[0].swell, min_connect_dis);

            if (line_num < 1)
            {
                line_num = PK_PathPlan_TraExplorer_Con(
                    Exp_finpoint[explor_num_last], Exp_stpoint_front[1],
                    PlanData_Ver[0].obj_danger_num, PlanData_Ver[0].obj_danger,
                    &Exp_traj[explor_num_first], 0, 0, PlanData_Ver[0].swell,
                    min_connect_dis);
                if (line_num > 0 && fabs(Exp_traj[explor_num_first][4]) < 1 &&
                    sign(Exp_stpoint_front[0][3]) != sign(Exp_traj[explor_num_first][3]))
                {
                    memcpy(Exp_traj[0], Exp_stpoint_front[0], 5 * sizeof(float));
                    line_num += explor_num_first;
                }
                else if (line_num > 0)
                {
                    line_num = 0;
                }
            }
        }

        if (line_num < 1 && fabs(explor_ds_max) > 2.5 &&
            fabs(explor_ds_max) > fabs(explor_ds / 2))
        {
            Exp_finpoint[explor_num_last][3] =
                sign(explor_ds_max) * 2.5; // explor_ds_max/2;
        }

        if (line_num < 1 ||
            (line_num > 0 &&
             PK_Check_Path(line_num, Exp_traj, PlanData_Ver[0].stpoint,
                           Exp_finpoint[explor_num_last], PlanData_Ver[0].obj_danger_num,
                           PlanData_Ver[0].obj_danger,
                           0) != PATHPLAN_SUCCESS)) // 1.5f*PlanData_Ver[0].swellAny small
                                                    // movement or Path damage
        {
            PK_Get_Path_EndPos(Exp_finpoint[explor_num_last],
                               Exp_finpoint[explor_num_last + 1]);
            explor_num_last++;
            line_num = 0;
        }
        else if (line_num > 0)
        {
            *IsDirConnect = 1;
            for (int j = 0; j < explor_num_last; j++)
            {
                PathPlan_ReverseTraject(Exp_finpoint[explor_num_last - 1 - j]);
                PK_CopyTra(Exp_traj[line_num + j], Exp_finpoint[explor_num_last - 1 - j]);
            }
            // memcpy(Exp_trajs, Exp_traj, sizeof (float)*5*(line_num + explor_num_last));
            // return line_num + explor_num_last;
            line_num += explor_num_last;
            break;
        }
    }
    if (line_num > 0 && line_num < MAX_SINGLE_TRAJ_NUM)
    {
        memcpy(Exp_trajs, Exp_traj, sizeof(float) * 5 * line_num);
    }
    else
    {
        memcpy(PlanData_Ver[0].finpoint, oldfinpoint, sizeof(oldfinpoint));
    }

    return line_num;
}

static int GetPath_StToMidPos_Rear_Simple(PlanDataCase PlanData_Ver[1],
                                          float StToMid_Traj[][TRAJITEM_LEN],
                                          int *IsDirConnect) //向后
{
    int TrajNum_NS = 0;
    float EndPos[3], oldstpoint[3];
    float Act_traj_dir[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
    PK_CopyPos(oldstpoint, PlanData_Ver[0].stpoint);
#if 0
    StToMid_Traj[0][3] = -VEHICLE_LEN*1.2 ;
    StToMid_Traj[0][4] = PlanData_Ver[0].left_fac*Rrmin ;
    memcpy(StToMid_Traj[0], PlanData_Ver[0].stpoint, 3 * sizeof(float));
    PathPlan_LandMark2(StToMid_Traj[0], 5, PlanData_Ver[0].obj_slot, 0);
    PathPlan_LandMark2(StToMid_Traj[0], PlanData_Ver[0].obj_danger_num, PlanData_Ver[0].obj_danger, 0);
    PK_PathPlan_TraExplorer_Con3(0,StToMid_Traj[0],  PlanData_Ver[0].finpoint, StToMid_Traj, 0.0f);

    PK_Get_Path_EndPos(StToMid_Traj[0], EndPos);
    memcpy(StToMid_Traj[1], EndPos, 3 * sizeof(float));
    rtheta = fabs( Round_PI( Project_PosTo1st_rtheta(StToMid_Traj[1], PlanData_Ver[0].finpoint)));
    StToMid_Traj[1][3] = rtheta * Rrmin;
    StToMid_Traj[1][4] = -PlanData_Ver[0].left_fac * Rrmin;
    PathPlan_LandMark2(StToMid_Traj[1], 5, PlanData_Ver[0].obj_slot, 0);
    PathPlan_LandMark2(StToMid_Traj[1], PlanData_Ver[0].obj_danger_num, PlanData_Ver[0].obj_danger, 0);

    PK_Get_Path_EndPos(StToMid_Traj[1], EndPos);
    memcpy(StToMid_Traj[2], EndPos, 3 * sizeof(float));
    TrajNum_NS = 3;
    PK_CopyPos(PlanData_Ver[0].stpoint, StToMid_Traj[TrajNum_NS - 1]);
    int line_num_dir = PathPlan_SpaceExplorer_Vert(PlanData_Ver, Act_traj_dir, IsDirConnect);
    PK_CopyPos(PlanData_Ver[0].stpoint,oldstpoint );
    //qDebug()<<"PK_PathPlan204"<<line_num_dir;
#endif
    FinpointChange(PlanData_Ver[0].finpoint);

    float rx1 = Project_PosTo1st_rx(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint);
    if (rx1 < 0)
    {
        StToMid_Traj[0][3] = rx1 - 15;
    }
    else
    {
        StToMid_Traj[0][3] = -15;
    }

    StToMid_Traj[0][4] = 0;
    memcpy(StToMid_Traj[0], PlanData_Ver[0].stpoint, 3 * sizeof(float));
    PathPlan_LandMark2(StToMid_Traj[0], PlanData_Ver[0].obj_danger_num,
                       PlanData_Ver[0].obj_danger, 0);
    PK_Get_Path_EndPos(StToMid_Traj[0], EndPos);
    PK_CopyPos(PlanData_Ver[0].stpoint, EndPos);
    TrajNum_NS = 2;
    int line_num_dir =
        PathPlan_SpaceExplorer_Vert_Head(PlanData_Ver, Act_traj_dir, IsDirConnect);
    PK_CopyPos(PlanData_Ver[0].stpoint, oldstpoint);
    if (line_num_dir > 0)
    {
        memcpy(StToMid_Traj[TrajNum_NS - 1], Act_traj_dir[0],
               PATH_ITEM_LEN * line_num_dir);

        return TrajNum_NS - 1 + line_num_dir;
    }

    return 0;
}

static int PathPlan_TwoEqualCircleCon_AllowStposRyErr(float stpos[3], float finpos[3],
                                                      float Rr, int objnum,
                                                      float slotobj[5][4],
                                                      float expTraj[][TRAJITEM_LEN],
                                                      float swell)
{
    float rx, fx, fy, rth;
    int i, j, PathNum = 0;
    float VertStVec_Left[2]  = {-sinf(stpos[2]), cosf(stpos[2])};
    float VertFinVec_Left[2] = {-sinf(finpos[2]), cosf(finpos[2])};
    float Cir1_R_Sign = 0, Cir2_R_Sign = 0;
    const float BigThetaGap = 0.2f; // rad
    float Cir1_CenPt[2], Cir2_CenPt[2], NewCir1_CenPt[2], NewCir2_CenPt[2];
    float CenPtPowDist = 0;
    float New_Stpos[3], New_FinPos[3], Temp_StPos[3];
    float b, c;
    float diff = 0; // the distance between newstpos and stpos
    float Delta, rx_temp_stpos, fx_temp_stpos;
    const float Stpos_Ry_Err_Max = 0.1f;
    int Get_Rst                  = 1;
    float line_ds                = 0;
    float TanPt[2], rx_tanpt, fx_tanpt, fx_temp_finpos, cross_angle;
    Vec2_T Vec_1, Vec_2;
    float TempPath[3][5], EndPos[3], ds, dth, dPowPos;
    const float dPowPos_max = 0.0009f; // 3cm
    const float dth_max     = 0.02f;   // 0.02 rad

    if (Rr < Rrmin)
    {
        return 0;
    }
    rx  = Project_PosTo1st_rx(stpos, finpos);
    fy  = Project_PosTo1st_ry(finpos, stpos);
    fx  = Project_PosTo1st_rx(finpos, stpos);
    rth = Round_PI(finpos[2] - stpos[2]);
    if (fabsf(rth) > BigThetaGap)
    {
        Cir1_R_Sign = sign(rx) * sign(rth);
    }
    else
    {
        Cir1_R_Sign = sign(fx) * sign(fy) * sign(rx);
    }
    // calculate the circle center point
    Cir2_R_Sign = -Cir1_R_Sign;

    for (i = 0; i < 2; i++)
    {
        Cir1_CenPt[i] = stpos[i] + Cir1_R_Sign * VertStVec_Left[i] * Rr;
        Cir2_CenPt[i] = finpos[i] + Cir2_R_Sign * VertFinVec_Left[i] * Rr;
    }
    // judge the relationship between these two circles
    CenPtPowDist =
        pow2(Cir1_CenPt[0] - Cir2_CenPt[0]) + pow2(Cir1_CenPt[1] - Cir2_CenPt[1]);
    memcpy(New_Stpos, stpos, sizeof(New_Stpos));

    if (CenPtPowDist < 4.0f * pow2(Rrmin))
    {
        b = 2 * Cir1_R_Sign *
            ((Cir2_CenPt[0] - Cir1_CenPt[0]) * sinf(stpos[2]) +
             (Cir1_CenPt[1] - Cir2_CenPt[1]) * cosf(stpos[2]));
        c     = CenPtPowDist - 4.0f * pow2(Rrmin);
        Delta = b * b - 4.0f * c; //  a = 1.0f;
        if (Delta >= 0)
        {
            for (j = 0; j < 2; j++)
            {
                if (j == 0)
                {
                    diff = (-b + sqrtf(Delta)) / 2.0f; //    a = 1.0f;
                }
                else
                {
                    diff = (-b - sqrtf(Delta)) / 2.0f;
                }
                if (diff >= 0 && diff <= Stpos_Ry_Err_Max)
                {
                    diff += 0.01f; //  add extra 1cm to avoid the boundary condition
                    for (i = 0; i < 2; i++)
                    {
                        New_Stpos[i] = stpos[i] + Cir1_R_Sign * VertStVec_Left[i] * diff;
                        Cir1_CenPt[i] =
                            New_Stpos[i] + Cir1_R_Sign * VertStVec_Left[i] *
                                               Rr; // update center point of circle 1
                    }
                    CenPtPowDist = pow2(Cir1_CenPt[0] - Cir2_CenPt[0]) +
                                   pow2(Cir1_CenPt[1] - Cir2_CenPt[1]); // update
                    break;
                }
                else if (j == 1)
                {
                    Get_Rst = 0;
                }
            }
        }
        else
        {
            return 0;
        }
    }
    if (Get_Rst == 0)
    {
        return 0;
    }

    memcpy(New_FinPos, finpos, sizeof(New_FinPos));
    // first try to move finpos towards stpos to make circle 1 and circle 2 tangle with
    // each other
    b = 2 * sign(fx) *
        ((Cir2_CenPt[0] - Cir1_CenPt[0]) * cosf(finpos[2]) +
         (Cir2_CenPt[1] - Cir1_CenPt[1]) * sinf(finpos[2]));
    c     = CenPtPowDist - 4.0f * pow2(Rrmin);
    Delta = b * b - 4.0f * c; //  a = 1.0f;
    if (Delta >= 0)
    {
        for (i = 0; i < 2; i++)
        {
            if (i == 0)
            {
                line_ds = (-b + sqrtf(Delta)) / 2.0f;
            }
            else
            {
                line_ds = (-b - sqrtf(Delta)) / 2.0f;
            }
            if (line_ds > 0 && line_ds < fabsf(rx))
            {
                NewCir2_CenPt[0] =
                    Cir2_CenPt[0] + cosf(finpos[2]) * line_ds; //  translate circle 2
                NewCir2_CenPt[1] = Cir2_CenPt[1] + sinf(finpos[2]) * line_ds;
                New_FinPos[0]    = finpos[0] + sign(fx) * cosf(finpos[2]) * line_ds;
                New_FinPos[1]    = finpos[1] + sign(fx) * sinf(finpos[2]) * line_ds;
                for (j = 0; j < 2; j++)
                {
                    TanPt[j] = (Cir1_CenPt[j] + NewCir2_CenPt[j]) / 2.0f;
                }
                rx_tanpt       = Project_PosTo1st_rx(New_Stpos, TanPt);
                fx_tanpt       = Project_PosTo1st_rx(finpos, TanPt);
                fx_temp_finpos = Project_PosTo1st_rx(finpos, New_FinPos);
                if (rx * rx_tanpt > 0 && fabsf(rx) > fabsf(rx_tanpt) &&
                    fx_temp_finpos * fx_tanpt > 0 &&
                    fabsf(fx_tanpt) > fabsf(fx_temp_finpos))
                {
                    Vec_1.vx    = TanPt[0] - Cir1_CenPt[0];
                    Vec_1.vy    = TanPt[1] - Cir1_CenPt[1];
                    Vec_2.vx    = New_Stpos[0] - Cir1_CenPt[0];
                    Vec_2.vy    = New_Stpos[1] - Cir1_CenPt[1];
                    cross_angle = fabsf(Get_CrossAngle_Vec2(Vec_1, Vec_2));
                    memcpy(TempPath[0], New_Stpos, sizeof(New_Stpos));
                    TempPath[0][3] = sign(rx) * cross_angle * Rr;
                    TempPath[0][4] = Cir1_R_Sign * Rr;
                    PK_Get_Path_EndPos(TempPath[0], EndPos);
                    Vec_1.vx    = EndPos[0] - Cir1_CenPt[0];
                    Vec_1.vy    = EndPos[1] - Cir1_CenPt[1];
                    Vec_2.vx    = New_FinPos[0] - Cir1_CenPt[0];
                    Vec_2.vy    = New_FinPos[1] - Cir1_CenPt[1];
                    cross_angle = fabsf(Get_CrossAngle_Vec2(Vec_1, Vec_2));
                    memcpy(TempPath[1], EndPos, sizeof(EndPos));
                    TempPath[1][3] = sign(rx) * cross_angle * Rr;
                    TempPath[1][4] = -Cir1_R_Sign * Rr;
                    PK_Get_Path_EndPos(TempPath[1], EndPos);
                    ds = Project_PosTo1st_rx(finpos, EndPos);
                    memcpy(TempPath[2], EndPos, sizeof(EndPos));
                    TempPath[2][3] = -ds;
                    TempPath[2][4] = 0;
                    PK_Get_Path_EndPos(TempPath[2], EndPos);
                    dPowPos = pow2(EndPos[0] - finpos[0]) + pow2(EndPos[1] - finpos[1]);
                    dth     = Round_PI(EndPos[2] - finpos[2]);
                    // 很少会执行的分支, 使用死循环用于测试是否正确.
                    // 可能原来的处理是错误的
                    if (dPowPos < dPowPos_max && fabsf(dth) < dth_max)
                    {
                        PathNum = 3;
                        printf("PathPlan_TwoEqualCircleCon_AllowStposRyErr 1!\n");
                        break;
                    }
                }
            }
        }
    }

    // if move cirlce 2 get no answer then move circle 1
    if (PathNum <= 0)
    {
        b = 2 * sign(rx) *
            ((Cir1_CenPt[0] - Cir2_CenPt[0]) * cosf(New_Stpos[2]) +
             (Cir1_CenPt[1] - Cir2_CenPt[1]) * sinf(New_Stpos[2]));
        Delta = pow2(b) - 4.0f * c;

        for (i = 0; i < 2 && Delta >= 0; i++)
        {
            if (i == 0)
            {
                line_ds = (-b + sqrtf(Delta)) / 2;
            }
            else
            {
                line_ds = (-b - sqrtf(Delta)) / 2;
            }
            if (line_ds > 0 && line_ds < fabsf(rx))
            {
                NewCir1_CenPt[0] =
                    Cir1_CenPt[0] + sign(rx) * line_ds * cosf(New_Stpos[2]);
                NewCir1_CenPt[1] =
                    Cir1_CenPt[1] + sign(rx) * line_ds * sinf(New_Stpos[2]);
                Temp_StPos[0] = New_Stpos[0] + sign(rx) * line_ds * cosf(New_Stpos[2]);
                Temp_StPos[1] = New_Stpos[1] + sign(rx) * line_ds * sinf(New_Stpos[2]);
                for (j = 0; j < 2; j++)
                {
                    TanPt[j] = (NewCir1_CenPt[j] + Cir2_CenPt[j]) / 2.0f;
                }
                rx_tanpt      = Project_PosTo1st_rx(New_Stpos, TanPt);
                fx_tanpt      = Project_PosTo1st_rx(finpos, TanPt);
                rx_temp_stpos = Project_PosTo1st_rx(New_Stpos, Temp_StPos);
                fx_temp_stpos = Project_PosTo1st_rx(finpos, Temp_StPos);
                if (rx_tanpt * rx_temp_stpos > 0 &&
                    fabsf(rx_tanpt) > fabsf(rx_temp_stpos) &&
                    fx_tanpt * fx_temp_stpos > 0 &&
                    fabsf(fx_temp_stpos) > fabsf(fx_tanpt))
                {
                    memcpy(TempPath[0], New_Stpos, sizeof(New_Stpos));
                    TempPath[0][3] = sign(rx) * line_ds;
                    TempPath[0][4] = 0;
                    PK_Get_Path_EndPos(TempPath[0], EndPos);
                    Vec_1.vx    = TanPt[0] - NewCir1_CenPt[0];
                    Vec_1.vy    = TanPt[1] - NewCir1_CenPt[1];
                    Vec_2.vx    = Temp_StPos[0] - NewCir1_CenPt[0];
                    Vec_2.vy    = Temp_StPos[1] - NewCir1_CenPt[1];
                    cross_angle = fabsf(Get_CrossAngle_Vec2(Vec_1, Vec_2));
                    memcpy(TempPath[1], EndPos, sizeof(EndPos));
                    TempPath[1][3] = sign(rx) * cross_angle * Rr;
                    TempPath[1][4] = Cir1_R_Sign * Rr;
                    PK_Get_Path_EndPos(TempPath[1], EndPos);
                    Vec_1.vx    = EndPos[0] - Cir2_CenPt[0];
                    Vec_1.vy    = EndPos[1] - Cir2_CenPt[1];
                    Vec_2.vx    = finpos[0] - Cir2_CenPt[0];
                    Vec_2.vy    = finpos[1] - Cir2_CenPt[1];
                    cross_angle = fabsf(Get_CrossAngle_Vec2(Vec_1, Vec_2));
                    memcpy(TempPath[2], EndPos, sizeof(EndPos));
                    TempPath[2][3] = sign(rx) * cross_angle * Rr;
                    TempPath[2][4] = -Cir1_R_Sign * Rr;
                    PK_Get_Path_EndPos(TempPath[2], EndPos);
                    dPowPos = pow2(EndPos[0] - finpos[0]) + pow2(EndPos[1] - finpos[1]);
                    dth     = Round_PI(EndPos[2] - finpos[2]);
                    // 很少会执行的分支, 使用死循环用于测试是否正确.
                    // 可能原来的处理是错误的
                    if (dPowPos < dPowPos_max && fabsf(dth) < dth_max)
                    {
                        PathNum = 3;
                        printf("PathPlan_TwoEqualCircleCon_AllowStposRyErr 2!!\n");
                        break;
                    }
                }
            }
        }
    }

    if (PathNum > 0 && PK_Check_Path(PathNum, TempPath, stpos, finpos, objnum, slotobj,
                                     swell) == PATHPLAN_SUCCESS) //
    {
        memcpy(expTraj, TempPath, sizeof(TempPath));
    }
    else
    {
        PathNum = 0;
        printf("PathPlan_TwoEqualCircleCon_AllowStposRyErr PK_Check_Path failed!\n");
    }

    return PathNum;
}

static int PathPlan_Line_Connection(float stpoint[3], PlanDataCase PlanData_Ver[1],
                                    float Exp_traj[][TRAJITEM_LEN])
{
    int line_num = 0;
    float endPos[3];

    float rtheta = Project_PosTo1st_rtheta(stpoint, PlanData_Ver[0].finpoint);
    /*直线*/
    float rx = Project_PosTo1st_rx(stpoint, PlanData_Ver[0].finpoint);
    float ry = Project_PosTo1st_ry(stpoint, PlanData_Ver[0].finpoint);

    if (fabs(rtheta) < 0.05 && fabs(ry) < 0.05)
    {
        PK_CopyPos(Exp_traj[0], stpoint);
        Exp_traj[0][3] = rx;
        Exp_traj[0][4] = 0;

        PK_Get_Path_EndPos(Exp_traj[0], endPos);
        ry     = Project_PosTo1st_ry(endPos, PlanData_Ver[0].finpoint);
        rtheta = Project_PosTo1st_rtheta(endPos, PlanData_Ver[0].finpoint);
        if (fabs(ry) < 0.05 && fabs(rtheta) < 0.05)
        {
            line_num = 1;
        }

        if (line_num > 0 &&
            PK_Check_Path(line_num, Exp_traj, PlanData_Ver[0].stpoint,
                          PlanData_Ver[0].finpoint, PlanData_Ver[0].obj_danger_num,
                          PlanData_Ver[0].obj_danger,
                          0) != PATHPLAN_SUCCESS) // 1.5f*PlanData_Ver[0].swellAny small
                                                  // movement or Path damage
        {
            line_num = 0;
        }
    }

    return line_num;
}

void Replan_When_Traj_Stpos_Is_Not_CurPos(
    PlanDataCase PlanDataVer[1], MultiPlanInfo &MultiPlanInfo,
    SlotInfo_T Multi_Slot_Array[MAX_PARKED_SLOTS], const int slotNum,
    uint8 IsTrajStPos[MAX_PARKED_SLOTS], int FS_ObjDir[FS_OBJ_ARR_NUM],
    float FS_Obj[FS_OBJ_ARR_NUM][4], int FS_ObjDir_Rear[FS_OBJ_ARR_NUM],
    float FS_Obj_Rear[FS_OBJ_ARR_NUM][4])
{
    int i = 0, slotindex = 0, tempPathNum = 0;
    int IsDirConnect                                            = 0;
    float curPlannedPath[4 * MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN] = {{0.0f}};

    for (i = 0; i < MultiPlanInfo.slotNum; i++)
    {
        if (IsTrajStPos[i] == 0) //  should replan
        {
            for (slotindex = 0; slotindex < slotNum; slotindex++)
            {
                if (MultiPlanInfo.slotPlans[i].Slot_index ==
                    Multi_Slot_Array[slotindex].slot_index)
                {
                    break;
                }
            }

            if (slotindex >= slotNum)
            {
                break;
            }

            memcpy(PlanDataVer[0].finpoint, &Multi_Slot_Array[slotindex].targpos,
                   sizeof(PlanDataVer[0].finpoint));
            SlotObj_Convert_float(Multi_Slot_Array[slotindex].slotobj,
                                  PlanDataVer[0].obj_slot);
            PlanDataVer[0].finpoint[2] = Round_PI(PlanDataVer[0].finpoint[2]);
            PathPlan_RangeDataProcess(PlanDataVer, Multi_Slot_Array[slotindex], FS_ObjDir,
                                      FS_Obj, FS_ObjDir_Rear, FS_Obj_Rear);

            memset(curPlannedPath, 0, sizeof(curPlannedPath));
            // tempPathNum = PathPlan_GetTraj(PlanDataVer, curPlannedPath, &IsDirConnect);
            if (tempPathNum > 0)
            {
                if (PathPlan_CheckPathOverFlow(&curPlannedPath[0][0]) != 0)
                {
                    printf(
                        "target plan path overflow ! index %d, number id: %d target: %f "
                        "%f %f\n",
                        i, Multi_Slot_Array[i].slot_index, PlanDataVer[0].finpoint[0],
                        PlanDataVer[0].finpoint[1], PlanDataVer[0].finpoint[2]);
                }
                else
                {
                    memcpy(MultiPlanInfo.slotPlans[i].Act_traj, curPlannedPath,
                           tempPathNum * PATH_ITEM_LEN); //  change the old traj
                    MultiPlanInfo.slotPlans[i].Path_num    = tempPathNum;
                    MultiPlanInfo.slotPlans[i].IsDirConect = IsDirConnect;
                }
            }
        }
    }
}

static int PathPlan_SearchMinDsTra(const float left_fac, const float stpoint[3],
                                   const float finpoint[3], const int obj_num,
                                   const float obj_slot[][4], float &swell,
                                   float Plan_Path[MAX_PARKED_SLOTS][TRAJITEM_LEN])
{
    const int trycount = 5;
    static float Planned_Path_temp[trycount][6][TRAJITEM_LEN];
    float Rr_array[trycount] = {
        Rrmin, 5.4f, 0, -5.4f,
        -Rrmin}; // 4.2f  0831: revise min Radium with G3 //20180809: the search dirction
                 // can be known by the relative theta between stpos and finpos
    float tra_dist_sum[trycount] = {0};
    int tra_num_temp[trycount]   = {0};
    float temp_ds_sum            = 0;
    float temp_tra[trycount]     = {0};
    int SearchPath_num = 0, BestPath_index = -1, tra_num = 0;
    float sideSlotObj[2][4]; //  BC,DE
    memcpy(sideSlotObj[0], obj_slot[1], 4 * sizeof(float));
    memcpy(sideSlotObj[1], obj_slot[3], 4 * sizeof(float));
    PK_CopyPos(temp_tra, stpoint);
    if (left_fac < 0)
    {
        for (int i = 0; i < 5; i++)
        {
            Rr_array[i] = -Rr_array[i];
        }
    }

    float checkSwell = 0.6;
    do
    {
        SearchPath_num = 0;
        for (int i = 0; i < 5; i++) // four kind of plan;
        {
            temp_tra[4] = Rr_array[i];
            temp_tra[3] = 2.5f; //  0831: this value is restricted by r and obstacles
            tra_num_temp[i] =
                PK_PathPlan_TraExplorer_Con(temp_tra, finpoint, obj_num, obj_slot,
                                            Planned_Path_temp[i], 0, 1, checkSwell, 0.5f);
            // CSJ 0831: should check the path!!!
            if (tra_num_temp[i] > 0 && tra_num_temp[i] < 7 &&
                PK_Check_Path(tra_num_temp[i], Planned_Path_temp[i], stpoint, finpoint, 2,
                              sideSlotObj, checkSwell) == PATHPLAN_SUCCESS)
            {
                SearchPath_num++;
                for (int j = 0; j < tra_num_temp[i]; j++)
                {
                    tra_dist_sum[i] = tra_dist_sum[i] + fabsf(Planned_Path_temp[i][j][3]);
                }
            }
            else
            {
                tra_num_temp[i] = 0;
            }
        }

        if (SearchPath_num > 0)
        {
            break;
        }
        checkSwell = checkSwell - 0.05;
    } while (checkSwell > swell);

    if (SearchPath_num > 0)
    {
        printf("PathPlan_SearchMinDsTra traj num %d, check swell %06f, swell %06f\n",
               SearchPath_num, checkSwell, swell);
        swell = checkSwell;
        for (int i = 0; i < 5; i++)
        {
            if (tra_num_temp[i] > 0)
            {
                if (temp_ds_sum < 0.01)
                {
                    temp_ds_sum    = tra_dist_sum[i];
                    BestPath_index = i;
                }
                else
                {
                    if (temp_ds_sum > tra_dist_sum[i])
                    {
                        temp_ds_sum    = tra_dist_sum[i];
                        BestPath_index = i;
                    }
                }
            }
        }
        memcpy(Plan_Path, Planned_Path_temp[BestPath_index], 30 * sizeof(float));
        tra_num = tra_num_temp[BestPath_index];
    }
    else
    {
        tra_num = 0;
    }
    return tra_num;
}

int GetPath_StToMidPos_Na(PlanDataCase PlanData_Ver[1],
                          float StToMid_Traj[3][TRAJITEM_LEN])
{
    Point_T SlotE, SlotB, newCurPt, newCurPt_1;
    float R_inner, crossAngle, E_Cir_theta, ry, dist, rx_cir, rth, CirPt[2]; // run_dist;
    const float ds_err        = 0.01f; //  to avoid can't pass the collision check
    float verCurPosLeftVec[2] = {-sinf(PlanData_Ver[0].stpoint[2]),
                                 cosf(PlanData_Ver[0].stpoint[2])};
    float EndPos[3];
    int TrajNum_NS = 0;
    float Nearest_dy;
    const float Near_dy_max    = 2.0f;  //  m,calibration with Rmin = 4.4m
    const float Near_theta_max = 0.59f; //  rad,calibration with Rmin = 4.4m
    const float p1 = 3.04f, p2 = -3.654f, p3 = -0.9056f; //  p1*th^2 + p2*th + p3 + dy = 0
    float theta_dy;
    float rx, ds;
    const float big_ds            = 5.0f; // set a big value pi/2*4.4 = 6.9m
    const float no_collision_dist = 0.05f;

    SlotB.x = PlanData_Ver[0].obj_slot[1][0];
    SlotB.y = PlanData_Ver[0].obj_slot[1][1];
    SlotE.x = PlanData_Ver[0].obj_slot[4][0];
    SlotE.y = PlanData_Ver[0].obj_slot[4][1];
    R_inner = Rrmin - VEHICLE_WID / 2;

    rx  = Project_PosTo1st_rx(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint);
    ry  = Project_PosTo1st_ry(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint);
    rth = Round_PI(PlanData_Ver[0].finpoint[2] - PlanData_Ver[0].stpoint[2]);

    newCurPt.x = PlanData_Ver[0].stpoint[0] + sign(ry) * verCurPosLeftVec[0] * Rrmin;
    newCurPt.y = PlanData_Ver[0].stpoint[1] + sign(ry) * verCurPosLeftVec[1] * Rrmin;

    newCurPt_1.x = newCurPt.x + cosf(PlanData_Ver[0].stpoint[2]);
    newCurPt_1.y = newCurPt.y + sinf(PlanData_Ver[0].stpoint[2]);
    Get_Dist_Dir_Pt2PointLine(newCurPt, newCurPt_1, SlotE, &dist, NULL);
    if (dist / R_inner <= 1) // asin()中的參數不能大於1 HAVECHANGE Tangsj 2023/5/29
    {
        crossAngle  = asin(dist / R_inner);
        E_Cir_theta = PlanData_Ver[0].stpoint[2] + sign(ry) * crossAngle;
        CirPt[0]    = SlotE.x + R_inner * cosf(E_Cir_theta);
        CirPt[1]    = SlotE.y + R_inner * sinf(E_Cir_theta);
    }
    else
    {
        newCurPt.x = PlanData_Ver[0].stpoint[0] + sign(ry) * verCurPosLeftVec[0] * Rrmin;
        newCurPt.y = PlanData_Ver[0].stpoint[1] + sign(ry) * verCurPosLeftVec[1] * Rrmin;
        newCurPt_1.x = newCurPt.x - cosf(PlanData_Ver[0].stpoint[2]);
        newCurPt_1.y = newCurPt.y - sinf(PlanData_Ver[0].stpoint[2]);
        Get_Dist_Dir_Pt2PointLine(newCurPt, newCurPt_1, SlotB, &dist, NULL);
        crossAngle  = asin(dist / R_inner);
        E_Cir_theta = PlanData_Ver[0].stpoint[2] + sign(ry) * crossAngle;
        CirPt[0]    = SlotB.x + R_inner * cosf(E_Cir_theta);
        CirPt[1]    = SlotB.y + R_inner * sinf(E_Cir_theta);
    }

    rx_cir = Project_PosTo1st_rx(PlanData_Ver[0].stpoint, CirPt);
    memcpy(StToMid_Traj[0], PlanData_Ver[0].stpoint, 3 * sizeof(float));
    StToMid_Traj[0][3] = rx_cir + sign(rx_cir) * ds_err;
    StToMid_Traj[0][4] = 0;

    PK_Get_Path_EndPos(StToMid_Traj[0], EndPos);
    memcpy(StToMid_Traj[1], EndPos, 3 * sizeof(float));
    StToMid_Traj[1][3] = big_ds * sign(rx);
    StToMid_Traj[1][4] = sign(rx_cir) * sign(rth) * Rrmin;
    PathPlan_LandMark2(StToMid_Traj[1], 5, PlanData_Ver[0].obj_slot, 0);
    TrajNum_NS = 2;
    PK_Get_Path_EndPos(StToMid_Traj[TrajNum_NS - 1], EndPos);
    if (PK_Check_Path(TrajNum_NS, StToMid_Traj, PlanData_Ver[0].stpoint, EndPos,
                      PlanData_Ver[0].obj_danger_num, PlanData_Ver[0].obj_danger,
                      0) != PATHPLAN_SUCCESS)
    {
        TrajNum_NS = 0; //  init
        Nearest_dy = Get_AnotherObs_Nearest_dy_to_CurPos(
            PlanData_Ver[0].stpoint, PlanData_Ver[0].slotshape,
            PlanData_Ver[0].obj_danger, PlanData_Ver[0].obj_danger_num);
        Nearest_dy = Nearest_dy - no_collision_dist;
        if (Nearest_dy <= Near_dy_max)
        {
            theta_dy =
                (-p2 + sqrtf(pow2(p2) - 4 * p1 * (Nearest_dy + p3))) / (2 * p1); // root 1
            if (theta_dy <= 0 || theta_dy > Near_theta_max)
            {
                theta_dy = (-p2 - sqrtf(pow2(p2) - 4 * p1 * (Nearest_dy + p3))) /
                           (2 * p1); // root 2
            }
            if (theta_dy > 0 && theta_dy <= Near_theta_max)
            {
                ds                 = theta_dy * Rrmin;
                StToMid_Traj[1][3] = ds * sign(rx);
                PK_Get_Path_EndPos(StToMid_Traj[1], EndPos);
                memcpy(StToMid_Traj[2], EndPos, 3 * sizeof(float));
                ds = big_ds - fabsf(StToMid_Traj[1][3]);
                if (ds > 0.1f)
                {
                    StToMid_Traj[2][3] = ds * sign(rx); // set a big value
                    StToMid_Traj[2][4] = 0;             //  go straight
                    PathPlan_LandMark2(StToMid_Traj[2], 5, PlanData_Ver[0].obj_slot, 0);
                    TrajNum_NS = 3;
                }
            }
        }
    }
    return TrajNum_NS;
}

// to expend obstacles a dist in the direction close to stpos
// 0912: only used for verticle slot!
void Trans_ObsObj_To_stPos_By_Swell(float swell_ABEF, float stPos[4], VehPos_T finPos,
                                    int obj_num, float obj[][4], uint8_t obj_dir[],
                                    int slotshap)
{
    int i;
    Vec2_T TarOutVec;
    float TarOutTheta = 0;
    float dx, dy;
    float swell = Cal_MinObsSideDist_CurPos(obj_num, obj, stPos) -
                  0.05f; // 20181022: 0.4f -> 0.3f to improve the success rate of plan
    float dir = 0.0f;
    swell     = rte_min(swell, swell_ABEF); //  20181023: maybe swell_ABEF is smaller
    swell = rte_max(0.18f, rte_min(swell, 0.3f)); //  20181023: the swell should update to
                                                  //  the minimun side dist at start pos
    if (slotshap > 2)
    {
        TarOutTheta = finPos.theta;
    }
    else
    {
        if (slotshap == 1)
        {
            TarOutTheta = finPos.theta - PI / 2.0f;
        }
        else
        {
            TarOutTheta = finPos.theta + PI / 2.0f;
        }
    }
    TarOutVec.vx = cosf(TarOutTheta);
    TarOutVec.vy = sinf(TarOutTheta);
    dx           = TarOutVec.vx * swell;
    dy           = TarOutVec.vy * swell;
    for (i = 0; i < obj_num; i++)
    {
        if ((PathPlan_SlotDir(slotshap) == 1 && obj_dir[i] == 2) ||
            (PathPlan_SlotDir(slotshap) == 2 && obj_dir[i] == 1))
        {
            dir = -1.0f;
        }
        else
        {
            dir = 1.0f;
        }
        obj[i][0] += dir * dx;
        obj[i][1] += dir * dy;
        obj[i][2] += dir * dx;
        obj[i][3] += dir * dy;
    }
}

// calculate the minimum side dist from vehicle to obstacles
float Cal_MinObsSideDist_CurPos(int obsnum, float obsobj[][4], float curpos[4])
{
    int i;
    float sideDist_min = 1e6;
    float rx1, rx2, ry1, ry2, side_dist;
    for (i = 0; i < obsnum; i++)
    {
        rx1 = Project_PosTo1st_rx(curpos, &obsobj[i][0]);
        if (rx1 > -REAR_SUSPENSION && rx1 < VEHICLE_LEN - REAR_SUSPENSION)
        {
            ry1       = Project_PosTo1st_ry(curpos, &obsobj[i][0]);
            ry2       = Project_PosTo1st_ry(curpos, &obsobj[i][2]);
            side_dist = rte_min(fabsf(ry1), fabsf(ry2)) - VEHICLE_WID / 2.0f;
            if (side_dist > 0 && side_dist < sideDist_min)
            {
                sideDist_min = side_dist;
            }
        }
        else
        {
            rx2 = Project_PosTo1st_rx(curpos, &obsobj[i][2]); //  to decent calculation
            if (rx2 > -REAR_SUSPENSION && rx2 < VEHICLE_LEN - REAR_SUSPENSION)
            {
                ry1       = Project_PosTo1st_ry(curpos, &obsobj[i][0]);
                ry2       = Project_PosTo1st_ry(curpos, &obsobj[i][2]);
                side_dist = rte_min(fabsf(ry1), fabsf(ry2)) - VEHICLE_WID / 2.0f;
                if (side_dist > 0 && side_dist < sideDist_min)
                {
                    sideDist_min = side_dist;
                }
            }
        }
    }
    return sideDist_min;
}

/**
 * @brief
 *
 * @param
 * @return
 */
#if 0
int PK_PathPlan_RangeDataProcess(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN], const float finpoint[3], const float enviObj[5][4], const float curpos[4], const int slotshap, const int is_vison_slot)
{
    PlanDataCase PlanData_Ver[1] = {{0}};
    //障碍物
    FusionObj_T FS_Info = {{0}};
    float rect_pr_possb[4] = {0};
    float Fusion_Obj_A[FS_OBJ_ARR_NUM][4] = {{0.0f}};
    int  Fusion_ObjDir_A[FS_OBJ_ARR_NUM] = {0};
    float Fusion_Obj_B[FS_OBJ_ARR_NUM][4] = {{0.0f}};
    int  Fusion_ObjDir_B[FS_OBJ_ARR_NUM] = { 0 };

//    float Fusion_Obj_temp_A[FS_OBJ_ARR_NUM][4] = {{0.0f}};
//    int Fusion_ObjDir_temp_A[FS_OBJ_ARR_NUM] = {0};
//   float Fusion_Obj_temp_B[FS_OBJ_ARR_NUM][4] = {{0.0f}};
//    int Fusion_ObjDir_temp_B[FS_OBJ_ARR_NUM] = {0};
    //float rx_fin = Project_PosTo1st_rx(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint);//
    float ry_fin = Project_PosTo1st_ry(curpos, finpoint);//

    VehPos_T stPos,targPos;
    SlotObj_T slotObj;

    PlanData_Ver[0].left_fac = (ry_fin > 0) ? 1.0 : -1.0;
    PlanData_Ver[0].obj_danger_num = 0;
    PlanData_Ver[0].slotshape = slotshap;
    PlanData_Ver[0].is_vision_slot = is_vison_slot;

    memcpy(PlanData_Ver[0].stpoint, curpos, sizeof(PlanData_Ver[0].stpoint));
    memcpy(PlanData_Ver[0].finpoint, finpoint, sizeof(PlanData_Ver[0].stpoint));
    memcpy(PlanData_Ver[0].obj_slot, enviObj, sizeof(PlanData_Ver[0].obj_slot));

    PlanData_Ver[0].len   = VEHICLE_LEN;
    PlanData_Ver[0].width = VEHICLE_WID;
    PlanData_Ver[0].h     = REAR_SUSPENSION;

    PlanData_Ver[0].pointb_danger[0] = PlanData_Ver[0].obj_slot[0][2];  //  slot point B
    PlanData_Ver[0].pointb_danger[1] = PlanData_Ver[0].obj_slot[0][3];  //  slot point B
    PlanData_Ver[0].pointf_danger[0] = PlanData_Ver[0].obj_slot[3][2];  //  slot point E
    PlanData_Ver[0].pointf_danger[1] = PlanData_Ver[0].obj_slot[3][3];  //  slot point E
    // 20180702：this judgement method is not stable

    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right(&FS_Info);
    BuildFusionObsInfo(FS_Info, 2, &Fusion_Obj_A[SF_OBJ_NUM], &Fusion_ObjDir_A[SF_OBJ_NUM]);
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Right(&FS_Info);
    BuildFusionObsInfo(FS_Info, 2, &Fusion_Obj_B[SF_OBJ_NUM], &Fusion_ObjDir_B[SF_OBJ_NUM]);

    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left(&FS_Info);
    BuildFusionObsInfo(FS_Info, 1, &Fusion_Obj_A[0], &Fusion_ObjDir_A[0]);
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Left(&FS_Info);
    BuildFusionObsInfo(FS_Info, 1, &Fusion_Obj_B[0], &Fusion_ObjDir_B[0]);

    PathPlan_ReportObjsInfo(Fusion_ObjDir_A, Fusion_Obj_A, Fusion_ObjDir_B, Fusion_Obj_B);

    /**调试用*******************************/
    PlanInfoType plansInfo;
    RTE_PK_DataConvt_Get_Target_PlanInfo(&plansInfo);
    Multi_Slot_Array_T multiSlots;
    RTE_PK_SlotDetect_Get_Multi_SlotInfo(&multiSlots);

    int targ_slot_counter = -1;
    for (int i = 0; i < multiSlots.multiNum; i++)
    {
        if (plansInfo.Slot_index == multiSlots.multiArray[i].slot_index)
        {
            targ_slot_counter = i;
        }
    }

    if (targ_slot_counter == -1)
    {
        log_warn("slotIndex is not find! planIndex: %d\n", plansInfo.Slot_index);
        return 0;
    }

    PathPlan_ReportSlotsInfo(multiSlots.multiArray[targ_slot_counter], curpos);
    /*********************************/
    // 当车辆泊入途中，车辆与车位的相对姿态改变，此时不适用根据车辆位置姿态划分障碍物类型
    // CheckObjType(curpos, finpoint, slotshap, Fusion_Obj_A, Fusion_ObjDir_A, Fusion_Obj_temp_A, Fusion_ObjDir_temp_A);
    // CheckObjType(curpos, finpoint, slotshap, Fusion_Obj_B, Fusion_ObjDir_B, Fusion_Obj_temp_B, Fusion_ObjDir_temp_B);
    if (PlanData_Ver[0].slotshape == PK_SLOT_LEFT_PARA || PlanData_Ver[0].slotshape == PK_SLOT_RIGHT_PARA)   //  parallel slot
    {
        memset(&Fusion_ObjDir_A[SF_OBJ_NUM], 0, sizeof(int) * SF_OBJ_NUM);
        memset(&Fusion_ObjDir_B[SF_OBJ_NUM], 0, sizeof(int) * SF_OBJ_NUM);
    }
    else
    {
        memset(&Fusion_ObjDir_A[0], 0, sizeof(int) * SF_OBJ_NUM);
        memset(&Fusion_ObjDir_B[0], 0, sizeof(int) * SF_OBJ_NUM);
    }

    if (PlanData_Ver[0].slotshape == PK_SLOT_LEFT_PARA || PlanData_Ver[0].slotshape == PK_SLOT_RIGHT_PARA)   //  parallel slot
    {
        PlanData_Ver[0].pointf_danger[2] = PlanData_Ver[0].finpoint[2]; //  along the direction while searching slot
        PlanData_Ver[0].pointb_danger[2] = PlanData_Ver[0].finpoint[2];
    }
    else
    {
        PlanData_Ver[0].pointf_danger[2] = PlanData_Ver[0].finpoint[2] + PlanData_Ver[0].left_fac*PI / 2.0f;    //  along the direction while searching slot
        PlanData_Ver[0].pointb_danger[2] = PlanData_Ver[0].pointf_danger[2];
    }

    PlanData_Ver[0].close_side_dist = fabsf(Project_PosTo1st_ry( PlanData_Ver[0].finpoint, PlanData_Ver[0].obj_slot[2]))- PlanData_Ver[0].width/2.0f;
    PlanData_Ver[0].swell = 0.35;

    memcpy(&stPos,PlanData_Ver[0].stpoint,sizeof(stPos));
    memcpy(&targPos,PlanData_Ver[0].finpoint,sizeof(targPos));
    SlotObj_Convert_struct(&slotObj,PlanData_Ver[0].obj_slot);

    Get_Plan_RangeArea(PlanData_Ver[0].stpoint, PlanData_Ver[0].stpoint[2], PlanData_Ver[0].obj_slot, rect_pr_possb, PlanData_Ver[0].slotshape);//获取路线规划区域

    // 得到障碍物
    PathPlan_BuildSlotObjDanger(PlanData_Ver, multiSlots.multiArray[targ_slot_counter], rect_pr_possb, Fusion_ObjDir_A, Fusion_Obj_A, Fusion_ObjDir_B, Fusion_Obj_B);

    //限位杆
    if (PlanData_Ver[0].slotshape <= PK_SLOT_RIGHT_PARA)
    {
        LineSeg_T parkBars[2];
        int counter = PathPlan_BuildObjsByStopper((const SlotInfo_T&)slotObj, parkBars);
        if (counter == 1 && PlanData_Ver[0].obj_danger_num < FS_OBJ_ARR_NUM - 1)
        {
            memcpy(PlanData_Ver[0].obj_danger[PlanData_Ver[0].obj_danger_num], &parkBars[0], sizeof(LineSeg_T));
            PlanData_Ver[0].obj_danger_dir[PlanData_Ver[0].obj_danger_num] = (PlanData_Ver[0].slotshape == PK_SLOT_LEFT_PARA) ? 1 : 2;
            PlanData_Ver[0].obj_danger_num++;
        }
        else if (counter == 2 && PlanData_Ver[0].obj_danger_num < FS_OBJ_ARR_NUM - 2)
        {
            memcpy(PlanData_Ver[0].obj_danger[PlanData_Ver[0].obj_danger_num], &parkBars[0], sizeof(LineSeg_T));
            PlanData_Ver[0].obj_danger_dir[PlanData_Ver[0].obj_danger_num] = (PlanData_Ver[0].slotshape == PK_SLOT_LEFT_PARA) ? 1 : 2;
            PlanData_Ver[0].obj_danger_num++;

            memcpy(PlanData_Ver[0].obj_danger[PlanData_Ver[0].obj_danger_num], &parkBars[1], sizeof(LineSeg_T));
            PlanData_Ver[0].obj_danger_dir[PlanData_Ver[0].obj_danger_num] = (PlanData_Ver[0].slotshape == PK_SLOT_LEFT_PARA) ? 1 : 2;
            PlanData_Ver[0].obj_danger_num++;
        }
    }
    //  0906: for obs in slot ingored by slotdetect!
    int result = PathPlan_UpdateSlotSwell(PlanData_Ver[0]);
    if (result <= 0) {
        log_info("PathPlan_UpdateSlotSwell6 failed!\n");
        return 0;
    }

    
    PathPlan_ExpandObjBySwell(PlanData_Ver[0].swell, PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint, PlanData_Ver[0].obj_danger_num, PlanData_Ver[0].obj_danger, PlanData_Ver[0].obj_danger_dir, PlanData_Ver[0].slotshape);

    PlanData_Ver->rear_safe_dist = PlanData_Ver->rear_dist - PlanData_Ver[0].swell;
    PlanData_Ver[0].rect_ds_max = PlanData_Ver->rear_safe_dist;
    PlanData_Ver[0].swell = 0;

    int IsDirConnect = 0;
    int pathNum = PathPlan_GetTraj(PlanData_Ver,plannedPath, &IsDirConnect);

    if(pathNum > 0)
    {
        if (PathPlan_CheckPathOverFlow(&plannedPath[0][0]) != 0)
        {
            printf("target plan path overflow ! target: %f %f %f slotinfo: %d %d\n",
                PlanData_Ver[0].finpoint[0], PlanData_Ver[0].finpoint[1], PlanData_Ver[0].finpoint[2], PlanData_Ver[0].is_vision_slot, PlanData_Ver[0].slotshape);
            pathNum = 0;
        }
        else
        {
            pathNum = PK_PathMerge(plannedPath, pathNum, plansInfo.Slot_index);
            PathPlan_ReportTrajsInfo(plannedPath, pathNum, PlanData_Ver[0], plansInfo.Slot_index);
        }
    }

    PathPlan_ReportTrajsInfo(plannedPath, pathNum, PlanData_Ver[0], plansInfo.Slot_index);

    return pathNum;
}

#endif