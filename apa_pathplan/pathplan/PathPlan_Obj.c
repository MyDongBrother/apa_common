#include "PathPlan_Func.h"
#include "PathPlan_Config.h"
#include "SystemPara.h"
#include "PK_PathPlanP.h"
#include "MathFunc.h"
#include "Record_Log.h"

static bool IsSameObj(const float obj1[4], const float obj2[4])
{
    float dist1 = Cal_Dis_Pt2Pt(&obj1[0], &obj2[0]);
    float dist2 = Cal_Dis_Pt2Pt(&obj1[0], &obj2[2]);
    float dist3 = Cal_Dis_Pt2Pt(&obj1[2], &obj2[0]);
    float dist4 = Cal_Dis_Pt2Pt(&obj1[2], &obj2[2]);

    if (dist1 < 0.15 && dist4 < 0.15)
    {
        return true;
    }
    if (dist2 < 0.15 && dist3 < 0.15)
    {
        return true;
    }

    float center1[2] = {0.5f * (obj1[0] + obj1[2]), 0.5f * (obj1[1] + obj1[3])};
    float center2[2] = {0.5f * (obj2[0] + obj2[2]), 0.5f * (obj2[1] + obj2[3])};

    dist1 = Cal_Dis_Pt2Pt(center1, center2);
    Vec2_T v1, v2;
    v1.vx = obj1[0] - obj1[2];
    v1.vy = obj1[1] - obj1[3];
    v2.vx = obj2[0] - obj2[2];
    v2.vy = obj2[1] - obj2[3];

    Normalize_Vec2(&v1);
    Normalize_Vec2(&v2);
    float dotV = Get_Dot_Vec2(v1, v2);

    if (dist1 < 0.15 && dotV > 0.95)
    {
        return true;
    }

    return false;
}

static bool IsSlotContained(const float slot[5][4], const float line[4])
{
    Quad_T quad;
    memcpy((float *)&quad.pt_fl, &slot[3][2], sizeof(Point_T));
    memcpy((float *)&quad.pt_fr, &slot[3][0], sizeof(Point_T));
    memcpy((float *)&quad.pt_rr, &slot[1][2], sizeof(Point_T));
    memcpy((float *)&quad.pt_rl, &slot[1][0], sizeof(Point_T));
    Point_T point[2];
    memcpy((float *)&point[0], line, sizeof(point));
    if (Is_Point_In_Quad(quad, point[0]) == PK_OUTSIDE ||
        Is_Point_In_Quad(quad, point[1]) == PK_OUTSIDE)
    {
        return false;
    }
    return true;
}

static int BuildObjByRadar(float alarmObj[4][4])
{
    const float objswell = 0.4;
    U_RadarType uradar;
    RTE_PD_Get_U_Radar(&uradar); // 超声波实时反馈数据

    uint8_t alarmObjNum = 0;
    if (rte_max(uradar.FCL, uradar.FCR) < 1200 && rte_min(uradar.FCL, uradar.FCR) > 0)
    {
        float radardist = rte_min(uradar.FCL, uradar.FCR);
        radardist       = radardist / 1000.0 + VEHICLE_LEN - REAR_SUSPENSION - objswell;
        alarmObj[alarmObjNum][0] = radardist;
        alarmObj[alarmObjNum][2] = radardist;
        alarmObj[alarmObjNum][1] = 0.3;
        alarmObj[alarmObjNum][3] = VEHICLE_WID * 0.6;
        alarmObjNum++;

        alarmObj[alarmObjNum][0] = radardist;
        alarmObj[alarmObjNum][2] = radardist;
        alarmObj[alarmObjNum][1] = -0.3;
        alarmObj[alarmObjNum][3] = -VEHICLE_WID * 0.6;
        alarmObjNum++;
    }
    else if (rte_max(uradar.FCL, uradar.FOL) < 1200 &&
             rte_min(uradar.FCL, uradar.FOL) > 0)
    {
        float radardist = rte_min(uradar.FCL, uradar.FOL);
        radardist       = radardist / 1000.0 + VEHICLE_LEN - REAR_SUSPENSION - objswell;
        alarmObj[alarmObjNum][0] = radardist;
        alarmObj[alarmObjNum][2] = radardist;
        alarmObj[alarmObjNum][1] = 0.3;
        alarmObj[alarmObjNum][3] = VEHICLE_WID * 0.6;
        alarmObjNum++;
    }
    else if (rte_max(uradar.FCR, uradar.FOR) < 1200 &&
             rte_min(uradar.FCR, uradar.FOR) > 0)
    {
        float radardist = rte_min(uradar.FCR, uradar.FOR);
        radardist       = radardist / 1000.0 + VEHICLE_LEN - REAR_SUSPENSION - objswell;
        alarmObj[alarmObjNum][0] = radardist;
        alarmObj[alarmObjNum][2] = radardist;
        alarmObj[alarmObjNum][1] = -0.3;
        alarmObj[alarmObjNum][3] = -VEHICLE_WID * 0.6;
        alarmObjNum++;
    }

    if (rte_max(uradar.RCL, uradar.RCR) < 1200 && rte_min(uradar.RCL, uradar.RCR) > 0)
    {
        float radardist = rte_min(uradar.RCL, uradar.RCR);
        radardist       = radardist / 1000.0 + REAR_SUSPENSION - objswell;

        alarmObj[alarmObjNum][0] = -radardist;
        alarmObj[alarmObjNum][2] = -radardist;
        alarmObj[alarmObjNum][1] = 0.3;
        alarmObj[alarmObjNum][3] = VEHICLE_WID * 0.6;
        alarmObjNum++;

        alarmObj[alarmObjNum][0] = -radardist;
        alarmObj[alarmObjNum][2] = -radardist;
        alarmObj[alarmObjNum][1] = -0.3;
        alarmObj[alarmObjNum][3] = -VEHICLE_WID * 0.6;
        alarmObjNum++;
    }
    else if (rte_max(uradar.RCL, uradar.ROL) < 1200 &&
             rte_min(uradar.RCL, uradar.ROL) > 0)
    {
        float radardist          = rte_min(uradar.RCL, uradar.ROL);
        radardist                = radardist / 1000.0 + REAR_SUSPENSION - objswell;
        alarmObj[alarmObjNum][0] = -radardist;
        alarmObj[alarmObjNum][2] = -radardist;
        alarmObj[alarmObjNum][1] = 0.3;
        alarmObj[alarmObjNum][3] = VEHICLE_WID * 0.6;
        alarmObjNum++;
    }
    else if (rte_max(uradar.RCR, uradar.ROR) < 1200 &&
             rte_min(uradar.RCR, uradar.ROR) > 0)
    {
        float radardist          = rte_min(uradar.RCR, uradar.ROR);
        radardist                = radardist / 1000.0 + REAR_SUSPENSION - objswell;
        alarmObj[alarmObjNum][0] = -radardist;
        alarmObj[alarmObjNum][2] = -radardist;
        alarmObj[alarmObjNum][1] = -0.3;
        alarmObj[alarmObjNum][3] = -VEHICLE_WID * 0.6;
        alarmObjNum++;
    }

    return alarmObjNum;
}

void PathPlan_DyanObj(const PlanDataCase planData[1], FusionObj_T &extObj,
                      const float curPoint[3])
{
    // 过滤属性不为0的对象
    for (int i = 0; i < extObj.num; i++)
    {
        if (extObj.attr[i] != 0)
        {
            float rsPt[2][2];
            RevConvert(curPoint, (float *)&extObj.obj[i].pt1, rsPt[0]);
            RevConvert(curPoint, (float *)&extObj.obj[i].pt2, rsPt[1]);

            bool isValid = rsPt[0][0] >= -REAR_SUSPENSION &&
                           rsPt[0][0] < VEHICLE_LEN - REAR_SUSPENSION &&
                           rsPt[1][0] >= -REAR_SUSPENSION &&
                           rsPt[1][0] < VEHICLE_LEN - REAR_SUSPENSION &&
                           fabs(rsPt[0][1]) < 0.5 * VEHICLE_WID &&
                           fabs(rsPt[1][1]) < 0.5 * VEHICLE_WID;

            extObj.attr[i] = isValid ? 0 : 40000;
        }
    }

    // 压缩有效对象
    int count = 0;
    for (int i = 0; i < extObj.num; i++)
    {
        if (extObj.attr[i] != 0)
        {
            if (count != i)
            {
                extObj.attr[count] = extObj.attr[i];
                extObj.ang[count]  = extObj.ang[i];
                extObj.odom[count] = extObj.odom[i];
                memcpy(&extObj.obj[count], &extObj.obj[i], sizeof(LineSeg_T));
                extObj.attr[i] = 0;
            }
            count++;
        }
    }
    extObj.num = count;

    // 获取停车对象并合并
    extern void PK_PathExec_GetParkObj(FusionObj_T & fusion);
    FusionObj_T parkObj;
    PK_PathExec_GetParkObj(parkObj);
    for (int i = 0; i < parkObj.num && count < SF_OBJ_NUM; i++)
    {
        if (parkObj.attr[i] == 0)
        {
            continue;
        }
        int j = 0;
        for (j = 0; j < count; j++)
        {
            if (IsSameObj((float *)&extObj.obj[j], (float *)&parkObj.attr[i]))
            {
                break;
            }
        }

        if (j >= count)
        {
            extObj.attr[count] = 80000;
            extObj.ang[count]  = parkObj.ang[i];
            extObj.odom[count] = parkObj.odom[i];
            memcpy(&extObj.obj[count], &parkObj.obj[i], sizeof(LineSeg_T));
            count++;
        }
    }

    // 处理雷达对象
    float alarmObj[4][4];
    int objNum = BuildObjByRadar(alarmObj);
    for (int i = 0; i < objNum && count < SF_OBJ_NUM; i++)
    {
        if (IsSlotContained(planData[0].obj_slot, alarmObj[i]))
        {
            continue;
        }

        float obj[4];
        Convert(curPoint, &alarmObj[i][0], &obj[0]);
        Convert(curPoint, &alarmObj[i][2], &obj[2]);
        int j = 0;
        for (j = 0; j < count; j++)
        {
            if (IsSameObj((float *)&extObj.obj[j], obj))
            {
                break;
            }
        }

        if (j >= count)
        {
            extObj.attr[count] = 90000;
            memcpy(&extObj.obj[count], obj, sizeof(LineSeg_T));
            count++;
        }
    }

    extObj.num = count;
    if (extObj.num >= SF_OBJ_NUM)
    {
        log_err("DynObj overflow!\n");
    }
}

int PathPlan_PathVerifyFusion(const float path[TRAJITEM_LEN],
                              const FusionObj_T &FS_Info_A, const float swell)
{
    for (int i = 0; i < FS_Info_A.num; i++)
    {
        if (FS_Info_A.attr[i] == 0)
        {
            continue;
        }
        if (Cal_Dis_Pt2Pt((float *)&FS_Info_A.obj[i].pt1,
                          (float *)&FS_Info_A.obj[i].pt2) < 0.3)
        {
            continue;
        }

        float obj[4];
        memcpy(obj, (float *)&FS_Info_A.obj[i], sizeof(obj));
        if (PK_PathPlan_PathVerify(path, 1, &obj, swell) !=
            PATHPLAN_SUCCESS) //  0912:add environmnet obs check
        {
            return (1 + i);
        }
    }

    return 0;
}

static float ClacDistToOtherSide(const Multi_Slot_Array_T &slots,
                                 const PlanDataCase planData[1])
{
    if (planData[0].slotshape < PK_SLOT_LEFT_VERT || planData[0].is_vision_slot == 0)
    {
        return 0.0f;
    }
    float rsB[3], rsE[3];
    for (int i = 0; i < slots.multiNum; i++)
    {
        if (slots.multiArray[i].is_vision_slot == 0)
        {
            continue;
        }

        if (Cal_Dis_Pt2Pt(planData[0].finpoint, (float *)&slots.multiArray[i].targpos) <
            2.0f * VEHICLE_WID)
        {
            continue;
        }

        RevConvert(planData[0].finpoint,
                   (float *)&slots.multiArray[i].avm_point.near_front, rsB);
        RevConvert(planData[0].finpoint,
                   (float *)&slots.multiArray[i].avm_point.near_rear, rsE);

        if (fabs(rsB[0]) < 1.5f * VEHICLE_LEN || fabs(rsE[0]) < 1.5f * VEHICLE_LEN)
        {
            continue;
        }

        if (fabs(rsB[1]) > 1.5f * VEHICLE_WID && fabs(rsE[1]) > 1.5f * VEHICLE_WID)
        {
            continue;
        }

        if (fabs(rsB[0] - rsE[0]) > 0.6f)
        {
            continue;
        }

        return (fabs(rsB[0]) + fabs(rsE[0])) * 0.5f - 0.8f;
    }
    return 0.0f;
}

int Add_Obj_Temp(const Multi_Slot_Array_T &slots, PlanDataCase planData[1])
{
    const float wide_max = 11.0f;
    const float wide_min = 9.6f;

    if (planData[0].is_vision_slot == 2)
    {
        return 0;
    }

    int obj_danger_dir = planData[0].left_fac > 0 ? 2 : 1;
    // 对向障碍物（延长--前方障碍物无法识别的预防措施）

    U_RadarType uradar;
    RTE_PD_Get_U_Radar(&uradar); // 超声波实时反馈数据

    float front_limit = 2550;
    if (rte_max(uradar.FCL, uradar.FCR) < 2000 && rte_min(uradar.FCL, uradar.FCR) > 0)
    {
        front_limit = rte_min(uradar.FCL, uradar.FCR);
    }

    if (rte_max(uradar.FCL, uradar.FOL) < 2000 && rte_min(uradar.FCL, uradar.FOL) > 0)
    {
        front_limit = rte_min(uradar.FCL, front_limit);
    }

    if (rte_max(uradar.FCR, uradar.FOR) < 2000 && rte_min(uradar.FCR, uradar.FOR) > 0)
    {
        front_limit = rte_min(uradar.FCR, front_limit);
    }

    front_limit  = front_limit / 1000.0;
    float rtheta = Project_PosTo1st_rtheta(planData[0].stpoint, planData[0].finpoint);
    // float ryv =  Project_PosTo1st_ry(planData[0].finpoint, planData[0].stpoint);
    // float ryb =  Project_PosTo1st_ry(planData[0].finpoint,
    // &planData[0].obj_slot[1][0]);

    // 判断是否是提前释放车位
    bool addFlag = true;
    if (planData[0].slotshape <= PK_SLOT_RIGHT_PARA)
    {
        addFlag = false;
    }
    else
    {
        for (int i = 0; i < planData[0].obj_danger_num; i++)
        {
            if (Cal_Dis_Pt2Pt(&planData[0].obj_danger[i][0],
                              &planData[0].obj_danger[i][2]) < 0.5f)
            {
                continue;
            }

            float rsObj[2];
            RevConvert(planData[0].stpoint, &planData[0].obj_danger[i][0], rsObj);
            if ((rsObj[0] > VEHICLE_LEN - REAR_SUSPENSION) &&
                ((obj_danger_dir == 2 && rsObj[1] < -0.01f) ||
                 (obj_danger_dir == 1 && rsObj[1] > 0.01f)))
            {
                addFlag = false;
                break;
            }

            RevConvert(planData[0].stpoint, &planData[0].obj_danger[i][2], rsObj);
            if ((rsObj[0] > VEHICLE_LEN - REAR_SUSPENSION) &&
                ((obj_danger_dir == 2 && rsObj[1] < -0.01f) ||
                 (obj_danger_dir == 1 && rsObj[1] > 0.01f)))
            {
                addFlag = false;
                break;
            }
        }
    }

    if (planData[0].obj_danger_num < SF_OBJ_NUM * 2 &&
        (fabs(rtheta) > PI * 0.35 && fabs(rtheta) < PI * 0.65) && addFlag)
    // &&(ryv * ryb > 0 && fabs(ryv) > VEHICLE_WID))//添加对向空间障碍物
    {
        float sidedist = 0.0f;
        if (planData[0].left_fac < 0 && uradar.FSL < 3500)
        {
            sidedist = uradar.FSL / 1000.0f;
        }
        if (planData[0].left_fac > 0 && uradar.FSR < 3500)
        {
            sidedist = uradar.FSR / 1000.0f;
        }

        float rsstart[3];
        CoordinadteTransfer(planData[0].finpoint, planData[0].stpoint, rsstart);
        float offset = rsstart[0];
        float rx1    = offset;

        if (sidedist > ZERO_FLOAT)
        {
            rx1 = offset + VEHICLE_WID * 0.5f + rte_min(sidedist, 3.5f);
            rx1 = rte_max(rx1, wide_min);
        }
        else
        {
            float frx = Project_PosTo1st_rx(planData[0].finpoint, planData[0].stpoint);
            float defaultDist = ClacDistToOtherSide(slots, planData);
            defaultDist       = rte_max(defaultDist, wide_min);
            defaultDist       = rte_min(defaultDist, wide_max);
            if (offset + VEHICLE_WID * 0.5f + 0.8f > defaultDist)
            {
                rx1 = offset + VEHICLE_WID * 0.5f + 0.6f;
                if (frx > VEHICLE_LEN - REAR_SUSPENSION + 0.5 * VEHICLE_WID + 2.5f)
                {
                    rx1 = offset + VEHICLE_WID * 0.5f + 0.4f;
                }
            }
            else
            {
                rx1 = defaultDist;
                if (frx > VEHICLE_LEN - REAR_SUSPENSION + 0.5 * VEHICLE_WID + 2.5f)
                {
                    rx1 = offset + VEHICLE_WID * 0.5f + 0.4f;
                }
            }
        }

        float dir        = (planData[0].left_fac > 0.001f) ? 1.0f : -1.0f;
        float rsultro[3] = {FCL_DELTAX + 0.1f, -dir * VEHICLE_WID / 2.0f, 0}; // 探头
        float fsultro[3];
        Convert(rsstart, rsultro, fsultro); // 探头坐标

        float obj[3] = {rx1, fsultro[1], 0};
        Convert(planData[0].finpoint, obj,
                &planData[0].obj_danger[planData[0].obj_danger_num][0]);

        obj[1] = fsultro[1] + 10.0 * sign(dir);
        Convert(planData[0].finpoint, obj,
                &planData[0].obj_danger[planData[0].obj_danger_num][2]);
        planData[0].obj_danger_num += 1;
    }

    if (front_limit < 2.0)
    {
        float front_dist = rte_max(front_limit - 0.30, 0.10);
        front_dist += (VEHICLE_LEN - REAR_SUSPENSION);

        float objAdd[4] = {front_dist, -2, front_dist, 2};

        if (planData[0].obj_danger_num < SF_OBJ_NUM * 2)
        {
            Convert(planData[0].stpoint, &objAdd[0],
                    &planData[0].obj_danger[planData[0].obj_danger_num][0]);
            Convert(planData[0].stpoint, &objAdd[2],
                    &planData[0].obj_danger[planData[0].obj_danger_num][2]);
            planData[0].obj_danger_dir[planData[0].obj_danger_num] = obj_danger_dir;
            planData[0].obj_danger_num += 1;
        }
        else
        {
            return -1;
        }

        if (front_limit > 0.80)
        {
            return 1; // 前方有障碍物物
        }
        else
        {
            return -1; // 不向前
        }
    }
    return 0;
}

static void UpdateLineEnd(const float startPos[2], float lineLen, float endPos[2])
{
    Vec2_T vec;
    vec.vx = endPos[0] - startPos[0];
    vec.vy = endPos[1] - startPos[1];
    Normalize_Vec2(&vec);
    endPos[0] = startPos[0] + lineLen * vec.vx;
    endPos[1] = startPos[1] + lineLen * vec.vy;
}

static void UpdateByRaw(const LineSeg_T slot[2], int slotindex, FusionObj_T &fusion,
                        int objDir[FS_OBJ_ARR_NUM], float obj[FS_OBJ_ARR_NUM][4])
{
    const float offset = 0.05f;
    for (int i = 0; i < FS_OBJ_ARR_NUM; i++)
    {
        if (objDir[i] == 0)
        {
            continue;
        }
        LineSeg_T lineObj;
        memcpy((float *)&lineObj, obj[i], sizeof(LineSeg_T));

        float distobj = Get_Segment_Len(lineObj);
        if (distobj > 2 * PathPlanConstCfg::valid_obj_len)
        {
            continue;
        }

        for (int j = 0; j < SF_OBJ_NUM; j++)
        {
            if (fusion.attr[j] == 0)
            {
                continue;
            }

            if (fusion.ang[j] + offset < distobj)
            {
                continue;
            }

            // 角度相差很小
            float dotvalue = (fusion.obj[j].pt1.x - fusion.obj[j].pt2.x) *
                                 (lineObj.pt1.x - lineObj.pt2.x) +
                             (fusion.obj[j].pt1.y - fusion.obj[j].pt2.y) *
                                 (lineObj.pt1.y - lineObj.pt2.y);
            dotvalue = dotvalue / distobj / fusion.ang[j];
            if (fabs(dotvalue) < 0.98f)
            {
                continue;
            }

            // 距离相差很小
            float linedist = Calc_Dist_Line2Line(
                (float *)&fusion.obj[j].pt1, (float *)&fusion.obj[j].pt2,
                (float *)&lineObj.pt1, (float *)&lineObj.pt2);
            if (linedist > offset)
            {
                continue;
            }

            float midx = (lineObj.pt1.x + lineObj.pt2.x) * 0.5f;
            if (midx > rte_max(fusion.obj[j].pt1.x, fusion.obj[j].pt2.x) ||
                midx < rte_min(fusion.obj[j].pt1.x, fusion.obj[j].pt2.x))
            {
                continue;
            }

            float midy = (lineObj.pt1.y + lineObj.pt2.y) * 0.5f;
            if (midy > rte_max(fusion.obj[j].pt1.y, fusion.obj[j].pt2.y) + offset ||
                midy < rte_min(fusion.obj[j].pt1.y, fusion.obj[j].pt2.y) - offset)
            {
                continue;
            }

            // 是同一个障碍物,但是长度一样,不需要更新
            if (fabs(distobj - fusion.ang[j]) < offset)
            {
                fusion.attr[j] = 0;
                continue;
            }

            // 和边缘不相交
            float bcdist = Calc_Dist_Line2Line(
                (float *)&fusion.obj[j].pt1, (float *)&fusion.obj[j].pt2,
                (float *)&slot[0].pt1, (float *)&slot[0].pt2);
            float dedist = Calc_Dist_Line2Line(
                (float *)&fusion.obj[j].pt1, (float *)&fusion.obj[j].pt2,
                (float *)&slot[1].pt1, (float *)&slot[1].pt2);
            if (bcdist > 0.01f && dedist > 0.01f)
            {
                // printf("slotindex %d: object expand old:%f %f %f %f new: %f %f %f
                // %f\n",
                //     slotindex, obj[i][0], obj[i][1], obj[i][2], obj[i][3],
                //     fusion.obj[j].pt1.x, fusion.obj[j].pt1.y, fusion.obj[j].pt2.x,
                //     fusion.obj[j].pt2.y);

                // 和自己不相交, 有可能被其他车位裁剪短了, 直接使用原来的值
                memcpy(obj[i], (float *)&fusion.obj[j], sizeof(LineSeg_T));
                fusion.attr[j] = 0;
                break;
            }
            else
            {
                // 和自己相交, 如果裁减太多, 会被忽略掉, 适当增加长度
                if (distobj < PathPlanConstCfg::valid_obj_len + offset &&
                    fusion.ang[j] > PathPlanConstCfg::valid_obj_len + offset)
                {
                    float line[4];
                    Relation_T dir[2];
                    Point_T farPoint;
                    if (bcdist <= 0.01f)
                    {
                        memcpy(line, (float *)&slot[0], sizeof(LineSeg_T));
                        float dist1 = PK_PointToLineDist(line, &obj[i][0]);
                        float dist2 = PK_PointToLineDist(line, &obj[i][2]);
                        if (dist1 < dist2)
                        {
                            // pt1 更近, 作为起点, pt2作为终点
                            memcpy((float *)&farPoint, &obj[i][2], sizeof(Point_T));
                            Get_Dist_Dir_Pt2PointLine(slot[0].pt1, slot[0].pt2,
                                                      slot[1].pt1, NULL, &dir[0]);
                            Get_Dist_Dir_Pt2PointLine(slot[0].pt1, slot[0].pt2, farPoint,
                                                      NULL, &dir[1]);
                            float extDir = (dir[0] == dir[1]) ? -1.0 : 1.0;

                            UpdateLineEnd(&obj[i][0],
                                          extDir *
                                              (PathPlanConstCfg::valid_obj_len + offset),
                                          &obj[i][2]);
                        }
                        else
                        {
                            memcpy((float *)&farPoint, &obj[i][0], sizeof(Point_T));
                            Get_Dist_Dir_Pt2PointLine(slot[0].pt1, slot[0].pt2,
                                                      slot[1].pt1, NULL, &dir[0]);
                            Get_Dist_Dir_Pt2PointLine(slot[0].pt1, slot[0].pt2, farPoint,
                                                      NULL, &dir[1]);
                            float extDir = (dir[0] == dir[1]) ? -1.0 : 1.0;

                            UpdateLineEnd(&obj[i][2],
                                          extDir *
                                              (PathPlanConstCfg::valid_obj_len + offset),
                                          &obj[i][0]);
                        }
                    }
                    else
                    {
                        memcpy(line, (float *)&slot[1], sizeof(LineSeg_T));
                        float dist1 = PK_PointToLineDist(line, &obj[i][0]);
                        float dist2 = PK_PointToLineDist(line, &obj[i][2]);
                        if (dist1 < dist2)
                        {
                            memcpy((float *)&farPoint, &obj[i][2], sizeof(Point_T));
                            Get_Dist_Dir_Pt2PointLine(slot[1].pt1, slot[1].pt2,
                                                      slot[0].pt1, NULL, &dir[0]);
                            Get_Dist_Dir_Pt2PointLine(slot[1].pt1, slot[1].pt2, farPoint,
                                                      NULL, &dir[1]);
                            float extDir = (dir[0] == dir[1]) ? -1.0 : 1.0;

                            UpdateLineEnd(&obj[i][0],
                                          extDir *
                                              (PathPlanConstCfg::valid_obj_len + offset),
                                          &obj[i][2]);
                        }
                        else
                        {
                            memcpy((float *)&farPoint, &obj[i][0], sizeof(Point_T));
                            Get_Dist_Dir_Pt2PointLine(slot[1].pt1, slot[1].pt2,
                                                      slot[0].pt1, NULL, &dir[0]);
                            Get_Dist_Dir_Pt2PointLine(slot[1].pt1, slot[1].pt2, farPoint,
                                                      NULL, &dir[1]);
                            float extDir = (dir[0] == dir[1]) ? -1.0 : 1.0;

                            UpdateLineEnd(&obj[i][2],
                                          extDir *
                                              (PathPlanConstCfg::valid_obj_len + offset),
                                          &obj[i][0]);
                        }
                    }
                }
            }
        }
    }
}

void UpdateObjLenByRaw(const SlotInfo_T &slot, int ObjDir[FS_OBJ_ARR_NUM],
                       float Obj[FS_OBJ_ARR_NUM][4], int ObjRearDir[FS_OBJ_ARR_NUM],
                       float ObjRear[FS_OBJ_ARR_NUM][4])
{
    LineSeg_T bcde[2];
    memcpy((float *)&bcde[0].pt1, (float *)&slot.avm_point.far_rear, sizeof(Point_T));
    memcpy((float *)&bcde[0].pt2, (float *)&slot.avm_point.near_rear, sizeof(Point_T));
    memcpy((float *)&bcde[1].pt1, (float *)&slot.avm_point.far_front, sizeof(Point_T));
    memcpy((float *)&bcde[1].pt2, (float *)&slot.avm_point.near_front, sizeof(Point_T));

    float distCB = Cal_Dis_Pt2Pt(bcde[0].pt1, bcde[0].pt2);
    distCB       = rte_max(distCB, 1.5 * VEHICLE_LEN);
    UpdateLineEnd((float *)&bcde[0].pt1, distCB, (float *)&bcde[0].pt2);

    float distDE = Cal_Dis_Pt2Pt(bcde[1].pt1, bcde[1].pt2);
    distDE       = rte_max(distDE, 1.5 * VEHICLE_LEN);
    UpdateLineEnd((float *)&bcde[1].pt1, distDE, (float *)&bcde[1].pt2);

    FusionObj_T fusion;
    for (int dir = 0; dir < 2; dir++)
    {
        if (dir == 0)
        {
            RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left_Raw(&fusion);
        }
        else
        {
            RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right_Raw(&fusion);
        }

        for (int i = 0; i < SF_OBJ_NUM; i++)
        {
            if (fusion.attr[i] == 0)
            {
                continue;
            }
            fusion.ang[i] = Get_Segment_Len(fusion.obj[i]);
            if (fusion.ang[i] < PathPlanConstCfg::valid_obj_len)
            {
                fusion.attr[i] = 0;
            }
        }

        UpdateByRaw(bcde, slot.slot_index, fusion, ObjDir, Obj);
        UpdateByRaw(bcde, slot.slot_index, fusion, ObjRearDir, ObjRear);
    }
}

static void FilterObjByLen1(int objDir[SF_OBJ_NUM], float obj[SF_OBJ_NUM][4])
{
    auto ClearObj = [](float obj[4]) {
        obj[0] = 0.0f;
        obj[1] = 0.0f;
        obj[2] = 0.0f;
        obj[3] = 0.0f;
    };

    int count = 0;
    for (int i = 0; i < SF_OBJ_NUM && count < SF_OBJ_NUM; i++)
    {
        if (objDir[i] == 0)
        {
            ClearObj(obj[i]);
            continue;
        }

        // 小于最小长度，舍去
        if (Cal_Dis_Pt2Pt(&obj[i][0], &obj[i][2]) < PathPlanConstCfg::valid_obj_len)
        {
            objDir[i] = 0;
            ClearObj(obj[i]);
            continue;
        }

        if (count != i)
        {
            objDir[count] = objDir[i];
            obj[count][0] = obj[i][0];
            obj[count][1] = obj[i][1];
            obj[count][2] = obj[i][2];
            obj[count][3] = obj[i][3];

            objDir[i] = 0;
            ClearObj(obj[i]);
        }
        count++;
    }
}

void FilterObjByLen(int objDir[FS_OBJ_ARR_NUM], float obj[FS_OBJ_ARR_NUM][4],
                    int objRearDir[FS_OBJ_ARR_NUM], float objRear[FS_OBJ_ARR_NUM][4])
{
    for (int j = 0; j < 2; j++)
    {
        int indexOffset = j * SF_OBJ_NUM;
        FilterObjByLen1(&objDir[indexOffset], &obj[indexOffset]);
        FilterObjByLen1(&objRearDir[indexOffset], &objRear[indexOffset]);
    }
}