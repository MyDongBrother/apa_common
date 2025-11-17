#include "PK_Calibration.h"
#include "PK_PathExecute.h"
#include "Uradar_ObjAvoid.h"
#include "MathFunc.h"
#include "PK_Utility.h"
#include "SystemPara.h"

const float angle15_cosvalue = 0.2508;
const float angle15_offset   = PI / 12;
const float angle20_cosvalue = 0.3402;
void PK_GetObsLocation(float radar[4], VehPos_T &local, VehPos_T &point)
{
    float aerfa      = radar[3];
    float sintemp1   = sinf(aerfa);
    float costemp1   = cosf(aerfa);
    float sintemp2   = sinf(local.theta);
    float costemp2   = cosf(local.theta);
    float x_detected = radar[0] + radar[3] * costemp1;
    float y_detected = radar[1] + radar[3] * sintemp1;
    point.x          = x_detected * costemp2 - y_detected * sintemp2 + local.x;
    point.y          = x_detected * sintemp2 + y_detected * costemp2 + local.y;
    point.theta      = Round_PI(local.theta + radar[2]);
}

static int CalcRadarPos(int radarIdx, float radarPos[4])
{
    if (radarIdx == Radar_ID_RSL)
    {
        radarPos[0] = RSL_DELTAX;
        radarPos[1] = RSL_DELTAY;
        radarPos[2] = RSL_AERFA;
    }
    else if (radarIdx == Radar_ID_RSR)
    {
        radarPos[0] = RSR_DELTAX;
        radarPos[1] = RSR_DELTAY;
        radarPos[2] = RSR_AERFA;
    }
    else if (radarIdx == Radar_ID_RCR)
    {
        radarPos[0] = RCR_DELTAX;
        radarPos[1] = RCR_DELTAY;
        radarPos[2] = RCR_AERFA;
    }
    else if (radarIdx == Radar_ID_RCL)
    {
        radarPos[0] = RCL_DELTAX;
        radarPos[1] = RCL_DELTAY;
        radarPos[2] = RCL_AERFA;
    }
    else if (radarIdx == Radar_ID_ROL)
    {
        radarPos[0] = ROL_DELTAX;
        radarPos[1] = ROL_DELTAY;
        radarPos[2] = ROL_AERFA;
    }
    else if (radarIdx == Radar_ID_ROR)
    {
        radarPos[0] = ROR_DELTAX;
        radarPos[1] = ROR_DELTAY;
        radarPos[2] = ROR_AERFA;
    }
    else if (radarIdx == Radar_ID_FSL)
    {
        radarPos[0] = FSL_DELTAX;
        radarPos[1] = FSL_DELTAY;
        radarPos[2] = FSL_AERFA;
    }
    else if (radarIdx == Radar_ID_FSR)
    {
        radarPos[0] = FSR_DELTAX;
        radarPos[1] = FSR_DELTAY;
        radarPos[2] = FSR_AERFA;
    }
    else if (radarIdx == Radar_ID_FCR)
    {
        radarPos[0] = FCR_DELTAX;
        radarPos[1] = FCR_DELTAY;
        radarPos[2] = FCR_AERFA;
    }
    else if (radarIdx == Radar_ID_FCL)
    {
        radarPos[0] = FCL_DELTAX;
        radarPos[1] = FCL_DELTAY;
        radarPos[2] = FCL_AERFA;
    }
    else if (radarIdx == Radar_ID_FOL)
    {
        radarPos[0] = FOL_DELTAX;
        radarPos[1] = FOL_DELTAY;
        radarPos[2] = FOL_AERFA;
    }
    else if (radarIdx == Radar_ID_FOR)
    {
        radarPos[0] = FOR_DELTAX;
        radarPos[1] = FOR_DELTAY;
        radarPos[2] = FOR_AERFA;
    }
    else
    {
        return 1;
    }

    return 0;
}

int CheckNearExistObj(int radarIdx, const float curPos[3])
{
#define near_dist 0.75
    float radarPos[4];
    if (CalcRadarPos(radarIdx, radarPos) != 0)
    {
        return 0;
    }

    radarPos[3] = 0.0;
    VehPos_T vehPos, resRadarPos;
    PK_CopyPos((float *)&vehPos, curPos);
    PK_GetObsLocation(radarPos, vehPos, resRadarPos);

    Point_T endPoint, startPoint;
    startPoint.x = resRadarPos.x;
    startPoint.y = resRadarPos.y;
    endPoint.x   = resRadarPos.x + near_dist * cosf(resRadarPos.theta);
    endPoint.y   = resRadarPos.y + near_dist * sinf(resRadarPos.theta);

    FusionObj_T fusionSlot; //  for replan
    auto CheckNearObj = [](const Point_T &startP, const Point_T &endP,
                           const FusionObj_T &nearSlot) {
        for (int index = 0; index < SF_OBJ_NUM; index++)
        {
            if (Get_Segment_Len(nearSlot.obj[index]) < 0.20)
            {
                continue;
            }
            if (Is_TwoSegment_Cross(startP, endP, nearSlot.obj[index].pt1,
                                    nearSlot.obj[index].pt2))
            {
                return 1;
            }
            else
            {
                float dist1 = Cal_Dis_Pt2Pt(startP, nearSlot.obj[index].pt1);
                float dist2 = Cal_Dis_Pt2Pt(startP, nearSlot.obj[index].pt2);
                if (dist1 < near_dist || dist2 < near_dist)
                {
                    return 1;
                }
                return 0;
            }
        }
        return 0;
    };

    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right(&fusionSlot); // get the opposite obs
    if (CheckNearObj(startPoint, endPoint, fusionSlot) == 1)
    {
        return 1;
    }

    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Right(
        &fusionSlot); // get the opposite obs
    if (CheckNearObj(startPoint, endPoint, fusionSlot) == 1)
    {
        return 1;
    }

    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Right(&fusionSlot);
    if (CheckNearObj(startPoint, endPoint, fusionSlot) == 1)
    {
        return 1;
    }

    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Left(&fusionSlot);
    if (CheckNearObj(startPoint, endPoint, fusionSlot) == 1)
    {
        return 1;
    }
    return 0;
}

float CalcRadarSlotsDist(int radarIdx, const float curPos[3])
{
    SlotObj_T slotObj;
    float slotArray[5][4];
    RTE_PK_SlotDetect_Get_SlotObj(&slotObj);
    SlotObj_Convert_float(slotObj, slotArray);
    float result = 2550.0f;

    float radarPos[4];
    if (CalcRadarPos(radarIdx, radarPos) != 0)
    {
        return result;
    }

    radarPos[3] = 0.0;
    VehPos_T vehPos, resRadarPos;
    PK_CopyPos((float *)&vehPos, curPos);
    PK_GetObsLocation(radarPos, vehPos, resRadarPos);

    float target[3];
    RTE_PK_SlotDetect_Get_TargPos(target);

    float crossAngle = Round_PI(target[2] - resRadarPos.theta);

    // 大角度直接计算点的距离
    if (fabs(cosf(crossAngle)) > angle20_cosvalue)
    {
        float dist = Cal_Dis_Pt2Pt(slotArray[1], (float *)&resRadarPos);
        result     = rte_min(result, dist * 1000.0f);
        dist       = Cal_Dis_Pt2Pt(slotArray[4], (float *)&resRadarPos);
        result     = rte_min(result, dist * 1000.0f);
        return result;
    }

    float dist = PK_PointToLineDist(slotArray[1], (float *)&resRadarPos);
    result     = rte_min(result, dist * 1000.0f);
    dist       = PK_PointToLineDist(slotArray[3], (float *)&resRadarPos);
    result     = rte_min(result, dist * 1000.0f);

    return result;
}

float CalcSideNextRadarSlotsDist(int radarIdx, Veh_AvoidState &curSt)
{
    SlotObj_T slotObj;
    float slotArray[5][4];
    RTE_PK_SlotDetect_Get_SlotObj(&slotObj);
    SlotObj_Convert_float(slotObj, slotArray);

    float target[3];
    RTE_PK_SlotDetect_Get_TargPos(target);
    float distMin = 1000.0f;
    float step    = 0.2;
    int count     = 10;
    if (fabs(curSt.Eps) < 200.0)
    {
        count = 1;
        step  = 1;
    }
    else if (fabs(curSt.Eps) < 300.0)
    {
        count = 5;
        step  = 0.4;
    }

    for (int i = 1; i < count; i++)
    {
        float finpoint[4];
        PK_Pre_locat(curSt.curPos, finpoint, curSt.Eps, curSt.Speed, step * i);
        float dist = CalcRadarSlotsDist(radarIdx, finpoint);
        distMin    = rte_min(dist, distMin);
    }

    return distMin;
}
