#include "PK_Calibration.h"
#include "PK_PathPlan.h"
#include "PK_PathPlanP.h"
#include "PK_Utility.h"
#include "Module_API.h"
#include "Rte_BSW.h"
#include "Rte_ComIF.h"

enum
{
    NO_OBJ,
    REMOTE_OBJ,
    NEAR_OBJ
};
enum
{
    LEFT_SIDE,
    RIGHT_SIDE,
    FRONT_CENTER,
    REAR_CENTER,
    FRONT_LEFT,
    FRONT_RIGHT,
    REAR_LEFT,
    REAR_RIGHT
};
void PK_BuildAvmSlotInfo(const Point_T avmPoints[4], const PK_SlotShapeType slotShap,
                         SlotInfo_T &manuSlot)
{
    const float ToCurbSafeDist = 0.5f;
    manuSlot.has_stopper       = 0;
    manuSlot.is_vision_slot    = 2;
    manuSlot.slot_status       = YELLOW_SLOT;
    manuSlot.slotshap          = slotShap;
    memcpy(&manuSlot.avm_point.near_front, &avmPoints[3], sizeof(Point_T));
    memcpy(&manuSlot.avm_point.near_rear, &avmPoints[0], sizeof(Point_T));
    memcpy(&manuSlot.avm_point.far_front, &avmPoints[2], sizeof(Point_T));
    memcpy(&manuSlot.avm_point.far_rear, &avmPoints[1], sizeof(Point_T));

    Vec2_T slotVec, vertSlotVec;
    float obsVehLen = 0, obsVehDepth = 0;
    LineSeg_T NearLineSeg;
    VehPos_T NearVec;
    NearLineSeg.pt1 = avmPoints[0];
    NearLineSeg.pt2 = avmPoints[3];
    Convert_Line_to_Pos(NearLineSeg, &NearVec);
    float dyfr = fabsf(Project_PointTo1stPos_ry(NearVec, avmPoints[1]));
    float dyff = fabsf(Project_PointTo1stPos_ry(NearVec, avmPoints[2]));

    float averageLength = (dyfr + dyff) / 2.0f;
    float targTheta     = NearVec.theta; // slot orientation

    slotVec.vx = cos(NearVec.theta);
    slotVec.vy = sin(NearVec.theta);

    NearLineSeg.pt1 = avmPoints[0];
    NearLineSeg.pt2 = avmPoints[1];
    Convert_Line_to_Pos(NearLineSeg, &NearVec);
    vertSlotVec.vx = sign(cos(NearVec.theta)) * fabs(slotVec.vy);
    vertSlotVec.vy = sign(sin(NearVec.theta)) * fabs(slotVec.vx);

    float divx, divy;
    if (slotShap == PK_SLOT_LEFT_PARA || slotShap == PK_SLOT_RIGHT_PARA)
    {
        divx = (Cal_Dis_Pt2Pt(avmPoints[3], avmPoints[0]) - VEHICLE_LEN) * 0.5f +
               REAR_SUSPENSION;
        divy = (rte_max(averageLength - VEHICLE_WID, 0) + VEHICLE_WID) / 2;

        manuSlot.targpos.x = avmPoints[0].x + slotVec.vx * divx + vertSlotVec.vx * divy;
        manuSlot.targpos.y = avmPoints[0].y + slotVec.vy * divx + vertSlotVec.vy * divy;
        manuSlot.targpos.theta = Round_PI(targTheta);
    }
    else if (slotShap == PK_SLOT_LEFT_VERT || slotShap == PK_SLOT_RIGHT_VERT)
    {
        divx = Cal_Dis_Pt2Pt(avmPoints[3], avmPoints[0]) * 0.5f;
        divy = (VEHICLE_LEN - REAR_SUSPENSION); // AVM_Vert_TargPos_off_dy = 0.05

        manuSlot.targpos.x = avmPoints[0].x + slotVec.vx * divx + vertSlotVec.vx * divy;
        manuSlot.targpos.y = avmPoints[0].y + slotVec.vy * divx + vertSlotVec.vy * divy;
        manuSlot.targpos.theta = Round_PI(NearVec.theta + PI);
    }

    if (slotShap == PK_SLOT_LEFT_PARA || slotShap == PK_SLOT_RIGHT_PARA)
    {
        obsVehLen   = VEHICLE_LEN * 0.5f;
        obsVehDepth = VEHICLE_WID + ToCurbSafeDist;
    }
    else if (slotShap == PK_SLOT_LEFT_VERT || slotShap == PK_SLOT_RIGHT_VERT)
    {
        obsVehLen   = VEHICLE_WID;
        obsVehDepth = VEHICLE_LEN + ToCurbSafeDist;
    }

    manuSlot.slotobj.ptB.x = avmPoints[0].x;
    manuSlot.slotobj.ptB.y = avmPoints[0].y;
    manuSlot.slotobj.ptA.x = manuSlot.slotobj.ptB.x - obsVehLen * slotVec.vx;
    manuSlot.slotobj.ptA.y = manuSlot.slotobj.ptB.y - obsVehLen * slotVec.vy;
    manuSlot.slotobj.ptC.x = manuSlot.slotobj.ptB.x + obsVehDepth * vertSlotVec.vx;
    manuSlot.slotobj.ptC.y = manuSlot.slotobj.ptB.y + obsVehDepth * vertSlotVec.vy;
    manuSlot.slotobj.ptE.x = avmPoints[3].x;
    manuSlot.slotobj.ptE.y = avmPoints[3].y;
    manuSlot.slotobj.ptF.x = manuSlot.slotobj.ptE.x + obsVehLen * slotVec.vx;
    manuSlot.slotobj.ptF.y = manuSlot.slotobj.ptE.y + obsVehLen * slotVec.vy;
    manuSlot.slotobj.ptD.x = manuSlot.slotobj.ptE.x + obsVehDepth * vertSlotVec.vx;
    manuSlot.slotobj.ptD.y = manuSlot.slotobj.ptE.y + obsVehDepth * vertSlotVec.vy;

    if (PathPlan_SlotDir(slotShap) == 1)
    {
        manuSlot.slot_index =
            ((int)sqrt(pow2(manuSlot.targpos.x) + pow2(manuSlot.targpos.y)) % 100) + 100;
    }
    else
    {
        manuSlot.slot_index =
            ((int)sqrt(pow2(manuSlot.targpos.x) + pow2(manuSlot.targpos.y)) % 100) + 300;
    }
}

static void PathPlan_BuildUradarRoundObj(LineSeg_T objs[8], int result[8])
{
#define long_radar_dist  MAX_RANGE_ULTRASONIC
#define short_radar_dist 2550
#define min_radar_dist   250

    U_RadarType uradar;
    RTE_PD_Get_U_Radar(&uradar);
    auto SFunc = [](const float fvalue, const float rvalue, const float dir,
                    LineSeg_T &obj) {
        obj.pt1.y = (FSL_DELTAY + fvalue * 0.001) * dir;
        if (fvalue - ZERO_FLOAT < long_radar_dist)
        {
            obj.pt1.x = VEHICLE_LEN - REAR_SUSPENSION;
        }
        else
        {
            obj.pt1.x = FSL_DELTAX;
        }

        obj.pt2.y = (RSL_DELTAY + rvalue * 0.001) * dir;
        if (rvalue - ZERO_FLOAT < long_radar_dist)
        {
            obj.pt2.x = -REAR_SUSPENSION;
        }
        else
        {
            obj.pt2.x = RSL_DELTAX;
        }

        return NEAR_OBJ;
    };

    result[LEFT_SIDE]  = SFunc(uradar.FSL, uradar.RSL, 1.0, objs[0]);
    result[RIGHT_SIDE] = SFunc(uradar.FSR, uradar.RSR, -1.0, objs[1]);

    auto FRFunc = [](const float lvalue, const float rvalue, const float baseoffset,
                     LineSeg_T &obj) {
        int objtype = NEAR_OBJ;
        if (lvalue + ZERO_FLOAT > short_radar_dist &&
            rvalue + ZERO_FLOAT > short_radar_dist)
        {
            objtype = NO_OBJ;
        }
        else if (lvalue - ZERO_FLOAT < short_radar_dist &&
                 rvalue - ZERO_FLOAT < short_radar_dist)
        {
            obj.pt1.x = baseoffset + lvalue * sign(baseoffset) * 0.001;
            obj.pt1.y = 0.4;
            obj.pt2.x = baseoffset + rvalue * sign(baseoffset) * 0.001;
            obj.pt2.y = -0.4;
            objtype = (rte_min(lvalue, rvalue) > min_radar_dist) ? REMOTE_OBJ : NEAR_OBJ;
        }
        else if (lvalue - ZERO_FLOAT < short_radar_dist &&
                 rvalue + ZERO_FLOAT > short_radar_dist)
        {
            obj.pt1.x = baseoffset + lvalue * sign(baseoffset) * 0.001;
            obj.pt1.y = 0.6;
            obj.pt2.x = baseoffset + rvalue * sign(baseoffset) * 0.001;
            obj.pt2.y = 0.2;
            objtype   = (lvalue > min_radar_dist) ? REMOTE_OBJ : NEAR_OBJ;
            ;
        }
        else if (rvalue - ZERO_FLOAT < short_radar_dist &&
                 lvalue + ZERO_FLOAT > short_radar_dist)
        {
            obj.pt1.x = baseoffset + lvalue * sign(baseoffset) * 0.001;
            obj.pt1.y = -0.2;
            obj.pt2.x = baseoffset + rvalue * sign(baseoffset) * 0.001;
            obj.pt2.y = -0.6;
            objtype   = (rvalue > min_radar_dist) ? REMOTE_OBJ : NEAR_OBJ;
        }
        else
        {
            objtype = NO_OBJ;
        }

        return objtype;
    };

    result[FRONT_CENTER] = FRFunc(uradar.FCL, uradar.FCR, FCL_DELTAX, objs[2]);
    result[REAR_CENTER]  = FRFunc(uradar.RCL, uradar.RCR, -RCL_DELTAX, objs[3]);

    auto FOFunc = [](const float fvalue, const float ovalue, const float dir,
                     LineSeg_T &obj) {
        int objtype = NEAR_OBJ;
        if (fvalue + ZERO_FLOAT > short_radar_dist &&
            ovalue + ZERO_FLOAT > short_radar_dist)
        {
            objtype = NO_OBJ;
        }
        else if (fvalue - ZERO_FLOAT < short_radar_dist &&
                 ovalue - ZERO_FLOAT < short_radar_dist)
        {
            obj.pt1.x = FOL_DELTAX + ovalue * cos(FOL_AERFA) * 0.001;
            obj.pt1.y = FOL_DELTAY + ovalue * sin(FOL_AERFA) * 0.001;
            obj.pt2.x = FCL_DELTAX + fvalue * cos(FCL_AERFA) * 0.001;
            obj.pt2.y = FCL_DELTAY + fvalue * sin(FCL_AERFA) * 0.001;
            objtype = (rte_min(fvalue, ovalue) > min_radar_dist) ? REMOTE_OBJ : NEAR_OBJ;
        }
        else if (fvalue - ZERO_FLOAT < short_radar_dist &&
                 ovalue + ZERO_FLOAT > short_radar_dist)
        {
            obj.pt1.x = FCL_DELTAX + fvalue * 0.001;
            obj.pt1.y = 0.6;
            obj.pt2.x = FCL_DELTAX + fvalue * 0.001;
            obj.pt2.y = 0.2;
            objtype   = (fvalue > min_radar_dist) ? REMOTE_OBJ : NEAR_OBJ;
        }
        else if (ovalue - ZERO_FLOAT < short_radar_dist &&
                 fvalue + ZERO_FLOAT > short_radar_dist)
        {
            float x = FOL_DELTAX + ovalue * cos(FOL_AERFA) * 0.001;
            float y = FOL_DELTAY + ovalue * sin(FOL_AERFA) * 0.001;

            obj.pt1.x = x - 0.2;
            obj.pt1.y = y + 0.1;
            obj.pt2.x = x + 0.2;
            obj.pt2.y = y - 0.1;
            objtype   = (ovalue > min_radar_dist) ? REMOTE_OBJ : NEAR_OBJ;
        }
        else
        {
            objtype = NEAR_OBJ;
        }

        obj.pt1.y = obj.pt1.y * dir;
        obj.pt2.y = obj.pt2.y * dir;

        return objtype;
    };

    result[FRONT_LEFT]  = FOFunc(uradar.FCL, uradar.FOL, 1.0, objs[4]);
    result[FRONT_RIGHT] = FOFunc(uradar.FCR, uradar.FOR, -1.0, objs[5]);

    auto ROFunc = [](const float rvalue, const float ovalue, const float dir,
                     LineSeg_T &obj) {
        int objtype = NEAR_OBJ;
        if (rvalue + ZERO_FLOAT > short_radar_dist &&
            ovalue + ZERO_FLOAT > short_radar_dist)
        {
            objtype = NO_OBJ;
        }
        else if (rvalue - ZERO_FLOAT < short_radar_dist &&
                 ovalue - ZERO_FLOAT < short_radar_dist)
        {
            obj.pt1.x = ROL_DELTAX + ovalue * cos(ROL_AERFA) * 0.001;
            obj.pt1.y = ROL_DELTAY + ovalue * sin(ROL_AERFA) * 0.001;
            obj.pt2.x = RCL_DELTAX + rvalue * cos(RCL_AERFA) * 0.001;
            obj.pt2.y = RCL_DELTAY + rvalue * sin(RCL_AERFA) * 0.001;
            objtype = (rte_min(rvalue, ovalue) > min_radar_dist) ? REMOTE_OBJ : NEAR_OBJ;
        }
        else if (rvalue - ZERO_FLOAT < short_radar_dist &&
                 ovalue + ZERO_FLOAT > short_radar_dist)
        {
            obj.pt1.x = RCL_DELTAX + rvalue * 0.001;
            obj.pt1.y = 0.6;
            obj.pt2.x = RCL_DELTAX + rvalue * 0.001;
            obj.pt2.y = 0.2;
            objtype   = (rvalue > min_radar_dist) ? REMOTE_OBJ : NEAR_OBJ;
        }
        else if (ovalue - ZERO_FLOAT < short_radar_dist * 0.7 &&
                 rvalue + ZERO_FLOAT > short_radar_dist)
        {
            float x = ROL_DELTAX + ovalue * cos(ROL_AERFA) * 0.001;
            float y = ROL_DELTAY + ovalue * sin(ROL_AERFA) * 0.001;

            obj.pt1.x = x - 0.2;
            obj.pt1.y = y + 0.1;
            obj.pt2.x = x + 0.2;
            obj.pt2.y = y - 0.1;
            objtype   = (ovalue > min_radar_dist) ? REMOTE_OBJ : NEAR_OBJ;
        }
        else
        {
            objtype = NEAR_OBJ;
        }

        obj.pt1.y = obj.pt1.y * dir;
        obj.pt2.y = obj.pt2.y * dir;
        return objtype;
    };

    result[REAR_LEFT]  = ROFunc(uradar.RCL, uradar.ROL, 1.0, objs[6]);
    result[REAR_RIGHT] = ROFunc(uradar.RCR, uradar.ROR, -1.0, objs[7]);
}

static void PathPlan_SetAvmObjs(const float curPos[4], const PK_SlotShapeType slotShape,
                                const LineSeg_T objs[8], const int result[8],
                                FusionObj_T &fusion)
{
    fusion.num = fusion.num_accu = 0;
    auto AddObj = [&curPos](const LineSeg_T &line, const int dir, FusionObj_T &fobj) {
        int count = fobj.num;
        Convert(curPos, (float *)&line.pt1, (float *)&fobj.obj[count].pt1);
        Convert(curPos, (float *)&line.pt2, (float *)&fobj.obj[count].pt2);
        fobj.attr[count] = dir * 10000 + count;
        fobj.num         = count + 1;
    };

    if (slotShape == PK_SLOT_LEFT_PARA)
    {
        if (result[RIGHT_SIDE] != NO_OBJ)
        {
            AddObj(objs[RIGHT_SIDE], 2, fusion);
        }
        if (result[FRONT_CENTER] != NO_OBJ)
        {
            AddObj(objs[FRONT_CENTER], 1, fusion);
        }
        if (result[REAR_CENTER] != NO_OBJ)
        {
            AddObj(objs[REAR_CENTER], 1, fusion);
        }
        if (result[FRONT_RIGHT] != NO_OBJ)
        {
            AddObj(objs[FRONT_RIGHT], 1, fusion);
        }
        if (result[REAR_RIGHT] != NO_OBJ)
        {
            AddObj(objs[REAR_RIGHT], 1, fusion);
        }
    }
    else if (slotShape == PK_SLOT_RIGHT_PARA)
    {
        if (result[LEFT_SIDE] != NO_OBJ)
        {
            AddObj(objs[LEFT_SIDE], 1, fusion);
        }
        if (result[FRONT_CENTER] != NO_OBJ)
        {
            AddObj(objs[FRONT_CENTER], 2, fusion);
        }
        if (result[REAR_CENTER] != NO_OBJ)
        {
            AddObj(objs[REAR_CENTER], 2, fusion);
        }
        if (result[FRONT_LEFT] != NO_OBJ)
        {
            AddObj(objs[FRONT_LEFT], 2, fusion);
        }
        if (result[REAR_LEFT] != NO_OBJ)
        {
            AddObj(objs[REAR_LEFT], 2, fusion);
        }
    }
    else
    {
        if (result[RIGHT_SIDE] != NO_OBJ)
        {
            AddObj(objs[RIGHT_SIDE], 1, fusion);
        }
        if (result[LEFT_SIDE] != NO_OBJ)
        {
            AddObj(objs[LEFT_SIDE], 1, fusion);
        }
    }
}

int PathPlan_InverseSlot(const PK_SlotShapeType slotShape, const float stpoint[4],
                         SlotInfo_T &manuSlot)
{
    LineSeg_T objs[8];
    int result[8];
    PathPlan_BuildUradarRoundObj(objs, result);

    float wide, rear, front, bcde[4][2];
    PK_SlotShapeType invSlotShape = slotShape;
    if (slotShape == PK_SLOT_LEFT_VERT && result[FRONT_CENTER] <= REMOTE_OBJ)
    {
        wide       = VEHICLE_WID / 2 + 0.3;
        rear       = REAR_SUSPENSION;
        front      = VEHICLE_LEN - REAR_SUSPENSION;
        bcde[0][0] = front, bcde[0][1] = wide;
        bcde[1][0] = -rear, bcde[1][1] = wide;
        bcde[2][0] = -rear, bcde[2][1] = -wide;
        bcde[3][0] = front, bcde[3][1] = -wide;

        invSlotShape = PK_SLOT_RIGHT_VERT;
    }
    else if (slotShape == PK_SLOT_RIGHT_VERT && result[FRONT_CENTER] <= REMOTE_OBJ)
    {
        wide       = VEHICLE_WID / 2 + 0.3;
        rear       = REAR_SUSPENSION;
        front      = VEHICLE_LEN - REAR_SUSPENSION;
        bcde[0][0] = front, bcde[0][1] = -wide;
        bcde[1][0] = -rear, bcde[1][1] = -wide;
        bcde[2][0] = -rear, bcde[2][1] = wide;
        bcde[3][0] = front, bcde[3][1] = wide;

        invSlotShape = PK_SLOT_LEFT_VERT;
    }
    else if (slotShape == PK_SLOT_LEFT_PARA && result[FRONT_CENTER] <= REMOTE_OBJ)
    {
        wide       = VEHICLE_WID / 2 + 0.3;
        rear       = REAR_SUSPENSION;
        front      = VEHICLE_LEN - REAR_SUSPENSION;
        bcde[0][0] = -rear, bcde[0][1] = wide;
        bcde[1][0] = -rear, bcde[1][1] = -wide;
        bcde[2][0] = front, bcde[2][1] = -wide;
        bcde[3][0] = front, bcde[3][1] = wide;
        invSlotShape = PK_SLOT_RIGHT_PARA;
    }
    else if (slotShape == PK_SLOT_RIGHT_PARA && result[FRONT_CENTER] <= REMOTE_OBJ)
    {
        wide       = VEHICLE_WID / 2 + 0.3;
        rear       = REAR_SUSPENSION;
        front      = VEHICLE_LEN - REAR_SUSPENSION;
        bcde[0][0] = -rear, bcde[0][1] = -wide;
        bcde[1][0] = -rear, bcde[1][1] = wide;
        bcde[2][0] = front, bcde[2][1] = wide;
        bcde[3][0] = front, bcde[3][1] = -wide;
        invSlotShape = PK_SLOT_LEFT_PARA;
    }
    else
    {
        return 1;
    }

    float rspoint[2];
    Point_T avmPoints[4];
    Convert(stpoint, bcde[0], rspoint);
    memcpy(&avmPoints[0], rspoint, sizeof(rspoint));
    Convert(stpoint, bcde[1], rspoint);
    memcpy(&avmPoints[1], rspoint, sizeof(rspoint));
    Convert(stpoint, bcde[2], rspoint);
    memcpy(&avmPoints[2], rspoint, sizeof(rspoint));
    Convert(stpoint, bcde[3], rspoint);
    memcpy(&avmPoints[3], rspoint, sizeof(rspoint));

    auto slotIndex = manuSlot.slot_index;
    PK_BuildAvmSlotInfo(avmPoints, invSlotShape, manuSlot);
    manuSlot.slot_index = slotIndex;
    return PATHPLAN_SUCCESS;
}

int PathPlan_SetParkingSlot(SlotInfo_T &manuSlot)
{
    PlanInfoType paths;
    RTE_PK_PathExecute_Get_CurTraj(&paths);

    if (paths.IsDirConect == PARK_NOUPDATE)
    {
        return 1;
    }

    manuSlot.slot_index = paths.Slot_index;
    manuSlot.slotshap   = (PK_SlotShapeType)paths.slotshape;
    memcpy(&manuSlot.slotobj, &paths.slotObjs, sizeof(SlotObj_T));
    memcpy(&manuSlot.avm_point, &paths.avm_point, sizeof(Avm_Pot_T));

    RTE_PK_SlotDetect_Get_TargPos((float *)&manuSlot.targpos);
    if (RTE_PK_DataConvt_Get_IsAVM_Slot() == 1 || RTE_PK_DataConvt_Get_IsAVM_Slot() == 3)
    {
        manuSlot.is_vision_slot = 1;
    }

    if (manuSlot.slotshap == PK_SLOT_LEFT_VERT || manuSlot.slotshap == PK_SLOT_RIGHT_VERT)
    {
        float rspoint[2];
        RevConvert((float *)&manuSlot.targpos, (float *)&manuSlot.slotobj.ptA, rspoint);
        rspoint[0] = rte_min(fabs(rspoint[0]), 2.0) * sign(rspoint[0]);
        rspoint[1] = rte_min(fabs(rspoint[1]) + 0.5, 10.0) * sign(rspoint[1]);
        Convert((float *)&manuSlot.targpos, rspoint, (float *)&manuSlot.slotobj.ptA);

        RevConvert((float *)&manuSlot.targpos, (float *)&manuSlot.slotobj.ptB, rspoint);
        rspoint[0] = rte_min(fabs(rspoint[0]), 2.0) * sign(rspoint[0]);
        rspoint[1] = rte_min(fabs(rspoint[1]) + 0.5, 3.0) * sign(rspoint[1]);
        Convert((float *)&manuSlot.targpos, rspoint, (float *)&manuSlot.slotobj.ptB);

        RevConvert((float *)&manuSlot.targpos, (float *)&manuSlot.slotobj.ptC, rspoint);
        rspoint[1] = rte_min(fabs(rspoint[1]) + 0.5, 3.0) * sign(rspoint[1]);
        Convert((float *)&manuSlot.targpos, rspoint, (float *)&manuSlot.slotobj.ptC);

        RevConvert((float *)&manuSlot.targpos, (float *)&manuSlot.slotobj.ptD, rspoint);
        rspoint[1] = rte_min(fabs(rspoint[1]) + 0.5, 3.0) * sign(rspoint[1]);
        Convert((float *)&manuSlot.targpos, rspoint, (float *)&manuSlot.slotobj.ptD);

        RevConvert((float *)&manuSlot.targpos, (float *)&manuSlot.slotobj.ptE, rspoint);
        rspoint[0] = rte_min(fabs(rspoint[0]), 2.0) * sign(rspoint[0]);
        rspoint[1] = rte_min(fabs(rspoint[1]) + 0.5, 3.0) * sign(rspoint[1]);
        Convert((float *)&manuSlot.targpos, rspoint, (float *)&manuSlot.slotobj.ptE);

        RevConvert((float *)&manuSlot.targpos, (float *)&manuSlot.slotobj.ptF, rspoint);
        rspoint[0] = rte_min(fabs(rspoint[0]), 2.0) * sign(rspoint[0]);
        rspoint[1] = rte_min(fabs(rspoint[1]) + 0.5, 10.0) * sign(rspoint[1]);
        Convert((float *)&manuSlot.targpos, rspoint, (float *)&manuSlot.slotobj.ptF);
    }
    else if (manuSlot.slotshap == PK_SLOT_LEFT_PARA ||
             manuSlot.slotshap == PK_SLOT_RIGHT_PARA)
    {
        float rspoint[2];
        RevConvert((float *)&manuSlot.targpos, (float *)&manuSlot.slotobj.ptA, rspoint);
        rspoint[0] = rte_min(fabs(rspoint[0]), VEHICLE_LEN * 2.0f) * sign(rspoint[0]);
        rspoint[1] = rte_min(fabs(rspoint[1]), VEHICLE_WID * 0.2f) * sign(rspoint[1]);
        Convert((float *)&manuSlot.targpos, rspoint, (float *)&manuSlot.slotobj.ptA);

        RevConvert((float *)&manuSlot.targpos, (float *)&manuSlot.slotobj.ptB, rspoint);
        rspoint[0] = rte_min(fabs(rspoint[0]), VEHICLE_LEN * 1.5f) * sign(rspoint[0]);
        rspoint[1] = rte_min(fabs(rspoint[1]), VEHICLE_WID * 0.2f) * sign(rspoint[1]);
        Convert((float *)&manuSlot.targpos, rspoint, (float *)&manuSlot.slotobj.ptB);

        RevConvert((float *)&manuSlot.targpos, (float *)&manuSlot.slotobj.ptC, rspoint);
        rspoint[0] = rte_min(fabs(rspoint[0]), VEHICLE_LEN * 1.5f) * sign(rspoint[0]);
        Convert((float *)&manuSlot.targpos, rspoint, (float *)&manuSlot.slotobj.ptC);

        RevConvert((float *)&manuSlot.targpos, (float *)&manuSlot.slotobj.ptD, rspoint);
        rspoint[0] = rte_min(fabs(rspoint[0]), VEHICLE_LEN * 1.5f) * sign(rspoint[0]);
        Convert((float *)&manuSlot.targpos, rspoint, (float *)&manuSlot.slotobj.ptD);

        RevConvert((float *)&manuSlot.targpos, (float *)&manuSlot.slotobj.ptE, rspoint);
        rspoint[0] = rte_min(fabs(rspoint[0]), VEHICLE_LEN * 1.5f) * sign(rspoint[0]);
        rspoint[1] = rte_min(fabs(rspoint[1]), VEHICLE_WID * 0.2f) * sign(rspoint[1]);
        Convert((float *)&manuSlot.targpos, rspoint, (float *)&manuSlot.slotobj.ptE);

        RevConvert((float *)&manuSlot.targpos, (float *)&manuSlot.slotobj.ptF, rspoint);
        rspoint[0] = rte_min(fabs(rspoint[0]), VEHICLE_LEN * 2.0f) * sign(rspoint[0]);
        rspoint[1] = rte_min(fabs(rspoint[1]), VEHICLE_WID * 0.2f) * sign(rspoint[1]);
        Convert((float *)&manuSlot.targpos, rspoint, (float *)&manuSlot.slotobj.ptF);
    }

    return PATHPLAN_SUCCESS;
}
