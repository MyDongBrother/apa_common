#include "EthIf.h"
#include "Ethenet_AvmSlot.h"
#include "PK_Calibration.h"
#include "PK_Location.h"
#include "Record_Log.h"
#include "Rte.h"
#include "MathFunc.h"
#include "CanIf.h"
#include "json/json.h"
#include <fstream>

static inline void SwapPoints(Point_T &p1, Point_T &p2)
{
    {
        float value = p1.x;
        p1.x        = p2.x;
        p2.x        = value;
    }
    {
        float value = p1.y;
        p1.y        = p2.y;
        p2.y        = value;
    }
}

float PointToLineDist(const float line1[2], const float line2[2], const float point[2])
{
    float A = line2[1] - line1[1];
    float B = line1[0] - line2[0];
    float C = line2[0] * line1[1] - line1[0] * line2[1];
    return fabsf(A * point[0] + B * point[1] + C) / sqrtf(A * A + B * B);
}

static int RollPoints(float pointOrd[4][2], Relation_T slotDir)
{
    Point_T bcde[4];
    memcpy(bcde, pointOrd, sizeof(bcde));

    for (int i = 0; i < 4; i++)
    {
        Relation_T dir[2];
        int indexb = (i + 0) % 4;
        int indexc = (i + 1) % 4;
        int indexd = (i + 2) % 4;
        int indexe = (i + 3) % 4;
        Get_Dist_Dir_Pt2PointLine(bcde[indexb], bcde[indexe], bcde[indexc], NULL,
                                  &dir[0]);
        Get_Dist_Dir_Pt2PointLine(bcde[indexb], bcde[indexe], bcde[indexd], NULL,
                                  &dir[1]);
        float distBE = PointToLineDist((float *)&bcde[indexd], (float *)&bcde[indexe],
                                       (float *)&bcde[indexb]);
        float distBC = PointToLineDist((float *)&bcde[indexc], (float *)&bcde[indexd],
                                       (float *)&bcde[indexb]);

        if (dir[0] == dir[1] && dir[0] == slotDir && bcde[indexe].y > bcde[indexb].y &&
            (distBC > distBE || distBE > VEHICLE_LEN))
        {
            memcpy(pointOrd[0], &bcde[indexb], sizeof(Point_T));
            memcpy(pointOrd[1], &bcde[indexc], sizeof(Point_T));
            memcpy(pointOrd[2], &bcde[indexd], sizeof(Point_T));
            memcpy(pointOrd[3], &bcde[indexe], sizeof(Point_T));
            return AVM_SUCCESS;
        }
    }

    memcpy(bcde, pointOrd, sizeof(bcde));
    for (int i = 0; i < 4; i++)
    {
        Relation_T dir[2];
        int indexb = (7 - i) % 4;
        int indexc = (6 - i) % 4;
        int indexd = (5 - i) % 4;
        int indexe = (4 - i) % 4;
        Get_Dist_Dir_Pt2PointLine(bcde[indexb], bcde[indexe], bcde[indexc], NULL,
                                  &dir[0]);
        Get_Dist_Dir_Pt2PointLine(bcde[indexb], bcde[indexe], bcde[indexd], NULL,
                                  &dir[1]);
        float distBE = PointToLineDist((float *)&bcde[indexd], (float *)&bcde[indexe],
                                       (float *)&bcde[indexb]);
        float distBC = PointToLineDist((float *)&bcde[indexc], (float *)&bcde[indexd],
                                       (float *)&bcde[indexb]);

        if (dir[0] == dir[1] && dir[0] == slotDir && bcde[indexe].y > bcde[indexb].y &&
            (distBC > distBE || distBE > VEHICLE_LEN))
        {
            memcpy(pointOrd[0], &bcde[indexb], sizeof(Point_T));
            memcpy(pointOrd[1], &bcde[indexc], sizeof(Point_T));
            memcpy(pointOrd[2], &bcde[indexd], sizeof(Point_T));
            memcpy(pointOrd[3], &bcde[indexe], sizeof(Point_T));
            return AVM_SUCCESS;
        }
    }

    return 1;
}

int CheckStopPoint(VisionSlotInfo &slot)
{
    Point_T pt1 = {slot.stopPoint[0][0], slot.stopPoint[0][1]};
    Point_T pt2 = {slot.stopPoint[1][0], slot.stopPoint[1][1]};

    for (int i = 0; i < 2; i++)
    {
        Point_T ptA = {slot.stopPoint[i % 4][0], slot.stopPoint[i % 4][1]};
        Point_T ptB = {slot.stopPoint[(i + 2) % 4][0], slot.stopPoint[(i + 2) % 4][1]};

        if (Is_TwoSegment_Cross(pt1, pt2, ptA, ptB) != 0)
        {
            return 1;
        }
    }

    for (int i = 0; i < 2; i++)
    {
        Point_T ptA, ptB;
        ptA.x = (slot.stopPoint[i % 4][0] + slot.stopPoint[(i + 1) % 4][0]) * 0.5;
        ptA.y = (slot.stopPoint[i % 4][1] + slot.stopPoint[(i + 1) % 4][1]) * 0.5;
        ptB.x = (slot.stopPoint[(i + 2) % 4][0] + slot.stopPoint[(i + 3) % 4][0]) * 0.5;
        ptB.y = (slot.stopPoint[(i + 2) % 4][1] + slot.stopPoint[(i + 3) % 4][1]) * 0.5;

        if (Is_TwoSegment_Cross(pt1, pt2, ptA, ptB) != 0)
        {
            return 1;
        }
    }

    for (int i = 0; i < 4; i++)
    {
        Point_T ptA = {slot.stopPoint[i % 4][0], slot.stopPoint[i % 4][1]};
        Point_T ptB = {slot.stopPoint[(i + 1) % 4][0], slot.stopPoint[(i + 1) % 4][1]};

        if (Is_TwoSegment_Cross(pt1, pt2, ptA, ptB) == 0)
        {
            continue;
        }
        return 1;
    }

    return 0;
}

int CheckSlotEntry(VisionSlotInfo &slot)
{
    enum
    {
        idx_fl,
        idx_fr,
        idx_rl,
        idx_rr
    };

    float dist[4];
    Relation_T dir[4];
    Point_T avmpoint[4];
    int pointIdx[4];
    LineSeg_T line;

    memcpy(avmpoint, slot.pointOrd, sizeof(Point_T) * 4);
    memcpy(&line, slot.stopPoint, sizeof(line));

    if (slot.stopPoint[0][1] <= slot.stopPoint[1][1])
    {
        memcpy(&line.pt1, slot.stopPoint[0], sizeof(Point_T));
        memcpy(&line.pt2, slot.stopPoint[1], sizeof(Point_T));
    }
    else
    {
        memcpy(&line.pt1, slot.stopPoint[1], sizeof(Point_T));
        memcpy(&line.pt2, slot.stopPoint[0], sizeof(Point_T));
    }

    Get_Dist_Dir_Pt2SegLine(line, avmpoint[0], &dist[0], &dir[0]);
    Get_Dist_Dir_Pt2SegLine(line, avmpoint[1], &dist[1], &dir[1]);
    Get_Dist_Dir_Pt2SegLine(line, avmpoint[2], &dist[2], &dir[2]);
    Get_Dist_Dir_Pt2SegLine(line, avmpoint[3], &dist[3], &dir[3]);

    if (dir[0] == dir[1] && dir[2] == dir[3] && dir[0] != dir[3])
    {
        auto v1 = rte_min(dist[0], dist[1]);
        auto v2 = rte_min(dist[2], dist[3]);

        if (v1 < v2)
        {
            pointIdx[idx_rl] = 0;
            pointIdx[idx_rr] = 1;
            pointIdx[idx_fl] = 2;
            pointIdx[idx_fr] = 3;
        }
        else
        {
            pointIdx[idx_rl] = 2;
            pointIdx[idx_rr] = 3;
            pointIdx[idx_fl] = 0;
            pointIdx[idx_fr] = 1;
        }
    }
    else if (dir[0] == dir[3] && dir[1] == dir[2] && dir[0] != dir[1])
    {
        auto v1 = rte_min(dist[0], dist[3]);
        auto v2 = rte_min(dist[1], dist[2]);

        if (v1 < v2)
        {
            pointIdx[idx_rl] = 0;
            pointIdx[idx_rr] = 3;
            pointIdx[idx_fl] = 1;
            pointIdx[idx_fr] = 2;
        }
        else
        {
            pointIdx[idx_rl] = 1;
            pointIdx[idx_rr] = 2;
            pointIdx[idx_fl] = 0;
            pointIdx[idx_fr] = 3;
        }
    }
    else
    {
        return 1;
    }

    line.pt1.x = (avmpoint[pointIdx[idx_rl]].x + avmpoint[pointIdx[idx_rr]].x) * 0.5;
    line.pt1.y = (avmpoint[pointIdx[idx_rl]].y + avmpoint[pointIdx[idx_rr]].y) * 0.5;
    line.pt2.x = (avmpoint[pointIdx[idx_fl]].x + avmpoint[pointIdx[idx_fr]].x) * 0.5;
    line.pt2.y = (avmpoint[pointIdx[idx_fl]].y + avmpoint[pointIdx[idx_fr]].y) * 0.5;

    Get_Dist_Dir_Pt2SegLine(line, avmpoint[pointIdx[idx_rl]], &dist[0], &dir[0]);
    Get_Dist_Dir_Pt2SegLine(line, avmpoint[pointIdx[idx_rr]], &dist[1], &dir[1]);
    Get_Dist_Dir_Pt2SegLine(line, avmpoint[pointIdx[idx_fl]], &dist[2], &dir[2]);
    Get_Dist_Dir_Pt2SegLine(line, avmpoint[pointIdx[idx_fr]], &dist[3], &dir[3]);

    if (dir[0] == dir[1] || dir[2] == dir[3])
    {
        return 2;
    }
    else if ((dir[0] != PK_ON_LEFT && dir[0] != PK_ON_RIGHT) ||
             (dir[1] != PK_ON_LEFT && dir[1] != PK_ON_RIGHT) ||
             (dir[2] != PK_ON_LEFT && dir[2] != PK_ON_RIGHT) ||
             (dir[3] != PK_ON_LEFT && dir[3] != PK_ON_RIGHT))
    {
        return 3;
    }
    else
    {
        if (dir[0] == PK_ON_RIGHT && dir[1] == PK_ON_LEFT)
        {
            int temp         = pointIdx[idx_rl];
            pointIdx[idx_rl] = pointIdx[idx_rr];
            pointIdx[idx_rr] = temp;
        }

        if (dir[2] == PK_ON_RIGHT && dir[3] == PK_ON_LEFT)
        {
            int temp         = pointIdx[idx_fl];
            pointIdx[idx_fl] = pointIdx[idx_fr];
            pointIdx[idx_fr] = temp;
        }
    }

    Vec2_T slotVec = {line.pt2.x - line.pt1.x, line.pt2.y - line.pt1.y};
    Normalize_Vec2(&slotVec);
    if (slotVec.vy < -0.8)
    {
        return 4;
    }

    if (AvmSlotDir(slot.pointOrd) == PK_ON_RIGHT)
    {
        memcpy(slot.pointOrd[0], &avmpoint[pointIdx[idx_fl]], sizeof(Point_T));
        memcpy(slot.pointOrd[1], &avmpoint[pointIdx[idx_rl]], sizeof(Point_T));
        memcpy(slot.pointOrd[2], &avmpoint[pointIdx[idx_rr]], sizeof(Point_T));
        memcpy(slot.pointOrd[3], &avmpoint[pointIdx[idx_fr]], sizeof(Point_T));
    }
    else if (AvmSlotDir(slot.pointOrd) == PK_ON_LEFT)
    {
        memcpy(slot.pointOrd[0], &avmpoint[pointIdx[idx_fr]], sizeof(Point_T));
        memcpy(slot.pointOrd[1], &avmpoint[pointIdx[idx_rr]], sizeof(Point_T));
        memcpy(slot.pointOrd[2], &avmpoint[pointIdx[idx_rl]], sizeof(Point_T));
        memcpy(slot.pointOrd[3], &avmpoint[pointIdx[idx_fl]], sizeof(Point_T));
    }
    else
    {
        return 5;
    }

    return AVM_SUCCESS;
}
