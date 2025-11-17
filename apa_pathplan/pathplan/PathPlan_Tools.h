#ifndef PK_PATHPLANTOOLS_H
#define PK_PATHPLANTOOLS_H
#include "PathPlan_Config.h"

int PathPlan_CheckPathOverFlow(const float *curPlannedPath);

int PathPlan_CheckPlannedPathValid(const float traject[][TRAJITEM_LEN], int pathNum);

void PathPlan_ReverseTraject(float traject[TRAJITEM_LEN]);

int PathPlan_SlotDir(const int shapeType);

float PathPlan_ShortDist(float trajs[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                         const int lineNum, int *revNum);

void PathPlan_UpdateTrajLen(float traject[TRAJITEM_LEN], const float finpoint[3],
                            uint8_t slotShape);

int Path_Check_Connect(const float stpoint[3], const float finpoint[3],
                       float TempPath[][TRAJITEM_LEN], int line_num);

void PathPlan_CheckTrajLen(const char *info, const float length);

int PathPlan_CheckTrajValid(const float traject[TRAJITEM_LEN]);

int PathPlan_CheckPosDiv(const float oldPos[3], const float newPos[3]);

int PathPlan_PathMerge(float trajs[][TRAJITEM_LEN], const int lineNum);

void PathPlan_UpdateTrajsLen(float trajs[][TRAJITEM_LEN], const int count);

void PathPlan_ClipLineSegmenty(float pt1[2], float pt2[2], float limit, float dir);

void PathPlan_ClipLineSegmentx(float pt1[2], float pt2[2], float limit, float dir);

#endif
