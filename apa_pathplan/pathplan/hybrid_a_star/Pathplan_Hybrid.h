#ifndef _PATHPLAN_HYBRID_H_
#define _PATHPLAN_HYBRID_H_
#include "PK_Utility.h"

struct Traject_Node
{
    float traj[TRAJITEM_LEN];
};

void HybridAStarResult(const int slot, const float target[3],
                       std::vector<Traject_Node> &result,
                       std::vector<Traject_Node> &trajs);

int HybridAStarRun(const char *jsonstr, const std::vector<Traject_Node> &trajs, int slot);

void HybridAClearResult(const int *slot, int slotNum);

bool HybridASlot(const int slotId);

int PathPlan_Rs(const float start[3], const float end[3], float trajs[][5]);

float HybridGetredius(const int steerAngle);

#endif