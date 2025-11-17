#ifndef _PK_PATHEXECUTEREPLAN_H_
#define _PK_PATHEXECUTEREPLAN_H_

#include "PK_Calibration.h"
#include "PK_Utility.h"

int PathExec_ParaPostTraj(PK_Cur_AutodrPara curSt[1], float cvel_direct_fac,
                          float enviObj[5][4]);

int PathExec_ParaPostTrajPre(const PK_Cur_AutodrPara curSt[1], float enviObj[5][4],
                             float traject[2][TRAJITEM_LEN]);

bool PathExec_CheckVehColliionInSlot(const float finpoint[3], const float enviObj[4],
                                     float ckpoint[4], float dangerDist);

int PathExec_Replan_PathCheck(
    const int tra_num_temp,
    const float Planned_Path_Out[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
    const float min_arround_dist, const float cur_swell, const float finpoint[3],
    const float Envi_obj_left[4], const float Envi_obj_right[4],
    const float Envi_obj_side[2][4]);

int ExplorerConnect(float temp_tra[TRAJITEM_LEN], const float tarpos[3],
                    const int obj_num, const float obj_slot[][4],
                    float stp_trajs[][TRAJITEM_LEN], float swell, float startDs);

int ExplorePointToPoint(const float left_fac, const float stpoint[3],
                        const float finpoint[3], const int obj_num,
                        const float obj_slot[][4], float &swell,
                        float planPath[6][TRAJITEM_LEN]);

int ExplorePointToTra(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                      float dstTra[TRAJITEM_LEN], float enviObj[5][4],
                      const float curpos[4], const int slotshap);

int ExploreTraToTra(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                    const float dstPos[3], const float enviObj[5][4],
                    const float stPos[3]);

int PathExec_ParaPostParkOutTraj(PK_Cur_AutodrPara curSt[1], float cvel_direct_fac,
                                 float enviObj[5][4]);

#endif
