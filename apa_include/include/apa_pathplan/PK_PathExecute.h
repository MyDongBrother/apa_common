#ifndef PK_PATHEXECUTE_H
#define PK_PATHEXECUTE_H

#include "PK_PathPlan.h"
#include "MathFunc.h"
#include "Rte.h"

/** Version and module identification */
#define PK_PathExecute_VENDOR_ID (1u)  // SCU
#define PK_PathExecute_MODULE_ID (12u) // PK_PathExecute

/** Component Version Information */
#define PK_PathExecute_SW_MAJOR_VERSION (3u)
#define PK_PathExecute_SW_MINOR_VERSION (2u)
#define PK_PathExecute_SW_PATCH_VERSION (61u)

void PK_InitPathExecute(void);
void PK_PathExecuteRun(void);
void PK_DistanceCtrlRun_yuan();

void PK_PathExecute_RunRate(float paras[8]);

void PK_Pre_locat(const float curpoint[4], float finpoint[4], const float EPS_angle,
                  const float cur_vel, const float dt);
void PathExec_PathInverse(int tra_num, float Act_traj[][TRAJITEM_LEN]);

#endif
