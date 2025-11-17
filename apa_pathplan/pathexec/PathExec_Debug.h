#ifndef _PK_PATHEXECUTE_DEBUG_H_
#define _PK_PATHEXECUTE_DEBUG_H_
#include "MathFunc.h"
#include "PK_PathExecuteP.h"
#include "Rte.h"

void PathExec_PrintPathStart();

void PathExec_PrintPathEnd();

void PathExec_PrintPathSwitch(const char *pathInf, const int flag);

void PathExec_PrintPathInfo(const float path[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                            const int pathNum);

void PathExec_PrintSlotInfo(const float obj[5][4], const float finpos[3],
                            const float avmpoint[8]);

void PathExec_PrintCurState(const PK_Cur_AutodrPara &CurState);

void PathExec_PrintObjInfo();

void PathExec_PrintSlotObjInfo(const FusionObj_T &objs);

void PathExec_PrintSlotBInfo();

#define PRINT_SWITCH_INFO(info)                   \
    {                                             \
        PathExec_PrintPathSwitch(info, __LINE__); \
    }

#endif
