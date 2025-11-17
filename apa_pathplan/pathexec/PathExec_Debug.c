#include "PK_Calibration.h"
#include "PK_PathExecute.h"
#include "PK_PathExecuteP.h"
#include "PathExec_Func.h"
#include "PathExec_Obj.h"
#include "Rte_BSW.h"
#include "Rte_Func.h"
#include "MathFunc.h"
#include "Record_Buff.h"
#include <stdarg.h>

static void *execlog                      = NULL;
const int log_buff_len                    = 60 * 1024;
static char print_buff[log_buff_len / 10] = {0};

static void *PathExec_SaveLog(void *args);

static void PathExec_ReportInfo(const int start, const int end)
{
    PK_SendJsonData(&print_buff[start], end - start);
}

static void PathExec_RecordInfo(const int start, const int end)
{
    if (execlog == NULL)
    {
        return;
    }

    char strhead[128] = {0};
    struct timeval stamp;
    gettimeofday(&stamp, 0);
    int headlen = snprintf(strhead, sizeof(strhead) - 1, "%06ld:%06ld: ", stamp.tv_sec,
                           stamp.tv_usec);

    Buff_Put(execlog, strhead, headlen);
    int endpos = rte_min(end, (int)sizeof(print_buff) - 2);

    print_buff[endpos++] = '\n';
    // print_buff[endpos++] = '\0';

    Buff_Put(execlog, &print_buff[start], endpos - start);
}

static void PathExec_PrintPosInfo(const float curpos[4])
{
    int end = snprintf(print_buff, sizeof(print_buff),
                       "curposinfo:{\"curpos\": [%06lf,%06lf,%06lf,%06lf]}", curpos[0],
                       curpos[1], curpos[2], curpos[3]);
    PathExec_ReportInfo(0, end);
}

void PathExec_PrintPathStart()
{
    static uint8_t logbuff[log_buff_len] = {0};
    if (execlog != NULL)
    {
        return;
    }
    char fileName[128] = {0};
    struct tm absStamp;
    RTE_BSW_Get_AbsStamp(&absStamp);

    char pathName[64];
    Rte_BSW_Get_LogPath(pathName);
    snprintf(fileName, sizeof(fileName) - 1, "%s/%d_%02d_%02d_%02d_%02d_%02d_flog.txt",
             pathName, absStamp.tm_year, absStamp.tm_mon, absStamp.tm_mday,
             absStamp.tm_hour, absStamp.tm_min, absStamp.tm_sec);

    execlog     = NULL;
    auto logFid = fopen(fileName, "w");
    if (logFid == NULL)
    {
        return;
    }

    execlog = Buff_Init(logbuff, sizeof(logbuff), logFid);
    if (execlog == NULL)
    {
        fclose(logFid);
    }

    struct timeval stamp;
    gettimeofday(&stamp, 0);
    int end =
        snprintf(&print_buff[0], sizeof(print_buff) - 1, "start: sensor %d shape %d ",
                 RTE_PK_DataConvt_Get_IsAVM_Slot(), RTE_PK_SlotDetect_Get_SlotShape());
    end += strftime(&print_buff[end], sizeof(print_buff) - end - 1, "%Y-%m-%d %H:%M:%S",
                    localtime(&(stamp.tv_sec)));

    PathExec_RecordInfo(0, end);
}

void PathExec_PrintPathEnd()
{
    if (execlog == NULL)
    {
        return;
    }

    int end = snprintf(print_buff, sizeof(print_buff) - 1, "end: ");

    struct timeval stamp;
    gettimeofday(&stamp, 0);
    end += strftime(&print_buff[end], sizeof(print_buff) - end - 1, "%Y-%m-%d %H:%M:%S",
                    localtime(&(stamp.tv_sec)));

    PathExec_RecordInfo(0, end);
    Buff_UnInit(execlog);
    execlog = NULL;
}

void PathExec_PrintPathSwitch(const char *pathInf, const int flag)
{
    int end =
        snprintf(print_buff, sizeof(print_buff) - 1, "switch: %d %s", flag, pathInf);
    PathExec_RecordInfo(0, end);
}

void PathExec_PrintPathInfo(const float path[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                            const int pathNum)
{
    int end = 40;
    for (int index = 0; index < pathNum; index++)
    {
        float endPos[3];
        PK_Get_Path_EndPos(path[index], endPos);
        end += snprintf(&print_buff[end], sizeof(print_buff) - 1 - end,
                        "%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,",
                        path[index][0], path[index][1], path[index][2], path[index][3],
                        path[index][4], endPos[0], endPos[1], endPos[2]);
    }

    if (end < (int)sizeof(print_buff) - 2)
    {
        const char *jsonHead = "pathinfo:{\"pathstr\": [";
        int start            = 40 - strlen(jsonHead);
        memcpy(&print_buff[start], jsonHead, strlen(jsonHead));
        print_buff[end - 1] = ']';
        print_buff[end - 0] = '}';
        end                 = end + 1;
        PathExec_ReportInfo(start, end);

        const char *strhead = "pathstr: ";
        start               = 40 - strlen(strhead);
        memcpy(&print_buff[start], strhead, strlen(strhead));
        end = end - 2;
        PathExec_RecordInfo(start, end);
    }
}

void PathExec_PrintSlotInfo(const float obj[5][4], const float finpos[3],
                            const float avmpoint[8])
{
    int end = 40;
    float avm[8];
    if (RTE_PK_DataConvt_Get_IsAVM_Slot() == 0)
    {
        memset(avm, 0, sizeof(avm));
    }
    else
    {
        memcpy(avm, avmpoint, sizeof(avm));
    }

    end += snprintf(
        &print_buff[end], sizeof(print_buff) - 1 - end,
        "%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%"
        "06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf",
        obj[0][0], obj[0][1], obj[1][0], obj[1][1], obj[2][0], obj[2][1], obj[3][0],
        obj[3][1], obj[4][0], obj[4][1], obj[4][2], obj[4][3], finpos[0], finpos[1],
        finpos[2], avm[0], avm[1], avm[2], avm[3], avm[4], avm[5], avm[6], avm[7]);

    if (end < (int)sizeof(print_buff) - 2)
    {
        const char *jsonHead = "slotinfo:{\"slotobj\": [";
        int start            = 40 - strlen(jsonHead);
        memcpy(&print_buff[start], jsonHead, strlen(jsonHead));
        print_buff[end - 1] = ']';
        print_buff[end - 0] = '}';
        end                 = end + 1;
        PathExec_ReportInfo(start, end);

        const char *strhead = "slotobj: ";
        start               = 40 - strlen(strhead);
        memcpy(&print_buff[start], strhead, strlen(strhead));
        end = end - 2;
        PathExec_RecordInfo(start, end);
    }
}

void PathExec_PrintSlotBInfo()
{
    Avm_Pot_T avmSlot;
    RTE_PD_Get_AVM_SlotInfo_B(&avmSlot);

    Avm_Obj_T avmObj;
    RTE_PD_Get_AVM_SlotObs_B(&avmObj);

    Avm_Pot_T targetPot;
    RTE_PK_DataConvt_Get_TargAVM_SlotInfo(&targetPot);

    int end = 40;
    end += snprintf(&print_buff[end], sizeof(print_buff) - 1 - end,
                    "%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%"
                    "06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf,%06lf\n",
                    avmSlot.near_rear.x, avmSlot.near_rear.y, avmSlot.far_rear.x,
                    avmSlot.far_rear.y, avmSlot.far_front.x, avmSlot.far_front.y,
                    avmSlot.near_front.x, avmSlot.near_front.y, avmObj.ls.pt1.x,
                    avmObj.ls.pt1.y, avmObj.ls.pt2.x, avmObj.ls.pt2.y,
                    targetPot.near_rear.x, targetPot.near_rear.y, targetPot.far_rear.x,
                    targetPot.far_rear.y, targetPot.far_front.x, targetPot.far_front.y,
                    targetPot.near_front.x, targetPot.near_front.y);

    if (end < (int)sizeof(print_buff) - 2)
    {
        const char *strhead = "slotobjB: ";
        int start           = 40 - strlen(strhead);
        memcpy(&print_buff[start], strhead, strlen(strhead));
        end = end - 2;
        PathExec_RecordInfo(start, end);
    }
}

void PathExec_PrintObjInfo()
{
    FusionObj_T objs;
    PathExec_GetObsObj_AnotherSide(objs);
    int end = snprintf(print_buff, sizeof(print_buff) - 1, "object: ");
    for (int index = 0; index < objs.num; index++)
    {
        if (Get_Segment_Len(objs.obj[index]) < 0.10)
        {
            continue;
        }

        end +=
            snprintf(&print_buff[end], sizeof(print_buff) - end - 1,
                     "%04f,%04f,%04f,%04f,", objs.obj[index].pt1.x, objs.obj[index].pt1.y,
                     objs.obj[index].pt2.x, objs.obj[index].pt2.y);
    }
    end = end - 1;
    PathExec_RecordInfo(0, end);

    end = snprintf(print_buff, sizeof(print_buff) - 1, "object: ");
    FusionObj_T fusInfo;
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Left(&fusInfo);
    for (int index = 0; index < fusInfo.num; index++)
    {
        if (fusInfo.attr[index] == 0 || Get_Segment_Len(fusInfo.obj[index]) < 0.10)
        {
            continue;
        }

        end += snprintf(&print_buff[end], sizeof(print_buff) - end - 1,
                        "%04f,%04f,%04f,%04f,", fusInfo.obj[index].pt1.x,
                        fusInfo.obj[index].pt1.y, fusInfo.obj[index].pt2.x,
                        fusInfo.obj[index].pt2.y);
    }
    end = end - 1;
    PathExec_RecordInfo(0, end);

    end = snprintf(print_buff, sizeof(print_buff) - 1, "object: ");
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Right(&fusInfo);
    for (int index = 0; index < fusInfo.num; index++)
    {
        if (fusInfo.attr[index] == 0 || Get_Segment_Len(fusInfo.obj[index]) < 0.10)
        {
            continue;
        }

        end += snprintf(&print_buff[end], sizeof(print_buff) - end - 1,
                        "%04f,%04f,%04f,%04f,", fusInfo.obj[index].pt1.x,
                        fusInfo.obj[index].pt1.y, fusInfo.obj[index].pt2.x,
                        fusInfo.obj[index].pt2.y);
    }

    end = end - 1;
    PathExec_RecordInfo(0, end);
}

void PathExec_PrintSlotObjInfo(const FusionObj_T &objs)
{
    int end   = snprintf(print_buff, sizeof(print_buff) - 1, "object slot: ");
    int count = 0;
    for (int index = 0; index < SF_OBJ_NUM / 2 && count < objs.num; index++)
    {
        if (Get_Segment_Len(objs.obj[index]) < 0.10 || objs.attr[index] == 0)
        {
            continue;
        }

        count++;
        end +=
            snprintf(&print_buff[end], sizeof(print_buff) - end - 1,
                     "%04f,%04f,%04f,%04f,", objs.obj[index].pt1.x, objs.obj[index].pt1.y,
                     objs.obj[index].pt2.x, objs.obj[index].pt2.y);
    }

    count = 0;
    for (int index = SF_OBJ_NUM / 2; index < SF_OBJ_NUM && count < objs.num_accu; index++)
    {
        if (Get_Segment_Len(objs.obj[index]) < 0.10 || objs.attr[index] == 0)
        {
            continue;
        }

        count++;
        end +=
            snprintf(&print_buff[end], sizeof(print_buff) - end - 1,
                     "%04f,%04f,%04f,%04f,", objs.obj[index].pt1.x, objs.obj[index].pt1.y,
                     objs.obj[index].pt2.x, objs.obj[index].pt2.y);
    }

    end = end - 1;
    PathExec_RecordInfo(0, end);
}

#define PRINT_NAME_VALUE(FORMAT, NAME)                                                 \
    {                                                                                  \
        buff_len += snprintf(&print_buff[buff_len], sizeof(print_buff) - buff_len - 1, \
                             "%s:", #NAME);                                            \
        buff_len += snprintf(&print_buff[buff_len], sizeof(print_buff) - buff_len - 1, \
                             FORMAT, CurState.NAME);                                   \
        print_buff[buff_len++] = ' ';                                                  \
    }

#define PRINT_VALUE(NAME, VALUE)                                                       \
    {                                                                                  \
        buff_len += snprintf(&print_buff[buff_len], sizeof(print_buff) - buff_len - 1, \
                             "%s: %d", NAME, VALUE);                                   \
    }

void PathExec_PrintCurState(const PK_Cur_AutodrPara &CurState)
{
    int buff_len = snprintf(print_buff, sizeof(print_buff) - 1, "%d %f ", pre_nout_ccp,
                            RTE_PK_PathExecute_Get_rx_brake());

    PRINT_NAME_VALUE("%02d", cur_tra_nth);
    PRINT_NAME_VALUE("%02d", tra_num);
    PRINT_NAME_VALUE(
        "%01d", cur_tra_state); //  0- static curvature path; 1- dynamic curvature path;
    PRINT_NAME_VALUE(
        "%01d",
        slotshape); // 0/no_slot left para=1, right para=2, left vert= 3 right para = 4
    PRINT_NAME_VALUE("%04d", step_gone);
    PRINT_NAME_VALUE("%01d", B_BrakeState);      // 1 start brake
    PRINT_NAME_VALUE("%01d", B_DynaPlanCounter); // the counter of dynamic planning
    PRINT_NAME_VALUE("%06f", ry_RePathSt);       // the first plan flag
    PRINT_NAME_VALUE("%02d", B_Danger_filter);   // from obj aviod
    PRINT_NAME_VALUE("%04x", B_replan);
    PRINT_NAME_VALUE("%01d", brakeSt);
    PRINT_NAME_VALUE("%01d", B_Stop_Counter);
    PRINT_NAME_VALUE("%02d", AfterTra_nth);
    PRINT_NAME_VALUE("%04d ", stillCount);
    PRINT_NAME_VALUE("%01d ", B_ReplanResult);
    PRINT_NAME_VALUE("%06f", curpos[0]);
    PRINT_NAME_VALUE("%06f", curpos[1]);
    PRINT_NAME_VALUE("%06f", curpos[2]);
    PRINT_NAME_VALUE("%06f", curpos[3]);
    PRINT_NAME_VALUE("%06f", finpos[0]);
    PRINT_NAME_VALUE("%06f", finpos[1]);
    PRINT_NAME_VALUE("%06f", finpos[2]);
    PRINT_NAME_VALUE("%06f", cur_eps);
    PRINT_NAME_VALUE("%06f", cur_vel);
    PRINT_NAME_VALUE("%06f", ds_gone_dir);
    PRINT_NAME_VALUE("%06f", ds_start_dir);
    PRINT_NAME_VALUE("%06f", ceps);
    PRINT_NAME_VALUE("%06f", cvel);
    PRINT_NAME_VALUE("%06f", cvel_last);
    PRINT_NAME_VALUE("%06f", de);
    PRINT_NAME_VALUE("%06f", de_last);
    PRINT_NAME_VALUE("%06f", sum_de);
    PRINT_NAME_VALUE("%06f", rx_cur_to_fin);
    PRINT_NAME_VALUE("%06f", ry_cur_to_fin);
    PRINT_NAME_VALUE("%06f", rtheta_cur_to_fin);
    PRINT_NAME_VALUE("%06f", deps);
    PRINT_NAME_VALUE("%06f", Rou_req);
    PRINT_NAME_VALUE("%06f", EPS_req);
    PRINT_NAME_VALUE("%06f",
                     drx_brake); // the predict sliding distance to stop the vehicle
    PRINT_NAME_VALUE(
        "%06f", drx_crou); // the dist in advance near a curve rate change point to change
                           // eps angle to next path for a continuous path sate
    PRINT_NAME_VALUE("%06f", rx_RePathSt);
    PRINT_NAME_VALUE(
        "%06f",
        rx_brake); // the distance to a path 's endpos that need to stop the vehicle
    PRINT_NAME_VALUE(
        "%06f",
        rxf); //=(x-xf)*cosf(thetaf)+(y-yf)*sinf(thetaf));  //curpos relative to targpos
    PRINT_NAME_VALUE("%06f", rxs);
    PRINT_NAME_VALUE("%06f", leftDist);
    PRINT_NAME_VALUE("%06f", rthetaf); //=Round_PI(thetaf-theta));銆€
    PRINT_NAME_VALUE("%06f", rthetas); //=Round_PI(thetas-theta));
    PRINT_NAME_VALUE("%06f", min_arround_dist);
    PRINT_NAME_VALUE("%06f", min_arround_dist_dt);
    PRINT_NAME_VALUE("%06f", de_dt);
    PRINT_NAME_VALUE("%02d", bUpdate);       //  for AVM slot update during process B
    PRINT_NAME_VALUE("%03d", dangerCounter); //  for AVM slot update during process B
    PRINT_NAME_VALUE("%06f", percent);
    PRINT_NAME_VALUE("%2d", B_FirstPlanFlag);
    PRINT_NAME_VALUE("%d", updatePathNum);
    PRINT_VALUE("still ", PathExec_IsVehStill());
    PRINT_NAME_VALUE("%.2f", pathDist);
    PRINT_NAME_VALUE("%d", gear);
    PRINT_NAME_VALUE("%d", replanSt);
    PRINT_NAME_VALUE("%d", targState);
    PRINT_NAME_VALUE("%01d", hasStop);
    PathExec_RecordInfo(0, buff_len);

    PathExec_PrintPosInfo(CurState.curpos);
}
