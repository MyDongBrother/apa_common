#include "PK_PathPlanP.h"
#include "json/json.h"
#include "Record_Buff.h"
#include "PathPlan_Debug.h"
#include <fstream>

static void PathPlan_SlotsJsonInfo(const SlotInfo_T &slot, const float stpoint[4],
                                   Json::Value &root);
static void PathPlan_ObjsJsonInfo(const int FS_ObjDir[64], const float FS_Obj[64][4],
                                  const int FS_ObjDir_Rear[64],
                                  const float FS_Obj_Rear[64][4], Json::Value &root);
static void PathPlan_TrajsJsonInfo(const float trajs[][TRAJITEM_LEN], const uint32_t num,
                                   const PlanDataCase &PlanDataVer, const int slotIndex,
                                   Json::Value &root);

static uint32_t s_pathplan_type[4];
static uint32_t s_pathplan_count[4];
static uint32_t s_pathplan_step[4];
static uint8_t s_pathplan_index = 0;

void PathPlan_InitFootPrints()
{
    for (size_t i = 0; i < sizeof(s_pathplan_step) / sizeof(s_pathplan_step[0]); i++)
    {
        s_pathplan_type[i]  = 0;
        s_pathplan_count[i] = 0;
        s_pathplan_step[i]  = 0;
    }
    s_pathplan_index = 0;
}

void PathPlan_FootPrints(int type, uint8_t level, uint8_t step, int num)
{
    if (type >= sizeof(s_pathplan_type) / sizeof(s_pathplan_type[0]))
    {
        return;
    }
    if (level >= 4)
    {
        return;
    }

    uint32_t data = s_pathplan_type[type];
    if (level == 0)
    {
        data = (data & 0x00ffffff) | (step << 24);
    }
    if (level == 1)
    {
        data = (data & 0xff00ffff) | (step << 16);
    }
    if (level == 2)
    {
        data = (data & 0xffff00ff) | (step << 8);
    }
    if (level == 3)
    {
        data = (data & 0xffffff00) | (step << 0);
    }
    s_pathplan_type[type] = data;

    data          = s_pathplan_step[type];
    uint8_t index = s_pathplan_index & 0xff;
    if (level == 0)
    {
        data = (data & 0x00ffffff) | (index << 24);
    }
    if (level == 1)
    {
        data = (data & 0xff00ffff) | (index << 16);
    }
    if (level == 2)
    {
        data = (data & 0xffff00ff) | (index << 8);
    }
    if (level == 3)
    {
        data = (data & 0xffffff00) | (index << 0);
    }
    s_pathplan_step[type] = data;
    s_pathplan_index      = s_pathplan_index + 1;

    data = s_pathplan_count[type];
    if (num < 0)
    {
        num = 0;
    }
    uint8_t number = num & 0xff;
    if (level == 0)
    {
        data = (data & 0x00ffffff) | (number << 24);
    }
    if (level == 1)
    {
        data = (data & 0xff00ffff) | (number << 16);
    }
    if (level == 2)
    {
        data = (data & 0xffff00ff) | (number << 8);
    }
    if (level == 3)
    {
        data = (data & 0xffffff00) | (number << 0);
    }
    s_pathplan_count[type] = data;
}

int PathPlan_PrintFootPrints(uint32_t *debugs)
{
    memcpy(&debugs[0], s_pathplan_type, sizeof(s_pathplan_type));
    memcpy(&debugs[4], s_pathplan_step, sizeof(s_pathplan_step));
    memcpy(&debugs[8], s_pathplan_count, sizeof(s_pathplan_count));
    return 12;
}

bool record_result = false;
static void PathPlan_RecordFile(char fileName[256], const int slotId,
                                const char *nameProx)
{
    struct timeval stamp;
    gettimeofday(&stamp, NULL);

    char pathName[64];
    Rte_BSW_Get_LogPath(pathName);
    snprintf(fileName, 255, "%s/%ld_%ld_%d_%s", pathName, stamp.tv_sec,
             stamp.tv_usec / 1000, slotId, nameProx);
}

void PathPlan_ReportSlotsInfo(const SlotInfo_T &slot, const float stpoint[4])
{
    if (!is_save_log)
    {
        return;
    }

    Json::Value root;
    PathPlan_SlotsJsonInfo(slot, stpoint, root);

    std::string strtemp = root.toStyledString();
    if (record_result)
    {
        char fileName[256];
        PathPlan_RecordFile(fileName, slot.slot_index, "slotdata.json");
        std::ofstream cfgFile(fileName);
        cfgFile << strtemp;
        cfgFile.close();
        return;
    }

    strtemp.insert(0, "$start$slotinfo:");
    strtemp.append("$end$");
    PK_SendJsonData(strtemp.c_str(), strtemp.size());
}

void PathPlan_ReportObjsInfo(const int FS_ObjDir[FS_OBJ_ARR_NUM],
                             const float FS_Obj[FS_OBJ_ARR_NUM][4],
                             const int FS_ObjDir_Rear[FS_OBJ_ARR_NUM],
                             const float FS_Obj_Rear[FS_OBJ_ARR_NUM][4])
{
    if (!is_save_log)
    {
        return;
    }

    Json::Value root;
    PathPlan_ObjsJsonInfo(FS_ObjDir, FS_Obj, FS_ObjDir_Rear, FS_Obj_Rear, root);

    std::string strtemp = root.toStyledString();
    if (record_result)
    {
        char fileName[256];
        PathPlan_RecordFile(fileName, 0, "obj.json");
        std::ofstream cfgFile(fileName);
        cfgFile << strtemp;
        cfgFile.close();
        return;
    }

    strtemp.insert(0, "$start$objinfo:");
    strtemp.append("$end$");
    PK_SendJsonData(strtemp.c_str(), strtemp.size());
}

void PathPlan_ReportTrajsInfo(const float trajs[][TRAJITEM_LEN], const uint32_t num,
                              const PlanDataCase &PlanDataVer, const int slotIndex)
{
    if (!is_save_log)
    {
        return;
    }

    Json::Value root;
    PathPlan_TrajsJsonInfo(trajs, num, PlanDataVer, slotIndex, root);

    std::string strtemp = root.toStyledString();
    if (record_result)
    {
        std::ofstream cfgFile("pathdata.json");
        cfgFile << strtemp;
        cfgFile.close();
    }

    strtemp.insert(0, "$start$pathinfo:");
    strtemp.append("$end$");
    PK_SendJsonData(strtemp.c_str(), strtemp.size());
}

static void PathPlan_SlotsJsonInfo(const SlotInfo_T &slot, const float stpoint[4],
                                   Json::Value &root)
{
    root["slotindex"] = (int)slot.slot_index;
    root["curpos"].append((float)stpoint[0]);
    root["curpos"].append((float)stpoint[1]);
    root["curpos"].append((float)stpoint[2]);
    root["curpos"].append((float)stpoint[3]);

    root["slotobjs"].append((float)slot.slotobj.ptA.x);
    root["slotobjs"].append((float)slot.slotobj.ptA.y);
    root["slotobjs"].append((float)slot.slotobj.ptB.x);
    root["slotobjs"].append((float)slot.slotobj.ptB.y);
    root["slotobjs"].append((float)slot.slotobj.ptC.x);
    root["slotobjs"].append((float)slot.slotobj.ptC.y);
    root["slotobjs"].append((float)slot.slotobj.ptD.x);
    root["slotobjs"].append((float)slot.slotobj.ptD.y);
    root["slotobjs"].append((float)slot.slotobj.ptE.x);
    root["slotobjs"].append((float)slot.slotobj.ptE.y);
    root["slotobjs"].append((float)slot.slotobj.ptF.x);
    root["slotobjs"].append((float)slot.slotobj.ptF.y);

    if (slot.is_vision_slot != 0)
    {
        root["slotamvs"].append((float)slot.avm_point.near_rear.x);
        root["slotamvs"].append((float)slot.avm_point.near_rear.y);
        root["slotamvs"].append((float)slot.avm_point.far_rear.x);
        root["slotamvs"].append((float)slot.avm_point.far_rear.y);
        root["slotamvs"].append((float)slot.avm_point.far_front.x);
        root["slotamvs"].append((float)slot.avm_point.far_front.y);
        root["slotamvs"].append((float)slot.avm_point.near_front.x);
        root["slotamvs"].append((float)slot.avm_point.near_front.y);
    }

    root["slottarg"].append((float)slot.targpos.x);
    root["slottarg"].append((float)slot.targpos.y);
    root["slottarg"].append((float)slot.targpos.theta);

    root["slotshape"]    = (int)slot.slotshap;
    root["slotisvision"] = (int)slot.is_vision_slot;
    root["slotstopper"]  = (int)slot.has_stopper;
}

static void PathPlan_ObjsJsonInfo(const int FS_ObjDir[FS_OBJ_ARR_NUM],
                                  const float FS_Obj[FS_OBJ_ARR_NUM][4],
                                  const int FS_ObjDir_Rear[FS_OBJ_ARR_NUM],
                                  const float FS_Obj_Rear[FS_OBJ_ARR_NUM][4],
                                  Json::Value &root)
{
    auto FillJsonFunc = [](const int ObjDir[SF_OBJ_NUM], const float Obj[SF_OBJ_NUM][4],
                           const char *keyValue, Json::Value &root) {
        for (int i = 0; i < SF_OBJ_NUM; i++)
        {
            if (ObjDir[i] == 0)
            {
                continue;
            }

            int nameIdx          = ObjDir[i] + i;
            std::string itemName = std::to_string(nameIdx);
            if (itemName.empty())
            {
                continue;
            }
            while (root[keyValue].isMember(itemName))
            {
                nameIdx++;
                itemName = std::to_string(nameIdx);
            }
            if (itemName.empty())
            {
                continue;
            }
            root[keyValue][itemName].append((float)Obj[i][0]);
            root[keyValue][itemName].append((float)Obj[i][1]);
            root[keyValue][itemName].append((float)Obj[i][2]);
            root[keyValue][itemName].append((float)Obj[i][3]);
        }
    };

    FillJsonFunc(&FS_ObjDir[0], &FS_Obj[0], "ultrolobjs", root);
    FillJsonFunc(&FS_ObjDir[SF_OBJ_NUM], &FS_Obj[SF_OBJ_NUM], "ultrorobjs", root);
    FillJsonFunc(&FS_ObjDir_Rear[0], &FS_Obj_Rear[0], "ultrolobjs_rear", root);
    FillJsonFunc(&FS_ObjDir_Rear[SF_OBJ_NUM], &FS_Obj_Rear[SF_OBJ_NUM], "ultrorobjs_rear",
                 root);
}

static void PathPlan_TrajsJsonInfo(const float trajs[][TRAJITEM_LEN], const uint32_t num,
                                   const PlanDataCase &PlanDataVer, const int slotIndex,
                                   Json::Value &root)
{
    root["slotindex"] = slotIndex;
    root["pointb_danger"].append((float)PlanDataVer.pointb_danger[0]);
    root["pointb_danger"].append((float)PlanDataVer.pointb_danger[1]);
    root["pointb_danger"].append((float)PlanDataVer.pointb_danger[2]);

    root["pointf_danger"].append((float)PlanDataVer.pointf_danger[0]);
    root["pointf_danger"].append((float)PlanDataVer.pointf_danger[1]);
    root["pointf_danger"].append((float)PlanDataVer.pointf_danger[2]);

    root["runslotobjs"].append((float)PlanDataVer.obj_slot[0][0]);
    root["runslotobjs"].append((float)PlanDataVer.obj_slot[0][1]);
    root["runslotobjs"].append((float)PlanDataVer.obj_slot[0][2]);
    root["runslotobjs"].append((float)PlanDataVer.obj_slot[0][3]);
    root["runslotobjs"].append((float)PlanDataVer.obj_slot[2][0]);
    root["runslotobjs"].append((float)PlanDataVer.obj_slot[2][1]);
    root["runslotobjs"].append((float)PlanDataVer.obj_slot[2][2]);
    root["runslotobjs"].append((float)PlanDataVer.obj_slot[2][3]);
    root["runslotobjs"].append((float)PlanDataVer.obj_slot[4][0]);
    root["runslotobjs"].append((float)PlanDataVer.obj_slot[4][1]);
    root["runslotobjs"].append((float)PlanDataVer.obj_slot[4][2]);
    root["runslotobjs"].append((float)PlanDataVer.obj_slot[4][3]);

    for (auto i = 0; i < PlanDataVer.obj_danger_num; i++)
    {
        if (PlanDataVer.obj_danger_dir[i] == 1)
        {
            std::string itemName = std::to_string(i);
            if (itemName.empty())
            {
                continue;
            }
            root["runlobjs"][itemName].append((float)PlanDataVer.obj_danger[i][0]);
            root["runlobjs"][itemName].append((float)PlanDataVer.obj_danger[i][1]);
            root["runlobjs"][itemName].append((float)PlanDataVer.obj_danger[i][2]);
            root["runlobjs"][itemName].append((float)PlanDataVer.obj_danger[i][3]);
        }
    }

    for (auto i = 0; i < PlanDataVer.obj_danger_num; i++)
    {
        if (PlanDataVer.obj_danger_dir[i] == 2)
        {
            std::string itemName = std::to_string(i);
            if (itemName.empty())
            {
                continue;
            }
            root["runrobjs"][itemName].append((float)PlanDataVer.obj_danger[i][0]);
            root["runrobjs"][itemName].append((float)PlanDataVer.obj_danger[i][1]);
            root["runrobjs"][itemName].append((float)PlanDataVer.obj_danger[i][2]);
            root["runrobjs"][itemName].append((float)PlanDataVer.obj_danger[i][3]);
        }
    }

    for (uint32_t i = 0; i < num; i++)
    {
        char itemName[3];
        sprintf(itemName, "%02d", i);
        root["pathstrs"][itemName].append((float)trajs[i][0]);
        root["pathstrs"][itemName].append((float)trajs[i][1]);
        root["pathstrs"][itemName].append((float)Round_PI(trajs[i][2]));
        root["pathstrs"][itemName].append((float)trajs[i][3]);
        root["pathstrs"][itemName].append((float)trajs[i][4]);
    }

    if (num > 0)
    {
        float endPos[3];
        PK_Get_Path_EndPos(trajs[num - 1], endPos);
        float dist = endPos[0] * endPos[0] + endPos[1] * endPos[1];
        if (dist < 0.001)
        {
            printf("---------------------");
        }
    }
}

static void *pathlog = NULL;
void PathPlan_PrintPathStart()
{
    static uint8_t logbuff[40 * 1024] = {0};
    if (pathlog != NULL)
    {
        return;
    }
    char fileName[128] = {0};
    struct tm absStamp;
    RTE_BSW_Get_AbsStamp(&absStamp);

    char pathName[64];
    Rte_BSW_Get_LogPath(pathName);
    snprintf(fileName, sizeof(fileName) - 1, "%s/%d_%02d_%02d_%02d_%02d_%02d_path.txt",
             pathName, absStamp.tm_year, absStamp.tm_mon, absStamp.tm_mday,
             absStamp.tm_hour, absStamp.tm_min, absStamp.tm_sec);

    pathlog     = NULL;
    auto logFid = fopen(fileName, "w");
    if (logFid == NULL)
    {
        return;
    }

    pathlog = Buff_Init(logbuff, sizeof(logbuff), logFid);
    if (pathlog == NULL)
    {
        fclose(logFid);
    }

    PlanInfoType slotPlan;
    RTE_PK_DataConvt_Get_Target_PlanInfo(&slotPlan);
    PathPlan_PrintPathInfo(slotPlan);
}

void PathPlan_PrintPathEnd()
{
    if (pathlog == NULL)
    {
        return;
    }

    struct timeval stamp;
    gettimeofday(&stamp, 0);

    char endStr[256];
    int end = strftime(endStr, sizeof(endStr) - 1, "%Y-%m-%d %H:%M:%S\n",
                       localtime(&(stamp.tv_sec)));

    Buff_Put(pathlog, endStr, end);
    Buff_UnInit(pathlog);
    pathlog = NULL;
}

void PathPlan_PrintPathInfo(const PlanInfoType &slotPla)
{
    if (pathlog == NULL)
    {
        return;
    }

    struct timeval stamp;
    gettimeofday(&stamp, NULL);
    char printbuf[256] = {0};
    float end[3];

    int pathNum = slotPla.Path_num;
    PK_Get_Path_EndPos(slotPla.Act_traj[pathNum - 1], end);
    int dataLen = snprintf(
        printbuf, sizeof(printbuf), "%ld:%ld start:%05f,%05f,%05f end:%05f,%05f,%05f\n",
        stamp.tv_sec, stamp.tv_usec, slotPla.Act_traj[0][0], slotPla.Act_traj[0][1],
        slotPla.Act_traj[0][2], end[0], end[1], end[2]);
    Buff_Put(pathlog, printbuf, dataLen);

    for (int i = 0; i < pathNum; i++)
    {
        int dataLen = snprintf(printbuf, sizeof(printbuf), "%05f,%05f,%05f,%05f,%05f\n",
                               slotPla.Act_traj[i][0], slotPla.Act_traj[i][1],
                               slotPla.Act_traj[i][2], slotPla.Act_traj[i][3],
                               slotPla.Act_traj[i][4]);
        Buff_Put(pathlog, printbuf, dataLen);
    }
}

void PathPlan_PrintPathUpdateResult(const int check[6])
{
    if (pathlog == NULL)
    {
        return;
    }

    struct timeval stamp;
    gettimeofday(&stamp, NULL);
    char printbuf[256] = {0};

    PlanInfoType curpath;
    RTE_PK_PathExecute_Get_CurTraj(&curpath);

    // printf("Path update: %d %d %d\n", check[0], check[1], check[2]);
    int dataLen =
        snprintf(printbuf, sizeof(printbuf),
                 "%ld:%ld check:%d,%d,%d,%d,%d,%d path:%d,%d traj:%f,%f,%f,%f,%f\n",
                 stamp.tv_sec, stamp.tv_usec, check[0], check[1], check[2], check[3],
                 check[4], check[5], curpath.Path_num, curpath.IsDirConect,
                 curpath.Act_traj[0][0], curpath.Act_traj[0][1], curpath.Act_traj[0][2],
                 curpath.Act_traj[0][3], curpath.Act_traj[0][4]);

    Buff_Put(pathlog, printbuf, dataLen);
}

void PathPlan_PrintDangerObj(const char *inf, const float traj[][TRAJITEM_LEN],
                             int pathNum, const PlanDataCase &PlanDataVer)
{
    if (pathlog == NULL)
    {
        PathPlan_PrintPathStart();
    }

    if (pathlog == NULL)
    {
        return;
    }

    char printbuf[128] = {0};

    int dataLen = snprintf(
        printbuf, sizeof(printbuf), "%s:%04f,%04f,%04f %04f,%04f,%04f\n", inf,
        PlanDataVer.stpoint[0], PlanDataVer.stpoint[1], PlanDataVer.stpoint[2],
        PlanDataVer.finpoint[0], PlanDataVer.finpoint[1], PlanDataVer.finpoint[2]);
    Buff_Put(pathlog, printbuf, dataLen);

    for (int i = 0; i < pathNum; i++)
    {
        int dataLen =
            snprintf(printbuf, sizeof(printbuf), "traj:%04f,%04f,%04f,%04f,%04f\n",
                     traj[i][0], traj[i][1], traj[i][2], traj[i][3], traj[i][4]);
        Buff_Put(pathlog, printbuf, dataLen);
    }

    for (int i = 0; i < PlanDataVer.obj_danger_num; i++)
    {
        int dataLen =
            snprintf(printbuf, sizeof(printbuf), "obj:%d %04f,%04f,%04f,%04f\n",
                     PlanDataVer.obj_danger_dir[i], PlanDataVer.obj_danger[i][0],
                     PlanDataVer.obj_danger[i][1], PlanDataVer.obj_danger[i][2],
                     PlanDataVer.obj_danger[i][3]);
        Buff_Put(pathlog, printbuf, dataLen);
    }
}

static void *planlog = NULL;
void PathPlan_PrintPlanStart()
{
    static uint8_t logbuff[40 * 1024] = {0};
    if (planlog != NULL)
    {
        return;
    }
    char fileName[128] = {0};
    struct tm absStamp;
    RTE_BSW_Get_AbsStamp(&absStamp);

    char pathName[64];
    Rte_BSW_Get_LogPath(pathName);
    snprintf(fileName, sizeof(fileName) - 1, "%s/%d_%02d_%02d_%02d_%02d_%02d_plan.txt",
             pathName, absStamp.tm_year, absStamp.tm_mon, absStamp.tm_mday,
             absStamp.tm_hour, absStamp.tm_min, absStamp.tm_sec);

    planlog     = NULL;
    auto logFid = fopen(fileName, "w");
    if (logFid == NULL)
    {
        return;
    }

    planlog = Buff_Init(logbuff, sizeof(logbuff), logFid);
    if (planlog == NULL)
    {
        fclose(logFid);
    }

    PlanInfoType slotPlan;
    RTE_PK_DataConvt_Get_Target_PlanInfo(&slotPlan);
    PathPlan_PrintPathInfo(slotPlan);
}

void PathPlan_PrintPlanEnd()
{
    if (planlog == NULL)
    {
        return;
    }

    struct timeval stamp;
    gettimeofday(&stamp, 0);

    char endStr[256];
    int end = strftime(endStr, sizeof(endStr) - 1, "%Y-%m-%d %H:%M:%S\n",
                       localtime(&(stamp.tv_sec)));

    Buff_Put(planlog, endStr, end);
    Buff_UnInit(planlog);
    planlog = NULL;
}

void PathPlan_PrintPlanDebug(const char *inf, const float traj[][TRAJITEM_LEN],
                             int pathNum, const PlanDataCase &PlanDataVer)
{
    if (planlog == NULL)
    {
        PathPlan_PrintPlanStart();
    }

    if (planlog == NULL)
    {
        return;
    }

    char printbuf[128] = {0};

    struct timeval stamp;
    gettimeofday(&stamp, NULL);

    int dataLen = snprintf(printbuf, sizeof(printbuf), "%ld:%ld %s\n", stamp.tv_sec,
                           stamp.tv_usec, inf);
    Buff_Put(planlog, printbuf, dataLen);

    dataLen = snprintf(
        printbuf, sizeof(printbuf), "point:%04f,%04f,%04f %04f,%04f,%04f\n",
        PlanDataVer.stpoint[0], PlanDataVer.stpoint[1], PlanDataVer.stpoint[2],
        PlanDataVer.finpoint[0], PlanDataVer.finpoint[1], PlanDataVer.finpoint[2]);
    Buff_Put(planlog, printbuf, dataLen);

    for (int i = 0; i < pathNum; i++)
    {
        int dataLen =
            snprintf(printbuf, sizeof(printbuf), "traj:%04f,%04f,%04f,%04f,%04f\n",
                     traj[i][0], traj[i][1], traj[i][2], traj[i][3], traj[i][4]);
        Buff_Put(planlog, printbuf, dataLen);
    }

    for (int i = 0; i < PlanDataVer.obj_danger_num; i++)
    {
        int dataLen =
            snprintf(printbuf, sizeof(printbuf), "obj:%04f,%04f,%04f,%04f\n",
                     PlanDataVer.obj_danger[i][0], PlanDataVer.obj_danger[i][1],
                     PlanDataVer.obj_danger[i][2], PlanDataVer.obj_danger[i][3]);
        Buff_Put(planlog, printbuf, dataLen);
    }
}
