#include "PK_ObjAvoid.h"
#include "Record_Buff.h"
#include "Rte.h"
#include "Rte_Func.h"
#include "PK_Calibration.h"
#include "Uradar_ObjAvoid.h"
#include "Uradar_AvoidState.h"
#include "MathFunc.h"

static int UradarCount[Radar_ID_MAX] = {0};
static void *objlog                  = NULL;
static FILE *objDbglog               = NULL;

uint16_t ObjAvoidRun_Step; //  闈跨妯″潡杩濊杩涚▼

static void PKObjAvoid_Init();

int ObjAvoid_RadarCount(const int radarId)
{
    if (radarId >= Radar_ID_MAX)
    {
        return 0;
    }
    return UradarCount[radarId];
}

static uint8_t PKAvoid_DistTransfer(const float pathLen, const float brakdist,
                                    float &distCfg)
{
    const float avoid_path = 0.90;
    uint8_t flag           = 0;
    if (brakdist > avoid_path)
    {
        distCfg = pathLen;
        flag    = 0;
    }
    else if (pathLen <= brakdist + 0.001)
    {
        distCfg = pathLen;
        flag    = 1;
    }
    else
    {
        distCfg = brakdist + fmodf(pathLen, 0.02);
        flag    = 3;
    }

    return flag;
}

void U_RadarDataTransfer(U_RadarType U_Radar_Dat, float U_Radar[12])
{
    float URadar_org[12];
    static float Uradar_temp1[12]    = {0};
    static float Uradar_temp2[12]    = {0};
    static float Uradar[12]          = {5100, 5100, 5100, 5100, 2550, 2550,
                                        2550, 2550, 2550, 2550, 2550, 2550};
    static float UradarGap_temp1[12] = {0};
    static float UradarGap_temp2[12] = {0};
    static float UradarGap1[12]      = {0};
    static float UradarGap2[12]      = {0};
    static float DelayClear_flag[12] = {0};
    const int UradarSideDelay_cnt    = 8;
    const int UradarForRearDelay_cnt = 25;
    const float BigGapValue          = 400;
    int UradarDelay_cnt              = 25; //

    int i;
    float Gap_ratio_temp     = 0.0f;
    float Gap_ratio          = 0.0f;
    URadar_org[Radar_ID_FSR] = (int)U_Radar_Dat.FSR;
    URadar_org[Radar_ID_FSL] = (int)U_Radar_Dat.FSL;
    URadar_org[Radar_ID_RSL] = (int)U_Radar_Dat.RSL;
    URadar_org[Radar_ID_RSR] = (int)U_Radar_Dat.RSR;
    URadar_org[Radar_ID_FOL] = (int)U_Radar_Dat.FOL;
    URadar_org[Radar_ID_FCL] = (int)U_Radar_Dat.FCL;
    URadar_org[Radar_ID_FCR] = (int)U_Radar_Dat.FCR;
    URadar_org[Radar_ID_FOR] = (int)U_Radar_Dat.FOR;
    URadar_org[Radar_ID_ROL] = (int)U_Radar_Dat.ROL;
    URadar_org[Radar_ID_RCL] = (int)U_Radar_Dat.RCL;
    URadar_org[Radar_ID_RCR] = (int)U_Radar_Dat.RCR;
    URadar_org[Radar_ID_ROR] = (int)U_Radar_Dat.ROR;
    // U_Radar = URadar_org;
    // U_Radar = zeros(1, 12);
    // Radar_clock = [4 5 2 12 6 7 8 1 3 9 10 11];
    // Radar_aiti = [8 3 9 1 2 5 6 7 10 11 12 4];
    for (i = 0; i < Radar_ID_MAX; i++)
    {
        if (i < 4)
            UradarDelay_cnt = UradarSideDelay_cnt;
        else
            UradarDelay_cnt = UradarForRearDelay_cnt;

        if (fabs(Uradar_temp1[i]) < 0.01) // Uradar_temp1 initialization
            Uradar_temp1[i] = URadar_org[i];
        else if (fabs(Uradar_temp2[i]) < 0.01) // Uradar_temp2 initialization
        {
            UradarGap_temp1[i] = URadar_org[i] - Uradar_temp1[i];
            if ((fabsf(UradarGap_temp1[i]) <= 10) && (UradarCount[i] < UradarDelay_cnt))
                UradarCount[i] = UradarCount[i] + 1;
            else
            {
                Uradar_temp2[i] = Uradar_temp1[i];
                Uradar_temp1[i] = URadar_org[i];
                UradarCount[i]  = 0;
            }
        }
        else
        {
            UradarGap_temp1[i] = URadar_org[i] - Uradar_temp1[i];
            UradarGap_temp2[i] = Uradar_temp1[i] - Uradar_temp2[i];
            if (UradarGap_temp1[i] != 0)
                Gap_ratio_temp = UradarGap_temp2[i] / UradarGap_temp1[i];

            UradarGap2[i] = Uradar_temp2[i] - Uradar[i];
            UradarGap1[i] = URadar_org[i] - Uradar_temp2[i];
            if (UradarGap1[i] != 0)
                Gap_ratio = UradarGap2[i] / UradarGap1[i];

            if (fabsf(UradarGap_temp1[i]) <= 20)
            {
                if (1 == DelayClear_flag[i] && fabsf(UradarGap_temp2[i]) <= 20)
                    UradarCount[i] = 0; // UradarDelay_cnt

                if (0 == UradarCount[i])
                    DelayClear_flag[i] = 0;

                if (UradarCount[i] < UradarDelay_cnt)
                    UradarCount[i]++;
                else
                {
                    Uradar_temp2[i]    = Uradar_temp1[i];
                    Uradar_temp1[i]    = URadar_org[i];
                    Uradar[i]          = URadar_org[i];
                    UradarCount[i]     = 0;
                    DelayClear_flag[i] = 0;
                }
            }
            else if (fabsf(UradarGap_temp1[i]) < BigGapValue)
            {
                if (fabsf(UradarGap_temp2[i]) >= BigGapValue)
                {
                    if (Gap_ratio_temp < 0)
                    {
                        if (Gap_ratio < 2.0f && Gap_ratio > 0.5f)
                            Uradar[i] = URadar_org[i];

                        Uradar_temp1[i] = URadar_org[i];
                        DelayClear_flag[i]++;
                        UradarCount[i]++;
                    }
                    else
                    {
                        if ((i < 5 && URadar_org[i] < 300 && Uradar_temp1[i] > 300) ||
                            (i > 5 && URadar_org[i] < 600 &&
                             Uradar_temp1[i] > 600)) // 假设不会跳变到危险距离内
                        {
                            Uradar_temp1[i] = URadar_org[i];
                            DelayClear_flag[i]++;
                            UradarCount[i]++;
                        }
                        else
                        {
                            Uradar[i]       = Uradar_temp1[i];
                            Uradar_temp2[i] = Uradar_temp1[i];
                            Uradar_temp1[i] = URadar_org[i];
                            UradarCount[i]  = 0;
                        }
                    }
                }
                else if (fabsf(UradarGap_temp2[i]) >= 0)
                {
                    if (Gap_ratio_temp <= 0)
                    {
                        Uradar_temp1[i] = URadar_org[i];
                        UradarCount[i]  = 0;
                    }
                    else if (Gap_ratio_temp < 5)
                    {
                        Uradar_temp2[i] = Uradar_temp1[i];
                        Uradar_temp1[i] = URadar_org[i];
                        Uradar[i]       = URadar_org[i];
                        UradarCount[i]  = 0;
                    }
                    else
                    {
                        Uradar_temp1[i] = URadar_org[i];
                        UradarCount[i]  = 0;
                    }
                }
            }
            else if (fabsf(UradarGap_temp1[i]) >= BigGapValue)
            {
                if (fabsf(UradarGap_temp2[i]) >= BigGapValue)
                {
                    if ((Gap_ratio_temp < 4.0f && Gap_ratio_temp > 0.25f) ||
                        (Gap_ratio < 4.0f && Gap_ratio > 0.25f))
                    {
                        Uradar[i]       = URadar_org[i];
                        Uradar_temp1[i] = URadar_org[i];
                        UradarCount[i]  = 0;
                    }
                    else
                    {
                        Uradar_temp1[i] = URadar_org[i];
                        DelayClear_flag[i]++;
                        UradarCount[i]++;
                    }
                }
                else if (fabsf(UradarGap_temp2[i]) >= 0)
                {
                    if (Gap_ratio_temp >= 0)
                    {
                        Uradar[i]       = Uradar_temp1[i];
                        Uradar_temp2[i] = Uradar_temp1[i];
                        Uradar_temp1[i] = URadar_org[i];
                        UradarCount[i]  = 0;
                    }
                    else
                    {
                        Uradar_temp1[i] = URadar_org[i];
                        UradarCount[i]  = 0;
                    }
                }
            }
        }
    }

    memcpy(U_Radar, Uradar, sizeof(Uradar));
}

/*********************************************************************

Function description : PK_ObjAvoid
Calibration state    : NO
Edition              : 2.0 2018/12/24 yanf

Input list:
    UU_Radar_Dat  ：Radar date type of structure
    MCom_ObjAvoid ：Command of ObjAvoid Module from StateManage Module
    Cur_VehParm   ：Current eps angle and speed of vehicle
    rx_brake      ：Brake distance from pathExecut
    Com_VehParm   ：PathExecut command eps angle and speed of vehicle
    curPos[4]     ：Vehicle state:x,y,theta,s
    NumPath=0     ：the curent number of path

Output list：
    MSt_ObjAvoid ：State of this module
    LimitSpeed   ：Target eps angle and speed of vehicle
    Brake_Dist   ：Signal of the rest distence for brake
    B_danger     ：Signal of danger state
    CDDBrkSt     ：Signal of CDD brake state:0-no brake,1-slow down,2-stop immediately

***********************************************************************/
extern float lastBrkDist;
void PK_ObjAvoid(void)
{
    static uint8_t fatalState  = 0;
    static uint8_t dangerState = 0;

    //======Input variable============
    U_RadarType U_Radar_Dat;
    RTE_PD_Get_U_Radar(&U_Radar_Dat); // Radar date type of structure
    PK_ModuleComType MCom_ObjAvoid; // Command of ObjAvoid Module from StateManage Module
    Veh_Param Com_VehParm;          // PathExecut command eps angle and speed of vehicle

    //=====Output variable==========
    Veh_Param targVehParm = {0.0f, 0.0f}; // Target eps angle and speed of vehicle

    //=====Intermediate variable for compute======
    uint8 IsSummonUp = 0; // Signal of the Summon state:0-parking,1-summon,2-summon stop
    float U_Radar[Radar_ID_MAX]; // Radar date type of array
    Veh_AvoidState curSt;

    //==================== Get Data From RTE =================================

    MCom_ObjAvoid         = RTE_PK_StateManage_Get_ModuleCom_ObjAvoid();
    Com_VehParm.Speed     = RTE_PK_PathExecute_Get_TargVelspd();
    Com_VehParm.Eps       = RTE_PK_PathExecute_Get_TargEPS_Angle();
    Com_VehParm.brakeDist = RTE_PK_PathExecute_Get_rx_brake();

    curSt.Speed      = RTE_BSW_Get_ESC_VehSpd() / 3.6;
    curSt.Eps        = RTE_BSW_Get_EPS_Angle();
    curSt.gearSt     = RTE_BSW_Get_CurrentGear();
    curSt.isSummonUp = 0;
    RTE_PK_Location_Get_CurPos(curSt.curPos);
    curSt.velMoveSt = RTE_PK_Location_Get_Vel_MoveMentSt();

    targVehParm.Eps   = Com_VehParm.Eps;
    targVehParm.Speed = Com_VehParm.Speed;

    //===================== Radar data filter ===================================
    U_RadarDataTransfer(U_Radar_Dat, U_Radar);

#define RUN_FRONT(spd)  ((spd) > 0.001f)
#define RUN_BACK(spd)   ((spd) < -0.001f)
#define TURN_LEFT(eps)  ((eps) > 1.0f)
#define TURN_RIGHT(eps) ((eps) < -1.0f)

    //====================== Compute==============================================
    if (MCom_ObjAvoid == MCOM_OFF || MCom_ObjAvoid == MCOM_DATCLC ||
        MCom_ObjAvoid == MCOM_INIT) // Don't run Obj Avoiding Condition;
    {
        // Internal variables initialization
        ObjAvoid_InitInternalVar(U_Radar);
        RTE_PK_StateManage_Set_ModuleCom_DistCtrl(MCOM_OFF);
        RTE_PK_ObjAvoid_Set_ModuleState_ObjAvoid(MSTAT_OFF);

        if (objlog != NULL)
        {
            Buff_Put(objlog, "end!\n", 5);
            Buff_UnInit(objlog);
            objlog = NULL;
        }
        if (objlog != NULL)
        {
            objlog = NULL;
        }

        if (objDbglog != NULL)
        {
            fclose(objDbglog);
            objDbglog = NULL;
        }

        fatalState  = 0;
        dangerState = 0;
    }
    else if (MCom_ObjAvoid == MCOM_ON_PARKING) // Run Obj Avoiding Condition
    {
        if (objlog == NULL)
        {
            PKObjAvoid_Init();
        }
        curSt.isSummonUp = IsSummonUp;

        uint8 dangerWatch = 0;
        int B_danger      = AutoDrive_ObjAvoid(Com_VehParm, curSt, U_Radar, &targVehParm,
                                               dangerWatch, objDbglog);
        float pathLen     = RTE_PK_PathExecute_Get_rx_brake();

        float distCfg = rte_min(fabs(targVehParm.brakeDist), fabs(pathLen));
        float cfgSpd  = rte_max(fabs(targVehParm.Speed), 0.01) * sign(Com_VehParm.Speed);
        float velSpd  = RTE_BSW_Get_ESC_VehSpd() / 3.6;

        float cfgEps   = targVehParm.Eps;
        int brakeState = 0;
        if (ObjAvoid_IsStopState(B_danger))
        {
            brakeState = 2;
        }
        else if (ObjAvoid_IsDangerState(B_danger))
        {
            brakeState = 1;
        }

        float targetDist = fabs(targVehParm.brakeDist);
        uint8_t objstate = 0;
        if ((fabs(cfgEps) > LINE_STEER_ANGLE && ObjAvoid_IsStopState(B_danger)) ||
            fatalState != 0)
        {
            auto objstate1 = (TURN_LEFT(cfgEps) && RUN_BACK(cfgSpd) &&
                              ObjAvoid_IsStopIdx(Radar_ID_FSR) &&
                              PK_ObjAvoid_Dir() == DIR_FRONTDANGER);
            auto objstate2 = (TURN_RIGHT(cfgEps) && RUN_BACK(cfgSpd) &&
                              ObjAvoid_IsStopIdx(Radar_ID_FSL) &&
                              PK_ObjAvoid_Dir() == DIR_FRONTDANGER);
            auto objstate3 =
                (TURN_LEFT(cfgEps) && RUN_FRONT(cfgSpd) &&
                 ObjAvoid_IsStopIdx(Radar_ID_RSR) && PK_ObjAvoid_Dir() == DIR_BACKDANGER);
            auto objstate4 =
                (TURN_RIGHT(cfgEps) && RUN_FRONT(cfgSpd) &&
                 ObjAvoid_IsStopIdx(Radar_ID_RSL) && PK_ObjAvoid_Dir() == DIR_BACKDANGER);
            uint8_t stop =
                objstate1 | (objstate2 << 1) | (objstate3 << 2) | (objstate4 << 3);
            objstate = ((objstate & 0xf0) | stop);

            printf(
                "---------------------- objstate 0x%x, %d %f %f %d "
                "%d,%d,%d,%d------------------\n",
                stop, fatalState, cfgEps, cfgSpd, PK_ObjAvoid_Dir(),
                PK_RadarDangerSt(Radar_ID_FSL), PK_RadarDangerSt(Radar_ID_FSR),
                PK_RadarDangerSt(Radar_ID_RSL), PK_RadarDangerSt(Radar_ID_RSR));

            if (stop != 0 || fatalState != 0)
            {
                cfgEps     = 0;
                cfgSpd     = rte_min(fabs(cfgSpd), 0.2) * sign(cfgSpd);
                brakeState = 0;
                targetDist = 0.6;
                if (stop != 0 && fatalState < 30)
                {
                    fatalState++;
                }

                if (PK_ObjAvoid_Dir() == 0 && fatalState > 0)
                {
                    fatalState--;
                };
            }
        }
        else if ((fabs(cfgEps) > LINE_STEER_ANGLE && ObjAvoid_IsDangerState(B_danger)) ||
                 dangerState != 0)
        {
            // 左转后退
            auto objstate1 =
                (cfgEps > 1.0 && cfgSpd < 0.001 && ObjAvoid_IsDangerIdx(Radar_ID_FSR) &&
                 PK_ObjAvoid_Dir() == DIR_FRONTDANGER);
            auto objstate2 =
                (cfgEps < -1.0 && cfgSpd < -0.001 && ObjAvoid_IsDangerIdx(Radar_ID_FSL) &&
                 PK_ObjAvoid_Dir() == DIR_FRONTDANGER);
            auto objstate3 =
                (cfgEps > -1.0 && cfgSpd > 0.001 && ObjAvoid_IsDangerIdx(Radar_ID_RSR) &&
                 PK_ObjAvoid_Dir() == DIR_BACKDANGER);
            auto objstate4 =
                (cfgEps < -1.0 && cfgSpd < -0.001 && ObjAvoid_IsDangerIdx(Radar_ID_RSL) &&
                 PK_ObjAvoid_Dir() == DIR_BACKDANGER); // 右  进 左盲区
            uint8_t danger =
                objstate1 | (objstate2 << 1) | (objstate3 << 2) | (objstate4 << 3);

            printf(
                "---------------------- objstate 0x%x, %d %f %f %d "
                "%d,%d,%d,%d------------------\n",
                danger, dangerState, cfgEps, cfgSpd, PK_ObjAvoid_Dir(),
                PK_RadarDangerSt(Radar_ID_FSL), PK_RadarDangerSt(Radar_ID_FSR),
                PK_RadarDangerSt(Radar_ID_RSL), PK_RadarDangerSt(Radar_ID_RSR));

            if (danger != 0 || dangerState != 0)
            {
                cfgEps     = LINE_STEER_ANGLE * 0.75f;
                cfgSpd     = rte_min(fabs(cfgSpd), 0.2) * sign(cfgSpd);
                brakeState = 0;
                targetDist = 0.6;
                if (danger != 0 && dangerState < 20)
                {
                    dangerState++;
                }

                if (PK_ObjAvoid_Dir() == 0 && dangerState > 0)
                {
                    dangerState--;
                };
            }
            objstate = ((objstate & 0x0f) | (danger << 4));
        }

        RTE_PK_ObjAvoid_Set_ObjAvoid_EPS_Angle(cfgEps);
        RTE_PK_ObjAvoid_Set_BrkSt(brakeState);
        RTE_PK_ObjAvoid_Set_ObjAvoid_Velspd(cfgSpd);
        RTE_PK_ObjAvoid_Set_BrkDist(targetDist * sign(pathLen));
        if (objlog != NULL)
        {
            struct timeval stamp;
            gettimeofday(&stamp, NULL);
            static char printbuf[1024] = {0};
            int dataLen =
                snprintf(printbuf, sizeof(printbuf),
                         "%ld:%ld step:%d danger:%d brakestate:%d,%d,%d,%d gear:%02d "
                         "dist:%06f,%06f,%06f spd:%06f,%06f,%06f eps:%06f,%06f "
                         "Danger_Watch:%d state:%d ",
                         stamp.tv_sec, stamp.tv_usec, ObjAvoidRun_Step, B_danger,
                         brakeState, objstate, fatalState, dangerState, curSt.gearSt,
                         pathLen, distCfg, lastBrkDist, velSpd, cfgSpd, Com_VehParm.Speed,
                         Com_VehParm.Eps, targVehParm.Eps, dangerWatch, curSt.velMoveSt);
            Buff_Put(objlog, printbuf, dataLen);

            dataLen = snprintf(
                printbuf, sizeof(printbuf),
                "uradars:%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d ",
                (int)U_Radar[0], (int)U_Radar[1], (int)U_Radar[2], (int)U_Radar[3],
                (int)U_Radar[4], (int)U_Radar[5], (int)U_Radar[6], (int)U_Radar[7],
                (int)U_Radar[8], (int)U_Radar[9], (int)U_Radar[10], (int)U_Radar[11]);
            Buff_Put(objlog, printbuf, dataLen);

            dataLen = snprintf(
                printbuf, sizeof(printbuf),
                "uradar:%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d ",
                (int)U_Radar_Dat.FSR, (int)U_Radar_Dat.FSL, (int)U_Radar_Dat.RSL,
                (int)U_Radar_Dat.RSR, (int)U_Radar_Dat.FOL, (int)U_Radar_Dat.FCL,
                (int)U_Radar_Dat.FCR, (int)U_Radar_Dat.FOR, (int)U_Radar_Dat.ROL,
                (int)U_Radar_Dat.RCL, (int)U_Radar_Dat.RCR, (int)U_Radar_Dat.ROR);
            Buff_Put(objlog, printbuf, dataLen);

            dataLen =
                snprintf(printbuf, sizeof(printbuf),
                         "count:%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%"
                         "02d,%02d objstate:%d\n",
                         (int)UradarCount[0], (int)UradarCount[1], (int)UradarCount[2],
                         (int)UradarCount[3], (int)UradarCount[4], (int)UradarCount[5],
                         (int)UradarCount[6], (int)UradarCount[7], (int)UradarCount[8],
                         (int)UradarCount[9], (int)UradarCount[10], (int)UradarCount[11],
                         curSt.isVehInSlot, RTE_PK_ObjAvoid_Get_ModuleState_ObjAvoid());
            Buff_Put(objlog, printbuf, dataLen);
        }

        if (RTE_PK_ObjAvoid_Get_ModuleState_ObjAvoid() == MSTAT_COMPLETE)
        {
            RTE_PK_StateManage_Set_ModuleCom_DistCtrl(MCOM_INIT);
        }
        else if (RTE_PK_ObjAvoid_Get_ModuleState_ObjAvoid() == MSTAT_NORM)
        {
            RTE_PK_StateManage_Set_ModuleCom_DistCtrl(MCOM_ON_PARKING);
        }
        RTE_PK_ObjAvoid_Set_DangerSt((uint16)B_danger);
    }
}

void PKObjAvoid_Init()
{
    char fileName[128] = {0};
    struct tm absStamp;
    RTE_BSW_Get_AbsStamp(&absStamp);

    char pathName[64];
    Rte_BSW_Get_LogPath(pathName);
    snprintf(fileName, sizeof(fileName) - 1, "%s/%d_%02d_%02d_%02d_%02d_%02d_objvoid.txt",
             pathName, absStamp.tm_year, absStamp.tm_mon, absStamp.tm_mday,
             absStamp.tm_hour, absStamp.tm_min, absStamp.tm_sec);

    objlog      = NULL;
    auto logFid = fopen(fileName, "w");
    if (logFid == NULL)
    {
        return;
    }

    static uint8_t logbuff[40 * 1024] = {0};
    objlog                            = Buff_Init(logbuff, sizeof(logbuff), logFid);
    if (objlog == NULL)
    {
        fclose(logFid);
    }

    // snprintf(fileName, sizeof(fileName) - 1,
    // "%s/%d_%02d_%02d_%02d_%02d_%02d_voiddbg.txt", pathName,
    //          absStamp.tm_year, absStamp.tm_mon, absStamp.tm_mday, absStamp.tm_hour,
    //          absStamp.tm_min, absStamp.tm_sec);

    // objDbglog = fopen(fileName, "w");
}
