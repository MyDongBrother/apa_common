#include "PK_Calibration.h"
#include "PK_PathExecute.h"
#include "Uradar_ObjAvoid.h"
#include "MathFunc.h"
#include "PK_Utility.h"
#include "SystemPara.h"

static int g_danger_last      = 0;   // the radars' value when vehicles near the obstacles
static float g_LastUradar[12] = {0}; //  last Uradar data
static float g_last_collision_uradar[12] = {
    0}; //  the radars' value when vehicles stoped after obstacles' avoid
static int g_radar_collision_flag[12] = {0}; //  The avoidance is caused by which radar
static float g_Danger_brake_last      = 1.0f;
static float DangerDistPre            = 80.0f;
static uint8 g_Danger_Watch_last      = 0; // the last state of Danger_Watch
static uint8 Danger_Watch_Cnt         = 0;
static float OuterSidePerdict_dist =
    1000.0f; //  The predicted distance of outerside deceleration is 1.2m
static float SidePerdict_dist =
    400.0f; //  The predicted distance of outerside deceleration is 0.4m
static const float apa_max_speed = 0.5;

bool IsVehInSlot(const float curPos[4])
{
    Rect_T vehCornerPt;
    Relation_T dir1, dir2;

    PK_SlotShapeType slotShap = RTE_PK_SlotDetect_Get_SlotShape();
    SlotObj_T slotObj;
    RTE_PK_SlotDetect_Get_SlotObj(&slotObj);

    VehPos_T curVehPos;
    memcpy(&curVehPos, curPos, sizeof(curVehPos));
    if (slotShap == PK_SLOT_NO)
    {
        return false;
    }

    Point_T slotB, slotE;
    memcpy(&slotB, &slotObj.ptB, sizeof(slotB));
    memcpy(&slotE, &slotObj.ptE, sizeof(slotE));
    Get_Veh_CornerPt(curVehPos, &vehCornerPt, 0);

    Get_Dist_Dir_Pt2PointLine(slotB, slotE, vehCornerPt.pt_rl, NULL, &dir1);
    Get_Dist_Dir_Pt2PointLine(slotB, slotE, vehCornerPt.pt_rr, NULL, &dir2);

    bool veldir =
        (slotShap == PK_SLOT_LEFT_PARA && (dir1 == PK_ON_LEFT || dir2 == PK_ON_LEFT)) ||
        (slotShap == PK_SLOT_RIGHT_PARA &&
         (dir1 == PK_ON_RIGHT || dir2 == PK_ON_RIGHT)) ||
        (slotShap == PK_SLOT_LEFT_VERT && (dir1 == PK_ON_LEFT && dir2 == PK_ON_LEFT)) ||
        (slotShap == PK_SLOT_RIGHT_VERT && (dir1 == PK_ON_RIGHT && dir2 == PK_ON_RIGHT));

    if (veldir)
    {
        return true;
    }

    float target[3], rspoint[3];
    RTE_PK_SlotDetect_Get_TargPos(target);
    CoordinadteTransfer(target, curPos, rspoint);

    if ((fabs(rspoint[2]) < PI_RAD * 10) &&
        (dir1 != dir2 || rspoint[0] < VEHICLE_LEN * 0.8))
    {
        return true;
    }

    return false;
}

int CollisionCheck(const Veh_Param &Com_VehParm, Veh_AvoidState &curSt,
                   float Uradar[Radar_ID_MAX], float *danger_brake, uint8 *Danger_Watch,
                   void *objDbglog)
{
    const int Min_ObsLeaveTime_Cnt =
        20; // Minimum required time for obstacle avoidance release, 2s
    const float SideRadarBlindDist = 280.0f; //  Blind area of side probe is 260mm
    const float FrontRearBlindDist =
        300.0f;                            // Blind area of front and rear probe is 260mm
    const uint8_t DangerWatchStayCnt = 15; // Count of State Stay of Danger_Watch

    static int ObsLeaveTime_Cnt = 0; //  Obstacle undo continuous counting
    FILE *logFid                = (FILE *)objDbglog;

    int B_danger = 0; //  initialize
    int i;

    float Uradar_alt[12]; //
    float UradarReactionDist[12];
    float Danger_Dist[Radar_ID_MAX]   = {0};
    float Perdict_Dist[Radar_ID_MAX]  = {0};
    float dangerDistPre[Radar_ID_MAX] = {0};
    // float CollisionGap = 0.0f; //record the gap value of the radar which cause the min
    // brake_dist

    float curEps                 = curSt.Eps;
    Vel_MoveMentStType curMoveSt = curSt.velMoveSt;

    float comSpd = Com_VehParm.Speed;
    float comEps = Com_VehParm.Eps;

    *danger_brake = OuterSidePerdict_dist * 0.0015f;

    curSt.isVehInSlot = IsVehInSlot(curSt.curPos);

    memcpy(Uradar_alt, Uradar, sizeof(Uradar_alt));

    //  Calculate the danger distance and prediction distance of each probe
    for (i = 0; i < Radar_ID_MAX; i++)
    {
        int count    = ObjAvoid_RadarCount(i);
        float velSpd = RTE_BSW_Get_ESC_VehSpd() / 3.6f;
        UradarReactionDist[i] =
            rte_min(9, count) * velSpd *
            20; //  9* 20;the delayed distance caused by the radar reaction time
        dangerDistPre[i] = velSpd * 700.0f; // 0.5m
        dangerDistPre[i] = rte_max(dangerDistPre[i], 30.0f);

        if (i < 4)
        {
            Danger_Dist[i]  = SideRadarBlindDist;
            Perdict_Dist[i] = SidePerdict_dist;
        }
        else
        {
            if (i == Radar_ID_FOL || i == Radar_ID_FOR || i == Radar_ID_ROL ||
                i == Radar_ID_ROR)
            {
                Danger_Dist[i]  = rte_max(FrontRearBlindDist, SideRadarBlindDist);
                Perdict_Dist[i] = OuterSidePerdict_dist;
            }
            else
            {
                if (curSt.velMoveSt == LOCAT_STILL)
                {
                    Danger_Dist[i] = FrontRearBlindDist;
                }
                else
                {
                    Danger_Dist[i] = FrontRearBlindDist + 30.0f;
                }
                Perdict_Dist[i] = OuterSidePerdict_dist;
            }
        }

        B_danger = SetUradarStateBit(i, B_danger, CANCEL_BRAKE);
        B_danger = SetUradarStateBit(i, B_danger, CANCEL_STOP);
    }

    struct timeval stamp;
    gettimeofday(&stamp, NULL);
    //===============================   step1:  Obstacle entry judgment
    //===========================================
    if (curMoveSt == LOCAT_CONSTSPD_FOR ||
        (curMoveSt == LOCAT_STILL &&
         comSpd > 0.001f)) //==========move forward======================
    {
        for (i = Radar_ID_ROL; i <= Radar_ID_ROR; i++)
        {
            Danger_Dist[i] = 0;
        }

        if (curEps > LINE_STEER_ANGLE || (curEps > 0 && comEps > LINE_STEER_ANGLE))
        {
            // Danger_Dist[0] = 0;
            Danger_Dist[Radar_ID_RSL] = 0;
        }
        else if (curEps < -1.0 * LINE_STEER_ANGLE ||
                 (curEps < 0 && comEps < -1.0 * LINE_STEER_ANGLE))
        {
            // Danger_Dist[1] = 0;
            Danger_Dist[Radar_ID_RSR] = 0;
        }
        else
        {
            Danger_Dist[Radar_ID_RSL] = 0;
            Danger_Dist[Radar_ID_RSR] = 0;
        }

        //======== side obstacle avoidance ==========
        for (i = 0; i <= 3; i++)
        {
            if (Uradar[i] <= Danger_Dist[i] + Perdict_Dist[i] + UradarReactionDist[i])
            {
                *Danger_Watch = 3;
                if (Uradar[i] <= Danger_Dist[i])
                {
                    B_danger      = SetUradarStateBit(i, B_danger, SET_BRAKE);
                    B_danger      = SetUradarStateBit(i, B_danger, SET_STOP);
                    *danger_brake = 0.0f;
                }

                if (logFid != NULL)
                {
                    fprintf(logFid,
                            "For %ldd:%ld, side danger index = %d %d throld %f, dist is "
                            "%f %f %f\n",
                            stamp.tv_sec, stamp.tv_usec, i, B_danger, Danger_Dist[i],
                            Uradar[i], Perdict_Dist[i], UradarReactionDist[i]);
                }
            }
        }
        //========= front and rear obstacle avoidance==========
        for (i = Radar_ID_FOL; i <= Radar_ID_FOR; i++)
        {
            if (Uradar[i] < Danger_Dist[i] + Perdict_Dist[i])
            {
                float upaThreld =
                    Danger_Dist[i] + UradarReactionDist[i] + dangerDistPre[i];
                if (*danger_brake >
                    (Uradar[i] + 0.4 * Danger_Dist[i] - 0.2 * UradarReactionDist[i]) *
                        0.001f)
                {
                    *danger_brake =
                        (Uradar[i] + 0.4 * Danger_Dist[i] - 0.2 * UradarReactionDist[i]) *
                        0.001f;
                    if (fabs(curSt.Speed) < 0.05f && Uradar[i] > upaThreld + 40.0f)
                    {
                        *danger_brake = rte_max(*danger_brake, 0.25f);
                    }
                    // CollisionGap = g_Danger_brake_last - *danger_brake;
                }

                *Danger_Watch  = 1;
                int dangerType = 0;
                if (Uradar[i] < upaThreld)
                {
                    *danger_brake = 0.0f;
                    B_danger      = SetUradarStateBit(i, B_danger, SET_BRAKE);
                    dangerType    = 1;
                }

                if (Uradar[i] < FrontRearBlindDist)
                {
                    *danger_brake = 0.0f;
                    B_danger      = SetUradarStateBit(i, B_danger, SET_STOP);
                    dangerType    = 2;
                }

                if (logFid != NULL)
                {
                    fprintf(logFid,
                            "For %ld:%ld, type: %d index = %d %d throld %f, dist is %f "
                            "%f %f %f %f\n",
                            stamp.tv_sec, stamp.tv_usec, dangerType, i, B_danger,
                            Danger_Dist[i], Uradar[i], Perdict_Dist[i],
                            UradarReactionDist[i], dangerDistPre[i], *danger_brake);
                }
            }
        }
    }
    else if (curMoveSt == LOCAT_CONSTSPD_BACK ||
             (curMoveSt == LOCAT_STILL &&
              comSpd < -0.001f)) //  =======move backward=============
    {
        for (i = Radar_ID_FOL; i <= Radar_ID_FOR; i++)
        {
            Danger_Dist[i] = 0;
        }

        if (curEps > LINE_STEER_ANGLE || (curEps > 0 && comEps > LINE_STEER_ANGLE))
        {
            Danger_Dist[1] = 0;
            // Danger_Dist[3] = 0;
        }
        else if (curEps < -1.0 * LINE_STEER_ANGLE ||
                 (curEps < 0 && comEps < -1.0 * LINE_STEER_ANGLE))
        {
            Danger_Dist[0] = 0;
            // Danger_Dist[2] = 0;
        }
        else
        {
            Danger_Dist[0] = 0;
            Danger_Dist[1] = 0;
        }

        //======== side obstacle avoidance ==========
        for (i = Radar_ID_FSR; i <= Radar_ID_RSR; i++)
        {
            float dangerTheld = Danger_Dist[i] + Perdict_Dist[i] +
                                UradarReactionDist[i]; // 260 + 400mm + ~ > 660mm
            if (Uradar[i] <= dangerTheld)
            {
                *Danger_Watch = 3;
                if (Uradar[i] <= Danger_Dist[i]) // 260 mm
                {
                    B_danger      = SetUradarStateBit(i, B_danger, SET_BRAKE);
                    B_danger      = SetUradarStateBit(i, B_danger, SET_STOP);
                    *danger_brake = 0.0f;
                }

                if (logFid != NULL)
                {
                    fprintf(logFid,
                            "back %ld:%ld, upa danger index = %d %d throld %f, dist is "
                            "%f %f %f\n",
                            stamp.tv_sec, stamp.tv_usec, i, B_danger, Danger_Dist[i],
                            Uradar[i], Perdict_Dist[i], UradarReactionDist[i]);
                }
            }
        }

        //========= front and rear obstacle avoidance ==========
        for (i = Radar_ID_ROL; i <= Radar_ID_ROR; i++)
        {
            float slotDist = CalcRadarSlotsDist(i, curSt.curPos); // angle < 20deg
            float watchTheld =
                Danger_Dist[i] + Perdict_Dist[i] + UradarReactionDist[i]; // 1300mm
            float brakeTheld = Danger_Dist[i] + dangerDistPre[i] + UradarReactionDist[i];
            int nearObj      = CheckNearExistObj(i, curSt.curPos);

            const float slotdist_threld = 750.0f; // 850mm

            int dangerType  = 0;
            bool dangerPass = false;
            // 没有检测到障碍物，但是足够近，先将距离压一压
            if (Uradar[i] > watchTheld && slotDist < watchTheld && nearObj)
            {
                auto temp     = (slotDist + slotdist_threld) * 0.5f;
                *danger_brake = temp * 0.001f;
                *Danger_Watch = 2;
                dangerType    = 1;
            }
            else if (Uradar[i] < watchTheld)
            {
                *Danger_Watch = 2;
                // 大角度行驶已经越过最危险位置
                if (slotDist < slotdist_threld &&
                    (Radar_ID_ROL == i || Radar_ID_ROR == i))
                {
                    dangerType     = 5;
                    float nextDist = CalcSideNextRadarSlotsDist(i, curSt);
                    if (nextDist <= slotDist - 30.0f)
                    {
                        dangerPass = true;
                        dangerType = 6;
                    }
                }

                // 大于超声波距离加刹车距离，刹车距离等于超声波长度
                if (!dangerPass && *danger_brake > (Uradar[i] + brakeTheld) * 0.001f)
                {
                    *danger_brake =
                        (Uradar[i] + 0.4 * Danger_Dist[i] - 0.2 * UradarReactionDist[i]) *
                        0.001f;
                    // CollisionGap = g_Danger_brake_last - *danger_brake;
                    dangerType = 2;
                }

                if (fabs(curEps) > 200 && (Radar_ID_ROL == i || Radar_ID_ROR == i))
                {
                    brakeTheld = Danger_Dist[i] + UradarReactionDist[i] +
                                 rte_max(dangerDistPre[i] * 0.4, 40.0);
                }

                if (Uradar[i] < brakeTheld)
                {
                    B_danger      = SetUradarStateBit(i, B_danger, SET_BRAKE);
                    *danger_brake = 0.0f;
                    dangerType    = 3;
                }

                if (Uradar[i] < Danger_Dist[i] + UradarReactionDist[i])
                {
                    B_danger      = SetUradarStateBit(i, B_danger, SET_STOP);
                    *danger_brake = 0.0f;
                    dangerType    = 4;
                }
            }

            // if (dangerType != 0)
            {
                if (logFid != NULL)
                {
                    fprintf(logFid,
                            "back %ld:%ld dangerType:%d idx:%d,%d,%d,%d "
                            "dist:%04f,%04f,%04f,%04f brake:%04f flast:%04f,%04f\n",
                            stamp.tv_sec, stamp.tv_usec, dangerType, i, B_danger, nearObj,
                            dangerPass, Danger_Dist[i], Perdict_Dist[i],
                            UradarReactionDist[i], dangerDistPre[i], *danger_brake,
                            slotDist, Uradar[i]);
                }
            }
        }
    }

    //==============================    step3: Condition to release the danger
    //===================================//
    int collision_off_flag =
        1; //  if this value is 1, it means the collision before can be cleared
    if (B_danger != 0)
    {
        // result print and clear the release count
        ObsLeaveTime_Cnt = 0;
    }
    else if (g_danger_last != 0)
    {
        //  danger release condition:no new collision detect,last Collision can be
        //  released and vehicle stays still
        for (i = Radar_ID_FSR; i <= Radar_ID_RSR; i++)
        {
            B_danger = SetUradarStateBit(i, B_danger, CANCEL_STOP);
            if (Danger_Dist[i] > 0 && Uradar[i] < Danger_Dist[i] + 50)
            {
                collision_off_flag =
                    0; // collision_off_flag==0 means the Danger need to be kept
            }
            else
            {
                B_danger = SetUradarStateBit(i, B_danger, CANCEL_BRAKE);
            }
        }

        for (i = Radar_ID_FOL; i <= Radar_ID_ROR; i++)
        {
            B_danger = SetUradarStateBit(i, B_danger, CANCEL_STOP);
            if (Danger_Dist[i] > 0 && Uradar[i] < Danger_Dist[i] + 60)
            {
                collision_off_flag = 0;
            }
            else
            {
                B_danger = SetUradarStateBit(i, B_danger, CANCEL_BRAKE);
            }
        }

        if (collision_off_flag == 1 &&
            ++ObsLeaveTime_Cnt >= Min_ObsLeaveTime_Cnt) // release collision
        {
            B_danger            = 0;
            ObsLeaveTime_Cnt    = 0;
            g_Danger_Watch_last = 0;
            g_Danger_brake_last = 1.0f;
        }
        else
        {
            // if not satisfied the danger release condition ,maintain the last result
            B_danger      = g_danger_last;
            *Danger_Watch = g_Danger_Watch_last;
            *danger_brake = g_Danger_brake_last;
        }
    }
    else // no collision danger
    {
        B_danger = 0;
    }

    if (*Danger_Watch > 0)
    {
        if (g_Danger_Watch_last == 0)
        {
            // the first time Watch Danger : record the g_Danger_Watch_last and
            // g_Danger_brake_last;turn up the perdict distance
            if (++Danger_Watch_Cnt > DangerWatchStayCnt)
            {
                g_Danger_Watch_last = *Danger_Watch;
                g_Danger_brake_last = *danger_brake;
                B_danger            = g_danger_last;
                SidePerdict_dist    = 1000.0f;
                Danger_Watch_Cnt    = 0;
            }
        }
        else
        {
            // if (g_Danger_Watch_last == *Danger_Watch)
            if (g_Danger_Watch_last == *Danger_Watch &&
                *danger_brake >
                    g_Danger_brake_last) // ensure the Danger_brake keep diminishing
            {
                *danger_brake = g_Danger_brake_last;
            }
            else
            {
                g_Danger_brake_last = *danger_brake;
            }
        }
    }
    else
    {
        g_Danger_brake_last = 1.0f;
        SidePerdict_dist    = 800.0f;
    }

    if (logFid != NULL)
    {
        fprintf(logFid,
                "CollisionCheck %ld:%ld, watch:%d,%d,%d,%d brake:%f,%f dist:%f,%f\n",
                stamp.tv_sec, stamp.tv_usec, *Danger_Watch, g_Danger_Watch_last,
                Danger_Watch_Cnt, ObsLeaveTime_Cnt, *danger_brake, g_Danger_brake_last,
                OuterSidePerdict_dist, SidePerdict_dist);
    }

    return B_danger;
}

//  Internal variables initialization
void ObjAvoid_InitInternalVar(float Uradar[Radar_ID_MAX])
{
    memset(g_last_collision_uradar, 0, sizeof(g_last_collision_uradar));
    memset(g_radar_collision_flag, 0, sizeof(g_radar_collision_flag));
    memcpy(g_LastUradar, Uradar, sizeof(g_LastUradar));
    g_danger_last       = 0; // the radars' value when vehicles near the obstacles
    g_Danger_brake_last = 1.0f;
    g_Danger_Watch_last = 0; // the last state of Danger_Watch
    Danger_Watch_Cnt    = 0;
    SidePerdict_dist =
        400.0f; //  The predicted distance of outerside deceleration is 0.4m
}

int Uradar_ObjAvoid(float comSpd, const Veh_AvoidState &curSt, uint8 *Danger_Watch)
{
    //  condition to release the danger immediately
    if ((g_danger_last == 1 && comSpd < 0 && curSt.velMoveSt == LOCAT_STILL) ||
        (g_danger_last == 2 && comSpd > 0 && curSt.velMoveSt == LOCAT_STILL))
    {
        g_danger_last = 0;
        *Danger_Watch = 0;

        //*danger_brake = 1.0f;
        // memset(g_radar_collision_flag,0,sizeof(g_radar_collision_flag));
        // memset(g_last_collision_uradar,0,sizeof(g_last_collision_uradar));
    }

    return g_danger_last;
}

// calculate the target speed
float Brake_Cmp(float cur_Speed, float BrakeDist, float ComSpeed, uint8 Dist_Type,
                uint8 *CDDSt)
{
    float BrakeSpd, Dist_Emg, Spd_Ign, Dist_pre, spd_ampl, brake_bg;
    float ACC_EMG; // accleration for emergency brake
    float ACC_ORD; // accleration for ordinary brake
    Spd_Ign  = 0.25f;
    ACC_ORD  = 0.3f;
    ACC_EMG  = 1.5f;
    spd_ampl = 1.3f; // speed ammlification factor,used to calculate a speed when need to
                     // use the CDD
    brake_bg = 0.5f; // the largest brake distance begain to consider using the CDD
    Dist_pre = 0.02f;
    if (0 == Dist_Type)
    {
        Dist_pre = 0.02f;
    }
    else if (1 == Dist_Type)
    {
        Dist_pre = 0.15f;
        Spd_Ign  = 0.2f;
        ACC_ORD  = 0.10; // 0.15f;
        ACC_EMG  = 1.0f;
    }
    else if (2 == Dist_Type)
    {
        spd_ampl = 1.2f;
        Dist_pre = DangerDistPre / 1000.0f;
        Spd_Ign  = 0.2f;
        ACC_ORD  = 0.10; // 0.15f;
        ACC_EMG  = 1.0f;
    }

    Dist_Emg = pow2(cur_Speed) / 2.0f / ACC_EMG;
    if (BrakeDist > rte_max(Dist_Emg, Dist_pre))
    {
        BrakeSpd = rte_min(ComSpeed, sqrtf(2.0f * BrakeDist * ACC_ORD));
    }
    else
    {
        BrakeSpd = 0.0f;
        if (BrakeDist < (rte_min(0.05f, Dist_Emg - 0.07f)) && cur_Speed > Spd_Ign &&
            Dist_Type > 1)
        {
            *CDDSt = 2;
        }
    }

    if (0 == Dist_Type && BrakeDist <= 0.5f)
    {
        *CDDSt = rte_max(*CDDSt, 1);
    }
    else
    {
        if ((cur_Speed > spd_ampl * rte_min(ComSpeed, rte_max(BrakeSpd, Spd_Ign)) &&
             BrakeDist <= brake_bg) ||
            BrakeDist < 0.15f) // //temp test close middle brake 2018-11-02
        {
            *CDDSt = rte_max(*CDDSt, 1);
        }
    }
    return BrakeSpd;
}

/*********************************************************************
    Function description : AutoDrive_ObjAvoid-
    Calibration state    : NO
    Edition              : 1.0 2017/02/17 chensj  :

    Input list:
                        curPos[4]:   x,y,theta,s
                        cur_time?¨oo   s
                        Uradar[12]?¨oo mm
                        cur_EPS_angle : current eps angle
                        curSpd?¨oocurent Speed
                        comSpd?¨oocommand speed form pathexcutive module
                        Targ_EPS_angle?¨oocommand Eps form pathexcutive module

    *********************************************************************
   David2 probe number list ?¨oo
        5 6 7 8
        -------
    2  @|     |@  1
        |     |
        |     |
    3  @|     |@  4
        -------
      9 10 11 12
code stored number list?¨oo
      4 5 6 7
      -------
  1  @|     |@  0
      |     |
      |     |
  2  @|     |@  3
      -------
      8 9 10 11

******************************************************************************/
float lastBrkDist = 0.0;
int AutoDrive_ObjAvoid(const Veh_Param &Com_VehParm, Veh_AvoidState &curSt,
                       float Uradar[Radar_ID_MAX], Veh_Param *targVehParm,
                       uint8 &dangerWatch, void *objDbglog)
{
    //  probe installation position compensation
    const float radar_offset_center = 10;
    Uradar[Radar_ID_FCL]            = Uradar[Radar_ID_FCL] - radar_offset_center; // FCL
    Uradar[Radar_ID_FCR]            = Uradar[Radar_ID_FCR] - radar_offset_center; // FCR
    Uradar[Radar_ID_RCL]            = Uradar[Radar_ID_RCL] - radar_offset_center; // RCL
    Uradar[Radar_ID_RCR]            = Uradar[Radar_ID_RCR] - radar_offset_center; // RCR

    float rx_brake_abs = fabsf(Com_VehParm.brakeDist);
    float Danger_brake = rx_brake_abs;

    g_danger_last = CollisionCheck(Com_VehParm, curSt, Uradar, &Danger_brake,
                                   &dangerWatch, objDbglog);

    //*******************   control for the target speed
    //*****************************************//

    // when rx_brake keep in a big value,while the current speed need to be changed in a
    // contrary direction fastly if ((velMoveSt == LOCAT_CONSTSPD_BACK &&
    // Com_VehParm.Speed > 0.0f) || (velMoveSt == LOCAT_CONSTSPD_FOR && Com_VehParm.Speed
    // < 0.0f))
    //{
    //   rx_brake_abs = rte_min(0.3f, rx_brake_abs);
    // }
    // 2018-11-12-yf:closed for this part has been done in the pathexcutive module

    // float rx_Speed = Brake_Cmp(fabsf(curSt.Speed), rx_brake_abs,
    // fabsf(Com_VehParm.Speed), 0, &rx_CDDBrkSt);// Brake_type=0;

    float Brk_Dist = 0.0f;
    if (0 == dangerWatch || rx_brake_abs < Danger_brake)
    {
        Brk_Dist     = rx_brake_abs;
        float velSpd = RTE_BSW_Get_ESC_VehSpd() / 3.6;
        if (dangerWatch == 0 && velSpd < 0.4 && fabs(Brk_Dist) > 1.50f)
        {
            Brk_Dist = Brk_Dist + 0.5;
        }
        else if (Brk_Dist * lastBrkDist > ZERO_FLOAT)
        {
            Brk_Dist = 0.5 * Brk_Dist + 0.5 * lastBrkDist;
        }
    }
    else
    {
        Brk_Dist = Danger_brake;
        if (Brk_Dist * lastBrkDist > ZERO_FLOAT)
        {
            Brk_Dist = 0.5 * Danger_brake + 0.5 * lastBrkDist;
        }
    }
    lastBrkDist = Brk_Dist;

    //==================output====================
    if (dangerWatch != 0)
    {
        targVehParm->Speed =
            rte_min(fabs(Com_VehParm.Speed), 1.5) * sign(Com_VehParm.Speed);
    }
    else if (curSt.isVehInSlot != 0)
    {
        targVehParm->Speed =
            rte_min(fabs(Com_VehParm.Speed), 2.0) * sign(Com_VehParm.Speed);
    }

    targVehParm->brakeDist = lastBrkDist;
    return g_danger_last;
}
