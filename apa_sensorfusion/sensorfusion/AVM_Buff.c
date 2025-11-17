#include "PK_Calibration.h"
#include "PK_Utility.h"
#include "MathFunc.h"
#include "PK_SF_Subfun.h"
#include "Rte_Types.h"
#include "Module_API.h"
#include "Rte_BSW.h"
#include "SystemPara.h"
#include "AVM_Buff.h"
#include "PK_SF_A_FreeSpace.h"
#include "PK_SF.h"
#include "Record_Log.h"
#include "hobotlog/hobotlog.hpp"

#include "string.h"
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <chrono>

#define IS_VERT_PARA 1 // 1 is vert or para park,0 is not
#define MAXLINE      1024

static AVM_Buff_Info g_Left_Buff, g_Right_Buff, g_Left_Invalid_Buff, g_Right_Invalid_Buff;
static float locat_last_path = 0;
uint8 GearCh_last            = 0;
float DGearS                 = 0;

static void AVM_Valid_Buff_Replace_Old_Slot(AVM_Buff_Info *Slot_Buff, int bufIdx,
                                            Point_T NF, Point_T NR, Point_T FF,
                                            Point_T FR, int8_t has_stopper,
                                            int8_t has_lock, PK_SlotShapeType Result,
                                            float side_dis_mirr, float Odom,
                                            Point_T stopBar);
static int AVM_Valid_Buff_Add_New_Slot(AVM_Buff_Info *Slot_Buff, Point_T NF, Point_T NR,
                                       Point_T FF, Point_T FR, int8_t has_stopper,
                                       int8_t has_lock, PK_SlotShapeType Result,
                                       float side_dis_mirr, float Odom, Point_T stopBar);

void Clean_All_AVM_SlotBuff(void)
{
    memset(&g_Left_Buff, 0, sizeof(g_Left_Buff));
    memset(&g_Right_Buff, 0, sizeof(g_Right_Buff));
    memset(&g_Left_Invalid_Buff, 0, sizeof(g_Left_Buff));
    memset(&g_Right_Invalid_Buff, 0, sizeof(g_Right_Buff));

    locat_last_path = 0;
    DGearS          = 0;
    // set RTE
    RTE_PK_SF_Set_AVM_Buff_Left(&g_Left_Buff);
    RTE_PK_SF_Set_AVM_Buff_Right(&g_Right_Buff);
    RTE_PK_SF_Set_AVM_Invalid_Buff_Left(&g_Left_Invalid_Buff);
    RTE_PK_SF_Set_AVM_Invalid_Buff_Right(&g_Right_Invalid_Buff);
}

void AVM_Set_Index(PK_SlotShapeType slotshap, int index, AVM_Buff_Info *buff)
{
    buff->AVM_slot_index[index] = 0;
    if (slotshap == PK_SLOT_LEFT_PARA || slotshap == PK_SLOT_LEFT_VERT ||
        slotshap == PK_SLOT_LEFT_ANG_FORWARD || slotshap == PK_SLOT_LEFT_ANG_REVERSE)
    {
        buff->AVM_slot_index[index] = index + 100;
    }
    else if (slotshap == PK_SLOT_RIGHT_PARA || slotshap == PK_SLOT_RIGHT_VERT ||
             slotshap == PK_SLOT_RIGHT_ANG_FORWARD ||
             slotshap == PK_SLOT_RIGHT_ANG_REVERSE)
    {
        buff->AVM_slot_index[index] = index + 200;
    }
}

void Loop_AVM_Buff_Add_New_Point(Point_T near_front_pt, Point_T near_rear_pt,
                                 Point_T far_front_pt, Point_T far_rear_pt,
                                 Point_T stopbar, int8_t has_stopper, int8_t has_lock,
                                 PK_SlotShapeType slotshap, float side_dist_mirr,
                                 float Odom, AVM_Buff_Info *buff)
{
    int i;
    for (i = 0; i < AVM_BUFF_NUM_MAX - 1; i++)
    {
        memcpy(&buff->NF[i], &buff->NF[i + 1], sizeof(Point_T));
        memcpy(&buff->NR[i], &buff->NR[i + 1], sizeof(Point_T));
        memcpy(&buff->FF[i], &buff->FF[i + 1], sizeof(Point_T));
        memcpy(&buff->FR[i], &buff->FR[i + 1], sizeof(Point_T));
        memcpy(&buff->BAR[i], &buff->BAR[i + 1], sizeof(Point_T));
        buff->SlotShap[i]       = buff->SlotShap[i + 1];
        buff->Side_Mirr_Dist[i] = buff->Side_Mirr_Dist[i + 1];
        buff->Otom[i]           = buff->Otom[i + 1];
        buff->has_stopper[i]    = buff->has_stopper[i + 1];
        buff->has_lock[i]       = buff->has_lock[i + 1];
    }

    memcpy(&buff->NF[AVM_BUFF_NUM_MAX - 1], &near_front_pt, sizeof(Point_T));
    memcpy(&buff->NR[AVM_BUFF_NUM_MAX - 1], &near_rear_pt, sizeof(Point_T));
    memcpy(&buff->FF[AVM_BUFF_NUM_MAX - 1], &far_front_pt, sizeof(Point_T));
    memcpy(&buff->FR[AVM_BUFF_NUM_MAX - 1], &far_rear_pt, sizeof(Point_T));
    memcpy(&buff->BAR[AVM_BUFF_NUM_MAX - 1], &stopbar, sizeof(Point_T));
    buff->SlotShap[AVM_BUFF_NUM_MAX - 1]       = slotshap;
    buff->Side_Mirr_Dist[AVM_BUFF_NUM_MAX - 1] = side_dist_mirr;
    buff->Otom[AVM_BUFF_NUM_MAX - 1]           = Odom;
    buff->has_stopper[AVM_BUFF_NUM_MAX - 1]    = has_stopper;
    buff->has_lock[AVM_BUFF_NUM_MAX - 1]       = has_lock;
    AVM_Set_Index(slotshap, AVM_BUFF_NUM_MAX - 1, buff);
}

// Calibration of parking orientation according to the symbol Y of parking space
// coordinates
PK_SlotShapeType Check_AVM_SlotShap(PK_SlotShapeType SlotShap, VehPos_T curPos,
                                    Point_T NearFrontPt, Point_T NearRearPt)
{
    PK_SlotShapeType real_AVM_SlotResult = SlotShap;
    float ry                             = Project_PointTo1stPos_ry(curPos, NearFrontPt);
    float slot_len                       = Cal_Dis_Pt2Pt(NearFrontPt, NearRearPt);

    if (ry < 0) //  The vehicle is on the right side of the vehicle.
    {
        if (slot_len < 4.1f) //  Vertical parking slot
        {
            real_AVM_SlotResult = PK_SLOT_RIGHT_VERT;
        }
        else if (slot_len > MIN_PARK_LEN_PARALLEL)
        {
            real_AVM_SlotResult = PK_SLOT_RIGHT_PARA;
        }
    }
    else if (ry > 0) //  The vehicle is on the left side of the vehicle.
    {
        if (slot_len < 4.1f) //  Vertical parking slot
        {
            real_AVM_SlotResult = PK_SLOT_LEFT_VERT;
        }
        else if (slot_len > MIN_PARK_LEN_PARALLEL)
        {
            real_AVM_SlotResult = PK_SLOT_LEFT_PARA;
        }
    }
    return real_AVM_SlotResult;
}

int Delete_Specify_SlotBuff(int index, AVM_Buff_Info *buff)
{
    int i;
    int buffNum = buff->Num;
    if (index < 0 || index >= buffNum || buffNum == 0)
    {
        return 0;
    }
    if (index == buffNum - 1)
    {
        memset(&buff->NF[index], 0, sizeof(Point_T));
        memset(&buff->NR[index], 0, sizeof(Point_T));
        memset(&buff->FF[index], 0, sizeof(Point_T));
        memset(&buff->FR[index], 0, sizeof(Point_T));
        buff->SlotShap[index]       = PK_SLOT_NO;
        buff->Side_Mirr_Dist[index] = 0;
        buff->Otom[index]           = 0;
        buff->has_lock[index]       = 0;
        buff->has_stopper[index]    = 0;
        (buff->Num)--;
    }
    else
    {
        for (i = index; i < buffNum - 1; i++)
        {
            buff->NF[i]             = buff->NF[i + 1];
            buff->NR[i]             = buff->NR[i + 1];
            buff->FF[i]             = buff->FF[i + 1];
            buff->FR[i]             = buff->FR[i + 1];
            buff->SlotShap[i]       = buff->SlotShap[i + 1];
            buff->Side_Mirr_Dist[i] = buff->Side_Mirr_Dist[i + 1];
            buff->Otom[i]           = buff->Otom[i + 1];
            buff->has_lock[i]       = buff->has_lock[i + 1];
            buff->has_stopper[i]    = buff->has_stopper[i + 1];
        }
        (buff->Num)--;
    }
    return 1;
}

//  2017/01/20: The parking coordinate point is transformed into vehicle coordinate
//  system.
void Transfer_AVM_Locat_To_Odom(Point_T *AVM_Point, float CurPos[4])
{
    float x, y, theta;
    theta = CurPos[2];
    // The coordinate system of AVM detecting the parking lot is consistent with the
    // direction defined by the odometery.
    x = AVM_Point->x;
    y = AVM_Point->y;

    // coordinate is transformed into odometry coordinate system.
    AVM_Point->x = (x * cosf(theta)) - (y * sinf(theta)) + CurPos[0];
    AVM_Point->y = x * sinf(theta) + y * cosf(theta) + CurPos[1];
}

void AVM_Add_Odom(Point_T *NearPt, float curpos[4])
{
    NearPt->x += curpos[0];
    NearPt->y += curpos[1];
}

bool Update_Buff_Exist_Slot(AVM_Buff_Info *slotBuff, const int slotIdx,
                            AVM_SlotPointsType &slotPt)
{
    Point_T newFMid, newRMid, oldFMid, oldRMid;

    if (slotPt.shape == PK_SLOT_LEFT_VERT || slotPt.shape == PK_SLOT_RIGHT_VERT)
    {
        newFMid.x = (slotPt.NearFront[0] + slotPt.NearRear[0]) * 0.5;
        newFMid.y = (slotPt.NearFront[1] + slotPt.NearRear[1]) * 0.5;
        newRMid.x = (slotPt.FarFront[0] + slotPt.FarRear[0]) * 0.5;
        newRMid.y = (slotPt.FarFront[1] + slotPt.FarRear[1]) * 0.5;

        oldFMid.x = (slotBuff->NF[slotIdx].x + slotBuff->NR[slotIdx].x) * 0.5;
        oldFMid.y = (slotBuff->NF[slotIdx].y + slotBuff->NR[slotIdx].y) * 0.5;
        oldRMid.x = (slotBuff->FF[slotIdx].x + slotBuff->FR[slotIdx].x) * 0.5;
        oldRMid.y = (slotBuff->FF[slotIdx].y + slotBuff->FR[slotIdx].y) * 0.5;
    }
    else if (slotPt.shape == PK_SLOT_LEFT_PARA || slotPt.shape == PK_SLOT_RIGHT_PARA)
    {
        newFMid.x = (slotPt.NearFront[0] + slotPt.FarFront[0]) * 0.5;
        newFMid.y = (slotPt.NearFront[1] + slotPt.FarFront[1]) * 0.5;
        newRMid.x = (slotPt.NearRear[0] + slotPt.FarRear[0]) * 0.5;
        newRMid.y = (slotPt.NearRear[1] + slotPt.FarRear[1]) * 0.5;

        oldFMid.x = (slotBuff->NF[slotIdx].x + slotBuff->FF[slotIdx].x) * 0.5;
        oldFMid.y = (slotBuff->NF[slotIdx].y + slotBuff->FF[slotIdx].y) * 0.5;
        oldRMid.x = (slotBuff->NR[slotIdx].x + slotBuff->FR[slotIdx].x) * 0.5;
        oldRMid.y = (slotBuff->NR[slotIdx].y + slotBuff->FR[slotIdx].y) * 0.5;
    }
    else
    {
        return false;
    }

    VehPos_T oldTarget, newTarget;
    newTarget.x     = (newFMid.x + newRMid.x) * 0.5;
    newTarget.y     = (newFMid.y + newRMid.y) * 0.5;
    newTarget.theta = atan2(newFMid.y - newRMid.y, newFMid.x - newRMid.x);

    oldTarget.x     = (oldFMid.x + oldRMid.x) * 0.5;
    oldTarget.y     = (oldFMid.y + oldRMid.y) * 0.5;
    oldTarget.theta = atan2(oldFMid.y - oldRMid.y, oldFMid.x - oldRMid.x);

    // float rx = Project_PosTo1st_rx((float *)&oldTarget, (float *)&newTarget);
    float ry     = Project_PosTo1st_ry((float *)&oldTarget, (float *)&newTarget);
    float rtheta = Project_PosTo1st_rtheta((float *)&oldTarget, (float *)&newTarget);

    float factor = 0.0;
    float offset = Cal_Dis_Pt2Pt(newFMid, oldFMid);
    if (offset > 0.2 || fabs(ry) > 0.02 || fabs(rtheta) > 0.01)
    {
        slotPt.NearFront[0] =
            factor * slotBuff->NF[slotIdx].x + (1.0 - factor) * slotPt.NearFront[0];
        slotPt.NearFront[1] =
            factor * slotBuff->NF[slotIdx].y + (1.0 - factor) * slotPt.NearFront[1];
        slotPt.FarFront[0] =
            factor * slotBuff->FF[slotIdx].x + (1.0 - factor) * slotPt.FarFront[0];
        slotPt.FarFront[1] =
            factor * slotBuff->FF[slotIdx].y + (1.0 - factor) * slotPt.FarFront[1];
        slotPt.NearRear[0] =
            factor * slotBuff->NR[slotIdx].x + (1.0 - factor) * slotPt.NearRear[0];
        slotPt.NearRear[1] =
            factor * slotBuff->NR[slotIdx].y + (1.0 - factor) * slotPt.NearRear[1];
        slotPt.FarRear[0] =
            factor * slotBuff->FR[slotIdx].x + (1.0 - factor) * slotPt.FarRear[0];
        slotPt.FarRear[1] =
            factor * slotBuff->FR[slotIdx].y + (1.0 - factor) * slotPt.FarRear[1];
        // the following 2 lines add by xls@2022.8.19
        slotPt.hasStop = (slotBuff->has_stopper[slotIdx] || slotPt.hasStop);
        slotPt.hasLock = slotPt.hasLock;

        return true;
    }

    return false;
}

int Check_Buff_Exist_Slot(AVM_Buff_Info *Slot_Buff, AVM_SlotPointsType &slotPt)
{
    // same
    float newPoint[2];
    newPoint[0] = (slotPt.NearFront[0] + slotPt.NearRear[0] + slotPt.FarFront[0] +
                   slotPt.FarRear[0]) /
                  4;
    newPoint[1] = (slotPt.NearFront[1] + slotPt.NearRear[1] + slotPt.FarFront[1] +
                   slotPt.FarRear[1]) /
                  4;
    int bufIdx = -1;
    float dist = 0.0;
    for (int index = 0; index < Slot_Buff->Num; index++)
    {
        // if (Slot_Buff->SlotShap[index] != slotPt.shape) {  continue;  }

        float oldPoint[2] = {(Slot_Buff->NF[index].x + Slot_Buff->NR[index].x +
                              Slot_Buff->FF[index].x + Slot_Buff->FR[index].x) /
                                 4,
                             (Slot_Buff->NF[index].y + Slot_Buff->NR[index].y +
                              Slot_Buff->FF[index].y + Slot_Buff->FR[index].y) /
                                 4};

        dist =
            sqrtf(pow(oldPoint[1] - newPoint[1], 2) + pow(oldPoint[0] - newPoint[0], 2));

        if (dist <= MIN_PARK_LEN_VERTICAL * 0.3)
        {
            // printf("old: idx %d dist %f: %f %f %f %f %f %f %f %f lock: %d new: %f %f %f
            // %f %f %f %f %f lock: %d\n", index, dist,
            //  Slot_Buff->NR[index].x, Slot_Buff->NR[index].y, Slot_Buff->FR[index].x,
            //  Slot_Buff->FR[index].y, Slot_Buff->FF[index].x, Slot_Buff->FF[index].y,
            //  Slot_Buff->NF[index].x, Slot_Buff->NF[index].y,
            //  Slot_Buff->has_lock[index], slotPt.NearFront[0], slotPt.NearFront[1],
            //  slotPt.FarFront[0], slotPt.FarFront[1], slotPt.NearRear[0],
            //  slotPt.NearRear[1], slotPt.FarRear[0], slotPt.FarRear[1], slotPt.hasLock);

            bufIdx = index;
            break;
        }
        else if (dist <= rte_max(MIN_PARK_LEN_VERTICAL * 0.7, 1.7))
        {
            return AVM_BUFF_NUM_MAX;
        }
    }

    if (bufIdx < 0)
    {
        return -1;
    }

    if (Update_Buff_Exist_Slot(Slot_Buff, bufIdx, slotPt))
    {
        return bufIdx;
    }

    return AVM_BUFF_NUM_MAX;
}

void AvmSlotBuffer(AVM_SlotPointsType &AVM_SlotPt, int buffIdx)
{
    Point_T NearFrontPt = {0}, NearRearPt = {0}, FarFrontPt = {0}, FarRearPt = {0},
            StopBar; //  Coordinates of vehicle odometery
    uint8 DGearValid  = 1;
    float SideDisMirr = 0;
    float AveTheta    = 0;

    memcpy(&NearFrontPt, AVM_SlotPt.NearFront, sizeof(Point_T));
    memcpy(&NearRearPt, AVM_SlotPt.NearRear, sizeof(Point_T));
    memcpy(&FarFrontPt, AVM_SlotPt.FarFront, sizeof(Point_T));
    memcpy(&FarRearPt, AVM_SlotPt.FarRear, sizeof(Point_T));

    int8_t has_stopper = AVM_SlotPt.hasStop;
    int8_t has_lock    = AVM_SlotPt.hasLock;
    if (has_stopper != 0)
    {
        StopBar.x = 0.5f * (AVM_SlotPt.stopPoint[0] + AVM_SlotPt.stopPoint[2]);
        StopBar.y = 0.5f * (AVM_SlotPt.stopPoint[1] + AVM_SlotPt.stopPoint[3]);
    }

    float CurPos[4] = {0}; //  x,y,theta,s
    RTE_PK_Location_Get_CurPos(CurPos);

    PK_SlotShapeType SlotType = AVM_SlotPt.shape;

    // 确定车位的类型
    PK_SlotShapeType checkResult = SlotType; //  AVM real-time detection results, reserved
                                             //  interface, 1-left side, 2-right side;
    Output_SlotShape(SlotType, NearFrontPt, NearRearPt, FarFrontPt, FarRearPt, CurPos,
                     &AveTheta, &checkResult);
    // 计算车位到车后视镜的横向距离，NR，NF取最大值 add by xls@2022.4.30
    Distance_Cur_MaxNFNR(NearFrontPt, NearRearPt, CurPos, SlotType, &SideDisMirr);

    // printf("4 restore slot:%d type:%d lock:%d stop:%d
    // Point:%06f,%06f,%06f,%06f,%06f,%06f,%06f,%06f\n",
    //      AVM_SlotResult_Check, AVM_SlotPt.shape, AVM_SlotPt.hasLock,
    //      AVM_SlotPt.hasStop, AVM_SlotPt.NearRear[0], AVM_SlotPt.NearRear[1],
    //      AVM_SlotPt.FarRear[0], AVM_SlotPt.FarRear[1], AVM_SlotPt.FarFront[0],
    //      AVM_SlotPt.FarFront[1], AVM_SlotPt.NearFront[0], AVM_SlotPt.NearFront[1]);

    if (DGearValid) //  New parking spaces were created.
    {
        // printf("AVM: Result %d Check %d NF %f %f NR %f %f FF %f %f FR %f %f\r\n",
        //     AVM_SlotResult, AVM_SlotResult_Check,
        //     NearFrontPt.x, NearFrontPt.y, NearRearPt.x, NearRearPt.y,
        //     FarFrontPt.x, FarFrontPt.y, FarRearPt.x, FarRearPt.y);

        // 根据车位NF,NR点，车位类型和CurPos，计算出最终的车位矩形框坐标并计算车位到车后轴中心的距离
        // add by xls@2022.4.30
        if ((checkResult == PK_SLOT_LEFT_PARA || checkResult == PK_SLOT_LEFT_VERT ||
             checkResult == PK_SLOT_LEFT_ANG_REVERSE ||
             checkResult == PK_SLOT_LEFT_ANG_FORWARD))
        {
            if (checkResult != PK_SLOT_LEFT_ANG_FORWARD)
            {
                if (buffIdx >= 0)
                {
                    // printf("--------------replace left avm buff %d %d avmId:
                    // %d----------------------\n",
                    //         buffIdx, g_Left_Buff.AVM_slot_index[buffIdx],
                    //         (int)AVM_SlotPt.timestamp);
                    AVM_Valid_Buff_Replace_Old_Slot(&g_Left_Buff, buffIdx, NearFrontPt,
                                                    NearRearPt, FarFrontPt, FarRearPt,
                                                    has_stopper, has_lock, checkResult,
                                                    SideDisMirr, CurPos[3], StopBar);
                }
                else
                {
                    buffIdx = AVM_Valid_Buff_Add_New_Slot(
                        &g_Left_Buff, NearFrontPt, NearRearPt, FarFrontPt, FarRearPt,
                        has_stopper, has_lock, checkResult, SideDisMirr, CurPos[3],
                        StopBar);
                    // printf("--------------add left avm buff %d %d avmId:
                    // %d----------------------\n",
                    //        buffIdx, g_Left_Buff.AVM_slot_index[buffIdx],
                    //        (int)AVM_SlotPt.timestamp);
                }
            }
        }
        else if ((checkResult == PK_SLOT_RIGHT_PARA ||
                  checkResult == PK_SLOT_RIGHT_VERT ||
                  checkResult == PK_SLOT_RIGHT_ANG_REVERSE ||
                  checkResult == PK_SLOT_RIGHT_ANG_FORWARD))
        {
            if (checkResult != PK_SLOT_RIGHT_ANG_FORWARD)
            {
                if (buffIdx >= 0)
                {
                    // printf("--------------replace right avm buff %d %d avmId:
                    // %d----------------------\n",
                    //      buffIdx, g_Right_Buff.AVM_slot_index[buffIdx],
                    //      (int)AVM_SlotPt.timestamp);
                    AVM_Valid_Buff_Replace_Old_Slot(&g_Right_Buff, buffIdx, NearFrontPt,
                                                    NearRearPt, FarFrontPt, FarRearPt,
                                                    has_stopper, has_lock, checkResult,
                                                    SideDisMirr, CurPos[3], StopBar);
                }
                else
                {
                    buffIdx = AVM_Valid_Buff_Add_New_Slot(
                        &g_Right_Buff, NearFrontPt, NearRearPt, FarFrontPt, FarRearPt,
                        has_stopper, has_lock, checkResult, SideDisMirr, CurPos[3],
                        StopBar);
                    // printf("--------------add right avm buff %d %d avmId:
                    // %d----------------------\n",
                    //        buffIdx, g_Right_Buff.AVM_slot_index[buffIdx],
                    //        (int)AVM_SlotPt.timestamp);
                }
            }
        }
    }
    else
    {
        // 非D档的时候，Put the parking all slots in Invalidbuffer.目前不会执行。add by
        // xls@2022.4.30
        if ((checkResult == PK_SLOT_LEFT_PARA || checkResult == PK_SLOT_LEFT_VERT ||
             checkResult == PK_SLOT_LEFT_ANG_REVERSE ||
             checkResult == PK_SLOT_LEFT_ANG_FORWARD))
        {
            AVM_Invalid_Buff_Add_New_Slot(&g_Left_Invalid_Buff, NearFrontPt, NearRearPt,
                                          FarFrontPt, FarRearPt, checkResult, SideDisMirr,
                                          CurPos[3]);
        }
        else
        {
            AVM_Invalid_Buff_Add_New_Slot(&g_Right_Invalid_Buff, NearFrontPt, NearRearPt,
                                          FarFrontPt, FarRearPt, checkResult, SideDisMirr,
                                          CurPos[3]);
        }
    }
}

void AVM_Slot_Buffer_Manager()
{
    const int slot_num = 16;
    AVM_SlotPointsType avmSlotPt[slot_num];
    for (int i = 0; i < slot_num; i++)
    {
        memset(&avmSlotPt[i], 0, sizeof(AVM_SlotPointsType));
        RTE_PD_Get_AVM_SlotPoints(&avmSlotPt[i]);
    }

    for (int i = 0; i < slot_num; i++)
    {
        PK_SlotShapeType slotShape = avmSlotPt[i].shape;

        if (avmSlotPt[i].shape == PK_SLOT_NO)
        {
            continue;
        }

        if (Cal_Dis_Pt2Pt(avmSlotPt[i].FarFront, avmSlotPt[i].NearRear) < 0.1)
        {
            continue;
        }

        float nfndDist = Cal_Dis_Pt2Pt(avmSlotPt[i].NearFront, avmSlotPt[i].NearRear);
        if (nfndDist < NF_NR_DIS)
        {
            return;
        }

        int buffIdx = -1;
        if (slotShape == PK_SLOT_LEFT_PARA || slotShape == PK_SLOT_LEFT_VERT ||
            slotShape == PK_SLOT_LEFT_ANG_REVERSE)
        {
            buffIdx = Check_Buff_Exist_Slot(&g_Left_Buff, avmSlotPt[i]);
        }
        else if (slotShape == PK_SLOT_RIGHT_PARA || slotShape == PK_SLOT_RIGHT_VERT ||
                 slotShape == PK_SLOT_RIGHT_ANG_REVERSE)
        {
            buffIdx = Check_Buff_Exist_Slot(&g_Right_Buff, avmSlotPt[i]);
        }
        else
        {
            slotShape = PK_SLOT_NO;
        }

        if (buffIdx >= AVM_BUFF_NUM_MAX)
        {
            continue;
        }

        //  Cache original grid data,判断该车位是否存在，若存在则shape置为PK_SLOT_NO
        if (slotShape != PK_SLOT_NO && slotShape > 0 &&
            slotShape <= PK_SLOT_RIGHT_VERT) // 每次有更新才进行计算
        {
            AvmSlotBuffer(avmSlotPt[i], buffIdx);
        }
        else
        {
            if (slotShape > PK_SLOT_RIGHT_VERT)
            {
                LOGW << "DROPPED current AVM slot because it is not a regular slot!";
            }
        }
    }

    Check_AVMBuff_Out_Of_FreeSpace_Odometry_Range(sf_loopflag_left_S, &g_Left_Buff,
                                                  &g_Left_Invalid_Buff);
    Check_AVMBuff_Out_Of_FreeSpace_Odometry_Range(sf_loopflag_right_S, &g_Right_Buff,
                                                  &g_Right_Invalid_Buff);

    RTE_PK_SF_Set_AVM_Buff_Left(&g_Left_Buff);
    RTE_PK_SF_Set_AVM_Buff_Right(&g_Right_Buff);
}

void AVM_Slot_Buffer_Manager_Visualize(int visualfd, struct sockaddr_in servaddr,
                                       AVM_SlotPointsType *AVM_Slot_Visualize)
{
    Point_T NearFrontPt, NearRearPt, FarFrontPt, FarRearPt,
        StopBar; //  Coordinates of vehicle odometery
    AVM_SlotPointsType AVM_SlotPt = {0};

    float NF_NRDist            = 0;
    AVM_ATTRIBUTE AVM_SlotAttr = DIAGONAL_SLOT;

    Point_T NFSimuPt = {0}, NRSimuPt = {0}, FFSimuPt = {0}, FRSimuPt = {0};

    float DisCurToNF_NR = 0;
    uint8 DGearValid    = 0;

    float Side_Dis_Mirr = 0;
    float Ave_Theta     = 0;

    memset(&AVM_SlotPt, 0, sizeof(AVM_SlotPt));
    RTE_PD_Get_AVM_SlotPoints(&AVM_SlotPt); //  Get AVM parking lot data
    memcpy(AVM_Slot_Visualize, &AVM_SlotPt, sizeof(AVM_SlotPointsType));
    PK_SlotShapeType slotShape = AVM_SlotPt.shape;

    if (AVM_SlotPt.shape == 0)
    {
        return;
    }

    // printf("4 restore slot:%d type:%d lock:%d stop:%d
    // Point:%06f,%06f,%06f,%06f,%06f,%06f,%06f,%06f\n",
    //      (int)AVM_SlotPt.timestamp, AVM_SlotPt.shape, AVM_SlotPt.hasLock,
    //      AVM_SlotPt.hasStop, AVM_SlotPt.NearRear[0], AVM_SlotPt.NearRear[1],
    //      AVM_SlotPt.FarRear[0], AVM_SlotPt.FarRear[1], AVM_SlotPt.FarFront[0],
    //      AVM_SlotPt.FarFront[1], AVM_SlotPt.NearFront[0], AVM_SlotPt.NearFront[1]);

    if (Cal_Dis_Pt2Pt(AVM_SlotPt.FarFront, AVM_SlotPt.NearRear) < 0.1)
    {
        return;
    }

    DGearValid  = 1;
    int buffIdx = -1;
    if (slotShape == PK_SLOT_LEFT_PARA || slotShape == PK_SLOT_LEFT_VERT ||
        slotShape == PK_SLOT_LEFT_ANG_REVERSE)
    {
        buffIdx = Check_Buff_Exist_Slot(&g_Left_Buff, AVM_SlotPt);
    }
    else if (slotShape == PK_SLOT_RIGHT_PARA || slotShape == PK_SLOT_RIGHT_VERT ||
             slotShape == PK_SLOT_RIGHT_ANG_REVERSE)
    {
        buffIdx = Check_Buff_Exist_Slot(&g_Right_Buff, AVM_SlotPt);
    }
    else
    {
        slotShape = PK_SLOT_NO;
    }

    if (buffIdx >= AVM_BUFF_NUM_MAX)
    {
        return;
    }

    memcpy(&NearFrontPt, AVM_SlotPt.NearFront, sizeof(Point_T));
    memcpy(&NearRearPt, AVM_SlotPt.NearRear, sizeof(Point_T));
    memcpy(&FarFrontPt, AVM_SlotPt.FarFront, sizeof(Point_T));
    memcpy(&FarRearPt, AVM_SlotPt.FarRear, sizeof(Point_T));

    int8_t has_stopper = AVM_SlotPt.hasStop;
    int8_t has_lock    = AVM_SlotPt.hasLock;
    if (has_stopper != 0)
    {
        StopBar.x = 0.5f * (AVM_SlotPt.stopPoint[0] + AVM_SlotPt.stopPoint[2]);
        StopBar.y = 0.5f * (AVM_SlotPt.stopPoint[1] + AVM_SlotPt.stopPoint[3]);
    }

    NF_NRDist = Cal_Dis_Pt2Pt(NearFrontPt, NearRearPt);
    if (NF_NRDist < NF_NR_DIS)
    {
        return;
    }

    float CurPos[4] = {0}; //  x,y,theta,s
    RTE_PK_Location_Get_CurPos(CurPos);

    //  Cache original grid data,判断该车位是否存在，若存在则shape置为PK_SLOT_NO
    if (slotShape != PK_SLOT_NO && slotShape > 0 &&
        slotShape <= 4) // 每次有更新才进行计算
    {
        // 确定车位的类型
        PK_SlotShapeType newSlotShape;
        Output_SlotShape(slotShape, NearFrontPt, NearRearPt, FarFrontPt, FarRearPt,
                         CurPos, &Ave_Theta, &newSlotShape);
        // 计算车位到车后视镜的横向距离，NR，NF取最大值 add by xls@2022.4.30
        Distance_Cur_MaxNFNR(NearFrontPt, NearRearPt, CurPos, slotShape, &Side_Dis_Mirr);

        // printf("4 restore slot:%d type:%d lock:%d stop:%d
        // Point:%06f,%06f,%06f,%06f,%06f,%06f,%06f,%06f\n",
        //      newSlotShape, AVM_SlotPt.shape, AVM_SlotPt.hasLock, AVM_SlotPt.hasStop,
        //      AVM_SlotPt.NearRear[0], AVM_SlotPt.NearRear[1], AVM_SlotPt.FarRear[0],
        //      AVM_SlotPt.FarRear[1], AVM_SlotPt.FarFront[0], AVM_SlotPt.FarFront[1],
        //      AVM_SlotPt.NearFront[0], AVM_SlotPt.NearFront[1]);

        if (DGearValid) //  New parking spaces were created.
        {
            // printf("AVM: Result %d Check %d NF %f %f NR %f %f FF %f %f FR %f %f\r\n",
            //     slotShape, newSlotShape,
            //     NearFrontPt.x, NearFrontPt.y, NearRearPt.x, NearRearPt.y,
            //     FarFrontPt.x, FarFrontPt.y, FarRearPt.x, FarRearPt.y);

            // 根据车位NF,NR点，车位类型和CurPos，计算出最终的车位矩形框坐标并计算车位到车后轴中心的距离
            // add by xls@2022.4.30
            Vector_Output_Rectangle(NearFrontPt, NearRearPt, slotShape, CurPos, &NFSimuPt,
                                    &NRSimuPt, &FFSimuPt, &FRSimuPt, &DisCurToNF_NR);
            if (newSlotShape == PK_SLOT_LEFT_PARA || newSlotShape == PK_SLOT_LEFT_VERT ||
                newSlotShape == PK_SLOT_LEFT_ANG_REVERSE ||
                newSlotShape == PK_SLOT_LEFT_ANG_FORWARD)
            {
                AVM_SlotAttr = AVM_EMPTY;
                if (AVM_SlotAttr == AVM_EMPTY && newSlotShape != PK_SLOT_LEFT_ANG_FORWARD)
                {
                    if (buffIdx >= 0)
                    {
                        // printf("--------------replace left avm buff %d %d avmidx:
                        // %d----------------------\n",
                        //        buffIdx, g_Left_Buff.AVM_slot_index[buffIdx],
                        //        (int)AVM_SlotPt.timestamp);
                        AVM_Valid_Buff_Replace_Old_Slot(
                            &g_Left_Buff, buffIdx, NearFrontPt, NearRearPt, FarFrontPt,
                            FarRearPt, has_stopper, has_lock, newSlotShape, Side_Dis_Mirr,
                            CurPos[3], StopBar);
                    }
                    else
                    {
                        buffIdx = AVM_Valid_Buff_Add_New_Slot(
                            &g_Left_Buff, NearFrontPt, NearRearPt, FarFrontPt, FarRearPt,
                            has_stopper, has_lock, newSlotShape, Side_Dis_Mirr, CurPos[3],
                            StopBar);
                        // printf("--------------add left avm buff %d %d  avmidx:
                        // %d----------------------\n",
                        //        buffIdx, g_Left_Buff.AVM_slot_index[buffIdx],
                        //        (int)AVM_SlotPt.timestamp);
                    }
                }
            }
            else if (newSlotShape == PK_SLOT_RIGHT_PARA ||
                     newSlotShape == PK_SLOT_RIGHT_VERT ||
                     newSlotShape == PK_SLOT_RIGHT_ANG_REVERSE ||
                     newSlotShape == PK_SLOT_RIGHT_ANG_FORWARD)
            {
                AVM_SlotAttr = AVM_EMPTY;
                if (AVM_SlotAttr == AVM_EMPTY &&
                    newSlotShape != PK_SLOT_RIGHT_ANG_FORWARD)
                {
                    if (buffIdx >= 0)
                    {
                        // printf("--------------replace right avm buff %d %d  avmidx:
                        // %d----------------------\n",
                        //        buffIdx, g_Right_Buff.AVM_slot_index[buffIdx],
                        //        (int)AVM_SlotPt.timestamp);
                        AVM_Valid_Buff_Replace_Old_Slot(
                            &g_Right_Buff, buffIdx, NearFrontPt, NearRearPt, FarFrontPt,
                            FarRearPt, has_stopper, has_lock, newSlotShape, Side_Dis_Mirr,
                            CurPos[3], StopBar);
                    }
                    else
                    {
                        buffIdx = AVM_Valid_Buff_Add_New_Slot(
                            &g_Right_Buff, NearFrontPt, NearRearPt, FarFrontPt, FarRearPt,
                            has_stopper, has_lock, newSlotShape, Side_Dis_Mirr, CurPos[3],
                            StopBar);
                        // printf("--------------add right avm buff %d %d  avmidx:
                        // %d----------------------\n",
                        //        buffIdx, g_Right_Buff.AVM_slot_index[buffIdx],
                        //        (int)AVM_SlotPt.timestamp);
                    }
                }
            }
        }
        else
        {
            // 非D档的时候，Put the parking all slots in Invalidbuffer.目前不会执行。add
            // by xls@2022.4.30
            if (newSlotShape == PK_SLOT_LEFT_PARA || newSlotShape == PK_SLOT_LEFT_VERT ||
                newSlotShape == PK_SLOT_LEFT_ANG_REVERSE ||
                newSlotShape == PK_SLOT_LEFT_ANG_FORWARD)
            {
                AVM_Invalid_Buff_Add_New_Slot(&g_Left_Invalid_Buff, NearFrontPt,
                                              NearRearPt, FarFrontPt, FarRearPt,
                                              newSlotShape, Side_Dis_Mirr, CurPos[3]);
            }
            else
            {
                AVM_Invalid_Buff_Add_New_Slot(&g_Right_Invalid_Buff, NearFrontPt,
                                              NearRearPt, FarFrontPt, FarRearPt,
                                              newSlotShape, Side_Dis_Mirr, CurPos[3]);
            }
        }
    }
    else
    {
        if (slotShape > 4)
        {
            LOGW << "DROPPED current AVM slot because it is not a regular slot!";
        }
    }

    Check_AVMBuff_Out_Of_FreeSpace_Odometry_Range(sf_loopflag_left_S, &g_Left_Buff,
                                                  &g_Left_Invalid_Buff);
    Check_AVMBuff_Out_Of_FreeSpace_Odometry_Range(sf_loopflag_right_S, &g_Right_Buff,
                                                  &g_Right_Invalid_Buff);

    // set RTE
    RTE_PK_SF_Set_AVM_Buff_Left(&g_Left_Buff);
    RTE_PK_SF_Set_AVM_Buff_Right(&g_Right_Buff);
}

void AVM_Slot_OffClear(void)
{
    int i              = 0;
    float curpos[4]    = {0, 0, 0, 0};
    VehPos_T curVehPos = {0, 0, 0};
    Point_T NF         = {0, 0};
    float rx           = 0;

    //  get current pos
    RTE_PK_Location_Get_CurPos(curpos);
    curVehPos.x     = curpos[0];
    curVehPos.y     = curpos[1];
    curVehPos.theta = curpos[2];

    for (i = g_Left_Buff.Num - 1; i >= 0; i--)
    {
        NF.x = g_Left_Buff.NF[i].x;
        NF.y = g_Left_Buff.NF[i].y;
        rx   = Project_PointTo1stPos_rx(curVehPos, NF);
        if (rx > 0)
        {
            Delete_Specify_SlotBuff(i, &g_Left_Buff);
        }
        else
        {
            break;
        }
    }

    for (i = g_Right_Buff.Num - 1; i >= 0; i--)
    {
        NF.x = g_Right_Buff.NF[i].x;
        NF.y = g_Right_Buff.NF[i].y;
        rx   = Project_PointTo1stPos_rx(curVehPos, NF);
        if (rx > 0)
        {
            Delete_Specify_SlotBuff(i, &g_Right_Buff);
        }
        else
        {
            break;
        }
    }

    for (i = g_Left_Invalid_Buff.Num - 1; i >= 0; i--)
    {
        NF.x = g_Left_Invalid_Buff.NF[i].x;
        NF.y = g_Left_Invalid_Buff.NF[i].y;
        rx   = Project_PointTo1stPos_rx(curVehPos, NF);
        if (rx > 0)
        {
            Delete_Specify_SlotBuff(i, &g_Left_Invalid_Buff);
        }
        else
        {
            break;
        }
    }

    for (i = g_Right_Invalid_Buff.Num - 1; i >= 0; i--)
    {
        NF.x = g_Right_Invalid_Buff.NF[i].x;
        NF.y = g_Right_Invalid_Buff.NF[i].y;
        rx   = Project_PointTo1stPos_rx(curVehPos, NF);
        if (rx > 0)
        {
            Delete_Specify_SlotBuff(i, &g_Right_Invalid_Buff);
        }
        else
        {
            break;
        }
    }

    locat_last_path = 0;
    GearCh_last     = 0;
    DGearS          = 0;
}
/**********************************************************************************
Description     :Check Whether Endpoint of Line Segment In Rectangle
Owner           :ZB
Modefied Date   :2018.09.24
Parameter[In]   :Four Points of a Rectangle and Line Segment Information
Return          :
***********************************************************************************/
int PK_LineSegIn_Check(Point_T NR, Point_T NF, Point_T FR, Point_T FF, int obj_num,
                       LineSeg_T obj[])
{
    int i = 0;

    Vec2_T Vec_NR_NF = {0}, Vec_NR_FR = {0};
    VehPos_T cd0 = {0}, cd1 = {0}, cd2 = {0}, cd3 = {0};
    float value0_pt1 = 0, value1_pt1 = 0, angle0 = 0, angle1 = 0;
    float value0_pt2 = 0, value1_pt2 = 0;
    float Length_NR_NF = 0, Length_NR_FR = 0;

    if (fabsf(NR.x) < 0.5f || fabsf(NF.x) < 0.5f)
        return 1;
    if (obj_num <= 0)
        return 0;

    Vec_NR_NF.vx = NF.x - NR.x;
    Vec_NR_NF.vy = NF.y - NR.y;
    Vec_NR_FR.vx = FR.x - NR.x;
    Vec_NR_FR.vy = FR.y - NR.y;
    angle0       = atan2f(Vec_NR_NF.vy, Vec_NR_NF.vx);
    angle1       = atan2f(Vec_NR_FR.vy, Vec_NR_FR.vx);
    Length_NR_NF = Cal_Dis_Pt2Pt(NR, NF);
    Length_NR_FR = Cal_Dis_Pt2Pt(NR, FR);
    cd0.theta    = angle0;
    cd1.theta    = angle1;

    if (obj_num > 7)
    {
        obj_num = 7;
    }

    for (i = 0; i < obj_num; i++)
    {
        cd2.x = obj[SF_OBJ_NUM - 1 - i].pt1.x - NR.x;
        cd2.y = obj[SF_OBJ_NUM - 1 - i].pt1.y - NR.y;
        cd3.x = obj[SF_OBJ_NUM - 1 - i].pt2.x - NR.x;
        cd3.y = obj[SF_OBJ_NUM - 1 - i].pt2.y - NR.y;

        value0_pt1 = Project_PosTo1stPos_rx(cd0, cd2);
        value1_pt1 = Project_PosTo1stPos_rx(cd1, cd2);
        value0_pt2 = Project_PosTo1stPos_rx(cd0, cd3);
        value1_pt2 = Project_PosTo1stPos_rx(cd1, cd3);
        if (value0_pt2 < 0)
        {
            return 0;
        }
        if (0 < value0_pt1 && value0_pt1 < Length_NR_NF && 0 < value1_pt1 &&
            value1_pt1 < Length_NR_FR)
        {
            return 1;
        }
        if (0 < value0_pt2 && value0_pt2 < Length_NR_NF && 0 < value1_pt2 &&
            value1_pt2 < Length_NR_FR)
        {
            return 1;
        }
    }

    return 0;
}

/**********************************************************************************
Description     :Through NF and NR Point Output Decision Rectangle Box
Owner           :ZB
Modefied Date   :2018.11.05
Parameter[In]   :Four Points of a Rectangle and Line Segment Information
Return          :void
***********************************************************************************/
void Vector_Output_Rectangle(Point_T NF, Point_T NR, PK_SlotShapeType Result,
                             float CurPos[4], Point_T *NFSimu, Point_T *NRSimu,
                             Point_T *FFSimu, Point_T *FRSimu, float *Cur_Dis)
{
    Line_T Line_NFNR;
    Vec2_T nr_nf              = {0, 0};
    Vec2_T nr_fr              = {0, 0};
    Vec2_T nr_fr_uint         = {0, 0};
    float TransCoefficient[3] = {0};

    if (Result == PK_SLOT_LEFT_PARA)
    {
        TransCoefficient[0] = -1.0f;
        TransCoefficient[1] = 1.0f;
        TransCoefficient[2] = 2.0f;
    }
    if (Result == PK_SLOT_LEFT_VERT)
    {
        TransCoefficient[0] = -1.0f;
        TransCoefficient[1] = 1.0f;
        TransCoefficient[2] = 5.0f;
    }
    if (Result == PK_SLOT_RIGHT_PARA)
    {
        TransCoefficient[0] = 1.0f;
        TransCoefficient[1] = -1.0f;
        TransCoefficient[2] = 2.0f;
    }
    if (Result == PK_SLOT_RIGHT_VERT)
    {
        TransCoefficient[0] = 1.0f;
        TransCoefficient[1] = -1.0f;
        TransCoefficient[2] = 5.0f;
    }
    // abc为直线一般式Ax+By+C=0的系数, add by xls@2022.4.30
    Getlineabc2(NF, NR, &Line_NFNR);
    // 根据系数计算出后轴中心点点到直线的垂直距离add by xls@2022.4.30
    *Cur_Dis      = DistToPt(Line_NFNR.a, Line_NFNR.b, Line_NFNR.c, CurPos[0], CurPos[1]);
    nr_nf.vx      = NF.x - NR.x;
    nr_nf.vy      = NF.y - NR.y;
    nr_fr.vx      = TransCoefficient[0] * nr_nf.vy;
    nr_fr.vy      = TransCoefficient[1] * nr_nf.vx;
    nr_fr_uint.vx = nr_fr.vx / sqrtf((nr_fr.vx) * (nr_fr.vx) + (nr_fr.vy) * (nr_fr.vy));
    nr_fr_uint.vy = nr_fr.vy / sqrtf((nr_fr.vx) * (nr_fr.vx) + (nr_fr.vy) * (nr_fr.vy));
    FFSimu->x     = NF.x + TransCoefficient[2] * nr_fr_uint.vx;
    FFSimu->y     = NF.y + TransCoefficient[2] * nr_fr_uint.vy;
    FRSimu->x     = NR.x + TransCoefficient[2] * nr_fr_uint.vx;
    FRSimu->y     = NR.y + TransCoefficient[2] * nr_fr_uint.vy;
    NFSimu->x     = NF.x - *Cur_Dis * nr_fr_uint.vx;
    NFSimu->y     = NF.y - *Cur_Dis * nr_fr_uint.vy;
    NRSimu->x     = NR.x - *Cur_Dis * nr_fr_uint.vx;
    NRSimu->y     = NR.y - *Cur_Dis * nr_fr_uint.vy;
}

/**********************************************************************************
Description     :AVM_Buffer Add New Point.
Owner           :ZB
Modefied Date   :2018.11.05
Parameter[In]   :Four Points and SlotShape and AVM_SlotIndex
Return          :void
***********************************************************************************/
static int AVM_Valid_Buff_Add_New_Slot(AVM_Buff_Info *Slot_Buff, Point_T NF, Point_T NR,
                                       Point_T FF, Point_T FR, int8_t has_stopper,
                                       int8_t has_lock, PK_SlotShapeType Result,
                                       float side_dis_mirr, float Odom, Point_T stopBar)
{
    int num = Slot_Buff->Num;
    if (Slot_Buff->Num < AVM_BUFF_NUM_MAX)
    {
        memcpy(&Slot_Buff->NF[num], &NF, sizeof(Point_T));
        memcpy(&Slot_Buff->NR[num], &NR, sizeof(Point_T));
        memcpy(&Slot_Buff->FF[num], &FF, sizeof(Point_T));
        memcpy(&Slot_Buff->FR[num], &FR, sizeof(Point_T));
        memcpy(&Slot_Buff->BAR[num], &stopBar, sizeof(Point_T));

        Slot_Buff->SlotShap[num]       = Result;
        Slot_Buff->Side_Mirr_Dist[num] = side_dis_mirr;
        Slot_Buff->Otom[num]           = Odom;
        Slot_Buff->has_stopper[num]    = has_stopper;
        Slot_Buff->has_lock[num]       = has_lock;
        AVM_Set_Index(Result, num, Slot_Buff);
        (Slot_Buff->Num)++;
        (Slot_Buff->AccNum)++;
    }
    else
    {
        num = AVM_BUFF_NUM_MAX - 1;
        Loop_AVM_Buff_Add_New_Point(NF, NR, FF, FR, stopBar, has_stopper, has_lock,
                                    Result, side_dis_mirr, Odom, Slot_Buff);
        (Slot_Buff->AccNum)++;
    }

    // printf("add new: idx %d : %f %f %f %f %f %f %f %f lock: %d\n", num,
    //     Slot_Buff->NR[num].x, Slot_Buff->NR[num].y, Slot_Buff->FR[num].x,
    //     Slot_Buff->FR[num].y, Slot_Buff->FF[num].x, Slot_Buff->FF[num].y,
    //     Slot_Buff->NF[num].x, Slot_Buff->NF[num].y, Slot_Buff->has_lock[num]);
    return num;
}

static void AVM_Valid_Buff_Replace_Old_Slot(AVM_Buff_Info *Slot_Buff, int bufIdx,
                                            Point_T NF, Point_T NR, Point_T FF,
                                            Point_T FR, int8_t has_stopper,
                                            int8_t has_lock, PK_SlotShapeType Result,
                                            float side_dis_mirr, float Odom,
                                            Point_T stopBar)
{
    // printf("replace: idx %d old bcde: %f %f %f %f %f %f %f %f lock:%d new bcde: %f %f
    // %f %f %f %f %f %f lock:%d\n", bufIdx,
    //     Slot_Buff->NR[bufIdx].x, Slot_Buff->NR[bufIdx].y, Slot_Buff->FR[bufIdx].x,
    //     Slot_Buff->FR[bufIdx].y, Slot_Buff->FF[bufIdx].x, Slot_Buff->FF[bufIdx].y,
    //     Slot_Buff->NF[bufIdx].x, Slot_Buff->NF[bufIdx].y, Slot_Buff->has_lock[bufIdx],
    //     NR.x, NR.y, FR.x, FR.y, FF.x, FF.y, NF.x, NF.y, has_lock);

    memcpy((float *)&Slot_Buff->NF[bufIdx], (float *)&NF, sizeof(Point_T));
    memcpy((float *)&Slot_Buff->NR[bufIdx], (float *)&NR, sizeof(Point_T));
    memcpy((float *)&Slot_Buff->FF[bufIdx], (float *)&FF, sizeof(Point_T));
    memcpy((float *)&Slot_Buff->FR[bufIdx], (float *)&FR, sizeof(Point_T));
    memcpy((float *)&Slot_Buff->BAR[bufIdx], (float *)&stopBar, sizeof(Point_T));

    Slot_Buff->SlotShap[bufIdx]       = Result;
    Slot_Buff->Side_Mirr_Dist[bufIdx] = side_dis_mirr;
    Slot_Buff->Otom[bufIdx]           = Odom;
    Slot_Buff->has_stopper[bufIdx]    = has_stopper;
    Slot_Buff->has_lock[bufIdx]       = has_lock;

    AVM_Set_Index(Result, bufIdx, Slot_Buff);
}

/*******************************************************************************************
Description     :Get the max distance from curpos to NF and NR no more than 4.0m
Owner           :ZB
Modefied Date   :2018.11.26
Parameter[In]   :Points NF and NR and CurPos and SlotShape
Return          :void
*********************************************************************************************/
void Distance_Cur_MaxNFNR(Point_T NF, Point_T NR, float CurPos[4],
                          PK_SlotShapeType Result, float *Dis_Mirr_Line)
{
    float NF_RY = 0, NR_RY = 0, Max_NFNR = 0;
    VehPos_T Cur_VehVec = {0};
    Cur_VehVec.x        = CurPos[0];
    Cur_VehVec.y        = CurPos[1];
    Cur_VehVec.theta    = CurPos[2];
    NF_RY = fabsf(Project_PointTo1stPos_ry(Cur_VehVec, NF)) - REAR_VIEW_MIRROR_X / 2.0f;
    NR_RY = fabsf(Project_PointTo1stPos_ry(Cur_VehVec, NR)) - REAR_VIEW_MIRROR_X / 2.0f;
    Max_NFNR = rte_max(NF_RY, NR_RY);
    if (Max_NFNR >= DIS_CUR_NFNR_MAX)
    {
        *Dis_Mirr_Line = DIS_CUR_NFNR_MAX;
    }
    else
    {
        *Dis_Mirr_Line = Max_NFNR;
    }
    return;
}
/*******************************************************************************************
Description     :Get the slotshape through judging the angle of NearRear
Owner           :ZB
Modefied Date   :2018.11.24
Parameter[In]   :Points NF and NR and CurPos and SlotShape
Return          :void
*********************************************************************************************/
void Output_SlotShape(PK_SlotShapeType InShape, Point_T NF, Point_T NR, Point_T FF,
                      Point_T FR, float CurPos[4], float *Ave_theta,
                      PK_SlotShapeType *OutShape)
{
    float SlotTheta_NF = 0, SlotTheta_NR = 0;
    VehPos_T Cur_VehVec = {0};
    Vec2_T vec0 = {0, 0}, vec1 = {0, 0}, vec2 = {0, 0};
    float RX_NR = 0, RX_FR = 0, RY_NR = 0;
    vec0.vx          = NR.x - NF.x;
    vec0.vy          = NR.y - NF.y;
    vec1.vx          = FR.x - NR.x;
    vec1.vy          = FR.y - NR.y;
    vec2.vx          = FF.x - NF.x;
    vec2.vy          = FF.y - NF.y;
    Cur_VehVec.x     = CurPos[0];
    Cur_VehVec.y     = CurPos[1];
    Cur_VehVec.theta = CurPos[2];
    SlotTheta_NF     = Get_CrossAngle_Vec2(vec0, vec1);
    SlotTheta_NR     = Get_CrossAngle_Vec2(vec0, vec2);
    *Ave_theta       = (SlotTheta_NF + SlotTheta_NR) / 2.0f;
    RX_NR            = Project_PointTo1stPos_rx(Cur_VehVec, NR);
    RX_FR            = Project_PointTo1stPos_rx(Cur_VehVec, FR);
    RY_NR            = Project_PointTo1stPos_ry(Cur_VehVec, NR);
    if (*Ave_theta < OBLIQUE_ANGLE_MIN ||
        *Ave_theta > OBLIQUE_ANGLE_MAX) // Determination of oblique parking spaces(<75
                                        // degree or > 105 degree)
    {
#if IS_VERT_PARA
        if (RY_NR < 0 && RX_NR > RX_FR)
        {
            *OutShape = PK_SLOT_NO;
        }
        else if (RY_NR < 0 && RX_NR < RX_FR)
        {
            *OutShape = PK_SLOT_NO;
        }
        else if (RY_NR > 0 && RX_NR > RX_FR)
        {
            *OutShape = PK_SLOT_NO;
        }
        else if (RY_NR > 0 && RX_NR < RX_FR)
        {
            *OutShape = PK_SLOT_NO;
        }
#else
        if (RY_NR < 0 && RX_NR > RX_FR)
        {
            *OutShape = PK_SLOT_RIGHT_ANG_REVERSE;
        }
        else if (RY_NR < 0 && RX_NR < RX_FR)
        {
            *OutShape = PK_SLOT_RIGHT_ANG_FORWARD;
        }
        else if (RY_NR > 0 && RX_NR > RX_FR)
        {
            *OutShape = PK_SLOT_LEFT_ANG_REVERSE;
        }
        else if (RY_NR > 0 && RX_NR < RX_FR)
        {
            *OutShape = PK_SLOT_LEFT_ANG_FORWARD;
        }
#endif
    }
    else
    {
        *OutShape = InShape;
    }
}

/**********************************************************************************
Description     :Add New Slot to Invalid Buff
Owner           :ZB
Modefied Date   :2018.11.30
Parameter[In]   :Slot_Buff
Return          :void
***********************************************************************************/
int AVM_Invalid_Buff_Add_New_Slot(AVM_Buff_Info *Slot_Buff, Point_T NF, Point_T NR,
                                  Point_T FF, Point_T FR, PK_SlotShapeType Result,
                                  float side_dis_mirr, float Odom)
{
    int i, j;
    int SlotNum = 0;
    int flag    = 0;
    SlotNum     = Slot_Buff->Num;
    if (SlotNum == 0)
    {
        AVM_Buff_Add_Single_Slot(Slot_Buff, SlotNum, FF, FR, NF, NR, Odom, side_dis_mirr,
                                 Result);
        (Slot_Buff->Num)++;
        (Slot_Buff->AccNum)++;
        return 0;
    }
    else
    {
        for (i = 0; i < SlotNum; i++)
        {
            if (Odom > Slot_Buff->Otom[i])
            {
                flag++;
            }
            else
            {
                break;
            }
        }

        if (SlotNum < AVM_BUFF_NUM_MAX && SlotNum != flag)
        {
            for (j = SlotNum; j > flag; j--)
            {
                AVM_Buff_Add_Single_Slot(
                    Slot_Buff, j, Slot_Buff->FF[j - 1], Slot_Buff->FR[j - 1],
                    Slot_Buff->NF[j - 1], Slot_Buff->NR[j - 1], Slot_Buff->Otom[j - 1],
                    Slot_Buff->Side_Mirr_Dist[j - 1], Slot_Buff->SlotShap[j - 1]);
            }
            AVM_Buff_Add_Single_Slot(Slot_Buff, flag, FF, FR, NF, NR, Odom, side_dis_mirr,
                                     Result);
            (Slot_Buff->Num)++;
            (Slot_Buff->AccNum)++;
            return flag;
        }
        if (SlotNum < AVM_BUFF_NUM_MAX && SlotNum == flag)
        {
            AVM_Buff_Add_Single_Slot(Slot_Buff, SlotNum, FF, FR, NF, NR, Odom,
                                     side_dis_mirr, Result);
            (Slot_Buff->Num)++;
            (Slot_Buff->AccNum)++;
            return SlotNum;
        }
        if (SlotNum == AVM_BUFF_NUM_MAX && flag != AVM_BUFF_NUM_MAX)
        {
            for (j = 0; j < flag; j++)
            {
                AVM_Buff_Add_Single_Slot(
                    Slot_Buff, j, Slot_Buff->FF[j + 1], Slot_Buff->FR[j + 1],
                    Slot_Buff->NF[j + 1], Slot_Buff->NR[j + 1], Slot_Buff->Otom[j + 1],
                    Slot_Buff->Side_Mirr_Dist[j + 1], Slot_Buff->SlotShap[j + 1]);
            }
            AVM_Buff_Add_Single_Slot(Slot_Buff, flag, FF, FR, NF, NR, Odom, side_dis_mirr,
                                     Result);
            Slot_Buff->Num = AVM_BUFF_NUM_MAX;
            (Slot_Buff->AccNum)++;
            return flag;
        }
        if (SlotNum == AVM_BUFF_NUM_MAX && flag == AVM_BUFF_NUM_MAX)
        {
            for (j = 0; j < flag - 1; j++)
            {
                AVM_Buff_Add_Single_Slot(
                    Slot_Buff, j, Slot_Buff->FF[j + 1], Slot_Buff->FR[j + 1],
                    Slot_Buff->NF[j + 1], Slot_Buff->NR[j + 1], Slot_Buff->Otom[j + 1],
                    Slot_Buff->Side_Mirr_Dist[j + 1], Slot_Buff->SlotShap[j + 1]);
            }
            AVM_Buff_Add_Single_Slot(Slot_Buff, flag - 1, FF, FR, NF, NR, Odom,
                                     side_dis_mirr, Result);
            Slot_Buff->Num = AVM_BUFF_NUM_MAX;
            (Slot_Buff->AccNum)++;
            return flag - 1;
        }
    }
    return -1;
}
/**********************************************************************************
Description     :Add Single Slot to Buff
Owner           :ZB
Modefied Date   :2018.12.01
Parameter[In]   :Slot_Buff
Return          :void
***********************************************************************************/
void AVM_Buff_Add_Single_Slot(AVM_Buff_Info *Slot_Buff, int flag, Point_T FF, Point_T FR,
                              Point_T NF, Point_T NR, float Odom, float side_dis_mirr,
                              PK_SlotShapeType slotshap)
{
    Slot_Buff->FF[flag]             = FF;
    Slot_Buff->FR[flag]             = FR;
    Slot_Buff->NF[flag]             = NF;
    Slot_Buff->NR[flag]             = NR;
    Slot_Buff->Otom[flag]           = Odom;
    Slot_Buff->Side_Mirr_Dist[flag] = side_dis_mirr;
    Slot_Buff->SlotShap[flag]       = slotshap;
}

/**********************************************************************************
Description     :Check AVM Slot Out Of FreeSpace Range
Owner           :ZB
Modefied Date   :2018.12.05
Parameter[In]   :Odometry and S_range
Return          :void
***********************************************************************************/
void Check_AVMBuff_Out_Of_FreeSpace_Odometry_Range(float Space_Odom,
                                                   AVM_Buff_Info *AVM_Valid_Buff,
                                                   AVM_Buff_Info *AVM_Invalid_Buff)
{
    float curpos[4];
    RTE_PK_Location_Get_CurPos(curpos);
    int number = AVM_Valid_Buff->Num;
    for (int i = 0; i < number; i++)
    {
        float odmValue = curpos[3] - AVM_Valid_Buff->Otom[i];
        float centerPoint[2];
        centerPoint[0] = (AVM_Valid_Buff->FF[i].x + AVM_Valid_Buff->FR[i].x +
                          AVM_Valid_Buff->NF[i].x + AVM_Valid_Buff->NR[i].x) /
                         4;
        centerPoint[1] = (AVM_Valid_Buff->FF[i].y + AVM_Valid_Buff->FR[i].y +
                          AVM_Valid_Buff->NF[i].y + AVM_Valid_Buff->NR[i].y) /
                         4;

        float ry   = Project_PosTo1st_ry(curpos, centerPoint);
        float dir  = (Slot_Dir(AVM_Valid_Buff->SlotShap[i]) == 1) ? 1 : -1;
        float dist = Cal_Dis_Pt2Pt(centerPoint, curpos);
        if (fabs(odmValue) > 25.0 || dist > 25.0 || ry * dir < 0)
        {
            AVM_Invalid_Buff_Add_New_Slot(
                AVM_Invalid_Buff, AVM_Valid_Buff->NF[i], AVM_Valid_Buff->NR[i],
                AVM_Valid_Buff->FF[i], AVM_Valid_Buff->FR[i], AVM_Valid_Buff->SlotShap[i],
                AVM_Valid_Buff->Side_Mirr_Dist[i], AVM_Valid_Buff->Otom[i]);
            Delete_Specify_SlotBuff(i, AVM_Valid_Buff);
            number = AVM_Valid_Buff->Num;
        }
    }
}
