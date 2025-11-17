#include "Ethenet_Vision.h"
#include "Ethenet_AvmSlot.h"
#include "Ethenet_Object.h"
#include "PK_Calibration_Config.h"
#include "Rte.h"
#include "CanIf.h"

#ifndef SIMULATE
#ifdef VISION_V3
#include "apa_msgs/objects.pb.h"
#include "apa_msgs/localization.pb.h"
#include "apa_msgs/chassis.pb.h"
#include "raincom_com/init.hpp"
#include "raincom_com/node.hpp"

static FILE *avmlog = NULL;

// 找到四边形中心点
static void RectCenter(const Point_T point[4], Point_T &center)
{
    center.x = 0.0f;
    center.y = 0.0f;
    for (uint8_t i = 0; i < 4; i++)
    {
        center.x = center.x + point[i].x;
        center.y = center.y + point[i].y;
    }

    center.x = center.x * 0.25f;
    center.y = center.y * 0.25f;
}

// 转换为限位杆格式(线段)
static void ConvertToStopBar(const Point_T point[4], float line[4])
{
    Point_T center;
    RectCenter(point, center);

    float len1 = Cal_Dis_Pt2Pt(point[0], point[1]);
    float len2 = Cal_Dis_Pt2Pt(point[0], point[3]);
    Vec2_T vec;
    if (len1 > len2)
    {
        vec.vx = point[0].x - point[1].x;
        vec.vy = point[0].y - point[1].y;
    }
    else
    {
        vec.vx = point[0].x - point[3].x;
        vec.vy = point[0].y - point[3].y;
    }

    Normalize_Vec2(&vec);

    line[0] = center.x + vec.vx * 0.6;
    line[1] = center.y + vec.vy * 0.6;
    line[2] = center.x - vec.vx * 0.6;
    line[3] = center.y - vec.vy * 0.6;
}

// 转换为上报的矩形点格式
static void ConvertToRect(const AVM_SlotPointsType &slot, Point_T slotPoint[4])
{
    memcpy((void *)&slotPoint[0], slot.NearRear, sizeof(Point_T));
    memcpy((void *)&slotPoint[1], slot.FarRear, sizeof(Point_T));
    memcpy((void *)&slotPoint[2], slot.FarFront, sizeof(Point_T));
    memcpy((void *)&slotPoint[3], slot.NearFront, sizeof(Point_T));
}

// 转换为内部车位格式
static void ConvertToSlot(AVM_SlotPointsType &slot, const Point_T rect[4])
{
    memcpy(slot.NearRear, (void *)&rect[0], sizeof(Point_T));
    memcpy(slot.FarRear, (void *)&rect[1], sizeof(Point_T));
    memcpy(slot.FarFront, (void *)&rect[2], sizeof(Point_T));
    memcpy(slot.NearFront, (void *)&rect[3], sizeof(Point_T));
}

// 转换为障碍物格式
static void ConvertToObj(AVM_ObjPointsType &obj, const Point_T rect[4])
{
    Point_T center;
    RectCenter(rect, center);
    obj.mid[0]        = center.x;
    obj.mid[1]        = center.y;
    obj.obstacleTheta = 0;

    obj.beLen  = Cal_Dis_Pt2Pt(rect[0], rect[3]);
    obj.pt1[0] = rect[0].x;
    obj.pt1[1] = rect[0].y;
    obj.pt2[0] = rect[3].x;
    obj.pt2[1] = rect[3].y;
}

// 判断点是否在四边形内部
bool PointInsideQuad(const Point_T &point, const Point_T quad[4])
{
    int n       = 4;
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++)
    {
        const Point_T &p1 = quad[i];
        const Point_T &p2 = quad[j];
        if (((p1.y > point.y) != (p2.y > point.y)) &&
            (point.x < (p2.x - p1.x) * (point.y - p1.y) / (p2.y - p1.y) + p1.x))
        {
            inside = !inside;
        }
    }
    return inside;
}

// 判断四边形A是否包含四边形B
bool IsQuadContained(const Point_T quadA[4], const Point_T quadB[4])
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (!PointInsideQuad(quadB[i], quadA))
        {
            return false;
        }
    }
    return true;
}

// 根据障碍物查找对应的车位
static int FindAvmSlot(AVM_SlotPointsType *existSlots, const int slotNum,
                       const Point_T inputPoint[4])
{
    for (int i = 0; i < slotNum; i++)
    {
        Point_T center1, center2, slotPoint[4];
        ConvertToRect(existSlots[i], slotPoint);
        RectCenter(slotPoint, center1);
        RectCenter(inputPoint, center2);
        if (IsQuadContained(slotPoint, inputPoint))
        {
            return i;
        }
    }
    return -1;
}

/**
 * @brief 判断车辆侧方有效车位范围
 *
 * 用于提前释放车位，在有效范围内的车位数据会用于更新车位坐标
 *
 * @param
 * @return bool true - 在范围内 false - 不在范围内，舍弃
 */
static bool InSideEreaV3()
{
    float target[3], curpos[4], center[2];
    Avm_Pot_T avmPoint;

    RTE_PK_Location_Get_CurPos(curpos);
    RTE_PK_SlotDetect_Get_TargPos(target);
    RTE_PK_DataConvt_Get_TargAVM_SlotInfo(&avmPoint);

    center[0] = 0.5f * (avmPoint.near_front.x + avmPoint.near_rear.x);
    center[1] = 0.5f * (avmPoint.near_front.y + avmPoint.near_rear.y);

    Vec2_T beVect, vehVect;
    beVect.vx = avmPoint.near_front.x - avmPoint.near_rear.x;
    beVect.vy = avmPoint.near_front.y - avmPoint.near_rear.y;
    Normalize_Vec2(&beVect);

    vehVect.vx = cos(curpos[2]);
    vehVect.vy = sin(curpos[2]);

    float dotv = Get_Dot_Vec2(beVect, vehVect);
    float rx   = Project_PosTo1st_rx(curpos, center);
    return (rx < REAR_VIEW_MIRROR_X + 1.2 && rx > REAR_VIEW_MIRROR_X - 1.0f &&
            fabs(dotv) > cosf(20 * PI_RAD));
}

static void RecordAvmLog(const AVM_SlotPointsType *pSlot, int slotIndex, int flag)
{
    if (avmlog != NULL)
    {
        struct timeval stamp;
        gettimeofday(&stamp, NULL);
        fprintf(avmlog,
                "%06ld:%06ld %d %d type:%d stop:%06f,%06f,%06f,%06f "
                "point:%06f,%06f,%06f,%06f,%06f,%06f,%06f,%06f\n",
                stamp.tv_sec, stamp.tv_usec, slotIndex, flag, pSlot[0].shape,
                pSlot[0].stopPoint[0], pSlot[0].stopPoint[1], pSlot[0].stopPoint[2],
                pSlot[0].stopPoint[3], pSlot[0].NearRear[0], pSlot[0].NearRear[1],
                pSlot[0].FarRear[0], pSlot[0].FarRear[1], pSlot[0].FarFront[0],
                pSlot[0].FarFront[1], pSlot[0].NearFront[0], pSlot[0].NearFront[1]);
    }
}

enum
{
    UNKNOWN_CLF    = 0,  // ?′?aàà±e
    LEFT_VER_SLOT  = 1,  // ×ó′1?±3μ??
    RIGHT_VER_SLOT = 2,  // óò′1?±3μ??
    LEFT_PAR_SLOT  = 3,  // ×ó??DD3μ??
    RIGHT_PAR_SLOT = 4,  // óò??DD3μ??
    SLB            = 5,  // ·?ì?ê??T????
    CONE           = 6,  // ×?D?í°
    PL_CLOSED      = 7,  // μ???￡¨1?±?×′ì?￡?
    PL_OPENED      = 8,  // μ???￡¨?a??×′ì?￡?
    OPLB_R         = 9,  // ò?ì?ê??T????(?e?o°?)
    OPLB_I         = 10, // ò?ì?ê??T????￡¨ìú1ü°?￡?
    HANDICAP       = 11, // 2D?2è?3μ??±ê??
    PVC            = 12, // PVC1ü
    CARTON         = 13, // ????
    CHAIR          = 14, // μê×ó
    PEDSTRAIN      = 15, // DDè?
    BUCKET         = 16, // ??í°
    E_BIKE         = 17, // μ???3μ
    STOP_SIGN      = 18, // ??í￡??
    CHARGING_STOP  = 19, // 3?μ?×?
    DECEL_STRIP    = 20, // ???ù′?
    BUMPER         = 21, // ·à×2í°
    FENCE          = 22, // ?¤à?
    WHEEL_CHAIR    = 23, // ??ò?
    FIRE_KIT       = 24, // ??·à??
    BIKE           = 25, // 自行车
    SHOPPING_CART  = 26, // 购物车
    WARNING_POST   = 27, // 警示柱
    BOLLARDS       = 28, // 石墩
};

static int CheckObstacleType(int objtype)
{
    int obstacleType = Obstacle_Type_No;

    switch (objtype)
    {
        case LEFT_VER_SLOT:
            obstacleType = PK_SLOT_LEFT_VERT;
            break;
        case RIGHT_VER_SLOT:
            obstacleType = PK_SLOT_RIGHT_VERT;
            break;
        case LEFT_PAR_SLOT:
            obstacleType = PK_SLOT_LEFT_PARA;
            break;
        case RIGHT_PAR_SLOT:
            obstacleType = PK_SLOT_RIGHT_PARA;
            break;
        case SLB:
            obstacleType = Obstacle_Type_StopFlag;
            break;
        case CONE:
            obstacleType = Obstacle_Type_Traffic_Cone;
            break;
        case PL_CLOSED:
            obstacleType = Obstacle_Type_Ground_UnLock;
            break;
        case PL_OPENED:
            obstacleType = Obstacle_Type_Ground_Lock;
            break;
        case OPLB_R:
            obstacleType = Obstacle_Type_StopFlag;
            break;
        case OPLB_I:
            obstacleType = Obstacle_Type_StopFlag;
            break;
        case PEDSTRAIN:
            obstacleType = Obstacle_Type_PEDSTRAIN;
            break;
        case WARNING_POST:
            obstacleType = Obstacle_Type_Warning_Post;
            break;
        default:
            obstacleType = Obstacle_Type_No;
            break;
    }

    return obstacleType;
}

static bool SetAvmSlotPointB(const AVM_SlotPointsType *pSlot)
{
    Avm_Pot_T avmPot;
    RTE_PK_DataConvt_Get_TargAVM_SlotInfo(&avmPot);

    Point_T target, rect[4], center;
    memcpy((void *)&rect[0], (void *)&avmPot.near_rear, sizeof(Point_T));
    memcpy((void *)&rect[1], (void *)&avmPot.far_rear, sizeof(Point_T));
    memcpy((void *)&rect[2], (void *)&avmPot.far_front, sizeof(Point_T));
    memcpy((void *)&rect[3], (void *)&avmPot.near_front, sizeof(Point_T));
    RectCenter(rect, target);

    ConvertToRect(pSlot[0], rect);
    RectCenter(rect, center);

    if (Cal_Dis_Pt2Pt(center, target) > VEHICLE_WID / 3)
    {
        return false;
    }

    if (RTE_PK_DataConvt_Get_IsAVM_Slot() == 3)
    {
        RTE_PK_DataConvt_Set_IsAVM_Slot(1);
    }

    memcpy((void *)&avmPot.near_rear, (void *)&pSlot[0].NearRear, sizeof(Point_T));   // B
    memcpy((void *)&avmPot.far_rear, (void *)&pSlot[0].FarRear, sizeof(Point_T));     // C
    memcpy((void *)&avmPot.far_front, (void *)&pSlot[0].FarFront, sizeof(Point_T));   // D
    memcpy((void *)&avmPot.near_front, (void *)&pSlot[0].NearFront, sizeof(Point_T)); // E

    avmPot.slot_index = avmPot.slot_index + 1;
    RTE_PD_Set_AVM_SlotInfo_B(&avmPot);

    if (pSlot[0].hasStop == 1)
    {
        Avm_Obj_T avmObj;
        avmObj.ls.pt1.x      = pSlot[0].stopPoint[0];
        avmObj.ls.pt1.y      = pSlot[0].stopPoint[1];
        avmObj.ls.pt2.x      = pSlot[0].stopPoint[2];
        avmObj.ls.pt2.y      = pSlot[0].stopPoint[3];
        avmObj.slot_index    = avmPot.slot_index;
        avmObj.SlotObsNum    = 1;
        avmObj.AlignIndex[0] = 1;
        RTE_PD_Set_AVM_SlotObs_B(&avmObj);
        RecordAvmLog(pSlot, avmObj.slot_index, 1);
    }

    return true;
}

static void shortenSegment(float *p1, float *p2, float length)
{

    float angle = atan2f(p2[1] - p1[1], p2[0] - p1[0]);
    // è????—??-??1
    float midPoint[2] = {(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2};

    // è????—?–°?o???μ?????ˉ??1??????
    p1[0] = midPoint[0] - length * cosf(angle);
    p1[1] = midPoint[1] - length * sinf(angle);
    p2[0] = midPoint[0] + length * cosf(angle);
    p2[1] = midPoint[1] + length * sinf(angle);
}

static void UpdateAvmObstacle(AVM_ObjPointsType *avmObj, int objNum)
{
    AvmObstacle_T avmObstacle;
    memset(&avmObstacle, 0, sizeof(avmObstacle));
    int count = 0;
    for (int i = 0; i < objNum; i++)
    {
        if (avmObj[i].obstacleType == Obstacle_Type_No)
        {
            return;
        }
        if (count >= SF_OBJ_NUM)
        {
            break;
        }

        avmObstacle.central_point_valid[count] = 1;
        avmObstacle.central_point[count].x     = avmObj[i].mid[0];
        avmObstacle.central_point[count].y     = avmObj[i].mid[1];
        avmObstacle.angle_valid[count]         = 1;
        avmObstacle.angle[count]               = avmObj[i].obstacleTheta;
        avmObstacle.length[count]              = avmObj[i].beLen;
        avmObstacle.width[count]               = 0;

        shortenSegment(avmObj[i].pt1, avmObj[i].pt2, 0.10f);
        avmObstacle.obj_valid[count] = 1;
        avmObstacle.obj[count].pt1.x = avmObj[i].pt1[0];
        avmObstacle.obj[count].pt1.y = avmObj[i].pt1[1];
        avmObstacle.obj[count].pt2.x = avmObj[i].pt2[0];
        avmObstacle.obj[count].pt2.y = avmObj[i].pt2[1];
        avmObstacle.type[count]      = (int)(avmObj[i].obstacleType);
        count++;
    }

    avmObstacle.num = count;
    RTE_PK_SensorFusion_Set_Avm_Obstacle(&avmObstacle);
}

static void ObstacleMsg_Callback(
    const std::shared_ptr<const ::raincom::apa::msgs::Objects> msg)
{
    const uint8_t buff_size = 30;
    AVM_SlotPointsType existSlots[buff_size];
    AVM_ObjPointsType existObjs[buff_size];
    memset(existSlots, 0, sizeof(AVM_SlotPointsType) * buff_size);
    memset(existObjs, 0, sizeof(AVM_ObjPointsType) * buff_size);

    int avmSlotNum = 0;
    int avmObjNum  = 0;

    // 如果路径执行模块没有处于泊车状态，且当前车辆处于倒车档（R档），直接返回
    if (RTE_PK_StateManage_Get_ModuleCom_PathExecute() != MCOM_ON_PARKING)
    {
        if (RTE_BSW_Get_CurrentGear() == GEAR_REAL_R)
        {
            return;
        }
    }

    if (RTE_PK_StateManage_Get_ModuleCom_Location() == MCOM_ON)
    {
        if (avmlog == NULL && is_save_log)
        {
            avmlog = OpenLogFile("avm.txt");
        }
    }

    // 解析数据
    for (int i = 0; i < msg->objects_size(); i++)
    {
        auto object    = msg->objects(i);
        auto objtype   = object.clf();
        auto available = object.available();
        auto avmObjId  = object.id();
        if (object.corners_size() != 4)
        {
            continue;
        }

        Point_T pointOrd[4];
        for (int i = 0; i < 4; i++)
        {
            pointOrd[i].x = object.corners(i).x();
            pointOrd[i].y = object.corners(i).y();
        }

        if ((uint32_t)objtype == (uint32_t)LEFT_VER_SLOT && avmSlotNum < buff_size)
        {
            ConvertToSlot(existSlots[avmSlotNum], pointOrd);
            existSlots[avmSlotNum].shape = (PK_SlotShapeType)CheckObstacleType(objtype);
            existSlots[avmSlotNum].timestamp = avmObjId;
            if (!available)
            {
                existSlots[avmSlotNum].hasLock = 3;
            }

            avmSlotNum++;
        }
        else if ((uint32_t)objtype == (uint32_t)RIGHT_VER_SLOT && avmSlotNum < buff_size)
        {
            ConvertToSlot(existSlots[avmSlotNum], pointOrd);
            existSlots[avmSlotNum].shape = (PK_SlotShapeType)CheckObstacleType(objtype);
            existSlots[avmSlotNum].timestamp = avmObjId;
            if (!available)
            {
                existSlots[avmSlotNum].hasLock = 3;
            }

            avmSlotNum++;
        }
        else if ((uint32_t)objtype == (uint32_t)LEFT_PAR_SLOT && avmSlotNum < buff_size)
        {
            ConvertToSlot(existSlots[avmSlotNum], pointOrd);
            existSlots[avmSlotNum].shape = (PK_SlotShapeType)CheckObstacleType(objtype);
            existSlots[avmSlotNum].timestamp = avmObjId;
            if (!available)
            {
                existSlots[avmSlotNum].hasLock = 3;
            }

            avmSlotNum++;
        }
        else if ((uint32_t)objtype == (uint32_t)RIGHT_PAR_SLOT && avmSlotNum < buff_size)
        {
            ConvertToSlot(existSlots[avmSlotNum], pointOrd);
            existSlots[avmSlotNum].shape = (PK_SlotShapeType)CheckObstacleType(objtype);
            existSlots[avmSlotNum].timestamp = avmObjId;
            if (!available)
            {
                existSlots[avmSlotNum].hasLock = 3;
            }

            avmSlotNum++;
        }
        else if ((uint32_t)objtype == (uint32_t)SLB ||
                 (uint32_t)objtype == (uint32_t)OPLB_R ||
                 (uint32_t)objtype == (uint32_t)OPLB_I)
        {
            auto index = FindAvmSlot(existSlots, avmSlotNum, pointOrd);
            if (index >= 0)
            {
                ConvertToStopBar(pointOrd, existSlots[index].stopPoint);
                existSlots[index].hasStop = 1;
            }
        }
        else if ((uint32_t)objtype == (uint32_t)PL_CLOSED ||
                 (uint32_t)objtype == (uint32_t)PL_OPENED)
        {
            auto index = FindAvmSlot(existSlots, avmSlotNum, pointOrd);
            if (index >= 0)
            {
                existSlots[index].hasLock =
                    ((uint32_t)objtype == (uint32_t)PL_OPENED) ? 1 : 0;
            }
        }
        else if ((uint32_t)objtype == (uint32_t)CONE && avmObjNum < buff_size)
        {
            ConvertToObj(existObjs[avmObjNum], pointOrd);
            existObjs[avmObjNum].obstacleType = CheckObstacleType(objtype);
            existObjs[avmObjNum].index        = avmObjNum;
            avmObjNum++;
        }
        else if ((uint32_t)objtype == (uint32_t)PEDSTRAIN && avmObjNum < buff_size)
        {
            ConvertToObj(existObjs[avmObjNum], pointOrd);
            existObjs[avmObjNum].obstacleType = CheckObstacleType(objtype);
            existObjs[avmObjNum].index        = avmObjNum;
            avmObjNum++;
        }
        else if ((uint32_t)objtype == (uint32_t)WARNING_POST && avmObjNum < buff_size)
        {
            ConvertToObj(existObjs[avmObjNum], pointOrd);
            existObjs[avmObjNum].obstacleType = CheckObstacleType(objtype);
            existObjs[avmObjNum].index        = avmObjNum;
            avmObjNum++;
        }
        else
        {
            continue;
        }
    }

    // 设置障碍物数据
    // printf("recv objNum: %d\n", avmObjNum);
    UpdateAvmObstacle(existObjs, avmObjNum);

    // 设置车位数据
    // printf("recv slotNum: %d\n", avmObjNum);
    for (int i = 0; i < avmSlotNum; i++)
    {
        if (RTE_PK_StateManage_Get_ModuleCom_PathExecute() == MCOM_ON_PARKING)
        {
            if (avm_slot_update_on_parking == 0)
            {
                continue;
            }

            if (RTE_PK_DataConvt_Get_IsAVM_Slot() == 0 ||
                RTE_PK_DataConvt_Get_IsAVM_Slot() == 2)
            {
                continue;
            };

            if (InSideEreaV3() || InRearErea())
            {
                if (SetAvmSlotPointB(&existSlots[i]))
                {
                    break;
                } // 设置车位点信息
            }
        }
        else
        {
            RTE_PD_Set_AVM_SlotPoints(&existSlots[i]);
            RecordAvmLog(&existSlots[i], (int)existSlots[i].timestamp, 0);
        }
    }
}
static void SLAM_Odom_Callback(
    const std::shared_ptr<const ::raincom::apa::msgs::Odom3d> msg)
{
    // OdomLoc odomData;

    // // 读取头部信息
    // odomData.header.stamp = msg->header().stamp();
    // odomData.header.id    = msg->header().id();

    // // 读取位置变换
    // odomData.transform.x = msg->transform().x();
    // odomData.transform.y = msg->transform().y();
    // odomData.transform.z = msg->transform().z();

    // // 读取旋转
    // odomData.rotation.r = msg->rotation().r();
    // odomData.rotation.p = msg->rotation().p();
    // odomData.rotation.y = msg->rotation().y();

    // // 读取速度
    // odomData.twist.linear.x = msg->twist().linear().x();
    // odomData.twist.linear.y = msg->twist().linear().y();
    // odomData.twist.linear.z = msg->twist().linear().z();

    // odomData.twist.angular.r = msg->twist().angular().r();
    // odomData.twist.angular.p = msg->twist().angular().p();
    // odomData.twist.angular.y = msg->twist().angular().y();

    // // 读取置信度
    // odomData.confidence = msg->confidence();

    // printf("odomData.transform.x : %f\n", odomData.transform.x);
    // RTE_RD_Set_SLAM_Odom(&odomData);
}

static void *DetectVisionLoop(void *paras)
{
    const char *argv[2] = {"apa_pk_bst", "a1000"};
    raincom::com::init(2, (char **)argv);
    printf("Sense_InitCom init !\n");
    usleep(100000);

    raincom::com::Node node = raincom::com::Node("apa_park");
    auto sub_obstacle       = node.create_subscription<raincom::apa::msgs::Objects>(
        "apa/fusion/objs", 10, ObstacleMsg_Callback);
    auto sub_odom = node.create_subscription<raincom::apa::msgs::Odom3d>(
        "apa/slam/odom", 10, SLAM_Odom_Callback);

    auto pub_location =
        node.create_publisher<raincom::apa::msgs::Odom>("apa/location", 3);
    auto pub_chassis =
        node.create_publisher<raincom::apa::msgs::Chassis>("apa/chassis", 3);

    uint32_t count = 0;
    while (node.ok())
    {
        raincom::com::Node::spin_once();

        if ((count++ % 20) == 0) // 20ms
        {
            ::raincom::apa::msgs::Odom locMsg;
            ::raincom::apa::msgs::Chassis chassisMsg;
            RD_PosStamp posStamp;
            RTE_PK_Location_Get_CurPos_With_Timestamp(&posStamp);

            locMsg.set_x(posStamp.pos[0]);
            locMsg.set_y(posStamp.pos[1]);
            locMsg.set_heading(posStamp.pos[2]);
            locMsg.set_s(posStamp.pos[3]);
            locMsg.mutable_header()->set_stamp(posStamp.t);
            pub_location->publish(locMsg);

            using namespace ::raincom::apa::msgs;
            chassisMsg.mutable_header()->set_stamp(posStamp.t);

            // Assuming we have functions to get these values
            chassisMsg.set_steering_wheel_angle_valid(true);
            chassisMsg.set_steering_wheel_angle(RTE_BSW_Get_EPS_Angle());

            chassisMsg.set_wheel_speed_fl_valid(RTE_BSW_Get_ESC_WheelSpdFLVD());
            chassisMsg.set_wheel_speed_fl(RTE_BSW_Get_ESC_WheelSpdFL());

            chassisMsg.set_wheel_speed_fr_valid(RTE_BSW_Get_ESC_WheelSpdFRVD());
            chassisMsg.set_wheel_speed_fr(RTE_BSW_Get_ESC_WheelSpdFR());

            chassisMsg.set_wheel_speed_rl_valid(RTE_BSW_Get_ESC_WheelSpdRLVD());
            chassisMsg.set_wheel_speed_rl(RTE_BSW_Get_ESC_WheelSpdRL());

            chassisMsg.set_wheel_speed_rr_valid(RTE_BSW_Get_ESC_WheelSpdRRVD());
            chassisMsg.set_wheel_speed_rr(RTE_BSW_Get_ESC_WheelSpdRR());

            Chassis_BeamStatus BeamSt;
            if (RTE_BSW_Get_Dipped_Headlight_St() == 0 &&
                RTE_BSW_Get_High_Beam_Light_St() == 0)
            {
                BeamSt = Chassis_BeamStatus_BEAM_INVALID;
            }
            else if (RTE_BSW_Get_Dipped_Headlight_St() == 2 &&
                     RTE_BSW_Get_High_Beam_Light_St() == 2)
            {
                BeamSt = Chassis_BeamStatus_BEAM_OFF;
            }
            else if (RTE_BSW_Get_High_Beam_Light_St() == 1)
            {
                BeamSt = Chassis_BeamStatus_BEAM_HIGH_ON;
            }
            else if (RTE_BSW_Get_Dipped_Headlight_St() == 1)
            {
                BeamSt = Chassis_BeamStatus_BEAM_LOW_ON;
            }
            else if (RTE_BSW_Get_Dipped_Headlight_St() == 3 &&
                     RTE_BSW_Get_High_Beam_Light_St() == 3)
            {
                BeamSt = Chassis_BeamStatus_BEAM_RESERVED;
            }
            else
            {
                BeamSt = Chassis_BeamStatus_BEAM_INVALID;
            }

            chassisMsg.set_beam(BeamSt);

            RTE_BSW_BCM_StateType bcmSt;
            Chassis_DoorStatus doorStatus;
            RTE_BSW_Get_BCM_VehicleState(&bcmSt);

            doorStatus = bcmSt.BCM_PsngrDoorAjarSt == 1 ? Chassis_DoorStatus_DOOR_OPEND
                                                        : Chassis_DoorStatus_DOOR_CLOSED;
            chassisMsg.set_rf_door(doorStatus);
            doorStatus = bcmSt.BCM_DriverDoorAjarSt == 1 ? Chassis_DoorStatus_DOOR_OPEND
                                                         : Chassis_DoorStatus_DOOR_CLOSED;
            chassisMsg.set_lf_door(doorStatus);
            doorStatus = bcmSt.BCM_RRDoorAjarSt == 1 ? Chassis_DoorStatus_DOOR_OPEND
                                                     : Chassis_DoorStatus_DOOR_CLOSED;
            chassisMsg.set_rr_door(doorStatus);
            doorStatus = bcmSt.BCM_RLDoorAjarSt == 1 ? Chassis_DoorStatus_DOOR_OPEND
                                                     : Chassis_DoorStatus_DOOR_CLOSED;
            chassisMsg.set_lr_door(doorStatus);
            doorStatus = bcmSt.BCM_BonnetAjarSt == 1 ? Chassis_DoorStatus_DOOR_OPEND
                                                     : Chassis_DoorStatus_DOOR_CLOSED;
            chassisMsg.set_front_hatch(doorStatus);
            doorStatus = bcmSt.RDM_RearDoorStatus == 1 ? Chassis_DoorStatus_DOOR_OPEND
                                                       : Chassis_DoorStatus_DOOR_CLOSED;
            chassisMsg.set_back_hatch(doorStatus);

            pub_chassis->publish(chassisMsg);
        }

        if (RTE_PK_StateManage_Get_ModuleCom_Location() != MCOM_ON)
        {
            if (avmlog != NULL)
            {
                fclose(avmlog);
                avmlog = NULL;
            }
        }

        usleep(1000);
    }

    printf("Sense_InitCom failed !\n");
    return NULL;
}

VisionMsgRecvPtr GetVisionLoopFunc() { return DetectVisionLoop; }
#endif
#endif
