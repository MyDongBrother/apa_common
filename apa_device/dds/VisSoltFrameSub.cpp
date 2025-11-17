/**
 * @file VisSoltFrameSub.cpp
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-11-04
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-11-04 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/log/Log.hpp>
#include <fastdds/dds/log/StdoutConsumer.hpp>
#include <fastdds/dds/log/FileConsumer.hpp>

#include "MultiPolygonFramePubSubTypes.h"
#include "VisSoltFrameSub.h"
#include <Record_Log.h>
#include <Rte_Types.h>
#include <Module_API.h>
#include <Rte_ComIF.h>
#include <SystemPara.h>
#include <Rte_BSW.h>
#include <MathPara.h>
#include "MathFunc.h"

using namespace eprosima::fastdds::dds;

VisSoltFrameSub::VisSoltFrameSub()
    : participant_(nullptr),
      subscriber_(nullptr),
      topic_(nullptr),
      reader_(nullptr),
      type_(new apa_msgs::msg::MultiPolygonFramePubSubType())
{
}

VisSoltFrameSub::~VisSoltFrameSub()
{
    if (reader_ != nullptr)
    {
        subscriber_->delete_datareader(reader_);
    }
    if (topic_ != nullptr)
    {
        participant_->delete_topic(topic_);
    }
    if (subscriber_ != nullptr)
    {
        participant_->delete_subscriber(subscriber_);
    }
    DomainParticipantFactory::get_instance()->delete_participant(participant_);
}

bool VisSoltFrameSub::init()
{
    // 清空默认输出（避免重复）
    Log::ClearConsumers();

    // 注册控制台输出
    std::unique_ptr<StdoutConsumer> stdout_consumer(new StdoutConsumer());
    Log::RegisterConsumer(std::move(stdout_consumer));

    // 同时输出到文件（比如放到 /data/local/tmp 方便看）
    // std::unique_ptr<FileConsumer> file_consumer(
    //     new FileConsumer("/data/local/tmp/fastdds.log", true));
    // Log::RegisterConsumer(std::move(file_consumer));

    // 设置打印级别：Info / Warning / Error
    Log::SetVerbosity(Log::Kind::Error);

    // 打印文件名与函数名（更详细）
    Log::ReportFilenames(true);
    Log::ReportFunctions(true);

    // 创建 Domain Participant（通信域节点，相当于 DDS 世界的“进程身份”）
    DomainParticipantQos pqos;
    pqos.name("Participant_sub");
    participant_ = DomainParticipantFactory::get_instance()->create_participant(0, pqos);
    if (participant_ == nullptr)
    {
        return false;
    }

    // 注册 Topic 对应的数据类型（IDL 生成的类型支持）
    type_.register_type(participant_);

    // 创建 Subscriber（订阅端）
    subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
    if (subscriber_ == nullptr)
    {
        return false;
    }

    // 创建 Topic（逻辑通道，名称需与发布端一致）
    topic_ = participant_->create_topic("rt/vis_solt", type_.get_type_name(),
                                        TOPIC_QOS_DEFAULT);
    if (topic_ == nullptr)
    {
        return false;
    }

    // 创建 DataReader（数据订阅对象）
    DataReaderQos rqos = DATAREADER_QOS_DEFAULT;
    // 可靠传输模式（适合车位信息等关键数据）
    rqos.reliability().kind = RELIABLE_RELIABILITY_QOS;

    reader_ = subscriber_->create_datareader(topic_, rqos, &listener_);
    if (reader_ == nullptr)
    {
        return false;
    }

    return true;
}

void VisSoltFrameSub::SubListener::on_subscription_matched(
    DataReader *, const SubscriptionMatchedStatus &info)
{
    // 当前有新的 Publisher 连接（+1）
    if (info.current_count_change == 1)
    {
        matched = info.total_count;
        log_info("Subscriber matched.");
    }
    // 有 Publisher 断开（-1）
    else if (info.current_count_change == -1)
    {
        matched = info.total_count;
        log_info("Subscriber unmatched.");
    }
    // 其他情况（理论上不会出现）
    else
    {
        log_info(
            "%d is not a valid value for SubscriptionMatchedStatus current count change.",
            info.current_count_change);
    }
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

/**
 * @brief 判断车辆正后方有效车位范围
 *
 * 用于泊车过程中，最后一段路径时校正车位，在有效范围内的车位数据会用于更新车位坐标
 *
 * @param
 * @return bool true - 在范围内 false - 不在范围内，舍弃
 */
bool InRearErea()
{
    float target[3], curpos[4], rspoint[3];
    RTE_PK_SlotDetect_Get_TargPos(target);
    RTE_PK_Location_Get_CurPos(curpos);
    CoordinadteTransfer(target, curpos, rspoint);

    if (fabs(rspoint[2]) > 15 * PI_RAD || fabs(rspoint[1]) > 0.6)
    {
        return false;
    }
    if (fabs(rspoint[0]) > 2.5 || fabs(rspoint[0]) < 0.5)
    {
        return false;
    }

    RTE_BSW_GearType gear = RTE_BSW_Get_CurrentGear();
    return (GEAR_REAL_R == gear && sign(rspoint[0]) == 1) ||
           (GEAR_REAL_D == gear && sign(rspoint[0]) == -1);
    return true;
}

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

// 转换为上报的矩形点格式
static void ConvertToRect(const AVM_SlotPointsType &slot, Point_T slotPoint[4])
{
    memcpy((void *)&slotPoint[0], slot.NearRear, sizeof(Point_T));
    memcpy((void *)&slotPoint[1], slot.FarRear, sizeof(Point_T));
    memcpy((void *)&slotPoint[2], slot.FarFront, sizeof(Point_T));
    memcpy((void *)&slotPoint[3], slot.NearFront, sizeof(Point_T));
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
    }

    return true;
}

/**
 * @brief dds接收回调
 *
 * @param reader
 */
void VisSoltFrameSub::SubListener::on_data_available(DataReader *reader)
{
    apa_msgs::msg::MultiPolygonFrame st;
    std::vector<apa_msgs::msg::PolygonFrame> &polygon = st.polygon();
    // std::vector<apa_msgs::msg::Pos2D> &pos = polygon.p;
    SampleInfo info;
    AVM_SlotPointsType existSlots;
    if (reader->take_next_sample(&st, &info) == ReturnCode_t::RETCODE_OK)
    {
        if (info.valid_data)
        {
            for (auto polygon : st.polygon())
            {
                existSlots.NearRear[0]  = polygon.points()[0].x();
                existSlots.NearRear[1]  = polygon.points()[0].y();
                existSlots.FarRear[0]   = polygon.points()[1].x();
                existSlots.FarRear[1]   = polygon.points()[1].y();
                existSlots.FarFront[0]  = polygon.points()[2].x();
                existSlots.FarFront[1]  = polygon.points()[2].y();
                existSlots.NearFront[0] = polygon.points()[3].x();
                existSlots.NearFront[1] = polygon.points()[3].y();
                existSlots.shape        = (PK_SlotShapeType)polygon.type();
                existSlots.hasStop      = false;
                memset(existSlots.stopPoint, 0, sizeof(existSlots.stopPoint));
                existSlots.hasLock   = false;
                existSlots.timestamp = polygon.header().stamp();

                // 在泊车阶段
                if (RTE_PK_StateManage_Get_ModuleCom_PathExecute() == MCOM_ON_PARKING)
                {
                    if (RTE_PK_DataConvt_Get_IsAVM_Slot() == 0 ||
                        RTE_PK_DataConvt_Get_IsAVM_Slot() == 2)
                    {
                        continue;
                    };
                    if (InSideEreaV3() || InRearErea())
                    {
                        if (SetAvmSlotPointB(&existSlots))
                        {
                            break;
                        }
                    }
                }
                else
                {
                    RTE_PD_Set_AVM_SlotPoints(&existSlots);
                }
            }
            log_warn(
                "NearRear.x(%f),NearRear.y(%f),NearFront.x(%f),NearFront.y(%f),pos.t(%"
                "lld)",
                st.polygon()[0].points()[0].x(), st.polygon()[0].points()[0].y(),
                st.polygon()[0].points()[3].x(), st.polygon()[0].points()[3].y(),
                st.header().stamp());
        }
    }
}

void VisSoltFrameSub::run() { return; }
