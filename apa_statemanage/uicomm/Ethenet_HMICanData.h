#ifndef ETHENET_HMICANDATA_H
#define ETHENET_HMICANDATA_H

#include <stdio.h>
#include <stdint.h>

#define APA_STATUSMANAGE_SIZE 9   // APA状态数据长度
#define VEH_POSITION_SIZE     9   // 泊车姿态数据长度
#define RADER_DATA_SIZE       33  // 雷达数据长度
#define COMMAND_HMI_SIZE      9   // 控制指令数据长度
#define SLOT_DESIGNATED_SIZE  25  // 指定车位数据长度
#define CAR_DOORSTATUS_SIZE   9   // 车门状态数据长度
#define OBSTACLE_PACKET_SIZE  129 // 视觉障碍物数据包长度

typedef enum
{
    CanId_ApaStatusManage = 0x00,
    CanId_SlotPosition    = 0x01,
    CanId_VehPosition     = 0x02,
    CanId_RaderData       = 0x03,
    CanId_Obstacle        = 0x04,
    CanId_CarDoorStatus   = 0x05
} CADID_APA2HMI;

typedef enum
{
    CanId_CommandHMI     = 0x00,
    CanId_SlotDesignated = 0x01,
} CADID_HMI2APA;

#pragma pack(push, 1) // 保存当前对齐方式，设置为 1 字节对齐
/*发送*/
//定义APA状态数据结构体
typedef struct
{
    uint8_t canId;          // 0x00
    unsigned apa_mode : 4;  // 泊车模式 APA_MODE
    unsigned apa_state : 4; // 泊车状态 APA_STATE
    uint8_t apa_info;       // 泊车文字提示 APA_INFO
    unsigned apa_avp : 4;   // AVP状态 APA_AVP
    unsigned apa_gear : 4;  // 档位 APA_GEAR
    int16_t apa_radius;     // 车子的转弯半径R
    uint16_t empty_data;    // 空数据
    uint8_t checksum;       // Checksum校验码
} apaStatusManage;

//单角点数据
typedef struct
{
    uint8_t canId;          //发送的数据组编号ID
    uint16_t slot_index;    //发送的车位编号
    unsigned stopbarv : 1;  //限位杆有效
    unsigned stopbarp : 1;  //限位杆位置（距车远端/近端）
    unsigned lockflagv : 1; //地锁有效
    unsigned lockflagp : 1; //地锁打开状态
    unsigned point_x : 14;  //角点x
    unsigned point_y : 14;  //角点y值
    uint8_t checksum;       //校验码
} slotPoint;

//车位数据
typedef struct
{
    uint8_t canId; // 0x01
    slotPoint slot_position[33];
} slotPosition;

//泊车姿态veh position
typedef struct
{
    uint8_t canId;            // 0x02
    int16_t curpos_x;         //泊车过程中车子的位置x
    int16_t curpos_y;         //泊车过程中车子的位置y
    int16_t angle;            //泊车过程中车子的位置angle
    uint8_t parking_progress; //泊车当前进度
    uint8_t checksum;         //校验码
} vehPosition;

//倒车雷达数据——分
typedef struct
{
    uint8_t canId;    //数据组编号
    uint16_t rader_a; //超声波数据
    uint16_t rader_b;
    uint16_t rader_c;
    uint8_t checksum;
} raderData;

//倒车雷达数据——总
typedef struct
{
    uint8_t canId; // 0x03
    raderData rader_data[4];
} raderDataArray;

typedef struct
{
    int8_t TaperedBarrel_type;       //类型
    float TaperedBarrel_CenterPos_x; //中心坐标x
    float TaperedBarrel_CenterPos_y; //中心坐标y

} TaperedBarrel; //障碍物信息(锥形桶)

typedef struct
{
    TaperedBarrel TaperedBarrel_data[14];

} AppTypeTaperedBarrel; //障碍物信息

//左前/右前/左后/右后/后背门/引擎盖开关状态显示
typedef struct
{
    unsigned car_door_fl : 1; // 左前车门
    unsigned car_door_fr : 1; // 右前车门
    unsigned car_door_rl : 1; // 左后车门
    unsigned car_door_rr : 1; // 右后车门
    unsigned car_hood : 1;    // 前舱盖
    unsigned car_trunk : 1;   // 后舱盖
    unsigned empty_data : 2;  // 空数据
    uint32_t empty_data1;     // 空数据
    uint16_t empty_data2;     // 空数据
    uint8_t checksum;         // Checksum校验码
} CarDoorStatus;

typedef struct
{
    uint8_t canId; // id 0x05
    CarDoorStatus carDoorStatus;
} CarDoorCanData;

typedef struct
{
    uint8_t canId;             //发送的数据组编号ID
    uint16_t obstacle_type;    //障碍物类型
    int16_t obstacle_center_x; //障碍物中心点x
    int16_t obstacle_center_y; //障碍物中心点y
    uint8_t checksum;          //校验码

} ObstaclePosition;

typedef struct
{
    uint8_t canId;           //发送的数据组编号ID
    int16_t obstacle_angle;  //障碍物角度
    int16_t obstacle_length; //障碍物长度
    int16_t obstacle_width;  //障碍物宽度
    uint8_t checksum;        //校验码
} ObstacleShape;

typedef struct
{
    ObstaclePosition objPosition;
    ObstacleShape objShape;
} ObstacleData;

typedef struct
{
    uint8_t canId; // 0x04
    ObstacleData objData[8];
} ObstaclePacket;

/*接收*/
//屏幕按键指令
typedef struct
{
    uint8_t canId;                // 0x00
    unsigned apa_start : 4;       //泊车启动状态 APA_STAR
    unsigned apa_state : 4;       //泊车状态 APA_STATE
    unsigned single_boundary : 4; //单边界检测状态(预留)
    unsigned parking_pattern : 4; //泊入泊出模式
    uint16_t slot_index;          //选择泊车的车位序号
    unsigned empty_data : 24;     //空数据
    uint8_t checksum;
} commandHMI;

//指定车位部分数据1
typedef struct
{
    uint8_t canId;    //数据组编号
    int16_t point_a;  //
    int16_t point_b;  //
    int16_t point_c;  //
    uint8_t checksum; //校验码
} slotDataPartial;

//指定车位部分数据2
typedef struct
{
    uint8_t canId;       //数据组编号
    int16_t point_x;     // x
    int16_t point_y;     // y
    uint16_t empty_data; //空数据
    uint8_t checksum;    //校验码
} slotDataLast;

//指定车位
typedef struct
{
    uint8_t canId; // 0x01
    slotDataPartial data_partial[2];
    slotDataLast data_last;
} slotDesignated;

#pragma pack(pop) // 恢复之前保存的对齐方式

//校验码
uint8_t calculateChecksum(const uint8_t *data);
void BuildRaderData(raderData *data, int can_id, int a, int b, int c);
void BuildVehPosition(vehPosition *data, float x, float y, float angle, int run_rate);
void BuildSlotPosition(slotPoint *data, const float *point, const float curPos[4],
                       int slot_index, int stopflags[4], int id);

#endif