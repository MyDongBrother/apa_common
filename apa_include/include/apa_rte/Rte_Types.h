/**
 * @file Rte_Types.h
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-08-28
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-08-28 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#ifndef RTE_TYPES_H
#define RTE_TYPES_H

#include "Std_Types.h"

// ================= BSW Basic Software ======================
typedef enum
{
    GEAR_REAL_NONE,
    GEAR_REAL_D,
    GEAR_REAL_N,
    GEAR_REAL_R,
    GEAR_REAL_P,
    GEAR_REAL_MAX
} RTE_BSW_GearType;

/**
 * @brief EPS控制状态枚举
 */
typedef enum
{
    EPS_CTRL_TEMP_INHIBITED = 0x0, /**< 暂时的抑制,不可泊车控制,EPS自恢复 */
    EPS_CTRL_AVAILABLE      = 0x1, /**< 可供控制：APA 可以请求并接管 ESC 控制 */
    EPS_CTRL_ACTIVE         = 0x2, /**< 激活状态：APA 正在使用 ESC 进行泊车控制 */
    EPS_CTRL_PERM_INHIBITED = 0x3  /**< 永久抑制：不可泊车控制,需要重启车辆 */
} RTE_BSW_EpsCtrlStatType;

/**
 * @brief ESC 控制状态枚举
 *
 * 该枚举表示 ESC（电子稳定控制系统）在 APA 控制下的可用性与工作状态。
 */
typedef enum
{
    ESC_CTRL_TEMP_INHIBITED = 0x0, /**< 暂时的抑制,不可泊车控制,EPS自恢复 */
    ESC_CTRL_AVAILABLE      = 0x1, /**< 可供控制：APA 可以请求并接管 ESC 控制 */
    ESC_CTRL_ACTIVE         = 0x2, /**< 激活状态：APA 正在使用 ESC 进行泊车控制 */
    ESC_CTRL_PERM_INHIBITED = 0x3  /**< 永久抑制：不可泊车控制,需要重启车辆 */
} RTE_BSW_EscCtrlStatType;

/**
 * @brief TCU 挡位控制反馈状态枚举
 *
 * 该枚举表示 TCU（变速箱控制单元）在 APA 挡位请求下的执行与反馈情况。
 */
typedef enum
{
    TCU_NoOperation        = 0x0, /**< APA 未下发任何挡位控制请求 */
    TCU_NormalOperation    = 0x1, /**< APA 下发的挡位请求已由 TCU 正常执行 */
    TCU_DriverInterference = 0x2, /**< 驾驶员操作挡杆，导致 APA 挡位控制被打断 */
    TCU_Reserved           = 0x3  /**< 预留：暂无定义，保留后续扩展 */
} RTE_TCU_ApaStatusType;

/**
 * @brief EPB状态
 */
typedef enum
{
    EPB_Releasing = 0, /**< 松开中 */
    EPB_Released  = 1, /**< 已松开 */
    EPB_Applying  = 2, /**< 施加中 */
    EPB_Applied   = 3, /**< 已施加 */
    EPB_Fault     = 4, /**< 故障（禁止遥控驾驶） */
    EPB_Reserved5 = 5  /**< 预留 */
} RTE_BSW_EpbModeType;

/**
 * @brief 移动方向枚举
 */
typedef enum
{
    MoveDir_Invalid  = 0, /**< 无效/松开中 */
    MoveDir_Forward  = 1, /**< 前进 */
    MoveDir_Backward = 2, /**< 后退 */
    MoveDir_Stop     = 3, /**< 停止 */
} RTE_BSW_MoveDirType;

/**
 * @brief BCM 状态数据结构体（来自车身控制模块）
 */
typedef struct
{
    uint8_t BCM_DriverDoorLockSt;      /**< 驾驶员车门锁状态：0-解锁，1-上锁 */
    uint8_t BCM_DriverDoorAjarSt;      /**< 驾驶员车门未关状态：0-关闭，1-未关好 */
    uint8_t BCM_PsngrDoorAjarSt;       /**< 副驾驶车门未关状态：0-关闭，1-未关好 */
    uint8_t BCM_RLDoorAjarSt;          /**< 左后车门未关状态：0-关闭，1-未关好 */
    uint8_t BCM_RRDoorAjarSt;          /**< 右后车门未关状态：0-关闭，1-未关好 */
    uint8_t BCM_BonnetAjarSt;          /**< 引擎盖未关状态：0-关闭，1-未关好 */
    uint8_t BCM_DriverSeatOccupied;    /**< 驾驶员座椅是否有人：0-无人，1-有人 */
    uint8_t SRS_DriverSeatBeltWarning; /**< 驾驶员安全带报警状态：0-未报警，1-报警 */
    uint8_t RDM_RearDoorStatus;        /**< 后备箱：0-未启用，1-启用 */
} RTE_BSW_BCM_StateType;

/**
 * @brief 外后视镜折叠/展开状态枚举
 */
typedef enum
{
    FldigExtrrNoAction = 0x0, /**< 无动作 */
    FldigExtrrFolded   = 0x1, /**< 折叠*/
    FldigExtrrExpanded = 0x2, /**< 展开 */
    FldigExtrrReserved = 0x3  /**< 预留 */
} RTE_BSW_FldigExtrrMrorsType;

/* 灯光状态枚举 */
typedef enum
{
    LIGHT_INVALID  = 0x0, /**< 无效 */
    LIGHT_ON       = 0x1, /**< 打开 */
    LIGHT_OFF      = 0x2, /**< 关闭 */
    LIGHT_RESERVED = 0x3, /**< 预留 */
} RTE_BSW_LightStateType;

typedef union {
    uint8_t raw[32]; ///< 原始访问，32字节总长度

    struct
    {
        uint8_t cmd_id; ///< 录制命令ID : 1

        // echo record 0 > debugCmd   关闭功能
        // echo record 1 > debugCmd   开启录制功能
        // echo record 2 > debugCmd   开启回灌功能
        uint8_t switch_flag;
        // 功能暂时未实现
        uint8_t record_name[30]; ///< 当 cmd_id = 1 时，录制的路径
    } record_cmd;

    struct
    {
        uint8_t cmd_id;       ///< 命令ID，2 = set_park_slot
        uint16_t slot_id;     ///< 当 cmd_id = 2 时，表示车位ID
        uint8_t reserved[29]; ///< 填充保持32字节
    } set_park_slot_cmd;

    // 可继续添加其他命令结构
} DebugCmdType;

/**
 * @brief 内部故障码定义（最多支持 32 bit）
 */
typedef enum
{
    kInternalMainThreadStarve = 1u << 0, ///< 主线程饿死
    // 预留后续内部扩展位
} InternalFaultBit;

/**
 * @brief 外部关联件故障码定义（最多支持 32 bit）
 */
typedef enum
{
    kExternalEps    = 1u << 0, ///< EPS 故障（转向助力）
    kExternalEsc    = 1u << 1, ///< ESC 故障（稳定/制动系统）
    kExternalTcu    = 1u << 2, ///< Tcu 故障（档位控制器）
    kExternalEpb    = 1u << 3, ///< EPB 故障
    kExternalCamera = 1u << 4, ///< 摄像头故障
    kExternalUss    = 1u << 5, ///< 超声波雷达故障
    // 预留后续外部扩展位
} ExternalFaultBit;

// ================= SWC Software Component ======================

/**
 * @brief EPS控制状态枚举
 */
typedef enum
{
    CTRL_TEMP_INHIBITED = 0x0, /**< 暂时的抑制,不可泊车控制,EPS自恢复 */
    CTRL_AVAILABLE      = 0x1, /**< 可供控制：APA 可以请求并接管 ESC 控制 */
    CTRL_ACTIVE         = 0x2, /**< 激活状态：APA 正在使用 ESC 进行泊车控制 */
    CTRL_PERM_INHIBITED = 0x3  /**< 永久抑制：不可泊车控制,需要重启车辆 */
} RTE_BSW_RelateCtrlStatType;

/**
 * @brief 车辆工作状态枚举（自动泊车流程）
 */
typedef enum
{
    CarSta_Passive                 = 0,  /**< 被动状态，系统未激活 */
    CarSta_Serching                = 1,  /**< 正在搜索车位 */
    CarSta_GuidanceActive          = 2,  /**< 引导中，车辆正在执行泊车路径 */
    CarSta_GuidanceSuspend         = 3,  /**< 引导暂停，例如驾驶员干预或异常触发 */
    CarSta_GuidanceTerminated      = 4,  /**< 引导中止，可能由于错误或取消操作 */
    CarSta_GuidanceCompleted       = 5,  /**< 引导完成，车辆成功泊入车位 */
    CarSta_Failure                 = 6,  /**< 系统故障状态 */
    CarSta_ParkAssist_Standby      = 7,  /**< 泊车准备状态，完成关联件握手等动作 */
    CarSta_GuidanceInitParkingData = 8,  /**< 正在初始化泊车数据（车位、路径等） */
    CarSta_Passive_SpdOver         = 9,  /**< 被动状态-车速超限，无法进入泊车流程 */
    CarSta_PowerOn                 = 10, /**< 上电初始化状态 */
    CarSta_GuidanceAvpActive             /**< AVP 模式下的引导激活（自主代客泊车） */
} Apa_WorkStateType;

typedef enum
{
    CAR_TYPE_NO   = 0,
    CAR_TYPE_YUAN = 1,
    CAR_TYPE_HAN  = 2,
    CAR_TYPE_SONG = 3,
    CAR_TYPE_MAX,
} PK_CarType;

typedef enum
{
    APA_MODE,
    AVP_MODE,
    PARKOUT_MODE,
    RPA_MODE,
    INVALID_MODE
} HMI_ApaModeType;

typedef enum
{
    MCOM_INIT,          // 初始化状态,该状态下的模块应该清除所有模块中使用的全局和静态变量
    MCOM_ON,            // 开启状态
    MCOM_OFF,           // 关闭状态
    MCOM_DATCLC,        // 数据计算
    MCOM_ON_PARKING,    // 泊车状态进入
    MCOM_ON_PARKINGOUT, // 泊车状态退出
    MCOM_ON_SUMMONUP_BACK, // 唤起状态进入（后退） PK_ObjAvoid
    MCOM_ON_SUMMONUP_FOR,  // 唤起状态进入（前进） PK_ObjAvoid
    MCOM_ON_SUMMONUP_STOP, // 唤起状态停止 PK_ObjAvoid
    MCOM_ON_PARKING_PAUSE,
    MCOM_ON_PARKINGOUT_PAUSE,
    MCOM_ON_AVMPARKING, // 自动泊车状态进入
} PK_ModuleComType;

typedef enum
{
    MSTAT_NORM,       ///< 正常
    MSTAT_ABNORM,     ///< 异常
    MSTAT_OFF,        ///< 关闭状态
    MSTAT_DATCLC,     ///< 数据清除
    MSTAT_OUT_RANGE,  ///< 超出测量范围
    MSTAT_GET_RESULT, ///< 获得结果（用于 PK_SlotDetect 与 PK_PathPlan）
    MSTAT_COMPLETE,   ///< 执行完成（PK_PathExecute）
} PK_ModuleStateType;

typedef enum
{
    LOCAT_SLOWDOWN_FOR,  ///< 前方减速
    LOCAT_SPDUP_FOR,     ///< 前方加速
    LOCAT_SLOWDOWN_BACK, ///< 后方减速
    LOCAT_SPDUP_BACK,    ///< 后方加速
    LOCAT_STILL,         ///< 静止
    LOCAT_CONSTSPD_FOR,  ///< 前方匀速
    LOCAT_CONSTSPD_BACK, ///< 后方匀速
} Vel_MoveMentStType;

typedef enum
{
    PK_SLOT_NO,                ///< 无车位
    PK_SLOT_LEFT_PARA,         ///< 左侧平行车位
    PK_SLOT_RIGHT_PARA,        ///< 右侧平行车位
    PK_SLOT_LEFT_VERT,         ///< 左侧垂直车位
    PK_SLOT_RIGHT_VERT,        ///< 右侧垂直车位
    PK_SLOT_LEFT_ANG_FORWARD,  ///< 左侧斜前方(逆鱼骨)车位
    PK_SLOT_LEFT_ANG_REVERSE,  ///< 左侧斜后方(顺鱼骨)车位
    PK_SLOT_RIGHT_ANG_FORWARD, ///< 右侧斜前方(逆鱼骨)车位
    PK_SLOT_RIGHT_ANG_REVERSE  ///< 右侧斜后方(顺鱼骨)车位
} PK_SlotShapeType;

typedef struct
{
    float NearFront[2];     // 车位前侧近端点坐标 (x, y)
    float NearRear[2];      // 车位后侧近端点坐标 (x, y)
    float FarFront[2];      // 车位前侧远端点坐标 (x, y)
    float FarRear[2];       // 车位后侧远端点坐标 (x, y)
    PK_SlotShapeType shape; // 车位形状类型
    int8_t hasStop;         // 是否有限位杆
    float stopPoint[4];     // 限位杆坐标
    int8_t hasLock;         // 是否有车锁
    uint64_t timestamp;     // 数据时间戳，微秒或毫秒
} AVM_SlotPointsType;

// APA不可恢复中断状态
typedef enum
{
    UrInt_Normal             = 0,
    UrInt_WheelStuck         = 1,
    UrInt_PowerOff           = 2,
    UrInt_IpbBrakingForceErr = 3,
    UrInt_EpsBrakingForceErr = 4,
    UrInt_SlopeOver          = 5,
    UrInt_GearErr            = 6,
    UrInt_EPBErr             = 7
} Apa_UrIntSta;

typedef struct
{
    float OS_TargDistX_ACC;
    float OS_TargDistY_ACC;
    float OS_TargRelVelX_ACC;
    float OS_TargRelVelY_ACC;
    float OS_TargRelAX_ACC;
    float OS_TargRelAY_ACC;
    float OS_TargObstacleProb_ACC;
    float OS_TargExistProb_ACC;
    uint8 OS_TargID_ACC;
    uint8 OS_TargAge_ACC;
    uint8 OS_Type_ACC;
    uint8 V_id;
} RD_OSTargACCType;

typedef struct
{
    uint32_t t;
    float pos[4];
} RD_PosStamp;

// ================= ComIF ======================

// HMI按钮枚举
typedef enum
{
    ButtonState_Invalid  = 0, ///< 无动作，一般处于防抖状态
    ButtonState_IntoAct  = 1, ///< 进入泊车界面
    ButtonState_Continue = 2, ///< 泊车开始/继续
    ButtonState_Suspend  = 3, ///< 泊车暂停(遥控泊车)
    ButtonState_OutPark  = 4, ///< 泊车退出
} HMI_ButtonStateType;

/**
 * @brief HMI的用户按钮信息
 *
 */
typedef struct
{
    int HmiSelectSlotInx;             ///< HMI选中的车位
    bool HmiSwitchSta;                ///< Hmi开关状态
    HMI_ButtonStateType HmiButtonSta; ///< HMI按钮枚举
    bool HmiSinglrSlotSta;            ///< 单边车位状态，项目要求？
    HMI_ApaModeType HmiApaMode;       ///< HMI模式枚举：APA,RPA,泊出
    uint8_t HmiSearchedSlots;         ///< 搜索到车位数（弃用）
    uint8_t HmiRadarAlarm;            ///< 障碍物过近标志位（弃用）
} HMI_ButtonInfo;

/**
 * @brief HMI工作状态
 *
 */
typedef enum
{
    HmiState_Invalid = 0, /**< 非 APA 状态 */
    HmiState_Active  = 1, /**< APA 激活 */
    HmiState_Exit    = 2  /**< 退出 APA */
} HMI_StateType;

/**
 * @brief HMI 工作状态枚举
 *
 */
typedef enum
{
    HmiWorkState_Passive            = 0, /**< 被动状态，系统未激活，APA 功能关闭 */
    HmiWorkState_Serching           = 1, /**< 搜索车位状态 */
    HmiWorkState_GuidanceActive     = 2, /**< 引导中 */
    HmiWorkState_GuidanceSuspend    = 3, /**< 引导暂停，由可恢复中断导致 */
    HmiWorkState_GuidanceTerminated = 4, /**< 引导终止，由不可恢复中断导致 */
    HmiWorkState_GuidanceCompleted  = 5, /**< 引导完成 */
    HmiWorkState_Failure            = 6, /**< 系统故障，APA 功能无法开启 */
    HmiWorkState_ParkAssist_Standby = 7, /**< 辅助待机，等待关联件握手 */
} HMI_WorkStateType;

/**
 * @brief 工作信息显示
 *
 */
typedef enum
{
    ApaWorkingText_NoInfo          = 0, // 无信息显示
    ApaWorkingText_Serching_NoPark = 1, // searching 请向前行驶搜索车位...
    ApaWorkingText_Serching_Park   = 2, // 2 searching  已找到车位，如需泊入请踩刹车
    ApaWorkingText_Serching_NoticePG_Brake =
        3, //     3 searching  请踩住刹车，并点击开始泊入
    ApaWorkingText_Parking_RealseBrakeToStart =
        4,                                //  4 activity  泊车开始，请松开刹车和方向盘
    ApaWorkingText_IsParking         = 5, // 5 activity  泊车中，请注意周围环境安全
    ApaWorkingText_GuidanceCompleted = 6, // 6 complete  泊车已完成
    ApaWorkingText_RadarToNearAlarm =
        7, // 7  searching，activity pause 车辆距离障碍物太近，请停车
    ApaWorkingText_Parking_RealseBrake = 8,  // 8 activity  泊车中，请松开刹车和方向盘
    ApaWorkingText_ParkOut_Select      = 9,  // 9  searching 请选择泊出方向
    ApaWorkingText_GuidanceStop        = 10, // 10  activity 泊车失败
    ApaWorkingText_Serching_NoticePG_NoBrake = 11, // 11 activity 溜车
    ApaWorkingText_CloseFrontHatchCover      = 12, // 12 searching，pause 请关闭前舱盖
    ApaWorkingText_CloseTrunk                = 13, // 13 searching，pause 请关闭后备箱
    ApaWorkingText_CloseDoor                 = 14, // 14 searching，pause 请关闭车门
    ApaWorkingText_OpenRearviewMirror        = 15, // 15 searching，pause 请打开后视镜
    ApaWorkingText_SeatBeltNotFastened       = 16, // 16 pause 安全带未系
    ApaWorkingText_DriverIntervent           = 17, // 17 pause/fail 人为干预
    ApaWorkingText_GuidanceSuspend           = 18, // 18 pause 泊车暂停
    ApaWorkingText_FastenSeatAndStartParking = 19, // 19 searching 系好安全带并开始泊车
    ApaWorkingText_UltronicFail =
        20, // 20 searching，activity超声波雷达故障   未使用，等待超声波提供状态
    ApaWorkingText_ReduceSpdInSerch = 21, // 21 searching 车速过快
    ApaWorkingText_Resever
} HMI_WorkTextInfo;

typedef struct
{
    uint32_t simpleTime;
    uint16_t fDirect;
    uint8_t peak;
    uint8_t width;
    uint8_t status;
    uint8_t ret2nd;
} Eth_Radar_Data;

/**
 * @brief 超声波雷达探头布局示意图 (俯视)
 *
 *                  车头
 *
 *      [FOL]   [FCL]   [FCR]   [FOR]
 *         |       |       |       |
 *         +-----------------------+
 *         |                       |
 *   [FSL] |                       | [FSR]
 *         |                       |
 *         |                       |
 *         |                       |
 *         |                       |
 *         |                       |
 *   [RSL] |                       | [RSR]
 *         +-----------------------+
 *         |       |       |       |
 *      [ROL]   [RCL]   [RCR]   [ROR]
 *
 *                   车尾
 *
 * 说明：
 * - FOL/FOR: 前左外 / 前右外
 * - FCL/FCR: 前左中 / 前右中
 * - ROL/ROR: 后左外 / 后右外
 * - RCL/RCR: 后左中 / 后右中
 * - FSL/FSR: 前左侧 / 前右侧
 * - RSL/RSR: 后左侧 / 后右侧
 * - *_2     : 二次回波
 * - X_Y     : 探头交叉回波
 */
typedef struct
{
    float FOL; ///< Front Outer Left，前左外探头
    float FCL; ///< Front Center Left，前左中探头
    float FCR; ///< Front Center Right，前右中探头
    float FOR; ///< Front Outer Right，前右外探头

    float ROL; ///< Rear Outer Left，后左外探头
    float RCL; ///< Rear Center Left，后左中探头
    float RCR; ///< Rear Center Right，后右中探头
    float ROR; ///< Rear Outer Right，后右外探头

    float FSR; ///< Front Side Right，前右侧探头
    float FSL; ///< Front Side Left，前左侧探头
    float RSL; ///< Rear Side Left，后左侧探头
    float RSR; ///< Rear Side Right，后右侧探头

    float FSR_2; ///< Front Side Right Second，前右侧二次回波
    float FSL_2; ///< Front Side Left Second，前左侧二次回波
    float RSL_2; ///< Rear Side Left Second，后左侧二次回波
    float RSR_2; ///< Rear Side Right Second，后右侧二次回波

    float FCL_FOL; ///< 前左中与前左外交叉回波
    float FCL_FCR; ///< 前左中与前右中交叉回波
    float FCR_FOR; ///< 前右中与前右外交叉回波
    float FCR_FCL; ///< 前右中与前左中交叉回波
    float RCL_ROL; ///< 后左中与后左外交叉回波
    float RCL_RCR; ///< 后左中与后右中交叉回波
    float RCR_ROR; ///< 后右中与后右外交叉回波
    float RCR_RCL; ///< 后右中与后左中交叉回波
} U_RadarType;

typedef struct
{
    uint32_t Timestamp;
    uint16_t Distance;
    uint8_t PeakLevel;
    uint8_t WaveWidth;
} U_RadarDataType;

typedef struct
{
    uint32_t Timestamp;
    uint16_t Distance;
    uint8_t Status;
    int8_t Ret2nd;
} U_RadarInfoType;

enum
{
    Radar_ID_FSR = 0,
    Radar_ID_FSL,
    Radar_ID_RSL,
    Radar_ID_RSR,
    Radar_ID_FOL,
    Radar_ID_FCL,
    Radar_ID_FCR,
    Radar_ID_FOR,
    Radar_ID_ROL,
    Radar_ID_RCL,
    Radar_ID_RCR,
    Radar_ID_ROR,
    Radar_ID_MAX,
};

// HMI button
typedef enum
{
    VisionState_Alive   = 0,
    VisionState_Invalid = 1
} VisionState;

#endif

// End of file
