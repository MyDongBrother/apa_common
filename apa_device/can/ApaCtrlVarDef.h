/*
 * ApaCtrlVarDef.h
 *
 *  Created on: 2021年8月27日
 *      Author: longyunx
 */

#ifndef APACTRL_APACTRLVARDEF_H_
#define APACTRL_APACTRLVARDEF_H_

//////////////test//////////////////
#define Test_Data 0
////////////////////////////////////

enum
{
    EPS_APA_TemporaryInhibited = 0,
    EPS_APA_Availableforcontrol,
    EPS_APA_Active,
    EPS_APA_PermanentInhibited,
    EPS_APA_Reserved
};

typedef enum
{
    NO_Request_For_EPS = 0,
    Request_For_EPS
} ApaStrCtrlReqForEPS;

typedef enum
{
    APA_Passive = 0,
    APA_Serching,
    APA_GuidanceActive,
    APA_GuidanceSuspend,
    APA_GuidanceTerminated,
    APA_GuidanceCompleted,
    APA_Failure,
    APA_ParkAssistStandby
} ApaMainStas;

typedef enum
{
    EPS_Off = 0,
    EPS_On,
    POWER_UP_Initialization,
    Temporary_Inhibited,
    Permanent_Inhibited,
    Available_for_Control,
    Active,
} ApaTranStatus;

typedef enum
{
    IDLE = 0,
    APA,
    RPA,
    RESERVED
} ParkingMode;

typedef enum
{
    ESC_APA_Off = 0,
    ESC_APA_Standby,
    ESC_APA_Active_AutomaticPark,
    ESC_APA_Error_The_others_Reserved,
    ESC_APA_Reserved1
} EscApaStatus;

#define GetSysPowerStatus()    Test_Data //电源档位
#define GetApaCanSignIsValid() Test_Data // APA 相关的 CAN
                                         // 信号有效（APA，SAS，车速，档位）
#define Init_Target_Angle         7800      // Init targe angle
#define GetApaInitTargeAngleVal() Test_Data // Init targe angle
#define GetApaStopCmd()           Test_Data //泊车停止命令
#define GetAbstacleExist()        Test_Data //障碍物存在
#define GetCarDoorOpen()          Test_Data //车门开
#define GetBrakeStepOn()          Test_Data //刹车踏板踩下
#define GetOutsideMirrorFold()    Test_Data //外后视镜折叠
#define GetSafeBeltRelease()      Test_Data //安全带松开
#define GetBlueToothDisconnect()  Test_Data //蓝牙连接断开（未超时）

#endif /* APACTRL_APACTRLVARDEF_H_ */
