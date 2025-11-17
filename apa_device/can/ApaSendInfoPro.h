/*
 * ApaSendInfoPro.h
 *
 *  Created on: 2021年8月9日
 *      Author: longyunx
 */

#ifndef APASENDINFOPRO_H_
#define APASENDINFOPRO_H_
#include "Rte_Types.h"

enum
{
    CANID_INDEX_134 = 0,
    CANID_INDEX_13F,
    CANID_INDEX_1D1,
    CANID_INDEX_207,
    CANID_INDEX_267,
    CANID_INDEX_308,
    CANID_INDEX_31A,
    CANID_INDEX_365,
    CANID_INDEX_454,
    CANID_INDEX_4F4,
    CANID_INDEX_526,
    CANID_INDEX_MAX
};

#define CANID_TX_LIST \
    0x134, 0x13F, 0x1D1, 0x207, 0x267, 0x308, 0x31A, 0x365, 0x454, 0x4F4, 0x526,

typedef struct
{
    uint16_t CanId[CANID_INDEX_MAX];
    uint8_t data[CANID_INDEX_MAX][8];
} ApaCanTxInterface;

typedef struct
{
    uint8_t PANORAMA_SUBID;                        /* 子ID */
    uint8_t PANORAMA_CURR_WORK_TYPE : 4;           /* 目前全景处于哪种工作状态 */
    uint8_t PANORAMA_WORK_MODE : 4;                /* 全景工作模式 */
    uint8_t PANORAMA_CURR_WORK_STATUS : 2;         /* 全景工作状态 */
    uint8_t PANORAMA_VIEW_SCREEN_MODE : 2;         /* 全景视频模式 */
    uint8_t PANORAMA_ROTATE_SATUS : 2;             /* 旋转屏状态 */
    uint8_t PANORAMA_LUCENCY_MODE : 2;             /* 全景透明模式 */
    uint8_t PANORAMA_SEC_SET_STATUS : 2;           /* sec倒车线配置状态 */
    uint8_t PANORAMA_LVDS_OUTPUT_MODE : 3;         /* LVDS信号输出模式 */
    uint8_t PANORAMA_IMAGE_SET : 3;                /* 影像配置 */
    uint8_t PANORAMA_CARBODY_STATUS : 2;           /* 车体状态 */
    uint8_t PANORAMA_CAMERA_INCAR_CHECK : 2;       /* 车内监控摄像头检测 */
    uint8_t PANORAMA_SUPPORT_REMOTE_CALL_FALG : 2; /* 是否支持远程调用 */
    uint8_t Alive_Rolling_counter : 4;
    uint8_t Checksum;
} ApaCanTxInfo4F4;

typedef struct
{
    uint8_t APA_MainStas : 3;
    uint8_t APA_SpeedLimit : 8;
    uint8_t APA_SteeringCtrlReqForEPS : 1;
    uint8_t APA_SteeringCtrlReqForEPSVD : 1;
    uint8_t APA_FailureBrakeMode : 2;
    uint8_t APA_RequestEpsValidFlag : 1;
    uint16_t APA_Tar_StrAgl;
    uint16_t APA_StopDistance;
    uint8_t EPB_EmergencyBrake : 1;
    uint8_t EPB_EmergencyBrakeValid : 1;
    uint8_t APS_SCU_TargetGearRequest : 4;
    uint8_t APA_AutoStopMode : 2;
    uint8_t Alive_Rolling_counter : 4;
    uint8_t APA_Checksum;
} ApaCanTxInfo134;

typedef struct
{
    uint8_t APA_LSMBrakeEmgcyPrep_S : 1;
    uint8_t APA_LSMSubMTReq_S : 2;
    uint8_t APA_LSMSubMTLevel_S : 2;
    uint8_t APA_LSMNudgeReq_S : 1;
    uint8_t APA_LSMVehSecReq_S : 1;
    uint8_t APA_LSMVehDirRq_S : 2;
    uint8_t APA_LSMComfBrakeReq_S : 2;
    uint16_t APA_LSMDistToStop_S : 12;
    uint8_t APA_SafeDrvrHandoverReq_S : 1;
    uint8_t AVM_DVR_Sts_S : 3;
    uint8_t APA_MEB_States_S : 4;
    uint8_t APA_RMEB_States_S : 4;
    uint8_t DVR_WorkingMode_S : 2;
    uint8_t APA_LSMSubMTLong_S : 2;
    uint8_t APA_LSMvMaxRq_S;
    uint8_t Scroll_Cycle_Cnt_13F_S : 4;
    uint8_t Checksum_13F_S;
} ApaCanTxInfo13F;

typedef struct
{
    uint8_t AVM_APA_SuspendedReason : 4;
    uint8_t AVM_APA_ComunicationInfo : 4;
    uint8_t AVM_APA_AutoSearchSts : 3;
    uint8_t AVM_APA_AbortReason : 5;
    uint8_t AVM_APA_SavePositionMemory : 3;
    uint8_t AVM_APA_Active : 2;
    uint8_t AVM_APA_AutoStopType : 4;
    uint8_t AVM_APA_AutoStopSwitchStatus : 2;
    uint8_t AVM_APA_CurrentPathNum : 4;
    uint8_t AVM_APA_TotalPathNum : 4;
    uint8_t AVM_APA_CurrentStepNum : 4;
    uint8_t AVM_APA_SupportRpaFlag : 2;
    uint8_t Alive_Rolling_counter : 4;
    uint8_t APA_Checksum;
} ApaCanTxInfo1D1;

extern ApaCanTxInterface ApaCanTxDataBuff;     /* APA CAN DATA OUTPUT*/
extern ApaCanTxInfo4F4 ApaSendPanoramaInfo;    /* 全景影像功能 */
extern ApaCanTxInfo134 ApaSendAutoStopInfo134; /* 自动泊车功能0x134 */
extern ApaCanTxInfo1D1 ApaSendAutoStopInfo1d1; /* 自动泊车功能0x1d1 */
extern ApaCanTxInfo13F ApaSendAutoStopInfo13F; /* 自动泊车功能0x13F */

extern void ApaDataInit(void);
extern void ApaDataPro(void);
#endif /* APASENDINFOPRO_H_ */
