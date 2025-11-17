/*
 * ApaSendInfoPro1.c
 *
 *  Created on: 2021年8月10日
 *      Author: longyunx
 */
#include "ApaSendInfoPro.h"
#include "EthIf.h"

const uint16_t ApaTxCanIdTable[CANID_INDEX_MAX] = {CANID_TX_LIST};

ApaCanTxInterface ApaCanTxDataBuff; /* APA CAN DATA OUTPUT*/

ApaCanTxInfo4F4 ApaSendPanoramaInfo;    /* 全景影像功能 */
ApaCanTxInfo134 ApaSendAutoStopInfo134; /* 自动泊车功能0x134 */
ApaCanTxInfo1D1 ApaSendAutoStopInfo1d1; /* 自动泊车功能0x1d1 */
ApaCanTxInfo13F ApaSendAutoStopInfo13F;

static uint8_t ApaCanTxData[CANID_INDEX_MAX][8] = {0};

static void ApaSendInfo_Id3D0(void);
static void ApaSendInfo_Id4F4(void);
static void ApaSendInfo_Id134(void);
static void ApaSendInfo_Id1D1(void);
static void ApaSendInfo_Id13F(void);

void ApaDataInit(void);
void ApaDataPro(void);
uint8_t CanSumCheck(uint8_t *CanDate, uint8_t length);

/*====================================================================
 * Project:
 * Function:
 *====================================================================*/
uint8_t CanSumCheck(uint8_t *CanDate, uint8_t length)
{
    uint8_t i;
    uint8_t retSum;

    retSum = 0;
    for (i = 0; i < length - 1; i++)
    {
        retSum += CanDate[i];
    }
    retSum ^= 0xFF;

    return retSum;
}

/*====================================================================
 * Project:
 * Function:
 *====================================================================*/
void ApaDataInit(void)
{
    uint8_t i, j;

    for (i = 0; i < CANID_INDEX_MAX; i++)
    {
        ApaCanTxDataBuff.CanId[i] = ApaTxCanIdTable[i];
        ;
        for (j = 0; j < 8; j++)
        {
            ApaCanTxDataBuff.data[i][j] = 0x00;
            ApaCanTxData[i][j]          = 0x00;
        }
    }

    /* ApaSendPanoramaInfo data init */
    ApaSendPanoramaInfo.PANORAMA_SUBID              = 0;
    ApaSendPanoramaInfo.PANORAMA_CURR_WORK_TYPE     = 0; /* 目前全景处于哪种工作状态 */
    ApaSendPanoramaInfo.PANORAMA_WORK_MODE          = 0; /* 全景工作模式 */
    ApaSendPanoramaInfo.PANORAMA_CURR_WORK_STATUS   = 0; /* 全景工作状态 */
    ApaSendPanoramaInfo.PANORAMA_VIEW_SCREEN_MODE   = 0; /* 全景视频模式 */
    ApaSendPanoramaInfo.PANORAMA_ROTATE_SATUS       = 0; /* 旋转屏状态 */
    ApaSendPanoramaInfo.PANORAMA_LUCENCY_MODE       = 0; /* 全景透明模式 */
    ApaSendPanoramaInfo.PANORAMA_SEC_SET_STATUS     = 0; /* sec倒车线配置状态 */
    ApaSendPanoramaInfo.PANORAMA_LVDS_OUTPUT_MODE   = 0; /* LVDS信号输出模式 */
    ApaSendPanoramaInfo.PANORAMA_IMAGE_SET          = 0; /* 影像配置 */
    ApaSendPanoramaInfo.PANORAMA_CARBODY_STATUS     = 0; /* 车体状态 */
    ApaSendPanoramaInfo.PANORAMA_CAMERA_INCAR_CHECK = 0; /* 车内监控摄像头检测 */
    ApaSendPanoramaInfo.PANORAMA_SUPPORT_REMOTE_CALL_FALG = 0; /* 是否支持远程调用 */
    ApaSendPanoramaInfo.Alive_Rolling_counter             = 0;
    ApaSendPanoramaInfo.Checksum                          = 0;

    /* ApaSendAutoStopInfo134 data init */
    ApaSendAutoStopInfo134.APA_MainStas                = 0;
    ApaSendAutoStopInfo134.APA_SpeedLimit              = 0;
    ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = 0;
    ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = 0;
    ApaSendAutoStopInfo134.APA_FailureBrakeMode        = 0;
    ApaSendAutoStopInfo134.APA_RequestEpsValidFlag     = 0;
    ApaSendAutoStopInfo134.APA_Tar_StrAgl              = 0;
    ApaSendAutoStopInfo134.APA_StopDistance            = 0;
    ApaSendAutoStopInfo134.EPB_EmergencyBrake          = 0;
    ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid     = 0;
    ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest   = 0;
    ApaSendAutoStopInfo134.APA_AutoStopMode            = 0;
    ApaSendAutoStopInfo134.Alive_Rolling_counter       = 0;
    ApaSendAutoStopInfo134.APA_Checksum                = 0;

    /* ApaSendAutoStopInfo1d1 data init */
    ApaSendAutoStopInfo1d1.AVM_APA_SuspendedReason      = 0;
    ApaSendAutoStopInfo1d1.AVM_APA_ComunicationInfo     = 0;
    ApaSendAutoStopInfo1d1.AVM_APA_AutoSearchSts        = 0;
    ApaSendAutoStopInfo1d1.AVM_APA_AbortReason          = 0;
    ApaSendAutoStopInfo1d1.AVM_APA_SavePositionMemory   = 0;
    ApaSendAutoStopInfo1d1.AVM_APA_Active               = 0;
    ApaSendAutoStopInfo1d1.AVM_APA_AutoStopType         = 0;
    ApaSendAutoStopInfo1d1.AVM_APA_AutoStopSwitchStatus = 0;
    ApaSendAutoStopInfo1d1.AVM_APA_CurrentPathNum       = 0;
    ApaSendAutoStopInfo1d1.AVM_APA_TotalPathNum         = 0;
    ApaSendAutoStopInfo1d1.AVM_APA_CurrentStepNum       = 0;
    ApaSendAutoStopInfo1d1.AVM_APA_SupportRpaFlag       = 0;
    ApaSendAutoStopInfo1d1.Alive_Rolling_counter        = 0;
    ApaSendAutoStopInfo1d1.APA_Checksum                 = 0;
}

/*====================================================================
 * Project:
 * Function:
 *====================================================================*/
static void ApaSendInfo_Id4F4(void)
{
    ApaSendPanoramaInfo.Alive_Rolling_counter++;
    /* Intel */
    ApaCanTxData[CANID_INDEX_4F4][0] = ApaSendPanoramaInfo.PANORAMA_SUBID;

    ApaCanTxData[CANID_INDEX_4F4][1] = ApaSendPanoramaInfo.PANORAMA_WORK_MODE;
    ApaCanTxData[CANID_INDEX_4F4][1] <<= 4;
    ApaCanTxData[CANID_INDEX_4F4][1] += ApaSendPanoramaInfo.PANORAMA_CURR_WORK_TYPE;

    ApaCanTxData[CANID_INDEX_4F4][2] = ApaSendPanoramaInfo.PANORAMA_LUCENCY_MODE;
    ApaCanTxData[CANID_INDEX_4F4][2] <<= 2;
    ApaCanTxData[CANID_INDEX_4F4][2] += ApaSendPanoramaInfo.PANORAMA_ROTATE_SATUS;
    ApaCanTxData[CANID_INDEX_4F4][2] <<= 2;
    ApaCanTxData[CANID_INDEX_4F4][2] += ApaSendPanoramaInfo.PANORAMA_VIEW_SCREEN_MODE;
    ApaCanTxData[CANID_INDEX_4F4][2] <<= 2;
    ApaCanTxData[CANID_INDEX_4F4][2] += ApaSendPanoramaInfo.PANORAMA_CURR_WORK_STATUS;

    ApaCanTxData[CANID_INDEX_4F4][3] = ApaSendPanoramaInfo.PANORAMA_IMAGE_SET;
    ApaCanTxData[CANID_INDEX_4F4][3] <<= 3;
    ApaCanTxData[CANID_INDEX_4F4][3] += ApaSendPanoramaInfo.PANORAMA_LVDS_OUTPUT_MODE;
    ApaCanTxData[CANID_INDEX_4F4][3] <<= 3;
    ApaCanTxData[CANID_INDEX_4F4][3] += ApaSendPanoramaInfo.PANORAMA_SEC_SET_STATUS;

    ApaCanTxData[CANID_INDEX_4F4][4] =
        (uint8_t)ApaSendPanoramaInfo.PANORAMA_CARBODY_STATUS;
    ApaCanTxData[CANID_INDEX_4F4][4] <<= 2;
    ApaCanTxData[CANID_INDEX_4F4][4] += ApaSendPanoramaInfo.PANORAMA_CAMERA_INCAR_CHECK;
    ApaCanTxData[CANID_INDEX_4F4][4] <<= 2;
    ApaCanTxData[CANID_INDEX_4F4][4] += ApaSendPanoramaInfo.PANORAMA_CARBODY_STATUS;

    // ApaCanTxData[CANID_INDEX_4F4][5] = 0x00;

    ApaCanTxData[CANID_INDEX_4F4][6] = ApaSendPanoramaInfo.Alive_Rolling_counter;
    ApaCanTxData[CANID_INDEX_4F4][6] <<= 4;

    ApaCanTxData[CANID_INDEX_4F4][7] = CanSumCheck(&ApaCanTxData[CANID_INDEX_4F4][0], 8);
    ApaSendPanoramaInfo.Checksum     = ApaCanTxData[CANID_INDEX_4F4][7];
}

/*====================================================================
 * Project:
 * Function:
 *====================================================================*/
static void ApaSendInfo_Id134(void)
{
    ApaSendAutoStopInfo134.Alive_Rolling_counter++;
    /* Intel */
    ApaCanTxData[CANID_INDEX_134][0] = ((ApaSendAutoStopInfo134.APA_SpeedLimit) & 0x1F);
    ApaCanTxData[CANID_INDEX_134][0] <<= 3;
    ApaCanTxData[CANID_INDEX_134][0] += ApaSendAutoStopInfo134.APA_MainStas;

    ApaCanTxData[CANID_INDEX_134][1] = ApaSendAutoStopInfo134.APA_RequestEpsValidFlag;
    ApaCanTxData[CANID_INDEX_134][1] <<= 2;
    ApaCanTxData[CANID_INDEX_134][1] += ApaSendAutoStopInfo134.APA_FailureBrakeMode;
    ApaCanTxData[CANID_INDEX_134][1] <<= 1;
    ApaCanTxData[CANID_INDEX_134][1] +=
        ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD;
    ApaCanTxData[CANID_INDEX_134][1] <<= 1;
    ApaCanTxData[CANID_INDEX_134][1] += ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS;
    ApaCanTxData[CANID_INDEX_134][1] <<= 3;
    ApaCanTxData[CANID_INDEX_134][1] +=
        ((ApaSendAutoStopInfo134.APA_SpeedLimit >> 5) & 0x07);

    // printf("ApaSendAutoStopInfo134.APA_Tar_StrAgl %d\r\n",
    // ApaSendAutoStopInfo134.APA_Tar_StrAgl);
    ApaCanTxData[CANID_INDEX_134][2] = (uint8_t)ApaSendAutoStopInfo134.APA_Tar_StrAgl;
    ApaCanTxData[CANID_INDEX_134][3] =
        (uint8_t)(ApaSendAutoStopInfo134.APA_Tar_StrAgl >> 8);

    ApaCanTxData[CANID_INDEX_134][4] = (uint8_t)ApaSendAutoStopInfo134.APA_StopDistance;

    ApaCanTxData[CANID_INDEX_134][5] =
        (ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest) & 0x03;
    ApaCanTxData[CANID_INDEX_134][5] <<= 1;
    ApaCanTxData[CANID_INDEX_134][5] += ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid;
    ApaCanTxData[CANID_INDEX_134][5] <<= 1;
    ApaCanTxData[CANID_INDEX_134][5] += ApaSendAutoStopInfo134.EPB_EmergencyBrake;
    ApaCanTxData[CANID_INDEX_134][5] <<= 4;
    ApaCanTxData[CANID_INDEX_134][5] +=
        (uint8_t)((ApaSendAutoStopInfo134.APA_StopDistance >> 8) & 0x000F);

    ApaCanTxData[CANID_INDEX_134][6] = ApaSendAutoStopInfo134.Alive_Rolling_counter;
    ApaCanTxData[CANID_INDEX_134][6] <<= 2;
    ApaCanTxData[CANID_INDEX_134][6] += ApaSendAutoStopInfo134.APA_AutoStopMode;
    ApaCanTxData[CANID_INDEX_134][6] <<= 2;
    ApaCanTxData[CANID_INDEX_134][6] +=
        (ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest >> 2) & 0x03;

    ApaCanTxData[CANID_INDEX_134][7] = CanSumCheck(&ApaCanTxData[CANID_INDEX_134][0], 8);
    ApaSendAutoStopInfo134.APA_Checksum = ApaCanTxData[CANID_INDEX_134][7];

    // printf("------------------Can 134:
    // %d,%d=====%d--%d---n:%d--\r\n",dataBuf[0],dataBuf[1],dataBuf[2],ApaCanTxData[CANID_INDEX_134][0],dataBuf[8]);
}

/*====================================================================
 * Project:
 * Function:
 *====================================================================*/
static void ApaSendInfo_Id1D1(void)
{
    ApaSendAutoStopInfo1d1.Alive_Rolling_counter++;
    /* Intel */
    ApaCanTxData[CANID_INDEX_1D1][0] = ApaSendAutoStopInfo1d1.AVM_APA_ComunicationInfo;
    ApaCanTxData[CANID_INDEX_1D1][0] <<= 4;
    ApaCanTxData[CANID_INDEX_1D1][0] += ApaSendAutoStopInfo1d1.AVM_APA_SuspendedReason;

    ApaCanTxData[CANID_INDEX_1D1][1] = ApaSendAutoStopInfo1d1.AVM_APA_AbortReason;
    ApaCanTxData[CANID_INDEX_1D1][1] <<= 3;
    ApaCanTxData[CANID_INDEX_1D1][1] += ApaSendAutoStopInfo1d1.AVM_APA_AutoSearchSts;

    ApaCanTxData[CANID_INDEX_1D1][2] = ApaSendAutoStopInfo1d1.AVM_APA_SavePositionMemory;

    ApaCanTxData[CANID_INDEX_1D1][3] =
        ApaSendAutoStopInfo1d1.AVM_APA_AutoStopSwitchStatus;
    ApaCanTxData[CANID_INDEX_1D1][3] <<= 4;
    ApaCanTxData[CANID_INDEX_1D1][3] += ApaSendAutoStopInfo1d1.AVM_APA_AutoStopType;
    ApaCanTxData[CANID_INDEX_1D1][3] <<= 2;
    ApaCanTxData[CANID_INDEX_1D1][3] += ApaSendAutoStopInfo1d1.AVM_APA_Active;

    ApaCanTxData[CANID_INDEX_1D1][4] = ApaSendAutoStopInfo1d1.AVM_APA_TotalPathNum;
    ApaCanTxData[CANID_INDEX_1D1][4] <<= 4;
    ApaCanTxData[CANID_INDEX_1D1][4] += ApaSendAutoStopInfo1d1.AVM_APA_CurrentPathNum;

    ApaCanTxData[CANID_INDEX_1D1][5] = ApaSendAutoStopInfo1d1.AVM_APA_SupportRpaFlag;
    ApaCanTxData[CANID_INDEX_1D1][5] <<= 4;
    ApaCanTxData[CANID_INDEX_1D1][5] += ApaSendAutoStopInfo1d1.AVM_APA_CurrentStepNum;

    ApaCanTxData[CANID_INDEX_1D1][6] = ApaSendAutoStopInfo1d1.Alive_Rolling_counter;
    ApaCanTxData[CANID_INDEX_1D1][6] <<= 4;

    ApaCanTxData[CANID_INDEX_1D1][7] = CanSumCheck(&ApaCanTxData[CANID_INDEX_1D1][0], 8);
    ApaSendAutoStopInfo1d1.APA_Checksum = ApaCanTxData[CANID_INDEX_1D1][7];
}

/*====================================================================
 * Project:
 * Function:
 *====================================================================*/
static void ApaSendInfo_Id13F(void)
{
    ApaSendAutoStopInfo13F.Scroll_Cycle_Cnt_13F_S++;
    /* Intel */
    ApaCanTxData[CANID_INDEX_13F][0] =
        ApaSendAutoStopInfo13F.APA_LSMSubMTReq_S; // 1.6~1.7
    ApaCanTxData[CANID_INDEX_13F][0] <<= 6;
    ApaCanTxData[CANID_INDEX_13F][0] +=
        (ApaSendAutoStopInfo13F.APA_LSMBrakeEmgcyPrep_S << 5); // 1.5

    ApaCanTxData[CANID_INDEX_13F][1] =
        ApaSendAutoStopInfo13F.APA_LSMComfBrakeReq_S; // 2.6~2.7
    ApaCanTxData[CANID_INDEX_13F][1] <<= 2;
    ApaCanTxData[CANID_INDEX_13F][1] +=
        ApaSendAutoStopInfo13F.APA_LSMVehDirRq_S; // 2.4~2.5
    ApaCanTxData[CANID_INDEX_13F][1] <<= 2;
    ApaCanTxData[CANID_INDEX_13F][1] += ApaSendAutoStopInfo13F.APA_LSMVehSecReq_S; // 2.3
    ApaCanTxData[CANID_INDEX_13F][1] <<= 1;
    ApaCanTxData[CANID_INDEX_13F][1] += ApaSendAutoStopInfo13F.APA_LSMNudgeReq_S; // 2.2
    ApaCanTxData[CANID_INDEX_13F][1] <<= 1;
    ApaCanTxData[CANID_INDEX_13F][1] +=
        ApaSendAutoStopInfo13F.APA_LSMSubMTLevel_S; // 2.0~2.1

    ApaCanTxData[CANID_INDEX_13F][2] =
        (uint8_t)ApaSendAutoStopInfo13F.APA_LSMDistToStop_S; // 3.0~4.3

    ApaCanTxData[CANID_INDEX_13F][3] = ApaSendAutoStopInfo13F.AVM_DVR_Sts_S; // 4.5~4.7
    ApaCanTxData[CANID_INDEX_13F][3] <<= 3;
    ApaCanTxData[CANID_INDEX_13F][3] +=
        ApaSendAutoStopInfo13F.APA_SafeDrvrHandoverReq_S; // 4.4
    ApaCanTxData[CANID_INDEX_13F][3] <<= 1;
    ApaCanTxData[CANID_INDEX_13F][3] +=
        (uint8_t)(ApaSendAutoStopInfo13F.APA_LSMDistToStop_S >> 8); // 3.0~4.3

    ApaCanTxData[CANID_INDEX_13F][4] =
        (uint8_t)ApaSendAutoStopInfo13F.APA_RMEB_States_S; // 5.4~5.7
    ApaCanTxData[CANID_INDEX_13F][4] <<= 4;
    ApaCanTxData[CANID_INDEX_13F][4] +=
        (uint8_t)ApaSendAutoStopInfo13F.APA_MEB_States_S; // 5.0~5.3

    ApaCanTxData[CANID_INDEX_13F][5] =
        (uint8_t)(ApaSendAutoStopInfo13F.APA_LSMvMaxRq_S & 0x0F); // 6.4~7.3
    ApaCanTxData[CANID_INDEX_13F][5] <<= 2;
    ApaCanTxData[CANID_INDEX_13F][5] +=
        ApaSendAutoStopInfo13F.APA_LSMSubMTLong_S; // 6.3~6.2
    ApaCanTxData[CANID_INDEX_13F][5] <<= 2;
    ApaCanTxData[CANID_INDEX_13F][5] +=
        ApaSendAutoStopInfo13F.DVR_WorkingMode_S; // 6.0~6.1

    ApaCanTxData[CANID_INDEX_13F][6] =
        (uint8_t)ApaSendAutoStopInfo13F.Scroll_Cycle_Cnt_13F_S; // 7.4~7.7
    ApaCanTxData[CANID_INDEX_13F][6] <<= 4;
    ApaCanTxData[CANID_INDEX_13F][6] +=
        (uint8_t)(ApaSendAutoStopInfo13F.APA_LSMvMaxRq_S >> 4); // 6.4~7.3

    ApaCanTxData[CANID_INDEX_13F][7] = CanSumCheck(&ApaCanTxData[CANID_INDEX_13F][0], 8);
    ApaSendAutoStopInfo13F.Checksum_13F_S = ApaCanTxData[CANID_INDEX_13F][7];
}

/*====================================================================
 * Project:
 * Function:
 *====================================================================*/
static void ApaTakeDataToBuff(void)
{
    uint8_t i, j;

    for (i = 0; i < CANID_INDEX_MAX; i++)
    {
        ApaCanTxDataBuff.CanId[i] = ApaTxCanIdTable[i];
        for (j = 0; j < 8; j++)
        {
            ApaCanTxDataBuff.data[i][j] = ApaCanTxData[i][j];
        }
    }
}

/*====================================================================
 * Project:
 * Function:
 *====================================================================*/
void ApaDataPro(void)
{
    ApaSendInfo_Id4F4();
    ApaSendInfo_Id134();
    ApaSendInfo_Id1D1();
    ApaSendInfo_Id13F();

    ApaTakeDataToBuff();
}
