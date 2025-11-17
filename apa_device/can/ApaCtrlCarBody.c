/*
 * ApaCtrlCarBody.c
 *
 *  Created on: 2021??ê9????1????
 *      Author: longyunx
 */
#include "ApaCtrlCarBody.h"
#include "ApaSendInfoPro.h"
#include "PK_Calibration.h"
#include "Rte_Types.h"
#include "SpiIf.h"
#include "EthIf.h"
#include "UartIf.h"
#include <time.h>
#include "PK_StateManage.h"
#include "hobotlog/hobotlog.hpp"

extern uint8_t ApaPathActFlg;
extern uint8_t ApaAngelActFlg;

static void ApaOutHandShakeStatus(void);
static void ApaEpsHandleJudge(void);
static void ApaIpbHandleJudge(void);

static uint8_t ApaCurrentGear()
{
    uint8_t curGear = RTE_BSW_Get_CurrentGear();
    if (curGear == GEAR_REAL_D)
    {
        return GEAR_D;
    }
    else if (curGear == GEAR_REAL_R)
    {
        return GEAR_R;
    }
    else
    {
        return GEAR_P;
    }
}

/*====================================================================
 * Project:
 * Function:
 *====================================================================*/
static void ApaIpbHandleJudge(void)
{
    // static uint8_t ApaModeConp100ms = 0;
    // static uint8_t ApaModeStandby200ms = 10;
    // static Apa_WorkStateType  PreStaManRet = CarSta_PowerOn;
    // static uint8_t ApaHandleOutCnt = 0;
    Apa_WorkStateType staManRet = CarSta_PowerOn;

    const uint8_t APA_LSMSubMTReq_S  = 0x2;
    const uint8_t APA_LSMSubMTLong_S = 0x1;
    uint8_t APA_LSMSubMTLevel_S      = 0x1;
    uint8_t apaMode                  = APA_MODE; // RTE_PK_SM_GetApaMode();
    if (apaMode == APA_MODE)
    {
        APA_LSMSubMTLevel_S = 0x1;
    }
    else if (apaMode == RPA_MODE)
    {
        APA_LSMSubMTLevel_S = 0x2;
    }
    else if (apaMode == AVP_MODE)
    {
        APA_LSMSubMTLevel_S = 0x3;
    }

    staManRet = RTE_SM_Get_ApaWorkState();
    switch (staManRet)
    {
        case CarSta_PowerOn:
        case CarSta_Passive:
            ApaSendAutoStopInfo13F.APA_LSMBrakeEmgcyPrep_S   = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTReq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTLevel_S       = 0;
            ApaSendAutoStopInfo13F.APA_LSMNudgeReq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMVehSecReq_S        = 0;
            ApaSendAutoStopInfo13F.APA_LSMVehDirRq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMComfBrakeReq_S     = 0;
            ApaSendAutoStopInfo13F.APA_LSMDistToStop_S       = 0;
            ApaSendAutoStopInfo13F.APA_SafeDrvrHandoverReq_S = 0;
            ApaSendAutoStopInfo13F.AVM_DVR_Sts_S             = 0;
            ApaSendAutoStopInfo13F.APA_MEB_States_S          = 0;
            ApaSendAutoStopInfo13F.APA_RMEB_States_S         = 0;
            ApaSendAutoStopInfo13F.DVR_WorkingMode_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTLong_S        = 0;
            ApaSendAutoStopInfo13F.APA_LSMvMaxRq_S           = 0;

            break;

        case CarSta_GuidanceInitParkingData:
        case CarSta_Serching:
            ApaSendAutoStopInfo13F.APA_LSMBrakeEmgcyPrep_S   = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTReq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTLevel_S       = 0;
            ApaSendAutoStopInfo13F.APA_LSMNudgeReq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMVehSecReq_S        = 0;
            ApaSendAutoStopInfo13F.APA_LSMVehDirRq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMComfBrakeReq_S     = 0;
            ApaSendAutoStopInfo13F.APA_LSMDistToStop_S       = 0;
            ApaSendAutoStopInfo13F.APA_SafeDrvrHandoverReq_S = 0;
            ApaSendAutoStopInfo13F.AVM_DVR_Sts_S             = 0;
            ApaSendAutoStopInfo13F.APA_MEB_States_S          = 0;
            ApaSendAutoStopInfo13F.APA_RMEB_States_S         = 0;
            ApaSendAutoStopInfo13F.DVR_WorkingMode_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTLong_S        = 0;
            ApaSendAutoStopInfo13F.APA_LSMvMaxRq_S           = 0;

            break;

        case CarSta_GuidanceActive:
        case CarSta_GuidanceAvpActive:

            ApaSendAutoStopInfo13F.APA_LSMSubMTReq_S   = APA_LSMSubMTReq_S;
            ApaSendAutoStopInfo13F.APA_LSMSubMTLevel_S = APA_LSMSubMTLevel_S;
            ApaSendAutoStopInfo13F.APA_LSMNudgeReq_S   = 0;
            ApaSendAutoStopInfo13F.APA_LSMVehSecReq_S  = 0;
            if (4 == RTE_BSW_Get_TargetGearVal()) // D
            {
                ApaSendAutoStopInfo13F.APA_LSMVehDirRq_S = 1;
            }
            else if (2 == RTE_BSW_Get_TargetGearVal()) // R
            {
                ApaSendAutoStopInfo13F.APA_LSMVehDirRq_S = 2;
            }
            else
            {
                ApaSendAutoStopInfo13F.APA_LSMVehDirRq_S = 0;
            }

            if (1 == RTE_BSW_Get_FailureBrakeModeVal())
            {
                ApaSendAutoStopInfo13F.APA_LSMComfBrakeReq_S = 2;
                ApaSendAutoStopInfo13F.APA_LSMSubMTLong_S    = 1;
            }
            else if (2 == RTE_BSW_Get_FailureBrakeModeVal())
            {
                ApaSendAutoStopInfo13F.APA_LSMComfBrakeReq_S = 0;
                ApaSendAutoStopInfo13F.APA_LSMSubMTLong_S    = 2;
            }
            else
            {
                ApaSendAutoStopInfo13F.APA_LSMComfBrakeReq_S = 0;
                ApaSendAutoStopInfo13F.APA_LSMSubMTLong_S    = 1;
            }

            if (3 == RTE_BSW_Get_APA_MEB_States_S())
            {
                ApaSendAutoStopInfo13F.APA_MEB_States_S        = 3;
                ApaSendAutoStopInfo13F.APA_LSMBrakeEmgcyPrep_S = 1;
            }
            else if (4 == RTE_BSW_Get_APA_MEB_States_S())
            {
                ApaSendAutoStopInfo13F.APA_MEB_States_S        = 4;
                ApaSendAutoStopInfo13F.APA_LSMBrakeEmgcyPrep_S = 0;
            }
            else
            {
                ApaSendAutoStopInfo13F.APA_MEB_States_S        = 0;
                ApaSendAutoStopInfo13F.APA_LSMBrakeEmgcyPrep_S = 0;
            }

            ApaSendAutoStopInfo13F.APA_LSMDistToStop_S = RTE_BSW_Get_TargetDistanceVal();
            ApaSendAutoStopInfo13F.APA_SafeDrvrHandoverReq_S = 0;
            ApaSendAutoStopInfo13F.AVM_DVR_Sts_S             = 0;
            ApaSendAutoStopInfo13F.APA_RMEB_States_S         = 0;
            ApaSendAutoStopInfo13F.DVR_WorkingMode_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMvMaxRq_S = RTE_BSW_Get_TargetSpeedLimitVal();

            break;

        case CarSta_GuidanceTerminated:
            ApaSendAutoStopInfo13F.APA_LSMBrakeEmgcyPrep_S   = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTReq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTLevel_S       = 0;
            ApaSendAutoStopInfo13F.APA_LSMNudgeReq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMVehSecReq_S        = 0;
            ApaSendAutoStopInfo13F.APA_LSMVehDirRq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMComfBrakeReq_S     = 0;
            ApaSendAutoStopInfo13F.APA_LSMDistToStop_S       = 0;
            ApaSendAutoStopInfo13F.APA_SafeDrvrHandoverReq_S = 0;
            ApaSendAutoStopInfo13F.AVM_DVR_Sts_S             = 0;
            ApaSendAutoStopInfo13F.APA_MEB_States_S          = 0;
            ApaSendAutoStopInfo13F.APA_RMEB_States_S         = 0;
            ApaSendAutoStopInfo13F.DVR_WorkingMode_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTLong_S        = 0;
            ApaSendAutoStopInfo13F.APA_LSMvMaxRq_S           = 0;

            break;

        case CarSta_GuidanceCompleted:
            ApaSendAutoStopInfo13F.APA_LSMBrakeEmgcyPrep_S   = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTReq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTLevel_S       = 0;
            ApaSendAutoStopInfo13F.APA_LSMNudgeReq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMVehSecReq_S        = 0;
            ApaSendAutoStopInfo13F.APA_LSMVehDirRq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMComfBrakeReq_S     = 0;
            ApaSendAutoStopInfo13F.APA_LSMDistToStop_S       = 0;
            ApaSendAutoStopInfo13F.APA_SafeDrvrHandoverReq_S = 0;
            ApaSendAutoStopInfo13F.AVM_DVR_Sts_S             = 0;
            ApaSendAutoStopInfo13F.APA_MEB_States_S          = 0;
            ApaSendAutoStopInfo13F.APA_RMEB_States_S         = 0;
            ApaSendAutoStopInfo13F.DVR_WorkingMode_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTLong_S        = 0;
            ApaSendAutoStopInfo13F.APA_LSMvMaxRq_S           = 0;

            break;

        case CarSta_Failure:
            ApaSendAutoStopInfo13F.APA_LSMBrakeEmgcyPrep_S   = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTReq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTLevel_S       = 0;
            ApaSendAutoStopInfo13F.APA_LSMNudgeReq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMVehSecReq_S        = 0;
            ApaSendAutoStopInfo13F.APA_LSMVehDirRq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMComfBrakeReq_S     = 0;
            ApaSendAutoStopInfo13F.APA_LSMDistToStop_S       = 0;
            ApaSendAutoStopInfo13F.APA_SafeDrvrHandoverReq_S = 0;
            ApaSendAutoStopInfo13F.AVM_DVR_Sts_S             = 0;
            ApaSendAutoStopInfo13F.APA_MEB_States_S          = 0;
            ApaSendAutoStopInfo13F.APA_RMEB_States_S         = 0;
            ApaSendAutoStopInfo13F.DVR_WorkingMode_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTLong_S        = 0;
            ApaSendAutoStopInfo13F.APA_LSMvMaxRq_S           = 0;

            break;

        case CarSta_ParkAssist_Standby:
            ApaSendAutoStopInfo13F.APA_LSMBrakeEmgcyPrep_S   = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTReq_S         = APA_LSMSubMTReq_S;
            ApaSendAutoStopInfo13F.APA_LSMSubMTLevel_S       = APA_LSMSubMTLevel_S;
            ApaSendAutoStopInfo13F.APA_LSMNudgeReq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMVehSecReq_S        = 0;
            ApaSendAutoStopInfo13F.APA_LSMVehDirRq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMComfBrakeReq_S     = 0;
            ApaSendAutoStopInfo13F.APA_LSMDistToStop_S       = 0;
            ApaSendAutoStopInfo13F.APA_SafeDrvrHandoverReq_S = 0;
            ApaSendAutoStopInfo13F.AVM_DVR_Sts_S             = 0;
            ApaSendAutoStopInfo13F.APA_MEB_States_S          = 0;
            ApaSendAutoStopInfo13F.APA_RMEB_States_S         = 0;
            ApaSendAutoStopInfo13F.DVR_WorkingMode_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTLong_S        = APA_LSMSubMTLong_S;
            ApaSendAutoStopInfo13F.APA_LSMvMaxRq_S           = 0;

            break;

        case CarSta_GuidanceSuspend:
            ApaSendAutoStopInfo13F.APA_LSMBrakeEmgcyPrep_S = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTReq_S       = APA_LSMSubMTReq_S;
            ApaSendAutoStopInfo13F.APA_LSMSubMTLevel_S     = APA_LSMSubMTLevel_S;
            ApaSendAutoStopInfo13F.APA_LSMNudgeReq_S       = 0;
            ApaSendAutoStopInfo13F.APA_LSMVehSecReq_S      = 0;
            // if (4 == RTE_BSW_Get_TargetGearVal())  //D
            // {
            //     ApaSendAutoStopInfo13F.APA_LSMVehDirRq_S = 1;
            // }
            // else if (2 == RTE_BSW_Get_TargetGearVal())  //R
            // {
            //     ApaSendAutoStopInfo13F.APA_LSMVehDirRq_S = 2;
            // }
            // else
            // {
            ApaSendAutoStopInfo13F.APA_LSMVehDirRq_S = 0;
            // }

            // if (1 == RTE_BSW_Get_FailureBrakeModeVal())
            // {
            //     ApaSendAutoStopInfo13F.APA_LSMComfBrakeReq_S = 1;
            //     ApaSendAutoStopInfo13F.APA_LSMSubMTLong_S = 1;
            // }
            // else if (2 == RTE_BSW_Get_FailureBrakeModeVal())
            // {
            //     ApaSendAutoStopInfo13F.APA_LSMComfBrakeReq_S = 0;
            //     ApaSendAutoStopInfo13F.APA_LSMSubMTLong_S = 2;
            // }
            // else
            // {
            ApaSendAutoStopInfo13F.APA_LSMComfBrakeReq_S = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTLong_S    = 0;
            // }

            ApaSendAutoStopInfo13F.APA_LSMDistToStop_S =
                0; // RTE_BSW_Get_TargetDistanceVal();
            ApaSendAutoStopInfo13F.APA_SafeDrvrHandoverReq_S = 0;
            ApaSendAutoStopInfo13F.AVM_DVR_Sts_S             = 0;
            ApaSendAutoStopInfo13F.APA_MEB_States_S          = 0;
            ApaSendAutoStopInfo13F.APA_RMEB_States_S         = 0;
            ApaSendAutoStopInfo13F.DVR_WorkingMode_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMvMaxRq_S =
                0; // RTE_BSW_Get_TargetSpeedLimitVal();

            break;

        case CarSta_Passive_SpdOver:
            ApaSendAutoStopInfo13F.APA_LSMBrakeEmgcyPrep_S   = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTReq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTLevel_S       = 0;
            ApaSendAutoStopInfo13F.APA_LSMNudgeReq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMVehSecReq_S        = 0;
            ApaSendAutoStopInfo13F.APA_LSMVehDirRq_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMComfBrakeReq_S     = 0;
            ApaSendAutoStopInfo13F.APA_LSMDistToStop_S       = 0;
            ApaSendAutoStopInfo13F.APA_SafeDrvrHandoverReq_S = 0;
            ApaSendAutoStopInfo13F.AVM_DVR_Sts_S             = 0;
            ApaSendAutoStopInfo13F.APA_MEB_States_S          = 0;
            ApaSendAutoStopInfo13F.APA_RMEB_States_S         = 0;
            ApaSendAutoStopInfo13F.DVR_WorkingMode_S         = 0;
            ApaSendAutoStopInfo13F.APA_LSMSubMTLong_S        = 0;
            ApaSendAutoStopInfo13F.APA_LSMvMaxRq_S           = 0;

            break;

        default:

            break;
    }

    // PreStaManRet = staManRet;
}

/*====================================================================
 * Project:
 * Function:
 *====================================================================*/
static void ApaEpsHandleJudge(void)
{
    static uint8_t ApaModeConp100ms       = 0;
    static uint8_t ApaModeStandby200ms    = 10;
    static Apa_WorkStateType PreStaManRet = CarSta_PowerOn;
    Apa_WorkStateType staManRet           = CarSta_PowerOn;
    static uint8_t ApaHandleOutCnt        = 0;

    staManRet = RTE_SM_Get_ApaWorkState();
    switch (staManRet)
    {
        case CarSta_PowerOn:
            ApaSendAutoStopInfo134.APA_MainStas                = CarSta_Passive;
            ApaSendAutoStopInfo134.APA_SpeedLimit              = 0;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = NO_Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = INVALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode        = 0;
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag     = INVALID;
            ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
            ApaSendAutoStopInfo134.APA_StopDistance          = 0;
            ApaSendAutoStopInfo134.EPB_EmergencyBrake        = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid   = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest = ApaCurrentGear();
            ApaSendAutoStopInfo134.APA_AutoStopMode          = IDLE;
            break;

        case CarSta_Passive:
            // 0x134 info
            if (PreStaManRet != CarSta_PowerOn && PreStaManRet != CarSta_Passive &&
                PreStaManRet != CarSta_Passive_SpdOver)
            {
                ApaModeConp100ms = 51;
                printf("PreStaManRet :%d\r\n", PreStaManRet);
            }
            if (ApaModeConp100ms)
            {
                ApaModeConp100ms--;
                ApaSendAutoStopInfo134.APA_MainStas = CarSta_GuidanceCompleted;
            }
            else
            {
                ApaSendAutoStopInfo134.APA_MainStas = CarSta_Passive;
            }

            ApaSendAutoStopInfo134.APA_SpeedLimit              = 0;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = NO_Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = INVALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode        = 0;
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag     = INVALID;
            ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
            ApaSendAutoStopInfo134.APA_StopDistance          = 0;
            ApaSendAutoStopInfo134.EPB_EmergencyBrake        = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid   = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest = ApaCurrentGear();
            ApaSendAutoStopInfo134.APA_AutoStopMode          = IDLE;

            break;

        case CarSta_GuidanceInitParkingData:
        case CarSta_Serching:
            ApaModeStandby200ms = 10;
            // 0x134 info
            ApaSendAutoStopInfo134.APA_MainStas                = CarSta_Serching;
            ApaSendAutoStopInfo134.APA_SpeedLimit              = 0;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = NO_Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = INVALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode        = 0;
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag     = INVALID;
            ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
            ApaSendAutoStopInfo134.APA_StopDistance          = 0;
            ApaSendAutoStopInfo134.EPB_EmergencyBrake        = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid   = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest = ApaCurrentGear();
            ApaSendAutoStopInfo134.APA_AutoStopMode          = IDLE;

            break;

        case CarSta_GuidanceActive:
        case CarSta_GuidanceAvpActive:

            if (0x02 == RTE_BSW_Get_EpsCtrlStat())
            {
                ApaHandleOutCnt = 0;
                // 0x134 info
                ApaSendAutoStopInfo134.APA_MainStas   = CarSta_GuidanceActive;
                ApaSendAutoStopInfo134.APA_SpeedLimit = RTE_BSW_Get_TargetSpeedLimitVal();
                ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = Request_For_EPS;
                ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = VALID;
                ApaSendAutoStopInfo134.APA_FailureBrakeMode =
                    RTE_BSW_Get_FailureBrakeModeVal();
                ApaSendAutoStopInfo134.APA_RequestEpsValidFlag = VALID;
                if (APA_ACTIVE == RTE_BSW_Get_OutStopCmdVal())
                {
                    ApaSendAutoStopInfo134.APA_Tar_StrAgl = RTE_BSW_Get_TargeAngleVal();
                }
                else
                {
                    ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                        (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
                }

                ApaSendAutoStopInfo134.APA_StopDistance = RTE_BSW_Get_TargetDistanceVal();
                ApaSendAutoStopInfo134.EPB_EmergencyBrake      = 1;
                ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid = 1;
                ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest =
                    RTE_BSW_Get_TargetGearVal(); // Target Gear from J3
                ApaSendAutoStopInfo134.APA_AutoStopMode = APA;
            }
            else
            {
                if (ApaHandleOutCnt < 100)
                {
                    ApaHandleOutCnt++;
                }
                // 0x134 info
                ApaSendAutoStopInfo134.APA_MainStas   = CarSta_GuidanceActive;
                ApaSendAutoStopInfo134.APA_SpeedLimit = RTE_BSW_Get_TargetSpeedLimitVal();
                if (RTE_BSW_Get_EpsCtrlStat() == 0x01)
                {
                    ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS = Request_For_EPS;
                }
                else if (ApaHandleOutCnt > 5)
                {
                    ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS = NO_Request_For_EPS;
                }
                // ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS = Request_For_EPS;
                ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = VALID;
                ApaSendAutoStopInfo134.APA_FailureBrakeMode =
                    RTE_BSW_Get_FailureBrakeModeVal();
                ApaSendAutoStopInfo134.APA_RequestEpsValidFlag = VALID;
                ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                    (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
                ApaSendAutoStopInfo134.APA_StopDistance          = 0;
                ApaSendAutoStopInfo134.EPB_EmergencyBrake        = 1;
                ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid   = 1;
                ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest = ApaCurrentGear();
                ApaSendAutoStopInfo134.APA_AutoStopMode          = APA;
            }

            break;

        case CarSta_GuidanceSuspend:
            // 0x134 info
            ApaSendAutoStopInfo134.APA_MainStas   = CarSta_GuidanceSuspend;
            ApaSendAutoStopInfo134.APA_SpeedLimit = RTE_BSW_Get_TargetSpeedLimitVal();
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = VALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode =
                RTE_BSW_Get_FailureBrakeModeVal();
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag = VALID;
            if (APA_ACTIVE == RTE_BSW_Get_OutStopCmdVal())
            {
                ApaSendAutoStopInfo134.APA_Tar_StrAgl = RTE_BSW_Get_TargeAngleVal();
            }
            else
            {
                ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                    (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
            }
            ApaSendAutoStopInfo134.APA_StopDistance   = RTE_BSW_Get_TargetDistanceVal();
            ApaSendAutoStopInfo134.EPB_EmergencyBrake = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest =
                RTE_BSW_Get_TargetGearVal(); // Target Gear from J3
            ApaSendAutoStopInfo134.APA_AutoStopMode = APA;

            break;

        case CarSta_GuidanceTerminated:
            // 0x134 info   是否需要和J3交互停车，需多方讨论？？
            ApaSendAutoStopInfo134.APA_MainStas   = CarSta_GuidanceTerminated;
            ApaSendAutoStopInfo134.APA_SpeedLimit = RTE_BSW_Get_TargetSpeedLimitVal();
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = VALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode =
                RTE_BSW_Get_FailureBrakeModeVal();
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag = VALID;
            ApaSendAutoStopInfo134.APA_Tar_StrAgl          = RTE_BSW_Get_TargeAngleVal();
            ApaSendAutoStopInfo134.APA_StopDistance   = RTE_BSW_Get_TargetDistanceVal();
            ApaSendAutoStopInfo134.EPB_EmergencyBrake = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest =
                RTE_BSW_Get_TargetGearVal(); // Target Gear from J3
            ApaSendAutoStopInfo134.APA_AutoStopMode = IDLE;

            break;

        case CarSta_GuidanceCompleted:
            // 0x134 info
            ApaSendAutoStopInfo134.APA_MainStas   = CarSta_GuidanceCompleted;
            ApaSendAutoStopInfo134.APA_SpeedLimit = RTE_BSW_Get_TargetSpeedLimitVal();
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = NO_Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = VALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode =
                RTE_BSW_Get_FailureBrakeModeVal();
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag = VALID;
            if (APA_ACTIVE == RTE_BSW_Get_OutStopCmdVal())
            {
                ApaSendAutoStopInfo134.APA_Tar_StrAgl = RTE_BSW_Get_TargeAngleVal();
            }
            else
            {
                ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                    (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
            }
            ApaSendAutoStopInfo134.APA_StopDistance   = RTE_BSW_Get_TargetDistanceVal();
            ApaSendAutoStopInfo134.EPB_EmergencyBrake = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest =
                RTE_BSW_Get_TargetGearVal(); // Target Gear from J3
            ApaSendAutoStopInfo134.APA_AutoStopMode = IDLE;

            break;

        case CarSta_Failure:
            // 0x134 info
            ApaSendAutoStopInfo134.APA_MainStas   = CarSta_Failure;
            ApaSendAutoStopInfo134.APA_SpeedLimit = RTE_BSW_Get_TargetSpeedLimitVal();
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = NO_Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = VALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode        = 2; // brake
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag     = VALID;
            ApaSendAutoStopInfo134.APA_Tar_StrAgl     = RTE_BSW_Get_TargeAngleVal();
            ApaSendAutoStopInfo134.APA_StopDistance   = RTE_BSW_Get_TargetDistanceVal();
            ApaSendAutoStopInfo134.EPB_EmergencyBrake = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest =
                RTE_BSW_Get_TargetGearVal(); // Target Gear from J3
            ApaSendAutoStopInfo134.APA_AutoStopMode = IDLE;

            break;

        case CarSta_ParkAssist_Standby:
            // 0x134 info
            if (RTE_BSW_Get_EpsCtrlStat() != 0x02)
            {
                ApaSendAutoStopInfo134.APA_MainStas = CarSta_ParkAssist_Standby;
                if (RTE_BSW_Get_EpsCtrlStat() == 0x00)
                {
                    ApaModeStandby200ms                              = 10;
                    ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS = NO_Request_For_EPS;
                }
                else if (RTE_BSW_Get_EpsCtrlStat() == 0x01)
                {
                    ApaModeStandby200ms--;
                    if (ApaModeStandby200ms > 5) // 100ms
                    {
                        ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS =
                            NO_Request_For_EPS;
                    }
                    else
                    {
                        ApaModeStandby200ms = 5;
                        ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS =
                            Request_For_EPS;
                    }
                }
                else
                {
                    printf("eps Permanent Inhibited %d\n", RTE_BSW_Get_EpsCtrlStat());
                }
            }
            else
            {
                ApaSendAutoStopInfo134.APA_MainStas              = CarSta_GuidanceActive;
                ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS = Request_For_EPS;
            }

            ApaSendAutoStopInfo134.APA_SpeedLimit = 0;
            // ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS = NO_Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = VALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode        = 0;
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag     = VALID;
            ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
            ApaSendAutoStopInfo134.APA_StopDistance   = RTE_BSW_Get_TargetDistanceVal();
            ApaSendAutoStopInfo134.EPB_EmergencyBrake = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest =
                RTE_BSW_Get_TargetGearVal();
            ApaSendAutoStopInfo134.APA_AutoStopMode = APA;

            break;

        case CarSta_Passive_SpdOver:
            // 0x134 info
            ApaSendAutoStopInfo134.APA_MainStas                = CarSta_Passive;
            ApaSendAutoStopInfo134.APA_SpeedLimit              = 0x00;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = NO_Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = INVALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode        = 0;
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag     = INVALID;
            ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
            ApaSendAutoStopInfo134.APA_StopDistance          = 0;
            ApaSendAutoStopInfo134.EPB_EmergencyBrake        = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid   = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest = ApaCurrentGear();
            ApaSendAutoStopInfo134.APA_AutoStopMode          = IDLE;

            break;

        default:

            break;
    }

    PreStaManRet = staManRet;
}

/*====================================================================
 * Project:
 * Function:
 *====================================================================*/
static void ApaOutHandShakeStatus(void)
{
    static uint8_t ApaModeConp100ms       = 0;
    static uint8_t ApaModeStandby200ms    = 10;
    static Apa_WorkStateType PreStaManRet = CarSta_PowerOn;
    Apa_WorkStateType staManRet           = CarSta_PowerOn;
    static uint8_t ApaHandleOutCnt        = 0;

    staManRet = RTE_SM_Get_ApaWorkState();
    switch (staManRet)
    {
        case CarSta_PowerOn:
            ApaSendAutoStopInfo134.APA_MainStas                = CarSta_Passive;
            ApaSendAutoStopInfo134.APA_SpeedLimit              = 0;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = NO_Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = INVALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode        = 0;
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag     = INVALID;
            ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
            ApaSendAutoStopInfo134.APA_StopDistance          = 0;
            ApaSendAutoStopInfo134.EPB_EmergencyBrake        = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid   = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest = ApaCurrentGear();
            ApaSendAutoStopInfo134.APA_AutoStopMode          = IDLE;
            break;

        case CarSta_Passive:
            // 0x134 info
            if (PreStaManRet != CarSta_PowerOn && PreStaManRet != CarSta_Passive &&
                PreStaManRet != CarSta_Passive_SpdOver)
            {
                ApaModeConp100ms = 51;
                printf("PreStaManRet :%d\r\n", PreStaManRet);
            }
            if (ApaModeConp100ms)
            {
                ApaModeConp100ms--;
                ApaSendAutoStopInfo134.APA_MainStas = CarSta_GuidanceCompleted;
            }
            else
            {
                ApaSendAutoStopInfo134.APA_MainStas = CarSta_Passive;
            }

            ApaSendAutoStopInfo134.APA_SpeedLimit              = 0;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = NO_Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = INVALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode        = 0;
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag     = INVALID;
            ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
            ApaSendAutoStopInfo134.APA_StopDistance          = 0;
            ApaSendAutoStopInfo134.EPB_EmergencyBrake        = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid   = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest = ApaCurrentGear();
            ApaSendAutoStopInfo134.APA_AutoStopMode          = IDLE;

            break;

        case CarSta_GuidanceInitParkingData:
        case CarSta_Serching:
            ApaModeStandby200ms = 10;
            // 0x134 info
            ApaSendAutoStopInfo134.APA_MainStas                = CarSta_Serching;
            ApaSendAutoStopInfo134.APA_SpeedLimit              = 0;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = NO_Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = INVALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode        = 0;
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag     = INVALID;
            ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
            ApaSendAutoStopInfo134.APA_StopDistance          = 0;
            ApaSendAutoStopInfo134.EPB_EmergencyBrake        = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid   = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest = ApaCurrentGear();
            ApaSendAutoStopInfo134.APA_AutoStopMode          = IDLE;

            break;

        case CarSta_GuidanceActive:
            if ((0x02 == RTE_BSW_Get_EscApaStat()) && (0x02 == RTE_BSW_Get_EpsCtrlStat()))
            {
                ApaHandleOutCnt = 0;
                // 0x134 info
                ApaSendAutoStopInfo134.APA_MainStas   = CarSta_GuidanceActive;
                ApaSendAutoStopInfo134.APA_SpeedLimit = RTE_BSW_Get_TargetSpeedLimitVal();
                ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = Request_For_EPS;
                ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = VALID;
                ApaSendAutoStopInfo134.APA_FailureBrakeMode =
                    RTE_BSW_Get_FailureBrakeModeVal();
                ApaSendAutoStopInfo134.APA_RequestEpsValidFlag = VALID;
                if (APA_ACTIVE == RTE_BSW_Get_OutStopCmdVal())
                {
                    ApaSendAutoStopInfo134.APA_Tar_StrAgl = RTE_BSW_Get_TargeAngleVal();
                }
                else
                {
                    ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                        (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
                }
                ApaSendAutoStopInfo134.APA_StopDistance = RTE_BSW_Get_TargetDistanceVal();
                ApaSendAutoStopInfo134.EPB_EmergencyBrake      = 1;
                ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid = 1;
                ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest =
                    RTE_BSW_Get_TargetGearVal(); // Target Gear from J3
                ApaSendAutoStopInfo134.APA_AutoStopMode = APA;
            }
            else
            {
                if (ApaHandleOutCnt < 100)
                {
                    ApaHandleOutCnt++;
                }
                // 0x134 info
                ApaSendAutoStopInfo134.APA_MainStas   = CarSta_GuidanceActive;
                ApaSendAutoStopInfo134.APA_SpeedLimit = 0;
                if (RTE_BSW_Get_EpsCtrlStat() == 0x01)
                {
                    ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS = Request_For_EPS;
                }
                else if (ApaHandleOutCnt > 5)
                {
                    ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS = NO_Request_For_EPS;
                }
                // ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS = Request_For_EPS;
                ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = VALID;
                ApaSendAutoStopInfo134.APA_FailureBrakeMode        = 0;
                ApaSendAutoStopInfo134.APA_RequestEpsValidFlag     = VALID;
                ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                    (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
                ApaSendAutoStopInfo134.APA_StopDistance          = 0;
                ApaSendAutoStopInfo134.EPB_EmergencyBrake        = 1;
                ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid   = 1;
                ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest = ApaCurrentGear();
                ApaSendAutoStopInfo134.APA_AutoStopMode          = APA;
            }

            break;

        case CarSta_GuidanceSuspend:
            // 0x134 info
            ApaSendAutoStopInfo134.APA_MainStas   = CarSta_GuidanceSuspend;
            ApaSendAutoStopInfo134.APA_SpeedLimit = RTE_BSW_Get_TargetSpeedLimitVal();
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = VALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode =
                RTE_BSW_Get_FailureBrakeModeVal();
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag = VALID;
            if (APA_ACTIVE == RTE_BSW_Get_OutStopCmdVal())
            {
                ApaSendAutoStopInfo134.APA_Tar_StrAgl = RTE_BSW_Get_TargeAngleVal();
            }
            else
            {
                ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                    (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
            }
            ApaSendAutoStopInfo134.APA_StopDistance   = RTE_BSW_Get_TargetDistanceVal();
            ApaSendAutoStopInfo134.EPB_EmergencyBrake = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest =
                RTE_BSW_Get_TargetGearVal(); // Target Gear from J3
            ApaSendAutoStopInfo134.APA_AutoStopMode = APA;

            break;

        case CarSta_GuidanceTerminated:
            // 0x134 info   是否需要和J3交互停车，需多方讨论？？
            ApaSendAutoStopInfo134.APA_MainStas   = CarSta_GuidanceTerminated;
            ApaSendAutoStopInfo134.APA_SpeedLimit = RTE_BSW_Get_TargetSpeedLimitVal();
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = VALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode =
                RTE_BSW_Get_FailureBrakeModeVal();
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag = VALID;
            ApaSendAutoStopInfo134.APA_Tar_StrAgl          = RTE_BSW_Get_TargeAngleVal();
            ApaSendAutoStopInfo134.APA_StopDistance   = RTE_BSW_Get_TargetDistanceVal();
            ApaSendAutoStopInfo134.EPB_EmergencyBrake = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest =
                RTE_BSW_Get_TargetGearVal(); // Target Gear from J3
            ApaSendAutoStopInfo134.APA_AutoStopMode = IDLE;

            break;

        case CarSta_GuidanceCompleted:
            // 0x134 info
            ApaSendAutoStopInfo134.APA_MainStas   = CarSta_GuidanceCompleted;
            ApaSendAutoStopInfo134.APA_SpeedLimit = RTE_BSW_Get_TargetSpeedLimitVal();
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = NO_Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = VALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode =
                RTE_BSW_Get_FailureBrakeModeVal();
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag = VALID;
            if (APA_ACTIVE == RTE_BSW_Get_OutStopCmdVal())
            {
                ApaSendAutoStopInfo134.APA_Tar_StrAgl = RTE_BSW_Get_TargeAngleVal();
            }
            else
            {
                ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                    (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
            }
            ApaSendAutoStopInfo134.APA_StopDistance   = RTE_BSW_Get_TargetDistanceVal();
            ApaSendAutoStopInfo134.EPB_EmergencyBrake = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest =
                RTE_BSW_Get_TargetGearVal(); // Target Gear from J3
            ApaSendAutoStopInfo134.APA_AutoStopMode = IDLE;

            break;

        case CarSta_Failure:
            // 0x134 info
            ApaSendAutoStopInfo134.APA_MainStas   = CarSta_Failure;
            ApaSendAutoStopInfo134.APA_SpeedLimit = RTE_BSW_Get_TargetSpeedLimitVal();
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = NO_Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = VALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode        = 2; // brake
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag     = VALID;
            ApaSendAutoStopInfo134.APA_Tar_StrAgl     = RTE_BSW_Get_TargeAngleVal();
            ApaSendAutoStopInfo134.APA_StopDistance   = RTE_BSW_Get_TargetDistanceVal();
            ApaSendAutoStopInfo134.EPB_EmergencyBrake = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest =
                RTE_BSW_Get_TargetGearVal(); // Target Gear from J3
            ApaSendAutoStopInfo134.APA_AutoStopMode = IDLE;

            break;

        case CarSta_ParkAssist_Standby:
            // 0x134 info
            if (ApaModeStandby200ms)
            {
                ApaModeStandby200ms--;
                ApaSendAutoStopInfo134.APA_MainStas = CarSta_ParkAssist_Standby;
                ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS = NO_Request_For_EPS;
            }
#if 0
            else if (RTE_BSW_Get_EscApaStat() != 0x02)
            {
                ApaSendAutoStopInfo134.APA_MainStas = CarSta_ParkAssist_Standby;
                if (RTE_BSW_Get_EscApaStat() == 0x01)
                {
                //    ApaSendAutoStopInfo134.APA_MainStas = CarSta_GuidanceActive;
                }
                else
                {
                //  ApaSendAutoStopInfo134.APA_MainStas = CarSta_ParkAssist_Standby;
                }
                ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS = NO_Request_For_EPS;
            }
#endif
            else if (RTE_BSW_Get_EpsCtrlStat() != 0x02)
            {
                ApaSendAutoStopInfo134.APA_MainStas = CarSta_GuidanceActive;
                if (RTE_BSW_Get_EpsCtrlStat() == 0x01)
                {
                    ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS = Request_For_EPS;
                }
                else
                {
                    ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS = NO_Request_For_EPS;
                }
            }
            else
            {
                ApaSendAutoStopInfo134.APA_MainStas              = CarSta_GuidanceActive;
                ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS = Request_For_EPS;
            }

            ApaSendAutoStopInfo134.APA_SpeedLimit = 0;
            // ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS = NO_Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = VALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode        = 0;
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag     = VALID;
            ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
            ApaSendAutoStopInfo134.APA_StopDistance          = 0;
            ApaSendAutoStopInfo134.EPB_EmergencyBrake        = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid   = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest = ApaCurrentGear();
            ApaSendAutoStopInfo134.APA_AutoStopMode          = APA;

            break;

        case CarSta_Passive_SpdOver:
            // 0x134 info
            ApaSendAutoStopInfo134.APA_MainStas                = CarSta_Passive;
            ApaSendAutoStopInfo134.APA_SpeedLimit              = 0x00;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPS   = NO_Request_For_EPS;
            ApaSendAutoStopInfo134.APA_SteeringCtrlReqForEPSVD = INVALID;
            ApaSendAutoStopInfo134.APA_FailureBrakeMode        = 0;
            ApaSendAutoStopInfo134.APA_RequestEpsValidFlag     = INVALID;
            ApaSendAutoStopInfo134.APA_Tar_StrAgl =
                (uint16)(TARGET_ANGLE_OFFSET + RTE_BSW_Get_EPS_Angle() * 10); // 0
            ApaSendAutoStopInfo134.APA_StopDistance          = 0;
            ApaSendAutoStopInfo134.EPB_EmergencyBrake        = 1;
            ApaSendAutoStopInfo134.EPB_EmergencyBrakeValid   = 1;
            ApaSendAutoStopInfo134.APS_SCU_TargetGearRequest = ApaCurrentGear();
            ApaSendAutoStopInfo134.APA_AutoStopMode          = IDLE;

            break;

        default:

            break;
    }

    PreStaManRet = staManRet;
}

/*====================================================================
 * Project:
 * Function:
 *====================================================================*/
static void ApaCtrlCarBodyPro(void)
{
    uint8_t currCarTypeDef =
        PK_VehConfigType() < CAR_TYPE_MAX ? PK_VehConfigType() : CAR_TYPE_NO;
    if (CAR_TYPE_HAN == currCarTypeDef || CAR_TYPE_YUAN == currCarTypeDef)
    {
        ApaEpsHandleJudge();
        ApaIpbHandleJudge();
    }
    else if (CAR_TYPE_SONG == currCarTypeDef)
    {
        ApaOutHandShakeStatus(); // 握手
    }
    ApaDataPro();
}

void *can_sample_send_msg(void *argv)
{
    uint8_t pub_counter = 0;
    struct timespec ts, last_ts, now_ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    while (1)
    {
        ApaCtrlCarBodyPro();

        Can_PushData(0x6300, ApaCanTxDataBuff.data[CANID_INDEX_134]);
        Can_PushData(0x6500, ApaCanTxDataBuff.data[CANID_INDEX_13F]);

        // === 精准延时到下一个周期点 ===
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

        // 设定下一个20ms周期时间点
        ts.tv_nsec += 20 * 1000 * 1000;
        if (ts.tv_nsec >= 1000000000)
        {
            ts.tv_sec += 1;
            ts.tv_nsec -= 1000000000;
        }

        Uart_PushCanData(ApaCanTxDataBuff.CanId, ApaCanTxDataBuff.data, CANID_INDEX_MAX);

        // 打印时间差
        // clock_gettime(CLOCK_MONOTONIC, &now_ts);
        // long diff_nsec = (now_ts.tv_sec - last_ts.tv_sec) * 1000000000L +
        //                  (now_ts.tv_nsec - last_ts.tv_nsec);
        // printf("[CAN] datat = %ld ns = %.3f ms\n", diff_nsec, diff_nsec / 1e6);
        // last_ts = now_ts;
    }

    return NULL;
}

void Can_InitSendInfo()
{
    ApaDataInit();
    // int result         = 0;
    // pthread_t threadId = 0;
    // result             = pthread_create(&threadId, NULL, can_sample_send_msg, NULL);
    // if (result == 0)
    // {
    //     pthread_setname_np(threadId, "comm_can_send");
    // }
    ASyncRun(can_sample_send_msg, NULL, "apa_can_S2mcu");
}