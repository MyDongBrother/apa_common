#include <stdio.h>
#include <unistd.h>

#include "data_io/dataInput_adi.h"  //模块输入接口
#include "data_io/dataOutput_adi.h" //模块输出接口
#include "data_io/funlist.h"        //函数调用接口
#include <ctime>
#include "Rte_BSW.h"

int PK_Odometry_init()
{
    g_ModuleInit_vd(); // 初始化函数,必须初始化一次
    return 0;
}

int PK_Odometry()
{

    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    UInt16 cur_ms =
        static_cast<UInt16>((ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL) & 0xFFFF);
    g_SetTimeMs_ui16(cur_ms);

    gType_FVInputCanFrame_st can;
    can.ESC_FLWheelSpeedRC       = RTE_BSW_Get_WheelCounter_FL();
    can.ESC_FRWheelSpeedRC       = RTE_BSW_Get_WheelCounter_FR();
    can.ESC_RLWheelSpeedRC       = RTE_BSW_Get_WheelCounter_RL();
    can.ESC_RRWheelSpeedRC       = RTE_BSW_Get_WheelCounter_RR();
    can.ESC_FLWheelSpeedRC_valid = 0;
    can.ESC_FRWheelSpeedRC_valid = 0;
    can.ESC_RLWheelSpeedRC_valid = 0;
    can.ESC_RRWheelSpeedRC_valid = 0;
    RTE_BSW_GearType raw_gear    = RTE_BSW_Get_CurrentGear();
    if (GEAR_REAL_D == raw_gear) // D
    {
        can.Gear_ui8             = 3;
        can.ESC_FLWheelDirection = 1;
        can.ESC_FRWheelDirection = 1;
        can.ESC_RLWheelDirection = 1;
        can.ESC_RRWheelDirection = 1;
    }
    else if (GEAR_REAL_R == raw_gear) // R
    {
        can.Gear_ui8             = 1;
        can.ESC_FLWheelDirection = 2;
        can.ESC_FRWheelDirection = 2;
        can.ESC_RLWheelDirection = 2;
        can.ESC_RRWheelDirection = 2;
    }
    else
    {
        can.Gear_ui8             = 0;
        can.ESC_FLWheelDirection = 0;
        can.ESC_FRWheelDirection = 0;
        can.ESC_RLWheelDirection = 0;
        can.ESC_RRWheelDirection = 0;
    }

    can.ESC_FLWheelSpeed_valid = 1;
    can.ESC_FRWheelSpeed_valid = 1;
    can.ESC_RLWheelSpeed_valid = 1;
    can.ESC_RRWheelSpeed_valid = 1;
    float steeringAngle        = RTE_BSW_Get_EPS_Angle();
    can.SteerWheelAngle_si32   = (SInt32)(steeringAngle * 100.0);
    can.SteerWheelAngle_valid  = 0;
    can.PowerMode              = 1;
    can.PowerMode_Valid        = 0;
    can.EngineStatus           = 1;

    // can信号输入接口
    g_SetMsgCan_vd((const UInt8 *)&can, (UInt16)sizeof(gType_FVInputCanFrame_st));

    // 调用周期函数
    g_Function_10msTask_vd();

    // 获取ODO输出
    gType_VehOdoFrame_st l_VehPosOdo_pst;
    g_adiGetMsgOdo_vd(&l_VehPosOdo_pst);
    // printf("[ODOdata] ODO-X: %d, ODO-Y: %d, ODO-S: %d, ODO-Ang: %d, \n",
    //        l_VehPosOdo_pst.VehPosOdoXMm_si32, l_VehPosOdo_pst.VehPosOdoYMm_si32,
    //        l_VehPosOdo_pst.VehPosOdoSMm_si32, l_VehPosOdo_pst.VehPosOdoYawAngle_ui32);
    return 0;
}
