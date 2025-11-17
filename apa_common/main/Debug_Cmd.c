/**
 * @file Debug_Cmd.c
 * @brief Debug_Cmd.c主要负责系统的调试命令处理：
 *  定义和管理一系列调试命令，包括命令名称、帮助说明、参数解析和执行函数。
 *  启动独立线程，通过命名管道（FIFO）监听外部输入的调试命令。
 *  解析用户输入的命令字符串，根据命令表匹配对应的命令函数并执行。
 *  支持通过内部接口设置命令并异步执行，保证调试命令的灵活触发。
 *  在执行命令前后打印日志，记录命令的开始和结束时间，方便调试和性能分析。
 *  实时监控停车位的规划和检测状态变化，并打印最新停车位信息。
 *  提供命令帮助信息输出，方便用户查询可用调试命令及用法。
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-07-09
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-07-09 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */
#include "PK_Location.h"
#include "PK_PathPlan.h"
#include "PK_SlotDetect.h"
#include "PK_PathExecute.h"
#include "PK_Calibration.h"
#include "PK_StateManage.h"
#include "CanIf.h"
#include "main.h"
#include "Debug_Cmd.h"
#include "Record_Log.h"
#include "MathFunc.h"
#include "Ram.h"
#include "Rte.h"
/**
 * @brief 命令处理结构体，定义每条命令的执行函数、解析函数、命令字符串等信息。
 */
/// 指向命令执行函数的函数指针，输入为标准长度的字节数组
typedef void (*RUN_Func)(const uint8_t data[candata_bytes]);
/// 指向命令解析函数的函数指针，负责从命令行字符串提取参数并填充 data
typedef int (*PARSE_Func)(const char *input, uint8_t data[candata_bytes]);
typedef struct
{
    RUN_Func runFunc;     ///< 命令的执行函数指针，非空时被调用以执行命令
    bool log;             ///< 是否记录日志（true：CAN 发送；false：写入共享变量）
    PARSE_Func parseFunc; ///< 命令的参数解析函数指针，可为空
    const char *cmdName;  ///< 命令名称字符串，用于匹配用户输入
    const char *cmdHelp;  ///< 命令帮助信息，可用于打印帮助
} COMMAND_LINE_INFO;

static int pauseRecover    = 0; // 0 run   1  pause
static int ptsFd           = 1;
static uint8_t command[32] = {0};

/**
 * @brief 等待系统从暂停状态恢复。
 *
 * 该函数会持续轮询 `pauseRecover` 标志，若为 1 表示系统处于暂停状态，函数会阻塞等待；
 * 每 200 次（大约 4 秒）打印一次日志。
 * `pauseRecover` 恢复为 0 后函数返回。
 */
void PK_RestoreState()
{
    int count = 0;
    while (pauseRecover == 1)
    {
        if ((count % 200) == 0)
        {
            printf("----------------- pause ----------------\n");
        }
        count++;
        usleep(20 * 1000);
    }
}

/**
 * @brief 带时间戳的日志输出函数，向指定文件描述符输出格式化字符串。
 *
 * @param fmt 可变参数格式字符串，支持 printf 风格格式化。
 *
 * 输出格式为：
 * ```
 * YYYY-MM-DD HH:MM:SS.mmm <用户自定义内容>
 * ```
 * 时间精度为毫秒。
 */
static void OutputToFd(const char *fmt, ...)
{
    char printbuf[1024];
    memset(printbuf, 0, sizeof(printbuf));

    struct timeval stamp;
    gettimeofday(&stamp, 0);

    const struct tm *ptm = localtime(&(stamp.tv_sec));
    int writeLen = strftime(printbuf, sizeof(printbuf) - 1, "%Y-%m-%d %H:%M:%S", ptm);
    writeLen += snprintf(&printbuf[writeLen], sizeof(printbuf) - 1 - writeLen, ".%03ld ",
                         stamp.tv_usec / 1000);

    va_list args;
    va_start(args, fmt);
    writeLen +=
        vsnprintf(&printbuf[writeLen], sizeof(printbuf) - 1 - writeLen, fmt, args);
    va_end(args);
    if (writeLen > 0)
    {
        write(ptsFd, printbuf, writeLen);
    }
}

/**
 * @brief 解析一个 uint8_t 类型参数，并写入 cmdData[1]
 *
 * @param cmd     输入命令字符串
 * @param cmdData 用于存储解析结果的 buffer，cmdData[1] 写入解析值
 * @return int    0 表示成功，1 表示解析失败或值超出范围
 */
static int Parse1U8Paras(const char *cmd, uint8_t cmdData[candata_bytes])
{
    int value1;
    int result = sscanf(cmd, "%d", &value1);
    if (result != 1 || abs(value1) > 99)
    {
        return 1;
    }
    else
    {
        cmdData[1] = (uint8_t)value1;
        return 0;
    }
}

/**
 * @brief 解析两个 uint8_t 类型参数，分别写入 cmdData[1] 和 cmdData[2]
 *
 * @param cmd     输入命令字符串
 * @param cmdData 用于存储解析结果的 buffer，cmdData[1]/[2] 写入解析值
 * @return int    0 表示成功，1 表示解析失败或值超出范围
 */
static int Parse2U8Paras(const char *cmd, uint8_t cmdData[candata_bytes])
{
    int value1, value2;
    int result = sscanf(cmd, "%d %d", &value1, &value2);
    if (result != 2 || abs(value1) > 99 || abs(value2) > 99)
    {
        return 1;
    }
    else
    {
        cmdData[1] = (uint8_t)value1;
        cmdData[2] = (uint8_t)value2;
        return 0;
    }
}

/**
 * @brief 解析一个 uint32_t 类型参数，并以字节流形式写入 cmdData[1~4]
 *
 * @param cmd     输入命令字符串
 * @param cmdData 用于存储解析结果的 buffer，从 cmdData[1] 开始写入 4 字节整数
 * @return int    0 表示成功，1 表示解析失败
 */
static int Parse1U32Paras(const char *cmd, uint8_t cmdData[candata_bytes])
{
    int value1;
    int result = sscanf(cmd, "%d", &value1);
    if (result != 1)
    {
        return 1;
    }
    else
    {
        Uint32ToBuf(static_cast<uint32_t>(value1), &cmdData[1]);
        return 0;
    }
}

/**
 * @brief 打开指定的虚拟终端设备（/dev/pts/X），用于后续日志输出。
 *
 * @param cmdData 命令数据，cmdData[1] 表示终端编号
 */
static void OpenPrintPts(const uint8_t cmdData[candata_bytes])
{
    char termName[32];
    sprintf(termName, "/dev/pts/%d", cmdData[1]);
    ptsFd = open(termName, O_CREAT | O_WRONLY, 0666);
}

/**
 * @brief 输出软件版本和编译时间信息到虚拟终端。
 *
 * @param cmdData 命令数据，未使用
 */
static void ShowVerInfo(const uint8_t cmdData[candata_bytes])
{
    OutputToFd("bmi software version: %s date: %s\n", APA_VERSION, APA_STAMP);
    OutputToFd("compile date: %s %s\n", __DATE__, __TIME__);
}

/**
 * @brief 设置日志等级
 *
 * @param cmdData 命令数据，cmdData[1] 表示要设置的日志等级（LOG_LEVEL 枚举）
 */
static void SetLogLevel(const uint8_t cmdData[candata_bytes])
{
    LOG_LEVEL level = (LOG_LEVEL)cmdData[1];
    if (level <= LOG_FATAL)
    {
        LOG_SetLevel(0, level);
    }
}

/**
 * @brief 设置标准输出（stdout）重定向到指定的虚拟终端（/dev/pts/X）。
 *
 * 该函数会启动一个异步线程，持续尝试将标准输出（文件描述符 1）重定向到指定的终端设备。
 * 直到打开的文件描述符为 1（即标准输出）为止，才认为重定向成功。
 *
 * @param cmdData 命令数据，cmdData[1] 表示目标终端编号
 */
static void SetConsolePts(const uint8_t cmdData[candata_bytes])
{
    static char consoleName[32];
    sprintf(consoleName, "/dev/pts/%d", cmdData[1]);

    auto SetNewFd = [](void *) -> void * {
        while (1)
        {
            close(1);
            int ptsFd = open(consoleName, O_CREAT | O_WRONLY, 0666);
            OutputToFd("name %s id %d\n", consoleName, ptsFd);
            if (ptsFd == 1)
            {
                break;
            }
            close(ptsFd);
            sleep(1);
        }
        return NULL;
    };
    ASyncRun(SetNewFd, NULL, "dbg_new_fd");
}

/**
 * @brief 异步打印 CCP 接口采集的传感器原始值，用于调试和验证传感器输出。
 *
 * 该函数根据 data[1] 指定的循环次数，每隔 2 秒打印一次传感器采集的数据，
 * 包括加速度、角速度、轮速计数器、方向盘角度、车辆速度、当前定位、IMU 姿态角等。
 *
 * @param data 命令数据，data[1] 表示循环次数
 */
static void ShowCcpVal(const uint8_t data[candata_bytes])
{
    command[0]        = data[1];
    auto OutputSensor = [](void *args) -> void * {
        int count = command[0];
        for (int loop = 0; loop < count; loop++)
        {
            OutputToFd("-------------------- count = %d start -------------------\n",
                       loop);
            OutputToFd("Accel_X1_CCP = %f\n", RTE_BSW_Get_Accel_X());
            OutputToFd("Accel_Y1_CCP = %f\n", RTE_BSW_Get_Accel_Y());
            OutputToFd("Accel_Z1_CCP = %f\n", RTE_BSW_Get_Accel_Z());
            OutputToFd("AngRate_X_CCP = %f\n", RTE_BSW_Get_AngRate_X());
            OutputToFd("AngRate_Y_CCP = %f\n", RTE_BSW_Get_AngRate_Y());
            OutputToFd("AngRate_Z_CCP = %f\n", RTE_BSW_Get_AngRate_Z());

            OutputToFd("Counter_RL_ccp = %d\n", RTE_BSW_Get_WheelCounter_RL());
            OutputToFd("Counter_RR_ccp = %d\n", RTE_BSW_Get_WheelCounter_RR());
            OutputToFd("Counter_FL_ccp = %d\n", RTE_BSW_Get_WheelCounter_FL());
            OutputToFd("Counter_FR_ccp = %d\n", RTE_BSW_Get_WheelCounter_FR());

            OutputToFd("EPS_angle_ccp = %f\n", RTE_BSW_Get_EPS_Angle());
            OutputToFd("EPS_angle_spd_ccp = %f\n", RTE_BSW_Get_EPS_AngleSpd());
            OutputToFd("ESC_VehSpd_ccp = %f\n", RTE_BSW_Get_ESC_VehSpd());

            float curpos[4];
            RTE_PK_Location_Get_CurPos(curpos);
            OutputToFd("CurPos_ccp0 = %f\n", curpos[0]);
            OutputToFd("CurPos_ccp1 = %f\n", curpos[1]);
            OutputToFd("CurPos_ccp2 = %f\n", curpos[2]);
            OutputToFd("CurPos_ccp3 = %f\n", curpos[3]);

            OutputToFd("RTE_BSW_Get_ESC_YAW_RATE = %f\n", RTE_BSW_Get_ESC_YAW_RATE());

            OutputToFd("RTE_BSW_Get_Accel_X = %f\n", RTE_BSW_Get_Accel_X());
            OutputToFd("RTE_BSW_Get_Accel_Y = %f\n", RTE_BSW_Get_Accel_Y());
            OutputToFd("RTE_BSW_Get_Accel_Z = %f\n", RTE_BSW_Get_Accel_Z());

            OutputToFd("IMU_Roll_ccp = %f\n", RTE_BSW_Get_IMU_Roll());
            OutputToFd("IMU_Pitch_ccp = %f\n", RTE_BSW_Get_IMU_Pitch());
            OutputToFd("IMU_Head_ccp = %f\n", RTE_BSW_Get_IMU_Head());

            OutputToFd("RTE_BSW_Get_CurrentGear = %d\n", RTE_BSW_Get_CurrentGear());

            OutputToFd("CurSpd_Motor_ccp(+/- m/s) = %f\n", RTE_BSW_Get_Motorspd());
            OutputToFd("Motor circle (+/- circle) = %f\n", RTE_BSW_Get_Motorspd());

            // RTE_BSW_Get_Motorspd
            OutputToFd("imu speed(m/s) = %f\n", RTE_BSW_Get_IMU_VelSpd());
            OutputToFd("-------------------- count = %d end --------------------\n",
                       loop);

            sleep(2);
        }
        return NULL;
    };

    ASyncRun(OutputSensor, NULL, "dbg_out_veh");
}

/**
 * @brief 打印当前车辆位置与方向盘角度信息。
 *
 * 从定位模块获取当前车辆位置，包括 X、Y 坐标、航向角、累计距离，
 * 同时获取当前方向盘转角并一并输出，便于调试车辆定位与姿态。
 *
 * @param data 输入数据（未使用，但保留接口一致性）
 */
static void ShowCurPos(const uint8_t data[candata_bytes])
{
    float curPos[4];
    RTE_PK_Location_Get_CurPos(curPos);
    OutputToFd("Current Position : x=%f, y=%f, head=%f, dis=%f, angle=%f\n", curPos[0],
               curPos[1], curPos[2], curPos[3], RTE_BSW_Get_EPS_Angle());
}

/**
 * @brief 打印车位对象的几何尺寸和转换后的坐标信息。
 *
 * 计算车位各关键点之间的距离和方向角，输出原始车位几何形状数据，
 * 并将车位坐标从全局坐标系转换到相对当前位置坐标系后再次输出。
 *
 * @param curPos 当前车辆位置坐标（数组，长度4）
 * @param slotobj 车位对象结构体
 */
static void ShowSlotObj(const float curPos[4], const SlotObj_T &slotobj)
{
    float rpos[3], temp[5][4];

    SlotObj_Convert_float(slotobj, temp);
    float abLen = sqrtf(pow2(temp[0][0] - temp[1][0]) + pow2(temp[0][1] - temp[1][1]));
    float bcLen = sqrtf(pow2(temp[1][0] - temp[2][0]) + pow2(temp[1][1] - temp[2][1]));
    float beLen = sqrtf(pow2(temp[1][0] - temp[4][0]) + pow2(temp[1][1] - temp[4][1]));
    float efLen = sqrtf(pow2(temp[4][0] - temp[4][2]) + pow2(temp[4][1] - temp[4][3]));

    float abdir = atan2f(temp[0][3] - temp[0][1], temp[0][2] - temp[0][0]);
    float bcdir = atan2f(temp[1][3] - temp[1][1], temp[1][2] - temp[1][0]);
    float cddir = atan2f(temp[2][3] - temp[2][1], temp[2][2] - temp[2][0]);
    float dedir = atan2f(temp[3][3] - temp[3][1], temp[3][2] - temp[3][0]);
    float efdir = atan2f(temp[4][3] - temp[4][1], temp[4][2] - temp[4][0]);

    OutputToFd(
        "Len: AB=%2.4f BC=%2.4f BE=%2.4f EF=%2.4f Angle: AB=%2.4f BC=%2.4f BE=%2.4f "
        "CD=%2.4f EF=%2.4f\n",
        abLen, bcLen, beLen, efLen, abdir, bcdir, cddir, dedir, efdir);

    OutputToFd(
        "OBJW: A: %2.4f %2.4f, B: %2.4f %2.4f, C: %2.4f %2.4f, D: %2.4f %2.4f, E: %2.4f "
        "%2.4f, F:%2.4f %2.4f\n",
        temp[0][0], temp[0][1], temp[1][0], temp[1][1], temp[2][0], temp[2][1],
        temp[3][0], temp[3][1], temp[4][0], temp[4][1], temp[4][2], temp[4][3]);

    RevConvert(curPos, (float *)&temp[0][0], rpos);
    temp[0][0] = rpos[0];
    temp[0][1] = rpos[1];
    for (int i = 0; i < 4; i++)
    {
        RevConvert(curPos, (float *)&temp[i][2], rpos);
        temp[i][2]     = rpos[0];
        temp[i][3]     = rpos[1];
        temp[i + 1][0] = rpos[0];
        temp[i + 1][1] = rpos[1];
    }

    RevConvert(curPos, (float *)&temp[4][2], rpos);
    temp[4][2] = rpos[0];
    temp[4][3] = rpos[1];

    OutputToFd(
        "OBJV: A: %2.4f %2.4f, B: %2.4f %2.4f, C: %2.4f %2.4f, D: %2.4f %2.4f, E: %2.4f "
        "%2.4f, F: %2.4f %2.4f\n",
        temp[0][0], temp[0][1], temp[1][0], temp[1][1], temp[2][0], temp[2][1],
        temp[3][0], temp[3][1], temp[4][0], temp[4][1], temp[4][2], temp[4][3]);
}

/**
 * @brief 显示AVM点信息，包括距离、角度及坐标转换后的结果
 *
 * @param curPos 当前车辆位置及姿态数组，长度为4
 * @param avmpoint AVM点的4个二维坐标点，格式为4x2数组
 */
static void ShowAvmPoint(const float curPos[4], const float avmpoint[4][2])
{
    // e b d c
    float bcLen = sqrtf(pow2(avmpoint[1][0] - avmpoint[3][0]) +
                        pow2(avmpoint[1][1] - avmpoint[3][1]));
    float beLen = sqrtf(pow2(avmpoint[1][0] - avmpoint[0][0]) +
                        pow2(avmpoint[1][1] - avmpoint[0][1]));
    float deLen = sqrtf(pow2(avmpoint[2][0] - avmpoint[0][0]) +
                        pow2(avmpoint[2][1] - avmpoint[0][1]));

    float bcdir =
        atan2f(avmpoint[3][1] - avmpoint[1][1], avmpoint[3][0] - avmpoint[1][0]);
    float cddir =
        atan2f(avmpoint[2][1] - avmpoint[3][1], avmpoint[2][0] - avmpoint[3][0]);
    float dedir =
        atan2f(avmpoint[0][1] - avmpoint[2][1], avmpoint[0][0] - avmpoint[2][0]);

    OutputToFd("Len: BC=%2.4f BE=%2.4f DE=%2.4f Angle: BC=%2.4f CD=%2.4f DE=%2.4f\n",
               bcLen, beLen, deLen, bcdir, cddir, dedir);

    OutputToFd("AVMW: %2.4f %2.4f, %2.4f %2.4f, %2.4f %2.4f, %2.4f %2.4f\n",
               avmpoint[1][0], avmpoint[1][1], avmpoint[3][0], avmpoint[3][1],
               avmpoint[2][0], avmpoint[2][1], avmpoint[0][0], avmpoint[0][1]);

    float temp[4][2];
    for (int i = 0; i < 4; i++)
    {
        float rpos[3];
        RevConvert(curPos, (float *)&avmpoint[i][0], rpos);
        temp[i][0] = rpos[0];
        temp[i][1] = rpos[1];
    }

    OutputToFd("AVMV: B: %2.4f %2.4f, C: %2.4f %2.4f, D: %2.4f %2.4f, E: %2.4f %2.4f\n",
               temp[1][0], temp[1][1], temp[3][0], temp[3][1], temp[2][0], temp[2][1],
               temp[0][0], temp[0][1]);
}

/**
 * @brief 显示AVM缓存信息，分别输出左右缓存中的车位数据
 *
 * @param data 输入数据，未使用但保持接口一致
 */
static void ShowAvmBuff(const uint8_t data[candata_bytes])
{
    AVM_Buff_Info avmBuff;
    float curPos[4];
    RTE_PK_Location_Get_CurPos(curPos);
    for (int count = 0; count < 2; count++)
    {
        if (count == 0)
        {
            RTE_PK_SF_Get_AVM_Buff_Left(&avmBuff);
            OutputToFd("avm left buff: %d %d\n", avmBuff.Num, avmBuff.AccNum);
        }
        else
        {
            RTE_PK_SF_Get_AVM_Buff_Right(&avmBuff);
            OutputToFd("avm right buff: %d %d\n", avmBuff.Num, avmBuff.AccNum);
        }

        for (int index = 0; index < avmBuff.Num; index++)
        {
            float avmpoint[4][2];
            OutputToFd("index %d stopper %d type %d lock %d otom %06f\n",
                       avmBuff.AVM_slot_index[index], avmBuff.has_stopper[index],
                       avmBuff.SlotShap[index], avmBuff.has_lock[index],
                       avmBuff.Otom[index]);
            memcpy(avmpoint[0], (float *)&avmBuff.NF[index], sizeof(Point_T));
            memcpy(avmpoint[1], (float *)&avmBuff.NR[index], sizeof(Point_T));
            memcpy(avmpoint[2], (float *)&avmBuff.FF[index], sizeof(Point_T));
            memcpy(avmpoint[3], (float *)&avmBuff.FR[index], sizeof(Point_T));
            ShowAvmPoint(curPos, avmpoint);
        }
    }
}

/**
 * @brief 显示车位信息，包括位置、状态、目标坐标和形状
 *
 * @param slotInf 输入的车位信息结构体
 */
static void ShowSlotInfo(const SlotInfo_T &slotInf)
{
    float rpos[3], curPos[4];
    RTE_PK_Location_Get_CurPos(curPos);
    OutputToFd(
        "Slot index: %d dist_curveh %2.4f, status %d sensor %d shape %d stopper: %d "
        "lock: %d\n",
        slotInf.slot_index, slotInf.dist_curveh, slotInf.slot_status,
        slotInf.is_vision_slot, slotInf.slotshap, slotInf.has_stopper, slotInf.occupied);

    CoordinadteTransfer(curPos, (float *)&(slotInf.targpos), rpos);
    OutputToFd("target: %2.4f, %2.4f, %2.4f: %2.4f, %2.4f, %2.4f\n", slotInf.targpos.x,
               slotInf.targpos.y, slotInf.targpos.theta, rpos[0], rpos[1], rpos[2]);

    ShowSlotObj(curPos, slotInf.slotobj);

    if (slotInf.is_vision_slot)
    {
        float avmpoint[4][2];
        memcpy(&avmpoint[0][0], (void *)&slotInf.avm_point, sizeof(avmpoint));
        ShowAvmPoint(curPos, avmpoint);
    }
}

/**
 * @brief 显示所有车位信息，包括多车位和手动获取的车位
 *
 * @param data 未使用，仅接口统一
 */
static void ShowAllSlots(const uint8_t data[candata_bytes])
{
    Multi_Slot_Array_T multiSlots;
    RTE_PK_SlotDetect_Get_Multi_SlotInfo(&multiSlots);
    OutputToFd("Total Slots: %d\n", multiSlots.multiNum);
    for (int slot = 0; slot < multiSlots.multiNum; slot++)
    {
        ShowSlotInfo(multiSlots.multiArray[slot]);
        OutputToFd("\n");
    }

    SlotInfo_T manuSlot;
    RTE_PK_DataConv_Get_SlotInfo(&manuSlot);
    if (manuSlot.slot_index != 0 || manuSlot.slotshap != PK_SLOT_NO)
    {
        ShowSlotInfo(manuSlot);
        OutputToFd("\n");
    }
}

/**
 * @brief 显示路径规划信息，包括路径数量、形状、连接状态及各路径坐标
 *
 * @param planInfo 路径规划信息结构体
 */
static void ShowPathInfo(const PlanInfoType &planInfo)
{
    float endPos[3];
    OutputToFd("slot %d path info: PathNum %d shape %d connect %d\n", planInfo.Slot_index,
               planInfo.Path_num, planInfo.slotshape, planInfo.IsDirConect);

    // OutputToFd("planned info:0x%x 0x%x 0x%x 0x%x,0x%x 0x%x 0x%x 0x%x,0x%x 0x%x 0x%x
    // 0x%x\n",
    //     planInfo.debug[0], planInfo.debug[1], planInfo.debug[2], planInfo.debug[3],
    //     planInfo.debug[4], planInfo.debug[5], planInfo.debug[6], planInfo.debug[7],
    //     planInfo.debug[8], planInfo.debug[9], planInfo.debug[10], planInfo.debug[11]);

    for (int index = 0; index < planInfo.Path_num; index++)
    {
        PK_Get_Path_EndPos(planInfo.Act_traj[index], endPos);
        OutputToFd("path %d: %2.4f %2.4f %2.4f %2.4f %2.4f end: %2.4f %2.4f %2.4f\n",
                   index, planInfo.Act_traj[index][0], planInfo.Act_traj[index][1],
                   planInfo.Act_traj[index][2], planInfo.Act_traj[index][3],
                   planInfo.Act_traj[index][4], endPos[0], endPos[1], endPos[2]);
    }
}
/**
 * @brief 显示所有路径规划信息，包含当前位置和多个车位的路径详情
 *
 * @param data 输入数据，暂未使用，传递给 ShowCurPos 显示当前位置
 */
static void ShowPathsInfo(const uint8_t data[candata_bytes])
{
    ShowCurPos(data);
    MultiPlanInfo planInfs;
    RTE_PK_PathPlan_Get_Multi_PlanInfo(&planInfs);
    OutputToFd("planed slots number: %d\n", planInfs.slotNum);
    for (int slot = 0; slot < planInfs.slotNum; slot++)
    {
        ShowPathInfo(planInfs.slotPlans[slot]);
    }

    OutputToFd("\n");
}

/**
 * @brief 显示当前目标路径信息，包括当前位置和路径详情
 *
 * @param data 输入数据，用于显示当前位置
 */
static void ShowParkPath(const uint8_t data[candata_bytes])
{
    ShowCurPos(data);
    PlanInfoType planInf;
    RTE_PK_DataConvt_Get_Target_PlanInfo(&planInf);
    OutputToFd("Path number: %d Id:%d, Type:%d\n", planInf.Path_num, planInf.Slot_index,
               planInf.slotshape);
    ShowPathInfo(planInf);
    OutputToFd("\n");
}

/**
 * @brief 打印融合目标信息，遍历并显示有效目标的属性和位置
 *
 * @param fusInfo 融合目标结构体，包含多个目标的信息
 */
static void PrintObjInfo(const FusionObj_T &fusInfo)
{
    for (int index = 0; index < SF_OBJ_NUM && fusInfo.num > 0; index++)
    {
        if (fusInfo.attr[index] == 0)
        {
            continue;
        }
        OutputToFd(
            "objs attr:%d angle:%2.4f odom:%2.4f x:%2.4f y:%2.4f x:%2.4f y:%2.4f\n",
            fusInfo.attr[index], fusInfo.ang[index], fusInfo.odom[index],
            fusInfo.obj[index].pt1.x, fusInfo.obj[index].pt1.y, fusInfo.obj[index].pt2.x,
            fusInfo.obj[index].pt2.y);
    }
}

/**
 * @brief 显示各个方向融合目标信息，包括目标数量和准确度
 *
 * @param data 输入数据，使用其中索引信息打印
 */
static void ShowObjInfo(const uint8_t data[candata_bytes])
{
    FusionObj_T fusInfo;
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left(&fusInfo);
    OutputToFd("Font Left ObjIndex %d num %d, accu %d\n", data[1], fusInfo.num,
               fusInfo.num_accu);
    PrintObjInfo(fusInfo);

    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right(&fusInfo);
    OutputToFd("Font Right ObjIndex %d num %d, accu %d\n", data[1], fusInfo.num,
               fusInfo.num_accu);
    PrintObjInfo(fusInfo);

    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Left(&fusInfo);
    OutputToFd("Rear Left ObjIndex %d num %d, accu %d\n", data[1], fusInfo.num,
               fusInfo.num_accu);
    PrintObjInfo(fusInfo);

    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Right(&fusInfo);
    OutputToFd("Rear Right ObjIndex %d num %d, accu %d\n", data[1], fusInfo.num,
               fusInfo.num_accu);
    PrintObjInfo(fusInfo);
}

/**
 * @brief 显示停车方向融合目标信息，包含左右两侧目标数量和准确度
 *
 * @param data 输入数据，使用其中索引信息打印
 */
static void ShowBObjInfo(const uint8_t data[candata_bytes])
{
    FusionObj_T fusInfo;
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Left(&fusInfo);
    OutputToFd("Parking Left ObjIndex %d num %d, accu %d\n", data[1], fusInfo.num,
               fusInfo.num_accu);
    PrintObjInfo(fusInfo);

    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Right(&fusInfo);
    OutputToFd("Parking Right ObjIndex %d num %d, accu %d\n", data[1], fusInfo.num,
               fusInfo.num_accu);
    PrintObjInfo(fusInfo);
}
/**
 * @brief 显示原始传感器融合目标信息，包含前左、前右、后左、后右四个区域
 *
 * @param data 输入数据，使用其中索引信息打印
 */
static void ShowRawObjInfo(const uint8_t data[candata_bytes])
{
    FusionObj_T fusInfo;
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left_Raw(&fusInfo);
    OutputToFd("Font Left ObjIndex %d num %d, accu %d\n", data[1], fusInfo.num,
               fusInfo.num_accu);
    PrintObjInfo(fusInfo);

    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right_Raw(&fusInfo);
    OutputToFd("Font Right ObjIndex %d num %d, accu %d\n", data[1], fusInfo.num,
               fusInfo.num_accu);
    PrintObjInfo(fusInfo);

    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Left_Raw(&fusInfo);
    OutputToFd("Rear Left ObjIndex %d num %d, accu %d\n", data[1], fusInfo.num,
               fusInfo.num_accu);
    PrintObjInfo(fusInfo);

    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Right_Raw(&fusInfo);
    OutputToFd("Rear Right ObjIndex %d num %d, accu %d\n", data[1], fusInfo.num,
               fusInfo.num_accu);
    PrintObjInfo(fusInfo);
}

/**
 * @brief 显示标定信息，逐条打印直到没有更多数据
 *
 * @param data 输入数据（未直接使用，仅保持接口一致）
 */
static void ShowCalibrate(const uint8_t data[candata_bytes])
{
    char buff[128];
    int index = 0;
    while (PK_CalibrationPrint(index, buff) > 0)
    {
        index++;
        OutputToFd("%s\n", buff);
    }
}

/**
 * @brief 测试自动泊车辅助命令，根据输入距离控制车辆移动
 *
 * @param data 输入数据，第1字节为距离和方向信息（有符号int8）
 */
static void TestApaCmd(const uint8_t data[candata_bytes])
{
    int startCountL = RTE_BSW_Get_WheelCounter_RL();
    int startCountR = RTE_BSW_Get_WheelCounter_RR();

    OutputToFd("Test Apa Cmd Start!\n");
    int curAngle = RTE_BSW_Get_EPS_Angle();
    printf("cur Angle = %d\n", curAngle);

    usleep(20 * 1000);
    int count = 0;

    int input      = int8_t(data[1]);
    float distance = fabs(input);
    int dir        = sign(input);
    OutputToFd("start test %f %d\n", distance, dir);
    RTE_PK_StateManage_Set_ModuleCom_ObjAvoid(MCOM_ON_PARKING);
    while (distance > 0.01)
    {
        int curCountL = RTE_BSW_Get_WheelCounter_RL();
        int curCountR = RTE_BSW_Get_WheelCounter_RR();
        int runCountL = (curCountL + 1022 - startCountL) % 1022;
        int runCountR = (curCountR + 1022 - startCountR) % 1022;

        count++;
        if ((count % 20) == 0 || distance < 0.3)
        {
            OutputToFd("Test Apa Cmd Continue: %d Left distance %f, run plus %d\n", count,
                       distance, runCountR + runCountL);
        }

        distance    = distance - (runCountR + runCountL) * Rear_wheel_fac_ds;
        startCountL = curCountL;
        startCountR = curCountR;

        RTE_PK_PathExecute_Set_TargVelspd(0.6 * dir);
        RTE_PK_PathExecute_Set_TargEPS_Angle(0);
        RTE_PK_PathExecute_Set_rx_brake(
            distance); //  20181101: 0.5 -> 0.45 suit for pk_objavoid to send ccd_st

        usleep(20 * 1000);
    }

    OutputToFd("Test Apa Cmd End! %f\n", distance);
    RTE_PK_StateManage_Set_ModuleCom_ObjAvoid(MCOM_OFF);
}

/**
 * @brief 恢复自动泊车运行（取消暂停）
 * @param data 未使用
 */
static void ResumeRun(const uint8_t data[candata_bytes]) { pauseRecover = 0; }

/**
 * @brief 暂停自动泊车运行
 * @param data 未使用
 */
static void PauseRun(const uint8_t data[candata_bytes]) { pauseRecover = 1; }

/**
 *
 * @brief 打印各模块当前状态，状态用字符串表示
 *
 * @param data CAN数据，未使用
 */
static void ShowModState(const uint8_t data[candata_bytes])
{
    const char *state[] = {"MCOM_INIT",
                           "MCOM_ON",
                           "MCOM_OFF",
                           "MCOM_DATCLC",
                           "MCOM_ON_PARKING",
                           "MCOM_ON_PARKINGOUT",
                           "MCOM_ON_SUMMONUP_BACK",
                           "MCOM_ON_SUMMONUP_FOR",
                           "MCOM_ON_SUMMONUP_STOP",
                           "MCOM_ON_PARKING_PAUSE",
                           "MCOM_ON_PARKINGOUT_PAUSE"};

    auto ModeFunc = [](PK_ModuleComType (*func)(), const char *state[],
                       const int len) -> const char * {
        int index = func();
        return (index < len) ? state[index] : "INVALID STATE";
    };

    int stateNum = sizeof(state) / sizeof(state[0]);
    OutputToFd("location : %s\n",
               ModeFunc(RTE_PK_StateManage_Get_ModuleCom_Location, state, stateNum));
    OutputToFd("slotdetect : %s\n",
               ModeFunc(RTE_PK_StateManage_Get_ModuleCom_SlotDetect, state, stateNum));
    OutputToFd("objavoid : %s\n",
               ModeFunc(RTE_PK_StateManage_Get_ModuleCom_ObjAvoid, state, stateNum));
    OutputToFd("pathplan : %s\n",
               ModeFunc(RTE_PK_StateManage_Get_ModuleCom_PathPlan, state, stateNum));
    OutputToFd("pathexecute : %s\n",
               ModeFunc(RTE_PK_StateManage_Get_ModuleCom_PathExecute, state, stateNum));
    OutputToFd("distctrl : %s\n",
               ModeFunc(RTE_PK_StateManage_Get_ModuleCom_DistCtrl, state, stateNum));
    OutputToFd("sensorfusion : %s\n",
               ModeFunc(RTE_PK_StateManage_Get_ModuleCom_SensorFusion, state, stateNum));
    OutputToFd("speedctrl : %s\n",
               ModeFunc(RTE_PK_StateManage_Get_ModuleCom_SpeedCtrl, state, stateNum));
    OutputToFd("execute : %d\n", RTE_PK_PathExecute_Get_ModuleState_PathExecute());
    OutputToFd("statemode : %d\n", RTE_SM_Get_ApaWorkState());
}

/**
 * @brief 解析录制命令参数
 * @param cmd 用户输入的命令参数字符串（不包含命令名称）
 * @param cmdData 输出数据缓冲区
 * @return 0成功，非0失败
 */
static int RecordCmdParas(const char *cmd, uint8_t cmdData[candata_bytes])
{
    int switch_flag;
    char record_name[30] = {0};

    // 解析参数格式: <switch_flag> [record_name]
    // 例如: "2" 或 "2 /path/to/file.bin"
    int parsed_count = sscanf(cmd, "%d %29s", &switch_flag, record_name);

    if (parsed_count < 1)
    {
        // 没有找到任何参数
        printf("Error: Missing switch_flag parameter\n");
        return -1;
    }

    // 验证开关标志范围 (0, 1, 2)
    if (switch_flag < 0 || switch_flag > 2)
    {
        printf("Error: Invalid switch_flag %d. Must be 0, 1, or 2\n", switch_flag);
        return -2;
    }

    // 清空数据缓冲区
    memset(cmdData, 0, candata_bytes);

    // 填充开关标志到 data[1]
    cmdData[1] = (uint8_t)switch_flag;

    // 如果有路径参数，填充到 data[2] 之后
    if (parsed_count >= 2 && record_name[0] != '\0')
    {
        // 确保路径不以空格开头
        const char *path_start = record_name;
        while (*path_start == ' ') path_start++;

        if (*path_start != '\0')
        {
            strncpy((char *)&cmdData[2], path_start, sizeof(record_name) - 1);
            cmdData[2 + sizeof(record_name) - 1] = '\0'; // 确保字符串终止
        }
    }

    printf("Parse success: switch_flag=%d", switch_flag);
    if (record_name[0] != '\0')
    {
        printf(", path=%s", record_name);
    }
    printf("\n");

    return 0; // 成功
}

/**
 * @brief 解析录制命令
 *
 * record_name 未实现
 *
 * @param data
 */
static void RecordCmd(const uint8_t data[candata_bytes])
{
    DebugCmdType cmd_temp           = {};
    cmd_temp.record_cmd.cmd_id      = 1;
    cmd_temp.record_cmd.switch_flag = data[1];
    RTE_BSW_Set_Debug_Command(&cmd_temp);
}

/**
 * @brief 解析 ussdata 命令参数
 * @param cmd 用户输入的命令参数字符串（不包含命令名称）
 * @param cmdData 输出数据缓冲区
 * @return 0成功，非0失败
 */
static int UssDataCmdParas(const char *cmd, uint8_t cmdData[candata_bytes])
{
    // 清空数据缓冲区
    memset(cmdData, 0, candata_bytes);

    int loop_count = 0;
    int parsed     = sscanf(cmd, "%d", &loop_count);

    if (parsed != 1)
    {
        printf("Error: Missing loop count parameter\n");
        return -1;
    }

    if (loop_count <= 0 || loop_count > 255)
    {
        printf("Error: Invalid loop count %d. Must be in [1, 255]\n", loop_count);
        return -2;
    }

    // 把循环次数放在 data[1]（data[0] 一般是预留/命令号）
    cmdData[1] = (uint8_t)loop_count;
    return 0;
}

/**
 * @brief 异步打印超声雷达数据，循环指定次数，每次读取一帧所有雷达信息。
 *
 * 该函数从 `cmdData[1]` 中读取循环次数，并在异步线程中每隔 2
 * 秒打印一次所有超声雷达位置的原始数据， 包括时间戳、距离、强度、脉宽等信息。
 *
 * @param data 命令数据，data[1] 表示读取次数（即循环次数）
 */
static void UssDataCmd(const uint8_t data[candata_bytes])
{
    command[0]      = data[1];
    auto outputFunc = [](void *paras) -> void * {
        int count                = command[0];
        const char *position[24] = {
            "FOL",     "FCL",     "FCR",     "FOR",     "ROL",     "RCL",
            "RCR",     "ROR",     "FSL",     "FSR",     "RSL",     "RSR",
            "FCL_FOL", "FCL_FCR", "FCR_FOR", "FCR_FCL", "RCL_ROL", "RCL_RCR",
            "RCR_ROR", "RCR_RCL", "FSL_2",   "FSR_2",   "RSL_2",   "RSR_2",
        };
        U_RadarType raw_radar;
        const float *value[24] = {
            &raw_radar.FOL,     &raw_radar.FCL,     &raw_radar.FCR,
            &raw_radar.FOR,     &raw_radar.ROL,     &raw_radar.RCL,
            &raw_radar.RCR,     &raw_radar.ROR,     &raw_radar.FSL,
            &raw_radar.FSR,     &raw_radar.RSL,     &raw_radar.RSR,
            &raw_radar.FCL_FOL, &raw_radar.FCL_FCR, &raw_radar.FCR_FOR,
            &raw_radar.FCR_FCL, &raw_radar.RCL_ROL, &raw_radar.RCL_RCR,
            &raw_radar.RCR_ROR, &raw_radar.RCR_RCL, &raw_radar.FSL_2,
            &raw_radar.FSR_2,   &raw_radar.RSL_2,   &raw_radar.RSR_2,
        };

        U_RadarDataType radar[24];
        for (int loop = 0; loop < count; loop++)
        {
            OutputToFd("count = %d start\n", loop);
            RTE_PD_Get_U_RadarExt(radar);
            OutputToFd("--------------------RTE_PD_Get_U_RadarExt -------------------\n");
            for (uint32_t i = 0; i < 24; i++)
            {
                OutputToFd(" %s : time %d, dist %d, peak %d, wide %d\n", position[i],
                           radar[i].Timestamp, radar[i].Distance, radar[i].PeakLevel,
                           radar[i].WaveWidth);
            }

            OutputToFd("--------------------RTE_PD_Get_U_Radar -------------------\n");
            RTE_PD_Get_U_Radar(&raw_radar);
            for (size_t i = 0; i < 24; i++)
            {
                OutputToFd(" %s: dist = %.2f\n", position[i], *value[i]);
            }
            OutputToFd("count = %d end\n", loop);
            sleep(1);
        }

        return NULL;
    };

    ASyncRun(outputFunc, NULL, "dbg_out_uss");
}

// 命令行函数声明
static void HelpCmd(const uint8_t data[candata_bytes]);
// 命令信息结构体数组，定义了所有命令及参数解析
static COMMAND_LINE_INFO cmdLines[] = {
    {HelpCmd, false, NULL, "help", "help"},
    {OpenPrintPts, false, Parse1U8Paras, "setoutputpts",
     "setoutputpts id --- output to special pts;"},
    {ShowCcpVal, false, Parse1U8Paras, "showccp", "showccp count: 10 ~ 99;"},
    {ShowAvmBuff, false, NULL, "showamvbuf", "showamvbuf --- print received amv buff"},
    {ShowPathsInfo, false, NULL, "showpath", "showpath --- print apa path"},
    {ShowParkPath, false, NULL, "showparkpath", "showparkpath --- print park path"},
    {ShowAllSlots, false, NULL, "showslot", "showslot --- print slot path"},
    {ShowCurPos, false, NULL, "showcurpos", "showcurpos --- print current position"},
    {TestApaCmd, true, Parse1U8Paras, "testrun", "testrun --- start test control"},
    {ResumeRun, false, NULL, "resume", "resume --- stop apa"},
    {PauseRun, false, NULL, "pause", "pause --- stop apa"},
    {ShowObjInfo, false, NULL, "showobjinfo", "showobjinfo --- show ultronic objs"},
    {ShowRawObjInfo, false, NULL, "showrawobjinfo",
     "shorawwobjinfo --- show raw ultronic objs"},
    {ShowModState, false, NULL, "showmodstate", "showmodstate --- show mod state"},
    {ShowVerInfo, false, NULL, "showversion", "showversion ---- print version and stamp"},
    {SetConsolePts, false, Parse1U8Paras, "setconsolepts", "setconsolepts id"},
    {SetLogLevel, false, Parse1U8Paras, "setloglevel", "setloglevel 0~3"},
    {ShowCalibrate, false, NULL, "showcalibrate", "showcalibrate"},
    {ShowBObjInfo, false, NULL, "showparkobjinfo", "showparkobjinfo"},
    {RecordCmd, false, RecordCmdParas, "record", "record"},
    {UssDataCmd, false, UssDataCmdParas, "ussdata", "ussdata"},

};

/**
 * @brief 帮助命令，打印所有命令及说明
 * @param data CAN数据，未使用
 */
static void HelpCmd(const uint8_t data[candata_bytes])
{
    OutputToFd("apa version: %s stamp: %s %s\n", VERSION_CONFIG, __DATE__, __TIME__);
    for (uint32_t cmdIndex = 0; cmdIndex < sizeof(cmdLines) / sizeof(cmdLines[0]);
         cmdIndex++)
    {
        OutputToFd("%d. %s\n", cmdIndex + 1, cmdLines[cmdIndex].cmdHelp);
    }
}

static int s_pipeFd = -1;
/**
 * @brief 命令管道循环线程
 *
 * 该线程负责从命名管道中读取用户输入的命令，并解析后执行相应的回调函数。
 * 命令行解析流程：
 * 1. 确保管道文件存在，不存在则创建。
 * 2. 循环等待用户输入命令。
 * 3. 过滤无效字符（非法字符、大写字母、连续空格等）。
 * 4. 匹配命令表 `cmdLines`。
 * 5. 调用对应命令的解析函数 parseFunc，将解析结果保存到 cmdData。
 * 6. 如果解析成功，则调用对应命令的执行函数 runFunc。
 * 7. 未识别命令时打印错误日志。
 *
 * @param paras 命名管道路径（const char*）
 * @return void* 线程退出时返回 NULL
 */
static void *DebugLoop(void *paras)
{
    const char *pipename = (char *)paras;
    char buffer[256];  ///< 存储从管道读取的命令字符串
    int result;        ///< read/parse 返回值
    uint32_t cmdIndex; ///< 遍历命令表时使用的索引

    // ---------------- 初始化命令管道 ----------------
    if (access(pipename, F_OK) == 0)
    {
        // 如果管道文件已存在，先清空
        s_pipeFd = open(pipename, O_CREAT | O_TRUNC | O_RDWR, 0666);
        close(s_pipeFd);
    }

    // 创建命名管道（FIFO）
    mkfifo(pipename, 0777);

    // 以只读方式打开命名管道
    s_pipeFd = open(pipename, O_RDONLY);
    ASSERT(s_pipeFd > 0);
    if (s_pipeFd <= -1)
    {
        return NULL;
    }

    int count = 1;                    ///< 命令计数器（用于输出日志）
    struct timeval curStamp;          ///< 保存当前时间戳
    uint8_t l_cmdData[candata_bytes]; ///< 命令解析结果缓冲区

    // ---------------- 主循环：不断读取管道命令 ----------------
    while (1)
    {
        memset(buffer, 0, sizeof(buffer));
        memset(l_cmdData, 0, sizeof(l_cmdData));

        // 从管道读取命令数据
        result = read(s_pipeFd, buffer, sizeof(buffer) - 1);
        if (result > 0)
        {
            buffer[result] = '\0'; // 保证字符串以 '\0' 结尾

            // ---------------- 命令字符串预处理 ----------------
            for (int index = 0; index < result; index++)
            {
                if (!isalnum(buffer[index]) && buffer[index] != ' ')
                {
                    buffer[index] = '\0'; // 非字母数字和空格 → 截断
                    break;
                }
                if (isupper(buffer[index]))
                {
                    buffer[index] = '\0'; // 大写字母不允许 → 截断
                    break;
                }
                if (buffer[index] == ' ' && index + 1 < result &&
                    buffer[index + 1] == ' ')
                {
                    buffer[index] = '\0'; // 连续空格 → 截断
                    break;
                }
            }

            // ---------------- 命令匹配与执行 ----------------
            int found = -1;
            for (cmdIndex = 0; cmdIndex < sizeof(cmdLines) / sizeof(cmdLines[0]);
                 cmdIndex++)
            {
                // 匹配命令表中的命令关键字
                if (strstr(buffer, cmdLines[cmdIndex].cmdName))
                {
                    found  = cmdIndex;
                    result = 0;
                    // 执行命令解析函数
                    if (cmdLines[cmdIndex].parseFunc)
                    {
                        result = cmdLines[cmdIndex].parseFunc(
                            buffer + strlen(cmdLines[cmdIndex].cmdName) + 1, l_cmdData);
                    }
                    //
                    if (result == 0)
                    {
                        l_cmdData[0] = cmdIndex + 1;
                    }
                    break; // 找到命令就退出循环
                }
            }

            // ---------------- 错误处理 ----------------
            if (found == -1)
            {
                // 未找到命令 → 输出错误日志
                gettimeofday(&curStamp, 0);
                OutputToFd("\n##%06d:%06d invalid command: %s\n", curStamp.tv_sec,
                           curStamp.tv_usec, buffer);
            }
            // ---------------- 执行命令 ----------------
            else if (l_cmdData[0] != 0)
            {
                gettimeofday(&curStamp, 0);
                OutputToFd("\n##%06d %06d:%06d command %s start:\n", count++,
                           curStamp.tv_sec, curStamp.tv_usec, cmdLines[found].cmdName);

                // 调用命令执行函数
                cmdLines[found].runFunc(l_cmdData);

                gettimeofday(&curStamp, 0);
                OutputToFd("\n%06d:%06d command %s end:\n", curStamp.tv_sec,
                           curStamp.tv_usec, cmdLines[found].cmdName);
            }
        }
        else
        {
            // 管道无数据时，休眠 100ms 避免 busy loop
            usleep(100 * 1000);
        }
    }

    close(s_pipeFd);
    return NULL;
}

/**
 * @brief 初始化调试功能，启动异步调试命令处理线程
 *
 * 通过异步运行 DebugLoop 函数实现调试命令的循环处理。
 */
void InitDebug() { ASyncRun(DebugLoop, (void *)"debugCmd", "dbg_cmd"); }
