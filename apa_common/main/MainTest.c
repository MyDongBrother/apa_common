/**
 * @file MainLoop.c
 * @brief APA 主程序入口文件，负责初始化各个模块并运行主循环。
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-09-05
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-09-05 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */
#include "Debug_Cmd.h"
#include "PK_Location.h"
#include "PK_Odometry.h"
#include "PK_SlotDetect.h"
#include "PK_PathExecute.h"
#include "PK_SF.h"
#include "PK_ObjAvoid.h"
#include "PK_SensorT2S.h"
#include "PK_SceneRecog.h"
#include "PK_DistCtrl.h"
#include "Rte_BSW.h"
#include "Module_API.h"
#include "Rte_Func.h"
#include "CanIf.h"
#include "EthIf.h"
#include "DdsIf.h"
#include "UartIf.h"
#include "Ram.h"
#include "Record_Log.h"
#include "hobotlog/hobotlog.hpp"
#include "PK_StateManage.h"
#include "MathPara.h"
#include "Rte_ComIF.h"
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <chrono>

#define MAXLINE   1024
#define DEST_PORT 18809          // 目标地址端口号
#define DEST_IP   "192.168.1.51" // MCU的IP Address

/**
 * @brief 打印使用说明
 *
 * @param programe 程序名
 */
static void usage(char *programe)
{
    printf("invalid paras!\r\n");
    printf("%s: count [period] \r\n", strrchr(programe, '/'));
    printf("count   ---------  run count\r\n");
    printf(
        "logLevel  ---------  logLevel\t-d => debug \t-i => info \t-w=>warning "
        "\t-e=>error \t-f=>fatal \r\n");
}
/**
 * @brief 输出版本号到文件
 *
 * 根据编译时间生成版本信息，并写入到 "apa_version" 文件。
 */
static void OutPutVersion() {}
/**
 * @brief 创建日志路径
 *
 * 若路径不存在则自动创建。
 *
 * @param path 路径名
 */
static void CreatePath(const char *path)
{
    if (access(cfg_log_path, F_OK) == -1)
    {
        mkdir(cfg_log_path, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
    }
}
/**
 * @brief 检查应用是否重复运行
 *
 * 使用文件锁防止程序多开。
 *
 * @return true 已有实例运行
 * @return false 当前为首次运行
 */
static bool CheckAppMulti() { return false; }
static bool ReInitSystem()
{
    // 接口内存反初始化
    MEMORY_POOL_IMPL::ReInit();
}
/**
 * @brief 系统准备
 *
 * @return true 初始化成功
 * @return false 初始化失败
 */
static bool InitSystem()
{
    printf("=== InitSystem start ===\n");
    // 创建必要的文件夹
    CreatePath("/tmp/apalog/");
    CreatePath("./log/");

    // 忽略 SIGPIPE
    if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
    {
        printf("WARNING: fail to ignore SIGPIPE\n");
    }
    else
    {
        printf("SIGPIPE ignored\n");
    }

    // 检查平台 int 与 float 大小
    if (sizeof(int) != sizeof(float))
    {
        printf("ERROR: sizeof(int)=%zu != sizeof(float)=%zu\n", sizeof(int),
               sizeof(float));
        return false;
    }
    printf("Platform check passed (int vs float)\n");

    // 配置文件校验
    PK_Calibration();
    printf("Calibration done\n");

    // 配置本地 IP
    RTE_BSW_Set_LocalIPAddr(local_ip_address);
    printf("Local IP set: %s\n", local_ip_address);

    printf("=== InitSystem end ===\n");

    // 接口内存初始化
    MEMORY_POOL_IMPL::Init();
    return true;
}

/**
 * @brief 日志配置
 *
 * @param argc
 * @param argv
 */
static void InitLog(int argc, char *argv[])
{
    // 打印版本日志
    int month                = 0;
    const char *g_months[12] = {"Jan", "Feb", "Mar",  "Apr", "May", "Jun",
                                "Jul", "Aug", "Sept", "Oct", "Nov", "Dec"};
    for (int i = 0; i < 12; i++)
    {
        if (strncmp(g_months[i], __DATE__, 3) == 0)
        {
            month = i + 1;
            break;
        }
    }
    char version[128];
    sprintf(version, "%s date: %d %s %s\n", "2.1.25.2", month, __DATE__, __TIME__);
    FILE *fid = fopen("apa_version", "w");
    if (fid != NULL)
    {
        fwrite(version, strlen(version), 1, fid);
        fclose(fid);
    }

    // 打印ret接口的内存占用
    printf("ret uses a memory size of %zu\r\n", MEMORY_POOL_IMPL::GET_MEMORY_SIZE());

    // 设置日志等级
    if (argc == 3)
    {
        std::string log_level(argv[2]);
        if (log_level == "-i")
            SetLogLevel(HOBOT_LOG_INFO);
        else if (log_level == "-d")
            SetLogLevel(HOBOT_LOG_DEBUG);
        else if (log_level == "-w")
            SetLogLevel(HOBOT_LOG_WARN);
        else if (log_level == "-e")
            SetLogLevel(HOBOT_LOG_ERROR);
        else if (log_level == "-f")
            SetLogLevel(HOBOT_LOG_FATAL);
        else
        {
            usage(argv[0]);
            return;
        }
    }
    else
    {
        SetLogLevel(HOBOT_LOG_WARN);
    }
    Rte_BSW_Set_LogPath(cfg_log_path);
    CreatePath(cfg_log_path);
}

/**
 * @brief 硬件和通信初始化
 *
 */
static void InitHardwareAndComm()
{
#ifndef SIMULATE
    Apa_CanInit(NULL);
    Eth_InitLedDisplay();
    Eth_InitVisionSlot();
#endif
    Uart_InitRxThread();
    Dds_InitThread();
}

/**
 * @brief 算法模块初始化
 *
 */
static void InitAlgorithm()
{
    PK_PathPlan();
    PK_InitPathExecute();
    PK_DistanceCtrl();
}

/**
 * @brief 主循线程
 *
 *
 * @param paras 线程循环周期,单位ms
 * @return void*
 */
static void *MainLoop(void *paras)
{
    printf("start MainLoop.... \r\n");
    int collectPeriod = *(int *)paras;

    // 设置线程为实时调度 SCHED_FIFO，优先级最大
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    // 获取初始时间
    struct timespec ts, now_ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    // 主循环
    while (1)
    {
        // 更新算法系统时间
        struct timeval stamp;
        gettimeofday(&stamp, 0);
        uint64_t system_time_ms = stamp.tv_sec * 1000 + stamp.tv_usec / 1000;
        RTE_BSW_Set_CurTime(system_time_ms);

#ifdef DEBUG_VERSION
        // 检查录制回灌调试功能
        MEMORY_POOL_IMPL::REPLAY();
#endif
        // 状态机模块
        PK_SM_Main();
        // 定位模块
        PK_Location();
        // PK_Odometry();
        /* PK_SensorT2S_CacheLocation must follow PK_Location!!! */
        // 超声波传感器时间同步
        PK_SensorT2S_CacheLocation();
        PK_SensorT2S();

        // 超声波雷达避障模块
        PK_ObjAvoid();

        // 路径追踪
        PK_PathExecuteRun();

        // 路径控制
        PK_DistanceCtrlRun_yuan();

        // 控制周期
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        if ((now_ts.tv_sec > ts.tv_sec) ||
            (now_ts.tv_sec == ts.tv_sec && now_ts.tv_nsec > ts.tv_nsec))
        {
            printf("MainLoop overrun! current=%ld.%09ld, target=%ld.%09ld\n",
                   now_ts.tv_sec, now_ts.tv_nsec, ts.tv_sec, ts.tv_nsec);
            ts = now_ts;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        ts.tv_nsec += collectPeriod * 1000 * 1000;
        if (ts.tv_nsec >= 1000000000)
        {
            ts.tv_sec += 1;
            ts.tv_nsec -= 1000000000;
        }
    }

    printf("MainLoop end!\r\n");
    return NULL;
}

/**
 * @brief APA 融合模块线程
 *
 * @param paras 输入参数数组，paras[1] 为周期（us）
 * @return void*
 */
static void *FusionLoop(void *paras)
{
    printf("start FusionLoop.... \r\n");
    int collectPeriod = *(int *)paras;

    // 设置线程为实时调度 SCHED_FIFO，优先级最大
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    // 获取初始时间
    struct timespec ts, now_ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    while (1)
    {
        // 感知融合
        PK_SF_Main();

        // 车位管理
        PK_SlotDetect();

        // 控制周期
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        if ((now_ts.tv_sec > ts.tv_sec) ||
            (now_ts.tv_sec == ts.tv_sec && now_ts.tv_nsec > ts.tv_nsec))
        {
            printf("FusionLoop overrun! current=%ld.%09ld, target=%ld.%09ld\n",
                   now_ts.tv_sec, now_ts.tv_nsec, ts.tv_sec, ts.tv_nsec);
            ts = now_ts;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        ts.tv_nsec += collectPeriod * 1000 * 1000;
        if (ts.tv_nsec >= 1000000000)
        {
            ts.tv_sec += 1;
            ts.tv_nsec -= 1000000000;
        }
    }
    printf("FusionLoop end!\r\n");
    return NULL;
}

/**
 * @brief 主循线程
 *
 *
 * @param paras 线程循环周期,单位ms
 * @return void*
 */
static void *PlanLoop(void *paras)
{
    printf("start PlanLoop.... \r\n");

    uint32_t collectPeriod = 300; // 默认规划周期 300ms

    // 获取初始时间
    struct timespec ts, now_ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    while (1)
    {
        // 如果当前模块状态是泊车状态中，则使用更高频率规划（80ms）
        (RTE_PK_StateManage_Get_ModuleCom_PathPlan() == MCOM_ON_PARKING)
            ? collectPeriod = 80
            : collectPeriod = 300;

        // 路径规划
        PK_PathPlanRun();

        // 控制周期
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        if ((now_ts.tv_sec > ts.tv_sec) ||
            (now_ts.tv_sec == ts.tv_sec && now_ts.tv_nsec > ts.tv_nsec))
        {
            printf("MainLoop overrun! current=%ld.%09ld, target=%ld.%09ld\n",
                   now_ts.tv_sec, now_ts.tv_nsec, ts.tv_sec, ts.tv_nsec);
            ts = now_ts;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        ts.tv_nsec += collectPeriod * 1000 * 1000;
        if (ts.tv_nsec >= 1000000000)
        {
            ts.tv_sec += 1;
            ts.tv_nsec -= 1000000000;
        }
    }
    printf("PlanLoop end!\r\n");
    return NULL;
}

/**
 * @brief 诊断线程
 *
 * 周期性检查超声波雷达、IMU、轮速、EPS/ESC 状态，若异常则输出提示。
 *
 * @param paras 未使用
 * @return void*
 */
static void *DiagnoseLoop(void *paras)
{
    while (1)
    {
        U_RadarType radar;
        RTE_PD_Get_U_Radar(&radar);

        if (radar.FOL == 0 && radar.FCL == 0 && radar.FCR == 0 && radar.FSL == 0 &&
            radar.RSL == 0)
        {
            printf("-------------- No Ultra Data ------------------\r\n");
        }

        float speeds[3];
        RTE_BSW_Get_Imuspd(speeds);
        float speed =
            sqrtf(speeds[0] * speeds[0] + speeds[1] * speeds[1] + speeds[2] * speeds[2]);
        if (speed < ZERO_FLOAT)
        {
            printf("-------------- No Imu Data ------------------\r\n");
        }

        if (speed > 0.1)
        {
            if (RTE_BSW_Get_WheelCounter_RL() + RTE_BSW_Get_WheelCounter_RR() == 0)
            {
                sleep(5);
            }
            if (RTE_BSW_Get_WheelCounter_RL() + RTE_BSW_Get_WheelCounter_RR() == 0)
            {
                printf("-------------- No Wheel Data ------------------\r\n");
            }
        }

        PK_ModuleComType modState = RTE_PK_StateManage_Get_ModuleCom_PathExecute();
        PK_ModuleStateType state  = RTE_PK_PathExecute_Get_ModuleState_PathExecute();
        if (state == MSTAT_NORM && modState == MCOM_ON_PARKING)
        {
            uint8_t epsStat = RTE_BSW_Get_EpsCtrlStat();
            uint8_t escStat = RTE_BSW_Get_EscApaStat();
            if (epsStat != 2 || escStat != 2)
            {
                printf("-------------eps state %d esc state %d ----------------\r\n",
                       epsStat, escStat);
            }
        }

        sleep(5);
    }
}

/**
 * @brief 程序入口
 *
 * 1. 输出版本信息
 * 2. 初始化日志等级
 * 3. 检查单实例运行
 * 4. 初始化硬件与中间件接口（CAN、ETH、UART 等）
 * 5. 初始化 APA 各个功能模块
 * 6. 启动 MainLoop 和 FusionLoop 线程
 *
 * @param argc 参数个数
 * @param argv 参数列表
 * @return int
 */
int main(int argc, char *argv[])
{
    /// 系统资源初始化
    if (!InitSystem())
    {
        return 2;
    }
    /// 日志初始化
    InitLog(argc, argv);
#ifdef DEBUG_VERSION
    /// debug初始化
    InitDebug();
#endif
    /// 硬件通信初始化
    InitHardwareAndComm();
    /// 功能模块初始化
    InitAlgorithm();
    /// 主线程
    static int period_main_ms = 20;
    pthread_t collectTh       = SyncRun(MainLoop, &period_main_ms, "apa_main");
    /// 融合算法线程
    static int period_fusion_ms = 50;
    ASyncRun(FusionLoop, &period_fusion_ms, "apa_fusion");
    /// 路径规划线程
    ASyncRun(PlanLoop, (void *)NULL, "apa_plan");
    /// 诊断线程
    /// ASyncRun(DiagnoseLoop, (void *)NULL, "apa_diag");
    /// 等待主线程结束
    WaitSyncRun(collectTh);
    /// 系统资源释放
    ReInitSystem();
    return 0;
}
