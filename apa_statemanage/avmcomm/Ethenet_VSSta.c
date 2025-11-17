#include "EthIf.h"
#include "PK_Calibration.h"
#include "Record_Log.h"
#include "Rte.h"
#include "Ram.h"
#include "PK_Location.h"
#include "Rte_Func.h"
#include "MathFunc.h"
#include <string>
#include "PK_StateManage.h"
#include "Ethenet_PushCan.h"

#define APA2AVM_PORT 18811

const uint32_t item_count = 75; // 20ms * 30 = 400ms
static FILE *avp_log      = NULL;

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static inline FILE *OpenLogFile(const char *name)
{
    // 检查并创建目录
    const char *dirName = "./avp_log";
    if (access(dirName, F_OK) != 0) // 检查目录是否存在
    {
        // 目录不存在，尝试创建
        if (mkdir(dirName, 0755) != 0)
        {
            perror("Failed to create log directory");
            return NULL; // 创建失败，返回 NULL
        }
    }

    // 构建文件名
    char fileName[128] = {0};
    struct tm absStamp;
    RTE_BSW_Get_AbsStamp(&absStamp);
    snprintf(fileName, sizeof(fileName) - 1, "./avp_log/%d_%02d_%02d_%02d_%02d_%02d_%s",
             absStamp.tm_year, absStamp.tm_mon, absStamp.tm_mday, absStamp.tm_hour,
             absStamp.tm_min, absStamp.tm_sec, name);

    // 打开日志文件
    FILE *logFile = fopen(fileName, "w");
    if (!logFile)
    {
        perror("Failed to open log file");
        return NULL;
    }
    return logFile;
}

static void GetVisionSta(const char *str)
{
    // int Vstatus;
    // int number = sscanf(str, "%d", &Vstatus);

    VisionState l_VisionState = VisionState_Alive;
    RTE_PK_SM_SetVisiInfo(l_VisionState);
}

static void SetVisionFault(const char * /*errstr */)
{
    VisionState l_VisionState = VisionState_Invalid;
    RTE_PK_SM_SetVisiInfo(l_VisionState);
}

static void SetManageSta(std::string &buff)
{
    buff.clear();
    buff.append("");

    int Vstatus = RTE_SM_Get_ApaWorkState();
    buff += std::to_string(Vstatus);
}

static void *DetectVisionStaLoop(void *paras)
{
    const char *portStr = (char *)paras;
    if (portStr == NULL)
    {
        return NULL;
    }

    int reportFd = socket(PF_INET, SOCK_DGRAM, 0);
    if (reportFd < 0)
    {
        return NULL;
    }

    printf("DetectVisionStaLoop port %s\r\n", portStr);
    int port = atoi(portStr);

    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    char localIp[16] = {0};
    RTE_BSW_Get_LocalIPAddr(localIp);

    servaddr.sin_family      = AF_INET;
    servaddr.sin_port        = htons(port); // 12345
    servaddr.sin_addr.s_addr = inet_addr(localIp);

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(reportFd, &rfds);
    int maxfd = reportFd;

    printf("wait connect ...\r\n");

    struct timeval timeout = {1, 0};

    int errCount = 0;
    while (1)
    {
        fd_set curRdfs  = rfds;
        timeout.tv_sec  = 0;
        timeout.tv_usec = 100000;

        int result = select(maxfd + 1, &curRdfs, NULL, NULL, &timeout);
        if (result < 0)
        {
            log_warn("select errno=%d %s\n", errno, strerror(errno));
        }
        else if (result == 0)
        {
            std::string slotinfo;
            SetManageSta(slotinfo);
            result = sendto(reportFd, slotinfo.c_str(), slotinfo.length(), 0,
                            (struct sockaddr *)&servaddr, sizeof(servaddr));
            if ((errCount++ % 10) == 1)
            {
                SetVisionFault(nullptr);
            }
        }
        else
        {
            char buf[256];
            memset(buf, 0, sizeof(buf));
            result = recvfrom(reportFd, buf, sizeof(buf), 0, NULL, NULL);
            if (result < 0)
            {
                log_err("recvfrom errno=%d %s\n", errno, strerror(errno));
            }
            else if (result > 0)
            {
                GetVisionSta(buf);
            }

            errCount = 0;
        }
    }

    close(reportFd);
    reportFd = -1;

    return NULL;
}

static void *ReportCanDataLoop(void *paras)
{
    struct sockaddr_in remote_addr;

    remote_addr.sin_family      = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(local_ip_address);
    remote_addr.sin_port        = htons(APA2AVM_PORT); // 18811

    int socketFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketFd < 0)
    {
        perror("creat socket fail\n");
        return NULL;
    }

    while (1)
    {
        if (CarSta_Passive == RTE_SM_Get_ApaWorkState() ||
            CarSta_PowerOn == RTE_SM_Get_ApaWorkState())
        {
            int ret = sendto(socketFd, "$PAUSE$", sizeof("$PAUSE$"), 0,
                             (struct sockaddr *)&remote_addr,
                             sizeof(remote_addr)); // 发送暂停黑芝麻环视摄像头推流指令
            if (ret < 0)
            {
                log_warn("send to avm failed %d %s\n", errno, strerror(errno));
            }
            else if (ret > 0)
            {
                // printf("%s %d-----send $PAUSE$\n",__FILE__,__LINE__);
            }
        }
        else if (CarSta_Serching == RTE_SM_Get_ApaWorkState() ||
                 CarSta_GuidanceActive == RTE_SM_Get_ApaWorkState() ||
                 CarSta_ParkAssist_Standby == RTE_SM_Get_ApaWorkState())
        {
            int ret = sendto(socketFd, "$RESUME$", sizeof("$RESUME$"), 0,
                             (struct sockaddr *)&remote_addr,
                             sizeof(remote_addr)); // 发送恢复黑芝麻环视摄像头推流指令
            if (ret < 0)
            {
                log_warn("send to avm failed %d %s\n", errno, strerror(errno));
            }
            else if (ret > 0)
            {
                // printf("%s %d-----send $RESUME$\n",__FILE__,__LINE__);
            }
        }

        uint8_t data[10];
        BuildAvmCanData1(data);
        int ret = sendto(socketFd, data, sizeof(data), 0, (struct sockaddr *)&remote_addr,
                         sizeof(remote_addr));
        if (ret < 0)
        {
            log_warn("send to avm failed %d %s\n", errno, strerror(errno));
        }

        BuildAvmCanData2(data);
        ret = sendto(socketFd, data, sizeof(data), 0, (struct sockaddr *)&remote_addr,
                     sizeof(remote_addr));

        usleep(20 * 1000);
    }

    close(socketFd); // 4
    return NULL;
}

void Eth_InitVisionSta()
{
#ifndef SIMULATE
    ASyncRun(DetectVisionStaLoop, (void *)"12345", "apa_eth_avm1");
    ASyncRun(ReportCanDataLoop, (void *)"NULL", "apa_eth_avm2");
#endif
}
