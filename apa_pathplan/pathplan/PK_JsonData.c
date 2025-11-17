#include "Rte.h"
#include "Rte_Func.h"
#include "MathFunc.h"
#include "PK_PathPlan.h"
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

static int reportFd         = -1;
static char recv_buff[4096] = {0};
const int local_port        = 18810;

static int PathPlan_InitLocalFd();
static void *PathPlan_RecvData(void *msg);
#ifndef DEBUG_PATHEXECUTE
void PK_ProcJsonCmd(const char *recv, const int len) {}
#endif

void PK_InitJsonReport()
{
    int result = PathPlan_InitLocalFd();
    if (result < 0)
    {
        return;
    }

    auto RecvFunc = [](void *paras) -> void * {
        PathPlan_RecvData(paras);
        return NULL;
    };

    ASyncRun(RecvFunc, NULL, "dbg_plan_recv");
}

int PK_SendJsonData(const char *buff, const int len)
{
    if (reportFd < 0 || buff == NULL || len <= 0)
    {
        return -1;
    }

    int sendlen = 0;
    while (sendlen < len)
    {
        int result = sendto(reportFd, &buff[sendlen], len - sendlen, 0, NULL, 0);
        if (result > 0)
        {
            sendlen = sendlen + result;
        }
        else
        {
            // printf("result %d errno=%d %s\n", result , errno, strerror(errno));
            break;
        }
    }
    return sendlen;
}

static int PathPlan_InitLocalFd()
{
    int result    = 0;
    auto erroFunc = [&result](const char *info) {
        printf("%s : result = %d errno=%d %s\n", info, result, errno, strerror(errno));
        if (reportFd > 0)
        {
            close(reportFd), reportFd = -1;
        }
        return -1;
    };

    reportFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (reportFd < 0)
    {
        return erroFunc("socket 0");
    }

    struct sockaddr_in localAddr;
    memset(&localAddr, 0x0, sizeof(struct sockaddr_in));
    localAddr.sin_family      = AF_INET;
    localAddr.sin_port        = htons(local_port); // 18810
    localAddr.sin_addr.s_addr = inet_addr(local_ip_address);

    int opt = 1;
    result  = setsockopt(reportFd, SOL_SOCKET, SO_REUSEADDR, (void *)&opt, sizeof(opt));
    if (result < 0)
    {
        return erroFunc("setsockopt 0");
    }

    opt    = 10 * 1024;
    result = setsockopt(reportFd, SOL_SOCKET, SO_SNDBUF, &opt, sizeof(opt));
    if (result < 0)
    {
        erroFunc("setsockopt sendbuff 1");
    }

    result = bind(reportFd, (struct sockaddr *)&localAddr, sizeof(localAddr));
    if (result < 0)
    {
        return erroFunc("bind 0");
    }

    return 0;
}

static void *PathPlan_RecvData(void *msg)
{
    int result = 0;

    auto erroFunc = [&result](const char *info) {
        printf("%s : result = %d errno=%d %s\n", info, result, errno, strerror(errno));
    };

    struct sockaddr_in remoteAddr;

    while (1)
    {
        memset(&remoteAddr, 0x0, sizeof(sockaddr_in));
        socklen_t socklen = sizeof(sockaddr_in);
        result            = recvfrom(reportFd, recv_buff, sizeof(recv_buff), 0,
                                     (sockaddr *)&remoteAddr, &socklen);
        if (result < 0)
        {
            erroFunc("recvfrom");
            break;
        }

        recv_buff[result] = 0;
        PK_ProcJsonCmd(recv_buff, result);

        socklen = sizeof(sockaddr_in);
        result  = connect(reportFd, (sockaddr *)&remoteAddr, socklen);
        if (result < 0)
        {
            erroFunc("connect 1");
            continue;
        }
        printf("connected socket ip: %s port:%d\n", inet_ntoa(remoteAddr.sin_addr),
               remoteAddr.sin_port);
    }

    close(reportFd);
    reportFd = -1;
    return NULL;
}
