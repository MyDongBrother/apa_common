#include "Eth_ResLog.h"
#include "EthIf.h"
#include "SystemPara.h"
#include "PK_Calibration.h"
#include "MathFunc.h"
#include "Rte.h"
#include "Rte_Func.h"
#include "Debug_Cmd.h"
#include <fstream>
#include <string>
#include "PK_StateManage.h"
#include "hobotlog/hobotlog.hpp"

void Eth_ResLog();

static void *LedResLog(void *paras)
{
    const char *input   = (char *)paras;
    const char *portPos = strstr((char *)input, ":");
    if (portPos == NULL)
    {
        return NULL;
    }

    int reportFd = socket(PF_INET, SOCK_DGRAM, 0);
    if (reportFd < 0)
    {
        return NULL;
    }

    char servStr[32], localStr[32], portStr[16];
    memset(servStr, 0, sizeof(servStr));
    memset(portStr, 0, sizeof(portStr));
    strncpy(servStr, input, rte_min((size_t)(portPos - input), sizeof(servStr)));
    strncpy(portStr, portPos + 1,
            rte_min((size_t)(strlen(input) - strlen(servStr) - 1), sizeof(portStr)));

    printf("server ip %s port %s\r\n", servStr, portStr);
    int port = atoi(portStr);

    struct sockaddr_in servaddr, localaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family      = AF_INET;
    servaddr.sin_port        = htons(port); // 7777
    servaddr.sin_addr.s_addr = inet_addr(servStr);

    int localPort = 18813;
    strcpy(localStr, local_ip_address);
    localaddr.sin_family = AF_INET;
    localaddr.sin_port   = htons(localPort); // 18813
    // localaddr.sin_port = htons(port);
    localaddr.sin_addr.s_addr = inet_addr(localStr);

    int flag = 1;
    setsockopt(reportFd, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag));
    if (bind(reportFd, (struct sockaddr *)&localaddr, sizeof(localaddr)) < 0)
    {
        printf("bind errno=%d %s\n", errno, strerror(errno));
    }

    int result;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(reportFd, &rfds);
    int maxfd              = reportFd;
    struct timeval timeout = {1, 0};

    // int startPark = 0;
    while (1)
    {
        FD_SET(reportFd, &rfds);
        timeout.tv_sec  = 0;
        timeout.tv_usec = 50000;

        result = select(maxfd + 1, &rfds, NULL, NULL, &timeout);
        if (result < 0)
        {
            printf("select errno=%d %s\n", errno, strerror(errno));
            break;
        }
        else if (result == 0)
        {
            static int EthSendTimeCnt = 0;
            std::string slotinfo;
            EthSendTimeCnt++;
            if (EthSendTimeCnt > 10)
            {
                EthSendTimeCnt = 0;
            }
        }
        else
        {
            unsigned int buf[11];
            memset(buf, 0, sizeof(buf));
            result = recvfrom(reportFd, buf, sizeof(buf), 0, NULL, NULL);
            if (result < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }
                break;
            }
            else if (result > 0)
            {
                Apa_WorkStateType sta = RTE_SM_Get_ApaWorkState();
                if (sta == CarSta_Passive || sta == CarSta_Serching)
                {
                    FILE *fp = NULL;
                    fp       = fopen("Ethenet_ResLog.txt", "a");
                    if (fp != NULL)
                    {
                        fprintf(fp,
                                "p_stack_frame:0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, "
                                "0x%x, cfsr_val:%x, SelectCarSpaceNum:%x, sendflag:%x\n",
                                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6],
                                buf[7], buf[8], buf[9], buf[10]);
                        fclose(fp);
                    }
                    else
                    {
                        printf("Ethenet_ResLog.c:fp is NULL\n");
                    }
                }
            }
            else
            {
            }
        }
    }

    close(reportFd);
    reportFd = -1;
    return NULL;
}

void Eth_ResLog()
{
    static char buf[32];
    memset(buf, 0, sizeof(buf));
#if 0
    std::ifstream fin("server.cfg");
    if (fin.is_open()) {
        fin.getline(buf, sizeof(buf));
        fin.close();
    } else {
        strcpy(buf, "192.168.2.13:8899");
    }
#else
    strcpy(buf, "192.168.1.102:7777");
    printf(
        "----------------led connect ip "
        "192.168.1.102:7777--------------------------\r\n");
#endif

    remove("Ethenet_ResLog.txt");

    ASyncRun(LedResLog, (void *)buf, "debug_hmi_log");
}