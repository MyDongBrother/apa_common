#include "EthIf.h"
#include "MathFunc.h"
#include "PK_StateManage.h"

static void RtHeartBeatChk(const char *str)
{
    char headBuf[100];
    const char head[]    = "HeartBeatCnt:";
    uint8_t HeartBeatCnt = 0;

    memset(headBuf, 0, sizeof(headBuf));
    sscanf(str, "%s %d", headBuf, (int *)&HeartBeatCnt);

    if (!strcmp(headBuf, head))
    {
        printf("HeartBeatPage do not Resever!\r\n");
        return;
    }

    // RTE_PK_SM_SetHeartBeatCnt(HeartBeatCnt);
}

static void *DetectHeartBeatLoop(void *paras)
{
    const char *input = (char *)paras;
    if (input == NULL)
    {
        return NULL;
    }
    int port = atoi(input);

    struct sockaddr_in serverAddr, clientAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family      = AF_INET;
    serverAddr.sin_port        = htons(port);       // 2234
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY); // 连接所有的客户端

    int clientfd = 0;
    int serverfd = socket(AF_INET, SOCK_STREAM, 0);
    auto CloseFd = [&clientfd, &serverfd](const char *info) {
        printf("%s errno=%d %s\n", info, errno, strerror(errno));
        if (clientfd >= 0)
        {
            close(clientfd);
        }
        if (serverfd >= 0)
        {
            close(serverfd);
        }
    };

    if (serverfd < 0)
    {
        CloseFd("Create Server Socket failed!");
        return NULL;
    }

    int flag = 1;
    if (setsockopt(serverfd, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag)) < 0)
    {
        CloseFd("Error , set socket reuse addr failued\n");
        return NULL;
    }

    int len = sizeof(struct sockaddr);
    if (bind(serverfd, (struct sockaddr *)&serverAddr, len) < 0)
    {
        CloseFd("bind e rror\n");
        return NULL;
    }

    if (listen(serverfd, 1) < 0)
    {
        CloseFd("bind error\n");
        return NULL;
    }

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(serverfd, &rfds);
    int maxfd = serverfd;

    char buf[1024];
    printf("wait connect ...\r\n");

    while (1)
    {
        fd_set curRdfs = rfds;
        int result     = select(maxfd + 1, &curRdfs, NULL, NULL, NULL);
        if (result < 0)
        {
            CloseFd("select error\n");
            return NULL;
        }

        for (int fd = 0; fd <= maxfd; fd++)
        {
            if (FD_ISSET(fd, &curRdfs))
            {
                if (serverfd == fd)
                {
                    len = sizeof(struct sockaddr);
                    if ((clientfd = accept(serverfd, (struct sockaddr *)&clientAddr,
                                           (socklen_t *)&len)) < 0)
                    {
                        CloseFd("accept error\n");
                        return NULL;
                    }

                    // printf("connected socket:%d\n", clientfd);
                    maxfd = maxfd > clientfd ? maxfd : clientfd;
                    FD_SET(clientfd, &rfds);
                }
                else
                {
                    int readLen = recv(fd, buf, sizeof(buf), 0);
                    if (readLen == 0)
                    {
                        close(fd);
                        // printf("broken socket:%d\n", fd);
                        FD_CLR(fd, &rfds);
                        continue;
                    }

                    if (readLen < 0 && EINTR != errno)
                    {
                        // CloseFd("recv err or\n");
                        // return NULL;
                        continue;
                    }

                    // printf("Heart receive data len: %d data: %s\n", readLen, buf);
                    buf[readLen] = 0;
                    RtHeartBeatChk(buf);
                }
            }
        }
    }

    CloseFd("Server eixt");
    return NULL;
}

void Eth_InitHeartBeat() { ASyncRun(DetectHeartBeatLoop, (void *)"2234", "apa_eth_hb"); }
