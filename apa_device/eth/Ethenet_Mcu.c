/**
 * @file Ethenet_Mcu.c
 * @brief 实现与MCU之间的以太网UDP通信和CAN数据交互，
 *        包括SocketCAN初始化、数据接收处理等逻辑。
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-07-02
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-07-02 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */
#include "EthIf.h"
#include "MathFunc.h"
#include "Record_Log.h"
#include "McuMsgData.h"
#include "CanIf.h"
#include "Rte.h"
#include <sys/time.h>
#include "Debug_Cmd.h"
#include "PK_Calibration.h"
#include "PK_StateManage.h"
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

/// MCU通信的socket句柄，初始化为无效值
static int s_socketId = -1;
/// MCU的网络地址信息
static sockaddr_in s_mcu_addr;
/// 网络发送接收缓冲区结构体，存放发送和接收的数据
static netSenddata recvMsg, sendMsg;
/// 网络准备接收数据结构体，存放准备收发的数据
static netReadydata recvReadyData, sendReadyData;

/**
 * @brief 通过以太网发送数据到MCU
 * @param data    发送数据指针
 * @param dataLen 发送数据长度（字节）
 * @return 发送成功返回发送字节数，失败返回-1
 */
static int EthSendData(netSenddata *data, const uint32_t dataLen)
{
    int result = 0;
    if (s_socketId == -1)
    {
        return -1;
    }
    result = sendto(s_socketId, data, dataLen, 0, (struct sockaddr *)&s_mcu_addr,
                    sizeof(s_mcu_addr));
    if (result < 0)
    {
        printf("send to mcu: len:%d result:%d error:%d %s\n", dataLen, result, errno,
               strerror(errno));
        printf("EthSendData IP : %s  Port: %d\n", inet_ntoa(s_mcu_addr.sin_addr),
               ntohs(s_mcu_addr.sin_port));
    }
    return result;
}

/**
 * @brief 发送多条CAN数据到MCU
 * @param canId  CAN ID数组指针
 * @param data   CAN数据二维数组指针，每条8字节
 * @param number 发送数据条数
 *
 * @note 每次最多打包发送75条CAN数据，防止发送缓冲区溢出
 */
void Eth_PushCanData(const uint16_t *canId, const uint8_t data[][8],
                     const uint32_t number)
{
    sendMsg.cmdid    = htons(0xA000);
    int count        = 0;
    int dataBlkStart = 0;
    for (uint32_t i = 0; i < number; i++)
    {
        sendReadyData.canLen = htons(10);
        sendReadyData.canId  = htons(canId[i]);
        memcpy(&sendReadyData.data, data[i], candata_bytes);

        memcpy(&sendMsg.data[dataBlkStart], &sendReadyData,
               (2 + 2 + candata_bytes)); // 2+2+8=12字节
        dataBlkStart += (2 + 2 + candata_bytes);

        count++;

        if (count > 75) // 限制单次发送最大条数
        {
            uint16_t sendLen = dataBlkStart + CmdDataHeadLen;
            sendMsg.datalen  = htons(sendLen);
            EthSendData(&sendMsg, sendLen);

            count        = 0;
            dataBlkStart = 0;
            // printf("Ethenet_Mcu.c: Eth_PushCanData: number: %d\n", number);
            return;
        }
    }

    if (count > 0)
    {
        uint16_t sendLen = dataBlkStart + CmdDataHeadLen;
        sendMsg.datalen  = htons(sendLen);
        EthSendData(&sendMsg, sendLen);
    }
}

/**
 * @brief 打印接收的字节数据，十六进制格式，每字节以空格分隔
 * @param buf 指向数据缓冲区的指针
 * @param dataLen 数据长度
 */
static void PrintRecvData(const uint8_t *buf, const int dataLen)
{
    for (int i = 0; i < dataLen; i++)
    {
        printf("%02x ", buf[i]);
    }
    printf("\n");
}

/**
 * @brief  处理接收到的 MCU CAN 打包数据，拆解并分发每条 CAN 消息。
 *
 * 数据格式：
 * ┌────────────┬────────────┬────────────────────┐
 * │ cmdid(2B)  │ datalen(2B)│ data[]（多条 CAN 帧） │
 * └────────────┴────────────┴────────────────────┘
 *
 * 每条 CAN 帧格式如下（嵌套在 data[] 中）：
 * ┌────────────┬────────────┬────────────────────┐
 * │ canLen(2B) │ canId(2B)  │ data[canLen - 2]   │
 * └────────────┴────────────┴────────────────────┘
 *
 * @param buf       MCU 发送的数据缓冲区
 * @param dataLen   实际接收到的数据长度
 * @return int      成功返回 0，失败返回 -1
 */
static int RecvDataProc(const uint8_t *buf, const int dataLen)
{
    static int error_count = 0;
    if (buf == NULL || dataLen < 4 || dataLen > MAX_CAN_DATA_SIZE)
    {
        return -1;
    }

    memcpy(&recvMsg.cmdid, &buf[0], 2);
    recvMsg.cmdid = ntohs(recvMsg.cmdid);

    memcpy(&recvMsg.datalen, &buf[2], 2);
    recvMsg.datalen = ntohs(recvMsg.datalen);

    // if(recvMsg.cmdid == 0x1a0 || recvMsg.cmdid == 0xA001) {  return -1;  }

    if (dataLen < 9)
    {
        printf("can data len error: id 0x%x, Len %d, len %d\n", recvMsg.cmdid,
               recvMsg.datalen, dataLen);
    }

    if (recvMsg.datalen != dataLen)
    {
        printf("-------------------------can data len error: id 0x%x, Len %d, len %d\n",
               recvMsg.cmdid, recvMsg.datalen, dataLen);
        // log_err("can data len error: id %s, bit16Len %s, len %s\n",msg.cmdid,
        // msg.datalen, dataLen);
        return -1;
    }
    memcpy(&recvMsg.data, &buf[4], recvMsg.datalen - CmdDataHeadLen);

    int dataBlkStart = 0;
    while (dataBlkStart + CmdDataHeadLen < dataLen)
    {
        if (dataBlkStart + CmdDataHeadLen + 2 > dataLen)
        {
            printf("can data len error1 !");
            return -1;
        }
        memcpy(&recvReadyData.canLen, &recvMsg.data[dataBlkStart], 2);
        recvReadyData.canLen = ntohs(recvReadyData.canLen);
        dataBlkStart += 2;

        if (dataBlkStart + CmdDataHeadLen + recvReadyData.canLen > dataLen ||
            recvReadyData.canLen > 10)
        {
            error_count = error_count + 1;
            printf("can data len error2 ! %d %d %d error count:%d\n",
                   recvReadyData.canLen, dataLen, dataBlkStart, error_count);
            // PrintRecvData(buf, dataLen);
            return -1;
        }

        memcpy(&recvReadyData.canId, &recvMsg.data[dataBlkStart], ReadydataHeadLen);
        recvReadyData.canId = ntohs(recvReadyData.canId);
        dataBlkStart += ReadydataHeadLen;

        if (recvReadyData.canLen - ReadydataHeadLen < 1)
        {
            error_count = error_count + 1;
            printf("can data len error3 ! %d %d %d error count:%d\n",
                   recvReadyData.canLen, dataLen, dataBlkStart, error_count);
            // PrintRecvData(buf, dataLen);
            return -1;
        }

        memcpy(recvReadyData.data, &recvMsg.data[dataBlkStart],
               recvReadyData.canLen - ReadydataHeadLen);
        dataBlkStart += recvReadyData.canLen - ReadydataHeadLen;

        if (recvReadyData.canId != 0)
        {
            Can_PushData(recvReadyData.canId, recvReadyData.data);
        }
    }

    return 0;
}

/**
 * @brief MCU数据接收线程函数，基于UDP协议接收数据并处理
 *
 * 该函数作为线程入口，创建UDP socket，绑定到指定本地IP和端口，
 * 不断接收来自远端的数据包，将接收到的数据传递给数据处理函数进行解析。
 *
 * @param paras 传入端口号字符串指针，例如 "18806"
 * @return void* 线程返回值，正常情况下不会退出，异常时返回NULL
 */
static void *McuDataLoop(void *paras)
{
    uint8_t buf[MAX_CAN_DATA_SIZE];

    const char *input = (char *)paras;
    if (input == NULL)
    {
        return NULL;
    }
    int port = atoi(input);

    s_socketId = socket(PF_INET, SOCK_DGRAM, 0);

    struct sockaddr_in remoteaddr, localaddr;

    localaddr.sin_family      = AF_INET;
    localaddr.sin_port        = htons(port); // 18806
    localaddr.sin_addr.s_addr = inet_addr(local_ip_address);

    printf("%d  Bind IP: %s\n", __LINE__, local_ip_address);

    int flag = 1;
    if (setsockopt(s_socketId, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag)) < 0)
    {
        log_err("setsockopt errno=%d %s\n", errno, strerror(errno));
        close(s_socketId);
        s_socketId = -1;
        return NULL;
    }

    if (bind(s_socketId, (struct sockaddr *)&localaddr, sizeof(localaddr)) < 0)
    {
        log_err("bind errno=%d %s\n", errno, strerror(errno));
        close(s_socketId);
        s_socketId = -1;
        return NULL;
    }
    printf("Local IP : %s  Port: %d\n", inet_ntoa(localaddr.sin_addr),
           ntohs(localaddr.sin_port));
    usleep(10000);
    while (1)
    {
        memset(buf, 0, sizeof(buf));
        int len    = sizeof(remoteaddr);
        int result = recvfrom(s_socketId, buf, sizeof(buf), 0,
                              (struct sockaddr *)&remoteaddr, (socklen_t *)&len);
        if (result < 0)
        {
            printf("%s %d recvfrom errno!!", __FILE__, __LINE__);
            log_err("recvfrom errno=%d %s\n", errno, strerror(errno));
        }
        else if (result > 0 && result <= MAX_CAN_DATA_SIZE)
        {
            RecvDataProc(buf, result);
        }
    }

    close(s_socketId);
    s_socketId = -1;
    return NULL;
}

/**
 * @brief CAN数据接收线程函数，从CAN设备文件描述符循环读取CAN帧并处理
 *
 * @param paras 指向整型数组的指针，数组第一个元素为CAN设备文件描述符
 * @return void* 线程返回值，通常不会返回
 */
static void *CanRecvDataLoop(void *paras)
{
    int *input = (int *)paras;
    struct can_frame frame;
    while (1)
    {
        int nbytes = read(input[0], &frame, sizeof(frame));
        if (nbytes != sizeof(frame))
        {
            log_err("dir0: read failed! result %d, errno = %d, errinfo = %s\n", nbytes,
                    errno, strerror(errno));
        }

        uint16_t canid = frame.can_id & 0x7ff;
        Can_PushData(canid, frame.data);
    }

    close(input[0]);
    return NULL;
}
/**
 * @brief 初始化 CAN FD 接口，并返回 socket 描述符
 *
 * @param canname CAN 接口名称，例如 "can0"
 * @param canType CAN 配置类型（不同类型对应不同速率设置）
 * @return int 成功返回 socket 描述符，失败返回 -1
 */
static int InitCanFd(const char *canname, int canType)
{
    char canconfig[256];
    char candown[64];

    //  ?adas_mcu?????
    if (0) // boardType == A1000)
    {
        sprintf(candown, "ip link set %s down", canname);
        system(candown);
        usleep(200 * 1000);

        if (canType == 1)
        {
            sprintf(canconfig,
                    "ip link set %s up type can bitrate 500000 sample-point 0.80 "
                    "dbitrate 2000000 dsample-point 0.80 fd on restart-ms 100",
                    canname);
        }
        else
        {
            sprintf(canconfig,
                    "ip link set %s up type can bitrate 500000 sample-point 0.750 "
                    "dbitrate 500000 dsample-point 0.750 fd on restart-ms 100",
                    canname);
        }

        // sprintf(canconfig, "ip link set %s up type can bitrate 500000 sample-point
        // 0.750 fd off", canname);
        system(canconfig);
        usleep(200 * 1000);
    }
    /* init socket */
    int canfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (canfd < 0)
    {
        printf("create socket can id failed!\n");
        return -1;
    }

    printf("create can id success!\n");

    struct ifreq canifrs;
    strcpy(canifrs.ifr_name, canname);
    canifrs.ifr_ifindex = if_nametoindex(canifrs.ifr_name);
    if (!canifrs.ifr_ifindex)
    {
        close(canfd);
        log_err("if_nametoindex can failed!\n");
        return -1;
    }

    struct sockaddr_can canaddrs;
    memset(&canaddrs, 0, sizeof(sockaddr_can));
    canaddrs.can_family  = AF_CAN;
    canaddrs.can_ifindex = canifrs.ifr_ifindex;
    if (ioctl(canfd, SIOCGIFINDEX, &canifrs) < 0)
    {
        close(canfd);
        log_err("ioctl SIOCGIFINDEX can failed!\n");
        return -1;
    }

    usleep(200 * 1000);
    printf("ioctl can id success!\n");

    int result = bind(canfd, (struct sockaddr *)&canaddrs, sizeof(struct sockaddr_can));
    if (result < 0)
    {
        close(canfd);
        log_err("bind: result %d, errno = %d, errinfo = %s", result, errno,
                strerror(errno));
        return -1;
    }

    printf("bind can id success!\n");
    return canfd;
}

/**
 * @brief 初始化 MCU 以太网通信和 CAN 接收线程（如果配置了）
 */
void Eth_InitUlatroSlot()
{
#ifndef SIMULATE
    memset(&s_mcu_addr, 0, sizeof(s_mcu_addr));
    s_mcu_addr.sin_family      = AF_INET;
    s_mcu_addr.sin_addr.s_addr = inet_addr(local_ip_address);
    s_mcu_addr.sin_port        = htons(18807);

    char config[64];
    if (PK_Calibration_GetDebugValue("cfg_monitor_ipaddress", config) > 0)
    {
        char *pgap                 = strstr(config, ":");
        pgap[0]                    = 0;
        int port                   = atoi(&pgap[1]);
        s_mcu_addr.sin_addr.s_addr = inet_addr(config);
        s_mcu_addr.sin_port        = htons(port);
    }
    printf("MCP IP : %s  Port: %d\n", inet_ntoa(s_mcu_addr.sin_addr),
           ntohs(s_mcu_addr.sin_port));

    ASyncRun(McuDataLoop, (void *)"18806", "apa_eth_R2mcu");

#endif
}
