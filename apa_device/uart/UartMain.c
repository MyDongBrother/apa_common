/**
 * @file UartMain.c
 * @brief UART接收并解析HDLC协议封装的CAN数据帧
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-07-31
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-07-31 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <poll.h>
#include <pthread.h>
#include <string.h>
#include <netinet/in.h>
#include "Record_Log.h"
#include "UartHdlc.h"
#include "CanIf.h"
#include "Rte_Func.h"
#include "Ram.h"

#define UART_PORT_DEVICE        "/dev/ttyAT6"
#define FRAME_MAX_SIZE          4096 // 一帧数据最大长度
#define HDLC_FLAG               0x7E // hdlc帧头
#define HDLC_CONTROL_H          0xFF // hdlc控制字
#define HDLC_CONTROL_L          0x10 // hdlc控制字
#define PLAYLOAD_HEAD_LEN       6    // FrameLen占位2字节
#define PLAYLOAD_HEAD_FRAME_LEN 2    // FrameLen占位2字节
#define PLAYLOAD_HEAD_FRAME_CT  2    // FrameCount占位2字节
#define PLAYLOAD_HEAD_FRAME_TS  2    // TimeStamp占位4字节

typedef struct
{
    uint16_t canLen;
    uint16_t canId;
    uint8_t data[candata_bytes];
} CanFrame_t;

static int uart_dev_fd          = 0;
static yahdlc_control_t control = {YAHDLC_FRAME_DATA, 0};
static uint8_t rx_buffer[FRAME_MAX_SIZE];
static uint8_t tx_buffer[FRAME_MAX_SIZE];
static char receive_data[FRAME_MAX_SIZE];
static uint8_t test_buffer_temp[FRAME_MAX_SIZE] = {
    0x7E, 0xFF, 0x10, 0x00, 0x8E, 0x6D, 0x12, 0x7D, 0x5E, 0x0D, 0x00, 0x08, 0x03,
    0x42, 0x00, 0x05, 0x4F, 0x32, 0xC6, 0xFF, 0x67, 0x4D, 0x00, 0x05, 0x01, 0x1F,
    0x30, 0xFF, 0x72, 0xFF, 0x9F, 0x00, 0x08, 0x01, 0x21, 0x00, 0x00, 0x00, 0x00,
    0xE0, 0xFF, 0x8C, 0x94, 0x00, 0x08, 0x01, 0x23, 0x92, 0xC0, 0x5D, 0xC0, 0x5D,
    0xC8, 0x06, 0x65, 0x00, 0x08, 0x01, 0x73, 0x00, 0x3F, 0xFE, 0xFF, 0xFF, 0xFF,
    0xFC, 0xC9, 0x00, 0x08, 0x01, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0,
    0x3F, 0x00, 0x08, 0x02, 0x20, 0xFF, 0xA8, 0x04, 0x8C, 0x00, 0x00, 0x8C, 0x3C,
    0x00, 0x08, 0x02, 0x22, 0xDB, 0x03, 0x04, 0x80, 0xC0, 0xFF, 0xFC, 0xE2, 0x00,
    0x08, 0x02, 0x23, 0x1E, 0xB3, 0x31, 0x1D, 0x03, 0x32, 0xFC, 0xAF, 0x00, 0x08,
    0x01, 0x35, 0xFF, 0xFF, 0xFF, 0xE3, 0x00, 0xFE, 0x57, 0xCA, 0x00, 0x08, 0x04,
    0x22, 0x66, 0x0E, 0x85, 0x3D, 0xDD, 0xE2, 0xCC, 0x3E, 0x28, 0x85, 0x7E};

/// RAM缓存（用于运行时uart数据的临时存储）
const uint32_t uart_cache_len                = FRAME_MAX_SIZE;
static uint8_t uartCacheBuff[uart_cache_len] = {0};
static RAM_INF uartRamInf = {uartCacheBuff, sizeof(uint8_t), sizeof(uartCacheBuff), 0, 0};
static inline uint32_t IncreaseUartReadIndex(uint32_t index, uint32_t max_len)
{
    index++;
    if (index >= max_len)
    {
        index = 0;
    }
    return index;
}

/**
 * @brief 串口接收线程入口
 */
static void *ProcessUartRx(void *paras)
{
    Ram_Clear(&uartRamInf);
    log_info("UART RX thread started");
    while (1)
    {
        ssize_t ret_read = read(uart_dev_fd, rx_buffer, FRAME_MAX_SIZE);
        if (ret_read > 0)
        {
            log_info("Received %zd bytes from UART", ret_read);
            // 将读取的数据保存到环形缓冲区
            for (ssize_t i = 0; i < ret_read; i++)
            {
                Ram_Put(&uartRamInf, -1, &rx_buffer[i], 1);
            }
        }
        // 没有读到数据
        else
        {
            memset(rx_buffer, 0, sizeof(rx_buffer));
            // 从UartReadIndex开始寻找帧头
            uint32_t IndexTemp  = 0;
            int32_t IndexHead   = -1;
            uint32_t RamLenTemp = Ram_Len(&uartRamInf);
            uint32_t SearchNum  = 0;
            uint16_t frame_len  = 0;
            for (SearchNum = 0; SearchNum < RamLenTemp; SearchNum++)
            {
                IndexHead = IndexTemp;

                uint32_t readCount = Ram_Get(&uartRamInf, IndexTemp, rx_buffer, 5);
                if (readCount < 5)
                {
                    IndexHead = -1;
                    break;
                }
                frame_len = ((uint16_t)rx_buffer[3] << 8) | rx_buffer[4];
                if (rx_buffer[0] == HDLC_FLAG && (rx_buffer[1] == HDLC_CONTROL_H) &&
                    (rx_buffer[2] == HDLC_CONTROL_L) && (frame_len != 0))
                {
                    log_info("Found HDLC frame header at index %u", IndexHead);
                    break;
                }
                // 不是帧头
                IndexTemp = IncreaseUartReadIndex(IndexTemp, uart_cache_len);
                IndexHead = -1;
            }
            // 找到帧头，验证包是否完整
            if (IndexHead != -1)
            {
                uint32_t readCount =
                    Ram_Get(&uartRamInf, IndexHead, rx_buffer, frame_len);
                if (readCount < frame_len)
                {
                    // 缓存中没有完整的数据包
                    // 更新指针到找到帧头的位置,等待完整的数据补齐
                    log_info("Incomplete frame: needed %u, available %u", frame_len,
                             readCount);
                    Ram_Remove(&uartRamInf, SearchNum);
                }
                else
                {
                    // hdlc解析数据
                    unsigned int decoded_len = 0;
                    memset(receive_data, 0, sizeof(receive_data));
                    // 按照协议，frame_len字段使用0x00进行hdlc校验
                    rx_buffer[3] = 0x00;
                    rx_buffer[4] = 0x00;
                    int ret =
                        yahdlc_get_data(&control, (const char *)rx_buffer, frame_len,
                                        (char *)receive_data, &decoded_len);
                    if (ret < 0 || decoded_len == 0 || frame_len == 0)
                    {
                        // 这里是丢帧或错帧了，丢弃这一帧
                        log_warn(
                            "HDLC decode failed, ret=%d, decoded_len=%u, frame_len =%u, "
                            "data = ",
                            ret, decoded_len, frame_len);
                        for (int i = 0; i < frame_len; i++)
                        {
                            printf("%02X ", rx_buffer[i]);
                        }
                        printf("\n");
                        Ram_Remove(&uartRamInf, SearchNum + 1);
                        continue;
                    }
                    else
                    {
                        // 更新指针到下一包的位置
                        log_info("Decoded %u bytes successfully", decoded_len);

                        if (SearchNum != 0)
                        {
                            // 缓存区处理的足够快时，不应该存在这种情况
                            // 如果出现说明有数据被覆盖了
                            log_warn(
                                "Ram_Remove SearchNum + frame_len %zd bytes from read",
                                SearchNum + frame_len);
                        }
                        Ram_Remove(&uartRamInf, SearchNum + frame_len);

                        // 5. 解析数据块
                        int offset_len              = PLAYLOAD_HEAD_FRAME_LEN;
                        int frame_count             = 0;
                        static int last_frame_count = 0;
                        int time_stamp              = 0;
                        static int last_time_stamp  = 0;
                        CanFrame_t frame;

                        // 5.0.1 提取 count
                        memcpy(&frame_count, &receive_data[offset_len],
                               PLAYLOAD_HEAD_FRAME_CT);
                        frame_count = ntohs(frame_count);
                        offset_len += PLAYLOAD_HEAD_FRAME_CT;
                        if (frame_count - last_frame_count != 1)
                        {
                            log_warn("FrameCount jump: now=%u, last=%u, diff=%u",
                                     frame_count, last_frame_count,
                                     frame_count - last_frame_count);
                        }
                        last_frame_count = frame_count;

                        // 5.0.2 提取 time_stamp
                        memcpy(&time_stamp, &receive_data[offset_len],
                               PLAYLOAD_HEAD_FRAME_CT);
                        time_stamp = ntohs(time_stamp);
                        offset_len += PLAYLOAD_HEAD_FRAME_CT;
                        if (time_stamp - last_time_stamp > 15)
                        {
                            log_info("TimeStamp jump: now=%u, last=%u, diff=%u",
                                     time_stamp, last_time_stamp,
                                     time_stamp - last_time_stamp);
                        }
                        last_time_stamp = time_stamp;

                        while (offset_len < decoded_len)
                        {
                            memset(&frame, 0, sizeof(CanFrame_t));

                            // 5.1 提取 canLen
                            memcpy(&frame.canLen, &receive_data[offset_len], 2);
                            frame.canLen = ntohs(frame.canLen);
                            offset_len += 2;

                            // 5.2 提取 canId
                            memcpy(&frame.canId, &receive_data[offset_len], 2);
                            frame.canId = ntohs(frame.canId);
                            offset_len += 2;

                            // 5.3 提取 data
                            memcpy(frame.data, &receive_data[offset_len], frame.canLen);
                            offset_len += (frame.canLen);

                            if (frame.canId != 0)
                            {
                                Can_PushData(frame.canId, frame.data);
                            }
                        }
                    }
                }
            }
            else // 环形队列中没有帧头
            {
                log_info("No HDLC frame header found in %u bytes of buffer", RamLenTemp);
            }
            usleep(200);
        }
    }
}

/**
 * @brief 串口发送
 *
 * @param data
 * @param data_length
 */
void Uart_SendData(uint8_t data[FRAME_MAX_SIZE], const uint32_t data_length)
{
    char encoded_buf[FRAME_MAX_SIZE];
    memset(encoded_buf, 0, sizeof(encoded_buf));
    unsigned int encoded_len = 0;
    yahdlc_control_t control = {YAHDLC_FRAME_DATA, 0};

    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint32_t timestamp_ms = (uint32_t)((tv.tv_sec * 1000) + (tv.tv_usec / 1000));

    data[0]      = 0; // FrameLen 预留，高字节
    data[1]      = 0; // FrameLen 预留，低字节
    data[2]      = (timestamp_ms >> 24) & 0xFF;
    data[3]      = (timestamp_ms >> 16) & 0xFF;
    data[4]      = (timestamp_ms >> 8) & 0xFF;
    data[5]      = (timestamp_ms) & 0xFF;
    int ret_code = yahdlc_frame_data(&control, (const char *)data, data_length,
                                     encoded_buf, &encoded_len);
    if (ret_code != 0)
    {
        fprintf(stderr, "HDLC encode failed: %d\n", ret_code);
        return;
    }
    struct pollfd pfd = {uart_dev_fd, POLLOUT, 0};
    int poll_ret      = poll(&pfd, 1, 100);
    if (poll_ret > 0 && (pfd.revents & POLLOUT))
    {
        ssize_t ret = write(uart_dev_fd, encoded_buf, encoded_len);
        if (ret < 0)
        {
            perror("write failed");
        }
    }
    else
    {
        fprintf(stderr, "uart not ready to write\n");
    }
}

/**
 * @brief
 *
 * @param canId
 * @param data
 * @param number
 */
void Uart_PushCanData(const uint16_t *canId, const uint8_t data[][8],
                      const uint32_t number)
{
    int data_blk_pos     = PLAYLOAD_HEAD_LEN;
    CanFrame_t send_data = {};

    for (uint32_t i = 0; i < number; i++)
    {
        send_data.canLen = htons(8);
        send_data.canId  = htons(canId[i]);
        memcpy(send_data.data, data[i], candata_bytes);

        memcpy(&tx_buffer[data_blk_pos], &send_data,
               (2 + 2 + candata_bytes)); // 12 字节块
        data_blk_pos += (2 + 2 + candata_bytes);

        if (data_blk_pos >= FRAME_MAX_SIZE / 2) // 考虑字节转义
        {
            Uart_SendData(tx_buffer, data_blk_pos);
            data_blk_pos = PLAYLOAD_HEAD_LEN;
        }
    }

    // 发送剩余的
    if (data_blk_pos > PLAYLOAD_HEAD_LEN)
    {
        Uart_SendData(tx_buffer, data_blk_pos);
    }
}

/**
 * @brief
 *
 */
void Uart_InitRxThread()
{
    // 打开串口设备
    const char *device = (const char *)UART_PORT_DEVICE;
    uart_dev_fd        = open(device, O_RDWR | O_NOCTTY);
    if (uart_dev_fd < 0)
    {
        perror("open uart failed");
        return;
    }

    // 设置串口参数
    struct termios options;
    tcgetattr(uart_dev_fd, &options);
    cfsetispeed(&options, B921600);
    cfsetospeed(&options, B921600);

    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;                // 关闭硬件流控
    options.c_cflag |= (CLOCAL | CREAD);        // 使能接收，忽略控制线
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // 关闭软件流控
    options.c_iflag &=
        ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // 输入处理关闭
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);               // 非规范模式
    options.c_oflag &= ~OPOST;                                        // 关闭输出处理

    if (tcsetattr(uart_dev_fd, TCSANOW, &options) != 0)
    {
        perror("tcsetattr failed");
        return;
    }

    // 设置非阻塞
    fcntl(uart_dev_fd, F_SETFL, O_NONBLOCK);
    yahdlc_get_data_reset();
    ASyncRun(ProcessUartRx, (void *)UART_PORT_DEVICE, "apa_uart_R2mcu");
}
