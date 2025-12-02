/**
 * @file Log_Buff.h
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-11-24
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-11-24 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#ifndef LOG_BUFF_H_
#define LOG_BUFF_H_

#include <cstring>
#include <cerrno>
#include <cstdio>
#include <cstdlib>

typedef struct
{
    FILE *logFd;
    uint8_t *data;
    int buffLen;
    int head;
    int validLen;
    int counter;
    pthread_mutex_t mutex;
} Buff_Inf;

void *Buff_Init(uint8_t *buff, const int buffLen, FILE *logFd);

void Buff_UnInit(void *bufFd);

int Buff_Put(void *bufFd, const char *data, const int dataLen);

int Buff_Get(void *bufFd, uint8_t *data, const int dataLen);

int Buff_Len(void *bufFd);

#endif
