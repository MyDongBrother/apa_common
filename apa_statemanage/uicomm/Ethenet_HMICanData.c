/*
 * @Author: error: error: git config user.name & please set dead value or install git &&
 * error: git config user.email & please set dead value or install git & please set dead
 * value or install git
 * @Date: 2024-05-10 21:59:48
 * @LastEditors: error: error: git config user.name & please set dead value or install git
 * && error: git config user.email & please set dead value or install git & please set
 * dead value or install git
 * @LastEditTime: 2024-07-11 20:18:27
 * @FilePath: \new_apa\apa_statemanage\uicomm\Ethenet_HMICanData.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "Ethenet_HMICanData.h"
#include "Rte.h"
#include "MathFunc.h"
//#include "PK_StateManage.h"

uint8_t calculateChecksum(const uint8_t *data)
{
    return (data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6]) ^ 0XFF;
}

void BuildSlotPosition(slotPoint *data, const float *point, const float curPos[4],
                       int slot_index, int stopflags[4], int id)
{
    float rPoint[3];

    if (data == NULL || point == NULL)
    {
        return;
    }
    RevConvert(curPos, point, rPoint);

    data->stopbarv  = (stopflags[0] != 0);
    data->stopbarp  = (stopflags[1] != 0);
    data->lockflagv = (stopflags[2] != 0);
    data->lockflagp = (stopflags[3] != 0);

    data->point_x = (int16_t)(rPoint[0] * 100);
    data->point_y = (int16_t)(rPoint[1] * 100);

    data->canId = id;

    data->slot_index = (slot_index == -1) ? 0xFFFF : slot_index;
    // printf("point_x: %d, point_y: %d \n",data->point_x, data->point_y);

    data->checksum = calculateChecksum((const uint8_t *)data);
}

void BuildVehPosition(vehPosition *data, float x, float y, float angle, int run_rate)
{
    if (data == NULL)
    {
        return;
    }
    data->curpos_x         = (int16_t)(x * 100);
    data->curpos_y         = (int16_t)(y * 100);
    data->angle            = (int16_t)(angle * 100);
    data->parking_progress = run_rate;
    data->checksum         = calculateChecksum((const uint8_t *)data);
}

void BuildRaderData(raderData *data, int can_id, int a, int b, int c)
{
    if (data == NULL)
    {
        return;
    }
    data->canId   = (uint8_t)can_id;
    data->rader_a = a;
    data->rader_b = b;
    data->rader_c = c;

    uint8_t *buf = (uint8_t *)data;
    data->checksum =
        (buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6]) ^ 0xFF;
}