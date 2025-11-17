#include <stdio.h>
#include "DistCtrl.h"

#define QUEUE_SIZE 10 // 队列大小，存储最近的 5 条记录

typedef struct
{
    float trends[QUEUE_SIZE]; // 存储速度记录的数组
    int front;                // 队列头部索引
    int rear;                 // 队列尾部索引
    int count;                // 队列中元素的数量
} trendQueue;

static trendQueue speed_queue; // 静态队列变量，生命周期贯穿整个程序
static trendQueue dist_queue;  // 静态队列变量，生命周期贯穿整个程序

// 初始化队列
void InitializeTrendQueue()
{
    speed_queue.front = 0;
    speed_queue.rear  = 0;
    speed_queue.count = 0;

    dist_queue.front = 0;
    dist_queue.rear  = 0;
    dist_queue.count = 0;
    // printf("速度队列已初始化。\n");
}

// 队列是否已满
int IsFull(trendQueue *queue) { return queue->count == QUEUE_SIZE; }

// 入队操作，插入新的速度记录
void Enqueue(trendQueue *queue, float trend)
{
    if (IsFull(queue))
    {
        // 如果队列已满，移动头部（丢弃最旧的速度记录）
        queue->front = (queue->front + 1) % QUEUE_SIZE;
    }
    else
    {
        queue->count++;
    }
    queue->trends[queue->rear] = trend;
    queue->rear                = (queue->rear + 1) % QUEUE_SIZE;
}

// 获取队列中指定索引的速度记录
float GetTrend(trendQueue *queue, int index)
{
    return queue->trends[(queue->front + index) % QUEUE_SIZE];
}

// 判断速度变化趋势并返回字符串
int GettTrend(trendQueue *queue)
{
    int accelerating = 0, decelerating = 0, stable = 0;

    for (int i = 1; i < queue->count; i++)
    {
        float prev_trend = GetTrend(queue, i - 1);
        float curr_trend = GetTrend(queue, i);

        if (curr_trend > prev_trend)
        {
            accelerating++;
        }
        else if (curr_trend < prev_trend)
        {
            decelerating++;
        }
        else
        {
            stable++;
        }
    }

    // 根据加速、减速和稳定的次数返回趋势字符串
    if (accelerating > 0 && decelerating == 0)
    {
        return 1;
    }
    else if (decelerating > 0 && accelerating == 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

// 输入速度并返回速度变化趋势
int AnalyzeSpeedTrend(float current_data)
{
    Enqueue(&speed_queue, current_data); // 将新速度加入队列

    if (speed_queue.count > 1)
    {
        return GettTrend(&speed_queue); // 返回速度变化趋势
    }
    else
    {
        return 0; // 如果队列中记录不足以比较，返回提示
    }
}

// 输入距离并返回距离变化趋势
int AnalyzeDistTrend(float current_data)
{
    Enqueue(&dist_queue, current_data); // 将新距离加入队列

    if (dist_queue.count > 1)
    {
        return GettTrend(&dist_queue); // 返回距离变化趋势
    }
    else
    {
        return 0; // 如果队列中记录不足以比较，返回提示
    }
}
