#ifndef RAINCOM_COM_TASK_QUEUE_MANAGER_H_
#define RAINCOM_COM_TASK_QUEUE_MANAGER_H_

#include "raincom_com/algorithm/circular_queue.hpp"
#include "raincom_com/log.hpp"
#include <condition_variable>
#include <memory>
#include <mutex>
#include <vector>

namespace raincom
{
namespace com
{

class Task;

class SafeTaskQueueManager
{
  public:
    using TaskPtr = std::shared_ptr<Task>;

    SafeTaskQueueManager();
    ~SafeTaskQueueManager();
    int create_queue(int queue_size);
    void put_task(int queue_id, TaskPtr task);
    TaskPtr wait_next_task();
    TaskPtr next_task();
    void terminate();
    bool terminated();

  private:
    std::vector<std::unique_ptr<algorithm::CircularQueue<TaskPtr>>> task_queue_list_;
    int num_queue_;
    int current_idx_;
    bool should_terminate_;
    std::mutex locker_;
    std::condition_variable mutex_condition_;
}; // class TaskQueueManager

class Task
{
  public:
    virtual ~Task() = default;
    virtual void init() {}
    virtual void run() { RERROR << "An unimplemented task has been called"; }
    void operator()() { run(); }
}; // class Task

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_TASK_QUEUE_MANAGER_H_
