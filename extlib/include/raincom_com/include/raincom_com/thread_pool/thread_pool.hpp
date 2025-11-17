#ifndef RAINCOM_COM_THREAD_POOL_H_
#define RAINCOM_COM_THREAD_POOL_H_

#include "raincom_com/macros.hpp"
#include "raincom_com/thread_pool/safe_task_queue_manager.hpp"
#include <thread>
#include <vector>

namespace raincom
{
namespace com
{

class Task;

class ThreadPool
{
    DEF_SINGLETON(ThreadPool)
  public:
    void stop();
    void start();
    void wait();
    void set_num_threads(int num_threads);
    SafeTaskQueueManager *task_queue_manager();

  private:
    ThreadPool();
    ~ThreadPool();
    void loop();

    bool started_;
    int num_threads_;
    std::vector<std::thread> threads_;
    SafeTaskQueueManager task_queue_manager_;
}; // class ThreadPool

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_THREAD_POOL_H_
