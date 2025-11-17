#ifndef RAINCOM_COM_TIMER_H_
#define RAINCOM_COM_TIMER_H_

#include "raincom_com/duration.hpp"
#include "raincom_com/thread_pool/safe_task_queue_manager.hpp"
#include "raincom_com/time.hpp"
#include <functional>
#include <pthread.h>

namespace raincom
{
namespace com
{

class Timer
{
  public:
    using CallbackT = std::function<void(const Time &)>;

    Timer(const Duration &&period, const CallbackT &callback, int queue_size,
          SafeTaskQueueManager *task_queue_manager, bool pure = false);
    ~Timer() = default;

    Duration to_timeout() const;
    void run_callback();
    bool set_realtime(int priority);
    static int max_realtime_priority();
    static int min_realtime_priority();

  private:
    SafeTaskQueueManager *task_queue_manager_;
    Duration period_;
    Time timeout_;
    pthread_t running_thread_;
    CallbackT callback_;
    const int queue_id_;
    const bool pure_;

    void reset_timeout();
    void run();
    static void *internal_run(void *this_)
    {
        reinterpret_cast<Timer *>(this_)->run();
        return nullptr;
    }
}; // class Timer

class TimerTask : public Task
{
  public:
    TimerTask(const Timer::CallbackT callback);
    ~TimerTask();
    virtual void init() override;
    virtual void run() override;

  private:
    Timer::CallbackT callback_;
    Time queue_time_;
}; // class TimerTask

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_TIMER_H_
