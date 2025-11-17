#ifndef RAINCOM_COM_TIMER_MANAGER_H_
#define RAINCOM_COM_TIMER_MANAGER_H_

#include "raincom_com/macros.hpp"
#include <list>
#include <memory>
#include <mutex>
#include <thread>

namespace raincom
{
namespace com
{

class Timer;
class ThreadPool;

class TimerManager
{
    DEF_SINGLETON(TimerManager)
  public:
    TimerManager();
    ~TimerManager();
    void add_timer(std::shared_ptr<Timer> timer);
    void remove_timer(std::shared_ptr<Timer> timer);
    void shutdown();

  private:
    bool terminated_;
    std::list<std::shared_ptr<Timer>> timers_;
    std::mutex locker_;
    std::thread manager_thread_;
    bool manager_thread_started_;

    void run_thread();
}; // class TimerManager

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_TIMER_MANAGER_H_
