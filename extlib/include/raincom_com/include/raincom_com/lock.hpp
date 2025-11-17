#ifndef RAINCOM_COMM_LOCK_H_
#define RAINCOM_COMM_LOCK_H_

#include <atomic>

namespace raincom
{
namespace com
{

class SpinLock
{
  public:
    SpinLock() {}
    SpinLock(const SpinLock &)            = delete;
    SpinLock &operator=(const SpinLock &) = delete;

    void lock()
    {
        while (flag.test_and_set())
            ;
    }

    void unlock() { flag.clear(); }

  private:
    std::atomic_flag flag;
}; // class SpinLock

} // namespace com
} // namespace raincom

#endif // RAINCOM_COMM_LOCK_H_
