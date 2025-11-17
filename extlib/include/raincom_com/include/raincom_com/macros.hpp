#ifndef RAINCOM_COM_MACROS_H_
#define RAINCOM_COM_MACROS_H_

#include <mutex>

#define DEF_SINGLETON(classname)                                                         \
  public:                                                                                \
    static inline classname *instance()                                                  \
    {                                                                                    \
        static classname *instance_ = nullptr;                                           \
        static std::once_flag flag;                                                      \
        if (!instance_)                                                                  \
        {                                                                                \
            std::call_once(flag, [&]() { instance_ = new (std::nothrow) classname(); }); \
        }                                                                                \
        return instance_;                                                                \
    }                                                                                    \
                                                                                         \
  private:                                                                               \
    classname(const classname &)             = delete;                                   \
    classname &operator=(const classname &)  = delete;                                   \
    classname(const classname &&)            = delete;                                   \
    classname &operator=(const classname &&) = delete;

#endif // RAINCOM_COM_MACROS_H_
