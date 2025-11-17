#ifndef RAINCOM_COM_CHANNEL_MANAGER_H_
#define RAINCOM_COM_CHANNEL_MANAGER_H_

#include "raincom_com/thread_pool/safe_task_queue_manager.hpp"
#include "raincom_com/log.hpp"
#include "raincom_com/macros.hpp"
#include <algorithm>
#include <unordered_map>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace raincom
{
namespace com
{
namespace shared_channel
{

struct BaseChannel_;
template <typename T>
struct Channel;

template <typename T>
class CallBackTask : public Task
{
    using CallbackData = const std::shared_ptr<const T>;

  public:
    CallBackTask(std::function<void(CallbackData)> func, CallbackData param)
        : func_(func), param_(param)
    {
    }
    virtual inline void init() override {}
    virtual inline void run() override { func_(param_); }

  private:
    std::function<void(CallbackData)> func_;
    CallbackData param_;
}; // class CallBackTask

class ChannelManager
{
    DEF_SINGLETON(ChannelManager)
  public:
    /**
     * @brief
     *
     * @tparam T
     * @param topic
     * @param data
     */
    template <typename T>
    void push(const std::string &topic, const T &data);

    /**
     * @brief
     *
     * @tparam T
     * @param topic
     * @param data
     */
    template <typename T>
    void push(const std::string &topic, std::shared_ptr<const T> data);

    /**
     * @brief
     *
     * @tparam T
     * @param topic
     * @param callback
     */
    template <typename T>
    void subscribe(const std::string &topic, int queue_size,
                   std::function<void(const std::shared_ptr<const T>)> &callback);

    /**
     * @brief Create a channel object
     *
     * @tparam T
     * @param topic
     * @param queue_size
     * @return Channel<T>*
     */
    template <typename T>
    Channel<T> *create_channel(const std::string &topic, int queue_size);

    /**
     * @brief Get the channel object
     *
     * @tparam T
     * @param topic
     * @return Channel<T>*
     */
    template <typename T>
    Channel<T> *get_channel(const std::string &topic);

  private:
    ChannelManager();
    ~ChannelManager();
    std::unordered_map<std::string, BaseChannel_ *> channels_;
    SafeTaskQueueManager *task_queue_manager_;

    /**
     * @brief
     *
     * @tparam T
     * @param topic
     * @param data
     */
    template <typename T>
    void push_data(const std::string &topic, std::shared_ptr<const T> data);

    /**
     * @brief
     *
     * @param topic
     * @return true
     * @return false
     */
    bool exists(const std::string &topic) const;
}; // class ChannelManager

struct BaseChannel_
{
    BaseChannel_() : queue_id(-1), queue_size(0) {}
    int queue_id;
    int queue_size;

  protected:
    std::mutex locker_;
}; // struct Channel

template <typename T>
struct Channel : public BaseChannel_
{
    Channel() : BaseChannel_() {}
    std::list<std::function<void(const std::shared_ptr<const T>)>> callbacks;
    /**
     * @brief add callback to the channel
     *
     * @param callabck
     */
    void add_callback(std::function<void(const std::shared_ptr<const T>)> callback);
}; // struct Channel

template <typename T>
void ChannelManager::push(const std::string &topic, std::shared_ptr<const T> data)
{
    push_data(topic, data);
}

template <typename T>
void ChannelManager::push(const std::string &topic, const T &data)
{
    auto data_ptr = std::make_shared<const T>(data);
    push_data(topic, data_ptr);
}

template <typename T>
void ChannelManager::push_data(const std::string &topic, std::shared_ptr<const T> data)
{
    Channel<T> *chan = get_channel<T>(topic);
    if (!chan->callbacks.empty())
    {
        // lazy create queue, create queue on push data
        if (chan->queue_id < 0)
        {
            chan->queue_id = task_queue_manager_->create_queue(chan->queue_size);
        }
        for (auto &cb : chan->callbacks)
        {
            auto task = std::make_shared<CallBackTask<T>>(cb, data);
            task_queue_manager_->put_task(chan->queue_id, task);
        }
    }
}

template <typename T>
void ChannelManager::subscribe(
    const std::string &topic, int queue_size,
    std::function<void(const std::shared_ptr<const T>)> &callback)
{
    Channel<T> *chan;
    if (exists(topic))
    {
        chan = get_channel<T>(topic);
        if (chan->queue_id < 0)
        {
            chan->queue_size = std::min(chan->queue_size, queue_size);
        }
    }
    else
    {
        chan = create_channel<T>(topic, queue_size);
    }
    // chan never been null here
    chan->add_callback(callback);
}

template <typename T>
Channel<T> *ChannelManager::create_channel(const std::string &topic, int queue_size)
{
    Channel<T> *chan = nullptr;
    if (!exists(topic))
    {
        chan             = new Channel<T>{};
        chan->queue_size = std::min(chan->queue_size, queue_size);
        chan->queue_id   = -1; // Lazy create queue
        channels_.insert(std::make_pair(topic, chan));
    }
    return chan;
}

template <typename T>
Channel<T> *ChannelManager::get_channel(const std::string &topic)
{
    Channel<T> *chan = nullptr;
    if (exists(topic))
    {
        chan = reinterpret_cast<Channel<T> *>(channels_[topic]);
    }
    else
    {
        RERROR << "Chennel " << topic << " not found";
    }
    return chan;
}

template <typename T>
void Channel<T>::add_callback(
    std::function<void(const std::shared_ptr<const T>)> callback)
{
    std::lock_guard<std::mutex> lck(locker_);
    callbacks.push_back(callback);
}

} // namespace shared_channel
} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_CHANNEL_MANAGER_H_
