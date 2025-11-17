#ifndef RAINCOM_COM_SUBSCRIBER_H_
#define RAINCOM_COM_SUBSCRIBER_H_

#include "raincom_com/communication/topic_manager/topic.hpp"
#include "raincom_com/communication/topic_manager/topic_manager.hpp"
#include "raincom_com/communication/base_subscriber.hpp"
#include "raincom_com/communication/fastdds/subscriber_impl.hpp"
#include "raincom_com/communication/shared_channel/subscriber_impl.hpp"
#include "raincom_com/macros.hpp"
#include <memory>
#include <functional>
#include <mutex>
#include <vector>

namespace raincom
{
namespace com
{

template <typename T>
class Subscriber
{
  public:
    using CallbackT = std::function<void(const std::shared_ptr<const T>)>;

    Subscriber(const std::string &topic_name, int queue_size, CallbackT callback,
               SafeTaskQueueManager *task_queue_manager, bool is_private_topic);
    ~Subscriber();
    inline const std::string &topic_name() const { return topic_->name(); }
    void remove();

  private:
    Topic<T> *topic_;
    CallbackT callback_;
    bool removed_;
    int callback_id_;
    std::shared_ptr<BaseSubscriber> underlay_sub_;
}; // class Subscriber

class UnderlaySubscriberManager
{
    DEF_SINGLETON(UnderlaySubscriberManager)
  public:
    void add(std::shared_ptr<BaseSubscriber> sub);
    void remove(std::shared_ptr<BaseSubscriber> sub);
    void destroy();

  private:
    UnderlaySubscriberManager() {}
    std::mutex locker_;
    std::vector<std::shared_ptr<BaseSubscriber>> subs_;
}; // class UnderlaySubscriberManager

template <typename T>
Subscriber<T>::Subscriber(const std::string &topic_name, int queue_size,
                          CallbackT callback, SafeTaskQueueManager *task_queue_manager,
                          bool is_private_topic)
    : callback_(callback), removed_(false), callback_id_(-1)
{
    if (is_private_topic)
    {
        underlay_sub_.reset(
            new shared_channel::SubscriberImpl<T>(topic_name, queue_size, callback));
    }
    else
    {
        auto topic_manager = TopicManager::instance();
        topic_manager->create_sub_topic<T>(topic_name, queue_size);
        topic_       = topic_manager->get_sub_topic<T>(topic_name);
        callback_id_ = topic_->add_callback(callback);
        underlay_sub_.reset(
            new fastdds::SubscriberImpl(topic_name, queue_size, task_queue_manager));
    }
    UnderlaySubscriberManager::instance()->add(underlay_sub_);
}

template <typename T>
void Subscriber<T>::remove()
{
    if (!removed_ && callback_id_ >= 0)
    {
        topic_->remove_callback(callback_id_);
        removed_ = true;
    }
}

template <typename T>
Subscriber<T>::~Subscriber()
{
    UnderlaySubscriberManager::instance()->remove(underlay_sub_);
    underlay_sub_->destroy();
    underlay_sub_ = nullptr;
}

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_SUBSCRIBER_H_
