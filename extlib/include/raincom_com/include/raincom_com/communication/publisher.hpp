#ifndef RAINCOM_COM_PUBLISHER_H_
#define RAINCOM_COM_PUBLISHER_H_

#include "raincom_com/communication/topic_manager/topic.hpp"
#include "raincom_com/communication/topic_manager/topic_manager.hpp"
#include "raincom_com/communication/base_publisher.hpp"
#include "raincom_com/communication/fastdds/publisher_impl.hpp"
#include "raincom_com/communication/shared_channel/publisher_impl.hpp"
#include "raincom_com/time.hpp"
#include <memory>
#include <vector>

namespace raincom
{
namespace com
{

class SafeTaskQueueManager;

template <typename T>
class Publisher
{
  public:
    Publisher(const std::string &topic_name, int queue_size,
              SafeTaskQueueManager *task_queue_manager, bool is_private_topic);
    ~Publisher();
    inline const std::string &topic_name() const { return topic_->name(); }
    void publish(const T &proto);
    // void publish(const std::shared_ptr<const T> proto);
    int num_subscriber() const;

  private:
    const int queue_size_;
    Topic<T> *topic_;
    std::shared_ptr<BasePublisher> underlay_publisher_;
    bool is_private_topic_;
}; // class Publisher

class UnderlayPublisherManager
{
    DEF_SINGLETON(UnderlayPublisherManager)
  public:
    void add(std::shared_ptr<BasePublisher> sub);
    void destroy();

  private:
    UnderlayPublisherManager() {}
    std::vector<std::shared_ptr<BasePublisher>> pubs_;
}; // class UnderlayPublisherManager

template <typename T>
Publisher<T>::Publisher(const std::string &topic_name, int queue_size,
                        SafeTaskQueueManager *task_queue_manager, bool is_private_topic)
    : queue_size_(queue_size), is_private_topic_(is_private_topic)
{
    if (is_private_topic)
    {
        underlay_publisher_.reset(
            new shared_channel::PublisherImpl<T>(topic_name, queue_size));
    }
    else
    {
        auto topic_manager = TopicManager::instance();
        topic_manager->create_pub_topic<T>(topic_name, queue_size);
        topic_ = topic_manager->get_pub_topic<T>(topic_name);
        underlay_publisher_.reset(
            new fastdds::PublisherImpl(topic_name, queue_size, task_queue_manager));
    }

    UnderlayPublisherManager::instance()->add(underlay_publisher_);
}

template <typename T>
Publisher<T>::~Publisher()
{
}

template <typename T>
void Publisher<T>::publish(const T &proto)
{
    if (is_private_topic_)
    {
        /// TODO: optimize me.
        auto pub = reinterpret_cast<shared_channel::PublisherImpl<T> *>(
            underlay_publisher_.get());
        pub->publish(proto);
    }
    else
    {
        int msg_len = proto.ByteSize();
        std::vector<uint8_t> bytes(msg_len);
        proto.SerializeToArray(&(bytes[0]), bytes.size());
        topic_->on_message(static_cast<void *>(&(bytes[0])), msg_len, proto.GetTypeName(),
                           typeid(T).name(), Time::now().to_sec());
    }
}

// template<typename T>
// void Publisher<T>::publish(const std::shared_ptr<const T> proto)
// {
//     underlay_publish(*proto);
// }

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_PUBLISHER_H_
