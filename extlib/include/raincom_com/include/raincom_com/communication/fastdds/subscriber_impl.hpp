#ifndef RAINCOM_COM_FASTDDS_SUBSCRIBER_H_
#define RAINCOM_COM_FASTDDS_SUBSCRIBER_H_

#include "raincom_com/communication/topic_manager/topic.hpp"
#include "raincom_com/thread_pool/safe_task_queue_manager.hpp"
#include "raincom_com/communication/fastdds/message/underlay_message.h"
#include "raincom_com/communication/base_subscriber.hpp"
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>

namespace raincom
{
namespace com
{
namespace fastdds
{

class SubListener;
class SubscriberTask;

class SubscriberImpl : public BaseSubscriber
{
  public:
    SubscriberImpl(const std::string &topic_name, int queue_size,
                   SafeTaskQueueManager *task_queue_manager);
    virtual ~SubscriberImpl() override;
    virtual void destroy() override;
    const std::string &topic_name() { return topic_name_; }

    friend class SubListener;

  private:
    SubscriberImpl(const SubscriberImpl &)             = delete;
    SubscriberImpl(const SubscriberImpl &&)            = delete;
    SubscriberImpl &operator=(const SubscriberImpl &)  = delete;
    SubscriberImpl &operator=(const SubscriberImpl &&) = delete;
    void msg_processor(const UnderlayMessage &msg);
    void destroy_();

    eprosima::fastdds::dds::DomainParticipant *participant_;
    eprosima::fastdds::dds::Subscriber *subscriber_;
    eprosima::fastdds::dds::Topic *dds_topic_;
    eprosima::fastdds::dds::DataReader *reader_;
    eprosima::fastdds::dds::TypeSupport type_;

    RawTopic *raw_topic_;
    SubListener *listener_;
    const std::string topic_name_;
    bool destroyed_;
    bool shutdown_;
}; // class Subscriber

class SubListener : public eprosima::fastdds::dds::DataReaderListener
{
  public:
    SubListener(SubscriberImpl *subscriber) : matched_(0), subscriber_(subscriber) {}
    ~SubListener() override = default;
    void on_data_available(eprosima::fastdds::dds::DataReader *reader) override;
    void on_subscription_matched(
        eprosima::fastdds::dds::DataReader *reader,
        const eprosima::fastdds::dds::SubscriptionMatchedStatus &info) override;

  private:
    int matched_;
    SubscriberImpl *subscriber_;
}; // class SubListener

} // namespace fastdds
} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_FASTDDS_SUBSCRIBER_H_
