#ifndef RAINCOM_COM_TOPIC_H_
#define RAINCOM_COM_TOPIC_H_

#include "raincom_com/thread_pool/safe_task_queue_manager.hpp"
#include <cstddef>
#include <mutex>
#include <raincom_com/log.hpp>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace raincom
{
namespace com
{

class RawTopic
{
  public:
    using RawCallbackT = std::function<void(const void *const, int, const std::string &,
                                            const std::string &, double stamp)>;

    explicit RawTopic(const std::string &topic_name);
    virtual ~RawTopic() = default;
    void on_message(const void *const msg, int msg_len, const std::string &msg_type_name,
                    const std::string &msg_type_id, double stamp);
    inline void add_raw_callback(RawCallbackT callback)
    {
        std::lock_guard<std::mutex> guard(locker_); // guard for on_raw_msg_callbacks_
        on_raw_msg_callbacks_.push_back(callback);
    }
    inline const std::string &name() const { return topic_name_; }

  protected:
    const std::string topic_name_;
    std::vector<RawCallbackT> on_raw_msg_callbacks_;

    virtual void proto_process(const void *const msg, int msg_len,
                               const std::string &msg_type_name,
                               const std::string &msg_type_id, double stamp){};

  private:
    std::mutex locker_;
}; // class RawTopic

template <typename T>
class Topic : public RawTopic
{
  public:
    using CallbackT = std::function<void(const std::shared_ptr<const T>)>;

    Topic(const std::string &topic_name, int queue_size,
          SafeTaskQueueManager *task_queue_manager)
        : RawTopic(topic_name),
          queue_id_(task_queue_manager->create_queue(queue_size)),
          task_queue_manager_(task_queue_manager)
    {
    }
    virtual ~Topic() = default;

    inline void remove_callback(int callback_id)
    {
        enabled_callbacks_.at(callback_id) = false;
    }

    inline int add_callback(CallbackT callback)
    {
        enabled_callbacks_.push_back(true);
        on_msg_callbacks_.push_back(callback);
        return static_cast<int>(on_msg_callbacks_.size() - 1);
    }

  private:
    void proto_process(const void *const msg, int msg_len,
                       const std::string &msg_type_name, const std::string &msg_type_id,
                       double stamp);

    const int queue_id_;
    SafeTaskQueueManager *task_queue_manager_;
    std::vector<CallbackT> on_msg_callbacks_;
    std::vector<bool> enabled_callbacks_;
}; // class Topic

template <typename T>
class SubscriberTask : public Task
{
  public:
    virtual void init() override {}
    virtual inline void run() override { processor(msg); }
    // the message
    std::shared_ptr<T> msg;
    // message processor
    std::function<void(const std::shared_ptr<const T>)> processor;
}; // class SubscriberTask

template <typename T>
void Topic<T>::proto_process(const void *const msg, int msg_len,
                             const std::string &msg_type_name,
                             const std::string &msg_type_id, double stamp)
{
    CHECK_RERROR(msg_type_id == typeid(T).name())
        << "Message type not matched, " << msg_type_id << " != " << typeid(T).name();

    auto proto_msg{std::make_shared<T>()};
    proto_msg->ParseFromArray(msg, msg_len);
    for (size_t i = 0; i < on_msg_callbacks_.size(); ++i)
    {
        if (!enabled_callbacks_.at(i))
        {
            continue;
        }

        auto task{std::make_shared<SubscriberTask<T>>()};
        task->msg       = proto_msg;
        task->processor = on_msg_callbacks_.at(i);
        task_queue_manager_->put_task(queue_id_, task);
    }
}

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_TOPIC_H_
