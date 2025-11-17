#ifndef RAINCOM_COM_NODELET_H_
#define RAINCOM_COM_NODELET_H_

#include "raincom_com/communication/publisher.hpp"
#include "raincom_com/communication/subscriber.hpp"
#include "raincom_com/duration.hpp"
#include "raincom_com/timer/timer.hpp"
#include "raincom_com/utils.hpp"
#include <memory>

class SafeTaskQueueManager;

namespace raincom
{
namespace com
{

class Node;

class Nodelet
{
  public:
    Nodelet();
    virtual ~Nodelet();

    virtual void init() = 0;

    /**
     * @brief Create a publisher object
     *
     * @tparam DataT
     * @param topic
     * @param queue_size
     * @return std::shared_ptr<Publisher>
     */
    template <typename T>
    [[nodiscard]] std::shared_ptr<Publisher<T>> create_publisher(const std::string &topic,
                                                                 int queue_size);

    /**
     * @brief Create a timer object
     *
     * @param duration
     * @param callback
     * @param queue_size
     */
    [[nodiscard]] std::shared_ptr<Timer> create_timer(const Duration &&period,
                                                      int queue_size,
                                                      Timer::CallbackT callback);

    /**
     * @brief Create a timer object
     *
     * @param duration
     * @param callback
     * @param queue_size
     */
    [[nodiscard]] std::shared_ptr<Timer> create_pure_timer(const Duration &&period,
                                                           int queue_size,
                                                           Timer::CallbackT callback);

    /**
     * @brief Create a subscription object
     *
     * @tparam T
     * @param topic
     * @param queue_size
     * @param callback
     * @return std::shared_ptr<Subscriber<T>>
     */
    template <typename T>
    [[nodiscard]] std::shared_ptr<Subscriber<T>> create_subscription(
        const std::string &topic, int queue_size,
        std::function<void(const std::shared_ptr<const T>)> callback);
    friend class Node;

  private:
    void initialize();
    bool initied_;
    SafeTaskQueueManager *task_queue_manager_;
}; // class Nodelet

template <typename T>
std::shared_ptr<Publisher<T>> Nodelet::create_publisher(const std::string &topic,
                                                        int queue_size)
{
    std::shared_ptr<Publisher<T>> pub = std::make_shared<Publisher<T>>(
        topic, queue_size, task_queue_manager_, is_private_topic(topic.c_str()));
    // publishers_.push_back(pub);
    return pub;
}

template <typename T>
std::shared_ptr<Subscriber<T>> Nodelet::create_subscription(
    const std::string &topic, int queue_size,
    std::function<void(const std::shared_ptr<const T>)> callback)
{
    std::shared_ptr<Subscriber<T>> sub =
        std::make_shared<Subscriber<T>>(topic, queue_size, callback, task_queue_manager_,
                                        is_private_topic(topic.c_str()));
    // subscribers_.push_back(sub);
    return sub;
}

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_NODELET_H_
