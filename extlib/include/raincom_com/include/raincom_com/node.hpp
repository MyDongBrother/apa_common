#ifndef RAINCOM_COM_NODE_H_
#define RAINCOM_COM_NODE_H_

#include "raincom_com/duration.hpp"
#include "raincom_com/timer/timer.hpp"
#include "raincom_com/nodelet.hpp"
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace raincom
{
namespace com
{

class Node
{
  public:
    static void spin(int num_threads = 1, bool handle_sig = false);
    static void add_shutdown_callback(std::function<void()> callback);
    static void shutdown();
    static void is_shutting_down();
    static void spin_once();

  public:
    /**
     * @brief Construct a new Node object
     *
     * @param name node name
     */
    Node(const std::string &name);
    ~Node();

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

    /**
     * @brief Return the node status
     *
     * @return true
     * @return false
     */
    bool ok();

    /**
     * @brief
     *
     * @param nodelet
     */
    void add_nodelet(std::shared_ptr<Nodelet> nodelet);

  private:
    const std::string name_;
    std::shared_ptr<Nodelet> main_nodelet_;
    std::vector<std::shared_ptr<Nodelet>> nodelets_;
    static bool terminated_;
    static std::vector<std::function<void()>> shutdown_callbacks_;
    // std::vector<std::shared_ptr<Subscriber>> subscribers_;
}; // class Node

template <typename T>
std::shared_ptr<Publisher<T>> Node::create_publisher(const std::string &topic,
                                                     int queue_size)
{
    return main_nodelet_->create_publisher<T>(topic, queue_size);
}

template <typename T>
std::shared_ptr<Subscriber<T>> Node::create_subscription(
    const std::string &topic, int queue_size,
    std::function<void(const std::shared_ptr<const T>)> callback)
{
    return main_nodelet_->create_subscription(topic, queue_size, callback);
}

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_NODE_H_
