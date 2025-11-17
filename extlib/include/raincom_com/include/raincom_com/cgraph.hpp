#ifndef RAINCOM_COM_CGRAPH_H_
#define RAINCOM_COM_CGRAPH_H_

#include "raincom_com/communication/subscriber.hpp"
#include "raincom_com/communication/publisher.hpp"
// #include "raincom_com/communication/topic_manager/topic.hpp"
#include "raincom_com/log.hpp"
#include "raincom_com/macros.hpp"
#include "raincom_com/thread_pool/safe_task_queue_manager.hpp"
#include "raincom_com/thread_pool/thread_pool.hpp"
#include "raincom_com/time.hpp"
#include "raincom_com/utils.hpp"
#include <unordered_map>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace raincom
{
namespace com
{

template <typename T>
class DNode;

static const int QUE_SIZE{10};

/**
 * @brief Computation graph
 *
 */
class CGraph
{
    DEF_SINGLETON(CGraph)
  public:
    template <typename T>
    static T *get(const std::string &name);

  public:
    bool exists(const std::string &name);

  private:
    CGraph() {}
    template <typename T>
    T *underlay_get(const std::string &name);

    std::mutex locker_;
    std::unordered_map<std::string, void *> nodes_;
}; // class CGraph

template <typename T>
class DNode
{
  private:
    using CallbackT = std::function<void(const std::shared_ptr<const T>)>;

  public:
    virtual ~DNode() = default;
    void update(const T &data);
    std::shared_ptr<const T> get();
    std::shared_ptr<T> mut();
    void subscribe(CallbackT cb, int que_size);
    virtual bool valid() const { return valid_; }
    const std::string &name() const { return name_; }
    double stamp() const { return data_->header().stamp(); }
    com::Time last_update_stamp() { return last_update_stamp_; }
    friend class CGraph;

  protected:
    explicit DNode(const std::string &name);
    virtual void on_update(const T &data) {}

    std::mutex locker_;
    std::shared_ptr<T> data_;

  private:
    void publish(const T &data);
    void underlay_update(const T &data, bool update_from_dss);
    void on_dds(const std::shared_ptr<const T> data);

    std::shared_ptr<Publisher<T>> pub_;
    std::shared_ptr<Subscriber<T>> sub_;
    std::vector<CallbackT> callbacks_;
    SafeTaskQueueManager *task_queue_manager_;
    std::string name_;
    com::Time last_update_stamp_;
    bool valid_;
    int queue_id_;
}; // class DNode

template <typename T>
DNode<T>::DNode(const std::string &name)
    : data_(std::make_shared<T>()), name_(name), valid_(false)
{
    task_queue_manager_ = ThreadPool::instance()->task_queue_manager();
    queue_id_           = task_queue_manager_->create_queue(QUE_SIZE);
    if (!is_private_topic(name.c_str()))
    {
        sub_ = std::make_shared<Subscriber<T>>(
            name_, QUE_SIZE, std::bind(&DNode::on_dds, this, std::placeholders::_1),
            task_queue_manager_, false);
    }
}

template <typename T>
void DNode<T>::update(const T &data)
{
    if (sub_)
    {
        sub_->remove();
        sub_ = nullptr;
    }
    underlay_update(data, false);
}

template <typename T>
void DNode<T>::underlay_update(const T &data, bool update_from_dss)
{
    {
        std::lock_guard<std::mutex> guard(locker_);
        last_update_stamp_ = com::Time::now();
        data_              = std::make_shared<T>(data); // guard
        valid_             = true;
    }

    if (!callbacks_.empty())
    {
        for (auto cb : callbacks_)
        {
            cb(data_);
            // auto task{std::make_shared<SubscriberTask<T>>()};
            // task->msg = data_ptr;
            // task->processor = cb;
            // task_queue_manager_->put_task(queue_id_, task);
        }
    }

    on_update(data);
    if (!update_from_dss && !com::is_private_topic(name_.c_str()))
    {
        publish(data);
    }
}

template <typename T>
void DNode<T>::on_dds(const std::shared_ptr<const T> data)
{
    underlay_update(*data, true);
}

template <typename T>
void DNode<T>::publish(const T &data)
{
    {
        std::lock_guard<std::mutex> guard(locker_);
        if (!pub_)
        {
            // lazy creation
            pub_ = std::make_shared<Publisher<T>>(name_, QUE_SIZE, task_queue_manager_,
                                                  false);
        }
    }

    pub_->publish(data);
}

template <typename T>
std::shared_ptr<const T> DNode<T>::get()
{
    std::lock_guard<std::mutex> guard(locker_);
    if (!valid())
    {
        return nullptr;
    }
    return data_;
}

template <typename T>
std::shared_ptr<T> DNode<T>::mut()
{
    std::lock_guard<std::mutex> guard(locker_);
    if (!valid())
    {
        return nullptr;
    }
    return data_;
}

template <typename T>
void DNode<T>::subscribe(CallbackT cb, [[maybe_unused]] int que_size)
{
    std::lock_guard<std::mutex> guard(locker_);
    callbacks_.push_back(cb);
}

template <typename T>
T *CGraph::get(const std::string &name)
{
    return instance()->underlay_get<T>(name);
}

template <typename T>
T *CGraph::underlay_get(const std::string &name)
{
    std::lock_guard<std::mutex> guard(locker_);
    if (!exists(name))
    {
        RINFO << "Create node " << name;
        T *node{new T(name)};
        nodes_.insert(std::make_pair(node->name(), reinterpret_cast<void *>(node)));
        return node;
    }
    return reinterpret_cast<T *>(nodes_.at(name));
}

inline bool CGraph::exists(const std::string &name)
{
    if (nodes_.find(name) == nodes_.end())
    {
        return false;
    }
    return true;
}

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_CGRAPH_H_
