#ifndef RAINCOM_COM_CIRCULAR_QUEUE_H_
#define RAINCOM_COM_CIRCULAR_QUEUE_H_

#include <cstddef>
#include <stdexcept>
#include <vector>

namespace raincom
{
namespace com
{
namespace algorithm
{

template <typename T>
class CircularQueue
{
  public:
    CircularQueue(int size = 1);
    // CircularQueue(const CircularQueue&);
    // CircularQueue(const CircularQueue&&);
    ~CircularQueue() = default;
    // CircularQueue& operator=(const CircularQueue&);
    // CircularQueue& operator=(const CircularQueue&&);
    bool push(const T &data, bool overwrite = false);
    bool front(T &data) const;
    const T &front() const;
    const T &back() const;
    bool pop();
    const T &at(int i) const;
    bool empty() const;
    bool full() const;
    size_t size() const;
    void clear()
    {
        index_front_ = 0;
        index_rear_  = 0;
    }

  private:
    std::vector<T> data_;
    int index_front_;
    int index_rear_;
    int size_;
    // For the case of size = 1
    T single_data_;
    bool single_data_seted_;
}; // class CircularQueue

template <typename T>
CircularQueue<T>::CircularQueue(int size)
    : data_(std::vector<T>(size)),
      index_front_(0),
      index_rear_(0),
      size_(size),
      single_data_seted_(false)
{
}

template <typename T>
bool CircularQueue<T>::push(const T &data, bool overwrite)
{
    if (size_ == 1)
    {
        // overwrite the data by anyway
        single_data_       = data;
        single_data_seted_ = true;
        return true;
    }

    if (full())
    {
        if (overwrite)
        {
            pop();
        }
        else
        {
            return false;
        }
    }

    data_.at(index_front_) = data;
    index_front_           = (index_front_ + 1) % size_;
    return true;
}

template <typename T>
bool CircularQueue<T>::pop()
{
    if (size_ == 1)
    {
        if (!empty())
        {
            single_data_seted_ = false;
            return true;
        }
        else
        {
            // Means the queue is empty
            return false;
        }
    }

    if (empty())
    {
        return false;
    }
    else
    {
        index_rear_ = (index_rear_ + 1) % size_;
        return true;
    }
}

template <typename T>
const T &CircularQueue<T>::at(int i) const
{
    if (i < 0 || i >= size())
    {
        throw std::runtime_error("index out of range");
    }

    return data_.at((i + index_rear_) % size_);
}

template <typename T>
const T &CircularQueue<T>::front() const
{
    if (empty())
    {
        throw std::runtime_error("Que empty.");
    }
    else
    {
        return data_.at(index_rear_);
    }
}

template <typename T>
const T &CircularQueue<T>::back() const
{
    if (empty())
    {
        throw std::runtime_error("Que empty.");
    }
    else
    {
        return data_.at((index_front_ - 1 + size_) % size_);
    }
}

template <typename T>
bool CircularQueue<T>::front(T &data) const
{
    if (size_ == 1)
    {
        if (!empty())
        {
            data = single_data_;
            return true;
        }
        else
        {
            return false;
        }
    }

    if (empty())
    {
        return false;
    }
    else
    {
        data = data_.at(index_rear_);
        return true;
    }
}

template <typename T>
bool CircularQueue<T>::empty() const
{
    if (size_ == 1)
    {
        if (!single_data_seted_)
            return true;
        else
            return false;
    }

    if (index_front_ == index_rear_)
        return true;
    else
        return false;
}

template <typename T>
bool CircularQueue<T>::full() const
{
    if (size_ == 1)
    {
        // ignore the full status
        // we overwrite it anyway
        return false;
    }

    if (((index_front_ + 1) % size_) == index_rear_)
        return true;
    else
        return false;
}

template <typename T>
size_t CircularQueue<T>::size() const
{
    if (index_front_ >= index_rear_)
    {
        return index_front_ - index_rear_;
    }
    else
    {
        return (index_front_ + size_ - index_rear_) % size_;
    }
}

} // namespace algorithm
} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_CIRCULAR_QUEUE_H_
