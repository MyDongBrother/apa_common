#ifndef RAINCOM_COM_ALGORITHM_INDEXED_LIST_H_
#define RAINCOM_COM_ALGORITHM_INDEXED_LIST_H_

#include "raincom_com/log.hpp"
#include <unordered_map>
#include <utility>
#include <vector>

namespace raincom
{
namespace com
{
namespace algorithm
{

template <typename I, typename T>
class IndexedVector
{
  public:
    IndexedVector() = default;
    IndexedVector(const IndexedVector &other);
    IndexedVector &operator=(const IndexedVector &other);
    bool exists(I key);
    void put(const I &key, const T &value);
    T *get(const I &key);
    const T *get(const I &key) const;
    const std::vector<const T *> &values() const;
    const std::unordered_map<I, T> &items() const;
    std::unordered_map<I, T> &mutable_items();

  private:
    std::vector<const T *> vec_;
    std::unordered_map<I, T> mapped_;
}; // class IndexedList

template <typename I, typename T>
IndexedVector<I, T>::IndexedVector(const IndexedVector<I, T> &other)
{
    *this = other;
}

template <typename I, typename T>
IndexedVector<I, T> &IndexedVector<I, T>::operator=(const IndexedVector &other)
{
    vec_.clear();
    mapped_ = other.mapped_;
    for (const auto &it : mapped_)
    {
        vec_.push_back(&it.second);
    }
    return *this;
}

template <typename I, typename T>
bool IndexedVector<I, T>::exists(I key)
{
    if (mapped_.find(key) == mapped_.end())
        return false;
    return true;
}

template <typename I, typename T>
void IndexedVector<I, T>::put(const I &key, const T &value)
{
    if (exists(key))
    {
        RWARN << "object " << key << " already exists.";
    }
    else
    {
        mapped_.insert(std::make_pair(key, value));
        vec_.push_back(&mapped_.at(key));
    }
}

template <typename I, typename T>
T *IndexedVector<I, T>::get(const I &key)
{
    if (exists(key))
    {
        return &mapped_.at(key);
    }

    return nullptr;
}

template <typename I, typename T>
const T *IndexedVector<I, T>::get(const I &key) const
{
    return get(key);
}

template <typename I, typename T>
const std::vector<const T *> &IndexedVector<I, T>::values() const
{
    return vec_;
}

template <typename I, typename T>
const std::unordered_map<I, T> &IndexedVector<I, T>::items() const
{
    return mapped_;
}

template <typename I, typename T>
std::unordered_map<I, T> &IndexedVector<I, T>::mutable_items()
{
    return mapped_;
}

} // namespace algorithm
} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_ALGORITHM_INDEXED_LIST_H_
