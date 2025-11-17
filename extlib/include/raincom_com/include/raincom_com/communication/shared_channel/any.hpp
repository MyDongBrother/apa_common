#ifndef RAINCOM_COM_SHARED_CHANNEL_ANY_H_
#define RAINCOM_COM_SHARED_CHANNEL_ANY_H_

namespace raincom
{
namespace com
{
namespace shared_channel
{

class Any
{
  public:
    template <typename T>
    explicit Any(const T &data) : content_(new Holder<T>(data))
    {
    }
    Any(const Any &other) : content_(other.content_ ? other.content_ : nullptr) {}
    ~Any() { delete content_; };

    template <typename T>
    T *cast();
    template <typename T>
    bool cast(T &data);

  private:
    class Placeholder
    {
      public:
        virtual ~Placeholder()       = 0;
        virtual Placeholder *clone() = 0;
    }; // class Placeholder

    template <typename T>
    class Holder : public Placeholder
    {
      public:
        explicit Holder(const T &data) : data_(data) {}
        virtual ~Holder() {}
        virtual inline Placeholder *clone() { return new T(data_); }

      private:
        T data_;
    }; // class Holder

    Placeholder *content_;
}; // class Any

template <typename T>
T *Any::cast()
{
    if (content_ != nullptr)
    {
        return static_cast<Holder<T> *>(content_)->data_;
    }
    return nullptr;
}

template <typename T>
bool Any::cast(T &data)
{
    if (content_ != nullptr)
    {
        data = *(static_cast<Holder<T> *>(content_)->data_);
        return true;
    }
    return false;
}

} // namespace shared_channel
} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_SHARED_CHANNEL_ANY_H_
