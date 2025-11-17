#pragma once
#include "kalman/LinearizedSystemModel.hpp"

namespace bicycle_motion
{

template <typename T>
class State : public Kalman::Vector<T, 3>
{
  public:
    KALMAN_VECTOR(State, T, 3)
    static constexpr size_t X     = 0;
    static constexpr size_t Y     = 1;
    static constexpr size_t THETA = 2;

    T x() const { return (*this)[X]; }
    T y() const { return (*this)[Y]; }
    T theta() const { return (*this)[THETA]; }

    T &x() { return (*this)[X]; }
    T &y() { return (*this)[Y]; }
    T &theta() { return (*this)[THETA]; }
};

template <typename T>
class Control : public Kalman::Vector<T, 2>
{
  public:
    KALMAN_VECTOR(Control, T, 2)

    static constexpr size_t V      = 0;
    static constexpr size_t DTHETA = 1;

    T v() const { return (*this)[V]; }
    T dtheta() const { return (*this)[DTHETA]; }

    T &v() { return (*this)[V]; }
    T &dtheta() { return (*this)[DTHETA]; }
};

template <typename T, template <class> class CovarianceBase = Kalman::StandardBase>
class SystemModel
    : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
  public:
    typedef bicycle_motion::State<T> S;
    typedef bicycle_motion::Control<T> C;

    S f(const S &x, const C &u) const
    {
        S x_;

        auto newOrientation = x.theta() + u.dtheta();
        x_.theta()          = newOrientation;

        x_.x() = x.x() + std::cos(newOrientation) * u.v();
        x_.y() = x.y() + std::sin(newOrientation) * u.v();

        return x_;
    }

  protected:
    void updateJacobians(const S &x, const C &u)
    {
        // F = df/dx (Jacobian of state transition w.r.t. the state)
        this->F.setZero();

        // partial derivative of x.x() w.r.t. x.x()
        this->F(S::X, S::X) = 1;
        // partial derivative of x.x() w.r.t. x.theta()
        this->F(S::X, S::THETA) = -std::sin(x.theta() + u.dtheta()) * u.v();

        // partial derivative of x.y() w.r.t. x.y()
        this->F(S::Y, S::Y) = 1;
        // partial derivative of x.y() w.r.t. x.theta()
        this->F(S::Y, S::THETA) = std::cos(x.theta() + u.dtheta()) * u.v();

        // partial derivative of x.theta() w.r.t. x.theta()
        this->F(S::THETA, S::THETA) = 1;

        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        this->W.setIdentity();
    }
};

} // namespace bicycle_motion