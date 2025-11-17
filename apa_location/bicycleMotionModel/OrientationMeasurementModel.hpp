#pragma once

#include "bicycleMotionModel/SystemModel.hpp"
#include "kalman/LinearizedMeasurementModel.hpp"

namespace bicycle_motion
{

template <typename T>
class OrientationMeasurement : public Kalman::Vector<T, 1>
{
  public:
    KALMAN_VECTOR(OrientationMeasurement, T, 1)

    static constexpr size_t THETA = 0;

    T theta() const { return (*this)[THETA]; }
    T &theta() { return (*this)[THETA]; }
};

template <typename T, template <class> class CovarianceBase = Kalman::StandardBase>
class OrientationMeasurementModel
    : public Kalman::LinearizedMeasurementModel<State<T>, OrientationMeasurement<T>,
                                                CovarianceBase>
{
  public:
    typedef bicycle_motion::State<T> S;

    typedef bicycle_motion::OrientationMeasurement<T> M;

    OrientationMeasurementModel()
    {
        this->H.setIdentity();
        this->V.setIdentity() * 0.02;
    }

    M h(const S &x) const
    {
        M measurement;

        measurement.theta() = x.theta();

        return measurement;
    }
};

} // namespace bicycle_motion