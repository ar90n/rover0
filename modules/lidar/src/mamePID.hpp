#ifndef MAMEPID_HPP_
#define MAMEPID_HPP_

#include <algorithm>
#include <concepts>
#include <limits>
#include <type_traits>

namespace mamePID {

template<typename T, typename U>
concept Component = requires(T t) {
  { t.calculate(U{}, U{}) } -> std::same_as<U>;
};

template<typename T>
concept CoeffMutable = requires(T t) {
  { t.set } -> std::invocable<T, typename T::value_type>;
};

template<typename T>
class Zero
{
public:
  using value_type = T;

  Zero() {}

  T    calculate(T, T) { return 0.0; }
  void set(T) {}
};

template<typename T>
class Proportional
{
public:
  using value_type = T;

  Proportional(T kp, T)
    : kp(kp)
  {
  }

  T calculate(T setpoint, T pv)
  {
    const T error = setpoint - pv;
    return kp * error;
  }

private:
  const T kp;
};

template<typename T>
class Integral
{
public:
  using value_type = T;
  Integral(T ki, T dt, T minv = std::numeric_limits<T>::lowest(), T maxv = std::numeric_limits<T>::max())
    : ki(ki * dt)
    , minv(minv)
    , maxv(maxv)
    , integral(0)
  {
  }

  T calculate(T setpoint, T pv)
  {
    const T error  = setpoint - pv;
    integral      += ki * error;
    integral       = std::clamp(integral, minv, maxv);
    return integral;
  }

private:
  const T ki;
  const T minv;
  const T maxv;
  T       integral;
};

template<typename T>
class Derivative
{
public:
  using value_type = T;

  Derivative(T kd, T dt)
    : kd(kd / dt)
    , pre_error(0)
  {
  }

  T calculate(T setpoint, T pv)
  {
    const T error      = setpoint - pv;
    const T derivative = error - pre_error;
    pre_error          = error;
    return kd * derivative;
  }

private:
  const T kd;
  T       pre_error;
};

template<typename T>
class PrecedingProportional
{
public:
  using value_type = T;

  PrecedingProportional(T kp, T)
    : kp(kp)
  {
  }

  T calculate(T, T pv) { return -kp * pv; }

private:
  const T kp;
};

template<typename T>
class PrecedingDerivative
{
public:
  using value_type = T;
  PrecedingDerivative(T kd, T dt)
    : kd(kd / dt)
    , pre_pv(0)
  {
  }

  T calculate(T, T pv)
  {
    const T derivative = pv - pre_pv;
    pre_pv             = pv;
    return -kd * derivative;
  }

private:
  const T kd;
  T       pre_pv;
};

template<typename T, Component<T> ProportionalT, Component<T> IntegralT, Component<T> DerivativeT>
class PID
{
public:
  using value_type = T;

  PID(ProportionalT proportional, IntegralT integral, DerivativeT derivative, T min, T max)
    : proportional(proportional)
    , integral(integral)
    , derivative(derivative)
    , min(min)
    , max(max)
  {
  }

  T calculate(T setpoint, T pv)
  {
    T output = proportional.calculate(setpoint, pv) + integral.calculate(setpoint, pv) +
               derivative.calculate(setpoint, pv);
    return std::clamp(output, min, max);
  }

  void setKp(T kp)
    requires CoeffMutable<ProportionalT>
  {
    proportional.set(kp);
  }

  void setKi(T ki)
    requires CoeffMutable<IntegralT>
  {
    integral.set(ki);
  }

  void setKd(T kd)
    requires CoeffMutable<DerivativeT>
  {
    derivative.set(kd);
  }

private:
  ProportionalT proportional;
  IntegralT     integral;
  DerivativeT   derivative;
  const T       min;
  const T       max;
};

template<typename T>
auto pi(T kp, T ki, T sp, T min = std::numeric_limits<T>::lowest(), T max = std::numeric_limits<T>::max())
{
  return PID<T, Proportional<T>, Integral<T>, Zero<T>>(
    Proportional<T>(kp, sp), Integral<T>(ki, sp, min, max), Zero<T>(), min, max
  );
}

template<typename T>
auto pd(T kp, T kd, T sp, T min = std::numeric_limits<T>::lowest(), T max = std::numeric_limits<T>::max())
{
  return PID<T, Proportional<T>, Zero<T>, Derivative<T>>(
    Proportional<T>(kp, sp), Zero<T>(), Derivative<T>(kd, sp), min, max
  );
}

template<typename T>
auto pid(
  T kp,
  T ki,
  T kd,
  T sp,
  T min = std::numeric_limits<T>::lowest(),
  T max = std::numeric_limits<T>::max()
)
{
  return PID<T, Proportional<T>, Integral<T>, Derivative<T>>(
    Proportional<T>(kp, sp), Integral<T>(ki, sp, min, max), Derivative<T>(kd, sp), min, max
  );
}

template<typename T>
auto pi_d(
  T kp,
  T ki,
  T kd,
  T sp,
  T min = std::numeric_limits<T>::lowest(),
  T max = std::numeric_limits<T>::max()
)
{
  return PID<T, Proportional<T>, Integral<T>, PrecedingDerivative<T>>(
    Proportional<T>(kp, sp), Integral<T>(ki, sp, min, max), PrecedingDerivative<T>(kd, sp), min, max
  );
}

template<typename T>
auto i_pd(
  T kp,
  T ki,
  T kd,
  T sp,
  T min = std::numeric_limits<T>::lowest(),
  T max = std::numeric_limits<T>::max()
)
{
  return PID<T, PrecedingProportional<T>, Integral<T>, PrecedingDerivative<T>>(
    PrecedingProportional<T>(kp, sp), Integral<T>(ki, sp, min, max), PrecedingDerivative<T>(kd, sp), min, max
  );
}

} // namespace mamePID

#endif // MAMEPID_HPP_