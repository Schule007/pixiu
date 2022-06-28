#include "exponential_swept_sine.h"

namespace sysid_tools {

Add::Add() {}

double Add::call(double a, double b) {
  return a + b;
}

ExponentialSweptSine::ExponentialSweptSine(
  double low, double high, double period, double reset_tol
): low_(low), high_(high), period_(period), reset_tol_(reset_tol) {}

void ExponentialSweptSine::reset() {
  t_ = 0;
  phase_ = 0;
}

double ExponentialSweptSine::step(float dt) {
  t_ = t_ + dt;
  if (
    (std::abs(phase_) < reset_tol_ || std::abs(phase_ - 2 * M_PI) < reset_tol_)
    && t_ > period_
  ) {t_ = 0;}
  double hl_ratio = high_ / low_;
  phase_ = fmod(
    2 * M_PI * low_ * period_
    * (exp(t_ * log(hl_ratio) / period_) - 1)
    / log(hl_ratio)
    , 2 * M_PI
  );
  return std::sin(phase_);
}

}  // namespace sysid_tools

