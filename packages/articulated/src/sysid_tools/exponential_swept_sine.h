#ifndef EXPONENTIAL_SWEPT_SINE_H_
#define EXPONENTIAL_SWEPT_SINE_H_

#include <math.h>

namespace sysid_tools {

class ExponentialSweptSine {
  public:
    ExponentialSweptSine(double low, double high, double period, double reset_tol);
    double step(float dt);
    void reset();
  private:
    double reset_tol_ = 0.1, low_, high_, period_, t_ = 0, phase_ = 0;
};

class Add {
  public:
    Add();
    double call(double a, double b);
};

}  // namespace sysid_tools



#endif // EXPONENTIAL_SWEPT_SINE_H_
