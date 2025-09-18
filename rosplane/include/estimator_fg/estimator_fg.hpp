#ifndef ESTIMATOR_FG_HPP
#define ESTIMATOR_FG_HPP

#include "estimator_ros.hpp"

namespace rosplane
{

class EstimatorFG : public EstimatorROS
{
public:
  EstimatorFG();

  virtual void estimate(const Input & input, Output & output);
};

} // namespace rosplane

#endif // ESTIMATOR_FG_HPP
