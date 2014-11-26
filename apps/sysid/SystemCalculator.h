#ifndef SYSTEMCALCULATOR_H
#define SYSTEMCALCULATOR_H

#include "dart/math/MathTypes.h"

namespace Eigen {
typedef Eigen::Matrix<double, 6, 10> Matrix0610d;
} // namespace Eigen

namespace dart {
namespace dynamics {
class Skeleton;
} // namespace dynamics
} // namespace dart

class SystemCalculator
{
public:
  SystemCalculator(const dart::dynamics::Skeleton* robot);

  Eigen::Matrix0610d computeA(size_t index) const;
  Eigen::Matrix6d computeSpatialTransform(size_t from, size_t to) const;


protected:
  const dart::dynamics::Skeleton* mRobot;

};

#endif // SYSTEMCALCULATOR_H
