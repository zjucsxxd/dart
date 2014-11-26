#include "SystemCalculator.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"


SystemCalculator::SystemCalculator(const dart::dynamics::Skeleton* robot) :
  mRobot(robot)
{

}

static Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d S;
  S <<    0,  -v[2],  v[1],
       v[2],      0, -v[0],
      -v[1],   v[0],     0;

  return S;
}

namespace Eigen {
typedef Eigen::Matrix<double,3,6> Matrix36d;
} // namespace Eigen

static Eigen::Matrix36d L(const Eigen::Vector3d& w)
{
  Eigen::Matrix36d result;
  result <<  w[0], w[1], w[2],    0,    0,    0,
                0, w[0],    0, w[1], w[2],    0,
                0,    0, w[0],    0, w[1], w[2];

  return result;
}

Eigen::Matrix0610d SystemCalculator::computeA(size_t index) const
{
  using namespace dart::dynamics;
  const BodyNode* bn = mRobot->getBodyNode(index);
  if(!bn)
  {
    std::cerr << "out-of-bounds index: " << index << " (max " << mRobot->getNumBodyNodes() << ")"
              << std::endl;
    return Eigen::Matrix0610d::Zero();
  }

  Eigen::Matrix0610d A;

  const Eigen::Vector6d& a = bn->getBodyAcceleration();
  const Eigen::Vector3d& d = a.block<3,1>(3,0);
  const Eigen::Vector3d& w_dot = a.block<3,1>(0,0);

  const Eigen::Vector6d& V = bn->getBodyVelocity();
  const Eigen::Vector3d& w = V.block<3,1>(0,0);

  const Eigen::Matrix3d& S_w = skew(w);
  A.block<3,1>(0,0).setZero();
  A.block<3,1>(3,0) = d;
  A.block<3,3>(0,3) = -skew(d);
  A.block<3,3>(3,3) = skew(w_dot) + S_w*S_w;
  A.block<3,6>(0,6) = L(w_dot) + S_w*L(w);
  A.block<3,6>(3,6).setZero();

  return A;
}


Eigen::Matrix6d SystemCalculator::computeSpatialTransform(size_t from, size_t to) const
{
  using namespace dart::dynamics;
  const BodyNode* bnFrom = mRobot->getBodyNode(from);
  const BodyNode* bnTo = mRobot->getBodyNode(to);
  if(!bnFrom || !bnTo)
  {
    std::cerr << "out-of-bounds index: " << from << ", " << to << " (max "
              << mRobot->getNumBodyNodes() << ")" << std::endl;
    return Eigen::Matrix6d::Identity();
  }

  const Eigen::Isometry3d& tfFrom = bnFrom->getTransform();
  const Eigen::Isometry3d& tfTo = bnTo->getTransform();
  const Eigen::Isometry3d& tf = tfTo.inverse()*tfFrom;

  Eigen::Matrix6d X;
  X.block<3,3>(0,0) = tf.linear();
  X.block<3,3>(3,0) = tf.linear()*skew(tf.translation()).transpose();
  X.block<3,3>(0,3).setZero();
  X.block<3,3>(3,3) = tf.linear();

  return X;
}
