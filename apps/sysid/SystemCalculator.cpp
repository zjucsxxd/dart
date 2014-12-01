#include "SystemCalculator.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"

#include <iomanip>

#include <Eigen/SVD>

static std::string convert_to_param(size_t index, const std::string& name)
{
  size_t param = index%10;

  std::stringstream str;
  str << name << "_";
  switch(param)
  {
    case 0: str << "mass"; break;
    case 1: str << "mx"; break;
    case 2: str << "my"; break;
    case 3: str << "mz"; break;
    case 4: str << "Ixx"; break;
    case 5: str << "Ixy"; break;
    case 6: str << "Ixz"; break;
    case 7: str << "Iyy"; break;
    case 8: str << "Iyz"; break;
    case 9: str << "Izz"; break;
  }

  return str.str();
}

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

  const Eigen::Vector6d& V = bn->getBodyVelocity();
  const Eigen::Vector3d& v = V.block<3,1>(3,0);
  const Eigen::Vector3d& w = V.block<3,1>(0,0);

  const Eigen::Vector6d& a = bn->getBodyAcceleration();
  const Eigen::Vector3d& v_dot = a.block<3,1>(3,0);
  const Eigen::Vector3d& w_dot = a.block<3,1>(0,0);
  const Eigen::Vector3d& d = v_dot + w.cross(v);

  const Eigen::Matrix3d& S_w = skew(w);
  A.block<3,1>(0,0).setZero();
  A.block<3,1>(3,0) = d;
  A.block<3,3>(0,1) = -skew(d);
  A.block<3,3>(3,1) = skew(w_dot) + S_w*S_w;
  A.block<3,6>(0,4) = L(w_dot) + S_w*L(w);
  A.block<3,6>(3,4).setZero();

  return A;
}


Eigen::Matrix6d SystemCalculator::computeSpatialForceTransform(size_t B, size_t A) const
{
  using namespace dart::dynamics;
  const BodyNode* bnA = mRobot->getBodyNode(A);
  const BodyNode* bnB = mRobot->getBodyNode(B);
  if(!bnA || !bnB)
  {
    std::cerr << "out-of-bounds index: " << A << ", " << B << " (max "
              << mRobot->getNumBodyNodes() << ")" << std::endl;
    return Eigen::Matrix6d::Identity();
  }

  const Eigen::Isometry3d& tfA = bnA->getTransform();
  const Eigen::Isometry3d& tfB = bnB->getTransform();
  const Eigen::Isometry3d& tf = tfB.inverse()*tfA;

  Eigen::Matrix6d X;
  X.block<3,3>(0,0) = tf.rotation();
  X.block<3,3>(0,3).setZero();
  X.block<3,3>(3,0) = skew(tf.translation())*tf.rotation();
  X.block<3,3>(3,3) = tf.rotation();
  Eigen::Matrix6d Xf = X.inverse().transpose();

//  Eigen::Matrix6d Xf;
//  Xf.block<3,3>(0,0) = tf.rotation();
//  Xf.block<3,3>(0,3) = skew(tf.translation())*tf.rotation();
//  Xf.block<3,3>(3,0).setZero();
//  Xf.block<3,3>(3,3) = tf.rotation();

  return Xf;
}

static void print_parameters(const Eigen::VectorXd& parameters, const dart::dynamics::Skeleton* robot)
{
  size_t nw = 4;
  for(size_t i=0; i<robot->getNumBodyNodes(); ++i)
    nw = std::max(nw, robot->getBodyNode(i)->getName().size());
  nw += 3;

  size_t w = 10;
  std::cout.width(nw);
  std::cout << "name" << std::setw(w)
            << "mass" << std::setw(w)
            << "mx" << std::setw(w)
            << "my" << std::setw(w)
            << "mz" << std::setw(w)
            << "Ixx" << std::setw(w)
            << "Ixy" << std::setw(w)
            << "Ixz" << std::setw(w)
            << "Iyy" << std::setw(w)
            << "Iyz" << std::setw(w)
            << "Izz" << std::endl;
  for(size_t i=0; i<(size_t)parameters.size()/10; ++i)
  {
    std::cout << std::setw(nw) << robot->getBodyNode(i)->getName();
    for(size_t p=0; p<10; ++p)
    {
      std::cout << std::setw(w) << std::setprecision(3) << parameters[10*i+p];
    }
    std::cout << "\n";
  }
  std::cout << std::flush;

}

static Eigen::VectorXd process_using_svd(const Eigen::MatrixXd& A, const Eigen::VectorXd& f,
                                         const dart::dynamics::Skeleton* robot)
{
  size_t C = A.cols();
  Eigen::VectorXd parameters(C); parameters.setZero();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

//  std::cout << "SVD constructed" << std::endl;

  double cutoff = 0.001*svd.singularValues()[0];

  size_t count = 0;
  for(int i=0; i<svd.singularValues().size(); ++i)
  {
    if(svd.singularValues()[i] >= cutoff)
      ++count;
  }

  std::cout << "We have " << count << " non-zero parameters out of a possible " << C << std::endl;

//  std::cout << "U: " << svd.matrixU().cols() << "\n"
//            << "V: " << svd.matrixV().cols() << std::endl;

//  for(size_t i=0; i<C; ++i)
//  {
//    const Eigen::VectorXd& V = svd.matrixV().col(i);
//    std::cout << std::setw(8) << svd.singularValues()[i] << ": ";
//    for(size_t j=0; j<C; ++j)
//    {
//      if(fabs(V[j]) > 1e-8)
//      {
//        std::cout << convert_to_param(j, robot->getBodyNode(j/10)->getName()) << " \t";
//      }
//    }
//    std::cout << std::endl;
//  }

  for(size_t j=0; j<C; ++j)
  {
    Eigen::VectorXd uj = svd.matrixU().col(j);
    Eigen::VectorXd vj = svd.matrixV().col(j);
    double mu = svd.singularValues()[j];
    if(mu < cutoff)
      continue;

    parameters += vj*(uj.transpose()*f)/mu;
  }

  for(size_t p=0; p<C; ++p)
    if(fabs(parameters[p]) < 1e-8)
      parameters[p] = 0;

  return parameters;
}

static Eigen::VectorXd process_using_dls(const Eigen::MatrixXd& A0, const Eigen::VectorXd& f0,
                                         const Eigen::VectorXd& actual, double confidence=1)
{
  size_t nP = A0.rows();
  size_t C = A0.cols();
  Eigen::MatrixXd A(nP+C, C);
  A.block(0,0,nP,C) = A0;
  A.block(nP,0,C,C) = Eigen::MatrixXd::Identity(C,C)*confidence;
  Eigen::VectorXd f(nP+C);
  f.setZero();
  f.block(0,0,nP,1) = f0 - A0*actual;

  Eigen::VectorXd parameters = (A.transpose()*A).inverse()*A.transpose()*f+actual;
  for(int p=0; p<parameters.size(); ++p)
    if(fabs(parameters[p]) < 1e-8)
      parameters[p] = 0;

  return parameters;
}

Eigen::VectorXd SystemCalculator::postProcessing(dart::dynamics::Skeleton* robot,
                                                 const std::vector<Eigen::VectorXd>& forces,
                                                 const std::vector<Eigen::MatrixXd>& dynamics)
{
//  std::cout << "Creating data structures: " << forces.size() << ", " << dynamics.size() << std::endl;
  size_t n = dynamics[0].rows();
//  std::cout << "n: " << n << std::endl;
  size_t P = dynamics.size();
  size_t C = dynamics[0].cols();
  Eigen::MatrixXd A0(n*P,C);
  Eigen::VectorXd f0(n*P);

//  std::cout << "Filling data structures" << std::endl;
  for(size_t i=0; i<P; ++i)
  {
    A0.block(i*n,0,n,C) = dynamics[i];
    f0.block(i*n,0,n,1) = forces[i];
  }
//  std::cout << "Data filled" << std::endl;


  Eigen::VectorXd actual(C);
  for(size_t i=0; i<robot->getNumBodyNodes(); ++i)
  {
    dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
    actual[i*10] = bn->getMass();
    actual.block<3,1>(i*10+1,0) = bn->getMass()*bn->getLocalCOM();
//    bn->getMomentOfInertia(actual[i*10+4], actual[i*10+7], actual[i*10+9],
//                           actual[i*10+5], actual[i*10+6], actual[i*10+8]);
    const Eigen::Matrix6d& I = bn->getSpatialInertia();
    actual[i*10+4] = I(0,0); actual[i*10+7] = I(1,1); actual[i*10+9] = I(2,2);
    actual[i*10+5] = I(0,1); actual[i*10+6] = I(0,2); actual[i*10+8] = I(1,2);
  }


//  std::cout << "\nActual Parameters:\n";
//  print_parameters(actual, robot);

//  std::cout << "\nEstimated Parameters (Raw SVD):\n";
//  print_parameters(process_using_svd(A0, f0, robot), robot);

//  std::cout << "\nDLS Confidence=0.1\n";
//  print_parameters(process_using_dls(A0, f0, actual, 0.1), robot);

//  std::cout << "\nConfidence=1\n";
//  print_parameters(process_using_dls(A0, f0, actual, 1), robot);

//  std::cout << "\nConfidence=100\n";
  Eigen::VectorXd parameters = process_using_dls(A0, f0, actual, 100);
//  print_parameters(parameters, robot);

  double diff = (parameters-actual).dot(parameters-actual);
  diff = sqrt(diff);

  std::cout << "Residual error is: " << diff/actual.norm()*100 << "%" << std::endl;

  std::cout << std::flush;

//  std::cout << "\nForces:\n" << f0.transpose() << std::endl;
  return parameters;
}
