/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/dynamics/Inertia.h"

#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/math/Geometry.h"

namespace dart {
namespace dynamics {

//==============================================================================
Inertia::Inertia()
  : mMomentOfInertia(Eigen::Matrix3d::Identity())
{

}

//==============================================================================
Inertia::Inertia(double _mass,
                 double _Ixx, double _Iyy, double _Izz,
                 double _Ixy, double _Ixz, double _Iyz)
{

}

//==============================================================================
Inertia::~Inertia()
{

}

//==============================================================================
void Inertia::setMass(double _mass)
{
  assert(_mass >= 0.0);

  mMass = _mass;
}

//==============================================================================
double Inertia::getMass() const
{
  return mMass;
}

//==============================================================================
void Inertia::setIxx(double _Ixx)
{
  assert(_Ixx >= 0.0);

  mMomentOfInertia(0, 0) = _Ixx;
}

//==============================================================================
void Inertia::setIyy(double _Iyy)
{
  assert(_Iyy >= 0.0);

  mMomentOfInertia(1, 1) = _Iyy;
}

//==============================================================================
void Inertia::setIzz(double _Izz)
{
  assert(_Izz >= 0.0);

  mMomentOfInertia(2, 2) = _Izz;
}

//==============================================================================
void Inertia::setIxy(double _Ixy)
{
  mMomentOfInertia(0, 1) = mMomentOfInertia(1, 0) = _Ixy;
}

//==============================================================================
void Inertia::setIxz(double _Ixz)
{
  mMomentOfInertia(0, 2) = mMomentOfInertia(2, 0) = _Ixz;
}

//==============================================================================
void Inertia::setIyz(double _Iyz)
{
  mMomentOfInertia(1, 2) = mMomentOfInertia(2, 1) = _Iyz;
}

//==============================================================================
double Inertia::getIxx() const
{
  return mMomentOfInertia(0, 0);
}

//==============================================================================
double Inertia::getIyy() const
{
  return mMomentOfInertia(1, 1);
}

//==============================================================================
double Inertia::getIzz() const
{
  return mMomentOfInertia(2, 2);
}

//==============================================================================
double Inertia::getIxy() const
{
  assert(mMomentOfInertia(0, 1) == mMomentOfInertia(1, 0));

  return mMomentOfInertia(0, 1);
}

//==============================================================================
double Inertia::getIxz() const
{
  assert(mMomentOfInertia(0, 2) == mMomentOfInertia(2, 0));

  return mMomentOfInertia(0, 2);
}

//==============================================================================
double Inertia::getIyz() const
{
  assert(mMomentOfInertia(1, 2) == mMomentOfInertia(2, 1));

  return mMomentOfInertia(1, 2);
}

//==============================================================================
void Inertia::setPrincipalMoments(double _Ixx, double _Iyy, double _Izz)
{
  setIxx(_Ixx);
  setIyy(_Iyy);
  setIzz(_Izz);
}

//==============================================================================
void Inertia::setPrincipalMoments(const Eigen::Vector3d& _principalMoments)
{
  setIxx(_principalMoments[0]);
  setIyy(_principalMoments[1]);
  setIzz(_principalMoments[2]);
}

//==============================================================================
void Inertia::setProducts(double _Ixy, double _Ixz, double _Iyz)
{
  setIxy(_Ixy);
  setIxz(_Ixz);
  setIyz(_Iyz);
}

//==============================================================================
void Inertia::setProducts(const Eigen::Vector3d& _products)
{
  setIxy(_products[0]);
  setIxz(_products[1]);
  setIyz(_products[2]);
}

//==============================================================================
Eigen::Vector3d Inertia::getPrincipalMoments() const
{
  return Eigen::Vector3d(getIxx(), getIyy(), getIzz());
}

//==============================================================================
Eigen::Vector3d Inertia::getProducts() const
{
  return Eigen::Vector3d(getIxy(), getIxz(), getIyz());
}

//==============================================================================
void Inertia::setMomentOfInertia(const Eigen::Matrix3d& _moi)
{
  // Validity check
  if(!isValidMomentOfInertia(_moi))
  {
    dterr << "Invalid moment of inertia" << std::endl;
    return;
  }

  mMomentOfInertia = _moi;
}

//==============================================================================
const Eigen::Matrix3d& Inertia::getMomentOfInertia() const
{
  return mMomentOfInertia;
}

//==============================================================================
void Inertia::setCenterOfMass(const Eigen::Vector3d& _com)
{
  mFrame.translation() = _com;
}

//==============================================================================
const Eigen::Vector3d& Inertia::getCenterOfMass() const
{
  const Eigen::Vector3d& com = mFrame.translation();

  return com;
}

//==============================================================================
void Inertia::setFrame(const Eigen::Isometry3d& _frame)
{
  mFrame = _frame;
}

//==============================================================================
const Eigen::Isometry3d& Inertia::getFrame() const
{
  return mFrame;
}

//==============================================================================
Eigen::Matrix6d Inertia::getTensor() const
{
  // TODO(JS): Need optimize

  Eigen::Matrix6d tensor(Eigen::Matrix6d::Zero());

  const double& Ixx = mMomentOfInertia(0, 0);
  const double& Iyy = mMomentOfInertia(1, 1);
  const double& Izz = mMomentOfInertia(2, 2);

  const double& Ixy = mMomentOfInertia(0, 1);
  const double& Ixz = mMomentOfInertia(0, 2);
  const double& Iyz = mMomentOfInertia(1, 2);

  tensor(0, 0) = Ixx;
  tensor(1, 1) = Iyy;
  tensor(2, 2) = Izz;
  tensor(0, 1) = tensor(1, 0) = Ixy;
  tensor(0, 2) = tensor(2, 0) = Ixz;
  tensor(1, 2) = tensor(2, 1) = Iyz;
  tensor(3, 3) = mMass;
  tensor(4, 4) = mMass;
  tensor(5, 5) = mMass;

  return math::transformInertia(mFrame.inverse(), tensor);
}

//==============================================================================
const Inertia& Inertia::operator=(const Inertia& _I)
{
  if (this == &_I) return *this;  // self assignment

  mMass            = _I.mMass;
  mMomentOfInertia = _I.mMomentOfInertia;
  mFrame           = _I.mFrame;

  return *this;
}

//==============================================================================
SpatialForce Inertia::operator*(const SpatialMotion& _vel) const
{
  SpatialForce result;

  const Eigen::Matrix3d& R = mFrame.linear();
  const Eigen::Vector3d& p = mFrame.translation();

  const Eigen::Vector3d& w = _vel.getAngular();
  const Eigen::Vector3d& v = _vel.getAngular();

  // Angular part: R Ic R^T w + m [p]^T [p] w + m [p] v
  result.getMoment().noalias()  = R*(mMomentOfInertia*(R.transpose()*w));
  result.getMoment().noalias() -= mMass*p.cross(p.cross(w));
  result.getMoment().noalias() += mMass*p.cross(v);

  // Linear part: m [p]^T w + m v
  result.getLinearForce().noalias()  = mMass*w.cross(p);
  result.getLinearForce().noalias() += mMass*v;

  return result;
}

//==============================================================================
Inertia Inertia::getRandom()
{
  using namespace Eigen;

  Inertia result;

  result.setMass(math::random(0.0, 100.0));

  result.setIxx(math::random(0.0, 100.0));
  result.setIyy(math::random(0.0, 100.0));
  result.setIzz(math::random(0.0, 100.0));

  result.setIxy(math::random(-100.0, 100.0));
  result.setIxz(math::random(-100.0, 100.0));
  result.setIyz(math::random(-100.0, 100.0));

  const double&        angle = math::random(-DART_PI, DART_PI);
  const Vector3d&      axis  = Vector3d::Random();
  const Vector3d&      trans = Vector3d::Random();
  const AngleAxisd&    R     = AngleAxisd(angle, axis);
  const Translation3d& p     = Translation3d(trans);
  const Isometry3d&    T     = R * p;

  result.setFrame(T);

  return result;
}

//==============================================================================
double Inertia::getKineticEnergy(const SpatialMotion& _vel) const
{
  return 0.5 * _vel.inner((*this)*_vel);
}

//==============================================================================
bool Inertia::isValidMomentOfInertia(const Eigen::Matrix3d& _moi)
{
  //----------------------------------------------
  // Check symmetricity of products of inertia
  //----------------------------------------------

  if (_moi(0, 1) != _moi(1, 0))
    return false;

  if (_moi(1, 2) != _moi(2, 1))
    return false;

  if (_moi(2, 0) != _moi(0, 2))
    return false;

  //-----------------------------------------------------
  // Check positivity of principal moments of inertia
  //-----------------------------------------------------

  if (_moi(0, 0) < 0.0)
    return false;

  if (_moi(1, 1) < 0.0)
    return false;

  if (_moi(2, 2) < 0.0)
    return false;


  return true;
}

}  // namespace dynamics
}  // namespace dart
