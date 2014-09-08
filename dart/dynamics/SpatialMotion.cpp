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

#include "dart/dynamics/SpatialMotion.h"

#include "dart/dynamics/SpatialForce.h"

namespace dart {
namespace dynamics {

//==============================================================================
SpatialMotion::SpatialMotion()
  : mLinearVelocity(Eigen::Vector3d::Zero()),
    mAngularVelocity(Eigen::Vector3d::Zero())
{

}

//==============================================================================
SpatialMotion::SpatialMotion(const Eigen::Vector3d& _linVel,
                             const Eigen::Vector3d& _angVel)
  : mLinearVelocity(_linVel),
    mAngularVelocity(_angVel)
{

}

//==============================================================================
SpatialMotion::~SpatialMotion()
{

}

//==============================================================================
void SpatialMotion::setLinearVelocity(const Eigen::Vector3d& _linVel)
{
  mLinearVelocity = _linVel;
}

//==============================================================================
void SpatialMotion::setAngularVelocity(const Eigen::Vector3d& _angVel)
{
  mAngularVelocity = _angVel;
}

//==============================================================================
const Eigen::Vector3d& SpatialMotion::getLinear() const
{
  return mLinearVelocity;
}

//==============================================================================
Eigen::Vector3d& SpatialMotion::getLinear()
{
  return mLinearVelocity;
}

//==============================================================================
const Eigen::Vector3d& SpatialMotion::getAngular() const
{
  return mAngularVelocity;
}

//==============================================================================
Eigen::Vector3d& SpatialMotion::getAngular()
{
  return mAngularVelocity;
}

//==============================================================================
double SpatialMotion::inner(const SpatialForce& _force) const
{
  return mLinearVelocity.dot(_force.getLinearForce())
      + mAngularVelocity.dot(_force.getMoment());
}

//==============================================================================
SpatialForce SpatialMotion::ad(const Inertia& _G, const SpatialMotion& _vel)
{
  SpatialForce result;



  if (&_vel == this)
    int a = 10;

  return result;
}

}  // namespace dynamics
}  // namespace dart
