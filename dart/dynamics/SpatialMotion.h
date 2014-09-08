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

#ifndef DART_DYNAMICS_SPATIALMOTION_H_
#define DART_DYNAMICS_SPATIALMOTION_H_

#include <Eigen/Dense>

#include "dart/math/MathTypes.h"

namespace dart {
namespace dynamics {

class Inertia;
class SpatialForce;

/// SpatialForce class represents linear velocity and angular velocity
///
/// The mathmematical concepts and notations of special force is came from
/// spatial vector algebra. This is also called as se(3), where se(3) is Lie
/// algebra of SE(3).
class SpatialMotion
{
public:
  /// Constructor
  SpatialMotion();

  /// Constructor
  /// \param[in] _linVel Linear velocity
  /// \param[in] _angVel Angular velocity
  SpatialMotion(const Eigen::Vector3d& _linVel,
                const Eigen::Vector3d& _angVel);

  /// Destructor
  virtual ~SpatialMotion();

  //----------------------------------------------------------------------------
  // Setter/getter for linear force and moment
  //----------------------------------------------------------------------------

  /// Set linear velocity
  void setLinearVelocity(const Eigen::Vector3d& _linVel);

  /// Set angular velocity
  void setAngularVelocity(const Eigen::Vector3d& _angVel);

  /// Return linear velocity
  const Eigen::Vector3d& getLinear() const;

  /// Return linear velocity
  Eigen::Vector3d& getLinear();

  /// Return angular velocity
  const Eigen::Vector3d& getAngular() const;

  /// Return angular velocity
  Eigen::Vector3d& getAngular();

  //----------------------------------------------------------------------------
  // Operators
  //----------------------------------------------------------------------------

  // TODO(JS): =, +, -, *

  /// Return the inner product with spatial force
  double inner(const SpatialForce& _force) const;

  ///
  SpatialForce ad(const Inertia& _G, const SpatialMotion& _vel);

private:
  /// Linear velocity
  Eigen::Vector3d mLinearVelocity;

  /// Angular velocity
  Eigen::Vector3d mAngularVelocity;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_SPATIALMOTION_H_
