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

#ifndef DART_DYNAMICS_SPATIALFORCE_H_
#define DART_DYNAMICS_SPATIALFORCE_H_

#include <Eigen/Dense>

#include <dart/math/MathTypes.h>

namespace dart {
namespace dynamics {

class SpatialMotion;

/// SpatialForce class represents linear force and moment
///
/// The mathmematical concepts and notations of special force is came from
/// spatial vector algebra. This is also called as wrench, and se*(3), where
/// se*(3) is dual space of Lie algebra of SE(3).
class SpatialForce
{
public:
  /// Constructor
  SpatialForce();

  /// Constructor
  /// \param[in] _linVel Linear velocity
  /// \param[in] _angVel Angular velocity
  SpatialForce(const Eigen::Vector3d& _linForce,
               const Eigen::Vector3d& _moment);

  /// Destructor
  virtual ~SpatialForce();

  //----------------------------------------------------------------------------
  // Setter/getter for linear force and moment
  //----------------------------------------------------------------------------

  /// Set linear force
  void setLinearForce(const Eigen::Vector3d& _linForce);

  /// Set moment
  void setMoment(const Eigen::Vector3d& _moment);

  /// Return the linear force
  const Eigen::Vector3d& getLinearForce() const;

  /// Return the linear force
  Eigen::Vector3d& getLinearForce();

  /// Return the moment
  const Eigen::Vector3d& getMoment() const;

  /// Return the moment
  Eigen::Vector3d& getMoment();

  //----------------------------------------------------------------------------
  // Operators
  //----------------------------------------------------------------------------

  // TODO(JS): =, +, -, *

  /// Return the inner product with spatial motion
  double inner(const SpatialMotion& _vel) const;

private:
  /// Linear force
  Eigen::Vector3d mLinearForce;

  /// Moment
  Eigen::Vector3d mMoment;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_SpatialMotion_H_
