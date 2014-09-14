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

#ifndef DART_DYNAMICS_INERTIA_H_
#define DART_DYNAMICS_INERTIA_H_

#include <Eigen/Dense>

#include "dart/math/MathTypes.h"
#include "dart/dynamics/SpatialMotion.h"
#include "dart/dynamics/SpatialForce.h"

namespace dart {
namespace dynamics {

/// Inertia is a class for inertial information about a link
///
/// Inertia class has mass, moment of inertia, and inertia frame expressed in
/// body frame (transformation matrix from body frame to the inertia frame). The
/// inertia frame is a frame that the mass and moment of inertia are specified.
class Inertia
{
public:
  /// Constructor
  Inertia();

  /// Constructor
  /// \param[in] _mass Mass in kg
  /// \param[in] _Ixx X component of the moment of inertia
  /// \param[in] _Iyy Y component of the moment of inertia
  /// \param[in] _Izz Z component of the moment of inertia
  /// \param[in] _Ixy XY component of the moment of inertia
  /// \param[in] _Ixz XZ component of the moment of inertia
  /// \param[in] _Iyz YZ component of the moment of inertia
  Inertia(double _mass,
          double _Ixx = 1.0, double _Iyy = 1.0, double _Izz = 1.0,
          double _Ixy = 0.0, double _Ixz = 0.0, double _Iyz = 0.0);

  /// Destructor
  virtual ~Inertia();

  //----------------------------------------------------------------------------
  // Mass
  //----------------------------------------------------------------------------

  /// Set mass
  void setMass(double _mass);

  /// Return mass
  double getMass() const;

  //----------------------------------------------------------------------------
  // Moment of inertia
  //----------------------------------------------------------------------------

  /// Set the X component of the moment of inertia
  /// \param[in] _Ixx X component of the moment of inertia
  void setIxx(double _Ixx);

  /// Set the Y component of the moment of inertia
  /// \param[in] _Iyy Y component of the moment of inertia
  void setIyy(double _Iyy);

  /// Set the Z component of the moment of inertia
  /// \param[in] _Izz Z component of the moment of inertia
  void setIzz(double _Izz);

  /// Set the XY component of the moment of inertia
  /// \param[in] _Ixy XY component of the moment of inertia
  void setIxy(double _Ixy);

  /// Set the XZ component of the moment of inertia
  /// \param[in] _Ixz XZ component of the moment of inertia
  void setIxz(double _Ixz);

  /// Set the YZ component of the moment of inertia
  /// \param[in] _Iyz YZ component of the moment of inertia
  void setIyz(double _Iyz);

  /// Return the X component of the moment of inertia
  /// \return X component of the moment of inertia
  double getIxx() const;

  /// Return the Y component of the moment of inertia
  /// \return Y component of the moment of inertia
  double getIyy() const;

  /// Return the Z component of the moment of inertia
  /// \return Z component of the moment of inertia
  double getIzz() const;

  /// Return the XY component of the moment of inertia
  /// \return XY component of the moment of inertia
  double getIxy() const;

  /// Return the XZ component of the moment of inertia
  /// \return XZ component of the moment of inertia
  double getIxz() const;

  /// Return the YZ component of the moment of inertia
  /// \return YZ component of the moment of inertia
  double getIyz() const;

  /// Set the principal moments of inertia
  /// \param[in] _Ixx X component of the moment of inertia
  /// \param[in] _Iyy Y component of the moment of inertia
  /// \param[in] _Izz Z component of the moment of inertia
  void setPrincipalMoments(double _Ixx, double _Iyy, double _Izz);

  /// Set the principal moments of inertia
  /// \param[in] _princialMoments Principal moments of inertia
  void setPrincipalMoments(const Eigen::Vector3d& _principalMoments);

  /// Set the products of inertia
  /// \param[in] _Ixy XY component of the moment of inertia
  /// \param[in] _Ixz XZ component of the moment of inertia
  /// \param[in] _Iyz YZ component of the moment of inertia
  void setProducts(double _Ixy, double _Ixz, double _Iyz);

  /// Set the products of inertia
  /// \param[in] _products Products of inertia
  void setProducts(const Eigen::Vector3d& _products);

  /// Return the principal moments of inertia as 3x1 vector
  /// \return Princial moments of inertia
  Eigen::Vector3d getPrincipalMoments() const;

  /// Return the products of inertia as 3x1 vector
  /// \return Products of inertia
  Eigen::Vector3d getProducts() const;

  /// Set the moment of inertia as 3x3 matrix
  /// \param[in] _moi Moment of inertia
  void setMomentOfInertia(const Eigen::Matrix3d& _moi);

  /// Return the moment of inertia as 3x3 matrix
  const Eigen::Matrix3d& getMomentOfInertia() const;

  //----------------------------------------------------------------------------
  // Inertia frame
  //----------------------------------------------------------------------------

  /// Set the center of mass
  /// \param[in] _com Center of mass
  void setCenterOfMass(const Eigen::Vector3d& _com);

  /// Return the center of mass
  /// \return Center of mass
  const Eigen::Vector3d& getCenterOfMass() const;

  /// Set the transformation from body frame to the frame that the mass and
  /// moment of inertia are specified
  /// \param[in] _frame Inertia frame
  void setFrame(const Eigen::Isometry3d& _frame);

  /// Return the transformation from body frame to the frame that the mass and
  /// moment of inertia are specified
  /// \return _frame Inertia frame
  const Eigen::Isometry3d& getFrame() const;

  //----------------------------------------------------------------------------
  // Inertia tensor
  //----------------------------------------------------------------------------

  /// Return the inertia tensor expressed at the body frame as 6x6 matrix
  Eigen::Matrix6d getTensor() const;

  //----------------------------------------------------------------------------
  // Operators
  //----------------------------------------------------------------------------

  /// Assignment operator
  const Inertia& operator=(const Inertia& _I);

  /// Multiplication operator with inertia and spatial velocity that results in
  /// spatial force
  /// \param[in] _vel Spatial velocity
  /// \return Spatial force
  SpatialForce operator*(const SpatialMotion& _vel) const;

  // TODO(JS): Add operators such as +, -

  //----------------------------------------------------------------------------
  // Utilities
  //----------------------------------------------------------------------------

  /// Return random inertia
  static Inertia getRandom();

  /// Set random inertia
  void setRandom();

  /// Return kinetic energy
  double getKineticEnergy(const SpatialMotion& _vel) const;

private:
  /// Return true if _moi is valid moment of inertia
  bool isValidMomentOfInertia(const Eigen::Matrix3d& _moi);

private:
  /// Mass
  double mMass;

  /// Moment of inertia
  Eigen::Matrix3d mMomentOfInertia;

  /// Transformation from body frame to the frame that the mass and moment of
  /// inertia are specified
  Eigen::Isometry3d mFrame;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_INERTIA_H_
