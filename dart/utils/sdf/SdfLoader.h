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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#ifndef DART_UTILS_SDFLOADER_H_
#define DART_UTILS_SDFLOADER_H_

#include <map>
#include <string>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <sdf/sdf.hh>

namespace dart {
namespace dynamics {
class BodyNode;
class Shape;
class Joint;
class WeldJoint;
class PrismaticJoint;
class RevoluteJoint;
class ScrewJoint;
class UniversalJoint;
class BallJoint;
class EulerXYZJoint;
class EulerJoint;
class TranslationalJoint;
class FreeJoint;
class Skeleton;
class SoftSkeleton;
}  // namespace dynamics
namespace simulation {
class World;
class SoftWorld;
}  // namespace simulation
}  // namespace dart

namespace dart {
namespace utils {

/// \class SdfLoader
class SdfLoader
{
public:
  /// \brief
  static simulation::SoftWorld* loadWorld(const std::string& _filename);

  /// \brief
  static dynamics::SoftSkeleton* loadSkeleton(const std::string& _filename);

protected:
  /// \brief
  static simulation::SoftWorld* loadWorld(sdf::ElementPtr _sdf);

  /// \brief
  static dynamics::Skeleton* loadSkeleton(sdf::ElementPtr _sdf);

  /// \brief
  static dynamics::BodyNode* loadBodyNode(sdf::ElementPtr _sdf);

  /// \brief
  static std::string findFile(const std::string& _filename);

  /// \brief
  static Eigen::Vector3d convVec3(const sdf::Vector3& _val);

  /// \brief
  static sdf::Vector3 convVec3(const Eigen::Vector3d& _val);
};

}  // namespace utils
}  // namespace dart

#endif // #ifndef DART_UTILS_SDFPARSER_H
