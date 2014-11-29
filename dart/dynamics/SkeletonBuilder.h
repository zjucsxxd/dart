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

#ifndef DART_DYNAMICS_SKELETONBUILDER_H_
#define DART_DYNAMICS_SKELETONBUILDER_H_

#include <vector>

#include "dart/common/Console.h"
#include "dart/common/Factory.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace dynamics {

struct BodyNodePair
{
  Joint* joint;
  std::string bodyNodeName1;
  std::string bodyNodeName2;
  BodyNode* bodyNode1;
  BodyNode* bodyNode2;
};

class SkeletonBuilder
{
public:
  /// \brief Constructor with Skeleton
  /// \param [in] _skel Building %Skeleton. Note that _skel shouldn't be nullptr.
  explicit SkeletonBuilder(dynamics::Skeleton* _skel);

  /// \brief Destructor
  virtual ~SkeletonBuilder();

  /// \brief Create a BodyNode
  virtual BodyNode* createBodyNode();

//  virtual Joint* createJoint(const std::string& _type,
//                             const std::string& _bodyNodeName1,
//                             const std::string& _bodyNodeName2)
//  {
//    Joint* newJoint = common::Factory<Joint>::createObject(_type);

//    if (newJoint == nullptr)
//    {
//      dterr << "Joint type [" << _type << "] is invalid." << std::endl;
//      return nullptr;
//    }

//    BodyNodePair pair;
//    pair.bodyNodeName1 = _bodyNodeName1;
//    pair.bodyNodeName2 = _bodyNodeName2;

//    mJointPool.push_back(pair);

//    return newJoint;
//  }

  virtual Joint* createJoint(const std::string& _type,
                             BodyNode* _bodyNode1,
                             BodyNode* _bodyNode2)
  {
    Joint* newJoint = common::Factory<Joint>::createObject(_type);

    if (newJoint == nullptr)
    {
      dterr << "Joint type [" << _type << "] is invalid." << std::endl;
      return nullptr;
    }

    BodyNodePair pair;
    pair.bodyNode1 = _bodyNode1;
    pair.bodyNode2 = _bodyNode2;

    mJointPool.push_back(pair);

    return newJoint;
  }

  /// \brief Build skeleton
  /// \return True if the skeleton is valid, false otherwise
  virtual bool build();

protected:
  Skeleton* mSkeleton;

  std::vector<BodyNode*> mBodyNodePool;
  std::vector<BodyNodePair> mJointPool;

private:

};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_SKELETONBUILDER_H_
