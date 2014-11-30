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

#include "dart/dynamics/SkeletonBuilder.h"

namespace dart {
namespace dynamics {

//==============================================================================
SkeletonBuilder::SkeletonBuilder()
{
//  assert(_skel != nullptr);
}

//==============================================================================
SkeletonBuilder::~SkeletonBuilder()
{

}

//==============================================================================
BodyNode* SkeletonBuilder::createBodyNode()
{
  BodyNode* newBodyNode = new BodyNode();

  return newBodyNode;
}

//==============================================================================
Joint* SkeletonBuilder::createJoint(const std::string& _type)
{
  Joint* newJoint = common::Factory<Joint>::createObject(_type);

  if (newJoint == nullptr)
  {
    dterr << "Joint type [" << _type << "] is invalid." << std::endl;
    return nullptr;
  }

  return newJoint;
}

//==============================================================================
TreeSpanningSkeletonBuilder::TreeSpanningSkeletonBuilder()
{

}

//==============================================================================
TreeSpanningSkeletonBuilder::~TreeSpanningSkeletonBuilder()
{

}

//==============================================================================
Skeleton* TreeSpanningSkeletonBuilder::makeSkeleton()
{
  Skeleton* newSkeleton = new Skeleton();


  return newSkeleton;
}

//==============================================================================
BodyNode* TreeSpanningSkeletonBuilder::createRootBodyNodeWithJoint(
    const std::string& _jointType)
{
  return nullptr;
}

//==============================================================================
BodyNode* TreeSpanningSkeletonBuilder::createBodyNodeWithJoint(
    BodyNode* _parentBodyNode,
    const std::string& _jointType)
{
  if (_parentBodyNode == nullptr)
  {
    // Check if this is the first BodyNode
    if (!mBodyNodes.empty())
    {
      dterr << "Parent body node should be specified except for the root."
            << std::endl;
      return nullptr;
    }
  }

  // Create joint
  Joint* newJoint = common::Factory<Joint>::createObject(_jointType);
  if (newJoint == nullptr)
  {
    dterr << "Joint type [" << _jointType << "] is not valid type."
          << std::endl;
    return nullptr;
  }

  // Create body node
  BodyNode* newBodyNode = new BodyNode();
//  newBodyNode->mSkeleton = this;
  newBodyNode->setParentJoint(newJoint);
//  mBodyNodes.push_back(newBodyNode);
  if (_parentBodyNode)
    _parentBodyNode->addChildBodyNode(newBodyNode);

//  if (!mIsAssembling)
//    assemble();

  return newBodyNode;
}

//==============================================================================
ListingSkeletonBuilder::ListingSkeletonBuilder()
{

}

//==============================================================================
ListingSkeletonBuilder::~ListingSkeletonBuilder()
{

}

//==============================================================================
Skeleton* ListingSkeletonBuilder::makeSkeleton()
{
  for (const auto& body : mBodyNodePool)
  {
    body->setParentJoint(nullptr);
    //    body->removeAllChildBodyNodes();
  }

  // TODO: need graph
//  buildGraph();
//  buildTree();

//  if (mBaseBodyNode == nullptr)
//  {}

  // TODO(JS): Implement graph and tree

//  mBodyNodes = mBodyNodePool;
//  mJoints = mJointPool;

//  for (const auto& body : mBodyNodes)
//  {
//    body->setParentJoint(nullptr);
//  }

//  for (auto& joint : mJoints)
//    joint->build();

  return nullptr;
}

//==============================================================================
BodyNode* ListingSkeletonBuilder::createBodyNode()
{
  BodyNode* newBodyNode = SkeletonBuilder::createBodyNode();



  mBodyNodePool.push_back(newBodyNode);
}

//==============================================================================
Joint* ListingSkeletonBuilder::createJoint(const std::string& _jointType,
                                              BodyNode* _bodyNode1,
                                              BodyNode* _bodyNode2)
{
  Joint* newJoint = SkeletonBuilder::createJoint(_jointType);

  if (newJoint == nullptr)
    return nullptr;

  BodyNodePair pair;
  pair.joint = newJoint;
  pair.bodyNode1 = _bodyNode1;
  pair.bodyNode2 = _bodyNode2;

  mJointPool.push_back(pair);

  return newJoint;
}


}  // namespace dynamics
}  // namespace dart

