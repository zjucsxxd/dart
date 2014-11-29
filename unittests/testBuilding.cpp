/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
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

#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.h"

#include "dart/math/Geometry.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/SkeletonBuilder.h"
#include "dart/simulation/World.h"

using namespace Eigen;

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
//using namespace utils;

//==============================================================================
TEST(Building, Basic)
{
  //--------------------------------------------------------------------------
  //
  //--------------------------------------------------------------------------
  // Bodies
  BodyNode* body1 = new BodyNode;
  BodyNode* body2 = new BodyNode;
  BodyNode* body3 = new BodyNode;

  // Joints
  RevoluteJoint* joint1 = new RevoluteJoint;
  RevoluteJoint* joint2 = new RevoluteJoint;
  RevoluteJoint* joint3 = new RevoluteJoint;

  // Skeletons
  Skeleton* skel1 = new Skeleton;

  // World
  World* world = new World;

  //--------------------------------------------------------------------------
  //
  //--------------------------------------------------------------------------
  // Bodies
  body1->addChildBodyNode(body2);
  body2->addChildBodyNode(body3);

  body1->setParentJoint(joint1);
  body2->setParentJoint(joint2);
  body3->setParentJoint(joint3);

  // Joints
  joint1->setTransformFromParentBodyNode(Isometry3d::Identity());
  joint1->setTransformFromChildBodyNode(Isometry3d::Identity());
  joint1->setAxis(Vector3d(1.0, 0.0, 0.0));

  joint2->setTransformFromParentBodyNode(Isometry3d::Identity());
  joint2->setTransformFromChildBodyNode(Isometry3d::Identity());
  joint2->setAxis(Vector3d(1.0, 0.0, 0.0));

  joint3->setTransformFromParentBodyNode(Isometry3d::Identity());
  joint3->setTransformFromChildBodyNode(Isometry3d::Identity());
  joint3->setAxis(Vector3d(1.0, 0.0, 0.0));

  // Skeleton
  skel1->addBodyNode(body1);
  skel1->addBodyNode(body2);
  skel1->addBodyNode(body3);

  // World
  world->addSkeleton(skel1);

  //--------------------------------------------------------------------------
  //
  //--------------------------------------------------------------------------
  EXPECT_TRUE(body1->getParentBodyNode() == NULL);
  EXPECT_TRUE(body1->getNumChildBodyNodes() == 1);
  EXPECT_TRUE(body1->getChildBodyNode(0) == body2);

  EXPECT_TRUE(body2->getParentBodyNode() == body1);
  EXPECT_TRUE(body2->getNumChildBodyNodes() == 1);
  EXPECT_TRUE(body2->getChildBodyNode(0) == body3);

  EXPECT_TRUE(body3->getParentBodyNode() == body2);
  EXPECT_TRUE(body3->getNumChildBodyNodes() == 0);
  //EXPECT_TRUE(body3.getChildBodyNode(0) == NULL);

  EXPECT_TRUE(skel1->getNumBodyNodes() == 3);
  EXPECT_TRUE(skel1->getNumDofs() == 3);

  EXPECT_TRUE(world->getNumSkeletons() == 1);

  int nSteps = 20;
  for (int i = 0; i < nSteps; ++i)
    world->step();

  delete world;
}

//==============================================================================
TEST(Building, Tree)
{
  Skeleton* skel = new Skeleton();

  skel->beginAssembly();

  BodyNode* body1 = skel->createRootBodyNode("RevoluteJoint");
  BodyNode* body2 = body1->createChildBodyNode("BallJoint");
  BodyNode* body3 = skel->createBodyNode(body2, "UniversalJoint");

  Joint* joint1 = body1->getParentJoint();
  joint1->as<RevoluteJoint>()->setAxis(Vector3d::UnitX());
  joint1->setPositionFromChildBodyNode(Vector3d(0, 0, 1));

  Joint* joint2 = body2->getParentJoint();

  Joint* joint3 = body3->getParentJoint();
  joint3->as<UniversalJoint>()->setAxis1(Vector3d::UnitX());
  joint3->as<UniversalJoint>()->setAxis2(Vector3d::UnitY());

  skel->endAssembly();

  EXPECT_TRUE(body1 != nullptr);
  EXPECT_TRUE(body2 != nullptr);
  EXPECT_TRUE(body3 != nullptr);
  EXPECT_TRUE(joint1 != nullptr);
  EXPECT_TRUE(joint2 != nullptr);
  EXPECT_TRUE(joint3 != nullptr);

  delete skel;
}

//==============================================================================
TEST(Building, TreeUsingBuilder)
{
  Skeleton* skel = new Skeleton();
  SkeletonBuilder* builder = new SkeletonBuilder(skel);

  BodyNode* body1 = builder->createBodyNode();
  BodyNode* body2 = builder->createBodyNode();

  Joint* joint1 = builder->createJoint("RevoluteJoint", nullptr, body2);
  Joint* joint2 = builder->createJoint("BallJoint", body1, body2);

  EXPECT_TRUE(body1 != nullptr);
  EXPECT_TRUE(body2 != nullptr);
  EXPECT_TRUE(joint1 != nullptr);
  EXPECT_TRUE(joint2 != nullptr);

  builder->build();

  delete builder;
  delete skel;
}

//==============================================================================
TEST(Building, ClosedLoop)
{

}

//==============================================================================
TEST(Building, ClosedLoopUsingBuilder)
{

}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
