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

#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.h"

#include "dart/math/Geometry.h"
#include "dart/dynamics/Inertia.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;

//==============================================================================
TEST(SpatialVectorAlgebra, SpatialMotion)
{
}

//==============================================================================
TEST(SpatialVectorAlgebra, Inertia)
{
  // Mass
  double mass = math::random(0.0, 100.0);

  // Components of moment of inertia
  double Ixx = math::random(0.0, 100.0);
  double Iyy = math::random(0.0, 100.0);
  double Izz = math::random(0.0, 100.0);
  double Ixy = math::random(-100.0, 100.0);
  double Ixz = math::random(-100.0, 100.0);
  double Iyz = math::random(-100.0, 100.0);

  // Moment of inertia
  Eigen::Matrix3d moi = Eigen::Matrix3d::Zero();
  moi(0, 0) = Ixx;
  moi(1, 1) = Iyy;
  moi(2, 2) = Izz;
  moi(0, 1) = moi(1, 0) = Ixy;
  moi(0, 2) = moi(2, 0) = Ixz;
  moi(1, 2) = moi(2, 1) = Iyz;

  // Frame
  Eigen::Isometry3d T = math::expMap(Eigen::Vector6d::Random());

  // Set inertia
  dynamics::Inertia G = dynamics::Inertia::getRandom();
  G.setMass(mass);
  G.setPrincipalMoments(Ixx, Iyy, Izz);
  G.setProducts(Ixy, Ixz, Iyz);
  G.setFrame(T);

  // Inertia tensor
  Eigen::Matrix6d tensor = Eigen::Matrix6d::Zero();
  tensor.topLeftCorner<3, 3>() = moi;
  tensor.bottomRightCorner<3, 3>() = mass*Eigen::Matrix3d::Identity();
  Eigen::Matrix6d AdInvT = math::AdInvT(T);
  Eigen::Matrix6d dAdInvT = math::dAdInvT(T);
  Eigen::Matrix6d transformedTensor = dAdInvT*tensor*AdInvT;

  // Test: Mass
  EXPECT_EQ(G.getMass(), mass);

  // Test: Components of moment of inertia
  EXPECT_EQ(G.getIxx(), Ixx);
  EXPECT_EQ(G.getIyy(), Iyy);
  EXPECT_EQ(G.getIzz(), Izz);
  EXPECT_EQ(G.getIxy(), Ixy);
  EXPECT_EQ(G.getIxz(), Ixz);
  EXPECT_EQ(G.getIyz(), Iyz);

  // Test:: Moment of inertia
  EXPECT_EQ(G.getMomentOfInertia(), moi);

  // Test: Frame
  EXPECT_EQ(G.getFrame().linear(),      T.linear());
  EXPECT_EQ(G.getFrame().translation(), T.translation());

  // Test: tensor
  EXPECT_TRUE(equals(G.getTensor(), transformedTensor, 1e-12));
}

/******************************************************************************/
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
