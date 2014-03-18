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

#include "dart/config.h"
#ifdef HAVE_SDFORMAT

#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/SoftSkeleton.h"
#include "dart/utils/Paths.h"
#include "dart/simulation/World.h"
#include "dart/simulation/SoftWorld.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/SoftParser.h"

#include "dart/utils/sdf/SdfLoader.h"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace utils;

TEST(SdfLoader, Empty)
{
  SoftWorld* world = SdfLoader::loadWorld(DART_DATA_PATH"sdf/empty.world");
  EXPECT_TRUE(world != NULL);

  delete world;
}

TEST(SdfLoader, DoublePendulum)
{
  SoftWorld* world
      = SdfLoader::loadWorld(DART_DATA_PATH"sdf/double_pendulum.world");
  EXPECT_TRUE(world != NULL);
//  EXPECT_EQ(world->getNumSkeletons(), 2);

//  Skeleton* skel0 = world->getSkeleton("ground_plane");
//  EXPECT_TRUE(skel0 != NULL);
//  EXPECT_EQ(skel0->getNumBodyNodes(), 1);

//  Skeleton* skel1 = world->getSkeleton("double_pendulum_with_base");
//  EXPECT_TRUE(skel1 != NULL);
//  EXPECT_EQ(skel1->getNumBodyNodes(), 1);

  delete world;
}

#endif  // HAVE_SDFORMAT

/******************************************************************************/
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
