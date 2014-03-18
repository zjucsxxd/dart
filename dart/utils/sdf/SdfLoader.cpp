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

#include "dart/utils/sdf/SdfLoader.h"

#include <map>
#include <iostream>
#include <fstream>
#include <boost/bind.hpp>

#include "dart/common/Console.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/ScrewJoint.h"
#include "dart/dynamics/TranslationalJoint.h"
#include "dart/dynamics/BallJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/EulerJoint.h"
#include "dart/dynamics/UniversalJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/SoftSkeleton.h"
#include "dart/simulation/World.h"
#include "dart/simulation/SoftWorld.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/Paths.h"

namespace dart {
namespace utils {

//==============================================================================
dynamics::SoftSkeleton* SdfLoader::loadSkeleton(const std::string& _filename)
{
  dynamics::SoftSkeleton* skeleton = new dynamics::SoftSkeleton();

  sdf::SDFPtr sdfSkeleton;
  sdf::readFile(_filename, sdfSkeleton);

//  sdfSkeleton->PrintDescription();

  return skeleton;
}

//==============================================================================
simulation::SoftWorld* SdfLoader::loadWorld(const std::string& _filename)
{
  //
  sdf::setFindCallback(boost::bind(&dart::utils::SdfLoader::findFile, _1));

  // Quick test for a valid file
  FILE *test = fopen(_filename.c_str(), "r");
  if (!test)
  {
    dterr << "Could not open file[" << _filename << "]\n";
    return NULL;
  }
  fclose(test);

  // Load the world file
  sdf::SDFPtr sdfWorld(new sdf::SDF);
  if (!sdf::init(sdfWorld))
  {
    dterr << "Unable to initialize sdf\n";
    return NULL;
  }

  if (!sdf::readFile(_filename, sdfWorld))
  {
    dterr << "Unable to read sdf file[" << _filename << "]\n";
    return NULL;
  }

  sdf::ElementPtr worldElem = sdfWorld->root->GetElement("world");

  if (!worldElem)
  {
    dterr << "The sdf file[" << _filename << "] does not have <world> tag.\n";
    return NULL;
  }
  else
  {
    return loadWorld(worldElem);
  }
}

//==============================================================================
simulation::SoftWorld* SdfLoader::loadWorld(sdf::ElementPtr _sdf)
{
  simulation::SoftWorld* world = new simulation::SoftWorld();

  // name attribute
  if (!_sdf->Get<std::string>("name").empty())
    dtwarn << "DART does not support name attribute of <world> tag\n";

  // <physics> element
  sdf::ElementPtr sdfPhys = _sdf->GetElement("physics");
  if (sdfPhys)
  {
    // <max_step_size>
    world->setTimeStep(sdfPhys->Get<double>("max_step_size"));

    // <real_time_factor>
    dtwarn << "DART does not support <real_time_factor> tag\n";

    // <real_time_update_rate>
    dtwarn << "DART does not support <real_time_update_rate> tag\n";

    // <gravity>
    world->setGravity(convVec3(sdfPhys->Get<sdf::Vector3>("gravity")));

    // <dart>
    sdf::ElementPtr sdfDart = _sdf->GetElement("dart");
    if (sdfDart)
    {
      // Some properties for DART will be added at SDFormat 1.5
    }
  }

  // load models
  if (_sdf->HasElement("model"))
  {
    sdf::ElementPtr childElem = _sdf->GetElement("model");

    while (childElem)
    {
//      dynamics::Skeleton* skel = loadSkeleton(childElem);
//      assert(skel != NULL);
//      world->addSkeleton(skel);

      childElem = childElem->GetNextElement("model");
    }
  }

  return world;
}

//==============================================================================
dynamics::Skeleton* SdfLoader::loadSkeleton(sdf::ElementPtr _sdf)
{
  dynamics::Skeleton* skel = new dynamics::Skeleton;

  if (_sdf->GetName() != "model")
  {
    dterr << "SDF is missing the <model> tag:\n";
    return NULL;
  }

  // load body nodes
  if (_sdf->HasElement("link"))
  {
    sdf::ElementPtr childElem = _sdf->GetElement("link");

    while (childElem)
    {
      dynamics::BodyNode* body = loadBodyNode(childElem);
      assert(body != NULL);
      skel->addBodyNode(body);

      childElem = childElem->GetNextElement("link");
    }
  }

  return skel;
}

//==============================================================================
dynamics::BodyNode* SdfLoader::loadBodyNode(sdf::ElementPtr _sdf)
{
  dynamics::BodyNode* body = new dynamics::BodyNode();

  return body;
}

//==============================================================================
std::string SdfLoader::findFile(const std::string& /*_filename*/)
{
  // TODO(JS): Not implemented yet.
  return std::string();
}

//==============================================================================
Eigen::Vector3d SdfLoader::convVec3(const sdf::Vector3& _val)
{

}

//==============================================================================
sdf::Vector3 SdfLoader::convVec3(const Eigen::Vector3d& _val)
{

}

}  // namespace utils
}  // namespace dart
