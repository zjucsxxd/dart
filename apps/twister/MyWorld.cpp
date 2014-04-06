#include "MyWorld.h"
#include "utils/Paths.h"
#include "utils/SkelParser.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Joint.h"
#include "dynamics/Marker.h"
#include <iostream>

using namespace Eigen;
using namespace dart::dynamics;

MyWorld::MyWorld() {
    // Load a skeleton from file
    mSkel = dart::utils::SkelParser::readSkeleton(DART_DATA_PATH"skel/human.skel");
    mJ = MatrixXd::Zero(3, mSkel->getNumGenCoords());    
    mConstrainedMarker = -1;
}

MyWorld::~MyWorld() {
    delete mSkel;
}

void MyWorld::solve() {
    int numIter = 300;
    double alpha = 0.01;
    int nDof = mSkel->getNumGenCoords();
    VectorXd gradients(nDof);
    VectorXd newPose(nDof);
    for (int i = 0; i < numIter; i++) {
        gradients = updateGradients();
        newPose = mSkel->getConfig() - alpha * gradients;
        mSkel->setConfig(newPose);
    }
}

VectorXd MyWorld::updateGradients() {
    // compute c(q)
    mC = mSkel->getMarker(mConstrainedMarker)->getWorldCoords() - mTarget;

    // compute J(q)
    Vector4d offset;
    offset << mSkel->getMarker(mConstrainedMarker)->getLocalCoords(), 1;
    // w.r.t ankle
    BodyNode *node = mSkel->getMarker(mConstrainedMarker)->getBodyNode();
    Joint *joint = node->getParentJoint();
    Matrix4d worldToParent = node->getParentBodyNode()->getWorldTransform().matrix();
    Matrix4d parentToJoint = joint->getTransformFromParentBodyNode().matrix();
    Matrix4d dR = joint->getTransformDerivative(0);
    Matrix4d R = joint->getTransform(1).matrix();
    Matrix4d jointToChild =joint->getTransformFromChildBodyNode().inverse().matrix();
    Vector4d J = worldToParent * parentToJoint * dR * R * jointToChild * offset;
    int colIndex = joint->getGenCoord(0)->getSkeletonIndex();
    mJ.col(colIndex) = J.head(3);
    dR = joint->getTransformDerivative(1);
    R = joint->getTransform(0).matrix();
    J = worldToParent * parentToJoint * R * dR * jointToChild * offset;
    colIndex = joint->getGenCoord(1)->getSkeletonIndex();
    mJ.col(colIndex) = J.head(3);
    
    VectorXd gradients = 2 * mJ.transpose() * mC;
    return gradients;
}

// TODO: Current code can only handle one constraint at the left foot.
void MyWorld::createConstraint(int _index) {
    if (_index == 3) {
        mConstrainedMarker = _index;
        mTarget = mSkel->getMarker(_index)->getWorldCoords();
    }
}

void MyWorld::modifyConstraint(Vector3d _deltaP) {
    mTarget += _deltaP;

}

void MyWorld::removeConstraint(int _index) {
    mConstrainedMarker = -1;
}


