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
    mSkel = dart::utils::SkelParser::readSkeleton(DART_DATA_PATH"skel/leg.skel");
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
    Matrix4d jointToChild =joint->getTransformFromChildBodyNode().inverse().matrix();
    Vector4d J = worldToParent * parentToJoint * dR * jointToChild * offset;
    int colIndex = joint->getGenCoord(0)->getSkeletonIndex();
    mJ.col(colIndex) = J.head(3);
    offset = joint->getLocalTransform().matrix() * offset;
    
    // w.r.t knee
    node = node->getParentBodyNode();
    joint = node->getParentJoint();
    worldToParent = node->getParentBodyNode()->getWorldTransform().matrix();
    parentToJoint = joint->getTransformFromParentBodyNode().matrix();
    dR = joint->getTransformDerivative(0);
    jointToChild =joint->getTransformFromChildBodyNode().inverse().matrix();
    J = worldToParent * parentToJoint * dR * jointToChild * offset;
    colIndex = joint->getGenCoord(0)->getSkeletonIndex();
    mJ.col(colIndex) = J.head(3);
    offset = joint->getLocalTransform() * offset;

    // w.r.t hip
    node = node->getParentBodyNode();
    joint = node->getParentJoint();
    worldToParent = node->getParentBodyNode()->getWorldTransform().matrix();
    parentToJoint = joint->getTransformFromParentBodyNode().matrix();
    dR = joint->getTransformDerivative(2);
    Matrix4d Rz = joint->getTransform(0).matrix();
    Matrix4d Ry = joint->getTransform(1).matrix();
    jointToChild =joint->getTransformFromChildBodyNode().inverse().matrix();
    J = worldToParent * parentToJoint * Rz * Ry * dR * jointToChild * offset;
    colIndex = joint->getGenCoord(2)->getSkeletonIndex();
    mJ.col(colIndex) = J.head(3);

    Matrix4d Rx = joint->getTransform(2).matrix();
    dR = joint->getTransformDerivative(1);
    J = worldToParent * parentToJoint * Rz * dR * Rx * jointToChild * offset;
    colIndex = joint->getGenCoord(1)->getSkeletonIndex();
    mJ.col(colIndex) = J.head(3);

    dR = joint->getTransformDerivative(0);
    J = worldToParent * parentToJoint * dR * Ry * Rx * jointToChild * offset;
    colIndex = joint->getGenCoord(0)->getSkeletonIndex();
    mJ.col(colIndex) = J.head(3);
    offset = joint->getLocalTransform().matrix() * offset;
    
    VectorXd gradients = 2 * mJ.transpose() * mC;
    return gradients;
}

// TODO: Current code can only handle one constraint at a time.
void MyWorld::createConstraint(int _index) {
    mConstrainedMarker = _index;
    mTarget = mSkel->getMarker(_index)->getWorldCoords();
}

void MyWorld::modifyConstraint(Vector3d _deltaP) {
    mTarget += _deltaP;

}

void MyWorld::removeConstraint(int _index) {
    mConstrainedMarker = -1;
}


