/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include "TransEulerJoint.h"

#include "common/Console.h"
#include "math/Helpers.h"
#include "math/Geometry.h"

namespace dart {
namespace dynamics {

TransEulerJoint::TransEulerJoint(const std::string& _name)
    : Joint(FREE, _name),
      mAxisOrder(AO_XYZ)
{
    mGenCoords.push_back(&mCoordinate[0]);
    mGenCoords.push_back(&mCoordinate[1]);
    mGenCoords.push_back(&mCoordinate[2]);
    mGenCoords.push_back(&mCoordinate[3]);
    mGenCoords.push_back(&mCoordinate[4]);
    mGenCoords.push_back(&mCoordinate[5]);

    mS = Eigen::Matrix<double,6,6>::Zero();
    mdS = Eigen::Matrix<double,6,6>::Zero();

    mDampingCoefficient.resize(6, 0);
}

TransEulerJoint::~TransEulerJoint()
{
}

void TransEulerJoint::setAxisOrder(TransEulerJoint::AxisOrder _order)
{
    mAxisOrder = _order;
}

TransEulerJoint::AxisOrder TransEulerJoint::getAxisOrder() const
{
    return mAxisOrder;
}

Eigen::Isometry3d TransEulerJoint::getTransform(size_t _index) const
{
    assert(_index < 6);

    if (_index < 3)
    {
        Eigen::Vector3d q = Eigen::Vector3d::Zero();
        q[_index] = mGenCoords[_index]->get_q();

        return Eigen::Isometry3d(Eigen::Translation3d(q));
    }
    else
    {
        Eigen::Vector3d q = Eigen::Vector3d::Zero();
        q[_index - 3] = mGenCoords[_index]->get_q();

        switch (mAxisOrder)
        {
            case AO_XYZ:
            {
                return Eigen::Isometry3d(math::eulerXYZToMatrix(q));
                break;
            }
            case AO_ZYX:
            {
                return Eigen::Isometry3d(math::eulerZYXToMatrix(q));
                break;
            }
            default:
            {
                dterr << "Undefined Euler axis order\n";
                return Eigen::Isometry3d::Identity();
                break;
            }
        }
    }
}

Eigen::Matrix4d TransEulerJoint::getTransformDerivative(size_t _index) const
{
    assert(_index < 6);

    Eigen::Matrix4d ret = Eigen::Matrix4d::Zero();

    if (_index < 3)
    {
        ret(_index, 3) = 1.0;
    }
    else
    {
        const double q0 = mCoordinate[3].get_q();
        const double q1 = mCoordinate[4].get_q();
        const double q2 = mCoordinate[5].get_q();

        switch (mAxisOrder)
        {
            case AO_XYZ:
                switch (_index - 3)
                {
                    case 0:
                        ret.topLeftCorner<3, 3>() = math::eulerToMatrixXDeriv(q0);
                        break;
                    case 1:
                        ret.topLeftCorner<3, 3>() = math::eulerToMatrixYDeriv(q1);
                        break;
                    case 2:
                        ret.topLeftCorner<3, 3>() = math::eulerToMatrixZDeriv(q2);
                        break;
                    default:
                        break;
                }
                break;
            case AO_ZYX:
                switch (_index - 3)
                {
                    case 0:
                        ret.topLeftCorner<3, 3>() = math::eulerToMatrixZDeriv(q0);
                        break;
                    case 1:
                        ret.topLeftCorner<3, 3>() = math::eulerToMatrixYDeriv(q1);
                        break;
                    case 2:
                        ret.topLeftCorner<3, 3>() = math::eulerToMatrixXDeriv(q2);
                        break;
                    default:
                        break;
                }
                break;
            default:
                dterr << "Undefined Euler axis order\n";
                break;
        }
    }

    return ret;
}

void TransEulerJoint::updateTransform()
{
    Eigen::Vector3d q1(mCoordinate[0].get_q(),
                       mCoordinate[1].get_q(),
                       mCoordinate[2].get_q());
    Eigen::Vector3d q2(mCoordinate[3].get_q(),
                       mCoordinate[4].get_q(),
                       mCoordinate[5].get_q());

    switch (mAxisOrder)
    {
    case AO_XYZ:
    {
        mT = mT_ParentBodyToJoint *
             Eigen::Translation3d(q2) *
             Eigen::Isometry3d(math::eulerXYZToMatrix(q1)) *
             mT_ChildBodyToJoint.inverse();
        break;
    }
    case AO_ZYX:
    {
        mT = mT_ParentBodyToJoint *
             Eigen::Translation3d(q2) *
             Eigen::Isometry3d(math::eulerZYXToMatrix(q1)) *
             mT_ChildBodyToJoint.inverse();
        break;
    }
    default:
    {
        dterr << "Undefined Euler axis order\n";
        break;
    }
    }

    assert(math::verifyTransform(mT));
}

void TransEulerJoint::updateJacobian()
{
    Eigen::Vector3d q(mCoordinate[0].get_q(),
                      mCoordinate[1].get_q(),
                      mCoordinate[2].get_q());

    //double q0 = mCoordinate[0].get_q();
    double q1 = mCoordinate[1].get_q();
    double q2 = mCoordinate[2].get_q();

    //double c0 = cos(q0);
    double c1 = cos(q1);
    double c2 = cos(q2);

    //double s0 = sin(q0);
    double s1 = sin(q1);
    double s2 = sin(q2);

    Eigen::Vector6d J0;
    Eigen::Vector6d J1;
    Eigen::Vector6d J2;
    Eigen::Vector6d J3;
    Eigen::Vector6d J4;
    Eigen::Vector6d J5;

    switch (mAxisOrder)
    {
    case AO_XYZ:
    {
        //--------------------------------------------------------------------------
        // S = [    c1*c2, s2,  0
        //       -(c1*s2), c2,  0
        //             s1,  0,  1
        //              0,  0,  0
        //              0,  0,  0
        //              0,  0,  0 ];
        //--------------------------------------------------------------------------
        J0 << c1*c2, -(c1*s2),  s1, 0.0, 0.0, 0.0;
        J1 <<    s2,       c2, 0.0, 0.0, 0.0, 0.0;
        J2 <<   0.0,      0.0, 1.0, 0.0, 0.0, 0.0;

        break;
    }
    case AO_ZYX:
    {
        //--------------------------------------------------------------------------
        // S = [   -s1,    0,   1
        //       s2*c1,   c2,   0
        //       c1*c2,  -s2,   0
        //           0,    0,   0
        //           0,    0,   0
        //           0,    0,   0 ];
        //--------------------------------------------------------------------------
        J0 << -s1, s2*c1, c1*c2, 0.0, 0.0, 0.0;
        J1 << 0.0,    c2,   -s2, 0.0, 0.0, 0.0;
        J2 << 1.0,   0.0,   0.0, 0.0, 0.0, 0.0;
        break;
    }
    default:
    {
        dterr << "Undefined Euler axis order\n";
        break;
    }
    }

    J3 << 0, 0, 0, 1, 0, 0;
    J4 << 0, 0, 0, 0, 1, 0;
    J5 << 0, 0, 0, 0, 0, 1;

    mS.col(0) = math::AdT(mT_ChildBodyToJoint, J0);
    mS.col(1) = math::AdT(mT_ChildBodyToJoint, J1);
    mS.col(2) = math::AdT(mT_ChildBodyToJoint, J2);
    mS.col(3) = math::AdT(mT_ChildBodyToJoint * Eigen::Isometry3d(math::eulerXYZToMatrix(q)).inverse(), J3);
    mS.col(4) = math::AdT(mT_ChildBodyToJoint * Eigen::Isometry3d(math::eulerXYZToMatrix(q)).inverse(), J4);
    mS.col(5) = math::AdT(mT_ChildBodyToJoint * Eigen::Isometry3d(math::eulerXYZToMatrix(q)).inverse(), J5);
}

void TransEulerJoint::updateJacobianTimeDeriv()
{
    Eigen::Vector3d q(mCoordinate[0].get_q(),
                      mCoordinate[1].get_q(),
                      mCoordinate[2].get_q());
    Eigen::Vector3d dq(mCoordinate[0].get_dq(),
                       mCoordinate[1].get_dq(),
                       mCoordinate[2].get_dq());

    double q0 = mCoordinate[0].get_q();
    double q1 = mCoordinate[1].get_q();
    double q2 = mCoordinate[2].get_q();

    //double dq0 = mCoordinate[0].get_dq();
    double dq1 = mCoordinate[1].get_dq();
    double dq2 = mCoordinate[2].get_dq();

    //double c0 = cos(q0);
    double c1 = cos(q1);
    double c2 = cos(q2);

    //double s0 = sin(q0);
    double s1 = sin(q1);
    double s2 = sin(q2);

    Eigen::Vector6d dJ0;
    Eigen::Vector6d dJ1;
    Eigen::Vector6d dJ2;
    Eigen::Vector6d dJ3;
    Eigen::Vector6d dJ4;
    Eigen::Vector6d dJ5;

    switch (mAxisOrder)
    {
    case AO_XYZ:
    {
        //--------------------------------------------------------------------------
        // dS = [  -(dq1*c2*s1) - dq2*c1*s2,    dq2*c2,  0
        //         -(dq2*c1*c2) + dq1*s1*s2, -(dq2*s2),  0
        //                           dq1*c1,         0,  0
        //                                0,         0,  0
        //                                0,         0,  0
        //                                0,         0,  0 ];
        //--------------------------------------------------------------------------
        dJ0 << -(dq1*c2*s1) - dq2*c1*s2, -(dq2*c1*c2) + dq1*s1*s2, dq1*c1, 0, 0, 0;
        dJ1 << dq2*c2,                -(dq2*s2),    0.0, 0.0, 0.0, 0.0;
        dJ2.setConstant(0.0);

        break;
    }
    case AO_ZYX:
    {
        //--------------------------------------------------------------------------
        // dS = [               -c1*dq1,        0,   0
        //          c2*c1*dq2-s2*s1*dq1,  -s2*dq2,   0
        //         -s1*c2*dq1-c1*s2*dq2,  -c2*dq2,   0
        //                            0,        0,   0
        //                            0,        0,   0
        //                            0,        0,   0 ];
        //--------------------------------------------------------------------------
        dJ0 << -c1*dq1, c2*c1*dq2 - s2*s1*dq1, -s1*c2*dq1 - c1*s2*dq2, 0.0, 0.0, 0.0;
        dJ1 <<     0.0,               -s2*dq2,                -c2*dq2, 0.0, 0.0, 0.0;
        dJ2.setConstant(0.0);
        break;
    }
    default:
    {
        dterr << "Undefined Euler axis order\n";
        break;
    }
    }

    dJ3 << 0, 0, 0, 1, 0, 0;
    dJ4 << 0, 0, 0, 0, 1, 0;
    dJ5 << 0, 0, 0, 0, 0, 1;

    mdS.col(0) = math::AdT(mT_ChildBodyToJoint, dJ0);
    mdS.col(1) = math::AdT(mT_ChildBodyToJoint, dJ1);
    mdS.col(2) = math::AdT(mT_ChildBodyToJoint, dJ2);
    mdS.col(3) = -math::ad(mS.leftCols<3>() * get_dq().head<3>(), math::AdT(mT_ChildBodyToJoint * Eigen::Isometry3d(math::eulerXYZToMatrix(q)).inverse(), dJ3));
    mdS.col(4) = -math::ad(mS.leftCols<3>() * get_dq().head<3>(), math::AdT(mT_ChildBodyToJoint * Eigen::Isometry3d(math::eulerXYZToMatrix(q)).inverse(), dJ4));
    mdS.col(5) = -math::ad(mS.leftCols<3>() * get_dq().head<3>(), math::AdT(mT_ChildBodyToJoint * Eigen::Isometry3d(math::eulerXYZToMatrix(q)).inverse(), dJ5));
}

} // namespace dynamics
} // namespace dart
