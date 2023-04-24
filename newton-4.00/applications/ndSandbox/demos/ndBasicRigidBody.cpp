/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndSkyBox.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

#if 0
void ndBasicRigidBody (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, ndGetIdentityMatrix());
	
	ndMatrix origin1(ndGetIdentityMatrix());

	//AddSphere(scene, origin1, 1.0f, 0.5f);
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 1, 1, 1);
	AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 10, 10, 7);
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 4, 4, 4);
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 2, 2, 7);

	ndQuaternion rot;
	ndVector origin(-60.0f, 5.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}

#else


class ndCustomJointHinge : public ndJointHinge
{
    public:
    ndCustomJointHinge(const ndMatrix& mFrameChild, const ndMatrix& mFrameParent, ndBodyKinematic* const child, ndBodyKinematic* const parent)
        :ndJointHinge(ndGetIdentityMatrix(), child, parent)
    {
        m_localMatrix0 = mFrameChild;
        m_localMatrix1 = mFrameParent;
    }

    void JacobianDerivative(ndConstraintDescritor& desc)
    {
        /*m_angle += ndFmod(5.0f * desc.m_timestep, 2.0f * ndPi);
        ndFloat32 dist = 150.0f * ndDegreeToRad * ndSin(m_angle);
        SetOffsetAngle(dist);*/
        ndJointHinge::JacobianDerivative(desc);
    }
};

static void BuildOrthoNormalBasis(ndVector& u, ndVector& v, ndVector& w)
{
    w.Normalize();

    if (ndAbs(w.GetX()) >= ndAbs(w.GetY()) && ndAbs(w.GetX()) >= ndAbs(w.GetZ()))
    {
        u.SetX(-w.GetY());
        u.SetY(w.GetX());
        u.SetZ(0.0);
    }
    else
    {
        u.SetX(0.0);
        u.SetY(w.GetZ());
        u.SetZ(-w.GetY());
    }

    u.Normalize();
    v = w.CrossProduct(u);
}

static ndMatrix OrthoInvert(const ndMatrix& mFrame)
{
    ndMatrix mRet = ndGetIdentityMatrix();
    mRet.m_front = ndVector(mFrame.m_front[0], mFrame.m_up[0], mFrame.m_right[0], ndFloat32 (0.0));
    mRet.m_up = ndVector(mFrame.m_front[1], mFrame.m_up[1], mFrame.m_right[1], ndFloat32(0.0));
    mRet.m_right = ndVector(mFrame.m_front[2], mFrame.m_up[2], mFrame.m_right[2], ndFloat32(0.0));
    mRet.m_posit = ndVector(-mFrame.m_front[0] * mFrame.m_posit.GetX() - mFrame.m_front[1] * mFrame.m_posit.GetY() - mFrame.m_front[2] * mFrame.m_posit.GetZ(),
        -mFrame.m_up[0] * mFrame.m_posit.GetX() - mFrame.m_up[1] * mFrame.m_posit.GetY() - mFrame.m_up[2] * mFrame.m_posit.GetZ(),
        -mFrame.m_right[0] * mFrame.m_posit.GetX() - mFrame.m_right[1] * mFrame.m_posit.GetY() - mFrame.m_right[2] * mFrame.m_posit.GetZ(), ndFloat32(1.0));

    return mRet;
}

static void CreateHinge(ndPhysicsWorld* const pWorld, ndBodyKinematic* const pBodyA, ndBodyKinematic* pBodyB, const ndVector& vPivotA, const ndVector& vPivotB, const ndVector& vAxisA, const ndVector& vAxisB)
{
    ndMatrix mFrameA = ndGetIdentityMatrix();
    ndVector vAx = vAxisA, vAy, vAz;
    BuildOrthoNormalBasis(vAy, vAz, vAx);
    mFrameA.m_posit = vPivotA;
    mFrameA.m_front = ndVector(vAx.GetX(), vAx.GetY(), vAx.GetZ(), ndFloat32(0.0));
    mFrameA.m_up = ndVector(vAy.GetX(), vAy.GetY(), vAy.GetZ(), ndFloat32(0.0));
    mFrameA.m_right = ndVector(vAz.GetX(), vAz.GetY(), vAz.GetZ(), ndFloat32(0.0));

    ndMatrix mFrameB = ndGetIdentityMatrix();
    ndVector vBx = vAxisB, vBy, vBz;
    BuildOrthoNormalBasis(vBy, vBz, vBx);
    mFrameB.m_posit = vPivotB;
    mFrameB.m_front = ndVector(vBx.GetX(), vBx.GetY(), vBx.GetZ(), ndFloat32(0.0));
    mFrameB.m_up = ndVector(vBy.GetX(), vBy.GetY(), vBy.GetZ(), ndFloat32(0.0));
    mFrameB.m_right = ndVector(vBz.GetX(), vBz.GetY(), vBz.GetZ(), ndFloat32(0.0));

    ndMatrix mBodyB = ndGetIdentityMatrix();
    mBodyB = OrthoInvert(mFrameB) * mFrameA * pBodyA->GetMatrix();
    pBodyB->SetMatrix(mBodyB);

    ndCustomJointHinge* const joint = new ndCustomJointHinge(mFrameB, mFrameA, pBodyB, pBodyA);
    joint->SetLimits(1.0, 0.0);      // Full rotation
    joint->SetLimitState(true);
    ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
    pWorld->AddJoint(jointPtr);
}

//void ndDraglineLeg(ndDemoEntityManager* const scene)
void ndBasicRigidBody(ndDemoEntityManager* const scene)
{
    // build a floor
    auto pFloorBody = BuildFloorBox(scene, ndGetIdentityMatrix());

    ndMatrix origin1(ndGetIdentityMatrix());
    origin1.m_posit = ndVector(0.0, 4.0, 0.0, 1.0);

    // Bodies
    auto pMainBody = AddBox(scene, origin1, 1500000.0, 4.0, 8.0, 8.0);
    auto pAxleBody = AddBox(scene, ndGetIdentityMatrix(), 100.0, 0.2, 1.3, 0.4);
    auto pCamLeverBody = AddBox(scene, ndGetIdentityMatrix(), 100.0, 0.2, 1.3, 0.4);
    auto pLegBody = AddBox(scene, ndGetIdentityMatrix(), 1000.0, 0.4, 3.0, 1.0);

    pMainBody->SetAutoSleep(false);

    // Joints
    CreateHinge(scene->GetWorld(), pMainBody, pAxleBody, ndVector(-2.1, -2.5, 0.0, 1.0), ndVector(0.0, -0.45, 0.0, 1.0), ndVector(1.0, 0.0, 0.0, 0.0), ndVector(1.0, 0.0, 0.0, 0.0));
    CreateHinge(scene->GetWorld(), pMainBody, pCamLeverBody, ndVector(-2.47, -1.0, 0.0, 1.0), ndVector(0.0, -0.45, 0.0, 1.0), ndVector(1.0, 0.0, 0.0, 0.0), ndVector(1.0, 0.0, 0.0, 0.0));
    CreateHinge(scene->GetWorld(), pAxleBody, pLegBody, ndVector(0.0, 0.3, 0.0, 1.0), ndVector(0.37, -0.35, 0.35, 1.0), ndVector(1.0, 0.0, 0.0, 0.0), ndVector(1.0, 0.0, 0.0, 0.0));
    CreateHinge(scene->GetWorld(), pCamLeverBody, pLegBody, ndVector(0.0, 0.45, 0.0, 1.0), ndVector(0.0, 1.25, 0.25, 1.0), ndVector(1.0, 0.0, 0.0, 0.0), ndVector(1.0, 0.0, 0.0, 0.0));

    // lower the floor 
    ndMatrix matrix(pFloorBody->GetMatrix());
    //matrix.m_posit.m_y -= 4.0f;
    pFloorBody->SetMatrix(matrix);

    ndQuaternion rot;
    ndVector origin(-60.0f, 5.0f, 0.0f, 1.0f);
    scene->SetCameraMatrix(rot, origin);
}


#endif
