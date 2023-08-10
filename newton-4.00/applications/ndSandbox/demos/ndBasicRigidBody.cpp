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
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

#if 0
void ndBasicRigidBody (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, ndGetIdentityMatrix());
	
	ndMatrix origin1(ndGetIdentityMatrix());

	//AddSphere(scene, origin1, 1.0f, 0.5f);
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 1, 2, 7);
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
        ndJointHinge::JacobianDerivative(desc);
    }
};

class TestIKSolver : public ndModelArticulation
{
    public:
    TestIKSolver(ndDemoEntityManager* const scene, ndMatrix& location)
        :ndModelArticulation()
        ,m_controlJoint(nullptr)
    {
        ndFloat32 xSize = 0.25f;
        ndFloat32 ySize = 0.125f;
        ndFloat32 zSize = 0.15f;
        ndFloat32 cartMass = 5.0f;
        ndFloat32 poleMass = 10.0f;
        ndFloat32 poleLength = 0.4f;
        ndFloat32 poleRadio = 0.05f;
        ndPhysicsWorld* const world = scene->GetWorld();

        // make cart
        ndSharedPtr<ndBody> cartBody(world->GetBody(AddBox(scene, location, cartMass, xSize, ySize, zSize, "smilli.tga")));
        ndModelArticulation::ndNode* const modelRoot = AddRootBody(cartBody);
        ndMatrix matrix(cartBody->GetMatrix());
        matrix.m_posit.m_y += ySize / 2.0f;
        cartBody->SetMatrix(matrix);
        cartBody->GetAsBodyDynamic()->SetSleepAccel(cartBody->GetAsBodyDynamic()->GetSleepAccel() * ndFloat32(0.1f));

        // make pole leg
        matrix.m_posit.m_y += ySize / 2.0f;
        ndSharedPtr<ndBody> poleBody(world->GetBody(AddCapsule(scene, ndGetIdentityMatrix(), poleMass, poleRadio, poleRadio, poleLength, "smilli.tga")));
        ndMatrix poleLocation(ndRollMatrix(90.0f * ndDegreeToRad) * matrix);
        poleLocation.m_posit.m_y += poleLength * 0.5f;

        ndMatrix T(ndGetIdentityMatrix());
        T.m_posit.m_y = -poleLength * 0.5f;
        poleLocation = poleLocation * T * ndRollMatrix(30.0f * ndDegreeToRad) * T.OrthoInverse();

        poleBody->SetMatrix(poleLocation);
        poleBody->GetAsBodyDynamic()->SetSleepAccel(poleBody->GetAsBodyDynamic()->GetSleepAccel() * ndFloat32(0.1f));

        // link cart and body with a hinge
        ndMatrix polePivot(ndYawMatrix(90.0f * ndDegreeToRad) * poleLocation);
        polePivot.m_posit -= poleLocation.m_front.Scale (poleLength * 0.5f);
        ndSharedPtr<ndJointBilateralConstraint> poleJoint(new ndJointHinge(polePivot, poleBody->GetAsBodyKinematic(), modelRoot->m_body->GetAsBodyKinematic()));

        // make the car move alone the z axis only (2d problem)
        ndSharedPtr<ndJointBilateralConstraint> xDirSlider(new ndJointFix6dof(cartBody->GetMatrix(), cartBody->GetAsBodyDynamic(), world->GetSentinelBody()));
        world->AddJoint(xDirSlider);

        // add path to the model
        world->AddJoint(poleJoint);
        AddLimb(modelRoot, poleBody, poleJoint);

        // save some useful data
        //m_cart = cartBody->GetAsBodyDynamic();
        //m_pole = poleBody->GetAsBodyDynamic();
        //m_cartMatrix = cartBody->GetMatrix();
        //m_poleMatrix = poleBody->GetMatrix();
    }

    ndJointBilateralConstraint* CreateHinge(ndBodyKinematic* const pBodyA, ndBodyKinematic* pBodyB, const ndVector& vPivotA, const ndVector& vPivotB, const ndVector& vAxisA, const ndVector& vAxisB)
    {
        ndAssert(0);
        return nullptr;
        //ndMatrix mFrameA = ndGetIdentityMatrix();
        //ndVector vAx = vAxisA, vAy, vAz;
        //BuildOrthoNormalBasis(vAy, vAz, vAx);
        //mFrameA.m_posit = vPivotA;
        //mFrameA.m_front = ndVector(vAx.GetX(), vAx.GetY(), vAx.GetZ(), ndFloat32(0.0));
        //mFrameA.m_up = ndVector(vAy.GetX(), vAy.GetY(), vAy.GetZ(), ndFloat32(0.0));
        //mFrameA.m_right = ndVector(vAz.GetX(), vAz.GetY(), vAz.GetZ(), ndFloat32(0.0));
        //
        //ndMatrix mFrameB = ndGetIdentityMatrix();
        //ndVector vBx = vAxisB, vBy, vBz;
        //BuildOrthoNormalBasis(vBy, vBz, vBx);
        //mFrameB.m_posit = vPivotB;
        //mFrameB.m_front = ndVector(vBx.GetX(), vBx.GetY(), vBx.GetZ(), ndFloat32(0.0));
        //mFrameB.m_up = ndVector(vBy.GetX(), vBy.GetY(), vBy.GetZ(), ndFloat32(0.0));
        //mFrameB.m_right = ndVector(vBz.GetX(), vBz.GetY(), vBz.GetZ(), ndFloat32(0.0));
        //
        //ndMatrix mBodyB = ndGetIdentityMatrix();
        ////mBodyB = OrthoInvert(mFrameB) * mFrameA * pBodyA->GetMatrix();
        //mBodyB = mFrameB.OrthoInverse() * mFrameA * pBodyA->GetMatrix();
        //pBodyB->SetMatrix(mBodyB);
        //
        //return new ndCustomJointHinge(mFrameB, mFrameA, pBodyB, pBodyA);
    }

    void Update(ndWorld* const, ndFloat32 timestep)
    {
        if (m_controlJoint)
        {
            m_angle += ndFmod(1.0f * timestep, 2.0f * ndPi);
            ndFloat32 dist = 150.0f * ndDegreeToRad * ndSin(m_angle);
            m_controlJoint->SetTargetAngle(dist);
        }
    }

    ndFloat32 m_angle;
    ndJointHinge* m_controlJoint;
};

void ndBasicRigidBody(ndDemoEntityManager* const scene)
{
    BuildFlatPlane(scene, true);

    ndSetRandSeed(42);
    ndWorld* const world = scene->GetWorld();
    ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));

    ndSharedPtr<ndModel> model(new TestIKSolver(scene, matrix));
    world->AddModel(model);

    matrix.m_posit.m_x -= 0.0f;
    matrix.m_posit.m_y += 0.5f;
    matrix.m_posit.m_z += 2.0f;
    ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
    scene->SetCameraMatrix(rotation, matrix.m_posit);
}

#endif
