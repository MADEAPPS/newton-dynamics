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

//#else
#elif 0

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

class ndModelDragLine : public ndModelArticulation
{
    public:
    ndModelDragLine(ndDemoEntityManager* const scene, ndMatrix& origin)
        :ndModelArticulation()
        ,m_controlJoint(nullptr)
    {
        auto pMainBody = AddBox(scene, origin, 1500000.0, 4.0, 8.0, 8.0);

        // Misho
        origin.m_posit.m_y += 20.f;
        auto pMainBody2 = AddBox(scene, origin, 1500000.0, 4.0, 8.0, 8.0);
        ndMatrix m(pMainBody2->GetMatrix());
        m.m_posit.m_y = 20.0f;
        pMainBody2->SetMatrix(m);

        ndMatrix Gmatrix(ndGetIdentityMatrix());
        Gmatrix.m_posit = ((pMainBody->GetMatrix().m_posit + pMainBody2->GetMatrix().m_posit).Scale(0.5f));
        ndJointFix6dof* const joint = new ndJointFix6dof(Gmatrix, pMainBody, pMainBody2);
        ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
        ndWorld* myworld = scene->GetWorld();
        myworld->AddJoint(jointPtr);

        auto pAxleBody = AddBox(scene, ndGetIdentityMatrix(), 100.0, 0.2, 1.3, 0.4);
        auto pCamLeverBody = AddBox(scene, ndGetIdentityMatrix(), 100.0, 0.2, 1.3, 0.4);
        auto pLegBody = AddBox(scene, ndGetIdentityMatrix(), 1000.0, 0.4, 3.0, 1.0);
        
        pMainBody->SetAutoSleep(false);
        
        ndSharedPtr<ndJointBilateralConstraint> axleHinge (CreateHinge(pMainBody, pAxleBody, ndVector(-2.1, -2.5, 0.0, 1.0), ndVector(0.0, -0.45, 0.0, 1.0), ndVector(1.0, 0.0, 0.0, 0.0), ndVector(1.0, 0.0, 0.0, 0.0)));
        ndSharedPtr<ndJointBilateralConstraint> camHinge(CreateHinge(pMainBody, pCamLeverBody, ndVector(-2.47, -1.0, 0.0, 1.0), ndVector(0.0, -0.45, 0.0, 1.0), ndVector(1.0, 0.0, 0.0, 0.0), ndVector(1.0, 0.0, 0.0, 0.0)));
        ndSharedPtr<ndJointBilateralConstraint> axleLegHinge (CreateHinge(pAxleBody, pLegBody, ndVector(0.0, 0.3, 0.0, 1.0), ndVector(0.37, -0.35, 0.35, 1.0), ndVector(1.0, 0.0, 0.0, 0.0), ndVector(1.0, 0.0, 0.0, 0.0)));
        ndSharedPtr<ndJointBilateralConstraint> camLegHinge (CreateHinge(pCamLeverBody, pLegBody, ndVector(0.0, 0.45, 0.0, 1.0), ndVector(0.0, 1.25, 0.25, 1.0), ndVector(1.0, 0.0, 0.0, 0.0), ndVector(1.0, 0.0, 0.0, 0.0)));
        
        // build the articulated model structure
        ndWorld* const world = scene->GetWorld();
        
        ndNode* const rootNode = AddRootBody(world->GetBody(pMainBody));
        
        ndNode* const axleNode = AddLimb(rootNode, world->GetBody(pAxleBody), axleHinge);
        AddLimb(axleNode, world->GetBody(pLegBody), axleLegHinge);
         
        ndNode* const camNode = AddLimb(rootNode, world->GetBody(pCamLeverBody), camHinge);
        AddLimb(camNode, world->GetBody(pLegBody), camLegHinge);
        
        // save the motor joint. for controlling
        m_angle = 0.0;
        m_controlJoint = (ndCustomJointHinge*) *axleHinge;
        m_controlJoint->SetAsSpringDamper(0.0001, 2000.0f, 30.0f);
    }

    void BuildOrthoNormalBasis(ndVector& u, ndVector& v, ndVector& w)
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

    ndJointBilateralConstraint* CreateHinge(ndBodyKinematic* const pBodyA, ndBodyKinematic* pBodyB, const ndVector& vPivotA, const ndVector& vPivotB, const ndVector& vAxisA, const ndVector& vAxisB)
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
        //mBodyB = OrthoInvert(mFrameB) * mFrameA * pBodyA->GetMatrix();
        mBodyB = mFrameB.OrthoInverse() * mFrameA * pBodyA->GetMatrix();
        pBodyB->SetMatrix(mBodyB);

        return new ndCustomJointHinge(mFrameB, mFrameA, pBodyB, pBodyA);
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
    // build a floor
    auto pFloorBody = BuildFloorBox(scene, ndGetIdentityMatrix());

    ndMatrix origin1(ndGetIdentityMatrix());
    origin1.m_posit = ndVector(0.0, 4.0, 0.0, 1.0);

    ndSharedPtr<ndModel> dragLine(new ndModelDragLine(scene, origin1));
    scene->GetWorld()->AddModel(dragLine);

    // lower the floor 
    ndMatrix matrix(pFloorBody->GetMatrix());
    //matrix.m_posit.m_y -= 4.0f;
    pFloorBody->SetMatrix(matrix);

    ndQuaternion rot;
    ndVector origin(-60.0f, 5.0f, 0.0f, 1.0f);
    scene->SetCameraMatrix(rot, origin);
}

#else

///* Copyright (c) <2003-2022> <Newton Game Dynamics>
//*
//* This software is provided 'as-is', without any express or implied
//* warranty. In no event will the authors be held liable for any damages
//* arising from the use of this software.
//*
//* Permission is granted to anyone to use this software for any purpose,
//* including commercial applications, and to alter it and redistribute it
//* freely
//*/
//



static ndSharedPtr<ndBody> AddBoxBody(ndDemoEntityManager* const scene, ndBigVector vPos, ndBigVector nvSize, ndFloat32 mass, bool Kinematic)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndShapeInstance box(new ndShapeBox(ndFloat32 (nvSize.m_x), ndFloat32(nvSize.m_y), ndFloat32(nvSize.m_z)));
	ndMatrix uvMatrix(ndGetIdentityMatrix());
	const char* const textureName = "default.tga";
	ndSharedPtr<ndDemoMeshInterface> geometry(new ndDemoMesh("box", scene->GetShaderCache(), &box, textureName, textureName, textureName, 1.0f, uvMatrix));

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit = vPos;
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(geometry);

	ndBodyKinematic* Body;
	if (Kinematic)
		Body = new ndBodyKinematic();
	else
		Body = new ndBodyDynamic();

	Body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	//    Body->SetNotifyCallback(new ndApplyFixForce());
	Body->SetMatrix(matrix);
	Body->SetCollisionShape(box);
	Body->SetMassMatrix(mass, box);
	scene->AddEntity(entity);

	ndSharedPtr<ndBody> bodyPtr(Body);
	world->AddBody(bodyPtr);
	return bodyPtr;

}

void ndHardLink2Bodies(ndDemoEntityManager* const scene, ndSharedPtr<ndBody> b1, ndSharedPtr<ndBody> b2)
{
	ndMatrix Gmatrix(ndGetIdentityMatrix());
	Gmatrix.m_posit = ((b1->GetMatrix().m_posit + b2->GetMatrix().m_posit).Scale(0.5f));
	ndJointFix6dof* const joint = new ndJointFix6dof(Gmatrix, b1->GetAsBodyKinematic(), b2->GetAsBodyKinematic());
	ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
	ndWorld* myworld = scene->GetWorld();
	myworld->AddJoint(jointPtr);
}

//void ndMishoTestCase(ndDemoEntityManager* const scene)
void ndBasicRigidBody(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, ndGetIdentityMatrix());

	ndBigVector BoxSize(10.f, 10.f, 10.f, 1.0f);
	//ndFloat32 massBox = 1500000.0;
    ndFloat32 massBox = 1500.0;
	ndBigVector PlankSize(4.f, 2.f, 40.f, 1.0f);
	ndFloat32 massPlank = 1500.0;

	ndBigVector posD(60.f, 5.f, -30.f, 1.0f);
	ndSharedPtr<ndBody> bodyDynamic = AddBoxBody(scene, posD, BoxSize, massBox, false);
	posD.m_y = 11.f;
	ndSharedPtr<ndBody> plankDynamic = AddBoxBody(scene, posD, PlankSize, massPlank, false);

	ndBigVector posK(60.f, 5.f, 30.f, 1.0f);
	ndSharedPtr<ndBody> bodyKinematic = AddBoxBody(scene, posK, BoxSize, massBox, true);
	posK.m_y = 11.f;
	ndSharedPtr<ndBody> plankKinematic = AddBoxBody(scene, posK, PlankSize, massPlank, false);


	ndHardLink2Bodies(scene, bodyKinematic, plankKinematic);
	ndHardLink2Bodies(scene, bodyDynamic, plankDynamic);

	ndMatrix FlightSimPosition(ndGetIdentityMatrix());

	ndQuaternion rot;
	ndVector origin(0.0f, 5.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}

#endif
