/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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
#include "ndTargaToOpenGl.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

#if 0
static ndBodyDynamic* AddRigidBody(ndDemoEntityManager* const scene,
	const ndMatrix& matrix, const ndShapeInstance& shape,
	ndDemoInstanceEntity* const rootEntity, ndFloat32 mass)
{
	ndBodyDynamic* const body = new ndBodyDynamic();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, rootEntity);

	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);

	ndWorld* const world = scene->GetWorld();
	world->AddBody(body);
	return body;
}

static void AddToCompoundShape(const ndMatrix& mLocalMatrix, ndShapeInstance& parentShape, ndShapeInstance& childInstance)
{
	auto pCompoundShape = parentShape.GetShape()->GetAsShapeCompound();
	childInstance.SetLocalMatrix(mLocalMatrix);
	pCompoundShape->AddCollision(&childInstance);
}

void CreateBoxCompoundShape(ndShapeInstance& parentInstance)
{
	ndShapeInstance wall1(new ndShapeBox(2.0f, 2.0f, 0.1f));
	ndShapeInstance wall2(new ndShapeBox(0.1f, 2.0f, 2.0f));
	ndShapeInstance wall3(new ndShapeBox(2.0f, 2.0f, 0.1f));
	ndShapeInstance wall4(new ndShapeBox(0.1f, 2.0f, 2.0f));
	ndShapeInstance floor(new ndShapeBox(2.0f, 0.1f, 2.0f));
	ndMatrix mWall1Local = dGetIdentityMatrix();
	mWall1Local.m_posit = ndVector(0.0f, 0.0f, -1.0f, 1.0f);
	ndMatrix mWall2Local = dGetIdentityMatrix();
	mWall2Local.m_posit = ndVector(1.0f, 0.0f, 0.0f, 1.0f);
	ndMatrix mWall3Local = dGetIdentityMatrix();
	mWall3Local.m_posit = ndVector(0.0f, 0.0f, 1.0f, 1.0f);
	ndMatrix mWall4Local = dGetIdentityMatrix();
	mWall4Local.m_posit = ndVector(-1.0f, 0.0f, 0.0f, 1.0f);
	ndMatrix mFloorLocal = dGetIdentityMatrix();
	mFloorLocal.m_posit = ndVector(0.0f, -1.0f, 0.0f, 1.0f);

	auto pCompoundShape = parentInstance.GetShape()->GetAsShapeCompound();
	pCompoundShape->BeginAddRemove();
	AddToCompoundShape(mWall1Local, parentInstance, wall1);
	AddToCompoundShape(mWall2Local, parentInstance, wall2);
	AddToCompoundShape(mWall3Local, parentInstance, wall3);
	AddToCompoundShape(mWall4Local, parentInstance, wall4);
	AddToCompoundShape(mFloorLocal, parentInstance, floor);
	pCompoundShape->EndAddRemove();
}

void ndBasicCompoundShapeDemo(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFlatPlane(scene, true);
	ndShapeInstance compoundShapeInstance(new ndShapeCompound());
	CreateBoxCompoundShape(compoundShapeInstance);

	ndDemoMeshIntance* const compGeometry = new ndDemoMeshIntance("compoundShape", scene->GetShaderCache(), &compoundShapeInstance, "earthmap.tga", "earthmap.tga", "earthmap.tga");
	ndDemoInstanceEntity* const compEntity = new ndDemoInstanceEntity(compGeometry);
	scene->AddEntity(compEntity);
	ndMatrix mBodyMatrix = dGetIdentityMatrix();
	mBodyMatrix.m_posit = ndVector(-2.0f, 5.0f, -5.0f, 1.0f);
	AddRigidBody(scene, mBodyMatrix, compoundShapeInstance, compEntity, 10.0);

	ndShapeInstance originShape(new ndShapeSphere(0.5));
	ndDemoMeshIntance* const origGeometry = new ndDemoMeshIntance("origShape", scene->GetShaderCache(), &originShape, "earthmap.tga", "earthmap.tga", "earthmap.tga");
	ndDemoInstanceEntity* const origEntity = new ndDemoInstanceEntity(origGeometry);
	scene->AddEntity(origEntity);
	ndMatrix mOrigMatrix = dGetIdentityMatrix();
	mOrigMatrix.m_posit.m_y += 2.0f;
	AddRigidBody(scene, mOrigMatrix, originShape, origEntity, 5.0);

	ndVector origin(ndVector::m_zero);
	ndQuaternion rot(dYawMatrix(45.0f * ndDegreeToRad));
	origin.m_x -= 3.0f;
	origin.m_y += 5.0f;

	origin.m_x -= 15.0f;
	origin.m_z += 15.0f;

	scene->SetCameraMatrix(rot, origin);
}

#else

/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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
#include "ndTargaToOpenGl.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

#include <iostream>

static ndBodyDynamic* AddRigidBody(ndDemoEntityManager* const scene,
	const ndMatrix& matrix, const ndShapeInstance& shape,
	ndDemoInstanceEntity* const rootEntity, ndFloat32 mass)
{
	ndBodyDynamic* const body = new ndBodyDynamic();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, rootEntity);

	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);

	ndWorld* const world = scene->GetWorld();
	world->AddBody(body);
	return body;
}

class CBasicGhostObject : public ndModel
{
	public:
	CBasicGhostObject(ndDemoEntityManager* const scene)
		:ndModel()
		, m_GhostShape(new ndShapeCone(2.0, 10.0))
		, m_WorldMatrix(dGetIdentityMatrix())
	{
		ndDemoMeshIntance* const geometry = new ndDemoMeshIntance("coneShape", scene->GetShaderCache(), &m_GhostShape, "earthmap.tga", "earthmap.tga", "earthmap.tga");
		ndDemoInstanceEntity* const entity = new ndDemoInstanceEntity(geometry);
		scene->AddEntity(entity);
		ndMatrix mMatrix = dGetIdentityMatrix();
		m_WorldMatrix.m_posit = ndVector(0.0f, 2.1f, 0.0f, 1.0f);
		//m_WorldMatrix.m_posit = ndVector(0.0f, 0.4f, 0.0f, 1.0f);
		AddRigidBody(scene, m_WorldMatrix, m_GhostShape, entity, 0.0);
	}
	virtual void Update(ndWorld* const world, ndFloat32)
	{
		ndBodiesInAabbNotify notifyCallback;
		world->BodiesInAabb(notifyCallback);

		for (ndInt32 iIndex = 0; iIndex < notifyCallback.m_bodyArray.GetCount(); ++iIndex)
		{
			auto otherShape = notifyCallback.m_bodyArray[iIndex]->GetAsBodyKinematic()->GetCollisionShape();
			auto otherMatrix = notifyCallback.m_bodyArray[iIndex]->GetMatrix();

			if (otherShape.GetShape() != m_GhostShape.GetShape())
			{
				ndFixSizeArray<ndContactPoint, 16> contactBuffer;
				ndContactSolver contSolver;
				contSolver.CalculateContacts(&m_GhostShape, m_WorldMatrix, ndVector::m_zero, &otherShape, otherMatrix, ndVector::m_zero, contactBuffer);
				ndInt32 iNumContacts = contactBuffer.GetCount();

				if (iNumContacts > 0)
				{
					std::cout << "Ghost object colliding with other object." << std::endl;
				}
			}
		}
	}

	ndShapeInstance m_GhostShape;
	ndMatrix m_WorldMatrix;
};

void CreateGhostObject(ndDemoEntityManager* const scene)
{
	auto pGhostObject = new CBasicGhostObject(scene);
	scene->GetWorld()->AddModel(pGhostObject);
}

void CreateBoxShape(ndDemoEntityManager* const scene)
{
	ndShapeInstance shapeInstance(new ndShapeBox(1.0, 1.0, 1.0));
	ndDemoMeshIntance* const geometry = new ndDemoMeshIntance("boxShape", scene->GetShaderCache(), &shapeInstance, "earthmap.tga", "earthmap.tga", "earthmap.tga");
	ndDemoInstanceEntity* const entity = new ndDemoInstanceEntity(geometry);
	scene->AddEntity(entity);
	ndMatrix mMatrix = dGetIdentityMatrix();
	mMatrix.m_posit = ndVector(-4.0f, 1.0f, -2.0f, 1.0f);
	//mMatrix.m_posit = ndVector(-4.0f, 0.0f, -2.0f, 1.0f);
	AddRigidBody(scene, mMatrix, shapeInstance, entity, 0.0);
}

void ndBasicCompoundShapeDemo(ndDemoEntityManager* const scene)
{
	CreateGhostObject(scene);
	CreateBoxShape(scene);

	ndVector origin(ndVector::m_zero);
	ndQuaternion rot(dYawMatrix(45.0f * ndDegreeToRad));
	origin.m_x -= 3.0f;
	origin.m_y += 5.0f;

	origin.m_x -= 15.0f;
	origin.m_z += 15.0f;

	scene->SetCameraMatrix(rot, origin);
}


#endif