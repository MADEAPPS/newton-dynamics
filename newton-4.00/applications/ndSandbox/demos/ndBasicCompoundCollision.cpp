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
	pCompoundShape->BeginAddRemove();
	childInstance.SetLocalMatrix(mLocalMatrix);
	pCompoundShape->AddCollision(&childInstance);
	pCompoundShape->EndAddRemove();
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
	AddToCompoundShape(mWall1Local, parentInstance, wall1);
	AddToCompoundShape(mWall2Local, parentInstance, wall2);
	AddToCompoundShape(mWall3Local, parentInstance, wall3);
	AddToCompoundShape(mWall4Local, parentInstance, wall4);
	AddToCompoundShape(mFloorLocal, parentInstance, floor);
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
	AddRigidBody(scene, mOrigMatrix, originShape, origEntity, 0.0);

	ndVector origin(ndVector::m_zero);
	ndQuaternion rot(dYawMatrix(45.0f * ndDegreeToRad));
	origin.m_x -= 3.0f;
	origin.m_y += 5.0f;

	origin.m_x -= 15.0f;
	origin.m_z += 15.0f;

	scene->SetCameraMatrix(rot, origin);
}