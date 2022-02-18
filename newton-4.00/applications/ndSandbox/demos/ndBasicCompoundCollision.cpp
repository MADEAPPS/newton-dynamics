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
#include "ndLoadFbxMesh.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndCompoundScene.h"
#include "ndContactCallback.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndHeightFieldPrimitive.h"
#include "ndMakeProceduralStaticMap.h"

static ndBodyDynamic* AddRigidBody(
	ndDemoEntityManager* const scene,
	const ndMatrix& matrix, const ndShapeInstance& shape,
	ndDemoEntity* const entity, ndFloat32 mass)
{
	ndBodyDynamic* const body = new ndBodyDynamic();

	// add the entity to the scene.
	scene->AddEntity(entity);

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

static void CreateBoxCompoundShape(ndShapeInstance& parentInstance)
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
	//mFloorLocal = dYawMatrix(3.14f / 4.0f);//45 degree
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

static void AddEmptyBox(ndDemoEntityManager* const scene)
{
	ndShapeInstance compoundShapeInstance(new ndShapeCompound());
	CreateBoxCompoundShape(compoundShapeInstance);

	ndDemoMesh* const compGeometry = new ndDemoMesh("compoundShape", scene->GetShaderCache(), &compoundShapeInstance, "earthmap.tga", "earthmap.tga", "earthmap.tga");
	ndDemoEntity* const compEntity = new ndDemoEntity(dGetIdentityMatrix(), nullptr);
	compEntity->SetMesh(compGeometry, dGetIdentityMatrix());

	ndMatrix mBodyMatrix = dGetIdentityMatrix();
	mBodyMatrix.m_posit = ndVector(-2.0f, 5.0f, -5.0f, 1.0f);
	AddRigidBody(scene, mBodyMatrix, compoundShapeInstance, compEntity, 10.0);

	compGeometry->Release();
}

static void AddBowls(ndDemoEntityManager* const scene)
{
	ndDemoEntity* const bowlEntity = scene->LoadFbxMesh("bowl.fbx");
	ndShapeInstance* const compoundShapeInstance = bowlEntity->CreateCompoundFromMesh();

	ndMatrix mOrigMatrix = dGetIdentityMatrix();
	for (ndInt32 i = 0; i < 4; i++)
	{
		ndDemoEntity* const entity = (ndDemoEntity*)bowlEntity->CreateClone();
		mOrigMatrix.m_posit.m_y += 1.0f;
		AddRigidBody(scene, mOrigMatrix, *compoundShapeInstance, entity, 5.0f);
	}

	delete compoundShapeInstance;
	delete bowlEntity;
}

static void AddSphere(ndDemoEntityManager* const scene)
{
	ndShapeInstance originShape(new ndShapeSphere(0.125f));
	ndDemoMesh* const origGeometry = new ndDemoMesh("origShape", scene->GetShaderCache(), &originShape, "earthmap.tga", "earthmap.tga", "earthmap.tga");

	ndDemoEntity* const origEntity = new ndDemoEntity(dGetIdentityMatrix(), nullptr);
	origEntity->SetMesh(origGeometry, dGetIdentityMatrix());

	ndMatrix mOrigMatrix = dGetIdentityMatrix();
	mOrigMatrix.m_posit.m_x = 2.0f;
	for (ndInt32 i = 0; i < 4; i++)
	{
		ndDemoEntity* const entity = (ndDemoEntity*)origEntity->CreateClone();
		mOrigMatrix.m_posit.m_y += 1.0f;
		AddRigidBody(scene, mOrigMatrix, originShape, entity, 1.0);
	}
	origGeometry->Release();
	delete origEntity;
}

void ndBasicCompoundShapeDemo(ndDemoEntityManager* const scene)
{
	ndMatrix heighfieldLocation(dGetIdentityMatrix());
	heighfieldLocation.m_posit.m_x = -200.0f;
	heighfieldLocation.m_posit.m_z = -200.0f;

	// build a floor
	//BuildPlayArena(scene);
	//BuildFlatPlane(scene, true);
	//BuildFloorBox(scene, dGetIdentityMatrix());
	//BuildCompoundScene(scene, dGetIdentityMatrix());
	BuildGridPlane(scene, 120, 4.0f, 0.0f);
	//BuildHeightFieldTerrain(scene, heighfieldLocation);
	//BuildProceduralMap(scene, 120, 4.0f, 0.0f);

	AddEmptyBox(scene);
	AddSphere(scene);
	AddBowls(scene);

	ndVector origin(ndVector::m_zero);
	origin.m_x -= 5.0f;
	origin.m_y += 1.0f;
	origin.m_z += 5.0f;
	ndQuaternion rot(dYawMatrix(45.0f * ndDegreeToRad));
	scene->SetCameraMatrix(rot, origin);
}
