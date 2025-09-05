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
#include "ndMeshLoader.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndCompoundScene.h"
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndHeightFieldPrimitive.h"
#include "ndMakeProceduralStaticMap.h"

static ndBodyDynamic* AddRigidBody(
	ndDemoEntityManager* const scene,
	const ndMatrix& matrix, const ndShapeInstance& shape,
	const ndSharedPtr<ndDemoEntity>& entity, ndFloat32 mass)
{
	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(shape);
	body->GetAsBodyDynamic()->SetMassMatrix(mass, shape);

	ndWorld* const world = scene->GetWorld();

	world->AddBody(body);
	scene->AddEntity(entity);
	return body->GetAsBodyDynamic();
}

static void AddToCompoundShape(const ndMatrix& mLocalMatrix, ndShapeInstance& parentShape, ndShapeInstance& childInstance)
{
	ndShapeCompound* const compoundShape = parentShape.GetShape()->GetAsShapeCompound();
	childInstance.SetLocalMatrix(mLocalMatrix);
	compoundShape->AddCollision(&childInstance);
}

static void CreateBoxCompoundShape(ndShapeInstance& parentInstance)
{
	ndShapeInstance wall1(new ndShapeBox(2.0f, 2.0f, 0.1f));
	ndShapeInstance wall2(new ndShapeBox(0.1f, 2.0f, 2.0f));
	ndShapeInstance wall3(new ndShapeBox(2.0f, 2.0f, 0.1f));
	ndShapeInstance wall4(new ndShapeBox(0.1f, 2.0f, 2.0f));
	ndShapeInstance floor(new ndShapeBox(2.0f, 0.1f, 2.0f));
	ndMatrix mWall1Local = ndGetIdentityMatrix();
	mWall1Local.m_posit = ndVector(0.0f, 0.0f, -1.0f, 1.0f);
	ndMatrix mWall2Local = ndGetIdentityMatrix();
	mWall2Local.m_posit = ndVector(1.0f, 0.0f, 0.0f, 1.0f);
	ndMatrix mWall3Local = ndGetIdentityMatrix();
	mWall3Local.m_posit = ndVector(0.0f, 0.0f, 1.0f, 1.0f);
	ndMatrix mWall4Local = ndGetIdentityMatrix();
	mWall4Local.m_posit = ndVector(-1.0f, 0.0f, 0.0f, 1.0f);
	ndMatrix mFloorLocal = ndGetIdentityMatrix();
	//mFloorLocal = ndYawMatrix(3.14f / 4.0f);//45 degree
	mFloorLocal.m_posit = ndVector(0.0f, -1.0f, 0.0f, 1.0f);

	ndShapeCompound* const compoundShape = parentInstance.GetShape()->GetAsShapeCompound();
	compoundShape->BeginAddRemove();
	AddToCompoundShape(mWall1Local, parentInstance, wall1);
	AddToCompoundShape(mWall2Local, parentInstance, wall2);
	AddToCompoundShape(mWall3Local, parentInstance, wall3);
	AddToCompoundShape(mWall4Local, parentInstance, wall4);
	AddToCompoundShape(mFloorLocal, parentInstance, floor);
	compoundShape->EndAddRemove();
}

static void AddSphere(ndDemoEntityManager* const scene)
{
	ndShapeInstance originShape(new ndShapeSphere(0.125f));
	ndSharedPtr<ndDemoMeshInterface> origGeometry (new ndDemoMesh("origShape", scene->GetShaderCache(), &originShape, "earthmap.png", "earthmap.png", "earthmap.png"));

	ndSharedPtr<ndDemoEntity>origEntity(new ndDemoEntity(ndGetIdentityMatrix()));
	origEntity->SetMesh(origGeometry);

	ndMatrix mOrigMatrix = ndGetIdentityMatrix();
	mOrigMatrix.m_posit.m_x = 2.0f;
	for (ndInt32 i = 0; i < 4; ++i)
	{
		ndSharedPtr<ndDemoEntity> entity (origEntity->CreateClone());
		ndVector floor(FindFloor(*scene->GetWorld(), mOrigMatrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		mOrigMatrix.m_posit.m_y = floor.m_y + 1.0f;
		AddRigidBody(scene, mOrigMatrix, originShape, entity, 1.0);
	}
}

static void AddEmptyBox(ndDemoEntityManager* const scene)
{
	ndShapeInstance compoundShapeInstance(new ndShapeCompound());
	CreateBoxCompoundShape(compoundShapeInstance);

	ndSharedPtr<ndDemoMeshInterface> compGeometry (new ndDemoMesh("compoundShape", scene->GetShaderCache(), &compoundShapeInstance, "earthmap.png", "earthmap.png", "earthmap.png"));
	ndSharedPtr<ndDemoEntity>compEntity(new ndDemoEntity(ndGetIdentityMatrix()));
	compEntity->SetMesh(compGeometry);

	ndMatrix mBodyMatrix = ndGetIdentityMatrix();
	mBodyMatrix.m_posit = ndVector(-2.0f, 5.0f, -5.0f, 1.0f);
	ndVector floor(FindFloor(*scene->GetWorld(), mBodyMatrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	mBodyMatrix.m_posit.m_y = floor.m_y + 1.5f;

	AddRigidBody(scene, mBodyMatrix, compoundShapeInstance, compEntity, 10.0);
}

static void AddSimpleConcaveMesh(ndDemoEntityManager* const scene, const ndMatrix& matrix, const char* const meshName, int count = 1)
{
	ndMeshLoader loader;
	ndSharedPtr<ndDemoEntity> rootEntity (loader.LoadEntity(meshName, scene));
	ndSharedPtr<ndDemoEntity> childEntity (rootEntity->GetChildren().GetFirst()->GetInfo());
	ndSharedPtr<ndShapeInstance>compoundShapeInstance (childEntity->CreateCompoundFromMesh());
	
	ndMatrix originMatrix (matrix);
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndDemoEntity* const entity = childEntity->CreateClone();
		originMatrix.m_posit.m_z += 2.0f;
		ndVector floor(FindFloor(*scene->GetWorld(), originMatrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		originMatrix.m_posit.m_y = floor.m_y + 2.0f;
		AddRigidBody(scene, originMatrix, *(*compoundShapeInstance), entity, 5.0f);
	}
}

void ndBasicCompoundShapeDemo(ndDemoEntityManager* const scene)
{
	ndMatrix heighfieldLocation(ndGetIdentityMatrix());
	heighfieldLocation.m_posit.m_x = -200.0f;
	heighfieldLocation.m_posit.m_z = -200.0f;

	// build a floor
	//BuildPlayArena(scene);
	BuildFlatPlane(scene, true);
	//BuildFloorBox(scene, ndGetIdentityMatrix());
	//BuildCompoundScene(scene, ndGetIdentityMatrix());
	//BuildGridPlane(scene, 120, 4.0f, 0.0f);
	//BuildHeightFieldTerrain(scene, heighfieldLocation);
	//BuildProceduralMap(scene, 120, 4.0f, 0.0f);

	ndMatrix location(ndGetIdentityMatrix());

	AddSphere(scene);
	AddEmptyBox(scene);

	location.m_posit.m_y = 0.5f;
	location.m_posit.m_z = -3.0f;
	//AddSimpleConcaveMesh(scene, location, "bowl.fbx", 4);
	AddSimpleConcaveMesh(scene, location, "bowl.fbx", 1);

	location.m_posit.m_z = -5.0f;
	AddSimpleConcaveMesh(scene, location, "testConcave.fbx", 1);
	
	location.m_posit.m_x = 5.0f;
	location.m_posit.m_z = -2.0f;
	location.m_posit.m_y = 1.7f;
	AddSimpleConcaveMesh(scene, location, "camel.fbx", 4);
	
	location.m_posit.m_x = 10.0f;
	location.m_posit.m_z = 5.0f;
	location.m_posit.m_y = 2.0f;
	AddSimpleConcaveMesh(scene, location, "dino.fbx", 4);

	ndVector origin(ndVector::m_zero);
	origin.m_x -= 10.0f;
	origin.m_y += 2.0f;
	origin.m_z += 10.0f;
	origin.m_w = 1.0f;
	ndQuaternion rot(ndYawMatrix(45.0f * ndDegreeToRad));
	scene->SetCameraMatrix(rot, origin);
}
