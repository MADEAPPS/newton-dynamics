/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

class ndTireNotifyNotify : public ndDemoEntityNotify
{
	public:
	ndTireNotifyNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity, ndBodyDynamic* const chassis)
		:ndDemoEntityNotify(manager, entity)
		,m_chassis(chassis)
	{
	}

	void OnTranform(dInt32 threadIndex, const dMatrix& matrix)
	{
		dMatrix parentMatrix(m_chassis->GetMatrix());
		dMatrix localMatrix(matrix * parentMatrix.Inverse());

		dQuaternion rot(localMatrix);
		dScopeSpinLock lock(m_entity->GetLock());
		m_entity->SetMatrixUsafe(rot, localMatrix.m_posit);
	}

	ndBodyDynamic* m_chassis;
};

/*
static void AddShape(ndDemoEntityManager* const scene,
	ndDemoInstanceEntity* const rootEntity, const ndShapeInstance& sphereShape,
	dFloat32 mass, const dVector& origin, const dFloat32 diameter, int count)
{
	dMatrix matrix(dRollMatrix(90.0f * dDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	//matrix.m_posit.m_y = floor.m_y + diameter * 0.5f + 7.0f;
	matrix.m_posit.m_y = floor.m_y + diameter * 0.5f + 7.0f;

	for (dInt32 i = 0; i < count; i++)
	{
		ndBodyDynamic* const body = new ndBodyDynamic();
		ndDemoEntity* const entity = new ndDemoEntity(matrix, rootEntity);

		body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
		body->SetMatrix(matrix);
		body->SetCollisionShape(sphereShape);
		body->SetMassMatrix(mass, sphereShape);
		body->SetGyroMode(true);

		world->AddBody(body);
		matrix.m_posit.m_y += diameter * 2.5f;
	}
}

static void AddCapsulesStacks(ndDemoEntityManager* const scene, const dVector& origin)
{
	dFloat32 diameter = 1.0f;
	ndShapeInstance shape(new ndShapeCapsule(diameter * 0.5f, diameter * 0.5f, diameter * 1.0f));
	ndDemoMeshIntance* const instanceMesh = new ndDemoMeshIntance("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

	ndDemoInstanceEntity* const rootEntity = new ndDemoInstanceEntity(instanceMesh);
	scene->AddEntity(rootEntity);

	//const int n = 2;
	//const int stackHigh = 7;
	//const int n = 1;
	//const int stackHigh = 1;
	const int n = 10;
	const int stackHigh = 7;
	for (dInt32 i = 0; i < n; i++)
	{
		for (dInt32 j = 0; j < n; j++)
		{
			dVector location((j - n / 2) * 4.0f, 0.0f, (i - n / 2) * 4.0f, 0.0f);
			AddShape(scene, rootEntity, shape, 10.0f, location + origin, diameter, stackHigh);
		}
	}

	instanceMesh->Release();
}
*/

static ndBodyDynamic* CreateChassis(ndDemoEntityManager* const scene, fbxDemoEntity* const chassisEntity)
{
	dFloat32 mass = 1000.0f;
	dMatrix matrix(chassisEntity->CalculateGlobalMatrix(nullptr));

	ndWorld* const world = scene->GetWorld();
	ndShapeInstance* const chassisCollision = chassisEntity->CreateCollisionFromchildren(scene->GetWorld());

	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, chassisEntity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(*chassisCollision);
	body->SetMassMatrix(mass, *chassisCollision);
	body->SetGyroMode(true);

	world->AddBody(body);
	delete chassisCollision;
	return body;
}

static void CalculateTireDimensions(const char* const tireName, dFloat32& width, dFloat32& radius, ndWorld* const world, ndDemoEntity* const vehEntity)
{
	// find the the tire visual mesh 
	ndDemoEntity* const tirePart = vehEntity->Find(tireName);
	dAssert(tirePart);
	
	// make a convex hull collision shape to assist in calculation of the tire shape size
	ndDemoMesh* const tireMesh = (ndDemoMesh*)tirePart->GetMesh();

	dArray<dVector> temp;
	tireMesh->GetVertexArray(temp);

	dVector minVal(1.0e10f);
	dVector maxVal(-1.0e10f);
	for (dInt32 i = 0; i < temp.GetCount(); i++)
	{
		minVal = minVal.GetMin(temp[i]);
		maxVal = maxVal.GetMax(temp[i]);
	}

	dVector size(maxVal - minVal);
	width = size.m_y;
	radius = size.m_x * 0.5f;
}

static ndBodyDynamic* AddTireVehicle(ndDemoEntityManager* const scene, ndBodyDynamic* const chassis, const char* const tireName)
{
	//rr_tire
	dFloat32 width;
	dFloat32 radius;
	ndWorld* const world = scene->GetWorld();
	ndDemoEntity* const chassisEntity = (ndDemoEntity*)chassis->GetNotifyCallback()->GetUserData();
	CalculateTireDimensions(tireName, width, radius, world, chassisEntity);

	//m_tireShape = NewtonCreateChamferCylinder(world, 0.75f, 0.5f, 0, NULL);

	dFloat32 mass(20.0f);
	ndShapeInstance tireCollision (new ndShapeSphere(width));

	ndDemoEntity* const tireEntiry = chassisEntity->Find(tireName);
	dMatrix matrix(tireEntiry->CalculateGlobalMatrix(nullptr));
	ndBodyDynamic* const tireBody = new ndBodyDynamic();
	tireBody->SetNotifyCallback(new ndTireNotifyNotify(scene, tireEntiry, chassis));
	tireBody->SetMatrix(matrix);
	tireBody->SetCollisionShape(tireCollision);
	tireBody->SetMassMatrix(mass, tireCollision);
	//tireBody->SetGyroMode(false);

	world->AddBody(tireBody);

	ndJointBallAndSocket* const joint = new ndJointBallAndSocket(matrix, chassis, tireBody);
	world->AddJoint(joint);

	return tireBody;
}

static void BuildVehicle(ndDemoEntityManager* const scene, const dMatrix& matrix)
{
	//fbxDemoEntity* const vehicleEntity = LoadFbxMesh("viper.fbx");
	fbxDemoEntity* const vehicleEntity = LoadFbxMesh("viper1.fbx");
	vehicleEntity->ResetMatrix(*scene, vehicleEntity->CalculateGlobalMatrix() * matrix);
	vehicleEntity->BuildRenderMeshes(scene);
	scene->AddEntity(vehicleEntity);

	ndBodyDynamic* const chassis = CreateChassis(scene, vehicleEntity);

	AddTireVehicle(scene, chassis, "rr_tire");
	//AddTireVehicle(scene, chassis, "rl_tire");
	//AddTireVehicle(scene, chassis, "fr_tire");
	//AddTireVehicle(scene, chassis, "fl_tire");
}

void ndBasicVehicle (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene);

	dVector origin1(0.0f, 0.0f, 0.0f, 0.0f);
//	AddCapsulesStacks(scene, origin1);

	dMatrix matrix(dGetIdentityMatrix());
	matrix = dRollMatrix(45.0f * dDegreeToRad) * dPitchMatrix(45.0f * dDegreeToRad);

	matrix.m_posit = dVector(0.0f, 4.0f, 0.0f, 1.0f);
	BuildVehicle(scene, matrix);


	dQuaternion rot;
	//dVector origin(-80.0f, 5.0f, 0.0f, 0.0f);
	dVector origin(-10.0f, 2.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
