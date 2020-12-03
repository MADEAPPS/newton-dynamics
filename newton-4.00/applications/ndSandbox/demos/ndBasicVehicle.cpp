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
	//body->SetGyroMode(true);
	body->SetGyroMode(false);

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


	dMatrix tireFrame(dGetIdentityMatrix());
	tireFrame.m_front = dVector(1.0f, 0.0f, 0.0f, 0.0f);
	tireFrame.m_up = dVector(0.0f, 1.0f, 0.0f, 0.0f);
	tireFrame.m_right = tireFrame.m_front.CrossProduct(tireFrame.m_up);
	matrix = tireFrame * tireEntiry->CalculateGlobalMatrix(nullptr);

	//ndJointBallAndSocket* const joint = new ndJointBallAndSocket(matrix, chassis, tireBody);
	ndJointHinge* const joint = new ndJointHinge(matrix, tireBody, chassis);
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
	//chassis;
	AddTireVehicle(scene, chassis, "rr_tire");
	AddTireVehicle(scene, chassis, "rl_tire");
	AddTireVehicle(scene, chassis, "fr_tire");
	AddTireVehicle(scene, chassis, "fl_tire");
}

void ndBasicVehicle (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene);

	dVector location(0.0f, 4.0f, 0.0f, 1.0f);

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = location;
	BuildVehicle(scene, matrix);


	dQuaternion rot;
	//dVector origin(-80.0f, 5.0f, 0.0f, 0.0f);
	dVector origin(-10.0f, 2.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
