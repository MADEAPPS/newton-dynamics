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
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

class DebugNotify : public ndDemoEntityNotify
{
	public: 
	DebugNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity)
		:ndDemoEntityNotify(manager, entity)
	{
	}

	void OnApplyExternalForce(dInt32 threadIndex, dFloat32 timestep)
	{
		static int xxxx;
		
		ndDemoEntityNotify::OnApplyExternalForce(threadIndex, timestep);
		dVector omega(GetBody()->GetOmega());
		dTrace(("%d wMag = %f  w(%f %f %f)\n", xxxx, dSqrt(omega.DotProduct(omega).GetScalar()), omega.m_x, omega.m_y, omega.m_z));
		xxxx++;
	}
};

static void AddShape(ndDemoEntityManager* const scene,
	ndDemoInstanceEntity* const rootEntity, const ndShapeInstance& sphereShape,
	dFloat32 mass, const dVector& origin, const dFloat32 diameter, int count)
{
	dMatrix matrix(dRollMatrix(90.0f * dDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y + diameter * 0.5f + 7.0f;

	for (dInt32 i = 0; i < count; i++)
	{
		ndBodyDynamic* const body = new ndBodyDynamic();
		ndDemoEntity* const entity = new ndDemoEntity(matrix, rootEntity);

matrix.m_posit.m_y += -6.0f;
matrix = dYawMatrix(30.0f * dDegreeToRad) * dRollMatrix(30.0f * dDegreeToRad) * matrix;


		//body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
		body->SetNotifyCallback(new DebugNotify(scene, entity));
		body->SetMatrix(matrix);
		body->SetCollisionShape(sphereShape);
		body->SetMassMatrix(mass, sphereShape);
		body->SetGyroMode(true);

		body->SetOmega(matrix.m_front.Scale(20.0f));


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
	const int n = 1;
	const int stackHigh = 1;
	//const int n = 10;
	//const int stackHigh = 7;
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

void ndBasicRigidBody (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene);
	//scene->GetWorld()->Load("C://tmp//newton-4.00//applications//ndSandbox//xxx.ngd");

	dVector origin1(0.0f, 0.0f, 0.0f, 0.0f);
	AddCapsulesStacks(scene, origin1);

	dQuaternion rot;
	//dVector origin(-80.0f, 5.0f, 0.0f, 0.0f);
	//dVector origin(-40.0f, 5.0f, 0.0f, 0.0f);
	dVector origin(-20.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
