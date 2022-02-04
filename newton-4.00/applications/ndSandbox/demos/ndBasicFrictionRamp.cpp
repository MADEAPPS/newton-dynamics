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
#include "ndContactCallback.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

static ndBodyDynamic* AddRigidBody(ndDemoEntityManager* const scene, const ndMatrix& matrix, ndDemoMesh* const geometry, const ndShapeInstance& shape, ndFloat32 mass)
{
	ndBodyDynamic* const body = new ndBodyDynamic();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);

	entity->SetMesh(geometry, dGetIdentityMatrix());
	scene->AddEntity(entity);

	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);
	
	ndWorld* const world = scene->GetWorld();
	world->AddBody(body);
	return body;
}

static void BuildFrictionRamp(ndDemoEntityManager* const scene)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndShapeInstance box(new ndShapeBox(30.0f, 0.25f, 30.f));
	ndMatrix uvMatrix(dGetIdentityMatrix());
	uvMatrix[0][0] *= 0.25f;
	uvMatrix[1][1] *= 0.25f;
	uvMatrix[2][2] *= 0.25f;
	uvMatrix.m_posit = ndVector(-0.5f, -0.5f, 0.0f, 1.0f);
	const char* const textureName = "wood_3.tga";
	ndDemoMesh* const geometry = new ndDemoMesh("box", scene->GetShaderCache(), &box, textureName, textureName, textureName, 1.0f, uvMatrix);

	ndMatrix matrix(dPitchMatrix(30.0f * ndDegreeToRad));
	matrix.m_posit.m_y = 5.0f;
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(geometry, dGetIdentityMatrix());
	geometry->Release();
	
	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(box);
	world->AddBody(body);
	scene->AddEntity(entity);
	
	ndVector boxSize(1.0f, 0.5f, 1.5f, 0.0f);
	matrix.m_posit.m_z -= 10.0f;
	matrix.m_posit.m_x -= 12.0f;
	ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));

	ndShapeInstance shape(new ndShapeBox(boxSize.m_x, boxSize.m_y, boxSize.m_z));
	matrix.m_posit.m_y = floor.m_y + boxSize.m_y;


	ndMatrix texMatrix(dGetIdentityMatrix());
	texMatrix.m_posit.m_x = -0.5f;
	texMatrix.m_posit.m_y = -0.5f;
	const char* const boxTexName = "wood_0.tga";
	ndDemoMesh* const boxGeometry = new ndDemoMesh("box", scene->GetShaderCache(), &shape, boxTexName, boxTexName, boxTexName, 1.0f, texMatrix);

	ndContactCallback* const callback = (ndContactCallback*)world->GetContactNotify();
	for (ndInt32 i = 0; i < 10; i++)
	{
		ndBodyDynamic* const boxBody = AddRigidBody(scene, matrix, boxGeometry, shape, 5.0f);
		matrix.m_posit.m_x += 2.5f;

		// assign material id to the collision shape.
		ndShapeInstance& instanceShape = boxBody->GetCollisionShape();
		ndInt32 newId = i + 1;
		instanceShape.m_shapeMaterial.m_userId = newId;
		//sound play sound only when the relative speed is larger that 10.0f
		instanceShape.m_shapeMaterial.m_userParam[0].m_floatData = 10.0f;

		// register a contact joint physics material pair and 
		// set the physics parameters and application custom options 
		ndMaterial& material = callback->RegisterMaterial(ndApplicationMaterial(), 0, newId);
		ndFloat32 frictionValue = ndFloat32(i) / 15.0f;
		material.m_staticFriction0 = frictionValue;
		material.m_staticFriction1 = frictionValue;
		material.m_dynamicFriction0 = frictionValue;
		material.m_dynamicFriction1 = frictionValue;

		// set the user flag so that it is read in the contact callback for doing stuff
		material.m_userFlags |= ndApplicationMaterial::m_playSound;
	}

	boxGeometry->Release();
}

void ndBasicFrictionRamp (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, dGetIdentityMatrix());

	BuildFrictionRamp(scene);

	ndVector origin1(0.0f, 0.0f, 0.0f, 0.0f);

	ndMatrix camMatrix(dRollMatrix(10.0f * ndDegreeToRad) *  dYawMatrix(20.0f * ndDegreeToRad));
	ndQuaternion rot(camMatrix);
	ndVector origin(-40.0f, 10.0f, 15.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
