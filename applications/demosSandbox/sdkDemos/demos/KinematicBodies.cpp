/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "toolbox_stdafx.h"
#include "SkyBox.h"
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "dCustomListener.h"

class KinematiocListener: public dCustomListener
{
	public:
	KinematiocListener(DemoEntityManager* const scene)
		:dCustomListener(scene->GetNewton(), "Kinematic demo")
	{
	}
	
	~KinematiocListener()
	{
	}

	void PreUpdate(dFloat timestep) 
	{
		dVector omega (0.0f, 1.0f, 0.0f, 0.0f);
		NewtonBodySetOmega(m_platform, &omega[0]);
	}

	virtual void PostUpdate(dFloat timestep) 
	{
		NewtonBodyIntegrateVelocity (m_platform, timestep);
	}

	void CreateKinematicPlatform(const dMatrix& location)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

		NewtonCollision* const box = NewtonCreateBox(world, 4.0f, 0.25f, 3.0f, 0, NULL);
		NewtonBody* const body = NewtonCreateKinematicBody(world, box, &location[0][0]);

		NewtonBodySetTransformCallback(body, DemoEntity::TransformCallback);
		DemoMesh* const geometry = new DemoMesh("platform", scene->GetShaderCache(), box, "wood_4.tga", "wood_4.tga", "wood_1.tga");
		DemoEntity* const entity = new DemoEntity(location, NULL);
		scene->Append(entity);
		entity->SetMesh(geometry, dGetIdentityMatrix());
		geometry->Release();

		NewtonBodySetUserData(body, entity);

		// players must have weight, otherwise they are infinitely strong when they collide
//		NewtonCollision* const shape = NewtonBodyGetCollision(m_body);
//		NewtonBodySetMassProperties(m_body, mass, shape);
//		NewtonBodySetCollidable(m_body, true);

		NewtonDestroyCollision(box);

		m_platform = body;
	}

//	dList<NewtonBody*> m_platformList;
	NewtonBody* m_platform;
};

void KinematicBodies (DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	CreateLevelMesh (scene, "flatPlane.ngd", true);

	dVector origin(-10.0f, 5.0f, 0.0f, 1.0f);
	KinematiocListener* const kinematicListener = new KinematiocListener(scene);

	dMatrix location (dGetIdentityMatrix());
	location.m_posit = origin;

	location.m_posit.m_x += 15.0f;
	location.m_posit.m_y -= 4.5f;
	kinematicListener->CreateKinematicPlatform(location);

	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);

//	ExportScene (scene->GetNewton(), "test1.ngd");
}
