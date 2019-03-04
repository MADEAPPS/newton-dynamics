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
	struct KinematicPlatform
	{
		dVector m_omega;
		dVector m_veloc;
		NewtonBody* m_plaform;
	};

	public:
	KinematiocListener(DemoEntityManager* const scene)
		:dCustomListener(scene->GetNewton(), "Kinematic demo")
		,m_platformList()
	{
	}
	
	~KinematiocListener()
	{
	}

	void PreUpdate(dFloat timestep) 
	{
		for (dList<KinematicPlatform>::dListNode* ptr = m_platformList.GetFirst(); ptr; ptr = ptr->GetNext()) {
			const KinematicPlatform& entry = ptr->GetInfo();

			NewtonBodySetOmega(entry.m_plaform, &entry.m_omega[0]);
			NewtonBodySetVelocity(entry.m_plaform, &entry.m_veloc[0]);
		}
	}

	virtual void PostUpdate(dFloat timestep) 
	{
		for (dList<KinematicPlatform>::dListNode* ptr = m_platformList.GetFirst(); ptr; ptr = ptr->GetNext()) {
			const KinematicPlatform& entry = ptr->GetInfo();

			NewtonBodyIntegrateVelocity (entry.m_plaform, timestep);
		}
	}

	void CreateKinematicTransparentPlatform(const dMatrix& location)
	{
		KinematicPlatform& entry = AddPlatform(location);
		entry.m_omega = dVector(0.0f, 1.0f, 0.0f, 0.0f);

		DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(entry.m_plaform);

		DemoMesh* const mesh = (DemoMesh*)entity->GetMesh();

		for (DemoMesh::dListNode* ptr = mesh->GetFirst(); ptr; ptr = ptr->GetNext()) {
			DemoSubMesh* const subMesh = &ptr->GetInfo();
			subMesh->SetOpacity(0.7f);
		}
		mesh->OptimizeForRender();
	}

	private: 
	KinematicPlatform& AddPlatform(const dMatrix& location)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

		NewtonCollision* const box = NewtonCreateBox(world, 5.0f, 0.25f, 4.0f, 0, NULL);
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

		KinematicPlatform& entry = m_platformList.Append()->GetInfo();
		entry.m_plaform = body;
		entry.m_veloc = dVector(0.0f);
		entry.m_omega = dVector(0.0f);

		return entry;
	}



	dList<KinematicPlatform> m_platformList;
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
	location.m_posit.m_y -= 4.25f;
	kinematicListener->CreateKinematicTransparentPlatform(location);


	// add some dynamic bodies 
	dMatrix shapeOffsetMatrix(dGetIdentityMatrix());
	dVector size(1.0f, 1.0f, 1.6f, 0.0f);
	int count = 3;
	AddPrimitiveArray(scene, 1.0f, location.m_posit, size, count, count, 6.0f, _BOX_PRIMITIVE, 0, shapeOffsetMatrix);

	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);

//	ExportScene (scene->GetNewton(), "test1.ngd");
}
