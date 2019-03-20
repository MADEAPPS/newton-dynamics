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
//		dVector m_veloc;
		dVector m_origin;
		NewtonBody* m_plaform;
		dFloat m_speed;
		dFloat m_radius;
		dTree<unsigned, NewtonBody*> m_cargo;
	};

	public:
	KinematiocListener(DemoEntityManager* const scene)
		:dCustomListener(scene->GetNewton(), "Kinematic demo")
		,m_platformList()
		,m_lru(0)
	{
		NewtonWorld* const world = GetWorld();
		NewtonCollision* const boxShape = NewtonCreateBox(world, 1.0f, 1.0f, 1.6f, 0, NULL);
		m_redBox = new DemoMesh("platform", scene->GetShaderCache(), boxShape, "smilli.tga", "smilli.tga", "smilli.tga");
		m_whiteBox = new DemoMesh("platform", scene->GetShaderCache(), boxShape, "smilli.tga", "smilli.tga", "smilli.tga");
		NewtonDestroyCollision(boxShape);

		for (DemoMesh::dListNode* ptr = m_redBox->GetFirst(); ptr; ptr = ptr->GetNext()) {
			DemoSubMesh* const subMesh = &ptr->GetInfo();
			subMesh->m_diffuse.m_x = 1.0f;
			subMesh->m_diffuse.m_y = 0.0f;
			subMesh->m_diffuse.m_z = 0.0f;
			subMesh->m_diffuse.m_w = 1.0f;
		}
		m_redBox->OptimizeForRender();
	}
	
	~KinematiocListener()
	{
		m_redBox->Release();
		m_whiteBox->Release();
	}

	void PreUpdate(dFloat timestep) 
	{
		m_lru++;
		for (dList<KinematicPlatform>::dListNode* ptr = m_platformList.GetFirst(); ptr; ptr = ptr->GetNext()) {
			KinematicPlatform& entry = ptr->GetInfo();

			// set the platform angular velocity
			NewtonBodySetOmega(entry.m_plaform, &entry.m_omega[0]);

			// calculate the speed alone a circular path
			dMatrix matrix;
			NewtonBodyGetMatrix(entry.m_plaform, &matrix[0][0]);
			dVector dir(matrix.m_posit - entry.m_origin);
			dir = dir.Normalize().Scale(entry.m_radius);
			dVector veloc(dir.CrossProduct(dVector(0.0f, -entry.m_speed / entry.m_radius, 0.0f, 0.0f)));

			NewtonBodySetVelocity(entry.m_plaform, &veloc[0]);

			UpdateBodies(entry);
		}
	}

	virtual void PostUpdate(dFloat timestep) 
	{
		for (dList<KinematicPlatform>::dListNode* ptr = m_platformList.GetFirst(); ptr; ptr = ptr->GetNext()) {
			const KinematicPlatform& entry = ptr->GetInfo();
			NewtonBodyIntegrateVelocity (entry.m_plaform, timestep);
		}
	}

	void CreateKinematicTransparentPlatform(const dMatrix& location, dFloat speed)
	{
		KinematicPlatform& entry = AddPlatform(location);
		entry.m_omega = dVector(0.0f, 1.5f, 0.0f, 0.0f);
		entry.m_radius = 5.0f;
		entry.m_speed = speed;
		entry.m_origin = location.m_posit + location.m_front.Scale(entry.m_radius);

		SetTransparent(entry);

		// this is what make the trigger transparent
		NewtonCollision* const collision = NewtonBodyGetCollision(entry.m_plaform);
		NewtonCollisionSetMode(collision, 0);
	}

	void CreateKinematicSolidPlatform(const dMatrix& location, dFloat speed)
	{
		KinematicPlatform& entry = AddPlatform(location);
		entry.m_omega = dVector(0.0f, -0.5f, 0.0f, 0.0f);
		entry.m_radius = 5.0f;
		entry.m_speed = speed;
		entry.m_origin = location.m_posit + location.m_front.Scale (entry.m_radius);
		
		// this is what make the trigger solid like a static body
		NewtonCollision* const collision = NewtonBodyGetCollision(entry.m_plaform);
		NewtonCollisionSetMode(collision, 1);
	}

	private: 
	void UpdateBodies(KinematicPlatform& entry)
	{
		for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(entry.m_plaform); joint; joint = NewtonBodyGetNextContactJoint(entry.m_plaform, joint)) {
			NewtonBody* const body0 = NewtonJointGetBody0(joint);
			NewtonBody* const body1 = NewtonJointGetBody1(joint);
			NewtonBody* const cargoBody = (body0 == entry.m_plaform) ? body1 : body0;
			dTree<unsigned, NewtonBody*>::dTreeNode* node = entry.m_cargo.Find(cargoBody); 
			if (!node) {
				node = entry.m_cargo.Insert(m_lru, cargoBody);
				DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(cargoBody);
				entity->SetMesh(m_redBox, entity->GetMeshMatrix());
			}
			node->GetInfo() = m_lru;
		}

		dTree<unsigned, NewtonBody*>::Iterator iter(entry.m_cargo);
		for (iter.Begin(); iter; ) {
			dTree<unsigned, NewtonBody*>::dTreeNode* const node = iter.GetNode();
			iter++;
			if (node->GetInfo() != m_lru) {
				NewtonBody* const cargoBody = node->GetKey();
				DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(cargoBody);
				entity->SetMesh(m_whiteBox, entity->GetMeshMatrix());
				entry.m_cargo.Remove(node);
			}
		}
	}

	void SetTransparent(KinematicPlatform& entry)
	{
		DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(entry.m_plaform);

		DemoMesh* const mesh = (DemoMesh*)entity->GetMesh();

		for (DemoMesh::dListNode* ptr = mesh->GetFirst(); ptr; ptr = ptr->GetNext()) {
			DemoSubMesh* const subMesh = &ptr->GetInfo();
			subMesh->SetOpacity(0.7f);
		}
		mesh->OptimizeForRender();
	}

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
		NewtonDestroyCollision(box);

		KinematicPlatform& entry = m_platformList.Append()->GetInfo();
		entry.m_plaform = body;
		entry.m_origin = dVector(0.0f);
		entry.m_omega = dVector(0.0f);
		entry.m_radius = 1.0f;
		entry.m_speed = 1.0f;

		return entry;
	}

	dList<KinematicPlatform> m_platformList;
	DemoMesh* m_redBox;
	DemoMesh* m_whiteBox;
	unsigned m_lru;
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
	location.m_posit.m_z -= 5.0f;
	kinematicListener->CreateKinematicSolidPlatform(location, 4.0f);

	location.m_posit.m_z += 10.0f;
	kinematicListener->CreateKinematicTransparentPlatform(location, -5.0f);

	// add some dynamic bodies 
	dMatrix shapeOffsetMatrix(dGetIdentityMatrix());
	dVector size(1.0f, 1.0f, 1.6f, 0.0f);
	int countx = 4;
	int countz = 4;
	AddPrimitiveArray(scene, 1.0f, location.m_posit, size, countx, countz, 6.0f, _BOX_PRIMITIVE, 0, shapeOffsetMatrix);

	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);

//	ExportScene (scene->GetNewton(), "test1.ngd");
}
