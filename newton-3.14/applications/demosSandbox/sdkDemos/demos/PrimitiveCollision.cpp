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

#include "toolbox_stdafx.h"
#include "SkyBox.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"
#include "OpenGlUtil.h"


class ShowCollisionCollide: public DemoEntity::UserData
{
	public:
	ShowCollisionCollide (NewtonBody* body)
		:m_body(body)
	{
	}

	virtual void OnRender (dFloat timestep) const
	{
		dVector p0(0.0f);
		dVector p1(0.0f);
		DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(m_body);
		const dMatrix& matrix = entity->GetRenderMatrix();
		NewtonCollision* const collision = NewtonBodyGetCollision(m_body);
		CalculateAABB (collision, matrix, p0, p1);

		NewtonWorld* const world = NewtonBodyGetWorld(m_body);
		NewtonWorldForEachBodyInAABBDo(world, &p0[0], &p1[0], CalculateContacts, (void*)this);
	}

	static int CalculateContacts (const NewtonBody* const otherBody, void* const userData)
	{
		ShowCollisionCollide* const me = (ShowCollisionCollide*)userData;
		if (me->m_body != otherBody) {
			const int cMaxContacts = 15;
			dFloat contacts[cMaxContacts][3];
			dFloat normals[cMaxContacts][3];
			dFloat penetrations[cMaxContacts];
			dLong attributeA[cMaxContacts];
			dLong attributeB[cMaxContacts];
			NewtonWorld* const world = NewtonBodyGetWorld(otherBody);

			//NewtonBodyGetMatrix(me->m_body, &matrixA[0][0]);
			//NewtonBodyGetMatrix(otherBody, &matrixB[0][0]);
			DemoEntity* const entityA = (DemoEntity*)NewtonBodyGetUserData(me->m_body);
			DemoEntity* const entityB = (DemoEntity*)NewtonBodyGetUserData(otherBody);

			const dMatrix& matrixA = entityA->GetRenderMatrix (); 
			const dMatrix& matrixB = entityB->GetRenderMatrix ();  


			NewtonCollision* const collisionA = NewtonBodyGetCollision(me->m_body);
			NewtonCollision* const collisionB = NewtonBodyGetCollision(otherBody);

			int count = NewtonCollisionCollide (world, cMaxContacts, collisionA, &matrixA[0][0], collisionB, &matrixB[0][0], 
												&contacts[0][0], &normals[0][0], penetrations, attributeA, attributeB, 0);

			dVector originColor (1.0f, 0.0f, 0.0f, 0.0f);
			dVector lineColor (0.0f, 0.0f, 1.0f, 0.0f);
			for (int i = 0; i < count; i ++) {

				dVector n (normals[i][0], normals[i][1], normals[i][2], 0.0f);
				dVector p0 (contacts[i][0], contacts[i][1], contacts[i][2], 0.0f);
				dVector p1 (p0 + n.Scale (0.5f));
				p0 = matrixA.UntransformVector(p0);
				p1 = matrixA.UntransformVector(p1);
				ShowMousePicking (p0, p1, originColor, lineColor);
			}
		}
		return 1;
	}

	NewtonBody* m_body;
};



static void PhysicsSpinBody (const NewtonBody* body, dFloat timestep, int threadIndex)
{
	dVector omega (0.0f, 0.f, 1.0f, 0.0f);
	NewtonBodySetOmega (body, &omega[0]);
}



static void AddSinglePrimitive (DemoEntityManager* const scene, dFloat x, PrimitiveType type, int materialID)
{

	dVector size(0.5f, 0.5f, 0.75f, 0.0f);
	dMatrix matrix (dGetIdentityMatrix());

	// create the shape and visual mesh as a common data to be re used
	NewtonWorld* const world = scene->GetNewton();
	NewtonCollision* const collision = CreateConvexCollision (world, dGetIdentityMatrix(), size, type, materialID);

	//	DemoMesh____* const geometry = new DemoMesh____("cylinder_1", collision, "wood_0.tga", "wood_0.tga", "wood_1.tga");
	DemoMesh* const geometry = new DemoMesh("cylinder_1", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");


	matrix = dRollMatrix(dPi/2.0f);
	matrix.m_posit.m_x = 0;
	matrix.m_posit.m_z = x;
	matrix.m_posit.m_y = 0;
	CreateSimpleSolid (scene, geometry, 1.0, matrix, collision, materialID);

	// do not forget to release the assets	
	geometry->Release(); 
	NewtonDestroyCollision (collision);
}





// create physics scene
void PrimitiveCollision (DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();


	// customize the scene after loading
	// set a user friction variable in the body for variable friction demos
	// later this will be done using LUA script
	NewtonWorld* const world = scene->GetNewton();
	dMatrix offsetMatrix (dGetIdentityMatrix());

	int materialID = NewtonMaterialGetDefaultGroupID (world);

	// disable collision
	NewtonMaterialSetDefaultCollidable (world, materialID, materialID, 0);

	AddSinglePrimitive (scene, -10.0f, _SPHERE_PRIMITIVE, materialID);
	AddSinglePrimitive (scene,  -8.0f, _BOX_PRIMITIVE, materialID);
	AddSinglePrimitive (scene,  -6.0f, _CAPSULE_PRIMITIVE, materialID);
	AddSinglePrimitive (scene,  -4.0f, _CYLINDER_PRIMITIVE, materialID);
	AddSinglePrimitive (scene,  -2.0f, _CONE_PRIMITIVE, materialID);
	AddSinglePrimitive (scene,   4.0f, _CHAMFER_CYLINDER_PRIMITIVE, materialID);
	AddSinglePrimitive (scene,   6.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, materialID);
	AddSinglePrimitive (scene,   8.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, materialID);


	// here we will change the standard gravity force callback and apply null and add a 
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		DemoEntity* const entity = (DemoEntity*) NewtonBodyGetUserData(body);
		entity->SetUserData (new ShowCollisionCollide(body));
		NewtonBodySetForceAndTorqueCallback(body, PhysicsSpinBody);
		NewtonBodySetAutoSleep (body, 0);
	}

	// place camera into position
	//dMatrix camMatrix (dYawMatrix(90.0f * dDegreeToRad));
	dMatrix camMatrix (dGetIdentityMatrix());
	dQuaternion rot (camMatrix);
	dVector origin (-15.0f, 0.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);

}




