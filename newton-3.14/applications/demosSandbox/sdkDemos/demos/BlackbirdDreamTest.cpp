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
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"
#include "toolBox/OpenGlUtil.h"


static void AttachLimbBody(DemoEntityManager* const scene, const dVector& dir, NewtonBody* const parent, float masse)
{
	NewtonWorld* const world = scene->GetNewton();
	dVector size(0.4f, 0.25f, 0.75f, 0.0f);

	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
	DemoMesh* const geometry = new DemoMesh("box", scene->GetShaderCache(), collision, "smilli.tga", "logo_php.tga", "frowny.tga");

	dMatrix location;
	NewtonBodyGetMatrix(parent, &location[0][0]);

	location.m_posit += dir.Scale(0.5f);
	location.m_posit.m_y -= 0.5f;

	// make a root body attached to the world
	NewtonBody* const rootBody = CreateSimpleSolid(scene, geometry, masse, location, collision, 0);

	// constrain these object to motion on the plane only
	//   location.m_posit -= dir.Scale (0.5f);
	location.m_posit -= dir.Scale(0.5f);

	dCustomHinge* const hinge = new dCustomHinge(location, rootBody, parent);
	//hinge->EnableLimits(true);
	hinge->SetLimits(-45.0f * dDegreeToRad, 45.0f * dDegreeToRad);
	hinge->SetAsSpringDamper(true, 500.f, 10.f);
	//hinge->SetMassIndependentSpringDamper(true, 0.3f, 500.f, 10.f);

	geometry->Release();
	NewtonDestroyCollision(collision);
}


static void AddRagdollBodies(DemoEntityManager* const scene, NewtonBody* const floor)
{
	NewtonWorld* const world = scene->GetNewton();
	dVector size(0.2f, 1.0f, 0.2f, 0.0f);

	dMatrix location(dGetIdentityMatrix());

	location.m_posit.m_y = 4.0f;

	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
	DemoMesh* const geometry = new DemoMesh("box", scene->GetShaderCache(), collision, "smilli.tga", "logo_php.tga", "frowny.tga");

	location.m_posit.m_z = 5.0f;
	float rootmass = 10.f;
	for (int i = 0; i < 2; i++) {
		// make a root body attached to the world

		//if (i == 3) { rootmass = 1000.f; }

		NewtonBody* const rootBody = CreateSimpleSolid(scene, geometry, rootmass, location, collision, 0);
		new dCustomSixdof (location, rootBody, NULL);

		// constrain these object to motion on the plane only
		dMatrix matrix;
		NewtonBodyGetMatrix(rootBody, &matrix[0][0]);
		//   new dCustomPlane(matrix.m_posit, matrix.m_front, rootBody);

		// now make some limb body and attach them to the root body
		//   AttachLimbBody (scene, matrix.m_right.Scale ( 1.0f), rootBody);
		AttachLimbBody(scene, matrix.m_right.Scale(-1.0f), rootBody, 10.0f * i + 0.01f);
		//   AttachLimbBody(scene, matrix.m_right.Scale(-1.0f), rootBody, 0.01f);

		location.m_posit.m_z += 2.5f;
	}

	geometry->Release();
	NewtonDestroyCollision(collision);
}


static NewtonBody* CreateBackground (DemoEntityManager* const scene)
{
	NewtonWorld* const world = scene->GetNewton();
	dVector tableSize(10.0f, 2.0f, 200.0f, 0.0f);

	// create the shape and visual mesh as a common data to be re used
	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), tableSize, _BOX_PRIMITIVE, 0);

	DemoMesh* const geometry = new DemoMesh("table", scene->GetShaderCache(), collision, NULL, "wood_3.tga", NULL);

	dMatrix matrix (dGetIdentityMatrix());
	NewtonBody* const tableBody = CreateSimpleSolid(scene, geometry, 0.0f, matrix, collision, 0);

	geometry->Release();
	NewtonDestroyCollision (collision);

	return tableBody;
}


// create physics scene
void BlackbirdDreamTest(DemoEntityManager* const scene)
{
	scene->CreateSkyBox();

	// make a floor for a 2d world
	NewtonBody* const ground = CreateBackground (scene);

	// add pseudo Rag doll
	AddRagdollBodies(scene, ground);

	// place camera into position
	dMatrix camMatrix (dGetIdentityMatrix());
	dQuaternion rot (camMatrix);
	camMatrix.m_posit.m_y = 5.0f;
	camMatrix.m_posit.m_z = 5.0f;
	camMatrix.m_posit.m_x = -10.0f;

	scene->SetCameraMatrix(rot, camMatrix.m_posit);
}



