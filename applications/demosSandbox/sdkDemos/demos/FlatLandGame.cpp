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

class Plane2dUpVector: public dCustomPlane
{
	public:
	Plane2dUpVector (const dVector& pivot, const dVector& normal, NewtonBody* const child)
		:dCustomPlane (pivot, normal, child)
	{
	}

	void SubmitConstraints (dFloat timestep, int threadIndex)
	{
		// add one row to prevent body from rotating on the plane of motion
		dMatrix matrix0;
		dMatrix matrix1;
		CalculateGlobalMatrix(matrix0, matrix1);

		dFloat pitchAngle = CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front);
		NewtonUserJointAddAngularRow(m_joint, -pitchAngle, &matrix1.m_front[0]);

		// add rows to fix matrix body the the plane
		dCustomPlane::SubmitConstraints(timestep, threadIndex);
	}
};

static void AddPlayerBodies (DemoEntityManager* const scene, NewtonBody* const floor)
{

	NewtonWorld* const world = scene->GetNewton();
	dVector size(1.0f, 1.0f, 1.0f, 0.0f);

	dMatrix location(dGetIdentityMatrix());

	location.m_posit.m_y = 4.0f;

	dMatrix gitzmoMatrix (dRollMatrix(90.0f * dDegreeToRad));
	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _CAPSULE_PRIMITIVE, 0);
	NewtonCollisionSetMatrix(collision, &gitzmoMatrix[0][0]);

	DemoMesh* const geometry = new DemoMesh("table", scene->GetShaderCache(), collision, "frowny.tga", "logo_php.tga", "smilli.tga");

	location.m_posit.m_z = 10.0f;
	for (int i = 0; i < 4; i++) {
		NewtonBody* const box = CreateSimpleSolid(scene, geometry, 10.0f, location, collision, 0);

		// constrain these object to motion on the plane only
		dMatrix matrix;
		NewtonBodyGetMatrix(box, &matrix[0][0]);
		new Plane2dUpVector(matrix.m_posit, matrix.m_front, box);
		location.m_posit.m_z += 2.5f;
	}

	geometry->Release();
	NewtonDestroyCollision(collision);
}

static void AddGravityBodies (DemoEntityManager* const scene, NewtonBody* const floor)
{
	NewtonWorld* const world = scene->GetNewton();
	dVector size(0.5f, 0.5f, 0.5f, 0.0f);

	dMatrix location (dGetIdentityMatrix());

	location.m_posit.m_y = 4.0f;
	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
	DemoMesh* const geometry = new DemoMesh("table", scene->GetShaderCache(), collision, "frowny.tga", "logo_php.tga", "smilli.tga");
	for (int i = 0; i < 4; i ++) {
		location.m_posit.m_z = -5.0f;
		for (int j = 0; j < 4; j ++) {
			NewtonBody* const box = CreateSimpleSolid(scene, geometry, 10.0f, location, collision, 0);

			// constrain these object to motion on the plane only
			dMatrix matrix; 
			NewtonBodyGetMatrix (box, &matrix[0][0]);
			new dCustomPlane (matrix.m_posit, matrix.m_front, box);

			location.m_posit.m_z += 2.5f;
		}
		location.m_posit.m_y += 2.0f;
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
void FlatLandGame (DemoEntityManager* const scene)
{
	scene->CreateSkyBox();

	// make a floor for a 2d world
	NewtonBody* const ground = CreateBackground (scene);

	// add some object constrained to a move on the x plane
	AddGravityBodies (scene, ground);

	// add players 
	AddPlayerBodies (scene, ground);

	// place camera into position
	dMatrix camMatrix (dGetIdentityMatrix());
	dQuaternion rot (camMatrix);
	camMatrix.m_posit.m_y = 10.0f;
	camMatrix.m_posit.m_x = -30.0f;
	scene->SetCameraMatrix(rot, camMatrix.m_posit);
}
