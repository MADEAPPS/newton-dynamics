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

class BlackbirdDreamJoint: public dCustomPlane
{
	public:
	BlackbirdDreamJoint (const dVector& pivot, const dVector& normal, NewtonBody* const child)
		:dCustomPlane (pivot, normal, child)
	{
		EnableControlRotation(false);
	}

	void SubmitConstraints (dFloat timestep, int threadIndex)
	{
		dMatrix matrix0;
		dMatrix matrix1;

		// add rows to fix matrix body the the plane
		dCustomPlane::SubmitConstraints(timestep, threadIndex);

		// add one row to prevent body from rotating on the plane of motion
		CalculateGlobalMatrix(matrix0, matrix1);

		//dFloat pitchAngle = CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front);
		NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front), &matrix1.m_front[0]);
		NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front), &matrix1.m_right[0]);
	}
};


static void AttachLimbBody(DemoEntityManager* const scene, const dVector& dir, NewtonBody* const parent, float masse)
{
	NewtonWorld* const world = scene->GetNewton();
	dVector size(0.4f, 0.25f, 0.75f, 0.0f);

	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
	DemoMesh* const geometry = new DemoMesh("box", scene->GetShaderCache(), collision, "smilli.tga", "logo_php.tga", "frowny.tga");

	dMatrix location;
	dVector ppoint;
	NewtonBodyGetMatrix(parent, &location[0][0]);

	location.m_posit += dir.Scale(0.5f);
	location.m_posit.m_y -= 0.5f;

	// make a root body attached to the world
	NewtonBody* const rootBody = CreateSimpleSolid(scene, geometry, masse, location, collision, 0);

	location.m_posit -= dir.Scale(0.5f);
	ppoint = location.m_posit;
	location = dGetIdentityMatrix() * dRollMatrix(90.0f * dDegreeToRad);
	location.m_posit = ppoint;

	dCustomBallAndSocket* const joint = new dCustomBallAndSocket(location, rootBody, parent);
	//hinge->EnableLimits(true);
	joint->EnableTwist(true);
	joint->SetTwistLimits(-30.f * dDegreeToRad, 30.f * dDegreeToRad);
	joint->EnableCone(true);
	joint->SetConeLimits(30.f * dDegreeToRad);

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
	float rootmass = 0.f;


	NewtonBody* const rootBody = CreateSimpleSolid(scene, geometry, rootmass, location, collision, 0);

	// constrain these object to motion on the plane only
	dMatrix matrix;
	NewtonBodyGetMatrix(rootBody, &matrix[0][0]);
	//   new dCustomPlane(matrix.m_posit, matrix.m_front, rootBody);

	// now make some limb body and attach them to the root body
	//   AttachLimbBody (scene, matrix.m_right.Scale ( 1.0f), rootBody);
	AttachLimbBody(scene, matrix.m_right.Scale(-1.0f), rootBody, 1.f);
	//   AttachLimbBody(scene, matrix.m_right.Scale(-1.0f), rootBody, 0.01f);

	location.m_posit.m_z += 2.5f;


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
	DemoMesh* const geometry = new DemoMesh("table", scene->GetShaderCache(), collision, "smilli.tga", "logo_php.tga", "frowny.tga");
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

// demonstration how to set velocity directly 
static void SetBodyAngularVelocity (const NewtonBody* const body, const dVector& desiredOmega, dFloat timestep)
{
	dMatrix bodyInertia;
	dVector bodyOmega(0.0f);

	// get body internal data
	NewtonBodyGetOmega(body, &bodyOmega[0]);
	NewtonBodyGetInertiaMatrix(body, &bodyInertia[0][0]);
	
	// calculate angular velocity error 
	dVector omegaError (desiredOmega - bodyOmega);

	// calculate impulse
	dVector angularImpulse (bodyInertia.RotateVector(omegaError));

	// apply impulse to achieve desired omega
	dVector linearImpulse (0.0f);
	NewtonBodyApplyImpulsePair (body, &linearImpulse[0], &angularImpulse[0], timestep);
}

static void SpecialForceAndTorque (const NewtonBody* body, dFloat timestep, int threadIndex)
{
	dVector omega (0, 5.0f, 0.0f, 0.0f);
	SetBodyAngularVelocity (body, omega, timestep);
	PhysicsApplyGravityForce (body, timestep, threadIndex);
}

static void AddPlayerBodies(DemoEntityManager* const scene, NewtonBody* const floor)
{
	NewtonWorld* const world = scene->GetNewton();
	dVector size(1.0f, 1.0f, 1.0f, 0.0f);

	dMatrix location(dGetIdentityMatrix());

	location.m_posit.m_y = 4.0f;

	dMatrix gitzmoMatrix(dRollMatrix(90.0f * dDegreeToRad));
	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _CAPSULE_PRIMITIVE, 0);
	NewtonCollisionSetMatrix(collision, &gitzmoMatrix[0][0]);

	DemoMesh* const geometry = new DemoMesh("table", scene->GetShaderCache(), collision, "frowny.tga", "logo_php.tga", "smilli.tga");

	location.m_posit.m_z = 15.0f;
	for (int i = 0; i < 4; i++) {
		NewtonBody* const capsule = CreateSimpleSolid(scene, geometry, 10.0f, location, collision, 0);
		if (i == 1) {
			NewtonBodySetForceAndTorqueCallback(capsule, SpecialForceAndTorque);
		}

		// constrain these object to motion on the plane only
		dMatrix matrix;
		NewtonBodyGetMatrix(capsule, &matrix[0][0]);
		new BlackbirdDreamJoint(matrix.m_posit, matrix.m_front, capsule);
		location.m_posit.m_z += 2.5f;
	}

	geometry->Release();
	NewtonDestroyCollision(collision);
}

// create physics scene
void BlackbirdDreamTest(DemoEntityManager* const scene)
{
	scene->CreateSkyBox();

	// make a floor for a 2d world
	NewtonBody* const ground = CreateBackground (scene);

	// add some object constrained to a move on the x plane
	//AddGravityBodies (scene, ground);

	// add players 
	//AddPlayerBodies (scene, ground);

	// add pseudo Rag doll
	AddRagdollBodies(scene, ground);

	// place camera into position
	dMatrix camMatrix (dGetIdentityMatrix());
	dQuaternion rot (camMatrix);
	camMatrix.m_posit.m_y = 5.0f;
	camMatrix.m_posit.m_x = -5.0f;
	camMatrix.m_posit.m_z = 5.0f;

	scene->SetCameraMatrix(rot, camMatrix.m_posit);
}
