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


/* Continuous Collision Detection Test Demo
*
*  <2014> by Auto Machine.
*/

#include "toolbox_stdafx.h"
#include "SkyBox.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "HeightFieldPrimitive.h"
#include "dCustomInputManager.h"
#include "DebugDisplay.h"
#include "dHighResolutionTimer.h"


static dVector GetLookAtDirction(DemoEntityManager* const scene)
{
	dMatrix m = scene->GetCamera()->GetCurrentMatrix();
	dVector f = m.m_front;

	dFloat x2 = f.m_x * f.m_x;
	dFloat y2 = f.m_y * f.m_y;
	dFloat z2 = f.m_z * f.m_z;

	dFloat u = dSqrt(x2 + y2 + z2);

	return dVector(f.m_x / u, f.m_y / u, f.m_z / u);
}

static dVector GetCamPosition(DemoEntityManager* const scene)
{
	dMatrix m = scene->GetCamera()->GetCurrentMatrix();
	return m.m_posit;
}

static void ApplyGravity(const NewtonBody* const body, dFloat timestep, int threadIndex)
{
	// apply gravity force to the body
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;

	NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
	dVector gravityForce(0.0f, -9.8f * mass, 0.0f, 0.0f);
	NewtonBodySetForce(body, &gravityForce[0]);
}

// it fires a box when a key is pressed.
static void FireNewtonCcdBox(NewtonWorld* world, const dVector & postion, const dVector & velocity)
{
	NewtonCollision* collision = NewtonCreateBox(world, 1.0f, 1.0f, 1.0f, 0, NULL);

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = postion;

	NewtonBody* const body = NewtonCreateDynamicBody(world, collision, &matrix[0][0]);

	// set the force callback for applying the force and torque
	NewtonBodySetForceAndTorqueCallback(body, ApplyGravity);

	// set the mass for this body
	dFloat mass = 1.0f;
	NewtonBodySetMassProperties(body, mass, collision);

	NewtonDestroyCollision(collision);

	NewtonBodySetVelocity(body, &velocity[0]);
	NewtonBodySetContinuousCollisionMode(body, 1);
}

// we recommend using and input manage to control input for all games
class CCDInputManager : public dCustomInputManager
{
	public:
	CCDInputManager(DemoEntityManager* const scene)
		:dCustomInputManager(scene->GetNewton())
		, m_scene(scene)
	{
		scene->Set2DDisplayRenderFunction (DrawHelpMenu, NULL, this);
	}

	static void DrawHelpMenu (DemoEntityManager* const scene, void* const context)
	{
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print (color, "Hit R key to shot  boxes to the wall");
	}

	void OnBeginUpdate(dFloat timestepInSecunds)
	{
        int key = 0;
        static dLong timer = dGetTimeInMicrosenconds() + 100000;
        if (dGetTimeInMicrosenconds() > timer) {
            timer = dGetTimeInMicrosenconds() + 100000;
            key = m_scene->GetKeyState('R');
        }
        dVector dir = GetLookAtDirction(m_scene);
        dVector pos = GetCamPosition(m_scene);

		// fire ammo
		if (key)
		{
			dFloat ammo_vel = 1000.0f;
			dVector vel(dir.m_x*ammo_vel, dir.m_y*ammo_vel, dir.m_z*ammo_vel);

			FireNewtonCcdBox(m_scene->GetNewton(), pos, vel);
		}
	}

	void OnEndUpdate(dFloat timestepInSecunds)
	{
	}

	void RenderPlayerHelp(DemoEntityManager* const scene, int lineNumber) const
	{
	}

	static void RenderPlayerHelp(DemoEntityManager* const scene, void* const context, int lineNumber)
	{
	}

	DemoEntityManager* m_scene;
};


static NewtonBody* CreateBackgroundWallsAndCellingBody(NewtonWorld* world)
{
	// make a flat quad 
	dFloat floor[4][3] =
	{
		{ -100.0f, 0.0f, 100.0f },
		{ 100.0f, 0.0f, 100.0f },
		{ 100.0f, 0.0f, -100.0f },
		{ -100.0f, 0.0f, -100.0f },
	};

	dFloat wall_N[4][3] =
	{
		{ -100.0f, 0.0f, 100.0f },
		{ -100.0f, 100.0f, 100.0f },
		{ 100.0f, 100.0f, 100.0f },
		{ 100.0f, 0.0f, 100.0f },
	};

	dFloat wall_W[4][3] =
	{
		{ 100.0f, 0.0f, 100.0f },
		{ 100.0f, 100.0f, 100.0f },
		{ 100.0f, 100.0f, -100.0f },
		{ 100.0f, 0.0f, -100.0f },
	};

	dFloat wall_S[4][3] =
	{
		{ 100.0f, 0.0f, -100.0f },
		{ 100.0f, 100.0f, -100.0f },
		{ -100.0f, 100.0f, -100.0f },
		{ -100.0f, 0.0f, -100.0f },
	};

	dFloat wall_E[4][3] =
	{
		{ -100.0f, 0.0f, -100.0f },
		{ -100.0f, 100.0f, -100.0f },
		{ -100.0f, 100.0f, 100.0f },
		{ -100.0f, 0.0f, 100.0f },
	};

	dFloat celling[4][3] =
	{
		{ -100.0f, 100.0f, -100.0f },
		{ 100.0f, 100.0f, -100.0f },
		{ 100.0f, 100.0f, 100.0f },
		{ -100.0f, 100.0f, 100.0f },
	};

	// crate a collision tree
	NewtonCollision* const collision = NewtonCreateTreeCollision(world, 0);

	// start building the collision mesh
	NewtonTreeCollisionBeginBuild(collision);

	// add the face one at a time
	NewtonTreeCollisionAddFace(collision, 4, &floor[0][0], 3 * sizeof (dFloat), 0);
	NewtonTreeCollisionAddFace(collision, 4, &wall_N[0][0], 3 * sizeof (dFloat), 0);
	NewtonTreeCollisionAddFace(collision, 4, &wall_W[0][0], 3 * sizeof (dFloat), 0);
	NewtonTreeCollisionAddFace(collision, 4, &wall_S[0][0], 3 * sizeof (dFloat), 0);
	NewtonTreeCollisionAddFace(collision, 4, &wall_E[0][0], 3 * sizeof (dFloat), 0);
	NewtonTreeCollisionAddFace(collision, 4, &celling[0][0], 3 * sizeof (dFloat), 0);

	// finish building the collision
	NewtonTreeCollisionEndBuild(collision, 1);

	// create a body with a collision and locate at the identity matrix position 
	dMatrix matrix(dGetIdentityMatrix());
	NewtonBody* const body = NewtonCreateDynamicBody(world, collision, &matrix[0][0]);

	// do no forget to destroy the collision after you not longer need it
	NewtonDestroyCollision(collision);
	return body;
}

void ContinuousCollision1(DemoEntityManager* const scene)
{
	scene->CreateSkyBox();

	// just see ground. it's not a rigid body.
	char fileName[2048];
	dGetWorkingFileName("flatPlane.ngd", fileName);
	scene->LoadScene(fileName);
	

	dVector pos(0.0f);

	pos.m_x = 0.0f;
	pos.m_y = 50.0f;
	pos.m_z = 0.0f;

	dQuaternion rot(dVector(1.f,0.f,0.f),(dFloat)dPi/4.0f);
	scene->SetCameraMatrix(rot, pos);

	CreateBackgroundWallsAndCellingBody(scene->GetNewton());
	new CCDInputManager(scene);

	// compel to change debug display mode. just for convenience.
	scene->SetDebugDisplay (2);
}