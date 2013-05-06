/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************
#include "StdAfx.h"

#include "dVector.h"
#include "dMatrix.h"
#include "dQuaternion.h"
#include "MousePick.h"
#include "JointLibrary.h"


#define USE_PICK_BODY_JOINT

static dFloat pickedParam;
static NewtonBody* pickedBody;
//static dVector rayLocalNormal;
static bool isPickedBodyDynamics;
//static dVector attachmentPoint;
static NewtonUserJoint* bodyPickController;

#pragma warning (disable: 4100) //unreferenced formal parameter

#define MAX_FRICTION_ANGULAR_GRAVITY			 250.0f
#define MAX_FRICTION_LINEAR_ACCELERATION		1000.0f



// implement a ray cast pre-filter
static unsigned RayCastPrefilter (const NewtonBody* body,  const NewtonCollision* collision, void* userData)
{
	// ray cannot pick trigger volumes

	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);

	// do not pick static bodies
	return (mass > 0.0f) ? 1 : 0;
//	return 1;
}


// implement a ray cast filter
static dFloat RayCastFilter (const NewtonBody* body, const dFloat* normal, int collisionID, void* userData, dFloat intersetParam)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;

	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
	if (intersetParam <  pickedParam) {
		isPickedBodyDynamics = (mass > 0.0f);
		pickedParam = intersetParam;
		pickedBody = (NewtonBody*)body;
//		rayLocalNormal = dVector (normal[0], normal[1], normal[2]);
	}
	return intersetParam;
}


bool MousePick (NewtonWorld* nWorld, const dVector& cursorPosit1)
{
	static int mouseKey1;
	static int mouseKey0;
	static dVector cursorPosit0;
	static bool mousePickMode = false;

	mouseKey1 = IsKeyDown (KeyCode_L_BUTTON) || IsKeyDown (KeyCode_R_BUTTON);
	if (mouseKey1) {
		if (!mouseKey0) {
			dVector p0 (ScreenToWorld(dVector (cursorPosit1.m_x, cursorPosit1.m_y, 0.0f, 0.0f)));
			dVector p1 (ScreenToWorld(dVector (cursorPosit1.m_x, cursorPosit1.m_y, 1.0f, 0.0f)));

			pickedBody = NULL;
			pickedParam = 1.1f;
			isPickedBodyDynamics = false;
			NewtonWorldRayCast(nWorld, &p0[0], &p1[0], RayCastFilter, NULL, RayCastPrefilter);
			if (pickedBody) {
				dMatrix matrix;
			
				mousePickMode = true;

				NewtonBodyGetMatrix(pickedBody, &matrix[0][0]);
				dVector p (p0 + (p1 - p0).Scale (pickedParam));

//				attachmentPoint = matrix.UntransformVector (p);

				// convert normal to local space
//				rayLocalNormal = matrix.UnrotateVector(rayLocalNormal);

				// Create PickBody Joint
				bodyPickController = CreateCustomKinematicController (pickedBody, &p[0]);
				if (IsKeyDown (KeyCode_R_BUTTON)) {
					CustomKinematicControllerSetPickMode (bodyPickController, 1);
					CustomKinematicControllerSetMaxAngularFriction (bodyPickController, MAX_FRICTION_ANGULAR_GRAVITY);
					CustomKinematicControllerSetMaxLinearFriction (bodyPickController, MAX_FRICTION_LINEAR_ACCELERATION);
				} else {
					CustomKinematicControllerSetPickMode (bodyPickController, 0);
					CustomKinematicControllerSetMaxAngularFriction (bodyPickController, 10.0f);
					CustomKinematicControllerSetMaxLinearFriction (bodyPickController, MAX_FRICTION_LINEAR_ACCELERATION);
				}
			}
		}

		if (mousePickMode) {
			// init pick mode
			dVector p0 (ScreenToWorld(dVector (cursorPosit1.m_x, cursorPosit1.m_y, 0.0f, 0.0f)));
			dVector p1 (ScreenToWorld(dVector (cursorPosit1.m_x, cursorPosit1.m_y, 1.0f, 0.0f)));
			dVector p (p0 + (p1 - p0).Scale (pickedParam));
			if (bodyPickController) {
				CustomKinematicControllerSetTargetPosit (bodyPickController, &p[0]);
			}
		}
	} else {
		mousePickMode = false;
		if (pickedBody) {
			if (bodyPickController) {
				CustomDestroyJoint (bodyPickController);
			}
			bodyPickController = NULL;
		}
	}

	cursorPosit0 = cursorPosit1;
	mouseKey0 = mouseKey1;

	bool retState;
	retState = isPickedBodyDynamics;
	return retState;
}




