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



// RenderPrimitive.cpp: implementation of the RenderPrimitive class.
//
//////////////////////////////////////////////////////////////////////
#include <toolbox_stdafx.h>
#include "DemoCamera.h"
#include "MousePick.h"
#include "NewtonDemos.h"
#include "DemoCameraListener.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define MOUSE_PICK_DAMP			 10.0f
#define MOUSE_PICK_STIFFNESS	 100.0f


dVector DemoCameraListener::m_pickedBodyDisplacement;
dVector DemoCameraListener::m_pickedBodyLocalAtachmentPoint;
dVector DemoCameraListener::m_pickedBodyLocalAtachmentNormal;
NewtonBody* DemoCameraListener::m_targetPicked = NULL;
NewtonBodyDestructor DemoCameraListener::m_bodyDestructor;
NewtonApplyForceAndTorque DemoCameraListener::m_chainPickBodyForceCallback; 

DemoCameraListener::DemoCameraListener(DemoEntityManager* const scene)
	:DemoListenerBase(scene, "cameraListener")
	,m_camera (new DemoCamera())
	,m_mousePosX(0)
	,m_mousePosY(0)
	,m_yaw (m_camera->GetYawAngle())
	,m_pitch (m_camera->GetPichAngle())
	,m_yawRate (0.01f)
	,m_pitchRate (0.01f)
	,m_frontSpeed(15.0f)
	,m_sidewaysSpeed(10.0f)
	,m_pickedBodyParam(0.0f)
	,m_prevMouseState(false)
{
}

DemoCameraListener::~DemoCameraListener()
{
	m_camera->Release();
}

void DemoCameraListener::PreUpdate (const NewtonWorld* const world, dFloat timestep)
{

}

void DemoCameraListener::PostUpdate (const NewtonWorld* const world, dFloat timestep)
{
	// update the camera;
	DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
//	m_camera->UpdateInputs (timestep, *scene);

	NewtonDemos* const mainWin = scene->GetRootWindow();

	dMatrix targetMatrix (m_camera->GetNextMatrix());

	int mouseX;
	int mouseY;
	mainWin->GetMousePosition (mouseX, mouseY);

	// slow down the Camera if we have a Body
	float slowDownFactor = 1.0f;
//	if (m_targetPicked) {
//		//slowDownFactor *= 0.125f;
//	}

	// do camera translation
	if (mainWin->GetKeyState ('W')) {
		targetMatrix.m_posit += targetMatrix.m_front.Scale(m_frontSpeed * timestep * slowDownFactor);
	}
	if (mainWin->GetKeyState ('S')) {
		targetMatrix.m_posit -= targetMatrix.m_front.Scale(m_frontSpeed * timestep * slowDownFactor);
	}
	if (mainWin->GetKeyState ('A')) {
		targetMatrix.m_posit -= targetMatrix.m_right.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}
	if (mainWin->GetKeyState ('D')) {
		targetMatrix.m_posit += targetMatrix.m_right.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}

	if (mainWin->GetKeyState ('Q')) {
		targetMatrix.m_posit -= targetMatrix.m_up.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}

	if (mainWin->GetKeyState ('E')) {
		targetMatrix.m_posit += targetMatrix.m_up.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}

	// do camera rotation, only if we do not have anything picked
	bool buttonState = mainWin->GetMouseKeyState(0);
	if (!m_targetPicked && buttonState) {
		int mouseSpeedX = mouseX - m_mousePosX;
		int mouseSpeedY = mouseY - m_mousePosY;

		if (mouseSpeedX > 0) {
			m_yaw = dMod(m_yaw + m_yawRate, 2.0f * 3.1416f);
		} else if (mouseSpeedX < 0){
			m_yaw = dMod(m_yaw - m_yawRate, 2.0f * 3.1416f);
		}

		if (mouseSpeedY > 0) {
			m_pitch += m_pitchRate;
		} else if (mouseSpeedY < 0){
			m_pitch -= m_pitchRate;
		}
		m_pitch = dClamp(m_pitch, -80.0f * 3.1416f / 180.0f, 80.0f * 3.1416f / 180.0f);
	}

	m_mousePosX = mouseX;
	m_mousePosY = mouseY;
	
	dMatrix matrix (dRollMatrix(m_pitch) * dYawMatrix(m_yaw));
	dQuaternion rot (matrix);
	m_camera->SetMatrix (*scene, rot, targetMatrix.m_posit);

	UpdatePickBody(scene, timestep);
}

void DemoCameraListener::OnPickedBodyDestroyedNotify (const NewtonBody* body)
{
	if (m_bodyDestructor) {
		m_bodyDestructor (body);
	}

	// the body was destroyed, set the pointer and call back to NULL
	m_targetPicked = NULL;
	m_bodyDestructor = NULL;
	m_chainPickBodyForceCallback = NULL; 
}


void DemoCameraListener::OnPickedBodyApplyForce (const NewtonBody* body, dFloat timestep, int threadIndex)
{
	dVector com;
	dMatrix matrix;

	// apply the thew body forces
	if (m_chainPickBodyForceCallback) {
		m_chainPickBodyForceCallback (body, timestep, threadIndex);
	}

	// add the mouse pick penalty force and torque
	NewtonBodyGetMatrix(body, &matrix[0][0]);
	NewtonBodyGetCentreOfMass (body, &com[0]);

	if (NewtonBodyGetType(body) == NEWTON_KINEMATIC_BODY) {
		// mouse pick a kinematic body, simple update the position to the mouse delta

		// calculate the displacement
		matrix.m_posit = matrix.TransformVector(m_pickedBodyDisplacement.Scale (0.3f));
		NewtonBodySetMatrix(body, &matrix[0][0]);

	} else {
		// we pick a dynamics body, update by applying forces
		dFloat mass;
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;

		dVector veloc;
		dVector omega;

		NewtonBodyGetOmega(body, &omega[0]);
		NewtonBodyGetVelocity(body, &veloc[0]);
		NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);

		dVector force (m_pickedBodyDisplacement.Scale (mass * MOUSE_PICK_STIFFNESS));
		dVector dampForce (veloc.Scale (MOUSE_PICK_DAMP * mass));
		force -= dampForce;


		// calculate local point relative to center of mass
		dVector point (matrix.RotateVector (m_pickedBodyLocalAtachmentPoint - com));
		dVector torque (point * force);

		dVector torqueDamp (omega.Scale (mass * 0.1f));

		NewtonBodyAddForce (body, &force.m_x);
		NewtonBodyAddTorque (body, &torque.m_x);

		// make sure the body is unfrozen, if it is picked
		NewtonBodySetFreezeState (body, 0);
	}
}

void DemoCameraListener::RenderPickedTarget () const
{
	if (m_targetPicked) {
		dMatrix matrix;
		NewtonBodyGetMatrix(m_targetPicked, &matrix[0][0]);

		dVector p0 (matrix.TransformVector(m_pickedBodyLocalAtachmentPoint));
		dVector p1 (p0 + matrix.RotateVector (m_pickedBodyLocalAtachmentNormal.Scale (0.5f)));
		ShowMousePicking (p0, p1);
	}
}


void DemoCameraListener::InterpolateMatrices (DemoEntityManager* const scene, dFloat param)
{
	NewtonWorld* const world = scene->GetNewton();

	// interpolate the Camera matrix;
	m_camera->InterpolateMatrix (*scene, param);

	// interpolate the location of all entities in the world
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(body);
		if (entity) {
			entity->InterpolateMatrix (*scene, param);
		}
	}
}

void DemoCameraListener::UpdatePickBody(DemoEntityManager* const scene, float timestep) 
{
	NewtonDemos* const mainWin = scene->GetRootWindow();

	// handle pick body from the screen

	bool mousePickState = mainWin->GetMouseKeyState(0);
	if (!m_targetPicked) {

		if (!m_prevMouseState && mousePickState) {
			dFloat param;
			dVector posit;
			dVector normal;

			float x = dFloat (m_mousePosX);
			float y = dFloat (m_mousePosY);
			dVector p0 (m_camera->ScreenToWorld(dVector (x, y, 0.0f, 0.0f)));
			dVector p1 (m_camera->ScreenToWorld(dVector (x, y, 1.0f, 0.0f)));
			NewtonBody* const body = MousePickByForce (scene->GetNewton(), p0, p1, param, posit, normal);
			if (body) {
				m_targetPicked = body;
				dMatrix matrix;
				NewtonBodyGetMatrix(m_targetPicked, &matrix[0][0]);

				// save point local to the body matrix
				m_pickedBodyParam = param;
				m_pickedBodyDisplacement = dVector (0.0f, 0.0f, 0.0f, 0.0f);
				m_pickedBodyLocalAtachmentPoint = matrix.UntransformVector (posit);

				// convert normal to local space
				m_pickedBodyLocalAtachmentNormal = matrix.UnrotateVector(normal);

				// link th force and torque callback
				m_chainPickBodyForceCallback = NewtonBodyGetForceAndTorqueCallback (m_targetPicked);
				dAssert (m_chainPickBodyForceCallback != OnPickedBodyApplyForce);

				// set a new call back
				NewtonBodySetForceAndTorqueCallback (m_targetPicked, OnPickedBodyApplyForce);

				// save the destructor call back
				m_bodyDestructor = NewtonBodyGetDestructorCallback(m_targetPicked);
				NewtonBodySetDestructorCallback(m_targetPicked, OnPickedBodyDestroyedNotify);
			}
		}

	} else {
		if (mainWin->GetMouseKeyState(0)) {
			dMatrix matrix;
			NewtonBodyGetMatrix(m_targetPicked, &matrix[0][0]);
			dVector p2 (matrix.TransformVector (m_pickedBodyLocalAtachmentPoint));

			float x = dFloat (m_mousePosX);
			float y = dFloat (m_mousePosY);
			dVector p0 (m_camera->ScreenToWorld(dVector (x, y, 0.0f, 0.0f)));
			dVector p1 (m_camera->ScreenToWorld(dVector (x, y, 1.0f, 0.0f)));
			dVector p (p0 + (p1 - p0).Scale (m_pickedBodyParam));
			m_pickedBodyDisplacement = p - p2;
			dFloat mag2 = m_pickedBodyDisplacement % m_pickedBodyDisplacement;
			if (mag2 > dFloat (20 * 20)) {
				m_pickedBodyDisplacement = m_pickedBodyDisplacement.Scale (20.0f / dSqrt (m_pickedBodyDisplacement % m_pickedBodyDisplacement));
			}
		} else {
			// unchain the callbacks
			NewtonBodySetForceAndTorqueCallback (m_targetPicked, m_chainPickBodyForceCallback);
			NewtonBodySetDestructorCallback(m_targetPicked, m_bodyDestructor);
			m_targetPicked = NULL;
			m_bodyDestructor = NULL;
		}
	}

	m_prevMouseState = mousePickState;
}