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



// RenderPrimitive.cpp: implementation of the RenderPrimitive class.
//
//////////////////////////////////////////////////////////////////////
#include <toolbox_stdafx.h>
#include "DemoCamera.h"
#include "MousePick.h"
#include "NewtonDemos.h"
#include "PhysicsUtils.h"
#include "DemoCameraListener.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//#define USE_PICK_BODY_BY_FORCE


#define D_CAMERA_LISTENER_NAMNE "cameraListener"

DemoCameraListener::DemoCameraListener(DemoEntityManager* const scene)
	:DemoListenerBase(scene, D_CAMERA_LISTENER_NAMNE)
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
	,m_mouseLockState(false)
	,m_pickedBodyTargetPosition(0.0f)
	,m_pickedBodyLocalAtachmentPoint(0.0f)
	,m_pickedBodyLocalAtachmentNormal(0.0f)
	,m_targetPicked(NULL)
	,m_pickJoint(NULL)
	,m_bodyDestructor(NULL)
{
}

DemoCameraListener::~DemoCameraListener()
{
	m_camera->Release();
}

void DemoCameraListener::PreUpdate (const NewtonWorld* const world, dFloat timestep)
{
	// update the camera;
	DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

	NewtonDemos* const mainWin = scene->GetRootWindow();

	dMatrix targetMatrix (m_camera->GetNextMatrix());

	int mouseX;
	int mouseY;
	mainWin->GetMousePosition (mouseX, mouseY);

	// slow down the Camera if we have a Body
	dFloat slowDownFactor = mainWin->IsShiftKeyDown() ? 0.5f/10.0f : 0.5f;

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
	bool buttonState = m_mouseLockState || mainWin->GetMouseKeyState(0);
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
		m_pitch = dClamp(m_pitch, dFloat (-80.0f * 3.1416f / 180.0f), dFloat (80.0f * 3.1416f / 180.0f));
	}

	m_mousePosX = mouseX;
	m_mousePosY = mouseY;

	dMatrix matrix (dRollMatrix(m_pitch) * dYawMatrix(m_yaw));
	dQuaternion rot (matrix);
	m_camera->SetMatrix (*scene, rot, targetMatrix.m_posit);

	UpdatePickBody(scene, timestep);
}

void DemoCameraListener::PostUpdate (const NewtonWorld* const world, dFloat timestep)
{
}

void DemoCameraListener::SetCameraMouseLock (bool loockState)
{
	m_mouseLockState = loockState;
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

void DemoCameraListener::OnBodyDestroy (NewtonBody* const body)
{
	// remove the references pointer because the body is going to be destroyed
	m_targetPicked = NULL;
	m_bodyDestructor = NULL;
}


void DemoCameraListener::UpdatePickBody(DemoEntityManager* const scene, dFloat timestep) 
{
	NewtonDemos* const mainWin = scene->GetRootWindow();

	// handle pick body from the screen
	bool mousePickState = mainWin->GetMouseKeyState(0);
	if (!m_targetPicked) {
		if (!m_prevMouseState && mousePickState) {
			dFloat param;
			dVector posit;
			dVector normal;

			dFloat x = dFloat (m_mousePosX);
			dFloat y = dFloat (m_mousePosY);
			dVector p0 (m_camera->ScreenToWorld(dVector (x, y, 0.0f, 0.0f)));
			dVector p1 (m_camera->ScreenToWorld(dVector (x, y, 1.0f, 0.0f)));
			NewtonBody* const body = MousePickBody (scene->GetNewton(), p0, p1, param, posit, normal);
			if (body) {
				dMatrix matrix;
				m_targetPicked = body;
				NewtonBodyGetMatrix(m_targetPicked, &matrix[0][0]);

				m_pickedBodyParam = param;
				#ifdef USE_PICK_BODY_BY_FORCE
				// save point local to the body matrix
				m_pickedBodyLocalAtachmentPoint = matrix.UntransformVector (posit);

				// convert normal to local space
				m_pickedBodyLocalAtachmentNormal = matrix.UnrotateVector(normal);
				#else
					if(m_pickJoint) {
						delete m_pickJoint;
					}
					
					dFloat Ixx;
					dFloat Iyy;
					dFloat Izz;
					dFloat mass;
					NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);

					const dFloat angularFritionAccel = 100.0f;
					const dFloat linearFrictionAccel = 10.0f * dAbs (dMax (DEMO_GRAVITY, 10.0f));
					const dFloat inertia = dMax (Izz, dMax (Ixx, Iyy));
					
					m_pickJoint = new dCustomKinematicController (body, posit);
					m_pickJoint->SetPickMode (0);
					m_pickJoint->SetMaxLinearFriction(mass * linearFrictionAccel);
					m_pickJoint->SetMaxAngularFriction(inertia * angularFritionAccel);
				#endif
			}
		}

	} else {
		if (mainWin->GetMouseKeyState(0)) {
			dFloat x = dFloat (m_mousePosX);
			dFloat y = dFloat (m_mousePosY);
			dVector p0 (m_camera->ScreenToWorld(dVector (x, y, 0.0f, 0.0f)));
			dVector p1 (m_camera->ScreenToWorld(dVector (x, y, 1.0f, 0.0f)));
			m_pickedBodyTargetPosition = p0 + (p1 - p0).Scale (m_pickedBodyParam);

			#ifdef USE_PICK_BODY_BY_FORCE
			dMatrix matrix;
			NewtonBodyGetMatrix (m_targetPicked, &matrix[0][0]);
			dVector point (matrix.TransformVector(m_pickedBodyLocalAtachmentPoint));
			CalculatePickForceAndTorque (m_targetPicked, point, m_pickedBodyTargetPosition, timestep);
			#else 
				if (m_pickJoint) {
					m_pickJoint->SetTargetPosit (m_pickedBodyTargetPosition); 
				}
			#endif
		} else {
			if (m_targetPicked) {
				NewtonBodySetSleepState (m_targetPicked, 0);
			}
			if (m_pickJoint) {
				delete m_pickJoint;
			}
			m_pickJoint = NULL;
			m_targetPicked = NULL; 
			m_bodyDestructor = NULL;
		}
	}

	m_prevMouseState = mousePickState;
}


