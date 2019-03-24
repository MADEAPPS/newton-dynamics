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
#include "toolbox_stdafx.h"
#include "DemoCamera.h"
#include "OpenGlUtil.h"
#include "PhysicsUtils.h"
#include "DemoCameraManager.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//#define USE_PICK_BODY_BY_FORCE


class DemoCameraPickBodyJoint: public dCustomKinematicController
{
	public:
	DemoCameraPickBodyJoint(NewtonBody* const body, const dVector& attachmentPointInGlobalSpace, DemoCameraManager* const camera)
		:dCustomKinematicController(body, attachmentPointInGlobalSpace)
		,m_manager (camera)
	{
	}
	
	~DemoCameraPickBodyJoint()
	{
		if (m_manager) {
			m_manager->ResetPickBody();
		}
	}
		
	DemoCameraManager* m_manager;
};

DemoCameraManager::DemoCameraManager(DemoEntityManager* const scene)
	:m_camera (new DemoCamera())
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

DemoCameraManager::~DemoCameraManager()
{
	if (m_targetPicked) {
		ResetPickBody();
	}
	m_camera->Release();
}

void DemoCameraManager::SetCameraMatrix(DemoEntityManager* const scene, const dQuaternion& rotation, const dVector& position)
{
	m_camera->SetMatrix(*scene, rotation, position);
	m_camera->SetMatrix(*scene, rotation, position);
	m_yaw = m_camera->GetYawAngle();
	m_pitch = m_camera->GetPichAngle();
}


void DemoCameraManager::FixUpdate (const NewtonWorld* const world, dFloat timestep)
{
	// update the camera;
	DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

	dMatrix targetMatrix (m_camera->GetNextMatrix());

	int mouseX;
	int mouseY;
	scene->GetMousePosition (mouseX, mouseY);

	// slow down the Camera if we have a Body
	dFloat slowDownFactor = scene->IsShiftKeyDown() ? 0.5f/10.0f : 0.5f;

	// do camera translation
	if (scene->GetKeyState ('W')) {
		targetMatrix.m_posit += targetMatrix.m_front.Scale(m_frontSpeed * timestep * slowDownFactor);
	}
	if (scene->GetKeyState ('S')) {
		targetMatrix.m_posit -= targetMatrix.m_front.Scale(m_frontSpeed * timestep * slowDownFactor);
	}
	if (scene->GetKeyState ('A')) {
		targetMatrix.m_posit -= targetMatrix.m_right.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}
	if (scene->GetKeyState ('D')) {
		targetMatrix.m_posit += targetMatrix.m_right.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}

	if (scene->GetKeyState ('Q')) {
		targetMatrix.m_posit -= targetMatrix.m_up.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}

	if (scene->GetKeyState ('E')) {
		targetMatrix.m_posit += targetMatrix.m_up.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}

	bool mouseState = scene->GetMouseKeyState(0) && !scene->GetMouseKeyState(1);

	// do camera rotation, only if we do not have anything picked
	bool buttonState = m_mouseLockState || mouseState;
	if (!m_targetPicked && buttonState) {
		int mouseSpeedX = mouseX - m_mousePosX;
		int mouseSpeedY = mouseY - m_mousePosY;

		if ((ImGui::IsMouseHoveringWindow() && ImGui::IsMouseDown(0))) {
			if (mouseSpeedX > 0) {
				m_yaw = dMod(m_yaw + m_yawRate, 2.0f * dPi);
			} else if (mouseSpeedX < 0){
				m_yaw = dMod(m_yaw - m_yawRate, 2.0f * dPi);
			}

			if (mouseSpeedY > 0) {
				m_pitch += m_pitchRate;
			} else if (mouseSpeedY < 0){
				m_pitch -= m_pitchRate;
			}
			m_pitch = dClamp(m_pitch, dFloat (-80.0f * dDegreeToRad), dFloat (80.0f * dDegreeToRad));
		}
	}

	m_mousePosX = mouseX;
	m_mousePosY = mouseY;

	dMatrix matrix (dRollMatrix(m_pitch) * dYawMatrix(m_yaw));
	dQuaternion rot (matrix);
	m_camera->SetMatrix (*scene, rot, targetMatrix.m_posit);

	// get the mouse pick parameter so that we can do replay for debugging
	dFloat x = dFloat(m_mousePosX);
	dFloat y = dFloat(m_mousePosY);
	dVector p0(m_camera->ScreenToWorld(dVector(x, y, 0.0f, 0.0f)));
	dVector p1(m_camera->ScreenToWorld(dVector(x, y, 1.0f, 0.0f)));

#if 0
	struct dReplay
	{
		dVector m_p0;
		dVector m_p1;
		int m_mouseState;
	};
	dReplay replay;

	#if 0
		replay.m_p0 = p0;
		replay.m_p1 = p1;
		replay.m_mouseState = mouseState ? 1 : 0;

		static FILE* file = fopen("log.bin", "wb");
		if (file) {
			fwrite(&replay, sizeof(dReplay), 1, file);
			fflush(file);
		}
	#else 
		static FILE* file = fopen("log.bin", "rb");
		if (file) {
			fread(&replay, sizeof(dReplay), 1, file);
			p0 = replay.m_p0;
			p1 = replay.m_p1;
			mouseState = replay.m_mouseState ? true : false;
		}
	#endif
#endif

	UpdatePickBody(scene, mouseState, p0, p1, timestep);
}


void DemoCameraManager::SetCameraMouseLock (bool loockState)
{
	m_mouseLockState = loockState;
}


void DemoCameraManager::RenderPickedTarget () const
{
	if (m_targetPicked) {
		dMatrix matrix;
		NewtonBodyGetMatrix(m_targetPicked, &matrix[0][0]);

		dVector p0 (matrix.TransformVector(m_pickedBodyLocalAtachmentPoint));
		dVector p1 (p0 + matrix.RotateVector (m_pickedBodyLocalAtachmentNormal.Scale (0.5f)));
		ShowMousePicking (p0, p1);
	}
}


void DemoCameraManager::InterpolateMatrices (DemoEntityManager* const scene, dFloat param)
{
	// interpolate the location of all entities in the world
//	DemoEntity* const entity
//	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
//	DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(body);
	for (DemoEntityManager::dListNode* node = scene->GetFirst(); node; node = node->GetNext()) {
		DemoEntity* const entity = node->GetInfo();
		entity->InterpolateMatrix(*scene, param);
	}

	// interpolate the Camera matrix;
	m_camera->InterpolateMatrix (*scene, param);
}

void DemoCameraManager::OnBodyDestroy (NewtonBody* const body)
{
	// remove the references pointer because the body is going to be destroyed
	m_targetPicked = NULL;
	m_bodyDestructor = NULL;
}


void DemoCameraManager::UpdatePickBody(DemoEntityManager* const scene, bool mousePickState, const dVector& p0, const dVector& p1, dFloat timestep) 
{
	// handle pick body from the screen
	if (!m_targetPicked) {
		if (!m_prevMouseState && mousePickState) {
			dFloat param;
			dVector posit;
			dVector normal;

			//dFloat x = dFloat (m_mousePosX);
			//dFloat y = dFloat (m_mousePosY);
			//dVector p0 (m_camera->ScreenToWorld(dVector (x, y, 0.0f, 0.0f)));
			//dVector p1 (m_camera->ScreenToWorld(dVector (x, y, 1.0f, 0.0f)));
			NewtonBody* const body = MousePickBody (scene->GetNewton(), p0, p1, param, posit, normal);
			if (body) {
				dMatrix matrix;
				m_targetPicked = body;
				NewtonBodyGetMatrix(m_targetPicked, &matrix[0][0]);

				//dTrace (("body Id: %d\n", NewtonBodyGetID(m_targetPicked)));

				m_pickedBodyParam = param;
				#ifdef USE_PICK_BODY_BY_FORCE
					// save point local to the body matrix
					m_pickedBodyLocalAtachmentPoint = matrix.UntransformVector (posit);

					// convert normal to local space
					m_pickedBodyLocalAtachmentNormal = matrix.UnrotateVector(normal);
				#else
					if(m_pickJoint) {
						delete m_pickJoint;
						m_pickJoint = NULL;
					}
					
					dFloat Ixx;
					dFloat Iyy;
					dFloat Izz;
					dFloat mass;
					NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);

					// change this to make the grabbing stronger or weaker
					//const dFloat gfactor = 500.0f;
					const dFloat angularFritionAccel = 10.0f;
					const dFloat linearFrictionAccel = 400.0f * dMax (dAbs (DEMO_GRAVITY), dFloat(10.0f));
					const dFloat inertia = dMax (Izz, dMax (Ixx, Iyy));

					m_pickJoint = new DemoCameraPickBodyJoint (body, posit, this);
					// set this to 1 for full matrix control
					m_pickJoint->SetPickMode (2);

					m_pickJoint->SetMaxLinearFriction(mass * linearFrictionAccel);
					m_pickJoint->SetMaxAngularFriction(inertia * angularFritionAccel);
				#endif
			}
		}

	} else {
		if (mousePickState) {
			//dFloat x = dFloat (m_mousePosX);
			//dFloat y = dFloat (m_mousePosY);
			//dVector p0 (m_camera->ScreenToWorld(dVector (x, y, 0.0f, 0.0f)));
			//dVector p1 (m_camera->ScreenToWorld(dVector (x, y, 1.0f, 0.0f)));
			m_pickedBodyTargetPosition = p0 + (p1 - p0).Scale (m_pickedBodyParam);

			#ifdef USE_PICK_BODY_BY_FORCE
			dMatrix matrix;
			NewtonBodyGetMatrix (m_targetPicked, &matrix[0][0]);
			dVector point (matrix.TransformVector(m_pickedBodyLocalAtachmentPoint));
			CalculatePickForceAndTorque (m_targetPicked, point, m_pickedBodyTargetPosition, timestep);
			#else 
				if (m_pickJoint) {
					// using Dave Gravel method of setting the min and Max friction base of mouse speed.
					#if 0
					// my math is very bad here
					// it is only some tests...
					// it is surely better to use the mouse move speed and not the object vel speed....

					dFloat Ixx;
					dFloat Iyy;
					dFloat Izz;
					dFloat mass;
					NewtonBodyGetMass(m_targetPicked, &mass, &Ixx, &Iyy, &Izz);

					dVector svel;
					NewtonBodyGetVelocity(m_targetPicked, &svel[0]);

					float speed = dSqrt(svel.m_x * svel.m_x + svel.m_y * svel.m_y + svel.m_z * svel.m_z);

					dFloat angularFritionAccel = (((5.0f*timestep*120.0f) + (5.0f + speed))*2.0f);
					dFloat linearFrictionAccel = angularFritionAccel * dAbs(dMax(-9.81f, 5.0f));
					dFloat inertia = dMax(Izz, dMax(Ixx, Iyy));

					if (speed >= 5.0f) {
						angularFritionAccel = (((5.0f*timestep*120.0f) + speed)*6.0f);
						linearFrictionAccel = angularFritionAccel * dAbs(dMax(-9.81f, 5.0f));
					} else {
						if (angularFritionAccel > 5.0f) angularFritionAccel = 5.0f;
					}

					m_pickJoint->SetMaxLinearFriction((mass*10.0f) * linearFrictionAccel);
					m_pickJoint->SetMaxAngularFriction(inertia * angularFritionAccel);

					#endif		

					m_pickJoint->SetTargetPosit (m_pickedBodyTargetPosition); 
				}
			#endif
		} else {
			if (m_pickJoint) {
				delete m_pickJoint;
			}
			ResetPickBody();
		}
	}

	m_prevMouseState = mousePickState;
}

void DemoCameraManager::ResetPickBody()
{
	if (m_targetPicked) {
		NewtonBodySetSleepState(m_targetPicked, 0);
	}
	if (m_pickJoint) {
		m_pickJoint->m_manager = NULL;
	}
	m_pickJoint = NULL;
	m_targetPicked = NULL;
	m_bodyDestructor = NULL;
}
