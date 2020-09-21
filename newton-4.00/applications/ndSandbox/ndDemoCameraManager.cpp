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


// RenderPrimitive.cpp: implementation of the RenderPrimitive class.
//
//////////////////////////////////////////////////////////////////////
#include "ndSandboxStdafx.h"
#include "ndDemoCamera.h"
#include "ndOpenGlUtil.h"
#include "ndPhysicsUtils.h"
#include "ndDemoCameraManager.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
/*
class ndDemoCameraPickBodyJoint: public dCustomKinematicController
{
	public:
	ndDemoCameraPickBodyJoint(NewtonBody* const body, const dVector& attachmentPointInGlobalSpace, ndDemoCameraManager* const camera)
		:dCustomKinematicController(body, attachmentPointInGlobalSpace)
		,m_manager (camera)
	{
	}
	
	~ndDemoCameraPickBodyJoint()
	{
		if (m_manager) {
			m_manager->ResetPickBody();
		}
	}
		
	ndDemoCameraManager* m_manager;
};
*/

ndDemoCameraManager::ndDemoCameraManager(ndDemoEntityManager* const scene)
	:m_camera (new ndDemoCamera())
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
	//,m_targetPicked(NULL)
	,m_pickJoint(NULL)
	//,m_bodyDestructor(NULL)
{
}

ndDemoCameraManager::~ndDemoCameraManager()
{
	//dAssert(0);
	//if (m_targetPicked) {
	//	ResetPickBody();
	//}
	//m_camera->Release();
	delete m_camera;
}

void ndDemoCameraManager::SetCameraMatrix(ndDemoEntityManager* const scene, const dQuaternion& rotation, const dVector& position)
{
	m_camera->SetMatrix(*scene, rotation, position);
	m_camera->SetMatrix(*scene, rotation, position);
	m_yaw = m_camera->GetYawAngle();
	m_pitch = m_camera->GetPichAngle();
}

/*
void ndDemoCameraManager::FixUpdate (const NewtonWorld* const world, dFloat32 timestep)
{
	// update the camera;
	ndDemoEntityManager* const scene = (ndDemoEntityManager*) NewtonWorldGetUserData(world);

	dMatrix targetMatrix (m_camera->GetNextMatrix());

	int mouseX;
	int mouseY;
	scene->GetMousePosition (mouseX, mouseY);

	// slow down the Camera if we have a Body
	dFloat32 slowDownFactor = scene->IsShiftKeyDown() ? 0.5f/10.0f : 0.5f;

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

	bool mouseState = !scene->GetCaptured() && (scene->GetMouseKeyState(0) && !scene->GetMouseKeyState(1));

	// do camera rotation, only if we do not have anything picked
	bool buttonState = m_mouseLockState || mouseState;
	if (!m_targetPicked && buttonState) {
		int mouseSpeedX = mouseX - m_mousePosX;
		int mouseSpeedY = mouseY - m_mousePosY;

		if ((ImGui::IsMouseHoveringWindow() && ImGui::IsMouseDown(0))) {
			if (mouseSpeedX > 0) {
				m_yaw = dMod(m_yaw + m_yawRate, dFloat32(2.0f * dPi));
			} else if (mouseSpeedX < 0){
				m_yaw = dMod(m_yaw - m_yawRate, dFloat32 (2.0f * dPi));
			}

			if (mouseSpeedY > 0) {
				m_pitch += m_pitchRate;
			} else if (mouseSpeedY < 0){
				m_pitch -= m_pitchRate;
			}
			m_pitch = dClamp(m_pitch, dFloat32 (-80.0f * dDegreeToRad), dFloat32 (80.0f * dDegreeToRad));
		}
	}

	m_mousePosX = mouseX;
	m_mousePosY = mouseY;

	dMatrix matrix (dRollMatrix(m_pitch) * dYawMatrix(m_yaw));
	dQuaternion rot (matrix);
	m_camera->SetMatrix (*scene, rot, targetMatrix.m_posit);

	// get the mouse pick parameter so that we can do replay for debugging
	dFloat32 x = dFloat32(m_mousePosX);
	dFloat32 y = dFloat32(m_mousePosY);
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

	#if 1
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
*/

void ndDemoCameraManager::SetCameraMouseLock (bool loockState)
{
	m_mouseLockState = loockState;
}

void ndDemoCameraManager::RenderPickedTarget () const
{
/*
	if (m_targetPicked) {
		dMatrix matrix;
		NewtonBodyGetMatrix(m_targetPicked, &matrix[0][0]);

		dVector p0 (matrix.TransformVector(m_pickedBodyLocalAtachmentPoint));
		dVector p1 (p0 + matrix.RotateVector (m_pickedBodyLocalAtachmentNormal.Scale (0.5f)));
		ShowMousePicking (p0, p1);
	}
*/
}

void ndDemoCameraManager::InterpolateMatrices (ndDemoEntityManager* const scene, dFloat32 param)
{
	// interpolate the location of all entities in the world
//	ndDemoEntity* const entity
//	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
//	ndDemoEntity* const entity = (ndDemoEntity*)NewtonBodyGetUserData(body);
	for (ndDemoEntityManager::dListNode* node = scene->GetFirst(); node; node = node->GetNext()) {
		ndDemoEntity* const entity = node->GetInfo();
		entity->InterpolateMatrix(*scene, param);
	}

	// interpolate the Camera matrix;
	m_camera->InterpolateMatrix (*scene, param);
}

/*
void ndDemoCameraManager::OnBodyDestroy (NewtonBody* const body)
{
	// remove the references pointer because the body is going to be destroyed
	m_targetPicked = NULL;
	m_bodyDestructor = NULL;
}
*/

void ndDemoCameraManager::UpdatePickBody(ndDemoEntityManager* const scene, bool mousePickState, const dVector& p0, const dVector& p1, dFloat32 timestep) 
{
/*
	// handle pick body from the screen
	if (!m_targetPicked) {
		dAssert(0);
		if (!m_prevMouseState && mousePickState) {
			dFloat32 param;
			dVector posit;
			dVector normal;

			NewtonBody* const body = MousePickBody (scene->GetNewton(), p0, p1, param, posit, normal);
			if (body) {
				dMatrix matrix;
				m_targetPicked = body;
				NewtonBodyGetMatrix(m_targetPicked, &matrix[0][0]);

				dTrace (("body Id: %d\n", NewtonBodyGetID(m_targetPicked)));

				m_pickedBodyParam = param;
				if(m_pickJoint) {
					delete m_pickJoint;
					m_pickJoint = NULL;
				}
					
				dFloat32 Ixx;
				dFloat32 Iyy;
				dFloat32 Izz;
				dFloat32 mass;
				NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);

				// change this to make the grabbing stronger or weaker
				//const dFloat32 angularFritionAccel = 10.0f;
				const dFloat32 angularFritionAccel = 5.0f;
				const dFloat32 linearFrictionAccel = 400.0f * dMax (dAbs (DEMO_GRAVITY), dFloat32(10.0f));
				const dFloat32 inertia = dMax (Izz, dMax (Ixx, Iyy));

				m_pickJoint = new ndDemoCameraPickBodyJoint (body, posit, this);
				m_pickJoint->SetControlMode(dCustomKinematicController::m_linearPlusAngularFriction);

				m_pickJoint->SetMaxLinearFriction(mass * linearFrictionAccel);
				m_pickJoint->SetMaxAngularFriction(inertia * angularFritionAccel);
			}
		}
	} else {
		if (mousePickState) {
			m_pickedBodyTargetPosition = p0 + (p1 - p0).Scale (m_pickedBodyParam);

			if (m_pickJoint) {
				m_pickJoint->SetTargetPosit (m_pickedBodyTargetPosition); 
			}

		} else {
			if (m_pickJoint) {
				delete m_pickJoint;
			}
			ResetPickBody();
		}
	}

	m_prevMouseState = mousePickState;
*/
}

void ndDemoCameraManager::ResetPickBody()
{
/*
	if (m_targetPicked) {
		NewtonBodySetSleepState(m_targetPicked, 0);
	}
	if (m_pickJoint) {
		m_pickJoint->m_manager = NULL;
	}
	m_pickJoint = NULL;
	m_targetPicked = NULL;
	m_bodyDestructor = NULL;
*/
}
