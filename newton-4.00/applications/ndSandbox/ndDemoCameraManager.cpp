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

#include "ndSandboxStdafx.h"
#include "ndDemoCamera.h"
#include "ndOpenGlUtil.h"
#include "ndPhysicsWorld.h"
#include "ndPhysicsUtils.h"
#include "ndDemoCameraManager.h"

class ndDemoCameraPickBodyJoint: public ndJointKinematicController
{
	public:
	ndDemoCameraPickBodyJoint(ndBodyKinematic* const childBody, ndBodyKinematic* const worldBody, const dVector& attachmentPointInGlobalSpace, ndDemoCameraManager* const camera)
		:ndJointKinematicController(childBody, worldBody, attachmentPointInGlobalSpace)
		,m_manager (camera)
	{
	}
	
	~ndDemoCameraPickBodyJoint()
	{
		if (m_manager) 
		{
			m_manager->ResetPickBody();
		}
	}
		
	ndDemoCameraManager* m_manager;
};

ndDemoCameraManager::ndDemoCameraManager(ndDemoEntityManager* const scene)
	:dClassAlloc()
	,m_camera (new ndDemoCamera())
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
	,m_pickedBodyTargetPosition(dVector::m_wOne)
	,m_pickedBodyLocalAtachmentPoint(dVector::m_wOne)
	,m_pickedBodyLocalAtachmentNormal(dVector::m_zero)
	,m_targetPicked(nullptr)
	,m_pickJoint(nullptr)
	//,m_bodyDestructor(nullptr)
{
}

ndDemoCameraManager::~ndDemoCameraManager()
{
	if (m_targetPicked) 
	{
		ResetPickBody();
	}
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


void ndDemoCameraManager::FixUpdate (ndDemoEntityManager* const scene, dFloat32 timestep)
{
	// update the camera;
	dMatrix targetMatrix (m_camera->GetNextMatrix());

	dFloat32 mouseX;
	dFloat32 mouseY;
	scene->GetMousePosition (mouseX, mouseY);

	// slow down the Camera if we have a Body
	dFloat32 slowDownFactor = scene->IsShiftKeyDown() ? 0.5f/10.0f : 0.5f;

	// do camera translation
	if (scene->GetKeyState ('W')) 
	{
		targetMatrix.m_posit += targetMatrix.m_front.Scale(m_frontSpeed * timestep * slowDownFactor);
	}
	if (scene->GetKeyState ('S')) 
	{
		targetMatrix.m_posit -= targetMatrix.m_front.Scale(m_frontSpeed * timestep * slowDownFactor);
	}
	if (scene->GetKeyState ('A')) 
	{
		targetMatrix.m_posit -= targetMatrix.m_right.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}
	if (scene->GetKeyState ('D')) 
	{
		targetMatrix.m_posit += targetMatrix.m_right.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}

	if (scene->GetKeyState ('Q')) 
	{
		targetMatrix.m_posit -= targetMatrix.m_up.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}

	if (scene->GetKeyState ('E')) 
	{
		targetMatrix.m_posit += targetMatrix.m_up.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}


	bool mouseState = !scene->GetCaptured() && (scene->GetMouseKeyState(0) && !scene->GetMouseKeyState(1));

	// do camera rotation, only if we do not have anything picked
	bool buttonState = m_mouseLockState || mouseState;
	if (!m_targetPicked && buttonState) 
	{
		dFloat32 mouseSpeedX = mouseX - m_mousePosX;
		dFloat32 mouseSpeedY = mouseY - m_mousePosY;

		if ((ImGui::IsMouseHoveringWindow() && ImGui::IsMouseDown(0))) 
		{
			if (mouseSpeedX > 0.0f) 
			{
				m_yaw = dMod(m_yaw + m_yawRate, dFloat32(2.0f * dPi));
			} 
			else if (mouseSpeedX < 0.0f)
			{
				m_yaw = dMod(m_yaw - m_yawRate, dFloat32 (2.0f * dPi));
			}

			if (mouseSpeedY > 0.0f)
			{
				m_pitch += m_pitchRate;
			} 
			else if (mouseSpeedY < 0.0f)
			{
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
	//dFloat32 x = m_mousePosX;
	//dFloat32 y = m_mousePosY;
	dVector p0(m_camera->ScreenToWorld(dVector(mouseX, mouseY, 0.0f, 0.0f)));
	dVector p1(m_camera->ScreenToWorld(dVector(mouseX, mouseY, 1.0f, 0.0f)));

#if 1
	struct ndReplay
	{
		dVector m_p0;
		dVector m_p1;
		int m_mouseState;
	};
	ndReplay replay;

	#if 1
		replay.m_p0 = p0;
		replay.m_p1 = p1;
		replay.m_mouseState = mouseState ? 1 : 0;

		static FILE* file = fopen("log.bin", "wb");
		if (file) 
		{
			fwrite(&replay, sizeof(ndReplay), 1, file);
			fflush(file);
		}
	#else 
		static FILE* file = fopen("log.bin", "rb");
		if (file) 
		{
			fread(&replay, sizeof(ndReplay), 1, file);
			p0 = replay.m_p0;
			p1 = replay.m_p1;
			mouseState = replay.m_mouseState ? true : false;
		}
	#endif
#endif

	UpdatePickBody(scene, mouseState, p0, p1, timestep);
}

void ndDemoCameraManager::SetCameraMouseLock (bool loockState)
{
	m_mouseLockState = loockState;
}

void ndDemoCameraManager::RenderPickedTarget () const
{
	if (m_targetPicked) 
	{
		dAssert(0);
		//dMatrix matrix;
		//NewtonBodyGetMatrix(m_targetPicked, &matrix[0][0]);
		//
		//dVector p0 (matrix.TransformVector(m_pickedBodyLocalAtachmentPoint));
		//dVector p1 (p0 + matrix.RotateVector (m_pickedBodyLocalAtachmentNormal.Scale (0.5f)));
		//ShowMousePicking (p0, p1);
	}
}

void ndDemoCameraManager::InterpolateMatrices (ndDemoEntityManager* const scene, dFloat32 param)
{
	// interpolate the location of all entities in the world
	for (ndDemoEntityManager::dListNode* node = scene->GetFirst(); node; node = node->GetNext()) 
	{
		ndDemoEntity* const entity = node->GetInfo();
		entity->InterpolateMatrix(*scene, param);
	}

	// interpolate the Camera matrix;
	m_camera->InterpolateMatrix (*scene, param);
}

void ndDemoCameraManager::UpdatePickBody(ndDemoEntityManager* const scene, bool mousePickState, const dVector& p0, const dVector& p1, dFloat32 timestep) 
{
	// handle pick body from the screen
	if (!m_targetPicked) 
	{
		if (!m_prevMouseState && mousePickState) 
		{
			dFloat32 param;
			dVector posit;
			dVector normal;
		
			ndBodyKinematic* const body = MousePickBody (scene->GetWorld(), p0, p1, param, posit, normal);
			if (body) 
			{
				m_targetPicked = body;
				dMatrix matrix (m_targetPicked->GetMatrix());
				
				m_pickedBodyParam = param;
				if(m_pickJoint) 
				{
					scene->GetWorld()->RemoveJoint(m_pickJoint);
					delete m_pickJoint;
					m_pickJoint = nullptr;
				}
					
				dVector mass (m_targetPicked->GetMassMatrix());

				// change this to make the grabbing stronger or weaker
				//const dFloat32 angularFritionAccel = 10.0f;
				const dFloat32 angularFritionAccel = 5.0f;
				const dFloat32 linearFrictionAccel = 400.0f * dMax (dAbs (DEMO_GRAVITY), dFloat32(10.0f));
				const dFloat32 inertia = dMax (mass.m_z, dMax (mass.m_x, mass.m_y));

				m_pickJoint = new ndDemoCameraPickBodyJoint (body, scene->GetWorld()->GetSentinelBody(), posit, this);
				scene->GetWorld()->AddJoint(m_pickJoint);
				m_pickJoint->SetControlMode(ndJointKinematicController::m_linearPlusAngularFriction);
				
				m_pickJoint->SetMaxLinearFriction(mass.m_w * linearFrictionAccel);
				m_pickJoint->SetMaxAngularFriction(inertia * angularFritionAccel);
			}
		}
	} 
	else 
	{
		if (mousePickState) 
		{
			m_pickedBodyTargetPosition = p0 + (p1 - p0).Scale (m_pickedBodyParam);
			
			if (m_pickJoint) 
			{
				dTrace(("set target Matrix: %s\n", __FUNCTION__));
				m_pickJoint->SetTargetPosit (m_pickedBodyTargetPosition); 
			}
		} 
		else 
		{
			if (m_pickJoint) 
			{
				scene->GetWorld()->RemoveJoint(m_pickJoint);
				delete m_pickJoint;
				m_pickJoint = nullptr;
			}
			ResetPickBody();
		}
	}

	m_prevMouseState = mousePickState;
}

void ndDemoCameraManager::ResetPickBody()
{
	if (m_targetPicked) 
	{
		m_targetPicked->SetSleepState(false);
	}
	if (m_pickJoint) 
	{
		m_pickJoint->m_manager = nullptr;
	}
	m_pickJoint = nullptr;
	m_targetPicked = nullptr;
}
