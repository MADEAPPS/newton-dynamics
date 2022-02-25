/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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

//#define D_ENABLE_CAMERA_REPLAY
#ifdef D_ENABLE_CAMERA_REPLAY
	#define D_RECORD_CAMERA
#endif

class ndDemoCameraPickBodyJoint: public ndJointKinematicController
{
	public:
	ndDemoCameraPickBodyJoint(ndBodyKinematic* const childBody, ndBodyKinematic* const worldBody, const ndVector& attachmentPointInGlobalSpace, ndDemoCameraManager* const camera)
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

ndDemoCameraManager::ndDemoCameraManager(ndDemoEntityManager* const)
	:ndClassAlloc()
	,m_pickedBodyTargetPosition(ndVector::m_wOne)
	,m_pickedBodyLocalAtachmentPoint(ndVector::m_wOne)
	,m_pickedBodyLocalAtachmentNormal(ndVector::m_zero)
	,m_camera(new ndDemoCamera())
	,m_targetPicked(nullptr)
	,m_pickJoint(nullptr)
	,m_mousePosX(0)
	,m_mousePosY(0)
	,m_yaw (m_camera->GetYawAngle())
	,m_pitch (m_camera->GetPichAngle())
	,m_yawRate (0.04f)
	,m_pitchRate (0.02f)
	,m_frontSpeed(15.0f)
	,m_sidewaysSpeed(10.0f)
	,m_pickedBodyParam(0.0f)
	,m_prevMouseState(false)
	,m_mouseLockState(false)
	,m_pickingMode(false)
{
}

ndDemoCameraManager::~ndDemoCameraManager()
{
	if (m_targetPicked) 
	{
		ResetPickBody();
	}
	delete m_camera;
}

void ndDemoCameraManager::SetCameraMatrix(const ndQuaternion& rotation, const ndVector& position)
{
	m_camera->SetMatrix(rotation, position);
	m_camera->SetMatrix(rotation, position);
	m_yaw = m_camera->GetYawAngle();
	m_pitch = m_camera->GetPichAngle();
}

void ndDemoCameraManager::FixUpdate (ndDemoEntityManager* const scene, ndFloat32 timestep)
{
	// update the camera;
	ndMatrix targetMatrix (m_camera->GetNextMatrix());

	ndFloat32 mouseX;
	ndFloat32 mouseY;
	scene->GetMousePosition (mouseX, mouseY);

	// slow down the Camera if we have a Body
	ndFloat32 slowDownFactor = scene->IsShiftKeyDown() ? 0.5f/10.0f : 0.5f;

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
		ndFloat32 mouseSpeedX = mouseX - m_mousePosX;
		ndFloat32 mouseSpeedY = mouseY - m_mousePosY;

		if (ImGui::IsMouseDown(0))
		{
			if (mouseSpeedX > 0.0f) 
			{
				m_yaw = AnglesAdd(m_yaw, m_yawRate);
			} 
			else if (mouseSpeedX < 0.0f)
			{
				m_yaw = AnglesAdd(m_yaw, -m_yawRate);
			}

			if (mouseSpeedY > 0.0f)
			{
				m_pitch += m_pitchRate;
			} 
			else if (mouseSpeedY < 0.0f)
			{
				m_pitch -= m_pitchRate;
			}
			m_pitch = dClamp(m_pitch, ndFloat32 (-80.0f * ndDegreeToRad), ndFloat32 (80.0f * ndDegreeToRad));
		}
	}

	m_mousePosX = mouseX;
	m_mousePosY = mouseY;

	//m_yaw += 0.01f;
	ndMatrix matrix (dRollMatrix(m_pitch) * dYawMatrix(m_yaw));
	ndQuaternion rot (matrix);
	m_camera->SetMatrix (rot, targetMatrix.m_posit);

	// get the mouse pick parameter so that we can do replay for debugging
	ndVector p0(m_camera->ScreenToWorld(ndVector(mouseX, mouseY, 0.0f, 0.0f)));
	ndVector p1(m_camera->ScreenToWorld(ndVector(mouseX, mouseY, 1.0f, 0.0f)));

#ifdef D_ENABLE_CAMERA_REPLAY
	struct ndReplay
	{
		ndVector m_p0;
		ndVector m_p1;
		ndInt32 m_mouseState;
	};
	ndReplay replay;

	#ifdef D_RECORD_CAMERA
		replay.m_p0 = p0;
		replay.m_p1 = p1;
		replay.m_mouseState = mouseState ? 1 : 0;

		static FILE* file = fopen("cameraLog.bin", "wb");
		if (file) 
		{
			fwrite(&replay, sizeof(ndReplay), 1, file);
			fflush(file);
		}
	#else 
		static FILE* file = fopen("cameraLog.bin", "rb");
		if (file) 
		{
			fread(&replay, sizeof(ndReplay), 1, file);
			p0 = replay.m_p0;
			p1 = replay.m_p1;
			mouseState = replay.m_mouseState ? true : false;
		}
	#endif
#endif

	//dTrace(("frame: %d  camera angle: %f\n", scene->GetWorld()->GetFrameIndex(), m_yaw * dRadToDegree));
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
		//ndMatrix matrix;
		//NewtonBodyGetMatrix(m_targetPicked, &matrix[0][0]);
		//
		//ndVector p0 (matrix.TransformVector(m_pickedBodyLocalAtachmentPoint));
		//ndVector p1 (p0 + matrix.RotateVector (m_pickedBodyLocalAtachmentNormal.Scale (0.5f)));
		//ShowMousePicking (p0, p1);
	}
}

void ndDemoCameraManager::InterpolateMatrices (ndDemoEntityManager* const scene, ndFloat32 param)
{
	// interpolate the location of all entities in the world
	ndWorld* const world = scene->GetWorld();
	world->UpdateTransformsLock();

	for (ndDemoEntityManager::ndNode* node = scene->GetFirst(); node; node = node->GetNext()) 
	{
		ndDemoEntity* const entity = node->GetInfo();
		entity->InterpolateMatrix(param);
	}

	// interpolate the Camera matrix;
	m_camera->InterpolateMatrix (param);

	world->UpdateTransformsUnlock();
}

void ndDemoCameraManager::UpdatePickBody(ndDemoEntityManager* const scene, bool mousePickState, const ndVector& p0, const ndVector& p1, ndFloat32) 
{
	// handle pick body from the screen
	if (!m_targetPicked) 
	{
		if (!m_prevMouseState && mousePickState) 
		{
			ndFloat32 param;
			ndVector posit;
			ndVector normal;
		
			ndBodyKinematic* const body = MousePickBody (scene->GetWorld(), p0, p1, param, posit, normal);
			if (body) 
			{
				ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)body->GetNotifyCallback();
				if (notify)
				{
					notify->OnObjectPick();
					m_targetPicked = body;

					m_pickedBodyParam = param;
					if (m_pickJoint)
					{
						scene->GetWorld()->RemoveJoint(m_pickJoint);
						delete m_pickJoint;
						m_pickJoint = nullptr;
					}

					ndVector mass(m_targetPicked->GetMassMatrix());

					//change this to make the grabbing stronger or weaker
					//const ndFloat32 angularFritionAccel = 10.0f;
					const ndFloat32 angularFritionAccel = 10.0f;
					const ndFloat32 linearFrictionAccel = 40.0f * dMax(dAbs(DEMO_GRAVITY), ndFloat32(10.0f));
					const ndFloat32 inertia = dMax(mass.m_z, dMax(mass.m_x, mass.m_y));

					m_pickJoint = new ndDemoCameraPickBodyJoint(body, scene->GetWorld()->GetSentinelBody(), posit, this);
					scene->GetWorld()->AddJoint(m_pickJoint);
					m_pickingMode ?
						m_pickJoint->SetControlMode(ndJointKinematicController::m_linear) :
						m_pickJoint->SetControlMode(ndJointKinematicController::m_linearPlusAngularFriction);

					m_pickJoint->SetMaxLinearFriction(mass.m_w * linearFrictionAccel);
					m_pickJoint->SetMaxAngularFriction(inertia * angularFritionAccel);
				}
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
