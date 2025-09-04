/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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
#include "ndDemoEntityNotify.h"
#include "ndDemoCameraManager.h"

//#define D_ENABLE_CAMERA_REPLAY
#ifdef D_ENABLE_CAMERA_REPLAY
	//#define D_RECORD_CAMERA
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

ndDemoCameraManager::ndDemoCameraManager(ndDemoEntityManager* const scene)
	:ndClassAlloc()
	,m_pickedBodyTargetPosition(ndVector::m_wOne)
	,m_pickedBodyLocalAtachmentPoint(ndVector::m_wOne)
	,m_pickedBodyLocalAtachmentNormal(ndVector::m_zero)
	,m_camera(new ndDemoCamera())
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
	ndInt32 display_w = scene->GetWidth();
	ndInt32 display_h = scene->GetHeight();
	m_camera->SetViewMatrix(display_w, display_h);
}

ndDemoCameraManager::~ndDemoCameraManager()
{
	if (*m_pickJoint)
	{
		ResetPickBody();
	}
	//delete m_camera;
}

void ndDemoCameraManager::SetCameraMatrix(const ndQuaternion& rotation, const ndVector& position)
{
	m_camera->SetMatrix(rotation, position);
	//m_camera->SetMatrix(rotation, position);
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
	if (!*m_pickJoint && buttonState)
	{
		ndFloat32 mouseSpeedX = mouseX - m_mousePosX;
		ndFloat32 mouseSpeedY = mouseY - m_mousePosY;

		if (ImGui::IsMouseDown(0))
		{
			if (mouseSpeedX > 0.0f) 
			{
				m_yaw = ndAnglesAdd(m_yaw, m_yawRate);
			} 
			else if (mouseSpeedX < 0.0f)
			{
				m_yaw = ndAnglesAdd(m_yaw, -m_yawRate);
			}

			if (mouseSpeedY > 0.0f)
			{
				m_pitch += m_pitchRate;
			} 
			else if (mouseSpeedY < 0.0f)
			{
				m_pitch -= m_pitchRate;
			}
			m_pitch = ndClamp(m_pitch, ndFloat32 (-80.0f * ndDegreeToRad), ndFloat32 (80.0f * ndDegreeToRad));
		}
	}

	m_mousePosX = mouseX;
	m_mousePosY = mouseY;

	//m_yaw += 0.01f;
	ndMatrix matrix (ndRollMatrix(m_pitch) * ndYawMatrix(m_yaw));
	ndQuaternion rot (matrix);
	m_camera->SetMatrix (rot, targetMatrix.m_posit);

	// get the mouse pick parameter so that we can do replay for debugging
	ndVector p0(m_camera->ScreenToWorld(ndVector(mouseX, mouseY, ndFloat32(0.0f), ndFloat32(0.0f))));
	ndVector p1(m_camera->ScreenToWorld(ndVector(mouseX, mouseY, ndFloat32(1.0f), ndFloat32(0.0f))));

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
	if (*m_pickJoint)
	{
		ndAssert(0);
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
	D_TRACKTIME();

	for (ndDemoEntityManager::ndNode* node = scene->GetFirst(); node; node = node->GetNext()) 
	{
		ndDemoEntity* const entity = *node->GetInfo();
		entity->InterpolateMatrix(param);
	}

	// interpolate the Camera matrix;
	m_camera->InterpolateMatrix (param);

	//world->UpdateTransformsUnlock();
}

void ndDemoCameraManager::UpdatePickBody(ndDemoEntityManager* const scene, bool mousePickState, const ndVector& p0, const ndVector& p1, ndFloat32) 
{
	// handle pick body from the screen
	if (!*m_pickJoint)
	{
		if (!m_prevMouseState && mousePickState) 
		{
			ndFloat32 param;
			ndVector posit;
			ndVector normal;
		
			ndBodyKinematic* const body = MousePickBody (scene->GetWorld(), p0, p1, param, posit, normal);
			if (body) 
			{
				ndBodyNotify* const notify = body->GetNotifyCallback();
				if (notify)
				{
					ndTrace(("picked body id: %d\n", body->GetId()));
					m_pickedBodyParam = param;

					if (body->GetAsBodyDynamic())
					{
						ndVector mass(body->GetMassMatrix());

						//change this to make the grabbing stronger or weaker
						//const ndFloat32 angularFritionAccel = 10.0f;
						const ndFloat32 angularFritionAccel = 10.0f;
						const ndFloat32 linearFrictionAccel = 40.0f * ndMax(ndAbs(DEMO_GRAVITY), ndFloat32(10.0f));
						const ndFloat32 inertia = ndMax(mass.m_z, ndMax(mass.m_x, mass.m_y));

						ndDemoCameraPickBodyJoint* const pickJoint = new ndDemoCameraPickBodyJoint(body, scene->GetWorld()->GetSentinelBody(), posit, this);
						m_pickJoint = ndSharedPtr<ndJointBilateralConstraint>(pickJoint);
						scene->GetWorld()->AddJoint(m_pickJoint);
						m_pickingMode ?
							pickJoint->SetControlMode(ndJointKinematicController::m_linear) :
							pickJoint->SetControlMode(ndJointKinematicController::m_linearPlusAngularFriction);

						pickJoint->SetMaxLinearFriction(mass.m_w * linearFrictionAccel);
						pickJoint->SetMaxAngularFriction(inertia * angularFritionAccel);
					}
				}
			}
		}
	} 
	else 
	{
		if (mousePickState) 
		{
			m_pickedBodyTargetPosition = p0 + (p1 - p0).Scale (m_pickedBodyParam);
			
			if (*m_pickJoint) 
			{
				ndDemoCameraPickBodyJoint* const pickJoint = (ndDemoCameraPickBodyJoint*)*m_pickJoint;
				pickJoint->SetTargetPosit (m_pickedBodyTargetPosition); 
			}
		} 
		else 
		{
			scene->GetWorld()->RemoveJoint(*m_pickJoint);
			ResetPickBody();
		}
	}

	m_prevMouseState = mousePickState;
}

void ndDemoCameraManager::ResetPickBody()
{
	if (*m_pickJoint)
	{
		m_pickJoint->GetBody0()->SetSleepState(false);
		ndDemoCameraPickBodyJoint* const pickJoint = (ndDemoCameraPickBodyJoint*)*m_pickJoint;
		pickJoint->m_manager = nullptr;
	}
	m_pickJoint = ndSharedPtr<ndJointBilateralConstraint>(nullptr);
}
