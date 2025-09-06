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
#include "ndDemoEntityManager.h"

ndDemoCamera::ndDemoCamera(ndRender* const owner)
	:ndRenderSceneCamera(owner)
	,m_yaw(ndFloat32(0.0f))
	,m_pitch(ndFloat32(0.0f))
	,m_yawRate(ndFloat32(0.04f))
	,m_pitchRate(ndFloat32(0.02f))
	,m_mousePosX(ndFloat32(0.0f))
	,m_mousePosY(ndFloat32(0.0f))
	,m_frontSpeed(ndFloat32(15.0f))
	,m_sidewaysSpeed(ndFloat32(10.0f))
{
}

void ndDemoCamera::SetTransform(const ndQuaternion& rotation, const ndVector& position)
{
	ndRenderSceneCamera::SetTransform(rotation, position);
	const ndMatrix matrix(GetMatrix());
	m_pitch = ndAsin(matrix.m_front.m_y);
	m_yaw = ndAtan2(-matrix.m_front.m_z, matrix.m_front.m_x);
}

void ndDemoCamera::TickUpdate(ndFloat32 timestep)
{
	ndRender* const renderer = GetOwner();
	ndAssert(renderer);
	ndDemoEntityManager::ndRenderCallback* const renderCallback = (ndDemoEntityManager::ndRenderCallback*)*renderer->GetOwner();
	ndDemoEntityManager* const scene = renderCallback->m_owner;

	ndFloat32 mouseX;
	ndFloat32 mouseY;
	scene->GetMousePosition(mouseX, mouseY);

	// slow down the Camera if we have a Body
	ndFloat32 slowDownFactor = scene->IsShiftKeyDown() ? 0.5f / 10.0f : 0.5f;

	ndMatrix targetMatrix(ndCalculateMatrix(m_transform1.m_rotation, m_transform1.m_position));

	// do camera translation
	if (scene->GetKeyState('W'))
	{
		targetMatrix.m_posit += targetMatrix.m_front.Scale(m_frontSpeed * timestep * slowDownFactor);
	}
	if (scene->GetKeyState('S'))
	{
		targetMatrix.m_posit -= targetMatrix.m_front.Scale(m_frontSpeed * timestep * slowDownFactor);
	}
	if (scene->GetKeyState('A'))
	{
		targetMatrix.m_posit -= targetMatrix.m_right.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}
	if (scene->GetKeyState('D'))
	{
		targetMatrix.m_posit += targetMatrix.m_right.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}

	if (scene->GetKeyState('Q'))
	{
		targetMatrix.m_posit -= targetMatrix.m_up.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}

	if (scene->GetKeyState('E'))
	{
		targetMatrix.m_posit += targetMatrix.m_up.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}

	bool mouseState = !scene->GetCaptured() && (scene->GetMouseKeyState(0) && !scene->GetMouseKeyState(1));

	// do camera rotation, only if we do not have anything picked
	//bool buttonState = m_mouseLockState || mouseState;
	//if (!*m_pickJoint && buttonState)
	if (mouseState)
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
			m_pitch = ndClamp(m_pitch, ndFloat32(-80.0f * ndDegreeToRad), ndFloat32(80.0f * ndDegreeToRad));
		}
	}

	m_mousePosX = mouseX;
	m_mousePosY = mouseY;

	ndMatrix matrix(ndRollMatrix(m_pitch) * ndYawMatrix(m_yaw));
	ndQuaternion newRotation(matrix);
	//ndTrace(("(%f %f %f %f)\n", newRotation.m_x, newRotation.m_y, newRotation.m_z, newRotation.m_w));
	ndRenderSceneCamera::SetTransform(newRotation, targetMatrix.m_posit);

#if 0
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
#endif
}
