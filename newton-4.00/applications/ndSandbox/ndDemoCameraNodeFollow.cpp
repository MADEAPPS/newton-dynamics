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
#include "ndPhysicsWorld.h"
#include "ndPhysicsUtils.h"
#include "ndDemoEntityManager.h"
#include "ndDemoCameraNodeFollow.h"

ndDemoCameraNodeFollow::ndDemoCameraNodeFollow(ndRender* const owner, const ndVector& pivot, ndFloat32 distance)
	:ndDemoCameraNode(owner)
	,m_pivot((pivot & ndVector::m_triplexMask) | ndVector::m_wOne)
	,m_yaw(ndFloat32(0.0f))
	,m_pitch(ndFloat32(0.0f))
	,m_yawRate(ndFloat32(0.02f))
	,m_pitchRate(ndFloat32(0.02f))
	,m_mousePosX(ndFloat32(0.0f))
	,m_mousePosY(ndFloat32(0.0f))
{
	ndRenderSceneCamera* const camera = FindCameraNode();
	ndTransform transform (camera->GetTransform());
	transform.m_position.m_x = distance;
	camera->SetTransform(transform);
	camera->SetTransform(transform);
}

void ndDemoCameraNodeFollow::SetTransform(const ndQuaternion& rotation, const ndVector& position)
{
	ndDemoCameraNode::SetTransform(rotation, position);
	const ndMatrix matrix(GetTransform().GetMatrix());
	m_pitch = ndAsin(matrix.m_front.m_y);
	m_yaw = ndAtan2(-matrix.m_front.m_z, matrix.m_front.m_x);
}

ndMatrix ndDemoCameraNodeFollow::CalculateLocalTransform() const
{
	ndAssert(m_parent);
	const ndMatrix globalMatrix(m_parent->CalculateGlobalTransform());
	ndMatrix uprightMatrix(ndGetIdentityMatrix());
	uprightMatrix.m_right = globalMatrix.m_front.CrossProduct(uprightMatrix.m_up).Normalize();
	uprightMatrix.m_front = uprightMatrix.m_up.CrossProduct(uprightMatrix.m_right).Normalize();
	uprightMatrix.m_posit = globalMatrix.m_posit;
	ndAssert(uprightMatrix.TestOrthogonal());
	const ndMatrix localUpMatrix(uprightMatrix * globalMatrix.OrthoInverse());
	ndMatrix camMatrix(ndRollMatrix(m_pitch) * ndYawMatrix(m_yaw) * localUpMatrix);
	camMatrix.m_posit = m_pivot;
	return camMatrix;
}

void ndDemoCameraNodeFollow::TickUpdate(ndFloat32)
{
	ndRender* const renderer = GetOwner();
	ndAssert(renderer);
	ndDemoEntityManager::ndRenderCallback* const renderCallback = (ndDemoEntityManager::ndRenderCallback*)*renderer->GetOwner();
	ndDemoEntityManager* const scene = renderCallback->m_owner;

	ndFloat32 mouseX;
	ndFloat32 mouseY;
	scene->GetMousePosition(mouseX, mouseY);

	const ndMatrix camMatrix(CalculateLocalTransform());
	ndDemoCameraNode::SetTransform(camMatrix, camMatrix.m_posit);

	bool mouseState = !scene->GetCaptured() && (scene->GetMouseKeyState(0) && !scene->GetMouseKeyState(1));
	// do camera rotation, only if we do not have anything picked
	if (!UpdatePickBody() && mouseState)
	{
		ndFloat32 mouseSpeedX = mouseX - m_mousePosX;
		ndFloat32 mouseSpeedY = mouseY - m_mousePosY;
	
		if (ImGui::IsMouseDown(0))
		{
			if (mouseSpeedX > 0.0f)
			{
				m_yaw = ndAnglesAdd(m_yaw, -m_yawRate);
			}
			else if (mouseSpeedX < 0.0f)
			{
				m_yaw = ndAnglesAdd(m_yaw, m_yawRate);
			}
	
			if (mouseSpeedY > 0.0f)
			{
				m_pitch -= m_pitchRate;
			}
			else if (mouseSpeedY < 0.0f)
			{
				m_pitch += m_pitchRate;
			}
			m_pitch = ndClamp(m_pitch, ndFloat32(-80.0f * ndDegreeToRad), ndFloat32(80.0f * ndDegreeToRad));
		}
	}
	
	m_mousePosX = mouseX;
	m_mousePosY = mouseY;
}
