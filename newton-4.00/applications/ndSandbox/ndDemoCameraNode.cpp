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
#include "ndDemoCameraNode.h"
#include "ndDemoEntityManager.h"

ndDemoCameraNode::ndDemoCameraNode(ndRender* const owner)
	:ndRenderSceneNode(ndGetIdentityMatrix())
	,m_pickedBodyTargetPosition(ndVector::m_wOne)
	,m_pickJoint(nullptr)
	,m_pickedBodyParam(ndFloat32(0.0f))
	,m_pickingMode(false)
	,m_prevMouseState(false)
{
	m_owner = owner;
	ndSharedPtr<ndRenderSceneNode> camera(new ndRenderSceneCamera(owner));
	AddChild(camera);
}

bool ndDemoCameraNode::UpdatePickBody()
{
	// handle pick body from the screen
	ndRender* const renderer = GetOwner();
	ndAssert(renderer);
	ndDemoEntityManager::ndRenderCallback* const renderCallback = (ndDemoEntityManager::ndRenderCallback*)*renderer->GetOwner();
	ndDemoEntityManager* const scene = renderCallback->m_owner;
	ndWorld* const world = scene->GetWorld();

	ndFloat32 mouseX;
	ndFloat32 mouseY;
	scene->GetMousePosition(mouseX, mouseY);

	const ndRenderSceneCamera* const camera = FindCameraNode();
	const ndVector p0(camera->ScreenToWorld(ndVector(mouseX, mouseY, ndFloat32(0.0f), ndFloat32(0.0f))));
	const ndVector p1(camera->ScreenToWorld(ndVector(mouseX, mouseY, ndFloat32(1.0f), ndFloat32(0.0f))));

	bool mousePickState = !scene->GetCaptured() && (scene->GetMouseKeyState(0) && !scene->GetMouseKeyState(1));

	bool retValue = false;
	if (!*m_pickJoint)
	{
		if (!m_prevMouseState && mousePickState)
		{
			retValue = true;
			class ndRayPickingCallback : public ndRayCastClosestHitCallback
			{
				public:
				ndRayPickingCallback()
					:ndRayCastClosestHitCallback()
				{
				}

				ndFloat32 OnRayCastAction(const ndContactPoint& contact, ndFloat32 intersetParam)
				{
					if (contact.m_body0->GetInvMass() == ndFloat32(0.0f))
					{
						return 1.2f;
					}
					return ndRayCastClosestHitCallback::OnRayCastAction(contact, intersetParam);
				}
			};

			ndRayPickingCallback rayCaster;
			if (world->RayCast(rayCaster, p0, p1))
			{
				ndBodyKinematic* const body = (ndBodyKinematic*)rayCaster.m_contact.m_body0;
				ndBodyNotify* const notify = body->GetNotifyCallback();
				if (notify)
				{
					ndTrace(("picked body id: %d\n", body->GetId()));
					m_pickedBodyParam = rayCaster.m_param;

					if (body->GetAsBodyDynamic())
					{
						ndVector mass(body->GetMassMatrix());

						//change this to make the grabbing stronger or weaker
						const ndFloat32 angularFritionAccel = 10.0f;
						const ndFloat32 linearFrictionAccel = 40.0f * ndMax(ndAbs(DEMO_GRAVITY), ndFloat32(10.0f));
						const ndFloat32 inertia = ndMax(mass.m_z, ndMax(mass.m_x, mass.m_y));

						ndDemoCameraPickBodyJoint* const pickJoint = new ndDemoCameraPickBodyJoint(body, scene->GetWorld()->GetSentinelBody(), rayCaster.m_contact.m_point, this);
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
			retValue = true;
			m_pickedBodyTargetPosition = p0 + (p1 - p0).Scale(m_pickedBodyParam);

			if (*m_pickJoint)
			{
				ndDemoCameraPickBodyJoint* const pickJoint = (ndDemoCameraPickBodyJoint*)*m_pickJoint;
				pickJoint->SetTargetPosit(m_pickedBodyTargetPosition);
			}
		}
		else
		{
			world->RemoveJoint(*m_pickJoint);
			ResetPickBody();
		}
	}

	m_prevMouseState = mousePickState;
	return retValue;
}

void ndDemoCameraNode::ResetPickBody()
{
	if (*m_pickJoint)
	{
		m_pickJoint->GetBody0()->SetSleepState(false);
		ndDemoCameraPickBodyJoint* const pickJoint = (ndDemoCameraPickBodyJoint*)*m_pickJoint;
		pickJoint->m_owner = nullptr;
	}
	m_pickJoint = ndSharedPtr<ndJointBilateralConstraint>(nullptr);
}