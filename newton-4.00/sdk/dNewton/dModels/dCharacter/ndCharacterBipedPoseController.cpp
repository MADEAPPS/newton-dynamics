/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndCharacter.h"
#include "ndBodyDynamic.h"
#include "ndCharacterRootNode.h"
#include "ndCharacterEffectorNode.h"
#include "ndCharacterBipedPoseController.h"

ndCharacterBipedPoseController::ndCharacterBipedPoseController()
	:ndCharacterPoseController(nullptr)
	,m_config()
	,m_idleCycle(this)
	,m_walkCycle(this)
{
}

ndCharacterBipedPoseController::~ndCharacterBipedPoseController()
{
}

void ndCharacterBipedPoseController::Init(ndCharacter* const owner, const ndBipedControllerConfig& config)
{
	m_owner = owner;
	m_config = config;
	m_idleCycle.Init();
	m_walkCycle.Init();
}

dRay ndCharacterBipedPoseController::CalculateSupportPoint(const dVector& comInGlobalSpace) const
{
	ndJointBilateralConstraint* const leftFootJoint = m_config.m_leftFootEffector->GetJoint();
	ndJointBilateralConstraint* const rightFootJoint = m_config.m_rightFootEffector->GetJoint();
	dMatrix leftFootMatrix(leftFootJoint->GetLocalMatrix0() * leftFootJoint->GetBody0()->GetMatrix());
	dMatrix rightFootMatrix(rightFootJoint->GetLocalMatrix0() * rightFootJoint->GetBody0()->GetMatrix());

	const dVector gravityDir(m_owner->GetRootNode()->GetGravityDir());
	const dVector p0(comInGlobalSpace);
	const dVector p1(comInGlobalSpace + gravityDir.Scale (dFloat32 (5.0f)));

	dFastRay supportSegment(p0, p1);
	dRay ray(supportSegment.RayDistance(leftFootMatrix.m_posit, rightFootMatrix.m_posit));
	return ray;
}

bool ndCharacterBipedPoseController::CalculateZeroMomentPoint(dVector& zeroMomentPointInGlobalSpace) const
{
	dFixSizeArray<dVector, 32> points;
	{
		ndBodyKinematic* const leftFootBody = m_config.m_leftFootNode->GetBody();
		ndBodyKinematic::ndContactMap::Iterator iter(leftFootBody->GetContactMap());
		for (iter.Begin(); iter; iter++)
		{
			const ndContact* const contact = iter.GetNode()->GetInfo();
			if (contact->IsActive())
			{
				ndJointBilateralConstraint* const leftFootJoint = m_config.m_leftFootEffector->GetJoint();
				dVector p(leftFootJoint->GetBody0()->GetMatrix().TransformVector(leftFootJoint->GetLocalMatrix0().m_posit));
				points.PushBack(p);
				break;
			}
		}
	}

	{
		ndBodyKinematic* const righFootBody = m_config.m_rightFootNode->GetBody();
		ndBodyKinematic::ndContactMap::Iterator iter(righFootBody->GetContactMap());
		for (iter.Begin(); iter; iter++)
		{
			const ndContact* const contact = iter.GetNode()->GetInfo();
			if (contact->IsActive())
			{
				ndJointBilateralConstraint* const rightFootJoint = m_config.m_rightFootEffector->GetJoint();
				dVector p(rightFootJoint->GetBody0()->GetMatrix().TransformVector(rightFootJoint->GetLocalMatrix0().m_posit));
				points.PushBack(p);
				break;
			}
		}
	}

	bool hasPoint = true;
	switch (points.GetCount()) 
	{
		case 0:
			hasPoint = false;
			break;

		case 1:
			zeroMomentPointInGlobalSpace = points[0];
			break;

		case 2:
			zeroMomentPointInGlobalSpace = (points[0] + points[1]).Scale (dFloat32 (0.5f));
			break;

		default:
			// here find the varicenter point of the conve hull
			dAssert(0);
	}

	return hasPoint;
}

void ndCharacterBipedPoseController::CalculateSuportPolygon(dFixSizeArray<dVector, 32>& supportPolygon) const
{
	if (m_config.m_leftFootNode)
	{
		ndBodyKinematic* const leftFootBody = m_config.m_leftFootNode->GetBody();
		ndBodyKinematic::ndContactMap::Iterator iter(leftFootBody->GetContactMap());
		for (iter.Begin(); iter; iter++)
		{
			const ndContact* const contact = iter.GetNode()->GetInfo();
			if (contact->IsActive())
			{
				const ndContactPointList& points = contact->GetContactPoints();
				if ((supportPolygon.GetCount() + points.GetCount()) < supportPolygon.GetCapacity())
				{
					for (ndContactPointList::dNode* node = points.GetFirst(); node; node = node->GetNext())
					{
						supportPolygon.PushBack(node->GetInfo().m_point);
					}
				}
			}
		}
	}

	if (m_config.m_rightFootNode)
	{
		ndBodyKinematic* const rightFootBody = m_config.m_rightFootNode->GetBody();
		ndBodyKinematic::ndContactMap::Iterator iter(rightFootBody->GetContactMap());
		for (iter.Begin(); iter; iter++)
		{
			const ndContact* const contact = iter.GetNode()->GetInfo();
			if (contact->IsActive())
			{
				const ndContactPointList& points = contact->GetContactPoints();
				if ((supportPolygon.GetCount() + points.GetCount()) < supportPolygon.GetCapacity())
				{
					for (ndContactPointList::dNode* node = points.GetFirst(); node; node = node->GetNext())
					{
						supportPolygon.PushBack(node->GetInfo().m_point);
					}
				}
			}
		}
	}
	supportPolygon.SetCount(dConvexHull2d(supportPolygon.GetCount(), &supportPolygon[0]));
}

void ndCharacterBipedPoseController::Debug(ndConstraintDebugCallback& context) const
{
	ndCharacterRootNode* const rootNode = m_owner->GetRootNode();
	ndBodyDynamic* const hip = rootNode->GetBody();

	dMatrix comMatrixInGlobalSpace(rootNode->GetCoronalFrame() * hip->GetMatrix());

	// show character center of mass.
	ndCharacterCentreOfMassState state(m_owner->CalculateCentreOfMassState());
	comMatrixInGlobalSpace.m_posit = state.m_centerOfMass;
	context.DrawFrame(comMatrixInGlobalSpace);

	//m_config.m_leftFootEffector->GetJoint()->DebugJoint(context);
	m_config.m_rightFootEffector->GetJoint()->DebugJoint(context);
	//m_config.m_leftFootNode->GetJoint()->DebugJoint(context);
	//m_config.m_rightFootNode->GetJoint()->DebugJoint(context);

return;
	const dRay suportPoint(CalculateSupportPoint(comMatrixInGlobalSpace.m_posit));

	ndJointBilateralConstraint* const leftFootJoint = m_config.m_leftFootEffector->GetJoint();
	ndJointBilateralConstraint* const rightFootJoint = m_config.m_rightFootEffector->GetJoint();
	dMatrix leftFootMatrix(leftFootJoint->GetLocalMatrix0() * leftFootJoint->GetBody0()->GetMatrix());
	dMatrix rightFootMatrix(rightFootJoint->GetLocalMatrix0() * rightFootJoint->GetBody0()->GetMatrix());

	context.DrawLine(leftFootMatrix.m_posit, rightFootMatrix.m_posit, dVector(dFloat32(1.0f), dFloat32(0.0f), dFloat32(1.0f), dFloat32(1.0f)));
	context.DrawLine(comMatrixInGlobalSpace.m_posit, suportPoint.m_p0, dVector(dFloat32(1.0f), dFloat32(1.0f), dFloat32(0.0f), dFloat32(1.0f)));
	context.DrawLine(suportPoint.m_p0, suportPoint.m_p1, dVector(dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f), dFloat32(1.0f)), dFloat32(2.0f));

	dFixSizeArray<dVector, 32> supportPolygon;
	CalculateSuportPolygon(supportPolygon);
	if (supportPolygon.GetCount())
	{
		dVector offset(m_owner->GetRootNode()->GetGravityDir().Scale(dFloat32(0.01f)));
		dVector p0(supportPolygon[supportPolygon.GetCount() - 1] - offset);
		for (dInt32 i = 0; i < supportPolygon.GetCount(); i++)
		{
			dVector p1(supportPolygon[i] - offset);
			context.DrawLine(p0, p1, dVector(dFloat32(1.0f), dFloat32(1.0f), dFloat32(0.0f), dFloat32(1.0f)));
			p0 = p1;
		}
	}
}

bool ndCharacterBipedPoseController::Evaluate(ndWorld* const , dFloat32 timestep)
{
	//ndCharacter::ndCharacterCentreOfMassState comState(m_owner->CalculateCentreOfMassState());
	//m_owner->UpdateGlobalPose(world, timestep);
	//m_owner->CalculateLocalPose(world, timestep);

	//m_walkCycle.Update(timestep);
	m_idleCycle.Update(timestep);
	return true;
}


