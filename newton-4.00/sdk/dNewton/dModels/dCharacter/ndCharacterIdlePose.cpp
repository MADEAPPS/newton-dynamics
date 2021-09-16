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
#include "ndCharacter.h"
#include "ndBodyDynamic.h"
#include "ndJointTwoBodyIK.h"
#include "ndCharacterRootNode.h"
#include "ndCharacterIdlePose.h"
#include "ndCharacterEffectorNode.h"
#include "ndCharacterPoseGenerator.h"
#include "ndCharacterBipedPoseController.h"

#define D_MIN_DISTANCE_TO_SUPPORT dFloat32 (0.01f)
#define D_MIN_DISTANCE_TO_SUPPORT2 (D_MIN_DISTANCE_TO_SUPPORT * D_MIN_DISTANCE_TO_SUPPORT)

ndCharacterIdlePose::ndCharacterIdlePose(ndCharacterBipedPoseController* const owner)
	:m_zeroMomentPoint(dVector::m_zero)
	,m_state(m_airborne)
	//,m_referencePose()
	,m_owner(owner)
	,m_invertedPendulumRadius(dFloat32 (0.0f))
{
}

void ndCharacterIdlePose::Init()
{
	const ndCharacter* const character = m_owner->GetCharacter();
	const ndBipedControllerConfig& config = m_owner->GetConfig();

	dVector p0;
	dVector p1;
	if (config.m_leftFootEffector)
	{
		//dVector leftFeetOffset(dFloat32(0.0f), dFloat32(0.0125f), dFloat32(0.0f), dFloat32(0.0f));
		//m_referencePose.PushBack(CalculateFeetKeyFrame(state.m_centerOfMass, leftFeetOffset, config.m_leftFootEffector));

		dMatrix matrix(config.m_leftFootNode->GetJoint()->GetLocalMatrix0() * config.m_leftFootNode->GetBody()->GetMatrix());
		config.m_leftFootEffector->SetTargetMatrix(matrix);
		p0 = matrix.m_posit;
	}

	if (config.m_rightFootEffector)
	{
		//dVector rightFeetOffset(dFloat32(-0.0f), dFloat32(0.0125f), dFloat32(0.0f), dFloat32(0.0f));
		//m_referencePose.PushBack(CalculateFeetKeyFrame(state.m_centerOfMass, rightFeetOffset, config.m_rightFootEffector));
		dMatrix matrix(config.m_rightFootNode->GetJoint()->GetLocalMatrix0() * config.m_rightFootNode->GetBody()->GetMatrix());
		config.m_rightFootEffector->SetTargetMatrix(matrix);
		p1 = matrix.m_posit;
	}

	// calculate the radius of the idle state
	ndCharacterCentreOfMassState state(character->CalculateCentreOfMassState());
	m_zeroMomentPoint = (p1 + p0).Scale(dFloat32(0.5f));
	m_zeroMomentPoint.m_w = dFloat32(1.0f);
	dVector radius(state.m_centerOfMass - m_zeroMomentPoint);
	m_invertedPendulumRadius = dSqrt(radius.DotProduct(radius & dVector::m_triplexMask).GetScalar());
}

//void ndCharacterIdlePose::SetEffectorMatrix(const dVector& localCom, const ndCharaterKeyFramePose& pose)
//void ndCharacterIdlePose::SetEffectorMatrix(const dVector&, const ndCharaterKeyFramePose& pose)
//{
//	if (pose.m_node)
//	{
//		dAssert(0);
//		//ndCharacterEffectorNode* const effector = pose.m_node->GetAsEffectorNode();
//		//dAssert(effector);
//		//
//		//const ndCharacter* const character = m_owner->GetCharacter();
//		//ndCharacterRootNode* const rootNode = character->GetRootNode();
//		//
//		//dMatrix matrix(pose.m_rotation, localCom + pose.m_position);
//		//matrix = matrix * rootNode->GetCoronalFrame();
//		//effector->SetTargetMatrix(matrix);
//	}
//}

void ndCharacterIdlePose::Update(dFloat32 timestep)
{
return;
	switch (m_state)
	{
		case m_airborne:
			AirBorneState(timestep);
			break;

		case m_twoFeet:
			TwoFeetState(timestep);
			break;

		default:
			dAssert(0);

	}
	//ndCharacterRootNode* const rootNode = character->GetRootNode();
	//dVector localCom (rootNode->GetInvCoronalFrame().TransformVector(rootNode->GetBody()->GetMatrix().UntransformVector(state.m_centerOfMass)));
	//
	//SetEffectorMatrix(localCom, m_referencePose[0]);
	//SetEffectorMatrix(localCom, m_referencePose[1]);

	//dVector zeroMomentPointInGlobalSpace;
	//if (m_owner->CalculateZeroMomentPoint(zeroMomentPointInGlobalSpace))
	//{
	//	const ndCharacter* const character = m_owner->GetCharacter();
	//	ndCharacterCentreOfMassState state(character->CalculateCentreOfMassState());
	//	
	//	dVector radius(state.m_centerOfMass - zeroMomentPointInGlobalSpace);
	//	dFloat32 invertedPendulumRadius = dSqrt(radius.DotProduct(radius & dVector::m_triplexMask).GetScalar());
	//
	//	dTrace(("r(%f %f %f) v(%f %f %f)\n", radius.m_x, radius.m_y, radius.m_z, 
	//		state.m_centerOfMassVeloc.m_x, state.m_centerOfMassVeloc.m_y, state.m_centerOfMassVeloc.m_z));
	//}
}


bool ndCharacterIdlePose::IsComSupported(const dVector& com) const
{
	dFixSizeArray<dVector, 32> supportPolygon;
	m_owner->CalculateSuportPolygon(supportPolygon);

	dAssert(supportPolygon.GetCount() >= 3);
	dVector normal(dVector::m_zero);
	for (dInt32 i = 2; i < supportPolygon.GetCount(); i++)
	{
		dVector e0(supportPolygon[i - 0] - supportPolygon[0]);
		dVector e1(supportPolygon[i - 1] - supportPolygon[0]);
		normal += e1.CrossProduct(e0);
	}
	normal.m_w = dFloat32(0.0f);
	normal = normal.Normalize();
	dPlane plane(normal, -normal.DotProduct(supportPolygon[0]).GetScalar());

	const ndCharacter* const character = m_owner->GetCharacter();
	const dVector gravityDir(character->GetRootNode()->GetGravityDir());

	dFloat32 den = gravityDir.DotProduct(plane).GetScalar();
	dAssert(dAbs(den) > dFloat32(0.0f));
	dVector pointInPlane(com + gravityDir.Scale(-plane.Evalue(com) / den));
	dInt32 i0 = supportPolygon.GetCount() - 1;
	for (dInt32 i = 0; i < supportPolygon.GetCount(); i++)
	{
		dVector e0(pointInPlane - supportPolygon[i0]);
		dVector e1(pointInPlane - supportPolygon[i]);
		dFloat32 side = e0.CrossProduct(e1).DotProduct(normal).GetScalar();
		if (side < dFloat32(0.0f))
		{
			return false;
		}

		i0 = i;
	}

	return true;
}

void ndCharacterIdlePose::GetHeelPoints(dFixSizeArray<dVector, 32>& points) const
{
	points.SetCount(0);
	const ndBipedControllerConfig& config = m_owner->GetConfig();
	{
		ndBodyKinematic* const leftFootBody = config.m_leftFootNode->GetBody();
		ndBodyKinematic::ndContactMap::Iterator iter(leftFootBody->GetContactMap());
		for (iter.Begin(); iter; iter++)
		{
			const ndContact* const contact = iter.GetNode()->GetInfo();
			if (contact->IsActive())
			{
				ndJointBilateralConstraint* const leftFootJoint = config.m_leftFootEffector->GetJoint();
				dVector p(leftFootJoint->GetBody0()->GetMatrix().TransformVector(leftFootJoint->GetLocalMatrix0().m_posit));
				points.PushBack(p);
				break;
			}
		}
	}

	{
		ndBodyKinematic* const righFootBody = config.m_rightFootNode->GetBody();
		ndBodyKinematic::ndContactMap::Iterator iter(righFootBody->GetContactMap());
		for (iter.Begin(); iter; iter++)
		{
			const ndContact* const contact = iter.GetNode()->GetInfo();
			if (contact->IsActive())
			{
				ndJointBilateralConstraint* const rightFootJoint = config.m_rightFootEffector->GetJoint();
				dVector p(rightFootJoint->GetBody0()->GetMatrix().TransformVector(rightFootJoint->GetLocalMatrix0().m_posit));
				points.PushBack(p);
				break;
			}
		}
	}
}

void ndCharacterIdlePose::AirBorneState(dFloat32 timestep)
{
	dFixSizeArray<dVector, 32> points;
	GetHeelPoints(points);

	if (points.GetCount() == 0)
	{
		// no state change
	}
	else if (points.GetCount() == 1)
	{
		dAssert(0);
	}
	else if (points.GetCount() == 2)
	{
		m_state = m_twoFeet;
		m_zeroMomentPoint = (points[0] + points[1]).Scale(dFloat32(0.5f));
		m_zeroMomentPoint.m_w = dFloat32(1.0f);
		TwoFeetState(timestep);
	}
}

void ndCharacterIdlePose::TwoFeetState(dFloat32 timestep)
{
	dFixSizeArray<dVector, 32> points;
	GetHeelPoints(points);

	if (points.GetCount() == 0)
	{
		m_state = m_airborne;
		AirBorneState(timestep);
	}
	else if (points.GetCount() == 1)
	{
		dAssert(0);
	}
	else if (points.GetCount() == 2)
	{
		// no state change
		const ndCharacter* const character = m_owner->GetCharacter();
		ndCharacterCentreOfMassState state(character->CalculateCentreOfMassState());
		if (!IsComSupported(state.m_centerOfMass))
		{
			// must enter recovering mode state
//			dAssert(0);
		}

		//static dFloat32 angle = 0.0f;
		//angle += 0.0005f;
		//dFloat32 r = 0.01f;
		//dVector offset(r * dCos(angle), 0.0f, 0.0f, 0.0f);
		//
		//static int xxxx;
		//xxxx++;
		//if (xxxx < 100)
		//	return;
		//static dMatrix xxxxx(character->GetRootNode()->GetBody()->GetMatrix());
		////dMatrix matrix (character->GetRootNode()->GetBody()->GetMatrix());
		//dMatrix matrix(character->GetRootNode()->GetBody()->GetMatrix());
		//matrix.m_posit = xxxxx.m_posit + offset;
		//
		//dMatrix matrix1(m_owner->GetConfig().m_leftFootEffector->GetReferenceMatrix() * matrix);
		//dMatrix matrix2(m_owner->GetConfig().m_rightFootEffector->GetReferenceMatrix() * matrix);
		//m_owner->GetConfig().m_leftFootEffector->SetTargetMatrix(matrix1);
		//m_owner->GetConfig().m_rightFootEffector->SetTargetMatrix(matrix2);
	

		// move the com to the apex of the inverted pendulum.
		//dVector zeroMomentPointInGlobalSpace;
		//if (m_owner->CalculateZeroMomentPoint(zeroMomentPointInGlobalSpace))
		//{
		//	//dAssert(0);
		//}
	}
	else
	{
		dAssert(0);
	}
}