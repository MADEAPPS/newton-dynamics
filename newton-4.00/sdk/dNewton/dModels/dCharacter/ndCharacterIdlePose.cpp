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
#include "ndCharacterRootNode.h"
#include "ndCharacterIdlePose.h"
#include "ndJointPid6dofActuator.h"
#include "ndCharacterEffectorNode.h"
#include "ndCharacterPoseGenerator.h"
#include "ndCharacterBipedPoseController.h"

#define D_MIN_DISTANCE_TO_SUPPORT dFloat32 (0.01f)
#define D_MIN_DISTANCE_TO_SUPPORT2 (D_MIN_DISTANCE_TO_SUPPORT * D_MIN_DISTANCE_TO_SUPPORT)

ndCharacterIdlePose::ndCharacterIdlePose(ndCharacterBipedPoseController* const owner)
	:m_angle(dFloat32(0.0f))
	,m_high(dFloat32(0.0f))
	,m_stride(dFloat32(0.0f))
	,m_owner(owner)
	,m_referencePose()
{
}

void ndCharacterIdlePose::Init()
{
	m_angle = dFloat32(0.0f);
	m_high = dFloat32(1.0f);
	m_stride = dFloat32(0.25f);

	const ndCharacter* const character = m_owner->GetCharacter();
	ndCharacterCentreOfMassState state(character->CalculateCentreOfMassState());

	ndCharacterRootNode* const rootNode = character->GetRootNode();
	dMatrix rootMatrix(rootNode->GetBody()->GetMatrix());
	rootMatrix.m_posit = state.m_centerOfMass;
	const dMatrix invRootMatrix(rootMatrix.Inverse());

	const dMatrix localFrame(rootNode->GetLocalFrame());
	const dMatrix invLocalFrame(localFrame.Inverse());
	const ndBipedControllerConfig& config = m_owner->GetConfig();
	{
		ndCharacterEffectorNode* const effector = config.m_leftFootEffector;
		ndJointPid6dofActuator* const joint = ((ndJointPid6dofActuator*)effector->GetJoint());
		dMatrix footMatrix(joint->GetReferenceMatrix() * joint->GetBody1()->GetMatrix() * invRootMatrix);

		footMatrix = footMatrix * invLocalFrame;
		footMatrix.m_posit.m_x = dFloat32 (0.0f);
		footMatrix = footMatrix * localFrame;
		m_referencePose.PushBack(ndCharaterKeyFramePose(effector, footMatrix));
	}

	{
		ndCharacterEffectorNode* const effector = config.m_rightFootEffector;
		ndJointPid6dofActuator* const joint = ((ndJointPid6dofActuator*)effector->GetJoint());
		dMatrix footMatrix(joint->GetReferenceMatrix() * joint->GetBody1()->GetMatrix() * invRootMatrix);

		footMatrix = footMatrix * invLocalFrame;
		footMatrix.m_posit.m_x = dFloat32(0.0f);
		footMatrix = footMatrix * localFrame;
		m_referencePose.PushBack(ndCharaterKeyFramePose(effector, footMatrix));
	}
}

//void ndCharacterIdlePose::MoveFoot(const ndCharacterCentreOfMassState& state, ndCharacterEffectorNode* const footEffector, dFloat32 angle)
void ndCharacterIdlePose::MoveFoot(const ndCharacterCentreOfMassState&, ndCharacterEffectorNode* const, dFloat32)
{
	dAssert(0);
	//const dFloat32 hipHigh = dFloat32(0.03f);
	//
	//dFloat32 y = hipHigh;
	//angle = dMod(angle, dFloat32(2.0f) * dPi);
	//dFloat32 x = m_stride * dSin(angle);
	//
	//dVector posit (x, y, dFloat32(0.0f), dFloat32(1.0f));
	//footEffector->SetTargetMatrix(posit);
}

////void ndCharacterIdlePose::BalanceCentreOfMass(dFloat32 timestep)
//void ndCharacterIdlePose::BalanceCentreOfMass(dFloat32)
//{
//	const ndCharacter* const character = m_owner->GetCharacter();
//	ndCharacter::ndCharacterCentreOfMassState state(character->CalculateCentreOfMassState());
//	const dRay supportPoint(m_owner->CalculateSupportPoint(state.m_centerOfMass));
//	dVector step(supportPoint.m_p0 - supportPoint.m_p1);
//	dFloat32 dist2 = step.DotProduct(step).GetScalar();
//	if (dist2 > D_MIN_DISTANCE_TO_SUPPORT2)
//	{
//		const ndBipedControllerConfig& config = m_owner->GetConfig();
//		const dMatrix leftFootMatrix(config.m_leftFootEffector->CalculateGlobalTargetMatrix());
//		const dMatrix rightFootMatrix(config.m_rightFootEffector->CalculateGlobalTargetMatrix());
//		dMatrix hipMatrix(character->GetRootNode()->GetBody()->GetMatrix());
//
//		dMatrix hipMatrix11(character->GetRootNode()->GetBody()->GetMatrix());
//	}
//}

void ndCharacterIdlePose::SetEffectorMatrix(const dVector& localCom, const ndCharaterKeyFramePose& pose)
{
	ndCharacterEffectorNode* const effector = pose.m_node->GetAsEffectorNode();
	dAssert(effector);

	//dMatrix matrix(dPitchMatrix(pose.m_pitch) * dPitchMatrix(pose.m_yaw) * dPitchMatrix(pose.m_roll));
	//matrix.m_posit = localCom + dVector(pose.m_x, pose.m_y, pose.m_z, dFloat32(0.0f));
	dVector posit(localCom + dVector(pose.m_x, pose.m_y, pose.m_z, dFloat32(0.0f)));
posit.m_x -= 0.025f;
	effector->SetTargetMatrix(posit, pose.m_pitch, pose.m_yaw, pose.m_roll);
}

//void ndCharacterIdlePose::Update(dFloat32 timestep)
void ndCharacterIdlePose::Update(dFloat32)
{
//	const ndBipedControllerConfig& config = m_owner->GetConfig();
//	const ndCharacter* const character = m_owner->GetCharacter();
//	ndCharacterCentreOfMassState state(character->CalculateCentreOfMassState());

//#if 1
//	// this stay up
//	if (config.m_rightFootEffector)
//	{
//		MoveFoot(state, config.m_rightFootEffector, m_angle + dFloat32(1.0f) * dPi);
//	}
//
//	if (config.m_leftFootEffector)
//	{
//		MoveFoot(state, config.m_leftFootEffector, m_angle + dFloat32(0.0f) * dPi);
//	}
//#else
//	// this fall ove the side
//	if (config.m_rightFootEffector)
//*	{
//		MoveFoot(config.m_rightFootEffector, m_angle + dFloat32(0.0f) * dPi);
//	}
//
//	if (config.m_leftFootEffector)
//	{
//		MoveFoot(config.m_leftFootEffector, m_angle + dFloat32(1.0f) * dPi);
//	}
//#endif
//
//	m_angle = 20.0f * dDegreeToRad;
//m_angle = 0.0f;

	const ndCharacter* const character = m_owner->GetCharacter();
	ndCharacterCentreOfMassState state(character->CalculateCentreOfMassState());

	ndCharacterRootNode* const rootNode = character->GetRootNode();
	dVector localCom (rootNode->GetBody()->GetMatrix().UntransformVector(state.m_centerOfMass));

	SetEffectorMatrix(localCom, m_referencePose[0]);
	SetEffectorMatrix(localCom, m_referencePose[1]);
}

