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
	:m_referencePose()
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

	ndCharacterCentreOfMassState state(character->CalculateCentreOfMassState());
	dVector xmp((p1 + p0).Scale(dFloat32(0.5f)));
	dVector radius(state.m_centerOfMass - xmp);
	m_invertedPendulumRadius = dSqrt(radius.DotProduct(radius & dVector::m_triplexMask).GetScalar());
}

//void ndCharacterIdlePose::SetEffectorMatrix(const dVector& localCom, const ndCharaterKeyFramePose& pose)
void ndCharacterIdlePose::SetEffectorMatrix(const dVector&, const ndCharaterKeyFramePose& pose)
{
	if (pose.m_node)
	{
		dAssert(0);
		//ndCharacterEffectorNode* const effector = pose.m_node->GetAsEffectorNode();
		//dAssert(effector);
		//
		//const ndCharacter* const character = m_owner->GetCharacter();
		//ndCharacterRootNode* const rootNode = character->GetRootNode();
		//
		//dMatrix matrix(pose.m_rotation, localCom + pose.m_position);
		//matrix = matrix * rootNode->GetCoronalFrame();
		//effector->SetTargetMatrix(matrix);
	}
}

//void ndCharacterIdlePose::Update(dFloat32 timestep)
void ndCharacterIdlePose::Update(dFloat32)
{
	//ndCharacterRootNode* const rootNode = character->GetRootNode();
	//dVector localCom (rootNode->GetInvCoronalFrame().TransformVector(rootNode->GetBody()->GetMatrix().UntransformVector(state.m_centerOfMass)));
	//
	//SetEffectorMatrix(localCom, m_referencePose[0]);
	//SetEffectorMatrix(localCom, m_referencePose[1]);

	dVector zeroMomentPointInGlobalSpace;
	if (m_owner->CalculateZeroMomentPoint(zeroMomentPointInGlobalSpace))
	{
		const ndCharacter* const character = m_owner->GetCharacter();
		ndCharacterCentreOfMassState state(character->CalculateCentreOfMassState());
		
		dVector radius(state.m_centerOfMass - zeroMomentPointInGlobalSpace);

		dTrace(("r(%f %f %f) v(%f %f %f)\n", radius.m_x, radius.m_y, radius.m_z, 
			state.m_centerOfMassVeloc.m_x, state.m_centerOfMassVeloc.m_y, state.m_centerOfMassVeloc.m_z));
	}
}

