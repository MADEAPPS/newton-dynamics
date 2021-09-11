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
{
}

void ndCharacterIdlePose::Init()
{
	const ndCharacter* const character = m_owner->GetCharacter();
	ndCharacterCentreOfMassState state(character->CalculateCentreOfMassState());

	const ndBipedControllerConfig& config = m_owner->GetConfig();
	if (config.m_leftFootEffector)
	{
		dVector leftFeetOffset(dFloat32(0.0f), dFloat32(0.0125f), dFloat32(0.0f), dFloat32(0.0f));
		m_referencePose.PushBack(CalculateFeetKeyFrame(state.m_centerOfMass, leftFeetOffset, config.m_leftFootEffector));
	}

	if (config.m_rightFootEffector)
	{
		dVector rightFeetOffset(dFloat32(-0.0f), dFloat32(0.0125f), dFloat32(0.0f), dFloat32(0.0f));
		m_referencePose.PushBack(CalculateFeetKeyFrame(state.m_centerOfMass, rightFeetOffset, config.m_rightFootEffector));
	}
}

ndCharaterKeyFramePose ndCharacterIdlePose::CalculateFeetKeyFrame(const dVector& centerOfMass, const dVector& keyFrameOffset, ndCharacterEffectorNode* const effector)
{
	const ndCharacter* const character = m_owner->GetCharacter();
	ndCharacterRootNode* const rootNode = character->GetRootNode();
	
	dMatrix rootMatrix(rootNode->GetBody()->GetMatrix());
	rootMatrix.m_posit = centerOfMass;
	const dMatrix invRootMatrix(rootMatrix.Inverse());
	
	ndJointTwoBodyIK* const joint = (ndJointTwoBodyIK*)effector->GetJoint();
	dMatrix footMatrix(joint->GetReferenceMatrix() * joint->GetBody1()->GetMatrix() * invRootMatrix);
	
	const dMatrix invLocalFrame(rootNode->GetInvCoronalFrame());
	footMatrix = footMatrix * invLocalFrame;
	// snap location to center of mass only in the coronal plane
	footMatrix.m_posit.m_x = dFloat32(0.0f);
	// add offset
	footMatrix.m_posit += keyFrameOffset;
	return ndCharaterKeyFramePose(effector, footMatrix);
}

void ndCharacterIdlePose::SetEffectorMatrix(const dVector& localCom, const ndCharaterKeyFramePose& pose)
{
	if (pose.m_node)
	{
		ndCharacterEffectorNode* const effector = pose.m_node->GetAsEffectorNode();
		dAssert(effector);

		const ndCharacter* const character = m_owner->GetCharacter();
		ndCharacterRootNode* const rootNode = character->GetRootNode();

		dMatrix matrix(pose.m_rotation, localCom + pose.m_position);
		//matrix.m_posit.m_y += 0.0125f;
		//matrix.m_posit.m_x -= 0.025f;
		matrix = matrix * rootNode->GetCoronalFrame();
		effector->SetTargetMatrix(matrix);
	}
}

//void ndCharacterIdlePose::Update(dFloat32 timestep)
void ndCharacterIdlePose::Update(dFloat32)
{
	const ndCharacter* const character = m_owner->GetCharacter();
	ndCharacterCentreOfMassState state(character->CalculateCentreOfMassState());

	ndCharacterRootNode* const rootNode = character->GetRootNode();
	dVector localCom (rootNode->GetInvCoronalFrame().TransformVector(rootNode->GetBody()->GetMatrix().UntransformVector(state.m_centerOfMass)));

	SetEffectorMatrix(localCom, m_referencePose[0]);
	SetEffectorMatrix(localCom, m_referencePose[1]);
}

