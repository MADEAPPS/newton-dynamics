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
}

void ndCharacterBipedPoseController::Debug(ndConstraintDebugCallback& context) const
{
	ndCharacterRootNode* const rootNode = m_owner->GetRootNode();
	ndBodyDynamic* const hip = rootNode->GetBody();
	
	dMatrix matrix(rootNode->GetLocalFrame() * hip->GetMatrix());
	
	// show character center of mass.
	ndCharacter::ndCentreOfMassState state(m_owner->CalculateCentreOfMassState());
	matrix.m_posit = state.m_centerOfMass;
	context.DrawFrame(matrix);

	ndBodyKinematic* const leftFootBody = m_config.m_leftFootEffector->GetJoint()->GetBody0();
	ndBodyKinematic* const rightFootBody = m_config.m_rightFootEffector->GetJoint()->GetBody0();
	dVector leftFootCenter(leftFootBody->GetMatrix().TransformVector(leftFootBody->GetCentreOfMass()));
	dVector rightFootCenter(rightFootBody->GetMatrix().TransformVector(rightFootBody->GetCentreOfMass()));
	context.DrawLine(leftFootCenter, rightFootCenter, dVector(dFloat32(1.0f), dFloat32(0.0f), dFloat32(1.0f), dFloat32(1.0f)));

	dVector xxxx(matrix.m_posit);
	xxxx.m_y -= 0.9f;
	context.DrawLine(matrix.m_posit, xxxx, dVector(dFloat32(1.0f), dFloat32(1.0f), dFloat32(0.0f), dFloat32(1.0f)));

	dVector xxxxx0(matrix.m_posit);
	xxxxx0.m_y -= 0.91f;
	dVector xxxxx1(xxxxx0);
	xxxxx1.m_y -= 0.002f;
	context.DrawLine(xxxxx0, xxxxx1, dVector(dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f), dFloat32(1.0f)), dFloat32 (8.0f));
}

bool ndCharacterBipedPoseController::Evaluate(ndWorld* const , dFloat32 timestep)
{
	//ndCharacter::ndCentreOfMassState comState(m_owner->CalculateCentreOfMassState());
	//m_owner->UpdateGlobalPose(world, timestep);
	//m_owner->CalculateLocalPose(world, timestep);

	//m_walkCycle.Update(timestep);
	m_idleCycle.Update(timestep);
	return true;
}


