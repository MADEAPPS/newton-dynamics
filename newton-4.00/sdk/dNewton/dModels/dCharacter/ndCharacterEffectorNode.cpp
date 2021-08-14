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
#include "ndJointPid6dofActuator.h"
#include "ndCharacterEffectorNode.h"

ndCharacterEffectorNode::ndCharacterEffectorNode(const dMatrix& matrixInGlobalScape, ndCharacterLimbNode* const child, ndCharacterLimbNode* const referenceNode)
	:ndCharacterLimbNode(child)
	,m_referenceNode(referenceNode)
{
	ndBodyDynamic* const body0 = child->GetBody();
	ndBodyDynamic* const body1 = referenceNode->GetBody();
	m_effector = new ndJointPid6dofActuator(matrixInGlobalScape, body0, body1);
}

ndCharacterEffectorNode::~ndCharacterEffectorNode()
{
}

//void ndCharacterEffectorNode::SetTargetPosition(const dVector& posit)
//{
//	dAssert(0);
//	m_effector->SetTargetPosition(posit);
//}

void ndCharacterEffectorNode::SetTargetMatrix(const dVector& posit, dFloat32 pitch, dFloat32 yaw, dFloat32 roll)
{
	m_effector->SetTargetPosition(posit);
	m_effector->SetTargetRotation(pitch, yaw, roll);
}

//dMatrix ndCharacterEffectorNode::CalculateGlobalTargetMatrix() const
//{
//	return m_effector->CalculateGlobalTargetMatrix();
//}

//void ndCharacterEffectorNode::UpdateGlobalPose(ndWorld* const, dFloat32)
//{
//	// for now just; 
//	//ndBodyDynamic* const body = m_effector->GetBody0()->GetAsBodyDynamic();
//	//m_globalPose = body->GetMatrix();
//}
