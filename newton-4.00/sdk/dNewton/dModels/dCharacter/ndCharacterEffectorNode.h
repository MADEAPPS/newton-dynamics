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

#ifndef __D_CHARACTER_EFFECTOR_NODE_H__
#define __D_CHARACTER_EFFECTOR_NODE_H__

#include "ndNewtonStdafx.h"
#include "ndCharacterLimbNode.h"

class ndJointPid6dofActuator;
class ndJointBilateralConstraint;

class ndCharacterEffectorNode: public ndCharacterLimbNode
{
	public:
	D_CLASS_RELECTION(ndCharacterEffectorNode);

	D_NEWTON_API ndCharacterEffectorNode(const dMatrix& matrixInGlobalScape, ndCharacterLimbNode* const child, ndCharacterLimbNode* const referenceNode);
	D_NEWTON_API virtual ~ndCharacterEffectorNode ();

	virtual ndJointBilateralConstraint* GetJoint() const;

	void SetTargetMatrix(const dVector& posit);

	protected:
	void UpdateGlobalPose(ndWorld* const world, dFloat32 timestep);

	ndJointPid6dofActuator* m_effector;
	ndCharacterLimbNode* m_referenceNode;

	//dMatrix m_globalPose;
};

inline ndJointBilateralConstraint* ndCharacterEffectorNode::GetJoint() const
{
	return (ndJointBilateralConstraint*)m_effector;
}

#endif