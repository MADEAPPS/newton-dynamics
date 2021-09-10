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
	D_CLASS_REFLECTION(ndCharacterEffectorNode);
	D_NEWTON_API ndCharacterEffectorNode(const ndCharacterLoadDescriptor& desc);
	D_NEWTON_API ndCharacterEffectorNode(const dMatrix& matrixInGlobalScape, ndCharacterLimbNode* const child, ndCharacterLimbNode* const referenceNode);
	D_NEWTON_API virtual ~ndCharacterEffectorNode ();

	virtual ndBodyDynamic* GetBody() const;
	virtual ndJointBilateralConstraint* GetJoint() const;
	virtual ndCharacterEffectorNode* GetAsEffectorNode();

	//dMatrix CalculateGlobalTargetMatrix() const;

	void SetTargetMatrix(const dMatrix& matrix);

	protected:
	void Save(const ndCharacterSaveDescriptor& desc) const;
	//void UpdateGlobalPose(ndWorld* const world, dFloat32 timestep);

	void Debug(ndConstraintDebugCallback& context) const;

	ndJointPid6dofActuator* m_effector;
	ndCharacterLimbNode* m_referenceNode;
};

inline ndJointBilateralConstraint* ndCharacterEffectorNode::GetJoint() const
{
	return (ndJointBilateralConstraint*)m_effector;
}

inline ndBodyDynamic* ndCharacterEffectorNode::GetBody() const
{
	ndBodyDynamic* const body = GetParent()->GetBody();
	dAssert(GetJoint()->GetBody0()->GetAsBodyDynamic() == body);
	return body;
}

inline ndCharacterEffectorNode* ndCharacterEffectorNode::GetAsEffectorNode()
{
	return this;
}

#endif