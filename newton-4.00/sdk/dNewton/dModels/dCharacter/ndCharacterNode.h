/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_CHARACTER_LIMB_NODE_H__
#define __ND_CHARACTER_LIMB_NODE_H__

#include "ndNewtonStdafx.h"
#include "ndModel.h"

class ndWorld;
class ndCharacter;
class ndLimbJoint;
class ndCharacterRootNode;
class ndCharacterSkeleton;
class ndJointBilateralConstraint;
class ndCharacterForwardDynamicNode;
class ndCharacterInverseDynamicNode;

class ndCharacterNode: public ndNodeHierarchy<ndCharacterNode>
{
	public:
	D_BASE_CLASS_REFLECTION(ndCharacterNode)
	D_NEWTON_API ndCharacterNode(ndCharacterNode* const parent);
	D_NEWTON_API virtual ~ndCharacterNode ();

	virtual ndBodyDynamic* GetBody() const;
	virtual ndJointBilateralConstraint* GetJoint() const;

	virtual ndCharacterNode* GetAsNode();
	virtual ndCharacterRootNode* GetAsRootNode();
	virtual ndCharacterForwardDynamicNode* GetAsForwardDynamicNode();
	virtual ndCharacterInverseDynamicNode* GetAsInverseDynamicNode();

	const ndMatrix& GetLocalPose() const;
	void SetLocalPose(const ndMatrix& matrix);

	//virtual void UpdateGlobalPose(ndWorld* const world, dFloat32 timestep);
	//virtual void CalculateLocalPose(ndWorld* const world, dFloat32 timestep);
	virtual void Debug(ndConstraintDebugCallback& context) const;

	//D_NEWTON_API virtual ndMatrix GetBoneMatrix() const;

	protected:
	D_NEWTON_API ndCharacterNode* CreateClone() const;
	ndMatrix m_localPose;
};

inline ndBodyDynamic* ndCharacterNode::GetBody() const
{
	return nullptr;
}

inline ndJointBilateralConstraint* ndCharacterNode::GetJoint() const
{
	return nullptr;
}

//inline void ndCharacterNode::UpdateGlobalPose(ndWorld* const, dFloat32)
//{
//}
//
//inline void ndCharacterNode::CalculateLocalPose(ndWorld* const, dFloat32)
//{
//}

inline ndCharacterNode* ndCharacterNode::GetAsNode()
{
	return this;
}

inline ndCharacterRootNode* ndCharacterNode::GetAsRootNode()
{
	ndAssert(0);
	return nullptr;
}

inline ndCharacterInverseDynamicNode* ndCharacterNode::GetAsInverseDynamicNode()
{
	ndAssert(0);
	return nullptr;
}

inline ndCharacterForwardDynamicNode* ndCharacterNode::GetAsForwardDynamicNode()
{
	ndAssert(0);
	return nullptr;
}

inline const ndMatrix& ndCharacterNode::GetLocalPose() const
{
	return m_localPose;
}

inline void ndCharacterNode::SetLocalPose(const ndMatrix& matrix)
{
	m_localPose = matrix;
}

#endif