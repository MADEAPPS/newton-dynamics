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

#ifndef __D_CHARACTER_LIMB_NODE_H__
#define __D_CHARACTER_LIMB_NODE_H__

#include "ndNewtonStdafx.h"
#include "ndModel.h"

class ndWorld;
class ndCharacter;
class ndLimbJoint;
class ndCharacterRootNode;
class ndCharacterEffectorNode;
class ndJointBilateralConstraint;
class ndCharacterForwardDynamicNode;
class ndCharacterInverseDynamicNode;

class ndCharacterLoadDescriptor: public dLoadSaveBase::dLoadDescriptor
{
	public:
	ndCharacterLoadDescriptor()
		:dLoadSaveBase::dLoadDescriptor()
		,m_limbMap(nullptr)
	{
	}

	ndCharacterLoadDescriptor(const dLoadSaveBase::dLoadDescriptor& desc)
		:dLoadDescriptor(desc)
		,m_limbMap(nullptr)
	{
		ndCharacterLoadDescriptor* const modelDesc = (ndCharacterLoadDescriptor*)&desc;
		m_rootNode = desc.m_rootNode;
		m_limbMap = modelDesc->m_limbMap;
	}

	ndCharacterLoadDescriptor(const dLoadSaveBase::dLoadDescriptor& desc, dTree<const ndCharacterLimbNode*, dUnsigned32>* const limbMap)
		:dLoadDescriptor(desc)
		,m_limbMap(limbMap)
	{
	}

	ndCharacterLoadDescriptor(const ndCharacterLoadDescriptor& desc)
		:dLoadDescriptor(desc)
		,m_limbMap(desc.m_limbMap)
	{
	}

	dTree<const ndCharacterLimbNode*, dUnsigned32>* m_limbMap;
};

class ndCharacterSaveDescriptor: public dLoadSaveBase::dSaveDescriptor
{
	public:
	ndCharacterSaveDescriptor()
		:dLoadSaveBase::dSaveDescriptor()
		,m_limbMap(nullptr)
	{
	}

	ndCharacterSaveDescriptor(const dLoadSaveBase::dSaveDescriptor& desc)
		:dLoadSaveBase::dSaveDescriptor(desc)
		,m_limbMap(nullptr)
	{
	}

	ndCharacterSaveDescriptor(const ndCharacterSaveDescriptor& desc, nd::TiXmlNode* const rootNode)
		:dLoadSaveBase::dSaveDescriptor(desc, rootNode)
		,m_limbMap(desc.m_limbMap)
	{
	}

	dTree<dInt32, const ndCharacterLimbNode*>* m_limbMap;
};

class ndCharacterLimbNode: public dNodeHierarchy<ndCharacterLimbNode>
{
	public:
	D_CLASS_REFLECTION(ndCharacterLimbNode);
	D_NEWTON_API ndCharacterLimbNode(const ndCharacterLoadDescriptor& desc);
	D_NEWTON_API ndCharacterLimbNode(ndCharacterLimbNode* const parent);
	D_NEWTON_API virtual ~ndCharacterLimbNode ();

	virtual ndBodyDynamic* GetBody() const;
	virtual ndJointBilateralConstraint* GetJoint() const;

	virtual ndCharacterLimbNode* GetAsLimbNode();
	virtual ndCharacterRootNode* GetAsRootNode();
	virtual ndCharacterEffectorNode* GetAsEffectorNode();
	virtual ndCharacterForwardDynamicNode* GetAsForwardDynamicNode();
	virtual ndCharacterInverseDynamicNode* GetAsInverseDynamicNode();

	virtual void UpdateGlobalPose(ndWorld* const world, dFloat32 timestep);
	virtual void CalculateLocalPose(ndWorld* const world, dFloat32 timestep);

	virtual void Debug(ndConstraintDebugCallback& context) const;

	protected:
	D_NEWTON_API dNodeBaseHierarchy* CreateClone() const;
	D_NEWTON_API virtual void Save(const ndCharacterSaveDescriptor& desc) const;
};

inline ndBodyDynamic* ndCharacterLimbNode::GetBody() const
{
	return nullptr;
}

inline ndJointBilateralConstraint* ndCharacterLimbNode::GetJoint() const
{
	return nullptr;
}

inline void ndCharacterLimbNode::UpdateGlobalPose(ndWorld* const, dFloat32)
{
}

inline void ndCharacterLimbNode::CalculateLocalPose(ndWorld* const, dFloat32)
{
}

inline ndCharacterLimbNode* ndCharacterLimbNode::GetAsLimbNode()
{
	return this;
}

inline ndCharacterRootNode* ndCharacterLimbNode::GetAsRootNode()
{
	dAssert(0);
	return nullptr;
}

inline ndCharacterEffectorNode* ndCharacterLimbNode::GetAsEffectorNode()
{
	return nullptr;
}

inline ndCharacterInverseDynamicNode* ndCharacterLimbNode::GetAsInverseDynamicNode()
{
	dAssert(0);
	return nullptr;
}

inline ndCharacterForwardDynamicNode* ndCharacterLimbNode::GetAsForwardDynamicNode()
{
	dAssert(0);
	return nullptr;
}

#endif