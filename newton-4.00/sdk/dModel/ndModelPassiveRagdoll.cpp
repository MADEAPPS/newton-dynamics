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

#include "ndModelStdafx.h"
#include "ndModelPassiveRagdoll.h"

ndModelPassiveRagdoll::ndRagdollNode::ndRagdollNode(ndBodyDynamic* const body, ndRagdollNode* const parent)
	:ndNodeHierarchy<ndRagdollNode>()
	,m_body(body)
{
	if (parent)
	{
		Attach(parent);
	}
}

ndModelPassiveRagdoll::ndRagdollNode::~ndRagdollNode()
{
}

ndModelPassiveRagdoll::ndModelPassiveRagdoll()
	:ndModelBase()
	,m_rootNode(nullptr)
{
}

ndModelPassiveRagdoll::~ndModelPassiveRagdoll()
{
	if (m_rootNode)
	{
		delete m_rootNode;
	}
}

ndModelPassiveRagdoll::ndRagdollNode* ndModelPassiveRagdoll::GetRoot() const
{
	return m_rootNode;
}

ndModelPassiveRagdoll::ndRagdollNode* ndModelPassiveRagdoll::AddRootBody(ndSharedPtr<ndBody>& rootBody)
{
	ndAssert(!m_rootNode);
	m_rootNode = new ndRagdollNode(rootBody->GetAsBodyDynamic(), nullptr);
	m_bodies.Append(rootBody);
	return m_rootNode;
}

ndModelPassiveRagdoll::ndRagdollNode* ndModelPassiveRagdoll::AddLimb(ndRagdollNode* const parent, ndSharedPtr<ndBody>& body, ndSharedPtr<ndJointBilateralConstraint>& joint)
{
	ndAssert(m_rootNode);
	ndAssert(joint->GetBody1() == parent->m_body);
	ndAssert(joint->GetBody0() == body->GetAsBodyKinematic());
	ndRagdollNode* const node = new ndRagdollNode(body->GetAsBodyDynamic(), parent);
	m_bodies.Append(body);
	m_joints.Append(joint);
	return node;
}


//void NormalizeMassDistribution(ndFloat32 mass, const ndFixSizeArray<ndBodyDynamic*, 64>& bodyArray, const ndFixSizeArray<ndFloat32, 64>& massWeight) const
void ndModelPassiveRagdoll::NormalizeMassDistribution(ndFloat32 totalMass)
{
	ndFixSizeArray<ndBodyDynamic*, 256> bodyArray;
	ndFixSizeArray<ndModelPassiveRagdoll::ndRagdollNode*, 256> stack;
	if (GetRoot())
	{
		stack.PushBack(GetRoot());
		while (stack.GetCount())
		{
			ndInt32 index = stack.GetCount() - 1;
			ndModelPassiveRagdoll::ndRagdollNode* const node = stack[index];
			stack.SetCount(index);

			bodyArray.PushBack(node->m_body);
			for (ndModelPassiveRagdoll::ndRagdollNode* child = node->GetFirstChild(); child; child = child->GetNext())
			{
				stack.PushBack(child);
			}
		}
	}

	ndFloat32 volume = 0.0f;
	for (ndInt32 i = 0; i < bodyArray.GetCount(); ++i)
	{
		//volume += bodyArray[i]->GetCollisionShape().GetVolume() * massWeight[i];
		volume += bodyArray[i]->GetCollisionShape().GetVolume();
	}
	ndFloat32 density = totalMass / volume;
	
	for (ndInt32 i = 0; i < bodyArray.GetCount(); ++i)
	{
		ndBodyDynamic* const body = bodyArray[i];
		//ndFloat32 scale = density * body->GetCollisionShape().GetVolume() * massWeight[i];
		ndFloat32 scale = density * body->GetCollisionShape().GetVolume();
		ndVector inertia(body->GetMassMatrix().Scale(scale));
		body->SetMassMatrix(inertia);
	}
}
