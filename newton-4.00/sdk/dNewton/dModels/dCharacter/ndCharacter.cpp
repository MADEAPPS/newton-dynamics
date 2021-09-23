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
#include "ndCharacterNode.h"
#include "ndCharacterRootNode.h"
#include "ndJointKinematicChain.h"
#include "ndCharacterForwardDynamicNode.h"
#include "ndCharacterInverseDynamicNode.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndCharacter)

ndCharacter::ndCharacter()
	:ndModel()
	,m_rootNode(nullptr)
	,m_effectors()
	,m_extraJointAttachments()
{
}

ndCharacter::ndCharacter(const dLoadSaveBase::dLoadDescriptor& desc)
	:ndModel(dLoadSaveBase::dLoadDescriptor(desc))
	,m_rootNode(nullptr)
	,m_effectors()
	,m_extraJointAttachments()
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	dTree<const ndCharacterNode*, dUnsigned32> limbMap;
	for (const nd::TiXmlNode* node = xmlNode->FirstChild(); node; node = node->NextSibling())
	{
		const char* const partName = node->Value();
		if (strcmp(partName, "ndCharacterRootNode") == 0)
		{
			ndCharacterLoadDescriptor loadDesc(desc, &limbMap);
			loadDesc.m_rootNode = node;
			m_rootNode = new ndCharacterRootNode(loadDesc);
			m_rootNode->m_owner = this;
		}
	}
}

ndCharacter::~ndCharacter()
{
	for (dList<ndEffetorInfo>::dNode* node = m_effectors.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo().m_effector;
		delete joint;
	}

	for (dList<ndJointBilateralConstraint*>::dNode* node = m_extraJointAttachments.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo();
		delete joint;
	}

	if (m_rootNode)
	{
		delete m_rootNode;
	}
}

void ndCharacter::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndModel::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));

	if (m_rootNode)
	{
		dTree<dInt32, const ndCharacterNode*> limbMap;
		ndCharacterSaveDescriptor childDesc(desc);
		childDesc.m_rootNode = childNode;
		childDesc.m_limbMap = &limbMap;
		m_rootNode->Save(childDesc);
	}
}

void ndCharacter::AddToWorld(ndWorld* const world)
{
	if (m_rootNode)
	{
		world->AddBody(m_rootNode->GetBody());

		dInt32 stack = 0;
		ndCharacterNode* nodePool[32];
		for (ndCharacterNode* child = m_rootNode->GetChild(); child; child = child->GetSibling())
		{
			nodePool[stack] = child;
			stack++;
		}

		while (stack)
		{
			stack--;
			ndCharacterNode* const node = nodePool[stack];
			if (node->GetBody())
			{
				world->AddBody(node->GetBody());
			}

			if (node->GetJoint())
			{
				world->AddJoint(node->GetJoint());
			}

			for (ndCharacterNode* child = node->GetChild(); child; child = child->GetSibling())
			{
				nodePool[stack] = child;
				stack++;
			}
		}
	}

	for (dList<ndEffetorInfo>::dNode* node = m_effectors.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo().m_effector;
		world->AddJoint(joint);
	}

	for (dList<ndJointBilateralConstraint*>::dNode* node = m_extraJointAttachments.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo();
		world->AddJoint(joint);
	}
}

void ndCharacter::RemoveFromToWorld(ndWorld* const world)
{
	for (dList<ndEffetorInfo>::dNode* node = m_effectors.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo().m_effector;
		world->RemoveJoint(joint);
	}

	for (dList<ndJointBilateralConstraint*>::dNode* node = m_extraJointAttachments.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo();
		world->RemoveJoint(joint);
	}

	if (m_rootNode)
	{
		world->RemoveBody(m_rootNode->GetBody());

		dInt32 stack = 0;
		ndCharacterNode* nodePool[32];
		for (ndCharacterNode* child = m_rootNode->GetChild(); child; child = child->GetSibling())
		{
			nodePool[stack] = child;
			stack++;
		}

		while (stack)
		{
			stack--;
			ndCharacterNode* const node = nodePool[stack];
			if (node->GetBody())
			{
				world->RemoveBody(node->GetBody());
			}

			if (node->GetJoint())
			{
				world->RemoveJoint(node->GetJoint());
			}

			for (ndCharacterNode* child = node->GetChild(); child; child = child->GetSibling())
			{
				nodePool[stack] = child;
				stack++;
			}
		}
	}
}

ndCharacterRootNode* ndCharacter::CreateRoot(ndBodyDynamic* const body)
{
	m_rootNode = new ndCharacterRootNode(this, body);
	return m_rootNode;
}

ndCharacterForwardDynamicNode* ndCharacter::CreateForwardDynamicLimb(const dMatrix& matrixInGlobalSpase, ndBodyDynamic* const body, ndCharacterNode* const parent)
{
	ndCharacterForwardDynamicNode* const limb = new ndCharacterForwardDynamicNode(matrixInGlobalSpase, body, parent);
	return limb;
}

ndCharacterInverseDynamicNode* ndCharacter::CreateInverseDynamicLimb(const dMatrix& matrixInGlobalSpace, ndBodyDynamic* const body, ndCharacterNode* const parent)
{
	ndCharacterInverseDynamicNode* const limb = new ndCharacterInverseDynamicNode(matrixInGlobalSpace, body, parent);
	return limb;
}

//ndCharacterEffectorNode* ndCharacter::CreateInverseDynamicEffector(const dMatrix& matrixInGlobalSpace, ndCharacterNode* const parent)
//{
//	ndCharacterEffectorNode* const effector = new ndCharacterEffectorNode(matrixInGlobalSpace, parent);
//	return effector;
//}

//ndCharacterCentreOfMassState ndCharacter::CalculateCentreOfMassState() const
//{
//	dInt32 stack = 1;
//	ndCharacterNode* nodePool[32];
//	
//	nodePool[0] = m_rootNode;
//
//	dVector com(dVector::m_zero);
//	dVector veloc(dVector::m_zero);
//	dFloat32 mass = dFloat32(0.0f);
//
//	while (stack)
//	{
//		stack--;
//		ndCharacterNode* const node = nodePool[stack];
//		if (!node->GetAsEffectorNode())
//		{
//			ndBodyDynamic* const body = node->GetBody();
//			if (body)
//			{
//				dFloat32 partMass = body->GetMassMatrix().m_w;
//				mass += partMass;
//				dMatrix bodyMatrix(body->GetMatrix());
//				com += bodyMatrix.TransformVector(body->GetCentreOfMass()).Scale(partMass);
//				veloc += body->GetVelocity().Scale(partMass);
//			}
//		}
//
//		for (ndCharacterNode* child = node->GetChild(); child; child = child->GetSibling())
//		{
//			nodePool[stack] = child;
//			stack++;
//		}
//	}
//	//inertia.m_posit.m_w = dFloat32(1.0f);
//	dVector invMass (dFloat32(1.0f) / mass);
//	com = com * invMass;
//	veloc = veloc * invMass;
//	com.m_w = dFloat32(1.0f);
//	veloc.m_w = dFloat32(0.0f);
//
//	ndCharacterCentreOfMassState state;
//	state.m_mass = mass;
//	state.m_centerOfMass = com;
//	state.m_centerOfMassVeloc = veloc;
//	return state;
//}

void ndCharacter::Debug(ndConstraintDebugCallback& context) const
{
	dFloat32 scale = context.GetScale();
	context.SetScale(scale * 0.25f);

	for (dList<ndEffetorInfo>::dNode* node = m_effectors.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo().m_effector;
		joint->DebugJoint(context);
		node->GetInfo().m_controlNode->GetJoint()->DebugJoint(context);
	}

	if (m_rootNode)
	{
		m_rootNode->Debug(context);
	}

	//if (m_controller)
	//{
	//	m_controller->Debug(context);
	//}
	context.SetScale(scale);
}

void ndCharacter::AddAttachment(ndJointBilateralConstraint* const joint)
{
	m_extraJointAttachments.Append(joint);
}

void ndCharacter::RemoveAttachment(ndJointBilateralConstraint* const joint)
{
	for (dList<ndJointBilateralConstraint*>::dNode* node = m_extraJointAttachments.GetFirst(); node; node = node->GetNext())
	{
		if (node->GetInfo() == joint)
		{
			m_extraJointAttachments.Remove(node);
			break;
		}
	}
}

void ndCharacter::CreateKinematicChain(const dMatrix& globalOrientation, const ndCharacterNode* const footNode)
{
	ndCharacterNode* const calf = footNode->GetParent();
	ndCharacterNode* const leg = calf->GetParent();
	ndCharacterNode* const hip = leg->GetParent();

	dMatrix pinAndPivot(globalOrientation);
	dVector hipPivot(leg->GetBody()->GetMatrix().m_posit);
	pinAndPivot.m_posit = footNode->GetBody()->GetMatrix().m_posit;
	ndJointKinematicChain* const joint = new ndJointKinematicChain(hipPivot, pinAndPivot, footNode->GetBody(), hip->GetBody());

	ndEffetorInfo& info = m_effectors.Append()->GetInfo();
	info.m_effector = joint;
	info.m_controlNode = (ndCharacterNode*)footNode;
}

//void ndCharacter::UpdateGlobalPose(ndWorld* const world, dFloat32 timestep)
//{
//	dInt32 stack = 1;
//	ndCharacterNode* nodePool[32];
//	nodePool[0] = m_rootNode;
//
//	while (stack)
//	{
//		stack--;
//		ndCharacterNode* const node = nodePool[stack];
//		node->UpdateGlobalPose(world, timestep);
//
//		for (ndCharacterNode* child = node->GetChild(); child; child = child->GetSibling())
//		{
//			nodePool[stack] = child;
//			stack++;
//		}
//	}
//}

//void ndCharacter::CalculateLocalPose(ndWorld* const world, dFloat32 timestep)
//{
//	dInt32 stack = 1;
//	ndCharacterNode* nodePool[32];
//	nodePool[0] = m_rootNode;
//
//	while (stack)
//	{
//		stack--;
//		ndCharacterNode* const node = nodePool[stack];
//		node->CalculateLocalPose(world, timestep);
//
//		for (ndCharacterNode* child = node->GetChild(); child; child = child->GetSibling())
//		{
//			nodePool[stack] = child;
//			stack++;
//		}
//	}
//}

void ndCharacter::PostUpdate(ndWorld* const, dFloat32)
{
}

//void ndCharacter::Update(ndWorld* const world, dFloat32 timestep)
void ndCharacter::Update(ndWorld* const, dFloat32)
{
	//if (m_controller)
	//{
	//	m_controller->Evaluate(world, timestep);
	//}
}

//ndCharacterSkeleton* ndCharacter::CreateSkeleton() const
//{
//	dAssert(0);
//	return nullptr;
//	//dInt32 stack = 0;
//	//ndCharacterNode* nodePool[32];
//	//ndCharacterSkeleton* parentPool[32];
//	//
//	//ndCharacterSkeleton* const skeleton = new ndCharacterSkeleton(m_rootNode, dGetIdentityMatrix(), nullptr);
//	//for (ndCharacterNode* child = m_rootNode->GetChild(); child; child = child->GetSibling())
//	//{
//	//	nodePool[stack] = child;
//	//	parentPool[stack] = skeleton;
//	//	stack++;
//	//}
//	//
//	//while (stack)
//	//{
//	//	stack--;
//	//	ndCharacterNode* const node = nodePool[stack];
//	//	ndCharacterSkeleton* const boneParent = parentPool[stack];
//	//	dMatrix matrix(node->GetBoneMatrix() * boneParent->m_node->GetBoneMatrix().Inverse());
//	//	ndCharacterSkeleton* const parent = new ndCharacterSkeleton(node, matrix, boneParent);
//	//
//	//	for (ndCharacterNode* child = node->GetChild(); child; child = child->GetSibling())
//	//	{
//	//		nodePool[stack] = child;
//	//		parentPool[stack] = parent;
//	//		stack++;
//	//	}
//	//}
//	//
//	//return skeleton;
//}

void ndCharacter::SetPose()
{
	for (dList<ndEffetorInfo>::dNode* node = m_effectors.GetFirst(); node; node = node->GetNext())
	{
		ndEffetorInfo& info = node->GetInfo();
		ndJointKinematicChain* const joint = info.m_effector;
		
		dMatrix matrix(info.m_controlNode->GetLocalPose());
		for (ndCharacterNode* bone = info.m_controlNode->GetParent(); bone->GetBody() != joint->GetBody1(); bone = bone->GetParent())
		{
			matrix = matrix * bone->GetLocalPose();
		}

		dVector xxx(matrix.m_posit);
		matrix = joint->GetLocalMatrix1();
		matrix.m_posit = xxx;
		matrix.m_posit.m_x -= 0.1f;
		joint->SetTargetLocalMatrix(matrix);
	}
}