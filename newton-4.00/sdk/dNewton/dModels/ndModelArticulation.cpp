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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndModel.h"
#include "ndUrdfFile.h"
#include "ndBodyDynamic.h"
#include "ndModelArticulation.h"

ndModelArticulation::ndCenterOfMassDynamics::ndCenterOfMassDynamics()
	:m_omega(ndVector::m_zero)
	,m_veloc(ndVector::m_zero)
	,m_alpha(ndVector::m_zero)
	,m_accel(ndVector::m_zero)
	,m_force(ndVector::m_zero)
	,m_torque(ndVector::m_zero)
	,m_momentum(ndVector::m_zero)
	,m_angularMomentum(ndVector::m_zero)
	,m_centerOfMass(ndGetIdentityMatrix())
	,m_inertiaMatrix(ndGetZeroMatrix())
	,m_mass(ndFloat32 (0.0f))
{
}

ndModelArticulation::ndNode::ndNode(const ndSharedPtr<ndBody>& body, const ndSharedPtr<ndJointBilateralConstraint>& joint, ndNode* const parent)
	:ndNodeHierarchy<ndNode>()
	,m_body(body)
	,m_joint(joint)
	,m_name("")
{
	if (parent)
	{
		Attach(parent);
	}
}

ndModelArticulation::ndNode::ndNode(const ndNode& src)
	:ndNodeHierarchy<ndNode>(src)
	,m_body(src.m_body)
	,m_joint(src.m_joint)
	,m_name(src.m_name)
{
}

ndModelArticulation::ndNode::~ndNode()
{
}

ndModelArticulation::ndModelArticulation()
	:ndModel()
	,m_name("")
	,m_rootNode(nullptr)
	,m_closeLoops()
{
}

ndModelArticulation::ndModelArticulation(const ndModelArticulation& src)
	:ndModel(src)
	,m_name(src.m_name)
	,m_rootNode(nullptr)
	,m_closeLoops()
{
	ndAssert(0);
	ndAssert(src.GetRoot()->m_body->GetAsBodyDynamic());

	ndFixSizeArray<ndNode*, 256> stack;
	stack.PushBack(src.GetRoot());
	while (stack.GetCount())
	{
		ndNode* const node = stack.Pop();
		AddRootBody(new ndBodyDynamic(*node->m_body->GetAsBodyDynamic()));
		if (*node->m_joint)
		{

		}

		for (ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
		{
			stack.PushBack(child);
		}
	}
}

ndModelArticulation::~ndModelArticulation()
{
	if (m_rootNode)
	{
		delete m_rootNode;
	}
}

ndModel* ndModelArticulation::Clone() const
{
	return new ndModelArticulation(*this);
}

ndModelArticulation* ndModelArticulation::GetAsModelArticulation()
{
	return this;
}

const ndString& ndModelArticulation::GetName() const
{
	return m_name;
}

void ndModelArticulation::SetName(const ndString& name)
{
	m_name = name;
}

ndModelArticulation::ndNode* ndModelArticulation::GetRoot() const
{
	return m_rootNode;
}

void ndModelArticulation::ClearMemory()
{
	if (m_rootNode)
	{
		for (ndNode* node = m_rootNode->GetFirstIterator(); node; node = node->GetNextIterator())
		{
			if (*node->m_joint)
			{
				node->m_joint->ClearMemory();
			}

			//ndBodyKinematic::ndJointList& jointList = body->GetJointList();
			//for (ndBodyKinematic::ndJointList::ndNode* jointNode = jointList.GetFirst(); jointNode; jointNode = jointNode->GetNext())
			//{
			//	jointNode->GetInfo()->ClearMemory();
			//}

			ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
			ndBodyKinematic::ndContactMap& contactMap = body->GetContactMap();
			ndBodyKinematic::ndContactMap::Iterator it(contactMap);
			for (it.Begin(); it; it++)
			{
				ndContact* const contact = it.GetNode()->GetInfo();
				contact->ClearMemory();
			}
		}
	}

	for (ndList<ndNode>::ndNode* node = m_closeLoops.GetFirst(); node; node = node->GetNext())
	{
		node->GetInfo().m_joint->ClearMemory();
	}
}

ndModelArticulation::ndNode* ndModelArticulation::AddRootBody(const ndSharedPtr<ndBody>& rootBody)
{
	ndAssert(!m_rootNode);
	ndSharedPtr <ndJointBilateralConstraint> dummyJoint;
	m_rootNode = new ndNode(rootBody, dummyJoint, nullptr);

	ndBodyKinematic* const dynBody = rootBody->GetAsBodyDynamic();
	ndAssert(dynBody);
	if (dynBody)
	{
		dynBody->SetModel(this);
	}
	return m_rootNode;
}

ndModelArticulation::ndNode* ndModelArticulation::AddLimb(ndNode* const parent, const ndSharedPtr<ndBody>& body, const ndSharedPtr<ndJointBilateralConstraint>& joint)
{
	ndAssert(m_rootNode);
	ndAssert(joint->GetBody0() == body->GetAsBodyKinematic());
	ndAssert(joint->GetBody1() == parent->m_body->GetAsBodyKinematic());

	ndBodyKinematic* const dynBody = body->GetAsBodyDynamic();
	ndAssert(dynBody);
	if (dynBody)
	{
		dynBody->SetModel(this);
	}
	return new ndNode(body, joint, parent);
}

const ndList<ndModelArticulation::ndNode>& ndModelArticulation::GetCloseLoops() const
{
	return m_closeLoops;
}

void ndModelArticulation::AddCloseLoop(const ndSharedPtr<ndJointBilateralConstraint>& joint, const char* const name)
{
	#ifdef _DEBUG
		auto Check = [&](const ndBodyKinematic* const body)
		{
			if (body->GetInvMass() == ndFloat32(0.0f))
			{
				return false;
			}
			ndFixSizeArray<ndNode*, 256> stack;
			stack.PushBack(m_rootNode);
			while (stack.GetCount())
			{
				ndInt32 index = stack.GetCount() - 1;
				ndNode* const node = stack[index];
				stack.SetCount(index);
				if (node->m_body->GetAsBodyKinematic() == body)
				{
					return true;
				}

				for (ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
				{
					stack.PushBack(child);
				}
			}

			return false;
		};

		ndAssert(Check(joint->GetBody0()) || Check(joint->GetBody1()));
	#endif

	for (ndList<ndNode>::ndNode* node = m_closeLoops.GetFirst(); node; node = node->GetNext())
	{
		if (*node->GetInfo().m_joint == *joint)
		{
			return;
		}
	}

	char loopName[256];
	//snprintf(loopName, sizeof (loopName), "loop_%d", m_closeLoops.GetCount());
	//if (name)
	//{
	//	ndAssert(0);
	//	snprintf(loopName, sizeof(loopName), "loop_%s", name);
	//}
	snprintf(loopName, sizeof(loopName), "%s", name);

	ndSharedPtr<ndBody> body;
	ndList<ndNode>::ndNode* const node = m_closeLoops.Append(ndNode(body, joint, nullptr));
	node->GetInfo().m_name = loopName;
}

//void ndModelArticulation::AddToWorld(ndWorld* const world)
//{
//	if (m_rootNode)
//	{
//		for (ndModelArticulation::ndNode* node = m_rootNode->GetFirstIterator(); node; node = node->GetNextIterator())
//		{
//			world->AddBody(node->m_body);
//			if (node->m_joint)
//			{
//				world->AddJoint(node->m_joint);
//			}
//		}
//	}
//	world->AddModel(this);
//}

ndModelArticulation::ndNode* ndModelArticulation::FindByBody(const ndBody* const body) const
{
	if (m_rootNode)
	{
		for (ndModelArticulation::ndNode* node = m_rootNode->GetFirstIterator(); node; node = node->GetNextIterator())
		{
			if (*node->m_body == body)
			{
				return node;
			}
		}
	}

	return nullptr;
}

ndModelArticulation::ndNode* ndModelArticulation::FindByName(const char* const name) const
{
	if (m_rootNode)
	{
		for (ndModelArticulation::ndNode* node = m_rootNode->GetFirstIterator(); node; node = node->GetNextIterator())
		{
			if (strcmp(node->m_name.GetStr(), name) == 0)
			{
				return node;
			}
		}
	}

	return nullptr;
}

ndModelArticulation::ndNode* ndModelArticulation::FindLoopByName(const char* const name) const
{
	if (m_rootNode)
	{
		for (ndList<ndNode>::ndNode* node = m_closeLoops.GetFirst(); node; node = node->GetNext())
		{
			if (strcmp(node->GetInfo().m_name.GetStr(), name) == 0)
			{
				return &node->GetInfo();
			}
		}
	}

	return nullptr;
}

ndModelArticulation::ndNode* ndModelArticulation::FindLoopByJoint(const ndJointBilateralConstraint* const joint) const
{
	if (m_rootNode)
	{
		for (ndList<ndNode>::ndNode* node = m_closeLoops.GetFirst(); node; node = node->GetNext())
		{
			if (*node->GetInfo().m_joint == joint)
			{
				return &node->GetInfo();
			}
		}
	}

	return nullptr;
}

void ndModelArticulation::SetTransform(const ndMatrix& matrix)
{
	if (m_rootNode)
	{
		const ndMatrix offset(m_rootNode->m_body->GetMatrix().OrthoInverse() * matrix);
		for (ndModelArticulation::ndNode* node = m_rootNode->GetFirstIterator(); node; node = node->GetNextIterator())
		{
			ndSharedPtr<ndBody> body(node->m_body);
			body->SetMatrix(body->GetMatrix() * offset);
		}
	}
}

void ndModelArticulation::ConvertToUrdf()
{
	class BodyInfo
	{
		public:
		ndVector m_centerOfMass;
		ndMatrix m_bodyMatrix;
		ndMatrix m_visualMatrix;
		ndMatrix m_collisionMatrix;
		ndMatrix m_jointMatrix0;
		ndMatrix m_jointMatrix1;
		ndJointBilateralConstraint* m_joint;
	};

	if (!m_rootNode)
	{
		return;
	}

	ndTree<BodyInfo, ndModelArticulation::ndNode*> map;
	for (ndModelArticulation::ndNode* node = m_rootNode->GetFirstIterator(); node; node = node->GetNextIterator())
	{
		if (*node->m_joint)
		{
			BodyInfo info;
			ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
			info.m_bodyMatrix = body->GetMatrix();
			info.m_centerOfMass = info.m_bodyMatrix.TransformVector(body->GetCentreOfMass());
			info.m_collisionMatrix = body->GetCollisionShape().GetLocalMatrix() * info.m_bodyMatrix;
			info.m_visualMatrix = ndGetIdentityMatrix();
			ndUrdfBodyNotify* const notify = body->GetNotifyCallback()->GetAsUrdfBodyNotify();
			if (notify)
			{
				info.m_visualMatrix = notify->m_offset * info.m_bodyMatrix;
			}

			info.m_joint = *node->m_joint;
			info.m_joint->CalculateGlobalMatrix(info.m_jointMatrix0, info.m_jointMatrix1);
			map.Insert(info, node);
		}
	}

	ndFixSizeArray<BodyInfo, 512> saved;
	for (ndModelArticulation::ndNode* child = m_rootNode->GetFirstChild(); child; child = child->GetNext())
	{
		BodyInfo info;
		ndBodyKinematic* const body = child->m_body->GetAsBodyKinematic();
		info.m_bodyMatrix = body->GetMatrix();
		info.m_centerOfMass = info.m_bodyMatrix.TransformVector(body->GetCentreOfMass());
		info.m_collisionMatrix = body->GetCollisionShape().GetLocalMatrix() * info.m_bodyMatrix;
		ndUrdfBodyNotify* const notify = body->GetNotifyCallback()->GetAsUrdfBodyNotify();
		if (notify)
		{
			info.m_visualMatrix = notify->m_offset * info.m_bodyMatrix;
		}

		info.m_joint = *child->m_joint;
		info.m_joint->CalculateGlobalMatrix(info.m_jointMatrix0, info.m_jointMatrix1);
		saved.PushBack(info);
	}

	BodyInfo rootBodyInfo;
	ndBodyKinematic* const rootBody = m_rootNode->m_body->GetAsBodyKinematic();
	ndShapeInstance& rootCollision = rootBody->GetCollisionShape();

	rootBodyInfo.m_bodyMatrix = rootBody->GetMatrix();
	rootBodyInfo.m_centerOfMass = rootBodyInfo.m_bodyMatrix.TransformVector(rootBody->GetCentreOfMass());
	rootBodyInfo.m_collisionMatrix = rootCollision.GetLocalMatrix() * rootBodyInfo.m_bodyMatrix;
	ndUrdfBodyNotify* const rootNotify = rootBody->GetNotifyCallback()->GetAsUrdfBodyNotify();
	if (rootNotify)
	{
		rootBodyInfo.m_visualMatrix = rootNotify->m_offset * rootBodyInfo.m_bodyMatrix;
	}

	rootBody->SetMatrix(ndGetIdentityMatrix());
	rootCollision.SetLocalMatrix(rootBodyInfo.m_collisionMatrix);
	rootBody->SetCentreOfMass(rootBodyInfo.m_centerOfMass);
	if (rootNotify)
	{
		rootNotify->m_offset = rootBodyInfo.m_visualMatrix;
	}

	for (ndInt32 i = 0; i < saved.GetCount(); ++i)
	{
		const BodyInfo& info = saved[i];
		info.m_joint->SetLocalMatrix1(info.m_jointMatrix1);
	}

	ndFixSizeArray<ndModelArticulation::ndNode*, 256> stack;
	stack.PushBack(m_rootNode);
	while (stack.GetCount())
	{
		ndModelArticulation::ndNode* const node = stack.Pop();
		if (*node->m_joint)
		{
			const BodyInfo& info = map.Find(node)->GetInfo();
			ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
			ndShapeInstance& collision = body->GetCollisionShape();
			
			body->SetMatrix(info.m_jointMatrix0);
			collision.SetLocalMatrix(info.m_collisionMatrix* info.m_jointMatrix0.OrthoInverse());
			body->SetCentreOfMass(info.m_jointMatrix0.UntransformVector(info.m_centerOfMass));

			ndUrdfBodyNotify* const notify = body->GetNotifyCallback()->GetAsUrdfBodyNotify();
			if (notify)
			{
				notify->m_offset = info.m_visualMatrix * info.m_jointMatrix0.OrthoInverse();
			}
		}

		for (ndModelArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
		{
			stack.PushBack(child);
		}
	}

	stack.PushBack(m_rootNode);
	while (stack.GetCount())
	{
		ndModelArticulation::ndNode* const node = stack.Pop();
		ndJointBilateralConstraint* const joint = *node->m_joint;
		if (joint)
		{
			const BodyInfo& info = map.Find(node)->GetInfo();
			ndBodyKinematic* const body0 = joint->GetBody0();
			ndBodyKinematic* const body1 = joint->GetBody1();

			ndMatrix localMatrix0(info.m_jointMatrix0 * body0->GetMatrix().OrthoInverse());
			ndMatrix localMatrix1(info.m_jointMatrix1 * body1->GetMatrix().OrthoInverse());
			joint->SetLocalMatrix0(localMatrix0);
			joint->SetLocalMatrix1(localMatrix1);
		}

		for (ndModelArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
		{
			stack.PushBack(child);
		}
	}
}


void ndModelArticulation::OnAddToWorld()
{
}

void ndModelArticulation::OnRemoveFromToWorld()
{
}

void ndModelArticulation::AddBodiesAndJointsToWorld()
{
	ndAssert(m_world);
	ndFixSizeArray<ndNode*, 256> stack;
	if (m_rootNode)
	{
		stack.PushBack(m_rootNode);
		while (stack.GetCount())
		{
			ndInt32 index = stack.GetCount() - 1;
			ndNode* const node = stack[index];
			stack.SetCount(index);
			m_world->AddBody(node->m_body);
			if (node->m_joint)
			{
				m_world->AddJoint(node->m_joint);
			}

			for (ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
			{
				stack.PushBack(child);
			}
		}
	}

	for (ndList<ndNode>::ndNode* node = m_closeLoops.GetFirst(); node; node = node->GetNext())
	{
		m_world->AddJoint(node->GetInfo().m_joint);
	}
}

void ndModelArticulation::SetSleep(ndFloat32 speed, ndFloat32 angularSpeed, ndFloat32 accel, ndFloat32 alpha) const
{
	accel *= accel;
	speed *= speed;
	alpha *= alpha;
	angularSpeed *= angularSpeed;

	bool isSleeping = true;
	for (ndModelArticulation::ndNode* node = m_rootNode->GetFirstIterator(); node; node = node->GetNextIterator())
	{
		const ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
		if (!body->GetSleepState())
		{
			const ndVector bodyAccel(body->GetAccel());
			if (bodyAccel.DotProduct(bodyAccel).GetScalar() > accel)
			{
				isSleeping = false;
				break;
			}
			const ndVector bodyAlpha(body->GetAlpha());
			if (bodyAlpha.DotProduct(bodyAlpha).GetScalar() > alpha)
			{
				isSleeping = false;
				break;
			}
			const ndVector veloc(body->GetOmega());
			if (veloc.DotProduct(veloc).GetScalar() > speed)
			{
				isSleeping = false;
				break;
			}
			const ndVector omega(body->GetVelocity());
			if (omega.DotProduct(omega).GetScalar() > angularSpeed)
			{
				isSleeping = false;
				break;
			}
		}
	}
	if (isSleeping)
	{
		for (ndModelArticulation::ndNode* node = m_rootNode->GetFirstIterator(); node; node = node->GetNextIterator())
		{
			ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
			body->RestoreSleepState(true);
		}
	}
}

void ndModelArticulation::RemoveBodiesAndJointsFromWorld()
{
	ndAssert(m_world);
	ndFixSizeArray<ndNode*, 256> stack;
	if (m_rootNode)
	{
		for (ndList<ndNode>::ndNode* node = m_closeLoops.GetFirst(); node; node = node->GetNext())
		{
			if (node->GetInfo().m_joint->m_worldNode)
			{
				m_world->RemoveJoint(*node->GetInfo().m_joint);
			}
		}

		stack.PushBack(m_rootNode);
		while (stack.GetCount())
		{
			ndInt32 index = stack.GetCount() - 1;
			ndNode* const node = stack[index];
			stack.SetCount(index);
			if (node->m_joint)
			{
				if (node->m_joint->m_worldNode)
				{
					m_world->RemoveJoint(*node->m_joint);
				}
			}
			if (node->m_body->GetAsBodyKinematic()->m_sceneNode)
			{
				m_world->RemoveBody(*node->m_body);
			}

			for (ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
			{
				stack.PushBack(child);
			}
		}
	}
}

void ndModelArticulation::CalculateCentreOfMass(ndCenterOfMassDynamics& dynamics, ndFixSizeArray<const ndBodyKinematic*, 256>& bodyArrayOut, ndFixSizeArray<ndVector, 256>& bodyCenterOut) const
{
	for (ndModelArticulation::ndNode* node = m_rootNode->GetFirstIterator(); node; node = node->GetNextIterator())
	{
		const ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
		bodyArrayOut.PushBack(body);
		const ndMatrix matrix(body->GetMatrix());
		const ndVector bodyCom(matrix.TransformVector(body->GetCentreOfMass()));
		bodyCenterOut.PushBack(bodyCom);

		ndFloat32 mass = body->GetMassMatrix().m_w;
		dynamics.m_mass += mass;
		dynamics.m_centerOfMass.m_posit += bodyCom.Scale(mass);
	}
	dynamics.m_centerOfMass.m_posit = dynamics.m_centerOfMass.m_posit.Scale(ndFloat32(1.0f) / dynamics.m_mass);
	dynamics.m_centerOfMass.m_posit.m_w = ndFloat32(1.0f);

	for (ndInt32 i = bodyArrayOut.GetCount() - 1; i >= 0; --i)
	{
		bodyCenterOut[i] = (bodyCenterOut[i] - dynamics.m_centerOfMass.m_posit) & ndVector::m_triplexMask;
	}
}

ndModelArticulation::ndCenterOfMassDynamics ndModelArticulation::CalculateCentreOfMassDynamics(ndIkSolver& solver, const ndMatrix& localFrame, ndFixSizeArray<ndJointBilateralConstraint*, 64>& extraJoints, ndFloat32 timestep) const
{
	ndCenterOfMassDynamics dynamics;
	if (!m_rootNode)
	{
		return dynamics;
	}

	ndBodyKinematic* const rootBody = GetRoot()->m_body->GetAsBodyKinematic();
	ndSkeletonContainer* const skeleton = rootBody->GetSkeleton();
	ndAssert(skeleton);
	if (!skeleton)
	{
		return dynamics;
	}

	ndFixSizeArray<ndVector, 256> bodyCenter;
	ndFixSizeArray<const ndBodyKinematic*, 256> bodyArray;
	CalculateCentreOfMass(dynamics, bodyArray, bodyCenter);

	ndJointBilateralConstraint** const extraJointsPtr = extraJoints.GetCount() ? &extraJoints[0] : nullptr;
	solver.SolverBegin(skeleton, extraJointsPtr, extraJoints.GetCount(), GetWorld(), timestep);
	solver.Solve();
	auto CalculateComFullDynamics = [&dynamics, &bodyArray, &bodyCenter]()
	{
		for (ndInt32 i = bodyArray.GetCount() - 1; i >= 0; --i)
		{
			const ndBodyKinematic* const body = bodyArray[i];
			//const ndMatrix bodyMatrix(body->GetMatrix());

			ndFloat32 mass = body->GetMassMatrix().m_w;

			ndMatrix bodyInertia(body->CalculateInertiaMatrix());
			const ndVector extForce(body->GetAccel().Scale(mass));
			const ndVector extForceTorque(bodyCenter[i].CrossProduct(extForce));
			const ndVector extTorque(bodyInertia.RotateVector(body->GetAlpha()));
			const ndVector angularMomnetum(bodyInertia.RotateVector(body->GetAlpha()));
			const ndVector gyroTorque(body->GetOmega().CrossProduct(angularMomnetum));

			dynamics.m_momentum += body->GetVelocity().Scale(mass);
			dynamics.m_angularMomentum += angularMomnetum;
			dynamics.m_angularMomentum += bodyCenter[i].CrossProduct(body->GetVelocity().Scale(mass));

			// centripetal should always be zero, or else the bodies will be flying apart from each other
			//const ndVector centripetal((comOmega.CrossProduct(bodyCom)).CrossProduct(linearMomentum));
			//totalToque += centripetal;
			dynamics.m_force += extForce;
			dynamics.m_torque += extTorque;
			dynamics.m_torque += gyroTorque;
			dynamics.m_torque += extForceTorque;

			ndFloat32 mag2 = bodyCenter[i].DotProduct(bodyCenter[i]).GetScalar();
			ndMatrix covariance(ndCovarianceMatrix(bodyCenter[i], bodyCenter[i]));
			for (ndInt32 j = 0; j < 3; j++)
			{
				bodyInertia[j][j] += mass * mag2;
				bodyInertia[j] -= covariance[j].Scale(mass);
				dynamics.m_inertiaMatrix[j] += bodyInertia[j];
			}
		}
		dynamics.m_inertiaMatrix.m_posit.m_w = ndFloat32(1.0f);
	};
	CalculateComFullDynamics();
	solver.SolverEnd();

	dynamics.m_force = localFrame.UnrotateVector(dynamics.m_force);
	dynamics.m_torque = localFrame.UnrotateVector(dynamics.m_torque);
	dynamics.m_momentum = localFrame.UnrotateVector(dynamics.m_momentum);
	dynamics.m_angularMomentum = localFrame.UnrotateVector(dynamics.m_angularMomentum);
	dynamics.m_inertiaMatrix = localFrame * dynamics.m_inertiaMatrix * localFrame.OrthoInverse();
	dynamics.m_inertiaMatrix.m_posit = ndVector::m_wOne;

	const ndMatrix invInertia(dynamics.m_inertiaMatrix.Inverse4x4());
	dynamics.m_omega = invInertia.RotateVector(dynamics.m_angularMomentum);
	dynamics.m_veloc = dynamics.m_momentum.Scale (ndFloat32 (1.0f) / dynamics.m_mass);
	dynamics.m_alpha = invInertia.RotateVector(dynamics.m_torque);
	dynamics.m_accel = dynamics.m_force.Scale(ndFloat32(1.0f) / dynamics.m_mass);

	dynamics.m_centerOfMass.m_front = localFrame.m_front;
	dynamics.m_centerOfMass.m_up = localFrame.m_up;
	dynamics.m_centerOfMass.m_right = localFrame.m_right;

	return dynamics;
}


ndModelArticulation::ndCenterOfMassDynamics ndModelArticulation::CalculateCentreOfMassKinematics(const ndMatrix& localFrame) const
{
	ndCenterOfMassDynamics dynamics;
	if (!m_rootNode)
	{
		return dynamics;
	}

	ndBodyKinematic* const rootBody = GetRoot()->m_body->GetAsBodyKinematic();
	ndSkeletonContainer* const skeleton = rootBody->GetSkeleton();
	if (!skeleton)
	{
		return dynamics;
	}

	ndFixSizeArray<ndVector, 256> bodyCenter;
	ndFixSizeArray<const ndBodyKinematic*, 256> bodyArray;
	CalculateCentreOfMass(dynamics, bodyArray, bodyCenter);

	auto CalculateTotalMomentum = [&dynamics, &bodyArray, &bodyCenter]()
	{
		for (ndInt32 i = bodyArray.GetCount() - 1; i >= 0; --i)
		{
			const ndBodyKinematic* const body = bodyArray[i];
			//const ndMatrix bodyMatrix(body->GetMatrix());

			ndFloat32 mass = body->GetMassMatrix().m_w;

			ndMatrix bodyInertia(body->CalculateInertiaMatrix());
			const ndVector angularMomnetum(bodyInertia.RotateVector(body->GetAlpha()));
			dynamics.m_momentum += body->GetVelocity().Scale(mass);
			dynamics.m_angularMomentum += angularMomnetum;
			dynamics.m_angularMomentum += bodyCenter[i].CrossProduct(body->GetVelocity().Scale(mass));

			ndFloat32 mag2 = bodyCenter[i].DotProduct(bodyCenter[i]).GetScalar();
			ndMatrix covariance(ndCovarianceMatrix(bodyCenter[i], bodyCenter[i]));
			for (ndInt32 j = 0; j < 3; j++)
			{
				bodyInertia[j][j] += mass * mag2;
				bodyInertia[j] -= covariance[j].Scale(mass);
				dynamics.m_inertiaMatrix[j] += bodyInertia[j];
			}
		}
		dynamics.m_inertiaMatrix.m_posit.m_w = ndFloat32(1.0f);
	};
	CalculateTotalMomentum();

	dynamics.m_momentum = localFrame.UnrotateVector(dynamics.m_momentum);
	dynamics.m_angularMomentum = localFrame.UnrotateVector(dynamics.m_angularMomentum);
	dynamics.m_inertiaMatrix = localFrame * dynamics.m_inertiaMatrix * localFrame.OrthoInverse();
	dynamics.m_inertiaMatrix.m_posit = ndVector::m_wOne;

	const ndMatrix invInertia(dynamics.m_inertiaMatrix.Inverse4x4());
	dynamics.m_omega = invInertia.RotateVector(dynamics.m_angularMomentum);
	dynamics.m_veloc = dynamics.m_momentum.Scale(ndFloat32(1.0f) / dynamics.m_mass);

	dynamics.m_centerOfMass.m_front = localFrame.m_front;
	dynamics.m_centerOfMass.m_up = localFrame.m_up;
	dynamics.m_centerOfMass.m_right = localFrame.m_right;

	return dynamics;
}