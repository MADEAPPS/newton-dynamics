/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodyDynamic.h"
#include "ndDynamicsUpdate.h"


ndDynamicsUpdate::ndDynamicsUpdate()
	:m_jointArray()
	,m_bodyProxyArray()
	,m_solverPasses(0)
	,m_maxRowsCount(0)
	,m_rowsCount(0)
{
}

ndDynamicsUpdate::~ndDynamicsUpdate()
{
}

void ndDynamicsUpdate::DynamicsUpdate()
{
	DefaultUpdate();
}

void ndDynamicsUpdate::DefaultUpdate()
{
	D_TRACKTIME();
	InitWeights();
	InitBodyArray();
	InitJacobianMatrix();
}

void ndDynamicsUpdate::InitWeights()
{
	D_TRACKTIME();
	const ndWorld* const world = (ndWorld*)this;
	const ndScene* const scene = world->GetScene();

	const dArray<ndContact*>& contactArray = scene->GetActiveContacts();
	const dArray<ndBodyKinematic*>& bodyArray = scene->GetWorkingBodyArray();

	if (m_bodyProxyArray.GetCapacity() < 256)
	{
		m_bodyProxyArray.Resize(256);
	}
	m_bodyProxyArray.SetCount(bodyArray.GetCount());
	memset(&m_bodyProxyArray[0], 0, bodyArray.GetCount() * sizeof(ndBodyProxy));

	dUnsigned32 maxRowCount = 0;
	m_jointArray.SetCount(contactArray.GetCount());
	for (dInt32 i = contactArray.GetCount() - 1; i >= 0; i--) 
	{
		const ndContact* const contact = contactArray[i];
		m_jointArray[i] = (ndConstraint*)contact;
		const ndBodyKinematic* const body0 = contact->GetBody0();
		const ndBodyKinematic* const body1 = contact->GetBody1();
		const dInt32 m0 = body0->GetIndex();
		const dInt32 m1 = body1->GetIndex();
		maxRowCount += contact->GetRowsCount();
		if (body0->GetInvMass() != dFloat32(0.0f))
		{
			m_bodyProxyArray[m0].m_weight = dFloat32(1.0f);
		}
		else
		{
			m_bodyProxyArray[m0].m_weight += dFloat32(1.0f);
		}
		
		if (body1->GetInvMass() != dFloat32(0.0f))
		{
			m_bodyProxyArray[m1].m_weight = dFloat32(1.0f);
		}
		else
		{
			m_bodyProxyArray[m1].m_weight += dFloat32(1.0f);
		}
	}
	m_maxRowsCount = maxRowCount;

	//m_bodyProxyArray[0].m_weight = dFloat32(1.0f);
	
	dFloat32 extraPasses = dFloat32(0.0f);

	//dgSkeletonList& skeletonList = *m_world;
	//const dInt32 lru = skeletonList.m_lruMarker;
	//skeletonList.m_lruMarker += 1;
	//m_skeletonCount = 0;

	for (dInt32 i = contactArray.GetCount() - 1; i >= 0; i--) 
	{
		const ndBodyKinematic* const body = bodyArray[i];
		if (body->GetInvMass() != dFloat32(0.0f))
		{
			extraPasses = dMax(m_bodyProxyArray[i].m_weight, extraPasses);
		}
	
	//	dgSkeletonContainer* const container = body->GetSkeleton();
	//	if (container && (container->m_lru != lru)) {
	//		container->m_lru = lru;
	//		m_skeletonArray[m_skeletonCount] = container;
	//		m_skeletonCount++;
	//	}
	}
	const dInt32 conectivity = 7;
	m_solverPasses = world->GetSolverIterations() + 2 * dInt32(extraPasses) / conectivity + 1;
}

void ndDynamicsUpdate::InitBodyArray()
{
	D_TRACKTIME();
	class ndInitBodyArray: public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();

			const dInt32 threadIndex = GetThredID();
			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetWorkingBodyArray();
			const dInt32 count = bodyArray.GetCount();

			ndWorld* const world = (ndWorld*)m_owner->GetWorld();
			dArray<ndBodyProxy>& bodyProxyArray = world->m_bodyProxyArray;
			
			for (dInt32 i = m_it->fetch_add(1); i < count; i = m_it->fetch_add(1))
			{
				ndBodyDynamic* const body = bodyArray[i]->GetAsBodyDynamic();
				if (body)
				{
					body->AddDampingAcceleration(m_timestep);
					body->UpdateInvInertiaMatrix();
					
					body->SetAccel(body->GetVelocity());
					body->SetAlpha(body->GetOmega());

					const dFloat32 w = bodyProxyArray[i].m_weight;
					bodyProxyArray[i].m_invWeight = dFloat32(1.0f) / w;
				}
			}
		}
	};

	const ndWorld* const world = (ndWorld*)this;
	ndScene* const scene = world->GetScene();
	scene->SubmitJobs<ndInitBodyArray>();
}

//dInt32 ndDynamicsUpdate::GetJacobianDerivatives(ndConstraintDescritor& constraintParam, dgJointInfo* const jointInfo, dgConstraint* const constraint, dgLeftHandSide* const leftHandSide, dgRightHandSide* const rightHandSide, dInt32 rowCount) const
dInt32 ndDynamicsUpdate::GetJacobianDerivatives(dInt32 baseIndex, ndConstraint* const joint) const
{
	ndConstraintDescritor constraintParam;
	dInt32 dof = joint->GetRowsCount();
	dAssert(dof <= D_CONSTRAINT_MAX_ROWS);
	for (dInt32 i = 0; i < dof; i++) 
	{
		constraintParam.m_forceBounds[i].m_low = D_MIN_BOUND;
		constraintParam.m_forceBounds[i].m_upper = D_MAX_BOUND;
		constraintParam.m_forceBounds[i].m_jointForce = nullptr;
		constraintParam.m_forceBounds[i].m_normalIndex = D_INDEPENDENT_ROW;
	}
	
	//dAssert(constraint->m_body0);
	//dAssert(constraint->m_body1);
	//ndBodyKinematic* const body0 = joint->GetBody0();
	//dgBody* const body1 = constraint->m_body1;
	//dAssert(body0->IsRTTIType(dgBody::m_dynamicBodyRTTI) || body0->IsRTTIType(dgBody::m_kinematicBodyRTTI));
	//dAssert(body1->IsRTTIType(dgBody::m_dynamicBodyRTTI) || body1->IsRTTIType(dgBody::m_kinematicBodyRTTI));
	//
	//dof = constraint->JacobianDerivative(constraintParam);
	//
	//if (constraint->GetId() == dgConstraint::m_contactConstraint) {
	//	dgContact* const contactJoint = (dgContact*)constraint;
	//	contactJoint->m_isInSkeletonLoop = false;
	//	dgSkeletonContainer* const skeleton0 = body0->GetSkeleton();
	//	dgSkeletonContainer* const skeleton1 = body1->GetSkeleton();
	//	if (skeleton0 && (skeleton0 == skeleton1)) {
	//		if (contactJoint->IsSkeletonSelftCollision()) {
	//			contactJoint->m_isInSkeletonLoop = true;
	//			skeleton0->AddSelfCollisionJoint(contactJoint);
	//		}
	//	}
	//	else if (contactJoint->IsSkeletonIntraCollision()) {
	//		if (skeleton0 && !skeleton1) {
	//			contactJoint->m_isInSkeletonLoop = true;
	//			skeleton0->AddSelfCollisionJoint(contactJoint);
	//		}
	//		else if (skeleton1 && !skeleton0) {
	//			contactJoint->m_isInSkeletonLoop = true;
	//			skeleton1->AddSelfCollisionJoint(contactJoint);
	//		}
	//	}
	//}
	//else if (constraint->IsBilateral() && !constraint->m_isInSkeleton && (constraint->m_solverModel == 3)) {
	//	dgSkeletonContainer* const skeleton0 = body0->GetSkeleton();
	//	dgSkeletonContainer* const skeleton1 = body1->GetSkeleton();
	//	if (skeleton0 || skeleton1) {
	//		if (skeleton0 && !skeleton1) {
	//			constraint->m_isInSkeletonLoop = true;
	//			skeleton0->AddSelfCollisionJoint(constraint);
	//		}
	//		else if (skeleton1 && !skeleton0) {
	//			constraint->m_isInSkeletonLoop = true;
	//			skeleton1->AddSelfCollisionJoint(constraint);
	//		}
	//	}
	//}
	//
	//jointInfo->m_pairCount = dof;
	//jointInfo->m_pairStart = rowCount;
	//
	//for (dInt32 i = 0; i < dof; i++) {
	//	dAssert(constraintParam.m_forceBounds[i].m_jointForce);
	//
	//	dgLeftHandSide* const row = &leftHandSide[rowCount];
	//	dgRightHandSide* const rhs = &rightHandSide[rowCount];
	//
	//	row->m_Jt = constraintParam.m_jacobian[i];
	//	rhs->m_diagDamp = dgFloat32(0.0f);
	//	rhs->m_diagonalRegularizer = dgClamp(constraintParam.m_diagonalRegularizer[i], dgFloat32(1.0e-5f), dgFloat32(1.0f));
	//	rhs->m_coordenateAccel = constraintParam.m_jointAccel[i];
	//	rhs->m_restitution = constraintParam.m_restitution[i];
	//	rhs->m_penetration = constraintParam.m_penetration[i];
	//	rhs->m_penetrationStiffness = constraintParam.m_penetrationStiffness[i];
	//	rhs->m_lowerBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_low;
	//	rhs->m_upperBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_upper;
	//	rhs->m_jointFeebackForce = constraintParam.m_forceBounds[i].m_jointForce;
	//
	//	dAssert(constraintParam.m_forceBounds[i].m_normalIndex >= -1);
	//	rhs->m_normalForceIndex = constraintParam.m_forceBounds[i].m_normalIndex;
	//	rowCount++;
	//}
	//
	//return rowCount;
return 0;
}

void ndDynamicsUpdate::InitJacobianMatrix()
{
	//dgLeftHandSide* const leftHandSide = &m_world->m_solverMemory.m_leftHandSizeBuffer[0];
	//dgRightHandSide* const rightHandSide = &m_world->m_solverMemory.m_righHandSizeBuffer[0];
	//dgJacobian* const internalForces = &m_world->m_solverMemory.m_internalForcesBuffer[0];
	//
	//dgContraintDescritor constraintParams;
	//constraintParams.m_world = m_world;
	//constraintParams.m_threadIndex = threadID;
	//constraintParams.m_timestep = m_timestep;
	//constraintParams.m_invTimestep = m_invTimestep;
	//
	//const dInt32 step = m_threadCounts;
	//const dInt32 jointCount = m_cluster->m_jointCount;
	//for (dInt32 i = threadID; i < jointCount; i += step) {
	//	dgJointInfo* const jointInfo = &m_jointArray[i];
	//	dgConstraint* const constraint = jointInfo->m_joint;
	//	dAssert(jointInfo->m_m0 >= 0);
	//	dAssert(jointInfo->m_m1 >= 0);
	//	dAssert(jointInfo->m_m0 != jointInfo->m_m1);
	//	const dInt32 rowBase = dgAtomicExchangeAndAdd(&m_jacobianMatrixRowAtomicIndex, jointInfo->m_pairCount);
	//	m_world->GetJacobianDerivatives(constraintParams, jointInfo, constraint, leftHandSide, rightHandSide, rowBase);
	//	BuildJacobianMatrix(jointInfo, leftHandSide, rightHandSide, internalForces);
	//}

	D_TRACKTIME();
	class ndInitJacobianMatrix : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();

			const dInt32 threadIndex = GetThredID();
			ndWorld* const world = m_owner->GetWorld();
			const dArray<ndConstraint*>& jointArray = world->m_jointArray;
			//const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetWorkingBodyArray();
			//const dInt32 count = bodyArray.GetCount();
			//
			//ndWorld* const world = (ndWorld*)m_owner->GetWorld();
			//dArray<ndBodyProxy>& bodyProxyArray = world->m_bodyProxyArray;

			dAtomic<dUnsigned32>& rowCount = world->m_rowsCount;
			//ndConstraintDescritor constraintParam;
			for (dInt32 i = m_it->fetch_add(1); i < jointArray.GetCount(); i = m_it->fetch_add(1))
			{
				//dgJointInfo* const jointInfo = &m_jointArray[i];
				//dgConstraint* const constraint = jointInfo->m_joint;
				//dAssert(jointInfo->m_m0 >= 0);
				//dAssert(jointInfo->m_m1 >= 0);
				//dAssert(jointInfo->m_m0 != jointInfo->m_m1);
				ndConstraint* const joint = jointArray[i];
				//const dUnsigned32 rowBase = dgAtomicExchangeAndAdd(&m_jacobianMatrixRowAtomicIndex, jointInfo->m_pairCount);
				const dUnsigned32 rowBase = rowCount.fetch_add(joint->GetRowsCount());
				dAssert(rowCount.load() <= world->m_maxRowsCount);
				//m_world->GetJacobianDerivatives(constraintParams, jointInfo, constraint, leftHandSide, rightHandSide, rowBase);
				world->GetJacobianDerivatives(rowBase, joint);
				//BuildJacobianMatrix(jointInfo, leftHandSide, rightHandSide, internalForces);
			}
		}
	};

	m_rowsCount.store(0);
	const ndWorld* const world = (ndWorld*)this;
	ndScene* const scene = world->GetScene();
	scene->SubmitJobs<ndInitJacobianMatrix>();
}