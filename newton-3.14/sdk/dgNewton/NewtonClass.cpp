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

#include "NewtonStdAfx.h"
#include "NewtonClass.h"


void* Newton::DefaultAllocMemory (dgInt32 size)
{
	return malloc (size_t (size));
}


void Newton::DefaultFreeMemory (void* const ptr, dgInt32 size)
{
	free (ptr);
}


Newton::Newton (dgMemoryAllocator* const allocator)
	:dgWorld(allocator) 
	,m_destructor(NULL)
{
}

Newton::~Newton ()
{
	if (m_destructor) {
		m_destructor ((NewtonWorld*)this);
	}
}

void Newton::UpdatePhysics (dgFloat32 timestep)
{
	Update (timestep);
}

void Newton::UpdatePhysicsAsync (dgFloat32 timestep)
{
	UpdateAsync (timestep);
}

NewtonUserJoint::NewtonUserJoint(NewtonUserBilateralCallback callback, dgBody* const body)
	:dgUserConstraint(NULL, body, NULL, 1)
	,m_forceArray(m_jointForce)
	,m_param(NULL)
	,m_rows(0)
{
	m_maxDOF = 6;
	m_jacobianFnt = callback;
	m_body1 = body->GetWorld()->GetSentinelBody();
}

NewtonUserJoint::NewtonUserJoint (dgWorld* const world, dgInt32 maxDof, NewtonUserBilateralCallback callback, dgBody* const dyn0, dgBody* const dyn1)
	:dgUserConstraint (world, dyn0, dyn1, 1)
	,m_forceArray(m_jointForce)
	,m_param(NULL)
	,m_rows(0)
{
	m_maxDOF = dgUnsigned8(maxDof);
	m_jacobianFnt = callback;

	dgAssert (world);
	if (m_maxDOF > DG_BILATERAL_CONTRAINT_DOF) {
		m_forceArray = (dgForceImpactPair*) world->GetAllocator()->Malloc (dgInt32 (m_maxDOF * sizeof (dgForceImpactPair)));
	}
	memset (m_forceArray, 0, m_maxDOF * sizeof (dgFloat32));
}

NewtonUserJoint::~NewtonUserJoint ()
{
	if (m_forceArray != m_jointForce) {
		m_body0->GetWorld()->GetAllocator()->Free (m_forceArray);
	}
}

void NewtonUserJoint::SetMassScale (dgFloat32 scale0, dgFloat32 scale1)
{
	m_massScaleBody0 = scale0;
	m_massScaleBody1 = scale1;

	dgWorld* const world = m_body0->GetWorld();
	dgSkeletonList& skelManager = *world;
	skelManager.m_skelListIsDirty = true;
}

dgUnsigned32 NewtonUserJoint::JacobianDerivative (dgContraintDescritor& params)
{
	m_rows = 0;
	m_param = &params;
	m_jacobianFnt ((NewtonJoint*)this, params.m_timestep, params.m_threadIndex);
	return dgUnsigned32 (m_rows);
}

void NewtonUserJoint::Serialize (dgSerialize serializeCallback, void* const userData)
{
	dgWorld::OnJointSerializationCallback serializeJoint;
	dgWorld::OnJointDeserializationCallback deserializeJoint;

	Newton* const world = m_body0 ? (Newton*)m_body0->GetWorld() : (Newton*)m_body1->GetWorld();
	world->GetJointSerializationCallbacks (&serializeJoint, &deserializeJoint);
	if (serializeJoint) {
		((NewtonOnJointSerializationCallback)serializeJoint) ((NewtonJoint*)this, (NewtonSerializeCallback)serializeCallback, userData);
	}
}

void NewtonUserJoint::AddLinearRowJacobian (const dgVector& pivot0, const dgVector& pivot1, const dgVector& dir)
{
	dgPointParam pointData;
	InitPointParam (pointData, m_defualtDiagonalRegularizer, pivot0, pivot1);

	CalculatePointDerivative (m_rows, *m_param, dir, pointData, &m_forceArray[m_rows]); 
	m_rows ++;
	dgAssert (m_rows <= dgInt32 (m_maxDOF));
}

void NewtonUserJoint::AddAngularRowJacobian (const dgVector& dir, dgFloat32 relAngle)
{
	CalculateAngularDerivative (m_rows, *m_param, dir, m_defualtDiagonalRegularizer, relAngle, &m_forceArray[m_rows]); 
	m_rows ++;
	dgAssert (m_rows <= dgInt32 (m_maxDOF));
}

void NewtonUserJoint::AddGeneralRowJacobian (const dgFloat32* const jacobian0, const dgFloat32* const jacobian1)
{
	SetJacobianDerivative (m_rows, *m_param, jacobian0, jacobian1, &m_forceArray[m_rows]);
	m_rows ++;
	dgAssert (m_rows <= dgInt32 (m_maxDOF));
}

dgInt32 NewtonUserJoint::GetJacobianCount() const
{
	return m_rows;
}

void NewtonUserJoint::GetJacobianAt(dgInt32 index, dgFloat32* const jacobian0, dgFloat32* const jacobian1) const
{
	if (index < m_rows) {
		for (dgInt32 i = 0; i < 3; i ++) {
			jacobian0[i] = m_param->m_jacobian[index].m_jacobianM0.m_linear[i];
			jacobian1[i] = m_param->m_jacobian[index].m_jacobianM1.m_linear[i];
			jacobian0[i + 3] = m_param->m_jacobian[index].m_jacobianM0.m_angular[i];
			jacobian1[i + 3] = m_param->m_jacobian[index].m_jacobianM1.m_angular[i];
		}
	}
}

void NewtonUserJoint::GetJacobian(dgJacobian& jacobian0, dgJacobian& jacobian1) const
{
	dgInt32 index = m_rows - 1;
	if ((index >= 0) && (index < dgInt32(m_maxDOF))) {
		jacobian0.m_linear = m_param->m_jacobian[index].m_jacobianM0.m_linear;
		jacobian0.m_angular = m_param->m_jacobian[index].m_jacobianM0.m_angular;
		jacobian1.m_linear = m_param->m_jacobian[index].m_jacobianM1.m_linear;
		jacobian1.m_angular = m_param->m_jacobian[index].m_jacobianM1.m_angular;
	}
}


dFloat NewtonUserJoint::GetAcceleration () const
{
	dgInt32 index = m_rows - 1;
	if ((index >= 0) &&  (index < dgInt32 (m_maxDOF))) {
		return GetRowAcceleration (index, *m_param);
	}
	return 0.0f;
}


void NewtonUserJoint::SetAcceleration (dgFloat32 acceleration)
{
	dgInt32 index = m_rows - 1;
	if ((index >= 0) &&  (index < dgInt32 (m_maxDOF))) {
		SetMotorAcceleration (index, acceleration, *m_param);
	}
}

dgFloat32 NewtonUserJoint::CalculateZeroMotorAcceleration() const
{
	dgInt32 index = m_rows - 1;
	dgFloat32 accel = dgFloat32 (0.0f);
	if ((index >= 0) && (index < dgInt32(m_maxDOF))) {
		accel = CalculateMotorAcceleration(index, *m_param);
	}
	return accel;
}

void NewtonUserJoint::SetMassIndependentSpringDamperAcceleration (dgFloat32 rowStiffness, dFloat spring, dFloat damper)
{
	dgInt32 index = m_rows - 1;
	if ((index >= 0) &&  (index < dgInt32 (m_maxDOF))) {
		dgBilateralConstraint::SetMassIndependentSpringDamperAcceleration (index, *m_param, rowStiffness, spring, damper);
	}
}

void NewtonUserJoint::SetMassDependentSpringDamperAcceleration(dgFloat32 spring, dgFloat32 damper)
{
	dgInt32 index = m_rows - 1;
	if ((index >= 0) && (index < dgInt32(m_maxDOF))) {
		dgBilateralConstraint::SetMassDependentSpringDamperAcceleration(index, *m_param, spring, damper);
	}
}

void NewtonUserJoint::SetHighFriction (dgFloat32 friction)
{
	dgInt32 index = m_rows - 1;
	if ((index >= 0) &&  (index < dgInt32 (m_maxDOF))) {
		m_param->m_forceBounds[index].m_upper = dgClamp (friction, dgFloat32(0.001f), dgFloat32(DG_MAX_BOUND));
		dgAssert (m_param->m_forceBounds[index].m_normalIndex == DG_INDEPENDENT_ROW);

		#ifdef _DEBUG
		dgInt32 i0 = 0; 
		dgInt32 i1 = m_rows - 1; 
		while ((i0 <= i1) && (m_param->m_forceBounds[i0].m_normalIndex == DG_INDEPENDENT_ROW)) i0 ++;
		while ((i1 >= i0) && (m_param->m_forceBounds[i1].m_normalIndex != DG_INDEPENDENT_ROW)) i1 --;
		dgAssert((i0 - i1) == 1);
		if ((i0 - i1) != 1) {
			dgTrace(("make sure that friction joint are issue at last\n"));
		}
		#endif
	}
}

void NewtonUserJoint::SetLowerFriction (dgFloat32 friction)
{
	dgInt32 index = m_rows - 1;
	if ((index >= 0) &&  (index < dgInt32 (m_maxDOF))) {
		m_param->m_forceBounds[index].m_low = dgClamp (friction, dgFloat32(DG_MIN_BOUND), dgFloat32(-0.001f));
		dgAssert (m_param->m_forceBounds[index].m_normalIndex == DG_INDEPENDENT_ROW);

		#ifdef _DEBUG
		dgInt32 i0 = 0;
		dgInt32 i1 = m_rows - 1;
		while ((i0 <= i1) && (m_param->m_forceBounds[i0].m_normalIndex == DG_INDEPENDENT_ROW)) i0++;
		while ((i1 >= i0) && (m_param->m_forceBounds[i1].m_normalIndex != DG_INDEPENDENT_ROW)) i1--;
		dgAssert((i0 - i1) == 1);
		if ((i0 - i1) != 1) {
			dgTrace(("make sure that friction joint are issue at last\n"));
		}
		#endif
	}
}

void NewtonUserJoint::SetRowStiffness (dgFloat32 stiffness)
{
	dgInt32 index = m_rows - 1;
	if ((index >= 0) &&  (index < dgInt32 (m_maxDOF))) {
		stiffness = dgClamp (stiffness, dgFloat32(0.0f), dgFloat32(1.0f));
		m_param->m_diagonalRegularizer[index] = stiffness;
	}
}

dgFloat32 NewtonUserJoint::GetRowForce (dgInt32 row) const
{
	dgFloat32 force = 0.0f;
	if ((row >= 0) && (row < dgInt32 (m_maxDOF))) {
		force = m_forceArray[row].m_force; 
	}
	return force;
}

void NewtonUserJoint::SetUpdateFeedbackFunction (NewtonUserBilateralCallback getFeedback)
{
	dgUserConstraint::SetUpdateFeedbackFunction ((ConstraintsForceFeeback) getFeedback);
}
