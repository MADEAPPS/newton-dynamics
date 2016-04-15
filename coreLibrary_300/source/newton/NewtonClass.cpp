/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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





NewtonDeadBodies::NewtonDeadBodies(dgMemoryAllocator* const allocator)
	:dgTree<dgBody*, void*>(allocator)
{
	Insert((dgBody*)NULL, 0);
}

void NewtonDeadBodies::DestroyBodies(Newton& world)
{
	Iterator iter (*this);
	for (iter.Begin(); iter; ) {
		dgTreeNode* const node = iter.GetNode();
		iter ++;
		dgBody* const body = node->GetInfo();
		if (body) {
			Remove (node);
			world.DestroyBody(body);
		}
	}
}


NewtonDeadJoints::NewtonDeadJoints(dgMemoryAllocator* const allocator)
	:dgTree<dgConstraint*, void*>(allocator)
{
	Insert((dgConstraint*)NULL, 0);
}


void NewtonDeadJoints::DestroyJoints(Newton& world)
{
	Iterator iter (*this);
	for (iter.Begin(); iter; ) {
		dgTreeNode* const node = iter.GetNode();
		iter ++;
		dgConstraint* const joint = node->GetInfo();
		if (joint) {
			Remove (node);
			world.DestroyConstraint (joint);
		}
	}
}


void* Newton::DefaultAllocMemory (dgInt32 size)
{
	return malloc (size_t (size));
}


void Newton::DefaultFreeMemory (void* const ptr, dgInt32 size)
{
	free (ptr);
}


Newton::Newton (dgFloat32 scale, dgMemoryAllocator* const allocator)
	:dgWorld(allocator) 
	,NewtonDeadBodies(allocator)
	,NewtonDeadJoints(allocator)
	,m_maxTimeStep(DG_TIMESTEP)
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

	NewtonDeadBodies& bodyList = *this;
	NewtonDeadJoints& jointList = *this;

	jointList.DestroyJoints (*this);
	bodyList.DestroyBodies (*this);
}

void Newton::UpdatePhysicsAsync (dgFloat32 timestep)
{
	UpdateAsync (timestep);

	if (!IsBusy()) {		
		NewtonDeadBodies& bodyList = *this;
		NewtonDeadJoints& jointList = *this;

		jointList.DestroyJoints (*this);
		bodyList.DestroyBodies (*this);
	} 
}


void Newton::DestroyJoint(dgConstraint* const joint)
{
	if (IsBusy()) {		
		// the engine is busy in the previous update, deferred the deletion
		NewtonDeadJoints& jointList = *this;
		jointList.Insert (joint, joint);
	} else {
		dgWorld::DestroyConstraint (joint);
	}
}

void Newton::DestroyBody(dgBody* const body)
{
	if (IsBusy()) {		
		// the engine is busy in the previous update, deferred the deletion
		NewtonDeadBodies& bodyList = *this;
		bodyList.Insert (body, body);
	} else {
		dgWorld::DestroyBody(body);
	}
}


NewtonUserJoint::NewtonUserJoint (dgWorld* const world, dgInt32 maxDof, NewtonUserBilateralCallback callback, NewtonUserBilateralGetInfoCallback getInfo, dgBody* const dyn0, dgBody* const dyn1)
	:dgUserConstraint (world, dyn0, dyn1, 1)
	,m_forceArray(m_jointForce)
	,m_param(NULL)
	,m_rows(0)
{
	m_maxDOF = dgUnsigned8(maxDof);
	m_jacobianFnt = callback;
	m_getInfoCallback = getInfo;

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
    InitPointParam (pointData, m_stiffness, pivot0, pivot1);

	CalculatePointDerivative (m_rows, *m_param, dir, pointData, &m_forceArray[m_rows]); 
	m_rows ++;
	dgAssert (m_rows <= dgInt32 (m_maxDOF));
}

void NewtonUserJoint::AddAngularRowJacobian (const dgVector& dir, dgFloat32 relAngle)
{
	CalculateAngularDerivative (m_rows, *m_param, dir, m_stiffness, relAngle, &m_forceArray[m_rows]); 
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


void NewtonUserJoint::SetAcceleration (dgFloat32 acceleration)
{
	dgInt32 index = m_rows - 1;
	if ((index >= 0) &&  (index < dgInt32 (m_maxDOF))) {
//		m_param->m_jointAccel[index] = acceleration;
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


void NewtonUserJoint::SetSpringDamperAcceleration (dFloat spring, dFloat damper)
{
	dgInt32 index = m_rows - 1;
	if ((index >= 0) &&  (index < dgInt32 (m_maxDOF))) {
//		dgFloat32 accel = CalculateSpringDamperAcceleration (index, *m_param, springK, damperD);
//		SetMotorAcceleration (index, accel, *m_param);
		dgBilateralConstraint::SetSpringDamperAcceleration (index, *m_param, spring, damper);
	}
}


void NewtonUserJoint::SetHighFriction (dgFloat32 friction)
{
	dgInt32 index = m_rows - 1;
	if ((index >= 0) &&  (index < dgInt32 (m_maxDOF))) {
		m_param->m_forceBounds[index].m_upper = dgClamp (friction, dgFloat32(0.001f), dgFloat32(DG_MAX_BOUND));
		m_param->m_forceBounds[index].m_normalIndex = DG_BILATERAL_FRICTION_CONSTRAINT;

		#ifdef _DEBUG
		dgInt32 i0 = 0; 
		dgInt32 i1 = m_rows - 1; 
		while ((i0 <= i1) && (m_param->m_forceBounds[i0].m_normalIndex != DG_BILATERAL_FRICTION_CONSTRAINT)) i0 ++;
		while ((i1 >= i0) && (m_param->m_forceBounds[i1].m_normalIndex == DG_BILATERAL_FRICTION_CONSTRAINT)) i1 --;
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
		m_param->m_forceBounds[index].m_normalIndex = DG_BILATERAL_FRICTION_CONSTRAINT;

		#ifdef _DEBUG
		dgInt32 i0 = 0;
		dgInt32 i1 = m_rows - 1;
		while ((i0 <= i1) && (m_param->m_forceBounds[i0].m_normalIndex != DG_BILATERAL_FRICTION_CONSTRAINT)) i0++;
		while ((i1 >= i0) && (m_param->m_forceBounds[i1].m_normalIndex == DG_BILATERAL_FRICTION_CONSTRAINT)) i1--;
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
		stiffness = dgFloat32 (1.0f) - dgClamp (stiffness, dgFloat32(0.0f), dgFloat32(1.0f));
		stiffness = -dgFloat32 (1.0f) - stiffness / DG_PSD_DAMP_TOL; 
		m_param->m_jointStiffness[index] = stiffness;
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


void NewtonUserJoint::GetInfo (dgConstraintInfo* const info) const
{
	memset (info, 0, sizeof (dgConstraintInfo));
	if (m_getInfoCallback) {
		InitInfo (info);
		m_getInfoCallback ((NewtonJoint*) this, (NewtonJointRecord*)info);
	}
}


void NewtonUserJoint::SetUpdateFeedbackFunction (NewtonUserBilateralCallback getFeedback)
{
	dgUserConstraint::SetUpdateFeedbackFunction ((ConstraintsForceFeeback) getFeedback);
}

