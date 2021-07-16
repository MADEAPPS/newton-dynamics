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
#include "ndCollisionStdafx.h"
#include "ndScene.h"
#include "ndContact.h"
#include "ndBodyKinematic.h"
#include "ndContactOptions.h"

dVector ndContact::m_initialSeparatingVector(dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f));

#define D_REST_RELATIVE_VELOCITY		dFloat32 (1.0e-3f)
#define D_MAX_DYNAMIC_FRICTION_SPEED	dFloat32 (0.3f)
#define D_MAX_PENETRATION_STIFFNESS		dFloat32 (50.0f)
#define D_DIAGONAL_REGULARIZER			dFloat32 (1.0e-3f)

ndContact::ndContact()
	:ndConstraint()
	,dContainersFreeListAlloc<ndContact*>()
	,m_positAcc(dFloat32(10.0f))
	,m_rotationAcc()
	,m_separatingVector(m_initialSeparatingVector)
	,m_contacPointsList()
	,m_body0(nullptr)
	,m_body1(nullptr)
	,m_linkNode(nullptr)
	,m_material()
	,m_timeOfImpact(dFloat32(1.0e10f))
	,m_separationDistance____(dFloat32(0.0f))
	,m_contactPruningTolereance(D_PRUNE_CONTACT_TOLERANCE)
	,m_maxDOF(0)
	,m_sceneLru(0)
	,m_isDead(0)
	,m_isAttached(0)
	,m_isIntersetionTestOnly(0)
	,m_skeletonIntraCollision(1)
	,m_skeletonSelftCollision(1)
{
	static int xxxxxx;
	xxxxx = xxxxxx;
	xxxxxx++;

	m_active = false;
}

ndContact::~ndContact()
{
}

void ndContact::SetBodies(ndBodyKinematic* const body0, ndBodyKinematic* const body1)
{
	dAssert(body0);
	dAssert(body1);
	m_body0 = body0;
	m_body1 = body1;
	if (m_body0->GetInvMass() == dFloat32(0.0f))
	{
		dSwap(m_body1, m_body0);
	}
	dAssert(m_body0->GetInvMass() > dFloat32(0.0f));
}

void ndContact::AttachToBodies()
{
	m_isAttached = true;
	m_body0->AttachContact(this);
	m_body1->AttachContact(this);
}

void ndContact::DetachFromBodies()
{
	m_isAttached = false;
	m_body0->DetachContact(this);
	m_body1->DetachContact(this);
}

void ndContact::JacobianDerivative(ndConstraintDescritor& desc)
{
	dInt32 frictionIndex = 0;
	if (m_maxDOF) 
	{
		dInt32 i = 0;
		frictionIndex = m_contacPointsList.GetCount();
		for (ndContactPointList::dNode* node = m_contacPointsList.GetFirst(); node; node = node->GetNext())
		{
			const ndContactMaterial& contact = node->GetInfo();
			JacobianContactDerivative(desc, contact, i, frictionIndex);
			i++;
		}
	}
	desc.m_rowsCount = frictionIndex;
}

void ndContact::CalculatePointDerivative(dInt32 index, ndConstraintDescritor& desc, const dVector& dir, const dgPointParam& param) const
{
	dAssert(m_body0);
	dAssert(m_body1);

	//desc.m_flags[index] = 0;
	ndJacobian &jacobian0 = desc.m_jacobian[index].m_jacobianM0;
	ndJacobian &jacobian1 = desc.m_jacobian[index].m_jacobianM1;
	jacobian0.m_linear = dir;
	jacobian1.m_linear = dir * dVector::m_negOne;

	jacobian0.m_angular = param.m_r0.CrossProduct(dir);
	jacobian1.m_angular = dir.CrossProduct(param.m_r1);

	dAssert(jacobian0.m_linear.m_w == dFloat32(0.0f));
	dAssert(jacobian0.m_angular.m_w == dFloat32(0.0f));
	dAssert(jacobian1.m_linear.m_w == dFloat32(0.0f));
	dAssert(jacobian1.m_angular.m_w == dFloat32(0.0f));
}

void ndContact::JacobianContactDerivative(ndConstraintDescritor& desc, const ndContactMaterial& contact, dInt32 normalIndex, dInt32& frictionIndex)
{
	dgPointParam pointData;
	InitPointParam(pointData, dFloat32(1.0f), contact.m_point, contact.m_point);
	CalculatePointDerivative(normalIndex, desc, contact.m_normal, pointData);

	const dVector omega0(m_body0->GetOmega());
	const dVector omega1(m_body1->GetOmega());
	const dVector veloc0 (m_body0->GetVelocity());
	const dVector veloc1(m_body1->GetVelocity());
	const dVector gyroAlpha0(m_body0->GetGyroAlpha());
	const dVector gyroAlpha1(m_body1->GetGyroAlpha());

	dAssert(contact.m_normal.m_w == dFloat32(0.0f));
	const ndJacobian& normalJacobian0 = desc.m_jacobian[normalIndex].m_jacobianM0;
	const ndJacobian& normalJacobian1 = desc.m_jacobian[normalIndex].m_jacobianM1;
	const dFloat32 restitutionCoefficient = contact.m_material.m_restitution;
	
	dFloat32 relSpeed = -(normalJacobian0.m_linear * veloc0 + normalJacobian0.m_angular * omega0 + normalJacobian1.m_linear * veloc1 + normalJacobian1.m_angular * omega1).AddHorizontal().GetScalar();
	dFloat32 penetration = dClamp(contact.m_penetration - D_RESTING_CONTACT_PENETRATION, dFloat32(0.0f), dFloat32(0.5f));
	desc.m_flags[normalIndex] = contact.m_material.m_flags & m_isSoftContact;
	desc.m_penetration[normalIndex] = penetration;
	desc.m_restitution[normalIndex] = restitutionCoefficient;
	desc.m_forceBounds[normalIndex].m_low = dFloat32(0.0f);
	desc.m_forceBounds[normalIndex].m_normalIndex = D_INDEPENDENT_ROW;
	desc.m_forceBounds[normalIndex].m_jointForce = (ndForceImpactPair*)&contact.m_normal_Force;
	
	const dFloat32 restitutionVelocity = (relSpeed > D_REST_RELATIVE_VELOCITY) ? relSpeed * restitutionCoefficient : dFloat32(0.0f);
	const dFloat32 penetrationStiffness = D_MAX_PENETRATION_STIFFNESS * contact.m_material.m_softness;
	const dFloat32 penetrationVeloc = penetration * penetrationStiffness;
	dAssert(dAbs(penetrationVeloc - D_MAX_PENETRATION_STIFFNESS * contact.m_material.m_softness * penetration) < dFloat32(1.0e-6f));
	desc.m_penetrationStiffness[normalIndex] = penetrationStiffness;
	relSpeed += dMax(restitutionVelocity, penetrationVeloc);
	
	const bool isHardContact = !(contact.m_material.m_flags & m_isSoftContact);
	desc.m_diagonalRegularizer[normalIndex] = isHardContact ? D_DIAGONAL_REGULARIZER : dMax(D_DIAGONAL_REGULARIZER, contact.m_material.m_skinThickness);
	const dFloat32 relGyro = (normalJacobian0.m_angular * gyroAlpha0 + normalJacobian1.m_angular * gyroAlpha1).AddHorizontal().GetScalar();
	
	desc.m_jointAccel[normalIndex] = relGyro + relSpeed * desc.m_timestep;
	if (contact.m_material.m_flags & m_overrideNormalAccel)
	{
		dAssert(0);
		desc.m_jointAccel[normalIndex] += contact.m_normal_Force.m_force;
	}
//return;

	// first dir friction force
	if (contact.m_material.m_flags & m_friction0Enable)
	{
		dInt32 jacobIndex = frictionIndex;
		frictionIndex += 1;
		dAssert(contact.m_dir0.m_w == dFloat32(0.0f));
		CalculatePointDerivative(jacobIndex, desc, contact.m_dir0, pointData);
	
		const ndJacobian &jacobian0 = desc.m_jacobian[jacobIndex].m_jacobianM0;
		const ndJacobian &jacobian1 = desc.m_jacobian[jacobIndex].m_jacobianM1;
		dFloat32 relVelocErr = -(jacobian0.m_linear * veloc0 + jacobian0.m_angular * omega0 + jacobian1.m_linear * veloc1 + jacobian1.m_angular * omega1).AddHorizontal().GetScalar();
	
		desc.m_flags[jacobIndex] = 0;
		desc.m_forceBounds[jacobIndex].m_normalIndex = dInt16((contact.m_material.m_flags & m_override0Friction) ? D_INDEPENDENT_ROW : normalIndex);
		desc.m_diagonalRegularizer[jacobIndex] = D_DIAGONAL_REGULARIZER;
	
		desc.m_restitution[jacobIndex] = dFloat32(0.0f);
		desc.m_penetration[jacobIndex] = dFloat32(0.0f);
	
		desc.m_penetrationStiffness[jacobIndex] = dFloat32(0.0f);
		if (contact.m_material.m_flags & m_override0Accel)
		{
			// note: using restitution been negative to indicate that the acceleration was override
			desc.m_restitution[jacobIndex] = dFloat32(-1.0f);
			dAssert(0);
			desc.m_jointAccel[jacobIndex] = contact.m_dir0_Force.m_force;
		}
		else 
		{
			const dFloat32 relFrictionGyro = (jacobian0.m_angular * gyroAlpha0 + jacobian1.m_angular * gyroAlpha1).AddHorizontal().GetScalar();
			desc.m_restitution[jacobIndex] = dFloat32(0.0f);
			desc.m_jointAccel[jacobIndex] = relFrictionGyro + relVelocErr * desc.m_timestep;
		}
		if (dAbs(relVelocErr) > D_MAX_DYNAMIC_FRICTION_SPEED) 
		{
			desc.m_forceBounds[jacobIndex].m_low = -contact.m_material.m_dynamicFriction0;
			desc.m_forceBounds[jacobIndex].m_upper = contact.m_material.m_dynamicFriction0;
		}
		else 
		{
			desc.m_forceBounds[jacobIndex].m_low = -contact.m_material.m_staticFriction0;
			desc.m_forceBounds[jacobIndex].m_upper = contact.m_material.m_staticFriction0;
		}
		desc.m_forceBounds[jacobIndex].m_jointForce = (ndForceImpactPair*)&contact.m_dir0_Force;
	}
	
	if (contact.m_material.m_flags & m_friction1Enable)
	{
		dInt32 jacobIndex = frictionIndex;
		frictionIndex += 1;
		dAssert(contact.m_dir1.m_w == dFloat32(0.0f));
		CalculatePointDerivative(jacobIndex, desc, contact.m_dir1, pointData);
	
		const ndJacobian &jacobian0 = desc.m_jacobian[jacobIndex].m_jacobianM0;
		const ndJacobian &jacobian1 = desc.m_jacobian[jacobIndex].m_jacobianM1;
		dFloat32 relVelocErr = -(jacobian0.m_linear * veloc0 + jacobian0.m_angular * omega0 + jacobian1.m_linear * veloc1 + jacobian1.m_angular * omega1).AddHorizontal().GetScalar();
	
		desc.m_flags[jacobIndex] = 0;
		desc.m_forceBounds[jacobIndex].m_normalIndex = dInt16((contact.m_material.m_flags & m_override1Friction) ? D_INDEPENDENT_ROW : normalIndex);
		desc.m_diagonalRegularizer[jacobIndex] = D_DIAGONAL_REGULARIZER;
	
		desc.m_restitution[jacobIndex] = dFloat32(0.0f);
		desc.m_penetration[jacobIndex] = dFloat32(0.0f);
		desc.m_penetrationStiffness[jacobIndex] = dFloat32(0.0f);
		if (contact.m_material.m_flags & m_override1Accel)
		{
			// note: using restitution been negative to indicate that the acceleration was override
			desc.m_restitution[jacobIndex] = dFloat32(-1.0f);
			dAssert(0);
			desc.m_jointAccel[jacobIndex] = contact.m_dir1_Force.m_force;
		}
		else 
		{
			const dFloat32 relFrictionGyro = (jacobian0.m_angular * gyroAlpha0 + jacobian1.m_angular * gyroAlpha1).AddHorizontal().GetScalar();
			desc.m_restitution[jacobIndex] = dFloat32(0.0f);
			desc.m_jointAccel[jacobIndex] = relFrictionGyro + relVelocErr * desc.m_timestep;
		}
		if (dAbs(relVelocErr) > D_MAX_DYNAMIC_FRICTION_SPEED) 
		{
			desc.m_forceBounds[jacobIndex].m_low = -contact.m_material.m_dynamicFriction1;
			desc.m_forceBounds[jacobIndex].m_upper = contact.m_material.m_dynamicFriction1;
		}
		else 
		{
			desc.m_forceBounds[jacobIndex].m_low = -contact.m_material.m_staticFriction1;
			desc.m_forceBounds[jacobIndex].m_upper = contact.m_material.m_staticFriction1;
		}
		desc.m_forceBounds[jacobIndex].m_jointForce = (ndForceImpactPair*)&contact.m_dir1_Force;
	}
}

void ndContact::JointAccelerations(ndJointAccelerationDecriptor* const desc)
{
	const dVector bodyOmega0(m_body0->GetOmega());
	const dVector bodyOmega1(m_body1->GetOmega());
	const dVector bodyVeloc0(m_body0->GetVelocity());
	const dVector bodyVeloc1(m_body1->GetVelocity());
	const dVector gyroAlpha0(m_body0->GetGyroAlpha());
	const dVector gyroAlpha1(m_body1->GetGyroAlpha());

	const dInt32 count = desc->m_rowsCount;
	const dFloat32 timestep = desc->m_timestep;
	const dFloat32 invTimestep = desc->m_invTimestep;
	dAssert(desc->m_timestep > dFloat32(0.0f));
	dAssert(desc->m_invTimestep > dFloat32(0.0f));

	ndRightHandSide* const rightHandSide = desc->m_rightHandSide;
	const ndLeftHandSide* const leftHandSide = desc->m_leftHandSide;

	//dTrace(("zzzzzz %d: ", xxxxxxx));
	for (dInt32 k = 0; k < count; k++) 
	{
		// note: using restitution been negative to indicate that the acceleration was override
		ndRightHandSide* const rhs = &rightHandSide[k];
		if (rhs->m_restitution >= dFloat32(0.0f)) 
		{
			const ndLeftHandSide* const row = &leftHandSide[k];
			const ndJacobian& jacobian0 = row->m_Jt.m_jacobianM0;
			const ndJacobian& jacobian1 = row->m_Jt.m_jacobianM1;
		
			dVector relVeloc(jacobian0.m_linear * bodyVeloc0 + jacobian0.m_angular * bodyOmega0 + jacobian1.m_linear * bodyVeloc1 + jacobian1.m_angular * bodyOmega1);
			dFloat32 vRel = relVeloc.AddHorizontal().GetScalar();
			dFloat32 aRel = rhs->m_deltaAccel;
		
			if (rhs->m_normalForceIndex == D_INDEPENDENT_ROW) 
			{
				dAssert(rhs->m_restitution >= dFloat32 (0.0f));
				dAssert(rhs->m_restitution <= dFloat32(2.0f));
		
				dFloat32 penetrationVeloc = dFloat32(0.0f);
				dFloat32 restitution = (vRel <= dFloat32(0.0f)) ? (dFloat32(1.0f) + rhs->m_restitution) : dFloat32(1.0f);
				if (rhs->m_penetration > D_RESTING_CONTACT_PENETRATION * dFloat32(0.125f)) 
				{
					if (vRel > dFloat32(0.0f)) 
					{
						dFloat32 penetrationCorrection = vRel * timestep;
						dAssert(penetrationCorrection >= dFloat32(0.0f));
						rhs->m_penetration = dMax(dFloat32(0.0f), rhs->m_penetration - penetrationCorrection);
					}
					else 
					{
						dFloat32 penetrationCorrection = -vRel * timestep * rhs->m_restitution * dFloat32(8.0f);
						if (penetrationCorrection > rhs->m_penetration) 
						{
							rhs->m_penetration = dFloat32(0.001f);
						}
					}
					penetrationVeloc = -(rhs->m_penetration * rhs->m_penetrationStiffness);
				}
				vRel = vRel * restitution + penetrationVeloc;
			}
		
			const dFloat32 relGyro = (jacobian0.m_angular * gyroAlpha0 + jacobian1.m_angular * gyroAlpha1).AddHorizontal().GetScalar();
			rhs->m_coordenateAccel = relGyro + aRel - vRel * invTimestep;
			//dTrace(("%f ", rhs->m_coordenateAccel));
		}
	}
	//dTrace(("\n"));
}