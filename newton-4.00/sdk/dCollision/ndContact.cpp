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

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndScene.h"
#include "ndContact.h"
#include "ndBodyKinematic.h"
#include "ndContactOptions.h"

ndVector ndContact::m_initialSeparatingVector(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));

#define D_REST_RELATIVE_VELOCITY		ndFloat32 (1.0e-3f)
#define D_MAX_DYNAMIC_FRICTION_SPEED	ndFloat32 (0.3f)
#define D_MAX_PENETRATION_STIFFNESS		ndFloat32 (50.0f)
#define D_DIAGONAL_REGULARIZER			ndFloat32 (1.0e-3f)

ndContact::ndContact()
	:ndConstraint()
	,m_positAcc(ndFloat32(10.0f))
	,m_rotationAcc()
	,m_separatingVector(m_initialSeparatingVector)
	,m_contacPointsList()
	,m_body0(nullptr)
	,m_body1(nullptr)
	,m_material(nullptr)
	,m_timeOfImpact(ndFloat32(1.0e10f))
	,m_separationDistance(ndFloat32(0.0f))
	,m_contactPruningTolereance(D_PRUNE_CONTACT_TOLERANCE)
	,m_maxDOF(0)
	,m_sceneLru(0)
	,m_isDead(0)
	,m_isAttached(0)
	,m_isIntersetionTestOnly(0)
	,m_skeletonIntraCollision(1)
	,m_skeletonSelftCollision(1)
{
	m_active = 0;
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
	if (m_body0->GetInvMass() == ndFloat32(0.0f))
	{
		dSwap(m_body1, m_body0);
	}
	dAssert(m_body0->GetInvMass() > ndFloat32(0.0f));
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
	ndInt32 frictionIndex = 0;
	if (m_maxDOF) 
	{
		ndInt32 i = 0;
		frictionIndex = m_contacPointsList.GetCount();
		for (ndContactPointList::ndNode* node = m_contacPointsList.GetFirst(); node; node = node->GetNext())
		{
			const ndContactMaterial& contact = node->GetInfo();
			JacobianContactDerivative(desc, contact, i, frictionIndex);
			i++;
		}
	}
	desc.m_rowsCount = frictionIndex;
}

void ndContact::CalculatePointDerivative(ndInt32 index, ndConstraintDescritor& desc, const ndVector& dir, const dgPointParam& param) const
{
	dAssert(m_body0);
	dAssert(m_body1);

	ndJacobian &jacobian0 = desc.m_jacobian[index].m_jacobianM0;
	ndJacobian &jacobian1 = desc.m_jacobian[index].m_jacobianM1;
	jacobian0.m_linear = dir;
	jacobian1.m_linear = dir * ndVector::m_negOne;

	jacobian0.m_angular = param.m_r0.CrossProduct(dir);
	jacobian1.m_angular = dir.CrossProduct(param.m_r1);

	dAssert(jacobian0.m_linear.m_w == ndFloat32(0.0f));
	dAssert(jacobian0.m_angular.m_w == ndFloat32(0.0f));
	dAssert(jacobian1.m_linear.m_w == ndFloat32(0.0f));
	dAssert(jacobian1.m_angular.m_w == ndFloat32(0.0f));
}

void ndContact::JacobianContactDerivative(ndConstraintDescritor& desc, const ndContactMaterial& contact, ndInt32 normalIndex, ndInt32& frictionIndex)
{
	dgPointParam pointData;
	InitPointParam(pointData, contact.m_point, contact.m_point);
	CalculatePointDerivative(normalIndex, desc, contact.m_normal, pointData);

	const ndVector omega0(m_body0->GetOmega());
	const ndVector omega1(m_body1->GetOmega());
	const ndVector veloc0 (m_body0->GetVelocity());
	const ndVector veloc1(m_body1->GetVelocity());
	const ndVector gyroAlpha0(m_body0->GetGyroAlpha());
	const ndVector gyroAlpha1(m_body1->GetGyroAlpha());

	dAssert(contact.m_normal.m_w == ndFloat32(0.0f));
	const ndJacobian& normalJacobian0 = desc.m_jacobian[normalIndex].m_jacobianM0;
	const ndJacobian& normalJacobian1 = desc.m_jacobian[normalIndex].m_jacobianM1;
	const ndFloat32 restitutionCoefficient = contact.m_material.m_restitution;
	
	ndFloat32 relSpeed = -(normalJacobian0.m_linear * veloc0 + normalJacobian0.m_angular * omega0 + normalJacobian1.m_linear * veloc1 + normalJacobian1.m_angular * omega1).AddHorizontal().GetScalar();
	ndFloat32 penetration = dClamp(contact.m_penetration - D_RESTING_CONTACT_PENETRATION, ndFloat32(0.0f), ndFloat32(0.5f));
	desc.m_flags[normalIndex] = contact.m_material.m_flags & m_isSoftContact;
	desc.m_penetration[normalIndex] = penetration;
	desc.m_restitution[normalIndex] = restitutionCoefficient;
	desc.m_forceBounds[normalIndex].m_low = ndFloat32(0.0f);
	desc.m_forceBounds[normalIndex].m_normalIndex = D_INDEPENDENT_ROW;
	desc.m_forceBounds[normalIndex].m_jointForce = (ndForceImpactPair*)&contact.m_normal_Force;
	
	const ndFloat32 restitutionVelocity = (relSpeed > D_REST_RELATIVE_VELOCITY) ? relSpeed * restitutionCoefficient : ndFloat32(0.0f);
	const ndFloat32 penetrationStiffness = D_MAX_PENETRATION_STIFFNESS * contact.m_material.m_softness;
	const ndFloat32 penetrationVeloc = penetration * penetrationStiffness;
	dAssert(dAbs(penetrationVeloc - D_MAX_PENETRATION_STIFFNESS * contact.m_material.m_softness * penetration) < ndFloat32(1.0e-6f));
	desc.m_penetrationStiffness[normalIndex] = penetrationStiffness;
	relSpeed += dMax(restitutionVelocity, penetrationVeloc);
	
	const bool isHardContact = !(contact.m_material.m_flags & m_isSoftContact);
	desc.m_diagonalRegularizer[normalIndex] = isHardContact ? D_DIAGONAL_REGULARIZER : dMax(D_DIAGONAL_REGULARIZER, contact.m_material.m_skinMargin);
	const ndFloat32 relGyro = (normalJacobian0.m_angular * gyroAlpha0 + normalJacobian1.m_angular * gyroAlpha1).AddHorizontal().GetScalar();
	
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
		ndInt32 jacobIndex = frictionIndex;
		frictionIndex += 1;
		dAssert(contact.m_dir0.m_w == ndFloat32(0.0f));
		CalculatePointDerivative(jacobIndex, desc, contact.m_dir0, pointData);
	
		const ndJacobian &jacobian0 = desc.m_jacobian[jacobIndex].m_jacobianM0;
		const ndJacobian &jacobian1 = desc.m_jacobian[jacobIndex].m_jacobianM1;
		ndFloat32 relVelocErr = -(jacobian0.m_linear * veloc0 + jacobian0.m_angular * omega0 + jacobian1.m_linear * veloc1 + jacobian1.m_angular * omega1).AddHorizontal().GetScalar();
	
		desc.m_flags[jacobIndex] = 0;
		desc.m_forceBounds[jacobIndex].m_normalIndex = ndInt16((contact.m_material.m_flags & m_override0Friction) ? D_INDEPENDENT_ROW : normalIndex);
		desc.m_diagonalRegularizer[jacobIndex] = D_DIAGONAL_REGULARIZER;
	
		desc.m_restitution[jacobIndex] = ndFloat32(0.0f);
		desc.m_penetration[jacobIndex] = ndFloat32(0.0f);
	
		desc.m_penetrationStiffness[jacobIndex] = ndFloat32(0.0f);
		if (contact.m_material.m_flags & m_override0Accel)
		{
			// note: using restitution been negative to indicate that the acceleration was override
			desc.m_restitution[jacobIndex] = ndFloat32(-1.0f);
			dAssert(0);
			desc.m_jointAccel[jacobIndex] = contact.m_dir0_Force.m_force;
		}
		else 
		{
			const ndFloat32 relFrictionGyro = (jacobian0.m_angular * gyroAlpha0 + jacobian1.m_angular * gyroAlpha1).AddHorizontal().GetScalar();
			desc.m_restitution[jacobIndex] = ndFloat32(0.0f);
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
		ndInt32 jacobIndex = frictionIndex;
		frictionIndex += 1;
		dAssert(contact.m_dir1.m_w == ndFloat32(0.0f));
		CalculatePointDerivative(jacobIndex, desc, contact.m_dir1, pointData);
	
		const ndJacobian &jacobian0 = desc.m_jacobian[jacobIndex].m_jacobianM0;
		const ndJacobian &jacobian1 = desc.m_jacobian[jacobIndex].m_jacobianM1;
		ndFloat32 relVelocErr = -(jacobian0.m_linear * veloc0 + jacobian0.m_angular * omega0 + jacobian1.m_linear * veloc1 + jacobian1.m_angular * omega1).AddHorizontal().GetScalar();
	
		desc.m_flags[jacobIndex] = 0;
		desc.m_forceBounds[jacobIndex].m_normalIndex = ndInt16((contact.m_material.m_flags & m_override1Friction) ? D_INDEPENDENT_ROW : normalIndex);
		desc.m_diagonalRegularizer[jacobIndex] = D_DIAGONAL_REGULARIZER;
	
		desc.m_restitution[jacobIndex] = ndFloat32(0.0f);
		desc.m_penetration[jacobIndex] = ndFloat32(0.0f);
		desc.m_penetrationStiffness[jacobIndex] = ndFloat32(0.0f);
		if (contact.m_material.m_flags & m_override1Accel)
		{
			// note: using restitution been negative to indicate that the acceleration was override
			desc.m_restitution[jacobIndex] = ndFloat32(-1.0f);
			dAssert(0);
			desc.m_jointAccel[jacobIndex] = contact.m_dir1_Force.m_force;
		}
		else 
		{
			const ndFloat32 relFrictionGyro = (jacobian0.m_angular * gyroAlpha0 + jacobian1.m_angular * gyroAlpha1).AddHorizontal().GetScalar();
			desc.m_restitution[jacobIndex] = ndFloat32(0.0f);
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
	const ndVector bodyOmega0(m_body0->GetOmega());
	const ndVector bodyOmega1(m_body1->GetOmega());
	const ndVector bodyVeloc0(m_body0->GetVelocity());
	const ndVector bodyVeloc1(m_body1->GetVelocity());
	const ndVector gyroAlpha0(m_body0->GetGyroAlpha());
	const ndVector gyroAlpha1(m_body1->GetGyroAlpha());

	const ndInt32 count = desc->m_rowsCount;
	const ndFloat32 timestep = desc->m_timestep;
	const ndFloat32 invTimestep = desc->m_invTimestep;
	dAssert(desc->m_timestep > ndFloat32(0.0f));
	dAssert(desc->m_invTimestep > ndFloat32(0.0f));

	ndRightHandSide* const rightHandSide = desc->m_rightHandSide;
	const ndLeftHandSide* const leftHandSide = desc->m_leftHandSide;

	for (ndInt32 k = 0; k < count; k++) 
	{
		// note: using restitution been negative to indicate that the acceleration was override
		ndRightHandSide* const rhs = &rightHandSide[k];
		if (rhs->m_restitution >= ndFloat32(0.0f)) 
		{
			const ndLeftHandSide* const row = &leftHandSide[k];
			const ndJacobian& jacobian0 = row->m_Jt.m_jacobianM0;
			const ndJacobian& jacobian1 = row->m_Jt.m_jacobianM1;
		
			ndVector relVeloc(jacobian0.m_linear * bodyVeloc0 + jacobian0.m_angular * bodyOmega0 + jacobian1.m_linear * bodyVeloc1 + jacobian1.m_angular * bodyOmega1);
			ndFloat32 vRel = relVeloc.AddHorizontal().GetScalar();
			ndFloat32 aRel = rhs->m_deltaAccel;
		
			if (rhs->m_normalForceIndex == D_INDEPENDENT_ROW) 
			{
				dAssert(rhs->m_restitution >= ndFloat32 (0.0f));
				dAssert(rhs->m_restitution <= ndFloat32(2.0f));
		
				ndFloat32 penetrationVeloc = ndFloat32(0.0f);
				ndFloat32 restitution = (vRel <= ndFloat32(0.0f)) ? (ndFloat32(1.0f) + rhs->m_restitution) : ndFloat32(1.0f);
				if (rhs->m_penetration > D_RESTING_CONTACT_PENETRATION * ndFloat32(0.125f)) 
				{
					if (vRel > ndFloat32(0.0f)) 
					{
						ndFloat32 penetrationCorrection = vRel * timestep;
						dAssert(penetrationCorrection >= ndFloat32(0.0f));
						rhs->m_penetration = dMax(ndFloat32(0.0f), rhs->m_penetration - penetrationCorrection);
					}
					else 
					{
						ndFloat32 penetrationCorrection = -vRel * timestep * rhs->m_restitution * ndFloat32(8.0f);
						if (penetrationCorrection > rhs->m_penetration) 
						{
							rhs->m_penetration = ndFloat32(0.001f);
						}
					}
					penetrationVeloc = -(rhs->m_penetration * rhs->m_penetrationStiffness);
				}
				vRel = vRel * restitution + penetrationVeloc;
			}
		
			const ndFloat32 relGyro = (jacobian0.m_angular * gyroAlpha0 + jacobian1.m_angular * gyroAlpha1).AddHorizontal().GetScalar();
			rhs->m_coordenateAccel = relGyro + aRel - vRel * invTimestep;
			//dTrace(("%f ", rhs->m_coordenateAccel));
		}
	}
	//dTrace(("\n"));
}