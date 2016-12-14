/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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

#include "dgPhysicsStdafx.h"
#include "dgWorld.h"
#include "dgContact.h"
#include "dgCollision.h"
#include "dgMeshEffect.h"
#include "dgDynamicBody.h"
#include "dgCollisionLumpedMassParticles.h"



dgCollisionLumpedMassParticles::dgCollisionLumpedMassParticles(dgWorld* const world, dgCollisionID collisionID)
	:dgCollisionConvex(world->GetAllocator(), 0, collisionID)
	,m_posit(DG_LUMPED_MASSES_GRANULARITY, world->GetAllocator())
	,m_veloc(DG_LUMPED_MASSES_GRANULARITY, world->GetAllocator())
	,m_accel(DG_LUMPED_MASSES_GRANULARITY, world->GetAllocator())
	,m_externalforce(DG_LUMPED_MASSES_GRANULARITY, world->GetAllocator())
	,m_body(NULL)
	,m_unitMass(dgFloat32 (1.0f))
	,m_unitInertia(dgFloat32 (1.0f))
	,m_particlesCount(0)
{
	m_rtti |= dgCollisionLumpedMass_RTTI;
}

dgCollisionLumpedMassParticles::dgCollisionLumpedMassParticles (const dgCollisionLumpedMassParticles& source)
	:dgCollisionConvex(source)
	,m_posit(source.m_particlesCount, source.GetAllocator())
	,m_veloc(source.m_particlesCount, source.GetAllocator())
	,m_accel(source.m_particlesCount, source.GetAllocator())
	,m_externalforce(source.m_particlesCount, source.GetAllocator())
	,m_body(NULL)
	,m_unitMass(source.m_unitMass)
	,m_unitInertia(source.m_unitMass)
	,m_particlesCount(source.m_particlesCount)
{
	m_rtti |= dgCollisionLumpedMass_RTTI;
	memcpy(&m_posit[0], &source.m_posit[0], m_particlesCount * sizeof(dgVector));
	memcpy(&m_veloc[0], &source.m_veloc[0], m_particlesCount * sizeof(dgVector));
	memcpy(&m_accel[0], &source.m_posit[0], m_particlesCount * sizeof(dgVector));
	memcpy(&m_externalforce[0], &source.m_externalforce[0], m_particlesCount * sizeof(dgVector));
}


dgCollisionLumpedMassParticles::dgCollisionLumpedMassParticles (dgWorld* const world, dgDeserialize deserialization, void* const userData, dgInt32 revisionNumber)
	:dgCollisionConvex(world, deserialization, userData, revisionNumber)
	,m_posit(0, world->GetAllocator())
	,m_veloc(0, world->GetAllocator())
	,m_accel(0, world->GetAllocator())
	,m_externalforce(0, world->GetAllocator())
	,m_body(NULL)
	,m_unitMass(dgFloat32(1.0f))
	,m_unitInertia(dgFloat32(1.0f))
	,m_particlesCount(0)
{
	m_rtti |= dgCollisionLumpedMass_RTTI;
	dgAssert (0);
}

dgCollisionLumpedMassParticles::~dgCollisionLumpedMassParticles(void)
{
}

void dgCollisionLumpedMassParticles::FinalizeBuild()
{
	m_veloc.Resize(m_particlesCount);
	m_accel.Resize(m_particlesCount);
	m_externalforce.Resize(m_particlesCount);

	dgVector com(dgFloat32(0.0f));
	dgVector* const posit = &m_posit[0];
	for (dgInt32 i = 0; i < m_particlesCount; i++) {
		com += posit[i];
		m_accel[i] = dgVector::m_zero;
		m_veloc[i] = dgVector::m_zero;
		m_externalforce[i] = dgVector::m_zero;
	}

	// for now use a fix size box
	m_boxSize = dgVector(dgFloat32(1.0f), dgFloat32(1.0f), dgFloat32(1.0f), dgFloat32(0.0f));
	m_boxOrigin = com.CompProduct4(dgFloat32(1.0f) / m_particlesCount);

	for (dgInt32 i = 0; i < m_particlesCount; i++) {
		posit[i] = posit[i] - m_boxOrigin;
	}
}

void dgCollisionLumpedMassParticles::DebugCollision (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dgAssert (0);
}

void dgCollisionLumpedMassParticles::SetOwnerAndUnitMass (dgDynamicBody* const body)
{
	m_body = body;

	dgMatrix matrix (body->GetMatrix());
	dgVector position (matrix.m_posit);
	matrix.m_posit = dgVector::m_wOne;

	dgVector* const posit = &m_posit[0];
	dgMatrix scaledTranform (body->m_collision->GetScaledTransform(matrix));
	for (dgInt32 i = 0; i < m_particlesCount; i++) {
		posit[i] = scaledTranform.TransformVector(posit[i]) & dgVector::m_triplexMask;
	}
	body->m_collision->SetScale(dgVector (dgFloat32 (1.0f)));
	body->m_collision->SetLocalMatrix (dgGetIdentityMatrix());
	matrix.m_posit = position;
	body->m_matrix = matrix;

	dgVector xxSum(dgFloat32(0.0f));
	dgVector xySum(dgFloat32(0.0f));
	dgVector xxSum2(dgFloat32(0.0f));
	for (dgInt32 i = 0; i < m_particlesCount; i++) {
		xxSum += posit[i];
		xxSum2 += posit[i].CompProduct4(posit[i]);
		xySum += posit[i].CompProduct4(posit[i].ShiftTripleRight());
	}
	dgVector den (dgFloat32(1.0f / m_particlesCount));

	body->m_localCentreOfMass = xxSum.CompProduct4(den);
	dgVector com2(body->m_localCentreOfMass.CompProduct4(body->m_localCentreOfMass));
	dgVector unitInertia(body->m_mass - com2.Scale4(body->m_mass.m_w));

	m_unitMass = body->m_mass.m_w / m_particlesCount;
	m_unitInertia = dgMax(unitInertia.m_x, unitInertia.m_y, unitInertia.m_z) / m_particlesCount;

//	dgVector yySum(xxSum.ShiftTripleRight());
//	dgVector com(xxSum.CompProduct4(den) + origin);
//	dgVector pxx0(origin - com);
//	dgVector pxy0(pxx0.ShiftTripleRight());
//	dgVector Ixx(unitMass.CompProduct4(xxSum2 + xxSum.CompProduct4(pxx0.CompProduct4(dgVector::m_two))) + pxx0.CompProduct4(pxx0.Scale4(m_body->m_mass.m_w)));
//	dgVector Ixy(unitMass.CompProduct4(xySum + xxSum.CompProduct4(pxy0) + yySum.CompProduct4(pxx0)) + pxx0.CompProduct4(pxy0.Scale4(m_body->m_mass.m_w)));
//	dgVector com2(body->m_localCentreOfMass.CompProduct4(body->m_localCentreOfMass));
}

int dgCollisionLumpedMassParticles::GetCount() const
{
	return m_particlesCount;
}

dgInt32 dgCollisionLumpedMassParticles::GetStrideInByte() const
{
	return sizeof (dgVector);
}

const dgVector* dgCollisionLumpedMassParticles::GetVelocity() const
{
	return &m_veloc[0];
}

const dgVector* dgCollisionLumpedMassParticles::GetPositions() const
{
	return &m_posit[0];
}
	
const dgVector* dgCollisionLumpedMassParticles::GetAcceleration() const
{
	return &m_accel[0];
}

dgInt32 dgCollisionLumpedMassParticles::CalculateSignature() const
{
	dgAssert (0);
	return 0;
}

void dgCollisionLumpedMassParticles::SetCollisionBBox(const dgVector& p0, const dgVector& p1)
{
	dgAssert (0);
}

void dgCollisionLumpedMassParticles::Serialize(dgSerialize callback, void* const userData) const
{
	dgAssert (0);
}

void dgCollisionLumpedMassParticles::RegisterCollision(const dgBody* const otherBody)
{
//	dgAssert (0);
}

void dgCollisionLumpedMassParticles::CalcAABB(const dgMatrix& matrix, dgVector& p0, dgVector& p1) const
{
	dgVector origin(matrix.TransformVector(m_boxOrigin));
	dgVector size(matrix.m_front.Abs().Scale4(m_boxSize.m_x) + matrix.m_up.Abs().Scale4(m_boxSize.m_y) + matrix.m_right.Abs().Scale4(m_boxSize.m_z));
	p0 = (origin - size) & dgVector::m_triplexMask;
	p1 = (origin + size) & dgVector::m_triplexMask;
}

dgFloat32 dgCollisionLumpedMassParticles::RayCast(const dgVector& localP0, const dgVector& localP1, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const
{
	// for now brute force ray cast
	dgVector p (m_posit[0]);
	dgFloat32 distance = dgFloat32 (1.0e10f);
	for (dgInt32 i = 0; i < m_particlesCount; i ++) {
		dgVector posit (dgPointToRayDistance (m_posit[i], localP0, localP1)); 
		dgVector step (posit - m_posit[i]);
		
		dgFloat32 dist2 = step.DotProduct3(step);
		if (dist2 < distance) {
			distance = dist2;
			p = m_posit[i];
		}
	}

	contactOut.m_point = p;
	contactOut.m_normal = dgVector (dgFloat32 (0.0f), dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector num (p - localP0);
	dgVector den (localP1 - localP0);
	dgFloat32 dist = num.DotProduct3(den) / den.DotProduct3(den);
	return dist;
}

dgMatrix dgCollisionLumpedMassParticles::CalculateInertiaAndCenterOfMass(const dgMatrix& m_alignMatrix, const dgVector& localScale, const dgMatrix& matrix) const
{
	dgVector com(dgFloat32(0.0f));
	for (dgInt32 i = 0; i < m_particlesCount; i++) {
		com = matrix.RotateVector(m_posit[i].CompProduct4(localScale));
	}
	dgVector den(dgFloat32(1.0f / m_particlesCount));
	dgMatrix inertia(dgGetIdentityMatrix());
	inertia.m_posit = com.CompProduct4(den);
	inertia.m_posit.m_w = dgFloat32(1.0f);
	return inertia;
}



