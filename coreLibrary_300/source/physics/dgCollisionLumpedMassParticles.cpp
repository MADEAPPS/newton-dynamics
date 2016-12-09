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
#include "dgCollisionLumpedMassParticles.h"

dgCollisionLumpedMassParticles::dgCollisionLumpedMassParticles(dgWorld* const world, dgInt32 reserveCount, dgCollisionID collisionID)
	:dgCollisionConvex(world->GetAllocator(), 0, collisionID)
	,m_posit(reserveCount, world->GetAllocator())
	,m_veloc(reserveCount, world->GetAllocator())
	,m_accel(reserveCount, world->GetAllocator())
	,m_externalforce(reserveCount, world->GetAllocator())
	,m_unitMass(dgFloat32 (1.0f))
	,m_unitInertia(dgFloat32 (1.0f))
	,m_particlesCount(reserveCount)
{
	m_rtti |= dgCollisionLumpedMass_RTTI;
}

dgCollisionLumpedMassParticles::dgCollisionLumpedMassParticles (const dgCollisionLumpedMassParticles& source)
	:dgCollisionConvex(source)
	,m_posit(source.m_particlesCount, source.GetAllocator())
	,m_veloc(source.m_particlesCount, source.GetAllocator())
	,m_accel(source.m_particlesCount, source.GetAllocator())
	,m_externalforce(source.m_particlesCount, source.GetAllocator())
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


int dgCollisionLumpedMassParticles::GetCount() const
{
	return m_particlesCount;
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

void dgCollisionLumpedMassParticles::CollideMasses(dgDynamicBody* const myBody, dgBody* const otherBody)
{
	dgAssert (0);
}

void dgCollisionLumpedMassParticles::CalcAABB(const dgMatrix& matrix, dgVector& p0, dgVector& p1) const
{
	dgVector origin(matrix.TransformVector(m_boxOrigin));
	dgVector size(matrix.m_front.Abs().Scale4(m_boxSize.m_x) + matrix.m_up.Abs().Scale4(m_boxSize.m_y) + matrix.m_right.Abs().Scale4(m_boxSize.m_z));
	p0 = (origin - size) & dgVector::m_triplexMask;
	p1 = (origin + size) & dgVector::m_triplexMask;
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



