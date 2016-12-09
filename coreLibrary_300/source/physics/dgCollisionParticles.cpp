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


#include "dgCollision.h"
#include "dgCollisionParticles.h"

dgCollisionParticles::dgCollisionParticles (const dgCollisionParticles& source)
{
	dgAssert (0);
}

dgCollisionParticles::dgCollisionParticles (dgWorld* const world, dgMeshEffect* const mesh, dgCollisionID collsionID)
{
	dgAssert (0);
}

dgCollisionParticles::dgCollisionParticles (dgWorld* const world, dgDeserialize deserialization, void* const userData, dgInt32 revisionNumber)
{
	dgAssert (0);
}

dgCollisionParticles::~dgCollisionParticles(void)
{
	dgAssert (0);
}


int dgCollisionParticles::GetCount() const
{
	dgAssert (0);
}

const dgVector* dgCollisionParticles::GetVelocity() const
{
	dgAssert (0);
}

const dgVector* dgCollisionParticles::GetPositions() const
{
	dgAssert (0);
}
	
const dgVector* dgCollisionParticles::GetAccelaration() const
{
	dgAssert (0);
}



dgInt32 CalculateSignature() const
{
	dgAssert (0);
}

void dgCollisionParticles::SetCollisionBBox(const dgVector& p0, const dgVector& p1)
{
	dgAssert (0);
}

void dgCollisionParticles::Serialize(dgSerialize callback, void* const userData) const
{
	dgAssert (0);
}

void dgCollisionParticles::CollideMasses(dgDynamicBody* const myBody, dgBody* const otherBody)
{
	dgAssert (0);
}

void dgCollisionParticles::CalcAABB(const dgMatrix& matrix, dgVector& p0, dgVector& p1) const
{
	dgAssert (0);
}

void dgCollisionParticles::IntegrateForces(dgDynamicBody* const body, dgFloat32 timestep)
{
	dgAssert (0);
}

dgMatrix dgCollisionParticles::CalculateInertiaAndCenterOfMass(const dgMatrix& m_alignMatrix, const dgVector& localScale, const dgMatrix& matrix) const
{
	dgAssert (0);
}



