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


#ifndef __DGCOLLISION_PARTICLES_H__
#define __DGCOLLISION_PARTICLES_H__


#include "dgCollision.h"
#include "dgCollisionConvex.h"

class dgCollisionParticles: public dgCollisionConvex
{
	public:
	dgCollisionParticles (const dgCollisionParticles& source);
	dgCollisionParticles (dgWorld* const world, dgMeshEffect* const mesh, dgCollisionID collsionID);
	dgCollisionParticles (dgWorld* const world, dgDeserialize deserialization, void* const userData, dgInt32 revisionNumber);
	virtual ~dgCollisionParticles(void);

	dgInt32 GetCount() const;
	const dgVector* GetVelocity() const;
	const dgVector* GetPositions() const;
	const dgVector* GetAccelaration() const;

	protected:
	virtual dgInt32 CalculateSignature() const;
	virtual void SetCollisionBBox(const dgVector& p0, const dgVector& p1);
	virtual void Serialize(dgSerialize callback, void* const userData) const;
	virtual void CollideMasses(dgDynamicBody* const myBody, dgBody* const otherBody);
	virtual void CalcAABB(const dgMatrix& matrix, dgVector& p0, dgVector& p1) const;
	virtual void IntegrateForces(dgDynamicBody* const body, dgFloat32 timestep);
	virtual dgMatrix CalculateInertiaAndCenterOfMass(const dgMatrix& m_alignMatrix, const dgVector& localScale, const dgMatrix& matrix) const;
//	void CalculateAcceleration(dgFloat32 timestep, const dgDynamicBody* const body);

	dgArray<dgVector> m_posit;
	dgArray<dgVector> m_veloc;
	dgArray<dgVector> m_accel;
	dgArray<dgVector> m_externalforce;
	dgInt32 m_particlesCount;

	friend class dgBroadPhase;
	friend class dgDynamicBody;
	friend class dgWorldDynamicUpdate;
};


#endif 

