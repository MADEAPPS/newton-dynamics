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


#ifndef __DGCOLLISION_DEFORMABLE_MESH_H__
#define __DGCOLLISION_DEFORMABLE_MESH_H__

#include "dgCollision.h"
#include "dgCollisionLumpedMassParticles.h"


class dgCollisionDeformableMesh: public dgCollisionLumpedMassParticles
{
	public:
	dgCollisionDeformableMesh (const dgCollisionDeformableMesh& source);
	dgCollisionDeformableMesh (dgWorld* const world, dgCollisionID collisionID);
	dgCollisionDeformableMesh (dgWorld* const world, dgDeserialize deserialization, void* const userData, dgInt32 revisionNumber);
	virtual ~dgCollisionDeformableMesh(void);

	dgInt32 GetLinksCount() const;
	const dgInt16* GetLinks() const;
	const dgInt32* GetIndexToVertexMap() const;
	
	virtual void ConstraintParticle(dgInt32 particleIndex, const dgVector& posit, const dgBody* const body);

	void DisableInactiveLinks ();

	protected:
	class dgSoftLink
	{
		public:
		dgInt16 m_m0;
		dgInt16 m_m1;
	};


	virtual void FinalizeBuild();
	virtual void Serialize(dgSerialize callback, void* const userData) const;
	virtual void IntegrateForces(dgFloat32 timestep);
	virtual void HandleCollision (dgFloat32 timestep, dgVector* const normalDir, dgVector* const normalAccel, dgFloat32* const frictionCoefficient) const;
	virtual void CalculateAcceleration(dgFloat32 timestep) = 0;

	static dgInt32 CompareEdges(const dgSoftLink* const A, const dgSoftLink* const B, void* const context);
	dgFloat32 CalculaleContactPenetration(const dgVector& point, const dgVector& normal) const;

	dgArray<dgSoftLink> m_linkList;
	dgArray<dgFloat32> m_restlength;
	dgArray<dgInt32> m_indexToVertexMap;
	dgFloat32 m_skinThickness;
	dgInt32 m_linksCount;
	dgInt32 m_indexToVertexCount;
	

	static dgVector m_smallestLenght2;
};


#endif 

