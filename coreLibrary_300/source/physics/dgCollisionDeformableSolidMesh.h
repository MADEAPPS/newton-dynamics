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


#ifndef __DGCOLLISION_DEFORMABLE_SOLID_MESH_H__
#define __DGCOLLISION_DEFORMABLE_SOLID_MESH_H__


#include "dgCollision.h"
#include "dgCollisionConvex.h"
#include "dgCollisionDeformableMesh.h"



class dgCollisionDeformableSolidMesh: public dgCollisionDeformableMesh
{
	public:
	class dgDeformationRegion;

	dgCollisionDeformableSolidMesh (const dgCollisionDeformableSolidMesh& source);
	dgCollisionDeformableSolidMesh (dgWorld* const world, dgMeshEffect* const mesh);
	dgCollisionDeformableSolidMesh (dgWorld* const world, dgDeserialize deserialization, void* const userData);
	virtual ~dgCollisionDeformableSolidMesh(void);

	void Serialize(dgSerialize callback, void* const userData) const;
	virtual dgInt32 CalculateSignature () const;

	void SetParticlesPositions (const dgMatrix& matrix);
	virtual void ApplyExternalForces (dgFloat32 timestep);
	virtual void ResolvePositionsConstraints (dgFloat32 timestep);

	virtual void EndConfiguration ();
	virtual void ConstraintParticle (dgInt32 particleIndex, const dgVector& posit, const dgBody* const body);

	void CreateRegions();

	dgVector* m_shapePosition;
	dgDeformationRegion* m_regions;
	dgInt32 m_regionsCount;

	dgFloat32 m_stiffness;
	friend class dgWorld;
};



#endif 

