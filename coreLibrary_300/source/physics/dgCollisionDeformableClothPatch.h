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


#ifndef __DGCOLLISION_DEFORMABLE_CLOTH_PATCH_MESH_H__
#define __DGCOLLISION_DEFORMABLE_CLOTH_PATCH_MESH_H__


#include "dgCollision.h"
#include "dgCollisionConvex.h"
#include "dgCollisionDeformableMesh.h"



class dgCollisionDeformableClothPatch: public dgCollisionDeformableMesh
{
	public:
	class dgEdgePair;
	class dgClothLink;
	class dgSoftBodyEdge;


	dgCollisionDeformableClothPatch (const dgCollisionDeformableClothPatch& source);
	dgCollisionDeformableClothPatch (dgWorld* const world, dgMeshEffect* const mesh, const dgClothPatchMaterial& structuralMaterial, const dgClothPatchMaterial& bendMaterial);
	dgCollisionDeformableClothPatch (dgWorld* const world, dgDeserialize deserialization, void* const userData);
	virtual ~dgCollisionDeformableClothPatch(void);

//	void SetStiffness (dgFloat32 stiffness);
//	void SetPlasticity (dgFloat32 plasticity);
//	void SetSkinThickness (dgFloat32 skinThickness);
//	virtual void SetParticlesMasses (dgFloat32 totalMass);
//	virtual void SetParticlesVelocities (const dgVector& velocity);
//	virtual void SetMatrix (const dgMatrix& matrix);
//	virtual void ApplyExternalAndInternalForces (dgDeformableBody* const myBody, dgFloat32 timestep, dgInt32 threadIndex);

	protected:
//	virtual void GetCollisionInfo(dgCollisionInfo* const info) const;
//	virtual void GetCollidingFaces (dgPolygonMeshDesc* const data) const;
//	virtual void GetCollidingFacesSimd (dgPolygonMeshDesc* const data) const;
//	virtual void DebugCollision (const dgMatrix& matrixPtr, OnDebugCollisionMeshCallback callback, void* const userData) const;

	void Serialize(dgSerialize callback, void* const userData) const;
	virtual dgInt32 CalculateSignature () const;

	virtual void SetMass (dgFloat32 mass);
    virtual void SetMatrix(const dgMatrix& matrix);
	virtual void ApplyExternalForces (dgFloat32 timestep);
	virtual void ResolvePositionsConstraints (dgFloat32 timestep);

	virtual void CreateClusters (dgInt32 count, dgFloat32 overlaringWidth);

	virtual void EndConfiguration ();
	virtual void ConstraintParticle (dgInt32 particleIndex, const dgVector& posit, const dgBody* const body);

	dgClothPatchMaterial m_materials[2];
	dgInt32 m_linksCount;
	dgInt32 m_graphCount;
	dgVector* m_posit;
	dgInt32* m_linkOrder;
	dgClothLink* m_links;
	dgSoftBodyEdge* m_graph;
	dgSoftBodyEdge** m_particleToEddgeMap;
	friend class dgWorld;
};



#endif 

