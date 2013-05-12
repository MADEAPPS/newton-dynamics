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

	dgCollisionDeformableSolidMesh (const dgCollisionDeformableSolidMesh& source);
	dgCollisionDeformableSolidMesh (dgMeshEffect* const mesh);
	dgCollisionDeformableSolidMesh (dgWorld* const world, dgDeserialize deserialization, void* const userData);
	virtual ~dgCollisionDeformableSolidMesh(void);

/*
	dgInt32 GetVisualPointsCount() const;
	void UpdateVisualNormals();
	void GetVisualVertexData(dgInt32 vertexStrideInByte, dgFloat32* const vertex,
		dgInt32 normalStrideInByte, dgFloat32* const normals, 
		dgInt32 uvStrideInByte0, dgFloat32* const uv0,
		dgInt32 uvStrideInByte1, dgFloat32* const uv1);

	void* GetFirtVisualSegment() const;
	void* GetNextVisualSegment(void* const segment) const;

	dgInt32 GetSegmentMaterial (void* const segment) const;
	dgInt32 GetSegmentIndexCount (void* const segment) const;
	const dgInt16* GetSegmentIndexList (void* const segment) const;

	void SetStiffness (dgFloat32 stiffness);
	void SetPlasticity (dgFloat32 plasticity);
	void SetSkinThickness (dgFloat32 skinThickness);


	
	virtual void SetParticlesMasses (dgFloat32 totalMass);
	virtual void SetParticlesVelocities (const dgVector& velocity);

	
	virtual void SetMatrix (const dgMatrix& matrix);
	virtual void ApplyExternalAndInternalForces (dgDeformableBody* const myBody, dgFloat32 timestep, dgInt32 threadIndex);

	protected:
	class dgDeformableNode;
	class dgDeformationRegion;

	
	virtual void GetCollisionInfo(dgCollisionInfo* const info) const;
	virtual dgFloat32 RayCast (const dgVector& localP0, const dgVector& localP1, dgContactPoint& contactOut, const dgBody* const body, void* const userData) const;
	virtual dgFloat32 RayCastSimd (const dgVector& localP0, const dgVector& localP1, dgContactPoint& contactOut, const dgBody* const body, void* const userData) const;
	virtual void GetCollidingFaces (dgPolygonMeshDesc* const data) const;
	virtual void GetCollidingFacesSimd (dgPolygonMeshDesc* const data) const;


	virtual void DebugCollision (const dgMatrix& matrixPtr, OnDebugCollisionMeshCallback callback, void* const userData) const;
	
	virtual void SetCollisionBBox (const dgVector& p0, const dgVector& p1);

	virtual void UpdateCollision ();
	void ImproveTotalFitness();
	void ImproveNodeFitness (dgDeformableNode* const node);
	dgDeformableNode* BuildTopDown (dgInt32 count, dgDeformableNode* const children, dgDeformableNode* const parent);
	dgFloat32 CalculateSurfaceArea (const dgDeformableNode* const node0, const dgDeformableNode* const node1, dgVector& minBox, dgVector& maxBox) const;

	dgInt32 CalculateContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy, dgInt32 useSimd);

	void CalculateContactsToCollisionTree (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy, dgInt32 useSimd);
	void CalculatePolygonContacts (dgDeformableNode* const node, const dgPlane& plane, dgCollisionConvexPolygon* const polygon, dgFloat32 timestep);
	void CreateRegions();
	bool SanityCheck () const;
*/

	void Serialize(dgSerialize callback, void* const userData) const;
	virtual dgInt32 CalculateSignature () const;

/*
	dgParticle m_particles;
	dgInt32 m_regionsCount;
	dgInt32 m_trianglesCount;
	dgInt32 m_nodesCount;
	dgFloat32 m_stiffness;
	dgFloat32 m_plasticity;
	dgFloat32 m_skinThickness;
	dgInt16* m_indexList;
	dgVector* m_faceNormals;
	dgDeformableNode* m_rootNode;
	dgDeformableNode* m_nodesMemory;
	dgDeformationRegion* m_regions;

	dgInt32 m_visualVertexCount;
	dgVisualVertexData* m_visualVertexData; 
	dgList<dgMeshSegment> m_visualSegments;
*/
	friend class dgWorld;
};



#endif 

