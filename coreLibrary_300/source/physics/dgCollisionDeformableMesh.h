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


#ifndef __DGCOLLISION_DEFORMABLE_MESH_H__
#define __DGCOLLISION_DEFORMABLE_MESH_H__


#include "dgCollision.h"
#include "dgCollisionConvex.h"

class dgDeformableBody;
class dgDeformableContact;
class dgCollisionConvexPolygon;

#define DG_SOFTBODY_BASE_SIZE	8

class dgCollisionDeformableMesh: public dgCollisionConvex
{
	public:
	class dgParticle
	{
		public:
		dgParticle(dgInt32 particlesCount);
		dgParticle(const dgParticle& source);
		dgParticle (dgWorld* const world, dgDeserialize deserialization, void* const userData);
		~dgParticle();
		
		dgVector* m_posit;
		dgVector* m_veloc;
		dgFloat32* m_unitMass;
		dgInt32 m_count;
	};

	class dgVisualVertexData
	{
		public:
		dgFloat32 m_normals[3];
		dgFloat32 m_uv0[2];
		dgInt32 m_vertexIndex;
	};

	class dgMeshSegment
	{
		public: 
		dgMeshSegment ()
		{
		}
		~dgMeshSegment ()
		{
			if (m_indexList) {
				dgFree(m_indexList);
			}
		}

		dgInt32 m_material;
		dgInt32 m_indexCount;
		dgInt32* m_indexList;
	};

	dgCollisionDeformableMesh (const dgCollisionDeformableMesh& source);
	dgCollisionDeformableMesh (dgWorld* const world, dgMeshEffect* const mesh, dgCollisionID collsionID);
	dgCollisionDeformableMesh (dgWorld* const world, dgDeserialize deserialization, void* const userData);
	virtual ~dgCollisionDeformableMesh(void);

	dgBody* GetBody() const;
	dgInt32 GetParticleCount() const;
	dgVector GetParticlePosition(dgInt32 index) const;

	void SetOnDebugDisplay (dgCollision::OnDebugCollisionMeshCallback debugDisplay);

	void UpdateVisualNormals();
	dgInt32 GetVisualPointsCount() const;
	void SetSkinThickness (dgFloat32 skinThickness);
	void GetVisualVertexData(dgInt32 vertexStrideInByte, dgFloat32* const vertex, dgInt32 normalStrideInByte, dgFloat32* const normals, dgInt32 uvStrideInByte0, dgFloat32* const uv0);


	
	virtual void SetMass (dgFloat32 mass) = 0;
    virtual void SetMatrix(const dgMatrix& matrix) = 0;
	virtual void ApplyExternalForces (dgFloat32 timestep) = 0;
	virtual void ResolvePositionsConstraints (dgFloat32 timestep) = 0;

	virtual void CreateClusters (dgInt32 count, dgFloat32 overlaringWidth) = 0;

	virtual void EndConfiguration () = 0;
	virtual void ConstraintParticle (dgInt32 particleIndex, const dgVector& posit, const dgBody* const body) = 0;

	void* GetFirtVisualSegment() const;
	void* GetNextVisualSegment(void* const segment) const;
	dgInt32 GetSegmentMaterial (void* const segment) const;
	dgInt32 GetSegmentIndexCount (void* const segment) const;
	const dgInt32* GetSegmentIndexList (void* const segment) const;

	protected:
	class dgDeformableNode;

	virtual void SetCollisionBBox (const dgVector& p0, const dgVector& p1);
	virtual void DebugCollision (const dgMatrix& matrixPtr, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const;
    virtual void CalcAABB (const dgMatrix& matrix, dgVector& p0, dgVector& p1) const;

	bool SanityCheck () const;
	void ImproveTotalFitness();
	void ImproveNodeFitness (dgDeformableNode* const node);
	dgDeformableNode* BuildTopDown (dgInt32 count, dgDeformableNode* const children, dgDeformableNode* const parent);
	dgFloat32 CalculateSurfaceArea (const dgDeformableNode* const node0, const dgDeformableNode* const node1, dgVector& minBox, dgVector& maxBox) const;
	dgInt32 CalculateContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy);

	dgFloat32 CalculateMassProperties (const dgMatrix& offset, dgVector& inertia, dgVector& crossInertia, dgVector& centerOfMass) const;

	dgVector m_basePosit;
	dgParticle m_particles;
	dgList<dgMeshSegment> m_visualSegments;
	dgFloat32 m_skinThickness;
	dgInt32 m_nodesCount;
	dgInt32 m_trianglesCount;
	dgInt32 m_visualVertexCount;

	dgWorld* m_world;
	dgDeformableBody* m_myBody;
	dgInt32* m_indexList;
	dgVector* m_faceNormals;
	dgDeformableNode* m_rootNode;
	dgDeformableNode* m_nodesMemory;
	dgVisualVertexData* m_visualVertexData; 
	mutable dgCollision::OnDebugCollisionMeshCallback m_onDebugDisplay;
	bool m_isdoubleSided;

	friend class dgWorld;
	friend class dgDeformableBody;
};


#endif 

