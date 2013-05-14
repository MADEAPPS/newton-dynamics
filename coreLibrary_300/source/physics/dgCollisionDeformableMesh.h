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

		dgVector m_com;
		dgInt32 m_count;
		dgFloat32* m_unitMass;
		dgVector* m_posit;
		dgVector* m_veloc;
		dgVector* m_force;
//		dgVector* m_deltaPosition;
//		dgVector* m_shapePosition;
//		dgVector* m_instantVelocity;
//		dgVector* m_internalVelocity;
	};

	class dgVisualVertexData
	{
		public:
		dgInt32 m_vertexIndex;
		dgFloat32 m_normals[3];
		dgFloat32 m_uv0[2];
		dgFloat32 m_uv1[2];
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
		dgInt16* m_indexList;
	};

	dgCollisionDeformableMesh (const dgCollisionDeformableMesh& source);
	dgCollisionDeformableMesh (dgWorld* const world, dgMeshEffect* const mesh, dgCollisionID collsionID);
	dgCollisionDeformableMesh (dgWorld* const world, dgDeserialize deserialization, void* const userData);
	virtual ~dgCollisionDeformableMesh(void);

	dgInt32 GetParticleCount() const;
	dgVector GetParticlePosition(dgInt32 index) const;

	void UpdateVisualNormals();
	dgInt32 GetVisualPointsCount() const;
	void SetSkinThickness (dgFloat32 skinThickness);
	void GetVisualVertexData(dgInt32 vertexStrideInByte, dgFloat32* const vertex, dgInt32 normalStrideInByte, dgFloat32* const normals, dgInt32 uvStrideInByte0, dgFloat32* const uv0, dgInt32 uvStrideInByte1, dgFloat32* const uv1);
	
	virtual void IntegrateVelocities (dgFloat32 timestep) = 0;
	virtual void CalculateInternalForces (dgFloat32 timestep) = 0;

	void ConstraintParticle (dgInt32 particleIndex, const dgVector& posit, const dgBody* const body);

	void* GetFirtVisualSegment() const;
	void* GetNextVisualSegment(void* const segment) const;
	dgInt32 GetSegmentMaterial (void* const segment) const;
	dgInt32 GetSegmentIndexCount (void* const segment) const;
	const dgInt16* GetSegmentIndexList (void* const segment) const;

/*
	

	void SetStiffness (dgFloat32 stiffness);
	void SetPlasticity (dgFloat32 plasticity);
	virtual void SetParticlesMasses (dgFloat32 totalMass);
	virtual void SetParticlesVelocities (const dgVector& velocity);
	virtual void SetMatrix (const dgMatrix& matrix);
	virtual void ApplyExternalAndInternalForces (dgDeformableBody* const myBody, dgFloat32 timestep, dgInt32 threadIndex);
*/
	protected:
	

	class dgDeformableNode;
/*
	void Serialize(dgSerialize callback, void* const userData) const;
	virtual void GetCollisionInfo(dgCollisionInfo* const info) const;
	virtual dgFloat32 RayCast (const dgVector& localP0, const dgVector& localP1, dgContactPoint& contactOut, const dgBody* const body, void* const userData) const;
	virtual dgFloat32 RayCastSimd (const dgVector& localP0, const dgVector& localP1, dgContactPoint& contactOut, const dgBody* const body, void* const userData) const;
	virtual void GetCollidingFaces (dgPolygonMeshDesc* const data) const;
	virtual void GetCollidingFacesSimd (dgPolygonMeshDesc* const data) const;
	virtual void DebugCollision (const dgMatrix& matrixPtr, OnDebugCollisionMeshCallback callback, void* const userData) const;
	virtual dgInt32 CalculateSignature () const;
	virtual void UpdateCollision ();

	void CalculateContactsToCollisionTree (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy, dgInt32 useSimd);
	void CalculatePolygonContacts (dgDeformableNode* const node, const dgPlane& plane, dgCollisionConvexPolygon* const polygon, dgFloat32 timestep);
	void CreateRegions();
	dgFloat32 m_stiffness;
	dgFloat32 m_plasticity;
*/

	virtual void SetCollisionBBox (const dgVector& p0, const dgVector& p1);

	bool SanityCheck () const;
	void ImproveTotalFitness();
	void ImproveNodeFitness (dgDeformableNode* const node);
	dgDeformableNode* BuildTopDown (dgInt32 count, dgDeformableNode* const children, dgDeformableNode* const parent);
	dgFloat32 CalculateSurfaceArea (const dgDeformableNode* const node0, const dgDeformableNode* const node1, dgVector& minBox, dgVector& maxBox) const;

	dgInt32 CalculateContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy, dgInt32 useSimd);
	//dgInt32 CalculateContacts (dgCollisionParamProxy& proxy, dgVector* const contactsOut) const;


	dgParticle m_particles;
	dgList<dgMeshSegment> m_visualSegments;
	dgFloat32 m_skinThickness;
	dgInt32 m_nodesCount;
	dgInt32 m_trianglesCount;
	dgInt32 m_visualVertexCount;

	dgWorld* m_world;
	dgInt16* m_indexList;
	dgVector* m_faceNormals;
	dgDeformableNode* m_rootNode;
	dgDeformableNode* m_nodesMemory;
	dgVisualVertexData* m_visualVertexData; 
	bool m_isdoubleSided;

	friend class dgWorld;
};


#endif 

