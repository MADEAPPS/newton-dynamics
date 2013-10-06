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

#ifndef __DGCOLLISIONCOMPOUND_H__
#define __DGCOLLISIONCOMPOUND_H__


#include "dgCollision.h"

class dgCollisionBVH;
class dgCollisionInstance;



#define DG_COMPOUND_STACK_DEPTH	256

class dgCollisionCompound: public dgCollision
{
	protected:
	enum dgNodeType
	{
		m_leaf,
		m_node,
	};

	public:
	DG_MSC_VECTOR_ALIGMENT
	class dgOOBBTestData
	{
		public:
		dgOOBBTestData (const dgMatrix& matrix);
		dgOOBBTestData (const dgMatrix& matrix, const dgVector& p0, const dgVector& p1);

		dgMatrix m_matrix;
		dgMatrix m_absMatrix;
		dgVector m_origin;
		dgVector m_size;
		dgVector m_localP0;
		dgVector m_localP1;
		dgVector m_aabbP0;
		dgVector m_aabbP1;

		dgVector m_crossAxis[9];
		dgVector m_crossAxisAbs[9];
		dgVector m_crossAxisDotAbs[9];
		dgVector m_extendsMinX[3];
		dgVector m_extendsMaxX[3];
	} DG_GCC_VECTOR_ALIGMENT;


	DG_MSC_VECTOR_ALIGMENT
	class dgNodeBase
	{
		public:
		DG_CLASS_ALLOCATOR(allocator)
		dgNodeBase (); 
		dgNodeBase (const dgNodeBase& copyFrom);
		dgNodeBase (dgCollisionInstance* const instance);
		dgNodeBase (dgNodeBase* const left, dgNodeBase* const right);
		~dgNodeBase();

		void CalculateAABB();
		void SetBox (const dgVector& p0, const dgVector& p1);
		bool BoxTest (const dgOOBBTestData& data) const;
		bool BoxTest (const dgOOBBTestData& data, const dgNodeBase* const otherNode) const;

		DG_INLINE dgCollisionInstance* GetShape() const 
		{
			return m_shape;
		}

		DG_INLINE dgInt32 BoxIntersect (const dgFastRayTest& ray, const dgVector& boxP0, const dgVector& boxP1) const
		{
			dgVector minBox (m_p0 - boxP1);
			dgVector maxBox (m_p1 - boxP0);
			return ray.BoxTest(minBox, maxBox);
		}

		dgVector m_p0;
		dgVector m_p1;
		dgVector m_size;
		dgVector m_origin;
		dgFloat32 m_area;
		dgInt32 m_type;
		dgNodeBase* m_left;
		dgNodeBase* m_right;
		dgNodeBase* m_parent;
		dgCollisionInstance* m_shape;
		dgTree<dgNodeBase*, dgInt32>::dgTreeNode* m_myNode; 
	} DG_GCC_VECTOR_ALIGMENT;

	protected:
	class dgNodePairs
	{
		public:
		dgVector m_treeNodeP0;
		dgVector m_treeNodeP1;
		dgVector m_treeNodeSize;
		dgVector m_treeNodeOrigin;
		dgInt32 m_treeNodeIsLeaf;
		dgFloat32 m_treeNodeArea;
		const void* m_treeNode;
		dgNodeBase* m_myNode;
	};

	class dgSpliteInfo;
	class dgHeapNodePair;

	public:
	
	dgCollisionCompound (dgWorld* const world);
	dgCollisionCompound (const dgCollisionCompound& source);
	dgCollisionCompound (dgWorld* const world, dgDeserialize deserialization, void* const userData);
	virtual ~dgCollisionCompound();

	void BeginAddRemove ();
	dgTree<dgNodeBase*, dgInt32>::dgTreeNode* AddCollision (dgCollisionInstance* const part);
	void RemoveCollision (dgTree<dgNodeBase*, dgInt32>::dgTreeNode* const node);
	void SetCollisionMatrix (dgTree<dgNodeBase*, dgInt32>::dgTreeNode* const node, const dgMatrix& matrix);
	void EndAddRemove ();

	void ApplyScale (const dgVector& scale);
	void GetAABB (dgVector& p0, dgVector& p1) const;

	dgInt32 GetNodeIndex(dgTree<dgNodeBase*, dgInt32>::dgTreeNode* const node) const;
	dgTree<dgNodeBase*, dgInt32>::dgTreeNode* FindNodeByIndex (dgInt32 index) const;


	dgTree<dgNodeBase*, dgInt32>::dgTreeNode* GetFirstNode () const;
	dgTree<dgNodeBase*, dgInt32>::dgTreeNode* GetNextNode (dgTree<dgNodeBase*, dgInt32>::dgTreeNode* const node) const;
	dgCollisionInstance* GetCollisionFromNode (dgTree<dgNodeBase*, dgInt32>::dgTreeNode* const node) const;

	protected:
	void RemoveCollision (dgNodeBase* const node);
	virtual dgFloat32 GetVolume () const;
	virtual dgFloat32 GetBoxMinRadius () const; 
	virtual dgFloat32 GetBoxMaxRadius () const;
	
	virtual dgVector SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const;
	virtual void CalcAABB (const dgMatrix& matrix, dgVector& p0, dgVector& p1) const;
	
	static void CalculateInertia (void* userData, int vertexCount, const dgFloat32* const FaceArray, int faceId);

	virtual void MassProperties ();
	dgMatrix CalculateInertiaAndCenterOfMass (const dgMatrix& m_alignMatrix, const dgVector& localScale, const dgMatrix& matrix) const;
	dgFloat32 CalculateMassProperties (const dgMatrix& offset, dgVector& inertia, dgVector& crossInertia, dgVector& centerOfMass) const;

	
	
	virtual dgVector CalculateVolumeIntegral (const dgMatrix& globalMatrix, const dgVector& plane) const;

	virtual void DebugCollision (const dgMatrix& matrix, OnDebugCollisionMeshCallback callback, void* const userData) const;
	virtual dgFloat32 RayCast (const dgVector& localP0, const dgVector& localP1, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData) const;
	virtual dgFloat32 ConvexRayCast (const dgCollisionInstance* const convexShape, const dgMatrix& origin, const dgVector& veloc, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const referenceBody, const dgCollisionInstance* const referenceShape, void* const userData, dgInt32 threadId) const; 

	virtual dgInt32 CalculateSignature () const;
	virtual void SetCollisionBBox (const dgVector& p0, const dgVector& p1);
	virtual void GetCollisionInfo(dgCollisionInfo* const info) const;
	virtual void Serialize(dgSerialize callback, void* const userData) const;

	dgInt32 CalculateContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;
	dgInt32 CalculateContactsToSingle (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;
	dgInt32 CalculateContactsToSingleContinue (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;
	dgInt32 CalculateContactsToCompound (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;
	dgInt32 CalculateContactsToCollisionTree (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;
	dgInt32 CalculateContactsToHeightField (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;
	dgInt32 CalculateContactsUserDefinedCollision (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;

	dgInt32 ClosestDitance (dgBody* const bodyA, dgTriplex& contactA, dgBody* const bodyB, dgTriplex& contactB, dgTriplex& normalAB) const;
	dgInt32 ClosestDitanceToConvex (dgBody* const bodyA, dgTriplex& contactA, dgBody* const ConvetvBodyB, dgTriplex& contactB, dgTriplex& normalAB) const;
	dgInt32 ClosestDitanceToCompound (dgBody* const compoundBodyA, dgTriplex& contactA, dgBody* const compoundBodyB, dgTriplex& contactB, dgTriplex& normalAB) const;
	

#ifdef _DEBUG
	dgVector InternalSupportVertex (const dgVector& dir) const;
#endif
	
	protected:
	dgNodeBase* BuildTopDown (dgNodeBase** const leafArray, dgInt32 firstBox, dgInt32 lastBox, dgList<dgNodeBase*>::dgListNode** const nextNode);

	dgFloat64 CalculateEntropy (dgList<dgNodeBase*>& list);

	void CalculateCollisionTreeArea(dgNodePairs& pairOut, const dgCollisionBVH* const collisionTree, const void* const treeNode) const;
	void ImproveNodeFitness (dgNodeBase* const node) const;
	dgFloat32 CalculateSurfaceArea (dgNodeBase* const node0, dgNodeBase* const node1, dgVector& minBox, dgVector& maxBox) const;

	dgInt32 CalculatePlaneIntersection (const dgVector& normal, const dgVector& point, dgVector* const contactsOut) const;

	dgFloat32 m_boxMinRadius;
	dgFloat32 m_boxMaxRadius;
	dgFloat64 m_treeEntropy;
	dgWorld* m_world;	
	dgNodeBase* m_root;
	dgThread::dgCriticalSection m_criticalSectionLock;
	dgTree<dgNodeBase*, dgInt32> m_array;
	dgInt32 m_idIndex;

//	static dgVector m_padding;
	static dgVector m_padding;
	friend class dgBody;
	friend class dgWorld;
	friend class dgCollisionScene;
};

#endif 

