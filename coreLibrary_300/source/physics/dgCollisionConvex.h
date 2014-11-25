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

#ifndef _DG_CONVEX_COLLISION_H__
#define _DG_CONVEX_COLLISION_H__

#include "dgCollision.h"

#define DG_CLIP_MAX_COUNT				512
#define DG_CLIP_MAX_POINT_COUNT			64


class dgConvexSimplexEdge
{
	public:
	dgConvexSimplexEdge* m_twin;
	dgConvexSimplexEdge* m_next;
	dgConvexSimplexEdge* m_prev;
	dgInt32 m_vertex;
};


DG_MSC_VECTOR_ALIGMENT
class dgCollisionConvex: public dgCollision
{
	class dgMinkFace;
	class dgMinkHull;
	class dgPerimenterEdge;
	class dgCollisionPriority
	{
		public:
		dgCollisionPriority()
		{
			// set all entry to be no swap order
			memset (m_swapPriority, false, sizeof (m_swapPriority));
			for (dgInt32 i = 0; i < m_nullCollision; i ++) {
				m_swapPriority[i][m_sphereCollision] = true;
				m_swapPriority[i][m_capsuleCollision] = true;
				m_swapPriority[i][m_chamferCylinderCollision] = true;
			}
			for (dgInt32 i = m_sphereCollision; i <= m_chamferCylinderCollision; i ++) {
				for (dgInt32 j = m_sphereCollision; j <= m_chamferCylinderCollision; j ++) {
					m_swapPriority[i][j] = false;
				}
			}
		}
		bool m_swapPriority[m_nullCollision][m_nullCollision];
	};

	public:
	virtual void CalcAABB (const dgMatrix& matrix, dgVector& p0, dgVector& p1) const;
	
	virtual dgFloat32 RayCast (const dgVector& localP0, const dgVector& localP1, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const;
	virtual dgFloat32 ConvexRayCast (const dgCollisionInstance* const convexShape, const dgMatrix& origin, const dgVector& veloc, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const referenceBody, const dgCollisionInstance* const referenceInstance, void* const userData, dgInt32 threadId) const; 

	virtual dgVector SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const;

	virtual dgInt32 CalculatePlaneIntersection (const dgVector& normal, const dgVector& point, dgVector* const contactsOut, dgFloat32 normalSign) const;
	virtual dgInt32 GetConvexVertexCount() const { return m_vertexCount;}

	bool IntesectionTest (dgCollisionParamProxy& proxy) const;
	bool CalculateClosestPoints (dgCollisionParamProxy& proxy) const;

	dgInt32 CalculateConvexCastContacts (dgCollisionParamProxy& proxy) const;
	dgInt32 CalculateConvexToConvexContact (dgCollisionParamProxy& proxy) const;
	//dgInt32 ConvexCastContacts (const dgMatrix& matrix, const dgMatrix& invMatrix, const dgVector& veloc, dgFloat32& timeStep, const dgCollisionConvex* const convexShape, dgContactPoint* const contact) const;

	protected:
	dgCollisionConvex (dgMemoryAllocator* const allocator, dgUnsigned32 signature, dgCollisionID id);
	dgCollisionConvex (dgWorld* const world, dgDeserialize deserialization, void* const userData);
	~dgCollisionConvex ();

	virtual void SerializeLow(dgSerialize callback, void* const userData) const;

	virtual dgVector CalculateVolumeIntegral (const dgMatrix& globalMatrix, const dgVector& plane, const dgCollisionInstance& parentScale) const;
	static void CalculateInertia (void *userData, int vertexCount, const dgFloat32* const FaceArray, int faceId);

	virtual dgFloat32 GetVolume () const;

	virtual dgFloat32 GetBoxMinRadius () const; 
	virtual dgFloat32 GetBoxMaxRadius () const;

	virtual void* GetUserData () const;
	virtual void SetUserData (void* const userData);

	dgInt32 RayCastClosestFace (dgVector* tetrahedrum, const dgVector& origin, dgFloat32& pointDist) const;
	dgInt32 SimplifyClipPolygon (dgInt32 count, const dgVector& normal, dgVector* const polygon) const;

	dgVector CalculateVolumeIntegral (const dgPlane& plane) const; 
	
	void SetVolumeAndCG ();
	bool SanityCheck (dgPolyhedra& hull) const;
	virtual void DebugCollision  (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const;

	virtual void MassProperties ();
	virtual dgMatrix CalculateInertiaAndCenterOfMass (const dgMatrix& m_alignMatrix, const dgVector& localScale, const dgMatrix& matrix) const;
	virtual dgFloat32 CalculateMassProperties (const dgMatrix& offset, dgVector& inertia, dgVector& crossInertia, dgVector& centerOfMass) const;

	bool SanityCheck(dgInt32 count, const dgVector& normal, dgVector* const contactsOut) const;

	dgPerimenterEdge* ReduceContacts (dgPerimenterEdge* poly, dgInt32 maxCount) const;
	dgInt32 RectifyConvexSlice (dgInt32 count, const dgVector& normal, dgVector* const contactsOut) const;

	// special feature based contact calculation for conics convex (ex spheres, capsules, tapered capsules, and chamfered cylinders)
	// in newton we only deal with sub set of conic function, that can be expressed by the equation
	// ((x - x0) / a)^2 + ((y - y0) / b)^2 + ((z - z0) / c)^2  = 1   and possible a linear or circular sweep of the same equation
    // this preclude parabolic and hyperbolic conics 
	virtual dgVector ConvexConicSupporVertex (const dgVector& dir) const;
	virtual dgVector ConvexConicSupporVertex (const dgVector& point, const dgVector& dir) const;
	virtual dgInt32 CalculateContacts (const dgVector& point, const dgVector& normal, dgCollisionParamProxy& proxy, dgVector* const contactsOut) const;

	dgInt32 CalculateContactsGeneric (const dgVector& point, const dgVector& normal, dgCollisionParamProxy& proxy, dgVector* const contactsOut) const;
	dgInt32 ConvexPolygonsIntersection (const dgVector& nornal, dgInt32 count1, dgVector* const shape1, dgInt32 count2, dgVector* const shape2, dgVector* const contactOut, dgInt32 maxContacts) const;
	dgInt32 ConvexPolygonToLineIntersection (const dgVector& normal, dgInt32 count1, dgVector* const shape1, dgInt32 count2, dgVector* const shape2, dgVector* const contactOut, dgVector* const mem) const;
	dgFloat32 ConvexConicConvexRayCast (const dgCollisionInstance* const convexConicShape, const dgMatrix& conicShapeMatrix, const dgCollisionInstance* const convexCastingShape, const dgMatrix& castingMatrix, const dgVector& castingVeloc, dgFloat32 maxT, dgContactPoint& contactOut) const;

	virtual const dgConvexSimplexEdge** GetVertexToEdgeMapping() const {return NULL;}
									
//	dgVector ReduceLine (dgInt32& indexOut, dgVector* const lineDiff, dgVector* const lineSum) const;
//	dgVector ReduceTriangle (dgInt32& indexOut, dgVector* const triangleDiff, dgVector* const triangleSum) const;
//	dgVector ReduceTetrahedrum (dgInt32& indexOut, dgVector* const tetraDiff, dgVector* const tetraSum) const;
//	void ReduceDegeneratedTriangle (dgVector& diff0, dgVector& sum0, dgVector& diff1, dgVector& sum1, const dgVector& diff2, const dgVector& sum2) const;
//	dgBigVector ReduceLineLarge (dgInt32& indexOut, dgBigVector* const lineDiff, dgBigVector* const lineSum) const;
//	dgBigVector ReduceTriangleLarge (dgInt32& indexOut, dgBigVector* const triangleDiff, dgBigVector* const triangleSum) const;
//	dgBigVector ReduceTetrahedrumLarge (dgInt32& indexOut, dgBigVector* const tetraDiff, dgBigVector* const tetraSum) const;
//	void ReduceDegeneratedTriangleLarge (dgBigVector& diff0, dgBigVector& sum0, dgBigVector& diff1, dgBigVector& sum1, const dgBigVector& diff2, const dgBigVector& sum2) const;
//	void ConvexSupportVertex (const dgVector& dir, const dgVector& scale1, const dgMatrix& matrix, const dgVector& invScale0, const dgCollisionConvex* const convexShape, dgVector& diff, dgVector& sum) const;
//	void ConvexConicSupportVertex (const dgVector& dir, const dgVector& scale1, const dgMatrix& matrix, const dgVector& invScale0, const dgCollisionConvex* const convexShape, dgVector& diff, dgVector& sum) const;
//	void ConvexConicSupportVertexLarge (const dgVector& dir, const dgVector& scale1, const dgMatrix& matrix, const dgVector& invScale0, const dgCollisionConvex* const convexShape, dgBigVector& diff, dgBigVector& sum) const;
//	dgInt32 CalculateConicClosestSimplex (const dgCollisionConvex* const convexShape, const dgVector& scale1, const dgMatrix& matrix, const dgVector& invScale0, const dgVector& separatingVectorHint, dgVector* const diff, dgVector* const sum, dgVector& normal) const;
//	dgInt32 CalculateConicClosestSimplexLarge (const dgCollisionConvex* const convexShape, const dgVector& scale1, const dgMatrix& matrix, const dgVector& invScale0, const dgVector& separatingVectorHint, dgVector* const diff, dgVector* const sum, dgVector& normal) const;
//	dgInt32 CalculateIntersectingPlane (const dgCollisionConvex* const convexShape, const dgVector& scale1, const dgMatrix& matrix, const dgVector& invScale0, const dgVector& separatingVectorHint, dgInt32 count, dgVector* const diff, dgVector* const sum, dgVector& normal) const;
	
	void* m_userData;
	dgVector* m_vertex;
	dgConvexSimplexEdge* m_simplex;
	
	
	dgFloat32 m_boxMinRadius;
	dgFloat32 m_boxMaxRadius;
	dgFloat32 m_simplexVolume;
	
	dgUnsigned16 m_edgeCount;
	dgUnsigned16 m_vertexCount;
	
	public:	
	static dgVector m_hullDirs[14]; 
	static dgVector m_dummySum[4];
	static dgInt32 m_rayCastSimplex[4][4];
	static dgCollisionPriority m_priorityOrder;
	
	friend class dgWorld;
	friend class dgMinkowskiConv;
	friend class dgCollisionCompound;
	friend class dgBroadPhase;
	friend class dgCollisionConvexModifier;
} DG_GCC_VECTOR_ALIGMENT;



#endif //AFX_DGCONVEXCOLLISION_H__57E159CE_6B6F_42DE_891C_1F6C38EB9D29_H


