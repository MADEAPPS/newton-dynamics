/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __D_CONTACT_SOLVER_H__
#define __D_CONTACT_SOLVER_H__

#include "ndCollisionStdafx.h"
#include "ntShape.h"
#include "ntShapeInstance.h"

class dPlane;
class ntBodyKinematic;
//class ntContactPoint;

D_MSV_NEWTON_ALIGN_32
class ntMinkFace
{
	public:
	dPlane m_plane;
	ntMinkFace* m_twin[3];
	dInt16 m_vertex[3];
	dInt8 m_mark;
	dInt8 m_alive;
} D_GCC_NEWTON_ALIGN_32 ;

#define D_SEPARATION_PLANES_ITERATIONS	8
#define D_CONVEX_MINK_STACK_SIZE		64
#define D_CONNICS_CONTATS_ITERATIONS	32
#define D_CONVEX_MINK_MAX_FACES			512
#define D_CONVEX_MINK_MAX_POINTS		256
#define D_MAX_EDGE_COUNT				2048
#define D_PENETRATION_TOL				dFloat32 (1.0f / 1024.0f)
#define D_MINK_VERTEX_ERR				dFloat32 (1.0e-3f)
#define D_MINK_VERTEX_ERR2				(D_MINK_VERTEX_ERR * D_MINK_VERTEX_ERR)

class ntContact;
class dCollisionParamProxy;

//class ntPair
//{
//	public:
//	ntContact* m_contact;
//	ntContactPoint* m_contactBuffer;
//	dFloat32 m_timestep;
//	dInt32 m_contactCount;
//	//dInt32 m_cacheIsValid : 1;
//	//dInt32 m_flipContacts : 1;
//};

D_MSV_NEWTON_ALIGN_32
class ntContactSolver: public dDownHeap<ntMinkFace *, dFloat32>  
{
	public: 
	ntContactSolver(const ntShapeInstance& instance0, const ntShapeInstance& instance1);
	//ntContactSolver(dCollisionParamProxy* const proxy);
	//ntContactSolver(ntShapeInstance* const instance0);

	
	//dInt32 CalculateConvexCastContacts();
	
	//dFloat32 RayCast (const dVector& localP0, const dVector& localP1, dFloat32 maxT, ntContactPoint& contactOut);

	//const dVector& GetNormal() const {return m_normal;}
	//const dVector& GetPoint0() const {return m_closestPoint0;}
	//const dVector& GetPoint1() const {return m_closestPoint1;}

	//void CalculateContacts(ntPair* const pair, dInt32 threadIndex, bool ccdMode, bool intersectionTestOnly);

	dInt32 ConvexContacts();
	dInt32 CalculateConvexToConvexContacts();
	dInt32 CalculatePairContacts(dInt32 threadIndex, ntContactPoint* const buffer);
	
	private:
	class dgPerimenterEdge
	{
		public:
		const dVector* m_vertex;
		dgPerimenterEdge* m_next;
		dgPerimenterEdge* m_prev;
	};

	class dgFaceFreeList
	{
		public:
		dgFaceFreeList* m_next;
	};

	//D_INLINE ntMinkFace* NewFace();
	//D_INLINE void PushFace(ntMinkFace* const face);
	//D_INLINE void DeleteFace(ntMinkFace* const face);
	//D_INLINE ntMinkFace* AddFace(dInt32 v0, dInt32 v1, dInt32 v2);
	
	//
	//D_INLINE void TranslateSimplex(const dVector& step);
	//
	//D_INLINE void CalculateContactFromFeacture(dInt32 featureType);
	//D_INLINE dgPerimenterEdge* OldReduceContacts(dgPerimenterEdge* poly, dInt32 maxCount) const;
	//
	//bool SanityCheck() const;
	//dInt32 ConvexPolygonsIntersection(const dVector& normal, dInt32 count1, dVector* const shape1, dInt32 count2, dVector* const shape2, dVector* const contactOut, dInt32 maxContacts) const;
	//dInt32 ConvexPolygonToLineIntersection(const dVector& normal, dInt32 count1, dVector* const shape1, dInt32 count2, dVector* const shape2, dVector* const contactOut, dVector* const mem) const;
	//dInt32 CalculateContacts (const dVector& point0, const dVector& point1, const dVector& normal);
	//dInt32 CalculateIntersectingPlane(dInt32 count);

	dInt32 CalculateClosestSimplex();
	bool CalculateClosestPoints();
	dInt32 ConvexToConvexContacts();

	D_INLINE dBigVector ReduceLine(dInt32& indexOut);
	D_INLINE dBigVector ReduceTriangle (dInt32& indexOut);
	D_INLINE dBigVector ReduceTetrahedrum (dInt32& indexOut);
	D_INLINE void SupportVertex(const dVector& dir, dInt32 vertexIndex);

	//dVector m_normal;
	//dVector m_closestPoint0;
	//dVector m_closestPoint1;
	//dCollisionParamProxy* m_proxy;
	//ntBodyKinematic* const body0;
	//ntBodyKinematic* const body1;
	ntShapeInstance m_instance0;
	ntShapeInstance m_instance1;
	dVector m_separatingVector;

	dVector m_hullDiff[D_CONVEX_MINK_MAX_POINTS];
	dVector m_hullSum[D_CONVEX_MINK_MAX_POINTS];

	//ntContact* m_contact;
	//dgFaceFreeList* m_freeFace; 

	dFloat32 m_timestep;
	dFloat32 m_closestDistance;
	dFloat32 m_separationDistance;

	dInt32 m_maxCount;
	dInt32 m_vertexIndex;
	bool m_ccdMode;
	bool m_intersectionTestOnly;
	
	//dInt32 m_faceIndex;
	//ntMinkFace* m_faceStack[D_CONVEX_MINK_STACK_SIZE];
	//ntMinkFace* m_coneFaceList[D_CONVEX_MINK_STACK_SIZE];
	//ntMinkFace* m_deletedFaceList[D_CONVEX_MINK_STACK_SIZE];
	//ntMinkFace m_facePool[D_CONVEX_MINK_MAX_FACES];
	dInt8 m_heapBuffer[D_CONVEX_MINK_MAX_FACES * (sizeof (dFloat32) + sizeof (ntMinkFace *))];

	static dVector m_hullDirs[14]; 
	static dInt32 m_rayCastSimplex[4][4];

	friend class ntBroadPhase;
} D_GCC_NEWTON_ALIGN_32 ;

#endif 


