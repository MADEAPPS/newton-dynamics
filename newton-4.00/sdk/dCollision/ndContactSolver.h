/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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
#include "ndShape.h"
#include "ndShapeInstance.h"

class dPlane;
class ndBodyKinematic;
class ndPolygonMeshDesc;

D_MSV_NEWTON_ALIGN_32
class ndMinkFace
{
	public:
	dPlane m_plane;
	ndMinkFace* m_twin[3];
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

class ndContact;
class dCollisionParamProxy;

D_MSV_NEWTON_ALIGN_32
class ndContactSolver: public dDownHeap<ndMinkFace *, dFloat32>  
{
	public: 
	ndContactSolver(ndContact* const contact, ndScene* const scene);
	ndContactSolver(ndShapeInstance* const instance, ndScene* const scene);
	ndContactSolver(const ndContactSolver& src, const ndShapeInstance& instance0, const ndShapeInstance& instance1);

	//dInt32 CalculateConvexCastContacts();
	//const dVector& GetNormal() const {return m_normal;}
	//const dVector& GetPoint0() const {return m_closestPoint0;}
	//const dVector& GetPoint1() const {return m_closestPoint1;}
	//void CalculateContacts(ntPair* const pair, dInt32 threadIndex, bool ccdMode, bool intersectionTestOnly);

	dInt32 ConvexContacts();
	dInt32 CompoundContacts();
	dInt32 CalculatePairContacts();
	dInt32 CalculateConvexToConvexContacts();
	dInt32 CalculateConvexToCompoundContacts();
	dInt32 CalculateCompoundToConvexContacts();
	dInt32 CalculateConvexToSaticMeshContacts();
	dInt32 CalculateCompoundToCompoundContacts();
	dInt32 CalculateCompoundToShapeStaticBvhContacts();
	dInt32 CalculateConvexToSaticStaticBvhContactsNode(const dAabbPolygonSoup::dNode* const node);

	dFloat32 RayCast (const dVector& localP0, const dVector& localP1, ndContactPoint& contactOut);
	
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

	D_INLINE ndMinkFace* NewFace();
	D_INLINE void PushFace(ndMinkFace* const face);
	D_INLINE void DeleteFace(ndMinkFace* const face);
	D_INLINE ndMinkFace* AddFace(dInt32 v0, dInt32 v1, dInt32 v2);

	dInt32 CalculateClosestSimplex();
	bool CalculateClosestPoints();
	dInt32 ConvexToConvexContacts();
	dInt32 ConvexToStaticMeshContacts();

	dInt32 CalculateIntersectingPlane(dInt32 count);
	dInt32 PruneContacts(dInt32 count, dInt32 maxCount) const;
	dInt32 PruneSupport(dInt32 count, const dVector& dir, const dVector* const points) const;
	dInt32 CalculateContacts(const dVector& point0, const dVector& point1, const dVector& normal);
	dInt32 Prune2dContacts(const dMatrix& matrix, dInt32 count, ndContactPoint* const contactArray, dInt32 maxCount) const;
	dInt32 Prune3dContacts(const dMatrix& matrix, dInt32 count, ndContactPoint* const contactArray, dInt32 maxCount) const;
	dInt32 ConvexPolygonsIntersection(const dVector& normal, dInt32 count1, dVector* const shape1, dInt32 count2, dVector* const shape2, dVector* const contactOut, dInt32 maxContacts) const;
	dInt32 ConvexPolygonToLineIntersection(const dVector& normal, dInt32 count1, dVector* const shape1, dInt32 count2, dVector* const shape2, dVector* const contactOut, dVector* const mem) const;

	dInt32 CalculatePolySoupToHullContactsDescrete(ndPolygonMeshDesc& data);

	D_INLINE dBigVector ReduceLine(dInt32& indexOut);
	D_INLINE dBigVector ReduceTriangle (dInt32& indexOut);
	D_INLINE dBigVector ReduceTetrahedrum (dInt32& indexOut);
	D_INLINE void SupportVertex(const dVector& dir, dInt32 vertexIndex);

	D_INLINE void CalculateContactFromFeacture(dInt32 featureType);

	ndShapeInstance m_instance0;
	ndShapeInstance m_instance1;
	dVector m_closestPoint0;
	dVector m_closestPoint1;
	dVector m_separatingVector;
	union
	{
		dVector m_buffer[2 * D_CONVEX_MINK_MAX_POINTS];
		struct
		{
			dVector m_hullDiff[D_CONVEX_MINK_MAX_POINTS];
			dVector m_hullSum[D_CONVEX_MINK_MAX_POINTS];
		};
	};

	ndScene* m_scene;
	ndContact* m_contact;
	
	dgFaceFreeList* m_freeFace; 

	ndContactPoint* m_contactBuffer;
	dFloat32 m_timestep;
	dFloat32 m_separationDistance;
	dFloat32 m_skinThickness;

	dInt32 m_maxCount;
	dInt32 m_vertexIndex;
	dUnsigned32 m_ccdMode				: 1;
	dUnsigned32 m_pruneContacts			: 1;
	dUnsigned32 m_intersectionTestOnly	: 1;
	
	dInt32 m_faceIndex;
	ndMinkFace* m_faceStack[D_CONVEX_MINK_STACK_SIZE];
	ndMinkFace* m_coneFaceList[D_CONVEX_MINK_STACK_SIZE];
	ndMinkFace* m_deletedFaceList[D_CONVEX_MINK_STACK_SIZE];
	ndMinkFace m_facePool[D_CONVEX_MINK_MAX_FACES];
	dInt8 m_heapBuffer[D_CONVEX_MINK_MAX_FACES * (sizeof (dFloat32) + sizeof (ndMinkFace *))];

	static dVector m_pruneUpDir;
	static dVector m_pruneSupportX;

	static dVector m_hullDirs[14]; 
	static dInt32 m_rayCastSimplex[4][4];

	friend class ndScene;
	friend class ndShapeInstance;
	friend class ndPolygonMeshDesc;
	friend class ndShapeConvexPolygon;
	friend class ndBodyPlayerCapsuleContactSolver;
} D_GCC_NEWTON_ALIGN_32 ;

#endif 


