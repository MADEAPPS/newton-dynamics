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

#ifndef __ND_CONTACT_SOLVER_H__
#define __ND_CONTACT_SOLVER_H__

#include "ndCollisionStdafx.h"
#include "ndShape.h"
#include "ndShapeInstance.h"

class ndPlane;
class ndBodyKinematic;
class ndContactNotify;
class ndPolygonMeshDesc;

D_MSV_NEWTON_ALIGN_32
class ndMinkFace
{
	public:
	ndPlane m_plane;
	ndMinkFace* m_twin[3];
	ndInt16 m_vertex[3];
	ndInt8 m_mark;
	ndInt8 m_alive;
} D_GCC_NEWTON_ALIGN_32 ;

#define D_SEPARATION_PLANES_ITERATIONS	8
#define D_CONVEX_MINK_STACK_SIZE		64
#define D_CONNICS_CONTATS_ITERATIONS	32
#define D_CONVEX_MINK_MAX_FACES			512
#define D_CONVEX_MINK_MAX_POINTS		256
#define D_MAX_EDGE_COUNT				2048
#define D_PENETRATION_TOL				ndFloat32 (1.0f / 1024.0f)
#define D_MINK_VERTEX_ERR				ndFloat32 (1.0e-3f)
#define D_MINK_VERTEX_ERR2				(D_MINK_VERTEX_ERR * D_MINK_VERTEX_ERR)

class ndContact;
class dCollisionParamProxy;

D_MSV_NEWTON_ALIGN_32
class ndContactSolver: public ndDownHeap<ndMinkFace *, ndFloat32>  
{
	public: 
	class ndBoxBoxDistance2;

	D_COLLISION_API ndContactSolver();
	~ndContactSolver() {}

	D_COLLISION_API void CalculateContacts(
		const ndShapeInstance* const shapeA, const ndMatrix& matrixA, const ndVector& velocA,
		const ndShapeInstance* const shapeB, const ndMatrix& matrixB, const ndVector& velocB,
		ndFixSizeArray<ndContactPoint, 16>& contactOut);

	private:
	ndContactSolver(ndContact* const contact, ndContactNotify* const notification, ndFloat32 timestep);
	ndContactSolver(ndShapeInstance* const instance, ndContactNotify* const notification, ndFloat32 timestep);
	ndContactSolver(const ndContactSolver& src, const ndShapeInstance& instance0, const ndShapeInstance& instance1);

	ndInt32 CalculateContactsDiscrete(); // done
	ndInt32 CalculateContactsContinue(); // done
	ndFloat32 RayCast (const ndVector& localP0, const ndVector& localP1, ndContactPoint& contactOut);
	
	ndInt32 ConvexContactsDiscrete(); // done
	ndInt32 CompoundContactsDiscrete(); // done
	ndInt32 ConvexToConvexContactsDiscrete(); // done
	ndInt32 ConvexToCompoundContactsDiscrete(); // done
	ndInt32 CompoundToConvexContactsDiscrete(); // done
	ndInt32 CompoundToCompoundContactsDiscrete(); // done
	ndInt32 ConvexToStaticMeshContactsDiscrete(); // done
	ndInt32 CompoundToShapeStaticBvhContactsDiscrete(); // done
	ndInt32 CompoundToStaticHeightfieldContactsDiscrete(); // done
	ndInt32 CalculatePolySoupToHullContactsDescrete(ndPolygonMeshDesc& data); // done
	ndInt32 ConvexToSaticStaticBvhContactsNodeDescrete(const ndAabbPolygonSoup::ndNode* const node); // done

	ndInt32 ConvexContactsContinue(); // done
	ndInt32 CompoundContactsContinue(); // done
	ndInt32 ConvexToConvexContactsContinue(); // done
	ndInt32 ConvexToCompoundContactsContinue(); // done
	ndInt32 ConvexToStaticMeshContactsContinue(); // done
	ndInt32 CalculatePolySoupToHullContactsContinue(ndPolygonMeshDesc& data); // done

	class dgPerimenterEdge
	{
		public:
		const ndVector* m_vertex;
		dgPerimenterEdge* m_next;
		dgPerimenterEdge* m_prev;
	};

	class dgFaceFreeList
	{
		public:
		dgFaceFreeList* m_next;
	};

	inline ndMinkFace* NewFace();
	inline void PushFace(ndMinkFace* const face);
	inline void DeleteFace(ndMinkFace* const face);
	inline ndMinkFace* AddFace(ndInt32 v0, ndInt32 v1, ndInt32 v2);

	bool CalculateClosestPoints();
	ndInt32 CalculateClosestSimplex();
	
	ndInt32 CalculateIntersectingPlane(ndInt32 count);
	ndInt32 PruneContacts(ndInt32 count, ndInt32 maxCount) const;
	ndInt32 PruneSupport(ndInt32 count, const ndVector& dir, const ndVector* const points) const;
	ndInt32 CalculateContacts(const ndVector& point0, const ndVector& point1, const ndVector& normal);
	ndInt32 Prune2dContacts(const ndMatrix& matrix, ndInt32 count, ndContactPoint* const contactArray, ndInt32 maxCount) const;
	ndInt32 Prune3dContacts(const ndMatrix& matrix, ndInt32 count, ndContactPoint* const contactArray, ndInt32 maxCount) const;
	ndInt32 ConvexPolygonsIntersection(const ndVector& normal, ndInt32 count1, ndVector* const shape1, ndInt32 count2, ndVector* const shape2, ndVector* const contactOut, ndInt32 maxContacts) const;
	ndInt32 ConvexPolygonToLineIntersection(const ndVector& normal, ndInt32 count1, ndVector* const shape1, ndInt32 count2, ndVector* const shape2, ndVector* const contactOut, ndVector* const mem) const;

	ndBigVector ReduceLine(ndInt32& indexOut);
	ndBigVector ReduceTriangle (ndInt32& indexOut);
	ndBigVector ReduceTetrahedrum (ndInt32& indexOut);
	void SupportVertex(const ndVector& dir, ndInt32 vertexIndex);

	void TranslateSimplex(const ndVector& step);
	void CalculateContactFromFeacture(ndInt32 featureType);

	ndShapeInstance m_instance0;
	ndShapeInstance m_instance1;
	ndVector m_closestPoint0;
	ndVector m_closestPoint1;
	ndVector m_separatingVector;
	union
	{
		ndVector m_buffer[2 * D_CONVEX_MINK_MAX_POINTS];
		struct
		{
			ndVector m_hullDiff[D_CONVEX_MINK_MAX_POINTS];
			ndVector m_hullSum[D_CONVEX_MINK_MAX_POINTS];
		};
	};

	ndContact* m_contact;
	dgFaceFreeList* m_freeFace;
	ndContactNotify* m_notification;
	ndContactPoint* m_contactBuffer;
	ndFloat32 m_timestep;
	ndFloat32 m_skinThickness;
	ndFloat32 m_separationDistance;

	ndInt32 m_maxCount;
	ndInt32 m_vertexIndex;
	ndUnsigned32 m_pruneContacts			: 1;
	ndUnsigned32 m_intersectionTestOnly	: 1;
	
	ndInt32 m_faceIndex;
	ndMinkFace* m_faceStack[D_CONVEX_MINK_STACK_SIZE];
	ndMinkFace* m_coneFaceList[D_CONVEX_MINK_STACK_SIZE];
	ndMinkFace* m_deletedFaceList[D_CONVEX_MINK_STACK_SIZE];
	ndMinkFace m_facePool[D_CONVEX_MINK_MAX_FACES];
	ndInt8 m_heapBuffer[D_CONVEX_MINK_MAX_FACES * (sizeof (ndFloat32) + sizeof (ndMinkFace *))];

	static ndVector m_pruneUpDir;
	static ndVector m_pruneSupportX;

	static ndVector m_hullDirs[14]; 
	static ndInt32 m_rayCastSimplex[4][4];

	friend class ndScene;
	friend class ndShapeConvex;
	friend class ndShapeInstance;
	friend class ndPolygonMeshDesc;
	friend class ndConvexCastNotify;
	friend class ndShapeConvexPolygon;
	friend class ndBodyPlayerCapsuleContactSolver;
} D_GCC_NEWTON_ALIGN_32;

#endif 


