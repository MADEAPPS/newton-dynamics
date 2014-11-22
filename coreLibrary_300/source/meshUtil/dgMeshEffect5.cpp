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

#include "dgPhysicsStdafx.h"
#include "dgBody.h"
#include "dgWorld.h"
#include "dgMeshEffect.h"
#include "dgCollisionConvexHull.h"


#define DG_BOOLEAN_ZERO_TOLERANCE	(1.0e-30)

#if 0
#define DG_BOOLEAN_COMBINE_MARK		(1 << 8)
#define DG_REMAP_VERTEX_MARK		(1 << 9)
#define DG_INTERIOR_FACE_MARK		(1 << 10)
#define DG_LOOP_BORDER_MARK			(1 << 11)
class dgBooleanMeshClipper: public dgMeshEffect::dgMeshBVH
{
	public:
	class dgFaceVertexPair
	{
		public:
		dgFaceVertexPair (dgEdge* const vertex, dgMeshBVHNode* const faceNode)
			:m_vertex (vertex), m_faceNode(faceNode)
		{
		}
		dgEdge* m_vertex;	
		dgMeshBVHNode* m_faceNode;
	};

	dgBooleanMeshClipper (dgMeshEffect* const mesh) 
		:dgMeshBVH (mesh)
		,m_vertexBase(mesh->GetVertexCount())
		,m_vertexMap(mesh->GetAllocator())
		,m_nodeEdgeMap(mesh->GetAllocator())
		,m_vertexAlias(mesh->GetVertexCount() + 512, mesh->GetAllocator())
	{
		const dgBigVector* const points = (dgBigVector*) m_mesh->GetVertexPool(); 
		for (dgInt32 i = 0; i < m_mesh->GetVertexCount(); i ++) {
			m_vertexAlias[i] = dgHugeVector (points[i]);
		}

		
		dgMeshBVHNode* stackPool[DG_MESH_EFFECT_BVH_STACK_DEPTH];
		dgInt32 stack = 1;
		stackPool[0] = m_rootNode;
		dgVector l0(dgFloat32 (-1.0e15f), dgFloat32 (-1.0e15f), dgFloat32 (-1.0e15f), dgFloat32 (0.0f)); 
		dgVector l1(dgFloat32 (1.0e15f), dgFloat32 ( 1.0e15f), dgFloat32 ( 1.0e15f), dgFloat32 (0.0f)); 

		while (stack) {
			stack --;
			dgMeshBVHNode* const me = stackPool[stack];
			if (me && dgOverlapTest (me->m_p0, me->m_p1, l0, l1)) {
				if (!me->m_left) {
					dgAssert (!me->m_right);
					dgPolyhedra::dgPairKey key (me->m_face->m_incidentVertex, me->m_face->m_twin->m_incidentVertex);
					m_nodeEdgeMap.Insert(me, key.GetVal());
				} else {
					dgAssert (me->m_left);
					dgAssert (me->m_right);
					stackPool[stack] = me->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack] = me->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
				}
			}
		}

		dgAssert (SanityCheck());
	}

	~dgBooleanMeshClipper () 
	{
	}

	bool SanityCheck() const
	{
		#ifdef _DEBUG
			dgAssert (dgMeshBVH::SanityCheck()); 
			dgMeshBVHNode* stackPool[DG_MESH_EFFECT_BVH_STACK_DEPTH];
			dgInt32 stack = 1;
			stackPool[0] = m_rootNode;
			dgVector l0(dgFloat32 (-1.0e15f), dgFloat32 (-1.0e15f), dgFloat32 (-1.0e15f), dgFloat32 (0.0f)); 
			dgVector l1(dgFloat32 (1.0e15f), dgFloat32 ( 1.0e15f), dgFloat32 ( 1.0e15f), dgFloat32 (0.0f)); 

			while (stack) {
				stack --;
				dgMeshBVHNode* const me = stackPool[stack];
				if (me && dgOverlapTest (me->m_p0, me->m_p1, l0, l1)) {
					if (!me->m_left) {
						dgAssert (!me->m_right);
						dgPolyhedra::dgPairKey key (me->m_face->m_incidentVertex, me->m_face->m_twin->m_incidentVertex);
						dgTree<dgMeshBVHNode*, dgUnsigned64>::dgTreeNode* const node = m_nodeEdgeMap.Find(key.GetVal());
						dgAssert (node);
						dgAssert (node->GetInfo() == me);
					} else {
						dgAssert (me->m_left);
						dgAssert (me->m_right);
						stackPool[stack] = me->m_left;
						stack++;
						dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

						stackPool[stack] = me->m_right;
						stack++;
						dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
					}
				}
			}
		#endif
		return true;
	}

	dgHugeVector FaceNormal (const dgEdge* const face) const
	{
		const dgEdge* edge = face;
		const dgHugeVector& p0 = m_vertexAlias[edge->m_incidentVertex];

		edge = edge->m_next;
		const dgHugeVector& p1 = m_vertexAlias[edge->m_incidentVertex];
		dgHugeVector e1 (p1 - p0);

		dgHugeVector normal (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		for (edge = edge->m_next; edge != face; edge = edge->m_next) {
			const dgHugeVector& p2 = m_vertexAlias[edge->m_incidentVertex]; 
			dgHugeVector e2 (p2 - p0);
			normal += e1 * e2;
			e1 = e2;
		} 
		return normal;
	}

	
	bool RayVertexIntersectQuickTest (dgEdge* const edge, const dgBooleanMeshClipper* const otherMeshBVH, dgEdge* const otherVertex) const 
	{
		dgTree<dgInt32,dgInt32>::dgTreeNode* const node = m_vertexMap.Find(otherVertex->m_incidentVertex);
		dgInt32 v = node ? node->GetInfo() : -1;
		if ((v == edge->m_incidentVertex) || (v == edge->m_twin->m_incidentVertex)) {
			return false;
		}

		dgBigVector q0 (otherMeshBVH->m_mesh->m_points[otherVertex->m_incidentVertex]);
		dgBigVector ray_p0 (m_mesh->m_points[edge->m_incidentVertex]);
		dgBigVector ray_p1 (m_mesh->m_points[edge->m_twin->m_incidentVertex]);

		dgBigVector p1p0 (ray_p1 - ray_p0);
		dgBigVector q0p0 (q0 - ray_p0);

		dgFloat64 tol = dgFloat64(1.0e-5f);
		dgFloat64 den = p1p0 % p1p0;        // always >= 0
		if (den < tol * tol) { 
			return ((q0p0 % q0p0) < (1.0e-6f));
		}

		dgFloat64 num = q0p0 % p1p0;        
		dgFloat64 tolDen = -tol * den;
		dgFloat64 tolDen1 = den + tol;
		if ((num < tolDen) || (num > tolDen1)) {
			return false;
		}

		dgBigVector r (ray_p0 + p1p0.Scale3 (num / den));
		dgBigVector q0r (q0 - r);
		dgFloat64 dist2 (q0r % q0r);
		return dist2 <= (tol * tol);
	}


	bool RayRayIntersectQuickTest (dgEdge* const edge, const dgBooleanMeshClipper* const otherMeshBVH, dgEdge* const otherEdge) const
	{
		dgTree<dgInt32,dgInt32>::dgTreeNode* const node0 = otherMeshBVH->m_vertexMap.Find(otherEdge->m_incidentVertex);
		dgTree<dgInt32,dgInt32>::dgTreeNode* const node1 = otherMeshBVH->m_vertexMap.Find(otherEdge->m_twin->m_incidentVertex);
		dgInt32 v0 = node0 ? node0->GetInfo() : -1;
		dgInt32 v1 = node1 ? node1->GetInfo() : -1;

		if ((v0 == edge->m_incidentVertex) || (v0 == edge->m_twin->m_incidentVertex) || (v1 == edge->m_incidentVertex) || (v1 == edge->m_twin->m_incidentVertex)) {
			return false;
		}

		dgBigVector ray_p0 (m_mesh->m_points[edge->m_incidentVertex]);
		dgBigVector ray_p1 (m_mesh->m_points[edge->m_twin->m_incidentVertex]);

		dgBigVector ray_q0 (otherMeshBVH->m_mesh->m_points[otherEdge->m_incidentVertex]);
		dgBigVector ray_q1 (otherMeshBVH->m_mesh->m_points[otherEdge->m_twin->m_incidentVertex]);

		dgBigVector p1p0 (ray_p1 - ray_p0);
		dgBigVector q1q0 (ray_q1 - ray_q0);
		dgBigVector p0q0 (ray_p0 - ray_q0);

		dgFloat64 tol = dgFloat64(1.0e-5f);

		dgFloat64 a = p1p0 % p1p0;        // always >= 0
		if ( a < tol * tol) {
			return true;
		}

		dgFloat64 c = q1q0 % q1q0;        // always >= 0
		if ( c < tol * tol) {
			return true;
		}
		dgFloat64 b = p1p0 % q1q0;

		dgFloat64 d = (p1p0 % p0q0);
		dgFloat64 e = (q1q0 % p0q0);
		dgFloat64 den = a * c - b * b;   // always >= 0
		

		// compute the line parameters of the two closest points
		if (den < tol) { 
			// almost parallel
			dgBigVector r = ray_q0 + q1q0.Scale3 (e / c);
			dgBigVector r1r0 (ray_p0 - r);
			dgFloat64 dist2 (r1r0 % r1r0);
			// almost parallel, intersection can not be tested, let the the extend precision routine figure it out 
			return dist2 <= (tol * tol);

		} else {         
			// get the closest points on the infinite lines
			dgFloat64 t = b * e - c * d;
			dgFloat64 s = a * e - b * d;

			dgFloat64 tolDen = -tol * den;
			dgFloat64 tolDen1 = den + tol;

			if ((t < tolDen) || (s < tolDen) || (t > tolDen1) || (s > tolDen1)) {
				return false;
			}

			dgBigVector r0 = ray_p0 + p1p0.Scale3 (t / den);
			dgBigVector r1 = ray_q0 + q1q0.Scale3 (s / den);
			dgBigVector r1r0 (r1 - r0);
			dgFloat64 dist2 (r1r0 % r1r0);
			if (dist2 > tol * tol) {
				return false;
			}
		}
		return true;
	}

	bool VertexFaceIntersectQuickTest (dgEdge* const vertex, const dgBooleanMeshClipper* const otherMeshBVH, dgEdge* const otherface) const
	{
		if (vertex->m_incidentVertex >= m_vertexBase) {
			return false;
		}
		if (m_vertexMap.Find(vertex->m_incidentVertex)) {
			return false;
		}

		const dgBigVector* const otherPoints = &otherMeshBVH->m_mesh->m_points[0];
		dgBigVector normal (otherMeshBVH->m_mesh->FaceNormal (otherface, &otherPoints[0].m_x, sizeof (dgBigVector)));
		dgFloat64 mag2 = normal % normal;
		if (mag2 < dgFloat64 (1.0e-12)) {
			// can not make decision let the extended precision test determine this
			return true;
		}
		normal = normal.Scale3 (1.0 / sqrt (mag2));
		

		dgBigVector p0 (m_mesh->m_points[vertex->m_incidentVertex]);
		dgFloat64 dist (normal % (p0 - otherPoints[otherface->m_incidentVertex]));
		if (fabs(dist) > dgFloat64 (1.0e-4f)) {
			return false;
		}
		
		dgEdge* ptr = otherface;
		do {
			dgInt32 index0 = ptr->m_incidentVertex;
			dgInt32 index1 = ptr->m_next->m_incidentVertex;
			dgBigVector p0v0 (otherPoints[index0] - p0);
			dgBigVector p0v1 (otherPoints[index1] - p0);
			dgFloat64 alpha = (normal * p0v0) % p0v1;
			if (alpha < -1.0e-2f) {
				return false;
			}
			
			ptr = ptr->m_next;
		} while (ptr != otherface);

		return true;
	}


	bool VertexFaceIntersect (dgEdge* const vertex, const dgBooleanMeshClipper* const otherMeshBVH, dgEdge* const otherface) const
	{
		const dgHugeVector* const otherPoints = &otherMeshBVH->m_vertexAlias[0];
		dgHugeVector normal (otherMeshBVH->FaceNormal (otherface));
		dgGoogol mag2 = normal % normal;
		dgAssert ((normal % normal) > m_tol2);

		dgHugeVector p0 (m_vertexAlias[vertex->m_incidentVertex]);
		dgGoogol dist (normal % (p0 - otherPoints[otherface->m_incidentVertex]));
		if (dist.Abs() > m_tol) {
			return false;
		}

		dgEdge* ptr = otherface;
		do {
			dgInt32 index0 = ptr->m_incidentVertex;
			dgInt32 index1 = ptr->m_next->m_incidentVertex;
			dgHugeVector p0v0 (otherPoints[index0] - p0);
			dgHugeVector p0v1 (otherPoints[index1] - p0);
			dgGoogol alpha ((normal * p0v0) % p0v1);
			if (alpha <= m_tol2) {
				return false;
			}

			ptr = ptr->m_next;
		} while (ptr != otherface);

		return true;
	}



	bool RayVertexIntersect (dgEdge* const edge, const dgBooleanMeshClipper* const otherMeshBVH, dgEdge* const otherVertex, dgGoogol& param) const 
	{
		dgHugeVector ray_p0 (m_vertexAlias[edge->m_incidentVertex]);
		dgHugeVector ray_p1 (m_vertexAlias[edge->m_twin->m_incidentVertex]);
		dgHugeVector q0 (otherMeshBVH->m_vertexAlias[otherVertex->m_incidentVertex]);

		dgHugeVector p1p0 (ray_p1 - ray_p0);
		dgHugeVector q0p0 (q0 - ray_p0);

		dgGoogol den = p1p0 % p1p0;        // always >= 0
		if (den < m_tol2) { 
			return false;
		}

		dgGoogol num = q0p0 % p1p0;        
		param = num / den;
		if ((param < m_tol) || (param > m_oneMinusTol)) {
			return false;
		}

		dgHugeVector r (ray_p0 + p1p0.Scale3 (param));
		dgHugeVector q0r (q0 - r);
		dgGoogol dist2 (q0r % q0r);
		return (dist2 <= m_tol2);
	}

	bool RayRayIntersect (dgEdge* const edge, const dgBooleanMeshClipper* const otherMeshBVH, dgEdge* const otherEdge, dgGoogol& param, dgGoogol& otherParam) const
	{
		dgHugeVector ray_p0 (m_vertexAlias[edge->m_incidentVertex]);
		dgHugeVector ray_p1 (m_vertexAlias[edge->m_twin->m_incidentVertex]);

		dgHugeVector ray_q0 (otherMeshBVH->m_vertexAlias[otherEdge->m_incidentVertex]);
		dgHugeVector ray_q1 (otherMeshBVH->m_vertexAlias[otherEdge->m_twin->m_incidentVertex]);

		dgHugeVector p1p0 (ray_p1 - ray_p0);
		dgHugeVector q1q0 (ray_q1 - ray_q0);
		dgHugeVector p0q0 (ray_p0 - ray_q0);

		dgGoogol a = p1p0 % p1p0;        // always >= 0
		dgGoogol c = q1q0 % q1q0;        // always >= 0
		dgGoogol b = p1p0 % q1q0;

		dgGoogol d = (p1p0 % p0q0);
		dgGoogol e = (q1q0 % p0q0);
		dgGoogol den = a * c - b * b;   // always >= 0

		// compute the line parameters of the two closest points
		if (den < m_tol4) { 
			// the lines are almost parallel
			return false;
		} else {         
			// get the closest points on the infinite lines
			dgGoogol t = (b * e - c * d) / den;
			dgGoogol s = (a * e - b * d) / den;

			if ((t < m_tol) || (s < m_tol) || (t > m_oneMinusTol) || (s > m_oneMinusTol)) {
				return false;
			}

			dgHugeVector r0 = ray_p0 + p1p0.Scale3 (t);
			dgHugeVector r1 = ray_q0 + q1q0.Scale3 (s);
			dgHugeVector r1r0 (r1 - r0);
			dgGoogol dist2 (r1r0 % r1r0);
			if (dist2 > m_tol2) {
				return false;
			}
			param = t;
			otherParam = s;
		}

		return true;
	}

	dgGoogol RayFaceIntersect (const dgMeshBVHNode* const faceNode, const dgHugeVector& p0, const dgHugeVector& p1) const
	{
		dgHugeVector normal (FaceNormal(faceNode->m_face));
		const dgHugeVector* const points = &m_vertexAlias[0];
	
	
		dgHugeVector diff (p1 - p0);
		dgGoogol tOut (dgGoogol::m_two);
		dgGoogol dir = normal % diff;
		if (dir < dgGoogol::m_zero) {
			dgEdge* ptr = faceNode->m_face;
			do {
				dgInt32 index0 = ptr->m_incidentVertex;
				dgInt32 index1 = ptr->m_next->m_incidentVertex;
				dgHugeVector p0v0 (points[index0] - p0);
				dgHugeVector p0v1 (points[index1] - p0);
				dgGoogol alpha ((diff * p0v1) % p0v0);
				if (alpha <= m_tol2) {
					return 1.2f;
				}

				ptr = ptr->m_next;
			} while (ptr != faceNode->m_face);

			dgInt32 index0 = ptr->m_incidentVertex;
			dgHugeVector p0v0 (points[index0] - p0);
			tOut = normal % p0v0;
			dgGoogol dist = normal % diff;
			tOut = tOut / dist;

		} else if (dir > dgGoogol::m_zero) {
			dgEdge* ptr = faceNode->m_face;
			do {
				dgInt32 index0 = ptr->m_incidentVertex;
				dgInt32 index1 = ptr->m_prev->m_incidentVertex;
				dgHugeVector p0v0 (points[index0] - p0);
				dgHugeVector p0v1 (points[index1] - p0);
				dgGoogol alpha ((diff * p0v1) % p0v0);
				if (alpha <= m_tol2) {
					return 1.2f;
				}

				ptr = ptr->m_prev;
			} while (ptr != faceNode->m_face);

			dgInt32 index0 = ptr->m_incidentVertex;
			dgHugeVector p0v0 (points[index0] - p0);
			tOut = normal % p0v0;
			dgGoogol dist = normal % diff;
			tOut = tOut / dist;
		}


		if (tOut < m_tol) {
			tOut = dgGoogol::m_two;
		} else if (tOut > (dgGoogol::m_one - m_tol)) {
			tOut = dgGoogol::m_two;
		}
		return tOut;
	}

	dgMeshBVHNode* FaceRayCast (const dgBooleanMeshClipper* const rayMeshBVH, const dgEdge* const edge, dgGoogol& paramOut) const
	{
		dgMeshBVHNode* stackPool[DG_MESH_EFFECT_BVH_STACK_DEPTH];

		dgInt32 stack = 1;
		dgMeshBVHNode* node = NULL;

		stackPool[0] = m_rootNode;
		dgGoogol maxParam = dgFloat32 (1.2f);

		dgTree<dgInt32,dgInt32>::dgTreeNode* const node0 = rayMeshBVH->m_vertexMap.Find(edge->m_incidentVertex);
		dgTree<dgInt32,dgInt32>::dgTreeNode* const node1 = rayMeshBVH->m_vertexMap.Find(edge->m_twin->m_incidentVertex);
		dgInt32 v0 = node0 ? node0->GetInfo() : -1;
		dgInt32 v1 = node1 ? node1->GetInfo() : -1;

		const dgMeshEffect* const mesh = rayMeshBVH->m_mesh;
		const dgBigVector* points = (dgBigVector*)mesh->GetVertexPool();
		const dgBigVector& q0 = points[edge->m_incidentVertex];
		const dgBigVector& q1 = points[edge->m_twin->m_incidentVertex];

		const dgHugeVector* const linePoints = &rayMeshBVH->m_vertexAlias[0];
		dgHugeVector p0 (linePoints[edge->m_incidentVertex]);
		dgHugeVector p1 (linePoints[edge->m_twin->m_incidentVertex]);

		dgVector l0(dgFloat32 (q0.m_x), dgFloat32 (q0.m_y), dgFloat32 (q0.m_z), dgFloat32 (0.0f));
		dgVector l1(dgFloat32 (q1.m_x), dgFloat32 (q1.m_y), dgFloat32 (q1.m_z), dgFloat32 (0.0f));
		dgFastRayTest ray (l0, l1);
		while (stack) {
			stack --;
			dgMeshBVHNode* const me = stackPool[stack];

			if (me && ray.BoxTest (me->m_p0, me->m_p1)) {

				if (!me->m_left) {
					dgAssert (!me->m_right);
					dgEdge* ptr = me->m_face;
					do {
						if ((ptr->m_incidentVertex == v0) || (ptr->m_incidentVertex == v1)) {
							break;
						}
						ptr = ptr->m_next;
					} while (ptr != me->m_face);
					if (ptr == me->m_face) {
						dgGoogol param = RayFaceIntersect (me, p0, p1);
						if (param < maxParam) {
							node = me;
							maxParam = param;
							ray.Reset (dgFloat32 (maxParam) + dgFloat32(0.01f));
						}
					}

				} else {
					dgAssert (me->m_left);
					dgAssert (me->m_right);
					stackPool[stack] = me->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack] = me->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
				}
			}
		}

		paramOut = maxParam;
		return node;
	}

	void InsertFaceVertex (dgEdge* const face, const dgHugeVector& point)
	{
		dgMeshEffect::dgVertexAtribute attribute;
		memset (&attribute, 0, sizeof (attribute));

		dgGoogol tol(m_tol);
		const dgMeshEffect::dgVertexAtribute* const attrib = (dgMeshEffect::dgVertexAtribute*)m_mesh->GetAttributePool(); 
		for (dgInt32 i = 0; i < 4; i ++) {

			dgGoogol posTol (tol + dgGoogol::m_one);
			dgGoogol negTol (dgGoogol::m_zero - m_tol);

			dgEdge* ptr = face;
			dgEdge* const edge0 = ptr;
			dgHugeVector q0 (m_vertexAlias[ptr->m_incidentVertex]);

			ptr = ptr->m_next;
			const dgEdge* edge1 = ptr;
			dgHugeVector q1 (m_vertexAlias[ptr->m_incidentVertex]);

			ptr = ptr->m_next;
			const dgEdge* edge2 = ptr;
			do {
				const dgHugeVector& q2 = m_vertexAlias[ptr->m_incidentVertex];

				dgHugeVector p10 (q1 - q0);
				dgHugeVector p20 (q2 - q0);

				dgGoogol dot = p20 % p10;
				dgGoogol mag1 = p10 % p10;
				dgGoogol mag2 = p20 % p20;
				dgGoogol collinear = dot * dot - mag2 * mag1;
				if (collinear.Abs() > m_tol) {
					dgHugeVector p_p0 (point - q0);
					dgHugeVector p_p1 (point - q1);
					dgHugeVector p_p2 (point - q2);

					dgGoogol alpha1 = p10 % p_p0;
					dgGoogol alpha2 = p20 % p_p0;
					dgGoogol alpha3 = p10 % p_p1;
					dgGoogol alpha4 = p20 % p_p1;
					dgGoogol alpha5 = p10 % p_p2;
					dgGoogol alpha6 = p20 % p_p2;

					dgGoogol vc = alpha1 * alpha4 - alpha3 * alpha2;
					dgGoogol vb = alpha5 * alpha2 - alpha1 * alpha6;
					dgGoogol va = alpha3 * alpha6 - alpha5 * alpha4;
					dgGoogol den = va + vb + vc;
					dgGoogol minError = den * negTol;
					dgGoogol maxError = den * posTol;

					if ((va > minError) && (vb > minError) && (vc > minError) && (va < maxError) && (vb < maxError) && (vc < maxError)) {

						edge2 = ptr;

						dgGoogol alpha0 = va / den;
						dgGoogol alpha1 = vb / den;
						dgGoogol alpha2 = vc / den;

						dgFloat64 falpha0 = alpha0;
						dgFloat64 falpha1 = alpha1;
						dgFloat64 falpha2 = alpha2;

						const dgMeshEffect::dgVertexAtribute& attr0 = attrib[edge0->m_userData];
						const dgMeshEffect::dgVertexAtribute& attr1 = attrib[edge1->m_userData];
						const dgMeshEffect::dgVertexAtribute& attr2 = attrib[edge2->m_userData];
						dgBigVector normal (attr0.m_normal_x * falpha0 + attr1.m_normal_x * falpha1 + attr2.m_normal_x * falpha2,
											attr0.m_normal_y * falpha0 + attr1.m_normal_y * falpha1 + attr2.m_normal_y * falpha2,
											attr0.m_normal_z * falpha0 + attr1.m_normal_z * falpha1 + attr2.m_normal_z * falpha2, dgFloat32 (0.0f));
						normal = normal.Scale3 (dgFloat64 (1.0f) / sqrt (normal % normal));


						attribute.m_vertex.m_x = point.m_x;
						attribute.m_vertex.m_y = point.m_y;
						attribute.m_vertex.m_z = point.m_z;
						attribute.m_vertex.m_w = point.m_w;
						attribute.m_normal_x = normal.m_x;
						attribute.m_normal_y = normal.m_y;
						attribute.m_normal_z = normal.m_z;
						attribute.m_u0 = attr0.m_u0 * falpha0 +  attr1.m_u0 * falpha1 + attr2.m_u0 * falpha2;
						attribute.m_v0 = attr0.m_v0 * falpha0 +  attr1.m_v0 * falpha1 + attr2.m_v0 * falpha2;
						attribute.m_u1 = attr0.m_u1 * falpha0 +  attr1.m_u1 * falpha1 + attr2.m_u1 * falpha2;
						attribute.m_v1 = attr0.m_v1 * falpha0 +  attr1.m_v1 * falpha1 + attr2.m_v1 * falpha2;

						attribute.m_material = attr0.m_material;
						dgAssert (attr0.m_material == attr1.m_material);
						dgAssert (attr0.m_material == attr2.m_material);

						m_mesh->AddPoint(&attribute.m_vertex.m_x, dgInt32 (attribute.m_material)); 
						m_vertexAlias[m_mesh->GetVertexCount()-1] = point;
						return;
					}
				}

				q1 = q2;
				edge1 = ptr;

				ptr = ptr->m_next;
			} while (ptr != face);
			dgAssert (0);
			tol = tol * dgGoogol::m_two;
		}
		// this should never happens
		dgAssert (0);
	}


	void SpliteEdge(dgEdge* const edge, dgGoogol param, dgEdge** edge0, dgEdge** edge1)
	{
		dgInt32 v0 = edge->m_incidentVertex;
		dgInt32 v1 = edge->m_twin->m_incidentVertex;
		dgPolyhedra::dgPairKey edgeKey (v0, v1);
		dgPolyhedra::dgPairKey twinKey (v1, v0);

		dgEdge* const newEdge = m_mesh->InsertEdgeVertex (edge, param);
		dgGoogol t0(dgGoogol::m_one - param);
		dgGoogol t1(param);
		dgInt32 v01 = newEdge->m_twin->m_incidentVertex;
		m_vertexAlias[v01] = m_vertexAlias[v0].Scale3 (t0) + m_vertexAlias[v1].Scale3 (t1);

		dgTree<dgMeshBVHNode*, dgUnsigned64>::dgTreeNode* const mapNode = m_nodeEdgeMap.Find(edgeKey.GetVal());
		if (mapNode) {
			dgMeshBVHNode* const node = mapNode->GetInfo();
			node->m_face = newEdge;
			m_nodeEdgeMap.Remove(mapNode);
			dgPolyhedra::dgPairKey key (newEdge->m_incidentVertex, newEdge->m_twin->m_incidentVertex);
			m_nodeEdgeMap.Insert(node, key.GetVal());
		}
		
		dgTree<dgMeshBVHNode*, dgUnsigned64>::dgTreeNode* const twinMapNode = m_nodeEdgeMap.Find(twinKey.GetVal());
		if (twinMapNode) {
			dgMeshBVHNode* const node = twinMapNode->GetInfo();
			node->m_face = newEdge->m_twin;
			m_nodeEdgeMap.Remove(twinMapNode);
			dgPolyhedra::dgPairKey key (newEdge->m_twin->m_incidentVertex, newEdge->m_incidentVertex);
			m_nodeEdgeMap.Insert(node, key.GetVal());
		}

		*edge0 = newEdge;
		*edge1 = newEdge->m_next;

		dgAssert (SanityCheck());
	}

	bool CalculateEdgeVertexIntersetions (dgBooleanMeshClipper& otherVertexMeshBVH)
	{
		dgMeshBVHNode* stackPool[4 * DG_MESH_EFFECT_BVH_STACK_DEPTH][2];

		dgInt32 stack = 1;
		stackPool[0][0] = m_rootNode;
		stackPool[0][1] = otherVertexMeshBVH.m_rootNode;

		dgList<dgEdge*> potencialEdge (m_mesh->GetAllocator());
		dgList<dgEdge*> otherPotencialVertex (m_mesh->GetAllocator());
		dgInt32 mark = dgMax(m_mesh->IncLRU(), otherVertexMeshBVH.m_mesh->IncLRU());
		while (stack) {
			stack --;
			dgMeshBVHNode* const me = stackPool[stack][0];
			dgMeshBVHNode* const other = stackPool[stack][1];

			dgAssert (me && other);
			if (dgOverlapTest (me->m_p0, me->m_p1, other->m_p0, other->m_p1)) {

				if (!me->m_left && !other->m_left) {
					dgAssert (!me->m_right);
					dgAssert (!other->m_right);

					dgEdge* myPtr = me->m_face;
					do {
						dgEdge* otherPtr = other->m_face;
						do {
							if ((myPtr->m_mark != mark) || (otherPtr->m_mark != mark)) {
				
								if (RayVertexIntersectQuickTest(myPtr, &otherVertexMeshBVH, otherPtr)) {
									if (myPtr->m_mark != mark) {
										potencialEdge.Append(myPtr);
									}

									if (otherPtr->m_mark != mark) {
										otherPotencialVertex.Append(otherPtr);
									}

									myPtr->m_mark = mark;
									myPtr->m_twin->m_mark = mark;

									dgEdge* ptr = otherPtr;
									do {
										ptr->m_mark = mark;
										ptr = ptr->m_twin->m_next;

									} while (ptr != otherPtr);
								}
							}
							otherPtr = otherPtr->m_next;
						} while (otherPtr != other->m_face);
						myPtr = myPtr->m_next;
					} while (myPtr != me->m_face);


				} else if (!me->m_left) {
					dgAssert (other->m_left);
					dgAssert (other->m_right);

					stackPool[stack][0] = me;
					stackPool[stack][1] = other->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me;
					stackPool[stack][1] = other->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

				} else if (!other->m_right) {
					dgAssert (me->m_left);
					dgAssert (me->m_right);

					stackPool[stack][0] = me->m_left;
					stackPool[stack][1] = other;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_right;
					stackPool[stack][1] = other;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
				} else {
					dgAssert (me->m_left && me->m_right);
					dgAssert (other->m_left && other->m_right);

					stackPool[stack][0] = me->m_left;
					stackPool[stack][1] = other->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_left;
					stackPool[stack][1] = other->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_right;
					stackPool[stack][1] = other->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_right;
					stackPool[stack][1] = other->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
				}
			}
		}


		bool intersectionFound = false;
		while (potencialEdge.GetFirst()) {
			dgEdge* const edge = potencialEdge.GetFirst()->GetInfo();
			potencialEdge.Remove(potencialEdge.GetFirst());
			for (dgList<dgEdge*>::dgListNode* otherNode = otherPotencialVertex.GetFirst(); otherNode; otherNode = otherNode->GetNext()) {
				dgEdge* const otherVertexEdge = otherNode->GetInfo();

				if (RayVertexIntersectQuickTest(edge, &otherVertexMeshBVH, otherVertexEdge)) {
					dgGoogol param;
					if (RayVertexIntersect (edge, &otherVertexMeshBVH, otherVertexEdge, param)) {

						intersectionFound = true;

						dgEdge* tmp0;
						dgEdge* tmp1;
						SpliteEdge (edge, param, &tmp0, &tmp1);
						m_vertexAlias[m_mesh->GetVertexCount()-1] = otherVertexMeshBVH.m_vertexAlias[otherVertexEdge->m_incidentVertex];
						potencialEdge.Addtop(tmp0);
						potencialEdge.Addtop(tmp1);
						m_vertexMap.Insert(otherVertexEdge->m_incidentVertex, m_mesh->GetVertexCount() - 1);

						otherVertexMeshBVH.m_vertexMap.Insert(m_mesh->GetVertexCount() - 1, otherVertexEdge->m_incidentVertex);
						break;
					}
				}
			}
		}
		return intersectionFound;
	}

	bool CalculateEdgeEdgeIntersetions (dgBooleanMeshClipper& otherEdgeBVH)
	{
		dgMeshBVHNode* stackPool[4 * DG_MESH_EFFECT_BVH_STACK_DEPTH][2];

		dgInt32 stack = 1;
		stackPool[0][0] = m_rootNode;
		stackPool[0][1] = otherEdgeBVH.m_rootNode;

		dgList<dgEdge*> potencialEdge (m_mesh->GetAllocator());
		dgList<dgEdge*> otherPotencialEdge (m_mesh->GetAllocator());
		dgInt32 mark = dgMax(m_mesh->IncLRU(), otherEdgeBVH.m_mesh->IncLRU());
		while (stack) {
			stack --;
			dgMeshBVHNode* const me = stackPool[stack][0];
			dgMeshBVHNode* const other = stackPool[stack][1];

			dgAssert (me && other);
			if (dgOverlapTest (me->m_p0, me->m_p1, other->m_p0, other->m_p1)) {

				if (!me->m_left && !other->m_left) {
					dgAssert (!me->m_right);
					dgAssert (!other->m_right);

					dgEdge* myPtr = me->m_face;
					do {
						dgEdge* otherPtr = other->m_face;
						do {
							if ((myPtr->m_mark != mark) || (otherPtr->m_mark != mark)) {
								if (RayRayIntersectQuickTest(myPtr, &otherEdgeBVH, otherPtr)) {
									if (myPtr->m_mark != mark) {
										potencialEdge.Append(myPtr);
									}
									if (otherPtr->m_mark != mark) {
										otherPotencialEdge.Append(otherPtr);
									}

									myPtr->m_mark = mark;
									myPtr->m_twin->m_mark = mark;
									otherPtr->m_mark = mark;
									otherPtr->m_twin->m_mark = mark;
								}
							}
							otherPtr = otherPtr->m_next;
						} while (otherPtr != other->m_face);
						myPtr = myPtr->m_next;
					} while (myPtr != me->m_face);


				} else if (!me->m_left) {
					dgAssert (other->m_left);
					dgAssert (other->m_right);

					stackPool[stack][0] = me;
					stackPool[stack][1] = other->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me;
					stackPool[stack][1] = other->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

				} else if (!other->m_right) {
					dgAssert (me->m_left);
					dgAssert (me->m_right);

					stackPool[stack][0] = me->m_left;
					stackPool[stack][1] = other;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_right;
					stackPool[stack][1] = other;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
				} else {
					dgAssert (me->m_left && me->m_right);
					dgAssert (other->m_left && other->m_right);

					stackPool[stack][0] = me->m_left;
					stackPool[stack][1] = other->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_left;
					stackPool[stack][1] = other->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_right;
					stackPool[stack][1] = other->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_right;
					stackPool[stack][1] = other->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
				}
			}
		}

		bool intersectionFound = false;
		while (potencialEdge.GetFirst()) {
			dgEdge* const edge = potencialEdge.GetFirst()->GetInfo();
			potencialEdge.Remove(potencialEdge.GetFirst());
			for (dgList<dgEdge*>::dgListNode* otherNode = otherPotencialEdge.GetFirst(); otherNode; otherNode = otherNode->GetNext()) {
				dgEdge* const otherEdge = otherNode->GetInfo();
				if (RayRayIntersectQuickTest(edge, &otherEdgeBVH, otherEdge)) {

					dgGoogol param; 
					dgGoogol otherParam;
					if (RayRayIntersect (edge, &otherEdgeBVH, otherEdge, param, otherParam)) {

						intersectionFound = true;

						dgEdge* tmp0;
						dgEdge* tmp1;
						SpliteEdge (edge, param, &tmp0, &tmp1);
						potencialEdge.Addtop(tmp0);
						potencialEdge.Addtop(tmp1);

						otherEdgeBVH.SpliteEdge (otherEdge, otherParam, &tmp0, &tmp1);
						otherPotencialEdge.Remove(otherNode);
						otherPotencialEdge.Addtop(tmp0);
						otherPotencialEdge.Addtop(tmp1);

						m_vertexMap.Insert(otherEdgeBVH.m_mesh->GetVertexCount() - 1, m_mesh->GetVertexCount() - 1);
						otherEdgeBVH.m_vertexMap.Insert(m_mesh->GetVertexCount() - 1, otherEdgeBVH.m_mesh->GetVertexCount() - 1);
						break;
					}
				}
			}
		}
		return intersectionFound;
	}


	void InsertFaceVertex (dgMeshBVH::dgMeshBVHNode* const nodeFace, const dgHugeVector& point) 
	{
		InsertFaceVertex (nodeFace->m_face, point);

		dgInt32 attibIndex = m_mesh->GetPropertiesCount() - 1;
		dgInt32 vertexIndex = m_mesh->GetVertexCount() - 1;

		dgInt32 count = 0;
		dgEdge* perimeter[256];

		dgEdge* ptr = nodeFace->m_face;
		do {
			perimeter[count] = ptr;
			count ++;
			ptr = ptr->m_next;
		} while (ptr != nodeFace->m_face);

		for(dgInt32 i = 0; i < count; i ++) {
			dgEdge* const perimeterFace = perimeter[i];
			dgEdge* const edge = m_mesh->AddHalfEdge(vertexIndex, perimeterFace->m_incidentVertex);
			dgEdge* const twin = m_mesh->AddHalfEdge(perimeterFace->m_incidentVertex, vertexIndex);

			edge->m_incidentFace = perimeterFace->m_incidentFace;
			twin->m_incidentFace = perimeterFace->m_incidentFace;

			edge->m_userData = attibIndex;
			twin->m_userData = perimeterFace->m_userData;

			twin->m_twin = edge;
			edge->m_twin = twin;

			perimeterFace->m_prev->m_next = twin;
			twin->m_prev = perimeterFace->m_prev;

			perimeterFace->m_prev = edge;
			edge->m_next = perimeterFace;
		}

		dgPolyhedra::dgPairKey key(nodeFace->m_face->m_incidentVertex, nodeFace->m_face->m_twin->m_incidentVertex);
		dgAssert (m_nodeEdgeMap.Find(key.GetVal()));
		m_nodeEdgeMap.Remove(key.GetVal());
		RemoveNode (nodeFace);

		for(dgInt32 i = 0; i < count; i ++) {
			dgEdge* const perimeterFace = perimeter[i];
			perimeterFace->m_next->m_next = perimeterFace->m_prev;
			perimeterFace->m_prev->m_prev = perimeterFace->m_next;

			dgMeshBVHNode* const newNode = AddNode (perimeterFace->m_next);
			dgPolyhedra::dgPairKey key(newNode->m_face->m_incidentVertex, newNode->m_face->m_twin->m_incidentVertex);
			m_nodeEdgeMap.Insert(newNode, key.GetVal());
		}
	}


	dgEdge* EdgeFaceIntersetion (dgEdge* const edgeSegment, dgBooleanMeshClipper& faceMeshBVH)
	{
		dgGoogol param;
		dgEdge* edge = NULL;
		dgMeshBVH::dgMeshBVHNode* const node = faceMeshBVH.FaceRayCast (this, edgeSegment, param);
		if (node) {
			// clip the line edge
			dgAssert (param > dgGoogol::m_zero);
			dgAssert (param < dgGoogol::m_one);
			dgEdge* next;
			SpliteEdge (edgeSegment, param, &edge, &next);
			dgAssert (!edge || edge->m_next == next);

			// tessellate the face
			faceMeshBVH.InsertFaceVertex (node, m_vertexAlias[m_mesh->GetVertexCount()-1]) ;

			m_vertexMap.Insert(faceMeshBVH.m_mesh->GetVertexCount() - 1, m_mesh->GetVertexCount() - 1);
			faceMeshBVH.m_vertexMap.Insert(m_mesh->GetVertexCount() - 1, faceMeshBVH.m_mesh->GetVertexCount() - 1);
		}
		return edge;
	}


	bool CalculateEdgeFacesIntersetions (dgBooleanMeshClipper& faceMeshBVH)
	{
		dgList<dgEdge*> edgeList (m_mesh->GetAllocator()) ;

		for (void* edgeNode = m_mesh->GetfirstClipperEdge(); edgeNode; edgeNode = m_mesh->GetNextEdge(edgeNode)) {
			dgEdge* const edge = &((dgMeshEffect::dgTreeNode*)edgeNode)->GetInfo();
			edgeList.Append(edge);
		}


		bool intersectionFound = false;
		while (edgeList.GetCount()) {
			dgEdge* const edgeSegment = edgeList.GetFirst()->GetInfo();
			edgeList.Remove(edgeList.GetFirst());

			dgEdge* const edge = EdgeFaceIntersetion (edgeSegment, faceMeshBVH);
			if (edge) {
				intersectionFound = true;
				edgeList.Addtop(edge);
				edgeList.Addtop(edge->m_next);
			}
		}
		if (intersectionFound) {
			faceMeshBVH.ImproveNodeFitness();
			dgAssert (faceMeshBVH.SanityCheck());
		}
		return intersectionFound;
	}


	bool CalculateVertexFacesIntersetions (dgBooleanMeshClipper& faceMeshBVH)
	{
		dgMeshBVHNode* stackPool[4 * DG_MESH_EFFECT_BVH_STACK_DEPTH][2];

		dgInt32 stack = 1;
		stackPool[0][0] = m_rootNode;
		stackPool[0][1] = faceMeshBVH.m_rootNode;

		dgList<dgFaceVertexPair> collidingPairs (m_mesh->GetAllocator());
		while (stack) {
			stack --;
			dgMeshBVHNode* const me = stackPool[stack][0];
			dgMeshBVHNode* const other = stackPool[stack][1];

			dgAssert (me && other);
			if (dgOverlapTest (me->m_p0, me->m_p1, other->m_p0, other->m_p1)) {

				if (!me->m_left && !other->m_left) {
					dgAssert (!me->m_right);
					dgAssert (!other->m_right);

					dgEdge* vertexEdge = me->m_face;
					do {
						if (VertexFaceIntersectQuickTest (vertexEdge, &faceMeshBVH, other->m_face)) {
							bool found = false;
							for (dgList<dgFaceVertexPair>::dgListNode* node = collidingPairs.GetFirst(); node; node = node->GetNext()) {
								const dgFaceVertexPair& pair = node->GetInfo();
								if ((other == pair.m_faceNode) && (pair.m_vertex->m_incidentVertex == vertexEdge->m_incidentVertex)) { 
									found = true;
									break;
								}
							}
							if (!found) {
								dgFaceVertexPair pair (vertexEdge, other);
								collidingPairs.Append(pair);
							}
						}
						vertexEdge = vertexEdge->m_next;
					} while (vertexEdge != me->m_face);
	
				} else if (!me->m_left) {
					dgAssert (other->m_left);
					dgAssert (other->m_right);

					stackPool[stack][0] = me;
					stackPool[stack][1] = other->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me;
					stackPool[stack][1] = other->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

				} else if (!other->m_right) {
					dgAssert (me->m_left);
					dgAssert (me->m_right);

					stackPool[stack][0] = me->m_left;
					stackPool[stack][1] = other;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_right;
					stackPool[stack][1] = other;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
				} else {
					dgAssert (me->m_left && me->m_right);
					dgAssert (other->m_left && other->m_right);

					stackPool[stack][0] = me->m_left;
					stackPool[stack][1] = other->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_left;
					stackPool[stack][1] = other->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_right;
					stackPool[stack][1] = other->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_right;
					stackPool[stack][1] = other->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
				}
			}
		}

		bool intersectionFound = false;
		while (collidingPairs.GetFirst()) {
			dgFaceVertexPair pair (collidingPairs.GetFirst()->GetInfo());
			collidingPairs.Remove(collidingPairs.GetFirst());

			if (VertexFaceIntersect (pair.m_vertex, &faceMeshBVH, pair.m_faceNode->m_face)) {
				dgList<dgFaceVertexPair>::dgListNode* nextNode;
				for (dgList<dgFaceVertexPair>::dgListNode* node = collidingPairs.GetFirst(); node; node = nextNode) {
					nextNode = node->GetNext();

					const dgFaceVertexPair& conflictingPair = node->GetInfo();
					if ((pair.m_faceNode == conflictingPair.m_faceNode) || (pair.m_vertex->m_incidentVertex == conflictingPair.m_vertex->m_incidentVertex)) { 
						collidingPairs.Remove(node);
					}
				}

				intersectionFound = true;
				faceMeshBVH.InsertFaceVertex (pair.m_faceNode, m_vertexAlias[pair.m_vertex->m_incidentVertex]) ;
				m_vertexMap.Insert(faceMeshBVH.m_mesh->GetVertexCount() - 1, pair.m_vertex->m_incidentVertex);
				faceMeshBVH.m_vertexMap.Insert(pair.m_vertex->m_incidentVertex, faceMeshBVH.m_mesh->GetVertexCount() - 1);
			}
		}

		if (intersectionFound) {
			faceMeshBVH.ImproveNodeFitness();
			dgAssert (faceMeshBVH.SanityCheck());
		}
		return intersectionFound;
	}


	dgEdge* FindMatchEdge (dgEdge* const vertex, const dgBooleanMeshClipper& otherMeshBVH) const
	{
		dgInt32 vA0 = vertex->m_incidentVertex;
		dgAssert (m_vertexMap.Find (vA0));
		dgAssert (otherMeshBVH.m_vertexMap.Find(m_vertexMap.Find (vA0)->GetInfo()));
		dgAssert (m_vertexMap.Find (vA0)->GetInfo() == otherMeshBVH.m_vertexMap.Find(m_vertexMap.Find (vA0)->GetInfo())->GetKey());

		dgInt32 vB0 = m_vertexMap.Find (vA0)->GetInfo();
		dgEdge* ptr = vertex;
		do {
			dgTree<dgInt32,dgInt32>::dgTreeNode* const node = m_vertexMap.Find(ptr->m_twin->m_incidentVertex);
			if (node) {
				dgInt32 vA1 = node->GetInfo();
				dgAssert (otherMeshBVH.m_vertexMap.Find(vA1));
				dgInt32 vB1 = otherMeshBVH.m_vertexMap.Find (vA1)->GetInfo();
				if (otherMeshBVH.m_mesh->FindEdge(vB0, vB1)) {
					return ptr;
				}
			}
			ptr = ptr->m_twin->m_next;
		} while (ptr != vertex);

		dgAssert (0);
		return NULL;
	}

	bool ConnectExistingEdges (dgBooleanMeshClipper* const otherMesh) const
	{
		bool foundEdge = false;
		dgTree<dgInt32,dgInt32>::Iterator iter (m_vertexMap);
		for (iter.Begin(); iter; iter ++) {
			dgInt32 vA0 = iter.GetKey();
			dgInt32 vB0 = iter.GetNode()->GetInfo();
			dgAssert (otherMesh->m_vertexMap.Find(vB0));

			dgPolyhedra::dgPairKey key (vA0, 0);
			dgAssert (m_mesh->FindGreaterEqual(key.GetVal()));

			dgEdge* const edgeA0 = &m_mesh->FindGreaterEqual(key.GetVal())->GetInfo();
			dgEdge* ptr = edgeA0;
			do {
				dgInt32 vA1 = ptr->m_twin->m_incidentVertex;
				dgTree<dgInt32,dgInt32>::dgTreeNode* const node = m_vertexMap.Find(vA1);
				if (node) {
					dgInt32 vB1 = node->GetInfo();
					const dgEdge* const otherEdge = otherMesh->m_mesh->FindEdge(vB1, vB0);
					if (!otherEdge) {
						dgEdge* const ret = otherMesh->m_mesh->SpliteFace (vB1, vB0);
						foundEdge |= ret ? true : false;
					}
				}
			
				ptr = ptr->m_twin->m_next;
			} while (ptr != edgeA0); 
		}
		return foundEdge;
	}

	void CalculateNewEdgeIntestions (dgEdge* const edgeSegment, dgBooleanMeshClipper* const otherMeshBVH)
	{
		dgEdge* const edge = EdgeFaceIntersetion (edgeSegment, *otherMeshBVH);
		if (edge) {
			dgEdge* const next = edge->m_next; 
			CalculateNewEdgeIntestions (edge, otherMeshBVH);
			CalculateNewEdgeIntestions (next, otherMeshBVH);
			return;
		}

		const dgHugeVector& q0 = m_vertexAlias[edgeSegment->m_incidentVertex];
		const dgHugeVector& q1 = m_vertexAlias[edgeSegment->m_twin->m_incidentVertex];

		dgBigVector l0 (q0.m_x, q0.m_y, q0.m_z, 0.0);
		dgBigVector l1 (q1.m_x, q1.m_y, q1.m_z, 0.0);
		dgBigVector p0 (dgMin (l0.m_x, l1.m_x) - 0.1, dgMin (l0.m_y, l1.m_y) - 0.1, dgMin (l0.m_z, l1.m_z) - 0.1, 0.0);
		dgBigVector p1 (dgMax (l0.m_x, l1.m_x) + 0.1, dgMax (l0.m_y, l1.m_y) + 0.1, dgMin (l0.m_z, l1.m_z) + 0.1, 0.0);

		dgList<dgMeshBVHNode*> nodeList (m_mesh->GetAllocator());
		otherMeshBVH->GetOverlapNodes (nodeList, p0, p1);

		for (dgList<dgMeshBVHNode*>::dgListNode* nodePtr = nodeList.GetFirst(); nodePtr; nodePtr = nodePtr->GetNext()) {
			dgMeshBVHNode* const node = nodePtr->GetInfo();

			dgEdge* const otherEdge = node->m_face; 
			dgEdge* ptr = otherEdge; 
			do {
				if (RayRayIntersectQuickTest(edgeSegment, otherMeshBVH, ptr)) {

					dgGoogol param; 
					dgGoogol otherParam;
					if (RayRayIntersect (edgeSegment, otherMeshBVH, ptr, param, otherParam)) {
						dgAssert (0);
/*
						dgEdge* tmp0;
						dgEdge* tmp1;
						SpliteEdge (edge, param, &tmp0, &tmp1);
						potencialEdge.Addtop(tmp0);
						potencialEdge.Addtop(tmp1);

						otherEdgeBVH.SpliteEdge (otherEdge, otherParam, &tmp0, &tmp1);
						otherPotencialEdge.Remove(otherNode);
						otherPotencialEdge.Addtop(tmp0);
						otherPotencialEdge.Addtop(tmp1);

						m_vertexMap.Insert(otherEdgeBVH.m_mesh->GetVertexCount() - 1, m_mesh->GetVertexCount() - 1);
						otherEdgeBVH.m_vertexMap.Insert(m_mesh->GetVertexCount() - 1, otherEdgeBVH.m_mesh->GetVertexCount() - 1);
*/
						return;
					}
				}

				ptr = ptr->m_next;
			} while (ptr != otherEdge);
		}

		for (dgList<dgMeshBVHNode*>::dgListNode* nodePtr = nodeList.GetFirst(); nodePtr; nodePtr = nodePtr->GetNext()) {
			dgMeshBVHNode* const node = nodePtr->GetInfo();

			dgEdge* const otherEdge = node->m_face; 
			dgEdge* ptr = otherEdge; 
			do {

				if (RayVertexIntersectQuickTest(edgeSegment, otherMeshBVH, ptr)) {
					dgGoogol param;
					if (RayVertexIntersect (edgeSegment, otherMeshBVH, ptr, param)) {
						dgEdge* edge;
						dgEdge* next;
						SpliteEdge (edgeSegment, param, &edge, &next);
						m_vertexAlias[m_mesh->GetVertexCount()-1] = otherMeshBVH->m_vertexAlias[ptr->m_incidentVertex];

						m_vertexMap.Insert(ptr->m_incidentVertex, m_mesh->GetVertexCount() - 1);
						otherMeshBVH->m_vertexMap.Insert(m_mesh->GetVertexCount() - 1, ptr->m_incidentVertex);

						nodeList.RemoveAll();
						CalculateNewEdgeIntestions (edge, otherMeshBVH);
						CalculateNewEdgeIntestions (next, otherMeshBVH);
						return;
					}
				}
				ptr = ptr->m_next;
			} while (ptr != otherEdge);
		}
		return;
	}

/*
	bool GetFirtAndLastFan (const dgEdge* const edge, dgEdge** const firstClipperEdge, dgEdge** const lastEdge) const
	{
		dgEdge* first = edge->m_next->m_next;
		dgEdge* last = edge->m_prev;

		const dgHugeVector& p0 = m_vertexAlias[edge->m_incidentVertex];
		const dgHugeVector& p1 = m_vertexAlias[edge->m_next->m_incidentVertex];
		dgHugeVector p1p0 (p1 - p0);
		dgGoogol p1p0Mag2 (p1p0 % p1p0);
		while (first != last) {
			const dgHugeVector& p2 = m_vertexAlias[first->m_incidentVertex];
			dgHugeVector p2p0 (p2 - p0);
			dgGoogol p2p0Mag2 (p2p0 % p2p0);
			dgGoogol dot (p1p0 % p2p0);
			dgGoogol error (p2p0Mag2 * p1p0Mag2 -  dot * dot);
			if (error > m_tol2) {
				dgAssert (first != last);
				const dgHugeVector& p1 = m_vertexAlias[last->m_incidentVertex];
				dgHugeVector p1p0 (p1 - p0);
				dgGoogol p1p0Mag2 (p1p0 % p1p0);
				do {
					const dgHugeVector& p2 = m_vertexAlias[last->m_prev->m_incidentVertex];
					dgHugeVector p2p0 (p2 - p0);
					dgGoogol p2p0Mag2 (p2p0 % p2p0);
					dgGoogol dot (p1p0 % p2p0);
					dgGoogol error (p2p0Mag2 * p1p0Mag2 -  dot * dot);
					if (error > m_tol2) {
						*firstClipperEdge = first;
						*lastEdge = last;
						return true;
					}
					last = last->m_prev;
				} while (last != first);
				break;
			}
			first = first->m_next;
		};

		return false;
	}
*/

	dgGoogol ColinealError (dgEdge* const edge) const
	{
		const dgHugeVector& p0 = m_vertexAlias[edge->m_incidentVertex];
		const dgHugeVector& p1 = m_vertexAlias[edge->m_next->m_incidentVertex];
		const dgHugeVector& p2 = m_vertexAlias[edge->m_next->m_next->m_incidentVertex];
		dgHugeVector p1p0 (p1 - p0);
		dgHugeVector p2p0 (p2 - p0);
		dgGoogol p1p0Mag2 (p1p0 % p1p0);
		dgGoogol p2p0Mag2 (p2p0 % p2p0);
		dgGoogol dot (p1p0 % p2p0);
		return p2p0Mag2 * p1p0Mag2 -  dot * dot;
	}

	dgEdge* SpliteFace (dgInt32 v1, dgInt32 v0)
	{
		dgEdge* const edge = m_mesh->SpliteFace(v1, v0);
		if (edge) {
			dgGoogol error0 (ColinealError (edge));
			dgGoogol error1 (ColinealError (edge->m_twin));
			if ((error0 < m_tol2) || (error1 < m_tol2)) {
				m_mesh->DeleteEdge(edge);
				return NULL;
			}
		}

		return edge;
	}

	bool ConnectMissingEdges (dgBooleanMeshClipper* const otherMeshBVH)
	{
		bool foundEdge = false;
		dgTree<dgInt32,dgInt32>::Iterator iter (m_vertexMap);
		for (iter.Begin(); iter; iter ++) {
			dgInt32 vA0 = iter.GetKey();
			dgInt32 vB0 = iter.GetNode()->GetInfo();
			dgAssert (otherMeshBVH->m_vertexMap.Find(vB0));

			dgPolyhedra::dgPairKey key (vA0, 0);
			dgAssert (m_mesh->FindGreaterEqual(key.GetVal()));

			dgEdge* const edgeA0 = &m_mesh->FindGreaterEqual(key.GetVal())->GetInfo();
			dgEdge* ptr = edgeA0;
			do {
				if (ptr->m_incidentFace > 0) {
					for (dgEdge* edgeA1 = ptr->m_next->m_next; edgeA1 != ptr->m_prev; edgeA1 = edgeA1->m_next) {
						dgInt32 vA1 = edgeA1->m_incidentVertex;
						if (m_vertexMap.Find(vA1)) {
							if (!m_mesh->FindEdge(vA1, vA0)) {
								dgTree<dgInt32,dgInt32>::dgTreeNode* const node = m_vertexMap.Find(vA1);
								if (node) {
									dgInt32 vB1 = node->GetInfo();
									dgEdge* const newOtherEdge = otherMeshBVH->SpliteFace(vB1, vB0);
									if (newOtherEdge) {
										foundEdge = true;
										otherMeshBVH->CalculateNewEdgeIntestions (newOtherEdge, this);
									}
								}
							}
						}
					}
				}
			
				ptr = ptr->m_twin->m_next;
			} while (ptr != edgeA0); 
		}
		return foundEdge;
	}

	const dgHugeVector& GetTestPoint(const dgEdge* const edge) const
	{
		const dgBigVector& p0 = m_mesh->m_points[edge->m_incidentVertex];
		const dgBigVector& p1 = m_mesh->m_points[edge->m_twin->m_incidentVertex];
		dgBigVector p1p0 (p1 - p0);
		dgInt32 index = -1;
		dgFloat64 mag2 = p1p0 % p1p0;
		if (mag2 > 1.0e-8) {
			dgFloat64 den = 1.0f / mag2;
			dgFloat64 maxDist = -1.0e10f;
			for (dgEdge* ptr = edge->m_next->m_next; ptr != edge; ptr = ptr->m_next) {
				const dgBigVector& q = m_mesh->m_points[ptr->m_incidentVertex];
				dgBigVector qp0 (q - p0);
				dgBigVector diff (qp0 - p1p0.Scale3 ((qp0 % p1p0) * den));
				dgFloat64 dist2 = diff % diff;
				if (dist2 > maxDist) {
					index = ptr->m_incidentVertex;
					maxDist = dist2;
				}
			}
		} else {
			const dgHugeVector& p0 = m_vertexAlias[edge->m_incidentVertex];
			const dgHugeVector& p1 = m_vertexAlias[edge->m_twin->m_incidentVertex];
			dgHugeVector p1p0 (p1 - p0);
			dgGoogol mag2 (p1p0 % p1p0);
			dgAssert (mag2 > dgGoogol::m_zero);
			dgGoogol den = dgGoogol::m_one / mag2;
			dgGoogol maxDist (-1.0e10f);
			for (dgEdge* ptr = edge->m_next->m_next; ptr != edge; ptr = ptr->m_next) {
				const dgHugeVector& q = m_mesh->m_points[ptr->m_incidentVertex];
				dgHugeVector qp0 (q - p0);
				dgHugeVector diff (qp0 - p1p0.Scale3 ((qp0 % p1p0) * den));
				dgGoogol dist2 = diff % diff;
				if (dist2 > maxDist) {
					index = ptr->m_incidentVertex;
					maxDist = dist2;
				}
			}
		}

		dgAssert (index >= 0);
		return m_vertexAlias[index];
	}

	void ColorInteriorFaces (const dgBooleanMeshClipper* const meshBvhB)
	{

		dgPolyhedra::Iterator iter (*m_mesh);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &(*iter);
			edge->m_mark = 0;
		}

		dgInt32 mark = m_mesh->IncLRU();
		dgAssert (mark < DG_INTERIOR_FACE_MARK);


		dgList<dgEdge*> borderStack (m_mesh->GetAllocator());
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edgeA = &iter.GetNode()->GetInfo();
			if (~edgeA->m_mark & mark) {
				edgeA->m_mark |= mark;
				edgeA->m_twin->m_mark |= mark;
				dgInt32 vA0 = edgeA->m_incidentVertex;
				dgInt32 vA1 = edgeA->m_twin->m_incidentVertex;
				dgTree<dgInt32, dgInt32>::dgTreeNode* const nodeA0 = m_vertexMap.Find(vA0);
				if (nodeA0) {
					dgTree<dgInt32, dgInt32>::dgTreeNode* const nodeA1 = m_vertexMap.Find(vA1);
					if (nodeA1) {
						dgInt32 vB0 = nodeA0->GetInfo();
						dgInt32 vB1 = nodeA1->GetInfo();
						dgEdge* const edgeB = meshBvhB->m_mesh->FindEdge(vB0, vB1);
						if (edgeB) {
							dgHugeVector normalB0 (meshBvhB->FaceNormal(edgeB));
							dgHugeVector normalB1 (meshBvhB->FaceNormal(edgeB->m_twin));
							const dgHugeVector& origin (m_vertexAlias[vA0]);
							//const dgHugeVector& testPoint1 (m_vertexAlias[edgeA->m_prev->m_incidentVertex]);
							const dgHugeVector& testPoint1 = GetTestPoint(edgeA);
							dgHugeVector dist0 (testPoint1 - origin);
							dgGoogol side10 (normalB0 % dist0);
							dgGoogol side11 (normalB1 % dist0);

							//const dgHugeVector& testPoint2 (m_vertexAlias[edgeA->m_twin->m_prev->m_incidentVertex]);
							const dgHugeVector& testPoint2 = GetTestPoint(edgeA->m_twin);
							dgHugeVector dist2 (testPoint2 - origin);
							dgGoogol side20 (normalB0 % dist2);
							dgGoogol side21 (normalB1 % dist2);

							bool edgeMarkA = false;
							bool twinMarkA = false;
							if ((side10 < m_negTol) || (side11 < m_negTol)) {
								edgeMarkA = true;
								if ((side20 < m_negTol) || (side21 < m_negTol)) {
									twinMarkA = true;
								}

							} else {
								if ((side20 < m_negTol) || (side21 < m_negTol)) {
									twinMarkA = true;
								}
							}

							if (edgeMarkA & !twinMarkA) {
								borderStack.Append(edgeA);
								dgEdge* ptr = edgeA->m_twin; 
								do {
									ptr->m_mark |= DG_LOOP_BORDER_MARK;
									ptr = ptr->m_next;
								} while (ptr != edgeA->m_twin);

							} else if (!edgeMarkA & twinMarkA) {
								borderStack.Append(edgeA->m_twin);
								dgEdge* ptr = edgeA; 
								do {
									ptr->m_mark |= DG_LOOP_BORDER_MARK;
									ptr = ptr->m_next;
								} while (ptr != edgeA);
							}
						}
					}
				}
			}
		}


		while (borderStack.GetCount()) {
			dgEdge* const edge = borderStack.GetFirst()->GetInfo();
			borderStack.Remove(borderStack.GetFirst());

			if ((~edge->m_mark & DG_LOOP_BORDER_MARK) && (~edge->m_mark & DG_INTERIOR_FACE_MARK)) {
				dgEdge* ptr = edge;
				do {
					ptr->m_mark |= DG_INTERIOR_FACE_MARK;
					if ((~ptr->m_twin->m_mark & DG_LOOP_BORDER_MARK) && (~ptr->m_twin->m_mark & DG_INTERIOR_FACE_MARK)) {
						borderStack.Addtop(ptr->m_twin);
					}
					ptr = ptr->m_next;
				} while (ptr != edge);
			}
		}
	}


	static void ConnectEntersections (dgBooleanMeshClipper* const meshBvhA, dgBooleanMeshClipper* const meshBvhB)
	{
		dgAssert (meshBvhA->m_vertexMap.GetCount() == meshBvhB->m_vertexMap.GetCount());				

		for (bool edgeFound = true; edgeFound;) {
			edgeFound = false;
			edgeFound |= meshBvhA->ConnectExistingEdges (meshBvhB);
			edgeFound |= meshBvhB->ConnectExistingEdges (meshBvhA);
		}

		for (bool edgeFound = true; edgeFound;) {
			edgeFound = false;
			while (meshBvhA->ConnectMissingEdges (meshBvhB)) {
				edgeFound = true;
				meshBvhB->ConnectExistingEdges (meshBvhA);
			}
			while (meshBvhB->ConnectMissingEdges (meshBvhA)) {
				edgeFound = true;
				meshBvhA->ConnectExistingEdges (meshBvhB);
			}
		}

		meshBvhA->ColorInteriorFaces (meshBvhB);
		meshBvhB->ColorInteriorFaces (meshBvhA);
	}

	static void ClipMeshesAndColorize (dgMeshEffect* const meshA, dgMeshEffect* const meshB)
	{
		dgBooleanMeshClipper BVHmeshA (meshA);
		dgBooleanMeshClipper BVHmeshB (meshB);

		dgInt32 baseAttibuteCountB = BVHmeshB.m_mesh->GetPropertiesCount();


BVHmeshA.m_mesh->SaveOFF("xxxA0.off");
BVHmeshB.m_mesh->SaveOFF("xxxB0.off");

		// edge-face, edge-edge and edge-vertex intersections until not more intersections are found 
		for (bool intersectionFound = true; intersectionFound;) {
			intersectionFound = false;

			intersectionFound |= BVHmeshA.CalculateEdgeFacesIntersetions (BVHmeshB);
			intersectionFound |= BVHmeshB.CalculateEdgeFacesIntersetions (BVHmeshA);

			intersectionFound |= BVHmeshA.CalculateVertexFacesIntersetions (BVHmeshB);
			intersectionFound |= BVHmeshB.CalculateVertexFacesIntersetions (BVHmeshA);


BVHmeshA.m_mesh->SaveOFF("xxxA1.off");
BVHmeshB.m_mesh->SaveOFF("xxxB1.off");

			intersectionFound |= BVHmeshA.CalculateEdgeEdgeIntersetions (BVHmeshB);

BVHmeshA.m_mesh->SaveOFF("xxxA2.off");
BVHmeshB.m_mesh->SaveOFF("xxxB2.off");

			intersectionFound |= BVHmeshA.CalculateEdgeVertexIntersetions (BVHmeshB);
			intersectionFound |= BVHmeshB.CalculateEdgeVertexIntersetions (BVHmeshA);

BVHmeshA.m_mesh->SaveOFF("xxxA3.off");
BVHmeshB.m_mesh->SaveOFF("xxxB3.off");

		};

BVHmeshA.m_mesh->SaveOFF("xxxA4.off");
BVHmeshB.m_mesh->SaveOFF("xxxB4.off");

		ConnectEntersections (&BVHmeshA, &BVHmeshB);

BVHmeshA.m_mesh->SaveOFF("xxxA5.off");
BVHmeshB.m_mesh->SaveOFF("xxxB5.off");


		// copy the vertex from the other mesh
		dgInt32 baseAttibuteCountA = BVHmeshA.m_mesh->GetPropertiesCount();
		const dgMeshEffect::dgVertexAtribute* const property = (dgMeshEffect::dgVertexAtribute*)BVHmeshB.m_mesh->GetAttributePool();
		for (dgInt32 i = 0; i < baseAttibuteCountB; i ++) {
			BVHmeshA.m_mesh->AddAtribute(property [i]);
		}

		dgMeshEffect::Iterator iter (*BVHmeshB.m_mesh);
		const dgBigVector* const vertexB = (dgBigVector*)BVHmeshB.m_mesh->GetVertexPool();
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const face = &iter.GetNode()->GetInfo();
			if (~face->m_mark & DG_REMAP_VERTEX_MARK) {
				
				dgInt32 newIndex = face->m_incidentVertex;
				if (newIndex < BVHmeshB.m_vertexBase) {
					BVHmeshA.m_mesh->AddVertex(vertexB[newIndex]);
					newIndex = BVHmeshA.m_mesh->GetVertexCount() - 1;
				} else {
					dgAssert (BVHmeshB.m_vertexMap.Find(newIndex));
					newIndex = BVHmeshB.m_vertexMap.Find(newIndex)->GetInfo();
				}

				dgEdge* ptr = face;
				do {
					if (dgInt32 (ptr->m_userData) < baseAttibuteCountB) {
						ptr->m_userData += baseAttibuteCountA;
					} else {
						BVHmeshA.m_mesh->AddAtribute(BVHmeshB.m_mesh->GetAttribute(dgInt32 (ptr->m_userData)));
						ptr->m_userData = BVHmeshA.m_mesh->GetPropertiesCount() - 1;
					}

					ptr->m_incidentVertex = newIndex;
					ptr->m_mark |= DG_REMAP_VERTEX_MARK;
					ptr = ptr->m_twin->m_next;
				} while (ptr != face);
			}
		}
	}

	static void CopyPoints(dgMeshEffect* const dest, const dgMeshEffect* const source)
	{
		dgInt32 vertexCount = source->GetVertexCount();
		const dgBigVector* const vertex = (dgBigVector*)source->GetVertexPool();
		for (dgInt32 i = 0; i < vertexCount; i ++) {
			dest->AddVertex(vertex[i]);
		}

		dgInt32 propertyCount = source->GetPropertiesCount();
		const dgMeshEffect::dgVertexAtribute* const property = (dgMeshEffect::dgVertexAtribute*)source->GetAttributePool();
		for (dgInt32 i = 0; i < propertyCount; i ++) {
			dest->AddAtribute(property [i]);
		}
	}

	static void AddFace (dgMeshEffect* const dest, dgPolyhedra* const source, dgEdge* const face)
	{
		dgInt32 faceIndex[1024];
		dgInt64 attibIndex[1024];

		dgInt32 count = 0;
		dgEdge* ptr = face;
		do {
			ptr->m_mark |= DG_BOOLEAN_COMBINE_MARK;
			faceIndex[count] = ptr->m_incidentVertex;
			attibIndex[count] = ptr->m_userData;
			count ++;
			dgAssert (count < dgInt32 (sizeof (faceIndex) / sizeof (faceIndex[0])));
			ptr = ptr->m_next;
		} while (ptr != face);
		dest->AddFace(count, faceIndex, attibIndex);
	}

	static void AddExteriorFaces (dgMeshEffect* const dest, dgPolyhedra* const source)
	{
		dgMeshEffect::Iterator iter (*source);
		for (iter.Begin(); iter; iter ++) {

			dgEdge* const face = &iter.GetNode()->GetInfo();
			if ((~face->m_mark & DG_BOOLEAN_COMBINE_MARK) && (face->m_incidentFace > 0) && (~face->m_mark & DG_INTERIOR_FACE_MARK)) {
				AddFace (dest, source, face);
			}
		}
	}

	static void AddInteriorFaces (dgMeshEffect* const dest, dgPolyhedra* const source)
	{
		dgMeshEffect::Iterator iter (*source);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const face = &iter.GetNode()->GetInfo();
			if ((~face->m_mark & DG_BOOLEAN_COMBINE_MARK) && (face->m_incidentFace > 0) && (face->m_mark & DG_INTERIOR_FACE_MARK)) {
				AddFace (dest, source, face);
			}
		}
	}

	static void AddInteriorFacesInvertWinding (dgMeshEffect* const dest, dgPolyhedra* const source)
	{
		dgStack<dgInt32> marks(dest->GetPropertiesCount());
		memset (&marks[0], 0,  marks.GetSizeInBytes());

		dgMeshEffect::dgVertexAtribute* const property = (dgMeshEffect::dgVertexAtribute*)dest->GetAttributePool();
		dgMeshEffect::Iterator iter (*source);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const face = &iter.GetNode()->GetInfo();
			if ((~face->m_mark & DG_BOOLEAN_COMBINE_MARK) && (face->m_incidentFace > 0) && (face->m_mark & DG_INTERIOR_FACE_MARK)) {
				dgInt32 faceIndex[1024];
				dgInt64 attibIndex[1024];

				dgInt32 count = 0;
				dgEdge* ptr = face;
				do {
					ptr->m_mark |= DG_BOOLEAN_COMBINE_MARK;
					faceIndex[count] = ptr->m_incidentVertex;
					attibIndex[count] = ptr->m_userData;
					if (!marks[dgInt32 (ptr->m_userData)]) {
						marks[dgInt32 (ptr->m_userData)] = 1;
						property[ptr->m_userData].m_normal_x *= dgFloat32 (-1.0f);
						property[ptr->m_userData].m_normal_y *= dgFloat32 (-1.0f);
						property[ptr->m_userData].m_normal_z *= dgFloat32 (-1.0f);
					}

					count ++;
					dgAssert (count < dgInt32 (sizeof (faceIndex) / sizeof (faceIndex[0])));
					ptr = ptr->m_prev;
				} while (ptr != face);
				dest->AddFace(count, faceIndex, attibIndex);
			}
		}
	}

	dgInt32 m_vertexBase;
	dgTree<dgInt32,dgInt32> m_vertexMap;
	dgTree<dgMeshBVHNode*, dgUnsigned64> m_nodeEdgeMap;
	dgArray<dgHugeVector> m_vertexAlias;

	static dgGoogol m_tol;
	static dgGoogol m_tol2;
	static dgGoogol m_tol3;
	static dgGoogol m_tol4;
	static dgGoogol m_negTol;
	static dgGoogol m_oneMinusTol;
};
dgGoogol dgBooleanMeshClipper::m_tol(DG_BOOLEAN_ZERO_TOLERANCE);
dgGoogol dgBooleanMeshClipper::m_oneMinusTol (dgGoogol(1.0) - m_tol);
dgGoogol dgBooleanMeshClipper::m_negTol(-DG_BOOLEAN_ZERO_TOLERANCE);
dgGoogol dgBooleanMeshClipper::m_tol2(m_tol * m_tol);
dgGoogol dgBooleanMeshClipper::m_tol3(m_tol2 * m_tol);
dgGoogol dgBooleanMeshClipper::m_tol4(m_tol2 * m_tol2);
#endif



void dgMeshEffect::ClipMesh (const dgMatrix& matrix, const dgMeshEffect* const clipMesh, dgMeshEffect** const back, dgMeshEffect** const front) const
{
	dgAssert (0);
/*
	dgMeshEffect clipper (*clipMesh);
	clipper.TransformMesh (matrix);

	dgMeshEffect* backMeshSource = NULL;
	dgMeshEffect* frontMeshSource = NULL;
	dgMeshEffect* backMeshClipper = NULL;
	dgMeshEffect* frontMeshClipper = NULL;

	ClipMesh (&clipper, &backMeshSource, &frontMeshSource);
	if (backMeshSource && frontMeshSource) {
		clipper.ClipMesh (this, &backMeshClipper, &frontMeshClipper);
		if (backMeshSource && frontMeshSource) {

			dgMeshEffect* backMesh;
			dgMeshEffect* frontMesh;

			backMesh = new (GetAllocator()) dgMeshEffect (GetAllocator(), true);
			frontMesh = new (GetAllocator()) dgMeshEffect (GetAllocator(), true);

			backMesh->BeginPolygon();
			frontMesh->BeginPolygon();

			backMesh->MergeFaces(backMeshSource);
			backMesh->MergeFaces(backMeshClipper);

			frontMesh->MergeFaces(frontMeshSource);
			frontMesh->ReverseMergeFaces(backMeshClipper);

			backMesh->EndPolygon(dgFloat64 (1.0e-5f));
			frontMesh->EndPolygon(dgFloat64 (1.0e-5f));

			*back = backMesh;
			*front = frontMesh;
		}
	}

	if (backMeshClipper) {
		delete backMeshClipper;
	}

	if (frontMeshClipper) {
		delete frontMeshClipper;
	}

	if (backMeshSource) {
		delete backMeshSource;
	}

	if (frontMeshSource) {
		delete frontMeshSource;
	}
*/
}




dgMeshEffect* dgMeshEffect::Union (const dgMatrix& matrix, const dgMeshEffect* const clipperMesh) const
{
	dgAssert (0);
	return NULL;
/*
	dgMeshEffect copy (*this);
	dgMeshEffect clipper (*clipperMesh);
	clipper.TransformMesh (matrix);

	dgBooleanMeshClipper::ClipMeshesAndColorize (&copy, &clipper);

	dgMeshEffect* const mesh = new (GetAllocator()) dgMeshEffect (GetAllocator());
	mesh->BeginFace();
	dgBooleanMeshClipper::CopyPoints(mesh, &copy);
	dgBooleanMeshClipper::AddExteriorFaces (mesh, &copy);

	dgBooleanMeshClipper::AddExteriorFaces (mesh, &clipper);
	mesh->EndFace ();
	mesh->RepairTJoints();
	mesh->RemoveUnusedVertices(NULL);
	return mesh;
*/
}

dgMeshEffect* dgMeshEffect::Difference (const dgMatrix& matrix, const dgMeshEffect* const clipperMesh) const
{
/*
	dgMeshEffect copy (*this);
	dgMeshEffect clipper (*clipperMesh);
	clipper.TransformMesh (matrix);

	dgBooleanMeshClipper::ClipMeshesAndColorize (&copy, &clipper);

	dgMeshEffect* const mesh = new (GetAllocator()) dgMeshEffect (GetAllocator());
	mesh->BeginFace();
	dgBooleanMeshClipper::CopyPoints(mesh, &copy);
	dgBooleanMeshClipper::AddExteriorFaces (mesh, &copy);
	dgBooleanMeshClipper::AddInteriorFacesInvertWinding (mesh, &clipper);
	mesh->EndFace ();
	mesh->RepairTJoints();
	mesh->RemoveUnusedVertices(NULL);
	return mesh;
*/

	dgAssert (0);
	return NULL;
}


class dgBooleanMeshClipper: public dgMeshEffect::dgMeshBVH
{
	public:

	class dgClusterFace: public dgList<dgMeshBVHNode*>
	{
		public:
		dgClusterFace(dgMeshEffect* const mesh, dgEdge* const face)
			:dgList<dgMeshBVHNode*>(mesh->GetAllocator())
			,m_plane (mesh->FaceNormal (face, mesh->GetVertexPool(), sizeof (dgBigVector)), dgFloat32(0.0))
		{
			const dgBigVector* const vertex = (dgBigVector*) mesh->GetVertexPool();
			m_plane = m_plane.Scale(1.0 / sqrt(m_plane % m_plane));
			m_plane.m_w = - (m_plane % vertex[face->m_incidentVertex]);
		}

		bool AddFace (const dgMeshEffect* const mesh, dgMeshBVHNode* const faceNode, const dgMeshEffect* const myMesh, dgMeshBVHNode* const myFaceNode) 
		{
			dgAssert (!Find(faceNode));

			bool added = false;
			dgInt32 posCount = 0;
			dgInt32 negCount = 0;
			const dgBigVector* const vertex = (dgBigVector*) mesh->GetVertexPool();
			dgEdge* ptr = faceNode->m_face;
			do {
				dgFloat64 test = m_plane.Evalue(vertex[ptr->m_incidentVertex]);
				posCount += (test > -1.0e-3);
				negCount += (test <  1.0e-3);
				ptr = ptr->m_next;
			} while (ptr != faceNode->m_face);

			if (posCount * negCount) {
				dgBigPlane plane (mesh->FaceNormal (faceNode->m_face, &vertex[0].m_x, sizeof (dgBigVector)), dgFloat32 (0.0f));
				plane = plane.Scale(1.0 / sqrt(plane % plane));
				plane.m_w = - (plane % vertex[faceNode->m_face->m_incidentVertex]);

				dgInt32 posCount = 0;
				dgInt32 negCount = 0;
				const dgBigVector* const myVertex = (dgBigVector*) myMesh->GetVertexPool();
				dgEdge* ptr = myFaceNode->m_face;
				do {
					dgFloat64 test = plane.Evalue (myVertex[ptr->m_incidentVertex]);
					posCount += (test > -1.0e-3);
					negCount += (test <  1.0e-3);
					ptr = ptr->m_next;
				} while (ptr != myFaceNode->m_face);

				if (posCount * negCount) {
					Append(faceNode);
					added = true;
				}
			}

			return added;
		}

		dgEdge* FindFirstClipperEdge (const dgHugeVector& normal, const dgHugeVector& origin, const dgBooleanMeshClipper* const clipperMeshBVH, dgInt32 clipperMark) const
		{
			dgEdge* vertexClipperEdge = NULL;
//normal.Trace();
//origin.Trace();

			for (dgListNode* clipperNode = GetFirst(); clipperNode; clipperNode = clipperNode->GetNext()) {
				dgMeshBVHNode* const clipperFaceNode = clipperNode ->GetInfo();

				dgEdge* ptr = clipperFaceNode->m_face;
//clipperMeshBVH->m_vertexAlias[ptr->m_incidentVertex].Trace();
				dgGoogol test0 (normal % (clipperMeshBVH->m_vertexAlias[ptr->m_incidentVertex] - origin));
				do {
					dgGoogol test1 (normal % (clipperMeshBVH->m_vertexAlias[ptr->m_next->m_incidentVertex] - origin));
//test0.Trace();
//test1.Trace();
					if (ptr->m_mark != clipperMark) {
						if ((test0 <= dgGoogol (m_negTol)) && (test1 >= dgGoogol (m_posTol))) {
							return ptr;
						} else if ((test0.Abs() < dgGoogol (m_posTol)) && (test1 >= dgGoogol (m_posTol))) {
							dgAssert (0);
							vertexClipperEdge = ptr;
						}
					}

					test0 = test1;
					ptr = ptr->m_next;
				} while (ptr != clipperFaceNode->m_face);
			}
			return vertexClipperEdge;
		}


		dgEdge* EmitPointOnPlaneAndGetNextCrossingEdge (const dgHugeVector& normal, const dgHugeVector& origin, const dgBooleanMeshClipper* const clipperMeshBVH, dgEdge* const clipperEdge, dgInt32 clipperMark, dgList<dgHugeVector>& curve) const
		{
			dgHugeVector& point = curve.Addtop()->GetInfo();
			point = clipperMeshBVH->m_vertexAlias[clipperEdge->m_twin->m_incidentVertex];

			dgEdge* ptr = clipperEdge;
			do {
				ptr->m_mark = clipperMark;
				ptr->m_twin->m_mark = clipperMark;
				ptr = ptr->m_next;
			} while (ptr != clipperEdge);

			dgEdge* nexEdge = NULL;
			ptr = clipperEdge->m_next->m_twin->m_next;
			do {
				ptr->m_mark = clipperMark;
				ptr->m_twin->m_mark = clipperMark;

				if (!nexEdge) {
					dgEdge* ptr1 = ptr->m_next;
					dgAssert (ptr1);
					dgGoogol test0 (normal % (clipperMeshBVH->m_vertexAlias[ptr1->m_incidentVertex] - origin));
					do {
						dgAssert (ptr1->m_next);
						dgGoogol test1 (normal % (clipperMeshBVH->m_vertexAlias[ptr1->m_next->m_incidentVertex] - origin));
						if (test0 >= m_posTol) {
							if (test1.Abs() < m_posTol) {
								nexEdge = EmitPointOnPlaneAndGetNextCrossingEdge (normal, origin, clipperMeshBVH, ptr1, clipperMark, curve);
							} else if (test1 <= m_negTol) {
								nexEdge = ptr1;
							}
						}

						test1 = test0;
						ptr1 = ptr1->m_next;
					} while (ptr1 != ptr->m_prev);
				}

				ptr = ptr->m_twin->m_next;
			} while (ptr != clipperEdge->m_next);

			dgAssert (nexEdge);
			return nexEdge;
		}


		bool Get2DCurve (const dgHugeVector& normal, const dgHugeVector& origin, const dgBooleanMeshClipper* const clipperMeshBVH, dgEdge* const clipperEdge, dgInt32 clipperMark, dgList<dgHugeVector>& curve) const
		{
			bool isClosedLoop = false;

static int xxx;
xxx ++;
if (xxx == 3){
clipperMeshBVH->m_mesh->Trace();
}

			dgInt32 clusterColor = clipperMark -1;	

			dgEdge* firstClipperEdge = clipperEdge;
			dgAssert (firstClipperEdge->m_mark >= clusterColor);

			for (bool state = true; state; ) {
				state = false;

				dgHugeVector p1p0 (clipperMeshBVH->m_vertexAlias[firstClipperEdge->m_next->m_incidentVertex] - clipperMeshBVH->m_vertexAlias[firstClipperEdge->m_incidentVertex]);
				dgGoogol num (normal % (clipperMeshBVH->m_vertexAlias[firstClipperEdge->m_incidentVertex] - origin));
				dgGoogol den (normal % p1p0);
				dgHugeVector& point = curve.Addtop()->GetInfo();
				point = clipperMeshBVH->m_vertexAlias[firstClipperEdge->m_incidentVertex] - p1p0.Scale3(num / den);

				if (firstClipperEdge->m_mark >= clusterColor) {
					firstClipperEdge->m_mark = clipperMark;
					firstClipperEdge->m_twin->m_mark = clipperMark;

					dgEdge* ptr = firstClipperEdge->m_next;
					dgGoogol test0 (normal % (clipperMeshBVH->m_vertexAlias[ptr->m_incidentVertex] - origin));
					do {
						dgGoogol test1 (normal % (clipperMeshBVH->m_vertexAlias[ptr->m_next->m_incidentVertex] - origin));
						dgAssert (ptr->m_mark >= clusterColor);
						if (test1.Abs() < m_posTol) {
							dgAssert (test0 >= m_posTol);
							state = true;
							firstClipperEdge = EmitPointOnPlaneAndGetNextCrossingEdge (normal, origin, clipperMeshBVH, ptr, clipperMark, curve);
							firstClipperEdge = firstClipperEdge->m_twin;
							break;

						} else if ((test0 >= m_posTol) && (test1 <= m_negTol)) {
							if (ptr->m_mark != clipperMark) {
								state = true;
								firstClipperEdge = ptr->m_twin;
							} else {
								isClosedLoop = true;
							}
							break;
						}

						test0 = test1;
						ptr = ptr->m_next;
					} while (ptr != firstClipperEdge);
				}
			}

			if (!isClosedLoop) {
				dgEdge* firstClipperEdge = clipperEdge->m_twin->m_next;
				for (bool state = true; state; ) {
					state = false;
					if (firstClipperEdge->m_mark >= clusterColor) {
						dgAssert (0);
/*
						firstClipperEdge->m_mark = clipperMark;
						firstClipperEdge->m_twin->m_mark = clipperMark;

						dgEdge* ptr = firstClipperEdge->m_next;
						dgGoogol test0 (normal % (clipperMeshBVH->m_vertexAlias[ptr->m_incidentVertex] - origin));
						do {
							dgGoogol test1 (normal % (clipperMeshBVH->m_vertexAlias[ptr->m_next->m_incidentVertex] - origin));
							dgAssert (ptr->m_mark >= clusterColor);
							if ((test0 >= dgGoogol (m_posTol)) && (test1 <= dgGoogol (m_negTol))) {
								if (ptr->m_mark != clipperMark) {
									state = true;
									firstClipperEdge = ptr->m_twin;
								} else {
									isClosedLoop = true;
								}
								break;
							}

							test0 = test1;
							ptr = ptr->m_next;
						} while (ptr != firstClipperEdge);

//						dgHugeVector p1p0 (clipperMeshBVH->m_vertexAlias[firstClipperEdge->m_next->m_incidentVertex] - clipperMeshBVH->m_vertexAlias[firstClipperEdge->m_incidentVertex]);
//						dgGoogol num (normal % (clipperMeshBVH->m_vertexAlias[firstClipperEdge->m_incidentVertex] - origin  ));
//						dgGoogol den (normal % p1p0);
//						dgHugeVector& point = curve.Addtop()->GetInfo();
//						point = clipperMeshBVH->m_vertexAlias[firstClipperEdge->m_incidentVertex] - p1p0.Scale(num / den);
*/
					}
				}
			}
			return isClosedLoop;
		}

		bool TestPlanar (const dgHugeVector& normal, const dgList<dgHugeVector>& curve) const
		{
			dgHugeVector n (0.0, 0.0, 0.0, 0.0);
			const dgHugeVector& p0 = curve.GetFirst()->GetInfo();
			const dgHugeVector& p1 = curve.GetFirst()->GetNext()->GetInfo();

//			p0.Trace();
//			p1.Trace();
			dgHugeVector e10 (p1 - p0);
			for (dgList<dgHugeVector>::dgListNode* node = curve.GetFirst()->GetNext()->GetNext(); node; node = node->GetNext()) {
				const dgHugeVector& p2 = node->GetInfo();
//				p2.Trace();
				dgHugeVector e20 (p2 - p0);
				n += e10 * e20;
				e10 = e20;
			}

			dgGoogol dir (n % normal);
			return dir > dgGoogol::m_zero;
		}


		void RayRayIntersect (const dgHugeVector& ray_p0, const dgHugeVector& ray_p1, dgGoogol& paramP, const dgHugeVector& ray_q0, const dgHugeVector& ray_q1, dgGoogol& paramQ) const
		{
			dgHugeVector p1p0 (ray_p1 - ray_p0);
			dgHugeVector q1q0 (ray_q1 - ray_q0);
			dgHugeVector p0q0 (ray_p0 - ray_q0);

			dgGoogol a = p1p0 % p1p0;        // always >= 0
			dgGoogol c = q1q0 % q1q0;        // always >= 0
			dgGoogol b = p1p0 % q1q0;

			dgGoogol d = (p1p0 % p0q0);
			dgGoogol e = (q1q0 % p0q0);
			dgGoogol den = a * c - b * b;   // always >= 0

			// compute the line parameters of the two closest points
			dgAssert (den > m_tol3);

			// get the closest points on the infinite lines
			paramP = (b * e - c * d) / den;
			paramQ = (a * e - b * d) / den;
			paramP = dgClamp (paramP, dgGoogol::m_zero, dgGoogol::m_one);
			paramQ = dgClamp (paramQ, dgGoogol::m_zero, dgGoogol::m_one);
		}

		void InsertOpenCurve (dgBooleanMeshClipper* const meshBVH, dgList<dgEdge*>& facePerimeter, const dgHugeVector& normal, const dgHugeVector& origin, dgList<dgHugeVector>& curve) const
		{
			dgInt32 edgeConnectCount = 0;
			dgEdge* edgeConnect[4];
			const dgHugeVector* const vertex = &meshBVH->m_vertexAlias[0];
			for (dgList<dgEdge*>::dgListNode* node = facePerimeter.GetFirst(); node; node = node->GetNext()) {
				dgEdge* const edge = node->GetInfo();

				dgInt32 index0 = edge->m_incidentVertex;
				dgInt32 index1 = edge->m_next->m_incidentVertex;
				const dgHugeVector& q0 (vertex[index0]);
				const dgHugeVector& q1 (vertex[index1]);

				for (dgList<dgHugeVector>::dgListNode* curveNode = curve.GetFirst(); curveNode != curve.GetLast(); curveNode = curveNode->GetNext()) {
					const dgHugeVector& p0 = curveNode->GetInfo();
					const dgHugeVector& p1 = curveNode->GetNext()->GetInfo();

					dgHugeVector q0p0 (q0 - p0);
					dgHugeVector q1p0 (q1 - p0);
					dgGoogol alpha0 ((normal * q0p0) % q1p0);

					dgHugeVector q0p1 (q0 - p1);
					dgHugeVector q1p1 (q1 - p1);
					dgGoogol alpha1 ((normal * q0p1) % q1p1);

					bool test0 = (alpha0 <= m_tol2);
					bool test1 = (alpha1 <= m_tol2);
					if (test0 ^ test1) {
						dgGoogol paramP;
						dgGoogol paramQ;
						RayRayIntersect (q0, q1, paramQ, p0, p1, paramP);

						bool test0ParamP = paramP >= m_posTol;
						bool test1ParamP = paramP <= m_oneMinusTol;
						bool test0ParamQ = paramQ >= m_posTol;
						bool test1ParamQ = paramQ <= m_oneMinusTol;

						if (test0ParamP & test1ParamP & test0ParamQ & test1ParamQ) {
							dgEdge* edge0;
							dgEdge* edge1;

							meshBVH->SpliteEdge(edge, paramQ, &edge0, &edge1);
							edgeConnect[edgeConnectCount] = edge1;
							edgeConnectCount ++; 

							node->GetInfo() = edge0;
							facePerimeter.InsertAfter(node, facePerimeter.Append(edge1));
							curve.InsertAfter (curveNode, curve.Append(meshBVH->m_vertexAlias[meshBVH->m_mesh->GetVertexCount()-1]));

						} else if (!((test0ParamP ^ test1ParamP) & (test0ParamQ ^ test1ParamQ))) {
							dgHugeVector p (p0 + (p1 - p0).Scale3 (paramP));
							dgHugeVector q (q0 + (q1 - q0).Scale3 (paramQ));
							dgHugeVector pq = (p - q);
							dgGoogol dist2 (pq % pq);
							if (dist2 < m_tol2) {
								if (test0ParamQ ^ test1ParamQ) {
									dgAssert (test0ParamP & test1ParamP);
									if (!test0ParamQ) {
										dgAssert (test1ParamQ);
										edgeConnect[edgeConnectCount] = edge;
										edgeConnectCount ++; 
										curve.InsertAfter (curveNode, curve.Append(meshBVH->m_vertexAlias[edge->m_next->m_incidentVertex]));
									} else {
										dgAssert (!test1ParamQ);
										edgeConnect[edgeConnectCount] = edge->m_next;
										edgeConnectCount ++; 
										curve.InsertAfter (curveNode, curve.Append(meshBVH->m_vertexAlias[edge->m_next->m_incidentVertex]));
									}
								} else {
									dgAssert (0);
									dgAssert (test0ParamP ^ test1ParamP);
									dgAssert (test0ParamQ & test1ParamQ);
								}
							}
						}

						break;
					}
				}
			}

			dgList<dgHugeVector>::dgListNode* nextNode;
			for (dgList<dgHugeVector>::dgListNode* node = curve.GetFirst(); node; node = nextNode) {
				nextNode = node->GetNext();
				const dgHugeVector& p0 = node->GetInfo();

				for (dgList<dgEdge*>::dgListNode* faceNode = facePerimeter.GetFirst(); faceNode; faceNode = faceNode->GetNext()) {
					dgEdge* const edge = faceNode->GetInfo();

					dgInt32 index0 = edge->m_incidentVertex;
					dgInt32 index1 = edge->m_next->m_incidentVertex;
					const dgHugeVector& q0 (vertex[index0]);
					const dgHugeVector& q1 (vertex[index1]);
					dgHugeVector q0p0 (q0 - p0);
					dgHugeVector q1p0 (q1 - p0);
					dgGoogol alpha0 ((normal * q1p0) % q0p0);
					if (alpha0 >= m_tol2) {
						curve.Remove(node);
						break;
					}
				}
			}


			dgAssert (edgeConnectCount == 2);
			if (curve.GetCount() == 2) {
				meshBVH->m_mesh->ConnectVertex(edgeConnect[0], edgeConnect[1]);
			} else {
				dgAssert (0);
			}
		}


		void InsertClosedCurve (dgBooleanMeshClipper* const meshBVH, dgList<dgEdge*>& facePerimeter, const dgHugeVector& normal, const dgHugeVector& origin, const dgList<dgHugeVector>& clipperCurve) const
		{
			dgAssert (TestPlanar (normal, clipperCurve));

			bool isInside = true;
			const dgHugeVector* const vertex = &meshBVH->m_vertexAlias[0];
			for (dgList<dgEdge*>::dgListNode* node = facePerimeter.GetFirst(); isInside && node; node = node->GetNext()) {
				dgEdge* const edge = node->GetInfo();
				dgInt32 index0 = edge->m_incidentVertex;
				dgInt32 index1 = edge->m_next->m_incidentVertex;

				const dgHugeVector& q0 (vertex[index0]);
				const dgHugeVector& q1 (vertex[index1]);
				for (dgList<dgHugeVector>::dgListNode* clipperNode = clipperCurve.GetFirst(); clipperNode; clipperNode = clipperNode->GetNext()) {
					const dgHugeVector& p0 = clipperNode->GetInfo();
					dgHugeVector q0p0 (q0 - p0);
					dgHugeVector q1p0 (q1 - p0);
					dgGoogol alpha ((normal * q0p0) % q1p0);
					if (alpha <= m_tol2) {
						isInside = false;
						break;
					}
				}
			} 

			if (isInside) {
				dgInt32 face[1024];
				dgInt64 attrb[1024];

				dgInt32 indexCount = 0;
				dgEdge* const exteriorFace = facePerimeter.GetFirst()->GetInfo();
				for (dgList<dgHugeVector>::dgListNode* clipperNode = clipperCurve.GetFirst(); clipperNode; clipperNode = clipperNode->GetNext()) {
					meshBVH->InsertFaceVertex (exteriorFace, clipperNode->GetInfo());
					face[indexCount] = meshBVH->m_mesh->GetVertexCount() - 1;
					attrb[indexCount] = meshBVH->m_mesh->GetPropertiesCount() - 1;
					indexCount ++;
				}
				dgEdge* const interiorFace = meshBVH->m_mesh->AddFace(indexCount, face, attrb);
				dgEdge* ptr = interiorFace;
				do {
					ptr->m_twin = meshBVH->m_mesh->AddHalfEdge(ptr->m_next->m_incidentVertex, ptr->m_incidentVertex);
					dgAssert (ptr->m_twin);
					ptr->m_twin->m_twin = ptr;
					ptr = ptr->m_next;
				} while (ptr != interiorFace);

				ptr = interiorFace;
				do {
					ptr->m_twin->m_incidentFace = ptr->m_incidentFace;
					ptr->m_twin->m_userData = ptr->m_next->m_userData;

					ptr->m_twin->m_prev = ptr->m_next->m_twin;
					ptr->m_next->m_twin->m_next = ptr->m_twin;

					ptr = ptr->m_next;
				} while (ptr != interiorFace);

				dgGoogol error2 (1.0e20);
				dgEdge* closestEdge = NULL;
				const dgHugeVector* const vertex = &meshBVH->m_vertexAlias[0];
				const dgHugeVector& p0 = vertex[exteriorFace->m_incidentVertex];
				ptr = interiorFace;
				do {
					const dgHugeVector& q0 = vertex[ptr->m_incidentVertex];
					dgHugeVector dist (q0 - p0);
					dgGoogol mag2 (dist % dist);

					if (mag2 < error2) {
						error2 = mag2;
						closestEdge = ptr;
					}

					ptr->m_twin->m_incidentFace = interiorFace->m_incidentFace;
					ptr = ptr->m_next;
				} while (ptr != interiorFace);

				dgAssert (closestEdge);
				closestEdge = closestEdge->m_twin->m_next;
				closestEdge = meshBVH->m_mesh->ConnectVertex (exteriorFace, closestEdge);
				dgAssert (closestEdge);

				meshBVH->m_mesh->PolygonizeFace(interiorFace, meshBVH->m_mesh->GetVertexPool(), sizeof (dgBigVector));
				meshBVH->m_mesh->PolygonizeFace(closestEdge, meshBVH->m_mesh->GetVertexPool(), sizeof (dgBigVector));

			} else {
				dgAssert (0);
			}
		}



		bool ClipFace (dgMeshBVHNode* const faceNode, dgBooleanMeshClipper* const meshBVH, const dgBooleanMeshClipper* const clipperMeshBVH)
		{
			dgHugeVector normal (meshBVH->FaceNormal (faceNode->m_face));
			dgGoogol mag2 (normal % normal);
			normal = normal.Scale3 (mag2.InvSqrt());
			const dgHugeVector& origin (meshBVH->m_vertexAlias[faceNode->m_face->m_incidentVertex]);

			dgClusterFace::dgListNode* nextNode;
			dgInt32 clusterColor = clipperMeshBVH->m_mesh->IncLRU();
			for (dgClusterFace::dgListNode* node = GetFirst(); node; node = nextNode) {
				dgMeshBVHNode* const clipperFaceNode = node->GetInfo();
				nextNode = node->GetNext();

				dgInt32 posCount = 0;
				dgInt32 negCount = 0;
				dgInt32 onPlaneCount = 0;
				dgEdge* ptr = clipperFaceNode->m_face;
				do {
					ptr->m_mark = clusterColor;
					dgGoogol test (normal % (clipperMeshBVH->m_vertexAlias[ptr->m_incidentVertex] - origin));
					bool pos = (test > dgBooleanMeshClipper::m_posTol);
					bool neg = (test < dgBooleanMeshClipper::m_negTol);
					posCount += pos;
					negCount += neg;
					onPlaneCount += !(pos || neg);
					ptr = ptr->m_next;
				} while (ptr != clipperFaceNode->m_face);


				if (!((posCount && negCount) || (posCount && onPlaneCount) || (negCount && onPlaneCount))) {
					dgAssert (0);
					Remove(node);
				}
			}


			if (GetCount()) {
				dgList<dgEdge*> facePerimeter (clipperMeshBVH->m_mesh->GetAllocator());
				dgEdge* perimterEdge = faceNode->m_face;
				do {
					facePerimeter.Append(perimterEdge);
					perimterEdge = perimterEdge->m_next;
				} while (perimterEdge != faceNode->m_face);

				dgPolyhedra::dgPairKey key (faceNode->m_face->m_incidentVertex, faceNode->m_face->m_twin->m_incidentVertex);

				dgInt32 clipperMark = clipperMeshBVH->m_mesh->IncLRU();
				for (bool found = true; found;) {
					found = false;
					dgList<dgHugeVector> clipperCurve (clipperMeshBVH->m_mesh->GetAllocator());

					dgEdge* const firstClipperEdge = FindFirstClipperEdge (normal, origin, clipperMeshBVH, clipperMark);
					if (firstClipperEdge) {
						dgAssert (firstClipperEdge->m_mark == clusterColor);
						found = true;
						bool isClosedLoop = Get2DCurve (normal, origin, clipperMeshBVH, firstClipperEdge, clipperMark, clipperCurve);
						if (clipperCurve.GetCount() >= 2) {
							if (isClosedLoop) {
								InsertClosedCurve (meshBVH, facePerimeter, normal, origin, clipperCurve);
							} else {
								InsertOpenCurve (meshBVH, facePerimeter, normal, origin, clipperCurve);
							}
						}
					}
				}

				meshBVH->m_nodeEdgeMap.Remove(key.GetVal());
				meshBVH->RemoveNode(faceNode);

				dgInt32 stack = 0;
				dgEdge* stackPool[512];
				dgInt32 mark = meshBVH->m_mesh->IncLRU();
				for (dgList<dgEdge*>::dgListNode* node = facePerimeter.GetFirst(); node; node = node->GetNext()) {
					dgEdge* const edge = node->GetInfo();
					edge->m_twin->m_mark = mark;
					stackPool[stack] = edge;
					stack ++;
				}

				while (stack) {
					stack --;
					dgEdge* const face = stackPool[stack];
					if (face->m_mark != mark) {
						dgMeshBVHNode* const newNode = meshBVH->AddFaceNode (face, NULL);
						dgPolyhedra::dgPairKey key(newNode->m_face->m_incidentVertex, newNode->m_face->m_twin->m_incidentVertex);
						meshBVH->m_nodeEdgeMap.Insert(newNode, key.GetVal());
					}

					dgEdge* ptr = face;
					do {
						ptr->m_mark = mark;
						if (ptr->m_twin->m_mark != mark) {
							stackPool[stack] = ptr->m_twin;
							stack ++;
						}
						ptr = ptr->m_next;
					} while (ptr != face);
				}
			}

			return false;
		}


		dgBigPlane m_plane;
	};

	class dgBooleanFaceCluster: public dgTree<dgClusterFace, dgMeshBVHNode*>
	{
		public:
		dgBooleanFaceCluster (dgMemoryAllocator* const allocator)
			:dgTree<dgClusterFace, dgMeshBVHNode*>(allocator)
		{
		}

		bool ClipCluster (dgTreeNode* const clusterNode, dgBooleanMeshClipper* const meshBVH, const dgBooleanMeshClipper* const clipperMeshBVH)
		{
			dgClusterFace& cluster = clusterNode->GetInfo();
			dgMeshBVHNode* const faceNode = clusterNode->GetKey();
			bool ret = cluster.ClipFace(faceNode, meshBVH, clipperMeshBVH);
			Remove (clusterNode);
			return ret;
		}
	};


	dgBooleanMeshClipper (dgMeshEffect* const mesh) 
		:dgMeshBVH (mesh)
		,m_vertexBase(mesh->GetVertexCount())
		,m_vertexAlias(mesh->GetVertexCount() + 512, mesh->GetAllocator())
		,m_nodeEdgeMap(mesh->GetAllocator())
	{
		Build ();
	}

	~dgBooleanMeshClipper () 
	{
	}


	void Build ()
	{
		dgMeshBVH::Build();

		const dgBigVector* const points = (dgBigVector*) m_mesh->GetVertexPool(); 
		for (dgInt32 i = 0; i < m_mesh->GetVertexCount(); i ++) {
			m_vertexAlias[i] = dgHugeVector (points[i]);
		}


		dgMeshBVHNode* stackPool[DG_MESH_EFFECT_BVH_STACK_DEPTH];
		dgInt32 stack = 1;
		stackPool[0] = m_rootNode;
		dgVector l0(dgFloat32 (-1.0e15f), dgFloat32 (-1.0e15f), dgFloat32 (-1.0e15f), dgFloat32 (0.0f)); 
		dgVector l1(dgFloat32 (1.0e15f), dgFloat32 ( 1.0e15f), dgFloat32 ( 1.0e15f), dgFloat32 (0.0f)); 

		while (stack) {
			stack --;
			dgMeshBVHNode* const me = stackPool[stack];
			if (me && dgOverlapTest (me->m_p0, me->m_p1, l0, l1)) {
				if (!me->m_left) {
					dgAssert (!me->m_right);
					dgPolyhedra::dgPairKey key (me->m_face->m_incidentVertex, me->m_face->m_twin->m_incidentVertex);
					m_nodeEdgeMap.Insert(me, key.GetVal());
				} else {
					dgAssert (me->m_left);
					dgAssert (me->m_right);
					stackPool[stack] = me->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack] = me->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
				}
			}
		}
		dgAssert (SanityCheck());
	}

	void Cleanup ()
	{
		dgMeshBVH::Cleanup();
	}


	bool SanityCheck() const
	{
		#ifdef _DEBUG
			dgAssert (dgMeshBVH::SanityCheck()); 
			dgMeshBVHNode* stackPool[DG_MESH_EFFECT_BVH_STACK_DEPTH];
			dgInt32 stack = 1;
			stackPool[0] = m_rootNode;
			dgVector l0(dgFloat32 (-1.0e15f), dgFloat32 (-1.0e15f), dgFloat32 (-1.0e15f), dgFloat32 (0.0f)); 
			dgVector l1(dgFloat32 (1.0e15f), dgFloat32 ( 1.0e15f), dgFloat32 ( 1.0e15f), dgFloat32 (0.0f)); 

			while (stack) {
				stack --;
				dgMeshBVHNode* const me = stackPool[stack];
				if (me && dgOverlapTest (me->m_p0, me->m_p1, l0, l1)) {
					if (!me->m_left) {
						dgAssert (!me->m_right);
						dgPolyhedra::dgPairKey key (me->m_face->m_incidentVertex, me->m_face->m_twin->m_incidentVertex);
						dgTree<dgMeshBVHNode*, dgUnsigned64>::dgTreeNode* const node = m_nodeEdgeMap.Find(key.GetVal());
						dgAssert (node);
						dgAssert (node->GetInfo() == me);
					} else {
						dgAssert (me->m_left);
						dgAssert (me->m_right);
						stackPool[stack] = me->m_left;
						stack++;
						dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

						stackPool[stack] = me->m_right;
						stack++;
						dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
					}
				}
			}
		#endif
		return true;
	}


	dgHugeVector FaceNormal (const dgEdge* const face) const
	{
		const dgEdge* edge = face;
		const dgHugeVector& p0 = m_vertexAlias[edge->m_incidentVertex];

		edge = edge->m_next;
		const dgHugeVector& p1 = m_vertexAlias[edge->m_incidentVertex];
		dgHugeVector e1 (p1 - p0);

		dgHugeVector normal (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		for (edge = edge->m_next; edge != face; edge = edge->m_next) {
			const dgHugeVector& p2 = m_vertexAlias[edge->m_incidentVertex]; 
			dgHugeVector e2 (p2 - p0);
			normal += e1 * e2;
			e1 = e2;
		} 
		return normal;
	}

	void InsertFaceVertex (dgEdge* const face, const dgHugeVector& point)
	{
		dgMeshEffect::dgVertexAtribute attribute;
		memset (&attribute, 0, sizeof (attribute));

		dgGoogol tol(m_posTol);
		const dgMeshEffect::dgVertexAtribute* const attrib = (dgMeshEffect::dgVertexAtribute*)m_mesh->GetAttributePool(); 
		for (dgInt32 i = 0; i < 4; i ++) {

			dgGoogol posTol (tol + dgGoogol::m_one);
			dgGoogol negTol (dgGoogol::m_zero - m_posTol);

			dgEdge* ptr = face;
			dgEdge* const edge0 = ptr;
			dgHugeVector q0 (m_vertexAlias[ptr->m_incidentVertex]);

			ptr = ptr->m_next;
			const dgEdge* edge1 = ptr;
			dgHugeVector q1 (m_vertexAlias[ptr->m_incidentVertex]);

			ptr = ptr->m_next;
			const dgEdge* edge2 = ptr;
			do {
				const dgHugeVector& q2 = m_vertexAlias[ptr->m_incidentVertex];

				dgHugeVector p10 (q1 - q0);
				dgHugeVector p20 (q2 - q0);

				dgGoogol dot = p20 % p10;
				dgGoogol mag1 = p10 % p10;
				dgGoogol mag2 = p20 % p20;
				dgGoogol collinear = dot * dot - mag2 * mag1;
				if (collinear.Abs() > m_posTol) {
					dgHugeVector p_p0 (point - q0);
					dgHugeVector p_p1 (point - q1);
					dgHugeVector p_p2 (point - q2);

					dgGoogol alpha1 = p10 % p_p0;
					dgGoogol alpha2 = p20 % p_p0;
					dgGoogol alpha3 = p10 % p_p1;
					dgGoogol alpha4 = p20 % p_p1;
					dgGoogol alpha5 = p10 % p_p2;
					dgGoogol alpha6 = p20 % p_p2;

					dgGoogol vc = alpha1 * alpha4 - alpha3 * alpha2;
					dgGoogol vb = alpha5 * alpha2 - alpha1 * alpha6;
					dgGoogol va = alpha3 * alpha6 - alpha5 * alpha4;
					dgGoogol den = va + vb + vc;
					dgGoogol minError = den * negTol;
					dgGoogol maxError = den * posTol;

					if ((va > minError) && (vb > minError) && (vc > minError) && (va < maxError) && (vb < maxError) && (vc < maxError)) {

						edge2 = ptr;

						dgGoogol alpha0 = va / den;
						dgGoogol alpha1 = vb / den;
						dgGoogol alpha2 = vc / den;

						dgFloat64 falpha0 = alpha0;
						dgFloat64 falpha1 = alpha1;
						dgFloat64 falpha2 = alpha2;

						const dgMeshEffect::dgVertexAtribute& attr0 = attrib[edge0->m_userData];
						const dgMeshEffect::dgVertexAtribute& attr1 = attrib[edge1->m_userData];
						const dgMeshEffect::dgVertexAtribute& attr2 = attrib[edge2->m_userData];
						dgBigVector normal (attr0.m_normal_x * falpha0 + attr1.m_normal_x * falpha1 + attr2.m_normal_x * falpha2,
											attr0.m_normal_y * falpha0 + attr1.m_normal_y * falpha1 + attr2.m_normal_y * falpha2,
											attr0.m_normal_z * falpha0 + attr1.m_normal_z * falpha1 + attr2.m_normal_z * falpha2, dgFloat32 (0.0f));
						normal = normal.Scale3 (dgFloat64 (1.0f) / sqrt (normal % normal));


						attribute.m_vertex.m_x = point.m_x;
						attribute.m_vertex.m_y = point.m_y;
						attribute.m_vertex.m_z = point.m_z;
						attribute.m_vertex.m_w = point.m_w;
						attribute.m_normal_x = normal.m_x;
						attribute.m_normal_y = normal.m_y;
						attribute.m_normal_z = normal.m_z;
						attribute.m_u0 = attr0.m_u0 * falpha0 +  attr1.m_u0 * falpha1 + attr2.m_u0 * falpha2;
						attribute.m_v0 = attr0.m_v0 * falpha0 +  attr1.m_v0 * falpha1 + attr2.m_v0 * falpha2;
						attribute.m_u1 = attr0.m_u1 * falpha0 +  attr1.m_u1 * falpha1 + attr2.m_u1 * falpha2;
						attribute.m_v1 = attr0.m_v1 * falpha0 +  attr1.m_v1 * falpha1 + attr2.m_v1 * falpha2;

						attribute.m_material = attr0.m_material;
						dgAssert (attr0.m_material == attr1.m_material);
						dgAssert (attr0.m_material == attr2.m_material);

						m_mesh->AddPoint(&attribute.m_vertex.m_x, dgInt32 (attribute.m_material)); 
						m_vertexAlias[m_mesh->GetVertexCount()-1] = point;
						return;
					}
				}

				q1 = q2;
				edge1 = ptr;

				ptr = ptr->m_next;
			} while (ptr != face);
			dgAssert (0);
			tol = tol * dgGoogol::m_two;
		}
		// this should never happens
		dgAssert (0);
	}

	void SpliteEdge(dgEdge* const edge, dgGoogol param, dgEdge** edge0, dgEdge** edge1)
	{
		dgInt32 v0 = edge->m_incidentVertex;
		dgInt32 v1 = edge->m_twin->m_incidentVertex;
		dgPolyhedra::dgPairKey edgeKey (v0, v1);
		dgPolyhedra::dgPairKey twinKey (v1, v0);

		dgEdge* const newEdge = m_mesh->InsertEdgeVertex (edge, param);
		dgGoogol t0(dgGoogol::m_one - param);
		dgGoogol t1(param);
		dgInt32 v01 = newEdge->m_twin->m_incidentVertex;
		m_vertexAlias[v01] = m_vertexAlias[v0].Scale3 (t0) + m_vertexAlias[v1].Scale3 (t1);

		dgTree<dgMeshBVHNode*, dgUnsigned64>::dgTreeNode* const mapNode = m_nodeEdgeMap.Find(edgeKey.GetVal());
		if (mapNode) {
			dgMeshBVHNode* const node = mapNode->GetInfo();
			node->m_face = newEdge;
			m_nodeEdgeMap.Remove(mapNode);
			dgPolyhedra::dgPairKey key (newEdge->m_incidentVertex, newEdge->m_twin->m_incidentVertex);
			m_nodeEdgeMap.Insert(node, key.GetVal());
		}

		dgTree<dgMeshBVHNode*, dgUnsigned64>::dgTreeNode* const twinMapNode = m_nodeEdgeMap.Find(twinKey.GetVal());
		if (twinMapNode) {
			dgMeshBVHNode* const node = twinMapNode->GetInfo();
			node->m_face = newEdge->m_twin;
			m_nodeEdgeMap.Remove(twinMapNode);
			dgPolyhedra::dgPairKey key (newEdge->m_twin->m_incidentVertex, newEdge->m_incidentVertex);
			m_nodeEdgeMap.Insert(node, key.GetVal());
		}

		*edge0 = newEdge;
		*edge1 = newEdge->m_next;

		dgAssert (SanityCheck());
	}



	bool ClippMesh (const dgBooleanMeshClipper& otherMeshBVH)
	{
		dgMeshBVHNode* stackPool[4 * DG_MESH_EFFECT_BVH_STACK_DEPTH][2];
	

		dgInt32 stack = 1;
		stackPool[0][0] = m_rootNode;
		stackPool[0][1] = otherMeshBVH.m_rootNode;

		dgBooleanFaceCluster bundaryClusters (m_mesh->GetAllocator());
		while (stack) {
			stack --;
			dgMeshBVHNode* const me = stackPool[stack][0];
			dgMeshBVHNode* const other = stackPool[stack][1];

			dgAssert (me && other);
			if (dgOverlapTest (me->m_p0, me->m_p1, other->m_p0, other->m_p1)) {

				if (!me->m_left && !other->m_left) {
					dgAssert (!me->m_right);
					dgAssert (!other->m_right);

					dgBooleanFaceCluster::dgTreeNode* node = bundaryClusters.Find(me);
					if (!node) {
						dgClusterFace tmp (m_mesh, me->m_face);
						node = bundaryClusters.Insert(tmp, me);
					}
					if (!node->GetInfo().AddFace (otherMeshBVH.m_mesh, other, m_mesh, me)) {
						if (!node->GetInfo().GetCount()) {
							bundaryClusters.Remove(node);
						}
					}

				} else if (!me->m_left) {
					dgAssert (other->m_left);
					dgAssert (other->m_right);

					stackPool[stack][0] = me;
					stackPool[stack][1] = other->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me;
					stackPool[stack][1] = other->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

				} else if (!other->m_right) {
					dgAssert (me->m_left);
					dgAssert (me->m_right);

					stackPool[stack][0] = me->m_left;
					stackPool[stack][1] = other;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_right;
					stackPool[stack][1] = other;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
				} else {
					dgAssert (me->m_left && me->m_right);
					dgAssert (other->m_left && other->m_right);

					stackPool[stack][0] = me->m_left;
					stackPool[stack][1] = other->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_left;
					stackPool[stack][1] = other->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_right;
					stackPool[stack][1] = other->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_right;
					stackPool[stack][1] = other->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
				}
			}
		}

		bool intersectionFound = false;
		while (bundaryClusters.GetCount()) {
			intersectionFound |= bundaryClusters.ClipCluster (bundaryClusters.GetRoot(), this, &otherMeshBVH);
		}
		ImproveNodeFitness();
		return intersectionFound;
	}



	static void ClipMeshesAndColorize (dgMeshEffect* const meshA, dgMeshEffect* const meshB)
	{
		dgBooleanMeshClipper BVHmeshA (meshA);
		dgBooleanMeshClipper BVHmeshB (meshB);


		BVHmeshA.ClippMesh (BVHmeshB);
BVHmeshA.m_mesh->SaveOFF("xxxA0.off");
		BVHmeshB.ClippMesh (BVHmeshA);
BVHmeshB.m_mesh->SaveOFF("xxxB0.off");
		BVHmeshA.ClippMesh (BVHmeshB);
		dgAssert (!BVHmeshB.ClippMesh (BVHmeshA));
		dgAssert (!BVHmeshA.ClippMesh (BVHmeshB));
		
	}


	dgInt32 m_vertexBase;
	dgArray<dgHugeVector> m_vertexAlias;
//	dgTree<dgInt32,dgInt32> m_vertexMap;
	dgTree<dgMeshBVHNode*, dgUnsigned64> m_nodeEdgeMap;

	static dgGoogol m_posTol;
	static dgGoogol m_negTol;
	static dgGoogol m_tol2;
	static dgGoogol m_tol3;
	static dgGoogol m_oneMinusTol;
};

dgGoogol dgBooleanMeshClipper::m_posTol ( DG_BOOLEAN_ZERO_TOLERANCE);
dgGoogol dgBooleanMeshClipper::m_negTol (-DG_BOOLEAN_ZERO_TOLERANCE);
dgGoogol dgBooleanMeshClipper::m_tol2 (m_posTol * m_posTol);
dgGoogol dgBooleanMeshClipper::m_tol3 (m_posTol * m_tol2);
dgGoogol dgBooleanMeshClipper::m_oneMinusTol (dgGoogol(1.0) - m_posTol);


dgMeshEffect* dgMeshEffect::Intersection (const dgMatrix& matrix, const dgMeshEffect* const clipperMesh) const
{
	dgMeshEffect copy (*this);
	dgMeshEffect clipper (*clipperMesh);
	clipper.TransformMesh (matrix);

	dgBooleanMeshClipper::ClipMeshesAndColorize (&copy, &clipper);
/*
	dgMeshEffect* const mesh = new (GetAllocator()) dgMeshEffect (GetAllocator());
	mesh->BeginFace();
	dgBooleanMeshClipper::CopyPoints(mesh, &copy);
	dgBooleanMeshClipper::AddInteriorFaces (mesh, &copy);
	dgBooleanMeshClipper::AddInteriorFaces (mesh, &clipper);
	mesh->EndFace ();
	mesh->RepairTJoints();
	mesh->RemoveUnusedVertices(NULL);

	return mesh;
*/

	dgAssert (0);
	return NULL;
}


bool dgMeshEffect::PlaneClip (const dgMeshEffect& convexMesh, const dgEdge* const convexFace)
{
	dgAssert (convexFace->m_incidentFace > 0);
	
	dgBigVector normal (convexMesh.FaceNormal(convexFace, &convexMesh.m_points[0].m_x, sizeof(dgBigVector)));
	dgFloat64 mag2 = normal % normal;
	if (mag2 < dgFloat64 (1.0e-30)) {
		dgAssert (0);
		return true;
	}  

	normal = normal.Scale3(dgFloat64 (1.0f) / sqrt (mag2));
	dgBigVector origin (convexMesh.m_points[convexFace->m_incidentVertex]);
	dgBigPlane plane (normal, - (origin % normal));
/*
static int xxxx;
static int xxx;
xxx ++;
int xxxxxxxxxx = 2028;
if (xxx == xxxxxxxxxx){

char xxxx[128];
sprintf (xxxx, "xxx__%d.off", xxx) ;

SaveOFF(xxxx);
dgInt32 xxxxx = IncLRU();
dgPolyhedra::Iterator iter (*this);
for (iter.Begin(); iter; iter ++) {
	dgEdge* const edge = &(*iter);

	if (edge->m_mark != xxxxx) {
		dgEdge* end = edge;
		do {
			end->m_mark = xxxxx;
			dgTrace (("%d ", end->m_incidentVertex));
			end = end->m_next;
		} while (end != edge);
		dgTrace (("\n"));
	}
}
}
*/
	dgAssert (!HasOpenEdges());
	
	dgInt32 pointCount = GetVertexCount();
	dgStack <dgFloat64> testPool (2 * pointCount + 1024);
	dgFloat64* const test = &testPool[0];
	for (dgInt32 i = 0; i < pointCount; i ++) {
		test[i] = plane.Evalue (m_points[i]);
		if (fabs (test[i]) < dgFloat32 (1.0e-5f)) {
			test[i] = dgFloat32 (0.0f);
		}
	}

	dgInt32 positive = 0;
	dgInt32 negative = 0;
	dgPolyhedra::Iterator iter (*this); 
	for (iter.Begin(); iter && !(positive && negative); iter ++){
		dgEdge* const edge = &(*iter);
		positive += test[edge->m_incidentVertex] > dgFloat32 (0.0f);
		negative += test[edge->m_incidentVertex] < dgFloat32 (0.0f);
	}
	if (positive  && !negative) {
		return false;
	}
	
	if (positive && negative) {

        const dgEdge* e0 = convexFace;
        const dgEdge* e1 = e0->m_next;
        const dgEdge* e2 = e1->m_next;

		dgMatrix matrix;
		dgBigVector p1 (convexMesh.m_points[e1->m_incidentVertex]);

		dgBigVector xDir (p1 - origin);
        dgAssert ((xDir % xDir) > dgFloat32 (0.0f));
		matrix[2] = dgVector (normal);
		matrix[0] = dgVector(xDir.Scale3(dgFloat64 (1.0f) / sqrt (xDir% xDir)));
		matrix[1] = matrix[2] * matrix[0];
		matrix[3] = dgVector (origin);
		matrix[3][3] = dgFloat32 (1.0f);

		dgVector q0 (matrix.UntransformVector(dgVector(convexMesh.m_points[e0->m_incidentVertex])));
        dgVector q1 (matrix.UntransformVector(dgVector(convexMesh.m_points[e1->m_incidentVertex])));
        dgVector q2 (matrix.UntransformVector(dgVector(convexMesh.m_points[e2->m_incidentVertex])));

        dgVector p10 (q1 - q0);
        dgVector p20 (q2 - q0);
        dgVector faceNormal (matrix.UnrotateVector (dgVector(normal)));
        dgFloat32 areaInv = (p10 * p20) % faceNormal;
        if (e2->m_next != e0) {
            const dgEdge* edge = e2;
            dgVector r1 (q2);
            dgVector p10 (p20);
            do {
                dgVector r2 (matrix.UntransformVector(dgVector(convexMesh.m_points[edge->m_next->m_incidentVertex])));
                dgVector p20 (r2 - q0);
                dgFloat32 areaInv1 = (p10 * p20) % faceNormal;
                if (areaInv1 > areaInv) {
                    e1 = edge;
                    e2 = edge->m_next;
                    q1 = r1;
                    q2 = r2;
                    areaInv = areaInv1;
                }
                r1 = r2;
                p10 = p20;
                edge = edge->m_next;
            } while (edge->m_next != e0);
        }
		
        dgAssert (areaInv > dgFloat32 (0.0f));
        areaInv = dgFloat32 (1.0f) / areaInv;

		dgVector uv0_0 (dgFloat32 (convexMesh.m_attrib[e0->m_userData].m_u0), dgFloat32 (convexMesh.m_attrib[e0->m_userData].m_v0), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgVector uv0_1 (dgFloat32 (convexMesh.m_attrib[e1->m_userData].m_u0), dgFloat32 (convexMesh.m_attrib[e1->m_userData].m_v0), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgVector uv0_2 (dgFloat32 (convexMesh.m_attrib[e2->m_userData].m_u0), dgFloat32 (convexMesh.m_attrib[e2->m_userData].m_v0), dgFloat32 (0.0f), dgFloat32 (0.0f));

		dgVector uv1_0 (dgFloat32 (convexMesh.m_attrib[e0->m_userData].m_u1), dgFloat32 (convexMesh.m_attrib[e0->m_userData].m_v1), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgVector uv1_1 (dgFloat32 (convexMesh.m_attrib[e1->m_userData].m_u1), dgFloat32 (convexMesh.m_attrib[e1->m_userData].m_v1), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgVector uv1_2 (dgFloat32 (convexMesh.m_attrib[e2->m_userData].m_u1), dgFloat32 (convexMesh.m_attrib[e2->m_userData].m_v1), dgFloat32 (0.0f), dgFloat32 (0.0f));

		for (iter.Begin(); iter; iter ++){
			dgEdge* const edge = &(*iter);
			
			dgFloat64 side0 = test[edge->m_prev->m_incidentVertex];
			dgFloat64 side1 = test[edge->m_incidentVertex];

			if ((side0 < dgFloat32 (0.0f)) && (side1 > dgFloat64 (0.0f))) {
				dgBigVector dp (m_points[edge->m_incidentVertex] - m_points[edge->m_prev->m_incidentVertex]);
				dgFloat64 param = - side0 / (plane % dp);

				dgEdge* const splitEdge = InsertEdgeVertex (edge->m_prev, param);
				test[splitEdge->m_next->m_incidentVertex] = dgFloat64 (0.0f);
			} 
		}

/*
if (xxx == xxxxxxxxxx){
dgInt32 xxxxx = IncLRU();
dgPolyhedra::Iterator iter (*this);
for (iter.Begin(); iter; iter ++) {
	dgEdge* const edge = &(*iter);

	if (edge->m_mark != xxxxx) {
		dgEdge* end = edge;
		do {
			end->m_mark = xxxxx;
			dgTrace (("%d ", end->m_incidentVertex));
			end = end->m_next;
		} while (end != edge);
		dgTrace (("\n"));
	}
}
}
*/

		dgInt32 colorMark = IncLRU();
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &(*iter);
			dgFloat64 side0 = test[edge->m_incidentVertex];
			dgFloat64 side1 = test[edge->m_next->m_incidentVertex];

			if ((side0 > dgFloat32 (0.0f)) || (side1 > dgFloat64 (0.0f))) {
				edge->m_mark = colorMark;
			}
		}

		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &(*iter);
			dgFloat64 side0 = test[edge->m_incidentVertex];
			dgFloat64 side1 = test[edge->m_next->m_incidentVertex];
			if ((side0 == dgFloat32 (0.0f)) && (side1 == dgFloat64 (0.0f))) {
				dgEdge* ptr = edge->m_next;
				do {
					if (ptr->m_mark == colorMark) {
						edge->m_mark = colorMark;
						break;
					}
					ptr = ptr->m_next;
				} while (ptr != edge);
			}
		}


		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &(*iter);
			if ((edge->m_mark == colorMark) && (edge->m_next->m_mark < colorMark)) {
				dgEdge* const startEdge = edge->m_next;
				dgEdge* end = startEdge;
				do {
					if (end->m_mark == colorMark) {
						break;
					}

					end = end->m_next;
				} while (end != startEdge);
				dgAssert (end != startEdge);
				dgEdge* const devideEdge = ConnectVertex (startEdge, end);
				dgAssert (devideEdge);
				dgAssert (devideEdge->m_next->m_mark != colorMark);
				dgAssert (devideEdge->m_prev->m_mark != colorMark);
				dgAssert (devideEdge->m_twin->m_next->m_mark == colorMark);
				dgAssert (devideEdge->m_twin->m_prev->m_mark == colorMark);
				devideEdge->m_mark = colorMark - 1;
				devideEdge->m_twin->m_mark = colorMark;
			}
		}

		dgInt32 mark = IncLRU();
		dgList<dgEdge*> faceList (GetAllocator());
		for (iter.Begin(); iter; iter ++){
			dgEdge* const face = &(*iter);
			if ((face->m_mark >= colorMark) && (face->m_mark != mark)) {
				faceList.Append(face);
				dgEdge* edge = face;
				do {
					edge->m_mark = mark;
					edge = edge->m_next;
				} while (edge != face);
			}
		}

		for (dgList<dgEdge*>::dgListNode* node = faceList.GetFirst(); node; node = node->GetNext()) {
			dgEdge* const face = node->GetInfo();
			DeleteFace(face);
		}

		mark = IncLRU();
		faceList.RemoveAll();
		for (iter.Begin(); iter; iter ++){
			dgEdge* const face = &(*iter);
			if ((face->m_mark != mark) && (face->m_incidentFace < 0)) {
				faceList.Append(face);
				dgEdge* edge = face;
				do {
					edge->m_mark = mark;
					edge = edge->m_next;
				} while (edge != face);
			}
		}

		const dgFloat64 capAttribute = convexMesh.m_attrib[convexFace->m_userData].m_material;
		for (dgList<dgEdge*>::dgListNode* node = faceList.GetFirst(); node; node = node->GetNext()) {
			dgEdge* const face = node->GetInfo();

			dgEdge* edge = face;
			do {
				dgVertexAtribute attibute;
				attibute.m_vertex = m_points[edge->m_incidentVertex];
				attibute.m_normal_x = normal.m_x;
				attibute.m_normal_y = normal.m_y;
				attibute.m_normal_z = normal.m_z;
				attibute.m_material = capAttribute;

				dgVector p (matrix.UntransformVector (attibute.m_vertex));

				dgVector p_p0 (p - q0);
				dgVector p_p1 (p - q1);
				dgVector p_p2 (p - q2);
				//dgFloat32 alpha1 = p10 % p_p0;
				//dgFloat32 alpha2 = p20 % p_p0;
				//dgFloat32 alpha3 = p10 % p_p1;
				//dgFloat32 alpha4 = p20 % p_p1;
				//dgFloat32 alpha5 = p10 % p_p2;
				//dgFloat32 alpha6 = p20 % p_p2;
				//dgFloat32 vc = alpha1 * alpha4 - alpha3 * alpha2;
				//dgFloat32 vb = alpha5 * alpha2 - alpha1 * alpha6;
				//dgFloat32 va = alpha3 * alpha6 - alpha5 * alpha4;
				//dgFloat32 den = va + vb + vc;
				//dgAssert (den > 0.0f);
                //den = dgFloat32 (1.0f) / (va + vb + vc);
                //dgFloat32 alpha0 = dgFloat32 (va * den);
                //alpha1 = dgFloat32 (vb * den);
                //alpha2 = dgFloat32 (vc * den);

				dgFloat32 alpha0 = ((p_p1 * p_p2) % faceNormal) * areaInv;
				dgFloat32 alpha1 = ((p_p2 * p_p0) % faceNormal) * areaInv;
				dgFloat32 alpha2 = ((p_p0 * p_p1) % faceNormal) * areaInv;

				attibute.m_u0 = uv0_0.m_x * alpha0 + uv0_1.m_x * alpha1 + uv0_2.m_x * alpha2; 
				attibute.m_v0 = uv0_0.m_y * alpha0 + uv0_1.m_y * alpha1 + uv0_2.m_y * alpha2; 
				attibute.m_u1 = uv1_0.m_x * alpha0 + uv1_1.m_x * alpha1 + uv1_2.m_x * alpha2; 
				attibute.m_v1 = uv1_0.m_y * alpha0 + uv1_1.m_y * alpha1 + uv1_2.m_y * alpha2; 

				AddAtribute (attibute);
				edge->m_incidentFace = 1;
				edge->m_userData = m_atribCount - 1;

				//faceIndices[indexCount] = edge->m_incidentVertex;
				//indexCount ++;
				//dgAssert (indexCount < sizeof (faceIndices) / sizeof (faceIndices[0]));

				edge = edge->m_next;
			} while (edge != face);

			//facePolygedra.AddFace(indexCount, faceIndices);
			//facePolygedra.EndFace();

			//dgPolyhedra leftOversOut(GetAllocator());
			//facePolygedra.ConvexPartition (&m_points[0].m_x, sizeof (dgBigVector), &leftOversOut);
			//dgAssert (leftOversOut.GetCount() == 0);
		}
	}

	return true;
}


dgMeshEffect* dgMeshEffect::ConvexMeshIntersection (const dgMeshEffect* const convexMeshSrc) const
{
	dgMeshEffect convexMesh (*convexMeshSrc);
	convexMesh.ConvertToPolygons();
	//return new (GetAllocator()) dgMeshEffect (*convexMesh);

	dgMeshEffect* const convexIntersection = new (GetAllocator()) dgMeshEffect (*this);
	//convexIntersection->ConvertToPolygons();
	//convexIntersection->Triangulate();
	convexIntersection->RemoveUnusedVertices(NULL);

	dgInt32 mark = convexMesh.IncLRU();
	dgPolyhedra::Iterator iter (convexMesh);

	for (iter.Begin(); iter; iter ++){
		 dgEdge* const convexFace = &(*iter);
		if ((convexFace->m_incidentFace > 0) && (convexFace->m_mark != mark)) {
			dgEdge* ptr = convexFace;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != convexFace);
			if (!convexIntersection->PlaneClip(convexMesh, convexFace)) {
				delete convexIntersection;
				return NULL;
			}
		}
	}

	convexIntersection->RemoveUnusedVertices(NULL);
	if (!convexIntersection->GetVertexCount()) {
		delete convexIntersection;
		return NULL;
	}

	convexIntersection->RemoveUnusedVertices(NULL);
	
	return convexIntersection;
}